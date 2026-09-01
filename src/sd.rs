//! Blocking driver for the on-board SD card slot, via the Arasan
//! "EMMC" SD host controller (the BCM2835 ARM Peripherals datasheet's
//! "External Mass Media Controller" — an SDHCI-compatible host).
//!
//! BCM2835/2836/2837 actually has *two* SD-capable controllers: this
//! one, and a separate, simpler "SDHOST" controller. Under Linux, a Pi
//! 3/Zero W's default `config.txt` drives the physical SD slot with
//! SDHOST instead, specifically to leave this EMMC controller free for
//! the on-board WiFi chip's SDIO interface (the two need independent
//! controllers to run concurrently). That constraint doesn't apply
//! here — this crate never drives the on-board WiFi at all — so using
//! this controller for the SD slot is safe on both Pi 2 and Pi 3, and
//! matches how established bare-metal Pi projects do it when they
//! don't also need SDIO for WiFi.
//!
//! GPIO alternate function 7 routes CLK/CMD/DAT0-3 (GPIO48-53) to this
//! controller — on BCM2836/2837; see the "BCM2711 (Pi 4)" section below,
//! where none of that applies. It reads and writes one 512-byte block at
//! a time
//! ([`read_block`](crate::sd::Sd::read_block)/
//! [`write_block`](crate::sd::Sd::write_block)) or a run of consecutive blocks
//! in a single command ([`read_blocks`](crate::sd::Sd::read_blocks)/
//! [`write_blocks`](crate::sd::Sd::write_blocks), `CMD18`/
//! `CMD25` with an auto-`CMD12` stop, cutting the per-block command
//! overhead on sequential transfers). Both paths poll the `DATA` FIFO a
//! word at a time by default; the `*_dma` variants (`read_blocks_dma`/
//! `write_blocks_dma`) instead move the data
//! over the system DMA controller ([`crate::dma`]), pacing it against the
//! EMMC FIFO's DREQ so the CPU isn't spent copying words. Card-detect
//! (GPIO47) isn't implemented yet.
//!
//! # DMA path
//!
//! This controller is the Arasan/SDHCI part, whose own internal SDMA the
//! BCM2835 doesn't wire up; instead the SoC routes the EMMC FIFO's data
//! request onto the general-purpose DMA controller's DREQ line 11, so a
//! [`crate::dma::Channel`] reads/writes the `DATA` register (bus address
//! `0x7E30_0020`) paced by that DREQ — the same route Linux's original
//! `bcm2835-mmc` driver took before it moved to the separate SDHOST
//! controller. The command is issued first and the blocking DMA started
//! right after: the controller flow-controls the SD clock against its own
//! FIFO (gating it when the FIFO fills on a read, or waiting for data on a
//! write), so the DMA can't overrun or underrun it regardless of when it
//! starts draining/filling relative to the card's clocking.
//!
//! Register sequencing, timing, and (particularly) the packed 32-bit
//! command "code" values passed to `CMDTM` follow two widely used,
//! hardware-confirmed bare-metal Pi 2/3 EMMC drivers — bztsrc's
//! `raspi3-tutorial` `0B_readsector/sd.c` and rsta2's `circle`
//! `addon/SDCard/emmc.cpp` — the authoritative references for the exact
//! bit-packing this hardware expects (SD physical layer command index +
//! response-type encoding is standard, but which flags a *specific*
//! command needs — CRC check, index check, data-direction,
//! block-count-enable — is fiddly enough that treating these as
//! verbatim, cited constants beat re-deriving them field-by-field). Two
//! places where the two references disagree and only one works on this
//! hardware are called out inline: the clock divider (`set_clock` needs
//! `circle`'s 10-bit SDHCI 3.0 form, not an 8-bit one, for the 200MHz
//! base clock firmware reports) and CMD55's response type
//! (`CMD_APP_CMD` must expect a response so the following `ACMD41`
//! isn't issued into it).
//!
//! # Async
//!
//! Under the `async` feature the same [`Sd`](crate::sd::Sd) also carries
//! interrupt-driven twins of the transfer methods — `read_blocks_async`
//! and friends — which park on the controller's interrupt rather than
//! spinning on `INTERRUPT`, so an executor gets the card's own thinking
//! time back. Most of a write is exactly that: the final `DATA_DONE`
//! only arrives once the card has programmed an entire internal erase
//! block, milliseconds at a time on a cheap card. The blocking methods
//! are untouched and remain the right choice for a program with nothing
//! else to do.
//!
//! They need the usual three gates plus a handler: the controller's own
//! `IRPT_EN` (opened by each transfer as it parks, so nothing is needed
//! from the application), `crate::lic::Lic::enable_emmc_irq`, the CPU
//! mask ([`enable_irq`](crate::irq::enable_irq)), and a call to
//! `sd::on_irq` from the application's `__irq_handler`.
//! Cancelling a transfer — dropping the future, as
//! `embassy_time::with_timeout` does — aborts it on the card and resets
//! the controller's data circuit before the drop returns, so the next
//! transfer starts clean; see `read_blocks_async` for the details.
//! `examples/sd_async.rs` is the whole thing end to end.
//!
//! # BCM2711 (Pi 4)
//!
//! This controller (`EMMC`, at the same address as BCM2836/2837) is
//! *not* what the physical SD card slot is wired to on a Pi 4 — confirmed
//! against the upstream device tree (`bcm2711-rpi-4-b.dts`: `/* EMMC2 is
//! used to drive the SD card */`), not assumed. So the controller
//! register block changes, hence `Emmc2` and the `ClockId::Emmc2` mailbox
//! clock id (EMMC2 has its own base clock, separate from the classic
//! `EMMC`'s) — and so does the pin routing, which goes away entirely.
//! EMMC2 drives dedicated pads outside the 54-pin bank (`bcm2711.dtsi`'s
//! `emmc2` node carries no `pinctrl` property at all), while GPIO48-53
//! on that board are the gigabit Ethernet PHY's RGMII interface — so
//! `route_gpio_to_emmc` is compiled out here rather than adapted, and
//! `Sd::init` takes its `GPIO` argument without using it. The
//! DMA-backed `Sd::read_blocks_dma`/`Sd::write_blocks_dma` aren't
//! available under `bcm2711`: EMMC2 sits on its own bus with its own
//! VideoCore bus-address mapping (`bcm2711.dtsi`'s `emmc2bus`, a
//! `0xc000_0000`-based range, not the classic `0x7e00_0000` alias
//! `DATA_REG_BUS_ADDRESS` uses) and likely its own DMA DREQ line — real
//! values neither confirmed nor needed yet (nothing here uses the DMA
//! path).

#[cfg(not(feature = "bcm2711"))]
use crate::dma::Channel;
use crate::emmc::clock_divider;
use crate::mailbox::{ClockId, Mailbox, PowerDeviceId};
#[cfg(not(feature = "bcm2711"))]
use crate::pac::EMMC;
use crate::pac::GPIO;
use crate::timer::Timer;

// Scoped to `bcm2837` as well as to the feature: routing this
// controller's line to the ARM core needs `crate::lic`, which the
// BCM2711 doesn't have (its GIC-400 isn't supported yet), so on that
// chip an async transfer could only ever park forever. The blocking
// path is unaffected and is what a Pi 4 uses.
#[cfg(all(feature = "async", not(feature = "bcm2711")))]
mod asynch;
#[cfg(all(feature = "async", not(feature = "bcm2711")))]
pub use asynch::on_irq;

// The pins below, and everything that touches them, are BCM2836/2837
// only: a Pi 4's card slot is on EMMC2, whose pins aren't in this bank
// at all -- see `route_gpio_to_emmc`.
/// GPIO pin carrying the SD clock (`CLK`). Only used computing the
/// pull-mask below.
#[cfg(not(feature = "bcm2711"))]
const GPIO_CLK: u32 = 48;
/// GPIO pin carrying the SD command line (`CMD`). See `GPIO_CLK`.
#[cfg(not(feature = "bcm2711"))]
const GPIO_CMD: u32 = 49;
/// GPIO pins carrying the SD data lines (`DAT0`..`DAT3`). See `GPIO_CLK`.
#[cfg(not(feature = "bcm2711"))]
const GPIO_DAT: [u32; 4] = [50, 51, 52, 53];
/// GPIO alternate function routing GPIO48-53 to this controller.
#[cfg(not(feature = "bcm2711"))]
const GPIO_ALT_FUNCTION: u8 = 0b111;

/// Firmware property tag's `ClockId` for the EMMC base clock feeding
/// [`set_clock`]'s divider — queried at runtime rather than hardcoded,
/// matching the reference driver (rsta2's `circle`, whose
/// `GetBaseClock` reads this same clock over the mailbox). On this
/// board's firmware it reports 200MHz — higher than the ~41.6MHz older
/// bare-metal tutorials assume, which is exactly why [`set_clock`] must
/// use the full 10-bit SDHCI 3.0 divider to reach the SD spec's
/// ≤400kHz identification clock from it (an 8-bit divider tops out at
/// ÷256, only reaching ~780kHz from 200MHz).
#[cfg(not(feature = "bcm2711"))]
const EMMC_CLOCK_ID: ClockId = ClockId::Emmc;
#[cfg(feature = "bcm2711")]
const EMMC_CLOCK_ID: ClockId = ClockId::Emmc2;

/// BCM2711's EMMC2 controller — the block actually wired to a Pi 4's
/// physical SD card slot (see this module's "BCM2711" doc section).
/// Not in `bcm2711-lpa`'s SVD, but confirmed to be the same "Arasan
/// SD3.0 Host AHB eMMC 4.4" register layout the PAC's own `EMMC` type
/// already models (same `0x100`-byte register window per the upstream
/// device tree, and Broadcom's usual practice of reusing this core
/// across SoC generations) — so this reuses that generated type via
/// `Deref` at EMMC2's own address instead of duplicating it.
#[cfg(feature = "bcm2711")]
pub struct Emmc2 {
    _marker: core::marker::PhantomData<*const ()>,
}
#[cfg(feature = "bcm2711")]
unsafe impl Send for Emmc2 {}
#[cfg(feature = "bcm2711")]
impl Emmc2 {
    /// Physical base address, from `bcm2711.dtsi`'s `emmc2` node
    /// (`mmc@7e340000`, the `0x7e00_0000`-bus-alias form of
    /// `0xfe34_0000`).
    const PTR: *const crate::pac::emmc::RegisterBlock = 0xfe34_0000 as *const _;

    /// Creates a handle to the EMMC2 controller.
    ///
    /// Not `unsafe`/`steal`-gated like the PAC's own peripheral tokens:
    /// this type exists only because EMMC2 isn't in the PAC at all, the
    /// same situation [`crate::rng::Rng::new`] is in, and for the same
    /// reason carries no extra ownership ceremony beyond a normal
    /// constructor.
    pub fn new() -> Self {
        Self {
            _marker: core::marker::PhantomData,
        }
    }
}
#[cfg(feature = "bcm2711")]
impl Default for Emmc2 {
    fn default() -> Self {
        Self::new()
    }
}
#[cfg(feature = "bcm2711")]
impl core::ops::Deref for Emmc2 {
    type Target = crate::pac::emmc::RegisterBlock;
    fn deref(&self) -> &Self::Target {
        unsafe { &*Self::PTR }
    }
}

/// Which controller type [`Sd`] drives: the classic `EMMC` on
/// BCM2836/2837, or [`Emmc2`] on BCM2711 (see this module's "BCM2711"
/// doc section) — selected at compile time, like every other chip
/// difference in this crate.
#[cfg(not(feature = "bcm2711"))]
type SdEmmc = EMMC;
#[cfg(feature = "bcm2711")]
type SdEmmc = Emmc2;

/// Target SD clock during the identification/initialization phase,
/// before the card's real capabilities are known — the SD physical
/// layer spec requires staying at or below 400kHz here.
const SETUP_CLOCK_HZ: u32 = 400_000;
/// Target SD clock once the card is in data-transfer state — a
/// conservative "default speed" rate every SD card supports, well
/// under the 25MHz ceiling that mode allows.
const TRANSFER_CLOCK_HZ: u32 = 25_000_000;

/// `CONTROL1.DATA_TOUNIT` value selecting the second-longest data
/// timeout the field supports (`0b1111` disables the timeout
/// entirely, which would hang forever on a genuinely wedged
/// controller instead of surfacing an error).
const DATA_TIMEOUT_MAX: u8 = 0b1110;

/// `CMDTM` value for CMD0 (`GO_IDLE_STATE`) — no response.
const CMD_GO_IDLE: u32 = 0x0000_0000;
/// `CMDTM` value for CMD2 (`ALL_SEND_CID`) — R2 (136-bit) response.
const CMD_ALL_SEND_CID: u32 = 0x0201_0000;
/// `CMDTM` value for CMD3 (`SEND_RELATIVE_ADDR`) — R6 (48-bit)
/// response.
const CMD_SEND_REL_ADDR: u32 = 0x0302_0000;
/// `CMDTM` value for CMD7 (`SELECT_CARD`) — R1b (48-bit + busy)
/// response.
const CMD_CARD_SELECT: u32 = 0x0703_0000;
/// `CMDTM` value for CMD8 (`SEND_IF_COND`) — R7 (48-bit) response.
const CMD_SEND_IF_COND: u32 = 0x0802_0000;
/// `CMDTM` value for CMD17 (`READ_SINGLE_BLOCK`) — R1 (48-bit)
/// response, data present, card-to-host direction.
const CMD_READ_SINGLE: u32 = 0x1122_0010;
/// `CMDTM` value for CMD24 (`WRITE_BLOCK`) — R1 (48-bit) response,
/// data present, host-to-card direction. Same encoding as
/// [`CMD_READ_SINGLE`] but with the command index 24 (`0x18`) and the
/// `TM_DAT_DIR` bit (`0x10`) cleared: a write drives data host→card,
/// the opposite direction from the read.
const CMD_WRITE_SINGLE: u32 = 0x1822_0000;
/// `CMDTM` value for CMD18 (`READ_MULTIPLE_BLOCK`) — R1 (48-bit)
/// response, data present, card-to-host. The multi-block form of
/// [`CMD_READ_SINGLE`]: command index 18 (`0x12`), plus three transfer-
/// mode bits the single-block read leaves clear — `TM_MULTI_BLOCK`
/// (`0x20`), `TM_BLKCNT_EN` (`0x02`, so the controller counts down
/// `BLKSIZECNT.blkcnt` blocks and stops), and `TM_AUTO_CMD_EN = CMD12`
/// (`0x04`, so the controller issues the `CMD12` `STOP_TRANSMISSION`
/// itself once the block count is exhausted, rather than the driver
/// sending it as a separate command).
const CMD_READ_MULTI: u32 = 0x1222_0036;
/// `CMDTM` value for CMD25 (`WRITE_MULTIPLE_BLOCK`) — R1 (48-bit)
/// response, data present, host-to-card. The multi-block form of
/// [`CMD_WRITE_SINGLE`]: command index 25 (`0x19`), with the same
/// `TM_MULTI_BLOCK`/`TM_BLKCNT_EN`/`TM_AUTO_CMD_EN = CMD12` bits as
/// [`CMD_READ_MULTI`] but `TM_DAT_DIR` clear (data flows host→card).
const CMD_WRITE_MULTI: u32 = 0x1922_0026;
/// `CMDTM` value for CMD55 (`APP_CMD`) — R1 (48-bit, CRC-checked)
/// response.
///
/// The response *must* be expected here even though this driver never
/// reads it: CMD55 always elicits an R1 response from the card (SD
/// spec), and telling the controller `RSPNS_TYPE = none` makes it skip
/// waiting for that response and issue the following `ACMD41` too
/// early, colliding with CMD55's still-in-flight response on the CMD
/// line — the `ACMD41` is then lost and times out. (`circle` sets the
/// response type as done here; `bztsrc` instead leaves it `none` but
/// pads with an explicit ~100ms delay after CMD55, which lets the bus
/// settle the same way. This driver has no such delay, so it relies on
/// the response type being correct.)
const CMD_APP_CMD: u32 = 0x370A_0000;
/// `CMDTM` value for ACMD41 (`SD_SEND_OP_COND`), OR'd with
/// [`CMD_NEED_APP`] so [`Sd::command`] sends the [`CMD_APP_CMD`]
/// prefix first.
const CMD_SEND_OP_COND: u32 = 0x2902_0000 | CMD_NEED_APP;
/// `CMDTM` value for ACMD51 (`SEND_SCR`) — R1 (48-bit) response, data
/// present, card-to-host direction (an 8-byte single-block read, same
/// data-transfer shape as [`CMD_READ_SINGLE`], just a different index
/// and length), OR'd with [`CMD_NEED_APP`].
const CMD_SEND_SCR: u32 = 0x3322_0010 | CMD_NEED_APP;
/// `CMDTM` value for ACMD6 (`SET_BUS_WIDTH`) — R1 (48-bit) response, OR'd
/// with [`CMD_NEED_APP`]. Argument `2` selects the 4-bit bus (`0` is
/// 1-bit; other values are reserved).
const CMD_SET_BUS_WIDTH: u32 = 0x0602_0000 | CMD_NEED_APP;
/// Marks a command as an application-specific command (`ACMD*`),
/// needing an `APP_CMD` (CMD55) sent immediately before it. Not a real
/// `CMDTM` bit — masked off before the value actually reaches the
/// register.
const CMD_NEED_APP: u32 = 0x8000_0000;
/// `SET_BUS_WIDTH` (ACMD6) argument selecting the 4-bit bus.
const BUS_WIDTH_4BIT: u32 = 2;

/// Mask over the `INTERRUPT` register's error bits — any of these set
/// means the just-issued command or data transfer failed.
const INT_ERROR_MASK: u32 = 0x017e_8000;
/// `INTERRUPT.CMD_DONE`.
const INT_CMD_DONE: u32 = 0x0000_0001;
/// `INTERRUPT.DATA_DONE` (transfer complete) — for a write, only
/// fires once the card has finished programming the block internally.
const INT_DATA_DONE: u32 = 0x0000_0002;
/// `INTERRUPT.WRITE_RDY` — the controller's write FIFO is ready to
/// accept the block's data.
const INT_WRITE_RDY: u32 = 0x0000_0010;
/// `INTERRUPT.READ_RDY`.
const INT_READ_RDY: u32 = 0x0000_0020;

/// ACMD41 argument requesting the standard voltage window plus host
/// capacity support (HCS — "I can handle an SDHC/SDXC card").
const ACMD41_ARG_HC: u32 = 0x51ff_8000;
/// ACMD41 response bit: the card has finished its power-up sequence.
const ACMD41_CMD_COMPLETE: u32 = 0x8000_0000;
/// ACMD41 response bit: the card accepted the requested voltage
/// window.
const ACMD41_VOLTAGE: u32 = 0x00ff_8000;
/// ACMD41 response bit ("Card Capacity Status"): set for an SDHC/SDXC
/// card (block addressing), clear for SDSC (byte addressing) — see
/// [`Sd::high_capacity`].
const ACMD41_CMD_CCS: u32 = 0x4000_0000;

/// Bit in the SCR register's first (high) word — SD spec's
/// `SD_BUS_WIDTHS` field, bit 2 — indicating the card supports the
/// 4-bit bus. Checked before [`Sd::init`] tries to switch to it.
const SCR_SD_BUS_WIDTH_4: u32 = 0x0000_0400;

/// The system DMA controller's DREQ (pacing) line for the EMMC
/// controller's `DATA` FIFO — SoC-fixed at 11 (line 13 is the *other*,
/// SDHOST, controller). Passed to
/// [`crate::dma::Channel::copy_from_peripheral`]/`copy_to_peripheral` so
/// the DMA engine only moves a word when the FIFO signals it has data
/// (read) or room (write). Not meaningful under `bcm2711` -- see this
/// module's "BCM2711" doc section.
#[cfg(not(feature = "bcm2711"))]
const DMA_DREQ_EMMC: u8 = 11;
/// VideoCore *bus* address of the EMMC `DATA` register, the fixed FIFO
/// port a DMA channel reads block data from / writes it to. The EMMC
/// block's ARM physical base `0x3F30_0000` plus `DATA`'s `0x20` offset,
/// expressed in the `0x7E00_0000` bus alias a bus master (the DMA engine)
/// must use rather than the ARM physical address. Not meaningful under
/// `bcm2711` -- see this module's "BCM2711" doc section.
#[cfg(not(feature = "bcm2711"))]
const DATA_REG_BUS_ADDRESS: u32 = 0x7e30_0020;

/// A 512-byte SD block. The unit [`Sd::read_blocks`]/[`Sd::write_blocks`]
/// and their DMA variants transfer, chosen over a flat `&[u8]` so the
/// block count is exact by construction (a slice length is always a whole
/// number of blocks) and the multiple-of-512 byte length the DMA path
/// needs is guaranteed.
pub type Block = [u8; 512];

/// Errors from [`Sd::init`] and the block read/write methods
/// ([`Sd::read_block`]/[`Sd::read_blocks`]/[`Sd::write_block`]/
/// [`Sd::write_blocks`] and their `_dma` variants).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// A wait for the controller or card to reach some state (clock
    /// stable, command/data-line idle, command/data done) exceeded its
    /// budget.
    Timeout,
    /// A wait for a specific `INTERRUPT` bit (via [`Sd::read_block`]/
    /// [`Sd::write_block`]/[`Sd::init`]) never saw that bit nor any
    /// error bit within its budget — captures what the controller was
    /// doing at the moment it gave up, to distinguish *which* handshake
    /// stalled (e.g. a command that never completed versus a data phase
    /// that never became ready).
    WaitTimeout {
        /// The `INTERRUPT` bit(s) that were being waited for but never
        /// appeared (e.g. `CMD_DONE`, `WRITE_RDY`, `DATA_DONE`).
        waiting_for: u32,
        /// The `INTERRUPT` register's value at the moment the wait gave
        /// up — shows which bits *did* fire (an error bit, an unexpected
        /// completion, or nothing at all) while the expected one didn't.
        interrupt: u32,
        /// The `STATUS` register's raw value at the same moment — its
        /// `CMD_INHIBIT`/`DAT_INHIBIT`/`DAT_ACTIVE`/line-level bits say
        /// whether the controller still thinks a command or data
        /// transfer is in flight.
        status: u32,
        /// The `CMDTM` code of the command whose completion or data
        /// phase was being awaited — its high byte is the SD command
        /// index, saying which command's handshake stalled.
        command: u32,
    },
    /// A command or data transfer finished with the `INTERRUPT`
    /// register's error mask set.
    ///
    /// The two fields are what identify a *card*-side failure: which
    /// error fired, and what the card was being asked to do. Controller
    /// state — clock enable/stable, the divisor, the physical line levels
    /// — is deliberately not carried here: a fault in any of those stops
    /// the bus before a command ever completes, so it surfaces as
    /// [`Self::LinesNotIdleHigh`] or [`Self::Timeout`] instead, and is
    /// readable from the registers directly when one of those is what
    /// happened.
    CardError {
        /// The `INTERRUPT` register's value when the error was
        /// detected, for diagnosing which specific error(s) fired.
        interrupt: u32,
        /// The `CMDTM` code of the command whose completion or data
        /// phase was in flight when the error fired — its high byte
        /// (bits 24:29) is the SD command index, so this says whether it
        /// was e.g. `CMD0` (`0x0000_0000`), `CMD8` (`0x0802_0000`),
        /// `CMD55` (`0x370A_0000`), `ACMD41` (`0x2902_0000`), or
        /// `CMD24` (`0x1822_0000`) that the card didn't answer.
        command: u32,
    },
    /// The card didn't accept the requested voltage window during
    /// initialization (`ACMD41`) — an unsupported or non-SD card.
    UnusableCard,
    /// The firmware's "Set Power State" call (tag `0x0002_8001`,
    /// [`PowerDeviceId::SdCard`]) reported the SD card power domain
    /// isn't present or wouldn't turn on.
    PowerOnFailed,
    /// A multi-block transfer requested more blocks than the controller's
    /// 16-bit `BLKSIZECNT.blkcnt` field can express (65535). Split the
    /// transfer into smaller runs.
    TooManyBlocks,
    /// A `read_blocks_dma`/`write_blocks_dma` transfer failed at the
    /// DMA-controller level (a channel length limit exceeded, or a
    /// hardware error mid-transfer); carries the underlying
    /// [`dma::Error`](crate::dma::Error).
    Dma(crate::dma::Error),
    /// Right after configuring GPIO48-53's pull-ups, before any
    /// command has touched the bus, `CMD` and/or `DAT0`..`DAT3` weren't
    /// idling high — a GPIO routing/pull problem rather than anything
    /// card-side, and the reason [`Self::CardError`] doesn't carry line
    /// levels of its own: by the time a command can complete and fail,
    /// the wiring has already been proven here.
    LinesNotIdleHigh {
        /// The physical `CMD` line's electrical level
        /// (`STATUS.CMD_LEVEL`).
        cmd_line_high: bool,
        /// The physical `DAT0`..`DAT3` lines' electrical levels
        /// (`STATUS.DAT_LEVEL0`, one bit per line, `DAT0` in bit 0).
        data_lines_high: u8,
    },
}

/// A configured SD card, ready for block reads and writes.
///
/// Build one with [`Self::init`], then call [`Self::read_block`] /
/// [`Self::write_block`] repeatedly.
pub struct Sd {
    emmc: SdEmmc,
    /// Whether the card uses block addressing (SDHC/SDXC) rather than
    /// byte addressing (SDSC) — see [`Self::high_capacity`].
    high_capacity: bool,
    /// The card's relative card address (`CMD3`'s response, already in
    /// argument position — bits 31:16), needed to correctly address
    /// this card in every later command. Also doubles as the argument
    /// [`Self::command`] sends with the `APP_CMD` (CMD55) prefix: `0`
    /// during [`Self::init`]'s own `ACMD41` (sent before the card is
    /// assigned an RCA, so `0` is the spec-correct argument there too),
    /// the real RCA for any application command sent after.
    rca: u32,
    /// Whether the card and controller negotiated the 4-bit bus (versus
    /// the SD default 1-bit) — see [`Self::four_bit_bus`].
    four_bit_bus: bool,
}

impl Sd {
    /// Steals whichever controller [`Self::init`] needs on the active
    /// chip: the classic `EMMC` peripheral (via `pac::Peripherals`) on
    /// BCM2836/2837, or `Emmc2` on BCM2711 (not part of `Peripherals`
    /// at all, since it isn't in the PAC — see this module's "BCM2711"
    /// doc section). Exists so callers don't need their own `#[cfg]`
    /// just to build [`Self::init`]'s second argument.
    ///
    /// # Safety
    ///
    /// On BCM2836/2837 this steals `pac::Peripherals` — the same
    /// requirement as [`crate::pac::Peripherals::steal`] itself: don't
    /// let two live instances race over the same registers (fine to
    /// call after already stealing `Peripherals` for other fields, as
    /// long as nothing else concurrently touches `EMMC`, the same
    /// pattern `rpi-loader` already re-steals under). On BCM2711,
    /// `Emmc2::new` carries no such requirement at all — this
    /// function is still marked `unsafe` uniformly so a caller doesn't
    /// need to know which is true.
    pub unsafe fn steal_emmc() -> SdEmmc {
        #[cfg(not(feature = "bcm2711"))]
        {
            unsafe { crate::pac::Peripherals::steal() }.EMMC
        }
        #[cfg(feature = "bcm2711")]
        {
            Emmc2::new()
        }
    }

    /// Brings the SD card up: routes GPIO48-53 to this controller (on
    /// BCM2836/2837 — a Pi 4's slot is on EMMC2, whose pins aren't in
    /// the GPIO bank, so `gpio` goes unused there), resets it, and runs
    /// the SD physical layer's card identification sequence (`CMD0`,
    /// `CMD8`, `ACMD41`, `CMD2`, `CMD3`, `CMD7`) — the same sequence
    /// real host controllers use, ending with the card in the "transfer"
    /// state. From there it also tries (best-effort — see
    /// [`Self::four_bit_bus`]) to negotiate the 4-bit bus via
    /// `ACMD51`/`ACMD6`, before returning ready for
    /// [`Self::read_block`].
    pub fn init(
        gpio: &GPIO,
        emmc: SdEmmc,
        mailbox: &mut Mailbox,
        timer: &Timer,
    ) -> Result<Self, Error> {
        // The SD card controller comes up unpowered from firmware (the
        // same situation `usb::power_on` handles for the DWC2
        // controller) -- register reads/writes against it can still
        // appear to succeed while unpowered, since the register bus
        // itself lives on a different, always-on power rail than the
        // analog/PHY side actually driving the pins.
        if !matches!(
            mailbox.set_power_state(PowerDeviceId::SdCard, true),
            Ok(true)
        ) {
            return Err(Error::PowerOnFailed);
        }

        #[cfg(not(feature = "bcm2711"))]
        route_gpio_to_emmc(gpio);
        // EMMC2's pins aren't in the GPIO bank at all -- see
        // `route_gpio_to_emmc` on why muxing anything here would be
        // actively wrong on a Pi 4. `gpio` stays in the signature so a
        // consumer's call site is the same on either chip.
        #[cfg(feature = "bcm2711")]
        let _ = gpio;
        // Let the pull-ups actually settle before trusting a level
        // read from them.
        timer.delay_ms(1);
        let status = emmc.status().read();
        let (cmd_line_high, data_lines_high) =
            (status.cmd_level().bit_is_set(), status.dat_level0().bits());
        if !cmd_line_high || data_lines_high != 0b1111 {
            return Err(Error::LinesNotIdleHigh {
                cmd_line_high,
                data_lines_high,
            });
        }

        // BCM2711's EMMC2 isn't among the clocks firmware enables for its
        // own purposes (unlike the classic EMMC, which is always on) --
        // see `Mailbox::set_clock_state`'s doc comment. Scoped to
        // `bcm2711` rather than made unconditional: the classic path is
        // already hardware-verified, and there's no way to confirm this
        // call is truly a harmless no-op there without a real BCM2836/
        // 2837 board to check it against.
        #[cfg(feature = "bcm2711")]
        mailbox
            .set_clock_state(EMMC_CLOCK_ID, true)
            .map_err(|_| Error::PowerOnFailed)?;

        let base_clock_hz = mailbox
            .clock_rate_hz(EMMC_CLOCK_ID)
            .map_err(|_| Error::PowerOnFailed)?;

        // Reset the whole host circuit and wait for the reset bit to
        // self-clear.
        emmc.control0().reset();
        emmc.control1().modify(|_, w| w.srst_hc().set_bit());
        wait_for(timer, 100_000, || {
            !emmc.control1().read().srst_hc().bit_is_set()
        })?;

        // BCM2711 needs the standard SDHCI "Power Control" byte (bits
        // 8-15 of this same Host-Control/Power-Control/Block-Gap-Control
        // register, left at CONTROL0's reset default of 0 otherwise)
        // actually set to select a bus voltage and enable bus power --
        // confirmed against barebox's `mci-bcm2835` driver, which added
        // exactly this write (`(SDHCI_BUS_VOLTAGE_330 |
        // SDHCI_BUS_POWER_EN) << 8` = `0x0f00`) when it gained BCM2711/
        // EMMC2 support, noting older chips tolerate the register
        // staying zero where BCM2711 doesn't. That's why the classic
        // path here never needed it, and why this isn't a named field
        // in this PAC's SVD -- the classic controller's hardware
        // doesn't use it, so it was never modeled -- hence the
        // read-modify-write through the raw escape hatch rather than a
        // named accessor.
        #[cfg(feature = "bcm2711")]
        emmc.control0()
            .modify(|r, w| unsafe { w.bits(r.bits() | 0x0f00) });

        // Enable the internal clock and a real (not disabled) data
        // timeout before the very first clock-divisor write, matching
        // the reference driver's ordering.
        emmc.control1().modify(|_, w| unsafe {
            w.clk_intlen()
                .set_bit()
                .data_tounit()
                .bits(DATA_TIMEOUT_MAX)
        });
        timer.delay_ms(10);

        set_clock(&emmc, base_clock_hz, SETUP_CLOCK_HZ, timer)?;

        // Unmask every interrupt status bit so it's visible in
        // `INTERRUPT` -- the blocking path polls that register directly.
        // `IRPT_EN`, the separate register deciding which of those
        // visible bits actually assert the controller's interrupt line,
        // stays untouched at zero: an async transfer opens only the bits
        // it is about to park on and closes them again on the way out
        // (see the `asynch` module), so a bit nobody is servicing can
        // never leave a level source asserted.
        emmc.irpt_mask().write(|w| unsafe { w.bits(0xffff_ffff) });

        let sd = Self {
            emmc,
            high_capacity: false,
            rca: 0,
            four_bit_bus: false,
        };

        sd.command(CMD_GO_IDLE, 0, timer)?;
        sd.command(CMD_SEND_IF_COND, 0x0000_01aa, timer)?;

        // ACMD41: poll until the card reports its power-up sequence
        // complete, budgeting a generous 1 second total -- the SD
        // spec allows a card up to ~1s here in the worst case.
        let start = timer.now_micros();
        let response = loop {
            let response = sd.command(CMD_SEND_OP_COND, ACMD41_ARG_HC, timer)?;
            if response & ACMD41_CMD_COMPLETE != 0 {
                break response;
            }
            if timer.now_micros() - start > 1_000_000 {
                return Err(Error::Timeout);
            }
            timer.delay_ms(10);
        };
        if response & ACMD41_VOLTAGE == 0 {
            return Err(Error::UnusableCard);
        }
        let high_capacity = response & ACMD41_CMD_CCS != 0;

        sd.command(CMD_ALL_SEND_CID, 0, timer)?;

        let rca = sd.command(CMD_SEND_REL_ADDR, 0, timer)? & 0xffff_0000;
        let sd = Self { rca, ..sd };

        set_clock(&sd.emmc, base_clock_hz, TRANSFER_CLOCK_HZ, timer)?;

        sd.command(CMD_CARD_SELECT, rca, timer)?;

        // Try to negotiate the 4-bit bus -- optional, not something
        // correctness depends on (every modern card supports it, but
        // it's a pure performance upgrade over the SD default 1-bit
        // bus), so any failure here (an old card that genuinely
        // doesn't support it, or a transient command error) just
        // leaves the bus at 1-bit rather than failing `init` outright.
        let four_bit_bus = sd.negotiate_four_bit_bus(timer).unwrap_or(false);

        Ok(Self {
            high_capacity,
            four_bit_bus,
            ..sd
        })
    }

    /// Whether the card uses block addressing (SDHC/SDXC, one unit per
    /// 512-byte block) rather than byte addressing (SDSC) — determines
    /// how [`Self::read_block`] forms its argument, but otherwise
    /// nothing a caller needs to handle differently.
    pub fn high_capacity(&self) -> bool {
        self.high_capacity
    }

    /// Whether [`Self::init`] negotiated the 4-bit bus (versus the SD
    /// default 1-bit bus) — a pure performance fact, not something a
    /// caller needs to branch on: [`Self::read_block`] works the same
    /// either way, since the bus width only affects the physical
    /// electrical transfer, not how words are pulled from `DATA`.
    pub fn four_bit_bus(&self) -> bool {
        self.four_bit_bus
    }

    /// Reads the 512-byte block at `block_index` (a logical block
    /// address, i.e. `block_index * 512` bytes into the card), polling
    /// the `DATA` FIFO a word at a time. A single-block read (`CMD17`
    /// `READ_SINGLE_BLOCK`); the multi-block [`Self::read_blocks`] cuts
    /// the per-block command overhead on a longer run.
    pub fn read_block(
        &self,
        block_index: u32,
        buf: &mut Block,
        timer: &Timer,
    ) -> Result<(), Error> {
        self.read_blocks_pio(block_index, 1, core::iter::once(buf), timer)
    }

    /// Reads `blocks.len()` consecutive 512-byte blocks starting at
    /// `block_index` into `blocks`, polling the `DATA` FIFO a word at a
    /// time. A run longer than one block goes out as a single `CMD18`
    /// `READ_MULTIPLE_BLOCK` (terminated by an auto-`CMD12`
    /// `STOP_TRANSMISSION` the controller issues itself), so the whole
    /// run costs one command instead of one per block. An empty slice is
    /// a no-op; [`Error::TooManyBlocks`] is returned for a run longer
    /// than the controller's 16-bit block count (65535). See
    /// `Self::read_blocks_dma` to move the data over DMA instead of the
    /// CPU.
    pub fn read_blocks(
        &self,
        block_index: u32,
        blocks: &mut [Block],
        timer: &Timer,
    ) -> Result<(), Error> {
        if blocks.is_empty() {
            return Ok(());
        }
        let count = checked_block_count(blocks.len())?;
        self.read_blocks_pio(block_index, count, blocks.iter_mut(), timer)
    }

    /// Reads `blocks.len()` consecutive 512-byte blocks starting at
    /// `block_index` into `blocks` over the system DMA controller
    /// (`channel`), rather than the CPU copying each word out of the
    /// `DATA` FIFO — the throughput win over [`Self::read_blocks`] on a
    /// larger transfer. Uses the same `CMD17`/`CMD18` (+ auto-`CMD12`)
    /// commands; the difference is purely how the data phase is drained.
    ///
    /// The command is issued first and the blocking DMA drain started
    /// right after — safe because the controller gates the SD clock
    /// against its FIFO, so it can't overrun the engine (see the module
    /// docs). `channel` need only be a plain DMA channel from
    /// [`crate::dma::Dma::channel`]; the EMMC FIFO's DREQ paces it. As
    /// with any DMA target, `blocks` should be cache-line aligned so the
    /// post-transfer cache invalidate doesn't discard a neighbour sharing
    /// a line (its length is already a whole number of 512-byte blocks).
    ///
    /// An empty slice is a no-op; [`Error::TooManyBlocks`] is returned for
    /// a run past the controller's 16-bit block count and [`Error::Dma`]
    /// for a channel length limit or a hardware error mid-transfer.
    ///
    /// Not available under `bcm2711` -- see this module's "BCM2711" doc
    /// section.
    #[cfg(not(feature = "bcm2711"))]
    pub fn read_blocks_dma(
        &self,
        block_index: u32,
        blocks: &mut [Block],
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<(), Error> {
        if blocks.is_empty() {
            return Ok(());
        }
        let count = checked_block_count(blocks.len())?;
        let cmd = if count == 1 {
            CMD_READ_SINGLE
        } else {
            CMD_READ_MULTI
        };
        self.start_transfer(cmd, block_index, count, timer)?;
        channel
            .copy_from_peripheral(
                blocks.as_flattened_mut(),
                DMA_DREQ_EMMC,
                DATA_REG_BUS_ADDRESS,
            )
            .map_err(Error::Dma)?;
        // With the FIFO drained by the engine, wait for the controller's
        // transfer-complete so any (auto-`CMD12`) error surfaces and it's
        // idle before the next command.
        self.wait_interrupt(INT_DATA_DONE, cmd, timer)?;
        Ok(())
    }

    /// Writes `buf` to the 512-byte block at `block_index` (a logical
    /// block address, i.e. `block_index * 512` bytes into the card).
    ///
    /// A single-block write (`CMD24` `WRITE_BLOCK`), the mirror of
    /// [`Self::read_block`]: programs `BLKSIZECNT`, issues the command,
    /// waits for the controller's write FIFO to report ready
    /// (`WRITE_RDY`), pushes all 128 words to `DATA`, then waits
    /// for transfer-complete (`DATA_DONE`) before returning. That
    /// last interrupt only fires once the card has finished programming
    /// the block internally, so a successful return means the data is
    /// committed to the card, not merely queued in the FIFO. The
    /// multi-block [`Self::write_blocks`] cuts the per-block command
    /// overhead on a longer run.
    pub fn write_block(&self, block_index: u32, buf: &Block, timer: &Timer) -> Result<(), Error> {
        self.write_blocks_pio(block_index, 1, core::iter::once(buf), timer)
    }

    /// Writes `blocks.len()` consecutive 512-byte blocks starting at
    /// `block_index` from `blocks`, pushing each word into the `DATA`
    /// FIFO. A run longer than one block goes out as a single `CMD25`
    /// `WRITE_MULTIPLE_BLOCK` (terminated by an auto-`CMD12`
    /// `STOP_TRANSMISSION`), so the whole run costs one command instead
    /// of one per block. Like [`Self::write_block`] it waits for
    /// transfer-complete before returning, so success means the data is
    /// committed to the card. An empty slice is a no-op;
    /// [`Error::TooManyBlocks`] is returned for a run longer than the
    /// controller's 16-bit block count (65535). See
    /// `Self::write_blocks_dma` to move the data over DMA instead of the
    /// CPU.
    pub fn write_blocks(
        &self,
        block_index: u32,
        blocks: &[Block],
        timer: &Timer,
    ) -> Result<(), Error> {
        if blocks.is_empty() {
            return Ok(());
        }
        let count = checked_block_count(blocks.len())?;
        self.write_blocks_pio(block_index, count, blocks.iter(), timer)
    }

    /// Writes `blocks.len()` consecutive 512-byte blocks starting at
    /// `block_index` from `blocks` over the system DMA controller
    /// (`channel`), rather than the CPU pushing each word into the `DATA`
    /// FIFO — the throughput win over [`Self::write_blocks`], and the
    /// write-side mirror of [`Self::read_blocks_dma`] (see there for the
    /// command-then-DMA ordering and the channel/alignment notes). Waits
    /// for transfer-complete before returning, so success means the data
    /// is committed to the card.
    ///
    /// An empty slice is a no-op; [`Error::TooManyBlocks`] is returned for
    /// a run past the controller's 16-bit block count and [`Error::Dma`]
    /// for a channel length limit or a hardware error mid-transfer.
    ///
    /// Not available under `bcm2711` -- see this module's "BCM2711" doc
    /// section.
    #[cfg(not(feature = "bcm2711"))]
    pub fn write_blocks_dma(
        &self,
        block_index: u32,
        blocks: &[Block],
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<(), Error> {
        if blocks.is_empty() {
            return Ok(());
        }
        let count = checked_block_count(blocks.len())?;
        let cmd = if count == 1 {
            CMD_WRITE_SINGLE
        } else {
            CMD_WRITE_MULTI
        };
        self.start_transfer(cmd, block_index, count, timer)?;
        channel
            .copy_to_peripheral(blocks.as_flattened(), DMA_DREQ_EMMC, DATA_REG_BUS_ADDRESS)
            .map_err(Error::Dma)?;
        self.wait_interrupt(INT_DATA_DONE, cmd, timer)?;
        Ok(())
    }

    /// PIO core shared by [`Self::read_block`]/[`Self::read_blocks`]:
    /// issues the read (single- or multi-block, per `count`) then drains
    /// `count` blocks out of the `DATA` FIFO a 32-bit word at a time, each
    /// the little-endian packing of four consecutive bytes.
    /// `as_chunks_mut::<4>` splits a block's 512 bytes into 128 `[u8; 4]`
    /// groups (`.0`; the remainder `.1` is empty) and avoids
    /// reinterpreting the buffer as `*u32`, which would assume a 4-byte
    /// alignment `[u8; 512]` doesn't guarantee. `blocks` must yield
    /// exactly `count` blocks (the callers guarantee this).
    fn read_blocks_pio<'a>(
        &self,
        block_index: u32,
        count: u16,
        blocks: impl Iterator<Item = &'a mut Block>,
        timer: &Timer,
    ) -> Result<(), Error> {
        let cmd = if count == 1 {
            CMD_READ_SINGLE
        } else {
            CMD_READ_MULTI
        };
        self.start_transfer(cmd, block_index, count, timer)?;
        for block in blocks {
            // `READ_RDY` fires once per block, each time the controller's
            // buffer has that block's worth of data ready to drain.
            self.wait_interrupt(INT_READ_RDY, cmd, timer)?;
            for chunk in block.as_chunks_mut::<4>().0 {
                *chunk = self.emmc.data().read().bits().to_le_bytes();
            }
        }
        // A multi-block read ends with an auto-`CMD12`; wait for
        // transfer-complete so any auto-command error surfaces and the
        // controller is idle before the next command. A single-block read
        // has no stop command and its `DATA_DONE` isn't load-bearing, so
        // it's skipped there to leave that path identical to before.
        if count > 1 {
            self.wait_interrupt(INT_DATA_DONE, cmd, timer)?;
        }
        Ok(())
    }

    /// PIO core shared by [`Self::write_block`]/[`Self::write_blocks`]:
    /// issues the write (single- or multi-block, per `count`), pushes each
    /// block into the `DATA` FIFO a 32-bit word at a time (the inverse of
    /// [`Self::read_blocks_pio`]'s drain; see there on `as_chunks`), then
    /// waits for transfer-complete. `blocks` must yield exactly `count`
    /// blocks (the callers guarantee this).
    fn write_blocks_pio<'a>(
        &self,
        block_index: u32,
        count: u16,
        blocks: impl Iterator<Item = &'a Block>,
        timer: &Timer,
    ) -> Result<(), Error> {
        let cmd = if count == 1 {
            CMD_WRITE_SINGLE
        } else {
            CMD_WRITE_MULTI
        };
        self.start_transfer(cmd, block_index, count, timer)?;
        for block in blocks {
            self.wait_interrupt(INT_WRITE_RDY, cmd, timer)?;
            for &chunk in block.as_chunks::<4>().0 {
                self.emmc
                    .data()
                    .write(|w| unsafe { w.bits(u32::from_le_bytes(chunk)) });
            }
        }
        // The final `DATA_DONE` only fires once the card has programmed
        // the last block (and, for a multi-block write, the auto-`CMD12`
        // has completed), so waiting here means a successful return is a
        // committed write, not merely a filled FIFO.
        self.wait_interrupt(INT_DATA_DONE, cmd, timer)?;
        Ok(())
    }

    /// Common front half of every block transfer: waits for the data
    /// lines to be free, programs `BLKSIZECNT` for `block_count` 512-byte
    /// blocks, and issues `cmd` addressed at `block_index`. The argument
    /// sent is a byte offset for byte-addressed SDSC cards and a block
    /// index for block-addressed SDHC/SDXC cards (see
    /// [`Self::high_capacity`]). The caller does the direction-specific
    /// data-phase handshake afterward.
    fn start_transfer(
        &self,
        cmd: u32,
        block_index: u32,
        block_count: u16,
        timer: &Timer,
    ) -> Result<(), Error> {
        self.wait_data_ready(timer)?;

        self.emmc
            .blksizecnt()
            .write(|w| unsafe { w.blksize().bits(512).blkcnt().bits(block_count) });

        let arg = if self.high_capacity {
            block_index
        } else {
            block_index * 512
        };
        // The single R1 response `command` returns isn't needed here:
        // the block's status comes from the data-phase interrupts the
        // caller waits on, not this command's response word.
        self.command(cmd, arg, timer).map(|_| ())
    }

    /// Reads the card's SCR (SD Configuration Register — an 8-byte,
    /// single-block data read via `ACMD51`) and, if it advertises 4-bit
    /// bus support ([`SCR_SD_BUS_WIDTH_4`]), switches to it: `ACMD6`
    /// (`SET_BUS_WIDTH`) tells the card, `CONTROL0.HCTL_DWIDTH` tells
    /// this controller. Returns whether the switch happened — `Ok(false)`
    /// for a card that doesn't support it is not an error, just the SD
    /// default 1-bit bus staying in effect.
    fn negotiate_four_bit_bus(&self, timer: &Timer) -> Result<bool, Error> {
        self.wait_data_ready(timer)?;
        self.emmc
            .blksizecnt()
            .write(|w| unsafe { w.blksize().bits(8).blkcnt().bits(1) });
        self.command(CMD_SEND_SCR, 0, timer)?;
        self.wait_interrupt(INT_READ_RDY, CMD_SEND_SCR, timer)?;

        let mut scr = [0u32; 2];
        for word in &mut scr {
            *word = self.emmc.data().read().bits();
        }
        if scr[0] & SCR_SD_BUS_WIDTH_4 == 0 {
            return Ok(false);
        }

        self.command(CMD_SET_BUS_WIDTH, BUS_WIDTH_4BIT, timer)?;
        self.emmc
            .control0()
            .modify(|_, w| w.hctl_dwidth().set_bit());
        Ok(true)
    }

    /// Waits for the data lines to be free — the prerequisite every
    /// data-bearing command ([`Self::read_block`],
    /// [`Self::negotiate_four_bit_bus`]'s SCR read) needs before
    /// issuing.
    fn wait_data_ready(&self, timer: &Timer) -> Result<(), Error> {
        wait_for(timer, 100_000, || {
            self.emmc.status().read().dat_inhibit().bit_is_clear()
        })
    }

    /// Issues one command: waits for the command line to be free,
    /// clears any stale interrupt status, writes the argument and
    /// command, and waits for `CMD_DONE` (or an error). Returns
    /// `RESP0` — sufficient for every command this driver sends; none
    /// of them need the 136-bit `RESP0..RESP3` response CMD2 (`R2`)
    /// carries, since this driver never decodes the CID itself. `code`
    /// may have [`CMD_NEED_APP`] OR'd in (see [`CMD_SEND_OP_COND`]),
    /// which sends an `APP_CMD` (CMD55) immediately before it, with
    /// `self.rca` as CMD55's own argument — `0` while called from
    /// [`Self::init`]'s own `ACMD41` (sent before the card is assigned
    /// an RCA, so `0` is the spec-correct argument there too), the real
    /// RCA for any application command sent after.
    fn command(&self, code: u32, arg: u32, timer: &Timer) -> Result<u32, Error> {
        if code & CMD_NEED_APP != 0 {
            self.command(CMD_APP_CMD, self.rca, timer)?;
        }
        let code = code & !CMD_NEED_APP;

        wait_for(timer, 100_000, || {
            self.emmc.status().read().cmd_inhibit().bit_is_clear()
        })?;

        // Clear any stale interrupt bits from a previous command
        // before issuing this one, by writing back whatever's
        // currently set (every `INTERRUPT` bit is write-1-to-clear).
        let stale = self.emmc.interrupt().read().bits();
        self.emmc.interrupt().write(|w| unsafe { w.bits(stale) });

        self.emmc.arg1().write(|w| unsafe { w.bits(arg) });
        self.emmc.cmdtm().write(|w| unsafe { w.bits(code) });

        self.wait_interrupt(INT_CMD_DONE, code, timer)?;

        Ok(self.emmc.resp0().read().bits())
    }

    /// Waits for `mask` (or any [`INT_ERROR_MASK`] bit) to appear in
    /// `INTERRUPT` and consumes it — see [`Self::poll_interrupt`], which
    /// does the consuming and documents what it clears. This is the
    /// spinning form; `command` is passed through only to
    /// [`Error::CardError`]/[`Error::WaitTimeout`]'s diagnostic fields.
    fn wait_interrupt(&self, mask: u32, command: u32, timer: &Timer) -> Result<u32, Error> {
        let start = timer.now_micros();
        loop {
            if let Some(result) = self.poll_interrupt(mask, command) {
                return result;
            }
            if timer.now_micros() - start > 1_000_000 {
                return Err(Error::WaitTimeout {
                    waiting_for: mask,
                    interrupt: self.emmc.interrupt().read().bits(),
                    status: self.emmc.status().read().bits(),
                    command,
                });
            }
        }
    }

    /// Reads `INTERRUPT` once and, if `mask` or any [`INT_ERROR_MASK`]
    /// bit is set there, consumes it: clears *only* those bits and
    /// returns the full register value from just before clearing (or
    /// [`Error::CardError`] if an error bit was among them). `None` means
    /// nothing being waited for has appeared yet — the caller decides
    /// whether to spin ([`Self::wait_interrupt`]) or park on the
    /// controller's interrupt (the `async` path).
    ///
    /// `command` is the `CMDTM` code of the command whose completion or
    /// data phase is being awaited (e.g. the same command for both
    /// `CMD_DONE` and its following `READ_RDY`/`WRITE_RDY`/`DATA_DONE`),
    /// carried only into [`Error::CardError`]'s diagnostic field.
    ///
    /// Clearing only `mask | INT_ERROR_MASK`, rather than every set bit,
    /// is load-bearing: a write command's `WRITE_RDY` (buffer-write-ready)
    /// asserts the instant the command completes — the host buffer is
    /// empty, so there's immediately room — which means it's often
    /// already set when this returns from the `CMD_DONE` wait. Writing
    /// the whole register back to clear would wipe that `WRITE_RDY` out,
    /// and the following [`Self::write_block`] wait for it would then
    /// hang forever. (Reads don't hit this: `READ_RDY` only asserts once
    /// the card has actually shipped data, well after `CMD_DONE`, so it's
    /// never set at clear-time.)
    ///
    /// Every consumer goes through here for that reason: the rule is
    /// subtle enough that a second copy of it — in an interrupt handler,
    /// say — would be a second chance to get it wrong.
    fn poll_interrupt(&self, mask: u32, command: u32) -> Option<Result<u32, Error>> {
        let full_mask = mask | INT_ERROR_MASK;
        let interrupt = self.emmc.interrupt().read().bits();
        if interrupt & full_mask == 0 {
            return None;
        }
        self.emmc
            .interrupt()
            .write(|w| unsafe { w.bits(interrupt & full_mask) });
        if interrupt & INT_ERROR_MASK != 0 {
            return Some(Err(Error::CardError { interrupt, command }));
        }
        Some(Ok(interrupt))
    }
}

/// Routes GPIO48-53 (`CLK`/`CMD`/`DAT0..DAT3`) to this controller
/// (alternate function 7) with their pull resistors set to pull-up
/// (see [`set_emmc_pull_up`] on why pull-up, unlike `uart.rs`
/// disabling its own pins' pulls entirely).
///
/// Not compiled under `bcm2711`, and that is about the board rather
/// than the chip. EMMC2 — the controller a Pi 4's card slot is actually
/// wired to — drives dedicated pads outside the 54-pin bank, which is
/// why `bcm2711.dtsi`'s `emmc2` node carries no `pinctrl` property at
/// all, and why the Pi 4 SD path works without any muxing here. What
/// those six pins carry on that board is the gigabit Ethernet PHY's
/// RGMII interface (`bcm2711-rpi-4-b.dts` names GPIO48-53
/// `RGMII_RXD0`..`RXD3`, `RGMII_TXCLK`, `RGMII_TXCTL`), so muxing them
/// to ALT3 there severs the MAC from the PHY — and points four lines
/// the PHY drives at a host controller that drives them back during a
/// transfer. ALT3 does still select SD1 on BCM2711, which is what a
/// `bcm2711-lpa`/`bcm2837-lpa` diff shows and what this comment used to
/// rest on; a PAC diff describes the SoC's function numbering and can't
/// see what a board wired to the pads.
#[cfg(not(feature = "bcm2711"))]
fn route_gpio_to_emmc(gpio: &GPIO) {
    gpio.gpfsel4().modify(|_, w| {
        w.fsel48()
            .bits(GPIO_ALT_FUNCTION)
            .fsel49()
            .bits(GPIO_ALT_FUNCTION)
    });
    gpio.gpfsel5().modify(|_, w| {
        w.fsel50()
            .bits(GPIO_ALT_FUNCTION)
            .fsel51()
            .bits(GPIO_ALT_FUNCTION)
            .fsel52()
            .bits(GPIO_ALT_FUNCTION)
            .fsel53()
            .bits(GPIO_ALT_FUNCTION)
    });

    set_emmc_pull_up(gpio);
}

/// Sets GPIO48-53's pull resistors to pull-up. The SD bus (particularly
/// `CMD`) needs a pull-up, not the pulls disabled the way `uart.rs`
/// leaves UART0's pins — matching the reference driver's own choice
/// here, and standard SD electrical requirements (open-drain-ish
/// behavior before the bus is fully driven).
///
/// The register access lives in [`crate::gpio`], which handles the
/// BCM2836/2837-vs-BCM2711 split; these pins all sit in bank 1
/// (GPIO32-53).
#[cfg(not(feature = "bcm2711"))]
fn set_emmc_pull_up(gpio: &GPIO) {
    let mask = (1 << (GPIO_CLK - 32))
        | (1 << (GPIO_CMD - 32))
        | GPIO_DAT
            .iter()
            .fold(0, |mask, pin| mask | (1 << (pin - 32)));
    crate::gpio::set_pull_bank(gpio, 1, mask, crate::gpio::Pull::Up);
}

/// Sets the SD clock to as close to (at or below) `target_hz` as this
/// controller can manage from a `base_hz` source, using the SDHCI 3.0
/// 10-bit divided-clock divisor split across `CLK_FREQ8` (low 8 bits)
/// and `CLK_FREQ_MS2` (high 2 bits). Ported from rsta2's `circle`
/// `GetClockDivider`.
///
/// The 10 bits (not the 8 an earlier version used) are load-bearing on
/// this board: firmware reports a 200MHz base clock, and reaching the
/// SD spec's ≤400kHz identification clock from it needs a divisor of
/// ~500, past what 8 bits can select — an 8-bit divisor tops out at
/// ÷256 (~780kHz, too fast), so the card never responded.
fn set_clock(emmc: &SdEmmc, base_hz: u32, target_hz: u32, timer: &Timer) -> Result<(), Error> {
    wait_for(timer, 100_000, || {
        let status = emmc.status().read();
        status.cmd_inhibit().bit_is_clear() && status.dat_inhibit().bit_is_clear()
    })?;

    emmc.control1().modify(|_, w| w.clk_en().clear_bit());
    timer.delay_ms(10);

    let (freq8, freq_ms2) = clock_divider(base_hz, target_hz);
    emmc.control1()
        .modify(|_, w| unsafe { w.clk_freq8().bits(freq8).clk_freq_ms2().bits(freq_ms2) });
    timer.delay_ms(10);

    emmc.control1().modify(|_, w| w.clk_en().set_bit());
    timer.delay_ms(10);

    wait_for(timer, 100_000, || {
        emmc.control1().read().clk_stable().bit_is_set()
    })
}

/// Narrows a block-slice length to the `BLKSIZECNT.blkcnt` field's 16
/// bits, the largest run a single (multi-block) command can express —
/// [`Error::TooManyBlocks`] if it doesn't fit.
fn checked_block_count(len: usize) -> Result<u16, Error> {
    u16::try_from(len).map_err(|_| Error::TooManyBlocks)
}

/// Polls `condition` until it's true, up to `timeout_us` microseconds
/// of real elapsed time (matching [`Sd::wait_interrupt`] and the
/// `ACMD41` loop in [`Sd::init`], which time out the same way).
fn wait_for(
    timer: &Timer,
    timeout_us: u64,
    mut condition: impl FnMut() -> bool,
) -> Result<(), Error> {
    let start = timer.now_micros();
    loop {
        if condition() {
            return Ok(());
        }
        if timer.now_micros() - start > timeout_us {
            return Err(Error::Timeout);
        }
    }
}

/// A [`Sd`] card wrapped as an `embedded-sdmmc`
/// [`BlockDevice`](embedded_sdmmc::BlockDevice), so a FAT filesystem can
/// be layered on top. Bundles the card with a borrow of the [`Timer`]
/// every transfer needs for its timeouts.
///
/// Reads ([`read`](embedded_sdmmc::BlockDevice::read)) and writes
/// ([`write`](embedded_sdmmc::BlockDevice::write)) are wired to the
/// driver's polled multi-block paths, so a request for several consecutive
/// blocks goes out as one `CMD18`/`CMD25` rather than one command per
/// block. (DMA isn't used here — that path needs a caller-supplied DMA
/// channel; reach for `Sd::read_blocks_dma`/`Sd::write_blocks_dma`
/// directly when throughput matters.)
/// [`num_blocks`](embedded_sdmmc::BlockDevice::num_blocks) still returns
/// [`SdCardError::Unsupported`] — the driver doesn't read the card's
/// capacity (CSD) yet — but `embedded-sdmmc` takes the block count from
/// the partition table, so it isn't called in practice. Pair this with a
/// [`TimeSource`](embedded_sdmmc::TimeSource) of your own — its timestamp
/// is stamped onto files as they're written — and hand both to
/// [`embedded_sdmmc::VolumeManager`].
///
/// Available only with the `embedded-sdmmc` feature enabled.
#[cfg(feature = "embedded-sdmmc")]
pub struct SdCard<'t> {
    sd: Sd,
    timer: &'t Timer,
}

#[cfg(feature = "embedded-sdmmc")]
impl<'t> SdCard<'t> {
    /// Wraps an initialized [`Sd`] and the [`Timer`] its reads need.
    pub fn new(sd: Sd, timer: &'t Timer) -> Self {
        Self { sd, timer }
    }
}

/// Error type for [`SdCard`]'s
/// [`BlockDevice`](embedded_sdmmc::BlockDevice) implementation.
///
/// Available only with the `embedded-sdmmc` feature enabled.
#[cfg(feature = "embedded-sdmmc")]
#[derive(Debug)]
pub enum SdCardError {
    /// A block read or write failed at the SD driver level; carries the
    /// underlying [`Error`].
    Sd(Error),
    /// A capacity query ([`num_blocks`](embedded_sdmmc::BlockDevice::num_blocks))
    /// was attempted, which this adapter can't serve: the SD driver has
    /// no capacity (CSD) readout, so it can't report a block count.
    Unsupported,
}

#[cfg(feature = "embedded-sdmmc")]
impl From<Error> for SdCardError {
    /// Wraps an SD driver [`Error`] as [`SdCardError::Sd`].
    fn from(e: Error) -> Self {
        SdCardError::Sd(e)
    }
}

#[cfg(feature = "embedded-sdmmc")]
impl embedded_sdmmc::BlockDevice for SdCard<'_> {
    type Error = SdCardError;

    /// Reads one or more consecutive 512-byte blocks in a single
    /// (multi-block, when more than one) polled SD read. The
    /// `embedded_sdmmc::Block`s aren't guaranteed to sit contiguously in
    /// memory, so this drives the FIFO drain block-by-block into each
    /// one's `contents` rather than the contiguous DMA path.
    fn read(
        &self,
        blocks: &mut [embedded_sdmmc::Block],
        start_block_idx: embedded_sdmmc::BlockIdx,
    ) -> Result<(), Self::Error> {
        if blocks.is_empty() {
            return Ok(());
        }
        let count = checked_block_count(blocks.len())?;
        self.sd.read_blocks_pio(
            start_block_idx.0,
            count,
            blocks.iter_mut().map(|block| &mut block.contents),
            self.timer,
        )?;
        Ok(())
    }

    /// Writes one or more consecutive 512-byte blocks in a single
    /// (multi-block, when more than one) polled SD write — the mirror of
    /// [`read`](embedded_sdmmc::BlockDevice::read), pushing each block's
    /// `contents` into the FIFO in turn.
    fn write(
        &self,
        blocks: &[embedded_sdmmc::Block],
        start_block_idx: embedded_sdmmc::BlockIdx,
    ) -> Result<(), Self::Error> {
        if blocks.is_empty() {
            return Ok(());
        }
        let count = checked_block_count(blocks.len())?;
        self.sd.write_blocks_pio(
            start_block_idx.0,
            count,
            blocks.iter().map(|block| &block.contents),
            self.timer,
        )?;
        Ok(())
    }

    /// Always [`SdCardError::Unsupported`] — the driver doesn't read the
    /// card's capacity (CSD) yet. `embedded-sdmmc`'s read path takes the
    /// block count from the partition table instead, so it never calls
    /// this.
    fn num_blocks(&self) -> Result<embedded_sdmmc::BlockCount, Self::Error> {
        Err(SdCardError::Unsupported)
    }
}

/// The transfer unit, where two crates' constants have to agree.
///
/// [`Block`] is `[u8; 512]` and `resident_fat::BLOCK_SIZE` is 512; the
/// `as_chunks` splits in [`SdBlockDevice`] are only well-typed while the two
/// agree, so this takes the value from `resident-fat` rather than repeating
/// the literal. Should that crate ever move off 512, the assertion below
/// says so by name instead of failing as a type mismatch further down.
#[cfg(feature = "resident-fat")]
const BLOCK_LEN: usize = resident_fat::BLOCK_SIZE;

#[cfg(feature = "resident-fat")]
const _: () = assert!(BLOCK_LEN == core::mem::size_of::<Block>());

/// A [`Sd`] card wrapped as a `resident-fat`
/// [`BlockDevice`](resident_fat::BlockDevice), so that crate's FAT32
/// filesystem can be layered on top. Bundles the card with a borrow of the
/// [`Timer`] every transfer needs for its timeouts, exactly as [`SdCard`]
/// does.
///
/// # Why this exists alongside [`SdCard`]
///
/// The two adapters differ in their unit of transfer, not in the filesystem
/// above them. `embedded-sdmmc` moves a slice of 512-byte newtypes, which
/// aren't guaranteed to sit contiguously in memory; `resident-fat` moves a
/// plain `&[u8]` spanning a whole run of consecutive blocks. That byte slice
/// is already exactly what the driver's multi-block path wants, so this
/// adapter splits it with `as_chunks` and hands the pieces straight over —
/// no staging buffer and no copy.
///
/// Reaching `resident-fat` through its own `embedded-sdmmc` bridge and
/// [`SdCard`] works and is the right route for a consumer already invested in
/// that trait, but it pays for the newtype twice: a bounded staging buffer
/// (64 KiB by default), and a copy of every byte in each direction. It also
/// caps [`max_transfer_blocks`] at the staging buffer's size, where this
/// adapter reports the controller's real ceiling.
///
/// # What it reports
///
/// [`max_transfer_blocks`] is 65535, the largest run `BLKSIZECNT.blkcnt` can
/// express — so a multi-megabyte read or write is split by the transfer limit
/// rather than by a buffer, and costs one `CMD18`/`CMD25` per 32 MiB. (DMA
/// isn't used here — that path needs a caller-supplied DMA channel; reach for
/// `Sd::read_blocks_dma`/`Sd::write_blocks_dma` directly when throughput
/// matters.)
///
/// [`block_count`] is `Ok(None)`, meaning "I cannot say", because the driver
/// has no capacity (CSD) readout. That is a case `resident-fat`'s trait
/// admits deliberately: it skips a sanity check on the volume's own size
/// claims and mounts anyway, rather than refusing a good card because the
/// driver one layer down is reticent. Note the contrast with [`SdCard`],
/// whose trait has no way to say it doesn't know and so must return
/// [`SdCardError::Unsupported`].
///
/// # Allocation
///
/// `resident-fat` uses `alloc`, so a binary that enables this feature must
/// register a `#[global_allocator]`. This crate cannot: only the final
/// binary may. See `examples/heap_alloc.rs`.
///
/// Available only with the `resident-fat` feature enabled.
///
/// [`max_transfer_blocks`]: resident_fat::BlockDevice::max_transfer_blocks
/// [`block_count`]: resident_fat::BlockDevice::block_count
#[cfg(feature = "resident-fat")]
pub struct SdBlockDevice<'t> {
    sd: Sd,
    timer: &'t Timer,
}

#[cfg(feature = "resident-fat")]
impl<'t> SdBlockDevice<'t> {
    /// Wraps an initialized [`Sd`] and the [`Timer`] its transfers need.
    pub fn new(sd: Sd, timer: &'t Timer) -> Self {
        Self { sd, timer }
    }

    /// The wrapped card, borrowed.
    ///
    /// `resident-fat` owns the device once a volume is mounted and lends it
    /// back through its own accessors, so this is the way to reach the
    /// driver's own methods — a DMA transfer, say — without unmounting.
    pub fn inner(&self) -> &Sd {
        &self.sd
    }

    /// Unwraps back to the card, dropping the timer borrow.
    pub fn into_inner(self) -> Sd {
        self.sd
    }
}

/// Error type for [`SdBlockDevice`]'s
/// [`BlockDevice`](resident_fat::BlockDevice) implementation.
///
/// Available only with the `resident-fat` feature enabled.
#[cfg(feature = "resident-fat")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SdBlockDeviceError {
    /// A block read or write failed at the SD driver level; carries the
    /// underlying [`Error`].
    Sd(Error),
    /// The transfer started past block 2^32, which the controller's 32-bit
    /// block address cannot reach.
    ///
    /// Refused rather than truncated. The alternative — letting the high
    /// bits fall off — turns an unreachable address into a *reachable* one
    /// and writes to the wrong place on the card, which is the kind of
    /// failure that is only ever diagnosed after the damage.
    ///
    /// A 32-bit block address covers 2 TiB, so nothing short of an SDUC card
    /// can produce this.
    BlockOutOfRange {
        /// The first block of the refused transfer.
        start_block: u64,
    },
}

#[cfg(feature = "resident-fat")]
impl From<Error> for SdBlockDeviceError {
    /// Wraps an SD driver [`Error`] as [`SdBlockDeviceError::Sd`].
    fn from(e: Error) -> Self {
        SdBlockDeviceError::Sd(e)
    }
}

/// Narrows a `resident-fat` block address to the controller's 32 bits.
#[cfg(feature = "resident-fat")]
fn checked_block_index(start_block: u64) -> Result<u32, SdBlockDeviceError> {
    u32::try_from(start_block).map_err(|_| SdBlockDeviceError::BlockOutOfRange { start_block })
}

#[cfg(feature = "resident-fat")]
impl resident_fat::BlockDevice for SdBlockDevice<'_> {
    type Error = SdBlockDeviceError;

    /// Reads a run of consecutive blocks in a single (multi-block, when
    /// longer than one) polled SD read.
    ///
    /// # Panics
    ///
    /// If `blocks.len()` isn't a multiple of 512, which the trait forbids.
    /// Asserted rather than rounded down: `as_chunks_mut` would hand back
    /// the odd tail as a remainder, and ignoring it would fill part of the
    /// caller's buffer, return `Ok`, and leave the rest holding whatever it
    /// held before.
    fn read(&mut self, start_block: u64, blocks: &mut [u8]) -> Result<(), Self::Error> {
        let index = checked_block_index(start_block)?;
        // Zero-copy, and safely so: `Block` is `[u8; 512]`, a type alias
        // rather than a newtype, so the split is a plain reborrow of the
        // caller's buffer with no layout assumption behind it. This is the
        // whole reason the adapter is worth having.
        let (blocks, rest) = blocks.as_chunks_mut::<BLOCK_LEN>();
        assert!(rest.is_empty(), "transfer length must be a multiple of 512");
        if blocks.is_empty() {
            return Ok(());
        }
        let count = checked_block_count(blocks.len())?;
        self.sd
            .read_blocks_pio(index, count, blocks.iter_mut(), self.timer)?;
        Ok(())
    }

    /// Writes a run of consecutive blocks in a single (multi-block, when
    /// longer than one) polled SD write — the mirror of
    /// [`read`](resident_fat::BlockDevice::read), with the same length rule
    /// and the same reason for it.
    ///
    /// Waits for transfer-complete, so a successful return means the card
    /// took the data. `resident-fat` still has its own `sync`, which is
    /// about the filesystem's metadata rather than this.
    ///
    /// # Panics
    ///
    /// If `blocks.len()` isn't a multiple of 512.
    fn write(&mut self, start_block: u64, blocks: &[u8]) -> Result<(), Self::Error> {
        let index = checked_block_index(start_block)?;
        let (blocks, rest) = blocks.as_chunks::<BLOCK_LEN>();
        assert!(rest.is_empty(), "transfer length must be a multiple of 512");
        if blocks.is_empty() {
            return Ok(());
        }
        let count = checked_block_count(blocks.len())?;
        self.sd
            .write_blocks_pio(index, count, blocks.iter(), self.timer)?;
        Ok(())
    }

    /// Always `Ok(None)` — the driver has no capacity (CSD) readout, so it
    /// doesn't know. See the type's documentation for why that is a better
    /// answer here than an error.
    fn block_count(&mut self) -> Result<Option<u64>, Self::Error> {
        Ok(None)
    }

    /// 65535 — the largest run the controller's 16-bit `BLKSIZECNT.blkcnt`
    /// field can express, and so the longest transfer one command can carry.
    fn max_transfer_blocks(&self) -> u64 {
        u64::from(u16::MAX)
    }
}
