//! Blocking driver for the on-board BCM43438 wireless chip's SDIO
//! interface — the bus the Wi-Fi side of the chip is reached over.
//!
//! This drives the *same* Arasan "EMMC" (SDHCI-compatible) host
//! controller as [`crate::sd`], but for a different device: where
//! `sd::Sd` routes the controller to the external SD-card slot
//! (GPIO48-53) and speaks SD memory commands, this routes it to the
//! wireless chip (GPIO34-39, the "SD1" pin group) and speaks SDIO
//! (`CMD5`/`CMD52`/`CMD53`). The two uses are therefore mutually
//! exclusive: the controller can only be muxed to one pin group at a
//! time, so a program that brings up Wi-Fi over this driver gives up
//! the SD-card slot (a program needing both concurrently would have to
//! drive the slot with the separate, simpler "SDHOST" controller
//! instead — not implemented here). This is exactly the split Linux
//! makes on a Pi 3, and why `sd.rs` is documented as safe to point at
//! the slot only because *this* crate doesn't otherwise need SDIO.
//!
//! This gets the chip powered, the SDIO bus enumerated
//! (`CMD0`/`CMD5`/`CMD3`/`CMD7`) at the ≤400kHz identification clock in
//! 1-bit mode, then widened to the 4-bit bus at 25MHz default speed, and
//! function 1 (the backplane register window) enabled — reaching chip
//! registers over that window via `CMD52`
//! ([`backplane_read32`](crate::sdio::Sdio::backplane_read32)/
//! [`backplane_write32`](crate::sdio::Sdio::backplane_write32)) and bulk RAM
//! via `CMD53` ([`backplane_read`](crate::sdio::Sdio::backplane_read)/
//! [`backplane_write`](crate::sdio::Sdio::backplane_write)). On
//! top of that, [`load_firmware`](crate::sdio::Sdio::load_firmware) walks the
//! chip's core
//! enumeration ROM, downloads the firmware image into its RAM and the
//! nvram at the top of RAM, releases the on-chip CPU, and waits for the
//! WLAN data function (F2) to come ready — leaving the chip's firmware
//! running. Speaking the SDPCM/CDC data protocol over F2 (the actual
//! network traffic) is the next stage and isn't here yet;
//! high-speed/UHS clocking past 25MHz isn't set up either.
//!
//! Pi 3 only: the BCM43438 exists on the Pi 3 (`B`/`B+`), not the Pi 2.
//!
//! Register sequencing follows the same two hardware-confirmed
//! bare-metal references `sd.rs` cites (bztsrc's `raspi3-tutorial` and
//! rsta2's `circle`) for the shared EMMC controller mechanics, and, for
//! the SDIO-specific parts (the `CMD5` op-cond negotiation, `CMD52`
//! direct register access, the function-1 backplane address window, and
//! the ChipCommon layout), Linux's `brcmfmac` driver and `circle`'s
//! `addon/wlan` port, which agree on every register value used here.

use crate::emmc::clock_divider;
use crate::mailbox::{ClockId, Mailbox, PowerDeviceId, EXPANDER_WL_ON};
use crate::pac::{EMMC, GPIO};
use crate::timer::Timer;

/// GPIO pins carrying the SDIO bus to the wireless chip
/// (`CLK`/`CMD`/`DAT0..DAT3`), the "SD1" pin group — distinct from the
/// SD-card slot's GPIO48-53 that [`crate::sd`] uses. The Arasan
/// controller reaches the wireless chip only on these pins.
const GPIO_CLK: u32 = 34;
/// SDIO command line (`CMD`).
const GPIO_CMD: u32 = 35;
/// SDIO data lines (`DAT0`..`DAT3`).
const GPIO_DAT: [u32; 4] = [36, 37, 38, 39];
/// GPIO alternate function (ALT3) routing GPIO34-39 to the Arasan
/// controller. The datasheet's alternate-function table prints these
/// cells as reserved, but the Linux device tree, `circle`, and Plan 9's
/// bare-metal port all route the controller here with ALT3.
const GPIO_ALT_FUNCTION: u8 = 0b111;
/// GPIO alternate function (ALT0) that routes the SD-card slot pins
/// (GPIO48-53) to the *other* SD controller, SDHOST — used only to pull
/// those pins off the shared Arasan/EMMC controller (which the boot
/// firmware leaves them on) so it drives the wireless pins alone. See
/// [`route_gpio_to_emmc`].
const SD_SLOT_ALT_FUNCTION: u8 = 0b100;

/// GPIO peripheral base — see [`crate::sd`]'s identical constant: the
/// legacy pull registers below aren't in `bcm2837-lpa`'s SVD, so this
/// pokes the known physical addresses directly.
const GPIO_BASE: usize = 0x3f20_0000;
/// GPIO Pull-up/down Enable (BCM2835 ARM Peripherals datasheet §6.1).
const GPPUD: *mut u32 = (GPIO_BASE + 0x94) as *mut u32;
/// GPIO Pull-up/down Enable Clock 1, covers GPIO32-53 — the pins this
/// driver uses (34-39) all fall here.
const GPPUDCLK1: *mut u32 = (GPIO_BASE + 0x9c) as *mut u32;
/// `GPPUD` value selecting pull-up (for `CMD`/`DAT0..DAT3`).
const GPPUD_PULL_UP: u32 = 2;
/// `GPPUD` value disabling the pull (for `CLK`). Unlike the SD-card
/// route — which pulls every line up — the wireless SDIO route leaves
/// `CLK` with no pull, matching the Linux device tree's pin config for
/// this exact pin group.
const GPPUD_PULL_NONE: u32 = 0;

/// Delay after asserting `WL_ON` before touching the bus, letting the
/// chip's power rails and internal reset settle. Not a datasheet-
/// mandated figure — the Linux power-sequence node sets no delay — but
/// a conservative convention (`circle` uses the same 150ms elsewhere).
const WL_ON_SETTLE_MS: u32 = 150;

/// Target SDIO clock during identification/initialization — the SDIO
/// spec's ≤400kHz ceiling, same as the SD physical layer's.
const SETUP_CLOCK_HZ: u32 = 400_000;
/// Target SDIO clock once the chip is enumerated and on the 4-bit bus:
/// the SDIO "default speed" rate (25MHz), well within what the chip and
/// this controller handle without high-speed timing. High-speed/UHS
/// rates past this aren't set up.
const TRANSFER_CLOCK_HZ: u32 = 25_000_000;

/// `CONTROL1.DATA_TOUNIT` value: the second-longest data timeout
/// (`0b1111` disables it entirely, which would hang forever). See
/// [`crate::sd`]'s identical constant.
const DATA_TIMEOUT_MAX: u8 = 0b1110;

/// `CMDTM` value for CMD0 (`GO_IDLE_STATE`) — no response. Same packed
/// encoding scheme as [`crate::sd`]'s command codes (index in bits
/// 24:29, response type in bits 16:18).
const CMD_GO_IDLE: u32 = 0x0000_0000;
/// `CMDTM` value for CMD3 (`SEND_RELATIVE_ADDR`) — R6 (48-bit)
/// response. On SDIO the card *publishes* its own RCA in the response
/// (bits 31:16), rather than being assigned one.
const CMD_SEND_REL_ADDR: u32 = 0x0302_0000;
/// `CMDTM` value for CMD5 (`IO_SEND_OP_COND`) — R4 (48-bit) response.
/// The SDIO analog of SD's `ACMD41`: negotiates the operating voltage
/// window and reports when the chip has finished powering up. No CRC
/// check is enabled (this controller only checks CRC when told to, and
/// R4 carries no valid CRC anyway).
const CMD_IO_SEND_OP_COND: u32 = 0x0502_0000;
/// `CMDTM` value for CMD7 (`SELECT_CARD`) — R1b (48-bit + busy)
/// response.
const CMD_CARD_SELECT: u32 = 0x0703_0000;
/// `CMDTM` value for CMD52 (`IO_RW_DIRECT`) — R5 (48-bit) response.
/// The single-byte SDIO register access this driver uses for
/// everything: reading/writing CCCR registers, enabling function 1,
/// and pointing the backplane address window. The read/written byte
/// travels in the argument (see [`cmd52_arg`]) and, on a read, comes
/// back in the low byte of `RESP0`.
const CMD_IO_RW_DIRECT: u32 = 0x3402_0000;
/// `CMDTM` value for CMD53 (`IO_RW_EXTENDED`) reading data — R5
/// (48-bit) response, data present, card-to-host direction (bit 4).
/// The bulk multi-byte SDIO transfer, moving data through the
/// controller's `DATA` FIFO rather than the command response: whether
/// it's byte- or block-mode, incrementing or fixed, and how many
/// bytes/blocks, all live in the *argument* (see [`cmd53_arg`]), not
/// this code. Used here for backplane block reads and, later, for
/// streaming the chip's firmware image over function 1.
const CMD_IO_RW_EXTENDED_READ: u32 = 0x3522_0010;
/// `CMDTM` value for CMD53 (`IO_RW_EXTENDED`) writing data — as
/// [`CMD_IO_RW_EXTENDED_READ`] but host-to-card (data-direction bit
/// clear). The write half of the bulk path, used to stream the firmware
/// image into chip RAM.
const CMD_IO_RW_EXTENDED_WRITE: u32 = 0x3522_0000;

/// Mask over the `INTERRUPT` register's error bits — see [`crate::sd`].
const INT_ERROR_MASK: u32 = 0x017e_8000;
/// `INTERRUPT.CMD_DONE`.
const INT_CMD_DONE: u32 = 0x0000_0001;
/// `INTERRUPT.DATA_DONE` — a data transfer (read or write) has
/// completed.
const INT_DATA_DONE: u32 = 0x0000_0002;
/// `INTERRUPT.WRITE_RDY` — the controller's `DATA` FIFO can accept a
/// block to be written out.
const INT_WRITE_RDY: u32 = 0x0000_0010;
/// `INTERRUPT.READ_RDY` — the controller's `DATA` FIFO holds a block
/// ready to be drained.
const INT_READ_RDY: u32 = 0x0000_0020;

/// SDIO function number 0: the always-accessible standard/CCCR register
/// space (card capabilities, per-function enable, bus width).
const FN0: u32 = 0;
/// SDIO function number 1: the chip's backplane register window, used
/// to reach the ChipCommon core (and, later, to load firmware).
const FN1: u32 = 1;

/// CCCR "I/O Enable" register (function-0 address `0x02`): bit N
/// enables function N.
const CCCR_IO_ENABLE: u32 = 0x02;
/// CCCR "I/O Ready" register (function-0 address `0x03`): bit N reads
/// set once function N is ready for use.
const CCCR_IO_READY: u32 = 0x03;
/// CCCR "Bus Interface Control" register (function-0 address `0x07`):
/// its low two bits select the SDIO bus width.
const CCCR_BUS_IFACE: u32 = 0x07;
/// Bus-width field mask within [`CCCR_BUS_IFACE`].
const BUS_WIDTH_MASK: u8 = 0x03;
/// [`CCCR_BUS_IFACE`] bus-width value selecting the 4-bit bus (`0` is
/// the 1-bit default enumeration uses).
const BUS_WIDTH_4BIT: u8 = 0x02;

/// CMD5 argument: the operating voltage window the host offers. Bits
/// 20-21 (3.2-3.4V) are the only ones the BCM43438 cares about; this
/// offers the whole 2.7-3.6V range (bits 15-23) the way Linux's generic
/// host does, which the chip masks down to what it supports.
const OCR_VOLTAGE_WINDOW: u32 = 0x00ff_8000;
/// CMD5's R4 response bit signalling the chip has finished powering up
/// and is ready to be enumerated.
const IO_OP_COND_READY: u32 = 0x8000_0000;

/// Function-1 register holding backplane address bits 15:8 of the
/// active 32KB window (`SBSDIO_FUNC1_SBADDRLOW`).
const FUNC1_SBADDR_LOW: u32 = 0x1_000a;
/// Function-1 register holding backplane address bits 23:16
/// (`SBSDIO_FUNC1_SBADDRMID`).
const FUNC1_SBADDR_MID: u32 = 0x1_000b;
/// Function-1 register holding backplane address bits 31:24
/// (`SBSDIO_FUNC1_SBADDRHIGH`).
const FUNC1_SBADDR_HIGH: u32 = 0x1_000c;
/// Mask selecting the 15-bit offset *within* the active backplane
/// window (`SBSDIO_SB_OFT_ADDR_MASK`); the high bits above this pick
/// the window and are set via the three `SBADDR*` registers.
const SB_WINDOW_OFFSET_MASK: u32 = 0x0000_7fff;
/// Mask selecting the window base (everything above the 15-bit offset)
/// from a full backplane address (`SBSDIO_SBWINDOW_MASK`).
const SB_WINDOW_BASE_MASK: u32 = 0xffff_8000;
/// Bit 15 of a function-1 access address, forcing a 32-bit-wide
/// backplane cycle rather than an 8-bit one
/// (`SBSDIO_SB_ACCESS_2_4B_FLAG`). Set on every access used to read the
/// 32-bit ChipCommon ID.
const SB_ACCESS_32BIT: u32 = 0x0000_8000;

/// Backplane address of the ChipCommon core's enumeration space
/// (`SI_ENUM_BASE_DEFAULT`); its first register is the chip ID.
const CHIPCOMMON_BASE: u32 = 0x1800_0000;
/// Mask over the ChipCommon ID register's low half — the chip id
/// proper (`CID_ID_MASK`). The rest of the register is revision,
/// package, core count, and backplane type, none of which this
/// bring-up needs.
const CHIP_ID_MASK: u32 = 0x0000_ffff;
/// The chip id a Pi 3 `B`'s BCM43438 reports. The "43438" on the module
/// is a marketing number; the silicon's ChipCommon core identifies as
/// 43430 (`0xA9A6`). A Pi 3 `B+` / Zero 2 W reports a different value
/// (43436/43455), so callers that want to accept those should read
/// [`Sdio::chip_id`] rather than compare against this.
pub const BCM43438_CHIP_ID: u32 = 0xa9a6;

/// ChipCommon register holding the pointer (a backplane address) to the
/// enumeration ROM (EROM) — the table [`Sdio::scan_cores`] walks to
/// discover the chip's internal cores. ChipCommon register 63, i.e.
/// [`CHIPCOMMON_BASE`] `+ 0xFC`.
const CHIPCOMMON_EROM_PTR: u32 = CHIPCOMMON_BASE + 0xfc;

/// Core ID of the ARM Cortex-M3 — the CPU on the 43430 that runs the
/// downloaded firmware; held in reset until the download completes.
const CORE_ARM_CM3: u16 = 0x82a;
/// Core ID of the internal RAM (SOCRAM/TCM) the firmware is downloaded
/// into.
const CORE_SOCRAM: u16 = 0x80e;
/// Core ID of the SDIO device core — its mailbox register carries the
/// host↔firmware ready handshake once the CPU is running.
const CORE_SDIO: u16 = 0x829;
/// Core ID of the D11 (802.11 MAC) core — held quiescent in reset
/// during the firmware download.
const CORE_D11: u16 = 0x812;

/// EROM descriptor type (low 4 bits): a component-identification word.
/// Two in a row (CIA then CIB) introduce a core.
const EROM_DESC_COMPONENT: u32 = 0x1;
/// EROM descriptor type: an address descriptor, giving one of a core's
/// register/wrapper base addresses.
const EROM_DESC_ADDRESS: u32 = 0x5;
/// EROM descriptor type: end of the table.
const EROM_DESC_EOT: u32 = 0xf;
/// Address-descriptor type field (bits 6:7): zero for a core's register
/// (slave) base, non-zero for a bridge or master/slave wrapper. The
/// wrapper base is where a core's reset/clock control registers live.
const EROM_ADDR_TYPE_MASK: u32 = 0xc0;

/// Wrapper register offset: the core-control register (`ioctrl`),
/// carrying a core's clock-enable and force-gated-clock bits.
const WRAP_IOCTRL: u32 = 0x408;
/// Wrapper register offset: the reset-control register (`resetctrl`);
/// bit 0 holds or releases the core's reset.
const WRAP_RESETCTRL: u32 = 0x800;
/// `ioctrl` bit: enable the core's clock.
const WRAP_CLK: u32 = 0x1;
/// `ioctrl` bit: force the gated clock on (held during reset).
const WRAP_FGC: u32 = 0x2;
/// `resetctrl` bit: the core is held in reset.
const WRAP_RESET: u32 = 0x1;

/// `ioctrl` bit for the D11 core: keep its PHY clock enabled.
const D11_PHY_CLOCK_EN: u32 = 0x4;
/// `ioctrl` bit for the D11 core: assert PHY reset.
const D11_PHY_RESET: u32 = 0x8;

/// Function-1 SDIO register selecting the chip's backplane clock
/// (`SBSDIO_FUNC1_CHIPCLKCSR`), reached by `CMD52` — not a backplane
/// address. Must request the ALP clock through here before reaching the
/// chip's internal cores/RAM over the backplane.
const FUNC1_CHIPCLKCSR: u32 = 0x1_000e;
/// Function-1 SDIO register disabling the SDIO data-line pull-ups
/// (`SBSDIO_FUNC1_SDIOPULLUP`), cleared as part of the ALP-clock setup.
const FUNC1_SDIOPULLUP: u32 = 0x1_000f;
/// `CHIPCLKCSR` bit: request the ALP (low-power) clock.
const CLKCSR_ALP_AVAIL_REQ: u8 = 0x08;
/// `CHIPCLKCSR` bit: force the ALP clock on.
const CLKCSR_FORCE_ALP: u8 = 0x01;
/// `CHIPCLKCSR` bit: squelch hardware clock requests, so the host's
/// explicit request is what drives the clock.
const CLKCSR_FORCE_HW_CLKREQ_OFF: u8 = 0x20;
/// `CHIPCLKCSR` status bit: the ALP clock is available.
const CLKCSR_ALP_AVAIL: u8 = 0x40;
/// `CHIPCLKCSR` bit: request the HT (high-throughput) clock — needed
/// once the firmware is running, before enabling the WLAN data function.
const CLKCSR_HT_AVAIL_REQ: u8 = 0x10;
/// `CHIPCLKCSR` bit: force the HT clock on.
const CLKCSR_FORCE_HT: u8 = 0x02;
/// `CHIPCLKCSR` status bit: the HT clock is available.
const CLKCSR_HT_AVAIL: u8 = 0x80;

/// SDIO function number 2: the WLAN data function. It only comes ready
/// after the downloaded firmware boots, which is how [`Sdio::load_firmware`]
/// confirms success.
const FN2: u32 = 2;
/// SDIO device-core register offset carrying the host→firmware mailbox
/// word (`Sbmboxdata`), written with the SDPCM protocol version to hand
/// the running firmware the channel version it expects.
const SDIO_CORE_SB_MBOX_DATA: u32 = 0x48;
/// SDPCM protocol version written into [`SDIO_CORE_SB_MBOX_DATA`], in
/// its high half, at firmware-ready handshake time.
const SDPCM_PROT_VERSION: u32 = 4;

/// Bytes per `CMD53` chunk when streaming the firmware image into RAM:
/// 512, the byte-mode maximum, and a divisor of the 32KB backplane
/// window so a chunk never straddles a window boundary.
const FIRMWARE_CHUNK_BYTES: usize = 512;
/// Maximum words moved in one `CMD53` data transfer. The controller's
/// PIO data buffer is 64 bytes, and a single transfer that pushes more
/// than that through the `DATA` FIFO after one buffer-ready wait
/// corrupts the write (a data-CRC error); 64 bytes is also the wireless
/// chip's function-1 block size. [`Sdio::backplane_read`]/
/// [`Sdio::backplane_write`] split larger buffers into transfers of this
/// size — which, being a divisor of the 32KB window, also means no
/// single transfer straddles a window boundary.
const MAX_TRANSFER_WORDS: usize = 16;
/// Scratch capacity for the condensed nvram. The processed form is
/// smaller than the source `.txt` (comments/blank lines stripped); a
/// Pi 3's `.txt` is under 1KB, so this is generous headroom.
const NVRAM_MAX_BYTES: usize = 2048;

/// SOCRAM register offset: `coreinfo`, whose bank-count field sizes the
/// per-bank loop in [`Sdio::ram_size`].
const SOCRAM_COREINFO: u32 = 0x00;
/// SOCRAM register offset: `bankidx`, selecting which bank
/// `SOCRAM_BANKINFO` reports on.
const SOCRAM_BANKIDX: u32 = 0x10;
/// SOCRAM register offset: `bankinfo`, whose low bits give the selected
/// bank's size in [`SOCRAM_BANK_SIZE_UNIT`] units (minus one).
const SOCRAM_BANKINFO: u32 = 0x40;
/// SOCRAM register offset: `bankpda`, written to disable the 43430's
/// bank-3 remap so all of RAM is plain, writable memory.
const SOCRAM_BANKPDA: u32 = 0x44;
/// Mask over `bankinfo`'s size field.
const SOCRAM_BANKINFO_SIZE_MASK: u32 = 0x7f;
/// Byte size of one `bankinfo` size unit (each bank is `(field + 1)`
/// of these).
const SOCRAM_BANK_SIZE_UNIT: u32 = 8192;
/// Mask/shift extracting the number of RAM banks from `coreinfo`.
const SOCRAM_COREINFO_BANKS_MASK: u32 = 0xf0;
/// The 43430's RAM bank whose remap must be disabled during download.
const SOCRAM_REMAP_BANK: u32 = 3;

/// Errors from [`Sdio::init`] and the register-access methods.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// A wait for the controller, chip, or a command/data phase to
    /// reach some state exceeded its budget.
    Timeout,
    /// A command finished with the `INTERRUPT` register's error mask
    /// set.
    CardError {
        /// The `INTERRUPT` register's value when the error was
        /// detected.
        interrupt: u32,
        /// The `CMDTM` command value in flight when the error fired —
        /// its bits 24:29 are the SDIO command index.
        command: u32,
    },
    /// The firmware's "Set Power State" call reported the SD/EMMC
    /// controller's power domain ([`PowerDeviceId::SdCard`]) isn't
    /// present or wouldn't turn on. This controller is shared with the
    /// SD-card slot, so it lives on that same power domain.
    ControllerPowerOnFailed,
    /// Reading the EMMC base clock from the firmware failed, so the
    /// clock divider couldn't be computed.
    ClockQueryFailed,
    /// The chip never reported ready (`CMD5`'s power-up-complete bit)
    /// within the budgeted time — the radio didn't come up.
    OpCondTimeout,
    /// Function 1 (the backplane register window) didn't report ready
    /// after being enabled — the chip enumerated but its register
    /// window isn't accessible.
    Function1NotReady,
    /// Walking the enumeration ROM ([`Sdio::scan_cores`]) didn't turn up
    /// a core the firmware download needs (the ARM CPU, the internal
    /// RAM, or the SDIO core) — either the EROM read back garbage or the
    /// walk ran past its bound without finding an end marker.
    CoreNotFound,
    /// After the download, the SOCRAM core wasn't up when it came time
    /// to release the CPU — the RAM the firmware was written into isn't
    /// reachable, so starting the CPU would be pointless.
    SocramNotUp,
    /// The firmware was downloaded and the CPU released, but the WLAN
    /// data function (F2) never reported ready within the budget — the
    /// firmware didn't come alive (a bad image/nvram, or a download that
    /// didn't land correctly).
    FirmwareNotReady,
    /// A `CMD53` data transfer (a bulk backplane or function-2 read or
    /// write) stalled waiting for a controller interrupt, with the
    /// `INTERRUPT`/`STATUS` registers captured at the timeout for
    /// diagnosis.
    DataStalled {
        /// Which wait timed out: 0 = command-done, 1 = FIFO ready
        /// (accept-for-write or data-for-read), 2 = transfer-done.
        stage: u8,
        /// The `INTERRUPT` register's value at the timeout.
        interrupt: u32,
        /// The `STATUS` register's value at the timeout.
        status: u32,
    },
}

/// The backplane addresses of the internal cores the firmware download
/// needs, as discovered by [`Sdio::scan_cores`] walking the chip's
/// enumeration ROM. Each core is reached at a register base; cores that
/// can be reset also have a separate wrapper base holding their
/// reset/clock-control registers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChipCores {
    /// The ARM Cortex-M3's wrapper base — where its reset/clock control
    /// lives, used to hold it in reset during download and release it
    /// afterward. (The CPU has no registers this driver reads, so only
    /// the wrapper is kept.)
    pub arm_wrapper: u32,
    /// The D11 (802.11 MAC) core's wrapper base — for holding it reset
    /// and quiescent during the download.
    pub d11_wrapper: u32,
    /// The internal RAM (SOCRAM) core's register base — for reading its
    /// bank layout to size RAM, and the 43430-specific bank remap
    /// disable.
    pub socram_base: u32,
    /// The internal RAM core's wrapper base — for bringing the RAM core
    /// up out of reset so it's writable.
    pub socram_wrapper: u32,
    /// The SDIO device core's register base — its mailbox register
    /// carries the firmware-ready handshake.
    pub sdio_base: u32,
}

/// A brought-up SDIO link to the wireless chip, with function 1 (the
/// backplane register window) enabled.
///
/// Build one with [`Self::init`], then read chip registers over the
/// backplane with [`Self::chip_id`] / [`Self::backplane_read32`], or
/// issue raw SDIO register accesses with [`Self::cmd52_read`] /
/// [`Self::cmd52_write`].
pub struct Sdio {
    emmc: EMMC,
    /// The chip's relative card address (`CMD3`'s response, kept in
    /// argument position — bits 31:16), used to select and address it.
    rca: u32,
    /// The base of the backplane window currently pointed at by the
    /// `SBADDR*` registers, cached so [`Self::backplane_read32`] only
    /// reprograms the window when the target moves outside it.
    window_base: u32,
    /// Backplane base of the SDIO device core, discovered during
    /// [`Self::load_firmware`] and kept so the protocol layer can reach
    /// the core's mailbox/interrupt registers (see
    /// [`Self::sdio_core_base`]). Zero until firmware is loaded.
    sdio_core_base: u32,
}

impl Sdio {
    /// Brings the wireless chip's SDIO link up: powers the shared EMMC
    /// controller and the radio, routes GPIO34-39 to the controller,
    /// resets and clocks it, enumerates the SDIO card
    /// (`CMD0`/`CMD5`/`CMD3`/`CMD7`), and enables function 1 so chip
    /// registers can be read over the backplane. Leaves the bus at the
    /// SDIO identification clock in 1-bit mode.
    pub fn init(
        gpio: &GPIO,
        emmc: EMMC,
        mailbox: &mut Mailbox,
        timer: &Timer,
    ) -> Result<Self, Error> {
        // The Arasan controller shares the SD-card slot's power domain
        // (it's the same controller), and comes up unpowered from
        // firmware — see `sd::Sd::init` for why register access can
        // still appear to work while the analog side is unpowered.
        if !matches!(
            mailbox.set_power_state(PowerDeviceId::SdCard, true),
            Ok(true)
        ) {
            return Err(Error::ControllerPowerOnFailed);
        }

        // Assert the radio's WL_ON enable line through the VideoCore
        // GPIO expander — the SoC has no direct GPIO to it. This is
        // best-effort: on a Pi 3 the boot firmware already powers the
        // chip up (rsta2's `circle` relies on exactly that and never
        // asserts WL_ON itself on Pi <= 4), and not every Pi 3 firmware
        // even answers the expander's "Set GPIO State" mailbox tag — it
        // was rejected on the 3B this was brought up on. A rejection is
        // therefore tolerated rather than fatal: the CMD5 enumeration
        // below is the real proof the radio is alive, so let *it* fail
        // (with a bus-level error) if the chip genuinely isn't powered,
        // instead of aborting here on a call that's often redundant.
        let _ = mailbox.set_expander_gpio(EXPANDER_WL_ON, true);
        timer.delay_ms(WL_ON_SETTLE_MS);

        route_gpio_to_emmc(gpio);

        let base_clock_hz = mailbox
            .clock_rate_hz(ClockId::Emmc)
            .map_err(|_| Error::ClockQueryFailed)?;

        // Reset the whole host circuit and wait for the reset bit to
        // self-clear.
        emmc.control0().reset();
        emmc.control1().modify(|_, w| w.srst_hc().set_bit());
        wait_for(timer, 100_000, || {
            !emmc.control1().read().srst_hc().bit_is_set()
        })?;

        // Enable the internal clock and a real data timeout before the
        // first clock-divisor write, matching `sd.rs`'s ordering.
        emmc.control1().modify(|_, w| unsafe {
            w.clk_intlen()
                .set_bit()
                .data_tounit()
                .bits(DATA_TIMEOUT_MAX)
        });
        timer.delay_ms(10);

        set_clock(&emmc, base_clock_hz, SETUP_CLOCK_HZ, timer)?;

        // Unmask every interrupt status bit so it's visible in
        // `INTERRUPT`; this driver polls that register directly rather
        // than routing to the CPU's interrupt controller.
        emmc.irpt_mask().write(|w| unsafe { w.bits(0xffff_ffff) });

        let mut sdio = Self {
            emmc,
            rca: 0,
            window_base: 0,
            sdio_core_base: 0,
        };

        sdio.enumerate(timer)?;
        sdio.enable_function1(timer)?;

        // With the chip enumerated, widen the bus to 4-bit and raise the
        // clock from the identification rate to default speed. Order
        // matters: both sides must agree on the width before the clock
        // goes up, so switch the width first, then re-clock.
        sdio.set_bus_width_4bit(timer)?;
        set_clock(&sdio.emmc, base_clock_hz, TRANSFER_CLOCK_HZ, timer)?;

        Ok(sdio)
    }

    /// Runs the SDIO card-identification sequence, leaving the chip
    /// selected and in the transfer state: `CMD0` to idle, `CMD5` to
    /// negotiate the voltage window and wait for power-up, `CMD3` to
    /// learn the chip's self-assigned RCA, `CMD7` to select it.
    fn enumerate(&mut self, timer: &Timer) -> Result<(), Error> {
        self.command(CMD_GO_IDLE, 0, timer)?;

        // First CMD5 (argument 0) probes the chip's OCR; the following
        // ones offer the real voltage window and poll until the chip
        // reports power-up complete. The SDIO spec allows up to ~1s.
        self.command(CMD_IO_SEND_OP_COND, 0, timer)?;
        let start = timer.now_micros();
        loop {
            let response = self.command(CMD_IO_SEND_OP_COND, OCR_VOLTAGE_WINDOW, timer)?;
            if response & IO_OP_COND_READY != 0 {
                break;
            }
            if timer.now_micros() - start > 1_000_000 {
                return Err(Error::OpCondTimeout);
            }
            timer.delay_ms(10);
        }

        // On SDIO the chip publishes its own RCA in bits 31:16 of the
        // CMD3 response; keep it there, in argument position, for CMD7
        // and every later command.
        self.rca = self.command(CMD_SEND_REL_ADDR, 0, timer)? & 0xffff_0000;
        self.command(CMD_CARD_SELECT, self.rca, timer)?;
        Ok(())
    }

    /// Enables SDIO function 1 (the backplane register window) via the
    /// CCCR "I/O Enable" register and waits for the matching "I/O
    /// Ready" bit, so [`Self::backplane_read32`] can reach chip
    /// registers afterward.
    fn enable_function1(&self, timer: &Timer) -> Result<(), Error> {
        let function1_bit = 1u8 << FN1;
        self.cmd52_write(FN0, CCCR_IO_ENABLE, function1_bit, timer)?;

        let start = timer.now_micros();
        loop {
            let ready = self.cmd52_read(FN0, CCCR_IO_READY, timer)?;
            if ready & function1_bit != 0 {
                return Ok(());
            }
            if timer.now_micros() - start > 1_000_000 {
                return Err(Error::Function1NotReady);
            }
            timer.delay_ms(10);
        }
    }

    /// Switches both the chip and the controller to the 4-bit SDIO bus,
    /// up from the 1-bit bus enumeration runs on: writes the CCCR "Bus
    /// Interface Control" register's width field (telling the chip),
    /// then sets the controller's matching `CONTROL0.HCTL_DWIDTH`. The
    /// BCM43438 always supports the 4-bit bus, so — unlike `sd::Sd`'s
    /// best-effort negotiation for removable cards — a failure here is a
    /// real error, not a fall back to 1-bit.
    fn set_bus_width_4bit(&self, timer: &Timer) -> Result<(), Error> {
        let ctrl = self.cmd52_read(FN0, CCCR_BUS_IFACE, timer)?;
        let ctrl = (ctrl & !BUS_WIDTH_MASK) | BUS_WIDTH_4BIT;
        self.cmd52_write(FN0, CCCR_BUS_IFACE, ctrl, timer)?;
        self.emmc
            .control0()
            .modify(|_, w| w.hctl_dwidth().set_bit());
        Ok(())
    }

    /// Reads the ChipCommon core's chip-ID register over the backplane
    /// — the smallest end-to-end proof the SDIO bus, power, and
    /// function-1 backplane window all work. On a Pi 3 `B` this returns
    /// [`BCM43438_CHIP_ID`] (`0xA9A6`, i.e. 43430); an all-zero or
    /// all-ones result means the backplane window or function-1 enable
    /// didn't take.
    pub fn chip_id(&mut self, timer: &Timer) -> Result<u32, Error> {
        let id = self.backplane_read32(CHIPCOMMON_BASE, timer)?;
        Ok(id & CHIP_ID_MASK)
    }

    /// Walks the chip's enumeration ROM (EROM) to discover the backplane
    /// addresses of the cores the firmware download needs — the ARM CPU
    /// (Cortex-M3), the internal RAM (SOCRAM), and the SDIO core. The
    /// EROM is a flat table of 32-bit descriptors starting at the
    /// address in ChipCommon's `CHIPCOMMON_EROM_PTR`: a pair of
    /// component-id words introduces each core (carrying its core ID),
    /// and the address descriptors that follow give that core's register
    /// base (address type 0) and its reset/clock wrapper base (a
    /// wrapper/master address type).
    ///
    /// This is the simplified walk that suffices for the 43430, whose
    /// EROM uses only plain 32-bit, 4KB address descriptors — none of
    /// the 64-bit or explicitly-sized descriptors that would need extra
    /// trailing words skipped. Encountering a core ID that isn't one of
    /// the three needed is fine and ignored; not finding one of the
    /// three is [`Error::CoreNotFound`].
    pub fn scan_cores(&mut self, timer: &Timer) -> Result<ChipCores, Error> {
        // The EROM pointer register holds a backplane address; its low
        // bits aren't part of the address.
        let mut erom = self.backplane_read32(CHIPCOMMON_EROM_PTR, timer)? & 0xffff_f000;

        let mut arm_wrapper = 0;
        let mut d11_wrapper = 0;
        let mut socram_base = 0;
        let mut socram_wrapper = 0;
        let mut sdio_base = 0;

        // The core currently being described, and whether its register/
        // wrapper bases have been captured — only the first address
        // descriptor of each kind is the base of interest.
        let mut core_id = 0u16;
        let mut got_base = false;
        let mut got_wrapper = false;

        // The EROM is only a few dozen descriptors; cap the walk well
        // above that so a garbage read can't spin forever.
        for _ in 0..512 {
            let desc = self.backplane_read32(erom, timer)?;
            erom += 4;
            match desc & 0xf {
                EROM_DESC_EOT => break,
                EROM_DESC_COMPONENT => {
                    // `desc` is the component-id A word, carrying the core
                    // ID (part number) in bits 8:19. The following word is
                    // the B half (port/wrapper counts, revision); consume
                    // it to advance past it, then start a fresh core.
                    core_id = ((desc >> 8) & 0xfff) as u16;
                    let _cib = self.backplane_read32(erom, timer)?;
                    erom += 4;
                    got_base = false;
                    got_wrapper = false;
                }
                EROM_DESC_ADDRESS => {
                    let addr = desc & 0xffff_f000;
                    // Address type: 0 = the core's register (slave) base;
                    // 0x80/0xC0 = a slave/master wrapper (its control
                    // registers); 0x40 = a bridge, not wanted here.
                    let addr_type = desc & EROM_ADDR_TYPE_MASK;
                    if addr_type == 0 && !got_base {
                        got_base = true;
                        match core_id {
                            CORE_SOCRAM => socram_base = addr,
                            CORE_SDIO => sdio_base = addr,
                            _ => {}
                        }
                    } else if (addr_type == 0x80 || addr_type == 0xc0) && !got_wrapper {
                        got_wrapper = true;
                        match core_id {
                            CORE_ARM_CM3 => arm_wrapper = addr,
                            CORE_D11 => d11_wrapper = addr,
                            CORE_SOCRAM => socram_wrapper = addr,
                            _ => {}
                        }
                    }

                    // Step over any trailing words this descriptor
                    // carries, so the walk stays aligned: a 64-bit base
                    // (bit 3) adds a high-address word, and a "sized"
                    // region (size field, bits 4:5, == 3) adds a size
                    // word — itself 64-bit if its own bit 3 is set. The
                    // 43430's internal RAM shows up as one such sized
                    // region, so this is load-bearing, not hypothetical.
                    if desc & 0x8 != 0 {
                        self.backplane_read32(erom, timer)?;
                        erom += 4;
                    }
                    if desc & 0x30 == 0x30 {
                        let size_desc = self.backplane_read32(erom, timer)?;
                        erom += 4;
                        if size_desc & 0x8 != 0 {
                            self.backplane_read32(erom, timer)?;
                            erom += 4;
                        }
                    }
                }
                _ => {}
            }
        }

        if arm_wrapper == 0
            || d11_wrapper == 0
            || socram_base == 0
            || socram_wrapper == 0
            || sdio_base == 0
        {
            return Err(Error::CoreNotFound);
        }
        Ok(ChipCores {
            arm_wrapper,
            d11_wrapper,
            socram_base,
            socram_wrapper,
            sdio_base,
        })
    }

    /// Brings the chip to the download-ready ("passive") state and
    /// returns its discovered core layout: scans the cores, forces the
    /// ALP clock on so the backplane RAM is reachable, disables the ARM
    /// CPU, holds the D11 MAC in reset, and brings the SOCRAM core up so
    /// RAM is plain, writable memory. The firmware download builds on
    /// this; it's exposed on its own so RAM access can be exercised
    /// (written and read back) before the full download exists.
    pub fn prepare_download(&mut self, timer: &Timer) -> Result<ChipCores, Error> {
        let cores = self.scan_cores(timer)?;
        self.force_alp_clock(timer)?;
        self.core_disable(cores.arm_wrapper, 0, 0, timer)?;
        self.core_reset(
            cores.d11_wrapper,
            D11_PHY_RESET | D11_PHY_CLOCK_EN,
            D11_PHY_CLOCK_EN,
            D11_PHY_CLOCK_EN,
            timer,
        )?;
        self.core_reset(cores.socram_wrapper, 0, 0, 0, timer)?;
        // 43430-specific: disable the bank-3 remap so all of RAM is
        // plain, writable memory during download.
        self.backplane_write32(cores.socram_base + SOCRAM_BANKIDX, SOCRAM_REMAP_BANK, timer)?;
        self.backplane_write32(cores.socram_base + SOCRAM_BANKPDA, 0, timer)?;
        Ok(cores)
    }

    /// Measures the chip's internal RAM size, in bytes, by summing its
    /// SOCRAM banks — the bank count comes from `coreinfo`, each bank's
    /// size from `bankinfo`. Needed to place the nvram block at the top
    /// of RAM. Requires the SOCRAM core to be up (see
    /// [`Self::prepare_download`]).
    pub fn ram_size(&mut self, cores: &ChipCores, timer: &Timer) -> Result<u32, Error> {
        let coreinfo = self.backplane_read32(cores.socram_base + SOCRAM_COREINFO, timer)?;
        let num_banks = (coreinfo & SOCRAM_COREINFO_BANKS_MASK) >> 4;
        let mut total = 0;
        for bank in 0..num_banks {
            self.backplane_write32(cores.socram_base + SOCRAM_BANKIDX, bank, timer)?;
            let bankinfo = self.backplane_read32(cores.socram_base + SOCRAM_BANKINFO, timer)?;
            total += ((bankinfo & SOCRAM_BANKINFO_SIZE_MASK) + 1) * SOCRAM_BANK_SIZE_UNIT;
        }
        Ok(total)
    }

    /// Downloads the chip's firmware image into RAM, writes its nvram,
    /// releases the on-chip CPU, and confirms the firmware came alive.
    ///
    /// `firmware` is the raw `.bin` (its first word is the CPU's reset
    /// vector, which lands at RAM base 0 and needs nothing written
    /// separately); `nvram` is the raw `.txt` config, which this strips
    /// and formats before placing at the top of RAM. Where the bytes
    /// come from — embedded via `include_bytes!` or read off the SD card
    /// — is the caller's choice.
    ///
    /// Runs [`Self::prepare_download`] itself, so call it on a freshly
    /// [`init`](Self::init)-ed link. On success the chip's firmware is
    /// running and function 2 (the WLAN data path) is ready.
    pub fn load_firmware(
        &mut self,
        firmware: &[u8],
        nvram: &[u8],
        timer: &Timer,
    ) -> Result<(), Error> {
        let cores = self.prepare_download(timer)?;
        // Keep the SDIO core's backplane base for the protocol layer's
        // mailbox/interrupt access after firmware is up.
        self.sdio_core_base = cores.sdio_base;
        let ram_size = self.ram_size(&cores, timer)?;

        // Firmware image to RAM base 0 (its first word is the reset
        // vector); condensed nvram + size token at the top of RAM.
        self.download_to_ram(0, firmware, timer)?;
        self.write_nvram(ram_size, nvram, timer)?;

        // Release the CPU: SOCRAM must still be up (else the image isn't
        // reachable), then take the ARM out of reset. The CM3's reset
        // vector is simply word 0 of the image already at address 0, so
        // nothing extra is written (only CR4-class chips need a reset
        // vector poked in).
        if !self.core_is_up(cores.socram_wrapper, timer)? {
            return Err(Error::SocramNotUp);
        }
        self.core_reset(cores.arm_wrapper, 0, 0, 0, timer)?;

        // Hand the now-running firmware the SDPCM channel version and
        // wait for the WLAN data function to come ready — the sign it
        // booted.
        self.force_ht_clock(timer)?;
        self.backplane_write32(
            cores.sdio_base + SDIO_CORE_SB_MBOX_DATA,
            SDPCM_PROT_VERSION << 16,
            timer,
        )?;
        self.wait_function2_ready(timer)
    }

    /// Streams `bytes` into chip RAM starting at backplane address
    /// `base`, in [`FIRMWARE_CHUNK_BYTES`] `CMD53` chunks. Each chunk's
    /// bytes are assembled into little-endian words; a final partial
    /// word is zero-padded (harmless trailing bytes in RAM). `base` must
    /// be chunk-aligned (0 for the image) or the region must sit within
    /// one 32KB backplane window (the nvram, near the top of RAM), so no
    /// single chunk straddles a window boundary.
    fn download_to_ram(&mut self, base: u32, bytes: &[u8], timer: &Timer) -> Result<(), Error> {
        let mut addr = base;
        for chunk in bytes.chunks(FIRMWARE_CHUNK_BYTES) {
            let mut words = [0u32; FIRMWARE_CHUNK_BYTES / 4];
            let word_count = chunk.len().div_ceil(4);
            for (i, word) in words[..word_count].iter_mut().enumerate() {
                let start = i * 4;
                let end = (start + 4).min(chunk.len());
                let mut b = [0u8; 4];
                b[..end - start].copy_from_slice(&chunk[start..end]);
                *word = u32::from_le_bytes(b);
            }
            self.backplane_write(addr, &words[..word_count], timer)?;
            addr += chunk.len() as u32;
        }
        Ok(())
    }

    /// Condenses the raw nvram text (see [`condense_nvram`]) and writes
    /// it to the top of RAM, followed by the 4-byte size token the
    /// firmware reads to locate it: the vars sit just below the token at
    /// the very top of RAM, and the token holds the vars' word count in
    /// its low half and that count's complement in its high half.
    fn write_nvram(&mut self, ram_size: u32, nvram: &[u8], timer: &Timer) -> Result<(), Error> {
        let mut buf = [0u8; NVRAM_MAX_BYTES];
        let len = condense_nvram(nvram, &mut buf);

        let vars_addr = ram_size - len as u32 - 4;
        self.download_to_ram(vars_addr, &buf[..len], timer)?;

        let word_count = (len / 4) as u32;
        let token = (!word_count << 16) | (word_count & 0xffff);
        self.backplane_write32(ram_size - 4, token, timer)?;
        Ok(())
    }

    /// Whether the core at wrapper base `wrap` is up: out of reset
    /// (`resetctrl` clear) and clocked (`ioctrl`'s clock bit set,
    /// without the forced gated clock).
    fn core_is_up(&mut self, wrap: u32, timer: &Timer) -> Result<bool, Error> {
        let ioctrl = self.backplane_read32(wrap + WRAP_IOCTRL, timer)?;
        let resetctrl = self.backplane_read32(wrap + WRAP_RESETCTRL, timer)?;
        Ok(resetctrl & WRAP_RESET == 0 && ioctrl & (WRAP_FGC | WRAP_CLK) == WRAP_CLK)
    }

    /// Requests and forces on the chip's HT clock via `CHIPCLKCSR` —
    /// needed once the firmware is running, before the WLAN data
    /// function will come ready.
    fn force_ht_clock(&self, timer: &Timer) -> Result<(), Error> {
        self.cmd52_write(FN1, FUNC1_CHIPCLKCSR, CLKCSR_HT_AVAIL_REQ, timer)?;
        let start = timer.now_micros();
        loop {
            let csr = self.cmd52_read(FN1, FUNC1_CHIPCLKCSR, timer)?;
            if csr & CLKCSR_HT_AVAIL != 0 {
                break;
            }
            if timer.now_micros() - start > 5_000_000 {
                return Err(Error::Timeout);
            }
            timer.delay_ms(1);
        }
        self.cmd52_write(
            FN1,
            FUNC1_CHIPCLKCSR,
            CLKCSR_HT_AVAIL_REQ | CLKCSR_FORCE_HT,
            timer,
        )?;
        Ok(())
    }

    /// Enables the WLAN data function (F2) and waits for it to report
    /// ready — the sign the downloaded firmware booted and is servicing
    /// the SDIO interface. F2 can take a few seconds to come up while the
    /// firmware initializes, so this budgets generously.
    fn wait_function2_ready(&self, timer: &Timer) -> Result<(), Error> {
        let function2_bit = 1u8 << FN2;
        let enable = self.cmd52_read(FN0, CCCR_IO_ENABLE, timer)? | function2_bit;
        self.cmd52_write(FN0, CCCR_IO_ENABLE, enable, timer)?;

        let start = timer.now_micros();
        loop {
            let ready = self.cmd52_read(FN0, CCCR_IO_READY, timer)?;
            if ready & function2_bit != 0 {
                return Ok(());
            }
            if timer.now_micros() - start > 4_000_000 {
                return Err(Error::FirmwareNotReady);
            }
            timer.delay_ms(10);
        }
    }

    /// Requests and forces on the chip's ALP clock via the function-1
    /// `CHIPCLKCSR` register — the prerequisite for reaching the chip's
    /// internal cores and RAM over the backplane. Asks for ALP, waits
    /// for it to report available, then forces it on and clears the
    /// SDIO pull-ups, matching `brcmfmac`'s bus-core-prep sequence.
    fn force_alp_clock(&self, timer: &Timer) -> Result<(), Error> {
        self.cmd52_write(
            FN1,
            FUNC1_CHIPCLKCSR,
            CLKCSR_FORCE_HW_CLKREQ_OFF | CLKCSR_ALP_AVAIL_REQ,
            timer,
        )?;
        let start = timer.now_micros();
        loop {
            let csr = self.cmd52_read(FN1, FUNC1_CHIPCLKCSR, timer)?;
            if csr & CLKCSR_ALP_AVAIL != 0 {
                break;
            }
            if timer.now_micros() - start > 1_000_000 {
                return Err(Error::Timeout);
            }
            timer.delay_ms(1);
        }
        self.cmd52_write(
            FN1,
            FUNC1_CHIPCLKCSR,
            CLKCSR_FORCE_HW_CLKREQ_OFF | CLKCSR_FORCE_ALP,
            timer,
        )?;
        timer.delay_us(65);
        self.cmd52_write(FN1, FUNC1_SDIOPULLUP, 0, timer)?;
        Ok(())
    }

    /// Writes a 32-bit register at backplane address `addr` — the write
    /// counterpart to [`Self::backplane_read32`], used for the AXI core
    /// wrapper and SOCRAM registers.
    ///
    /// A 32-bit backplane *write* is a single 4-byte `CMD53` transfer
    /// (the way `brcmfmac`'s `sdio_writel` does it), not four `CMD52`
    /// byte writes. Byte-wise writes carrying the 32-bit-access flag do
    /// *not* assemble into one backplane cycle: they leave wrapper
    /// registers (e.g. a core's clock-enable `ioctrl`) and RAM
    /// unwritten. Reads survive the byte-wise form only because a
    /// 32-bit-access read latches the whole word on the first byte and
    /// caches the rest — writes have no such accumulation.
    pub fn backplane_write32(&mut self, addr: u32, value: u32, timer: &Timer) -> Result<(), Error> {
        self.backplane_write(addr, &[value], timer)
    }

    /// Writes `buf` starting at backplane address `addr` — the bulk write
    /// path, the write counterpart to [`Self::backplane_read`] that
    /// streams the firmware image into chip RAM. Any length is accepted:
    /// it's split into `MAX_TRANSFER_WORDS` (64-byte) `CMD53` transfers
    /// (see the read for why that size and window boundaries).
    pub fn backplane_write(&mut self, addr: u32, buf: &[u32], timer: &Timer) -> Result<(), Error> {
        let mut addr = addr;
        for chunk in buf.chunks(MAX_TRANSFER_WORDS) {
            self.write_block(addr, chunk, timer)?;
            addr += (chunk.len() * 4) as u32;
        }
        Ok(())
    }

    /// Writes one `CMD53` transfer of at most `MAX_TRANSFER_WORDS`
    /// words, retrying a few times on a transient data error (a CRC
    /// mismatch or data timeout): over a whole firmware download these
    /// turn up occasionally, and the SDIO protocol expects the host to
    /// reset the data line and re-send rather than give up. A
    /// `DataStalled` (controller wedged, not a data error) is not
    /// retried.
    fn write_block(&mut self, addr: u32, buf: &[u32], timer: &Timer) -> Result<(), Error> {
        /// Attempts, including the first, before giving up.
        const ATTEMPTS: u32 = 4;
        for attempt in 0..ATTEMPTS {
            match self.backplane_write_attempt(addr, buf, timer) {
                Ok(()) => return Ok(()),
                Err(Error::CardError { .. }) if attempt + 1 < ATTEMPTS => {
                    self.reset_data_circuit(timer);
                }
                Err(e) => return Err(e),
            }
        }
        // Unreachable: the loop returns on the last attempt either way.
        Err(Error::Timeout)
    }

    /// One [`Self::write_block`] attempt (see there for the retry
    /// wrapper).
    fn backplane_write_attempt(
        &mut self,
        addr: u32,
        buf: &[u32],
        timer: &Timer,
    ) -> Result<(), Error> {
        debug_assert!(buf.len() <= MAX_TRANSFER_WORDS);
        self.ensure_window(addr, timer)?;

        let byte_count = (buf.len() * 4) as u32;
        self.wait_data_ready(timer)?;
        self.emmc
            .blksizecnt()
            .write(|w| unsafe { w.blksize().bits(byte_count as u16).blkcnt().bits(1) });

        // Issue the write command inline rather than via `command`: its
        // clear-all-on-CMD_DONE would clobber a WRITE_RDY that co-fires
        // with CMD_DONE (which happens on writes but not reads), leaving
        // the following WRITE_RDY wait to hang. Here each phase clears
        // only its own interrupt bit, via `wait_data_int`.
        wait_for(timer, 100_000, || {
            self.emmc.status().read().cmd_inhibit().bit_is_clear()
        })?;
        let stale = self.emmc.interrupt().read().bits();
        self.emmc.interrupt().write(|w| unsafe { w.bits(stale) });

        let offset = (addr & SB_WINDOW_OFFSET_MASK) | SB_ACCESS_32BIT;
        let arg = cmd53_arg(true, FN1, false, true, offset, byte_count & 0x1ff);
        self.emmc.arg1().write(|w| unsafe { w.bits(arg) });
        self.emmc
            .cmdtm()
            .write(|w| unsafe { w.bits(CMD_IO_RW_EXTENDED_WRITE) });

        self.wait_data_int(INT_CMD_DONE, 0, timer)?;
        self.wait_data_int(INT_WRITE_RDY, 1, timer)?;
        for &word in buf {
            self.emmc.data().write(|w| unsafe { w.bits(word) });
        }
        self.wait_data_int(INT_DATA_DONE, 2, timer)?;
        Ok(())
    }

    /// Waits for interrupt `mask` (or any error bit) in `INTERRUPT`,
    /// clearing *only* the matched bit so a co-pending later-stage bit
    /// isn't lost (see [`Self::backplane_write`]). On a plain timeout it
    /// returns [`Error::DataStalled`] with the controller registers and
    /// `stage` for diagnosis, rather than a bare `Timeout`.
    fn wait_data_int(&self, mask: u32, stage: u8, timer: &Timer) -> Result<(), Error> {
        let start = timer.now_micros();
        loop {
            let interrupt = self.emmc.interrupt().read().bits();
            if interrupt & INT_ERROR_MASK != 0 {
                self.emmc
                    .interrupt()
                    .write(|w| unsafe { w.bits(interrupt & INT_ERROR_MASK) });
                return Err(Error::CardError {
                    interrupt,
                    command: stage as u32,
                });
            }
            if interrupt & mask != 0 {
                self.emmc.interrupt().write(|w| unsafe { w.bits(mask) });
                return Ok(());
            }
            if timer.now_micros() - start > 1_000_000 {
                let status = self.emmc.status().read().bits();
                return Err(Error::DataStalled {
                    stage,
                    interrupt,
                    status,
                });
            }
        }
    }

    /// Resets the controller's data circuit (`CONTROL1.SRST_DATA`) and
    /// clears any stale interrupt bits, recovering it after a data error
    /// so [`Self::backplane_write`] can re-issue the transfer.
    fn reset_data_circuit(&self, timer: &Timer) {
        self.emmc.control1().modify(|_, w| w.srst_data().set_bit());
        let _ = wait_for(timer, 100_000, || {
            !self.emmc.control1().read().srst_data().bit_is_set()
        });
        let stale = self.emmc.interrupt().read().bits();
        self.emmc.interrupt().write(|w| unsafe { w.bits(stale) });
    }

    /// Backplane base address of the SDIO device core, discovered while
    /// loading firmware. Its mailbox/interrupt registers (reached with
    /// [`Self::backplane_read32`]/[`Self::backplane_write32`] at this
    /// base plus an offset) are how the protocol layer sees frame-ready
    /// interrupts and the firmware-ready handshake. Zero before
    /// [`Self::load_firmware`].
    pub fn sdio_core_base(&self) -> u32 {
        self.sdio_core_base
    }

    /// Writes a whole frame to SDIO function 2 (the WLAN data path). The
    /// SDPCM/CDC protocol layer frames a message and hands it here; the
    /// firmware reassembles it from the function-2 FIFO. `bytes` must be
    /// a multiple of 4 (the caller pads the frame). Split into 64-byte
    /// `CMD53` transfers for the controller's PIO buffer (see
    /// `MAX_TRANSFER_WORDS`); all go to the fixed, non-incrementing
    /// function-2 address (offset 0), which is the single FIFO window.
    pub fn f2_write(&mut self, bytes: &[u8], timer: &Timer) -> Result<(), Error> {
        debug_assert!(bytes.len().is_multiple_of(4));
        for chunk in bytes.chunks(MAX_TRANSFER_WORDS * 4) {
            let mut words = [0u32; MAX_TRANSFER_WORDS];
            let word_count = chunk.len() / 4;
            for (i, word) in words[..word_count].iter_mut().enumerate() {
                *word = u32::from_le_bytes([
                    chunk[i * 4],
                    chunk[i * 4 + 1],
                    chunk[i * 4 + 2],
                    chunk[i * 4 + 3],
                ]);
            }
            self.f2_write_chunk(&words[..word_count], timer)?;
        }
        Ok(())
    }

    /// Reads `bytes` (a multiple of 4) from SDIO function 2 — a frame, or
    /// part of one. The protocol layer reads the 12-byte SDPCM header
    /// first (to learn the frame length), then the remainder. Split into
    /// 64-byte `CMD53` transfers from the fixed function-2 address, the
    /// read counterpart to [`Self::f2_write`].
    pub fn f2_read(&mut self, bytes: &mut [u8], timer: &Timer) -> Result<(), Error> {
        debug_assert!(bytes.len().is_multiple_of(4));
        for chunk in bytes.chunks_mut(MAX_TRANSFER_WORDS * 4) {
            let mut words = [0u32; MAX_TRANSFER_WORDS];
            let word_count = chunk.len() / 4;
            self.f2_read_chunk(&mut words[..word_count], timer)?;
            for (i, word) in words[..word_count].iter().enumerate() {
                chunk[i * 4..i * 4 + 4].copy_from_slice(&word.to_le_bytes());
            }
        }
        Ok(())
    }

    /// Writes one ≤`MAX_TRANSFER_WORDS`-word `CMD53` transfer to
    /// function 2 (fixed, non-incrementing address 0, byte mode). Same
    /// FIFO-write sequence as [`Self::backplane_write_attempt`] but on a
    /// different function with no backplane window.
    fn f2_write_chunk(&mut self, words: &[u32], timer: &Timer) -> Result<(), Error> {
        let byte_count = (words.len() * 4) as u32;
        self.wait_data_ready(timer)?;
        self.emmc
            .blksizecnt()
            .write(|w| unsafe { w.blksize().bits(byte_count as u16).blkcnt().bits(1) });

        wait_for(timer, 100_000, || {
            self.emmc.status().read().cmd_inhibit().bit_is_clear()
        })?;
        let stale = self.emmc.interrupt().read().bits();
        self.emmc.interrupt().write(|w| unsafe { w.bits(stale) });

        let arg = cmd53_arg(true, FN2, false, false, 0, byte_count & 0x1ff);
        self.emmc.arg1().write(|w| unsafe { w.bits(arg) });
        self.emmc
            .cmdtm()
            .write(|w| unsafe { w.bits(CMD_IO_RW_EXTENDED_WRITE) });

        self.wait_data_int(INT_CMD_DONE, 0, timer)?;
        self.wait_data_int(INT_WRITE_RDY, 1, timer)?;
        for &word in words {
            self.emmc.data().write(|w| unsafe { w.bits(word) });
        }
        self.wait_data_int(INT_DATA_DONE, 2, timer)?;
        Ok(())
    }

    /// Reads one ≤`MAX_TRANSFER_WORDS`-word `CMD53` transfer from
    /// function 2 (fixed address 0, byte mode). Issues the command
    /// inline (rather than via `command`) and steps the phases with the
    /// capturing [`Self::wait_data_int`], so a stall reports which wait
    /// and the controller registers (`DataStalled { stage: 0 }` = the
    /// command didn't complete, `stage: 1` = read-ready never fired).
    fn f2_read_chunk(&mut self, out: &mut [u32], timer: &Timer) -> Result<(), Error> {
        let byte_count = (out.len() * 4) as u32;
        self.wait_data_ready(timer)?;
        self.emmc
            .blksizecnt()
            .write(|w| unsafe { w.blksize().bits(byte_count as u16).blkcnt().bits(1) });

        wait_for(timer, 100_000, || {
            self.emmc.status().read().cmd_inhibit().bit_is_clear()
        })?;
        let stale = self.emmc.interrupt().read().bits();
        self.emmc.interrupt().write(|w| unsafe { w.bits(stale) });

        let arg = cmd53_arg(false, FN2, false, false, 0, byte_count & 0x1ff);
        self.emmc.arg1().write(|w| unsafe { w.bits(arg) });
        self.emmc
            .cmdtm()
            .write(|w| unsafe { w.bits(CMD_IO_RW_EXTENDED_READ) });

        self.wait_data_int(INT_CMD_DONE, 0, timer)?;
        self.wait_data_int(INT_READ_RDY, 1, timer)?;

        for word in out {
            *word = self.emmc.data().read().bits();
        }
        Ok(())
    }

    /// Puts the core at wrapper base `wrap` into reset and holds it
    /// there, leaving `ioctrl` = `reset_ioctrl` (plus the forced gated
    /// clock). `pre_ioctrl` is applied while first asserting reset. The
    /// AXI-backplane reset primitive: a core is controlled through just
    /// its wrapper's `ioctrl`/`resetctrl` registers.
    fn core_disable(
        &mut self,
        wrap: u32,
        pre_ioctrl: u32,
        reset_ioctrl: u32,
        timer: &Timer,
    ) -> Result<(), Error> {
        // Already in reset: just set the requested ioctrl and return.
        if self.backplane_read32(wrap + WRAP_RESETCTRL, timer)? & WRAP_RESET != 0 {
            self.backplane_write32(
                wrap + WRAP_IOCTRL,
                reset_ioctrl | WRAP_FGC | WRAP_CLK,
                timer,
            )?;
            self.backplane_read32(wrap + WRAP_IOCTRL, timer)?;
            return Ok(());
        }
        self.backplane_write32(wrap + WRAP_IOCTRL, pre_ioctrl | WRAP_FGC | WRAP_CLK, timer)?;
        self.backplane_read32(wrap + WRAP_IOCTRL, timer)?;
        self.backplane_write32(wrap + WRAP_RESETCTRL, WRAP_RESET, timer)?;
        timer.delay_us(10);
        let start = timer.now_micros();
        while self.backplane_read32(wrap + WRAP_RESETCTRL, timer)? & WRAP_RESET == 0 {
            if timer.now_micros() - start > 100_000 {
                return Err(Error::Timeout);
            }
        }
        self.backplane_write32(
            wrap + WRAP_IOCTRL,
            reset_ioctrl | WRAP_FGC | WRAP_CLK,
            timer,
        )?;
        self.backplane_read32(wrap + WRAP_IOCTRL, timer)?;
        Ok(())
    }

    /// Resets the core at wrapper base `wrap` and brings it back up out
    /// of reset, leaving `ioctrl` = `post_ioctrl` (plus the clock).
    /// `pre_ioctrl`/`reset_ioctrl` are passed through to the initial
    /// [`Self::core_disable`]. Used to bring SOCRAM up (making RAM
    /// writable) and, at the end, to release the ARM CPU.
    fn core_reset(
        &mut self,
        wrap: u32,
        pre_ioctrl: u32,
        reset_ioctrl: u32,
        post_ioctrl: u32,
        timer: &Timer,
    ) -> Result<(), Error> {
        self.core_disable(wrap, pre_ioctrl, reset_ioctrl, timer)?;
        // Clear resetctrl until the core actually leaves reset.
        let start = timer.now_micros();
        loop {
            self.backplane_write32(wrap + WRAP_RESETCTRL, 0, timer)?;
            if self.backplane_read32(wrap + WRAP_RESETCTRL, timer)? & WRAP_RESET == 0 {
                break;
            }
            if timer.now_micros() - start > 100_000 {
                return Err(Error::Timeout);
            }
            timer.delay_us(50);
        }
        self.backplane_write32(wrap + WRAP_IOCTRL, post_ioctrl | WRAP_CLK, timer)?;
        self.backplane_read32(wrap + WRAP_IOCTRL, timer)?;
        Ok(())
    }

    /// Reads a 32-bit register at backplane address `addr`: points the
    /// function-1 address window at it (reprogramming the `SBADDR*`
    /// registers only when `addr` falls outside the current window) and
    /// reads the four bytes with `CMD52` accesses carrying the 32-bit-
    /// access flag, assembling them little-endian.
    pub fn backplane_read32(&mut self, addr: u32, timer: &Timer) -> Result<u32, Error> {
        self.ensure_window(addr, timer)?;

        // The in-window offset, OR'd with the flag that makes each
        // access a 32-bit backplane cycle. CMD52 still transfers one
        // byte at a time, so read all four and combine them.
        let offset = (addr & SB_WINDOW_OFFSET_MASK) | SB_ACCESS_32BIT;
        let mut value = 0u32;
        for i in 0..4 {
            let byte = self.cmd52_read(FN1, offset + i, timer)?;
            value |= (byte as u32) << (8 * i);
        }
        Ok(value)
    }

    /// Reads `buf` (up to 128 words / 512 bytes) starting at backplane
    /// address `addr` in a single `CMD53` byte-mode transfer over
    /// function 1 — the bulk data path, streaming words through the
    /// controller's `DATA` FIFO instead of one byte per command like
    /// [`Self::backplane_read32`]. This is the read half of the channel
    /// the firmware download will push the chip's image over.
    ///
    /// Any length is accepted: `buf` is split into `MAX_TRANSFER_WORDS`
    /// (64-byte) `CMD53` transfers, matching the controller's data buffer
    /// and the chip's function-1 block size, and — since that size
    /// divides the 32KB backplane window — no single transfer straddles a
    /// window boundary.
    ///
    /// Each transfer's in-window offset carries `SB_ACCESS_32BIT`,
    /// forcing 4-byte backplane accesses, the same as the single-register
    /// path and as `brcmfmac`'s RAM read/write. It's load-bearing for RAM
    /// (SOCRAM): byte-wide backplane access silently drops writes and
    /// stalls reads there. Register cores like ChipCommon happen to
    /// tolerate byte access, so a bulk read of *those* worked without it —
    /// but RAM does not, so the flag is always set.
    pub fn backplane_read(
        &mut self,
        addr: u32,
        buf: &mut [u32],
        timer: &Timer,
    ) -> Result<(), Error> {
        let mut addr = addr;
        for chunk in buf.chunks_mut(MAX_TRANSFER_WORDS) {
            self.read_block(addr, chunk, timer)?;
            addr += (chunk.len() * 4) as u32;
        }
        Ok(())
    }

    /// Reads one `CMD53` transfer of at most `MAX_TRANSFER_WORDS` words
    /// (see [`Self::backplane_read`], which splits into these).
    fn read_block(&mut self, addr: u32, buf: &mut [u32], timer: &Timer) -> Result<(), Error> {
        debug_assert!(buf.len() <= MAX_TRANSFER_WORDS);
        self.ensure_window(addr, timer)?;

        let byte_count = (buf.len() * 4) as u32;
        self.wait_data_ready(timer)?;
        self.emmc
            .blksizecnt()
            .write(|w| unsafe { w.blksize().bits(byte_count as u16).blkcnt().bits(1) });

        let offset = (addr & SB_WINDOW_OFFSET_MASK) | SB_ACCESS_32BIT;
        let arg = cmd53_arg(false, FN1, false, true, offset, byte_count & 0x1ff);
        self.command(CMD_IO_RW_EXTENDED_READ, arg, timer)?;
        self.wait_interrupt(INT_READ_RDY, CMD_IO_RW_EXTENDED_READ, timer)?;

        for word in buf {
            *word = self.emmc.data().read().bits();
        }
        Ok(())
    }

    /// Points the function-1 window at whichever 32KB block contains
    /// `addr`, reprogramming the `SBADDR*` registers only when the
    /// target moves outside the block currently selected.
    fn ensure_window(&mut self, addr: u32, timer: &Timer) -> Result<(), Error> {
        let base = addr & SB_WINDOW_BASE_MASK;
        if base != self.window_base || self.window_base == 0 {
            self.set_backplane_window(base, timer)?;
            self.window_base = base;
        }
        Ok(())
    }

    /// Waits for the data lines to be free — the prerequisite a
    /// data-bearing command ([`Self::backplane_read`]) needs before
    /// issuing.
    fn wait_data_ready(&self, timer: &Timer) -> Result<(), Error> {
        wait_for(timer, 100_000, || {
            self.emmc.status().read().dat_inhibit().bit_is_clear()
        })
    }

    /// Points the function-1 backplane address window at `base` (which
    /// must be 32KB-aligned) by writing the three `SBADDR*` registers
    /// with successive bytes of the base address.
    fn set_backplane_window(&self, base: u32, timer: &Timer) -> Result<(), Error> {
        self.cmd52_write(FN1, FUNC1_SBADDR_LOW, (base >> 8) as u8, timer)?;
        self.cmd52_write(FN1, FUNC1_SBADDR_MID, (base >> 16) as u8, timer)?;
        self.cmd52_write(FN1, FUNC1_SBADDR_HIGH, (base >> 24) as u8, timer)?;
        Ok(())
    }

    /// Reads one byte from SDIO register `addr` in function `func`
    /// (`CMD52` / `IO_RW_DIRECT`). The byte comes back in the low byte
    /// of the R5 response (`RESP0`).
    pub fn cmd52_read(&self, func: u32, addr: u32, timer: &Timer) -> Result<u8, Error> {
        let response = self.command(CMD_IO_RW_DIRECT, cmd52_arg(false, func, addr, 0), timer)?;
        Ok(response as u8)
    }

    /// Writes one byte to SDIO register `addr` in function `func`
    /// (`CMD52` / `IO_RW_DIRECT`).
    pub fn cmd52_write(&self, func: u32, addr: u32, data: u8, timer: &Timer) -> Result<(), Error> {
        self.command(CMD_IO_RW_DIRECT, cmd52_arg(true, func, addr, data), timer)?;
        Ok(())
    }

    /// Issues one command: waits for the command line to be free,
    /// clears stale interrupt status, writes the argument and command,
    /// waits for `CMD_DONE` (or an error), and returns `RESP0` — the
    /// 32-bit response argument field (response bits 39:8), which
    /// carries everything this driver reads: `CMD5`'s ready bit,
    /// `CMD3`'s RCA, and `CMD52`'s read-data byte.
    fn command(&self, code: u32, arg: u32, timer: &Timer) -> Result<u32, Error> {
        wait_for(timer, 100_000, || {
            self.emmc.status().read().cmd_inhibit().bit_is_clear()
        })?;

        // Clear any stale interrupt bits (all write-1-to-clear) before
        // issuing.
        let stale = self.emmc.interrupt().read().bits();
        self.emmc.interrupt().write(|w| unsafe { w.bits(stale) });

        self.emmc.arg1().write(|w| unsafe { w.bits(arg) });
        self.emmc.cmdtm().write(|w| unsafe { w.bits(code) });

        self.wait_interrupt(INT_CMD_DONE, code, timer)?;
        Ok(self.emmc.resp0().read().bits())
    }

    /// Waits for `mask` (or any [`INT_ERROR_MASK`] bit) in `INTERRUPT`,
    /// clears whatever is set, and returns the pre-clear value.
    /// `command` is the `CMDTM` code in flight, passed through to
    /// [`Error::CardError`] for diagnostics.
    fn wait_interrupt(&self, mask: u32, command: u32, timer: &Timer) -> Result<u32, Error> {
        let full_mask = mask | INT_ERROR_MASK;
        let start = timer.now_micros();
        let interrupt = loop {
            let interrupt = self.emmc.interrupt().read().bits();
            if interrupt & full_mask != 0 {
                break interrupt;
            }
            if timer.now_micros() - start > 1_000_000 {
                return Err(Error::Timeout);
            }
        };
        self.emmc
            .interrupt()
            .write(|w| unsafe { w.bits(interrupt) });
        if interrupt & INT_ERROR_MASK != 0 {
            return Err(Error::CardError { interrupt, command });
        }
        Ok(interrupt)
    }
}

/// Assembles a `CMD52` (`IO_RW_DIRECT`) argument: read (`write =
/// false`) or write of `data` to register `addr` in function `func`.
/// Bit 31 = R/W direction, 30:28 = function number, 25:9 = the 17-bit
/// register address, 7:0 = the write data (`0` on a read). The RAW
/// (read-after-write) flag at bit 27 stays clear — this driver never
/// needs a combined write-then-read.
fn cmd52_arg(write: bool, func: u32, addr: u32, data: u8) -> u32 {
    ((write as u32) << 31) | ((func & 0x7) << 28) | ((addr & 0x1_ffff) << 9) | (data as u32)
}

/// Condenses raw nvram `.txt` into the packed form the chip's firmware
/// expects, written into `out`, returning its length (a multiple of 4).
///
/// The transformation, matching `brcmfmac`'s `brcmf_fw_nvram_strip` and
/// plan9's `condense`: a `#` starts a comment discarded to end of line;
/// `\r` is dropped; each non-empty line's terminating newline becomes a
/// single `\0` (so entries read `var=value\0`); empty lines are dropped;
/// an extra `\0` terminates the whole block; and the result is padded
/// with `\0` to a 4-byte boundary. Writing stops if `out` fills up.
fn condense_nvram(raw: &[u8], out: &mut [u8]) -> usize {
    let mut n = 0;
    // The `out` index where the current line's output began — if the
    // line ends with nothing added past here, it was blank and produces
    // no entry.
    let mut line_start = 0;
    let mut in_comment = false;

    for &byte in raw {
        // Leave room for this byte plus the two trailing NULs and up to
        // three padding bytes the tail below may add.
        if n + 6 >= out.len() {
            break;
        }
        match byte {
            b'\r' => {}
            b'\n' => {
                in_comment = false;
                if n > line_start {
                    out[n] = 0;
                    n += 1;
                    line_start = n;
                }
            }
            b'#' => in_comment = true,
            _ if in_comment => {}
            _ => {
                out[n] = byte;
                n += 1;
            }
        }
    }

    // A final line with no trailing newline still terminates.
    if n > line_start {
        out[n] = 0;
        n += 1;
    }
    // Extra terminator marking the end of the block, then pad to 4.
    out[n] = 0;
    n += 1;
    while !n.is_multiple_of(4) {
        out[n] = 0;
        n += 1;
    }
    n
}

/// Assembles a `CMD53` (`IO_RW_EXTENDED`) argument. Bit 31 = R/W
/// direction, 30:28 = function, 27 = block mode (vs. byte), 26 = OP
/// code (`true` = address increments across the transfer, `false` =
/// fixed), 25:9 = the 17-bit start address, 8:0 = the count (blocks in
/// block mode, bytes in byte mode; `0` encodes the 512-byte maximum in
/// byte mode).
fn cmd53_arg(
    write: bool,
    func: u32,
    block_mode: bool,
    incrementing: bool,
    addr: u32,
    count: u32,
) -> u32 {
    ((write as u32) << 31)
        | ((func & 0x7) << 28)
        | ((block_mode as u32) << 27)
        | ((incrementing as u32) << 26)
        | ((addr & 0x1_ffff) << 9)
        | (count & 0x1ff)
}

/// Routes GPIO34-39 (`CLK`/`CMD`/`DAT0..DAT3`) to the Arasan controller
/// (ALT3), with `CMD`/`DAT0..DAT3` pulled up and `CLK` left with no
/// pull — the pin configuration the Linux device tree specifies for
/// this exact wireless-SDIO pin group. First disconnects the SD-card
/// slot pins from the same controller (see the body).
fn route_gpio_to_emmc(gpio: &GPIO) {
    // Disconnect the Arasan/EMMC controller from the SD-card slot
    // *before* wiring it to the wireless pins. The boot firmware reads
    // the card over EMMC on GPIO48-53 and leaves those pins at ALT3
    // (confirmed on this board); muxing GPIO34-39 to EMMC below without
    // this would leave the one controller driving both pin groups at
    // once — the wireless chip would then share CMD/CLK/DAT with the
    // (unselected) card slot and never answer, which shows up as a CMD5
    // timeout during enumeration. Pointing 48-53 at the *other* SD
    // controller (SDHOST, ALT0) takes them off EMMC so it drives only
    // the wireless pins, exactly as plan9's / circle's `ether4330.c`
    // `sdioinit` does on Pi <= 4. This is why bringing up Wi-Fi gives
    // up the SD-card slot (see the module docs).
    gpio.gpfsel4().modify(|_, w| {
        w.fsel48()
            .bits(SD_SLOT_ALT_FUNCTION)
            .fsel49()
            .bits(SD_SLOT_ALT_FUNCTION)
    });
    gpio.gpfsel5().modify(|_, w| {
        w.fsel50()
            .bits(SD_SLOT_ALT_FUNCTION)
            .fsel51()
            .bits(SD_SLOT_ALT_FUNCTION)
            .fsel52()
            .bits(SD_SLOT_ALT_FUNCTION)
            .fsel53()
            .bits(SD_SLOT_ALT_FUNCTION)
    });

    // GPIO34-39 all live in GPFSEL3 (pins 30-39).
    gpio.gpfsel3().modify(|_, w| {
        w.fsel34()
            .bits(GPIO_ALT_FUNCTION)
            .fsel35()
            .bits(GPIO_ALT_FUNCTION)
            .fsel36()
            .bits(GPIO_ALT_FUNCTION)
            .fsel37()
            .bits(GPIO_ALT_FUNCTION)
            .fsel38()
            .bits(GPIO_ALT_FUNCTION)
            .fsel39()
            .bits(GPIO_ALT_FUNCTION)
    });

    let pull_up_mask = (1 << (GPIO_CMD - 32))
        | GPIO_DAT
            .iter()
            .fold(0, |mask, pin| mask | (1 << (pin - 32)));
    let clk_mask = 1 << (GPIO_CLK - 32);
    set_pull(GPPUD_PULL_UP, pull_up_mask);
    set_pull(GPPUD_PULL_NONE, clk_mask);
}

/// Applies pull mode `pud` to the GPIO32-53 pins selected by `mask`,
/// via the legacy `GPPUD`/`GPPUDCLK1` clock-in sequence (BCM2835 ARM
/// Peripherals datasheet §6.1). Same dance as [`crate::sd`]'s pin
/// setup, but split into two calls here because `CLK` and the other
/// lines want different pulls.
fn set_pull(pud: u32, mask: u32) {
    unsafe {
        core::ptr::write_volatile(GPPUD, pud);
        spin_delay(150);
        core::ptr::write_volatile(GPPUDCLK1, mask);
        spin_delay(150);
        core::ptr::write_volatile(GPPUD, 0);
        core::ptr::write_volatile(GPPUDCLK1, 0);
    }
}

fn spin_delay(cycles: u32) {
    for _ in 0..cycles {
        unsafe { core::arch::asm!("nop") };
    }
}

/// Sets the SDIO clock to at or below `target_hz` from a `base_hz`
/// source, using the SDHCI 3.0 10-bit divisor. Same controller
/// sequence as [`crate::sd`]'s `set_clock` (and sharing its
/// [`clock_divider`]); the 10-bit form is load-bearing on this board
/// for the same reason — firmware reports a 200MHz base clock, and the
/// ≤400kHz identification clock needs a divisor past what 8 bits reach.
fn set_clock(emmc: &EMMC, base_hz: u32, target_hz: u32, timer: &Timer) -> Result<(), Error> {
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

/// Polls `condition` until true, up to `timeout_us` microseconds of
/// real elapsed time.
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
