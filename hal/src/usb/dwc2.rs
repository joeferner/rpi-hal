//! Host-mode bring-up for the BCM2836/2837's DWC2 USB OTG controller
//! — core reset, forcing host mode, powering and resetting the root
//! port, plus the low-level DMA-mode channel primitives (including
//! split transactions through a hub's transaction translator)
//! [`crate::usb::control`] builds actual control transfers on top of.
//! Nothing here understands USB protocol state (SETUP/DATA/STATUS
//! staging, retries) — see this module's parent for the split.
//!
//! ## The controller must be powered on first
//!
//! On this board the firmware hands the DWC2 core off only *partially*
//! powered: every global, host and per-channel register reads and
//! writes normally, the core enters host mode, the root port detects a
//! connection and enables, and the frame timer (`HFNUM`) runs — yet no
//! channel transaction ever executes. A channel arms perfectly (`CHENA`
//! set, `HCTSIZ`/`HCDMA`/`HCCHAR` all reading back exactly as
//! programmed) and then sits there forever: `CHENA` never clears, the
//! request queue never moves, and `HCINT` stays all-zero, no completion
//! and no error. Nothing programmed *through the DWC2 register block*
//! changes this, because the register block is not what's unpowered.
//! The fix lives outside this driver entirely — the caller must first
//! power the USB HCD on through the VideoCore mailbox
//! ([`crate::mailbox::Mailbox::set_power_state`] with
//! [`crate::mailbox::PowerDeviceId::UsbHcd`]) before calling
//! [`Dwc2Host::init`]; once fully
//! powered, the very same register sequence completes transfers
//! immediately.
//!
//! ## DMA, not slave mode
//!
//! An earlier version of this driver used slave (PIO/FIFO-push) mode
//! — program a channel, push bytes into its FIFO by hand, pop received
//! bytes back out. On real hardware that produced a channel that
//! looked fully armed (`HCCHAR`/`HCTSIZ` read back exactly as
//! programmed, FIFO/request-queue space was visibly consumed) but
//! never generated a single `HCINT` bit, not even an error — total
//! silence. `GHWCFG2.ARCHITECTURE` (this PAC calls it
//! `hw_config0().architecture()`) confirmed why: this SoC's DWC2
//! instantiation reports `internal_dma`, not `slave_only` — slave mode
//! isn't just unused here, it isn't implemented in this silicon at
//! all. Every transfer here goes through `HCDMA` instead.
//!
//! DMA means the buffer a channel reads from/writes to is touched by
//! the core directly over the bus, not through the ARM core's cache —
//! the same cache-coherency concern `mailbox.rs` already had to
//! handle, for the same reason (this crate's MMU maps RAM Cacheable).
//! `clean_range` runs before an OUT/SETUP transfer, `invalidate_range`
//! after an IN transfer, and the buffer's address is translated to the
//! VideoCore bus alias the same way (see `to_vc_bus_address`) before
//! it's ever written to `HCDMA`.

/// Interrupt-driven `async` twins of [`Channel`]'s transfer primitives —
/// see the module's own documentation for the interrupt wiring an
/// application must provide.
///
/// Available only with the `async` feature enabled.
#[cfg(feature = "async")]
#[cfg_attr(docsrs, doc(cfg(feature = "async")))]
pub mod asynch;

#[cfg(feature = "async")]
pub use asynch::on_irq;

use core::cell::Cell;

use crate::cache::{clean_range, invalidate_range, MIN_CACHE_LINE};
use crate::pac::{USB_OTG_GLOBAL, USB_OTG_HOST, USB_OTG_PWRCLK};
use crate::timer::Timer;

/// Translates a plain ARM physical address to the VideoCore's
/// `0xC000_0000`-based "direct, uncached" bus alias — see
/// `mailbox.rs`'s identical helper and doc comment. Every DMA-capable
/// peripheral on this SoC (the mailbox, the standalone DMA
/// controller, and this USB core alike) shares the same bus address
/// space, so the same translation applies here, not just there.
fn to_vc_bus_address(physical_address: u32) -> u32 {
    (physical_address & 0x3FFF_FFFF) | 0xC000_0000
}

/// Which data-toggle PID a packet on a host channel carries. The
/// numeric values are `HCTSIZ.DPID`'s encoding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DataPid {
    /// `2'b00` — DATA0. The first packet on a freshly configured
    /// interrupt/bulk endpoint, and every other packet as the toggle
    /// alternates.
    Data0,
    /// `2'b10` — DATA1. Every DATA/STATUS packet in a control transfer
    /// after the initial SETUP (control transfers always start their
    /// data stage at DATA1), and the alternate interrupt/bulk toggle.
    Data1,
    /// `2'b11` — every SETUP packet.
    Setup,
}

impl DataPid {
    fn bits(self) -> u8 {
        match self {
            DataPid::Data0 => 0b00,
            DataPid::Data1 => 0b10,
            DataPid::Setup => 0b11,
        }
    }

    /// The next data toggle after this one — `DATA0`↔`DATA1`. `Setup`
    /// has no toggle (a SETUP stage is always a single packet) and maps
    /// to itself.
    fn toggled(self) -> DataPid {
        match self {
            DataPid::Data0 => DataPid::Data1,
            DataPid::Data1 => DataPid::Data0,
            DataPid::Setup => DataPid::Setup,
        }
    }
}

/// Host-channel transfer type — `HCCHAR.EPTYP`'s encoding. Only the
/// types this driver issues so far.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EndpointType {
    /// Control endpoint (endpoint 0).
    Control,
    /// Interrupt endpoint (e.g. a hub's status-change endpoint or a HID
    /// report endpoint). Periodic on this core — for an *interrupt*
    /// endpoint a bulk channel type was tried instead and the
    /// transaction wouldn't run at all (see [`Channel::interrupt_in`]);
    /// a genuine bulk endpoint still uses [`Self::Bulk`] below.
    Interrupt,
    /// Bulk endpoint (e.g. the LAN9514 Ethernet controller's frame RX/TX
    /// endpoints). Non-periodic — scheduled on the non-periodic request
    /// queue like control, not the periodic one interrupt uses.
    Bulk,
}

impl EndpointType {
    fn bits(self) -> u8 {
        match self {
            EndpointType::Control => 0b00,
            EndpointType::Bulk => 0b10,
            EndpointType::Interrupt => 0b11,
        }
    }
}

/// Why a single-packet channel transfer didn't complete successfully.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransferError {
    /// The device returned STALL — it doesn't support/recognize the
    /// request.
    Stall,
    /// `HCINT.TXERR` — a transaction error (no response, bad CRC,
    /// etc.).
    TransactionError,
    /// `HCINT.BBERR` — babble (the device sent more than expected).
    Babble,
    /// The device NAK'd this attempt — normal USB flow control (“not
    /// ready yet, ask again”), not a real error. Left for the caller
    /// to retry; see [`crate::usb::control`]'s retry wrapper.
    Nak,
    /// A caller-level retry budget was exhausted while the device
    /// kept NAKing — distinct from a single [`Self::Nak`].
    NakTimeout,
    /// `HCINT.FRMOR` — frame overrun. A periodic (interrupt or
    /// isochronous) transaction was still in progress when the
    /// microframe it was scheduled in ran out, and the core abandoned
    /// it.
    ///
    /// This is about *when* the transaction was started, not about the
    /// device: a periodic transfer armed late in a microframe has too
    /// little of it left to finish. Retrying on the next interval is
    /// the normal response, which is why callers should treat this much
    /// as they treat [`Self::Nak`] rather than as a hard failure.
    FrameOverrun,
    /// `HCINT.DTERR` — the packet arrived carrying the opposite data
    /// toggle to the one expected, so it was discarded as a
    /// retransmission of one already received. Means host and device
    /// have lost toggle synchronisation.
    DataToggleError,
    /// The channel halted (`HCINT.CHH`) without `HCINT.XFRC` (transfer
    /// complete) and without any specific error/status bit set — an
    /// unexplained halt. Not expected in normal operation now that
    /// completion is detected via `CHH`; a defensive branch rather than
    /// a condition seen in practice.
    Halted,
    /// The channel never halted (`HCINT.CHH` never set) within this
    /// driver's timeout — the transaction appears stuck rather than
    /// having reached any terminal condition.
    Timeout,
}

/// Locates the hub transaction translator a full/low-speed device sits
/// behind, for split transactions. A full- or low-speed device plugged
/// into a high-speed hub can't talk to a high-speed host directly: the
/// host issues the transaction to the hub's transaction translator (a
/// "start split"), which relays it to the device at the slower speed,
/// then polls the translator for the result (a "complete split"). This
/// says which translator — identified by the hub's device address and
/// the downstream port the device is on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SplitTarget {
    /// Device address of the hub whose transaction translator relays to
    /// the device.
    pub hub_address: u8,
    /// The hub's downstream port number (1-based) the device is on.
    pub port: u8,
}

/// Which device (and at what speed/max packet size) a control
/// transfer's low-level primitives should talk to — bundled together
/// since every stage of a given transfer shares the same values;
/// passing them as separate parameters through every method pushed
/// `start_channel`'s argument count over clippy's limit for no real
/// benefit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlEndpoint {
    /// USB device address (0 before `SET_ADDRESS`).
    pub address: u8,
    /// Whether the device is low-speed — see [`Dwc2Host::port_speed`].
    pub low_speed: bool,
    /// Endpoint 0's max packet size for this transfer (`8` for the
    /// first, address-0 probe every real host issues before it knows
    /// the device's real value).
    pub max_packet_size: u16,
    /// `Some` if the device is a full/low-speed device behind a
    /// high-speed hub and its transfers must go through that hub's
    /// transaction translator as split transactions (see
    /// [`SplitTarget`]); `None` for a device that reaches the host
    /// directly (on the root port, or any high-speed device).
    pub split: Option<SplitTarget>,
}

/// The shape of one channel transaction, bundled so
/// [`Channel::start_channel`]/[`Channel::run_transaction`] don't
/// exceed clippy's argument-count limit.
#[derive(Clone, Copy)]
struct Transaction {
    /// Endpoint number (0 for control transfers).
    endpoint_number: u8,
    /// Endpoint transfer type.
    endpoint_type: EndpointType,
    /// Transfer direction: `true` = IN (device→host), `false` = OUT.
    direction_in: bool,
    /// Data toggle / PID for this transaction.
    pid: DataPid,
    /// Number of bytes to transfer.
    transfer_size: u32,
    /// Physical address of the DMA buffer (`start_channel` translates
    /// it to the VideoCore bus alias).
    dma_address: u32,
}

/// Largest control-transfer data payload this driver's shared DMA
/// scratch buffer can hold, in bytes. 256 covers a full device
/// descriptor (18 bytes), the configuration descriptors read during
/// enumeration of the devices this stack targets, and a HID *report*
/// descriptor — the one descriptor here that routinely runs past a
/// hundred bytes (a game controller's is typically 100-200; see
/// [`crate::usb::hid::gamepad`]), and which must be read in a single
/// transfer because GET_DESCRIPTOR has no way to request an offset into
/// one. A multiple of the 64-byte cache line, so [`DmaBuffer`] stays
/// line-aligned at both ends.
const DMA_BUFFER_LEN: usize = 256;

/// Largest data payload a single transfer through a channel's DMA
/// scratch buffer can carry, in bytes — every control-transfer stage
/// ([`Channel::control_data_in`], [`Channel::control_data_out`]) and an
/// interrupt-IN poll ([`Channel::interrupt_in`]). A caller sizing a
/// buffer for one of those (a descriptor read, a report endpoint's
/// packets) can check it against this instead of guessing. Bulk transfers
/// DMA straight from/to the caller's buffer and aren't bounded by it.
pub const MAX_TRANSFER_LEN: usize = DMA_BUFFER_LEN;

/// Cache-line aligned (64 bytes — a Cortex-A7 L1 D-cache line, so
/// [`invalidate_range`] after a DMA-IN can never discard a neighbouring
/// dirty line) scratch buffer every control-transfer stage's DMA reads
/// from / writes to. The DWC2 DMA engine walks it packet by packet for
/// multi-packet transfers, so it must be large enough for the whole
/// transfer at once — hence [`DMA_BUFFER_LEN`], which is also a whole
/// number of cache lines.
///
/// One per [`Channel`], not one per controller: two channels running at
/// once must not be staging their packets through the same memory.
#[repr(C, align(64))]
struct DmaBuffer([u8; DMA_BUFFER_LEN]);

/// Upper bound on host channels, from the PAC rather than the silicon:
/// `USB_OTG_HOST::host_channel(n)` indexes a fixed 12-entry array and
/// refuses to compile past it. The core reports its real count in
/// `GHWCFG2` (8 on this SoC), which is what [`Dwc2Host::alloc_channel`]
/// actually hands out — this only keeps a nonsense read from indexing
/// out of range.
const MAX_CHANNELS: usize = 12;

// Host channel interrupt (`HCINT`) bit masks — the terminal conditions
// a transaction can halt with. Used directly as masks rather than
// through the PAC's per-bit accessors so a single read can be tested
// for several conditions (the split logic cares about `ACK`/`NYET`,
// which the plain control path doesn't).
/// `HCINT.XFRC` — transfer completed successfully.
const HCINT_XFRC: u32 = 1 << 0;
/// `HCINT.CHH` — channel halted (the DMA-mode "transaction done" flag).
const HCINT_CHH: u32 = 1 << 1;
/// `HCINT.STALL` — the device returned STALL.
const HCINT_STALL: u32 = 1 << 3;
/// `HCINT.NAK` — the device returned NAK.
const HCINT_NAK: u32 = 1 << 4;
/// `HCINT.ACK` — an ACK handshake was received/transmitted (a hub's
/// transaction translator ACKs a start-split it has accepted).
const HCINT_ACK: u32 = 1 << 5;
/// `HCINT.NYET` — for a complete-split, the translator hasn't finished
/// the relayed downstream transaction yet; poll again.
const HCINT_NYET: u32 = 1 << 6;
/// `HCINT.TXERR` — a transaction error (no response, bad CRC, etc.).
const HCINT_TXERR: u32 = 1 << 7;
/// `HCINT.BBERR` — babble (the device sent more than expected).
const HCINT_BBERR: u32 = 1 << 8;
/// `HCINT.FRMOR` — frame overrun: a periodic transaction was still
/// running when its (micro)frame ended, so the core abandoned it. A
/// scheduling failure, not a device fault — see
/// [`TransferError::FrameOverrun`].
const HCINT_FRMOR: u32 = 1 << 9;
/// `HCINT.DTERR` — data toggle error: the packet arrived with the PID
/// this endpoint's toggle didn't expect.
const HCINT_DTERR: u32 = 1 << 10;

/// How many times to poll a split transaction's complete-split phase
/// while the hub's transaction translator answers `NYET`/`NAK` before
/// giving up. The translator relays the transaction at full/low speed,
/// which takes up to a frame, so completion can be a few microframes
/// out; this bounds the wait rather than spinning forever.
const MAX_CSPLIT_POLLS: u32 = 8;

/// Delay between complete-split polls, in microseconds — five 125µs
/// microframes, matching the reference driver's spacing. The
/// transaction translator needs time to run the relayed full/low-speed
/// transaction before it has a result, so polling faster than this
/// just wastes bus cycles getting `NYET`.
const CSPLIT_RETRY_DELAY_US: u32 = 5 * 125;

/// Host-mode DWC2 controller: reset, powered up, and (after
/// [`Self::reset_port`]) able to hand out [`Channel`]s — the host
/// channels every transfer actually runs on.
///
/// What lives here is bus-level and shared: the root port, and the pool
/// of host channels. What lives on [`Channel`] is everything to do with
/// moving bytes — DMA-mode control transfers (up to
/// [`MAX_TRANSFER_LEN`] bytes of data per transfer, the DWC2 handling
/// the multi-packet split), interrupt-IN polls and bulk transfers, on
/// directly-connected devices or full/low-speed devices behind a hub
/// (via split transactions — see [`SplitTarget`]). The split is what
/// lets two endpoints be driven at once: a [`Channel`] borrows this
/// type immutably, so several can be outstanding, each with its own
/// registers and its own DMA buffer. No enumeration state machine or
/// hub traversal lives here either — those belong above the channel
/// primitives.
pub struct Dwc2Host {
    global: USB_OTG_GLOBAL,
    host: USB_OTG_HOST,
    /// One bit per host channel, set while a [`Channel`] handle for it
    /// is outstanding. Interior-mutable so [`Self::alloc_channel`] can
    /// take `&self` and so hand out more than one channel at a time.
    allocated: Cell<u16>,
    /// How many host channels this core actually implements, read from
    /// `GHWCFG2` in [`Self::init`] rather than assumed.
    num_channels: u8,
}

impl Dwc2Host {
    /// Resets the DWC2 core, ungates its clock, forces host mode,
    /// sizes the FIFOs, and powers the root port — everything the
    /// Synopsys databook documents as needed before a device's
    /// connection can even be detected, let alone enumerated. The PHY
    /// clock select and frame interval are programmed later, in
    /// [`Self::reset_port`], once the enumerated port speed is known.
    ///
    /// The caller must have already powered the USB controller on
    /// through the VideoCore mailbox (see this module's doc); without
    /// that the register writes here all appear to succeed but no
    /// transfer ever runs.
    ///
    /// Takes a `&Timer` because two of these steps have real,
    /// documented settling times (not guesses) that must elapse
    /// before the next register access is meaningful: this crate's
    /// System Timer is already free-running from boot, so there's no
    /// reason to approximate them with a raw cycle-count busy-wait the
    /// way `pwm.rs` had to (no `Timer` reference was available there).
    pub fn init(
        global: USB_OTG_GLOBAL,
        host: USB_OTG_HOST,
        pwrclk: USB_OTG_PWRCLK,
        timer: &Timer,
    ) -> Self {
        // Core soft reset. The databook requires the AHB master be
        // idle (`AHBIDL`) before asserting it, then waiting for the
        // (self-clearing) reset bit to actually clear before touching
        // anything else.
        while global.grstctl().read().ahbidl().bit_is_clear() {}
        global.grstctl().modify(|_, w| w.csrst().set_bit());
        while global.grstctl().read().csrst().bit_is_set() {}
        // Databook-recommended settling time after core reset.
        timer.delay_ms(3);

        // Ungate the PHY/host clock: `PCGCCTL` (this SoC's "USB power"
        // register) gates the clock the entire USB transfer engine runs
        // on -- stop-Pclk, gate-HCLK, power-clamp, power-down. Its reset
        // state leaves that clock gated, so every config and per-channel
        // register (which live on a *different*, always-running bus
        // clock) programs and reads back perfectly while the transfer
        // engine itself is frozen: a channel arms flawlessly (`CHENA`
        // set, `HCTSIZ`/`HCDMA`/`HCCHAR` all correct) but is never
        // arbitrated onto the bus, `HFNUM`'s frame counter never
        // advances, and `HCINT` stays all-zero forever. Writing the
        // whole register to 0 clears every gating bit at once -- the
        // first thing a known-working reference driver for this SoC
        // (rsta2's `circle`/`uspi`) does in host init, before touching
        // FIFOs or powering the port.
        pwrclk.pcgcctl().write(|w| unsafe { w.bits(0) });

        // Configure FIFO sizes. Confirmed on real hardware, not
        // assumed: this SoC's reset default for `GNPTXFSIZ_HOST`
        // (0x0200) decodes to a non-periodic TX FIFO *depth* of 0
        // words (only `NPTXFSA`, the start address, is nonzero).
        // These FIFOs are still real and still used internally even
        // in DMA mode (the DMA engine fills/drains them automatically
        // instead of software doing it by hand) -- a zero-depth FIFO
        // is still wrong regardless of transfer mode.
        //
        // 1024 (0x400) words each, matching the reference driver for
        // this SoC (rsta2's `circle`/`uspi`), which fits comfortably in
        // this core's FIFO RAM (its total is far larger than the 3×0x400
        // used here). An earlier, far smaller RxFIFO (0x80 = 128 words)
        // worked for control and interrupt transfers but silently broke
        // bulk-IN: the host only issues an IN token once its RxFIFO can
        // hold a whole max-packet packet *plus* the status word the core
        // pushes alongside it, and 128 words is exactly one 512-byte
        // high-speed bulk packet with no room for that status -- so the
        // channel armed but `HCINT` stayed all-zero forever (the same
        // dead-channel signature as an unpowered core). A packet that
        // small (<=64 bytes, control/interrupt) left room and worked; a
        // 512-byte bulk packet did not. `NPTXFSA` must start after the
        // RxFIFO's own words, hence `0x400`.
        unsafe {
            global.grxfsiz().write(|w| w.rxfd().bits(0x400));
            global.gnptxfsiz_host().write(|w| {
                w.nptxfsa().bits(0x400);
                w.nptxfd().bits(0x400)
            });
            // Host periodic TX FIFO, sized right after the non-periodic
            // one (start 0x800 = 0x400 RX + 0x400 NPTX). On this core
            // this write does not stick -- `HPTXFSIZ` reads back 0
            // regardless -- so this SoC apparently has no separately
            // sizable host periodic TX FIFO. Harmless: interrupt (HID)
            // transfers work anyway, the core handling periodic FIFO
            // space itself.
            global.hptxfsiz().write(|w| {
                w.ptxsa().bits(0x800);
                w.ptxfd().bits(0x400)
            });
        }

        // Re-flush both FIFOs after resizing them -- the databook's
        // recommended sequence whenever FIFO sizes change, so stale
        // FIFO state from the old sizing can't linger.
        global.grstctl().modify(|_, w| w.rxfflsh().set_bit());
        while global.grstctl().read().rxfflsh().bit_is_set() {}
        unsafe {
            global
                .grstctl()
                .modify(|_, w| w.txfnum().bits(0x10).txfflsh().set_bit());
        }
        while global.grstctl().read().txfflsh().bit_is_set() {}

        // Force host mode. The databook requires waiting 25ms after
        // this before the mode change is guaranteed to have taken
        // effect.
        global.gusbcfg().modify(|_, w| w.fhmod().set_bit());
        timer.delay_ms(25);

        // Enable DMA mode. Confirmed on real hardware, not assumed:
        // `GHWCFG2.ARCHITECTURE` reports `internal_dma`, not
        // `slave_only` -- this core doesn't implement slave/PIO mode
        // at all, so this bit isn't optional the way it might be on
        // other DWC2 instantiations.
        // `AXI_WAIT`: wait for all AXI writes to actually land before
        // signaling DMA completion -- matches a known-working
        // Broadcom-targeting reference driver's core init (rsta2's
        // `uspi`/`circle`), not something the generic Synopsys
        // databook flags as host-mode-mandatory, but cheap and
        // directly relevant to whether this driver's own DMA
        // completion signaling can be trusted.
        // `GINT`: enables the core asserting its interrupt line to the
        // CPU. Irrelevant to this pure-polling driver (nothing in this
        // crate unmasks IRQ at the CPU level or routes USB through
        // `Lic`, so the line, if asserted, is observed by nothing), but
        // set to match the reference drivers' core init and leave the
        // core ready for a future interrupt-driven path.
        global
            .gahbcfg()
            .modify(|_, w| w.dmaen().set_bit().axi_wait().set_bit().gint().set_bit());

        // Unmask the host-channel and port interrupts in `GINTMSK`, and
        // mask start-of-frame. `GINTMSK` gates whether a pending source
        // propagates to the core's interrupt line, so it has no bearing
        // on the blocking path (which reads `HCINT`/`HFNUM` directly);
        // it decides what an application that routes USB to the ARM core
        // (`Lic::enable_usb_irq`) will actually see. `HCIM` (host
        // channels, bit 25) and `PRTIM` (port, bit 24) use raw bits
        // because this PAC generates a reader but no writer for the
        // port-interrupt mask.
        //
        // `SOFM` is deliberately *cleared*. A high-speed port raises SOF
        // every 125µs, so leaving it unmasked hands an application that
        // enables the USB IRQ an 8kHz interrupt with nothing to service
        // it — and since the line stays asserted until `GINTSTS.SOF` is
        // acked, the core re-enters the handler forever rather than
        // merely wasting cycles. Only [`Channel`]'s periodic-split
        // scheduling needs SOF, and it unmasks the bit for exactly as
        // long as some channel is waiting on a microframe.
        global.gintmsk().modify(|_, w| w.sofm().clear_bit());
        unsafe {
            global
                .gintmsk()
                .modify(|r, w| w.bits(r.bits() | (1 << 24) | (1 << 25)));
        }

        // HNP/SRP (OTG session negotiation capability) aren't used by
        // a plain host-only driver -- also matching the same
        // reference driver, which explicitly clears both. Left at
        // their reset defaults, this core may run OTG session/HNP
        // negotiation logic that competes with straightforward
        // host-mode operation; `SRQINT`/`CIDSCHG` (OTG session/
        // connector-ID events) showing up in this driver's own
        // `GINTSTS` dumps during bring-up is consistent with that.
        global
            .gusbcfg()
            .modify(|_, w| w.hnpcap().clear_bit().srpcap().clear_bit());

        // NB: `HCFG.FSLSPCS` (FS/LS PHY clock select) and `HFIR`
        // (frame interval) are deliberately *not* programmed here, even
        // though they're logically part of host setup. On real hardware,
        // the port's connect/enable hardware sequence overwrites both:
        // writing `HFIR = 48000` and `HCFG.FSLSPCS = 1` here, then
        // reading them back after a device connected, showed `HFIR`
        // reverted to its 60MHz reset default (0xea60) and `HCFG`
        // cleared -- the core recomputes them from the enumerated port
        // speed once the port comes up, discarding whatever was written
        // beforehand. They're programmed in [`Self::reset_port`]
        // instead, after the port is enabled and its speed is known,
        // which is also where real DWC2 host drivers do it (their
        // port-enable-change interrupt handler).

        // Power the root port. `.write()`, not `.modify()`: `PCDET`/
        // `PENCHNG`/`POCCHNG` are write-1-to-clear status bits, and
        // `PENA` disables an already-enabled port if written 1 without
        // meaning to (a well-known DWC2 footgun) — `.modify()` would
        // read back and re-write whatever they currently hold,
        // silently clearing or disabling something this call never
        // meant to touch. `.write()`'s zeroed baseline (this
        // register's reset value) plus only ever setting `ppwr` avoids
        // that entirely.
        host.hprt().write(|w| w.ppwr().set_bit());

        // How many host channels this core implements. `GHWCFG2`
        // (`hw_config0` in this PAC — see the module doc on its naming)
        // encodes it as the count minus one, so a core reporting 7 has
        // 8. Read rather than hardcoded, and still capped at the number
        // the PAC will index: `host_channel(n)` panics at compile time
        // past 12, and a bogus read must not turn into an out-of-range
        // register access.
        let num_channels =
            (global.hw_config0().read().host_channel_count().bits() + 1).min(MAX_CHANNELS as u8);

        Self {
            global,
            host,
            allocated: Cell::new(0),
            num_channels,
        }
    }

    /// How many host channels this core implements — the ceiling on how
    /// many [`Channel`]s can be outstanding at once (8 on this SoC).
    pub fn num_channels(&self) -> usize {
        self.num_channels as usize
    }

    /// Takes the lowest-numbered free host channel out of the pool,
    /// returning `None` when every channel this core has is already
    /// handed out.
    ///
    /// Exhaustion is reported, not queued: a caller that needs a channel
    /// and can't have one is in a position to say so (or to wait on its
    /// own terms), whereas a queue hidden in here would silently
    /// serialise transfers a caller believed were concurrent. The
    /// [`Channel`] returns its slot to the pool when dropped, so a
    /// short-lived transfer can scope one rather than holding it.
    ///
    /// Takes `&self`, so several channels can be outstanding at once —
    /// that's the whole point of the split between this type and
    /// [`Channel`].
    pub fn alloc_channel(&self) -> Option<Channel<'_>> {
        let allocated = self.allocated.get();
        let index = (0..self.num_channels as usize).find(|i| allocated & (1 << i) == 0)?;
        self.allocated.set(allocated | (1 << index));
        Some(Channel {
            owner: self,
            index,
            dma_buffer: DmaBuffer([0; DMA_BUFFER_LEN]),
            last_hcint: Cell::new(0),
        })
    }

    /// True if the root port currently reports a device electrically
    /// connected (`HPRT.PCSTS`). On this board, in practice, that's
    /// the on-board SMSC LAN9514 hub — see this module's parent.
    pub fn port_connected(&self) -> bool {
        self.host.hprt().read().pcsts().bit_is_set()
    }

    /// USB bus reset on the root port — required before any transfer
    /// will work, and before [`Self::port_speed`] is meaningful.
    /// Asserts `PRST` for 50ms (generous relative to the USB 2.0
    /// spec's 10ms minimum reset signaling time, matching what most
    /// real host controllers use for root-hub-facing ports), then
    /// clears it and waits 10ms recovery time (also spec-mandated).
    /// Finally programs the FS/LS PHY clock and frame interval now
    /// that the port is enabled and its speed is known (see below).
    ///
    /// Same `.write()`-with-zeroed-status-bits care as [`Self::init`]'s
    /// port power-up — see that method's doc comment.
    pub fn reset_port(&self, timer: &Timer) {
        self.host
            .hprt()
            .write(|w| w.ppwr().set_bit().prst().set_bit());
        timer.delay_ms(50);
        self.host.hprt().write(|w| w.ppwr().set_bit());
        timer.delay_ms(10);

        // Acknowledge the port-change status bits the reset/enable
        // sequence latched (`PCDET` device-connect, `PENCHNG` enable-
        // change, `POCCHNG` overcurrent-change) -- all write-1-to-clear.
        // Standard post-reset housekeeping the reference drivers do:
        // left pending they keep `GINTSTS.PRTINT` permanently asserted,
        // stale noise for any future interrupt-driven path. Same
        // `.write()`-with-`ppwr`-only care as [`Self::init`]'s port
        // power-up -- setting `PENA` here would *disable* the port, so
        // it's deliberately left 0 in this write (writing 0 to `PENA`
        // is a no-op; only writing 1 disables).
        self.host.hprt().write(|w| {
            w.ppwr().set_bit();
            w.pcdet().set_bit();
            w.penchng().set_bit();
            w.pocchng().set_bit()
        });

        // Program the FS/LS PHY clock select and frame interval *here*,
        // after the port has been reset and enabled, rather than in
        // `init`. On real hardware the port's connect/enable hardware
        // sequence overwrites both `HCFG.FSLSPCS` and `HFIR` with
        // values recomputed from the enumerated port speed, discarding
        // anything written before the port came up (see `init`'s note).
        // This is also where real DWC2 host drivers do it — in their
        // port-enable-change interrupt handler, keyed off the detected
        // speed — not once at core init.
        //
        // Only for a full/low-speed port. This SoC's DWC2 uses an
        // internal UTMI+ PHY that does reach high speed (the on-board
        // LAN9514 hub enumerates at HS on this root port), and at HS
        // the port bring-up already leaves `HFIR` at the correct 60MHz-
        // based reset default (0xea60 = 60000 = 1ms) and `FSLSPCS` is a
        // don't-care. Only a full/low-speed port needs the 48MHz FS/LS
        // PHY clock selected (`FSLSPCS = 1`) and the matching 1ms frame
        // interval (`HFIR.FRIVL = 48000`); forcing those at HS would
        // set the wrong frame interval.
        if self.port_speed() != 0 {
            unsafe {
                self.host.hcfg().modify(|_, w| w.fslspcs().bits(1));
                self.host.hfir().write(|w| w.frivl().bits(48_000));
            }
        }
    }

    /// True if the root port is enabled (`HPRT.PENA`) — set by
    /// hardware once a device successfully responds to
    /// [`Self::reset_port`], never something software sets directly
    /// (writing 1 here manually would actually *disable* an
    /// already-enabled port — see [`Self::init`]'s doc comment on the
    /// same footgun).
    pub fn port_enabled(&self) -> bool {
        self.host.hprt().read().pena().bit_is_set()
    }

    /// The root port's detected device speed (`HPRT.PSPD`), valid only
    /// once [`Self::port_enabled`] is true: `0` = high-speed, `1` =
    /// full-speed, `2` = low-speed. All three are reachable on this SoC
    /// — the on-board LAN9514 hub enumerates at high speed on this root
    /// port.
    pub fn port_speed(&self) -> u8 {
        self.host.hprt().read().pspd().bits()
    }

    /// Direct access to the underlying global/OTG control registers,
    /// for diagnostics or lower-level work this type doesn't expose
    /// yet — mirrors [`crate::pac`]'s own role for other drivers in
    /// this crate.
    pub fn global(&self) -> &USB_OTG_GLOBAL {
        &self.global
    }

    /// Direct access to the underlying host-mode registers (per-
    /// channel registers included) — see [`Self::global`].
    pub fn host(&self) -> &USB_OTG_HOST {
        &self.host
    }

    /// Returns a channel's slot to the pool. Called by [`Channel`]'s
    /// `Drop`, which is the only way a slot is ever freed.
    fn free_channel(&self, index: usize) {
        self.allocated.set(self.allocated.get() & !(1 << index));
    }
}

/// One host channel, taken from [`Dwc2Host::alloc_channel`] — the thing
/// a USB transfer actually runs on.
///
/// Every transfer primitive lives here rather than on [`Dwc2Host`]
/// because a host channel is genuinely independent hardware: its
/// registers are its own, and the core arbitrates between channels by
/// itself. Holding one is therefore what grants the right to run a
/// transfer, and two callers holding two channels can drive two
/// endpoints without either taking a mutable borrow of the controller.
///
/// The channel carries the DMA scratch buffer its control and
/// interrupt transfers stage through, so it is not a trivially cheap
/// value to move around — allocate one per endpoint that needs one and
/// keep it, rather than re-allocating per transfer. Dropping it frees
/// the slot for another caller.
pub struct Channel<'a> {
    /// The controller this channel belongs to — borrowed immutably, so
    /// its siblings can be outstanding at the same time.
    owner: &'a Dwc2Host,
    /// This channel's index into the core's host-channel registers.
    index: usize,
    /// This channel's own DMA staging buffer — see [`DmaBuffer`].
    dma_buffer: DmaBuffer,
    /// Raw `HCINT` value captured at the most recent halt on this
    /// channel — see [`Self::last_interrupt`]. Interior-mutable so the
    /// transfer path (which takes `&self`) can record it.
    last_hcint: Cell<u32>,
}

impl Drop for Channel<'_> {
    fn drop(&mut self) {
        self.owner.free_channel(self.index);
    }
}

impl Channel<'_> {
    /// This channel's index into the core's host-channel registers.
    /// Only of interest for diagnostics — nothing in this API takes a
    /// channel number any more.
    pub fn index(&self) -> usize {
        self.index
    }

    /// The raw `HCINT` (host channel interrupt) value captured the last
    /// time this channel halted — the exact bits the transfer's
    /// [`TransferError`]/success was derived from (`XFRC`, `STALL`,
    /// `NAK`, `ACK`, `NYET`, `TXERR`, `BBERR`, `CHH`, …). This driver
    /// collapses those bits into a coarse result; a caller debugging a
    /// transfer that fails or behaves unexpectedly can read the
    /// underlying bits here to see exactly what the hardware reported.
    /// Only meaningful immediately after a transfer method returns.
    pub fn last_interrupt(&self) -> u32 {
        self.last_hcint.get()
    }

    /// The core's host-mode register block, for the handful of shared
    /// registers a channel touches (`HAINTMSK`, `HFNUM`) alongside its
    /// own.
    fn regs(&self) -> &USB_OTG_HOST {
        &self.owner.host
    }

    /// This channel's own register block.
    fn ch(&self) -> &crate::pac::usb_otg_host::HOST_CHANNEL {
        self.owner.host.host_channel(self.index)
    }

    /// Runs one SETUP-stage transaction: 8 raw bytes, always PID SETUP,
    /// always OUT direction (a SETUP token is functionally an OUT
    /// transfer at the protocol level).
    pub fn control_setup(
        &mut self,
        endpoint: ControlEndpoint,
        setup: &[u8; 8],
        timer: &Timer,
    ) -> Result<(), TransferError> {
        self.dma_buffer.0[..8].copy_from_slice(setup);
        let address = self.dma_buffer.0.as_ptr() as u32;
        clean_range(address, 8);

        let txn = Transaction {
            endpoint_number: 0,
            endpoint_type: EndpointType::Control,
            direction_in: false,
            pid: DataPid::Setup,
            transfer_size: 8,
            dma_address: address,
        };
        self.run_transaction(endpoint, &txn, timer)?;
        Ok(())
    }

    /// Runs one DATA-stage IN transaction, reading up to `buf.len()`
    /// bytes (always starting at PID DATA1 — see [`DataPid::Data1`]; the
    /// DWC2 toggles DATA1/DATA0 itself across a multi-packet transfer).
    /// Returns the number of bytes actually received, which can be less
    /// than `buf.len()` if the device sent a short packet. `buf.len()`
    /// must be at most [`MAX_TRANSFER_LEN`] — this channel's DMA scratch
    /// buffer size.
    pub fn control_data_in(
        &mut self,
        endpoint: ControlEndpoint,
        buf: &mut [u8],
        timer: &Timer,
    ) -> Result<usize, TransferError> {
        debug_assert!(buf.len() <= DMA_BUFFER_LEN);

        let address = self.dma_buffer.0.as_ptr() as u32;
        let txn = Transaction {
            endpoint_number: 0,
            endpoint_type: EndpointType::Control,
            direction_in: true,
            pid: DataPid::Data1,
            transfer_size: buf.len() as u32,
            dma_address: address,
        };
        let received = self.run_transaction(endpoint, &txn, timer)?;

        invalidate_range(address, buf.len());
        buf[..received].copy_from_slice(&self.dma_buffer.0[..received]);
        Ok(received)
    }

    /// Runs one zero-length STATUS-stage OUT transaction (always PID
    /// DATA1 — see [`DataPid::Data1`]), completing a control transfer
    /// that had a device-to-host (IN) data stage, or no data stage in
    /// the host-to-device direction.
    pub fn control_status_out(
        &mut self,
        endpoint: ControlEndpoint,
        timer: &Timer,
    ) -> Result<(), TransferError> {
        let address = self.dma_buffer.0.as_ptr() as u32;
        let txn = Transaction {
            endpoint_number: 0,
            endpoint_type: EndpointType::Control,
            direction_in: false,
            pid: DataPid::Data1,
            transfer_size: 0,
            dma_address: address,
        };
        self.run_transaction(endpoint, &txn, timer)?;
        Ok(())
    }

    /// Runs one zero-length STATUS-stage IN transaction (always PID
    /// DATA1 — see [`DataPid::Data1`]), completing a control transfer
    /// that had no data stage (e.g. SET_ADDRESS, SET_CONFIGURATION): a
    /// no-data control transfer's status stage is always IN, the
    /// opposite of its (absent) data stage.
    pub fn control_status_in(
        &mut self,
        endpoint: ControlEndpoint,
        timer: &Timer,
    ) -> Result<(), TransferError> {
        let address = self.dma_buffer.0.as_ptr() as u32;
        let txn = Transaction {
            endpoint_number: 0,
            endpoint_type: EndpointType::Control,
            direction_in: true,
            pid: DataPid::Data1,
            transfer_size: 0,
            dma_address: address,
        };
        self.run_transaction(endpoint, &txn, timer)?;
        Ok(())
    }

    /// Runs one DATA-stage OUT transaction, sending `data` (always
    /// starting at PID DATA1 — see [`DataPid::Data1`]; the DWC2 toggles
    /// across a multi-packet transfer itself). For the host-to-device
    /// control transfers that carry data (e.g. a vendor register write —
    /// see [`crate::usb::control::vendor_out`]); the plain SET_ADDRESS/
    /// SET_CONFIGURATION requests have no data stage and use
    /// [`Self::control_status_in`] directly. `data.len()` must be at most
    /// [`MAX_TRANSFER_LEN`].
    pub fn control_data_out(
        &mut self,
        endpoint: ControlEndpoint,
        data: &[u8],
        timer: &Timer,
    ) -> Result<(), TransferError> {
        debug_assert!(data.len() <= DMA_BUFFER_LEN);

        self.dma_buffer.0[..data.len()].copy_from_slice(data);
        let address = self.dma_buffer.0.as_ptr() as u32;
        clean_range(address, data.len());

        let txn = Transaction {
            endpoint_number: 0,
            endpoint_type: EndpointType::Control,
            direction_in: false,
            pid: DataPid::Data1,
            transfer_size: data.len() as u32,
            dma_address: address,
        };
        self.run_transaction(endpoint, &txn, timer)?;
        Ok(())
    }

    /// Runs one interrupt-IN transaction on `endpoint_number` of
    /// `endpoint`'s device, reading up to `buf.len()` bytes into `buf`
    /// and returning how many were received (0 is possible). `endpoint`
    /// supplies the device address/speed/split and *this endpoint's*
    /// max packet size.
    ///
    /// `data_toggle` holds the endpoint's DATA0/DATA1 toggle, which —
    /// unlike a control transfer — persists across transfers: start it
    /// `false` (DATA0) when the device is configured, then pass the same
    /// `&mut bool` back on every poll. It flips only on a successful
    /// transfer, and is left unchanged on [`TransferError::Nak`] (the
    /// endpoint had nothing to send — the normal "no event yet" answer
    /// when polling), so a NAK'd poll can simply be retried.
    ///
    /// The caller is responsible for pacing polls (the endpoint's
    /// `bInterval`).
    pub fn interrupt_in(
        &mut self,
        endpoint: ControlEndpoint,
        endpoint_number: u8,
        data_toggle: &mut bool,
        buf: &mut [u8],
        timer: &Timer,
    ) -> Result<usize, TransferError> {
        debug_assert!(buf.len() <= DMA_BUFFER_LEN);

        let address = self.dma_buffer.0.as_ptr() as u32;
        let txn = Transaction {
            endpoint_number,
            endpoint_type: EndpointType::Interrupt,
            direction_in: true,
            pid: if *data_toggle {
                DataPid::Data1
            } else {
                DataPid::Data0
            },
            transfer_size: buf.len() as u32,
            dma_address: address,
        };
        let received = self.run_transaction(endpoint, &txn, timer)?;
        // Only a completed transfer advances the toggle; a NAK left it
        // untouched by returning early above.
        *data_toggle = !*data_toggle;

        invalidate_range(address, buf.len());
        buf[..received].copy_from_slice(&self.dma_buffer.0[..received]);
        Ok(received)
    }

    /// Runs one bulk-OUT transfer on `endpoint_number` of `endpoint`'s
    /// device, sending all of `buf` (which may span several max-packet
    /// packets — the DWC2 splits it and toggles the PID across them from
    /// one channel start). Returns the number of bytes sent. DMA reads
    /// straight from `buf`, so there's no size cap from the shared
    /// control scratch buffer.
    ///
    /// `data_toggle` is the endpoint's persistent DATA0/DATA1 toggle,
    /// like [`Self::interrupt_in`]'s: start it `false` after configuring
    /// the endpoint and pass the same `&mut bool` each call. On success
    /// it's updated to the toggle the *next* transfer must resume with
    /// (read back from the hardware, which is correct even across a
    /// multi-packet transfer); a [`TransferError::Nak`] leaves it
    /// unchanged so the caller can retry.
    ///
    /// Not yet exercised on real hardware — added as the primitive the
    /// LAN9514 Ethernet driver's frame TX will use (see
    /// [`crate::usb::lan9514`]).
    pub fn bulk_out(
        &mut self,
        endpoint: ControlEndpoint,
        endpoint_number: u8,
        data_toggle: &mut bool,
        buf: &[u8],
        timer: &Timer,
    ) -> Result<usize, TransferError> {
        let address = buf.as_ptr() as u32;
        clean_range(address, buf.len());

        let txn = Transaction {
            endpoint_number,
            endpoint_type: EndpointType::Bulk,
            direction_in: false,
            pid: Self::toggle_pid(*data_toggle),
            transfer_size: buf.len() as u32,
            dma_address: address,
        };
        let sent = self.run_transaction(endpoint, &txn, timer)?;
        *data_toggle = self.next_toggle();
        Ok(sent)
    }

    /// Runs one bulk-IN transfer on `endpoint_number` of `endpoint`'s
    /// device, reading up to `buf.len()` bytes into `buf` and returning
    /// how many arrived (a short packet ends it early). DMA writes
    /// straight into `buf`, so `buf` **must be cache-line aligned and
    /// occupy whole cache lines** — the cache invalidation after the
    /// transfer works at cache-line granularity, and a buffer sharing a
    /// line with other live data would have that data discarded.
    ///
    /// `data_toggle` behaves as in [`Self::bulk_out`].
    ///
    /// `buf` is rounded *down* to a whole number of max-packet packets
    /// for the transfer size: the DWC2 won't run a multi-packet IN whose
    /// programmed size isn't a multiple of the max packet size (the
    /// channel arms but never leaves `HCINT` all-zero). A short packet
    /// still ends the transfer early, so this only caps how much one call
    /// can read — `buf` should therefore be at least one max packet, and
    /// sizing it to a whole number of packets wastes nothing.
    pub fn bulk_in(
        &mut self,
        endpoint: ControlEndpoint,
        endpoint_number: u8,
        data_toggle: &mut bool,
        buf: &mut [u8],
        timer: &Timer,
    ) -> Result<usize, TransferError> {
        let address = buf.as_mut_ptr() as u32;
        debug_assert!(
            address.is_multiple_of(MIN_CACHE_LINE),
            "bulk IN buffer must be cache-line aligned"
        );

        let max_packet_size = endpoint.max_packet_size.max(1) as u32;
        let transfer_size = buf.len() as u32 / max_packet_size * max_packet_size;

        let txn = Transaction {
            endpoint_number,
            endpoint_type: EndpointType::Bulk,
            direction_in: true,
            pid: Self::toggle_pid(*data_toggle),
            transfer_size,
            dma_address: address,
        };
        let received = self.run_transaction(endpoint, &txn, timer)?;
        *data_toggle = self.next_toggle();

        invalidate_range(address, buf.len());
        Ok(received)
    }

    /// The [`DataPid`] a persistent toggle (`false` = DATA0, `true` =
    /// DATA1) selects for the next bulk/interrupt packet.
    fn toggle_pid(data_toggle: bool) -> DataPid {
        if data_toggle {
            DataPid::Data1
        } else {
            DataPid::Data0
        }
    }

    /// The data toggle (`true` = DATA1) to resume the next transfer on
    /// this channel with, read back from `HCTSIZ.DPID` after a completed
    /// transfer — the DWC2 leaves it pointing at the next expected PID,
    /// which is correct even when the transfer spanned several packets
    /// (so it can't just be flipped like the single-packet interrupt
    /// path does).
    fn next_toggle(&self) -> bool {
        self.ch().hctsiz().read().dpid().bits() == DataPid::Data1.bits()
    }

    /// Programs `HCSPLT`/`HCDMA`/`HCTSIZ`/`HCCHAR` for one DMA
    /// transaction described by `txn`
    /// and enables the channel — this alone starts the transaction (the
    /// DMA engine pulls from / pushes to `txn.dma_address` on its own
    /// once `CHENA` is set); [`Self::wait_for_halt`] just waits for it
    /// to finish. `txn.dma_address` is a plain physical address,
    /// translated to the VideoCore bus alias here (see
    /// [`to_vc_bus_address`]). `complete_split` selects the
    /// start-split (`false`) vs complete-split (`true`) phase and is
    /// meaningful only when `endpoint.split` is set.
    ///
    /// `.write()`, not `.modify()`, for `HCCHAR`: unlike `HPRT`, it's
    /// fully per-channel, so there's no cross-channel state to
    /// preserve — every field this transaction needs is specified
    /// here.
    ///
    /// `odd_frame` overrides the `HCCHAR.ODDFRM` bit: periodic split
    /// scheduling (see [`Self::run_periodic_split_packet`]) computes it
    /// per microframe and passes `Some`, while everything else passes
    /// `None` to get the default (frame parity for a direct transfer,
    /// cleared for a non-periodic split).
    fn start_channel(
        &self,
        endpoint: ControlEndpoint,
        txn: &Transaction,
        complete_split: bool,
        odd_frame: Option<bool>,
    ) {
        let ch = self.ch();

        // Clear any stale interrupt bits left over from a previous
        // transaction on this channel -- `HCINT` bits are
        // write-1-to-clear and don't self-clear just because a new
        // transaction is starting.
        unsafe {
            ch.hcint().write(|w| w.bits(0xffff_ffff));
        }

        // Program the per-channel interrupt mask explicitly, rather
        // than leaving it at its reset value — the known-working
        // reference driver for this SoC does the same before every
        // channel start.
        //
        // `CHH` alone, not every condition. `HCINTMSK` gates only what
        // reaches `HAINT` and the core's interrupt line; the `HCINT`
        // bits themselves latch regardless, so the blocking path (which
        // reads `HCINT` directly) sees exactly what it always did, and
        // so does [`Self::interpret_halt`] at the halt. What the
        // narrower mask buys is that "this channel raised `HAINT`" means
        // "this channel halted" and nothing else — which is what
        // [`asynch::on_irq`] needs. With every condition unmasked, an
        // intermediate `ACK` or `NAK` would assert the line too, and a
        // handler would have to either ack it (destroying a bit the
        // split logic reads at the halt — `run_split_packet` keys off
        // `ACK`) or leave it asserted (a re-entering interrupt forever).
        unsafe {
            ch.hcintmsk().write(|w| w.bits(HCINT_CHH));
        }

        // `HAINTMSK` -- which decides whether this channel's `HCINT`
        // reaches `GINTSTS` and so the CPU -- is deliberately *not*
        // touched here, even though this is where the reference driver
        // for this SoC (Broadcom/Raspberry Pi Foundation's `dwc_otg`)
        // sets it. It is managed by the async layer instead, per wait
        // (see `asynch`'s `arm`/`disarm`).
        //
        // The distinction only matters once something routes USB to the
        // ARM core, and then it matters a great deal. This function
        // serves both the blocking and the async path. A blocking
        // transfer wants no interrupt at all: it polls `HCINT` itself.
        // If its channel were unmasked here, its halt would assert the
        // controller's interrupt line, and the handler -- which
        // services only channels an async transfer is waiting on, and
        // must leave a polled channel's `HCINT` latched for its owner
        // to read -- would return without acking it. The line is level
        // triggered, so the core would re-enter the handler forever and
        // the polling loop it interrupted would never resume. A hang,
        // not a slow-down, and only in a program that mixes the two
        // styles.

        // Split control (`HCSPLT`): for a device behind a high-speed
        // hub's transaction translator, enable the split and address
        // the translator (hub address + downstream port), with
        // `complsplt` selecting the start-split (`false`) or
        // complete-split (`true`) phase and `xactpos = 0b11` meaning the
        // whole payload (not an isochronous-OUT slice). For a
        // direct-connected device, zero it — explicitly, so leftover
        // state can't make the channel act like a split.
        unsafe {
            match endpoint.split {
                Some(split) => ch.hcsplt().write(|w| {
                    w.hubaddr().bits(split.hub_address);
                    w.prtaddr().bits(split.port);
                    w.xactpos().bits(0b11);
                    w.complsplt().bit(complete_split);
                    w.spliten().set_bit()
                }),
                None => ch.hcsplt().write(|w| w.bits(0)),
            }
        }

        unsafe {
            ch.hcdma()
                .write(|w| w.dmaaddr().bits(to_vc_bus_address(txn.dma_address)));
        }

        // Packet count: how many max-packet-size packets `transfer_size`
        // spans, rounded up, minimum 1 (a zero-length transfer is still
        // one — empty — packet). With `PKTCNT > 1` the DWC2 DMA engine
        // moves each packet to/from consecutive buffer offsets and
        // toggles the DATA0/DATA1 PID itself, so a whole multi-packet
        // control-data stage runs from a single channel start.
        let max_packet_size = endpoint.max_packet_size.max(1) as u32;
        let packet_count = txn.transfer_size.div_ceil(max_packet_size).max(1);

        unsafe {
            ch.hctsiz().write(|w| {
                w.xfrsiz().bits(txn.transfer_size);
                w.pktcnt().bits(packet_count as u16);
                w.dpid().bits(txn.pid.bits())
            });
        }

        // Odd/even frame bit. A periodic split passes it in explicitly
        // (its scheduler picks the microframe each start-/complete-split
        // runs in — see [`Self::run_periodic_split_packet`]). Otherwise:
        // for a direct (non-split) transfer, set it from the current
        // frame's parity — the reference drivers for this SoC (rsta2's
        // `circle`/`uspi`) do this on every non-split channel start
        // regardless of endpoint type. For a non-periodic split transfer
        // it must be *cleared*: those same drivers' non-periodic split
        // scheduler reports odd-frame = false, and leaving frame parity
        // in it here mis-schedules the start-split (observed as a
        // transaction error on the start-split).
        let odd_frame = match odd_frame {
            Some(odd) => odd,
            None if endpoint.split.is_some() => false,
            None => self.regs().hfnum().read().frnum().bits() & 1 == 1,
        };

        unsafe {
            ch.hcchar().write(|w| {
                w.mpsiz().bits(endpoint.max_packet_size);
                w.epnum().bits(txn.endpoint_number);
                w.epdir().bit(txn.direction_in);
                w.lsdev().bit(endpoint.low_speed);
                w.eptyp().bits(txn.endpoint_type.bits());
                w.mc().bits(1);
                w.dad().bits(endpoint.address);
                w.oddfrm().bit(odd_frame);
                w.chena().set_bit()
            });
        }
    }

    /// Acknowledges (clears) every pending bit in this channel's
    /// `HCINT`.
    fn ack_hcint(&self) {
        unsafe {
            self.ch().hcint().write(|w| w.bits(0xffff_ffff));
        }
    }

    /// Forcibly halts this channel after [`Self::run_channel`] times out,
    /// leaving it in a clean, reusable state. A timeout means `HCINT`
    /// never signaled completion, so — per the DWC2 databook — `CHENA`
    /// may still be set and the transaction still technically active in
    /// hardware. Confirmed the hard way on real hardware: once one
    /// control transfer timed out, every subsequent attempt on that
    /// same channel *also* failed, each a different way (`Stall`, then
    /// `Timeout` again, ...) — consistent with each new attempt's
    /// `start_channel` call reprogramming and re-enabling a channel
    /// that was still live from the timed-out one, rather than starting
    /// clean. The databook's documented way to abort an active channel
    /// is requesting a disable (`CHDIS` set, `CHENA` left set) and
    /// waiting for `HCINT.CHH` (channel halted); this does that, with
    /// its own short timeout so a channel that won't halt can't wedge
    /// the caller forever either.
    fn abort_channel(&self, timer: &Timer) {
        const ABORT_TIMEOUT_US: u64 = 10_000;

        let ch = self.ch();
        ch.hcchar()
            .modify(|_, w| w.chdis().set_bit().chena().set_bit());

        let start = timer.now_micros();
        while ch.hcint().read().chh().bit_is_clear() {
            if timer.now_micros() - start > ABORT_TIMEOUT_US {
                break;
            }
        }
        self.ack_hcint();
    }

    /// Waits for this channel to halt (`HCINT.CHH` — the DMA-mode
    /// completion signal), returning the raw `HCINT` value at that
    /// point. On timeout (the channel never halts) forcibly disables it
    /// via [`Self::abort_channel`] and returns [`TransferError::Timeout`].
    ///
    /// Polls for `CHH` specifically, not `XFRC`: a halted channel is
    /// already in a clean, reusable state, so returning on `CHH`
    /// (whatever the reason) lets [`Self::start_channel`] re-arm it
    /// directly on the next call. Waiting on `XFRC` instead would spin
    /// the full timeout on any non-success halt (`STALL`, `NAK`, a
    /// complete-split `NYET`, or the reason-less `CHH`-only halt this
    /// SoC produces intermittently) and then have to force a disable —
    /// and forcing `CHDIS`/`CHENA` on an already-halted channel was
    /// observed to wedge the core into a dead, all-zero-`HCINT` state.
    fn wait_for_halt(&self, timer: &Timer) -> Result<u32, TransferError> {
        const TIMEOUT_US: u64 = 50_000;

        let ch = self.ch();
        let start = timer.now_micros();
        loop {
            let hcint = ch.hcint().read().bits();
            if hcint & HCINT_CHH != 0 {
                self.last_hcint.set(hcint);
                self.ack_hcint();
                return Ok(hcint);
            }
            if timer.now_micros() - start > TIMEOUT_US {
                // Record the channel's `HCINT` at the moment it timed out
                // (before the abort clears it) so [`Self::last_interrupt`]
                // reflects the stuck transfer, not the last successful one.
                self.last_hcint.set(hcint);
                self.abort_channel(timer);
                return Err(TransferError::Timeout);
            }
        }
    }

    /// Maps a halted channel's `HCINT` bits to a transfer result. On
    /// `XFRC` (success) returns the bytes actually transferred —
    /// `requested` (the originally programmed size) minus
    /// `HCTSIZ.XFRSIZ`'s leftover count, which is why `requested` is
    /// passed in; otherwise maps the reason bit to a [`TransferError`],
    /// or [`TransferError::Halted`] for a halt with no reason bit set.
    fn interpret_halt(&self, hcint: u32, requested: usize) -> Result<usize, TransferError> {
        if hcint & HCINT_XFRC != 0 {
            let remaining = self.ch().hctsiz().read().xfrsiz().bits() as usize;
            Ok(requested - remaining)
        } else if hcint & HCINT_STALL != 0 {
            Err(TransferError::Stall)
        } else if hcint & HCINT_TXERR != 0 {
            Err(TransferError::TransactionError)
        } else if hcint & HCINT_BBERR != 0 {
            Err(TransferError::Babble)
        } else if hcint & HCINT_FRMOR != 0 {
            Err(TransferError::FrameOverrun)
        } else if hcint & HCINT_DTERR != 0 {
            Err(TransferError::DataToggleError)
        } else if hcint & HCINT_NAK != 0 {
            Err(TransferError::Nak)
        } else {
            Err(TransferError::Halted)
        }
    }

    /// Runs one non-split channel transaction: wait for the (already
    /// armed) channel to halt, then interpret the result.
    fn run_channel(&self, timer: &Timer) -> Result<usize, TransferError> {
        let requested = self.ch().hctsiz().read().xfrsiz().bits() as usize;
        let hcint = self.wait_for_halt(timer)?;
        self.interpret_halt(hcint, requested)
    }

    /// Arms and runs one channel transfer, transparently doing split
    /// transactions when `endpoint.split` is set, or a plain transaction
    /// otherwise. `txn.transfer_size` bytes move to/from
    /// `txn.dma_address` (a plain physical address; [`Self::start_channel`]
    /// translates it). Returns the number of bytes moved.
    fn run_transaction(
        &self,
        endpoint: ControlEndpoint,
        txn: &Transaction,
        timer: &Timer,
    ) -> Result<usize, TransferError> {
        if endpoint.split.is_none() {
            // Direct: the DWC2 handles the whole (possibly multi-packet)
            // transfer from one channel start.
            self.start_channel(endpoint, txn, false, None);
            return self.run_channel(timer);
        }

        // Split: the DWC2 does one packet per start-split/complete-split,
        // so a multi-packet transfer is driven a packet at a time here,
        // advancing the buffer and toggling the data PID as it goes.
        // Interrupt (periodic) splits are scheduled against the
        // microframe counter; control (non-periodic) splits aren't.
        let periodic = txn.endpoint_type == EndpointType::Interrupt;
        let max_packet_size = endpoint.max_packet_size.max(1) as u32;
        let mut offset = 0;
        let mut pid = txn.pid;
        loop {
            let this_packet = (txn.transfer_size - offset).min(max_packet_size);
            let packet = Transaction {
                transfer_size: this_packet,
                dma_address: txn.dma_address + offset,
                pid,
                ..*txn
            };
            let received = if periodic {
                self.run_periodic_split_packet(endpoint, &packet, timer)? as u32
            } else {
                self.run_split_packet(endpoint, &packet, timer)? as u32
            };
            offset += received;
            pid = pid.toggled();
            // Done once the whole transfer is satisfied, or the device
            // ended it early with a short packet (fewer than a full max
            // packet size).
            if received < this_packet || offset >= txn.transfer_size {
                break;
            }
        }
        Ok(offset as usize)
    }

    /// Runs one single-packet split transaction — the start-split (which
    /// the hub's transaction translator must ACK) then complete-split
    /// polling until it has the relayed downstream result. Returns the
    /// bytes moved for this packet.
    ///
    /// The first complete-split poll goes out immediately after the
    /// start-split's ACK; subsequent polls are paced a few microframes
    /// apart ([`CSPLIT_RETRY_DELAY_US`]) to give the translator time to
    /// run the relayed full/low-speed transaction. `XFRC` means done;
    /// `NYET` (translator not finished) and `NAK` (device not ready)
    /// mean poll again, up to [`MAX_CSPLIT_POLLS`] times; any real error
    /// bit ends it.
    fn run_split_packet(
        &self,
        endpoint: ControlEndpoint,
        packet: &Transaction,
        timer: &Timer,
    ) -> Result<usize, TransferError> {
        let requested = packet.transfer_size as usize;

        self.start_channel(endpoint, packet, false, None);
        let hcint = self.wait_for_halt(timer)?;
        if hcint & HCINT_ACK == 0 {
            return self.interpret_halt(hcint, requested);
        }

        let mut hcint = 0;
        for attempt in 0..MAX_CSPLIT_POLLS {
            if attempt > 0 {
                timer.delay_us(CSPLIT_RETRY_DELAY_US);
            }
            self.start_channel(endpoint, packet, true, None);
            hcint = self.wait_for_halt(timer)?;
            if hcint & (HCINT_NYET | HCINT_NAK) != 0 && hcint & HCINT_XFRC == 0 {
                continue;
            }
            return self.interpret_halt(hcint, requested);
        }
        // Exhausted the poll budget -- report the last result (a bare
        // NYET/NAK maps to a retryable error the caller can re-drive).
        self.interpret_halt(hcint, requested)
    }

    /// Runs one single-packet *periodic* split transaction — an
    /// interrupt endpoint's start-/complete-split, which unlike the
    /// non-periodic control split ([`Self::run_split_packet`]) must be
    /// scheduled against the microframe counter (`HFNUM.FRNUM`, which
    /// counts 125µs microframes on this high-speed root port). The
    /// scheduling — which microframe the start-split goes out in, and
    /// the consecutive microframes the complete-splits retry in — mirrors
    /// the periodic frame scheduler of a known-working reference driver
    /// for this SoC (rsta2's `circle`/`uspi`, `dwhciframeschedper.c`):
    /// a fixed complete-split delivered on a fixed 5-microframe cadence
    /// (what [`Self::run_split_packet`] does) never lands in the window
    /// where the transaction translator has the relayed full/low-speed
    /// result, so it only ever reads `NYET` and the transfer never
    /// completes.
    ///
    /// - Start-split: the next microframe after now, skipping microframe
    ///   6 (the reference scheduler avoids scheduling a start-split there,
    ///   since its complete-splits would straddle the 1ms frame boundary).
    ///   Expects `ACK`.
    /// - Complete-split: first at start-frame + 2 microframes, then each
    ///   following microframe, `HCCHAR.ODDFRM` tracking each one's parity.
    ///   `XFRC` completes it; `NYET`/`ACK` means the translator isn't done
    ///   — retry a bounded number of times (matching the reference
    ///   driver's count); `NAK` means the device had nothing to send this
    ///   interval.
    ///
    /// Both an exhausted `NYET` retry budget and a device `NAK` map to
    /// [`TransferError::Nak`]: for an interrupt-IN poll both simply mean
    /// "no report ready this interval", which the caller re-drives on its
    /// next poll — not a real error.
    fn run_periodic_split_packet(
        &self,
        endpoint: ControlEndpoint,
        packet: &Transaction,
        timer: &Timer,
    ) -> Result<usize, TransferError> {
        let requested = packet.transfer_size as usize;

        // Start-split, scheduled for the next microframe (skipping
        // microframe 6, as the reference scheduler does).
        let mut next_frame = (self.current_microframe() + 1) & 7;
        if next_frame == 6 {
            next_frame = 7;
        }
        self.wait_for_microframe(next_frame, timer);
        self.start_channel(endpoint, packet, false, Some(next_frame & 1 == 1));
        let hcint = self.wait_for_halt(timer)?;
        // The transaction translator must ACK a start-split it accepted;
        // anything else (a real error, or a NAK/NYET) ends the attempt.
        if hcint & HCINT_ACK == 0 {
            return self.interpret_halt(hcint, requested);
        }

        // Complete-split: first two microframes on from the start-split,
        // then each consecutive microframe. `tries` bounds the NYET
        // retries the same way the reference scheduler does (one fewer
        // when the start-split landed in microframe 5, whose complete-
        // split window is shorter before the frame boundary).
        let mut tries: i32 = if next_frame != 5 { 3 } else { 2 };
        next_frame = (next_frame + 2) & 7;
        loop {
            self.wait_for_microframe(next_frame, timer);
            self.start_channel(endpoint, packet, true, Some(next_frame & 1 == 1));
            let hcint = self.wait_for_halt(timer)?;

            // A real error bit, or success, is terminal.
            if hcint & (HCINT_STALL | HCINT_TXERR | HCINT_BBERR | HCINT_XFRC) != 0 {
                return self.interpret_halt(hcint, requested);
            }
            // Translator not finished yet — retry in the next microframe
            // until the budget is spent.
            if hcint & (HCINT_NYET | HCINT_ACK) != 0 {
                if tries == 0 {
                    return Err(TransferError::Nak);
                }
                tries -= 1;
                next_frame = (next_frame + 1) & 7;
                continue;
            }
            // Device had nothing to send this interval.
            if hcint & HCINT_NAK != 0 {
                return Err(TransferError::Nak);
            }
            // No recognized bit set — an unexplained halt.
            return self.interpret_halt(hcint, requested);
        }
    }

    /// The current microframe within the 1ms frame (`HFNUM.FRNUM & 7`) —
    /// on this high-speed root port `FRNUM` counts 125µs microframes, so
    /// the low three bits are the microframe index periodic split
    /// scheduling keys off.
    fn current_microframe(&self) -> u8 {
        (self.regs().hfnum().read().frnum().bits() & 7) as u8
    }

    /// Busy-waits until [`Self::current_microframe`] reaches `frame`,
    /// bounded by a timeout so a frame counter that isn't advancing (the
    /// port down, say) can't wedge the caller — the scheduled microframe
    /// is at most 7 away (≈1ms), so a 2ms cap is generous while still
    /// guaranteeing forward progress.
    fn wait_for_microframe(&self, frame: u8, timer: &Timer) {
        let start = timer.now_micros();
        while self.current_microframe() != frame {
            if timer.now_micros() - start > 2_000 {
                break;
            }
        }
    }
}
