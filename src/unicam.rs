//! Blocking driver for the Unicam CSI-2 receiver (camera capture).
//!
//! "Unicam" is the BCM2835/6/7 MIPI CSI-2 receiver. It is a bus master
//! with its own integrated write-to-memory engine: given a destination
//! buffer (start/end address) it receives frames from a CSI-2 camera
//! sensor and writes the pixel data to RAM itself — it does *not* use the
//! [`crate::dma`] controller. This driver brings up Unicam1 (the
//! instance wired to the Pi 3 camera connector) for a two-data-lane RAW10
//! Bayer sensor (e.g. an OV5647) and captures whole frames by polling.
//!
//! The peripheral isn't in `bcm2837-lpa`'s SVD, nor in the public BCM2835
//! ARM Peripherals datasheet — it pokes known physical addresses directly.
//! The register layout, bitfields, and bring-up sequence follow Linux's
//! `bcm2835-unicam` driver (`drivers/media/platform/bcm2835/`), whose own
//! register header traces them to Broadcom's VideoCore4 RDB definitions.
//!
//! # Addresses and cache coherence
//!
//! Like the other bus masters (mailbox, SD, DMA), Unicam bypasses this
//! core's cache: the capture buffer's address is handed to it as a
//! VideoCore bus address (the `0xC000_0000` uncached alias), and this
//! driver cleans the buffer before the capture (so no dirty line writes
//! back over received data) and invalidates it after (so the CPU reads the
//! received frame from RAM, not a stale cached copy) — see
//! [`arm`](crate::unicam::Unicam::arm) and
//! [`wait_frame`](crate::unicam::Unicam::wait_frame). Buffers should be
//! cache-line aligned and padded.
//!
//! # Clock
//!
//! [`Unicam::new`](crate::unicam::Unicam::new) enables the receiver's
//! functional clock (`CM_CAM1` in
//! the clock manager) at ~50 MHz — `plld_per` (~250 MHz on this SoC: PLLD
//! VCO 1000 MHz ÷ 4) divided by 5. This is the exact divider and resulting
//! rate the Linux `bcm2835-unicam` driver runs on this hardware, confirmed
//! by dumping the clock manager while a libcamera capture was streaming.
//! `plld_per`'s rate isn't firmware-guaranteed (the same caveat as
//! `spi`/`i2c`'s core-clock dividers), so a board where it differs would
//! need the divider adjusted to keep this rate.

use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use crate::cache;
use crate::timer::Timer;

/// Power Manager (PM) block base: peripheral base + `0x10_0000`.
const PM_BASE: usize = 0x3f10_0000;
/// PM_CAM1 — the camera-1 analog D-PHY power/LDO control register.
const PM_CAM1: *mut u32 = (PM_BASE + 0x48) as *mut u32;
/// Password OR'd into the top byte of every PM-block write.
const PM_PASSWORD: u32 = 0x5a00_0000;
/// PM_CAM1 control (power-gate) enable — must be set before the LDO.
const PM_CAM1_CTRLEN: u32 = 1 << 0;
/// PM_CAM1 low-power LDO enable.
const PM_CAM1_LDOLPEN: u32 = 1 << 1;
/// PM_CAM1 high-power LDO enable.
const PM_CAM1_LDOHPEN: u32 = 1 << 2;

/// Clock manager (CPRMAN) base: peripheral base + `0x10_1000`.
const CPRMAN_BASE: usize = 0x3f10_1000;
/// CM_CAM1 control register — the Unicam1 functional clock gate/source.
const CM_CAM1CTL: *mut u32 = (CPRMAN_BASE + 0x48) as *mut u32;
/// CM_CAM1 divider register.
const CM_CAM1DIV: *mut u32 = (CPRMAN_BASE + 0x4c) as *mut u32;
/// Password OR'd into the top byte of every clock-manager write; a write
/// without it is ignored.
const CM_PASSWORD: u32 = 0x5a00_0000;
/// CM control: clock enable.
const CM_ENABLE: u32 = 1 << 4;
/// CM control: generator busy/running.
const CM_BUSY: u32 = 1 << 7;
/// CM clock source 6 = `plld_per`, ~250 MHz on this SoC (PLLD VCO 1000 MHz
/// ÷ 4).
const CM_SRC_PLLD_PER: u32 = 6;
/// CM_CAM1 integer divider ÷5 (`5` in bits [23:12] of the 12-fractional-bit
/// field) → ~50 MHz from the 250 MHz parent. Exactly the divider Linux's
/// `bcm2835-unicam` runs on this hardware, and the rate its D-PHY
/// `ths_settle` count is calibrated for.
const CM_CAM1_DIV: u32 = 5 << 12;

/// Unicam1 register block base: peripheral base + `0x80_1000`.
const UNICAM_BASE: usize = 0x3f80_1000;
/// Per-lane clock-gate register — a separate word from the main block,
/// password-gated like the clock manager.
const UNICAM_LANE_GATE: *mut u32 = 0x3f80_2004 as *mut u32;
/// Lane-gate pattern enabling the clock lane + two data lanes (two bits
/// each: `1`, then `<< 2 | 1` per data lane → `0b010101`).
const LANE_GATE_2LANE: u32 = 0x15;

// Register offsets within the block (from vc4-regs-unicam.h).
const CTRL: usize = 0x000;
const STA: usize = 0x004;
const ANA: usize = 0x008;
const PRI: usize = 0x00c;
const CLK: usize = 0x010;
const CLT: usize = 0x014;
const DAT0: usize = 0x018;
const DAT1: usize = 0x01c;
const DAT2: usize = 0x020;
const DAT3: usize = 0x024;
const DLT: usize = 0x028;
const CMP0: usize = 0x02c;
const ICTL: usize = 0x100;
const ISTA: usize = 0x104;
const IDI0: usize = 0x108;
const IPIPE: usize = 0x10c;
const IBSA0: usize = 0x110;
const IBEA0: usize = 0x114;
const IBLS: usize = 0x118;
const IBWP: usize = 0x11c;
const IHWIN: usize = 0x120;
const IVWIN: usize = 0x128;
const MISC: usize = 0x400;

// CTRL bits.
const CTRL_CPE: u32 = 1 << 0;
const CTRL_MEM: u32 = 1 << 1;
const CTRL_CPR: u32 = 1 << 2;
/// CTRL configured for CSI-2 (CPM=0), strobe (DCM=0), packet-framer
/// timeout 0xf (bits 11:8), output-engine timeout 128 (bits 20:12), with
/// the memory-write engine (MEM) enabled.
const CTRL_CONFIG: u32 = CTRL_MEM | (0xf << 8) | (128 << 12);

// STA / ISTA acknowledge masks (write-1-to-clear).
const STA_MASK_ALL: u32 = 0x1_b6fc;
const ISTA_MASK_ALL: u32 = 0x7;
/// STA bits that indicate a corrupted capture (FIFO overflows, ECC/CRC,
/// header overflow) — surfaced so a caller can tell a clean frame from a
/// torn one.
const STA_ERROR_MASK: u32 = (1 << 2) // SBE
    | (1 << 3)  // PBE
    | (1 << 4)  // HOE
    | (1 << 7)  // CRCE
    | (1 << 9)  // IFO
    | (1 << 10); // OFO

// ANA (analog/D-PHY) bits.
const ANA_AR: u32 = 1 << 2;
/// ANA during PHY reset: analog reset asserted, CTAT/PTAT adjust = 7 each.
const ANA_SETUP: u32 = ANA_AR | (7 << 4) | (7 << 8);
/// ANA running: reset deasserted, adjusts held, lane control enabled
/// (DDL = 0).
const ANA_RUN: u32 = (7 << 4) | (7 << 8);

/// PRI (AXI QoS): PE=1, PT=2, NP=8, PP=0xe, BS=0, BL=0.
const PRI_CONFIG: u32 = 1 | (2 << 1) | (8 << 4) | (0xe << 8);

/// CLK lane: enable + low-power enable (non-continuous clock).
const CLK_CONFIG: u32 = (1 << 0) | (1 << 2);
/// DATn lane: enable + low-power enable.
const DAT_CONFIG: u32 = (1 << 0) | (1 << 2);

/// CLT clock-lane timing: term-enable 2, settle 6.
const CLT_CONFIG: u32 = 2 | (6 << 8);
/// DLT data-lane timing: term-enable 2, settle 6, rx-enable 0.
const DLT_CONFIG: u32 = 2 | (6 << 8);

/// CMP0 packet-compare: enable (PCE), GI, compare-header (CPH), virtual
/// channel 0, data type 1 (CSI-2 Frame End) — makes frame-end detection
/// reliable via STA.PI0.
const CMP0_CONFIG: u32 = (1 << 31) | (1 << 9) | (1 << 8) | 1;

/// IPIPE: no unpack/repack (PUM=NONE, PPM=NONE) — store the sensor's native
/// packed RAW10 (4 pixels per 5 bytes), matching what Linux's driver
/// programs. (Unpacking to 16bpp is a possible later feature, but capture
/// was brought up against this packed layout.)
const IPIPE_PACKED: u32 = 0;
/// IDI0 image data id: virtual channel 0, CSI-2 data type 0x2b (RAW10).
const IDI0_RAW10: u32 = 0x2b;
/// MISC frame-length overrides FL0 | FL1.
const MISC_CONFIG: u32 = (1 << 6) | (1 << 9);

// ICTL bits.
const ICTL_FSIE: u32 = 1 << 0;
const ICTL_FEIE: u32 = 1 << 1;
const ICTL_IBOB: u32 = 1 << 2;
/// LIP field (bits 6:5): value 1 latches the image buffer pointers.
const ICTL_LIP: u32 = 1 << 5;

/// STA packet-compare-0 match — set at each CSI-2 frame-end short packet
/// (the reason `CMP0` is programmed). The per-frame "done" signal.
const STA_PI0: u32 = 1 << 15;

/// The receiver's image-line-buffer byte alignment (`BPL_ALIGNMENT` in the
/// Linux driver).
const LINE_ALIGNMENT: usize = 32;

/// Packed-RAW10 line stride for `width` pixels: 10 bits per pixel converted
/// to bytes, rounded up to [`LINE_ALIGNMENT`] — the value programmed into
/// `IBLS` and used to compute the buffer size.
fn packed_raw10_stride(width: u32) -> usize {
    let bytes = (width as usize * 10).div_ceil(8);
    bytes.next_multiple_of(LINE_ALIGNMENT)
}

/// Outcome of a [`Unicam::wait_frame`] call.
#[derive(Debug, Clone, Copy)]
pub struct CaptureResult {
    /// Number of image lines the receiver had written when capture ended
    /// (from the hardware write pointer). Equals the frame height on a
    /// complete capture.
    pub lines_captured: u32,
    /// Union of every `UNICAM_STA` value seen while polling — check against
    /// the FIFO-overflow/ECC bits (see [`Self::had_error`]) to tell a clean
    /// frame from a torn one.
    pub status: u32,
    /// Union of every `UNICAM_ISTA` (frame start/end/line-count) value seen
    /// while polling.
    pub image_status: u32,
    /// Whether the poll hit its timeout before a full frame arrived.
    pub timed_out: bool,
}

impl CaptureResult {
    /// True if the status word flagged a FIFO overflow, ECC, CRC, or header
    /// error during the capture — the received frame should be treated as
    /// unreliable.
    pub fn had_error(&self) -> bool {
        self.status & STA_ERROR_MASK != 0
    }
}

/// Book-keeping stored between [`Unicam::arm`] and [`Unicam::wait_frame`]:
/// where the frame is landing, so the wait can compute progress and do the
/// post-capture cache maintenance without re-borrowing the buffer.
#[derive(Clone, Copy)]
struct Armed {
    /// ARM-physical base of the capture buffer.
    phys: u32,
    /// Buffer size in bytes.
    size: usize,
    /// Bytes per image line.
    stride: usize,
    /// Bus (0xC000_0000-alias) address of the buffer start — what the write
    /// pointer counts up from.
    bus_start: u32,
    /// Bus address of the buffer end.
    bus_end: u32,
    /// Frame height in lines.
    height: u32,
}

/// The Unicam1 CSI-2 receiver.
///
/// There is no PAC token to own (the peripheral isn't in `bcm2837-lpa`), so
/// [`new`](Unicam::new) is safe and constructs from nothing, like
/// [`rng::Rng`](crate::rng::Rng); a caller wanting exclusive use holds the
/// single `Unicam` itself.
///
/// A capture is two steps — [`arm`](Unicam::arm) then
/// [`wait_frame`](Unicam::wait_frame) — so the sensor can be started
/// *between* them: the D-PHY must be armed and idle when the sensor begins
/// transmitting, or it won't lock onto the first frame.
pub struct Unicam {
    /// `Some` once [`arm`](Unicam::arm) has configured a capture, taken by
    /// [`wait_frame`](Unicam::wait_frame).
    armed: Option<Armed>,
}

impl Unicam {
    /// Powers the camera-1 analog D-PHY (the `PM_CAM1` LDO in the power
    /// manager) and enables the Unicam1 functional clock (`CM_CAM1`, ~50 MHz
    /// from `plld_per`), then returns a handle. Does not yet touch the
    /// receiver registers or start a capture.
    pub fn new() -> Self {
        unsafe {
            // Power up the camera-1 analog D-PHY (PM_CAM1): enable the
            // power gate, then the LDO regulator. Without this the digital
            // side works (MMIO, lane-state detection) but no high-speed
            // data is recovered — this is the analog rail the Linux stack
            // relies on the firmware to have powered, and it's not set on a
            // bare-metal boot with no camera stack running. Gate before LDO,
            // per the PM block's ordering; the analog block's settle time is
            // covered by the clock bring-up below and `arm`'s PHY delay.
            write_volatile(PM_CAM1, PM_PASSWORD | PM_CAM1_CTRLEN);
            write_volatile(
                PM_CAM1,
                PM_PASSWORD | PM_CAM1_CTRLEN | PM_CAM1_LDOLPEN | PM_CAM1_LDOHPEN,
            );

            // Gate the clock and wait for the generator to stop before
            // reprogramming source/divider.
            write_volatile(CM_CAM1CTL, CM_PASSWORD);
            while read_volatile(CM_CAM1CTL) & CM_BUSY != 0 {
                spin_loop();
            }
            write_volatile(CM_CAM1DIV, CM_PASSWORD | CM_CAM1_DIV);
            write_volatile(CM_CAM1CTL, CM_PASSWORD | CM_SRC_PLLD_PER);
            write_volatile(CM_CAM1CTL, CM_PASSWORD | CM_SRC_PLLD_PER | CM_ENABLE);
            while read_volatile(CM_CAM1CTL) & CM_BUSY == 0 {
                spin_loop();
            }
        }
        Self { armed: None }
    }

    /// Minimum buffer size, in bytes, that [`arm`](Self::arm) needs for a
    /// `width`×`height` packed-RAW10 frame (line stride × height).
    pub fn packed_raw10_size(width: u32, height: u32) -> usize {
        packed_raw10_stride(width) * height as usize
    }

    /// Configures and arms the receiver for one `width`×`height` RAW10
    /// frame, stored as native packed RAW10 (4 pixels per 5 bytes), into
    /// `buffer`, then leaves it waiting for the sensor's stream.
    ///
    /// Call this **before** starting the sensor streaming, and
    /// [`wait_frame`](Unicam::wait_frame) after: the D-PHY has to be armed
    /// and idle when the sensor begins transmitting so it locks onto the
    /// first frame — arming against an already-running stream is what makes
    /// the receiver never see a frame start.
    ///
    /// `buffer` must hold at least [`packed_raw10_size`](Self::packed_raw10_size)
    /// bytes and should be cache-line aligned/padded (see the module docs on
    /// coherence). `timer` provides the analog block's settle delay.
    ///
    /// # Panics
    ///
    /// Panics if `buffer` is too small for the packed frame.
    pub fn arm(&mut self, buffer: &mut [u8], width: u32, height: u32, timer: &Timer) {
        let stride = packed_raw10_stride(width);
        let size = stride * height as usize;
        assert!(buffer.len() >= size, "capture buffer too small");

        let phys = buffer.as_mut_ptr() as usize as u32;
        let bus_start = to_bus(phys);
        let bus_end = to_bus(phys + size as u32);

        // Flush the buffer so no dirty cache line writes back over the
        // frame the receiver is about to write in.
        cache::clean_range(phys, size);

        let line_int_freq = (height >> 2).max(128);

        unsafe {
            // Enable the lane clocks (clock lane + 2 data lanes).
            write_volatile(UNICAM_LANE_GATE, CM_PASSWORD | LANE_GATE_2LANE);

            reg_write(CTRL, CTRL_MEM);

            // Bring up the analog/D-PHY block: assert reset with the
            // trim adjusts, settle, then release reset.
            reg_write(ANA, ANA_SETUP);
            timer.delay_ms(2);
            reg_write(ANA, ANA_RUN);

            // Pulse the peripheral reset, leave it disabled.
            reg_write(CTRL, CTRL_MEM | CTRL_CPR);
            reg_write(CTRL, CTRL_MEM);
            reg_write(CTRL, CTRL_CONFIG);

            // No hardware cropping window.
            reg_write(IHWIN, 0);
            reg_write(IVWIN, 0);

            reg_write(PRI, PRI_CONFIG);

            // Interrupt/line-count control (polled, but the status bits it
            // gates are still what we watch), then clear stale status.
            reg_write(
                ICTL,
                ICTL_FSIE | ICTL_FEIE | ICTL_IBOB | (line_int_freq << 16),
            );
            reg_write(STA, STA_MASK_ALL);
            reg_write(ISTA, ISTA_MASK_ALL);

            // D-PHY lane timing.
            reg_write(CLT, CLT_CONFIG);
            reg_write(DLT, DLT_CONFIG);

            // Match the CSI-2 frame-end short packet for reliable
            // frame-boundary detection.
            reg_write(CMP0, CMP0_CONFIG);

            // Enable the clock lane and the two data lanes; the unused
            // lanes stay disabled.
            reg_write(CLK, CLK_CONFIG);
            reg_write(DAT0, DAT_CONFIG);
            reg_write(DAT1, DAT_CONFIG);
            reg_write(DAT2, 0);
            reg_write(DAT3, 0);

            // Destination buffer, packing, and image data type.
            reg_write(IBLS, stride as u32);
            reg_write(IBSA0, bus_start);
            reg_write(IBEA0, bus_end);
            reg_write(IPIPE, IPIPE_PACKED);
            reg_write(IDI0, IDI0_RAW10);

            let misc = reg_read(MISC) | MISC_CONFIG;
            reg_write(MISC, misc);

            // Enable the peripheral. The image-buffer pointers are latched
            // by wait_frame, which re-latches to capture each successive
            // frame without reconfiguring the D-PHY.
            reg_write(CTRL, CTRL_CONFIG | CTRL_CPE);
        }

        self.armed = Some(Armed {
            phys,
            size,
            stride,
            bus_start,
            bus_end,
            height,
        });
    }

    /// Captures the next frame into the armed buffer, blocking until it
    /// lands, `timeout_ms` elapses, or a hardware error latches, then does
    /// the post-capture cache invalidate and returns what happened.
    ///
    /// Reusable: it re-latches the image-buffer pointers each call to
    /// capture successive frames *without* reconfiguring or resetting the
    /// D-PHY (the receiver stays enabled between calls), so a preview loop
    /// just calls this repeatedly after a single [`arm`](Unicam::arm).
    /// Completion is the CSI-2 frame-end packet (`STA.PI0`, via the `CMP0`
    /// match), falling back to the write pointer reaching the full height.
    /// The [`CaptureResult`]'s status fields accumulate every bit seen
    /// across the poll.
    ///
    /// # Panics
    ///
    /// Panics if called before [`arm`](Unicam::arm).
    pub fn wait_frame(&mut self, timer: &Timer, timeout_ms: u32) -> CaptureResult {
        let armed = self.armed.expect("wait_frame called before arm");

        // Clear the previous frame's status and re-latch the image-buffer
        // pointers to begin capturing a fresh frame from the buffer start.
        unsafe {
            reg_write(STA, STA_MASK_ALL);
            reg_write(ISTA, ISTA_MASK_ALL);
            let ictl = reg_read(ICTL) | ICTL_LIP;
            reg_write(ICTL, ictl);
        }

        let deadline = timer.now_micros().wrapping_add(timeout_ms as u64 * 1000);
        let mut timed_out = false;
        let mut status = 0;
        let mut image_status = 0;
        loop {
            let sta = unsafe { reg_read(STA) };
            status |= sta;
            image_status |= unsafe { reg_read(ISTA) };
            let done = sta & STA_PI0 != 0
                || current_lines(armed.bus_start, armed.bus_end, armed.stride) >= armed.height;
            if done || sta & STA_ERROR_MASK != 0 {
                break;
            }
            if timer.now_micros() >= deadline {
                timed_out = true;
                break;
            }
            spin_loop();
        }

        let lines_captured = current_lines(armed.bus_start, armed.bus_end, armed.stride);

        // The receiver wrote the buffer in RAM behind this core's cache;
        // drop the stale lines so the frame reads back from RAM.
        cache::invalidate_range(armed.phys, armed.size);

        CaptureResult {
            lines_captured,
            status,
            image_status,
            timed_out,
        }
    }
}

/// Image lines the receiver has written so far, from the write pointer.
/// Guards against a pointer that hasn't been loaded/started yet: a value
/// outside the buffer range (e.g. a stale 0) reads as 0 lines rather than a
/// wrapped-around huge count.
fn current_lines(bus_start: u32, bus_end: u32, stride: usize) -> u32 {
    let written = unsafe { reg_read(IBWP) };
    if written >= bus_start && written <= bus_end {
        ((written - bus_start) as usize / stride) as u32
    } else {
        0
    }
}

impl Default for Unicam {
    /// Equivalent to [`Unicam::new`].
    fn default() -> Self {
        Self::new()
    }
}

/// Reads a Unicam register at `offset` within the block.
unsafe fn reg_read(offset: usize) -> u32 {
    read_volatile((UNICAM_BASE + offset) as *const u32)
}

/// Writes `value` to the Unicam register at `offset` within the block.
unsafe fn reg_write(offset: usize, value: u32) {
    write_volatile((UNICAM_BASE + offset) as *mut u32, value);
}

/// Translates a plain ARM physical address to the VideoCore `0xC000_0000`
/// "direct, uncached" bus alias the receiver writes through — the same
/// window [`mailbox`](crate::mailbox) and [`dma`](crate::dma) use.
fn to_bus(physical_address: u32) -> u32 {
    (physical_address & 0x3fff_ffff) | 0xc000_0000
}
