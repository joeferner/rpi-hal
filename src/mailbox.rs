//! Blocking driver for the VideoCore mailbox property interface — the
//! RPC channel to the GPU firmware for anything the ARM core can't
//! learn or do on its own: real clock rates, board identification, the
//! ARM/VideoCore memory split, and (see
//! [`allocate_framebuffer`](crate::mailbox::Mailbox::allocate_framebuffer))
//! a display framebuffer. Display
//! output itself (HDMI or the MIPI DSI touchscreen) is entirely
//! mediated by the VideoCore the same way: there's no ARM-side HDMI or
//! DSI PHY to program directly, only this mailbox request for a buffer
//! to write pixels into. Which physical output that buffer appears on
//! is decided by firmware/`config.txt` configuration, not by anything
//! this driver chooses.
//!
//! ## Wire format
//!
//! A call writes one 32-bit "message" to mailbox 1: the request
//! buffer's address (which must therefore be 16-byte aligned — its
//! low 4 bits double as the channel number) with the channel (8, for
//! property tags) or-ed into those low bits. The firmware processes
//! the buffer in place and writes the same message (echoing the
//! address) back to mailbox 0 once done.
//!
//! The buffer itself is `[size, request/response code, tag id, tag
//! value size, tag request/response code, ...tag value words...,
//! 0]` — this driver only ever builds single-tag requests, which is
//! all any of the queries below need.
//!
//! ## Cache coherency
//!
//! This crate's MMU setup (`mmu.rs`) maps RAM Cacheable, but the
//! VideoCore reads/writes the request buffer over the bus, entirely
//! outside the ARM core's cache — so without explicit maintenance,
//! the firmware could read a stale, not-yet-written-back request, or
//! this driver could read back the stale pre-call contents of the
//! response instead of what the firmware wrote. `clean_range` runs
//! before handing the buffer to the firmware, `invalidate_range`
//! after, so this is handled explicitly rather than assumed away.
//!
//! Separately, the buffer's address is passed to the firmware
//! translated to the `0xC000_0000` "direct, uncached" VideoCore bus
//! alias (see `to_vc_bus_address`) rather than its plain physical
//! address — that's what keeps the *VideoCore's own* L2 cache out of
//! the picture, a second cache this driver has no maintenance
//! operations for at all. Both pieces are needed: one for the ARM
//! core's cache, one for the VideoCore's.

use crate::cache::{clean_range, invalidate_range};
use crate::pac::VCMAILBOX;

/// Mailbox channel used for every call in this module — the
/// VideoCore "property tags" RPC channel. Other channels exist
/// (framebuffer alloc/release on older firmware, power management),
/// but every query this driver makes goes through this one.
const CHANNEL_PROPERTY_TAGS: u32 = 8;

/// Largest tag value area (in words) any query in this module needs —
/// currently 3, for "Set Clock Rate"'s (clock id, rate, skip-turbo)
/// request; most tags need 2 (a 64-bit board serial, a memory base+size
/// pair, a clock id and its rate). Bump this if a wider tag is ever added.
///
/// The value area is sized at this maximum for *every* call, not at the
/// individual tag's length. That costs a few words of stack and is what
/// the firmware expects — the tag's response code reports how much it
/// actually wrote — but it means an over-long call would silently overrun
/// this buffer, so [`Mailbox::property_call`] checks each call against it.
const MAX_VALUE_WORDS: usize = 3;

/// Total property buffer size in words: the 2-word message header
/// (buffer size, request/response code), one 3-word tag header (tag
/// id, value size, tag request/response code), the value area, and
/// the 1-word end tag.
const BUFFER_WORDS: usize = 2 + 3 + MAX_VALUE_WORDS + 1;

/// Set on the message header's/tag's response code once the firmware
/// has actually answered — the low 31 bits then carry (message
/// header) nothing meaningful, or (tag) the real response length in
/// bytes.
const RESPONSE_BIT: u32 = 1 << 31;

/// Errors from a [`Mailbox`] property-interface call.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The response's echoed buffer address didn't match the request
    /// the buffer was actually built for — would mean memory
    /// corruption, since this driver only ever has one call in flight
    /// at a time.
    AddressMismatch,
    /// The message header's response code didn't have its high bit
    /// set — the firmware rejected the whole request.
    RequestFailed,
    /// The tag's own response code didn't have its high bit set — the
    /// firmware didn't recognize or didn't answer this tag.
    TagNotAnswered,
    /// A tag asked for more request or response words than the shared
    /// value area holds ([`MAX_VALUE_WORDS`](self) — a private constant,
    /// since it is this module's own buffer, not the caller's). A bug in
    /// this driver rather than anything the hardware did: the fix is to
    /// widen that buffer for the tag that needs it.
    ValueTooLarge,
}

/// Identifies which SoC clock a [`Mailbox::clock_rate_hz`] query is
/// about.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum ClockId {
    /// EMMC/SD card controller clock.
    Emmc = 1,
    /// UART reference clock — already a fixed, firmware-guaranteed
    /// value this crate doesn't need to query (see `uart.rs`'s
    /// `init`); listed here for completeness against the firmware's
    /// own clock id list, not because any driver needs it.
    Uart = 2,
    /// ARM core (CPU) clock.
    Arm = 3,
    /// SoC "core" clock — what `spi.rs`'s and `i2c.rs`'s
    /// `clock_divider` parameters would be computed against, per
    /// their doc comments, if a caller wants an exact target
    /// frequency instead of a raw divider passthrough.
    Core = 4,
    /// 3D block (VideoCore GPU) clock.
    V3d = 5,
    /// H264 hardware video decoder clock.
    H264 = 6,
    /// Image Sensor Pipeline (camera) clock.
    Isp = 7,
    /// SDRAM clock.
    Sdram = 8,
    /// HDMI pixel clock.
    Pixel = 9,
    /// PWM clock.
    Pwm = 10,
    /// BCM2711 (Pi 4) EMMC2 controller clock — distinct from
    /// [`ClockId::Emmc`], which on that chip feeds a different controller (see
    /// `sd.rs`'s BCM2711 doc notes). Confirmed against the firmware's
    /// own mailbox property interface documentation, not assumed. Not
    /// meaningful on BCM2836/2837, which has no EMMC2 block at all.
    #[cfg(feature = "bcm2711")]
    Emmc2 = 12,
}

// Bits of [`Mailbox::throttled`]'s status word. Bits 0-3 report the
// board's state right now; the same conditions repeat at bits 16-19 as
// sticky "this has happened since boot" flags, which is where a dip that
// has already passed shows up.
/// Supply voltage is below the firmware's threshold *now*.
pub const THROTTLED_UNDER_VOLTAGE: u32 = 1 << 0;
/// The ARM clock is currently capped below its maximum.
pub const THROTTLED_ARM_FREQ_CAPPED: u32 = 1 << 1;
/// The board is currently being throttled.
pub const THROTTLED_NOW: u32 = 1 << 2;
/// The soft temperature limit is currently active.
pub const THROTTLED_SOFT_TEMP_LIMIT: u32 = 1 << 3;
/// Under-voltage has occurred since boot.
pub const THROTTLED_UNDER_VOLTAGE_EVER: u32 = 1 << 16;
/// The ARM clock has been capped since boot.
pub const THROTTLED_ARM_FREQ_CAPPED_EVER: u32 = 1 << 17;
/// Throttling has occurred since boot.
pub const THROTTLED_EVER: u32 = 1 << 18;
/// The soft temperature limit has been hit since boot.
pub const THROTTLED_SOFT_TEMP_LIMIT_EVER: u32 = 1 << 19;

/// The VideoCore GPIO-expander line carrying the on-board wireless
/// chip's `WL_ON` enable, for [`Mailbox::set_expander_gpio`]: expander
/// line 1, plus the firmware's expander base of 128. Driving it high
/// powers the radio up on a Pi 3 (both `B` and `B+`), which the SoC has
/// no direct ARM GPIO to reach.
pub const EXPANDER_WL_ON: u32 = 129;

/// The VideoCore GPIO-expander line carrying the on-board Bluetooth
/// controller's `BT_ON` enable, for [`Mailbox::set_expander_gpio`]:
/// expander line 0, plus the firmware's expander base of 128. Driving it
/// high releases the BCM43438's Bluetooth core from reset on a Pi 3
/// (device tree `bluetooth { shutdown-gpios = <&expgpio 0 …> }`), the
/// sibling of [`EXPANDER_WL_ON`]'s Wi-Fi enable; as with `WL_ON`, the SoC
/// has no direct ARM GPIO to reach it.
pub const EXPANDER_BT_ON: u32 = 128;

/// The VideoCore GPIO-expander line enabling the CSI camera connector's
/// power/regulator (`CAM_GPIO0`), for [`Mailbox::set_expander_gpio`]:
/// expander line 5, plus the firmware's expander base of 128. On a Pi 3
/// the camera module's supply is gated by this firmware line (device tree
/// `cam1_reg { gpio = <&expgpio 5 …> }`); it must be driven high before an
/// OV5647/IMX219 will respond on its I2C control bus. The SoC has no direct
/// ARM GPIO to it — this firmware call is the only way to power it.
pub const EXPANDER_CAM_GPIO0: u32 = 133;

/// The VideoCore GPIO-expander line driving the CSI camera connector's LED
/// (`CAM_GPIO1`), for [`Mailbox::set_expander_gpio`]: expander line 6, plus
/// the firmware's expander base of 128. Cosmetic — the activity LED on the
/// camera board — and independent of [`EXPANDER_CAM_GPIO0`]'s power line.
pub const EXPANDER_CAM_GPIO1: u32 = 134;

/// The VideoCore firmware power-domain id for the Unicam1 CSI-2 receiver,
/// for [`Mailbox::set_power_domain`]. This is the analog/PHY power island
/// the receiver needs to recover high-speed data — separate from register
/// (MMIO) access, which stays alive regardless — so it must be powered on
/// before [`crate::unicam`] can receive frames.
pub const POWER_DOMAIN_UNICAM1: u32 = 13;

/// A hardware block the firmware can power on or off via
/// [`Mailbox::set_power_state`] (tag `0x0002_8001`). Values are the
/// firmware's own device ids.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum PowerDeviceId {
    /// SD card controller.
    SdCard = 0,
    /// UART0 (PL011).
    Uart0 = 1,
    /// UART1 (mini-UART).
    Uart1 = 2,
    /// USB HCD — the DWC2 host controller. On this board the firmware
    /// may hand USB off only partially powered (enough for register
    /// access, not full operation), so a bare-metal host driver should
    /// power this on explicitly before bringing the controller up.
    UsbHcd = 3,
    /// I2C0 (BSC0).
    I2c0 = 4,
    /// I2C1 (BSC1).
    I2c1 = 5,
    /// I2C2 (BSC2).
    I2c2 = 6,
    /// SPI0.
    Spi = 7,
    /// CCP2TX.
    Ccp2tx = 8,
}

/// A contiguous memory range as reported by [`Mailbox::arm_memory`] /
/// [`Mailbox::vc_memory`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MemoryRegion {
    /// Physical base address of the region.
    pub base_address: u32,
    /// Size of the region, in bytes.
    pub size_bytes: u32,
}

/// Pixel channel order for a framebuffer, passed to
/// [`Mailbox::allocate_framebuffer`] (tag `0x0004_8006`, "Set Pixel
/// Order").
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum PixelOrder {
    /// Blue, green, red byte order.
    Bgr = 0,
    /// Red, green, blue byte order.
    Rgb = 1,
}

/// A framebuffer allocated by [`Mailbox::allocate_framebuffer`]: where
/// it lives in memory and how its pixels are laid out.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Framebuffer {
    /// ARM-side physical address of the first pixel. Identity-mapped by
    /// this crate's MMU setup (`mmu.rs`), so this doubles as a virtual
    /// address a caller can cast straight to a pointer and write
    /// through.
    pub address: u32,
    /// Total size of the buffer, in bytes — the whole virtual buffer,
    /// so with [`Mailbox::allocate_framebuffer_paged`] this covers
    /// every page, not just the one on screen.
    pub size_bytes: u32,
    /// Bytes from the start of one row to the start of the next. Not
    /// necessarily `width * bytes_per_pixel` — the firmware may pad
    /// each row wider than the requested width — so callers must
    /// stride writes by this, not a value they compute themselves.
    pub pitch_bytes: u32,
    /// Width, in pixels, as actually allocated (matches the request).
    pub width: u32,
    /// Height of the *displayed* area, in pixels, as actually
    /// allocated (matches the request).
    pub height: u32,
    /// Height of the whole allocated buffer, in pixels. Equal to
    /// [`height`](Self::height) for a plain
    /// [`Mailbox::allocate_framebuffer`]; `pages * height` for
    /// [`Mailbox::allocate_framebuffer_paged`], where the rows past the
    /// first `height` are off-screen until
    /// [`Mailbox::set_virtual_offset`] scrolls to them.
    pub virtual_height: u32,
    /// Bits per pixel, as actually allocated (matches the request).
    pub depth_bits: u32,
}

impl Framebuffer {
    /// How many [`height`](Self::height)-tall pages this buffer holds —
    /// what [`Mailbox::allocate_framebuffer_paged`] was asked for, and
    /// 1 for a plain [`Mailbox::allocate_framebuffer`].
    pub fn pages(&self) -> u32 {
        self.virtual_height / self.height
    }

    /// Byte offset from [`address`](Self::address) of page `page`'s
    /// first pixel — where to start writing the frame that
    /// [`Mailbox::set_virtual_offset`] will later bring on screen.
    pub fn page_offset_bytes(&self, page: u32) -> u32 {
        page * self.height * self.pitch_bytes
    }

    /// Writes back every cache line covering this framebuffer, making
    /// pixels the ARM core just wrote actually visible to the display
    /// hardware. The VideoCore's scanout reads this memory over the
    /// bus, entirely outside the ARM core's cache — the same reasoning
    /// as this module's mailbox call buffer (see the module doc
    /// comment) — so call this after writing a frame and before
    /// expecting it on screen.
    ///
    /// This covers the whole allocation. A paged buffer draws one page
    /// at a time, and cleaning the pages nothing was written to is
    /// wasted memory traffic — [`flush_page`](Self::flush_page) is the
    /// one to call there.
    pub fn flush(&self) {
        clean_range(self.address, self.size_bytes as usize);
    }

    /// Writes back only the cache lines covering page `page` — the
    /// per-frame flush for a buffer from
    /// [`Mailbox::allocate_framebuffer_paged`], and equivalent to
    /// [`flush`](Self::flush) for a single-page one.
    ///
    /// A `page` past the end of the allocation flushes nothing rather
    /// than cleaning memory that isn't the framebuffer's.
    pub fn flush_page(&self, page: u32) {
        if page >= self.pages() {
            return;
        }
        clean_range(
            self.address + self.page_offset_bytes(page),
            (self.height * self.pitch_bytes) as usize,
        );
    }
}

/// Blocking driver for the VideoCore mailbox property interface.
pub struct Mailbox {
    vcmailbox: VCMAILBOX,
}

impl Mailbox {
    /// Wraps the mailbox peripheral. Needs no setup — unlike every
    /// other peripheral this crate drives, there's no GPIO routing or
    /// clock divider to configure; the mailbox is always live.
    pub fn new(vcmailbox: VCMAILBOX) -> Self {
        Self { vcmailbox }
    }

    /// The running firmware's revision number, straight from tag
    /// `0x0000_0001` with no further interpretation.
    pub fn firmware_revision(&mut self) -> Result<u32, Error> {
        let response = self.property_call(0x0000_0001, &[], 1)?;
        Ok(response[0])
    }

    /// This board's revision code (tag `0x0001_0002`) — the same
    /// packed value `/proc/cpuinfo`'s `Revision` field reports under
    /// Linux, still undecoded here (board type/RAM size/manufacturer
    /// live in different bit ranges of it, and nothing in this crate
    /// needs them broken out yet).
    pub fn board_revision(&mut self) -> Result<u32, Error> {
        let response = self.property_call(0x0001_0002, &[], 1)?;
        Ok(response[0])
    }

    /// This board's 64-bit serial number (tag `0x0001_0004`), low word
    /// first as the firmware returns it.
    pub fn board_serial(&mut self) -> Result<u64, Error> {
        let response = self.property_call(0x0001_0004, &[], 2)?;
        Ok((response[0] as u64) | ((response[1] as u64) << 32))
    }

    /// This board's Ethernet MAC address (tag `0x0001_0003`), in
    /// transmission byte order. On this board the MAC lives only in the
    /// VideoCore firmware (the on-board LAN9514 USB-Ethernet controller
    /// has no EEPROM of its own), so this tag is the authoritative source
    /// — a driver bringing the LAN9514 up must read it here and program
    /// it into the chip's address registers, since the chip powers up
    /// without one.
    pub fn mac_address(&mut self) -> Result<[u8; 6], Error> {
        // Six bytes across two response words, low word first: word 0 is
        // MAC bytes 0..4, word 1's low half is bytes 4..6.
        let response = self.property_call(0x0001_0003, &[], 2)?;
        let low = response[0].to_le_bytes();
        let high = response[1].to_le_bytes();
        Ok([low[0], low[1], low[2], low[3], high[0], high[1]])
    }

    /// The RAM range reserved for the ARM core (tag `0x0001_0005`) —
    /// the complement of [`Self::vc_memory`]; together they're the
    /// "memory split" set at boot (`config.txt`'s `gpu_mem`).
    pub fn arm_memory(&mut self) -> Result<MemoryRegion, Error> {
        self.memory_region(0x0001_0005)
    }

    /// The RAM range reserved for the VideoCore GPU (tag
    /// `0x0001_0006`) — see [`Self::arm_memory`].
    pub fn vc_memory(&mut self) -> Result<MemoryRegion, Error> {
        self.memory_region(0x0001_0006)
    }

    /// The real, current rate of `clock`, in Hz (tag `0x0003_0002`,
    /// "Get Clock Rate") — what a caller needing an exact frequency
    /// (rather than `spi.rs`'s/`i2c.rs`'s raw divider passthrough)
    /// would compute a clock divider against, instead of assuming a
    /// nominal value that may not match this specific board's
    /// configuration.
    pub fn clock_rate_hz(&mut self, clock: ClockId) -> Result<u32, Error> {
        let response = self.property_call(0x0003_0002, &[clock as u32], 2)?;
        Ok(response[1])
    }

    /// The highest rate the firmware will run `clock` at, in Hz (tag
    /// `0x0003_0004`, "Get Max Clock Rate") — the board's configured
    /// maximum (`arm_freq` in `config.txt`), not necessarily what it is
    /// running now. Compare with [`Self::clock_rate_hz`].
    pub fn max_clock_rate_hz(&mut self, clock: ClockId) -> Result<u32, Error> {
        let response = self.property_call(0x0003_0004, &[clock as u32], 2)?;
        Ok(response[1])
    }

    /// Asks the firmware to run `clock` at `rate_hz` (tag `0x0003_8002`,
    /// "Set Clock Rate"), returning the rate it actually set — which is
    /// clamped to the firmware's own minimum and maximum, so passing
    /// [`Self::max_clock_rate_hz`] is the way to ask for "as fast as this
    /// board goes".
    ///
    /// **A bare-metal program that wants full speed has to do this.** The
    /// firmware brings the ARM core up at its *minimum* rate — 600 MHz on a
    /// Pi 3 whose maximum is 1200 — and something has to ask for more.
    /// Under Linux that something is the cpufreq governor, reacting to
    /// load through this same tag; with no OS there is no governor, so the
    /// clock simply stays at the floor and every measurement comes out half
    /// what the hardware can do, with nothing reported as wrong (see
    /// [`Self::throttled`], which stays clear — the board isn't being held
    /// back, it was never asked to go faster).
    pub fn set_clock_rate_hz(&mut self, clock: ClockId, rate_hz: u32) -> Result<u32, Error> {
        // Third word is `skip_setting_turbo`: 0 lets the firmware move the
        // other turbo-mode clocks (core, SDRAM) along with the ARM clock,
        // which is what makes the higher ARM rate worth having.
        let response = self.property_call(0x0003_8002, &[clock as u32, rate_hz, 0], 3)?;
        Ok(response[1])
    }

    /// Turns `clock` on (`on = true`) or off (tag `0x0003_8001`, "Set
    /// Clock State"), returning whether the firmware reports the clock
    /// exists afterwards.
    ///
    /// Distinct from [`Self::set_clock_rate_hz`]: rate selects *how
    /// fast* an already-running clock goes, this selects whether it
    /// runs at all. Every clock this crate has driven so far comes up
    /// already enabled by the boot firmware, since something upstream
    /// (the firmware itself, or Linux on another boot) already needed
    /// it — `ClockId::Emmc2` (BCM2711/Pi 4) is the first one this
    /// crate reaches that firmware may never have turned on for its own
    /// purposes.
    pub fn set_clock_state(&mut self, clock: ClockId, on: bool) -> Result<bool, Error> {
        let response = self.property_call(0x0003_8001, &[clock as u32, on as u32], 2)?;
        // Response bit 1: 0 = exists, 1 = does not exist.
        Ok(response[1] & 0b10 == 0)
    }

    /// The SoC's temperature, in thousandths of a degree Celsius (tag
    /// `0x0003_0006`, "Get Temperature") — `58_000` is 58°C.
    ///
    /// Read alongside [`Self::throttled`] when a workload that was keeping
    /// up stops doing so: the firmware caps the ARM clock as the die heats
    /// up, and a program that only watches its own frame times sees that as
    /// its code inexplicably getting slower.
    pub fn temperature_millicelsius(&mut self) -> Result<u32, Error> {
        // Sensor id 0 -- the only one this SoC has.
        let response = self.property_call(0x0003_0006, &[0], 2)?;
        Ok(response[1])
    }

    /// The firmware's throttling status word (tag `0x0003_0046`, "Get
    /// Throttled"), reporting both what is happening *now* and what has
    /// happened since boot. Test it with the `THROTTLED_*` constants.
    ///
    /// The two halves matter for different questions: the low bits say
    /// whether the board is being held back at this moment, and the sticky
    /// bits (16 and up) say whether it ever was — which is the only way to
    /// tell an under-voltage event that has since passed from one that
    /// never happened.
    pub fn throttled(&mut self) -> Result<u32, Error> {
        let response = self.property_call(0x0003_0046, &[0], 2)?;
        Ok(response[1])
    }

    /// Powers a firmware *power domain* on (`on = true`) or off (tag
    /// `0x0003_8030`, "Set Domain State"), returning the resulting state.
    ///
    /// Power domains are coarse analog/logic power islands the firmware
    /// gates — distinct from [`Self::set_power_state`]'s device list, and
    /// from register (MMIO) access, which stays alive independently. The
    /// camera receiver's analog D-PHY lives in one such domain
    /// ([`POWER_DOMAIN_UNICAM1`]) that must be powered before it can
    /// recover high-speed data. A firmware too old to implement the tag
    /// surfaces [`Error::TagNotAnswered`] from the underlying call.
    pub fn set_power_domain(&mut self, domain: u32, on: bool) -> Result<bool, Error> {
        let response = self.property_call(0x0003_8030, &[domain, on as u32], 2)?;
        Ok(response[1] != 0)
    }

    /// Enables (`enable = true`) or disables the V3D 3D pipeline (tag
    /// `0x0003_0012`, "Set Enable QPU") — the block containing the QPU
    /// shader cores (`crate::v3d`, behind this crate's `v3d` feature)
    /// is power/clock-gated off by default, and this is the only way
    /// to bring it up before its registers do anything. Not linked:
    /// this method is compiled unconditionally, but `v3d` isn't — a
    /// build without that feature would have a broken intra-doc link
    /// otherwise. The tag's response carries no documented
    /// meaning beyond acknowledging the request, so — like
    /// [`Self::set_expander_gpio`] — this only checks that the firmware
    /// answered at all rather than interpreting the response value.
    pub fn set_enable_qpu(&mut self, enable: bool) -> Result<(), Error> {
        self.property_call(0x0003_0012, &[enable as u32], 1)?;
        Ok(())
    }

    /// Powers `device` on (`on = true`) or off, waiting for its power
    /// state to stabilize before returning (tag `0x0002_8001`, "Set
    /// Power State", with the "wait for stable" bit set). Returns
    /// `true` if the device is powered and present afterwards, `false`
    /// if the firmware reports it does not exist. Needed for USB on
    /// this board — see [`PowerDeviceId::UsbHcd`].
    pub fn set_power_state(&mut self, device: PowerDeviceId, on: bool) -> Result<bool, Error> {
        // state bit 0 = power on/off, bit 1 = wait for the change to
        // stabilize before the firmware answers.
        let state = if on { 0b11 } else { 0b10 };
        let response = self.property_call(0x0002_8001, &[device as u32, state], 2)?;
        // response bit 0 = now powered, bit 1 = device does not exist.
        Ok(response[1] & 0b10 == 0 && response[1] & 0b1 == if on { 1 } else { 0 })
    }

    /// Drives one of the VideoCore firmware's GPIO-expander lines high
    /// (`on = true`) or low (tag `0x0003_8041`, "Set GPIO State").
    ///
    /// These are *not* the SoC's own ARM GPIO pins (those go through
    /// [`crate::gpio`]) — they're extra lines behind a small expander
    /// the VideoCore owns, addressed at a base of 128. On a Pi 3 the
    /// on-board wireless chip's enable line, `WL_ON`, is expander line
    /// 1, i.e. [`EXPANDER_WL_ON`] — the SoC has no direct GPIO to it, so
    /// this firmware call is the only way to power the radio up. The
    /// firmware echoes the request back; a mismatch surfaces as
    /// [`Error::TagNotAnswered`] from the underlying property call
    /// rather than being checked here.
    pub fn set_expander_gpio(&mut self, pin: u32, on: bool) -> Result<(), Error> {
        self.property_call(0x0003_8041, &[pin, on as u32], 2)?;
        Ok(())
    }

    /// Reads the current state of GPIO-expander line `pin` (tag
    /// `0x0003_0041`, "Get GPIO State") — the read counterpart to
    /// [`Self::set_expander_gpio`], returning `true` if the line reads
    /// high. Also useful purely to probe whether this firmware
    /// implements the expander mailbox interface at all: a firmware too
    /// old to know the tag answers with [`Error::TagNotAnswered`].
    pub fn get_expander_gpio(&mut self, pin: u32) -> Result<bool, Error> {
        let response = self.property_call(0x0003_0041, &[pin, 0], 2)?;
        Ok(response[1] != 0)
    }

    /// Allocates a `width`×`height` framebuffer at `depth_bits` bits per
    /// pixel (`8`/`16`/`24`/`32` are the depths the firmware actually
    /// supports; `32` is the common choice) in `pixel_order`, and
    /// returns where to write pixels.
    ///
    /// The buffer is exactly the size of the display, so pixels are
    /// written to the memory the VideoCore is scanning out at that very
    /// moment. Anything that redraws a whole frame will show a partly
    /// updated picture for as long as the redraw takes — see
    /// [`allocate_framebuffer_paged`](Self::allocate_framebuffer_paged)
    /// for the tear-free alternative.
    pub fn allocate_framebuffer(
        &mut self,
        width: u32,
        height: u32,
        depth_bits: u32,
        pixel_order: PixelOrder,
    ) -> Result<Framebuffer, Error> {
        self.allocate_framebuffer_paged(width, height, 1, depth_bits, pixel_order)
    }

    /// Allocates a framebuffer `pages` screens tall — a `width`×`height`
    /// display area backed by `pages` full-size pages, only one of which
    /// is on screen at a time.
    ///
    /// This is what makes tear-free output possible: draw the next frame
    /// into a page the display isn't showing, then bring it on screen in
    /// one step with [`set_virtual_offset`](Self::set_virtual_offset).
    /// Nothing is ever written to the page being scanned out, so a
    /// half-finished frame is never visible.
    ///
    /// Two pages is the minimum for that, but the page just retired
    /// stays on screen until the display hardware picks the new one up
    /// at its next vertical blank, so with two pages the next frame's
    /// drawing can start before that happens. Three pages puts two
    /// flips between a page leaving the screen and being drawn into
    /// again, which is enough margin to never wait.
    ///
    /// Requests physical size, virtual size, depth, pixel order, the
    /// buffer itself, and its pitch as one combined multi-tag call (tags
    /// `0x0004_8003`, `0x0004_8004`, `0x0004_8005`, `0x0004_8006`,
    /// `0x0004_0001`, `0x0004_0008`) rather than as separate calls —
    /// older firmware only processes a framebuffer request correctly
    /// when every tag arrives together in one buffer.
    ///
    /// The VideoCore, not this driver, decides which physical output
    /// (HDMI or the MIPI DSI touchscreen) this framebuffer appears on —
    /// that's `config.txt`/firmware configuration, not something this
    /// call chooses.
    pub fn allocate_framebuffer_paged(
        &mut self,
        width: u32,
        height: u32,
        pages: u32,
        depth_bits: u32,
        pixel_order: PixelOrder,
    ) -> Result<Framebuffer, Error> {
        /// Alignment (in bytes) requested for the allocated buffer —
        /// the firmware's own recommendation for a framebuffer.
        const ALLOCATE_ALIGNMENT: u32 = 4096;

        /// Word count of the whole request/response buffer: the 2-word
        /// message header, then each tag's (3-word header + value
        /// words) — (5 + 5 + 4 + 4 + 5 + 4) — plus the 1-word end tag.
        const WORDS: usize = 2 + (5 + 5 + 4 + 4 + 5 + 4) + 1;

        // 16-byte aligned for the same reason as `property_call`'s
        // buffer: the low 4 bits of its address double as the mailbox
        // channel number.
        #[repr(C, align(16))]
        struct Buffer {
            words: [u32; WORDS],
        }

        // Zero pages would ask the firmware for a zero-height buffer;
        // one page is the un-paged case, which is what the caller of
        // `allocate_framebuffer` gets.
        let virtual_height = height * pages.max(1);

        let mut buffer = Buffer { words: [0; WORDS] };
        buffer.words[0] = (WORDS * 4) as u32;
        buffer.words[1] = 0;

        // Set physical (display) width/height.
        buffer.words[2] = 0x0004_8003;
        buffer.words[3] = 8;
        buffer.words[4] = 0;
        buffer.words[5] = width;
        buffer.words[6] = height;

        // Set virtual (buffer) width/height. Taller than physical is
        // what gives the pages: the firmware allocates the lot, and the
        // display shows a `height`-tall window into it that
        // `set_virtual_offset` moves.
        buffer.words[7] = 0x0004_8004;
        buffer.words[8] = 8;
        buffer.words[9] = 0;
        buffer.words[10] = width;
        buffer.words[11] = virtual_height;

        // Set depth.
        buffer.words[12] = 0x0004_8005;
        buffer.words[13] = 4;
        buffer.words[14] = 0;
        buffer.words[15] = depth_bits;

        // Set pixel order.
        buffer.words[16] = 0x0004_8006;
        buffer.words[17] = 4;
        buffer.words[18] = 0;
        buffer.words[19] = pixel_order as u32;

        // Allocate buffer: the request value is the desired alignment;
        // the firmware overwrites it with the buffer's VideoCore bus
        // address, and the word after with the buffer's size.
        buffer.words[20] = 0x0004_0001;
        buffer.words[21] = 8;
        buffer.words[22] = 0;
        buffer.words[23] = ALLOCATE_ALIGNMENT;
        buffer.words[24] = 0;

        // Get pitch.
        buffer.words[25] = 0x0004_0008;
        buffer.words[26] = 4;
        buffer.words[27] = 0;
        buffer.words[28] = 0;

        // buffer.words[29] (the end tag) is already zero from the
        // initializer above.

        let address = &buffer as *const Buffer as u32;
        clean_range(address, WORDS * 4);

        let echoed = self.call_raw(CHANNEL_PROPERTY_TAGS, address);
        if echoed != address {
            return Err(Error::AddressMismatch);
        }

        invalidate_range(address, WORDS * 4);

        if buffer.words[1] & RESPONSE_BIT == 0 {
            return Err(Error::RequestFailed);
        }
        // Every tag's response code word sits 2 words after its id.
        for &code_index in &[4, 9, 14, 18, 22, 27] {
            if buffer.words[code_index] & RESPONSE_BIT == 0 {
                return Err(Error::TagNotAnswered);
            }
        }

        Ok(Framebuffer {
            address: from_vc_bus_address(buffer.words[23]),
            size_bytes: buffer.words[24],
            pitch_bytes: buffer.words[28],
            width: buffer.words[5],
            height: buffer.words[6],
            // The firmware's own answer, not `virtual_height` as asked
            // for: a request it clamped would otherwise have callers
            // computing page offsets past the end of the allocation.
            virtual_height: buffer.words[11],
            depth_bits: buffer.words[15],
        })
    }

    /// Scrolls the display window to `(x, y)` within the virtual buffer
    /// (tag `0x0004_8009`, "Set Virtual Offset") — the page flip for a
    /// framebuffer from
    /// [`allocate_framebuffer_paged`](Self::allocate_framebuffer_paged),
    /// where `y` is `page * height`.
    ///
    /// The call returns as soon as the firmware has taken the new
    /// offset, which is not the same moment the picture changes: the
    /// display hardware picks the new window up at its next vertical
    /// blank, so the previous page stays on screen for up to one refresh
    /// after this returns. That delay is why a page must not be drawn
    /// into again immediately after being flipped away from — see that
    /// method's note on page count.
    pub fn set_virtual_offset(&mut self, x: u32, y: u32) -> Result<(), Error> {
        self.property_call(0x0004_8009, &[x, y], 2)?;
        Ok(())
    }

    /// Blocks until the display's next vertical blank (tag
    /// `0x0004_800e`, "Set VSync").
    ///
    /// The firmware answers this tag when the display next finishes a
    /// frame, so a caller that draws straight into the scanned-out
    /// buffer of [`allocate_framebuffer`](Self::allocate_framebuffer)
    /// can at least start each redraw at the top of a refresh. That
    /// only narrows the window for tearing rather than closing it — a
    /// redraw slower than a refresh still gets caught up with — so
    /// prefer pages and [`set_virtual_offset`](Self::set_virtual_offset)
    /// where the memory is available.
    ///
    /// Note that this occupies the mailbox for up to a full refresh
    /// period, which on a shared mailbox blocks every other caller for
    /// that long.
    pub fn wait_for_vsync(&mut self) -> Result<(), Error> {
        self.property_call(0x0004_800e, &[0], 1)?;
        Ok(())
    }

    /// Tells the firmware to keep the DSI touchscreen's current touch
    /// points written into `address` from now on (tag `0x0004_801f`,
    /// "Set Touchbuffer") — `address` must stay valid and
    /// firmware-writable forever after this call, since the firmware
    /// won't be told again.
    ///
    /// The complementary tag, `0x0004_000f` ("Get Touchbuffer" — ask the
    /// firmware for a buffer *it* allocated, rather than handing it
    /// one), isn't exposed here: confirmed on real hardware to return
    /// an address inside the peripheral MMIO aperture rather than RAM,
    /// which then hangs the core on the first read rather than merely
    /// reading back zero. The Linux `rpi-ft5406` driver's own history
    /// shows the same shift — it now tries this push model first,
    /// falling back to `GET_TOUCHBUF` only for firmware old enough to
    /// not support this tag at all.
    pub fn set_touch_buffer_address(&mut self, address: u32) -> Result<(), Error> {
        self.property_call(0x0004_801f, &[to_vc_bus_address(address)], 1)?;
        Ok(())
    }

    /// Shared implementation of [`Self::arm_memory`]/
    /// [`Self::vc_memory`] — both are a no-argument tag whose response
    /// is a `(base_address, size_bytes)` pair, differing only in tag
    /// id.
    fn memory_region(&mut self, tag: u32) -> Result<MemoryRegion, Error> {
        let response = self.property_call(tag, &[], 2)?;
        Ok(MemoryRegion {
            base_address: response[0],
            size_bytes: response[1],
        })
    }

    /// Runs one single-tag property-interface request/response
    /// round-trip: builds the buffer, hands it to the firmware over
    /// the mailbox, and returns however many response words the
    /// caller declared it expects (`response_words`; the rest of the
    /// returned array is zero and meaningless).
    ///
    /// Neither `request` nor `response_words` may exceed
    /// [`MAX_VALUE_WORDS`], the shared value area's size; a call that does
    /// gets [`Error::ValueTooLarge`] rather than overrunning the buffer.
    /// Checked at runtime, not with a `debug_assert`: every call site is a
    /// fixed literal in this module, so getting it wrong is a build-time
    /// mistake — but release builds are the ones that run on hardware, and
    /// one that skipped the check turned a too-long tag into a panic on a
    /// board instead of an error at the call.
    fn property_call(
        &mut self,
        tag: u32,
        request: &[u32],
        response_words: usize,
    ) -> Result<[u32; MAX_VALUE_WORDS], Error> {
        if request.len() > MAX_VALUE_WORDS || response_words > MAX_VALUE_WORDS {
            return Err(Error::ValueTooLarge);
        }

        // 16-byte aligned: the low 4 bits of the buffer's address
        // double as the mailbox channel number in the message word,
        // so a misaligned buffer would silently corrupt its own
        // address.
        #[repr(C, align(16))]
        struct Buffer {
            words: [u32; BUFFER_WORDS],
        }

        let mut buffer = Buffer {
            words: [0; BUFFER_WORDS],
        };
        buffer.words[0] = (BUFFER_WORDS * 4) as u32;
        buffer.words[1] = 0;
        buffer.words[2] = tag;
        buffer.words[3] = (MAX_VALUE_WORDS * 4) as u32;
        buffer.words[4] = 0;
        buffer.words[5..5 + request.len()].copy_from_slice(request);
        // buffer.words[5 + MAX_VALUE_WORDS] (the end tag) is already
        // zero from the initializer above.

        let address = &buffer as *const Buffer as u32;
        clean_range(address, BUFFER_WORDS * 4);

        let echoed = self.call_raw(CHANNEL_PROPERTY_TAGS, address);
        if echoed != address {
            return Err(Error::AddressMismatch);
        }

        invalidate_range(address, BUFFER_WORDS * 4);

        if buffer.words[1] & RESPONSE_BIT == 0 {
            return Err(Error::RequestFailed);
        }
        if buffer.words[4] & RESPONSE_BIT == 0 {
            return Err(Error::TagNotAnswered);
        }

        let mut response = [0u32; MAX_VALUE_WORDS];
        response[..response_words].copy_from_slice(&buffer.words[5..5 + response_words]);
        Ok(response)
    }

    /// One raw mailbox round-trip: waits for room in mailbox 1 and
    /// writes `message` (`buffer_address`'s [`to_vc_bus_address`]
    /// translation, with `channel` or-ed into its low 4 bits), then
    /// waits for and returns mailbox 0's response message with that
    /// same channel — masked back to a plain address, still in VC bus
    /// address space (the caller already has the ARM-side address to
    /// compare against, from before the call).
    ///
    /// Other channels' responses (there are none in flight here, since
    /// this driver only ever issues one call at a time and waits for
    /// its reply before returning) are skipped rather than treated as
    /// this call's answer — matching how every mailbox client is
    /// documented to multiplex the same physical FIFO.
    fn call_raw(&mut self, channel: u32, buffer_address: u32) -> u32 {
        let bus_address = to_vc_bus_address(buffer_address);
        let message = (bus_address & !0xF) | (channel & 0xF);

        while self.vcmailbox.status1().read().bits() & (1 << 31) != 0 {}
        unsafe {
            self.vcmailbox.write().write_with_zero(|w| w.bits(message));
        }

        loop {
            while self.vcmailbox.status0().read().empty().bit_is_set() {}
            let response = self.vcmailbox.read().read().bits();
            if response & 0xF == channel {
                return (response & !0xF) - 0xC000_0000;
            }
        }
    }
}

/// Translates a plain ARM physical address to the VideoCore's
/// `0xC000_0000`-based "direct, uncached" bus alias — the window that
/// bypasses the VideoCore's own L2 cache entirely, so this driver only
/// has to reason about the ARM core's cache (see this module's doc
/// comment), not a second one it has no maintenance operations for.
fn to_vc_bus_address(physical_address: u32) -> u32 {
    (physical_address & 0x3FFF_FFFF) | 0xC000_0000
}

/// Translates a VideoCore bus address (as returned by, e.g.,
/// [`Mailbox::allocate_framebuffer`]'s buffer allocation) back to a
/// plain ARM physical address — the inverse of [`to_vc_bus_address`].
fn from_vc_bus_address(bus_address: u32) -> u32 {
    bus_address & 0x3FFF_FFFF
}
