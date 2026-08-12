//! Blocking driver for the BCM Power Management (PM) block's watchdog
//! timer: arms a hardware countdown that triggers a full board reset
//! (equivalent to a power cycle) if it's not re-armed before it
//! expires, for automatic recovery from a hang that leaves nothing
//! running to notice or act on it.
//!
//! Like `rng.rs`, this peripheral isn't modeled in `bcm2837-lpa`'s SVD,
//! so this pokes its known physical addresses directly. Register
//! layout, the password requirement, and the reset-type encodings
//! follow Linux's `bcm2835_wdt` driver (`drivers/watchdog/
//! bcm2835_wdt.c`), the standard reference for this undocumented-by-
//! Broadcom corner of the PM block.
//!
//! ## Password
//!
//! `PM_RSTC`/`PM_WDOG` only accept a write whose top byte is
//! `PM_PASSWORD` — any other value is silently ignored by the
//! hardware. This isn't a security mechanism, just a guard against
//! accidental writes to a register block that can reset the whole
//! board; every write here ORs the password into the value that
//! matters rather than clearing the rest of the register, so it never
//! needs separate handling beyond that OR.
//!
//! ## Countdown units and range
//!
//! `PM_WDOG` is a 20-bit tick count clocked at a fixed 65536 Hz, so the
//! representable range tops out around 16 seconds — see
//! [`MAX_TIMEOUT_MS`](crate::watchdog::MAX_TIMEOUT_MS). There's no way to
//! ask for a longer single timeout; a caller needing to survive a longer
//! expected stall must re-arm
//! ([`Watchdog::feed`](crate::watchdog::Watchdog::feed)) more often than
//! that, not request one
//! long period.
//!
//! ## Stopping
//!
//! [`Watchdog::disable`](crate::watchdog::Watchdog::disable) writes
//! `PM_RSTC_RESET`, the same "reset type" value Linux's driver uses to stop
//! the watchdog — distinct from the `WRCFG_FULL_RESET` type
//! [`Watchdog::start`](crate::watchdog::Watchdog::start)/
//! [`Watchdog::feed`](crate::watchdog::Watchdog::feed) arm.
//! Broadcom hasn't published what every `PM_RSTC` reset-type encoding
//! means; this one is taken on faith from the reference driver rather
//! than independently derived.

/// PM block base address: `crate::soc::PERIPHERAL_BASE` (BCM2836/2837 or,
/// under the `bcm2711` feature, BCM2711) plus the block's `0x0010_0000`
/// offset, same on both -- the PM block is unchanged IP, just relocated
/// with the rest of the peripheral block.
const PM_BASE: usize = crate::soc::PERIPHERAL_BASE as usize + 0x0010_0000;
/// Reset controller register: bits [5:4] select the reset type applied
/// when the watchdog countdown reaches zero (or immediately, for
/// [`PM_RSTC_RESET`]).
const PM_RSTC: *mut u32 = (PM_BASE + 0x1c) as *mut u32;
/// Watchdog register: bits [19:0] are the countdown, in ticks of a
/// fixed 65536Hz clock; writing it loads a new countdown, reading it
/// returns the countdown remaining.
const PM_WDOG: *mut u32 = (PM_BASE + 0x24) as *mut u32;

/// Password required in the top byte of every `PM_RSTC`/`PM_WDOG`
/// write for the hardware to accept it (see this module's doc
/// comment).
const PM_PASSWORD: u32 = 0x5a00_0000;
/// `PM_WDOG`'s 20-bit countdown field.
const PM_WDOG_TIME_MASK: u32 = 0x000f_ffff;
/// `PM_RSTC`'s reset-type field (bits [5:4]).
const PM_RSTC_WRCFG_MASK: u32 = 0x0000_0030;
/// `PM_RSTC` reset-type value that arms a full board reset once the
/// watchdog countdown reaches zero.
const PM_RSTC_WRCFG_FULL_RESET: u32 = 0x0000_0020;
/// `PM_RSTC` reset-type value that disarms the watchdog (see
/// [`Watchdog::disable`]).
const PM_RSTC_RESET: u32 = 0x0000_0102;

/// `PM_WDOG`'s countdown clock rate, fixed in hardware.
const WDOG_TICKS_PER_SEC: u64 = 1 << 16;

/// Largest timeout [`Watchdog::start`]/[`Watchdog::feed`] can arm,
/// imposed by `PM_WDOG`'s 20-bit countdown field at its fixed 65536Hz
/// clock rate (`0xF_FFFF` ticks, truncated down to a whole
/// millisecond).
pub const MAX_TIMEOUT_MS: u32 = ((PM_WDOG_TIME_MASK as u64 * 1000) / WDOG_TICKS_PER_SEC) as u32;

/// Blocking driver for the PM block's watchdog timer.
pub struct Watchdog {
    /// The timeout last armed via [`Self::start`], reloaded by
    /// [`Self::feed`] — `PM_WDOG` only exposes the countdown
    /// *remaining*, not the value it was last loaded with, so the
    /// original timeout has to be cached here to re-arm the same
    /// duration.
    timeout_ms: Option<u32>,
}

impl Watchdog {
    /// Constructs a driver over the PM watchdog registers. Doesn't arm
    /// anything — call [`Self::start`] to begin the countdown.
    ///
    /// # Safety of construction
    ///
    /// Like [`crate::rng::Rng`], there's no singleton token to hand
    /// over (the peripheral isn't in `bcm2837-lpa`), so nothing here
    /// prevents constructing two `Watchdog`s aliasing the same
    /// hardware. Unlike the RNG though, this hardware has real
    /// per-instance state (the armed/disarmed countdown) that two
    /// independent callers could stomp on each other's configuration
    /// of — `new` is still safe in the narrow "doesn't violate memory
    /// safety" sense, but a caller mixing timeouts across two
    /// `Watchdog`s would get the last-written one, not a combination of
    /// both.
    pub fn new() -> Self {
        Self { timeout_ms: None }
    }

    /// Arms the watchdog: the board resets in `timeout_ms` unless
    /// [`Self::feed`] or another [`Self::start`] runs first. Panics if
    /// `timeout_ms` exceeds [`MAX_TIMEOUT_MS`] — the hardware can't
    /// represent it, and silently clamping would arm a shorter timeout
    /// than the caller asked for without them knowing.
    pub fn start(&mut self, timeout_ms: u32) {
        assert!(
            timeout_ms <= MAX_TIMEOUT_MS,
            "watchdog timeout {timeout_ms}ms exceeds hardware maximum of {MAX_TIMEOUT_MS}ms"
        );
        self.timeout_ms = Some(timeout_ms);
        self.arm(timeout_ms);
    }

    /// Re-arms the countdown at the timeout last passed to
    /// [`Self::start`] — "pets" the watchdog to stave off the reset it
    /// would otherwise trigger.
    ///
    /// # Panics
    ///
    /// If called before [`Self::start`] has ever run, since there's no
    /// timeout yet to reload.
    pub fn feed(&mut self) {
        let timeout_ms = self
            .timeout_ms
            .expect("Watchdog::feed called before Watchdog::start");
        self.arm(timeout_ms);
    }

    /// Disarms the watchdog: no reset fires regardless of how long it's
    /// left unfed, until [`Self::start`] runs again.
    pub fn disable(&mut self) {
        unsafe {
            core::ptr::write_volatile(PM_RSTC, PM_PASSWORD | PM_RSTC_RESET);
        }
    }

    /// Loads `timeout_ms` into `PM_WDOG` and arms `PM_RSTC`'s
    /// full-reset type. Split out from `start`/`feed` since both do
    /// exactly this, differing only in whether `self.timeout_ms` also
    /// gets (re)written.
    fn arm(&self, timeout_ms: u32) {
        let ticks = ((timeout_ms as u64 * WDOG_TICKS_PER_SEC) / 1000) as u32 & PM_WDOG_TIME_MASK;
        unsafe {
            core::ptr::write_volatile(PM_WDOG, PM_PASSWORD | ticks);
            let rstc = core::ptr::read_volatile(PM_RSTC);
            core::ptr::write_volatile(
                PM_RSTC,
                PM_PASSWORD | (rstc & !PM_RSTC_WRCFG_MASK) | PM_RSTC_WRCFG_FULL_RESET,
            );
        }
    }
}

impl Default for Watchdog {
    /// Equivalent to [`Watchdog::new`].
    fn default() -> Self {
        Self::new()
    }
}
