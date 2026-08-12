//! Board reboot and shutdown via the BCM Power Management (PM) block.
//!
//! Both work through the same reset machinery [`crate::watchdog`] uses:
//! arm the PM watchdog with a tiny countdown and select `PM_RSTC`'s
//! full-reset type, so the board resets almost immediately instead of
//! after a timeout. [`shutdown`](crate::power::shutdown) additionally writes a
//! "halt" sentinel
//! into `PM_RSTS` first — the boot-partition field set to 63, which the
//! firmware (`bootcode.bin`) reads on the way back up and, seeing that
//! reserved value, halts instead of booting. That's the closest this
//! hardware has to a power-off: the board stays powered but idle until it
//! is physically power-cycled, since nothing on it can gate its own
//! supply.
//!
//! Like [`crate::watchdog`] and [`crate::rng`], the PM block isn't modeled
//! in `bcm2837-lpa`, so this pokes its known physical addresses directly.
//! Register layout, the `PM_PASSWORD` requirement, and the halt encoding
//! follow Linux's `bcm2835_wdt` driver (`drivers/watchdog/bcm2835_wdt.c`),
//! which registers the restart and power-off handlers alongside the
//! watchdog itself — the same reference [`crate::watchdog`] follows, and
//! the standard one for this undocumented-by-Broadcom corner of the PM
//! block. The constants are intentionally re-declared here rather than
//! shared with [`crate::watchdog`], keeping each PM-touching module
//! self-contained the way `rng`/`watchdog` already are.

/// PM block base address (peripheral base `0x3F00_0000` + the block's
/// `0x0010_0000` offset), matching [`crate::watchdog`] and `mmu.rs`'s
/// `PERIPHERAL_BASE`.
const PM_BASE: usize = 0x3f10_0000;
/// Reset controller register: bits [5:4] select the reset type applied
/// when the watchdog countdown reaches zero.
const PM_RSTC: *mut u32 = (PM_BASE + 0x1c) as *mut u32;
/// Reset status register. Beyond latched reset-cause flags, its
/// even-numbered low bits ([0], [2], … [10]) encode the boot-partition
/// number the firmware reads on the next reset — the field [`shutdown`]
/// sets to 63 to request a halt.
const PM_RSTS: *mut u32 = (PM_BASE + 0x20) as *mut u32;
/// Watchdog register: bits [19:0] are the countdown, in ticks of a fixed
/// 65536 Hz clock.
const PM_WDOG: *mut u32 = (PM_BASE + 0x24) as *mut u32;

/// Password required in the top byte of every `PM_RSTC`/`PM_RSTS`/`PM_WDOG`
/// write for the hardware to accept it — any other top byte is silently
/// ignored (see [`crate::watchdog`]'s module doc for the full rationale).
const PM_PASSWORD: u32 = 0x5a00_0000;
/// `PM_WDOG`'s 20-bit countdown field.
const PM_WDOG_TIME_MASK: u32 = 0x000f_ffff;
/// `PM_RSTC`'s reset-type field (bits [5:4]).
const PM_RSTC_WRCFG_MASK: u32 = 0x0000_0030;
/// `PM_RSTC` reset-type value that arms a full board reset once the
/// watchdog countdown reaches zero.
const PM_RSTC_WRCFG_FULL_RESET: u32 = 0x0000_0020;

/// Boot-partition 63 encoded across `PM_RSTS`'s even low bits ([0], [2],
/// … [10]): `0x1 | 0x4 | 0x10 | 0x40 | 0x100 | 0x400`. 63 is the reserved
/// "halt" partition — the firmware treats it as "do not boot" rather than
/// a real partition to load, which is how [`shutdown`] stops the board
/// from rebooting after the reset. This is the same magic value Linux
/// writes to power a Pi off.
const PM_RSTS_HALT_PARTITION: u32 = 0x0000_0555;

/// A short watchdog countdown for a deliberate reset — long enough for the
/// register writes to take effect, short enough to be indistinguishable
/// from immediate (~150 µs at the 65536 Hz clock). Mirrors the count
/// Linux's driver loads for a software-requested restart.
const RESET_WDOG_TICKS: u32 = 10;

/// Reboots the board immediately.
///
/// Arms the PM watchdog with a tiny countdown and a full-reset type, then
/// parks the core; the reset fires within roughly 150 µs, equivalent to a
/// power cycle. Does not return.
///
/// Any pending UART/peripheral output should be flushed *before* calling
/// this — the reset does not wait for in-flight transmissions to drain.
pub fn reboot() -> ! {
    // SAFETY: the PM reset registers are device-mapped (within
    // `PERIPHERAL_BASE`, see `mmu.rs`); each write carries the required
    // password, and a board reset is the intended effect.
    unsafe { trigger_full_reset() };
    crate::halt();
}

/// Shuts the board down (halts it).
///
/// Sets `PM_RSTS`'s boot-partition field to the reserved "halt" value
/// (63), then triggers the same reset [`reboot`] does. On the way back up
/// the firmware sees the halt sentinel and stops instead of booting, so
/// the board goes idle and stays that way until it is physically power-
/// cycled — this hardware can't actually cut its own power. Does not
/// return.
///
/// As with [`reboot`], flush any pending output first.
pub fn shutdown() -> ! {
    // SAFETY: as in `reboot` — device-mapped PM registers, password
    // carried on every write. The read-modify-write of `PM_RSTS` clears
    // only the partition field and sets it to 63, preserving the latched
    // reset-cause status flags in the other bits.
    unsafe {
        let rsts = core::ptr::read_volatile(PM_RSTS);
        core::ptr::write_volatile(
            PM_RSTS,
            PM_PASSWORD | (rsts & !PM_RSTS_HALT_PARTITION) | PM_RSTS_HALT_PARTITION,
        );
        trigger_full_reset();
    }
    crate::halt();
}

/// Loads the short reset countdown into `PM_WDOG` and arms `PM_RSTC`'s
/// full-reset type — the reset trigger shared by [`reboot`] and
/// [`shutdown`], which differ only in whether the halt sentinel is written
/// first.
///
/// # Safety
///
/// Writes the PM reset registers, resetting the board once the countdown
/// (~150 µs) elapses.
unsafe fn trigger_full_reset() {
    core::ptr::write_volatile(
        PM_WDOG,
        PM_PASSWORD | (RESET_WDOG_TICKS & PM_WDOG_TIME_MASK),
    );
    let rstc = core::ptr::read_volatile(PM_RSTC);
    core::ptr::write_volatile(
        PM_RSTC,
        PM_PASSWORD | (rstc & !PM_RSTC_WRCFG_MASK) | PM_RSTC_WRCFG_FULL_RESET,
    );
}
