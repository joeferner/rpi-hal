//! Facts shared by the Clock Manager's peripheral clock generators.
//!
//! `CM_PWM` and `CM_PCM` are two instances of the same generator — they even
//! share a register block in the PAC — and [`crate::pwm`] and [`crate::pcm`]
//! program them identically: `PLLD_per` as the source, integer division only,
//! kill-then-reconfigure. What differs is which peripheral consumes the
//! result.
//!
//! The divisor limit and the arithmetic around it live here rather than being
//! restated in each driver, because a hardware fact copied into two places is
//! a hardware fact that can be right in one of them.

/// The rate of `PLLD_per`, the source both generators are configured to use.
///
/// Nominal rather than measured: the figure is commonly cited but this crate
/// has not independently confirmed it, and because it comes from a PLL there
/// is no crystal to derive it from. Every rate computed from it inherits that
/// caveat — see [`crate::pwm`]'s "Clock" section.
pub const SOURCE_HZ: u32 = 500_000_000;

/// The smallest usable integer divisor.
///
/// Zero is not division by one; it is not a meaningful setting for the
/// generator at all, so [`clamp_divisor`] lifts it to this.
pub const MIN_DIVISOR: u16 = 1;

/// The largest integer divisor a generator can hold — **`DIVI` is 12 bits**.
///
/// This is the interesting one, because nothing in the hardware or the PAC
/// complains about exceeding it. The PAC's field writer *masks* the value it
/// is given, so a divisor of 12500 is programmed as `12500 & 0xFFF` = 212 and
/// the clock comes out 59 times too fast, with every register reading back
/// exactly as written. That failure is invisible from software and presents
/// only as a peripheral running at an inexplicable rate.
///
/// Hence [`clamp_divisor`], which every caller here goes through: a clamped
/// divisor is still not what was asked for, but it is off by a bounded factor
/// and in the right direction, which is the difference between a wrong number
/// and a mystery.
pub const MAX_DIVISOR: u16 = 4095;

/// The slowest clock a generator can produce, at [`MAX_DIVISOR`].
///
/// Worth knowing before designing around a target rate: anything slower than
/// this is simply not reachable through the integer divider, and asking for
/// it yields this instead.
pub const MIN_CLOCK_HZ: u32 = SOURCE_HZ / MAX_DIVISOR as u32;

/// `divisor` brought into the range the hardware can actually hold.
pub const fn clamp_divisor(divisor: u16) -> u16 {
    if divisor < MIN_DIVISOR {
        MIN_DIVISOR
    } else if divisor > MAX_DIVISOR {
        MAX_DIVISOR
    } else {
        divisor
    }
}

/// The rate a generator runs at once programmed with `divisor`.
///
/// Applies [`clamp_divisor`] first, so this reports what the hardware will
/// *do* rather than what the caller asked for — which is the point of it
/// existing.
pub const fn clock_hz(divisor: u16) -> u32 {
    SOURCE_HZ / clamp_divisor(divisor) as u32
}

/// The divisor giving the closest rate at or above `target_hz`, clamped to
/// what the hardware can hold.
///
/// A zero target yields [`MAX_DIVISOR`] — the slowest clock available —
/// rather than dividing by zero.
pub const fn divisor_for(target_hz: u32) -> u16 {
    if target_hz == 0 {
        return MAX_DIVISOR;
    }
    let divisor = SOURCE_HZ / target_hz;
    if divisor > u16::MAX as u32 {
        MAX_DIVISOR
    } else {
        clamp_divisor(divisor as u16)
    }
}
