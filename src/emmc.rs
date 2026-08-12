//! Register-level primitives shared by both drivers that talk to the
//! Arasan "EMMC" (SDHCI-compatible) host controller: the SD-card slot
//! driver ([`crate::sd`]) and the on-board wireless chip's SDIO driver
//! ([`crate::sdio`]). Both drive the *same* physical controller — just
//! routed to different GPIO pins and speaking different command sets —
//! so anything that's purely a property of the controller (rather than
//! of SD memory vs. SDIO) lives here so the two can't drift apart.

/// Computes the SDHCI clock divisor for `target_hz` from `base_hz`,
/// returned as the `(CLK_FREQ8, CLK_FREQ_MS2)` register field pair.
/// Faithfully ported from rsta2's `circle` `GetClockDivider`.
///
/// The value programmed is a power of two, and the controller divides
/// `base_hz` by *twice* it (SDHCI's inherent ÷2), so the effective
/// clock is `base_hz / (2 * value)` — e.g. from a 200MHz base, a
/// target of 400kHz yields value 256 → ~390kHz, and 25MHz yields value
/// 4 → exactly 25MHz. Always rounds the divisor *up* (clock down), so
/// the result never exceeds `target_hz` (important during
/// identification, where the SD/SDIO spec sets 400kHz as a ceiling).
pub(crate) fn clock_divider(base_hz: u32, target_hz: u32) -> (u8, u8) {
    let mut divisor = 1;
    if target_hz <= base_hz {
        divisor = base_hz / target_hz;
        if !base_hz.is_multiple_of(target_hz) {
            divisor -= 1;
        }
    }

    // Round `divisor` up to a power of two, expressed as an exponent:
    // the position of its highest set bit, plus one if any lower bits
    // remain (i.e. it wasn't already a power of two).
    let mut exponent: i32 = -1;
    for first_bit in (0..32).rev() {
        if divisor & (1 << first_bit) != 0 {
            exponent = first_bit;
            divisor &= !(1 << first_bit);
            if divisor != 0 {
                exponent += 1;
            }
            break;
        }
    }
    if !(0..32).contains(&exponent) {
        exponent = 31;
    }

    // `2^(exponent - 1)`, not `2^exponent`: the register holds half the
    // total division factor because the controller's own ÷2 supplies
    // the other half (see this function's doc comment). `exponent == 0`
    // (base already at/below target) leaves the value at 0, selecting
    // the controller's minimum division.
    let mut value = if exponent != 0 {
        1u32 << (exponent - 1)
    } else {
        0
    };
    // The field is 10 bits wide.
    if value >= 0x400 {
        value = 0x3ff;
    }

    ((value & 0xff) as u8, ((value >> 8) & 0x3) as u8)
}
