use crate::pac::SYSTMR;

/// Free-running microsecond counter (BCM System Timer) and blocking
/// delays built on it.
///
/// The System Timer is a plain memory-mapped 64-bit counter (as CLO +
/// CHI) running at a fixed 1MHz, needing no configuration — it's
/// already counting from boot.
///
/// Of the four compare registers C0-C3, only Compare 1 is used: the GPU
/// firmware reserves C0 and C2 for itself, and C3 is left free.
///
/// Two properties of that compare shape every deadline API below, and
/// neither is a limitation of this driver:
///
/// - **It is 32 bits wide, against a 64-bit counter.** The compare only
///   sees CLO, the low half, so a deadline can only be expressed within
///   2^32us (~71.6 minutes) of the present, and any value matches once
///   per wrap whether or not anyone armed it.
/// - **It is an equality test, not a threshold.** CS.M1 is set when C1
///   equals CLO, so a deadline the counter has already walked past does
///   not fire late — it fires a whole wrap later.
pub struct Timer {
    systmr: SYSTMR,
}

/// The furthest ahead [`Timer::set_compare1`] will arm, chosen as half
/// the ~71.6-minute Compare 1 range so a clamped deadline is nowhere
/// near the wrap it has to stay clear of.
///
/// At this size the clamp is unreachable in any practical test, since it
/// takes a deadline over half an hour out. Shrinking it temporarily — to
/// a few tens of milliseconds — puts every ordinary deadline through the
/// clamp instead, turning a path that fires twice an hour into one that
/// fires constantly, which is the only cheap way to exercise it.
const MAX_COMPARE1_DELTA_US: u64 = 1 << 31;

impl Timer {
    /// Wraps the System Timer peripheral. Needs no configuration —
    /// it's already free-running from boot.
    pub fn new(systmr: SYSTMR) -> Self {
        Self { systmr }
    }

    /// Microseconds elapsed since boot.
    pub fn now_micros(&self) -> u64 {
        // CLO/CHI are two separate 32-bit registers forming one 64-bit
        // counter — reading them isn't atomic, so guard against CLO
        // wrapping between the two reads by re-checking CHI.
        loop {
            let hi = self.systmr.chi().read().bits();
            let lo = self.systmr.clo().read().bits();
            if self.systmr.chi().read().bits() == hi {
                return ((hi as u64) << 32) | lo as u64;
            }
        }
    }

    /// Busy-waits for approximately `us` microseconds.
    pub fn delay_us(&self, us: u32) {
        let target = self.now_micros() + us as u64;
        while self.now_micros() < target {}
    }

    /// Busy-waits for approximately `ms` milliseconds.
    pub fn delay_ms(&self, ms: u32) {
        let target = self.now_micros() + (ms as u64) * 1000;
        while self.now_micros() < target {}
    }

    /// Arms Compare 1 for the absolute [`now_micros`](Self::now_micros)
    /// deadline `deadline_us`, raising an IRQ at that point (once routed
    /// via `crate::lic::Lic::enable_timer1_irq` and unmasked via
    /// [`crate::irq::enable_irq`]).
    ///
    /// Returns `false` if the deadline was already in the past by the
    /// time it was armed — including the narrow case where the counter
    /// passed it between computing the target and writing it. Because
    /// the compare is an equality test (see the module doc comment),
    /// that deadline would otherwise not fire for another ~71.6 minutes,
    /// so a caller that still wants a timely wake-up must recompute
    /// against a fresh `now_micros` and arm again rather than assume it
    /// is pending.
    ///
    /// A deadline further out than ~35.8 minutes is clamped to that,
    /// keeping the write inside the range the 32-bit compare can
    /// express. The match then fires early, which callers must tolerate
    /// regardless: they have to re-check their own deadline against
    /// `now_micros` on every match and re-arm if it hasn't arrived yet.
    ///
    /// Does not clear a match already pending from a previous arm —
    /// pair with [`clear_compare1_match`](Self::clear_compare1_match).
    pub fn set_compare1(&self, deadline_us: u64) -> bool {
        let now = self.now_micros();
        if deadline_us <= now {
            return false;
        }

        let target = deadline_us.min(now + MAX_COMPARE1_DELTA_US);
        unsafe {
            self.systmr.c1().write(|w| w.bits(target as u32));
        }

        // Re-read rather than trusting the pre-write `now`: if the
        // counter reached the target while the write was in flight, the
        // equality may have gone by unmatched. Clear whatever this arm
        // did latch, so a caller acting on `false` doesn't then take a
        // spurious IRQ for a deadline it has already given up on.
        if self.now_micros() >= target {
            self.clear_compare1_match();
            return false;
        }

        true
    }

    /// Whether Compare 1 has matched (CS.M1) and not yet been cleared.
    pub fn compare1_matched(&self) -> bool {
        self.systmr.cs().read().m1().bit_is_set()
    }

    /// Acknowledges a fired Compare 1 match (write-1-to-clear CS.M1)
    /// without arming another. Call this from the IRQ handler —
    /// leaving the match flag set would just re-trigger the same IRQ
    /// immediately after returning.
    pub fn clear_compare1_match(&self) {
        unsafe {
            self.systmr
                .cs()
                .write_with_zero(|w| w.m1().clear_bit_by_one());
        }
    }

    /// Arms Compare 1 to match `period_us` from now.
    ///
    /// A convenience wrapper over [`set_compare1`](Self::set_compare1)
    /// for the fixed-period case, where the deadline is always close
    /// enough to the present that neither the clamp nor the
    /// already-passed case can be reached by a period of more than a
    /// couple of microseconds.
    pub fn arm_periodic_c1(&self, period_us: u32) {
        self.set_compare1(self.now_micros() + period_us as u64);
    }

    /// Acknowledges a fired Compare 1 match and rearms the next period —
    /// [`clear_compare1_match`](Self::clear_compare1_match) followed by
    /// [`arm_periodic_c1`](Self::arm_periodic_c1).
    pub fn ack_c1(&self, period_us: u32) {
        self.clear_compare1_match();
        self.arm_periodic_c1(period_us);
    }
}

impl embedded_hal::delay::DelayNs for Timer {
    /// Rounds up to the System Timer's 1us resolution, then delegates
    /// to `delay_us` — a nonzero request always waits at least one
    /// tick instead of returning immediately.
    fn delay_ns(&mut self, ns: u32) {
        self.delay_us(ns.div_ceil(1000));
    }

    /// Delegates to the inherent `Timer::delay_us`.
    fn delay_us(&mut self, us: u32) {
        Timer::delay_us(self, us);
    }

    /// Delegates to the inherent `Timer::delay_ms`.
    fn delay_ms(&mut self, ms: u32) {
        Timer::delay_ms(self, ms);
    }
}
