//! The Performance Monitors Unit's cycle counter — a CPU-cycle clock for
//! profiling code too fine-grained to time any other way.
//!
//! The System Timer ([`crate::timer`]) is the right clock for almost
//! everything here: it is monotonic, shared by every core, and counts real
//! microseconds. What it cannot do is measure something that takes a few
//! hundred cycles, because reading it is an MMIO access over the
//! peripheral bus — hundreds of nanoseconds, comparable to or larger than
//! whatever is being measured, and the measurement changes the answer.
//!
//! `PMCCNTR` is a system register, read with a single `mrs`/`mrc`, so
//! reading it costs a handful of cycles. That makes it usable *inside* a
//! hot loop: bracket each part of the work, accumulate the differences, and
//! find out where the time actually goes rather than guessing from the
//! total.
//!
//! It counts this core's cycles, so the number is only comparable against
//! itself: it does not convert to wall-clock time unless the ARM clock is
//! known and steady (see
//! [`Mailbox::clock_rate_hz`](crate::mailbox::Mailbox::clock_rate_hz) —
//! the firmware brings the core up at its minimum rate and changes it out
//! from under a program that never asks), and a second core's counter is a
//! different counter, started at a different moment.

use core::arch::asm;

/// This core's PMU cycle counter, enabled and running.
///
/// Enabling is per-core, and so is the count: a [`CycleCounter`] obtained
/// on one core says nothing about another's. Cheap enough to read from
/// inside a loop — that is the whole point of it over the System Timer.
pub struct CycleCounter(());

impl CycleCounter {
    /// Enables the PMU and its cycle counter on the calling core, resets
    /// the count to zero, and returns a handle to read it.
    ///
    /// Calling this again re-zeroes the counter, which is harmless but
    /// invalidates any interval already in progress.
    #[cfg(target_arch = "aarch64")]
    pub fn enable() -> Self {
        // SAFETY: PMU control registers, accessible at EL1 (nothing here
        // sets MDCR_EL2.TPM, the trap that would forbid it). Writing them
        // affects only this core's counters.
        unsafe {
            let mut pmcr: u64;
            asm!("mrs {}, pmcr_el0", out(reg) pmcr);
            // E (bit 0) enables the counters, C (bit 2) resets the cycle
            // counter to zero, and D (bit 3) is cleared so it counts every
            // cycle rather than every 64th — the divider would make short
            // intervals unmeasurable, which is what this is for.
            pmcr = (pmcr & !(1 << 3)) | (1 << 0) | (1 << 2);
            asm!("msr pmcr_el0, {}", in(reg) pmcr);
            // PMCNTENSET_EL0 bit 31 is the cycle counter's own enable, on
            // top of PMCR's global one.
            asm!("msr pmcntenset_el0, {}", in(reg) 1u64 << 31);
        }
        Self(())
    }

    /// Enables the PMU and its cycle counter on the calling core (AArch32).
    #[cfg(target_arch = "arm")]
    pub fn enable() -> Self {
        // SAFETY: the CP15 performance-monitor registers, writable at PL1.
        // Affects only this core's counters.
        unsafe {
            let mut pmcr: u32;
            asm!("mrc p15, 0, {}, c9, c12, 0", out(reg) pmcr);
            // Same bits as the AArch64 path: E, C, and D cleared.
            pmcr = (pmcr & !(1 << 3)) | (1 << 0) | (1 << 2);
            asm!("mcr p15, 0, {}, c9, c12, 0", in(reg) pmcr);
            asm!("mcr p15, 0, {}, c9, c12, 1", in(reg) 1u32 << 31);
        }
        Self(())
    }

    /// The cycles counted since [`Self::enable`].
    ///
    /// 64 bits and free-running on AArch64. On AArch32 the counter is only
    /// 32 bits wide, so it wraps about every four seconds at 1 GHz: take
    /// differences with `wrapping_sub`, and only over intervals short
    /// enough that at most one wrap can fall inside one.
    pub fn read(&self) -> u64 {
        #[cfg(target_arch = "aarch64")]
        {
            let cycles: u64;
            // SAFETY: a read of this core's cycle counter, enabled above;
            // no side effects.
            unsafe { asm!("mrs {}, pmccntr_el0", out(reg) cycles, options(nomem, nostack)) };
            cycles
        }
        #[cfg(target_arch = "arm")]
        {
            let cycles: u32;
            // SAFETY: as above, through CP15.
            unsafe { asm!("mrc p15, 0, {}, c9, c13, 0", out(reg) cycles, options(nomem, nostack)) };
            cycles as u64
        }
    }
}
