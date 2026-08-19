//! Identity of the core executing this code.
//!
//! Separate from `multicore`, which is about *starting* cores 1-3 and is
//! compiled only behind that feature, because asking which core is
//! running is useful without it: [`crate::generic_timer`] indexes a
//! per-core register with it on a single-core build, and an application
//! that has brought up secondary cores needs it in code shared by all of
//! them -- an interrupt handler, or a panic handler naming the core it
//! died on. Neither should have to enable a feature about spawning to ask.

/// The calling core's id (0-3), from `MPIDR`'s Aff0 field.
///
/// A `usize` rather than a `u8` because every use is an index or an offset
/// multiplier: the ARM-local peripherals give each core its own copy of
/// several registers, addressed as `base + stride * core_id()`.
///
/// The value is fixed for the lifetime of the code reading it — a core
/// cannot migrate — but that is only true of the *core*, not of a task or
/// future, which an executor may well poll somewhere else. Code that wants
/// to pin work to a core has to keep it out of anything relocatable, not
/// merely read this once.
#[cfg(target_arch = "arm")]
#[inline(always)]
pub fn core_id() -> usize {
    let mpidr: u32;
    // SAFETY: a read of a system register with no side effects. `MPIDR` is
    // readable at PL1, which is where this crate's code runs (`boot.s`
    // drops out of Hyp mode into SVC before calling anything here).
    unsafe { core::arch::asm!("mrc p15, 0, {}, c0, c0, 5", out(reg) mpidr) };
    (mpidr & 3) as usize
}

/// See the AArch32 sibling above.
#[cfg(target_arch = "aarch64")]
#[inline(always)]
pub fn core_id() -> usize {
    let mpidr: u64;
    // SAFETY: as above. `MPIDR_EL1` is readable at EL1, which is where
    // `boot64.s` leaves every core.
    unsafe { core::arch::asm!("mrs {}, mpidr_el1", out(reg) mpidr) };
    (mpidr & 3) as usize
}
