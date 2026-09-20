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
#[cfg(all(target_arch = "arm", not(armv6)))]
#[inline(always)]
pub fn core_id() -> usize {
    let mpidr: u32;
    // SAFETY: a read of a system register with no side effects. `MPIDR` is
    // readable at PL1, which is where this crate's code runs (`boot.s`
    // drops out of Hyp mode into SVC before calling anything here).
    unsafe { core::arch::asm!("mrc p15, 0, {}, c0, c0, 5", out(reg) mpidr) };
    (mpidr & 3) as usize
}

/// Always 0: the BCM2835's ARM1176JZF-S is a uniprocessor part, so there
/// is one core and this is its id.
///
/// A constant rather than a register read, and not only because the
/// answer is knowable. `MPIDR` is an ARMv7 register; whether an ARMv6
/// core provides anything at `c0, c0, 5` is implementation-defined, and
/// reading a CP15 register a core does not implement is UNPREDICTABLE —
/// on this one, plausibly an undefined-instruction trap in code whose
/// whole job is to answer a question with one possible answer.
#[cfg(armv6)]
#[inline(always)]
pub fn core_id() -> usize {
    0
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

/// The `MIDR` Main ID register: which core this code is running on,
/// straight from the core itself.
///
/// The interesting field is the part number, bits\[15:4\]: `0xb76` is the
/// ARM1176JZF-S (BCM2835 — Pi 1, Pi Zero), `0xc07` the Cortex-A7
/// (BCM2836 — Pi 2), `0xc08` the Cortex-A53 (BCM2837 — Pi 3) and `0xd08`
/// the Cortex-A72 (BCM2711 — Pi 4). Bits\[31:24\] are the implementer
/// (`0x41`, ARM), and the rest are the variant and revision of that
/// particular part.
///
/// Worth printing as the first line out of a new board's console: it
/// says both that the console works and that the chip underneath is the
/// one the binary was built for, which are the two things in doubt at
/// that point.
///
/// `MIDR` is 32 bits on both execution states; the AArch64 register
/// `MIDR_EL1` is 64 bits wide with the top half reserved zero, and this
/// returns the meaningful low half.
#[cfg(target_arch = "arm")]
#[inline(always)]
pub fn main_id() -> u32 {
    let midr: u32;
    // SAFETY: a read of a system register with no side effects, readable
    // at PL1 -- where this crate's code runs -- on every ARM core this
    // crate targets. Unlike `MPIDR` above, `MIDR` is architecturally
    // required rather than an extension, so this arm covers ARMv6 too.
    unsafe { core::arch::asm!("mrc p15, 0, {}, c0, c0, 0", out(reg) midr) };
    midr
}

/// See the AArch32 sibling above.
#[cfg(target_arch = "aarch64")]
#[inline(always)]
pub fn main_id() -> u32 {
    let midr: u64;
    // SAFETY: as above. `MIDR_EL1` is readable at EL1, which is where
    // `boot64.s` leaves every core.
    unsafe { core::arch::asm!("mrs {}, midr_el1", out(reg) midr) };
    midr as u32
}
