//! The three memory barriers, in whichever spelling the target
//! architecture has.
//!
//! `dsb`, `dmb` and `isb` are ARMv7 mnemonics. ARMv6 has the same three
//! barriers but only as CP15 operations (`c7, c10, 4`, `c7, c10, 5` and
//! `c7, c5, 4`), and an assembler targeting ARMv6 rejects the mnemonics
//! outright -- "instruction requires: data-barriers". Since a barrier
//! appears in nearly every architecture-specific file in this crate, the
//! `cfg` lives here once rather than at each of those sites, where the
//! ARMv6 arm would be easy to add to some and forget in others.
//!
//! These are the full-system barriers: no domain or access-type
//! qualifier. ARMv6 has no domain-qualified forms at all -- `dsb sy` is
//! the only thing its CP15 operation can mean -- so the narrower ARMv7
//! and AArch64 variants (`dsb ish`, `dsb ishst`) stay written out at the
//! sites that use them, in modules (`multicore`, `mmu64`,
//! `generic_timer`) that are not built for ARMv6 anyway.

use core::arch::asm;

/// Data synchronization barrier over the full system domain: does not
/// complete until every memory access and cache/TLB maintenance
/// operation issued by this core before it has completed.
///
/// This is the barrier that makes cache maintenance (`crate::cache`)
/// actually finished before a bus master is told to run, and the
/// ordering barrier for the VideoCore shared-memory protocols
/// (`crate::vchiq`), whose region is mapped non-cacheable rather than
/// maintained by hand: no maintenance is needed there, but plain
/// Normal-memory accesses can still be reordered and merged, so
/// publishing a structure before the flag that advertises it still needs
/// this.
#[inline(always)]
pub(crate) fn dsb() {
    #[cfg(all(target_arch = "arm", not(armv6)))]
    unsafe {
        asm!("dsb")
    };
    // The register operand is ignored (should-be-zero), but the
    // instruction encoding still names one.
    #[cfg(armv6)]
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c10, 4", in(reg) 0u32)
    };
    #[cfg(target_arch = "aarch64")]
    unsafe {
        asm!("dsb sy")
    };
}

/// Data memory barrier over the full system domain: orders the memory
/// accesses either side of it, without waiting for them to complete the
/// way [`dsb`] does.
// Nothing in this crate needs the weaker barrier today -- every site
// wants completion, not just ordering. Kept anyway so the set is
// complete: a barrier module missing one of the three invites a bare
// `dmb` written inline at the first site that wants one, which is
// exactly the ARMv6 trap this module exists to close.
#[allow(dead_code)]
#[inline(always)]
pub(crate) fn dmb() {
    #[cfg(all(target_arch = "arm", not(armv6)))]
    unsafe {
        asm!("dmb")
    };
    #[cfg(armv6)]
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c10, 5", in(reg) 0u32)
    };
    #[cfg(target_arch = "aarch64")]
    unsafe {
        asm!("dmb sy")
    };
}

/// Instruction synchronization barrier: flushes the pipeline, so
/// instructions after it are fetched and decoded against whatever system
/// state was just written -- a new translation table, a newly enabled
/// MMU or cache, a coprocessor that has just been granted access.
// Used only from `mmu32.rs`, which the `mmu` feature gates -- so with
// that feature off this is genuinely unused rather than merely
// unreferenced yet.
#[allow(dead_code)]
#[inline(always)]
pub(crate) fn isb() {
    #[cfg(all(target_arch = "arm", not(armv6)))]
    unsafe {
        asm!("isb")
    };
    #[cfg(armv6)]
    unsafe {
        asm!("mcr p15, 0, {0}, c7, c5, 4", in(reg) 0u32)
    };
    #[cfg(target_arch = "aarch64")]
    unsafe {
        asm!("isb")
    };
}

#[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
compile_error!("rpi-hal supports only ARM (AArch32) and AArch64 targets");
