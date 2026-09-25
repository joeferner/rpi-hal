// The boot entry point is architecture-specific: boot.s on ARMv7-A
// AArch32, boot6.s on ARMv6, boot64.s on AArch64 (see each file's header
// for the differences).
#[cfg(all(target_arch = "arm", not(armv6)))]
core::arch::global_asm!(include_str!("boot.s"));
#[cfg(armv6)]
core::arch::global_asm!(include_str!("boot6.s"));
#[cfg(target_arch = "aarch64")]
core::arch::global_asm!(include_str!("boot64.s"));

// See mmu_fallback.s: only included when `mmu` is off, so a build of this
// crate never defines `rpi_hal_mmu_init` more than once itself (mmu.rs
// provides its own strong definition when the feature is on). One file per
// architecture, matching the boot stub above.
#[cfg(all(not(feature = "mmu"), target_arch = "arm"))]
core::arch::global_asm!(include_str!("mmu_fallback.s"));
#[cfg(all(not(feature = "mmu"), target_arch = "aarch64"))]
core::arch::global_asm!(include_str!("mmu_fallback64.s"));

// The strong `__unhandled_exception` behind the `fault-report` feature,
// overriding the weak one in vectors.s/vectors64.s. Each is the entry
// stub for `fault.rs`: see those files for why the capture has to happen
// in assembly rather than in the Rust function they tail-call.
#[cfg(all(feature = "fault-report", target_arch = "arm"))]
core::arch::global_asm!(include_str!("fault.s"));
#[cfg(all(feature = "fault-report", target_arch = "aarch64"))]
core::arch::global_asm!(include_str!("fault64.s"));
