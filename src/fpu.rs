//! Enables the CPU's hardware floating-point / SIMD unit (VFP + NEON).
//!
//! Both the Cortex-A7 (BCM2836) and Cortex-A53 (BCM2837) ship with the
//! FP/SIMD unit disabled out of reset: executing any VFP or NEON
//! instruction before enabling coprocessor access traps -- an undefined
//! instruction on AArch32, an EL1 FP/SIMD trap on AArch64.
//!
//! `rt`'s boot sequence calls [`enable`](crate::fpu::enable) (via the
//! `rpi_hal_fpu_init` symbol below) on *every* core before any Rust code runs
//! -- the primary
//! core in `boot.s`/`boot64.s` and each secondary in its bring-up path --
//! so even a hard-float build, where the compiler may emit FP/SIMD
//! instructions in ordinary code (register spills, `memcpy`), cannot
//! execute one ahead of the enable.
//!
//! The default targets are soft-float, so ordinary float arithmetic
//! lowers to `compiler_builtins` software routines and never touches the
//! unit regardless: enabling it costs nothing until something is built
//! against a hard-float target (`armv7a-none-eabihf` /
//! `aarch64-unknown-none`). See `examples/fpu_demo.rs` for such a build,
//! including how to confirm hardware FP opcodes are actually emitted.
//!
//! One thing to keep in mind if that ever changes: with the unit enabled,
//! the FP/SIMD registers become live state. Nothing in this crate saves
//! or restores them across an interrupt today, which is fine because the
//! IRQ path is integer-only. If FP is ever used inside an interrupt
//! handler, or a preemptive scheduler is added that switches between
//! tasks that use FP, the ~32 D-registers + `FPSCR`/`FPCR` would need
//! saving on that boundary.

/// Enables the VFP/NEON floating-point unit on the calling core.
///
/// Idempotent and safe to call from any core at PL1 (AArch32) or EL1
/// (AArch64). `rt`'s boot code already calls this on every core before
/// `kmain`, so an application using the default features never needs to.
/// A consumer supplying their own boot sequence
/// (`default-features = false`) must call it themselves, on each core,
/// before running any hard-float code.
pub fn enable() {
    // SAFETY: touches only the coprocessor-access control registers
    // (`CPACR`/`FPEXC` on AArch32, `CPACR_EL1` on AArch64) of the calling
    // core -- no memory and no other observable state.
    unsafe { rpi_hal_fpu_init() }
}

/// The enable primitive, also called directly from the boot stubs
/// (`boot.s`/`boot64.s`/`secondary64.s`) via its unmangled name, before
/// the first Rust call. `#[naked]` so its body is exactly the assembly
/// below with no compiler-inserted prologue/epilogue -- on a hard-float
/// build that guarantees the routine that *turns FP on* cannot itself be
/// the first thing to touch an FP register.
#[cfg(target_arch = "arm")]
#[unsafe(naked)]
#[no_mangle]
unsafe extern "C" fn rpi_hal_fpu_init() {
    core::arch::naked_asm!(
        // The assembler needs a VFP/NEON unit selected to accept the
        // `vmsr` below; the directive only widens what encodes, it does
        // not force FP codegen.
        ".fpu neon-vfpv4",
        // CPACR (CP15 c1,c0,2): grant CP10 and CP11 (the VFP/NEON
        // coprocessors) full access at PL0 and PL1 -- bits [23:20] = 1111.
        "mrc p15, 0, r0, c1, c0, 2",
        "orr r0, r0, #(0xf << 20)",
        "mcr p15, 0, r0, c1, c0, 2",
        "isb",
        // FPEXC.EN (bit 30): the VFP extension's master enable, writable
        // only now that CP10/11 access has been granted above.
        "mov r0, #(1 << 30)",
        "vmsr fpexc, r0",
        "bx lr",
    )
}

/// See the AArch32 sibling above.
#[cfg(target_arch = "aarch64")]
#[unsafe(naked)]
#[no_mangle]
unsafe extern "C" fn rpi_hal_fpu_init() {
    core::arch::naked_asm!(
        // CPACR_EL1.FPEN (bits [21:20]) = 0b11: do not trap FP/SIMD
        // instructions at either EL0 or EL1.
        "mrs x0, cpacr_el1",
        "mov x1, #(0b11 << 20)",
        "orr x0, x0, x1",
        "msr cpacr_el1, x0",
        "isb",
        "ret",
    )
}
