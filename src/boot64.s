// AArch64 (`rt` feature) boot entry point -- the counterpart to boot.s.
//
// The GPU firmware (with `arm_64bit=1`) releases core 0 here, at EL2,
// with the MMU and caches off. This sets up a stack, drops to EL1,
// installs the exception vectors, brings up the identity-mapped MMU, zeros
// .bss, and calls `kmain` -- the same responsibilities boot.s has on
// AArch32, expressed with AArch64 registers and an EL2->EL1 drop in place
// of boot.s's HYP->SVC drop.
//
// Only core 0 is ever released here; cores 1-3 are held in the firmware's
// own stub (AArch64 secondary-core bring-up is not implemented yet, so
// there is no `__secondary_core_entry` here -- unlike boot.s).

.section ".text.boot"
.global _start

_start:
    // Only core 0 is released to this entry point; park any other core
    // that somehow arrives rather than run core 0's one-time init again.
    // The low bits of MPIDR_EL1 are the core id (Aff0).
    mrs     x1, mpidr_el1
    and     x1, x1, #3
    cbnz    x1, .Lhalt

    // Drop EL2->EL1 if the firmware handed off at EL2 (the default for a
    // 64-bit kernel); fall straight through if already at EL1. This is the
    // AArch64 analog of boot.s's HYP->SVC drop, and everything below runs
    // at EL1.
    mrs     x0, CurrentEL
    lsr     x0, x0, #2
    cmp     x0, #2
    b.ne    .Lel1

    // Configure EL2 so EL1 executes in AArch64 (HCR_EL2.RW = bit 31).
    mrs     x0, hcr_el2
    mov     x1, #0x80000000
    orr     x0, x0, x1
    msr     hcr_el2, x0

    // Give EL1 a known SCTLR before entering it: MMU/caches off (they are
    // enabled below, after the vectors are live), RES1 bits set.
    mov     x0, #0x0800
    movk    x0, #0x30d0, lsl #16
    msr     sctlr_el1, x0

    // Return to EL1h (using SP_EL1) with DAIF masked, at .Lel1.
    mov     x0, #0x3c5
    msr     spsr_el2, x0
    adr     x0, .Lel1
    msr     elr_el2, x0
    eret

.Lel1:
    // Stack grows down from the kernel load address (_start); everything
    // below it is free at this point in boot. Exceptions taken to EL1h run
    // on this same SP_EL1 -- AArch64 has no separate banked IRQ stack like
    // boot.s sets up on AArch32.
    adrp    x0, _start
    add     x0, x0, #:lo12:_start
    mov     sp, x0

    // Point VBAR_EL1 at our exception vector table (vectors64.s) before
    // enabling the MMU, so a fault during MMU bring-up is at least
    // catchable. VBAR works regardless of where this is linked/loaded.
    adrp    x0, __vectors
    add     x0, x0, #:lo12:__vectors
    msr     vbar_el1, x0
    isb

    // Zero .bss BEFORE bringing up the MMU -- the opposite order from
    // boot.s, and it matters: mmu64.rs's translation tables are
    // zero-initialized, so they live in .bss and are filled at runtime by
    // rpi_hal_mmu_init. Zeroing .bss after that call would wipe the
    // just-installed tables, leaving TTBR0 pointing at an all-zero table so
    // the next TLB miss faults -- a silent hang through the (now-live)
    // exception vectors. (boot.s can zero .bss last because its AArch32
    // page table is a const-built `static`, landing in .data instead.)
    adrp    x1, __bss_start
    add     x1, x1, #:lo12:__bss_start
    adrp    x2, __bss_end
    add     x2, x2, #:lo12:__bss_end
    b       3f
2:  str     wzr, [x1], #4
3:  cmp     x1, x2
    b.lo    2b

    // Enable the FP/SIMD unit (clears CPACR_EL1.FPEN's traps) before the
    // first Rust call. It traps out of reset, and a hard-float build (see
    // examples/fpu_demo.rs) may emit FP/SIMD in ordinary code -- including
    // mmu_init below -- so this must run first. Harmless on the default
    // soft-float build. Logic lives in rpi_hal::fpu.
    bl      rpi_hal_fpu_init

    // Build the identity map and enable the MMU + caches (mmu64.rs, or the
    // weak no-op fallback when the `mmu` feature is off). Logic lives in
    // Rust; this is a plain call. Needs the stack set above and .bss zeroed.
    bl      rpi_hal_mmu_init

    bl      kmain

.Lhalt:
    wfe
    b       .Lhalt
