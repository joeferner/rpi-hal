// AArch64 secondary-core entry, reached via the armstub8 spin-table (see
// multicore.rs). A released core arrives here at EL2 with the MMU and
// caches off. It drops to EL1 (same sequence as boot64.s's _start), reads
// its stack pointer and entry point from SECONDARY_PARAMS -- populated and
// cache-cleaned by multicore::launch -- installs the shared exception
// vectors, brings up its own MMU/caches, and jumps to the entry.
//
// Only assembled in when the `multicore` feature is on (included from
// multicore.rs), so SECONDARY_PARAMS and this trampoline exist only then.

.section ".text"
.global __secondary_core_entry
__secondary_core_entry:
    // Drop EL2->EL1 if we arrived at EL2 (the armstub leaves us there),
    // identical to boot64.s's drop.
    mrs     x0, CurrentEL
    lsr     x0, x0, #2
    cmp     x0, #2
    b.ne    .Lsec_el1

    mrs     x0, hcr_el2
    mov     x1, #0x80000000            // HCR_EL2.RW: EL1 is AArch64
    orr     x0, x0, x1
    msr     hcr_el2, x0
    mov     x0, #0x0800
    movk    x0, #0x30d0, lsl #16
    msr     sctlr_el1, x0
    mov     x0, #0x3c5                  // SPSR: DAIF masked, return to EL1h
    msr     spsr_el2, x0
    adr     x0, .Lsec_el1
    msr     elr_el2, x0
    eret

.Lsec_el1:
    // Look up this core's sp and entry in SECONDARY_PARAMS (two u64s per
    // core, indexed by core id). Read with the MMU still off, straight
    // from RAM -- launch() cleaned the cache line before waking us.
    mrs     x0, mpidr_el1
    and     x0, x0, #3
    adrp    x1, SECONDARY_PARAMS
    add     x1, x1, #:lo12:SECONDARY_PARAMS
    add     x1, x1, x0, lsl #4          // + 16 * core_id
    ldr     x2, [x1]                    // sp
    ldr     x19, [x1, #8]               // entry (callee-saved: survives the
                                        // rpi_hal_mmu_init call below)
    mov     sp, x2

    // Enable this core's FP/SIMD unit before the first Rust call -- the
    // enable is per-core (CPACR_EL1 is banked per core), same reasoning as
    // core 0 in boot64.s. x19 (entry) is callee-saved, so it survives this
    // and the mmu_init call below.
    bl      rpi_hal_fpu_init

    // Per-core exception vectors + MMU/caches, exactly as core 0 does in
    // boot64.s. VBAR/TTBR0/SCTLR are per-core, so each core sets its own.
    adrp    x0, __vectors
    add     x0, x0, #:lo12:__vectors
    msr     vbar_el1, x0
    isb
    bl      rpi_hal_mmu_init

    br      x19                         // enter the user's code (never returns)

// sp and entry (two u64s) for cores 0-3, core 0's slot unused. In .bss,
// so it's zeroed by boot64.s before launch() ever writes it.
.section ".bss"
.align 3
.global SECONDARY_PARAMS
SECONDARY_PARAMS:
    .space 64
