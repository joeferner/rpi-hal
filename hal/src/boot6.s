// ARMv6 (ARM1176JZF-S, BCM2835) entry point -- the Pi 1 / Pi Zero
// counterpart to boot.s, which is ARMv7-A.
//
// Same job in the same order (mode stacks, vector table, FPU, MMU, .bss,
// kmain), and mostly the same instructions: `cps`, the banked-mode
// stacks, `wfe` and the vector table are all ARMv6 features that ARMv7
// inherited unchanged. What differs is what is missing, and all of it is
// missing because this core is simpler:
//
//   - No Hyp mode, so no drop out of it. `.arch_extension virt`,
//     `msr ELR_hyp` and `eret` do not assemble for this target at all.
//     Broadcom's firmware leaves this core in Supervisor mode, which is
//     where boot.s's Hyp drop was trying to get to anyway.
//   - No `MPIDR` core-id check. That register is ARMv7; the ARM1176 is a
//     uniprocessor part, so there is no other core that could arrive
//     here and no id to test (see `rpi_hal::cpu::core_id`).
//   - No `__secondary_core_entry`. One core, nothing to release.

// `wfe` in the halt loop below is an ARMv6K instruction, and although
// rustc reports `target_feature = "v6k"` for `armv6-none-eabi`, the
// assembler this file is handed to defaults to plain ARMv6 and rejects
// it ("instruction requires: armv6k"). Widen it to the architecture the
// ARM1176JZF-S actually implements. This only changes what the assembler
// will encode; it does not affect code generation anywhere else.
.arch armv6k

.section ".text.boot"
.global _start

.equ MODE_FIQ, 0x11
.equ MODE_IRQ, 0x12
.equ MODE_SVC, 0x13
.equ MODE_ABT, 0x17
.equ MODE_UND, 0x1b

_start:
    // Main (SVC) mode stack, from the region linker.ld reserves rather
    // than growing down from the load address -- see that script for
    // why the size is stated there instead of being whatever happened
    // to sit below the image.
    ldr     sp, =__stack_top

    // Each privileged mode has its own banked `sp`, and the linker
    // script gives each its own region, adjacent to the main stack
    // rather than carved out of the middle of it. `cps` just switches
    // the mode field, leaving IRQ/FIQ masked as they already are at
    // reset.
    //
    // ABT/UND/FIQ are set up for the same reason IRQ is, even though
    // this crate's default handler for them is a parking loop that
    // touches no memory: an application can override the (weak)
    // `__unhandled_exception` to report the fault, and a Rust function
    // pushes a frame. Without an initialized banked `sp` that push
    // faults again immediately, from a handler whose whole purpose is
    // to say what happened.
    cps     #MODE_IRQ
    ldr     sp, =__irq_stack_top
    cps     #MODE_ABT
    ldr     sp, =__abt_stack_top
    cps     #MODE_UND
    ldr     sp, =__und_stack_top
    cps     #MODE_FIQ
    ldr     sp, =__fiq_stack_top
    cps     #MODE_SVC

    // Point VBAR at our own vector table instead of relying on the
    // SCTLR.V=0 default (fixed low vectors at physical 0x0), exactly as
    // boot.s does -- VBAR works the same regardless of where this code
    // is linked/loaded, which is what keeps this crate free of absolute
    // address assumptions (see linker.ld).
    //
    // VBAR is not an ARMv7 addition: it arrives with the Security
    // Extensions, which the ARM1176JZF-S implements (its CP15 c12 holds
    // the Secure and Non-secure vector base registers and the Monitor
    // one). Firmware hands the core over in Secure state, where this
    // write lands on the Secure copy -- the one in use. If a board ever
    // turns out not to honour it, the symptom is specific and the
    // fallback is the classic Pi 1 one: leave SCTLR.V clear and copy the
    // eight vector words down to physical 0x0 instead.
    ldr     r0, =__vectors
    mcr     p15, 0, r0, c12, c0, 0

    // If SCTLR.V (bit 13) is set, the core ignores VBAR entirely and
    // always vectors through the fixed high address 0xFFFF0000 instead
    // -- we don't know what state incoming GPU firmware left this in,
    // so clear it explicitly rather than assume VBAR above actually
    // takes effect.
    mrc     p15, 0, r0, c1, c0, 0
    bic     r0, r0, #(1 << 13)
    mcr     p15, 0, r0, c1, c0, 0

    // Enable the VFP unit before the first Rust call. It's off out of
    // reset, and a hard-float build may emit FP in ordinary compiled
    // code -- including mmu_init below -- so this has to run first or
    // that code traps. Logic lives in rpi_hal::fpu; this is a plain call
    // into its (naked) enable primitive, whose ARMv6 arm knows this core
    // has VFPv2 and no NEON. Harmless on the default soft-float build.
    bl      rpi_hal_fpu_init

    // Build the identity-mapped page table and enable the MMU --
    // logic lives in Rust (mmu.rs), not here. Runs after VBAR is live
    // (so a fault during this sequence is at least catchable) and before
    // anything below relies on real memory ordering. With the `mmu`
    // feature off this resolves to the weak no-op in mmu_fallback.s.
    bl      rpi_hal_mmu_init

    // Zero .bss
    ldr     r4, =__bss_start
    ldr     r9, =__bss_end
    mov     r5, #0
    mov     r6, #0
    mov     r7, #0
    mov     r8, #0
    b       2f
1:
    stmia   r4!, {{r5-r8}}
2:
    cmp     r4, r9
    blo     1b

    bl      kmain

halt:
    wfe
    b       halt
