.arch_extension virt

.section ".text.boot"
.global _start

.equ MODE_HYP, 0x1a
.equ MODE_FIQ, 0x11
.equ MODE_IRQ, 0x12
.equ MODE_SVC, 0x13
.equ MODE_ABT, 0x17
.equ MODE_UND, 0x1b

_start:
    // Hypothesis under test: Broadcom's firmware on this board hands
    // off to the kernel still in Hyp mode, not the classic SVC mode
    // almost every bare-metal tutorial assumes — a documented quirk on
    // some Pi 2/3-era firmware. This would explain why IRQ never
    // reached our handler despite VBAR/LIC/CPU-mask all being set up
    // correctly: Hyp mode has its own separate vector base (HVBAR) and
    // its own banked registers, so all of that setup would have been
    // silently talking to the wrong mode's state. Hyp mode can only be
    // exited via a real exception return (`eret`) — the `cps`/`movs
    // pc, lr` tricks used for every other privileged mode are
    // UNPREDICTABLE here per the architecture — so drop to SVC first,
    // before anything else, if that's where we are. Harmless no-op if
    // we're already in SVC mode (the `bne` below just skips it).
    mrs     r0, cpsr
    and     r1, r0, #0x1f
    cmp     r1, #MODE_HYP
    bne     .Lhyp_drop_done
    bic     r0, r0, #0x1f
    orr     r0, r0, #MODE_SVC
    msr     spsr_cxsf, r0
    adr     lr, .Lhyp_drop_done
    msr     ELR_hyp, lr
    eret
.Lhyp_drop_done:

    // Only core 0 ever reaches this entry point: the GPU firmware
    // releases core 0 here but holds cores 1-3 in its own stub (see
    // `__secondary_core_entry` / `rpi_hal::multicore`). Any other core
    // arriving here is unexpected -- park it rather than run core 0's
    // one-time init a second time.
    mrc     p15, 0, r1, c0, c0, 5
    and     r1, r1, #3
    cmp     r1, #0
    bne     halt

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
    // SCTLR.V=0 default (fixed low vectors at physical 0x0). VBAR
    // works the same regardless of where this code is linked/loaded —
    // consistent with not hardcoding absolute-address assumptions (see
    // linker.ld / rpi-loader's linker.ld for why that matters here).
    ldr     r0, =__vectors
    mcr     p15, 0, r0, c12, c0, 0

    // If SCTLR.V (bit 13) is set, the core ignores VBAR entirely and
    // always vectors through the fixed high address 0xFFFF0000 instead
    // — we don't know what state incoming GPU firmware left this in,
    // so clear it explicitly rather than assume VBAR above actually
    // takes effect.
    mrc     p15, 0, r0, c1, c0, 0
    bic     r0, r0, #(1 << 13)
    mcr     p15, 0, r0, c1, c0, 0

    // Enable the VFP/NEON unit before the first Rust call. It's off out
    // of reset, and a hard-float build (see examples/fpu_demo.rs) may
    // emit FP/SIMD in ordinary compiled code -- including mmu_init below
    // -- so this has to run first or that code traps. Logic lives in
    // rpi_hal::fpu; this is a plain call into its (naked) enable
    // primitive. Harmless on the default soft-float build.
    bl      rpi_hal_fpu_init

    // Build the identity-mapped page table and enable the MMU --
    // logic lives in Rust (mmu.rs), not here: this is a plain function
    // call, not new inline assembly. Runs after VBAR is live (so a
    // fault during this sequence is at least catchable) and before
    // anything below relies on real memory ordering.
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

// A secondary core (1-3) starts executing here once
// `rpi_hal::multicore::spawn` releases it by writing its ARM-local
// mailbox 3 (the GPU firmware's own stub, which held the core until
// then, jumps it to whatever address that mailbox holds -- see the
// `multicore` module). spawn also left this core's stack pointers and
// entry point in mailboxes 0-2; read them straight back (Device
// memory, so no cache-coherency dance with core 0's writes), set up,
// and jump to the entry point.
.global __secondary_core_entry
__secondary_core_entry:
    // Enable this core's VFP/NEON unit first -- the enable is per-core,
    // and (as on core 0 in _start) it must precede any Rust call in case
    // a hard-float build emitted FP in it. Safe here before the stack is
    // set: the primitive is a naked leaf that touches no memory.
    bl      rpi_hal_fpu_init

    // Core id (1-3) -> r1.
    mrc     p15, 0, r1, c0, c0, 5
    and     r1, r1, #3

    // This core's mailbox READ base = 0x4000_00C0 + 0x10*core (the read
    // side of the write-set registers spawn used at 0x4000_0080).
    ldr     r2, =0x400000C0
    add     r2, r2, r1, lsl #4
    ldr     r3, [r2]               // mailbox 0: sp_main
    ldr     r5, [r2, #4]           // mailbox 1: sp_irq
    ldr     r6, [r2, #8]           // mailbox 2: entry

    // IRQ-mode banked stack, then back to SVC with the main stack --
    // same split core 0 sets up in _start. Both come from the caller's
    // `Stack<BYTES>` rather than the linker script: a secondary core's
    // stack is supplied by the application (see `multicore::Stack`),
    // and only core 0 runs on the reserved region.
    cps     #MODE_IRQ
    mov     sp, r5
    cps     #MODE_SVC
    mov     sp, r3

    // ABT/UND/FIQ do come from the linker script, and so are shared
    // with core 0 and with each other core. That is a deliberate
    // trade: two cores taking a fault at the same instant would
    // overwrite each other's report, but a fault is terminal anyway,
    // and the alternative -- an uninitialized banked `sp` -- turns any
    // application-supplied `__unhandled_exception` into a second fault
    // that reports nothing at all. Carving them out of the
    // application's `Stack<BYTES>` instead isn't an option: those are
    // sized for the core's own work (8 KiB in some examples), not for
    // three more mode stacks.
    cps     #MODE_ABT
    ldr     sp, =__abt_stack_top
    cps     #MODE_UND
    ldr     sp, =__und_stack_top
    cps     #MODE_FIQ
    ldr     sp, =__fiq_stack_top
    cps     #MODE_SVC

    // VBAR + SCTLR.V are banked per-core, so this core programs its own
    // -- identical to core 0's sequence in _start.
    ldr     r4, =__vectors
    mcr     p15, 0, r4, c12, c0, 0
    mrc     p15, 0, r4, c1, c0, 0
    bic     r4, r4, #(1 << 13)
    mcr     p15, 0, r4, c1, c0, 0

    // Identity MMU + caches + ACTLR.SMP for this core (safe to call
    // per-core -- see rpi_hal_mmu_init's doc comment). It's a function
    // call, so it needs the SVC stack set above; r6 (entry) is
    // caller-saved under AAPCS, so preserve it across the call.
    mov     r7, r6
    bl      rpi_hal_mmu_init
    mov     r6, r7

    bx      r6
    b       halt
