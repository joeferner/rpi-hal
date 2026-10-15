// AArch64 exception vector table + IRQ trampoline -- the counterpart to
// vectors.s. Installed via VBAR_EL1 (see boot64.s).
//
// Unlike the 8-entry AArch32 table, an AArch64 table has 16 entries in
// four groups of four (Synchronous, IRQ, FIQ, SError), each entry a
// 128-byte (0x80) aligned block, the whole table 2KB-aligned:
//
//   +0x000  Current EL with SP_EL0
//   +0x200  Current EL with SP_ELx   <- where this kernel runs (EL1h)
//   +0x400  Lower EL using AArch64
//   +0x600  Lower EL using AArch32
//
// This kernel runs at EL1h and never drops to EL0, so only the
// "Current EL with SP_ELx" IRQ slot needs a real handler; everything else
// parks in __unhandled_exception.
.section ".text.vectors"
.align 11
.global __vectors
// Each slot carries its own index into `__unhandled_exception` (in x0),
// because `ESR_EL1` cannot supply it. ESR is written by *synchronous*
// exceptions and by SError; an IRQ or an FIQ arriving here leaves it
// holding whatever the last synchronous exception put there, which reads
// as a confident and completely wrong diagnosis. The slot number is the
// only thing that distinguishes "an interrupt fired with no handler"
// from "a load faulted", and they are not the same bug.
//
// `b`, never `bl`: `ELR_EL1` is where the faulting address lives on this
// architecture, so nothing here needs `x30` -- but leaving it alone
// means the faulting context's link register is still readable, which is
// one frame of backtrace for free.
__vectors:
    // Current EL with SP_EL0 (unused: this kernel runs at EL1h).
    .align 7
    b       __fault_el0_sync
    .align 7
    b       __fault_el0_irq
    .align 7
    b       __fault_el0_fiq
    .align 7
    b       __fault_el0_serror

    // Current EL with SP_ELx (EL1h -- this kernel).
    .align 7
    b       __fault_elx_sync
    .align 7
    b       __irq_trampoline            // IRQ
    .align 7
    b       __fault_elx_fiq
    .align 7
    b       __fault_elx_serror

    // Lower EL using AArch64 (unused: no EL0 code).
    .align 7
    b       __fault_lower64_sync
    .align 7
    b       __fault_lower64_irq
    .align 7
    b       __fault_lower64_fiq
    .align 7
    b       __fault_lower64_serror

    // Lower EL using AArch32 (unused).
    .align 7
    b       __fault_lower32_sync
    .align 7
    b       __fault_lower32_irq
    .align 7
    b       __fault_lower32_fiq
    .align 7
    b       __fault_lower32_serror

// The stubs the table above branches to, outside it because a vector
// slot is 128 bytes and these would otherwise have to fit inside one.
// `kind` is `group << 2 | type`, with group 0-3 in the table's own order
// (current SP_EL0, current SP_ELx, lower AArch64, lower AArch32) and
// type 0 synchronous, 1 IRQ, 2 FIQ, 3 SError. Group 1 is where this
// kernel runs, so a report naming any other group is a kernel that got
// somewhere it has no code for.
__fault_el0_sync:       mov     x0, #0
                        b       __unhandled_exception
__fault_el0_irq:        mov     x0, #1
                        b       __unhandled_exception
__fault_el0_fiq:        mov     x0, #2
                        b       __unhandled_exception
__fault_el0_serror:     mov     x0, #3
                        b       __unhandled_exception
__fault_elx_sync:       mov     x0, #4
                        b       __unhandled_exception
__fault_elx_fiq:        mov     x0, #6
                        b       __unhandled_exception
__fault_elx_serror:     mov     x0, #7
                        b       __unhandled_exception
__fault_lower64_sync:   mov     x0, #8
                        b       __unhandled_exception
__fault_lower64_irq:    mov     x0, #9
                        b       __unhandled_exception
__fault_lower64_fiq:    mov     x0, #10
                        b       __unhandled_exception
__fault_lower64_serror: mov     x0, #11
                        b       __unhandled_exception
__fault_lower32_sync:   mov     x0, #12
                        b       __unhandled_exception
__fault_lower32_irq:    mov     x0, #13
                        b       __unhandled_exception
__fault_lower32_fiq:    mov     x0, #14
                        b       __unhandled_exception
__fault_lower32_serror: mov     x0, #15
                        b       __unhandled_exception

// Weak, for the same reason `__irq_handler` below is: a fault that
// parks silently is indistinguishable from a hang in a driver, a
// deadlock, or a wedged peripheral. A stack overflow is the common way
// to get here -- it runs off the end of the region linker64.ld reserves
// and takes a synchronous exception. An application that defines its
// own `#[no_mangle] extern "C" fn __unhandled_exception()` overrides
// this and can print what happened: `ESR_EL1` gives the exception class
// and `FAR_EL1` the faulting address, with `ELR_EL1` the instruction.
//
// Every slot in the table above reaches `__unhandled_exception` through
// a stub that numbers it, so a handler is `extern "C" fn(kind: u32)` --
// see those stubs for the encoding, and `ESR_EL1`/`FAR_EL1`/`ELR_EL1`
// for what happened within a kind.
//
// Unlike AArch32 there are no banked stacks to prepare: a handler runs
// on the same `SP_EL1` the faulting code was using. That matters most in
// the case most worth reporting -- a stack overflow faults with `sp`
// already past the end of the region, so a handler that pushes anything
// faults again, and the second fault is silent. One that wants to
// survive that has to move `sp` somewhere safe before doing real work.
//
// The symbol itself is defined elsewhere, and exactly one definition is
// in any build: `fault64.s` under the `fault-report` feature, which
// moves `sp` for exactly that reason and then reports, or
// `fault_fallback64.s` without it, which is weak and parks silently so
// an application can override it. They are separate files rather than a
// weak default here and a strong override there because both would be in
// this crate's single stream of `global_asm!`, where the assembler sees
// a duplicate definition rather than a resolvable weak symbol.

.global __irq_trampoline
__irq_trampoline:
    // AArch64 takes no registers automatically on exception entry, so save
    // the caller-saved GP registers (x0-x18) and the link register (x30)
    // that __irq_handler may clobber. Callee-saved registers (x19-x29) are
    // preserved by __irq_handler itself per the C ABI, so they need no
    // saving here. ELR_EL1/SPSR_EL1 hold the return state and are left
    // untouched (IRQ stays masked throughout, so no nested exception can
    // overwrite them).
    stp     x0, x1, [sp, #-160]!
    stp     x2, x3, [sp, #16]
    stp     x4, x5, [sp, #32]
    stp     x6, x7, [sp, #48]
    stp     x8, x9, [sp, #64]
    stp     x10, x11, [sp, #80]
    stp     x12, x13, [sp, #96]
    stp     x14, x15, [sp, #112]
    stp     x16, x17, [sp, #128]
    stp     x18, x30, [sp, #144]

    bl      __irq_handler

    ldp     x2, x3, [sp, #16]
    ldp     x4, x5, [sp, #32]
    ldp     x6, x7, [sp, #48]
    ldp     x8, x9, [sp, #64]
    ldp     x10, x11, [sp, #80]
    ldp     x12, x13, [sp, #96]
    ldp     x14, x15, [sp, #112]
    ldp     x16, x17, [sp, #128]
    ldp     x18, x30, [sp, #144]
    ldp     x0, x1, [sp], #160

    // Restores PSTATE from SPSR_EL1 and branches to ELR_EL1 -- the
    // AArch64 way to return from an exception (vs. AArch32's `movs pc, lr`).
    eret

// Weak default so examples that never enable IRQ don't need to define
// this; a strong `__irq_handler` (e.g. an example's own `#[no_mangle]
// extern "C" fn`) overrides it at link time. Never reached unless
// something unmasks IRQ and enables a source without registering a handler.
.weak __irq_handler
__irq_handler:
    ret
