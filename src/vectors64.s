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
__vectors:
    // Current EL with SP_EL0 (unused: this kernel runs at EL1h).
    .align 7
    b       __unhandled_exception       // Synchronous
    .align 7
    b       __unhandled_exception       // IRQ
    .align 7
    b       __unhandled_exception       // FIQ
    .align 7
    b       __unhandled_exception       // SError

    // Current EL with SP_ELx (EL1h -- this kernel).
    .align 7
    b       __unhandled_exception       // Synchronous
    .align 7
    b       __irq_trampoline            // IRQ
    .align 7
    b       __unhandled_exception       // FIQ
    .align 7
    b       __unhandled_exception       // SError

    // Lower EL using AArch64 (unused: no EL0 code).
    .align 7
    b       __unhandled_exception       // Synchronous
    .align 7
    b       __unhandled_exception       // IRQ
    .align 7
    b       __unhandled_exception       // FIQ
    .align 7
    b       __unhandled_exception       // SError

    // Lower EL using AArch32 (unused).
    .align 7
    b       __unhandled_exception       // Synchronous
    .align 7
    b       __unhandled_exception       // IRQ
    .align 7
    b       __unhandled_exception       // FIQ
    .align 7
    b       __unhandled_exception       // SError

// Weak, for the same reason `__irq_handler` below is: a fault that
// parks silently is indistinguishable from a hang in a driver, a
// deadlock, or a wedged peripheral. A stack overflow is the common way
// to get here -- it runs off the end of the region linker64.ld reserves
// and takes a synchronous exception. An application that defines its
// own `#[no_mangle] extern "C" fn __unhandled_exception()` overrides
// this and can print what happened: `ESR_EL1` gives the exception class
// and `FAR_EL1` the faulting address, with `ELR_EL1` the instruction.
//
// Every slot in the table above shares this one symbol, so an override
// reads `ESR_EL1` to tell which exception it is. Unlike AArch32 there
// are no banked stacks to prepare -- an override runs on the same
// `SP_EL1` the faulting code was using, which is worth knowing if the
// fault was a stack overflow: the report needs the room the overflow
// just ran out of, so a handler that must survive that case should
// switch `sp` itself before doing real work.
.weak __unhandled_exception
__unhandled_exception:
    wfe
    b       __unhandled_exception

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
