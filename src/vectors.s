// ARM exception vector table + IRQ trampoline. Installed via VBAR
// (see boot.s) rather than relying on the fixed low-vectors address
// 0x00000000, so this works the same regardless of where this code is
// linked/loaded.
//
// Standard 8-entry ARM vector table: each slot is a PC-relative load
// from a nearby literal (the assembler manages the exact offset/pool
// placement for `ldr pc, =label`), so unlike the rest of this
// project's hand-computed addressing, there's no manual offset to get
// wrong here.
.section ".text.vectors"
.align 5
.global __vectors
__vectors:
    ldr     pc, =_start                   // Reset (unused: VBAR is
                                           // programmed by boot.s
                                           // before this table is
                                           // reachable)
    ldr     pc, =__unhandled_exception    // Undefined instruction
    ldr     pc, =__unhandled_exception    // Supervisor call (SWI)
    ldr     pc, =__unhandled_exception    // Prefetch abort
    ldr     pc, =__unhandled_exception    // Data abort
    .word   0                             // Reserved, never taken
    ldr     pc, =__irq_trampoline         // IRQ
    ldr     pc, =__unhandled_exception    // FIQ (never unmasked)
.ltorg

// Weak, for the same reason `__irq_handler` below is: a fault that
// parks silently is indistinguishable from a hang in a driver, a
// deadlock, or a wedged peripheral. A stack overflow is the common way
// to get here -- it runs off the end of the reserved region and takes a
// data abort -- and finding that out has cost this project a debugging
// session that ruled out three peripherals first. An application that
// defines its own `#[no_mangle] extern "C" fn __unhandled_exception()`
// overrides this and can print what happened: `lr` is the faulting
// address (biased by the exception type), and `DFAR`/`DFSR` (data
// abort) or `IFAR`/`IFSR` (prefetch abort) say where and why.
//
// Every slot in the table above shares this one symbol, so an override
// has to read the current mode from `CPSR` to tell which exception it
// is. `boot.s` gives ABT/UND/FIQ real stacks so that an override can be
// an ordinary Rust function rather than something that has to avoid
// pushing.
.weak __unhandled_exception
__unhandled_exception:
    wfe
    b       __unhandled_exception

.global __irq_trampoline
__irq_trampoline:
    // The IRQ return address is one instruction ahead of where
    // execution should resume (architectural quirk of this exception
    // type); back it up before saving so `movs pc, lr` below resumes
    // at the correct instruction.
    sub     lr, lr, #4
    push    {{r0-r12, lr}}
    bl      __irq_handler
    pop     {{r0-r12, lr}}
    // Copies SPSR_irq back into CPSR and branches to lr in one step —
    // the standard ARM idiom for returning from an exception handler.
    movs    pc, lr

// Weak default so examples that never enable IRQ (the vast majority)
// don't need to define this themselves — a strong `__irq_handler`
// defined elsewhere (e.g. in an example, as an ordinary `#[no_mangle]
// extern "C" fn`) overrides this at link time, standard ELF weak-symbol
// semantics. Never actually reached unless something unmasks IRQ and
// enables a source without registering a real handler.
.weak __irq_handler
__irq_handler:
    bx      lr
