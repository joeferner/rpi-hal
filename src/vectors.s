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
    ldr     pc, =__fault_undefined        // Undefined instruction
    ldr     pc, =__fault_supervisor       // Supervisor call (SWI)
    ldr     pc, =__fault_prefetch_abort   // Prefetch abort
    ldr     pc, =__fault_data_abort       // Data abort
    .word   0                             // Reserved, never taken
    ldr     pc, =__irq_trampoline         // IRQ
    ldr     pc, =__fault_fiq              // FIQ (never unmasked)
.ltorg

// Every faulting slot above lands here rather than branching straight to
// `__unhandled_exception`, so that the handler is told *which* one it
// was. Nothing else can tell it: all four of these enter through
// different vector slots but only two distinct CPSR modes -- a prefetch
// abort and a data abort are both mode 0x17 -- and the two differ in
// which fault registers hold the answer (IFAR/IFSR against DFAR/DFSR)
// and in how far `lr` is biased past the faulting instruction. A handler
// reading the mode alone has to print both pairs and let a person guess.
//
// The kind is the vector's own index, passed in r0 so that
// `__unhandled_exception` can be an ordinary `extern "C"` function of
// one argument. Two properties matter and both are load-bearing:
//
// * `b`, never `bl`. `lr` on entry to `__unhandled_exception` is still
//   the biased faulting address, which is how a handler written before
//   these stubs existed -- taking no argument and reading `lr` itself --
//   goes on working unchanged.
// * r0 only. r0 is call-clobbered in the C ABI, so a handler that
//   ignores the argument loses nothing by it, and every other register
//   the faulting context was using is still what it was.
__fault_undefined:
    mov     r0, #1
    b       __unhandled_exception
__fault_supervisor:
    mov     r0, #2
    b       __unhandled_exception
__fault_prefetch_abort:
    mov     r0, #3
    b       __unhandled_exception
__fault_data_abort:
    mov     r0, #4
    b       __unhandled_exception
__fault_fiq:
    mov     r0, #7
    b       __unhandled_exception

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
// Every slot in the table above shares this one symbol, reached through
// the stubs that number it -- so an override is
// `extern "C" fn(kind: u32)`, with `kind` the vector index (1 undefined,
// 2 supervisor call, 3 prefetch abort, 4 data abort, 7 FIQ). An override
// written against the older signature, taking nothing and reading `lr`,
// still links and still works; see the stubs for why.
//
// `boot.s` gives ABT/UND/FIQ real stacks so that an override can be an
// ordinary Rust function rather than something that has to avoid
// pushing. Those three are shared by every core rather than per-core
// (see boot.s), so two cores faulting at once would overwrite each
// other's frame -- the report of the second is the one to trust.
//
// `rpi-hal`'s own implementation is behind its `fault-report` feature.
// Enabling that *and* defining this symbol is a duplicate definition and
// fails to link, which is the intended way to find out that both were
// asked for.
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
