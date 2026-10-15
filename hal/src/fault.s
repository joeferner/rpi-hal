// AArch32 entry point for the `fault-report` feature: the strong
// `__unhandled_exception` that overrides the weak one in vectors.s.
//
// It exists in assembly rather than being a `#[no_mangle]` Rust function
// for one reason, and it is the whole of the reason: on entry `lr` holds
// the faulting address, and a Rust function has no way to promise that
// it reads it before something else writes it. The generated prologue
// happens not to touch `lr` today, which is what makes the hand-written
// handlers this replaces work, but "happens not to" is not a property
// that survives a compiler upgrade -- and the failure is a plausible
// wrong address rather than a crash, pointing at whatever this function
// last called. Capturing it in the first instruction removes the
// question.
//
// `spsr` goes with it: it is the CPSR the faulting code was running
// under, and it is banked per mode, so it has to be read here while
// still in the mode the exception entered.
//
// No stack switch, unlike the AArch64 counterpart. boot.s gives ABT, UND
// and FIQ their own banked stacks, so a stack overflow in SVC mode lands
// here with a perfectly good `sp` that has nothing to do with the one
// that overflowed.
.section ".text.fault"
.global __unhandled_exception
__unhandled_exception:
    // r0 is already the vector index, set by the stub in vectors.s.
    mov     r1, lr
    mrs     r2, spsr
    // Tail call: nothing here needs to be resumed, and the Rust side
    // never returns.
    b       rpi_hal_fault_report
