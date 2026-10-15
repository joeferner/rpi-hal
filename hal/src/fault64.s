// AArch64 entry point for the `fault-report` feature: the strong
// `__unhandled_exception` that overrides the weak one in vectors64.s.
//
// Assembly rather than a `#[no_mangle]` Rust function because of the
// stack. There are no banked stacks on this architecture -- an exception
// taken at EL1h keeps using SP_EL1 -- so a handler entered because the
// stack overflowed is a handler whose own first push faults again. That
// second fault is taken with the vectors still installed, so it lands
// back here, and the result is a silent loop at the exact moment the
// report was worth the most.
//
// So `sp` moves to a small dedicated region below before anything else
// happens. The faulting `sp` is captured first and passed on, since it
// is the number that says how far past the end the overflow got.
//
// ELR_EL1/FAR_EL1/ESR_EL1 are read on the Rust side instead: they are
// ordinary system registers that survive a stack switch, and none of
// them is clobbered by anything between here and there.
.section ".text.fault"
.global __unhandled_exception
__unhandled_exception:
    // x0 is already the vector index, set by the stub in vectors64.s.
    mov     x1, sp
    adrp    x2, __rpi_hal_fault_stack_top
    add     x2, x2, #:lo12:__rpi_hal_fault_stack_top
    mov     sp, x2
    // Tail call: nothing here is resumable, and the Rust side never
    // returns.
    b       rpi_hal_fault_report

// The stack the report is built on.
//
// 4 KiB, which is enough for the formatting below it and deliberately
// not enough to hide a problem: nothing here recurses, allocates, or
// calls into a driver beyond writing bytes to a UART.
//
// In .bss rather than .data so it costs nothing in the kernel image --
// an image that on this project is routinely shipped over a 115200 baud
// UART -- and it is never read before it is written.
//
// One region for every core, which is the same trade boot.s makes for
// the AArch32 banked stacks: two cores faulting at once overwrite each
// other's frame. The alternative is a region per core sized at build
// time for a number of cores nothing else here needs to know, to make a
// simultaneous double fault legible. The second report is the one to
// trust.
.section ".bss.fault", "aw", %nobits
.align 4
__rpi_hal_fault_stack:
    .space 4096
__rpi_hal_fault_stack_top:
