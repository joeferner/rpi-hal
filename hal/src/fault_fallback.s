// The AArch32 `__unhandled_exception` used when the `fault-report`
// feature is off: park the core and say nothing.
//
// Its own file, and included only when that feature is off, for the
// reason `mmu_fallback.s` is: two definitions of one symbol in this
// crate's single stream of `global_asm!` are a duplicate definition to
// the assembler, not a weak symbol for the linker to resolve. Whether it
// complained depended on which codegen unit each landed in, which is to
// say it was luck.
//
// Weak, so an application can still define its own
// `#[no_mangle] extern "C" fn __unhandled_exception(kind: u32)` and
// override this -- which is the whole reason the default is as useless
// as it is. A fault that parks silently is indistinguishable from a hang
// in a driver, a deadlock or a wedged peripheral, and finding that out
// has cost this project a session that ruled out three peripherals
// first. `lr` is the faulting address (biased by exception type), and
// `DFAR`/`DFSR` (data abort) or `IFAR`/`IFSR` (prefetch abort) say where
// and why. `rpi-hal`'s own implementation of all that is the
// `fault-report` feature; this is what is there instead of it.
.section ".text.fault"
.weak __unhandled_exception
__unhandled_exception:
    wfe
    b       __unhandled_exception
