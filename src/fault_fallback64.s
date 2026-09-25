// The AArch64 `__unhandled_exception` used when the `fault-report`
// feature is off: park the core and say nothing. The counterpart to
// fault_fallback.s -- see that file for why this is a separate file
// rather than a weak default sitting in vectors64.s.
//
// Weak, so an application can define its own
// `#[no_mangle] extern "C" fn __unhandled_exception(kind: u32)` and
// override it. One written for this architecture should move `sp` before
// doing anything: there are no banked stacks here, so a handler entered
// because the stack overflowed faults again on its first push, and that
// second fault is the silent one. `fault64.s`, under the `fault-report`
// feature, is what does that.
.section ".text.fault"
.weak __unhandled_exception
__unhandled_exception:
    wfe
    b       __unhandled_exception
