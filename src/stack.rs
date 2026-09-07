//! Extent of the main stack, and how much of it is left.
//!
//! The stack is a region the linker script reserves and `boot.s` points
//! `sp` at, rather than whatever happened to sit below the load address
//! — so its size is a number in a file (`__stack_size`, 1 MiB by
//! default, overridable with `-Wl,--defsym=__stack_size=...`), the same
//! on both architectures.
//!
//! What this module is for is the question that is otherwise
//! unanswerable from inside a program: *how close am I?* An overflow
//! runs off the bottom of the region and takes a data abort, which
//! without an application-supplied `__unhandled_exception` (see
//! `vectors.s`) parks silently and looks exactly like a hang in a
//! driver. One line at startup —
//!
//! ```ignore
//! writeln!(uart, "sp {:#x}, {} KiB free", stack::pointer(),
//!          stack::headroom().unwrap_or(0) / 1024)?;
//! ```
//!
//! — turns that class of bug from a debugging session into a number.
//!
//! Only core 0's main stack lives in this region. A secondary core runs
//! on the [`Stack<BYTES>`](crate::multicore::Stack) its application
//! supplied, and the AArch32 exception modes run on their own banked
//! regions, which is why [`used`](crate::stack::used) and
//! [`headroom`](crate::stack::headroom) are `Option` — they
//! report `None` rather than a meaningless number when `sp` isn't in
//! this region at all.

extern "C" {
    /// Lowest address of the main stack region. Reserved by the linker
    /// script; `sp` passing below this is an overflow.
    static __stack_bottom: u8;
    /// One past the highest address of the main stack region — the
    /// value `boot.s` loads into `sp`, since the stack is
    /// full-descending.
    static __stack_top: u8;
}

/// Lowest address of the main stack region.
pub fn bottom() -> usize {
    &raw const __stack_bottom as usize
}

/// One past the highest address of the main stack region — where `sp`
/// starts.
pub fn top() -> usize {
    &raw const __stack_top as usize
}

/// Size of the main stack region in bytes (`__stack_size`).
pub fn size() -> usize {
    top() - bottom()
}

/// The calling core's current stack pointer.
///
/// On a secondary core, or in an AArch32 exception mode, this is a
/// pointer into that context's own stack rather than into this region —
/// see this module's doc comment.
pub fn pointer() -> usize {
    let sp: usize;
    // `mov <reg>, sp` is spelled the same on both architectures.
    unsafe {
        core::arch::asm!("mov {}, sp", out(reg) sp, options(nomem, nostack, preserves_flags))
    };
    sp
}

/// Bytes of the main stack currently in use, or `None` if the caller
/// isn't running on it.
pub fn used() -> Option<usize> {
    let sp = pointer();
    (bottom()..=top()).contains(&sp).then(|| top() - sp)
}

/// Bytes between the current stack pointer and the bottom of the main
/// stack — the headroom left before an overflow — or `None` if the
/// caller isn't running on it.
///
/// Worth logging once at startup in any program whose deepest call path
/// isn't obvious: a `rustls` configuration or a mounted FAT volume can
/// put tens of KiB in a single frame.
pub fn headroom() -> Option<usize> {
    let sp = pointer();
    (bottom()..=top()).contains(&sp).then(|| sp - bottom())
}
