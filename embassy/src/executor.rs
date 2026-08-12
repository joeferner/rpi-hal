//! Thread-mode executor, idling in `wfe` and woken by `sev`.
//!
//! `embassy-executor`'s scheduler is portable; what it needs per platform
//! is two things — a `__pender` that makes an idle core look at the run
//! queue again, and an idle instruction to spend the time in between.
//! This module supplies both, identically for AArch32 and AArch64.
//!
//! # Why not `embassy-executor`'s own backend
//!
//! `embassy-executor` has a `platform-cortex-ar` feature whose executor is
//! `sev` to pend and `wfe` to idle — exactly the sequence below, which is
//! good evidence this is the blessed approach for the architecture rather
//! than an invention here. It is AArch32 only, though (it depends on
//! `aarch32-cpu`), and there is no AArch64 backend, which would leave the
//! 64-bit build on `platform-spin`: a busy-loop that never idles the CPU
//! at all.
//!
//! Two executors with two different idle behaviors, in a crate whose point
//! is that both word sizes behave the same, is a worse trade than the
//! sixty lines here. `embassy_executor::raw` is public precisely so that a
//! HAL can do this.
//!
//! # Using it
//!
//! The executor has to outlive `run`, which is why it takes
//! `&'static mut self`. `static_cell` is the usual way to get one safely;
//! a `fn kmain() -> !` that never returns can also transmute a local's
//! lifetime, which is what `#[embassy_executor::main]` expands to.
//!
//! That attribute works here, in the form
//! `#[embassy_executor::main(executor = "rpi_hal_embassy::Executor")]` —
//! with no `platform-*` feature selected it resolves to the macro's
//! architecture-agnostic variant, which takes the executor path as an
//! argument. It also emits a plain `fn main`, though, and `rpi-hal`'s boot
//! code calls `kmain`, so it needs an `entry` attribute macro that
//! `rpi-hal` does not currently provide. Until then, call
//! [`Executor::run`](crate::executor::Executor::run) from `kmain`
//! directly — which is all the macro does anyway.

use core::marker::PhantomData;

use embassy_executor::{Spawner, raw};

/// Wakes a core that may be sitting in `wfe`.
///
/// `embassy-executor` calls this after putting a task on the run queue.
/// The context pointer distinguishes executors when there is more than
/// one; with a single thread-mode executor there is nothing to
/// distinguish, so it is ignored.
#[unsafe(export_name = "__pender")]
fn __pender(_context: *mut ()) {
    // SAFETY: `dsb`/`sev` touch no memory and have no operands; both are
    // unprivileged and valid on ARMv7-A and ARMv8-A alike.
    //
    // `dsb ish` before `sev` is ARM's prescribed order for signalling
    // another core: the run-queue stores this event is announcing have to
    // be observable before the event that sends someone looking for them.
    // Strictly redundant while only one core runs an executor — program
    // order already covers a core waking itself — but this is the
    // sequence that stays correct when a second one does, and it is
    // cheap next to the queue's own atomics.
    unsafe { core::arch::asm!("dsb ish", "sev", options(nomem, nostack)) };
}

/// Thread-mode executor: runs tasks on the main context, and idles the
/// core in `wfe` when none are ready.
///
/// `wfe` parks the core until an event or an interrupt arrives, so an
/// idle program draws no more power than a `wfi` loop — unlike a spinning
/// executor. Interrupts wake it regardless of the event register, which
/// is what lets the timer interrupt drive `embassy-time` deadlines.
///
/// There is no lost-wake-up race between polling and sleeping. `sev` sets
/// a sticky per-core event register, so a wake-up that lands in the window
/// after the last `poll` and before the `wfe` makes that `wfe` return
/// immediately rather than sleeping through it.
pub struct Executor {
    inner: raw::Executor,
    /// The executor must stay on the core that created it: `run` polls
    /// from one context, and the pender's `wfe`/`sev` pairing is per-core.
    not_send: PhantomData<*mut ()>,
}

// `Default` is not offered alongside `new`: an `Executor` is only usable
// through `&'static mut`, so a value produced by `Default::default()` in
// an expression position could never be run, and offering it would suggest
// otherwise.
#[allow(clippy::new_without_default)]
impl Executor {
    /// Creates an executor with no tasks.
    pub fn new() -> Self {
        Self {
            // No context to pass: this crate has one pender and one
            // executor, so there is nothing for the pointer to select
            // between. Per-core executors would give each its own.
            inner: raw::Executor::new(core::ptr::null_mut()),
            not_send: PhantomData,
        }
    }

    /// Spawns the initial tasks and then runs them forever.
    ///
    /// `init` receives a [`Spawner`] for this executor and is called once,
    /// before the first poll; keep a copy of the `Spawner` (it is `Copy`)
    /// to spawn more later from inside a task.
    ///
    /// Never returns. Between polls the core sits in `wfe`, so anything
    /// that should wake it — including the `embassy-time` deadline
    /// interrupt — must be enabled before this is called.
    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        init(self.inner.spawner());

        loop {
            // SAFETY: `poll` must not be re-entered. This is the only call
            // site, it is not reachable from a task, and `Executor` is
            // `!Send`, so no other core can be polling this one.
            unsafe { self.inner.poll() };

            // SAFETY: `wfe` is unprivileged and touches no memory. A
            // pending event or interrupt makes it return at once, so this
            // cannot park a core that has work waiting.
            unsafe { core::arch::asm!("wfe", options(nomem, nostack)) };
        }
    }
}
