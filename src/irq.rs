//! # Handling an interrupt
//!
//! Three independent gates all have to be open before a handler runs: the
//! CPU-level mask ([`enable_irq`](crate::irq::enable_irq)), the source
//! routed through the interrupt controller (`lic::Lic`), and the
//! peripheral itself configured to raise it.
//!
//! Dispatch is then the application's job, not this crate's. Under the
//! `rt` feature the exception vector table saves the caller-saved
//! registers and branches to one symbol, `__irq_handler`, which the
//! application defines as a plain `extern "C"` function:
//!
//! ```ignore
//! #[no_mangle]
//! pub extern "C" fn __irq_handler() {
//!     let lic = Lic::new(unsafe { pac::Peripherals::steal() }.LIC);
//!
//!     if lic.is_gpio_pending(BUTTON) {
//!         let mut button =
//!             unsafe { Pin::<BUTTON, Input>::assume_mode(pac::Peripherals::steal().GPIO) };
//!         // Clearing the source is what ends this interrupt. Everything
//!         // else -- reading the pin, driving an LED, waking a task --
//!         // is the application's business.
//!         button.clear_interrupt();
//!     }
//! }
//! ```
//!
//! There is no registration call and no table to fill in: a source is
//! live once the interrupt controller routes it, and every live source
//! arrives at this one function, which tests each `is_*_pending` in turn.
//! See `examples/gpio_irq_button.rs` for the whole program, and
//! `examples/uart_rx_irq_echo.rs` for two sources dispatched from one
//! handler.
//!
//! ## Both obligations have the same symptom
//!
//! An interrupt that fires and is never cleared is still asserted when
//! the handler returns, so the core takes it again immediately, forever.
//! The program stops making progress without panicking, without faulting,
//! and without any output — it looks like a hang in whatever ran last,
//! several steps away from the real cause. Two mistakes lead there:
//!
//! - **Not defining `__irq_handler` at all.** The vector table's branch
//!   target is a *weak* no-op, so the link succeeds and the program runs
//!   normally right up until the first interrupt. The strong definition
//!   also has to end up in the final binary: a `#[no_mangle]` function in
//!   a library the binary never references may not be linked in at all,
//!   which leaves the weak stub in place and behaves identically to having
//!   written no handler.
//! - **Returning without clearing the source.** Including the case where
//!   the handler checks the wrong pending bit, or handles one source and
//!   returns while a second is still asserted. Where clearing means
//!   something other than a write-1-to-clear, the driver says so — the ARM
//!   generic timer, for instance, acks a fired tick by moving its
//!   comparator forward rather than by writing a status bit.
//!
//! So when an IRQ-driven program hangs, suspect the handler before
//! anything the code was doing when it stopped.
//!
//! ## Async drivers
//!
//! The `async` feature's futures are woken from the same handler, by
//! calling the driver's `on_irq` (`gpio::on_irq`, `uart::on_irq`) instead
//! of clearing the source inline — those functions clear it and wake the
//! stored waker. An executor doesn't change the contract; it just means
//! the handler's job is usually one call. Drivers outside this crate that
//! need an interrupt expose their own equivalent for the same reason (the
//! `rpi-hal-embassy` crate's time driver is one).
//!
//! ## BCM2711
//!
//! There is no interrupt controller under the `bcm2711` feature yet — the
//! legacy controller this crate wraps isn't the one that chip has, so the
//! `lic` module is compiled out there. This module still builds, since the
//! CPU mask and the vector table are chip-independent, but with no way to
//! route a source there is also no way to make an interrupt fire: the
//! second of the three gates above has nothing behind it until GIC-400
//! support lands.

// The exception vector table is architecture-specific: vectors.s on
// AArch32 (8-entry, VBAR), vectors64.s on AArch64 (16-entry, VBAR_EL1).
#[cfg(target_arch = "arm")]
core::arch::global_asm!(include_str!("vectors.s"));
#[cfg(target_arch = "aarch64")]
core::arch::global_asm!(include_str!("vectors64.s"));

/// Unmasks IRQ at the CPU level (the CPSR `I` bit on AArch32, `PSTATE.I`
/// via `DAIF` on AArch64).
///
/// This is one of three independent gates that all have to be open for
/// an interrupt to actually fire: this CPU-level mask, the source being
/// routed through the interrupt controller (see `crate::lic`), and
/// the peripheral itself being configured to raise it (e.g.
/// [`crate::timer::Timer::arm_periodic_c1`]).
pub fn enable_irq() {
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("cpsie i")
    };
    // DAIFCLR immediate bit 1 is the IRQ (I) mask.
    #[cfg(target_arch = "aarch64")]
    unsafe {
        core::arch::asm!("msr daifclr, #2")
    };
}

/// Masks IRQ at the CPU level.
pub fn disable_irq() {
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("cpsid i")
    };
    // DAIFSET immediate bit 1 is the IRQ (I) mask.
    #[cfg(target_arch = "aarch64")]
    unsafe {
        core::arch::asm!("msr daifset, #2")
    };
}
