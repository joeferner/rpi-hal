//! Getting interrupts to the code waiting on them.
//!
//! Every `async` thing in this crate and in `rpi-hal` is parked on a
//! waker that some interrupt has to fire. `rpi-hal`'s `rt` feature
//! installs the vector table, which saves the caller-saved registers and
//! branches to one symbol, `__irq_handler` — and that symbol is the
//! application's to define.
//!
//! # The failure this exists to prevent
//!
//! `rpi-hal`'s default `__irq_handler` is a weak `bx lr`. An application
//! that forgets to define one therefore *links and boots*. The first
//! `embassy-time` deadline then fires, nothing acknowledges the System
//! Timer's Compare 1 match, the interrupt controller goes on asserting,
//! and the core re-enters the handler forever. Every task stops at that
//! instant.
//!
//! It is not a hang in whatever ran last, and it looks exactly like one.
//! Nothing reports it, and there is no link error to catch it — which is
//! why this module offers two ways to be right and no way to be silently
//! wrong.
//!
//! # Which one to use
//!
//! **An application with no interrupt sources of its own** turns on the
//! `irq-dispatch` feature and writes nothing: it defines
//! `__irq_handler` for you, calling [`dispatch`](crate::irq::dispatch).
//! (Qualified because a module's `//!` links are merged with the
//! `pub mod` declaration's and resolve in the crate root's scope, where
//! a bare `dispatch` is not in sight.)
//!
//! **An application with its own sources** leaves the feature off and
//! composes, which is the only way to reach a source this crate has
//! never heard of:
//!
//! ```ignore
//! #[unsafe(no_mangle)]
//! pub extern "C" fn __irq_handler() {
//!     rpi_hal_embassy::irq::dispatch();
//!
//!     let lic = Lic::new(unsafe { pac::Peripherals::steal() }.LIC);
//!     if lic.is_uart_pending() {
//!         my_uart_handler();
//!     }
//! }
//! ```
//!
//! Turning the feature on *and* defining `__irq_handler` is a duplicate
//! definition and fails to link, which is the intended way to find out
//! that both were asked for.

use rpi_hal::lic::Lic;
use rpi_hal::pac;

/// Services every interrupt source this crate and `rpi-hal` know about.
///
/// Safe to call with nothing pending, and safe to call from a handler
/// that goes on to check sources of its own: each source below is gated
/// on its own pending bit, and each entry point it calls is documented as
/// harmless when there is nothing to do.
///
/// # What it covers
///
/// The System Timer's Compare 1 always — that is this crate's
/// `embassy-time` driver, and the one source an application using this
/// crate is certain to have.
///
/// The rest only under the `async` feature, which is what turns on
/// `rpi-hal`'s interrupt-driven drivers in the first place: USB, GPIO
/// edges, I2C, UART and the SD controller. Without it there are no
/// futures parked on any of them and nothing to wake.
///
/// # What it does not cover
///
/// Anything an application drives itself. The sources here are the ones
/// whose wakers live in this crate or in `rpi-hal`; a peripheral the
/// application programmed is one only it can acknowledge, and the
/// composed form in this module's documentation is how that is reached.
///
/// # Why each source is checked, not chained
///
/// More than one can be pending on a single entry, and a handler that
/// stops at the first one leaves the others asserting — which is the
/// same livelock as having no handler at all, arrived at from a handler
/// that looked right. So these are independent `if`s rather than an
/// `else if` chain, and each source is serviced by the crate that owns
/// it.
pub fn dispatch() {
    // Stolen rather than borrowed because a handler is reached from
    // anywhere and owns nothing. Safe for the reason `rpi-hal`'s own
    // interrupt entry points give: this touches pending/enable bits for
    // sources the async layer armed, which no live handle is free to be
    // driving concurrently.
    let lic = Lic::new(unsafe { pac::Peripherals::steal() }.LIC);

    // First, because a deadline that has passed is the thing most likely
    // to have work waiting behind it, and because this is the source
    // that livelocks if it is missed.
    if lic.is_timer1_pending() {
        crate::time_driver::on_timer_irq();
    }

    #[cfg(feature = "async")]
    {
        if lic.is_usb_pending() {
            rpi_hal::usb::dwc2::on_irq();
        }

        // `is_gpio_pending` answers for the bank a pin is in rather than
        // for GPIO as a whole, and there are three: pins 0-27, 28-45 and
        // 46-53. One representative pin from each is how to ask about
        // all of them. `gpio::on_irq` then finds which pin actually
        // fired, so naming a pin here decides nothing but which bank is
        // looked at.
        if lic.is_gpio_pending(0) || lic.is_gpio_pending(28) || lic.is_gpio_pending(46) {
            rpi_hal::gpio::on_irq();
        }

        if lic.is_i2c_pending() {
            rpi_hal::i2c::on_irq();
        }

        if lic.is_uart_pending() {
            rpi_hal::uart::on_irq();
        }

        if lic.is_emmc_pending() {
            rpi_hal::sd::on_irq();
        }
    }
}

/// The `__irq_handler` the vector table branches to, for an application
/// with no interrupt sources of its own.
///
/// Behind `irq-dispatch` rather than unconditional because this is a
/// *definition*, not a hook: an application that has its own
/// `__irq_handler` and turns this on gets a duplicate-symbol link error,
/// and there is no sensible way for the crate to guess which of the two
/// was meant. Opting in is how an application says it has none.
// `unsafe(no_mangle)` rather than the bare form rpi-hal's own examples
// use: this crate is edition 2024, which requires the wrapper.
#[cfg(feature = "irq-dispatch")]
#[unsafe(no_mangle)]
pub extern "C" fn __irq_handler() {
    dispatch();
}
