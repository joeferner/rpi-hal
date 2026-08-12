//! `embedded-hal-async`'s [`Wait`] for GPIO pins, driven by the same
//! event-detect hardware the blocking interrupt API uses.
//!
//! # Wiring
//!
//! Nothing here resolves until the pin's interrupt reaches [`on_irq`].
//! That means the same three gates every interrupt source in this crate
//! goes through — the peripheral (opened by the future itself), the
//! interrupt controller
//! (`Lic::enable_gpio_irq`), and the
//! CPU mask ([`enable_irq`](crate::irq::enable_irq)) — plus a call to
//! [`on_irq`] from the application's `__irq_handler`. A library crate
//! can't claim that symbol, so dispatch stays the application's.
//!
//! # Coexisting with hand-written handlers
//!
//! [`on_irq`] only touches pins that a future is currently waiting on. A
//! pin armed by [`Pin::enable_interrupt`] and serviced by hand is left
//! entirely alone — its event stays latched for the application's own
//! code to ack — so the two styles can share one handler and one bank
//! interrupt.
//!
//! [`Wait`]: embedded_hal_async::digital::Wait
//! [`on_irq`]: crate::gpio::on_irq
//! [`Pin::enable_interrupt`]: crate::gpio::Pin::enable_interrupt

use core::cell::RefCell;
use core::future::Future;
use core::task::{Context, Poll, Waker};

use critical_section::Mutex;

use super::{clear_event, read_level, set_trigger, Input, Pin, Trigger};
use crate::pac::GPIO;

/// GPIO pins on this SoC, 0-53.
const NUM_PINS: usize = 54;

/// Per-pin handshake between a waiting future and [`on_irq`].
enum State {
    /// Nothing waiting. [`on_irq`] leaves this pin alone entirely, which
    /// is what lets hand-written handlers keep working.
    Idle,
    /// A future is parked on this pin.
    Waiting(Waker),
    /// The event arrived and the waker has been woken; the next poll
    /// consumes this. Needed because acking the event destroys the only
    /// evidence it happened — an edge leaves nothing behind to re-read.
    Fired,
}

static STATES: Mutex<RefCell<[State; NUM_PINS]>> =
    Mutex::new(RefCell::new([const { State::Idle }; NUM_PINS]));

/// Every pin with an event latched in GPEDS, as a bitmask.
fn pending_mask(gpio: &GPIO) -> u64 {
    let low = gpio.gpeds0().read().bits() as u64;
    let high = gpio.gpeds1().read().bits() as u64;
    low | (high << 32)
}

/// Services GPIO events for pins with a future waiting on them: acks
/// each one, disarms its detector, and wakes the future.
///
/// Call this from the application's `__irq_handler` when
/// `Lic::is_gpio_pending` reports a
/// pin in a bank an async wait is using. Harmless to call spuriously —
/// with nothing waiting it does nothing at all.
///
/// Disarming on fire is what makes a level trigger usable here: GPEDS
/// re-latches for as long as the pin sits at the level, so acking alone
/// would re-enter the handler forever. Since every wait is one-shot, the
/// detector has done its job the moment it fires.
pub fn on_irq() {
    // Safe to steal: this only touches event-detect and status registers
    // for pins the async layer armed, which no `Pin` handle is free to be
    // driving concurrently — a pin whose future is parked is borrowed by
    // that future.
    let gpio = unsafe { GPIO::steal() };

    let pending = pending_mask(&gpio);
    if pending == 0 {
        return;
    }

    critical_section::with(|cs| {
        let mut states = STATES.borrow_ref_mut(cs);

        for pin in 0..NUM_PINS as u8 {
            if pending & (1u64 << pin) == 0 {
                continue;
            }

            let slot = &mut states[pin as usize];
            if !matches!(slot, State::Waiting(_)) {
                // Not ours. Leave the event latched so a hand-written
                // handler still sees it.
                continue;
            }

            set_trigger(&gpio, pin, None);
            clear_event(&gpio, pin);

            if let State::Waiting(waker) = core::mem::replace(slot, State::Fired) {
                waker.wake();
            }
        }
    });
}

/// A pending wait for one event on pin `N`.
///
/// Arms the detector on first poll rather than at construction, so a
/// future that is built and dropped without ever being polled leaves the
/// hardware untouched.
struct WaitForEvent<'a, const N: u8> {
    pin: &'a Pin<N, Input>,
    trigger: Trigger,
    armed: bool,
}

impl<const N: u8> Future for WaitForEvent<'_, N> {
    type Output = ();

    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        let this = self.get_mut();

        critical_section::with(|cs| {
            let mut states = STATES.borrow_ref_mut(cs);
            let slot = &mut states[N as usize];

            if matches!(slot, State::Fired) {
                *slot = State::Idle;
                // Already disarmed by the handler, so `Drop` has nothing
                // left to undo.
                this.armed = false;
                return Poll::Ready(());
            }

            // Re-register every poll: a task can be moved between wakers,
            // and the stored one must be the current one.
            *slot = State::Waiting(cx.waker().clone());

            if !this.armed {
                set_trigger(&this.pin.gpio, N, Some(this.trigger));
                this.armed = true;
            }

            Poll::Pending
        })
    }
}

impl<const N: u8> Drop for WaitForEvent<'_, N> {
    /// Cancellation has to put the hardware back: a dropped future that
    /// left its detector armed would fire into a `State` no one is
    /// waiting on, and — worse for a level trigger — keep re-entering the
    /// handler.
    fn drop(&mut self) {
        if !self.armed {
            return;
        }

        set_trigger(&self.pin.gpio, N, None);
        critical_section::with(|cs| {
            STATES.borrow_ref_mut(cs)[N as usize] = State::Idle;
        });
    }
}

impl<const N: u8> embedded_hal_async::digital::Wait for Pin<N, Input> {
    /// Resolves as soon as the pin reads high, immediately if it already
    /// does.
    ///
    /// Uses the level detector rather than a rising edge, which closes
    /// the gap between testing the level and arming: GPHEN latches on the
    /// level itself, so a pin that goes high in that window still
    /// registers, where an edge trigger would have missed the transition.
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        if read_level(&self.gpio, N) {
            return Ok(());
        }
        WaitForEvent {
            pin: self,
            trigger: Trigger::HighLevel,
            armed: false,
        }
        .await;
        Ok(())
    }

    /// Resolves as soon as the pin reads low, immediately if it already
    /// does — the inverse of
    /// `wait_for_high`, on GPLEN.
    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        if !read_level(&self.gpio, N) {
            return Ok(());
        }
        WaitForEvent {
            pin: self,
            trigger: Trigger::LowLevel,
            armed: false,
        }
        .await;
        Ok(())
    }

    /// Resolves on the next low→high transition. A pin already high does
    /// not satisfy this — it must go low and rise again — so unlike
    /// `wait_for_high` there is no level shortcut.
    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        WaitForEvent {
            pin: self,
            trigger: Trigger::RisingEdge,
            armed: false,
        }
        .await;
        Ok(())
    }

    /// Resolves on the next high→low transition, with the same
    /// no-shortcut rule as
    /// `wait_for_rising_edge`.
    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        WaitForEvent {
            pin: self,
            trigger: Trigger::FallingEdge,
            armed: false,
        }
        .await;
        Ok(())
    }

    /// Resolves on the next transition in either direction.
    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        WaitForEvent {
            pin: self,
            trigger: Trigger::AnyEdge,
            armed: false,
        }
        .await;
        Ok(())
    }
}
