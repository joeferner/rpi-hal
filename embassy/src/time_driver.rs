//! `embassy-time` driver over the BCM System Timer.
//!
//! The System Timer is a free-running 64-bit counter at a fixed 1MHz, so
//! it maps onto `embassy-time`'s tick rate with no scaling at all — the
//! crate pins `tick-hz-1_000_000` for exactly that reason. Deadlines come
//! from its Compare 1 register, the one of C0-C3 not spoken for by the
//! GPU firmware or left free.
//!
//! The ARM generic timer has the better deadline primitives (per-core,
//! 64-bit compare) but is the wrong choice for a *global* timebase: at
//! 19.2MHz it matches no `tick-hz-*` rate `embassy-time` offers, so it
//! would need lossy scaling inside
//! [`Driver::now`](embassy_time_driver::Driver::now), the hottest function
//! on the path.
//!
//! # Wiring
//!
//! Three things have to happen before an `await` on a deadline resolves,
//! and only the first belongs to this crate:
//!
//! 1. [`init`](crate::time_driver::init) takes ownership of Compare 1 and
//!    enables its interrupt at the interrupt controller.
//! 2. The application unmasks interrupts at the CPU (`rpi_hal::irq`).
//! 3. The application's `__irq_handler` calls
//!    [`on_timer_irq`](crate::time_driver::on_timer_irq) when
//!    `Lic::is_timer1_pending` reports this source.
//!
//! Step 3 is the application's because `rpi-hal` leaves `__irq_handler`
//! to it — a library crate cannot define that symbol without taking the
//! whole vector away from the program that owns it.
//!
//! [`Driver::now`](embassy_time_driver::Driver::now) is deliberately usable
//! before any of that: the counter runs from boot with no configuration,
//! which is what lets this driver satisfy `embassy-time`'s requirement that
//! `now()` never fail even if the hardware has not been initialized.
//!
//! (The explicit link targets above are load-bearing: these module docs
//! are merged with the outer doc comment on `pub mod time_driver` in
//! lib.rs, and the merged text resolves intra-doc links in *that* scope --
//! the crate root -- where this module's own items are not in scope. Bare
//! `[`init`]`-style links work in item docs here, just not at module
//! level.)

use core::cell::RefCell;
use core::task::Waker;

use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use rpi_hal::lic::Lic;
use rpi_hal::pac::SYSTMR;
use rpi_hal::timer::Timer;

embassy_time_driver::time_driver_impl!(static DRIVER: TimeDriver = TimeDriver::new());

struct TimeDriver {
    // Touched from both task context (`schedule_wake`) and the interrupt
    // handler, so it needs a lock rather than a plain `RefCell`. Under
    // `rpi-hal`'s `multicore` feature this is a real cross-core spinlock,
    // which is what would make the driver safe on more than one core.
    queue: Mutex<RefCell<Queue>>,
}

impl TimeDriver {
    const fn new() -> Self {
        Self {
            queue: Mutex::new(RefCell::new(Queue::new())),
        }
    }

    /// The System Timer, stolen rather than owned.
    ///
    /// [`Driver`] requires `Send + Sync`, which a stored [`Timer`] would
    /// not satisfy, and `now()` has to work from any context at any time.
    /// Stealing is free here — the PAC's peripheral handle is a
    /// zero-sized marker — and sound by contract, because [`init`]
    /// consumes the caller's `Timer` to transfer Compare 1 to this
    /// driver.
    #[inline]
    fn timer(&self) -> Timer {
        Timer::new(unsafe { SYSTMR::steal() })
    }

    /// Wakes everything due, then arms Compare 1 for whatever is next.
    fn rearm(&self, cs: CriticalSection<'_>) {
        let mut queue = self.queue.borrow_ref_mut(cs);

        // `set_compare1` refusing a deadline is not a failure — it means
        // the counter reached it while we were deciding, and the compare
        // is an equality test, so arming it anyway would wait out a full
        // ~71.6-minute wrap instead of firing. Drain against a fresh
        // `now` and ask again. Each pass removes at least the entry that
        // just came due, so this terminates.
        let mut next = queue.next_expiration(self.now());
        while !self.timer().set_compare1(next) {
            next = queue.next_expiration(self.now());
        }
    }

    fn on_irq(&self) {
        critical_section::with(|cs| {
            // Clear before draining rather than after: a deadline that
            // comes due while this handler runs must be able to leave the
            // match flag set behind us, so the interrupt fires again
            // instead of being cleared away unserviced.
            self.timer().clear_compare1_match();
            self.rearm(cs);
        });
    }
}

impl Driver for TimeDriver {
    fn now(&self) -> u64 {
        self.timer().now_micros()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            // Only re-arm when this changed the head of the queue;
            // enqueuing behind an earlier deadline leaves the hardware
            // alarm already correct.
            let head_changed = self.queue.borrow_ref_mut(cs).schedule_wake(at, waker);
            if head_changed {
                self.rearm(cs);
            }
        });
    }
}

/// Hands the System Timer's Compare 1 to the `embassy-time` driver and
/// enables its interrupt at the interrupt controller.
///
/// Consuming `timer` is how the application gives up Compare 1: from here
/// on the driver arms and acknowledges it, and a second owner racing it
/// would lose deadlines. That transfer is a statement of intent rather
/// than something the type system enforces, since the PAC allows a
/// peripheral handle to be stolen from anywhere — but an application that
/// keeps no `Timer` cannot reach Compare 1 by accident.
///
/// `lic` is only borrowed: the application still needs it for its own
/// interrupt sources.
///
/// Two steps remain afterwards, both the application's (see the module
/// documentation): unmask interrupts at the CPU, and route this source to
/// [`on_timer_irq`] from `__irq_handler`. Until they are done, deadlines
/// never fire — though `embassy_time::Instant::now()` already reads
/// correctly, before and after this call.
pub fn init(timer: Timer, lic: &Lic) {
    // Start from a known state: a match left set from before this program
    // ran would otherwise fire the moment the interrupt is enabled.
    timer.clear_compare1_match();
    lic.enable_timer1_irq();
}

/// Services a System Timer Compare 1 interrupt: wakes every task whose
/// deadline has passed, and arms the next one.
///
/// Call this from the application's `__irq_handler` when
/// `Lic::is_timer1_pending` reports this source. It acknowledges the
/// match itself, so the caller must not also clear it.
///
/// Harmless to call spuriously — with nothing due it simply re-arms —
/// which is what makes the early wake-ups from a clamped far-future
/// deadline a non-event.
pub fn on_timer_irq() {
    DRIVER.on_irq();
}
