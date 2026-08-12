#![no_std]
#![no_main]

// The first program in which anything is actually awaited: two tasks on
// the thread-mode executor, both driven by `embassy-time` deadlines, with
// the core asleep in `wfe` in between.
//
// What each part of the output is evidence for:
//
// - `toggles` climbing by 4 every second -- one per 250ms tick -- shows
//   the two tasks genuinely interleaving. The reporting task never touches
//   the LED and the blink task never touches the UART, so a counter that
//   keeps up while reports keep printing can only mean both are being
//   polled.
//
//   Expect the count to sit one behind a multiple of 4 (3, 7, 11, ...)
//   rather than land on 4, 8, 12. Both tasks come due at the same instant
//   every whole second, and within one poll the reporting task is reached
//   first, so it reads the counter just before that second's toggle is
//   applied. A *constant* offset is the tell that this is scheduling
//   order, not a dropped tick -- a lost tick would make the gap grow.
// - `drift` is the real test of the driver's deadline arithmetic.
//   `Ticker` schedules *absolute* deadlines, so a correct driver holds
//   drift near zero indefinitely, however long each iteration's work
//   takes -- the ~3.8ms the UART write costs at 115200 baud is exactly
//   the sort of per-iteration cost that would accumulate visibly (about
//   4ms per second, a quarter second per minute) if deadlines were being
//   computed relative to wake-up instead.
//
// Together these exercise everything Milestones C and D added:
// `schedule_wake`, the Compare 1 alarm and its interrupt, the run queue,
// the `sev` pender and the `wfe` idle.

use core::fmt::Write;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_time::{Duration, Instant, Ticker};
use embedded_hal::digital::OutputPin;
use rpi_hal::gpio::{Input, Output, Pin};
use rpi_hal::halt;
use rpi_hal::{irq, lic::Lic, pac, timer::Timer, uart::Uart};
use rpi_hal_embassy::{Executor, time_driver};

/// LED state changes, published by the blink task so the reporting task
/// can show both are running without the two sharing a peripheral. Counts
/// every 250ms tick, so it advances at four times the visible blink rate.
static TOGGLES: AtomicU32 = AtomicU32::new(0);

/// GPIO4, header pin 7 -- the same LED `rpi-hal`'s `blink` example drives.
const LED: u8 = 4;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

#[embassy_executor::task]
async fn blink(mut led: Pin<LED, Output>) {
    let mut ticker = Ticker::every(Duration::from_millis(250));
    let mut on = false;

    loop {
        ticker.next().await;
        on = !on;
        let _ = if on { led.set_high() } else { led.set_low() };
        TOGGLES.fetch_add(1, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
async fn report(mut uart: Uart) {
    let start = Instant::now();
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut ticks: u64 = 0;

    loop {
        ticker.next().await;
        ticks += 1;

        // Cumulative, not per-interval: a per-interval measurement would
        // hide a systematic lag by re-baselining every second, which is
        // the same mistake that made `embassy_now.rs` look like it was
        // drifting. Measured from a single start instant, any systematic
        // error has nowhere to hide.
        let elapsed = start.elapsed().as_micros() as i64;
        let drift = elapsed - (ticks * 1_000_000) as i64;

        let _ = writeln!(
            uart,
            "t={ticks}s toggles={} drift={drift}us",
            TOGGLES.load(Ordering::Relaxed)
        );
    }
}

// `unsafe(no_mangle)` rather than the bare `no_mangle` rpi-hal's own
// examples use: this crate is edition 2024, which requires the wrapper.
#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "embassy executor + time driver");

    let led = Pin::<LED, Input>::new(peripherals.GPIO).into_output();

    // Hand Compare 1 to the driver, then open the other two gates it
    // needs: the CPU interrupt mask here, and the dispatch to
    // `on_timer_irq` in `__irq_handler` below. Deadlines do not fire
    // without all three.
    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);
    irq::enable_irq();

    // The executor must outlive `run`. Rather than a `StaticCell`, this
    // borrows a local and widens its lifetime, which is sound only
    // because `kmain` never returns -- exactly what
    // `#[embassy_executor::main]` generates, and the reason that macro
    // requires a diverging entry point.
    unsafe fn make_static<T>(t: &mut T) -> &'static mut T {
        unsafe { core::mem::transmute(t) }
    }
    let mut executor = Executor::new();
    let executor = unsafe { make_static(&mut executor) };

    executor.run(|spawner| {
        // The fallible half is building the token, not spawning it: a
        // `#[task]` function hands back `Err(SpawnError)` when its pool is
        // already in use. With one instance of each task here that cannot
        // happen, and `unwrap` is what `#[embassy_executor::main]` emits
        // for the same call.
        spawner.spawn(blink(led).unwrap());
        spawner.spawn(report(uart).unwrap());
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn __irq_handler() {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);

    if lic.is_timer1_pending() {
        // Acknowledges the match itself; nothing else here may clear it.
        time_driver::on_timer_irq();
    }
}
