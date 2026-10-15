#![no_std]
#![no_main]

// Awaiting a peripheral rather than a clock: one task blocks on a GPIO
// edge through `embedded_hal_async::digital::Wait`, another keeps time,
// and the core sleeps in `wfe` until either the button or the timer wakes
// it.
//
// This is the first example where two *different* interrupt sources feed
// the executor, which is what makes the dispatch shape visible: neither
// rpi-hal nor this crate can define `__irq_handler` — a library that
// claimed it would take the vector away from the program that owns it —
// so the application below routes each source to the crate that services
// it. The two coexist because `gpio::on_irq` only touches pins a future
// is actually parked on.
//
// What the output shows: `t=` lines keep arriving on their own second
// boundaries whether or not the button is pressed, and `press=` lines
// appear only on a real low→high transition. Presses not disturbing the
// timer's cadence is the point — a press wakes the core, runs one task,
// and the deadline for the other is still met.
//
// Wiring: an LED (with a series resistor) from GPIO4 (header pin 7) to
// GND, and a button between GPIO17 (header pin 11) and 3V3 with a ~10k
// pull-down from GPIO17 to GND. rpi-hal doesn't drive the pull-up/down
// hardware, so the input needs a defined idle level from outside. No
// debounce: a bouncing contact simply reports a few extra presses.

use core::fmt::Write;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_time::{Duration, Instant, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal_async::digital::Wait;
use rpi_hal::gpio::{self, Input, Output, Pin};
use rpi_hal::halt;
use rpi_hal::{irq, lic::Lic, pac, timer::Timer, uart::Uart};
use rpi_hal_embassy::{Executor, time_driver};

/// Button presses seen, published for the reporting task.
static PRESSES: AtomicU32 = AtomicU32::new(0);

/// LED output (GPIO4, header pin 7).
const LED: u8 = 4;
/// Button input (GPIO17, header pin 11).
const BUTTON: u8 = 17;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

#[embassy_executor::task]
async fn watch_button(mut button: Pin<BUTTON, Input>, mut led: Pin<LED, Output>) {
    loop {
        // Parks the task and disarms nothing else: the pin's detector is
        // armed on the first poll of this future and torn down when it
        // resolves, so no interrupt is left running between presses.
        let _ = button.wait_for_rising_edge().await;

        PRESSES.fetch_add(1, Ordering::Relaxed);
        let _ = led.toggle();
    }
}

#[embassy_executor::task]
async fn report(mut uart: Uart) {
    let start = Instant::now();
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut ticks: u64 = 0;
    let mut last = 0;

    loop {
        ticker.next().await;
        ticks += 1;

        let drift = start.elapsed().as_micros() as i64 - (ticks * 1_000_000) as i64;
        let presses = PRESSES.load(Ordering::Relaxed);

        // Only mention presses when the count moves, so an idle board
        // still shows the timer holding its cadence.
        if presses != last {
            let _ = writeln!(uart, "t={ticks}s press={presses} drift={drift}us");
            last = presses;
        } else {
            let _ = writeln!(uart, "t={ticks}s drift={drift}us");
        }
    }
}

// `unsafe(no_mangle)` rather than the bare `no_mangle` rpi-hal's own
// examples use: this crate is edition 2024, which requires the wrapper.
#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "embassy: awaiting a GPIO edge");

    let led = Pin::<LED, Input>::new(peripherals.GPIO).into_output();
    // A second GPIO token for the button: `new` configures the pin as an
    // input, which is exactly what's wanted, and the two pins touch
    // disjoint bits of the same block.
    let button = Pin::<BUTTON, Input>::new(unsafe { pac::Peripherals::steal() }.GPIO);

    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);

    // Route the button's bank to the CPU. The future arms the pin's own
    // detector when it is first polled; this is the interrupt-controller
    // gate, which is shared across the bank and so belongs to the
    // application, not to any one pin.
    lic.enable_gpio_irq(BUTTON);
    irq::enable_irq();

    // Sound only because `kmain` never returns — the same lifetime
    // widening `#[embassy_executor::main]` performs.
    unsafe fn make_static<T>(t: &mut T) -> &'static mut T {
        unsafe { core::mem::transmute(t) }
    }
    let mut executor = Executor::new();
    let executor = unsafe { make_static(&mut executor) };

    executor.run(|spawner| {
        // The `Result` is on building the token — a `#[task]` pool already
        // in use — not on spawning it.
        spawner.spawn(watch_button(button, led).unwrap());
        spawner.spawn(report(uart).unwrap());
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn __irq_handler() {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);

    // Checked independently, not else-if: both can be pending in one
    // entry, and each is serviced by the crate that owns the source.
    if lic.is_timer1_pending() {
        time_driver::on_timer_irq();
    }

    if lic.is_gpio_pending(BUTTON) {
        // Acks and disarms only pins a future is waiting on, so a
        // hand-serviced pin in the same bank would still be left for the
        // code below to handle.
        gpio::on_irq();
    }
}
