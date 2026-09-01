#![no_std]
#![no_main]

// Internal pull-up/pull-down resistors: checks that a floating input
// follows whichever pull is selected, then watches the pin so a jumper
// can be shown overriding it.
//
// Wiring: nothing at all for the first phase -- GPIO17 (header pin 11)
// must be left unconnected, since the point is that the *internal*
// resistor alone decides the level. For the second phase, touch a jumper
// wire between GPIO17 and GND (header pin 9, right next to it); the pin
// has its pull-up on, so grounding it should read low and releasing it
// should snap back to high. That is the real reason this API exists: a
// push-button between a pin and GND, with no external resistor.
//
// Phase 1 is self-checking, so an external resistor left over from
// examples/gpio_irq_button.rs shows up as a failure rather than silently
// passing -- a 10k pull-down to GND is comparable to the internal
// pull-up and holds the pin near mid-rail.

use core::fmt::Write;
use embedded_hal::digital::InputPin;
use rpi_hal::gpio::{Input, Pin, Pull};
use rpi_hal::timer::Timer;
use rpi_hal::{halt, pac, uart::Uart};

/// The pin under test (GPIO17, header pin 11) -- the same input
/// examples/gpio_irq_button.rs uses, and adjacent to a GND pin.
const PIN: u8 = 17;

/// Settle time after selecting a pull before the level is trusted. The
/// resistors are tens of kΩ and the pin plus any attached wire is a few
/// tens of pF, so the RC is microseconds; a millisecond is far past it
/// and still imperceptible.
const SETTLE_MS: u32 = 1;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);

    // A fresh GPIO token: the single PAC `GPIO` peripheral covers every
    // pin, and `Uart::init` above borrowed the first one.
    let mut pin = Pin::<PIN, Input>::new(unsafe { pac::Peripherals::steal() }.GPIO).into_input();

    let _ = writeln!(
        uart,
        "gpio_pull: testing GPIO{PIN}'s internal pull resistors"
    );
    let _ = writeln!(uart, "phase 1: leave GPIO{PIN} unconnected");

    // With nothing driving the pin, its level is whatever the selected
    // pull says. Pull-down first, so a pass on the pull-up that follows
    // can't just be the pin's power-up state (GPIO9-27 come out of reset
    // pulled down) going unchanged; then pull-down again, so the pull-up
    // is shown to be reversible rather than sticky.
    let mut passed = true;
    for (pull, expected_high) in [(Pull::Down, false), (Pull::Up, true), (Pull::Down, false)] {
        pin.set_pull(pull);
        timer.delay_ms(SETTLE_MS);
        let what = match pull {
            Pull::Up => "set_pull(Up)",
            Pull::Down => "set_pull(Down)",
            Pull::None => "set_pull(None)",
        };
        passed &= expect(&mut uart, what, &mut pin, expected_high);
    }

    // The same two settings reached through the `into_*_input`
    // converters instead of `set_pull`.
    let mut pin = pin.into_pull_up_input();
    timer.delay_ms(SETTLE_MS);
    passed &= expect(&mut uart, "into_pull_up_input", &mut pin, true);
    let mut pin = pin.into_pull_down_input();
    timer.delay_ms(SETTLE_MS);
    passed &= expect(&mut uart, "into_pull_down_input", &mut pin, false);

    let _ = writeln!(uart, "phase 1: {}", if passed { "PASS" } else { "FAIL" });

    // Phase 2: a pull is weak, so anything actually driving the pin wins.
    // Report changes rather than the level continuously, so a one-second
    // tap is one line and not a screenful.
    pin.set_pull(Pull::Up);
    timer.delay_ms(SETTLE_MS);
    let _ = writeln!(
        uart,
        "phase 2: pull-up on -- short GPIO{PIN} to GND (header pin 9), then release"
    );

    let mut last = pin.is_high().unwrap_or(true);
    let _ = writeln!(uart, "  {} (idle)", level(last));
    loop {
        let now = pin.is_high().unwrap_or(last);
        if now != last {
            let note = if now {
                "released, pull-up won"
            } else {
                "driven to GND"
            };
            let _ = writeln!(uart, "  {} -- {note}", level(now));
            last = now;
        }
        // Slow the poll to something a bouncing contact can't flood the
        // console from. This is a level poll, not edge detection -- see
        // examples/gpio_irq_button.rs for the interrupt-driven version.
        timer.delay_ms(20);
    }
}

/// Reads `pin` and prints a pass/fail line for `what`.
fn expect(uart: &mut Uart, what: &str, pin: &mut Pin<PIN, Input>, expected_high: bool) -> bool {
    let high = pin.is_high().unwrap_or(false);
    let ok = high == expected_high;
    let _ = writeln!(
        uart,
        "  {what}: reads {}, expected {} -- {}",
        level(high),
        level(expected_high),
        if ok { "ok" } else { "FAIL" }
    );
    ok
}

/// Renders a level as the word printed in the report lines.
fn level(high: bool) -> &'static str {
    if high {
        "high"
    } else {
        "low"
    }
}
