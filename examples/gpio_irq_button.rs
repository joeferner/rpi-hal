#![no_std]
#![no_main]

// Interrupt-driven GPIO input: the LED on GPIO4 mirrors a button on
// GPIO17 -- lit while the button is held, dark when released -- with the
// CPU parked in `wfe` the whole time, never polling. Both edges of the
// button fire an interrupt; the handler reads the pin's settled level and
// drives the LED to match.
//
// This exercises the same three-gate interrupt model as
// irq_timer_blink.rs / uart_rx_irq_echo.rs: the pin's edge detector
// (Pin::enable_interrupt), the interrupt-controller routing
// (Lic::enable_gpio_irq), and the CPU IRQ mask (irq::enable_irq) all have
// to be open.
//
// Wiring: the button goes between GPIO17 (header pin 11) and 3V3, and
// the pin's internal pull-down gives it its idle level, so no external
// resistor is needed. The pin idles low (LED off); pressing pulls it high
// (LED on). No debounce is done -- a bouncing contact just fires a few
// extra edges, and the last one leaves the LED matching the final level.
//
// Wiring the button to GND instead works just as well: swap the pull for
// `into_pull_up_input` and read a press as low.

use core::fmt::Write;
use embedded_hal::digital::{InputPin, OutputPin};
use rpi_hal::gpio::{Input, Output, Pin, Trigger};
use rpi_hal::halt;
use rpi_hal::{irq, lic::Lic, pac, uart::Uart};

/// LED output (GPIO4, header pin 7) -- same pin blink.rs drives.
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

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(
        uart,
        "gpio_irq_button: the LED on GPIO{LED} follows the button on GPIO{BUTTON}"
    );

    // Configure the LED as an output. Like uart_rx_irq_echo.rs, this
    // only sets the pin up; `__irq_handler` re-wraps it via `assume_mode`
    // rather than holding this value.
    Pin::<LED, Input>::new(peripherals.GPIO).into_output();

    // Configure the button to detect both edges. A fresh GPIO token is
    // stolen because the single PAC `GPIO` peripheral covers every pin
    // and the line above consumed the first one.
    let button =
        Pin::<BUTTON, Input>::new(unsafe { pac::Peripherals::steal() }.GPIO).into_pull_down_input();
    button.enable_interrupt(Trigger::AnyEdge);
    // Discard any edge latched before detection was armed, so startup
    // noise doesn't leave a stale event pending.
    button.clear_interrupt();

    let lic = Lic::new(peripherals.LIC);
    lic.enable_gpio_irq(BUTTON);

    irq::enable_irq();

    loop {
        unsafe { core::arch::asm!("wfe") };
    }
}

#[no_mangle]
pub extern "C" fn __irq_handler() {
    let lic = Lic::new(unsafe { pac::Peripherals::steal() }.LIC);

    if lic.is_gpio_pending(BUTTON) {
        // The bank line is shared across a whole pin range, so confirm
        // this pin is the one that latched an event before acting on it.
        let mut button =
            unsafe { Pin::<BUTTON, Input>::assume_mode(pac::Peripherals::steal().GPIO) };
        if button.is_interrupt_pending() {
            button.clear_interrupt();

            // Drive the LED to the button's settled level: high (pressed)
            // -> lit, low (released) -> dark.
            let pressed = button.is_high().unwrap_or(false);

            // Safe: `kmain` configured GPIO4 as an output before enabling
            // the IRQ that leads here.
            let mut led =
                unsafe { Pin::<LED, Output>::assume_mode(pac::Peripherals::steal().GPIO) };
            let _ = if pressed {
                led.set_high()
            } else {
                led.set_low()
            };
        }
    }
}
