#![no_std]
#![no_main]

// Diagnostic aid for UART bring-up: uses the already-verified LED on
// GPIO4 to signal progress checkpoints, since the UART itself is what
// we're trying to debug and can't be trusted as an output channel yet.
//
// Checkpoint 1: reached kmain (boot + GPIO always worked before this).
// Checkpoint 2: Uart::init() returned (GPIO mux + baud/LCRH/CR setup
//               didn't hang or panic).
// Checkpoint 3: first write_byte() returned (the TXFF busy-wait
//               actually cleared).
// Then repeats checkpoint 4 forever.
//
// If the LED gets stuck off after N blinks and never proceeds, the
// problem is between checkpoint N and N+1. A fast, continuous
// blink instead means a panic (see the panic handler below).

use embedded_hal::digital::OutputPin;
use rpi_hal::gpio::{Input, Output, Pin};
use rpi_hal::{pac, uart::Uart};

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    let mut led = Pin::<4, Input>::new(unsafe { pac::GPIO::steal() }).into_output();
    loop {
        toggle(&mut led, true);
        delay(200_000);
        toggle(&mut led, false);
        delay(200_000);
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    // Separate `GPIO::steal()`, not `peripherals.GPIO`: `Pin::new`
    // takes GPIO by value, and `peripherals.GPIO` is still needed
    // below for `Uart::init` — GPIO tokens are cheap/duplicable via
    // `steal()`, same as every other peripheral re-steal in this
    // codebase, so this doesn't cost anything real.
    let mut led = Pin::<4, Input>::new(unsafe { pac::GPIO::steal() }).into_output();

    blink(&mut led, 1);

    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);

    blink(&mut led, 2);

    uart.write_byte(b'H');

    blink(&mut led, 3);

    loop {
        blink(&mut led, 4);
    }
}

fn toggle(led: &mut Pin<4, Output>, on: bool) {
    if on {
        let _ = led.set_high();
    } else {
        let _ = led.set_low();
    }
}

fn blink(led: &mut Pin<4, Output>, times: u32) {
    for _ in 0..times {
        toggle(led, true);
        delay(1_000_000);
        toggle(led, false);
        delay(1_000_000);
    }
    delay(5_000_000);
}

fn delay(cycles: u32) {
    for _ in 0..cycles {
        unsafe { core::arch::asm!("nop") };
    }
}
