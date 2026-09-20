//! Prints a counter over the UART0 console, once a second or so, with
//! the GPIO4 LED toggling alongside it.
//!
//! The first line is the core's `MIDR` Main ID register (see
//! [`rpi_hal::cpu::main_id`] for how to read it): on a board whose
//! console has never worked before, it answers "are characters getting
//! out" and "is this the chip the binary was built for" at the same
//! time. `0x410fb767` is an ARM1176JZF-S (Pi 1, Pi Zero), `0x410fc075`
//! a Cortex-A7 (Pi 2), `0x410fd034` a Cortex-A53 (Pi 3).
//!
//! Wiring is the standard console setup: a 3.3V USB-serial cable on
//! GPIO14/15, 115200 8N1, plus an LED and resistor on GPIO4. See
//! `docs/getting-started.md`.

#![no_std]
#![no_main]

use core::fmt::Write;
use embedded_hal::digital::StatefulOutputPin;
use rpi_hal::cpu;
use rpi_hal::gpio::{Input, Pin};
use rpi_hal::halt;
use rpi_hal::{pac, uart::Uart};

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

    // Which core is actually running this -- see the module header for
    // what the value means. First, so that a console that only manages
    // one line still says something useful.
    let _ = writeln!(uart, "rpi-hal: MIDR {:#010x}", cpu::main_id());

    // LED heartbeat: toggles every loop iteration regardless of
    // whether the UART message actually gets anywhere, so you can
    // tell the loop is alive even with nothing showing in a terminal.
    let mut led = Pin::<4, Input>::new(peripherals.GPIO).into_output();

    let mut count: u32 = 0;
    loop {
        let _ = writeln!(uart, "hello from rpi-hal ({count})");
        count += 1;

        let _ = led.toggle();

        delay(10_000_000);
    }
}

fn delay(cycles: u32) {
    for _ in 0..cycles {
        unsafe { core::arch::asm!("nop") };
    }
}
