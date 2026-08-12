#![no_std]
#![no_main]

use core::fmt::Write;
use embedded_hal::digital::StatefulOutputPin;
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
