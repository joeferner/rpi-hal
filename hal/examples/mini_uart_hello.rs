#![no_std]
#![no_main]

// Mini-UART (UART1) "hello world" on GPIO14/15, the same pins UART0's
// debug console uses — the point being that this drives them through the
// separate AUX mini-UART instead, which is what a serial console has to
// fall back to once UART0 (PL011) is committed to something else (e.g.
// Bluetooth on the on-board wireless chip).
//
// The mini UART is clocked from the (dynamically scaled) VPU/core clock,
// so if the terminal shows garbage, pin the clock by adding `core_freq=250`
// to config.txt and rebooting. `enable_uart=1` alone is not enough when
// PL011 is the primary UART — see `rpi_hal::mini_uart::CORE_CLOCK_HZ`.

use core::fmt::Write;
use embedded_hal::digital::StatefulOutputPin;
use rpi_hal::gpio::{Input, Pin};
use rpi_hal::halt;
use rpi_hal::mini_uart::MiniUart;
use rpi_hal::pac;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = MiniUart::init(&peripherals.GPIO, &peripherals.AUX, peripherals.UART1);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    let mut uart = MiniUart::init(&peripherals.GPIO, &peripherals.AUX, peripherals.UART1);

    // LED heartbeat on GPIO4: toggles every loop iteration regardless of
    // whether the UART message reaches a terminal, so the loop is visibly
    // alive even with nothing attached.
    let mut led = Pin::<4, Input>::new(peripherals.GPIO).into_output();

    let mut count: u32 = 0;
    loop {
        let _ = writeln!(uart, "hello from rpi-hal mini UART ({count})");
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
