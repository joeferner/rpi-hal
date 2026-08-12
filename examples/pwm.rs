#![no_std]
#![no_main]

// PWM smoke test for both channels at once, meant to be checked with
// a scope rather than just eyeballing an LED:
//   - channel 1 (GPIO18, physical pin 12): fixed 50% duty cycle, set
//     once at startup -- should read as a steady, unchanging square
//     wave.
//   - channel 2 (GPIO19, physical pin 35): duty cycle swept 0..=RANGE
//     and back, continuously -- confirms `set_duty_cycle` actually
//     changes the output live, not just once at setup.
//
// PLLD_per (nominally 500MHz -- see pwm.rs's doc comment) / 2500 =
// 200kHz PWM clock; a 1000-tick range gives both channels a 200Hz
// output.

use core::fmt::Write;
use embedded_hal::pwm::SetDutyCycle;
use rpi_hal::halt;
use rpi_hal::pac;
use rpi_hal::pwm::{Channel1Pin, Channel2Pin, Pwm};
use rpi_hal::uart::Uart;

const RANGE: u16 = 1000;

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
    let _ = writeln!(uart, "Starting...");

    let pwm = Pwm::init(peripherals.PWM0, peripherals.CM_PWM, 2500);

    let mut fixed = pwm.channel1(&peripherals.GPIO, Channel1Pin::Gpio18, RANGE);
    let _ = fixed.set_duty_cycle(RANGE / 2);

    let mut variable = pwm.channel2(&peripherals.GPIO, Channel2Pin::Gpio19, RANGE);

    let mut count: u32 = 0;
    loop {
        for duty in 0..=RANGE {
            let _ = variable.set_duty_cycle(duty);
            delay(3_000);
        }
        for duty in (0..=RANGE).rev() {
            let _ = variable.set_duty_cycle(duty);
            delay(3_000);
        }

        // Heartbeat: proves the main loop is actually running, not
        // stuck somewhere in setup.
        count += 1;
        let _ = writeln!(uart, "sweep {count} complete");
    }
}

fn delay(cycles: u32) {
    for _ in 0..cycles {
        unsafe { core::arch::asm!("nop") };
    }
}
