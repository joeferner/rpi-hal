//! Blinks the Pi 5's green ACT LED at 1 Hz.
//!
//! The LED is on the BCM2712's always-on GPIO block rather than RP1, so
//! unlike `blink` this needs nothing from the PCIe link: it is the first
//! thing to try on a new Pi 5 setup, and the instrument to fall back on
//! when something behind RP1 isn't responding.

#![no_std]
#![no_main]

use embedded_hal::digital::OutputPin;
use rpi_hal::gio_aon::{Pin, ACT_LED};
use rpi_hal::gpio::Input;
use rpi_hal::pac;
use rpi_hal::timer::Timer;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        unsafe { core::arch::asm!("wfe") };
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    let mut led = Pin::<ACT_LED, Input>::new(peripherals.GIO_AON).into_output();
    let timer = Timer::new(peripherals.SYSTMR);

    // Active-low: driving the line low lights the LED.
    loop {
        let _ = led.set_low();
        timer.delay_ms(500);
        let _ = led.set_high();
        timer.delay_ms(500);
    }
}
