#![no_std]
#![no_main]

use embedded_hal::digital::OutputPin;
use rpi_hal::gpio::{Input, Pin};
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

    // GPIO4 (physical header pin 7) as output.
    let mut led = Pin::<4, Input>::new(peripherals.GPIO).into_output();

    // Real wall-clock delay off the 1MHz System Timer, not a spin of N
    // `nop`s: the CPU clock is ~900MHz-1.2GHz and unknown at build time,
    // so a fixed instruction count gives an unpredictable (and, at a
    // human-visible count, far too fast) period. 500ms on/off = 1Hz.
    let timer = Timer::new(peripherals.SYSTMR);

    loop {
        let _ = led.set_high();
        timer.delay_ms(500);
        let _ = led.set_low();
        timer.delay_ms(500);
    }
}
