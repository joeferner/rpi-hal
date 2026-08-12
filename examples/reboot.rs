#![no_std]
#![no_main]

//! Board reboot via the PM block (`power::reboot`). Prints a line over the
//! serial console, waits two seconds, then reboots -- so the console shows
//! the same startup line reappear every ~2 s, a visible, self-driving
//! reboot loop that proves the reset actually takes effect.
//!
//! For the halt path (`power::shutdown`), which resets once and then stays
//! off rather than looping, see `examples/shutdown.rs`.

use core::fmt::Write;
use rpi_hal::{pac, power, timer::Timer, uart::Uart};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let p = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&p.GPIO, p.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    rpi_hal::halt();
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let p = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&p.GPIO, p.UART0);
    let timer = Timer::new(p.SYSTMR);

    // Printed once per boot; with reboot() below it reappears every ~2 s.
    // The message drains over the wire during the delay that follows, so
    // it's fully sent before the reset hits.
    let _ = writeln!(uart, "reboot demo: booted, rebooting in 2 s");
    timer.delay_ms(2000);

    power::reboot();
}
