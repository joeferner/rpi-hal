#![no_std]
#![no_main]

//! Board shutdown (halt) via the PM block (`power::shutdown`). Prints a
//! line over the serial console, waits two seconds, then halts -- the
//! counterpart to `examples/reboot.rs`.
//!
//! Unlike `reboot`, this does not loop: `shutdown` writes the firmware's
//! "halt" boot-partition sentinel before resetting, so on the way back up
//! the firmware stops instead of booting. The console prints the line
//! once and then stays quiet until the board is physically power-cycled --
//! this hardware can't cut its own power, so "shutdown" means halted-and-
//! idle, not powered-off.

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

    // Printed once; it drains over the wire during the delay that follows,
    // so it's fully sent before the reset hits. With shutdown() below it is
    // not reprinted -- the board halts rather than rebooting.
    let _ = writeln!(uart, "shutdown demo: booted, halting in 2 s");
    timer.delay_ms(2000);

    power::shutdown();
}
