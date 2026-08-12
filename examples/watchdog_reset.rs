#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::halt;
use rpi_hal::{pac, timer::Timer, uart::Uart, watchdog::Watchdog};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// Feeds the watchdog for a while, then stops feeding it to demonstrate
/// the reset firing: the countdown continues on its own once `feed`
/// calls stop, and the board resets `TIMEOUT_MS` after the last feed.
#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut watchdog = Watchdog::new();

    const TIMEOUT_MS: u32 = 5_000;
    const FEED_COUNT: u32 = 3;

    watchdog.start(TIMEOUT_MS);
    let _ = writeln!(uart, "watchdog armed for {TIMEOUT_MS}ms");

    for i in 0..FEED_COUNT {
        timer.delay_ms(TIMEOUT_MS / 2);
        watchdog.feed();
        let _ = writeln!(uart, "fed watchdog ({}/{FEED_COUNT})", i + 1);
    }

    let _ = writeln!(uart, "no more feeding -- board resets in {TIMEOUT_MS}ms");
    halt();
}
