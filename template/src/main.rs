#![no_std]
#![no_main]

//! {{project-name}}, for the {{board_name}}.
//!
//! Boots on rpi-hal, starts rpi-kickstart's console on the UART
//! (GPIO14/15, 115200 8N1), and logs a heartbeat once a second. A count
//! that keeps rising proves the board is still running, not just that it
//! reached `kmain`.

use core::fmt::Write;

use rpi_hal::{halt, pac, timer::Timer, uart::Uart};
use rpi_kickstart::{console, logln};
use static_cell::StaticCell;

// The console borrows its sink for `'static`, so the UART can't live on
// `kmain`'s stack.
static UART: StaticCell<Uart> = StaticCell::new();

/// The clock the console stamps each line with, in microseconds.
///
/// The System Timer's counters are read-only and free-running, so
/// stealing the peripheral here disturbs nothing.
fn uptime() -> u64 {
    Timer::new(unsafe { pac::Peripherals::steal() }.SYSTMR).now_micros()
}

// A program has exactly one panic handler, so it's the application's.
// It writes to a fresh UART rather than through the console: whatever
// owned the console may be what just panicked.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// Where rpi-hal's boot code hands over, once the stack and `.bss` are
/// set up.
#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let timer = Timer::new(peripherals.SYSTMR);

    console::init(
        UART.init(Uart::init(&peripherals.GPIO, peripherals.UART0)),
        uptime,
    );
    logln!("{{project-name}}: hello from the {{board_name}}");

    let mut seconds = 0u32;
    loop {
        timer.delay_ms(1000);
        seconds += 1;
        logln!("up {seconds}s");
    }
}
