#![no_std]
#![no_main]

// Checks the `embassy-time` driver's monotonic half in isolation, with no
// executor, no interrupts, and nothing awaited: it compares
// `embassy_time::Instant` against the same System Timer read through
// rpi-hal's blocking API.
//
// Two things this proves that a library build cannot. First, linkage --
// `embassy-time` resolves `Instant::now()` to whichever time driver is
// linked, via `extern "Rust"` symbols, so a binary that reads a sensible
// clock is evidence this crate's driver is the one wired up. Second, the
// tick rate: `Instant` counts ticks, and only a driver whose ticks really
// are microseconds makes the two agree.
//
// Both clocks are sampled inside the *same* bracket, and the absolute
// counts are compared as well as the intervals. An earlier version of this
// example measured the embassy interval across the loop boundary instead,
// which quietly charged the previous iteration's blocking UART write to
// the clock: it reported a steady 1002423us against the HAL's 1000000us,
// looking like 0.24% of drift. That excess was the write -- 44 bytes at
// 115200 baud 8N1, less the 16 the FIFO swallows, is (44-16)*10/115200 =
// 2.43ms, which is the discrepancy to three digits. Sampling both clocks
// at the same two instants is what makes the comparison mean anything.
//
// Deadlines are deliberately out of scope here -- `schedule_wake` needs a
// waker, so it takes an executor to exercise.

use core::fmt::Write;
use embassy_time::Instant;
use rpi_hal::halt;
use rpi_hal::{pac, timer::Timer, uart::Uart};

// Nothing here is called by name, but the binary still has to name the
// crate: the time driver is installed by linkage, and a crate nothing
// refers to is never linked, so without this the build fails on
// `_embassy_time_now` undefined. An application that calls
// `time_driver::init` pulls the crate in that way instead and needs no
// such import.
use rpi_hal_embassy as _;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

// `unsafe(no_mangle)` rather than the bare `no_mangle` rpi-hal's own
// examples use: this crate is edition 2024, which requires the wrapper.
#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);

    // No `time_driver::init` call: it hands over Compare 1 and enables an
    // interrupt, neither of which reading the counter needs. That the
    // clock is already correct here is the point -- `embassy-time`
    // requires `now()` to work before the driver has been initialized at
    // all, which the System Timer satisfies by running from boot.
    let _ = writeln!(uart, "embassy-time driver: monotonic check");

    loop {
        // One second, bracketed by both clocks at the same two instants.
        // These should agree to within the few microseconds the reads
        // themselves take.
        let hal_before = timer.now_micros();
        let embassy_before = Instant::now();
        timer.delay_ms(1000);
        let hal_interval = timer.now_micros() - hal_before;
        let embassy_interval = (Instant::now() - embassy_before).as_micros();

        // The stronger check: at a 1MHz tick rate both of these are
        // microseconds since boot read off the same counter, so the two
        // absolute values should differ only by the gap between the reads
        // -- and stay that close forever. A wrong `TICK_HZ` would show up
        // here as a gap that grows in proportion to uptime, which equal
        // interval measurements alone could not distinguish.
        let hal_now = timer.now_micros();
        let embassy_ticks = Instant::now().as_ticks();

        let _ = writeln!(
            uart,
            "interval hal {hal_interval}us embassy {embassy_interval}us | \
             now hal {hal_now} embassy {embassy_ticks} (+{})",
            embassy_ticks.saturating_sub(hal_now)
        );
    }
}
