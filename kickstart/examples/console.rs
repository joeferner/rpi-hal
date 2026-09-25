#![no_std]
#![no_main]

// The console sink and `logln!`: what a stamped line looks like, that the
// stamp measures from `init` rather than from power-on, and that a log
// before `init` is discarded instead of faulting.
//
// The last of those is the one worth an example. Every module in a real
// image logs, and some of them run before bring-up has reached the
// console -- a fault reporter most of all -- so "discarded" has to be
// true rather than nearly true.
//
// Build it with `scripts/build-example.sh console` (kernel7.img) or
// `scripts/build-example64.sh console` (kernel8.img).

use core::fmt::Write;

use rpi_hal::{pac, timer::Timer, uart::Uart};
use rpi_kickstart::{console, logln};
use static_cell::StaticCell;

// The console borrows its sink for `'static`, so the sink cannot live on
// `kmain`'s stack. `StaticCell` is how an application hands over something
// it built at runtime without a `static mut` or a transmute.
static UART: StaticCell<Uart> = StaticCell::new();

/// The clock `console::init` stamps lines with.
///
/// A plain `fn` rather than a closure because that is what `init` takes,
/// and a closure capturing a `Timer` would not coerce to one. A board on
/// Embassy has the easier version of this -- `|| Instant::now().as_micros()`
/// captures nothing -- but this example has no time driver, so it reads
/// the System Timer directly.
///
/// Stealing the peripheral rather than sharing the `Timer` `kmain` owns:
/// `now_micros` reads CLO and CHI, which are read-only counters running
/// since power-on. There is no state here for a second handle to disturb,
/// and the alternative -- threading a `&'static Timer` into a `fn` that
/// cannot capture one -- is a static apiece to solve nothing.
fn uptime() -> u64 {
    Timer::new(unsafe { pac::Peripherals::steal() }.SYSTMR).now_micros()
}

// Deliberately not `logln!`. Two reasons, and either alone would be
// enough: the console may never have been installed, and a panic *inside*
// `write_line` would re-enter a `RefCell` that is already mutably
// borrowed -- turning one panic into an endless pair of them with nothing
// printed.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    rpi_hal::halt();
}

#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    // Before `init`, and the first thing this program does. It has to go
    // nowhere rather than fault: nothing is installed, and the console
    // does not exist to be borrowed.
    logln!("discarded: there is no console yet");

    let peripherals = unsafe { pac::Peripherals::steal() };
    let timer = Timer::new(peripherals.SYSTMR);

    // The counter is already well past zero by the time a kernel runs --
    // the firmware has been up for a second or more -- so this is what
    // the stamps would read if `init` did not record an origin.
    let raw = timer.now_micros();

    console::init(
        UART.init(Uart::init(&peripherals.GPIO, peripherals.UART0)),
        uptime,
    );

    logln!("rpi-kickstart console");
    logln!("system timer read {raw}us at boot; the stamp starts from there");
    logln!(
        "arch: {}",
        if cfg!(target_arch = "aarch64") {
            "aarch64"
        } else {
            "aarch32"
        }
    );

    // A deliberately uneven cadence. A column of identical intervals
    // proves only that the clock advances; a gap that is visibly longer
    // proves the stamp is measuring the delay and not counting lines.
    let mut line = 0u32;
    loop {
        line += 1;
        let pause = if line.is_multiple_of(4) { 2500 } else { 500 };
        timer.delay_ms(pause);
        logln!("line {line}, after a {pause}ms pause");
    }
}
