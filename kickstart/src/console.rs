//! The console every module logs to.
//!
//! One sink for the whole image, installed once during bring-up and
//! reached from anywhere by [`logln!`](crate::logln) — so a driver five
//! modules down can report a retry without being handed a UART it has no
//! other use for.
//!
//! ```ignore
//! static UART: StaticCell<Uart> = StaticCell::new();
//!
//! console::init(
//!     UART.init(Uart::init(&peripherals.GPIO, peripherals.UART0)),
//!     || embassy_time::Instant::now().as_micros(),
//! );
//! logln!("board: starting");
//! ```
//!
//! # Deliberately blocking
//!
//! [`logln!`](crate::logln) busies until the last byte has left the FIFO.
//! At 115200 baud a full line costs a few milliseconds, which is not free
//! once there are tasks with deadlines: a line logged from a task holds up
//! every other task on that executor for as long as it takes to shift out.
//!
//! It is accepted because the alternative — an interrupt-driven transmit
//! ring — is several times the code for a benefit that only matters once
//! logging is chatty in steady state. Bring-up logging happens before the
//! executor starts and costs nothing at all. What follows from it is a
//! rule about *where* to log rather than a reason not to: log decoded
//! events, never individual edges, because a line in the wrong place can
//! span a symbol period.
//!
//! # Why the sink is a trait object
//!
//! [`init`](crate::console::init) takes a `&'static mut (dyn Write +
//! Send)` rather than a
//! `rpi_hal::uart::Uart`, which costs the application a `StaticCell` and
//! buys two things.
//!
//! The first is that this module needs nothing from `rpi-hal` — and so
//! nothing from a chip PAC. `Uart::init` takes PAC singletons, so naming
//! that type here would mean this crate selecting a chip, with all of
//! `rpi-hal`'s precedence order (`bcm2711` > `bcm2837` > `bcm2835`) to
//! forward, for a module whose whole job is to call `write_str`.
//!
//! The second is that the console is genuinely not always UART0. A board
//! that has given the PL011 to a Bluetooth controller logs to
//! `rpi_hal::mini_uart::MiniUart` instead, and both implement
//! [`core::fmt::Write`]; so does a RAM ring buffer read back after a
//! fault, which is the arrangement a board with no serial cable attached
//! wants.
//!
//! # Why the clock is passed in
//!
//! Every line is stamped with the uptime, and
//! [`init`](crate::console::init) takes the function
//! that reads it rather than choosing one. A free-running microsecond
//! counter is available three different ways here — `embassy-time`'s
//! `Instant`, `rpi-hal`'s `Timer`, the ARM generic timer — and picking one
//! would make the most basic module in this crate impose the heaviest
//! dependency in it. A board on Embassy passes
//! `|| Instant::now().as_micros()`; one without passes a `fn` that reads
//! `rpi_hal::timer::Timer`.
//!
//! The stamp is the uptime and not the wall clock deliberately. A board
//! spends its first seconds establishing a date it does not yet have, so
//! the interval that most needs measuring is the interval during which a
//! wall clock would print nothing useful. What the stamp is for is reading
//! a boot log after the fact and seeing where the time went — which step
//! blocked, for how long, and whether a retry waited out a timeout or
//! failed immediately.

use core::cell::RefCell;
use core::fmt::{Arguments, Write};

use critical_section::Mutex;

/// The installed console: where lines go, how to read the clock, and what
/// to measure it from.
///
/// One struct in one slot rather than a static apiece, so there is no
/// state for [`init`] to leave half-written — a sink installed with the
/// previous clock's origin would stamp every line with an uptime measured
/// from the wrong zero.
struct Console {
    /// Where the bytes go.
    sink: &'static mut (dyn Write + Send),
    /// Reads a free-running microsecond counter. Never called before
    /// [`init`] has one.
    uptime: fn() -> u64,
    /// What `uptime` read when [`init`] ran — the zero every stamp is
    /// measured from.
    ///
    /// Recorded rather than assumed to be zero, because none of the
    /// counters a board might pass start there: they are free-running
    /// from power-on, so without this the stamp would read as time since
    /// the last reset of a peripheral nobody reset.
    origin: u64,
}

/// The console, once [`init`] has installed it.
///
/// `None` before then: [`write_line`] silently discards anything logged
/// that early rather than panicking, since a panic there would have
/// nowhere to report itself.
///
/// Behind a `critical-section` mutex rather than in a `static mut`, so a
/// log from an interrupt handler cannot interleave with one from a task
/// and produce shredded output. The implementation of that critical
/// section is the application's to supply — `rpi-hal`'s `rt` feature has
/// one.
static CONSOLE: Mutex<RefCell<Option<Console>>> = Mutex::new(RefCell::new(None));

/// Installs `sink` as the console, stamping each line with `uptime`.
///
/// `uptime` returns microseconds from any fixed origin; only differences
/// are used, so it need not start at zero. It is read once here to fix
/// that origin and then once per line.
///
/// Call this before anything logs. Anything logged earlier is discarded —
/// see [`write_line`] — and a second call replaces the console, which
/// restarts the stamp from the new origin.
pub fn init(sink: &'static mut (dyn Write + Send), uptime: fn() -> u64) {
    let origin = uptime();
    critical_section::with(|cs| {
        *CONSOLE.borrow_ref_mut(cs) = Some(Console {
            sink,
            uptime,
            origin,
        });
    });
}

/// Writes one stamped line, or discards it if [`init`] has not run yet.
///
/// The backing function for [`logln!`](crate::logln); there is no reason
/// to call it directly.
///
/// The clock read, the stamp and the line all go out inside a single
/// `critical_section::with`. Reading the clock outside it would leave a
/// window in which an interrupt-time log lands between a timestamp and
/// the text it belongs to — the interleaving the mutex is here to
/// prevent, and worse than no stamp at all, since the resulting line
/// attributes one module's message to another module's clock reading.
pub fn write_line(args: Arguments) {
    critical_section::with(|cs| {
        if let Some(console) = CONSOLE.borrow_ref_mut(cs).as_mut() {
            let elapsed = (console.uptime)().saturating_sub(console.origin);
            let seconds = elapsed / 1_000_000;
            let milliseconds = elapsed % 1_000_000 / 1_000;

            // Padded to a fixed width so the messages line up in a column
            // and a long gap between two lines is visible as a step in
            // the stamp rather than something to be read digit by digit.
            // Five places holds a little over a day before the column
            // widens.
            let _ = console
                .sink
                .write_fmt(format_args!("[{seconds:>5}.{milliseconds:03}] "));
            let _ = console.sink.write_fmt(args);
            // CRLF rather than LF because the far end is usually a
            // terminal emulator on a serial port, which does not
            // translate the line ending for us.
            let _ = console.sink.write_str("\r\n");
        }
    });
}

/// Logs a line to the console, stamped with the uptime and with a
/// trailing CRLF.
///
/// Takes the same arguments as [`format_args!`]. Discards the line if
/// [`console::init`](crate::console::init) has not run yet, so it is safe
/// to call from anywhere, including before the console exists.
///
/// ```ignore
/// logln!("dhcp: bound {ip} in {ms}ms");
/// ```
#[macro_export]
macro_rules! logln {
    ($($arg:tt)*) => {
        $crate::console::write_line(format_args!($($arg)*))
    };
}
