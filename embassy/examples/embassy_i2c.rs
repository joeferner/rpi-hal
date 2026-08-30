#![no_std]
#![no_main]

// Async I2C: transfers that park on the controller's interrupt instead of
// spinning on its status register, checked against a heartbeat that must
// keep running while the bus is busy.
//
// rpi-hal's blocking `I2c` owns the core for the length of a transfer —
// under an executor that means every other task stops for it. The
// `embedded_hal_async::i2c::I2c` impl this exercises gives that time back.
// Four things are worth proving, and they are run in this order because
// the first is the one that can invalidate the design:
//
// 1. **A NAK comes back as a NAK.** `S.ERR` has no interrupt enable of its
//    own on this controller, so an address nobody answers depends on the
//    fault reaching the core through `DONE`. If that assumption is wrong
//    the future never wakes: the heartbeat below keeps ticking and the
//    `no-device probe` line never appears. That failure mode is why this
//    check runs first, and it needs nothing wired to the board.
//
// 2. **The executor keeps running during a transfer.** The heartbeat task
//    wakes every millisecond and records the longest gap it sees; the bus
//    task resets that before each transfer and reports it after. A gap of
//    about 1000us means the executor was scheduling other work throughout.
//    A gap the length of the transfer means it was blocked — which is what
//    the blocking driver would produce, and is the thing being fixed.
//
// 3. **A cancelled transfer leaves the bus usable.** `with_timeout` drops
//    a read part-way through, and the transfer immediately after it must
//    still succeed. A broken cleanup shows up on that *next* transfer, not
//    on the cancelled one, which is what makes it worth testing
//    deliberately rather than waiting to meet it.
//
// 4. **Data survives the round trip**, via a register read whose answer is
//    known.
//
// The bus is deliberately slowed to 10kHz (`i2c::divider_for`) so a 32-byte
// read takes about 30ms. At the usual 100kHz the whole transfer would fit
// inside one heartbeat interval and check 2 would prove nothing.
//
// The console is async as well, and that is not incidental: a blocking
// `write!` holds the core for the ~11ms a line takes at 115200 baud, which
// is longer than the transfer being measured, and check 2 would end up
// reporting the console instead of the bus. Three interrupt sources share
// one handler here — timer, I2C and UART.
//
// Wiring: none for check 1. Checks 2-4 need something that answers reads
// at `DEVICE` — an ADS1115 at 0x48 by default, which answers a read at any
// time. With nothing there, the example says so and keeps repeating check
// 1. Every check repeats on a one-second loop, because a latched `CLKT` or
// a waker left in a slot shows up on the second pass, not the first.

use core::fmt::Write as _;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, with_timeout};
use embedded_hal_async::i2c::I2c as _;
use embedded_io_async::Write as _;
use rpi_hal::halt;
use rpi_hal::i2c::{self, I2c};
use rpi_hal::mailbox::{ClockId, Mailbox};
use rpi_hal::{
    irq,
    lic::Lic,
    pac,
    timer::Timer,
    uart::{self, Uart},
};
use rpi_hal_embassy::{Executor, time_driver};

/// An address with nothing on it, for check 1. Inside the conventional
/// scan range and clear of everything this board carries.
const EMPTY: u8 = 0x21;

/// The device checks 2-4 talk to: an ADS1115. Any part that answers a read
/// without being asked for one first will do — the SHT41 at 0x44 will not,
/// since it NAKs a read with no conversion pending, which is a fine thing
/// to know and a poor thing to time.
const DEVICE: u8 = 0x48;

/// The ADS1115's config register, and its power-on value. Read back as
/// evidence that bytes survive the async path intact; printed rather than
/// asserted, since anything that has configured the part since power-on
/// will legitimately have changed it.
const CONFIG_REGISTER: u8 = 0x01;
const CONFIG_AT_RESET: u16 = 0x8583;

/// Bus rate. Slow on purpose — see the header.
const BUS_HZ: u32 = 10_000;

/// Bytes in the long read that checks 2 and 3 are timed against. 32 bytes
/// at 10kHz is roughly 30ms, comfortably longer than both the heartbeat
/// interval and the timeout in check 3.
const LONG_READ: usize = 32;

/// Longest gap the heartbeat has seen between its own wakeups, in
/// microseconds. Reset by the bus task before each transfer.
static MAX_GAP_US: AtomicU32 = AtomicU32::new(0);

/// Both tasks print, so the UART is shared. `NoopRawMutex` is right here:
/// they run on the same executor on one core.
type SharedUart = Mutex<NoopRawMutex, Uart>;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// Wakes every millisecond and records the longest interval between its
/// own wakeups — the measurement the whole example turns on.
///
/// The maximum rather than a count, because a count is ambiguous: a
/// `Ticker` that falls behind fires repeatedly to catch up, so a blocked
/// executor and a running one can end a transfer with the same number of
/// ticks behind them. The longest gap cannot be caught up.
#[embassy_executor::task]
async fn heartbeat() {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut last = Instant::now();

    loop {
        ticker.next().await;
        let now = Instant::now();
        MAX_GAP_US.fetch_max((now - last).as_micros() as u32, Ordering::Relaxed);
        last = now;
    }
}

/// Runs the four checks in a loop.
#[embassy_executor::task]
async fn bus(mut i2c: I2c<'static, pac::BSC1>, uart: &'static SharedUart) {
    let mut pass = 0u32;

    loop {
        pass += 1;
        say(uart, format_args!("\r\n-- pass {pass} --")).await;

        // Check 1: an address with nothing on it. Fast, and needs no
        // hardware beyond the board.
        let mut byte = [0u8; 1];
        let (result, elapsed, _) = timed(|| i2c.read(EMPTY, &mut byte)).await;
        match result {
            Err(e) => {
                say(
                    uart,
                    format_args!("no-device probe at 0x{EMPTY:02x}: {e:?} after {elapsed}us"),
                )
                .await
            }
            // A device answering here means `EMPTY` is the wrong constant
            // for this board, not that anything is broken.
            Ok(()) => {
                say(
                    uart,
                    format_args!("no-device probe at 0x{EMPTY:02x}: answered — pick another"),
                )
                .await
            }
        }

        // Is the device for the remaining checks actually present?
        if i2c.read(DEVICE, &mut byte).await.is_err() {
            say(
                uart,
                format_args!("nothing at 0x{DEVICE:02x}; checks 2-4 skipped"),
            )
            .await;
            embassy_time::Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        // Check 2: a long transfer, and what the rest of the executor was
        // doing during it.
        let mut long = [0u8; LONG_READ];
        let (result, elapsed, gap) = timed(|| i2c.read(DEVICE, &mut long)).await;
        say(
            uart,
            format_args!(
                "{LONG_READ}-byte read: {} in {elapsed}us, longest heartbeat gap {gap}us",
                outcome(&result)
            ),
        )
        .await;

        // Check 3: cancel one part-way, then use the bus again straight
        // afterwards. The second half is the real assertion.
        let timeout = Duration::from_millis(5);
        let cancelled = with_timeout(timeout, i2c.read(DEVICE, &mut long)).await;
        let mut config = [0u8; 2];
        let after = i2c
            .write_read(DEVICE, &[CONFIG_REGISTER], &mut config)
            .await;
        say(
            uart,
            format_args!(
                "cancelled after 5ms: {}, next transfer: {}",
                if cancelled.is_err() {
                    "dropped mid-transfer"
                } else {
                    "completed first — lower the timeout"
                },
                outcome(&after)
            ),
        )
        .await;

        // Check 4: bytes that mean something.
        if after.is_ok() {
            let value = u16::from_be_bytes(config);
            say(
                uart,
                format_args!(
                    "config register: 0x{value:04x} ({})",
                    if value == CONFIG_AT_RESET {
                        "power-on default"
                    } else {
                        "configured since power-on"
                    }
                ),
            )
            .await;
        }

        embassy_time::Timer::after(Duration::from_secs(1)).await;
    }
}

/// Runs one transfer, returning its result, how long it took, and the
/// longest heartbeat gap seen while it was in flight.
async fn timed<F, Fut>(transfer: F) -> (Result<(), rpi_hal::i2c::Error>, u64, u32)
where
    F: FnOnce() -> Fut,
    Fut: core::future::Future<Output = Result<(), rpi_hal::i2c::Error>>,
{
    MAX_GAP_US.store(0, Ordering::Relaxed);
    let started = Instant::now();
    let result = transfer().await;
    let elapsed = started.elapsed().as_micros();
    (result, elapsed, MAX_GAP_US.load(Ordering::Relaxed))
}

/// One line to the shared console.
async fn say(uart: &SharedUart, args: core::fmt::Arguments<'_>) {
    let mut line = heapless::String::<128>::new();
    let _ = write!(line, "{args}\r\n");
    let mut uart = uart.lock().await;
    let _ = uart.write_all(line.as_bytes()).await;
}

/// `ok`, or the error, for a transfer result.
fn outcome(result: &Result<(), rpi_hal::i2c::Error>) -> &'static str {
    match result {
        Ok(()) => "ok",
        Err(rpi_hal::i2c::Error::NoAcknowledge) => "not acknowledged",
        Err(rpi_hal::i2c::Error::Timeout) => "timed out",
        Err(rpi_hal::i2c::Error::Incomplete { .. }) => "short",
        Err(rpi_hal::i2c::Error::ZeroLengthUnsupported) => "zero length",
    }
}

// `unsafe(no_mangle)` rather than the bare `no_mangle` rpi-hal's own
// examples use: this crate is edition 2024, which requires the wrapper.
#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "embassy: async I2C");

    // The divider is computed from the core clock the firmware reports,
    // not from the datasheet's nominal one — on a board running its core
    // at 250MHz the difference is a bus two thirds faster than asked for,
    // which would quietly shrink the transfer this example is timing.
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    let core_hz = match mailbox.clock_rate_hz(ClockId::Core) {
        Ok(hz) => hz,
        Err(e) => {
            let _ = writeln!(uart, "core clock query failed ({e:?}); assuming 250MHz");
            250_000_000
        }
    };
    let divider = i2c::divider_for(core_hz, BUS_HZ);
    let _ = writeln!(
        uart,
        "core {} MHz, CDIV {divider} -> {} Hz",
        core_hz / 1_000_000,
        core_hz / divider as u32
    );

    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);

    // Route the I2C line to the CPU. The transfers open the controller's
    // own DONE/TXW/RXR conditions as they park; this is the
    // interrupt-controller gate, which belongs to the application — and
    // it is one line for both BSC0 and BSC1, which share it.
    lic.enable_i2c_irq();
    // And the UART's, because the console below is async too. Printing
    // through the blocking `write!` would stall the executor for the ~11ms
    // a line takes at 115200 baud, which is longer than the transfer being
    // measured — the heartbeat would then be reporting the console rather
    // than the bus. Forgetting this gate parks the first line forever, the
    // TX interrupt it waits on never reaching the core.
    lic.enable_uart_irq();
    irq::enable_irq();

    // Sound only because `kmain` never returns — the same lifetime
    // widening `#[embassy_executor::main]` performs.
    unsafe fn make_static<T>(t: &mut T) -> &'static mut T {
        unsafe { core::mem::transmute(t) }
    }

    // A second System Timer handle: the driver bounds each transfer
    // against it, and `time_driver::init` above took the first one. Both
    // only ever read the free-running counter, and the compare register
    // the time driver owns is untouched here.
    let mut i2c_timer = Timer::new(unsafe { pac::Peripherals::steal() }.SYSTMR);
    let i2c_timer: &'static Timer = unsafe { make_static(&mut i2c_timer) };
    let i2c = I2c::<pac::BSC1>::init(
        &peripherals.GPIO,
        unsafe { pac::Peripherals::steal() }.BSC1,
        divider,
        i2c_timer,
    );

    let mut shared = SharedUart::new(uart);
    let uart: &'static SharedUart = unsafe { make_static(&mut shared) };

    let mut executor = Executor::new();
    let executor = unsafe { make_static(&mut executor) };

    executor.run(|spawner| {
        spawner.spawn(heartbeat().unwrap());
        spawner.spawn(bus(i2c, uart).unwrap());
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn __irq_handler() {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);

    // Checked independently, not else-if: both can be pending in one
    // entry, and each is serviced by the crate that owns the source.
    if lic.is_timer1_pending() {
        time_driver::on_timer_irq();
    }

    if lic.is_i2c_pending() {
        // Masks whichever controller raised it and wakes the transfer
        // parked there. A controller being driven by the blocking API
        // would be left alone — it arms none of these conditions.
        i2c::on_irq();
    }

    if lic.is_uart_pending() {
        uart::on_irq();
    }
}
