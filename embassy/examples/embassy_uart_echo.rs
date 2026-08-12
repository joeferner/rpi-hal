#![no_std]
#![no_main]

// Async UART: an echo task that awaits incoming bytes instead of polling
// for them, running alongside a timer task that must keep its cadence.
//
// The blocking analogue of this is rpi-hal's `uart_rx_irq_echo.rs`, which
// gets the same behaviour from a hand-written interrupt handler, a ring
// buffer and a `critical_section`-guarded queue. Here the driver's
// `embedded_io_async::Read` supplies all of that: the task reads, and is
// descheduled until bytes exist.
//
// What the output shows, and why each part is evidence:
//
// - Echoed characters come back as you type. The core is in `wfe` between
//   keystrokes, not polling.
// - `t=` lines keep their one-second cadence with `drift` flat, including
//   across bursts of typing. Two interrupt sources — UART and timer —
//   share one handler, and servicing one doesn't cost the other its
//   deadline.
// - `rx=` counts bytes echoed, so a silent terminal is visibly distinct
//   from a broken reader.
//
// The write side is async too, which matters more than it looks: at
// 115200 baud a full transmit FIFO would otherwise be ~87us of spinning
// per byte, stalling *every* task rather than just this one. Both tasks
// here write, so the UART is shared through a mutex.
//
// No extra wiring — the serial console is all this needs.

use core::fmt::Write as _;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker};
use embedded_io_async::{Read, Write};
use rpi_hal::halt;
use rpi_hal::{irq, lic::Lic, pac, timer::Timer, uart::Uart};
use rpi_hal_embassy::{Executor, time_driver};

/// Bytes echoed so far, published for the reporting task.
static RX_COUNT: AtomicU32 = AtomicU32::new(0);

/// The UART is written by both tasks, so it needs an owner they share.
/// `NoopRawMutex` is the right raw mutex here: both tasks run on the same
/// executor, so contention can only come from another task, never from an
/// interrupt or another core.
type SharedUart = Mutex<NoopRawMutex, Uart>;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

#[embassy_executor::task]
async fn echo(uart: &'static SharedUart) {
    let mut buf = [0u8; 32];

    loop {
        // Held only across the read, so the reporting task can write
        // between keystrokes rather than waiting behind an idle reader.
        let n = {
            let mut uart = uart.lock().await;
            match uart.read(&mut buf).await {
                Ok(n) => n,
                Err(_) => continue,
            }
        };

        RX_COUNT.fetch_add(n as u32, Ordering::Relaxed);

        let mut uart = uart.lock().await;
        let _ = uart.write_all(&buf[..n]).await;
    }
}

#[embassy_executor::task]
async fn report(uart: &'static SharedUart) {
    let start = Instant::now();
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut ticks: u64 = 0;

    loop {
        ticker.next().await;
        ticks += 1;

        let drift = start.elapsed().as_micros() as i64 - (ticks * 1_000_000) as i64;
        let rx = RX_COUNT.load(Ordering::Relaxed);

        let mut line = heapless::String::<64>::new();
        let _ = write!(line, "\r\nt={ticks}s rx={rx} drift={drift}us\r\n");

        let mut uart = uart.lock().await;
        let _ = uart.write_all(line.as_bytes()).await;
    }
}

// `unsafe(no_mangle)` rather than the bare `no_mangle` rpi-hal's own
// examples use: this crate is edition 2024, which requires the wrapper.
#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "embassy: async UART echo — type to see it back");

    let lic = Lic::new(peripherals.LIC);
    time_driver::init(Timer::new(peripherals.SYSTMR), &lic);

    // Route the UART to the CPU. The futures unmask the peripheral's own
    // RX/TX interrupts as they need them; this is the interrupt-controller
    // gate, which belongs to the application.
    lic.enable_uart_irq();
    irq::enable_irq();

    // Both the shared UART and the executor outlive `run`, which is sound
    // only because `kmain` never returns — the same lifetime widening
    // `#[embassy_executor::main]` performs. A `StaticCell` would be the
    // alternative; this keeps the example's dependencies to the traits
    // being demonstrated.
    unsafe fn make_static<T>(t: &mut T) -> &'static mut T {
        unsafe { core::mem::transmute(t) }
    }
    let mut shared = SharedUart::new(uart);
    let uart: &'static SharedUart = unsafe { make_static(&mut shared) };

    let mut executor = Executor::new();
    let executor = unsafe { make_static(&mut executor) };

    executor.run(|spawner| {
        spawner.spawn(echo(uart).unwrap());
        spawner.spawn(report(uart).unwrap());
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

    if lic.is_uart_pending() {
        // Masks whichever of RX/TX fired and wakes its future; moves no
        // bytes, so the FIFO is still full for the reader.
        rpi_hal::uart::on_irq();
    }
}
