#![no_std]
#![no_main]

// IRQ-driven analogue of uart_echo.rs, and the multi-source dispatch
// proof: the timer (500ms LED heartbeat, same as irq_timer_blink.rs)
// and UART RX are both enabled at once, so __irq_handler has to check
// which source(s) actually fired instead of assuming there's only one
// possible cause -- checked independently, not else-if, since nothing
// rules out both being pending in the same entry.
//
// The ring buffer below is deliberately example-owned, not part of
// rpi-hal: buffer capacity/overflow policy is an application choice,
// not a hardware abstraction, and neither embedded-hal nor embedded-io
// mandate any particular buffering scheme either -- rpi-hal only
// provides the peripheral-level primitives (Uart::try_read_byte/
// enable_rx_irq/clear_rx_irq). This is also the first real consumer of
// critical_section::with: nothing before this has had genuine shared
// mutable state between an ISR and main-line code.
//
// Wiring: an LED (with a series resistor) from GPIO4 (header pin 7) to
// GND, the same pin blink.rs drives. Without it the echo half still
// works over the serial console -- the LED is what shows the timer
// source is being serviced alongside UART RX rather than starved by it.

use core::cell::RefCell;
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};
use critical_section::Mutex;
use embedded_hal::digital::OutputPin;
use rpi_hal::gpio::{Input, Output, Pin};
use rpi_hal::halt;
use rpi_hal::{irq, lic::Lic, pac, timer::Timer, uart::Uart};

const PERIOD_US: u32 = 500_000;
const RX_BUFFER_CAPACITY: usize = 64;

// A real `AtomicBool` instead of a `static mut` toggled via
// `read_volatile`/`write_volatile` -- this only ever has one actual
// writer (this ISR; IRQ is masked for the duration of a handler, so
// there's no genuine cross-context contention here to stress-test),
// but it's the simplest possible real exercise of `ldrex`/`strex`
// (`fetch_xor`, below) now that the MMU maps RAM as real Cacheable,
// Shareable Normal memory.
static LED_ON: AtomicBool = AtomicBool::new(false);

struct RingBuffer {
    buf: [u8; RX_BUFFER_CAPACITY],
    head: usize,
    tail: usize,
    len: usize,
}

impl RingBuffer {
    const fn new() -> Self {
        Self {
            buf: [0; RX_BUFFER_CAPACITY],
            head: 0,
            tail: 0,
            len: 0,
        }
    }

    /// Drops the byte if the buffer is already full -- bounded, not
    /// unbounded, same philosophy as the bounded RX drain in
    /// `Uart::init`.
    fn push(&mut self, byte: u8) {
        if self.len == RX_BUFFER_CAPACITY {
            return;
        }
        self.buf[self.tail] = byte;
        self.tail = (self.tail + 1) % RX_BUFFER_CAPACITY;
        self.len += 1;
    }

    fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        let byte = self.buf[self.head];
        self.head = (self.head + 1) % RX_BUFFER_CAPACITY;
        self.len -= 1;
        Some(byte)
    }
}

static RX_BUFFER: Mutex<RefCell<RingBuffer>> = Mutex::new(RefCell::new(RingBuffer::new()));

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "Starting...");

    // Only configuring the pin here; `__irq_handler` re-wraps it via
    // `assume_mode` each time rather than holding onto this value,
    // same as it already does for `Timer`/`Lic`.
    Pin::<4, Input>::new(peripherals.GPIO).into_output();

    let timer = Timer::new(peripherals.SYSTMR);
    timer.arm_periodic_c1(PERIOD_US);

    let lic = Lic::new(peripherals.LIC);
    lic.enable_timer1_irq();
    lic.enable_uart_irq();

    uart.enable_rx_irq();

    irq::enable_irq();

    loop {
        let byte = critical_section::with(|cs| RX_BUFFER.borrow_ref_mut(cs).pop());
        match byte {
            Some(byte) => uart.write_byte(byte),
            None => unsafe { core::arch::asm!("wfe") },
        }
    }
}

#[no_mangle]
pub extern "C" fn __irq_handler() {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);

    if lic.is_timer1_pending() {
        let timer = Timer::new(peripherals.SYSTMR);
        timer.ack_c1(PERIOD_US);

        // `fetch_xor` toggles and returns the *previous* value in one
        // real ldrex/strex read-modify-write, not two separate
        // volatile accesses.
        let on = !LED_ON.fetch_xor(true, Ordering::Relaxed);

        // Safe: `kmain` already configured pin 4 as an output before
        // enabling the IRQ that leads here.
        let mut led = unsafe { Pin::<4, Output>::assume_mode(peripherals.GPIO) };
        let _ = if on { led.set_high() } else { led.set_low() };
    }

    if lic.is_uart_pending() {
        let uart = Uart::from_initialized(peripherals.UART0);
        while let Some(byte) = uart.try_read_byte() {
            critical_section::with(|cs| RX_BUFFER.borrow_ref_mut(cs).push(byte));
        }
        uart.clear_rx_irq();
    }
}
