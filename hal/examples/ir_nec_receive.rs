#![no_std]
#![no_main]

// Decode a NEC-protocol IR remote and print each button press over UART0.
//
// This is the receive half of an IR remote stack. The Pi never demodulates
// the 38 kHz carrier itself -- that's done by an IR receiver module (a TSOP-
// style part), which outputs a clean, already-demodulated logic signal:
// idle high, pulled low for each carrier burst ("mark"). All the Pi has to
// do is measure the mark/space timings and run them through a protocol
// decoder. That decoder is deliberately NOT in rpi-hal: NEC/RC5/Sony framing
// is chip-agnostic protocol logic, the same category as TCP/IP or FAT, so it
// lives in an external crate (`infrared`) exactly like `smoltcp` and
// `embedded-sdmmc` do. rpi-hal only supplies the two primitives it needs:
// GPIO edge interrupts (Pin::enable_interrupt) and a microsecond timebase
// (Timer::now_micros).
//
// How the capture works: the receiver pin is armed for AnyEdge interrupts.
// Every transition wakes __irq_handler, which reads the system timer, hands
// `infrared` the microseconds elapsed since the previous edge plus the pin's
// settled level, and lets its NEC state machine accumulate bits. When a full
// frame decodes, the handler drops the address/command into a ring buffer
// that the main loop drains and prints, so the UART writes happen outside
// interrupt context (same ISR-to-mainline hand-off as uart_rx_irq_echo.rs).
//
// `infrared`'s manual event API expects `edge = pin.is_low()` (see its own
// pin-based path, which passes `self.pin.is_low()`): true means the line is
// at the active/mark level of an active-low receiver module. We mirror that
// exactly rather than passing a rising/falling distinction.
//
// Wiring: connect the IR receiver module's OUT to GPIO23 (header pin 16),
// VCC to 3V3, GND to GND. Most modules have an internal pull-up, so the pin
// idles high on its own; rpi-hal doesn't drive GPPUD yet, so a module
// without one would need an external pull-up to 3V3.

use core::cell::RefCell;
use core::fmt::Write;
use critical_section::Mutex;
use embedded_hal::digital::InputPin;
use infrared::protocol::nec::NecCommand;
use infrared::protocol::Nec;
use infrared::receiver::NoPin;
use infrared::Receiver;
use rpi_hal::gpio::{Input, Pin, Trigger};
use rpi_hal::halt;
use rpi_hal::{irq, lic::Lic, pac, timer::Timer, uart::Uart};

/// IR receiver module output (GPIO23, header pin 16).
const IR_PIN: u8 = 23;

/// Timer resolution handed to `infrared`: 1 MHz means the `dt` we feed it is
/// plain microseconds, which is exactly what `Timer::now_micros` counts.
const IR_FREQ_HZ: u32 = 1_000_000;

/// Manual, no-pin NEC receiver over a `u32` microsecond monotonic. The
/// handler feeds edges explicitly, so `infrared` never touches a pin itself.
type IrReceiver = Receiver<Nec, NoPin, u32, NecCommand>;

/// Shared state between the ISR (which advances the decoder) and `kmain`
/// (which sets it up). `last_us` carries the previous edge's timestamp so the
/// handler can compute the delta the decoder needs.
struct IrState {
    receiver: IrReceiver,
    last_us: u64,
}

static IR_STATE: Mutex<RefCell<Option<IrState>>> = Mutex::new(RefCell::new(None));

const DECODED_CAPACITY: usize = 16;

/// A decoded button press, copied out of the ISR for the main loop to print.
#[derive(Copy, Clone)]
struct Decoded {
    addr: u8,
    cmd: u8,
    repeat: bool,
}

/// Bounded ring buffer of decoded presses, same drop-when-full policy as the
/// RX buffer in uart_rx_irq_echo.rs: a burst the main loop hasn't drained yet
/// loses the oldest overflow rather than blocking the ISR.
struct DecodedQueue {
    buf: [Decoded; DECODED_CAPACITY],
    head: usize,
    tail: usize,
    len: usize,
}

impl DecodedQueue {
    const fn new() -> Self {
        Self {
            buf: [Decoded {
                addr: 0,
                cmd: 0,
                repeat: false,
            }; DECODED_CAPACITY],
            head: 0,
            tail: 0,
            len: 0,
        }
    }

    fn push(&mut self, d: Decoded) {
        if self.len == DECODED_CAPACITY {
            return;
        }
        self.buf[self.tail] = d;
        self.tail = (self.tail + 1) % DECODED_CAPACITY;
        self.len += 1;
    }

    fn pop(&mut self) -> Option<Decoded> {
        if self.len == 0 {
            return None;
        }
        let d = self.buf[self.head];
        self.head = (self.head + 1) % DECODED_CAPACITY;
        self.len -= 1;
        Some(d)
    }
}

static DECODED: Mutex<RefCell<DecodedQueue>> = Mutex::new(RefCell::new(DecodedQueue::new()));

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
    let _ = writeln!(
        uart,
        "ir_nec_receive: point a NEC remote at the receiver on GPIO{IR_PIN}"
    );

    // Arm the receiver pin for both edges. Like the other IRQ examples, this
    // configures the pin but doesn't hold it; __irq_handler re-wraps it via
    // assume_mode each time.
    let ir = Pin::<IR_PIN, Input>::new(peripherals.GPIO).into_input();
    ir.enable_interrupt(Trigger::AnyEdge);
    // Discard any edge latched while detection was being armed.
    ir.clear_interrupt();

    let receiver = Receiver::builder().nec().frequency(IR_FREQ_HZ).build();
    critical_section::with(|cs| {
        *IR_STATE.borrow_ref_mut(cs) = Some(IrState {
            receiver,
            last_us: 0,
        });
    });

    let lic = Lic::new(peripherals.LIC);
    lic.enable_gpio_irq(IR_PIN);

    irq::enable_irq();

    loop {
        let decoded = critical_section::with(|cs| DECODED.borrow_ref_mut(cs).pop());
        match decoded {
            Some(d) => {
                let _ = writeln!(
                    uart,
                    "addr=0x{:02x} cmd=0x{:02x}{}",
                    d.addr,
                    d.cmd,
                    if d.repeat { " (repeat)" } else { "" }
                );
            }
            None => unsafe { core::arch::asm!("wfe") },
        }
    }
}

#[no_mangle]
pub extern "C" fn __irq_handler() {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);

    if !lic.is_gpio_pending(IR_PIN) {
        return;
    }

    // The bank line is shared across a pin range, so confirm this pin latched
    // the event before acting, then ack it.
    let mut ir = unsafe { Pin::<IR_PIN, Input>::assume_mode(peripherals.GPIO) };
    if !ir.is_interrupt_pending() {
        return;
    }
    ir.clear_interrupt();

    // Timestamp the edge and read the settled level. `infrared` wants
    // `edge = is_low()` (active-low mark), matching its own pin-based path.
    let now = Timer::new(peripherals.SYSTMR).now_micros();
    let edge = ir.is_low().unwrap_or(false);

    critical_section::with(|cs| {
        let mut state = IR_STATE.borrow_ref_mut(cs);
        let Some(state) = state.as_mut() else {
            return;
        };

        // Microseconds since the previous edge. `wrapping_sub` keeps the very
        // first edge (last_us == 0) and any 64-bit wrap harmless: a bogus-huge
        // gap just resets the decoder, which is the correct idle behaviour.
        let dt = now.wrapping_sub(state.last_us) as u32;
        state.last_us = now;

        if let Ok(Some(cmd)) = state.receiver.event(dt, edge) {
            DECODED.borrow_ref_mut(cs).push(Decoded {
                addr: cmd.addr,
                cmd: cmd.cmd,
                repeat: cmd.repeat,
            });
        }
    });
}
