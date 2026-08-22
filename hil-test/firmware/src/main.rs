//! Hardware-in-the-loop bench fixture firmware.
//!
//! One USB device with two interfaces:
//!
//! - **CDC ACM** — the board's console, bridged from a UART. It has to look
//!   like an ordinary serial port because `rpi-loader`'s CLI opens one, and
//!   its baud follows whatever the host sets so the loader's negotiation up
//!   to 1.5 Mbaud and back needs no cooperation here.
//! - **Vendor bulk** — commands and, later, capture data, spoken over
//!   libusb. See `proto.rs`.
//!
//! Bridging the console rather than wiring a separate USB-serial adapter to
//! the board's UART pins is what makes those pins testable at all: a second
//! permanently-attached driver on that net would fight the fixture whenever a
//! case drove them.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIO0, UART0, USB};
use embassy_rp::pio;
use embassy_rp::uart::{self, BufferedUart};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::{EndpointIn, EndpointOut};
use embassy_usb::{Builder, Config, UsbDevice};
// `portable_atomic` rather than `core::sync::atomic`: Cortex-M0+ has no
// atomic read-modify-write, so `swap` on a core `AtomicBool` does not exist
// on this target. These get it through a critical section.
use portable_atomic::{AtomicBool, Ordering};
use static_cell::StaticCell;

use panic_halt as _;

mod board;
mod console_pins;
mod marker;
mod proto;

use proto::{Cmd, Status, FIRMWARE_VERSION, PROTOCOL_VERSION};

bind_interrupts!(pub struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
    // Never actually fires: the marker capture is drained by DMA and polled
    // over the control interface, so nothing awaits a FIFO. `Pio::new` takes
    // the binding regardless, and leaving the vector unpopulated would turn
    // any future use of the async PIO API into a silent hang rather than a
    // compile error.
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

/// pid.codes test allocation. Fine for a bench fixture that never ships;
/// matching on VID/PID is how the runner finds the control interface without
/// caring which `ttyACM` number the console landed on.
const USB_VID: u16 = 0x1209;
const USB_PID: u16 = 0x0001;

/// Full-speed bulk endpoints are 64 bytes, so one packet is one exchange.
const PACKET: u16 = 64;

/// Set once the runner has successfully spoken to the control interface.
static HOST_SEEN: AtomicBool = AtomicBool::new(false);

/// Set whenever console bytes move in either direction, and cleared by the
/// status LED once it has shown them.
static CONSOLE_ACTIVITY: AtomicBool = AtomicBool::new(false);

/// False while a case has borrowed GPIO14/15, i.e. between `CONSOLE_DETACH`
/// and `CONSOLE_ATTACH`.
///
/// The bridge reads it to decide whether host bytes go to the board or in the
/// bin; the pins themselves are moved by `console_pins`. Starts attached,
/// because the resting state of the fixture is a working console.
static CONSOLE_ATTACHED: AtomicBool = AtomicBool::new(true);

/// Blinks the on-board LED to say what the fixture is doing.
///
/// Worth the pin because the fixture's only other output channel *is* USB:
/// if the stack fails to come up, or the host never opens the port, there is
/// otherwise no way to tell a hung board from an unplugged cable from a
/// wrong VID/PID. The pattern distinguishes those without a debugger:
///
/// - **Fast even blink** — running, but no host has spoken to the control
///   interface yet. Suspect cabling, udev, or the runner.
/// - **Short pulse once a second** — the runner has talked to us and the
///   fixture is idle. This is the healthy resting state.
/// - **Double pulse** — console bytes are moving, i.e. the board under test
///   is talking through the bridge.
/// - **Dark** — the firmware is not running at all.
#[embassy_executor::task]
async fn status_led(mut led: Output<'static>) -> ! {
    loop {
        if CONSOLE_ACTIVITY.swap(false, Ordering::Relaxed) {
            for _ in 0..2 {
                led.set_high();
                Timer::after(Duration::from_millis(40)).await;
                led.set_low();
                Timer::after(Duration::from_millis(80)).await;
            }
            Timer::after(Duration::from_millis(200)).await;
        } else if HOST_SEEN.load(Ordering::Relaxed) {
            led.set_high();
            Timer::after(Duration::from_millis(50)).await;
            led.set_low();
            Timer::after(Duration::from_millis(950)).await;
        } else {
            led.set_high();
            Timer::after(Duration::from_millis(100)).await;
            led.set_low();
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

/// Drives the USB device in its own task.
///
/// Deliberately not joined with the console and control futures: anything
/// that spins instead of awaiting would starve whatever it shares a task
/// with, and if that victim is the USB stack the board never enumerates at
/// all — a failure with no console to report it on, since the console *is*
/// the USB stack.
#[embassy_executor::task]
async fn usb_task(mut device: UsbDevice<'static, Driver<'static, USB>>) -> ! {
    device.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // The Pico's on-board LED, which is on GP25 and not brought out to the
    // header. Started before anything else so a hang during USB or UART
    // setup still leaves a visible "firmware is running" signal rather than
    // a dark board indistinguishable from a failed flash.
    //
    // Two boards this is wrong for. A Pico *W* puts its LED on the CYW43
    // chip's WL_GPIO0, not GP25, so this would drive nothing and the LED
    // would stay dark — the one reading that means "did not boot". And the
    // PICO2-XL's LED pin has not been checked against Olimex's schematic,
    // so the rp235x build inherits GP25 on no evidence. Confirm before
    // trusting a dark LED on either.
    spawner.must_spawn(status_led(Output::new(p.PIN_25, Level::Low)));

    // GP0/GP1 face the board's console pins: fixture TX into the board's
    // RXD, fixture RX from the board's TXD.
    static TX_BUF: StaticCell<[u8; 512]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 2048]> = StaticCell::new();
    let mut uart_config = uart::Config::default();
    uart_config.baudrate = board::CONSOLE_DEFAULT_BAUD;
    let uart = BufferedUart::new(
        p.UART0,
        p.PIN_0,
        p.PIN_1,
        Irqs,
        TX_BUF.init([0; 512]),
        RX_BUF.init([0; 2048]),
        uart_config,
    );

    let driver = Driver::new(p.USB, Irqs);

    let mut config = Config::new(USB_VID, USB_PID);
    config.manufacturer = Some("rpi-hal");
    config.product = Some("HIL bench fixture");
    config.serial_number = Some("hil-0001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static CDC_STATE: StaticCell<State> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    let cdc = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), PACKET);

    // A vendor-specific interface rather than a second CDC: this channel
    // carries binary bodies and, later, capture buffers, and going through a
    // tty would mean both an encoding overhead and a line discipline with
    // opinions about the bytes.
    let mut func = builder.function(0xff, 0x00, 0x00);
    let mut iface = func.interface();
    let mut alt = iface.alt_setting(0xff, 0x00, 0x00, None);
    let mut ctl_in = alt.endpoint_bulk_in(None, PACKET);
    let mut ctl_out = alt.endpoint_bulk_out(None, PACKET);
    drop(func);

    spawner.must_spawn(usb_task(builder.build()));

    // GP2 watches the board's marker pin. Started here rather than on the
    // first `MARKER_ARM` so the counter is already free-running when a capture
    // is armed: bringing the state machine up inside the arm would put its
    // start-up transient at the front of every capture.
    let mut marker = marker::Marker::new(p.PIO0, p.PIN_2);

    join(
        console_bridge(cdc, uart),
        control_loop(&mut ctl_out, &mut ctl_in, &mut marker),
    )
    .await;
}

/// Bridges the CDC interface to the board's UART in both directions, and
/// follows the host's requested baud.
///
/// Baud tracking is why this owns the whole `BufferedUart` rather than
/// spawning a task per direction: `set_baudrate` lives on the undivided
/// peripheral, not on the split halves. So the two directions run against
/// `split_ref` borrows that are released each time the select resolves,
/// which is when a rate change can be applied. It matters because the loader
/// idles at 115200 and negotiates up to 1.5 Mbaud mid-session — the bridge
/// follows that without knowing anything about the loader's protocol, since
/// the host's `tcsetattr` arrives here as an ordinary line-coding request.
///
/// The cost is that a control event cancels whichever transfer was in
/// flight. Both sides are cancel-safe — unread OUT packets are not ACKed and
/// UART bytes stay in the ring buffer — so this costs a wakeup, not data.
async fn console_bridge(cdc: CdcAcmClass<'static, Driver<'static, USB>>, mut uart: BufferedUart) {
    use embassy_futures::select::{select3, Either3};
    use embedded_io_async::{Read, Write};

    let (mut cdc_tx, mut cdc_rx, cdc_ctl) = cdc.split_with_control();
    let mut baud = board::CONSOLE_DEFAULT_BAUD;
    let mut to_board_buf = [0u8; PACKET as usize];
    let mut from_board_buf = [0u8; PACKET as usize];

    loop {
        let (uart_tx, uart_rx) = uart.split_ref();

        let to_board = async {
            let n = cdc_rx.read_packet(&mut to_board_buf).await.ok()?;
            CONSOLE_ACTIVITY.store(true, Ordering::Relaxed);
            // Discarded rather than queued while the pins are released.
            // Writing them would put them in the UART's TX FIFO, where they
            // would sit with the pad muxed away and then transmit the instant
            // the bridge is restored — injecting stale bytes into the board's
            // receiver at the one moment it is re-establishing its console.
            // Losing them is what `CONSOLE_DETACH` promises; delivering them
            // late is a corruption nobody asked for.
            if !CONSOLE_ATTACHED.load(Ordering::Relaxed) {
                return Some(());
            }
            uart_tx.write_all(&to_board_buf[..n]).await.ok()
        };

        let from_board = async {
            // Whatever the ring buffer holds is forwarded immediately rather
            // than waiting for a full packet, so a console line appears on
            // the host as it is printed.
            //
            // A zero-length read is looped on rather than returned: returning
            // would resolve the select below, and if the read completes
            // immediately every time, that becomes a busy loop that starves
            // whatever else shares this task.
            loop {
                let n = uart_rx.read(&mut from_board_buf).await.ok()?;
                if n == 0 {
                    continue;
                }
                CONSOLE_ACTIVITY.store(true, Ordering::Relaxed);
                return cdc_tx.write_packet(&from_board_buf[..n]).await.ok();
            }
        };

        let changed = match select3(to_board, from_board, cdc_ctl.control_changed()).await {
            Either3::Third(()) => true,
            Either3::First(_) | Either3::Second(_) => false,
        };

        // Borrows above are released here, so the peripheral can be
        // reconfigured.
        if changed {
            let requested = cdc_rx.line_coding().data_rate();
            if requested != 0 && requested != baud {
                baud = requested;
                uart.set_baudrate(baud);
            }
        }
    }
}

/// Serves the control interface: one request packet in, one response out.
async fn control_loop<O, I>(out_ep: &mut O, in_ep: &mut I, marker: &mut marker::Marker<'_>)
where
    O: EndpointOut,
    I: EndpointIn,
{
    let mut req = [0u8; PACKET as usize];
    let mut resp = [0u8; PACKET as usize];

    loop {
        out_ep.wait_enabled().await;
        let Ok(n) = out_ep.read(&mut req).await else {
            continue;
        };
        let len = handle(&req[..n], &mut resp, marker);
        if in_ep.write(&resp[..len]).await.is_ok() {
            HOST_SEEN.store(true, Ordering::Relaxed);
        }
    }
}

/// Handles one request, writing the response into `resp` and returning its
/// length. Split out from the endpoint plumbing so it stays testable.
fn handle(req: &[u8], resp: &mut [u8], marker: &mut marker::Marker<'_>) -> usize {
    // Truncates rather than panicking on an oversized body: a response that
    // cannot fit its packet is a firmware bug, and losing the tail of one
    // reply beats faulting the fixture out from under a running suite.
    let reply = |resp: &mut [u8], status: Status, body: &[u8]| -> usize {
        let n = body.len().min(proto::MAX_BODY);
        resp[0] = status as u8;
        resp[1] = n as u8;
        resp[2..2 + n].copy_from_slice(&body[..n]);
        2 + n
    };

    // Every request is at least an opcode and a body length.
    if req.len() < 2 {
        return reply(resp, Status::BadArgs, &[]);
    }
    let body_len = req[1] as usize;
    if body_len > proto::MAX_BODY || req.len() < 2 + body_len {
        return reply(resp, Status::BadArgs, &[]);
    }

    let Some(cmd) = Cmd::from_u8(req[0]) else {
        return reply(resp, Status::BadCommand, &[]);
    };

    match cmd {
        Cmd::Ping => reply(resp, Status::Ok, &[]),
        Cmd::Hello => {
            let caps = board::CAPABILITIES.to_le_bytes();
            let body = [
                PROTOCOL_VERSION,
                board::BOARD as u8,
                caps[0],
                caps[1],
                caps[2],
                caps[3],
                FIRMWARE_VERSION[0],
                FIRMWARE_VERSION[1],
                FIRMWARE_VERSION[2],
            ];
            reply(resp, Status::Ok, &body)
        }
        // Idempotent on purpose. The runner's recovery path reattaches a
        // console it is not sure it detached, and a second detach must not be
        // an error there — the alternative is every caller tracking a state
        // the fixture already knows.
        Cmd::ConsoleDetach => {
            CONSOLE_ATTACHED.store(false, Ordering::Relaxed);
            console_pins::release();
            reply(resp, Status::Ok, &[])
        }
        Cmd::ConsoleAttach => {
            console_pins::reconnect();
            CONSOLE_ATTACHED.store(true, Ordering::Relaxed);
            reply(resp, Status::Ok, &[])
        }
        Cmd::ConsoleStatus => {
            let attached = u8::from(CONSOLE_ATTACHED.load(Ordering::Relaxed));
            reply(resp, Status::Ok, &[attached])
        }
        Cmd::ConsolePins => reply(resp, Status::Ok, &[console_pins::levels()]),
        // Refused while attached, and this is the interlock that makes the
        // 1:1 shadow safe to build on: driving a pad the bridge's UART is
        // still muxed onto puts two drivers on one net, which is a short
        // whenever they disagree. The fixture cannot tell whether the *board*
        // has let go, but it knows perfectly well whether it has itself, so
        // it enforces the half it can see rather than trusting the caller
        // with both.
        Cmd::ConsoleDrive => {
            if body_len != 2 {
                return reply(resp, Status::BadArgs, &[]);
            }
            if CONSOLE_ATTACHED.load(Ordering::Relaxed) {
                return reply(resp, Status::BadState, &[]);
            }
            console_pins::drive(req[2], req[3]);
            reply(resp, Status::Ok, &[])
        }

        Cmd::MarkerArm => {
            marker.arm();
            reply(resp, Status::Ok, &[])
        }
        Cmd::MarkerStatus => {
            let count = marker.captured().to_le_bytes();
            let flags = u8::from(marker.overflowed());
            let hz = marker.tick_hz().to_le_bytes();
            let capacity = (marker::CAPACITY as u16).to_le_bytes();
            let body = [
                count[0],
                count[1],
                flags,
                hz[0],
                hz[1],
                hz[2],
                hz[3],
                capacity[0],
                capacity[1],
                marker.pin(),
            ];
            reply(resp, Status::Ok, &body)
        }
        Cmd::MarkerRead => {
            if body_len != 3 {
                return reply(resp, Status::BadArgs, &[]);
            }
            let start = u16::from_le_bytes([req[2], req[3]]) as usize;
            let count = req[4] as usize;
            if start >= marker::CAPACITY || count > proto::MAX_BODY / 4 {
                return reply(resp, Status::BadArgs, &[]);
            }
            let mut body = [0u8; proto::MAX_BODY];
            let written = marker.read(start, count, &mut body);
            reply(resp, Status::Ok, &body[..written])
        }
        // Bounded here rather than trusted to the caller: this busy-waits, and
        // it shares an executor with the USB stack. A request for a second's
        // worth of pulses would stop answering the host long enough for the
        // runner's own timeout to fire, which presents as a dead fixture.
        Cmd::MarkerPulse => {
            if body_len != 4 {
                return reply(resp, Status::BadArgs, &[]);
            }
            let count = u16::from_le_bytes([req[2], req[3]]);
            let half_period_us = u16::from_le_bytes([req[4], req[5]]);
            let total_us = (count as u32) * 2 * (half_period_us as u32);
            if count == 0 || total_us > MAX_PULSE_TRAIN_US {
                return reply(resp, Status::BadArgs, &[]);
            }
            marker.pulse(count, half_period_us);
            reply(resp, Status::Ok, &[])
        }
    }
}

/// Longest self-test pulse train, in microseconds of wall clock.
///
/// 25 ms, chosen against the host's own 1 s request timeout with two orders of
/// magnitude to spare — the USB stack is starved for exactly this long, and
/// the failure if it were ever too long is a fixture that looks hung.
const MAX_PULSE_TRAIN_US: u32 = 25_000;
