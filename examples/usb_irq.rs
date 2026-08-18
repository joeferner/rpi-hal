#![no_std]
#![no_main]

// Runs USB transfers off the controller's interrupt instead of by
// spinning on its registers -- the async half of `usb::dwc2`, with no
// executor crate involved.
//
// What it does is deliberately small: reset the root port, read the
// device descriptor of whatever is on it (the on-board LAN9514 hub, on
// this board), and pace a short wait in bus time. What makes it worth
// running is *how* each of those waits is taken. Every one is an
// interrupt:
//
// - the root port reporting a device is `GINTSTS.HPRTINT`, awaited with
//   `Dwc2Host::wait_for_port_change`;
// - each control-transfer stage completing is that channel's
//   `HCINT.CHH`, awaited inside `Channel::control_*_async`;
// - the microframe wait at the end is `GINTSTS.SOF`, awaited with
//   `Channel::wait_microframes` -- the same mechanism the async path
//   uses to schedule periodic split transactions, which is otherwise
//   only reachable through a full/low-speed device behind the hub.
//
// So this is the smoke test for the interrupt plumbing itself: that the
// DWC2 line reaches the ARM core through the legacy controller's USB
// bit, that a channel programmed with `HCINTMSK = CHH` still completes
// transfers, and that `on_irq` acknowledges everything it takes -- an
// unacked level source would show up as a hang right after the first
// wait, not as a wrong answer.
//
// It needs no executor because the futures are polled by hand below:
// `block_on` drives one future to completion, parking the core in `wfe`
// between polls. That is enough for one thing at a time, which is all
// this example does. Anything with more than one task wants a real
// executor -- see the `rpi-hal-embassy` crate.
//
// The blocking API is untouched by any of this and still works exactly
// as `usb_enum.rs` uses it; the two can even share a controller, on
// different channels.

use core::fmt::Write;
use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use rpi_hal::usb::descriptor::DeviceDescriptor;
use rpi_hal::usb::dwc2::{Channel, ControlEndpoint, Dwc2Host, TransferError};
use rpi_hal::{halt, irq, lic::Lic, mailbox::Mailbox, pac, timer::Timer, uart::Uart, usb};

/// Endpoint-0 max packet size every host assumes for its very first
/// transfer, before the device has had a chance to say otherwise. Eight
/// bytes is the minimum the spec allows, so it is always safe.
const INITIAL_MAX_PACKET_SIZE: u16 = 8;

/// `bmRequestType` for a standard device-to-host device request.
const REQUEST_TYPE_DEVICE_IN: u8 = 0x80;
/// `bRequest` for GET_DESCRIPTOR.
const REQUEST_GET_DESCRIPTOR: u8 = 0x06;
/// `wValue` high byte for a DEVICE descriptor.
const DESCRIPTOR_TYPE_DEVICE: u8 = 0x01;

/// Microframes to wait at the end, purely to exercise the start-of-frame
/// path: 64 × 125µs is 8ms, a typical HID polling interval.
const DEMO_MICROFRAMES: u32 = 64;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// Services the interrupts this example depends on.
///
/// Mandatory, and silently fatal to omit: `rpi-hal` provides only a weak
/// no-op `__irq_handler`, so without this the first channel halt fires,
/// nothing acknowledges `HCINT`, the controller keeps asserting its line,
/// and the core re-enters the handler forever. On the console that looks
/// like a hang at the first await, not like an error.
#[no_mangle]
pub extern "C" fn __irq_handler() {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);

    if lic.is_usb_pending() {
        usb::dwc2::on_irq();
    }
}

/// Wakes a core parked in `wfe`.
///
/// `dsb ish` before `sev` is ARM's prescribed order for signalling other
/// observers; here it also orders the waker's own stores before the
/// wake-up is broadcast.
fn signal_event() {
    // SAFETY: neither instruction has operands or touches memory.
    unsafe { core::arch::asm!("dsb ish", "sev", options(nomem, nostack)) };
}

/// A [`Waker`] whose only job is to make a `wfe` return.
///
/// There is nothing to store per-waker — the loop below re-polls its one
/// future on any wake-up — so the data pointer is null and the vtable's
/// clone/drop are no-ops.
fn event_waker() -> Waker {
    fn wake(_: *const ()) {
        signal_event();
    }
    fn clone(data: *const ()) -> RawWaker {
        RawWaker::new(data, &VTABLE)
    }
    fn drop(_: *const ()) {}

    static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, wake, wake, drop);

    // SAFETY: the vtable's functions are valid for the null data pointer
    // they are given -- none of them dereferences it.
    unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) }
}

/// Drives one future to completion, parking the core between polls.
///
/// No lost wake-up race, for the reason `rpi-hal-embassy`'s executor
/// relies on: `sev` sets the core's event register whether or not
/// anything is waiting on it, so a wake that lands after the last poll
/// and before the `wfe` simply makes that `wfe` return immediately.
fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let waker = event_waker();
    let mut context = Context::from_waker(&waker);
    loop {
        if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
            return output;
        }
        // SAFETY: `wfe` has no operands; at worst it returns immediately.
        unsafe { core::arch::asm!("wfe", options(nomem, nostack)) };
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let lic = Lic::new(peripherals.LIC);

    let _ = writeln!(uart, "interrupt-driven USB");

    // The USB controller comes up only partially powered from firmware;
    // power it fully via the mailbox before touching DWC2.
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    if !usb::power_on(&mut mailbox) {
        let _ = writeln!(uart, "USB power-on failed");
        halt();
    }

    let dwc2 = Dwc2Host::init(
        peripherals.USB_OTG_GLOBAL,
        peripherals.USB_OTG_HOST,
        peripherals.USB_OTG_PWRCLK,
        &timer,
    );
    let _ = writeln!(uart, "{} host channels", dwc2.num_channels());

    // Open the last two gates the interrupt has to pass: the legacy
    // interrupt controller's USB line, and the CPU's own mask. The
    // controller-side gate (`GINTMSK`) was set up by `init`.
    lic.enable_usb_irq();
    irq::enable_irq();

    // Wait for something on the root port. This latches, so a hub that
    // was already attached when the port was powered still reports --
    // which on this board is the normal case, since the LAN9514 is
    // soldered on.
    let _ = writeln!(uart, "waiting for a device on the root port...");
    block_on(dwc2.wait_for_port_change());
    if !dwc2.port_connected() {
        let _ = writeln!(uart, "port change, but nothing connected");
        halt();
    }

    dwc2.reset_port(&timer);
    if !dwc2.port_enabled() {
        let _ = writeln!(uart, "port did not enable after reset");
        halt();
    }
    let speed = match dwc2.port_speed() {
        0 => "high",
        1 => "full",
        _ => "low",
    };
    let _ = writeln!(uart, "device attached at {speed} speed");

    let Some(mut channel) = dwc2.alloc_channel() else {
        let _ = writeln!(uart, "no host channel available");
        halt();
    };
    let _ = writeln!(uart, "using host channel {}", channel.index());

    // Everything below is one control transfer and one paced wait, both
    // resolved by interrupts.
    block_on(run(&mut uart, &mut channel, &timer));
    halt();
}

/// Reads the attached device's descriptor over async control transfers,
/// then paces a wait on the start-of-frame interrupt.
async fn run(uart: &mut Uart, channel: &mut Channel<'_>, timer: &Timer) {
    // Address 0, the default before SET_ADDRESS: this reads the
    // descriptor of whatever answers on the just-reset port. `split` is
    // `None` because the root device reaches the host directly.
    let endpoint = ControlEndpoint {
        address: 0,
        low_speed: false,
        max_packet_size: INITIAL_MAX_PACKET_SIZE,
        split: None,
    };

    let mut descriptor = [0u8; 18];
    match get_device_descriptor(channel, endpoint, &mut descriptor, timer).await {
        Ok(len) => {
            let d = DeviceDescriptor::from_bytes(&descriptor);
            let _ = writeln!(
                uart,
                "device descriptor ({len} bytes): {:04x}:{:04x} class 0x{:02x} \
                 ep0 max packet {}",
                d.vendor_id, d.product_id, d.device_class, d.max_packet_size0
            );
        }
        Err(e) => {
            let _ = writeln!(uart, "descriptor read failed: {e:?}");
            return;
        }
    }

    // Nothing depends on this wait; it is here so the start-of-frame
    // path is exercised on a board with nothing but the hub attached.
    // Compare the elapsed time against the 8ms asked for: they should
    // agree to within a microframe, and a wildly larger number means SOF
    // is not arriving.
    let before = timer.now_micros();
    channel.wait_microframes(DEMO_MICROFRAMES, timer).await;
    let elapsed = timer.now_micros() - before;
    let _ = writeln!(
        uart,
        "{DEMO_MICROFRAMES} microframes on the SOF interrupt took {elapsed}us (expected ~{}us)",
        DEMO_MICROFRAMES * 125
    );

    let _ = writeln!(uart, "done");
}

/// Reads the full 18-byte device descriptor, in the two steps every real
/// host uses.
///
/// The first asks for only the first 8 bytes, with endpoint 0's max
/// packet size set to the 8 the spec guarantees. That is the *only* size
/// safe to assume before the device has spoken, and byte 7 of what comes
/// back is `bMaxPacketSize0` — the real one, 64 on a high-speed device.
/// The second read uses it.
///
/// Skipping the first step and asking for all 18 bytes with the channel
/// still programmed for 8 fails with [`TransferError::Babble`]: the
/// device answers in one 18-byte packet, the channel was told to expect
/// packets of at most 8, and more data than expected on the wire is
/// exactly what babble means. Nothing about the interrupt path is
/// involved — the blocking API needs the same two steps, which is why
/// `usb::control::get_device_descriptor` is also written this way.
async fn get_device_descriptor(
    channel: &mut Channel<'_>,
    endpoint: ControlEndpoint,
    buf: &mut [u8; 18],
    timer: &Timer,
) -> Result<usize, TransferError> {
    let mut header = [0u8; 8];
    control_in(channel, endpoint, &mut header, timer).await?;

    let endpoint = ControlEndpoint {
        max_packet_size: u16::from(header[7]),
        ..endpoint
    };
    control_in(channel, endpoint, buf, timer).await
}

/// One complete GET_DESCRIPTOR(DEVICE) control transfer — SETUP, DATA
/// IN, STATUS OUT — built by hand from the async channel primitives,
/// reading `buf.len()` bytes.
///
/// `usb::control` does this (and the retry-on-NAK wrapping around each
/// stage) for the blocking path; the async side has no equivalent yet,
/// which is exactly why this example spells the three stages out.
async fn control_in(
    channel: &mut Channel<'_>,
    endpoint: ControlEndpoint,
    buf: &mut [u8],
    timer: &Timer,
) -> Result<usize, TransferError> {
    let length = buf.len() as u16;
    let setup = [
        REQUEST_TYPE_DEVICE_IN,
        REQUEST_GET_DESCRIPTOR,
        0,
        DESCRIPTOR_TYPE_DEVICE,
        0,
        0,
        length as u8,
        (length >> 8) as u8,
    ];

    channel.control_setup_async(endpoint, &setup, timer).await?;
    let received = channel.control_data_in_async(endpoint, buf, timer).await?;
    channel.control_status_out_async(endpoint, timer).await?;
    Ok(received)
}
