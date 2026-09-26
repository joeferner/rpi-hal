#![no_std]
#![no_main]

// The LAN7800's interrupt-driven receive path, on a Pi 3B+.
//
// `usb_ethernet.rs` does the same job over the blocking transfers; this
// one exists to test the single thing that genuinely differs between the
// two, which is what an idle link costs. The blocking receive asks the
// chip to answer an empty FIFO immediately and returns nothing; the async
// receive asks it to NAK instead, and the DWC2 retries a NAK'd bulk
// channel in hardware without halting it, so the transfer stays parked
// until a frame actually arrives. `Lan7800::start_async` configures the
// chip for the second of those and `Lan7800::start` for the first -- see
// `reset_async`.
//
// So the thing to watch below is the "(idle)" count. If parking works it
// stays at or near zero however long this runs, and every line printed is
// a real frame. If it does not, the receive future resolves immediately
// with nothing, the count climbs as fast as the console can print it, and
// the async path is buying nothing over the blocking one -- which is a
// silent failure everywhere except here, because the frames still arrive.
//
// No executor: the futures are polled by hand, the same `block_on` as
// usb_irq.rs. One thing at a time is all this needs, and it keeps the
// example to the HAL rather than dragging in `rpi-hal-embassy`.
//
// Pi 3B+ only -- this drives the LAN7800 specifically, and a 2B/3B has a
// LAN9514 instead. Build with `--features bcm2837,async`.

use core::fmt::Write;
use core::future::Future;
use core::ops::ControlFlow;
use core::pin::pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::usb::dwc2::{Channel, Dwc2Host};
use rpi_hal::usb::lan7800::Lan7800;
use rpi_hal::usb::{Bus, Event};
use rpi_hal::{halt, irq, lic::Lic, pac, usb};

/// How long to wait between `Bus::poll` sweeps while waiting for the
/// Ethernet function to attach. It comes up seconds after power-on.
const POLL_INTERVAL_MS: u32 = 250;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// Services the USB interrupt. Without this the weak no-op in `rpi-hal`
/// takes the vector, nothing acknowledges `HCINT`, the controller keeps
/// asserting its line, and the core re-enters the handler forever — which
/// on the console looks like a hang at the first await rather than an
/// error.
#[no_mangle]
pub extern "C" fn __irq_handler() {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let lic = Lic::new(peripherals.LIC);

    if lic.is_usb_pending() {
        usb::dwc2::on_irq();
    }
}

/// Wakes a core parked in `wfe`. `dsb ish` before `sev` is ARM's
/// prescribed order for signalling other observers.
fn signal_event() {
    // SAFETY: neither instruction has operands or touches memory.
    unsafe { core::arch::asm!("dsb ish", "sev", options(nomem, nostack)) };
}

/// A [`Waker`] whose only job is to make a `wfe` return.
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
/// No lost wake-up race: `sev` sets the core's event register whether or
/// not anything is waiting on it, so a wake landing between the last poll
/// and the `wfe` makes that `wfe` return immediately.
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

    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    if !usb::power_on(&mut mailbox) {
        let _ = writeln!(uart, "USB power-on failed");
        halt();
    }
    let board_mac = match mailbox.mac_address() {
        Ok(mac) => mac,
        Err(e) => {
            let _ = writeln!(uart, "MAC read failed: {e:?}");
            halt();
        }
    };

    let dwc2 = Dwc2Host::init(
        peripherals.USB_OTG_GLOBAL,
        peripherals.USB_OTG_HOST,
        peripherals.USB_OTG_PWRCLK,
        &timer,
    );

    // The last two gates the interrupt has to pass; `GINTMSK` was set up
    // by `init`. Nothing below completes without these.
    lic.enable_usb_irq();
    irq::enable_irq();

    let _ = writeln!(uart, "waiting for the on-board hub...");
    let deadline_us = timer.now_micros() + 5_000_000;
    while !dwc2.port_connected() && timer.now_micros() < deadline_us {
        timer.delay_ms(100);
    }

    // The bus walk stays blocking: it runs once, during bring-up, before
    // there is anything else that wants the core.
    let mut bus = Bus::new(&dwc2);
    let mut found = None;
    let result = bus.enumerate(&timer, |channel, timer, event| {
        found = claim(&mut uart, channel, timer, event);
        flow(&found)
    });
    if let Err(e) = result {
        let _ = writeln!(uart, "enumeration failed: {e:?}");
        halt();
    }
    if found.is_none() {
        let _ = writeln!(uart, "waiting for the Ethernet function to attach...");
    }
    while found.is_none() {
        let result = bus.poll(&timer, |channel, timer, event| {
            found = claim(&mut uart, channel, timer, event);
            flow(&found)
        });
        if let Err(e) = result {
            let _ = writeln!(uart, "poll failed: {e:?}");
        }
        timer.delay_ms(POLL_INTERVAL_MS);
    }
    let mut lan7800 = found.expect("the loop above only exits once it is set");

    let Some(mut channel) = dwc2.alloc_channel() else {
        let _ = writeln!(uart, "no free host channel");
        halt();
    };

    // `start_async`, not `start`: the two configure the empty-FIFO
    // response differently and this example is about which.
    if let Err(e) = block_on(lan7800.start_async(&mut channel, &timer, board_mac)) {
        let _ = writeln!(uart, "start_async failed: {e:?}");
        halt();
    }

    let _ = writeln!(uart, "waiting for link...");
    loop {
        match block_on(lan7800.is_link_up_async(&mut channel, &timer)) {
            Ok(true) => break,
            Ok(false) => timer.delay_ms(100),
            Err(e) => {
                let _ = writeln!(uart, "link check failed: {e:?}");
                halt();
            }
        }
    }
    let _ = writeln!(uart, "link up -- receiving (idle count should stay near 0)");

    let mut frames = 0u32;
    let mut idle = 0u32;
    loop {
        // No pacing delay, deliberately. The blocking example needs one
        // because its poll returns immediately whether or not a frame is
        // there; if this one needs it too then parking is not happening,
        // which is exactly what the idle count is here to show.
        match block_on(lan7800.receive_frames_async(&mut channel, &timer)) {
            Ok(received) => {
                let mut any = false;
                for frame in received {
                    any = true;
                    frames += 1;
                    print_frame(&mut uart, frames, frame);
                }
                if !any {
                    idle += 1;
                    // Every tenth, so a spinning receive is obvious
                    // without the console becoming the bottleneck.
                    if idle.is_multiple_of(10) {
                        let _ = writeln!(uart, "(idle) x{idle}");
                    }
                }
            }
            Err(e) => {
                let _ = writeln!(uart, "receive error: {e:?}");
            }
        }
    }
}

/// Stops the walk once the Ethernet function has been claimed.
fn flow(found: &Option<Lan7800>) -> ControlFlow<()> {
    if found.is_some() {
        ControlFlow::Break(())
    } else {
        ControlFlow::Continue(())
    }
}

/// Takes `event`'s device as a LAN7800 if that is what it is, printing its
/// ID register as the confirmation that register access reaches it.
fn claim(uart: &mut Uart, channel: &mut Channel, timer: &Timer, event: Event) -> Option<Lan7800> {
    let Event::Attached(device) = event else {
        return None;
    };
    let lan7800 = match Lan7800::from_device(channel, timer, device) {
        Ok(Some(lan7800)) => lan7800,
        Ok(None) => return None,
        Err(e) => {
            let _ = writeln!(uart, "LAN7800 setup failed: {e:?}");
            return None;
        }
    };
    match lan7800.id_revision(channel, timer) {
        Ok(id) => {
            let _ = writeln!(
                uart,
                "LAN7800 on hub {} port {}: id=0x{:04x} revision=0x{:04x}",
                device.hub_address, device.port, id.id, id.revision
            );
            Some(lan7800)
        }
        Err(e) => {
            let _ = writeln!(uart, "LAN7800 id read failed: {e:?}");
            None
        }
    }
}

/// Prints a one-line summary of a received Ethernet frame.
fn print_frame(uart: &mut Uart, count: u32, frame: &[u8]) {
    if frame.len() < 14 {
        let _ = writeln!(uart, "rx #{count}: {} bytes (runt)", frame.len());
        return;
    }
    let ethertype = u16::from_be_bytes([frame[12], frame[13]]);
    let _ = writeln!(
        uart,
        "rx #{count}: {} bytes src={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} type=0x{ethertype:04x}",
        frame.len(),
        frame[6],
        frame[7],
        frame[8],
        frame[9],
        frame[10],
        frame[11],
    );
}
