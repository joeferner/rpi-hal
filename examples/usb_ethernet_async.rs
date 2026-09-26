#![no_std]
#![no_main]

// The interrupt-driven receive path, on whichever Ethernet chip the board
// has.
//
// `usb_ethernet.rs` does the same job over the blocking transfers; this
// one exists to test the single thing that genuinely differs between the
// two, which is what an idle link costs. The blocking receive asks the
// chip to answer an empty FIFO immediately and returns nothing; the async
// receive asks it to NAK instead, and the DWC2 retries a NAK'd bulk
// channel in hardware without halting it, so the transfer stays parked
// until a frame actually arrives. `start_async` configures the chip for
// the second of those and `start` for the first.
//
// So the thing to watch below is the "(idle)" count. If parking works it
// stays at or near zero however long this runs, and every line printed is
// a real frame. If it does not, the receive future resolves immediately
// with nothing, the count climbs as fast as the console can print it, and
// the async path is buying nothing over the blocking one -- which is a
// silent failure everywhere except here, because the frames still arrive.
//
// Everything past the bus walk is written against `EthernetAsync`, so a
// Pi 2B/3B's LAN9514 and a 3B+'s LAN7800 run the same code. It also holds
// the two halves `split` produces for the whole session, which is what an
// `embassy-net` adapter does -- exercising that here is the point rather
// than a convenience.
//
// One thing it does *not* test: transmitting while a receive is parked.
// `block_on` drives one future at a time, so the send below is sequential
// with the receives after it. Concurrency is the other half of what the
// split is for, and it needs a real executor.
//
// No executor here: the futures are polled by hand, the same `block_on`
// as usb_irq.rs. That keeps the example to the HAL rather than dragging
// in `rpi-hal-embassy`.
//
// Build with `--features bcm2837,async`.

use core::fmt::Write;
use core::future::Future;
use core::ops::ControlFlow;
use core::pin::pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::usb::dwc2::{Channel, Dwc2Host};
use rpi_hal::usb::ethernet::{Ethernet, EthernetAsync, EthernetRx, EthernetTx};
use rpi_hal::usb::lan7800::Lan7800;
use rpi_hal::usb::lan9514::Lan9514;
use rpi_hal::usb::{Bus, Device, Event};
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
    let board = found.expect("the loop above only exits once it is set");

    let Some(mut channel) = dwc2.alloc_channel() else {
        let _ = writeln!(uart, "no free host channel");
        halt();
    };

    // The only place either chip is named. Everything below is written
    // against `EthernetAsync` and never learns which it got.
    let name = board.name();
    match board {
        Board::Lan9514(mut dev) => run(&mut uart, &mut channel, &timer, &mut dev, name, board_mac),
        Board::Lan7800(mut dev) => run(&mut uart, &mut channel, &timer, &mut dev, name, board_mac),
    }
}

/// Brings the interface up, sends one frame, then receives forever —
/// generic over [`EthernetAsync`], so it is written once for both chips.
fn run<E: EthernetAsync>(
    uart: &mut Uart,
    channel: &mut Channel,
    timer: &Timer,
    ethernet: &mut E,
    name: &str,
    mac: [u8; 6],
) -> ! {
    // `start_async`, not `start`: the two configure the empty-FIFO
    // response differently and this example is about which.
    if let Err(e) = block_on(ethernet.start_async(channel, timer, mac)) {
        let _ = writeln!(uart, "{name} start_async failed: {e:?}");
        halt();
    }

    let _ = writeln!(uart, "waiting for link...");
    loop {
        match block_on(ethernet.is_link_up_async(channel, timer)) {
            Ok(true) => break,
            Ok(false) => timer.delay_ms(100),
            Err(e) => {
                let _ = writeln!(uart, "link check failed: {e:?}");
                halt();
            }
        }
    }

    // Split before doing anything with frames, and stay split: this is
    // what an `embassy-net` adapter holds, so exercising it here is the
    // point rather than a convenience. Register access needs the driver
    // whole, so the bring-up above had to happen first.
    let (mut rx, mut tx) = ethernet.split();

    // A minimal broadcast frame, to prove the transmit half moves bytes.
    // Sequential with the receive below rather than concurrent -- one
    // `block_on` drives one future, so this does not test transmitting
    // *past* a parked receive, which is the other half of what the split
    // is for.
    let mut frame = [0u8; 60];
    frame[0..6].copy_from_slice(&[0xff; 6]);
    frame[6..12].copy_from_slice(&mac);
    frame[12..14].copy_from_slice(&0x88b5u16.to_be_bytes());
    match block_on(tx.send_frame_async(channel, timer, &frame)) {
        Ok(()) => {
            let _ = writeln!(uart, "link up -- sent {} bytes", frame.len());
        }
        Err(e) => {
            let _ = writeln!(uart, "send_frame_async failed: {e:?}");
        }
    }

    let _ = writeln!(uart, "receiving (idle count should stay near 0)");
    let mut frames = 0u32;
    let mut idle = 0u32;
    loop {
        // No pacing delay, deliberately. The blocking example needs one
        // because its poll returns immediately whether or not a frame is
        // there; if this one needs it too then parking is not happening,
        // which is exactly what the idle count is here to show.
        match block_on(rx.receive_frames_async(channel, timer)) {
            Ok(received) => {
                let mut any = false;
                for frame in received {
                    any = true;
                    frames += 1;
                    print_frame(uart, frames, frame);
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

/// Whichever Ethernet chip this board turned out to have — the same
/// arrangement as `usb_ethernet.rs`, carrying the choice from the bus walk
/// to the one `match` that acts on it.
enum Board {
    /// A Pi 2B/3B's LAN9514.
    Lan9514(Lan9514),
    /// A Pi 3B+'s LAN7800.
    Lan7800(Lan7800),
}

impl Board {
    /// What to call it in a log line.
    fn name(&self) -> &'static str {
        match self {
            Board::Lan9514(_) => "LAN9514",
            Board::Lan7800(_) => "LAN7800",
        }
    }
}

/// Stops the walk once an Ethernet function has been claimed.
fn flow(found: &Option<Board>) -> ControlFlow<()> {
    if found.is_some() {
        ControlFlow::Break(())
    } else {
        ControlFlow::Continue(())
    }
}

/// Takes `event`'s device as whichever Ethernet chip it is, printing its
/// ID register as the confirmation that register access reaches it.
fn claim(uart: &mut Uart, channel: &mut Channel, timer: &Timer, event: Event) -> Option<Board> {
    let Event::Attached(device) = event else {
        return None;
    };

    match Lan9514::from_device(channel, timer, device) {
        Ok(Some(dev)) => {
            return announce(uart, channel, timer, "LAN9514", &dev, device)
                .then_some(Board::Lan9514(dev));
        }
        Ok(None) => {}
        Err(e) => {
            let _ = writeln!(uart, "LAN9514 setup failed: {e:?}");
            return None;
        }
    }

    match Lan7800::from_device(channel, timer, device) {
        Ok(Some(dev)) => {
            announce(uart, channel, timer, "LAN7800", &dev, device).then_some(Board::Lan7800(dev))
        }
        Ok(None) => None,
        Err(e) => {
            let _ = writeln!(uart, "LAN7800 setup failed: {e:?}");
            None
        }
    }
}

/// Prints what was found and where. Generic over the *blocking*
/// [`Ethernet`], which `EthernetAsync` requires as a supertrait — so the
/// bus walk stays blocking while the frame path does not.
fn announce<E: Ethernet>(
    uart: &mut Uart,
    channel: &mut Channel,
    timer: &Timer,
    name: &str,
    ethernet: &E,
    device: Device,
) -> bool {
    match ethernet.id_revision(channel, timer) {
        Ok(id) => {
            let _ = writeln!(
                uart,
                "{name} on hub {} port {}: id=0x{:04x} revision=0x{:04x}",
                device.hub_address, device.port, id.id, id.revision,
            );
            true
        }
        Err(e) => {
            let _ = writeln!(uart, "{name} id read failed: {e:?}");
            false
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
