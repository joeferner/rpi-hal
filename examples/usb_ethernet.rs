#![no_std]
#![no_main]

// On-board Ethernet bring-up over USB, on whichever of the two chips the
// board has. Powers the USB controller on, brings up the DWC2 core, walks
// the bus, and when an Ethernet function turns up: confirms it by its ID
// register, programs the board MAC (from the VideoCore mailbox) into it
// and enables RX/TX, waits for the Ethernet link, sends one broadcast
// frame, then prints a summary of each frame it receives. Receiving real
// broadcast/multicast traffic (ARP, mDNS, ...) is what validates the DWC2
// bulk transfer path end to end.
//
// Two chips, because the boards differ. A Pi 2B/3B has one soldered-on
// SMSC LAN9514 that is both the USB hub and the Ethernet. A Pi 3B+ has a
// Microchip LAN7515: two cascaded hubs, with a LAN7800 Ethernet behind the
// second of them. Whichever answers is what gets driven, so the same image
// covers both -- the drivers have the same shape, and everything below the
// `Ethernet` enum is written once.
//
// That is also why this walks the bus with `usb::Bus` rather than
// `usb::enumerate`. On a 3B+ the LAN7800 attaches seconds after power-on,
// later than any settling delay, so a one-shot walk finishes before it
// exists and finds nothing; `Bus::poll` is what turns it up. On a 2B/3B
// the LAN9514 is there from the start and the first walk finds it, so the
// poll loop never runs.
//
// A Pi 4 has neither chip: its USB host is a VL805 xHCI behind PCIe and
// its Ethernet is a native GENET MAC on RGMII pins, neither of which this
// crate drives yet -- so this builds for `bcm2711` and then finds an empty
// root port.

use core::fmt::Write;
use core::ops::ControlFlow;
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::usb;
use rpi_hal::usb::dwc2::{Channel, Dwc2Host, TransferError};
use rpi_hal::usb::lan7800::Lan7800;
use rpi_hal::usb::lan9514::Lan9514;
use rpi_hal::usb::{Bus, Event};

/// How long to wait between `Bus::poll` sweeps while waiting for an
/// Ethernet function to attach.
const POLL_INTERVAL_MS: u32 = 250;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// Whichever Ethernet chip this board turned out to have.
///
/// The two drivers have the same surface but not the same type, so the
/// choice is made once here and everything downstream is written against
/// this. A trait would do the same job and is the obvious next step if a
/// third chip ever appears; with two, an enum keeps the dispatch visible
/// and costs nothing.
enum Ethernet {
    /// A Pi 2B/3B's LAN9514 — hub and Ethernet in one chip.
    Lan9514(Lan9514),
    /// A Pi 3B+'s LAN7800, behind the LAN7515's two hubs.
    Lan7800(Lan7800),
}

impl Ethernet {
    /// What to call it in a log line.
    fn name(&self) -> &'static str {
        match self {
            Ethernet::Lan9514(_) => "LAN9514",
            Ethernet::Lan7800(_) => "LAN7800",
        }
    }

    /// Programs the MAC and opens the data paths.
    fn start(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        mac: [u8; 6],
    ) -> Result<(), TransferError> {
        match self {
            Ethernet::Lan9514(dev) => dev.start(channel, timer, mac),
            Ethernet::Lan7800(dev) => dev.start(channel, timer, mac),
        }
    }

    /// Whether the Ethernet link is up.
    fn is_link_up(&self, channel: &mut Channel, timer: &Timer) -> Result<bool, TransferError> {
        match self {
            Ethernet::Lan9514(dev) => dev.is_link_up(channel, timer),
            Ethernet::Lan7800(dev) => dev.is_link_up(channel, timer),
        }
    }

    /// Whether auto-negotiation settled on full duplex.
    fn is_full_duplex(&self, channel: &mut Channel, timer: &Timer) -> Result<bool, TransferError> {
        match self {
            Ethernet::Lan9514(dev) => dev.is_full_duplex(channel, timer),
            Ethernet::Lan7800(dev) => dev.is_full_duplex(channel, timer),
        }
    }

    /// Sends one Ethernet frame, without its CRC.
    fn send_frame(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        frame: &[u8],
    ) -> Result<(), TransferError> {
        match self {
            Ethernet::Lan9514(dev) => dev.send_frame(channel, timer, frame),
            Ethernet::Lan7800(dev) => dev.send_frame(channel, timer, frame),
        }
    }

    /// Polls for received frames and prints a summary of each.
    ///
    /// The printing happens in here rather than in the caller because the
    /// two drivers return their own `Frames` iterators: same item type,
    /// different types, so there is nothing to hand back without boxing.
    /// Draining each one where it is produced is also the behaviour that
    /// matters — a transfer can carry several frames, and taking only the
    /// first discards the rest silently.
    fn receive(&mut self, uart: &mut Uart, channel: &mut Channel, timer: &Timer) {
        let result = match self {
            Ethernet::Lan9514(dev) => dev.receive_frames(channel, timer).map(|frames| {
                for frame in frames {
                    print_frame(uart, frame);
                }
            }),
            Ethernet::Lan7800(dev) => dev.receive_frames(channel, timer).map(|frames| {
                for frame in frames {
                    print_frame(uart, frame);
                }
            }),
        };
        if let Err(e) = result {
            let _ = writeln!(
                uart,
                "receive error: {e:?} (hcint=0x{:08x})",
                channel.last_interrupt()
            );
        }
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);

    // The USB controller comes up only partially powered from firmware;
    // power it fully via the mailbox before touching DWC2.
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    if !usb::power_on(&mut mailbox) {
        let _ = writeln!(uart, "USB power-on failed");
        halt();
    }

    // The MAC lives in firmware on this board, not the chip -- read it
    // here so we can program it into whichever chip we find.
    let board_mac = match mailbox.mac_address() {
        Ok(mac) => {
            let _ = writeln!(uart, "board MAC: {}", FormatMac(mac));
            mac
        }
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

    // Bounded, because the hub is soldered on: a root port that hasn't
    // reported in five seconds has nothing behind it at all, which is
    // what a Pi 4 looks like (see the header). Falling through rather
    // than halting here -- the walk reports that as
    // `EnumerationError::NotConnected` through the same error path
    // everything else goes through, where an unbounded wait would sit
    // here silently and look like a lock-up.
    let _ = writeln!(uart, "waiting for the on-board hub...");
    let deadline_us = timer.now_micros() + 5_000_000;
    while !dwc2.port_connected() && timer.now_micros() < deadline_us {
        timer.delay_ms(100);
    }

    let mut bus = Bus::new(&dwc2);
    let mut found = None;
    let result = bus.enumerate(&timer, |channel, timer, event| {
        found = claim(&mut uart, channel, timer, event);
        break_when_found(&found)
    });
    if let Err(e) = result {
        let _ = writeln!(uart, "enumeration failed: {e:?}");
        halt();
    }

    // Nothing yet is the expected answer on a 3B+, not a failure: its
    // Ethernet function attaches seconds after power-on, so it turns up
    // through a later sweep.
    if found.is_none() {
        let _ = writeln!(uart, "waiting for an Ethernet function to attach...");
    }
    while found.is_none() {
        let result = bus.poll(&timer, |channel, timer, event| {
            found = claim(&mut uart, channel, timer, event);
            break_when_found(&found)
        });
        if let Err(e) = result {
            let _ = writeln!(uart, "poll failed: {e:?}");
        }
        timer.delay_ms(POLL_INTERVAL_MS);
    }

    let mut ethernet = found.expect("the loop above only exits once it is set");

    // The walk's own channel goes away with it, so take one to keep.
    let Some(mut channel) = dwc2.alloc_channel() else {
        let _ = writeln!(uart, "no free host channel");
        halt();
    };
    run_ethernet(&mut uart, &mut channel, &timer, &mut ethernet, board_mac);
    halt();
}

/// Stops the walk once something has been claimed.
fn break_when_found(found: &Option<Ethernet>) -> ControlFlow<()> {
    if found.is_some() {
        ControlFlow::Break(())
    } else {
        ControlFlow::Continue(())
    }
}

/// Takes `event`'s device as whichever Ethernet chip it is, printing its
/// ID register as the confirmation that register access reaches it.
///
/// Each driver declines a device that isn't its own by vendor/product ID,
/// so offering the device to both in turn is how the board is identified —
/// nothing here has to know which Pi it is running on.
fn claim(uart: &mut Uart, channel: &mut Channel, timer: &Timer, event: Event) -> Option<Ethernet> {
    let Event::Attached(device) = event else {
        return None;
    };

    let ethernet = match Lan9514::from_device(channel, timer, device) {
        Ok(Some(dev)) => Some(Ethernet::Lan9514(dev)),
        Ok(None) => None,
        Err(e) => {
            let _ = writeln!(uart, "LAN9514 setup failed: {e:?}");
            return None;
        }
    };
    let ethernet = match ethernet {
        Some(ethernet) => ethernet,
        None => match Lan7800::from_device(channel, timer, device) {
            Ok(Some(dev)) => Ethernet::Lan7800(dev),
            Ok(None) => return None,
            Err(e) => {
                let _ = writeln!(uart, "LAN7800 setup failed: {e:?}");
                return None;
            }
        },
    };

    let id = match &ethernet {
        Ethernet::Lan9514(dev) => dev.id_revision(channel, timer).map(|i| (i.id, i.revision)),
        Ethernet::Lan7800(dev) => dev.id_revision(channel, timer).map(|i| (i.id, i.revision)),
    };
    match id {
        Ok((id, revision)) => {
            let _ = writeln!(
                uart,
                "{} on hub {} port {}: id=0x{id:04x} revision=0x{revision:04x}",
                ethernet.name(),
                device.hub_address,
                device.port,
            );
        }
        Err(e) => {
            let _ = writeln!(uart, "{} id read failed: {e:?}", ethernet.name());
            return None;
        }
    }
    Some(ethernet)
}

/// Enables the interface, waits for link, sends one broadcast frame, then
/// prints a summary of each received frame forever.
fn run_ethernet(
    uart: &mut Uart,
    channel: &mut Channel,
    timer: &Timer,
    ethernet: &mut Ethernet,
    mac: [u8; 6],
) {
    if let Err(e) = ethernet.start(channel, timer, mac) {
        let _ = writeln!(uart, "{} start failed: {e:?}", ethernet.name());
        return;
    }
    // Auto-negotiation takes a second or three, so this is not instant
    // even on a healthy link.
    let _ = writeln!(uart, "waiting for link...");
    loop {
        match ethernet.is_link_up(channel, timer) {
            Ok(true) => break,
            Ok(false) => timer.delay_ms(100),
            Err(e) => {
                let _ = writeln!(uart, "link check failed: {e:?}");
                return;
            }
        }
    }
    match ethernet.is_full_duplex(channel, timer) {
        Ok(full) => {
            let _ = writeln!(
                uart,
                "link up ({}) -- sending a test broadcast frame",
                if full { "full duplex" } else { "half duplex" }
            );
        }
        Err(_) => {
            let _ = writeln!(uart, "link up -- sending a test broadcast frame");
        }
    }

    // A minimal broadcast frame: broadcast destination, our MAC as source,
    // an experimental EtherType, and enough payload to reach the 60-byte
    // minimum (the chip appends the 4-byte CRC).
    let mut frame = [0u8; 60];
    frame[0..6].copy_from_slice(&[0xff; 6]);
    frame[6..12].copy_from_slice(&mac);
    frame[12..14].copy_from_slice(&0x88b5u16.to_be_bytes());
    match ethernet.send_frame(channel, timer, &frame) {
        Ok(()) => {
            let _ = writeln!(uart, "sent {} byte frame", frame.len());
        }
        Err(e) => {
            let _ = writeln!(uart, "send failed: {e:?}");
        }
    }

    let _ = writeln!(uart, "receiving...");
    loop {
        // Pace polls -- bulk endpoints mustn't be hammered back to back.
        timer.delay_ms(10);
        ethernet.receive(uart, channel, timer);
    }
}

/// Prints a one-line summary of a received Ethernet frame.
fn print_frame(uart: &mut Uart, frame: &[u8]) {
    if frame.len() < 14 {
        let _ = writeln!(uart, "rx {} bytes (runt)", frame.len());
        return;
    }
    let mut dst = [0u8; 6];
    let mut src = [0u8; 6];
    dst.copy_from_slice(&frame[0..6]);
    src.copy_from_slice(&frame[6..12]);
    let ethertype = u16::from_be_bytes([frame[12], frame[13]]);
    let _ = writeln!(
        uart,
        "rx {} bytes: dst={} src={} type=0x{:04x}",
        frame.len(),
        FormatMac(dst),
        FormatMac(src),
        ethertype
    );
}

/// Formats a MAC address as colon-separated hex (`aa:bb:cc:dd:ee:ff`).
struct FormatMac([u8; 6]);

impl core::fmt::Display for FormatMac {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let m = self.0;
        write!(
            f,
            "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            m[0], m[1], m[2], m[3], m[4], m[5]
        )
    }
}
