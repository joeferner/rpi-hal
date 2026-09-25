#![no_std]
#![no_main]

// Enumerates every USB device on the board and prints the bus as a tree.
// Powers the USB controller on through the VideoCore mailbox, brings up
// the DWC2 core, then hands the whole enumeration off to
// `rpi_hal::usb::enumerate` -- which resets the root port, configures the
// hub it finds there, and reaches through it to reset and address each
// connected downstream device (high-speed directly, full/low-speed via
// split transactions), descending into any of those that is itself a hub.
// Each line is indented by how many hubs deep the device sits, and prints
// the transaction translator its transfers are routed through, which for
// a device below a full-speed hub is not the hub it is plugged into but
// the high-speed one further up. See usb_hid_keyboard.rs for an example
// that talks to a device it finds rather than only describing it.
//
// Then the raw port dump, which is what makes an empty tree readable. A
// port with nothing on it and a port whose device failed to come up look
// identical from the tree -- both are simply absent from it -- and the
// difference is in the status word and the hub descriptor's
// `DeviceRemovable` bits, which say which ports have a device soldered to
// them and so can never legitimately read as empty.
//
// Pi 2/3 only. Every device here hangs off a soldered-on hub behind the
// DWC2 controller: one chip on a Pi 2/3B (the SMSC LAN9514, hub and
// Ethernet in one), two cascaded hubs on a 3B+. A Pi 4 has neither: its
// USB host is a VL805 xHCI behind PCIe, which this crate doesn't drive
// yet, so this builds for `bcm2711` and then finds an empty root port.

use core::fmt::Write;
use core::ops::ControlFlow;
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::usb;
use rpi_hal::usb::control::{get_device_descriptor, get_hub_descriptor, get_port_status};
use rpi_hal::usb::descriptor::DeviceDescriptor;
use rpi_hal::usb::dwc2::{Channel, ControlEndpoint, Dwc2Host};

/// How many hubs the port dump at the end keeps track of. Four covers
/// the board's own hub(s) plus one or two plugged into them, which is
/// what the dump is for.
const MAX_HUBS: usize = 4;

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
    let timer = Timer::new(peripherals.SYSTMR);

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

    // Bounded, because the hub is soldered on: a root port that hasn't
    // reported in five seconds has nothing behind it at all, which is
    // what a Pi 4 looks like (see the header). Falling through rather
    // than halting here -- `usb::enumerate` reports that as
    // `EnumerationError::NotConnected` through the same error path
    // everything else goes through, where an unbounded wait would sit
    // here silently and look like a lock-up.
    let _ = writeln!(uart, "waiting for the on-board hub...");
    let deadline_us = timer.now_micros() + 5_000_000;
    while !dwc2.port_connected() && timer.now_micros() < deadline_us {
        timer.delay_ms(100);
    }

    let mut count = 0u32;
    let mut hubs = [None::<ControlEndpoint>; MAX_HUBS];
    let mut hub_count = 0usize;
    let result = usb::enumerate(&dwc2, &timer, |_channel, _timer, device| {
        count += 1;
        if device.descriptor.device_class == 9 && hub_count < hubs.len() {
            hubs[hub_count] = Some(device.endpoint);
            hub_count += 1;
        }

        // One level of indent per hub below the root hub, so the bus
        // reads as the tree it is. `depth` is bounded by USB's own
        // five-hub limit, so this can't run away.
        for _ in 0..device.depth {
            let _ = write!(uart, "  ");
        }

        let _ = write!(
            uart,
            "hub {} port {}: {:04x}:{:04x} class={} -> address {} ({})",
            device.hub_address,
            device.port,
            device.descriptor.vendor_id,
            device.descriptor.product_id,
            device.descriptor.device_class,
            device.endpoint.address,
            // The endpoint records only whether the device is low speed,
            // since that is the one bit a transfer puts on the wire -- but
            // the split target separates the other two, because a device
            // needs a transaction translator exactly when it is slower
            // than the high-speed bus above it. The one case this reads
            // wrong is a full-speed device on a full-speed root port,
            // which needs no translator either; the board's root hub
            // enumerates at high speed, so that doesn't arise here.
            if device.endpoint.low_speed {
                "low speed"
            } else if device.endpoint.split.is_some() {
                "full speed"
            } else {
                "high speed"
            },
        );
        match device.endpoint.split {
            // Whose translator this is, rather than just "split": for a
            // device more than one hub down these are what show the
            // routing, since the translator is the nearest high-speed hub
            // above the device and not necessarily its parent.
            Some(split) => {
                let _ = writeln!(
                    uart,
                    ", split through hub {} port {}",
                    split.hub_address, split.port
                );
            }
            None => {
                let _ = writeln!(uart, ", direct");
            }
        }

        ControlFlow::Continue(())
    });
    match result {
        Ok(()) => {
            let _ = writeln!(uart, "enumeration complete: {count} device(s)");
        }
        Err(e) => {
            let _ = writeln!(uart, "enumeration failed: {e:?}");
        }
    }

    // Then every hub's ports again, raw. Enumeration deliberately skips a
    // downstream port it can't bring up rather than failing the whole
    // walk, so a hub that reports devices the tree above doesn't leaves
    // no trace of itself otherwise -- and a hub that answers nothing at
    // all here says the problem is the hub, not its ports. Both of those
    // are invisible in the tree, which only ever shows what succeeded.
    if let Some(mut channel) = dwc2.alloc_channel() {
        // The root hub first, which the tree above never shows: enumeration
        // configures it rather than reporting it, so it is the one hub on
        // the board whose ports nothing has printed. Address 1 is where
        // enumeration puts it, and it is high speed on this board, so its
        // endpoint can be rebuilt here from the device descriptor alone.
        let probe = ControlEndpoint {
            address: 1,
            low_speed: false,
            max_packet_size: 8,
            split: None,
        };
        match get_device_descriptor(&mut channel, &timer, probe) {
            Ok(bytes) => {
                let descriptor = DeviceDescriptor::from_bytes(&bytes);
                let _ = writeln!(
                    uart,
                    "root hub: {:04x}:{:04x}",
                    descriptor.vendor_id, descriptor.product_id
                );
                dump_hub_ports(
                    &mut channel,
                    &timer,
                    &mut uart,
                    ControlEndpoint {
                        max_packet_size: descriptor.max_packet_size0 as u16,
                        ..probe
                    },
                );
            }
            Err(e) => {
                let _ = writeln!(uart, "root hub: device descriptor read failed: {e:?}");
            }
        }

        for hub in hubs.iter().flatten() {
            dump_hub_ports(&mut channel, &timer, &mut uart, *hub);
        }
    }

    halt();
}

/// Prints `hub`'s class descriptor and the raw status of every one of its
/// downstream ports.
///
/// The whole descriptor goes out as bytes rather than just the port count
/// because the two fields that decide whether an empty-looking port is
/// really empty are in there: `bPwrOn2PwrGood` (byte 5, in 2ms units) says
/// how long the hub needs after powering a port, and `DeviceRemovable`
/// (from byte 7, one bit per port) says which ports have a device soldered
/// to them and so can never legitimately read as unoccupied.
fn dump_hub_ports(channel: &mut Channel, timer: &Timer, uart: &mut Uart, hub: ControlEndpoint) {
    let mut descriptor = [0u8; 16];
    let len = match get_hub_descriptor(channel, timer, hub, &mut descriptor) {
        Ok(len) => len,
        Err(e) => {
            let _ = writeln!(uart, "hub {}: descriptor read failed: {e:?}", hub.address);
            return;
        }
    };
    // bNbrPorts is byte 2 of the hub class descriptor.
    if len < 6 {
        let _ = writeln!(uart, "hub {}: short descriptor ({len} bytes)", hub.address);
        return;
    }

    let _ = write!(
        uart,
        "hub {}: {} port(s), descriptor",
        hub.address, descriptor[2]
    );
    for byte in &descriptor[..len] {
        let _ = write!(uart, " {byte:02x}");
    }
    let _ = writeln!(uart);

    for port in 1..=descriptor[2] {
        match get_port_status(channel, timer, hub, port) {
            Ok((status, change)) => {
                let _ = writeln!(
                    uart,
                    "  port {port}: status {status:04x} change {change:04x}"
                );
            }
            Err(e) => {
                let _ = writeln!(uart, "  port {port}: status read failed: {e:?}");
            }
        }
    }
}
