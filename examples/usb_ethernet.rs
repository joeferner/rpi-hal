#![no_std]
#![no_main]

// On-board Ethernet bring-up over USB. Powers the USB controller on,
// brings up the DWC2 core, enumerates the bus with `rpi_hal::usb::enumerate`,
// and when the LAN9514 USB-Ethernet function turns up: confirms it by its
// ID register, programs the board MAC (from the VideoCore mailbox) into
// it and enables RX/TX, waits for the Ethernet link, sends one broadcast
// frame, then prints a summary of each frame it receives. Receiving real
// broadcast/multicast traffic (ARP, mDNS, ...) is what validates the DWC2
// bulk transfer path end to end.

use core::fmt::Write;
use core::ops::ControlFlow;
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::usb;
use rpi_hal::usb::dwc2::Dwc2Host;
use rpi_hal::usb::lan9514::Lan9514;

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

    // The MAC lives in firmware on this board, not the chip -- read it
    // here so we can program it into the LAN9514.
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

    let mut dwc2 = Dwc2Host::init(
        peripherals.USB_OTG_GLOBAL,
        peripherals.USB_OTG_HOST,
        peripherals.USB_OTG_PWRCLK,
        &timer,
    );

    let _ = writeln!(uart, "waiting for the on-board hub...");
    while !dwc2.port_connected() {
        timer.delay_ms(100);
    }

    // Enumerate the bus; bring the LAN9514 up and run traffic through it.
    let result = usb::enumerate(&mut dwc2, &timer, |dwc2, timer, device| {
        let mut lan9514 = match Lan9514::from_device(dwc2, timer, device) {
            Ok(Some(lan9514)) => lan9514,
            Ok(None) => return ControlFlow::Continue(()),
            Err(e) => {
                let _ = writeln!(uart, "LAN9514 setup failed: {e:?}");
                return ControlFlow::Break(());
            }
        };

        match lan9514.id_revision(dwc2, timer) {
            Ok(id) => {
                let _ = writeln!(
                    uart,
                    "LAN9514 on port {}: id=0x{:04x} revision=0x{:04x}",
                    device.port, id.id, id.revision
                );
            }
            Err(e) => {
                let _ = writeln!(uart, "LAN9514 id read failed: {e:?}");
                return ControlFlow::Break(());
            }
        }

        run_ethernet(&mut uart, dwc2, timer, &mut lan9514, board_mac);
        ControlFlow::Break(())
    });

    if let Err(e) = result {
        let _ = writeln!(uart, "enumeration failed: {e:?}");
    }
    halt();
}

/// Enables the interface, waits for link, sends one broadcast frame, then
/// prints a summary of each received frame forever.
fn run_ethernet(
    uart: &mut Uart,
    dwc2: &mut Dwc2Host,
    timer: &Timer,
    lan9514: &mut Lan9514,
    mac: [u8; 6],
) {
    if let Err(e) = lan9514.start(dwc2, timer, mac) {
        let _ = writeln!(uart, "LAN9514 start failed: {e:?}");
        return;
    }

    let _ = writeln!(uart, "waiting for link...");
    loop {
        match lan9514.is_link_up(dwc2, timer) {
            Ok(true) => break,
            Ok(false) => timer.delay_ms(100),
            Err(e) => {
                let _ = writeln!(uart, "link check failed: {e:?}");
                return;
            }
        }
    }
    let _ = writeln!(uart, "link up -- sending a test broadcast frame");

    // A minimal broadcast frame: broadcast destination, our MAC as source,
    // an experimental EtherType, and enough payload to reach the 60-byte
    // minimum (the chip appends the 4-byte CRC).
    let mut frame = [0u8; 60];
    frame[0..6].copy_from_slice(&[0xff; 6]);
    frame[6..12].copy_from_slice(&mac);
    frame[12..14].copy_from_slice(&0x88b5u16.to_be_bytes());
    match lan9514.send_frame(dwc2, timer, &frame) {
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

        match lan9514.receive_frame(dwc2, timer) {
            Ok(Some(frame)) => print_frame(uart, frame),
            // Nothing received this poll.
            Ok(None) => {}
            Err(e) => {
                let _ = writeln!(
                    uart,
                    "receive error: {e:?} (hcint=0x{:08x})",
                    dwc2.last_channel_interrupt()
                );
            }
        }
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
