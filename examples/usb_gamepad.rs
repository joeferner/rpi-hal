#![no_std]
#![no_main]

// Reports live stick, trigger, D-pad and button state from a USB game
// controller over the UART, decoded generically. Powers the USB controller on,
// brings up the DWC2 core, enumerates the bus with `rpi_hal::usb::enumerate`,
// and hands each device to the `rpi_hal::usb::hid::gamepad` driver: the one
// that turns out to be a game controller has its HID **report descriptor**
// read and parsed into a field map (`rpi_hal::hid_report`), and its live input
// reports are then decoded entirely through that map -- no hard-coded
// per-device byte offsets.
//
// This is the USB counterpart of `bt_hid_gamepad.rs` (the same thing over
// Classic Bluetooth), and it shares that example's decoding half
// (`common/hid_gamepad.rs`) unchanged: the descriptor says where each axis /
// button / hat lives in the report, so the *same* code decodes any controller
// whatever its layout, over either transport. Only how the controller is
// reached differs -- here it's plugged into a USB port, so there's no pairing,
// no firmware blob, and no SD card involved.
//
// Note there's no HID boot protocol for game controllers (unlike
// `usb_hid_keyboard.rs` / `usb_hid_mouse.rs`, which use it): a controller's
// reports are only readable through the report descriptor, which is why this
// example exists as the report-protocol side of `usb::hid`.
//
// A full/low-speed controller behind the on-board hub is reached through the
// hub's transaction translator via (periodic) split transactions -- all handled
// inside the driver.

use core::fmt::Write;
use core::ops::ControlFlow;
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::usb;
use rpi_hal::usb::control::get_configuration_descriptor;
use rpi_hal::usb::dwc2::Dwc2Host;
use rpi_hal::usb::hid::gamepad::Gamepad;
use rpi_hal::usb::Device;

#[path = "common/hid_gamepad.rs"]
mod hid_gamepad;
use hid_gamepad::{print_fields, Decoder};

/// Report buffer size — comfortably over any controller's report endpoint max
/// packet size, so no report is read truncated.
const MAX_REPORT: usize = 64;

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

    // Enumerate the bus; the first game controller found is polled forever
    // (poll_gamepad never returns, so enumeration stops there). Other devices
    // are left addressed and skipped.
    let result = usb::enumerate(&mut dwc2, &timer, |dwc2, timer, device| {
        let mut gamepad = match Gamepad::from_device(dwc2, timer, device) {
            Ok(Some(gamepad)) => gamepad,
            Ok(None) => return ControlFlow::Continue(()),
            Err(e) => {
                // The error names the request that failed; dump what the
                // device said about itself alongside it, since a device that
                // refuses setup is only debuggable against its own
                // descriptors.
                let _ = writeln!(uart, "port {}: gamepad setup failed: {e:?}", device.port);
                dump_configuration(&mut uart, dwc2, timer, device);
                return ControlFlow::Continue(());
            }
        };

        let _ = writeln!(
            uart,
            "gamepad {:04x}:{:04x} on port {}: interface {}, endpoint 0x{:02x}, \
             {} byte reports every {}ms",
            device.descriptor.vendor_id,
            device.descriptor.product_id,
            device.port,
            gamepad.interface(),
            0x80 | gamepad.report_endpoint(),
            gamepad.max_packet_size(),
            gamepad.poll_interval_ms(),
        );

        // The raw hexdump + parsed field map are printed to make report-decode
        // issues debuggable against the actual descriptor bytes.
        hexdump(
            &mut uart,
            "HID report descriptor",
            gamepad.report_descriptor_bytes(),
        );
        print_fields(&mut uart, gamepad.report_descriptor());

        let _ = writeln!(
            uart,
            "leave the controls at rest, then move them.\r\n\
             (axes are labelled by HID usage and calibrated from the first report;\r\n\
             which usage is a stick vs a trigger is a device convention, not in the\r\n\
             descriptor -- an OS resolves that with a per-device quirk database.)"
        );
        poll_gamepad(&mut uart, dwc2, timer, &mut gamepad)
    });

    match result {
        Ok(()) => {
            let _ = writeln!(uart, "no gamepad found");
        }
        Err(e) => {
            let _ = writeln!(uart, "enumeration failed: {e:?}");
        }
    }
    halt();
}

/// Prints `bytes` as a labelled hexdump, 16 to a line.
fn hexdump(uart: &mut Uart, label: &str, bytes: &[u8]) {
    let _ = writeln!(uart, "{label} ({} bytes):", bytes.len());
    for (i, byte) in bytes.iter().enumerate() {
        if i % 16 == 0 {
            let _ = write!(uart, "\r\n  ");
        }
        let _ = write!(uart, "{byte:02X} ");
    }
    let _ = writeln!(uart, "\r\n");
}

/// Reads and dumps `device`'s configuration descriptor — the interfaces and
/// endpoints it declares, which is what a device that refused setup has to be
/// read against to see why.
fn dump_configuration(uart: &mut Uart, dwc2: &mut Dwc2Host, timer: &Timer, device: Device) {
    let mut config = [0u8; 64];
    match get_configuration_descriptor(dwc2, timer, device.endpoint, 0, &mut config) {
        Ok(len) => hexdump(uart, "  configuration descriptor", &config[..len]),
        Err(e) => {
            let _ = writeln!(uart, "  configuration descriptor unreadable too: {e:?}");
        }
    }
}

/// Polls `gamepad` forever, printing each report decoded through its field map
/// whenever the decoded state changes (a controller streams reports
/// continuously, whether or not anything moved).
fn poll_gamepad(uart: &mut Uart, dwc2: &mut Dwc2Host, timer: &Timer, gamepad: &mut Gamepad) -> ! {
    let mut decoder = Decoder::new();
    let mut buf = [0u8; MAX_REPORT];
    loop {
        // Pace polls -- interrupt endpoints mustn't be hammered.
        timer.delay_ms(gamepad.poll_interval_ms());

        match gamepad.poll(dwc2, timer, &mut buf) {
            Ok(Some(report)) => {
                decoder.print_changes(uart, gamepad.report_descriptor(), report.id, report.payload)
            }
            // No new report this poll.
            Ok(None) => {}
            Err(e) => {
                let _ = writeln!(
                    uart,
                    "poll error: {e:?} (hcint=0x{:08x})",
                    dwc2.last_channel_interrupt()
                );
            }
        }
    }
}
