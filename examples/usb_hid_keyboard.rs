#![no_std]
#![no_main]

// Reads live key presses from a USB HID boot keyboard and echoes the
// typed characters over the UART. Powers the USB controller on, brings up
// the DWC2 core, and enumerates the bus with `rpi_hal::usb::enumerate`;
// the first device that turns out to be a HID keyboard is handed to the
// `rpi_hal::usb::hid::keyboard` driver, which switches it to the boot
// protocol and turns its interrupt-IN report endpoint into key events.
// A full/low-speed keyboard behind the on-board hub is reached through
// the hub's transaction translator via (periodic) split transactions --
// all handled inside the driver.

use core::fmt::Write;
use core::ops::ControlFlow;
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::usb;
use rpi_hal::usb::dwc2::{Channel, Dwc2Host};
use rpi_hal::usb::hid::keyboard::{usage_to_ascii, KeyEvent, Keyboard};

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

    let _ = writeln!(uart, "waiting for the on-board hub...");
    while !dwc2.port_connected() {
        timer.delay_ms(100);
    }

    // Enumerate the bus; the first HID keyboard found is polled forever
    // (poll_keyboard never returns, so enumeration stops there). Other
    // devices are left addressed and skipped.
    let result = usb::enumerate(&dwc2, &timer, |channel, timer, device| {
        let mut keyboard = match Keyboard::from_device(channel, timer, device) {
            Ok(Some(keyboard)) => keyboard,
            Ok(None) => return ControlFlow::Continue(()),
            Err(e) => {
                let _ = writeln!(uart, "port {}: HID setup failed: {e:?}", device.port);
                return ControlFlow::Continue(());
            }
        };

        let _ = writeln!(
            uart,
            "HID keyboard on port {}: interface {}, endpoint 0x{:02x}, {} byte reports -- type!",
            device.port,
            keyboard.interface(),
            0x80 | keyboard.report_endpoint(),
            keyboard.max_packet_size(),
        );
        poll_keyboard(&mut uart, channel, timer, &mut keyboard)
    });

    match result {
        Ok(()) => {
            let _ = writeln!(uart, "no keyboard found");
        }
        Err(e) => {
            let _ = writeln!(uart, "enumeration failed: {e:?}");
        }
    }
    halt();
}

/// Polls `keyboard` forever, echoing the characters of newly-pressed keys
/// over `uart`.
fn poll_keyboard(
    uart: &mut Uart,
    channel: &mut Channel,
    timer: &Timer,
    keyboard: &mut Keyboard,
) -> ! {
    loop {
        // Pace polls -- interrupt endpoints mustn't be hammered.
        timer.delay_ms(10);

        match keyboard.poll(channel, timer) {
            Ok(Some(events)) => {
                let shift = events.report().modifiers.shift();
                for event in events.iter() {
                    if let KeyEvent::Pressed(usage) = event {
                        if let Some(c) = usage_to_ascii(usage, shift) {
                            let _ = write!(uart, "{c}");
                        } else {
                            let _ = write!(uart, "{{{usage}}}");
                        }
                    }
                }
            }
            // No new report this poll.
            Ok(None) => {}
            Err(e) => {
                let _ = writeln!(
                    uart,
                    "\npoll error: {e:?} (hcint=0x{:08x})",
                    channel.last_interrupt()
                );
            }
        }
    }
}
