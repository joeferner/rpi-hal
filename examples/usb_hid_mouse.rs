#![no_std]
#![no_main]

// Reports live movement and button clicks from a USB HID boot mouse over
// the UART. Powers the USB controller on, brings up the DWC2 core, and
// enumerates the bus with `rpi_hal::usb::enumerate`; the first device
// that turns out to be a HID mouse is handed to the
// `rpi_hal::usb::hid::mouse` driver, which switches it to the boot
// protocol and turns its interrupt-IN report endpoint into relative
// movement plus button press/release events. A full/low-speed mouse
// behind the on-board hub is reached through the hub's transaction
// translator via (periodic) split transactions -- all handled inside the
// driver.
//
// Note: the boot mouse report is only buttons + relative X/Y; the scroll
// wheel isn't part of it (a mouse may send a 4th wheel byte, but many,
// like a strict 3-byte boot mouse, don't). Wheel support in general needs
// the HID report protocol, which this driver doesn't do yet.

use core::fmt::Write;
use core::ops::ControlFlow;
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::usb;
use rpi_hal::usb::dwc2::{Channel, Dwc2Host};
use rpi_hal::usb::hid::mouse::{ButtonEvent, Mouse};

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

    // Enumerate the bus; the first HID mouse found is polled forever
    // (poll_mouse never returns, so enumeration stops there). Other
    // devices are left addressed and skipped.
    let result = usb::enumerate(&dwc2, &timer, |channel, timer, device| {
        let mut mouse = match Mouse::from_device(channel, timer, device) {
            Ok(Some(mouse)) => mouse,
            Ok(None) => return ControlFlow::Continue(()),
            Err(e) => {
                let _ = writeln!(uart, "port {}: HID setup failed: {e:?}", device.port);
                return ControlFlow::Continue(());
            }
        };

        let _ = writeln!(
            uart,
            "HID mouse on port {}: interface {}, endpoint 0x{:02x}, {} byte reports -- move it!",
            device.port,
            mouse.interface(),
            0x80 | mouse.report_endpoint(),
            mouse.max_packet_size(),
        );
        poll_mouse(&mut uart, channel, timer, &mut mouse)
    });

    match result {
        Ok(()) => {
            let _ = writeln!(uart, "no mouse found");
        }
        Err(e) => {
            let _ = writeln!(uart, "enumeration failed: {e:?}");
        }
    }
    halt();
}

/// Polls `mouse` forever, printing movement deltas and button events.
fn poll_mouse(uart: &mut Uart, channel: &mut Channel, timer: &Timer, mouse: &mut Mouse) -> ! {
    loop {
        // Pace polls -- interrupt endpoints mustn't be hammered.
        timer.delay_ms(10);

        match mouse.poll(channel, timer) {
            Ok(Some(update)) => {
                for event in update.button_events() {
                    match event {
                        ButtonEvent::Pressed(button) => {
                            let _ = writeln!(uart, "{button:?} down");
                        }
                        ButtonEvent::Released(button) => {
                            let _ = writeln!(uart, "{button:?} up");
                        }
                    }
                }
                let report = update.report();
                if report.x != 0 || report.y != 0 {
                    let _ = writeln!(uart, "move dx={} dy={}", report.x, report.y);
                }
            }
            // No new report this poll.
            Ok(None) => {}
            Err(e) => {
                let _ = writeln!(
                    uart,
                    "poll error: {e:?} (hcint=0x{:08x})",
                    channel.last_interrupt()
                );
            }
        }
    }
}
