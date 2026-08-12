#![no_std]
#![no_main]

// Enumerates every USB device on the board and prints what it finds.
// Powers the USB controller on through the VideoCore mailbox, brings up
// the DWC2 core, then hands the whole enumeration off to
// `rpi_hal::usb::enumerate` -- which resets the root port, configures the
// on-board SMSC LAN9514 hub, and reaches through it to reset and address
// each connected downstream device (high-speed directly, full/low-speed
// via split transactions). This example just prints each device's
// vendor/product/class; see usb_hid_keyboard.rs for one that talks to a
// device it finds.

use core::fmt::Write;
use core::ops::ControlFlow;
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::usb;
use rpi_hal::usb::dwc2::Dwc2Host;

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

    let result = usb::enumerate(&mut dwc2, &timer, |_dwc2, _timer, device| {
        let _ = writeln!(
            uart,
            "port {}: {:04x}:{:04x} class={} -> address {}",
            device.port,
            device.descriptor.vendor_id,
            device.descriptor.product_id,
            device.descriptor.device_class,
            device.endpoint.address,
        );
        ControlFlow::Continue(())
    });
    match result {
        Ok(()) => {
            let _ = writeln!(uart, "enumeration complete");
        }
        Err(e) => {
            let _ = writeln!(uart, "enumeration failed: {e:?}");
        }
    }
    halt();
}
