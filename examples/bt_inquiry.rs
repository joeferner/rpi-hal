#![no_std]
#![no_main]

// Bluetooth Classic inquiry (Pi 3 only): brings the on-board BCM43438
// controller up and runs a BR/EDR *inquiry* -- Classic device discovery, the
// counterpart to `ble_scan.rs`'s LE scan. It prints each nearby discoverable
// Classic device: its address, Class of Device (decoded to a major class, and
// flagged when it's a game controller), signal strength, and name (from the
// Extended Inquiry Response, when present).
//
// This is Phase A of Bluetooth Classic support: it proves the Classic
// transport path end to end (inquiry command -> Inquiry Result events ->
// parsed device list) before the connection/pairing/L2CAP layers are built on
// top. Point it at a controller in pairing mode (e.g. an 8BitDo SN30 Pro+ or
// an older Xbox pad) -- it should appear flagged as a gamepad, which is how we
// confirm we can see it over Classic at all.
//
// Setup mirrors `ble_scan.rs`: the console is the mini UART (GPIO14/15, needs
// `core_freq=250` in `config.txt`), and the `.hcd` patchram blob is read off
// the SD card. In a `bt` directory on the boot partition, under an 8.3 name:
//   BT.HCD -- Broadcom's BCM43430A1.hcd patchram blob
//
// Get BCM43430A1.hcd (the original Pi 3B's BCM43438; not the 3B+/Zero 2 W's
// BCM4345C0.hcd) from RPi-Distro/bluez-firmware at
// https://github.com/RPi-Distro/bluez-firmware/blob/master/broadcom/BCM43430A1.hcd
// or copy it from /lib/firmware/brcm/ on any Raspberry Pi OS install.

use core::fmt::Write;
use rpi_hal::bluetooth::{Bluetooth, InquiryResult};
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::mini_uart::MiniUart;
use rpi_hal::pac;
use rpi_hal::sd::Sd;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;

#[path = "common/mod.rs"]
mod common;
use common::{firmware_from_sd, write_address, HCI_BAUD};

/// How long to wait for each inquiry result before looping, in ms.
const RESULT_WAIT_MS: u32 = 2_000;
/// How many distinct device addresses are remembered so each prints once.
const MAX_SEEN: usize = 32;

/// Names the major device class for the common cases, for readable output.
fn major_class_name(major: u8) -> &'static str {
    match major {
        0x01 => "computer",
        0x02 => "phone",
        0x03 => "network",
        0x04 => "audio/video",
        0x05 => "peripheral",
        0x06 => "imaging",
        0x07 => "wearable",
        0x08 => "toy",
        0x09 => "health",
        _ => "other",
    }
}

/// Prints one discovered device.
fn print_result(console: &mut MiniUart, result: &InquiryResult) {
    let _ = write!(console, "  ");
    write_address(console, &result.bd_addr);
    if let Some(name) = result.name() {
        let _ = write!(console, " '{name}'");
    }
    let _ = write!(
        console,
        " CoD {:#08x} ({})",
        result.class_of_device_u24(),
        major_class_name(result.major_device_class())
    );
    if result.is_gamepad() {
        let _ = write!(console, " <-- GAMEPAD");
    }
    if let Some(rssi) = result.rssi {
        let _ = write!(console, " ({rssi} dBm)");
    }
    let _ = writeln!(console);
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut console = MiniUart::init(&peripherals.GPIO, &peripherals.AUX, peripherals.UART1);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    let _ = writeln!(console, "reading Bluetooth firmware from SD card...");
    let sd = match Sd::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, &timer) {
        Ok(sd) => sd,
        Err(e) => {
            let _ = writeln!(console, "SD init failed: {e:?}");
            halt();
        }
    };
    let hcd = match firmware_from_sd(sd, &timer) {
        Ok(hcd) => hcd,
        Err(e) => {
            let _ = writeln!(
                console,
                "reading {}/{} failed: {e:?}",
                common::BT_DIR,
                common::FIRMWARE_FILE
            );
            halt();
        }
    };

    let _ = writeln!(console, "bringing up Bluetooth controller over HCI...");
    let hci = Uart::init_bluetooth(&peripherals.GPIO, peripherals.UART0);
    let mut bt = Bluetooth::new(hci, &mut mailbox, &timer);

    if let Err(e) = bt.load_firmware(hcd, &timer) {
        let _ = writeln!(console, "firmware load failed: {e:?}");
        halt();
    }
    if let Err(e) = bt.set_baud(HCI_BAUD, &timer) {
        let _ = writeln!(console, "baud bump failed: {e:?}");
        halt();
    }

    // Extended inquiry gives RSSI + the device name; tolerate a controller
    // that declines it (we still get address + Class of Device).
    if let Err(e) = bt.set_inquiry_mode_extended(&timer) {
        let _ = writeln!(
            console,
            "extended inquiry mode declined ({e:?}) -- continuing"
        );
    }
    let _ = writeln!(console, "controller ready");

    if let Err(e) = bt.start_inquiry(&timer) {
        let _ = writeln!(console, "start inquiry failed: {e:?}");
        halt();
    }
    let _ = writeln!(
        console,
        "inquiring for Classic devices (put the controller in pairing mode)..."
    );

    // Stream results, printing each distinct address once. Printing to the
    // slow mini-UART blocks the reader, so dedup keeps it to a burst of prints
    // as devices are found, then a quiet, drain-only steady state.
    let mut seen: [[u8; 6]; MAX_SEEN] = [[0u8; 6]; MAX_SEEN];
    let mut seen_count = 0;
    loop {
        match bt.next_inquiry_result(&timer, RESULT_WAIT_MS) {
            Ok(Some(result)) => {
                let already = seen[..seen_count].contains(&result.bd_addr);
                if !already {
                    if seen_count < seen.len() {
                        seen[seen_count] = result.bd_addr;
                        seen_count += 1;
                    }
                    print_result(&mut console, &result);
                }
            }
            Ok(None) => {} // quiet window; keep inquiring
            Err(e) => {
                let _ = writeln!(console, "inquiry error: {e:?}");
                halt();
            }
        }
    }
}
