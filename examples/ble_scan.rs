#![no_std]
#![no_main]

// BLE scan (Pi 3 only): brings the on-board BCM43438 Bluetooth controller
// up (see `bt_probe.rs`), then scans as a central and prints each nearby
// BLE device it hears -- address, signal strength, and name where the
// advertising data carries one. The inverse of `ble_advertise.rs`: this Pi
// is the scanner rather than the advertiser.
//
// Controller-level GAP scanning only (LE Set Scan Parameters/Enable +
// parsing LE Advertising Report events); no connection is made.
//
// Setup mirrors `bt_probe.rs`: the console is the mini UART (GPIO14/15,
// needs `core_freq=250` in `config.txt`), and the `.hcd` patchram blob is
// read off the SD card. In a `bt` directory on the boot partition, under
// an 8.3 name:
//   BT.HCD -- Broadcom's BCM43430A1.hcd patchram blob
//
// Get BCM43430A1.hcd (the original Pi 3B's BCM43438; not the 3B+/Zero 2 W's
// BCM4345C0.hcd) from RPi-Distro/bluez-firmware at
// https://github.com/RPi-Distro/bluez-firmware/blob/master/broadcom/BCM43430A1.hcd
// or copy it from /lib/firmware/brcm/ on any Raspberry Pi OS install.

use core::fmt::Write;
use rpi_hal::bluetooth::{AdvReport, Bluetooth};
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::mini_uart::MiniUart;
use rpi_hal::pac;
use rpi_hal::sd::Sd;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;

#[path = "common/mod.rs"]
mod common;
use common::{firmware_from_sd, HCI_BAUD};

/// How long to wait for each advertising report before printing a "still
/// scanning" heartbeat, in milliseconds.
const REPORT_WAIT_MS: u32 = 2_000;
/// Distinct device addresses remembered so each is printed once. Printing
/// to the slow mini-UART console blocks the reader for milliseconds, during
/// which HCI events streaming in at 3 Mbaud overrun the PL011 RX FIFO — so
/// the loop must print rarely and otherwise drain continuously. Deduping by
/// address gives that: a burst of prints as devices are discovered, then a
/// fast, print-free steady state.
const MAX_DEVICES: usize = 64;

/// A short human label for an advertising event type.
fn event_label(event_type: u8) -> &'static str {
    match event_type {
        0x00 => "connectable",
        0x01 => "directed",
        0x02 => "scannable",
        0x03 => "non-conn",
        0x04 => "scan-rsp",
        _ => "?",
    }
}

/// Prints one report as `AA:BB:CC:DD:EE:FF  -63 dBm  connectable  "Name"`.
fn print_report(console: &mut MiniUart, report: &AdvReport) {
    let a = report.address;
    let _ = write!(
        console,
        "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}  {:>4} dBm  {:<11}",
        a[5],
        a[4],
        a[3],
        a[2],
        a[1],
        a[0],
        report.rssi,
        event_label(report.event_type),
    );
    match report.name() {
        Some(name) => {
            let _ = writeln!(console, "  \"{name}\"");
        }
        None => {
            let _ = writeln!(console);
        }
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut console = MiniUart::init(&peripherals.GPIO, &peripherals.AUX, peripherals.UART1);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // Read the firmware blob off the SD card first (this owns EMMC).
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

    // Bring the controller up: commit the PL011 to its HCI pins, assert
    // BT_ON, download the patchram firmware, and raise the link baud.
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
    let _ = writeln!(console, "controller ready");

    // Start scanning and stream the reports. Duplicate filtering is on, so
    // each device is reported once per scan session.
    if let Err(e) = bt.start_scan(&timer) {
        let _ = writeln!(console, "start scan failed: {e:?}");
        halt();
    }
    let _ = writeln!(console, "scanning for BLE devices...");

    // Remember which devices we've printed, and whether we've shown a name
    // for each: a device's name often rides in its scan response, a
    // separate report from its initial advertisement, so a device first
    // seen without a name gets one follow-up line once the name arrives.
    let mut addrs = [[0u8; 6]; MAX_DEVICES];
    let mut named = [false; MAX_DEVICES];
    let mut count = 0;

    loop {
        let report = match bt.next_advertising_report(&timer, REPORT_WAIT_MS) {
            Ok(Some(report)) => report,
            // Quiet air — nothing to print. Keep looping (and draining).
            Ok(None) => continue,
            Err(e) => {
                let _ = writeln!(console, "scan read error: {e:?}");
                halt();
            }
        };

        match addrs[..count].iter().position(|a| *a == report.address) {
            // New device: record and print it.
            None => {
                if count < MAX_DEVICES {
                    addrs[count] = report.address;
                    named[count] = report.name().is_some();
                    count += 1;
                    print_report(&mut console, &report);
                    if count == MAX_DEVICES {
                        let _ = writeln!(console, "(device table full; new devices not shown)");
                    }
                }
            }
            // Seen before: print again only if we just learned its name,
            // otherwise skip silently so the loop keeps draining fast.
            Some(i) => {
                if !named[i] && report.name().is_some() {
                    named[i] = true;
                    print_report(&mut console, &report);
                }
            }
        }
    }
}
