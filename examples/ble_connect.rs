#![no_std]
#![no_main]

// BLE connection transport (Pi 3 only): brings the on-board BCM43438
// Bluetooth controller up (see `bt_probe.rs`/`ble_advertise.rs` for that
// path), then advertises *connectably* and services the connection over
// HCI -- reporting when a central (a phone) connects, dumping the raw ACL
// data it sends, and reporting when it disconnects.
//
// This is the transport layer beneath the LE host stack: it proves the
// connection path (LE Connection Complete -> a live handle), inbound ACL
// data (the phone's first L2CAP/ATT requests arrive as ACL fragments), and
// disconnect handling. There's no L2CAP/ATT/GATT above it yet, so a phone
// connects, finds no services, and drops the link after a moment -- the
// ACL bytes printed here are its ATT MTU-exchange / service-discovery
// attempt going unanswered. Point a scanner (nRF Connect, LightBlue) at
// "rpi-hal-le" and tap Connect to exercise it.
//
// Setup mirrors `ble_advertise.rs`: the console is the mini UART
// (GPIO14/15, needs `core_freq=250` in `config.txt`), and the `.hcd`
// patchram blob is read off the SD card. In a `bt` directory on the boot
// partition, under an 8.3 name:
//   BT.HCD -- Broadcom's BCM43430A1.hcd patchram blob
//
// Get BCM43430A1.hcd (the original Pi 3B's BCM43438; not the 3B+/Zero 2 W's
// BCM4345C0.hcd) from RPi-Distro/bluez-firmware at
// https://github.com/RPi-Distro/bluez-firmware/blob/master/broadcom/BCM43430A1.hcd
// or copy it from /lib/firmware/brcm/ on any Raspberry Pi OS install.

use core::fmt::Write;
use rpi_hal::bluetooth::{Advertising, Bluetooth, Connection, Event};
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

/// The name this peripheral advertises under.
const ADV_NAME: &str = "rpi-hal-le";
/// How long each [`Bluetooth::poll`] waits for the next event, in ms.
const POLL_MS: u32 = 1000;

/// Logs an established connection.
fn on_connected(console: &mut MiniUart, conn: &Connection) {
    let _ = write!(
        console,
        "connected: handle {:#06x}, role {:?}, peer ",
        conn.handle, conn.role
    );
    write_address(console, &conn.peer_address);
    let _ = writeln!(
        console,
        " (type {})  -- dumping inbound ACL (its ATT/L2CAP requests)",
        conn.peer_address_type
    );
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

    // Arm ACL flow control: learn the controller's ACL packet size and how
    // many packets it can buffer (the TX credit pool `send_acl` spends).
    match bt.le_read_buffer_size(&timer) {
        Ok((packet_len, total)) => {
            let _ = writeln!(
                console,
                "controller ready (ACL packet len {packet_len}, {total} buffers)"
            );
        }
        Err(e) => {
            let _ = writeln!(console, "LE read buffer size failed: {e:?}");
            halt();
        }
    }

    // Advertise connectably so a central can connect. The controller stops
    // advertising on its own once one does; we re-enable after a disconnect.
    if let Err(e) = bt.start_advertising(ADV_NAME, Advertising::Connectable, &timer) {
        let _ = writeln!(console, "start advertising failed: {e:?}");
        halt();
    }
    let _ = writeln!(
        console,
        "advertising connectably as '{ADV_NAME}' -- tap Connect in a phone scanner"
    );

    // Service the connection: report connect/disconnect and dump inbound
    // ACL. On disconnect, re-advertise to accept the next connection.
    loop {
        match bt.poll(&timer, POLL_MS) {
            Ok(Some(Event::Connected(conn))) => on_connected(&mut console, &conn),
            Ok(Some(Event::Acl(acl))) => {
                let _ = write!(
                    console,
                    "  ACL handle {:#06x} {} [{} bytes]:",
                    acl.handle,
                    if acl.first_fragment { "start" } else { "cont " },
                    acl.data().len()
                );
                for byte in acl.data() {
                    let _ = write!(console, " {byte:02x}");
                }
                let _ = writeln!(console);
            }
            Ok(Some(Event::Disconnected { handle, reason })) => {
                let _ = writeln!(
                    console,
                    "disconnected: handle {handle:#06x}, reason {reason:#04x} -- re-advertising"
                );
                if let Err(e) = bt.start_advertising(ADV_NAME, Advertising::Connectable, &timer) {
                    let _ = writeln!(console, "re-advertise failed: {e:?}");
                    halt();
                }
            }
            // Other events (LTK request, encryption change) don't arise
            // without pairing, which this transport-only example doesn't do.
            Ok(Some(_)) => {}
            Ok(None) => {} // quiet window; keep polling
            Err(e) => {
                let _ = writeln!(console, "poll error: {e:?}");
                halt();
            }
        }
    }
}
