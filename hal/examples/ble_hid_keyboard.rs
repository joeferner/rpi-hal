#![no_std]
#![no_main]

// BLE HID keyboard (Pi 3 only): advertises the Pi as a Bluetooth Low Energy
// keyboard (HID-over-GATT / HOGP) and, once a client connects, pairs to an
// encrypted link and streams key-press reports. This is Phase 5a of the
// USB->BLE keyboard: the BLE HID side driven by *synthetic* keystrokes (it
// "types" a, b, c, ... on a timer) so the HID service can be verified in
// isolation. Wiring in a real attached USB keyboard is the next step.
//
// It composes the layers built earlier: connection + L2CAP (`ble_l2cap.rs`),
// the generic GATT server (`ble_gatt.rs`) now hosting the standard HID
// service, and LE Legacy Just Works pairing *with bonding* (`ble_pair.rs` +
// SMP key distribution) for the encrypted, bonded link a HID host requires.
// The advertising data carries the keyboard Appearance and the HID service
// UUID so a host recognises it as a keyboard.
//
// Verify from the host OS, not nRF Connect: HID-over-GATT is a system
// profile, so the OS reserves the HID service and third-party GATT apps
// can't subscribe to its reports. Instead, pair "rpi-hal-kbd" from the
// phone/PC's Bluetooth *settings* (it should appear as a keyboard), then
// open any text field -- it types a, b, c, ... on a timer. Byte 2 of each
// report is the USB HID keycode (0x04='a', 0x05='b', ...). Bonding
// (distributing an LTK) is what lets the OS accept and keep it as a
// keyboard; the bond lives in RAM, so a Pi reboot needs a re-pair.
//
// Setup mirrors `ble_pair.rs`: the console is the mini UART (GPIO14/15, needs
// `core_freq=250` in `config.txt`), and the `.hcd` patchram blob is read off
// the SD card. In a `bt` directory on the boot partition, under an 8.3 name:
//   BT.HCD -- Broadcom's BCM43430A1.hcd patchram blob
//
// Get BCM43430A1.hcd (the original Pi 3B's BCM43438; not the 3B+/Zero 2 W's
// BCM4345C0.hcd) from RPi-Distro/bluez-firmware at
// https://github.com/RPi-Distro/bluez-firmware/blob/master/broadcom/BCM43430A1.hcd
// or copy it from /lib/firmware/brcm/ on any Raspberry Pi OS install.

use core::fmt::Write;
use rpi_hal::bluetooth::gatt::{
    attr, cccd, characteristic, primary_service, Attribute, Server, ATT_MAX_MTU,
};
use rpi_hal::bluetooth::l2cap::{self, Reassembler, CID_ATT, CID_SMP};
use rpi_hal::bluetooth::smp::Smp;
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
use common::{firmware_from_sd, HCI_BAUD};

/// The device name (matches the name in [`ADV_DATA`]).
const DEVICE_NAME: &str = "rpi-hal-kbd";
/// How long each [`Bluetooth::poll`] waits for the next event, in ms.
const POLL_MS: u32 = 500;
/// How often to emit a synthetic key event (down or up), in ms.
const KEYSTROKE_INTERVAL_MS: u32 = 1500;
/// HCI address type `Public` — the address we advertise/pair with.
const ADDR_TYPE_PUBLIC: u8 = 0x00;
/// Handle of the HID Input Report *value* — the one notified with key data.
const INPUT_REPORT_HANDLE: u16 = 0x003a;
/// USB HID keyboard usage code for the letter 'a'; 'b' is 0x05, and so on.
const KEYCODE_A: u8 = 0x04;

/// The HID Report Map: a standard boot-compatible keyboard descriptor with
/// an 8-byte input report (1 modifier byte, 1 reserved byte, 6 key codes),
/// no LED output. This is the same report layout a USB boot keyboard uses,
/// so a USB report forwards through unchanged.
static REPORT_MAP: [u8; 45] = [
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x06, // Usage (Keyboard)
    0xa1, 0x01, // Collection (Application)
    0x05, 0x07, //   Usage Page (Keyboard/Keypad)
    0x19, 0xe0, //   Usage Minimum (0xE0, Left Control)
    0x29, 0xe7, //   Usage Maximum (0xE7, Right GUI)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x08, //   Report Count (8)
    0x81, 0x02, //   Input (Data, Variable, Absolute) - modifier bits
    0x95, 0x01, //   Report Count (1)
    0x75, 0x08, //   Report Size (8)
    0x81, 0x01, //   Input (Constant) - reserved byte
    0x95, 0x06, //   Report Count (6)
    0x75, 0x08, //   Report Size (8)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x65, //   Logical Maximum (101)
    0x05, 0x07, //   Usage Page (Keyboard/Keypad)
    0x19, 0x00, //   Usage Minimum (0)
    0x29, 0x65, //   Usage Maximum (101)
    0x81, 0x00, //   Input (Data, Array) - the 6 key codes
    0xc0, // End Collection
];

/// The advertising data: Flags, Appearance = Keyboard (0x03C1), the HID
/// service UUID (0x1812) so a host recognises it as a keyboard, and the
/// complete local name.
static ADV_DATA: [u8; 24] = [
    0x02, 0x01, 0x06, // Flags: LE General Discoverable, BR/EDR not supported
    0x03, 0x19, 0xc1, 0x03, // Appearance = Keyboard (0x03C1)
    0x03, 0x03, 0x12, 0x18, // Complete list of 16-bit Service UUIDs: HID (0x1812)
    0x0c, 0x09, b'r', b'p', b'i', b'-', b'h', b'a', b'l', b'-', b'k', b'b', b'd', // Name
];

/// The HOGP attribute table (handle-ordered). Characteristic declaration
/// values are `[properties, value_handle(LE), uuid(LE)]`; see `bluetooth::gatt`.
static ATTRIBUTES: [Attribute; 26] = [
    // GAP service (0x1800): Device Name + Appearance (Keyboard).
    primary_service(0x0001, &[0x00, 0x18]),
    characteristic(0x0002, &[0x02, 0x03, 0x00, 0x00, 0x2a]),
    attr(0x0003, 0x2a00, DEVICE_NAME.as_bytes()),
    characteristic(0x0004, &[0x02, 0x05, 0x00, 0x01, 0x2a]),
    attr(0x0005, 0x2a01, &[0xc1, 0x03]), // Appearance = Keyboard
    // GATT service (0x1801).
    primary_service(0x0006, &[0x01, 0x18]),
    // Device Information service (0x180A): PnP ID (helps a host classify us).
    primary_service(0x0010, &[0x0a, 0x18]),
    characteristic(0x0011, &[0x02, 0x12, 0x00, 0x50, 0x2a]),
    // PnP ID: vendor source (USB-IF), VID 0x1D6B, PID 0x0246, version 0x0001.
    attr(0x0012, 0x2a50, &[0x02, 0x6b, 0x1d, 0x46, 0x02, 0x01, 0x00]),
    // Battery service (0x180F): a readable Battery Level.
    primary_service(0x0020, &[0x0f, 0x18]),
    characteristic(0x0021, &[0x12, 0x22, 0x00, 0x19, 0x2a]),
    attr(0x0022, 0x2a19, &[100]),
    cccd(0x0023),
    // HID service (0x1812).
    primary_service(0x0030, &[0x12, 0x18]),
    // HID Information: bcdHID 0x0111, country 0, flags 0x03 (remote wake +
    // normally connectable).
    characteristic(0x0031, &[0x02, 0x32, 0x00, 0x4a, 0x2a]),
    attr(0x0032, 0x2a4a, &[0x11, 0x01, 0x00, 0x03]),
    // Report Map (the keyboard HID descriptor above).
    characteristic(0x0033, &[0x02, 0x34, 0x00, 0x4b, 0x2a]),
    attr(0x0034, 0x2a4b, &REPORT_MAP),
    // HID Control Point: write without response (suspend/exit-suspend).
    characteristic(0x0035, &[0x04, 0x36, 0x00, 0x4c, 0x2a]),
    attr(0x0036, 0x2a4c, &[0x00]),
    // Protocol Mode: read + write without response; default Report Protocol.
    characteristic(0x0037, &[0x06, 0x38, 0x00, 0x4e, 0x2a]),
    attr(0x0038, 0x2a4e, &[0x01]),
    // Report (Input): read + notify, with its CCC and Report Reference.
    characteristic(0x0039, &[0x12, 0x3a, 0x00, 0x4d, 0x2a]),
    attr(0x003a, 0x2a4d, &[0, 0, 0, 0, 0, 0, 0, 0]),
    cccd(0x003b),
    // Report Reference: report ID 0, type 1 (Input).
    attr(0x003c, 0x2908, &[0x00, 0x01]),
];

/// Logs an established connection.
fn on_connected(console: &mut MiniUart, conn: &Connection) {
    let _ = write!(console, "connected: handle {:#06x}, peer ", conn.handle);
    for (i, byte) in conn.peer_address.iter().rev().enumerate() {
        if i != 0 {
            let _ = write!(console, ":");
        }
        let _ = write!(console, "{byte:02X}");
    }
    let _ = writeln!(console, " (type {})", conn.peer_address_type);
}

/// The 8-byte HID keyboard report for pressing a single key with no
/// modifiers: `[modifiers, reserved, key, 0, 0, 0, 0, 0]`.
fn key_report(keycode: u8) -> [u8; 8] {
    [0, 0, keycode, 0, 0, 0, 0, 0]
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

    // Bring the controller up.
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
    if let Err(e) = bt.le_read_buffer_size(&timer) {
        let _ = writeln!(console, "LE read buffer size failed: {e:?}");
        halt();
    }

    let own_addr = match bt.read_bd_addr(&timer) {
        Ok(addr) => addr,
        Err(e) => {
            let _ = writeln!(console, "read BD_ADDR failed: {e:?}");
            halt();
        }
    };
    // Bond (bonding = true): distribute an LTK so the OS keeps us paired as
    // a keyboard across reconnects.
    let mut smp = match Smp::new(&mut bt, &timer, true) {
        Ok(smp) => smp,
        Err(e) => {
            let _ = writeln!(console, "SMP init (crypto self-test) failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(console, "controller ready, pairing crypto ok");

    // Advertise as a keyboard (Appearance + HID service UUID in the AD).
    if let Err(e) = bt.start_advertising_raw(&ADV_DATA, Advertising::Connectable, &timer) {
        let _ = writeln!(console, "start advertising failed: {e:?}");
        halt();
    }
    let _ = writeln!(
        console,
        "advertising as BLE keyboard '{DEVICE_NAME}' -- connect, Bond, and enable Report notifications"
    );

    let mut reasm = Reassembler::new();
    let mut server = Server::new(&ATTRIBUTES);
    let mut att_out = [0u8; ATT_MAX_MTU as usize];
    let mut smp_out = [0u8; 32];
    let mut conn_handle: Option<u16> = None;
    let mut encrypted = false;
    // Synthetic typing state: which letter (0..25) and whether it's held.
    let mut letter: u8 = 0;
    let mut key_down = false;
    let mut next_key = timer.now_micros() + (KEYSTROKE_INTERVAL_MS as u64) * 1000;

    loop {
        match bt.poll(&timer, POLL_MS) {
            Ok(Some(Event::Connected(conn))) => {
                on_connected(&mut console, &conn);
                conn_handle = Some(conn.handle);
                encrypted = false;
                smp.begin(
                    conn.peer_address,
                    conn.peer_address_type,
                    own_addr,
                    ADDR_TYPE_PUBLIC,
                );
            }
            Ok(Some(Event::Acl(acl))) => {
                let handle = acl.handle;
                if let Some(pdu) = reasm.feed(&acl) {
                    match pdu.cid {
                        CID_ATT => {
                            if let Some(n) = server.handle(pdu.payload, &mut att_out) {
                                let _ = l2cap::send(&mut bt, handle, CID_ATT, &att_out[..n]);
                            }
                        }
                        CID_SMP => match smp.handle(&mut bt, &timer, pdu.payload, &mut smp_out) {
                            Ok(Some(n)) => {
                                let _ = l2cap::send(&mut bt, handle, CID_SMP, &smp_out[..n]);
                            }
                            Ok(None) => {}
                            Err(e) => {
                                let _ = writeln!(console, "  SMP error: {e:?}");
                            }
                        },
                        _ => {}
                    }
                }
            }
            Ok(Some(Event::LongTermKeyRequest { handle, ediv, rand })) => {
                if let Some(key) = smp.long_term_key(ediv, rand) {
                    let _ = bt.le_ltk_request_reply(handle, &key, &timer);
                } else {
                    let _ = bt.le_ltk_request_negative_reply(handle, &timer);
                }
            }
            Ok(Some(Event::EncryptionChange { handle, enabled })) => {
                encrypted = enabled;
                let _ = writeln!(
                    console,
                    "link {}",
                    if enabled {
                        "ENCRYPTED"
                    } else {
                        "not encrypted"
                    }
                );
                // On first encryption, distribute the bond keys so the OS
                // keeps us paired as a keyboard across reconnects.
                if enabled {
                    match smp.distribute_keys(&mut bt, &timer) {
                        Ok(Some(keys)) => {
                            let _ =
                                l2cap::send(&mut bt, handle, CID_SMP, &keys.encryption_information);
                            let _ =
                                l2cap::send(&mut bt, handle, CID_SMP, &keys.master_identification);
                            let _ = writeln!(console, "  distributed LTK -- bonded");
                        }
                        Ok(None) => {}
                        Err(e) => {
                            let _ = writeln!(console, "  key distribution failed: {e:?}");
                        }
                    }
                }
            }
            Ok(Some(Event::Disconnected { handle, reason })) => {
                let _ = writeln!(
                    console,
                    "disconnected: handle {handle:#06x}, reason {reason:#04x} -- re-advertising"
                );
                conn_handle = None;
                encrypted = false;
                reasm.reset();
                server.reset();
                smp.reset();
                if let Err(e) =
                    bt.start_advertising_raw(&ADV_DATA, Advertising::Connectable, &timer)
                {
                    let _ = writeln!(console, "re-advertise failed: {e:?}");
                    halt();
                }
            }
            Ok(None) => {}
            Err(e) => {
                let _ = writeln!(console, "poll error: {e:?}");
                halt();
            }
        }

        // Drive synthetic keystrokes once the link is encrypted and the
        // client has subscribed to the Report characteristic. Each tick
        // toggles a key down (a, b, c, ...) then up.
        if let Some(handle) = conn_handle {
            if encrypted
                && server.is_subscribed(INPUT_REPORT_HANDLE)
                && timer.now_micros() >= next_key
            {
                next_key = timer.now_micros() + (KEYSTROKE_INTERVAL_MS as u64) * 1000;
                let report = if key_down {
                    // Release: all-zero report.
                    key_down = false;
                    let letter_char = (b'a' + letter) as char;
                    let _ = writeln!(console, "  key up '{letter_char}'");
                    letter = (letter + 1) % 26;
                    [0u8; 8]
                } else {
                    // Press the current letter.
                    key_down = true;
                    let letter_char = (b'a' + letter) as char;
                    let _ = writeln!(console, "  key down '{letter_char}'");
                    key_report(KEYCODE_A + letter)
                };
                if let Some(n) = server.notification(INPUT_REPORT_HANDLE, &report, &mut att_out) {
                    let _ = l2cap::send(&mut bt, handle, CID_ATT, &att_out[..n]);
                }
            }
        }
    }
}
