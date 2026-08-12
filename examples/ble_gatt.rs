#![no_std]
#![no_main]

// BLE GATT server (Pi 3 only): brings the on-board BCM43438 Bluetooth
// controller up, advertises connectably, and runs a real GATT server on top
// of the L2CAP/ATT stack (see `ble_l2cap.rs` for the layer beneath). Where
// `ble_l2cap.rs` answers every ATT request with "not supported", this
// serves an actual attribute table, so a connecting phone completes service
// discovery and shows a named device with readable characteristics instead
// of giving up.
//
// The attribute table here is illustrative, not keyboard-specific: the
// `bluetooth::gatt` server is generic, and any peripheral supplies its own
// `&[Attribute]`. This one exposes the standard GAP service (Device Name
// "rpi-hal-le", Appearance), an (empty) GATT service, a Device Information
// service (Manufacturer Name String "rpi-hal"), and a Battery service whose
// Battery Level characteristic is readable *and notifiable*. Subscribe to
// Battery Level in a scanner (nRF Connect, LightBlue) and the value ticks
// down once every couple of seconds -- the server pushes a Handle Value
// Notification whenever the client has written the characteristic's CCC
// descriptor. This is the same subscribe/notify path a HID keyboard uses to
// push key reports.
//
// Setup mirrors `ble_l2cap.rs`: the console is the mini UART (GPIO14/15,
// needs `core_freq=250` in `config.txt`), and the `.hcd` patchram blob is
// read off the SD card. In a `bt` directory on the boot partition, under an
// 8.3 name:
//   BT.HCD -- Broadcom's BCM43430A1.hcd patchram blob
//
// Get BCM43430A1.hcd (the original Pi 3B's BCM43438; not the 3B+/Zero 2 W's
// BCM4345C0.hcd) from RPi-Distro/bluez-firmware at
// https://github.com/RPi-Distro/bluez-firmware/blob/master/broadcom/BCM43430A1.hcd
// or copy it from /lib/firmware/brcm/ on any Raspberry Pi OS install.

use core::fmt::Write;
use rpi_hal::bluetooth::gatt::{
    attr, cccd, characteristic, primary_service, Attribute, Server, ATT_MAX_MTU, CHAR_PROP_NOTIFY,
    CHAR_PROP_READ,
};
use rpi_hal::bluetooth::l2cap::{self, Reassembler, CID_ATT};
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
/// How often to push a Battery Level notification while a client is
/// subscribed, in ms.
const NOTIFY_INTERVAL_MS: u32 = 2000;
/// Handle of the Battery Level characteristic *value* — the one notified.
const BATTERY_VALUE_HANDLE: u16 = 0x000c;

// --- The GATT attribute table -----------------------------------------
//
// A flat, handle-ordered list of attributes. Characteristic declaration
// values are laid out by hand as `[properties, value_handle(LE), uuid(LE)]`;
// the value attribute that follows carries the actual data. See the
// `bluetooth::gatt` module docs for the layout.

/// Device Name characteristic declaration: READ, value at 0x0003, UUID
/// 0x2A00.
static DECL_DEVICE_NAME: [u8; 5] = [CHAR_PROP_READ, 0x03, 0x00, 0x00, 0x2a];
/// Appearance characteristic declaration: READ, value at 0x0005, UUID
/// 0x2A01.
static DECL_APPEARANCE: [u8; 5] = [CHAR_PROP_READ, 0x05, 0x00, 0x01, 0x2a];
/// Manufacturer Name String characteristic declaration: READ, value at
/// 0x0009, UUID 0x2A29.
static DECL_MANUFACTURER: [u8; 5] = [CHAR_PROP_READ, 0x09, 0x00, 0x29, 0x2a];
/// Appearance value: `0x0000` (Unknown) — this demo device isn't a
/// particular category. (A keyboard would use `0x03C1`.)
static APPEARANCE_UNKNOWN: [u8; 2] = [0x00, 0x00];
/// GAP service UUID `0x1800`, little-endian.
static SVC_GAP: [u8; 2] = [0x00, 0x18];
/// GATT service UUID `0x1801`, little-endian.
static SVC_GATT: [u8; 2] = [0x01, 0x18];
/// Device Information service UUID `0x180A`, little-endian.
static SVC_DEVICE_INFO: [u8; 2] = [0x0a, 0x18];
/// Battery service UUID `0x180F`, little-endian.
static SVC_BATTERY: [u8; 2] = [0x0f, 0x18];
/// Battery Level characteristic declaration: READ|NOTIFY, value at 0x000C,
/// UUID 0x2A19. NOTIFY makes it subscribable via its CCC descriptor.
static DECL_BATTERY_LEVEL: [u8; 5] = [CHAR_PROP_READ | CHAR_PROP_NOTIFY, 0x0c, 0x00, 0x19, 0x2a];
/// Battery Level initial value (percent) served on a Read; live values are
/// pushed by notification (the read-only table doesn't track the counter).
static BATTERY_INITIAL: [u8; 1] = [100];

/// The attribute table served to connecting clients (sorted by handle).
static ATTRIBUTES: [Attribute; 13] = [
    // GAP service (0x1800).
    primary_service(0x0001, &SVC_GAP),
    characteristic(0x0002, &DECL_DEVICE_NAME),
    attr(0x0003, 0x2a00, ADV_NAME.as_bytes()),
    characteristic(0x0004, &DECL_APPEARANCE),
    attr(0x0005, 0x2a01, &APPEARANCE_UNKNOWN),
    // GATT service (0x1801) — no characteristics.
    primary_service(0x0006, &SVC_GATT),
    // Device Information service (0x180A).
    primary_service(0x0007, &SVC_DEVICE_INFO),
    characteristic(0x0008, &DECL_MANUFACTURER),
    attr(0x0009, 0x2a29, b"rpi-hal"),
    // Battery service (0x180F): a readable + notifiable Battery Level, with
    // a CCC descriptor a client writes to subscribe.
    primary_service(0x000a, &SVC_BATTERY),
    characteristic(0x000b, &DECL_BATTERY_LEVEL),
    attr(0x000c, 0x2a19, &BATTERY_INITIAL),
    cccd(0x000d),
];

/// Logs an established connection.
fn on_connected(console: &mut MiniUart, conn: &Connection) {
    let _ = write!(
        console,
        "connected: handle {:#06x}, role {:?}, peer ",
        conn.handle, conn.role
    );
    write_address(console, &conn.peer_address);
    let _ = writeln!(console, " (type {})", conn.peer_address_type);
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
    if let Err(e) = bt.le_read_buffer_size(&timer) {
        let _ = writeln!(console, "LE read buffer size failed: {e:?}");
        halt();
    }
    let _ = writeln!(console, "controller ready");

    // Advertise connectably so a central can connect.
    if let Err(e) = bt.start_advertising(ADV_NAME, Advertising::Connectable, &timer) {
        let _ = writeln!(console, "start advertising failed: {e:?}");
        halt();
    }
    let _ = writeln!(
        console,
        "advertising as '{ADV_NAME}' with a GATT server -- tap Connect and browse services"
    );

    // Service the connection: reassemble ACL into L2CAP frames, hand ATT
    // frames to the GATT server, send its responses back, and — while a
    // client is subscribed — push a Battery Level notification on a timer.
    let mut reasm = Reassembler::new();
    let mut server = Server::new(&ATTRIBUTES);
    let mut out = [0u8; ATT_MAX_MTU as usize];
    let mut conn_handle: Option<u16> = None;
    let mut battery: u8 = 100;
    let mut next_notify = timer.now_micros() + (NOTIFY_INTERVAL_MS as u64) * 1000;
    loop {
        match bt.poll(&timer, POLL_MS) {
            Ok(Some(Event::Connected(conn))) => {
                on_connected(&mut console, &conn);
                conn_handle = Some(conn.handle);
            }
            Ok(Some(Event::Acl(acl))) => {
                let handle = acl.handle;
                if let Some(pdu) = reasm.feed(&acl) {
                    if pdu.cid != CID_ATT {
                        continue;
                    }
                    let req_opcode = pdu.payload.first().copied().unwrap_or(0);
                    if let Some(n) = server.handle(pdu.payload, &mut out) {
                        let rsp_opcode = out[0];
                        match l2cap::send(&mut bt, handle, CID_ATT, &out[..n]) {
                            Ok(()) => {
                                let _ = writeln!(
                                    console,
                                    "  ATT req {req_opcode:#04x} -> rsp {rsp_opcode:#04x} ({n} bytes)"
                                );
                            }
                            Err(e) => {
                                let _ = writeln!(console, "  ATT response send failed: {e:?}");
                            }
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
                battery = 100;
                reasm.reset();
                server.reset();
                if let Err(e) = bt.start_advertising(ADV_NAME, Advertising::Connectable, &timer) {
                    let _ = writeln!(console, "re-advertise failed: {e:?}");
                    halt();
                }
            }
            // Other events (LTK request, encryption change) don't arise
            // without pairing, which this GATT example doesn't do.
            Ok(Some(_)) => {}
            Ok(None) => {} // quiet window; keep polling
            Err(e) => {
                let _ = writeln!(console, "poll error: {e:?}");
                halt();
            }
        }

        // Time to push a notification? `notification` returns None unless the
        // client subscribed by writing the Battery Level CCC descriptor.
        if let Some(handle) = conn_handle {
            if timer.now_micros() >= next_notify {
                next_notify = timer.now_micros() + (NOTIFY_INTERVAL_MS as u64) * 1000;
                if let Some(n) = server.notification(BATTERY_VALUE_HANDLE, &[battery], &mut out) {
                    match l2cap::send(&mut bt, handle, CID_ATT, &out[..n]) {
                        Ok(()) => {
                            let _ = writeln!(console, "  -> notify Battery Level = {battery}%");
                            battery = if battery == 0 { 100 } else { battery - 1 };
                        }
                        Err(e) => {
                            let _ = writeln!(console, "  notify send failed: {e:?}");
                        }
                    }
                }
            }
        }
    }
}
