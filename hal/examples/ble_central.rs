#![no_std]
#![no_main]

// BLE central + GATT client (Pi 3 only): brings the on-board BCM43438
// Bluetooth controller up, then acts as a *central* -- the inverse of
// `ble_gatt.rs`, which is a peripheral. It scans for a peripheral by name,
// connects to it (`LE_Create_Connection`), walks its GATT database
// (services -> characteristics -> descriptors), subscribes to every
// notifiable characteristic by writing its CCC descriptor, and then prints
// each Handle Value Notification the peripheral pushes.
//
// This exercises the whole central/GATT-client path -- connect out, discover,
// subscribe, receive -- which is the same path a host uses to talk to a BLE
// HID device (a mouse, a gamepad). It runs *unencrypted*, so the ideal test
// peer is one you control: the nRF Connect app in peripheral mode (see below),
// where you tap a button to fire a notification and watch the bytes land here.
// A real HID device won't notify until the link is encrypted (pairing, a
// later step) -- this example stops at the subscribe/notify plumbing.
//
// --- Setting up the nRF Connect test peer (Android) -------------------
// 1. In nRF Connect, open the "Configure GATT server" screen and add a
//    service with one characteristic that has the NOTIFY property and a
//    Client Characteristic Configuration descriptor. (The "Nordic UART"
//    template works: its TX characteristic notifies.)
// 2. Open the "Advertiser" screen, make a new configuration whose advertising
//    data includes a Complete Local Name of `rpi-central` (matching
//    TARGET_NAME below) and is Connectable, and start advertising it.
// 3. Boot this example. It finds `rpi-central`, connects, and subscribes.
// 4. Back in nRF Connect's GATT server, change the notifying characteristic's
//    value and Send a notification -- its bytes print here.
//
// Setup mirrors `ble_gatt.rs`: the console is the mini UART (GPIO14/15,
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
    Uuid, CHAR_PROP_NOTIFY, CHAR_PROP_READ, CHAR_PROP_WRITE, CHAR_PROP_WRITE_NO_RESPONSE,
    UUID_CCC_DESCRIPTOR,
};
use rpi_hal::bluetooth::gatt_client::{
    Characteristic, Client, Descriptor, Service, CCC_INDICATE, CCC_NOTIFY,
};
use rpi_hal::bluetooth::{Bluetooth, Event};
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

/// The peripheral's Complete Local Name to scan for and connect to.
const TARGET_NAME: &str = "Pixel 10 Pro XL";
/// How long to wait for a matching advertiser before giving up, in ms.
const SCAN_TIMEOUT_MS: u32 = 30_000;
/// How long each scan report wait blocks before looping, in ms.
const SCAN_REPORT_MS: u32 = 1_000;
/// How long to wait for the `LE Connection Complete` after initiating, in ms.
const CONNECT_TIMEOUT_MS: u32 = 15_000;
/// How long each steady-state notification poll blocks, in ms.
const NOTIFY_POLL_MS: u32 = 1_000;
/// Characteristic property bit `Indicate` (`0x20`) — not defined in
/// `bluetooth::gatt`, so named here for the subscribe check.
const CHAR_PROP_INDICATE: u8 = 0x20;

/// Max primary services recorded during discovery.
const MAX_SERVICES: usize = 12;
/// Max characteristics recorded per service during discovery.
const MAX_CHARS: usize = 24;
/// Max descriptors recorded per characteristic during discovery.
const MAX_DESCS: usize = 12;

/// Prints a UUID: a 16-bit one as `0xXXXX`, a 128-bit one as the usual
/// MSB-first hex string (its stored bytes are little-endian, so printed in
/// reverse).
fn write_uuid(console: &mut MiniUart, uuid: &Uuid) {
    match uuid {
        Uuid::Bit16(v) => {
            let _ = write!(console, "{v:#06x}");
        }
        Uuid::Bit128(bytes) => {
            let _ = write!(console, "0x");
            for byte in bytes.iter().rev() {
                let _ = write!(console, "{byte:02X}");
            }
        }
    }
}

/// Renders a characteristic's property bits as short flags, e.g. `R W N`.
fn write_properties(console: &mut MiniUart, props: u8) {
    let flags = [
        (CHAR_PROP_READ, 'R'),
        (CHAR_PROP_WRITE, 'W'),
        (CHAR_PROP_WRITE_NO_RESPONSE, 'w'),
        (CHAR_PROP_NOTIFY, 'N'),
        (CHAR_PROP_INDICATE, 'I'),
    ];
    let mut first = true;
    for (bit, ch) in flags {
        if props & bit != 0 {
            if !first {
                let _ = write!(console, " ");
            }
            let _ = write!(console, "{ch}");
            first = false;
        }
    }
    if first {
        let _ = write!(console, "-");
    }
}

/// Scans for a connectable peripheral advertising [`TARGET_NAME`], returning
/// its `(address_type, address)` — or `None` if none appears within the
/// timeout. Prints every named device it hears along the way.
fn scan_for_target(
    bt: &mut Bluetooth,
    console: &mut MiniUart,
    timer: &Timer,
) -> Option<(u8, [u8; 6])> {
    if let Err(e) = bt.start_scan(timer) {
        let _ = writeln!(console, "start scan failed: {e:?}");
        return None;
    }
    let deadline = timer.now_micros() + (SCAN_TIMEOUT_MS as u64) * 1000;
    let found = loop {
        match bt.next_advertising_report(timer, SCAN_REPORT_MS) {
            Ok(Some(report)) => {
                if let Some(name) = report.name() {
                    let _ = write!(console, "  saw '{name}' at ");
                    write_address(console, &report.address);
                    let _ = writeln!(console, " ({} dBm)", report.rssi);
                    if name == TARGET_NAME {
                        break Some((report.address_type, report.address));
                    }
                }
            }
            Ok(None) => {}
            Err(e) => {
                let _ = writeln!(console, "scan error: {e:?}");
                break None;
            }
        }
        if timer.now_micros() >= deadline {
            break None;
        }
    };
    let _ = bt.stop_scan(timer);
    found
}

/// Discovers `service`'s characteristics, prints them, and subscribes to each
/// notifiable one by writing its CCC descriptor. Errors are logged, not fatal.
fn explore_service(
    bt: &mut Bluetooth,
    client: &mut Client,
    console: &mut MiniUart,
    service: &Service,
    timer: &Timer,
) {
    let mut chars = [Characteristic {
        decl_handle: 0,
        properties: 0,
        value_handle: 0,
        uuid: Uuid::Bit16(0),
    }; MAX_CHARS];
    let count = match client.discover_characteristics(
        bt,
        service.start_handle,
        service.end_handle,
        &mut chars,
        timer,
    ) {
        Ok(n) => n,
        Err(e) => {
            let _ = writeln!(console, "  characteristic discovery failed: {e:?}");
            return;
        }
    };

    for i in 0..count {
        let c = &chars[i];
        let _ = write!(console, "    char handle {:#06x} uuid ", c.value_handle);
        write_uuid(console, &c.uuid);
        let _ = write!(console, " props ");
        write_properties(console, c.properties);
        let _ = writeln!(console);

        if c.properties & (CHAR_PROP_NOTIFY | CHAR_PROP_INDICATE) == 0 {
            continue;
        }

        // Descriptors live between this characteristic's value handle and the
        // next characteristic declaration (or the end of the service).
        let range_start = c.value_handle.saturating_add(1);
        let range_end = if i + 1 < count {
            chars[i + 1].decl_handle.saturating_sub(1)
        } else {
            service.end_handle
        };
        let mut descs = [Descriptor {
            handle: 0,
            uuid: Uuid::Bit16(0),
        }; MAX_DESCS];
        let dcount =
            match client.discover_descriptors(bt, range_start, range_end, &mut descs, timer) {
                Ok(n) => n,
                Err(e) => {
                    let _ = writeln!(console, "      descriptor discovery failed: {e:?}");
                    continue;
                }
            };
        let ccc = descs[..dcount]
            .iter()
            .find(|d| d.uuid == Uuid::Bit16(UUID_CCC_DESCRIPTOR));
        // Enable whichever the characteristic actually supports: notify if it
        // has it, else indicate (e.g. the standard Service Changed).
        let bits = if c.properties & CHAR_PROP_NOTIFY != 0 {
            CCC_NOTIFY
        } else {
            CCC_INDICATE
        };
        match ccc {
            Some(d) => match client.subscribe(bt, d.handle, bits, timer) {
                Ok(()) => {
                    let _ = writeln!(console, "      subscribed (CCC handle {:#06x})", d.handle);
                }
                Err(e) => {
                    let _ = writeln!(console, "      subscribe failed: {e:?}");
                }
            },
            None => {
                let _ = writeln!(console, "      no CCC descriptor found");
            }
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
    if let Err(e) = bt.le_read_buffer_size(&timer) {
        let _ = writeln!(console, "LE read buffer size failed: {e:?}");
        halt();
    }
    let _ = writeln!(console, "controller ready");

    // 1. Scan for the target peripheral by name.
    let _ = writeln!(console, "scanning for '{TARGET_NAME}'...");
    let (addr_type, addr) = match scan_for_target(&mut bt, &mut console, &timer) {
        Some(target) => target,
        None => {
            let _ = writeln!(console, "'{TARGET_NAME}' not found -- giving up");
            halt();
        }
    };
    let _ = write!(console, "found target at ");
    write_address(&mut console, &addr);
    let _ = writeln!(console, " (type {addr_type}) -- connecting...");

    // 2. Initiate the connection and wait for the link to come up.
    if let Err(e) = bt.connect(addr_type, &addr, &timer) {
        let _ = writeln!(console, "connect command failed: {e:?}");
        halt();
    }
    let connect_deadline = timer.now_micros() + (CONNECT_TIMEOUT_MS as u64) * 1000;
    let conn = loop {
        match bt.poll(&timer, NOTIFY_POLL_MS) {
            Ok(Some(Event::Connected(c))) => break Some(c),
            Ok(Some(_)) | Ok(None) => {}
            Err(e) => {
                let _ = writeln!(console, "poll error while connecting: {e:?}");
                halt();
            }
        }
        if timer.now_micros() >= connect_deadline {
            break None;
        }
    };
    let conn = match conn {
        Some(c) => c,
        None => {
            let _ = bt.connect_cancel(&timer);
            let _ = writeln!(console, "connection timed out -- giving up");
            halt();
        }
    };
    let _ = write!(
        console,
        "connected: handle {:#06x}, role {:?}, peer ",
        conn.handle, conn.role
    );
    write_address(&mut console, &conn.peer_address);
    let _ = writeln!(console);

    // 3. Discover the GATT database and subscribe to notifiable characteristics.
    let mut client = Client::new(conn.handle);
    match client.exchange_mtu(&mut bt, &timer) {
        Ok(mtu) => {
            let _ = writeln!(console, "MTU negotiated: {mtu}");
        }
        Err(e) => {
            // A server without MTU exchange keeps the 23-byte default; not fatal.
            let _ = writeln!(console, "MTU exchange declined ({e:?}) -- using default");
        }
    }

    let mut services = [Service {
        start_handle: 0,
        end_handle: 0,
        uuid: Uuid::Bit16(0),
    }; MAX_SERVICES];
    let svc_count = match client.discover_primary_services(&mut bt, &mut services, &timer) {
        Ok(n) => n,
        Err(e) => {
            let _ = writeln!(console, "service discovery failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(console, "discovered {svc_count} primary service(s):");
    for svc in &services[..svc_count] {
        let _ = write!(
            console,
            "  service {:#06x}-{:#06x} uuid ",
            svc.start_handle, svc.end_handle
        );
        write_uuid(&mut console, &svc.uuid);
        let _ = writeln!(console);
        explore_service(&mut bt, &mut client, &mut console, svc, &timer);
    }

    // 4. Steady state: print notifications the peripheral pushes.
    let _ = writeln!(
        console,
        "listening for notifications (change & Send a value on the peer)..."
    );
    let mut value = [0u8; rpi_hal::bluetooth::gatt::ATT_MAX_MTU as usize];
    loop {
        match bt.poll(&timer, NOTIFY_POLL_MS) {
            Ok(Some(Event::Acl(acl))) => {
                if let Some(note) = client.feed(&acl, &mut value) {
                    let _ = write!(console, "notify handle {:#06x}:", note.value_handle);
                    for byte in &value[..note.len] {
                        let _ = write!(console, " {byte:02X}");
                    }
                    let _ = writeln!(console);
                }
            }
            Ok(Some(Event::Disconnected { handle, reason })) => {
                let _ = writeln!(
                    console,
                    "disconnected: handle {handle:#06x}, reason {reason:#04x}"
                );
                halt();
            }
            Ok(Some(_)) | Ok(None) => {}
            Err(e) => {
                let _ = writeln!(console, "poll error: {e:?}");
                halt();
            }
        }
    }
}
