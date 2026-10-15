#![no_std]
#![no_main]

// BLE HID mouse host (Pi 3 only): the step past `ble_central.rs`. It brings
// the on-board BCM43438 controller up, scans for a device advertising the HID
// service (0x1812), connects as central, **pairs and encrypts the link**
// (LE Legacy Just Works, as the SMP *initiator*), then discovers the HID
// service, subscribes to its input-report notifications, and prints each
// report's raw bytes as you move/click the mouse.
//
// The pairing step is the whole point: in `ble_central.rs` every attempt to
// subscribe to a real device's characteristic came back `Insufficient
// Encryption` -- a HID-over-GATT peripheral won't emit input until the link is
// encrypted. `bluetooth::smp::Initiator` drives the Pairing
// Request/Confirm/Random exchange from the central side and derives the Short
// Term Key; `Bluetooth::le_start_encryption` then turns encryption on. After
// the `EncryptionChange` event, the same GATT-client discovery/subscribe path
// from `ble_central.rs` works, and input reports arrive as notifications.
//
// Report *interpretation* (turning the raw bytes into buttons + dx/dy/wheel)
// isn't done here -- it needs the device's HID Report Map. This prints the raw
// report so that layout can be read off real data first.
//
// --- Using it ---------------------------------------------------------
// Put the mouse in **pairing mode** (e.g. an MX Master: hold the Bluetooth
// channel button until its LED blinks fast) so it advertises connectably and
// accepts a new bond. Boot this example; it connects to the first HID
// advertiser it sees. Move the mouse and watch the report bytes print.
//
// Setup mirrors `ble_central.rs`: the console is the mini UART (GPIO14/15,
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
use rpi_hal::bluetooth::gatt::{Uuid, CHAR_PROP_NOTIFY, UUID_CCC_DESCRIPTOR};
use rpi_hal::bluetooth::gatt_client::{Characteristic, Client, Descriptor, Service, CCC_NOTIFY};
use rpi_hal::bluetooth::l2cap::{self, Reassembler, CID_SMP};
use rpi_hal::bluetooth::smp::{Action, Initiator};
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

/// The 16-bit HID service UUID advertised by keyboards/mice/gamepads.
const HID_SERVICE_UUID: u16 = 0x1812;
/// AD structure type: incomplete list of 16-bit service UUIDs.
const AD_TYPE_SERVICE_UUIDS_16_INCOMPLETE: u8 = 0x02;
/// AD structure type: complete list of 16-bit service UUIDs.
const AD_TYPE_SERVICE_UUIDS_16_COMPLETE: u8 = 0x03;
/// AD structure type: Appearance (a 16-bit category).
const AD_TYPE_APPEARANCE: u8 = 0x19;
/// The Appearance *category* (top 10 bits) for Human Interface Devices —
/// Generic HID `0x03C0`, Keyboard `0x03C1`, Mouse `0x03C2`, Gamepad `0x03C4`
/// all share category `0x0F`. Many HID devices advertise this even when they
/// don't list the `0x1812` service UUID in the adv packet.
const APPEARANCE_CATEGORY_HID: u16 = 0x0f;
/// Ignore advertisers weaker than this (dBm). A HID device you're actively
/// pairing with is right next to the Pi and reads strong; a weak `[HID]` hit
/// is a distant stranger whose connection can't establish (HCI reason 0x3e).
/// Optionally set the target name below to also require a name match.
const MIN_RSSI_DBM: i8 = -80;
/// If non-empty, only connect to a device whose advertised name contains this
/// substring — set it (e.g. "MX") when several strong HID devices are around.
const TARGET_NAME_SUBSTR: &str = "";
/// How long to scan for a HID advertiser before giving up, in ms.
const SCAN_TIMEOUT_MS: u32 = 30_000;
/// How long each scan report wait blocks before looping, in ms.
const SCAN_REPORT_MS: u32 = 1_000;
/// How long to wait for the `LE Connection Complete`, in ms.
const CONNECT_TIMEOUT_MS: u32 = 15_000;
/// Overall budget for the pairing exchange (request → encrypted), in ms.
const PAIR_TIMEOUT_MS: u32 = 20_000;
/// Grace window after encryption to collect the peer's bond keys, in ms.
const BOND_DRAIN_MS: u32 = 1_500;
/// How long each poll blocks in the pairing and steady-state loops, in ms.
const POLL_MS: u32 = 500;

/// Max primary services recorded during discovery.
const MAX_SERVICES: usize = 16;
/// Max characteristics recorded in the HID service during discovery.
const MAX_CHARS: usize = 24;
/// Max descriptors recorded per characteristic during discovery.
const MAX_DESCS: usize = 8;

/// Returns the value bytes of the first AD structure of type `ad_type` in the
/// advertising data, or `None`. Advertising data is a sequence of
/// `[length, type, value…]` structures where `length` counts the type byte
/// plus the value.
fn find_ad_value(data: &[u8], ad_type: u8) -> Option<&[u8]> {
    let mut i = 0;
    while i < data.len() {
        let len = data[i] as usize;
        if len == 0 {
            break;
        }
        let end = i + 1 + len;
        if end > data.len() {
            break;
        }
        if data[i + 1] == ad_type {
            return Some(&data[i + 2..end]);
        }
        i = end;
    }
    None
}

/// Returns `true` if the advertising data lists the 16-bit service `uuid16` in
/// a complete or incomplete 16-bit-service-UUIDs AD structure.
fn adv_has_service_16(data: &[u8], uuid16: u16) -> bool {
    let want = uuid16.to_le_bytes();
    for ad_type in [
        AD_TYPE_SERVICE_UUIDS_16_INCOMPLETE,
        AD_TYPE_SERVICE_UUIDS_16_COMPLETE,
    ] {
        if let Some(uuids) = find_ad_value(data, ad_type) {
            let mut j = 0;
            while j + 2 <= uuids.len() {
                if uuids[j..j + 2] == want {
                    return true;
                }
                j += 2;
            }
        }
    }
    false
}

/// Returns `true` if the advertising data's Appearance names a Human Interface
/// Device (category `0x0F`) — a keyboard, mouse, or gamepad.
fn adv_appearance_is_hid(data: &[u8]) -> bool {
    match find_ad_value(data, AD_TYPE_APPEARANCE) {
        Some(v) if v.len() >= 2 => {
            (u16::from_le_bytes([v[0], v[1]]) >> 6) == APPEARANCE_CATEGORY_HID
        }
        _ => false,
    }
}

/// Scans for a nearby HID device to pair with, returning its
/// `(address_type, address)` — or `None` on timeout. A device qualifies if it
/// looks like a HID (lists the `0x1812` service *or* has a HID Appearance),
/// reads at least [`MIN_RSSI_DBM`] (so a distant stranger isn't picked — that
/// causes the `0x3e` connect-failed drop), and, if [`TARGET_NAME_SUBSTR`] is
/// set, whose name matches. Every named/HID advertiser is printed with why it
/// was or wasn't chosen.
fn scan_for_hid(
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
                let is_hid = adv_has_service_16(report.data(), HID_SERVICE_UUID)
                    || adv_appearance_is_hid(report.data());
                let strong = report.rssi >= MIN_RSSI_DBM;
                let name_ok = TARGET_NAME_SUBSTR.is_empty()
                    || report
                        .name()
                        .is_some_and(|n| n.contains(TARGET_NAME_SUBSTR));
                if is_hid || report.name().is_some() {
                    let _ = write!(console, "  saw ");
                    write_address(console, &report.address);
                    if let Some(name) = report.name() {
                        let _ = write!(console, " '{name}'");
                    }
                    let tag = if !is_hid {
                        ""
                    } else if !strong {
                        " [HID, too weak]"
                    } else if !name_ok {
                        " [HID, name mismatch]"
                    } else {
                        " [HID]"
                    };
                    let _ = writeln!(console, "{tag} ({} dBm)", report.rssi);
                }
                if is_hid && strong && name_ok {
                    break Some((report.address_type, report.address));
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

/// Runs the SMP initiator exchange to an encrypted link. Returns `true` once
/// the `EncryptionChange` confirms encryption (after a short grace window to
/// collect bond keys), `false` on failure/disconnect/timeout.
fn pair_and_encrypt(
    bt: &mut Bluetooth,
    console: &mut MiniUart,
    initiator: &mut Initiator,
    handle: u16,
    timer: &Timer,
) -> bool {
    let mut reasm = Reassembler::new();
    let mut out = [0u8; 32];

    // Open the exchange with our Pairing Request.
    let n = initiator.start_pairing(&mut out);
    if let Err(e) = l2cap::send(bt, handle, CID_SMP, &out[..n]) {
        let _ = writeln!(console, "send pairing request failed: {e:?}");
        return false;
    }

    let overall_deadline = timer.now_micros() + (PAIR_TIMEOUT_MS as u64) * 1000;
    let mut encrypted = false;
    // Once encrypted, keep reading briefly so the peer's key-distribution
    // PDUs (bonding) are captured before returning.
    let mut drain_deadline = 0u64;
    loop {
        let now = timer.now_micros();
        if now >= overall_deadline {
            let _ = writeln!(console, "pairing timed out");
            return false;
        }
        if encrypted && (initiator.bond().is_some() || now >= drain_deadline) {
            return true;
        }

        match bt.poll(timer, POLL_MS) {
            Ok(Some(Event::Acl(acl))) => {
                if acl.handle != handle {
                    continue;
                }
                let Some(pdu) = reasm.feed(&acl) else {
                    continue;
                };
                if pdu.cid != CID_SMP {
                    continue;
                }
                match initiator.handle(bt, timer, pdu.payload, &mut out) {
                    Ok(Action::Send(n)) => {
                        if let Err(e) = l2cap::send(bt, handle, CID_SMP, &out[..n]) {
                            let _ = writeln!(console, "SMP send failed: {e:?}");
                            return false;
                        }
                    }
                    Ok(Action::StartEncryption) => {
                        let _ = writeln!(console, "pairing verified -- starting encryption");
                        if let Err(e) = bt.le_start_encryption(
                            handle,
                            [0u8; 8],
                            0,
                            &initiator.short_term_key(),
                            timer,
                        ) {
                            let _ = writeln!(console, "start encryption failed: {e:?}");
                            return false;
                        }
                    }
                    Ok(Action::Failed(n)) => {
                        let _ = l2cap::send(bt, handle, CID_SMP, &out[..n]);
                        let _ = writeln!(console, "pairing failed (confirm/protocol)");
                        return false;
                    }
                    Ok(Action::Idle) => {}
                    Err(e) => {
                        let _ = writeln!(console, "SMP error: {e:?}");
                        return false;
                    }
                }
            }
            Ok(Some(Event::EncryptionChange { enabled: true, .. })) => {
                let _ = writeln!(console, "link encrypted");
                encrypted = true;
                drain_deadline = timer.now_micros() + (BOND_DRAIN_MS as u64) * 1000;
            }
            Ok(Some(Event::EncryptionChange { enabled: false, .. })) => {
                let _ = writeln!(console, "encryption was disabled -- aborting");
                return false;
            }
            Ok(Some(Event::Disconnected { reason, .. })) => {
                let _ = writeln!(
                    console,
                    "disconnected during pairing (reason {reason:#04x})"
                );
                return false;
            }
            Ok(Some(_)) | Ok(None) => {}
            Err(e) => {
                let _ = writeln!(console, "poll error during pairing: {e:?}");
                return false;
            }
        }
    }
}

/// Discovers the HID service's characteristics and subscribes to every
/// notifiable one (the input reports), printing what it finds.
fn subscribe_hid_reports(
    bt: &mut Bluetooth,
    client: &mut Client,
    console: &mut MiniUart,
    hid: &Service,
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
        hid.start_handle,
        hid.end_handle,
        &mut chars,
        timer,
    ) {
        Ok(n) => n,
        Err(e) => {
            let _ = writeln!(console, "HID characteristic discovery failed: {e:?}");
            return;
        }
    };

    for i in 0..count {
        let c = &chars[i];
        let _ = write!(console, "  char handle {:#06x} uuid ", c.value_handle);
        write_uuid(console, &c.uuid);
        let _ = writeln!(console, " props {:#04x}", c.properties);

        if c.properties & CHAR_PROP_NOTIFY == 0 {
            continue;
        }
        let range_start = c.value_handle.saturating_add(1);
        let range_end = if i + 1 < count {
            chars[i + 1].decl_handle.saturating_sub(1)
        } else {
            hid.end_handle
        };
        let mut descs = [Descriptor {
            handle: 0,
            uuid: Uuid::Bit16(0),
        }; MAX_DESCS];
        let dcount =
            match client.discover_descriptors(bt, range_start, range_end, &mut descs, timer) {
                Ok(n) => n,
                Err(e) => {
                    let _ = writeln!(console, "    descriptor discovery failed: {e:?}");
                    continue;
                }
            };
        match descs[..dcount]
            .iter()
            .find(|d| d.uuid == Uuid::Bit16(UUID_CCC_DESCRIPTOR))
        {
            Some(d) => match client.subscribe(bt, d.handle, CCC_NOTIFY, timer) {
                Ok(()) => {
                    let _ = writeln!(console, "    subscribed to input reports");
                }
                Err(e) => {
                    let _ = writeln!(console, "    subscribe failed: {e:?}");
                }
            },
            None => {
                let _ = writeln!(console, "    no CCC descriptor");
            }
        }
    }
}

/// Prints a UUID: a 16-bit one as `0xXXXX`, a 128-bit one MSB-first.
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
    if let Err(e) = bt.le_read_buffer_size(&timer) {
        let _ = writeln!(console, "LE read buffer size failed: {e:?}");
        halt();
    }

    // Our own public address — an input to the pairing crypto.
    let own_addr = match bt.read_bd_addr(&timer) {
        Ok(addr) => addr,
        Err(e) => {
            let _ = writeln!(console, "read BD_ADDR failed: {e:?}");
            halt();
        }
    };

    // Build the pairing initiator up front (also runs the SMP crypto
    // self-test against the controller). Bond so the mouse stays paired.
    let mut initiator = match Initiator::new(&mut bt, &timer, true) {
        Ok(i) => i,
        Err(e) => {
            let _ = writeln!(console, "SMP init failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(console, "controller ready");

    // 1. Scan for a HID advertiser.
    let _ = writeln!(
        console,
        "scanning for a HID device (put the mouse in pairing mode)..."
    );
    let (addr_type, addr) = match scan_for_hid(&mut bt, &mut console, &timer) {
        Some(target) => target,
        None => {
            let _ = writeln!(console, "no HID advertiser found -- giving up");
            halt();
        }
    };
    let _ = write!(console, "connecting to ");
    write_address(&mut console, &addr);
    let _ = writeln!(console, " (type {addr_type})...");

    // 2. Connect as central.
    if let Err(e) = bt.connect(addr_type, &addr, &timer) {
        let _ = writeln!(console, "connect command failed: {e:?}");
        halt();
    }
    let connect_deadline = timer.now_micros() + (CONNECT_TIMEOUT_MS as u64) * 1000;
    let conn = loop {
        match bt.poll(&timer, POLL_MS) {
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
            let _ = writeln!(console, "connection timed out");
            halt();
        }
    };
    let _ = writeln!(console, "connected: handle {:#06x}", conn.handle);

    // 3. Pair and encrypt (SMP initiator). own_addr_type 0 = public (the type
    //    `connect` used); the peer's type comes from the connection.
    initiator.begin(own_addr, 0, conn.peer_address, conn.peer_address_type);
    let _ = writeln!(console, "pairing (Just Works)...");
    if !pair_and_encrypt(&mut bt, &mut console, &mut initiator, conn.handle, &timer) {
        halt();
    }
    if initiator.bond().is_some() {
        let _ = writeln!(console, "bonded (LTK stored for this session)");
    }

    // 4. Discover the HID service and subscribe to its input reports.
    let mut client = Client::new(conn.handle);
    if let Err(e) = client.exchange_mtu(&mut bt, &timer) {
        let _ = writeln!(console, "MTU exchange declined ({e:?}) -- using default");
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
    let hid = services[..svc_count]
        .iter()
        .find(|s| s.uuid == Uuid::Bit16(HID_SERVICE_UUID));
    let hid = match hid {
        Some(s) => *s,
        None => {
            let _ = writeln!(
                console,
                "no HID service (0x1812) found in {svc_count} services -- giving up"
            );
            halt();
        }
    };
    let _ = writeln!(
        console,
        "HID service at {:#06x}-{:#06x}:",
        hid.start_handle, hid.end_handle
    );
    subscribe_hid_reports(&mut bt, &mut client, &mut console, &hid, &timer);

    // 5. Steady state: print each input report as it arrives.
    let _ = writeln!(
        console,
        "listening for input reports (move/click the mouse)..."
    );
    let mut value = [0u8; rpi_hal::bluetooth::gatt::ATT_MAX_MTU as usize];
    loop {
        match bt.poll(&timer, POLL_MS) {
            Ok(Some(Event::Acl(acl))) => {
                if let Some(note) = client.feed(&acl, &mut value) {
                    let _ = write!(console, "report {:#06x}:", note.value_handle);
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
