#![no_std]
#![no_main]

// USB keyboard -> BLE keyboard bridge (Pi 3 only): turns a wired USB keyboard
// attached to the Pi into a Bluetooth Low Energy keyboard. This is the whole
// point of the BLE HID stack -- Phase 5b, the real thing behind the
// synthetic-keystroke demo in `ble_hid_keyboard.rs`.
//
// It runs two subsystems in one loop:
//  - the BLE HID-over-GATT peripheral (advertise as a keyboard, pair + bond
//    to an encrypted link, expose the HID service) built up across
//    `ble_l2cap.rs` / `ble_gatt.rs` / `ble_pair.rs` / `ble_hid_keyboard.rs`;
//  - the USB host (`rpi_hal::usb`) enumerating an attached HID boot keyboard.
// Each USB report the keyboard produces is forwarded unchanged as a BLE HID
// Input Report notification -- the USB boot report and the BLE Input Report
// are the same 8-byte layout, so it's a straight hand-off.
//
// Wiring: attach a USB keyboard to the Pi, and pair "rpi-hal-kbd" from the
// host's Bluetooth *settings* (not nRF Connect -- the OS reserves the HID
// service). Then type on the USB keyboard; the characters appear on the host
// as if from a Bluetooth keyboard. The bond is persisted to `bt/BOND.BIN` on
// the SD card next to the firmware, so it survives a reboot: a returning host
// re-encrypts with the stored key without a re-pair.
//
// Console + firmware setup mirror `ble_hid_keyboard.rs`: console on the mini
// UART (GPIO14/15, `core_freq=250` in `config.txt`), and the `.hcd` patchram
// blob read off the SD card at `bt/BT.HCD` (Broadcom's BCM43430A1.hcd; see
// that example's header for where to get it).

use core::fmt::Write;
use core::ops::ControlFlow;
use core::ptr::{addr_of, addr_of_mut};
use embedded_sdmmc::{Mode, VolumeIdx, VolumeManager};
use rpi_hal::bluetooth::gatt::{
    attr, cccd, characteristic, primary_service, Attribute, Server, ATT_MAX_MTU,
};
use rpi_hal::bluetooth::l2cap::{self, Reassembler, CID_ATT, CID_SMP};
use rpi_hal::bluetooth::smp::{Bond, Smp};
use rpi_hal::bluetooth::{Advertising, Bluetooth, Connection, Event};
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::mini_uart::MiniUart;
use rpi_hal::pac;
use rpi_hal::sd::{Sd, SdCard, SdCardError};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::usb;
use rpi_hal::usb::dwc2::Dwc2Host;
use rpi_hal::usb::hid::keyboard::Keyboard;

#[path = "common/mod.rs"]
mod common;
use common::{FixedTime, HCI_BAUD};

/// Persisted bond file, within `BT_DIR` (8.3 name) — 26 bytes: LTK(16) +
/// EDIV(2, little-endian) + Rand(8). Lets the bond survive a reboot so a
/// returning host re-encrypts without a re-pair.
const BOND_FILE: &str = "BOND.BIN";
/// Serialized [`Bond`] size on disk: LTK(16) + EDIV(2) + Rand(8).
const BOND_LEN: usize = 26;
/// The device name (matches the name in [`ADV_DATA`]).
const DEVICE_NAME: &str = "rpi-hal-kbd";
/// How long each [`Bluetooth::poll`] waits in the bridge loop, in ms — also
/// the pacing between USB keyboard polls (interrupt endpoints mustn't be
/// hammered), so kept short for responsive typing.
const BRIDGE_POLL_MS: u32 = 10;
/// HCI address type `Public` — the address we advertise/pair with.
const ADDR_TYPE_PUBLIC: u8 = 0x00;
/// Handle of the HID Input Report *value* — the one notified with key data.
const INPUT_REPORT_HANDLE: u16 = 0x003a;

/// The HID Report Map: a standard boot-compatible keyboard descriptor with
/// an 8-byte input report (1 modifier byte, 1 reserved byte, 6 key codes),
/// no LED output — the same layout a USB boot keyboard produces.
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
/// service UUID (0x1812), and the complete local name.
static ADV_DATA: [u8; 24] = [
    0x02, 0x01, 0x06, // Flags: LE General Discoverable, BR/EDR not supported
    0x03, 0x19, 0xc1, 0x03, // Appearance = Keyboard (0x03C1)
    0x03, 0x03, 0x12, 0x18, // Complete list of 16-bit Service UUIDs: HID (0x1812)
    0x0c, 0x09, b'r', b'p', b'i', b'-', b'h', b'a', b'l', b'-', b'k', b'b', b'd', // Name
];

/// The HOGP attribute table (handle-ordered), identical to
/// `ble_hid_keyboard.rs`. Characteristic declaration values are
/// `[properties, value_handle(LE), uuid(LE)]`; see `bluetooth::gatt`.
static ATTRIBUTES: [Attribute; 26] = [
    // GAP service (0x1800): Device Name + Appearance (Keyboard).
    primary_service(0x0001, &[0x00, 0x18]),
    characteristic(0x0002, &[0x02, 0x03, 0x00, 0x00, 0x2a]),
    attr(0x0003, 0x2a00, DEVICE_NAME.as_bytes()),
    characteristic(0x0004, &[0x02, 0x05, 0x00, 0x01, 0x2a]),
    attr(0x0005, 0x2a01, &[0xc1, 0x03]), // Appearance = Keyboard
    // GATT service (0x1801).
    primary_service(0x0006, &[0x01, 0x18]),
    // Device Information service (0x180A): PnP ID.
    primary_service(0x0010, &[0x0a, 0x18]),
    characteristic(0x0011, &[0x02, 0x12, 0x00, 0x50, 0x2a]),
    attr(0x0012, 0x2a50, &[0x02, 0x6b, 0x1d, 0x46, 0x02, 0x01, 0x00]),
    // Battery service (0x180F): a readable Battery Level.
    primary_service(0x0020, &[0x0f, 0x18]),
    characteristic(0x0021, &[0x12, 0x22, 0x00, 0x19, 0x2a]),
    attr(0x0022, 0x2a19, &[100]),
    cccd(0x0023),
    // HID service (0x1812).
    primary_service(0x0030, &[0x12, 0x18]),
    characteristic(0x0031, &[0x02, 0x32, 0x00, 0x4a, 0x2a]),
    attr(0x0032, 0x2a4a, &[0x11, 0x01, 0x00, 0x03]),
    characteristic(0x0033, &[0x02, 0x34, 0x00, 0x4b, 0x2a]),
    attr(0x0034, 0x2a4b, &REPORT_MAP),
    characteristic(0x0035, &[0x04, 0x36, 0x00, 0x4c, 0x2a]),
    attr(0x0036, 0x2a4c, &[0x00]),
    characteristic(0x0037, &[0x06, 0x38, 0x00, 0x4e, 0x2a]),
    attr(0x0038, 0x2a4e, &[0x01]),
    characteristic(0x0039, &[0x12, 0x3a, 0x00, 0x4d, 0x2a]),
    attr(0x003a, 0x2a4d, &[0, 0, 0, 0, 0, 0, 0, 0]),
    cccd(0x003b),
    attr(0x003c, 0x2908, &[0x00, 0x01]),
];

/// Buffer for the `.hcd` blob (the 43438's `BCM43430A1.hcd` is ~40KB);
/// zeroed BSS.
static mut HCD_BUF: [u8; 64 * 1024] = [0; 64 * 1024];

/// The `VolumeManager` type over the SD card, kept alive for the whole
/// program so the bond file can be read at boot and written after pairing.
type Volumes<'t> = VolumeManager<SdCard<'t>, FixedTime>;

/// Reads the firmware blob from `bt/BT.HCD` into the static buffer,
/// returning its length.
fn read_firmware(volumes: &Volumes) -> Result<usize, embedded_sdmmc::Error<SdCardError>> {
    let volume = volumes.open_volume(VolumeIdx(0))?;
    let root = volume.open_root_dir()?;
    let bt = root.open_dir(common::BT_DIR)?;

    // Safety: single-threaded bare-metal; touched only here, then read-only.
    let file = bt.open_file_in_dir(common::FIRMWARE_FILE, Mode::ReadOnly)?;
    let buf = unsafe { &mut *addr_of_mut!(HCD_BUF) };
    let mut total = 0;
    while !file.is_eof() && total < buf.len() {
        let n = file.read(&mut buf[total..])?;
        if n == 0 {
            break;
        }
        total += n;
    }
    file.close()?;
    Ok(total)
}

/// Reads a persisted bond from `bt/BOND.BIN`, or `None` if it's absent or
/// short (first boot, or never paired). Any SD error is treated as "no bond".
fn read_bond(volumes: &Volumes) -> Option<Bond> {
    let volume = volumes.open_volume(VolumeIdx(0)).ok()?;
    let root = volume.open_root_dir().ok()?;
    let bt = root.open_dir(common::BT_DIR).ok()?;
    let file = bt.open_file_in_dir(BOND_FILE, Mode::ReadOnly).ok()?;

    let mut buf = [0u8; BOND_LEN];
    let mut total = 0;
    while total < buf.len() {
        let n = file.read(&mut buf[total..]).ok()?;
        if n == 0 {
            break;
        }
        total += n;
    }
    let _ = file.close();
    if total < BOND_LEN {
        return None;
    }

    let mut ltk = [0u8; 16];
    ltk.copy_from_slice(&buf[0..16]);
    let mut rand = [0u8; 8];
    rand.copy_from_slice(&buf[18..26]);
    Some(Bond {
        ltk,
        ediv: u16::from_le_bytes([buf[16], buf[17]]),
        rand,
    })
}

/// Writes `bond` to `bt/BOND.BIN` (creating/truncating), so it survives a
/// reboot.
fn write_bond(volumes: &Volumes, bond: &Bond) -> Result<(), embedded_sdmmc::Error<SdCardError>> {
    let volume = volumes.open_volume(VolumeIdx(0))?;
    let root = volume.open_root_dir()?;
    let bt = root.open_dir(common::BT_DIR)?;
    let file = bt.open_file_in_dir(BOND_FILE, Mode::ReadWriteCreateOrTruncate)?;

    let mut buf = [0u8; BOND_LEN];
    buf[0..16].copy_from_slice(&bond.ltk);
    buf[16..18].copy_from_slice(&bond.ediv.to_le_bytes());
    buf[18..26].copy_from_slice(&bond.rand);
    file.write(&buf)?;
    file.close()
}

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

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut console = MiniUart::init(&peripherals.GPIO, &peripherals.AUX, peripherals.UART1);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // Bring up the SD card and keep it (the `VolumeManager`) alive for the
    // whole program: it holds EMMC, and we read the firmware + bond at boot
    // and rewrite the bond after pairing.
    let _ = writeln!(console, "reading from SD card...");
    let sd = match Sd::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, &timer) {
        Ok(sd) => sd,
        Err(e) => {
            let _ = writeln!(console, "SD init failed: {e:?}");
            halt();
        }
    };
    let volumes = VolumeManager::new(SdCard::new(sd, &timer), FixedTime);
    let hcd_len = match read_firmware(&volumes) {
        Ok(len) => len,
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
    // A saved bond (if any) lets a returning host reconnect without pairing.
    let saved_bond = read_bond(&volumes);

    // Bring up the Bluetooth controller (but don't advertise yet).
    let _ = writeln!(console, "bringing up Bluetooth controller over HCI...");
    let hci = Uart::init_bluetooth(&peripherals.GPIO, peripherals.UART0);
    let mut bt = Bluetooth::new(hci, &mut mailbox, &timer);
    let hcd = &unsafe { &*addr_of!(HCD_BUF) }[..hcd_len];
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
    let mut smp = match Smp::new(&mut bt, &timer, true) {
        Ok(smp) => smp,
        Err(e) => {
            let _ = writeln!(console, "SMP init (crypto self-test) failed: {e:?}");
            halt();
        }
    };
    if let Some(bond) = saved_bond {
        smp.restore_bond(&bond);
        let _ = writeln!(console, "restored bond from {}/{BOND_FILE}", common::BT_DIR);
    }

    // Bring up the USB host and wait for the on-board hub.
    let _ = writeln!(console, "bringing up USB host...");
    if !usb::power_on(&mut mailbox) {
        let _ = writeln!(console, "USB power-on failed");
        halt();
    }
    let mut dwc2 = Dwc2Host::init(
        peripherals.USB_OTG_GLOBAL,
        peripherals.USB_OTG_HOST,
        peripherals.USB_OTG_PWRCLK,
        &timer,
    );
    let _ = writeln!(console, "waiting for a USB keyboard...");
    while !dwc2.port_connected() {
        timer.delay_ms(100);
    }

    // Persists a freshly-established bond to SD. Borrows the SD volumes
    // (shared); returns whether the write succeeded so the bridge can log it.
    let mut persist = |bond: &Bond| write_bond(&volumes, bond).is_ok();

    // Enumerate; the first HID keyboard found runs the bridge forever.
    let result = usb::enumerate(&mut dwc2, &timer, |dwc2, timer, device| {
        let mut keyboard = match Keyboard::from_device(dwc2, timer, device) {
            Ok(Some(keyboard)) => keyboard,
            Ok(None) => return ControlFlow::Continue(()),
            Err(e) => {
                let _ = writeln!(console, "port {}: HID setup failed: {e:?}", device.port);
                return ControlFlow::Continue(());
            }
        };
        let _ = writeln!(
            console,
            "USB keyboard on port {} -- ready to bridge",
            device.port
        );
        run_bridge(
            &mut console,
            &mut bt,
            &mut smp,
            own_addr,
            dwc2,
            timer,
            &mut keyboard,
            &mut persist,
        )
    });

    match result {
        Ok(()) => {
            let _ = writeln!(console, "no USB keyboard found");
        }
        Err(e) => {
            let _ = writeln!(console, "USB enumeration failed: {e:?}");
        }
    }
    halt();
}

/// Runs the bridge forever: advertises as a BLE keyboard, services pairing
/// and GATT, and forwards each USB keyboard report as a BLE HID Input Report
/// notification once a host is connected, encrypted, and subscribed.
#[allow(clippy::too_many_arguments)]
fn run_bridge(
    console: &mut MiniUart,
    bt: &mut Bluetooth,
    smp: &mut Smp,
    own_addr: [u8; 6],
    dwc2: &mut Dwc2Host,
    timer: &Timer,
    keyboard: &mut Keyboard,
    persist: &mut dyn FnMut(&Bond) -> bool,
) -> ! {
    // Advertise now that everything's ready to service a connection.
    if let Err(e) = bt.start_advertising_raw(&ADV_DATA, Advertising::Connectable, timer) {
        let _ = writeln!(console, "start advertising failed: {e:?}");
        halt();
    }
    let _ = writeln!(
        console,
        "advertising as '{DEVICE_NAME}' -- pair from the host's Bluetooth settings, then type"
    );

    let mut reasm = Reassembler::new();
    let mut server = Server::new(&ATTRIBUTES);
    let mut att_out = [0u8; ATT_MAX_MTU as usize];
    let mut smp_out = [0u8; 32];
    let mut conn_handle: Option<u16> = None;
    let mut encrypted = false;

    loop {
        // Service the BLE side with a short poll (also paces USB polling).
        match bt.poll(timer, BRIDGE_POLL_MS) {
            Ok(Some(Event::Connected(conn))) => {
                on_connected(console, &conn);
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
                                let _ = l2cap::send(bt, handle, CID_ATT, &att_out[..n]);
                            }
                        }
                        CID_SMP => match smp.handle(bt, timer, pdu.payload, &mut smp_out) {
                            Ok(Some(n)) => {
                                let _ = l2cap::send(bt, handle, CID_SMP, &smp_out[..n]);
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
                    let _ = bt.le_ltk_request_reply(handle, &key, timer);
                } else {
                    let _ = bt.le_ltk_request_negative_reply(handle, timer);
                }
            }
            Ok(Some(Event::EncryptionChange { handle, enabled })) => {
                encrypted = enabled;
                let _ = writeln!(
                    console,
                    "link {}",
                    if enabled { "ENCRYPTED" } else { "plain" }
                );
                if enabled {
                    match smp.distribute_keys(bt, timer) {
                        Ok(Some(keys)) => {
                            let _ = l2cap::send(bt, handle, CID_SMP, &keys.encryption_information);
                            let _ = l2cap::send(bt, handle, CID_SMP, &keys.master_identification);
                            // Persist the new bond so it survives a reboot.
                            let saved = smp.bond().map(|b| persist(&b)).unwrap_or(false);
                            let _ = writeln!(
                                console,
                                "  bonded{} -- type on the USB keyboard",
                                if saved {
                                    " (saved to SD)"
                                } else {
                                    " (SD save failed)"
                                }
                            );
                        }
                        // No key distribution means this is a bonded host
                        // reconnecting. It won't re-write the Report CCC (it
                        // expects the subscription persisted across the bond),
                        // so restore it here or the keyboard stays silent.
                        Ok(None) => {
                            server.subscribe(INPUT_REPORT_HANDLE);
                            let _ = writeln!(console, "  reconnected -- type on the USB keyboard");
                        }
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
                if let Err(e) = bt.start_advertising_raw(&ADV_DATA, Advertising::Connectable, timer)
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

        // Poll the USB keyboard; forward each new report over BLE. A poll
        // with no new report, or a transient endpoint error, is ignored.
        if let Ok(Some(events)) = keyboard.poll(dwc2, timer) {
            let report = events.report().boot_report();
            if let Some(handle) = conn_handle {
                if encrypted && server.is_subscribed(INPUT_REPORT_HANDLE) {
                    if let Some(n) = server.notification(INPUT_REPORT_HANDLE, &report, &mut att_out)
                    {
                        let _ = l2cap::send(bt, handle, CID_ATT, &att_out[..n]);
                    }
                }
            }
        }
    }
}
