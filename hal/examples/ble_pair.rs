#![no_std]
#![no_main]

// BLE pairing to an encrypted link (Pi 3 only): brings the on-board
// BCM43438 controller up, advertises a small GATT peripheral (see
// `ble_gatt.rs`), and runs LE Legacy "Just Works" pairing (see
// `bluetooth::smp`) when a central initiates it -- taking the connection all
// the way to an encrypted link.
//
// Connect with a scanner (nRF Connect: connect, then the overflow menu ->
// "Bond"), and the console traces the pairing exchange: Pairing Request ->
// Response, the Confirm and Random exchange, the controller's Long Term Key
// Request answered with the derived STK, and finally the Encryption Change
// event confirming the link is encrypted. This is the security step a
// HID-over-GATT host requires before it will accept keyboard input.
//
// Just Works only (IO capability NoInputNoOutput, no MITM, no bonding): the
// link is encrypted for the session, but nothing is persisted, so a
// reconnect pairs again. The pairing crypto self-tests against known vectors
// at startup (`bluetooth::smp::Smp::new`); if that fails the example stops
// rather than pairing on untrusted crypto.
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
    attr, characteristic, primary_service, Attribute, Server, ATT_MAX_MTU, CHAR_PROP_READ,
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

/// The name this peripheral advertises under.
const ADV_NAME: &str = "rpi-hal-le";
/// How long each [`Bluetooth::poll`] waits for the next event, in ms.
const POLL_MS: u32 = 1000;
/// HCI address type `Public` — the address we advertise/pair with.
const ADDR_TYPE_PUBLIC: u8 = 0x00;

/// Device Name characteristic declaration: READ, value at 0x0003, UUID
/// 0x2A00.
static DECL_DEVICE_NAME: [u8; 5] = [CHAR_PROP_READ, 0x03, 0x00, 0x00, 0x2a];
/// Appearance characteristic declaration: READ, value at 0x0005, UUID
/// 0x2A01.
static DECL_APPEARANCE: [u8; 5] = [CHAR_PROP_READ, 0x05, 0x00, 0x01, 0x2a];
/// Appearance value `0x0000` (Unknown).
static APPEARANCE_UNKNOWN: [u8; 2] = [0x00, 0x00];
/// GAP service UUID `0x1800`, little-endian.
static SVC_GAP: [u8; 2] = [0x00, 0x18];
/// GATT service UUID `0x1801`, little-endian.
static SVC_GATT: [u8; 2] = [0x01, 0x18];

/// A minimal attribute table (GAP + GATT) so the peripheral discovers
/// cleanly; the focus of this example is pairing, not the GATT content.
static ATTRIBUTES: [Attribute; 6] = [
    primary_service(0x0001, &SVC_GAP),
    characteristic(0x0002, &DECL_DEVICE_NAME),
    attr(0x0003, 0x2a00, ADV_NAME.as_bytes()),
    characteristic(0x0004, &DECL_APPEARANCE),
    attr(0x0005, 0x2a01, &APPEARANCE_UNKNOWN),
    primary_service(0x0006, &SVC_GATT),
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

    // Our public address is the responder address used in the pairing
    // crypto; read it before pairing.
    let own_addr = match bt.read_bd_addr(&timer) {
        Ok(addr) => addr,
        Err(e) => {
            let _ = writeln!(console, "read BD_ADDR failed: {e:?}");
            halt();
        }
    };

    // Build the pairing responder (this runs the crypto self-test).
    // Session encryption only (no bonding) — this example just demonstrates
    // the pairing-to-encryption path.
    let mut smp = match Smp::new(&mut bt, &timer, false) {
        Ok(smp) => smp,
        Err(e) => {
            let _ = writeln!(console, "SMP init (crypto self-test) failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(console, "controller ready, pairing crypto ok");

    // Advertise connectably so a central can connect and pair.
    if let Err(e) = bt.start_advertising(ADV_NAME, Advertising::Connectable, &timer) {
        let _ = writeln!(console, "start advertising failed: {e:?}");
        halt();
    }
    let _ = writeln!(
        console,
        "advertising as '{ADV_NAME}' -- connect and Bond in a phone scanner"
    );

    let mut reasm = Reassembler::new();
    let mut server = Server::new(&ATTRIBUTES);
    let mut att_out = [0u8; ATT_MAX_MTU as usize];
    let mut smp_out = [0u8; 32];
    loop {
        match bt.poll(&timer, POLL_MS) {
            Ok(Some(Event::Connected(conn))) => {
                on_connected(&mut console, &conn);
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
                                let _ = writeln!(
                                    console,
                                    "  SMP req {:#04x} -> rsp {:#04x}",
                                    pdu.payload.first().copied().unwrap_or(0),
                                    smp_out[0]
                                );
                                if let Err(e) = l2cap::send(&mut bt, handle, CID_SMP, &smp_out[..n])
                                {
                                    let _ = writeln!(console, "  SMP send failed: {e:?}");
                                }
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
                    match bt.le_ltk_request_reply(handle, &key, &timer) {
                        Ok(()) => {
                            let _ = writeln!(console, "  LTK request -> supplied key, encrypting");
                        }
                        Err(e) => {
                            let _ = writeln!(console, "  LTK reply failed: {e:?}");
                        }
                    }
                } else {
                    let _ = writeln!(console, "  LTK request with no matching key -> rejecting");
                    let _ = bt.le_ltk_request_negative_reply(handle, &timer);
                }
            }
            Ok(Some(Event::EncryptionChange { handle, enabled })) => {
                let _ = writeln!(
                    console,
                    "encryption change: handle {handle:#06x}, enabled={enabled} -- link is {}",
                    if enabled { "ENCRYPTED" } else { "plain" }
                );
            }
            Ok(Some(Event::Disconnected { handle, reason })) => {
                let _ = writeln!(
                    console,
                    "disconnected: handle {handle:#06x}, reason {reason:#04x} -- re-advertising"
                );
                reasm.reset();
                server.reset();
                smp.reset();
                if let Err(e) = bt.start_advertising(ADV_NAME, Advertising::Connectable, &timer) {
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
    }
}
