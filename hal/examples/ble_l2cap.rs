#![no_std]
#![no_main]

// BLE L2CAP layer (Pi 3 only): brings the on-board BCM43438 Bluetooth
// controller up and advertises connectably (see `ble_connect.rs` for the
// bare connection transport this builds on), then runs the L2CAP framing
// layer on top of ACL -- reassembling ACL fragments into L2CAP B-frames,
// routing them by channel ID, and sending a reply back through L2CAP.
//
// Where `ble_connect.rs` dumps raw ACL bytes and lets the phone give up,
// this decodes them: a connecting phone's first traffic is a GATT
// service-discovery request on the ATT channel (CID 0x0004). There's no
// GATT server yet (that's the next milestone), so this answers every ATT
// request with an ATT Error Response ("Request Not Supported") -- a
// hand-built 5-byte PDU sent via `l2cap::send`, purely to exercise the
// L2CAP transmit path and give the phone a well-formed reply instead of a
// timeout. Traffic on the SMP (0x0006) and signaling (0x0005) channels is
// decoded and logged but not answered.
//
// Point a scanner (nRF Connect, LightBlue) at "rpi-hal-le" and tap Connect;
// the console shows each reassembled L2CAP frame by channel and the ATT
// error responses going back out.
//
// Setup mirrors `ble_connect.rs`: the console is the mini UART (GPIO14/15,
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
use rpi_hal::bluetooth::l2cap::{self, Reassembler, CID_ATT, CID_LE_SIGNALING, CID_SMP};
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

/// ATT opcode `Error Response`.
const ATT_OP_ERROR_RESPONSE: u8 = 0x01;
/// ATT error code `Request Not Supported` — the honest answer while there's
/// no GATT server: the request is understood as ATT but nothing serves it.
const ATT_ERR_REQUEST_NOT_SUPPORTED: u8 = 0x06;
/// ATT opcode `Command` flag (bit 6): set on client commands, which — unlike
/// requests — expect no response, so the stub responder leaves them alone.
const ATT_COMMAND_FLAG: u8 = 0x40;

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

/// Logs a reassembled L2CAP frame: its channel and payload bytes.
fn log_pdu(console: &mut MiniUart, cid: u16, payload: &[u8]) {
    let channel = match cid {
        CID_ATT => "ATT",
        CID_LE_SIGNALING => "SIGNALING",
        CID_SMP => "SMP",
        _ => "?",
    };
    let _ = write!(
        console,
        "  L2CAP cid {cid:#06x} ({channel}) [{} bytes]:",
        payload.len()
    );
    for byte in payload {
        let _ = write!(console, " {byte:02x}");
    }
    let _ = writeln!(console);
}

/// Answers an ATT request with an ATT Error Response citing the request's
/// opcode and "Request Not Supported" (there's no GATT server yet). Skips
/// ATT commands, which expect no response, and empty payloads. Returns
/// `true` if a response was sent.
fn answer_att(bt: &mut Bluetooth, console: &mut MiniUart, handle: u16, payload: &[u8]) -> bool {
    let Some(&opcode) = payload.first() else {
        return false;
    };
    if opcode & ATT_COMMAND_FLAG != 0 {
        // A command expects no reply.
        return false;
    }
    // Error Response: opcode, request opcode, attribute handle (0x0000,
    // none), error code.
    let response = [
        ATT_OP_ERROR_RESPONSE,
        opcode,
        0x00,
        0x00,
        ATT_ERR_REQUEST_NOT_SUPPORTED,
    ];
    match l2cap::send(bt, handle, CID_ATT, &response) {
        Ok(()) => {
            let _ = writeln!(
                console,
                "    -> ATT Error Response (req {opcode:#04x}, Request Not Supported)"
            );
            true
        }
        Err(e) => {
            let _ = writeln!(console, "    -> ATT response send failed: {e:?}");
            false
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

    // Advertise connectably so a central can connect.
    if let Err(e) = bt.start_advertising(ADV_NAME, Advertising::Connectable, &timer) {
        let _ = writeln!(console, "start advertising failed: {e:?}");
        halt();
    }
    let _ = writeln!(
        console,
        "advertising connectably as '{ADV_NAME}' -- tap Connect in a phone scanner"
    );

    // Service the connection: reassemble ACL into L2CAP frames, log them by
    // channel, and answer ATT requests with a stub error response.
    let mut reasm = Reassembler::new();
    loop {
        match bt.poll(&timer, POLL_MS) {
            Ok(Some(Event::Connected(conn))) => on_connected(&mut console, &conn),
            Ok(Some(Event::Acl(acl))) => {
                let handle = acl.handle;
                if let Some(pdu) = reasm.feed(&acl) {
                    log_pdu(&mut console, pdu.cid, pdu.payload);
                    // The PDU borrows the reassembler, which is independent
                    // of `bt`/`console`, so it can be used across the reply.
                    if pdu.cid == CID_ATT {
                        answer_att(&mut bt, &mut console, handle, pdu.payload);
                    }
                }
            }
            Ok(Some(Event::Disconnected { handle, reason })) => {
                let _ = writeln!(
                    console,
                    "disconnected: handle {handle:#06x}, reason {reason:#04x} -- re-advertising"
                );
                reasm.reset();
                if let Err(e) = bt.start_advertising(ADV_NAME, Advertising::Connectable, &timer) {
                    let _ = writeln!(console, "re-advertise failed: {e:?}");
                    halt();
                }
            }
            // Other events (LTK request, encryption change) don't arise
            // without pairing, which this L2CAP example doesn't do.
            Ok(Some(_)) => {}
            Ok(None) => {} // quiet window; keep polling
            Err(e) => {
                let _ = writeln!(console, "poll error: {e:?}");
                halt();
            }
        }
    }
}
