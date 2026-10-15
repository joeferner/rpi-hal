#![no_std]
#![no_main]

// Bluetooth Classic HID gamepad, decoded generically (Pi 3 only): the payoff of
// the whole Classic HID stack. It connects to a game controller (inquiry ->
// page -> SSP pairing -> encrypt, as in `bt_gamepad.rs`), fetches the
// controller's HID **report descriptor** over SDP (`bt_hid_descriptor.rs`),
// parses it into a field map (`rpi_hal::hid_report`), opens the HID L2CAP
// channels (`bluetooth::hid_host`), then decodes the live input reports
// entirely through that field map -- no hard-coded per-device byte offsets.
//
// This is how an OS handles an unknown gamepad: the descriptor says where each
// axis / button / hat lives in the report, so the *same* code decodes an 8BitDo
// SN30 Pro+ and an Xbox controller despite their completely different report
// layouts. `bt_gamepad.rs` hard-codes the SN30's byte offsets; this doesn't
// know them, and works for both. The decoding half is shared verbatim with
// `usb_gamepad.rs` (`common/hid_gamepad.rs`), which does the same over USB --
// past the descriptor, the transport stops mattering.
//
// Put the controller in **pairing mode** (fast-blinking LED) near the Pi.
//
// Setup mirrors `bt_gamepad.rs`: the console is the mini UART (GPIO14/15, needs
// `core_freq=250` in `config.txt`), and the `.hcd` patchram blob is read off
// the SD card. In a `bt` directory on the boot partition, under an 8.3 name:
//   BT.HCD -- Broadcom's BCM43430A1.hcd patchram blob
//
// Get BCM43430A1.hcd (the original Pi 3B's BCM43438; not the 3B+/Zero 2 W's
// BCM4345C0.hcd) from RPi-Distro/bluez-firmware at
// https://github.com/RPi-Distro/bluez-firmware/blob/master/broadcom/BCM43430A1.hcd
// or copy it from /lib/firmware/brcm/ on any Raspberry Pi OS install.

use core::fmt::Write;
use rpi_hal::bluetooth::hid_host::HidHost;
use rpi_hal::bluetooth::{sdp, Bluetooth};
use rpi_hal::halt;
use rpi_hal::hid_report::ReportDescriptor;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::mini_uart::MiniUart;
use rpi_hal::pac;
use rpi_hal::sd::Sd;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;

#[path = "common/mod.rs"]
mod common;
use common::{firmware_from_sd, write_address, HCI_BAUD};

#[path = "common/hid_gamepad.rs"]
mod hid_gamepad;
use hid_gamepad::{print_fields, Decoder};

/// How long to gather inquiry results before picking the strongest gamepad, ms.
const GAMEPAD_SCAN_MS: u32 = 8_000;
/// How long to wait for each inquiry result before looping, in ms.
const RESULT_WAIT_MS: u32 = 1_500;
/// A gamepad at least this strong (dBm) is close enough to page immediately.
const EARLY_CONNECT_RSSI_DBM: i8 = -60;
/// How long to allow the HID L2CAP channels to open, in ms.
const HID_OPEN_TIMEOUT_MS: u32 = 10_000;
/// How long each input-report poll blocks, in ms.
const REPORT_POLL_MS: u32 = 1_000;
/// Max HID report descriptor bytes we buffer.
const MAX_DESCRIPTOR: usize = 512;

/// Inquires for a window and returns the address of the strongest gamepad
/// found, or `None`. Connects early to a close, strong one.
fn find_nearest_gamepad(
    bt: &mut Bluetooth,
    console: &mut MiniUart,
    timer: &Timer,
) -> Option<[u8; 6]> {
    if let Err(e) = bt.start_inquiry(timer) {
        let _ = writeln!(console, "start inquiry failed: {e:?}");
        return None;
    }
    let deadline = timer.now_micros() + (GAMEPAD_SCAN_MS as u64) * 1000;
    let mut best_addr: Option<[u8; 6]> = None;
    let mut best_rssi = i8::MIN;
    while timer.now_micros() < deadline {
        match bt.next_inquiry_result(timer, RESULT_WAIT_MS) {
            Ok(Some(result)) if result.is_gamepad() => {
                let rssi = result.rssi.unwrap_or(i8::MIN);
                let _ = write!(console, "  gamepad ");
                write_address(console, &result.bd_addr);
                let _ = writeln!(console, " ({rssi} dBm)");
                if best_addr.is_none() || rssi > best_rssi {
                    best_addr = Some(result.bd_addr);
                    best_rssi = rssi;
                }
                if rssi >= EARLY_CONNECT_RSSI_DBM {
                    break;
                }
            }
            Ok(_) => {}
            Err(e) => {
                let _ = writeln!(console, "inquiry error: {e:?}");
                break;
            }
        }
    }
    let _ = bt.inquiry_cancel(timer);
    best_addr
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
    if let Err(e) = bt.read_buffer_size(&timer) {
        let _ = writeln!(console, "read buffer size failed: {e:?}");
        halt();
    }
    if let Err(e) = bt.set_simple_pairing_mode(true, &timer) {
        let _ = writeln!(console, "enable simple pairing failed: {e:?}");
        halt();
    }
    if let Err(e) = bt.set_inquiry_mode_extended(&timer) {
        let _ = writeln!(
            console,
            "extended inquiry mode declined ({e:?}) -- continuing"
        );
    }
    let _ = writeln!(console, "controller ready");

    // 1. Find, connect, pair, encrypt.
    let _ = writeln!(
        console,
        "inquiring for a gamepad (put the controller in pairing mode)..."
    );
    let addr = match find_nearest_gamepad(&mut bt, &mut console, &timer) {
        Some(addr) => addr,
        None => {
            let _ = writeln!(console, "no gamepad found -- giving up");
            halt();
        }
    };
    let _ = write!(console, "connecting to ");
    write_address(&mut console, &addr);
    let _ = writeln!(console, "...");
    let handle = match bt.classic_connect(&addr, &timer) {
        Ok(h) => h,
        Err(e) => {
            let _ = writeln!(console, "connect failed: {e:?}");
            halt();
        }
    };
    if let Err(e) = bt.classic_pair(&addr, handle, &timer) {
        let _ = writeln!(console, "pairing failed: {e:?}");
        halt();
    }
    if let Err(e) = bt.classic_set_encryption(handle, &timer) {
        let _ = writeln!(console, "enable encryption failed: {e:?}");
        halt();
    }
    let _ = writeln!(
        console,
        "connected + encrypted -- fetching HID descriptor..."
    );

    // 2. Fetch the HID report descriptor over SDP and parse it into a field map.
    //    The raw hexdump + parsed field map are printed to make report-decode
    //    issues debuggable against the actual descriptor bytes.
    let mut descriptor = [0u8; MAX_DESCRIPTOR];
    let rd = match sdp::read_report_descriptor(&mut bt, handle, &timer, &mut descriptor) {
        Ok(n) => {
            let _ = writeln!(console, "HID report descriptor ({n} bytes):");
            for (i, byte) in descriptor[..n].iter().enumerate() {
                if i % 16 == 0 {
                    let _ = write!(console, "\r\n  ");
                }
                let _ = write!(console, "{byte:02X} ");
            }
            let _ = writeln!(console, "\r\n");
            let rd = ReportDescriptor::parse(&descriptor[..n]);
            print_fields(&mut console, &rd);
            rd
        }
        Err(e) => {
            let _ = writeln!(console, "SDP read failed: {e:?}");
            halt();
        }
    };

    // 3. Open the HID L2CAP channels (control + interrupt).
    let _ = writeln!(console, "opening HID channels...");
    let mut hid = HidHost::new(handle);
    if let Err(e) = hid.open(&mut bt, &timer, HID_OPEN_TIMEOUT_MS) {
        let _ = writeln!(console, "HID channel open failed: {e:?}");
        halt();
    }
    let _ = writeln!(
        console,
        "HID channels open -- leave the controls at rest, then move them."
    );
    let _ = writeln!(
        console,
        "(axes are labelled by HID usage and calibrated from the first report;"
    );
    let _ = writeln!(
        console,
        " which usage is a stick vs a trigger is a device convention, not in the"
    );
    let _ = writeln!(
        console,
        " descriptor -- an OS resolves that with a per-device quirk database.)"
    );

    // 4. Decode reports through the field map -- device-agnostic, and only
    //    reprinted when the decoded state changes (see `Decoder`).
    let mut decoder = Decoder::new();
    let mut report = [0u8; 64];
    loop {
        match hid.next_report(&mut bt, &timer, REPORT_POLL_MS, &mut report) {
            Ok(Some(n)) if n > 0 => {
                // Split off the report-ID byte the same way the parser assumes.
                let (report_id, payload) = if rd.uses_report_ids() {
                    (report[0], &report[1..n])
                } else {
                    (0u8, &report[..n])
                };
                decoder.print_changes(&mut console, &rd, report_id, payload);
            }
            Ok(_) => {} // empty report or quiet window
            Err(e) => {
                let _ = writeln!(console, "report error: {e:?}");
                halt();
            }
        }
    }
}
