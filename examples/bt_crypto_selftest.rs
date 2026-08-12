#![no_std]
#![no_main]

// BLE SMP crypto self-test (Pi 3 only): brings the on-board BCM43438
// Bluetooth controller up, then runs the Security Manager pairing crypto
// self-test (`bluetooth::smp::self_test`) against published test vectors and
// prints the result. No phone or connection is involved -- this proves the
// pairing crypto (the AES primitive via HCI LE_Encrypt, the byte-order
// handling, and the c1 confirm function) is correct on real hardware before
// the pairing state machine is built on top of it.
//
// Expected output on a healthy controller:
//   crypto self-test: AES ok (convention: <direct|swapped>), c1 ok -> PASSED
//
// Setup mirrors `bt_probe.rs`: the console is the mini UART (GPIO14/15,
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
use rpi_hal::bluetooth::{smp, Bluetooth};
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
    let _ = writeln!(console, "controller ready");

    // Run the SMP crypto self-test against published vectors.
    match smp::self_test(&mut bt, &timer) {
        Ok(result) => {
            let convention = if result.swapped { "swapped" } else { "direct" };
            let aes = if result.aes_ok { "ok" } else { "FAILED" };
            let c1 = if result.c1_ok { "ok" } else { "FAILED" };
            let verdict = if result.passed() { "PASSED" } else { "FAILED" };
            let _ = writeln!(
                console,
                "crypto self-test: AES {aes} (convention: {convention}), c1 {c1} -> {verdict}"
            );
        }
        Err(e) => {
            let _ = writeln!(console, "crypto self-test error: {e:?}");
        }
    }

    halt();
}
