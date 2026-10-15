#![no_std]
#![no_main]

// Bluetooth controller probe (Pi 3 only): brings the on-board BCM43438
// Bluetooth controller up over its HCI UART, downloads the `.hcd`
// patchram firmware blob, then reads back the controller's local version
// and Bluetooth device address -- proof the HCI path round-trips and the
// firmware is running. The Bluetooth equivalent of `wifi_scan.rs`; it
// stops once the controller is alive (no L2CAP/GATT/RFCOMM above it).
//
// The controller's HCI UART is wired to the SoC's PL011 on GPIO30-33, the
// *same* PL011 the GPIO14/15 debug console normally uses -- so this moves
// the console to the mini UART (GPIO14/15) and commits the PL011 to
// Bluetooth. The mini UART's baud tracks the VPU/core clock, so a legible
// console needs `core_freq=250` pinned in `config.txt` (see the mini UART
// driver's notes); without it this still runs, but its output is garbled.
//
// The firmware blob is read off the SD card first (over the EMMC SD
// driver), the same way `wifi_scan.rs` reads its Wi-Fi blobs. In a `bt`
// directory on the boot partition, under an 8.3 name:
//   BT.HCD -- Broadcom's BCM43430A1.hcd patchram blob
//
// Get BCM43430A1.hcd (the original Pi 3B's BCM43438; not the 3B+/Zero 2 W's
// BCM4345C0.hcd) from RPi-Distro/bluez-firmware at
// https://github.com/RPi-Distro/bluez-firmware/blob/master/broadcom/BCM43430A1.hcd
// or copy it from /lib/firmware/brcm/ on any Raspberry Pi OS install.

use core::fmt::Write;
use rpi_hal::bluetooth::Bluetooth;
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
    let _ = writeln!(
        console,
        "  {}/{}: {} bytes",
        common::BT_DIR,
        common::FIRMWARE_FILE,
        hcd.len()
    );

    // Commit the PL011 to the controller's HCI pins (GPIO30-33, with
    // hardware flow control) and power the controller up via BT_ON.
    let _ = writeln!(console, "bringing up Bluetooth controller over HCI...");
    let hci = Uart::init_bluetooth(&peripherals.GPIO, peripherals.UART0);
    let mut bt = Bluetooth::new(hci, &mut mailbox, &timer);

    // Download and launch the patchram firmware.
    let _ = writeln!(console, "downloading firmware...");
    if let Err(e) = bt.load_firmware(hcd, &timer) {
        let _ = writeln!(console, "firmware load failed: {e:?}");
        halt();
    }
    let _ = writeln!(console, "firmware running: controller ready");

    // Raise the HCI link from the 115200 the controller boots at to
    // 3 Mbaud (the rate Raspberry Pi OS uses). The version/address reads
    // below then happen at the new rate, so a successful round-trip is
    // itself proof the bump took on both ends.
    if let Err(e) = bt.set_baud(HCI_BAUD, &timer) {
        let _ = writeln!(console, "baud bump to {HCI_BAUD} failed: {e:?}");
        halt();
    }
    let _ = writeln!(console, "HCI link raised to {HCI_BAUD} baud");

    // Proof the HCI control path round-trips: read the local version and
    // BD_ADDR from the running firmware (now at the higher baud).
    match bt.read_local_version(&timer) {
        Ok(v) => {
            let vendor = if v.manufacturer == 0x000f {
                " (Broadcom)"
            } else {
                ""
            };
            let _ = writeln!(
                console,
                "local version: hci {} rev {:#06x}, lmp {} subver {:#06x}, manufacturer {:#06x}{}",
                v.hci_version,
                v.hci_revision,
                v.lmp_version,
                v.lmp_subversion,
                v.manufacturer,
                vendor
            );
        }
        Err(e) => {
            let _ = writeln!(console, "read local version failed: {e:?}");
            halt();
        }
    }

    match bt.read_bd_addr(&timer) {
        Ok(addr) => {
            // Returned little-endian (LSB first); print MSB first.
            let _ = writeln!(
                console,
                "BD_ADDR: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]
            );
        }
        Err(e) => {
            let _ = writeln!(console, "read BD_ADDR failed: {e:?}");
            halt();
        }
    }

    let _ = writeln!(console, "Bluetooth controller alive.");
    halt();
}
