//! Shared support code for the Bluetooth examples.
//!
//! Every `ble_*` / `bt_*` example needs the same handful of things: a panic
//! handler that prints over the mini UART, the `embedded-sdmmc` glue for
//! reading the controller's `.hcd` patchram blob off the SD card, and a way to
//! format a Bluetooth address. Those live here so each example file is just its
//! own protocol logic. Include it with `#[path = "common/mod.rs"] mod common;`
//! and pull what you need from `common::…`.
//!
//! This lives in a subdirectory so Cargo doesn't build it as its own example
//! binary (only top-level files in `examples/` become examples).

// Not every example uses every helper — that's expected for shared support code.
#![allow(dead_code)]

use core::fmt::Write;
use core::ptr::{addr_of, addr_of_mut};
use embedded_sdmmc::{Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rpi_hal::halt;
use rpi_hal::mini_uart::MiniUart;
use rpi_hal::pac;
use rpi_hal::sd::{Sd, SdCard, SdCardError};
use rpi_hal::timer::Timer;

/// Directory on the FAT boot partition holding the firmware blob.
pub const BT_DIR: &str = "BT";
/// Patchram firmware blob, within [`BT_DIR`] (8.3 name).
pub const FIRMWARE_FILE: &str = "BT.HCD";
/// HCI link baud raised to after firmware load (see `bt_probe`).
pub const HCI_BAUD: u32 = 3_000_000;

/// Buffer for the `.hcd` blob (the 43438's `BCM43430A1.hcd` is ~40KB);
/// zeroed BSS.
static mut HCD_BUF: [u8; 64 * 1024] = [0; 64 * 1024];

/// Panic handler shared by the Bluetooth examples. The console is the mini UART
/// here — the PL011 is committed to Bluetooth — so panic output goes there too.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = MiniUart::init(&peripherals.GPIO, &peripherals.AUX, peripherals.UART1);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// A fixed timestamp for `embedded-sdmmc` (only used for file mtimes on
/// writes, which the read-only firmware path never does).
pub struct FixedTime;

impl TimeSource for FixedTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 56,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

/// Mounts the boot partition, reads the `.hcd` firmware blob into a static
/// buffer, and returns it as a slice ready for `Bluetooth::load_firmware`.
/// Consumes the SD driver (and with it EMMC); the Bluetooth path uses the UART,
/// not EMMC, so nothing needs it back.
pub fn firmware_from_sd(
    sd: Sd,
    timer: &Timer,
) -> Result<&'static [u8], embedded_sdmmc::Error<SdCardError>> {
    let volume_mgr = VolumeManager::new(SdCard::new(sd, timer), FixedTime);
    let volume = volume_mgr.open_volume(VolumeIdx(0))?;
    let root = volume.open_root_dir()?;
    let bt = root.open_dir(BT_DIR)?;

    let file = bt.open_file_in_dir(FIRMWARE_FILE, Mode::ReadOnly)?;
    // Safety: single-threaded bare-metal; this buffer is written only here and,
    // once this returns, read-only via the slice handed back.
    let buf = unsafe { &mut *addr_of_mut!(HCD_BUF) };
    let mut total = 0;
    while !file.is_eof() && total < buf.len() {
        let n = file.read(&mut buf[total..])?;
        if n == 0 {
            break;
        }
        total += n;
    }
    Ok(&unsafe { &*addr_of!(HCD_BUF) }[..total])
}

/// Prints a Bluetooth device address MSB-first from its little-endian
/// on-the-wire byte order.
pub fn write_address(console: &mut MiniUart, address: &[u8; 6]) {
    for (i, byte) in address.iter().rev().enumerate() {
        if i != 0 {
            let _ = write!(console, ":");
        }
        let _ = write!(console, "{byte:02X}");
    }
}
