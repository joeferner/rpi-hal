#![no_std]
#![no_main]

// Reads files from the FAT partition on the SD card, on top of the
// single-block SD reads `rpi_hal::sd` provides, using the `no_std`
// `embedded-sdmmc` crate as the FAT layer. This is the read-only half:
// it lists the root directory of the first partition (the Pi's boot
// FAT partition on a stock card) and prints the contents of `config.txt`
// if present -- a human-verifiable end-to-end check that a real
// filesystem parses off a real card. It never writes: the SD driver has
// no write path yet, so `rpi_hal::sd::SdCard` rejects writes.
//
// The `BlockDevice` adapter itself is `rpi_hal::sd::SdCard`, behind the
// crate's `embedded-sdmmc` feature. The `TimeSource` stays here, though:
// a real clock (an RTC or wall-clock the application owns) is application
// policy, not something the HAL should impose -- so this example supplies
// a fixed one, which is all a read-only mount needs.

use core::fmt::Write;
use embedded_sdmmc::{Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::sd::{Sd, SdCard, SdCardError};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// A fixed timestamp for `embedded-sdmmc`. Only ever consulted for the
/// modification time stamped onto files as they're created or written,
/// so it's irrelevant to this read-only example; a constant is fine
/// until there's a real clock (an RTC or the ARM generic timer) to read.
struct FixedTime;

impl TimeSource for FixedTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 56, // 2026
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

/// Lists the root directory of the first partition and prints
/// `config.txt` if present. Split out from `kmain` so the filesystem
/// steps can use `?` against one error type.
fn run(sd: Sd, timer: &Timer, uart: &mut Uart) -> Result<(), embedded_sdmmc::Error<SdCardError>> {
    let volume_mgr = VolumeManager::new(SdCard::new(sd, timer), FixedTime);

    // VolumeIdx(0) is the first MBR partition -- the FAT boot partition
    // on a stock Raspberry Pi card.
    let volume = volume_mgr.open_volume(VolumeIdx(0))?;
    let root = volume.open_root_dir()?;

    let _ = writeln!(uart, "root directory:");
    root.iterate_dir(|entry| {
        let _ = writeln!(
            uart,
            "  {:<12} {:>10} bytes{}",
            entry.name,
            entry.size,
            if entry.attributes.is_directory() {
                " <DIR>"
            } else {
                ""
            }
        );
    })?;

    match root.open_file_in_dir("config.txt", Mode::ReadOnly) {
        Ok(file) => {
            let _ = writeln!(uart, "\nconfig.txt ({} bytes):", file.length());
            let mut buf = [0u8; 64];
            while !file.is_eof() {
                let n = file.read(&mut buf)?;
                // config.txt is ASCII text; print it as-is, substituting
                // '.' for any non-printable byte so a stray value can't
                // scramble the terminal.
                for &b in &buf[..n] {
                    let c = if (0x20..=0x7e).contains(&b) || b == b'\n' || b == b'\r' {
                        b as char
                    } else {
                        '.'
                    };
                    let _ = uart.write_char(c);
                }
            }
            let _ = writeln!(uart);
        }
        Err(embedded_sdmmc::Error::NotFound) => {
            let _ = writeln!(uart, "\nconfig.txt not found");
        }
        Err(e) => return Err(e),
    }

    Ok(())
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    let _ = writeln!(uart, "initializing SD card...");
    let sd = match Sd::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, &timer) {
        Ok(sd) => sd,
        Err(e) => {
            let _ = writeln!(uart, "SD init failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(uart, "SD card ready, mounting FAT filesystem...");

    match run(sd, &timer, &mut uart) {
        Ok(()) => {
            let _ = writeln!(uart, "\ndone");
        }
        Err(e) => {
            let _ = writeln!(uart, "\nFAT read failed: {e:?}");
        }
    }

    halt();
}
