#![no_std]
#![no_main]

// Writes a file to the FAT partition on the SD card and reads it back to
// verify, exercising the single-block SD write path (`CMD24`) that
// `rpi_hal::sd` provides through the `embedded-sdmmc` FAT layer -- the
// write-side companion to `sd_fat_read.rs`.
//
// It draws a random 32-bit value from the hardware RNG, writes it as a
// decimal-text line to a scratch file, then reopens that file and reads
// it back, checking the bytes round-trip exactly. A fresh random value
// each run means a stale file left over from a previous run can't make
// the check pass by accident.
//
// SAFETY OF THE BOOT CARD: this writes only to `TEST.TXT` in the root of
// the boot FAT partition, and never to the boot files (`kernel*.img`,
// `config.txt`, bootcode, etc.). Writing a scratch file is safe; a bad
// write to a boot file would brick the card.
//
// The `BlockDevice` adapter is `rpi_hal::sd::SdCard`, behind the crate's
// `embedded-sdmmc` feature. The `TimeSource` stays here: a real clock is
// application policy, not something the HAL should impose -- so this
// example supplies a fixed one, which is the timestamp stamped onto the
// file as it's created.

use core::fmt::Write;
use embedded_sdmmc::{Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::rng::Rng;
use rpi_hal::sd::{Sd, SdCard, SdCardError};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;

/// Name of the scratch file this example writes. Deliberately not any of
/// the boot files -- see the module comment.
const SCRATCH_FILE: &str = "TEST.TXT";

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// A fixed timestamp for `embedded-sdmmc`, stamped onto the scratch file
/// as its modification time. A constant is fine here: nothing in this
/// example depends on the timestamp being real, and a wall clock is
/// application policy, not the HAL's to impose.
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

/// A `core::fmt::Write` sink over a fixed byte buffer, so the random
/// value can be formatted as decimal text without an allocator. Anything
/// past the buffer's end is dropped; the callers here format a single
/// `u32` (at most 10 digits plus a newline) into a 16-byte buffer, so
/// that never happens in practice.
struct BufWriter<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> BufWriter<'a> {
    /// Wraps `buf`, starting empty.
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, len: 0 }
    }
}

impl Write for BufWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            if self.len >= self.buf.len() {
                break;
            }
            self.buf[self.len] = b;
            self.len += 1;
        }
        Ok(())
    }
}

/// Writes `value` as a decimal-text line to [`SCRATCH_FILE`], reads it
/// back, and confirms the bytes match. Split out from `kmain` so the
/// filesystem steps can use `?` against one error type.
fn run(
    sd: Sd,
    timer: &Timer,
    uart: &mut Uart,
    value: u32,
) -> Result<bool, embedded_sdmmc::Error<SdCardError>> {
    // The exact bytes to round-trip: the value as decimal, newline
    // terminated. Read-back is compared against this same buffer.
    let mut expected = [0u8; 16];
    let mut w = BufWriter::new(&mut expected);
    let _ = writeln!(w, "{value}");
    let written_len = w.len;

    let volume_mgr = VolumeManager::new(SdCard::new(sd, timer), FixedTime);

    // VolumeIdx(0) is the first MBR partition -- the FAT boot partition
    // on a stock Raspberry Pi card.
    let volume = volume_mgr.open_volume(VolumeIdx(0))?;
    let root = volume.open_root_dir()?;

    // Write phase: create-or-truncate so each run starts from a known
    // empty file rather than appending to whatever a previous run left.
    let _ = writeln!(uart, "writing {value} to {SCRATCH_FILE}...");
    {
        let file = root.open_file_in_dir(SCRATCH_FILE, Mode::ReadWriteCreateOrTruncate)?;
        file.write(&expected[..written_len])?;
        // Close (not just drop) so a flush error surfaces here rather
        // than being swallowed -- the write isn't committed until this
        // returns.
        file.close()?;
    }

    // Read phase: reopen and pull the bytes back.
    let _ = writeln!(uart, "reading {SCRATCH_FILE} back...");
    let mut read_buf = [0u8; 16];
    let read_len = {
        let file = root.open_file_in_dir(SCRATCH_FILE, Mode::ReadOnly)?;
        let n = file.read(&mut read_buf)?;
        file.close()?;
        n
    };

    let ok = read_len == written_len && read_buf[..read_len] == expected[..written_len];
    if ok {
        let _ = writeln!(uart, "read back {read_len} bytes, matches");
    } else {
        let _ = writeln!(
            uart,
            "MISMATCH: wrote {written_len} bytes, read {read_len} bytes"
        );
    }
    Ok(ok)
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // A fresh random value each run, so a stale scratch file can't make
    // the round-trip check pass by coincidence.
    let value = Rng::new().next_u32();

    let _ = writeln!(uart, "initializing SD card...");
    let sd = match Sd::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, &timer) {
        Ok(sd) => sd,
        Err(e) => {
            let _ = writeln!(uart, "SD init failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(uart, "SD card ready, mounting FAT filesystem...");

    match run(sd, &timer, &mut uart, value) {
        Ok(true) => {
            let _ = writeln!(uart, "\nPASS: write/read-back verified");
        }
        Ok(false) => {
            let _ = writeln!(uart, "\nFAIL: read-back did not match what was written");
        }
        Err(e) => {
            let _ = writeln!(uart, "\nFAT write failed: {e:?}");
        }
    }

    halt();
}
