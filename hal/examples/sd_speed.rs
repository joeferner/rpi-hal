//! SD card read throughput, in every bus mode the driver can negotiate.
//!
//! For each [`BusMode`] from slowest to fastest — on a Pi 5 up to UHS-I
//! SDR104 — the card is brought up with [`Sd::init_with`] capped at that
//! mode (a mode the card or chip doesn't have is reported and skipped),
//! and the same 32 MiB from the start of the card is read in 1 MiB
//! multi-block commands. Only the reads
//! are timed. Each pass also hashes what it read, and every pass must hash
//! the same: a mode that reads fast but corrupts data shows up as a
//! mismatch rather than a good number.
//!
//! Then, at the fastest mode, the same 8 MiB is read in commands of 4 KiB,
//! 64 KiB and 1 MiB, which shows what each command costs on top of the
//! bytes it moves.
//!
//! Reading only, so it is safe on any card. Output goes to UART0 at 115200
//! 8N1. MB/s here is 10^6 bytes a second.

#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::sd::{Block, BusMode, Sd};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;

/// Blocks per command in the mode comparison: 1 MiB.
const COMMAND_BLOCKS: usize = 2048;
/// Bytes each mode reads.
const MODE_BYTES: usize = 32 * 1024 * 1024;
/// Bytes each command size reads.
const SIZE_BYTES: usize = 8 * 1024 * 1024;

/// The read buffer: one command's worth.
static mut BUFFER: [Block; COMMAND_BLOCKS] = [[0; 512]; COMMAND_BLOCKS];

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// FNV-1a, continued from `hash` over `bytes`.
fn fnv1a(mut hash: u64, bytes: &[u8]) -> u64 {
    for &b in bytes {
        hash ^= u64::from(b);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

/// Reads `bytes` from block 0 on in commands of `blocks` blocks. Returns
/// the microseconds spent in the reads and the hash of what they read.
fn read_run(
    sd: &Sd,
    buffer: &mut [Block; COMMAND_BLOCKS],
    blocks: usize,
    bytes: usize,
    timer: &Timer,
) -> Result<(u64, u64), rpi_hal::sd::Error> {
    let mut hash = 0xcbf2_9ce4_8422_2325;
    let mut us = 0;
    let commands = bytes / (blocks * 512);
    for c in 0..commands {
        let run = &mut buffer[..blocks];
        let started = timer.now_micros();
        sd.read_blocks((c * blocks) as u32, run, timer)?;
        us += timer.now_micros() - started;
        hash = fnv1a(hash, run.as_flattened());
    }
    Ok((us, hash))
}

/// Prints `bytes` in `us` as MB/s with one decimal.
fn rate(bytes: usize, us: u64) -> (u64, u64) {
    let tenths = bytes as u64 * 10 / us.max(1);
    (tenths / 10, tenths % 10)
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    // SAFETY: single-threaded `kmain`; the buffer is touched only here.
    let buffer = unsafe { &mut *core::ptr::addr_of_mut!(BUFFER) };

    let _ = writeln!(
        uart,
        "sd_speed: {} MiB per mode in {} KiB commands",
        MODE_BYTES >> 20,
        COMMAND_BLOCKS / 2
    );

    let mut reference = None;
    let mut fastest = BusMode::OneBit;
    for mode in [
        BusMode::OneBit,
        BusMode::FourBit,
        BusMode::HighSpeed,
        BusMode::Sdr50,
        BusMode::Sdr104,
    ] {
        // SAFETY: the previous `Sd`, if any, was dropped at the end of the
        // last iteration, so nothing else holds the controller.
        let emmc = unsafe { Sd::steal_emmc() };
        // Printed first, so that a mode which hangs the card says which.
        let _ = writeln!(uart, "{mode:?}: bringing the card up");
        let started = timer.now_micros();
        let sd = match Sd::init_with(&peripherals.GPIO, emmc, &mut mailbox, &timer, mode) {
            Ok(sd) => sd,
            Err(e) => {
                let _ = writeln!(uart, "{mode:?}: init failed: {e:?}");
                continue;
            }
        };
        let _ = writeln!(
            uart,
            "{mode:?}: up as {:?} in {} ms",
            sd.bus_mode(),
            (timer.now_micros() - started) / 1000
        );
        if sd.bus_mode() != mode {
            let _ = writeln!(uart, "{mode:?}: not available; skipped");
            continue;
        }
        match read_run(&sd, buffer, COMMAND_BLOCKS, MODE_BYTES, &timer) {
            Ok((us, hash)) => {
                let (whole, tenth) = rate(MODE_BYTES, us);
                let same = *reference.get_or_insert(hash) == hash;
                let _ = writeln!(
                    uart,
                    "{mode:?}: {whole}.{tenth} MB/s ({} ms), data {}",
                    us / 1000,
                    if same {
                        "matches"
                    } else {
                        "DIFFERS from the first mode's"
                    }
                );
                fastest = mode;
            }
            Err(e) => {
                let _ = writeln!(uart, "{mode:?}: read failed: {e:?}");
            }
        }
    }

    // Command size at the fastest mode that worked.
    // SAFETY: as above.
    let emmc = unsafe { Sd::steal_emmc() };
    match Sd::init_with(&peripherals.GPIO, emmc, &mut mailbox, &timer, fastest) {
        Ok(sd) => {
            let _ = writeln!(
                uart,
                "{fastest:?}, {} MiB per command size:",
                SIZE_BYTES >> 20
            );
            for blocks in [8, 128, COMMAND_BLOCKS] {
                match read_run(&sd, buffer, blocks, SIZE_BYTES, &timer) {
                    Ok((us, _)) => {
                        let (whole, tenth) = rate(SIZE_BYTES, us);
                        let commands = SIZE_BYTES / (blocks * 512);
                        let _ = writeln!(
                            uart,
                            "  {:>4} KiB commands: {whole}.{tenth} MB/s, {} us a command",
                            blocks / 2,
                            us / commands as u64
                        );
                    }
                    Err(e) => {
                        let _ = writeln!(uart, "  {} KiB commands: failed: {e:?}", blocks / 2);
                    }
                }
            }
        }
        Err(e) => {
            let _ = writeln!(uart, "{fastest:?}: init failed: {e:?}");
        }
    }

    let _ = writeln!(uart, "sd_speed: done");
    halt();
}
