//! SD/MMC multi-block and DMA transfer smoke test.
//!
//! Exercises the four ways [`rpi_hal::sd::Sd`] moves more than one block:
//! polled multi-block reads ([`Sd::read_blocks`], one `CMD18` for the whole
//! run) and DMA reads ([`Sd::read_blocks_dma`], the same command with the
//! data phase moved onto a DMA channel), then cross-checks the two against
//! each other. Reading is non-destructive, so this runs safely on any card.
//!
//! Flip `RUN_WRITE_TEST` to `true` to also exercise both write paths
//! ([`Sd::write_blocks`] polled and [`Sd::write_blocks_dma`]) with a
//! round trip that both *discriminates* a real write from a silent no-op
//! and leaves the card as it found it:
//!
//!  1. read the original contents of `WRITE_TEST_BLOCK`..`+RUN_BLOCKS`,
//!  2. write a bit-inverted copy with the polled path and read it back —
//!     the read-back must equal the *inverted* data, so a write that did
//!     nothing (which would read back as the original) fails the check,
//!  3. write the original back with the DMA path and read it back — the
//!     medium currently holds the inverted data, so this likewise fails
//!     unless the DMA write really landed.
//!
//! It restores the original bytes, so a completed run is non-destructive —
//! but for the window between steps 2 and 3 the region holds inverted data,
//! and a bug in a write path could corrupt it. So it's off by default, and
//! the block offset is left high to steer clear of a partition table /
//! filesystem.

#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::dma::Dma;
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::sd::{Block, Error, Sd};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;

/// Number of consecutive blocks each transfer moves. Eight 512-byte blocks
/// (4 KiB) is enough to span several `READ_RDY` handshakes and a DMA run of
/// real length while staying small.
const RUN_BLOCKS: usize = 8;

/// Whether to run the (opt-in) non-destructive write round trip — see the
/// module docs on why it defaults off.
const RUN_WRITE_TEST: bool = true;
/// First block the write round trip touches, kept high to avoid a card's
/// partition table / filesystem. Only consulted when `RUN_WRITE_TEST`.
const WRITE_TEST_BLOCK: u32 = 0x0010_0000;

/// A run of blocks aligned to a cache line, so the DMA cache maintenance on
/// it never spills onto neighbouring data (each block is already a whole
/// number of 64-byte lines; this fixes the start too).
#[repr(C, align(64))]
struct Blocks([Block; RUN_BLOCKS]);

/// Filled by the polled read, then compared against the DMA read.
static mut PIO_BUF: Blocks = Blocks([[0; 512]; RUN_BLOCKS]);
/// Filled by the DMA read.
static mut DMA_BUF: Blocks = Blocks([[0; 512]; RUN_BLOCKS]);

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
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
    let _ = writeln!(
        uart,
        "SD card ready ({}, {}-bit bus)",
        if sd.high_capacity() {
            "SDHC/SDXC"
        } else {
            "SDSC"
        },
        if sd.four_bit_bus() { 4 } else { 1 }
    );

    // A full DMA channel (0–6). On a bare-metal board that has taken over
    // the machine any channel works; see the DMA driver docs on the
    // firmware's channel mask if the firmware is left running.
    let mut dma = Dma::new();
    let mut channel = dma.channel(5).expect("channel 5 available");

    // SAFETY: single-threaded `kmain`; these statics are touched only here.
    let pio = unsafe { &mut *core::ptr::addr_of_mut!(PIO_BUF) };
    let dma_buf = unsafe { &mut *core::ptr::addr_of_mut!(DMA_BUF) };

    // Read blocks 0..RUN_BLOCKS both ways and confirm the DMA path returns
    // exactly what the polled path does.
    if let Err(e) = sd.read_blocks(0, &mut pio.0, &timer) {
        let _ = writeln!(uart, "polled multi-block read failed: {e:?}");
        halt();
    }
    if let Err(e) = sd.read_blocks_dma(0, &mut dma_buf.0, &mut channel, &timer) {
        let _ = writeln!(uart, "DMA multi-block read failed: {e:?}");
        halt();
    }

    let mismatches = pio
        .0
        .as_flattened()
        .iter()
        .zip(dma_buf.0.as_flattened())
        .filter(|(a, b)| a != b)
        .count();
    if mismatches == 0 {
        let _ = writeln!(
            uart,
            "read PASS: {RUN_BLOCKS} blocks agree between the polled and DMA paths"
        );
    } else {
        let _ = writeln!(uart, "read FAIL: {mismatches} byte(s) differ between paths");
    }

    let block0 = &pio.0[0];
    let _ = writeln!(
        uart,
        "block 0 signature (510-511): {:02x} {:02x}{}",
        block0[510],
        block0[511],
        if block0[510] == 0x55 && block0[511] == 0xaa {
            " -- matches 0x55 0xAA"
        } else {
            " -- not 0x55 0xAA (unpartitioned card, or blank region)"
        }
    );

    if RUN_WRITE_TEST {
        write_round_trip(&sd, &mut channel, pio, dma_buf, &timer, &mut uart);
    }

    let _ = writeln!(uart, "done -- halting");
    halt();
}

/// Discriminating, self-restoring write check — see the module docs for the
/// three steps. `original` holds the card's real contents throughout (the
/// restore source); `scratch` is reused for the inverted copy and every
/// read-back. Exercises the polled write ([`Sd::write_blocks`]) with the
/// inverted data and the DMA write ([`Sd::write_blocks_dma`]) with the
/// restore, so both paths are proven to actually change the medium.
///
/// The DMA restore (step 3) is attempted even if the polled step failed, so
/// the region is left as we found it whenever we got far enough to modify
/// it.
fn write_round_trip(
    sd: &Sd,
    channel: &mut rpi_hal::dma::Channel,
    original: &mut Blocks,
    scratch: &mut Blocks,
    timer: &Timer,
    uart: &mut Uart,
) {
    // 1. Capture the real contents so we can restore them at the end.
    if let Err(e) = sd.read_blocks(WRITE_TEST_BLOCK, &mut original.0, timer) {
        let _ = writeln!(
            uart,
            "write test: initial read failed: {e:?} (card untouched)"
        );
        return;
    }

    // 2. Write a bit-inverted copy with the polled path.
    for (dst, src) in scratch
        .0
        .as_flattened_mut()
        .iter_mut()
        .zip(original.0.as_flattened())
    {
        *dst = !*src;
    }
    let polled_write = sd.write_blocks(WRITE_TEST_BLOCK, &scratch.0, timer);

    // Read it back and confirm the *inverted* bytes landed. Comparing to
    // `!original` (not to `scratch`, which the read overwrites) is what makes
    // this discriminating: a write that silently did nothing reads back as
    // `original`, which differs from the inverted data, and fails here.
    let polled_verify = sd
        .read_blocks(WRITE_TEST_BLOCK, &mut scratch.0, timer)
        .map(|()| {
            scratch
                .0
                .as_flattened()
                .iter()
                .zip(original.0.as_flattened())
                .all(|(read, orig)| *read == !*orig)
        });

    // 3. Restore the original contents with the DMA path — attempted even if
    // a step above failed, so the region is left as we found it.
    let dma_write = sd.write_blocks_dma(WRITE_TEST_BLOCK, &original.0, channel, timer);

    // Read back and confirm the original is restored. This also
    // discriminates the DMA write: the medium currently holds the inverted
    // data, so a no-op DMA write would read back inverted, not restored.
    let dma_verify = sd
        .read_blocks(WRITE_TEST_BLOCK, &mut scratch.0, timer)
        .map(|()| scratch.0.as_flattened() == original.0.as_flattened());

    report_write(uart, "polled write_blocks", polled_write, polled_verify);
    report_write(
        uart,
        "DMA write_blocks_dma (restore)",
        dma_write,
        dma_verify,
    );
}

/// Prints the outcome of one write leg: the write call's result and, if that
/// succeeded, whether the read-back matched what was expected.
fn report_write(
    uart: &mut Uart,
    label: &str,
    write: Result<(), Error>,
    verify: Result<bool, Error>,
) {
    let _ = match (write, verify) {
        (Err(e), _) => writeln!(uart, "{label} FAIL: write errored: {e:?}"),
        (Ok(()), Err(e)) => writeln!(uart, "{label} FAIL: verify read errored: {e:?}"),
        (Ok(()), Ok(true)) => writeln!(uart, "{label} PASS: read-back matched"),
        (Ok(()), Ok(false)) => {
            writeln!(
                uart,
                "{label} FAIL: read-back did not match what was written"
            )
        }
    };
}
