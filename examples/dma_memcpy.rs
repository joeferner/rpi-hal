//! Memory-to-memory DMA copy self-test.
//!
//! Fills a source buffer with a pattern, copies it into a destination
//! buffer over a DMA channel (no CPU byte-copy involved), and reports
//! whether the destination matches over UART. A self-contained smoke test
//! of the DMA controller that needs no external hardware.

#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::dma::Dma;
use rpi_hal::halt;
use rpi_hal::{pac, uart::Uart};

/// Bytes to copy. A multiple of the cache line so the aligned buffers below
/// occupy whole lines — the destination invalidate then can't touch
/// anything else.
const LEN: usize = 4096;

/// A `LEN`-byte buffer aligned to a cache line, so DMA cache maintenance on
/// it never spills onto neighbouring data.
#[repr(C, align(64))]
struct Buffer([u8; LEN]);

/// Source buffer, filled with a pattern before the copy.
static mut SRC: Buffer = Buffer([0; LEN]);
/// Destination buffer, checked against `SRC` after the copy.
static mut DST: Buffer = Buffer([0; LEN]);

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

    // A full channel (0–6). On a bare-metal board that has taken over the
    // machine, any channel works; see the driver docs on the firmware's
    // channel mask if the firmware is left running.
    let mut dma = Dma::new();
    let mut channel = dma.channel(5).expect("channel 5 available");

    // SAFETY: single-threaded `kmain`; these statics are touched only here.
    let src = unsafe { &mut *core::ptr::addr_of_mut!(SRC) };
    let dst = unsafe { &mut *core::ptr::addr_of_mut!(DST) };

    // A pattern that isn't all-equal, so a partial or misaligned copy shows
    // up rather than accidentally matching.
    for (i, byte) in src.0.iter_mut().enumerate() {
        *byte = (i as u8) ^ 0xa5;
    }
    dst.0.fill(0);

    match channel.memcpy(&mut dst.0, &src.0) {
        Ok(()) => {
            let mismatches = src
                .0
                .iter()
                .zip(dst.0.iter())
                .filter(|(a, b)| a != b)
                .count();
            if mismatches == 0 {
                let _ = writeln!(uart, "DMA memcpy PASS: {LEN} bytes copied and verified");
            } else {
                let _ = writeln!(uart, "DMA memcpy FAIL: {mismatches} byte(s) differ");
                let _ = writeln!(uart, "  dst[0..4] = {:02x?}", &dst.0[..4]);
            }
        }
        Err(error) => {
            let _ = writeln!(uart, "DMA memcpy ERROR: {error:?}");
        }
    }

    halt();
}
