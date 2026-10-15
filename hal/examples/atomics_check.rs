//! Proves this core's exclusive monitor actually works, which is the
//! whole reason the `mmu` feature exists.
//!
//! `ldrex`/`strex` (and `ldxr`/`stxr` on AArch64) are architecturally
//! UNPREDICTABLE against the Strongly-Ordered memory every address is
//! with the MMU off, and on these cores the monitor is tied to cache
//! line state — so RAM has to be mapped Normal *and* Cacheable before a
//! `strex` can ever succeed. `rpi_hal::mmu` does that; this checks the
//! result rather than assuming it.
//!
//! The check is a hang, not a wrong answer. Every operation below is a
//! retry loop in hardware: `fetch_add` reads exclusively, adds, and
//! stores exclusively, looping while the store reports that the monitor
//! was lost. A monitor that never succeeds does not return an error —
//! it spins, forever, inside `fetch_add`. So each step prints what it is
//! about to do *before* doing it, and the line you never see names the
//! operation that never completed.
//!
//! Expected output, on a board where the map is right:
//!
//! ```text
//! rpi-hal: MIDR 0x410fb767
//! compare_exchange... ok (swapped 0 -> 1)
//! failing compare_exchange... ok (rejected, saw 1)
//! fetch_add x100000... ok (counter = 100001)
//! all atomics ok
//! ```
//!
//! Nothing here is chip- or architecture-specific: it is worth running
//! on any board after a change to the translation table, and it is how a
//! new one earns the `mmu` feature.

#![no_std]
#![no_main]

use core::fmt::Write;
use core::sync::atomic::{AtomicU32, Ordering};
use rpi_hal::halt;
use rpi_hal::{cpu, pac, uart::Uart};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// In `.bss`, so it is ordinary RAM covered by the identity map's Normal
/// memory — the case that matters. A `static mut` in `.data` would be
/// mapped identically, but `.bss` also exercises the boot code's zeroing
/// of it.
static COUNTER: AtomicU32 = AtomicU32::new(0);

/// Enough iterations to be slow if each one is doing real work, and
/// instant if the compiler folded the loop away. 100,000 read-modify-write
/// cycles is a few milliseconds even on a 1GHz ARM1176.
const ITERATIONS: u32 = 100_000;

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);

    let _ = writeln!(uart, "rpi-hal: MIDR {:#010x}", cpu::main_id());

    // The simplest exclusive sequence there is: one read-exclusive, one
    // write-exclusive, no loop of its own. If the monitor is dead this
    // returns `Err` rather than hanging — `compare_exchange` (unlike
    // `_weak`) retries internally only on spurious failure, and a monitor
    // that never grants is not spurious failure, it is every failure.
    let _ = write!(uart, "compare_exchange... ");
    match COUNTER.compare_exchange(0, 1, Ordering::SeqCst, Ordering::SeqCst) {
        Ok(previous) => {
            let _ = writeln!(uart, "ok (swapped {previous} -> 1)");
        }
        Err(actual) => {
            let _ = writeln!(uart, "FAILED (expected 0, saw {actual})");
            halt();
        }
    }

    // The other half of the contract: a compare that should *not* match
    // has to be rejected rather than swapped. A monitor that succeeds
    // unconditionally would pass the test above and fail this one.
    let _ = write!(uart, "failing compare_exchange... ");
    match COUNTER.compare_exchange(0, 99, Ordering::SeqCst, Ordering::SeqCst) {
        Ok(_) => {
            let _ = writeln!(uart, "FAILED (swapped on a value that did not match)");
            halt();
        }
        Err(actual) => {
            let _ = writeln!(uart, "ok (rejected, saw {actual})");
        }
    }

    // And the loop, which is where a broken monitor hangs instead of
    // reporting anything.
    let _ = write!(uart, "fetch_add x{ITERATIONS}... ");
    for _ in 0..ITERATIONS {
        COUNTER.fetch_add(1, Ordering::Relaxed);
    }
    let total = COUNTER.load(Ordering::SeqCst);
    let expected = ITERATIONS + 1;
    if total == expected {
        let _ = writeln!(uart, "ok (counter = {total})");
    } else {
        let _ = writeln!(uart, "FAILED (counter = {total}, expected {expected})");
        halt();
    }

    let _ = writeln!(uart, "all atomics ok");
    halt();
}
