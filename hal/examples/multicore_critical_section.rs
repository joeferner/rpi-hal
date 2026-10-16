#![no_std]
#![no_main]

//! `critical-section` with the `multicore` feature: nesting, and mutual
//! exclusion between cores.
//!
//! Two checks, each reported as PASS or FAIL over the UART:
//!
//! 1. **Nesting.** Core 0 takes a critical section inside another. The
//!    `critical-section` crate allows that, and code does it without
//!    noticing (a log call inside a `with`). The lock used to be a plain
//!    flag that couldn't tell its own core from another, so this hung
//!    here: no PASS line, and nothing after it.
//! 2. **Exclusion.** Cores 0 and 1 each add to a shared counter
//!    [`ROUNDS`] times, a read, a pause, and a write inside a nested
//!    critical section. A lost update — both cores reading the same value
//!    — leaves the total short of `2 * ROUNDS`, which only real mutual
//!    exclusion prevents. The pause widens the window so a broken lock
//!    can't pass by luck.
//!
//! Requires the `multicore` feature.

use core::cell::Cell;
use core::fmt::Write;
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::Mutex;
use rpi_hal::multicore::{Cores, Stack};
use rpi_hal::{pac, uart::Uart};

/// How many times each core adds to [`COUNTER`].
const ROUNDS: u32 = 100_000;

/// The shared counter: a `Cell`, so nothing but the critical section keeps
/// the two cores' updates apart.
static COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

/// Set by core 1 once its rounds are done.
static CORE1_DONE: AtomicBool = AtomicBool::new(false);

/// Core 1's stack, owned statically so it outlives the spawn.
static mut CORE1_STACK: Stack<0x4000> = Stack::new();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let p = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&p.GPIO, p.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    rpi_hal::halt();
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let p = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&p.GPIO, p.UART0);
    let _ = writeln!(uart, "multicore_critical_section");

    // 1. Nesting, on one core, before core 1 exists.
    let depth = critical_section::with(|_| critical_section::with(|_| 2));
    let _ = writeln!(uart, "nesting: PASS (reached depth {depth})");

    // 2. Exclusion, across two cores.
    // SAFETY: called once; the stack is a dedicated static handed to
    // exactly one spawn; `core1_main` never returns and touches only the
    // counter and the flag.
    unsafe {
        let cores = Cores::steal();
        cores
            .core1
            .spawn(&mut *addr_of_mut!(CORE1_STACK), core1_main);
    }
    add_rounds();
    while !CORE1_DONE.load(Ordering::Acquire) {
        core::hint::spin_loop();
    }

    let total = critical_section::with(|cs| COUNTER.borrow(cs).get());
    let expected = 2 * ROUNDS;
    let verdict = if total == expected { "PASS" } else { "FAIL" };
    let _ = writeln!(
        uart,
        "exclusion: {verdict} (counter {total}, expected {expected})"
    );
    rpi_hal::halt();
}

extern "C" fn core1_main() -> ! {
    add_rounds();
    CORE1_DONE.store(true, Ordering::Release);
    rpi_hal::halt();
}

/// Adds one to [`COUNTER`] [`ROUNDS`] times, each as a separate read and
/// write inside a nested critical section.
fn add_rounds() {
    for _ in 0..ROUNDS {
        critical_section::with(|outer| {
            let value = COUNTER.borrow(outer).get();
            // The window a broken lock would lose an update in.
            for _ in 0..20 {
                core::hint::spin_loop();
            }
            critical_section::with(|inner| COUNTER.borrow(inner).set(value + 1));
        });
    }
}
