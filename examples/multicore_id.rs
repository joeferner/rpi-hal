#![no_std]
#![no_main]

//! `cpu::core_id`: one entry function, four cores, each identifying itself.
//!
//! `Core1::spawn` and its siblings take a bare `extern "C" fn() -> !` with
//! no argument, so a function spawned on three cores is handed nothing that
//! distinguishes them -- which makes this the case `core_id` exists for.
//! The same `core_main` below runs on cores 1, 2 and 3, and everything it
//! does per core it does by asking which core it is.
//!
//! It also stands in for the shape real code takes: the `__irq_handler` and
//! the panic handler are single symbols shared by every core, so anything
//! they record per core has to be indexed the same way. The panic handler
//! here names the core it died on for exactly that reason.
//!
//! The output is a self-check, not just a printout. Each core writes into
//! `TICKS[core_id()]`, so distinct ids are what make all three slots
//! advance: an id that came back the same on two cores would leave one slot
//! at zero and give another double the count, and an id out of range would
//! panic on the index rather than corrupt something quietly.
//!
//! The rates pin the ids down rather than merely showing they differ. Each
//! core's delay is proportional to the id it read, so the three counts
//! advance in the ratio 1 : 1/2 : 1/3 -- roughly 11, 5 and 4 per report.
//! Only the values 1, 2 and 3 produce that spacing; three distinct but
//! wrong ids would not.
//!
//! The first report reads all zeros. Core 0 reaches its loop while the
//! others are still in their trampolines bringing up their own MMU and
//! caches, so the first line is printed before anyone has arrived to count.
//!
//! Requires the `multicore` feature:
//!
//! ```sh
//! ./scripts/build-example.sh multicore_id      # kernel7.img, AArch32
//! ./scripts/build-example64.sh multicore_id    # kernel8.img, AArch64
//! ```

use core::fmt::Write;
use core::sync::atomic::{AtomicU32, Ordering};

use rpi_hal::cpu::core_id;
use rpi_hal::multicore::{Cores, Stack};
use rpi_hal::{pac, uart::Uart};

/// Loop counts, indexed by the id each core reads for itself. Slot 0 stays
/// zero: core 0 reports rather than counts.
static TICKS: [AtomicU32; 4] = [const { AtomicU32::new(0) }; 4];

/// A stack per secondary core. All three run the same entry function, so
/// the only thing that has to differ between the spawns is which stack each
/// one is given.
static mut CORE1_STACK: Stack<0x4000> = Stack::new();
static mut CORE2_STACK: Stack<0x4000> = Stack::new();
static mut CORE3_STACK: Stack<0x4000> = Stack::new();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let p = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&p.GPIO, p.UART0);
    // Which core panicked is the first thing worth knowing and the hardest
    // to recover afterwards: with four cores sharing this handler, a
    // message without an id leaves it ambiguous.
    let _ = writeln!(uart, "PANIC on core {}: {info}", core_id());
    loop {
        unsafe { core::arch::asm!("wfe") };
    }
}

/// Runs on cores 1, 2 and 3 -- the same code on all three.
///
/// The delay is scaled by the core's own id purely so the three counters
/// advance at visibly different rates: equal counts would be consistent
/// with one core writing all three slots, and unequal ones are not.
extern "C" fn core_main() -> ! {
    let id = core_id();
    loop {
        TICKS[id].fetch_add(1, Ordering::SeqCst);
        delay(2_000_000 * id as u32);
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let p = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&p.GPIO, p.UART0);

    let _ = writeln!(uart, "multicore_id: this is core {}", core_id());

    // SAFETY: `Cores::steal` is called once, each stack is a dedicated
    // static handed to exactly one spawn, `core_main` never returns, and the
    // only state it shares with this core is `TICKS` -- atomics, each slot
    // written by one core alone.
    unsafe {
        let stack1 = &raw mut CORE1_STACK;
        let stack2 = &raw mut CORE2_STACK;
        let stack3 = &raw mut CORE3_STACK;

        let cores = Cores::steal();
        cores.core1.spawn(&mut *stack1, core_main);
        cores.core2.spawn(&mut *stack2, core_main);
        cores.core3.spawn(&mut *stack3, core_main);
    }

    loop {
        let _ = writeln!(
            uart,
            "core0: ticks core1={} core2={} core3={}",
            TICKS[1].load(Ordering::SeqCst),
            TICKS[2].load(Ordering::SeqCst),
            TICKS[3].load(Ordering::SeqCst),
        );
        delay(20_000_000);
    }
}

fn delay(cycles: u32) {
    for _ in 0..cycles {
        unsafe { core::arch::asm!("nop") };
    }
}
