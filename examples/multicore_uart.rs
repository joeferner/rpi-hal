#![no_std]
#![no_main]

//! Multicore smoke test with UART output. Core 0 spawns core 1 on a
//! background counter, then prints the counter from core 0. Because core 1
//! increments a shared atomic and core 0 reads it, seeing the value climb
//! proves the whole secondary-core path: the wake handoff, the secondary's
//! own EL2->EL1 drop + MMU/cache bring-up, and cross-core cache coherency
//! (each core's `rpi_hal_mmu_init` maps RAM Normal/Cacheable/Shareable, so
//! the atomic is actually visible between cores).
//!
//! Unlike `multicore_blink` (LED only), this narrates over UART, so it
//! works for bring-up on a board without a wired LED -- e.g. validating
//! the AArch64 spin-table path in rpi-hal's `multicore`/`secondary64.s`.
//!
//! Requires the `multicore` feature. On AArch64, build and upload through
//! rpi-loader:
//!
//! ```sh
//! ./scripts/build-example64.sh multicore_uart
//! python3 <rpi-loader>/scripts/upload.py --load-addr 0x8000 <device> \
//!     target/kernel8-example.img
//! ```

use core::fmt::Write;
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};
use rpi_hal::multicore::{Cores, Stack};
use rpi_hal::{pac, uart::Uart};

/// Incremented by core 1, read by core 0 -- the cross-core coherency check.
static CORE1_TICKS: AtomicU32 = AtomicU32::new(0);

/// Core 1's stack (16 KiB), owned statically so it outlives the spawn.
static mut CORE1_STACK: Stack<0x4000> = Stack::new();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let p = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&p.GPIO, p.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    loop {
        unsafe { core::arch::asm!("wfe") };
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let p = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&p.GPIO, p.UART0);

    let _ = writeln!(uart, "multicore_uart: core0 up, spawning core1");

    // SAFETY: called once; CORE1_STACK is a dedicated static handed to
    // exactly one spawn; core1_main never returns and only touches the
    // shared atomic.
    unsafe {
        let cores = Cores::steal();
        cores
            .core1
            .spawn(&mut *addr_of_mut!(CORE1_STACK), core1_main);
    }

    loop {
        let ticks = CORE1_TICKS.load(Ordering::SeqCst);
        let _ = writeln!(uart, "core0: core1 ticks = {ticks}");
        delay(20_000_000);
    }
}

/// Runs on core 1: just bumps the shared counter forever.
extern "C" fn core1_main() -> ! {
    loop {
        CORE1_TICKS.fetch_add(1, Ordering::SeqCst);
        delay(2_000_000);
    }
}

fn delay(cycles: u32) {
    for _ in 0..cycles {
        unsafe { core::arch::asm!("nop") };
    }
}
