#![no_std]
#![no_main]

//! AArch64 smoke test for rpi-hal, running on the crate's own `rt` boot
//! sequence (now that `rt` supports AArch64). It exercises the full 64-bit
//! runtime path end to end:
//!
//! 1. `rt`'s `_start` (boot64.s) drops EL2->EL1, installs the exception
//!    vectors, and brings up the identity-mapped MMU + caches before
//!    calling `kmain`; this reports the resulting exception level;
//! 2. an atomic-counter loop -- `ldxr`/`stxr` only make forward progress
//!    against cacheable Normal memory, so this succeeding is a direct check
//!    that `rt`'s MMU/cache setup is correct;
//! 3. a VideoCore mailbox loop -- with the D-cache on, this exercises the
//!    D-cache maintenance in rpi-hal's `cache` module (clean before the
//!    VideoCore reads, invalidate before we read its response), so correct
//!    values here validate cache *coherency*, not just that the code runs.
//!
//! Nothing here is AArch64-specific except the exception-level readout, so
//! it also builds and runs as an ordinary AArch32 example.
//!
//! Build and run through rpi-loader (a 64-bit `kernel8.img` loader):
//!
//! ```sh
//! ./scripts/build-example64.sh aarch64_smoke bcm2711   # -> target/kernel8.img
//! # from the rpi-loader checkout, with the board already running the loader.
//! # The load address matches linker64.ld's 0x80000, not the 32-bit 0x8000:
//! python3 scripts/upload.py <device> boot --load-addr 0x80000 \
//!     path/to/rpi-hal/target/kernel8.img
//! ```

use core::fmt::Write;
use core::sync::atomic::{AtomicU32, Ordering};
use rpi_hal::halt;
use rpi_hal::mailbox::{ClockId, Mailbox};
use rpi_hal::{pac, uart::Uart};

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

    let _ = writeln!(uart, "aarch64_smoke: rpi-hal running via rt");

    // rt has already dropped to EL1 and enabled the MMU/caches by now; read
    // the exception level back as confirmation the EL2->EL1 drop happened.
    #[cfg(target_arch = "aarch64")]
    {
        let current_el: u64;
        unsafe { core::arch::asm!("mrs {0}, CurrentEL", out(reg) current_el) };
        let _ = writeln!(uart, "exception level: EL{}", (current_el >> 2) & 0b11);
    }

    // With caches on, exclusive accesses must make forward progress; if the
    // MMU/cache setup were wrong, this loop would spin forever rather than
    // print 5.
    static COUNTER: AtomicU32 = AtomicU32::new(0);
    for _ in 0..5 {
        COUNTER.fetch_add(1, Ordering::SeqCst);
    }
    let _ = writeln!(
        uart,
        "atomic counter (expect 5): {}",
        COUNTER.load(Ordering::SeqCst)
    );

    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // Re-query every iteration: with the D-cache on, each call really
    // exercises the clean/invalidate maintenance around the DMA buffers, so
    // values staying correct call after call is the coherency check.
    let mut count: u32 = 0;
    loop {
        let _ = writeln!(uart, "-- mailbox query {count} --");

        match mailbox.firmware_revision() {
            Ok(rev) => {
                let _ = writeln!(uart, "firmware revision: 0x{rev:08x}");
            }
            Err(e) => {
                let _ = writeln!(uart, "firmware revision: error {e:?}");
            }
        }

        match mailbox.board_serial() {
            Ok(serial) => {
                let _ = writeln!(uart, "board serial:      0x{serial:016x}");
            }
            Err(e) => {
                let _ = writeln!(uart, "board serial:      error {e:?}");
            }
        }

        match mailbox.arm_memory() {
            Ok(region) => {
                let _ = writeln!(
                    uart,
                    "ARM memory:        {} MiB at 0x{:08x}",
                    region.size_bytes / (1024 * 1024),
                    region.base_address
                );
            }
            Err(e) => {
                let _ = writeln!(uart, "ARM memory:        error {e:?}");
            }
        }

        match mailbox.clock_rate_hz(ClockId::Arm) {
            Ok(hz) => {
                let _ = writeln!(uart, "ARM clock:         {} MHz", hz / 1_000_000);
            }
            Err(e) => {
                let _ = writeln!(uart, "ARM clock:         error {e:?}");
            }
        }

        count += 1;
        delay(20_000_000);
    }
}

fn delay(cycles: u32) {
    for _ in 0..cycles {
        unsafe { core::arch::asm!("nop") };
    }
}
