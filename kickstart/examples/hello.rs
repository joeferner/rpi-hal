#![no_std]
#![no_main]

// The smallest whole program this repository can produce: boot on
// `rpi-hal`'s `rt`, bring up the UART, and say so.
//
// It exists to prove the parts of the build that have nothing to do with
// what the crate does -- that the toolchain, the target, `rpi-link.x` and
// the two load addresses are all wired up, and that an image built here
// runs on a board. Those are the things that break once and then stay
// broken, and a greeting is enough to see all of them.
//
// Build it for either architecture with `scripts/build-example.sh hello`
// or `scripts/build-example64.sh hello`.

use core::fmt::Write;

use rpi_hal::{halt, pac, timer::Timer, uart::Uart};

// This example takes no features, so the library it links against has no
// modules compiled into it and there is nothing here to call -- see
// `console.rs` for an example that uses one. Named anyway because a crate
// nothing refers to is never linked, and this crate is destined to
// install things by linkage the way `rpi-hal-embassy`'s time driver does.
// So this stays the image that proves the scaffolding on its own: if it
// boots, nothing about the toolchain, the targets or the linker script is
// what broke.
use rpi_kickstart as _;

// A library cannot supply this: a program may have exactly one, so it is
// the application's, and an example is an application. The console is the
// only place a bare-metal panic can go, so it is reinitialized here rather
// than shared -- whatever owned the UART is exactly what may have just
// panicked.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

// `unsafe(no_mangle)` rather than the bare `no_mangle` `rpi-hal`'s own
// examples use: this crate is edition 2024, which requires the wrapper.
#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);

    let _ = writeln!(uart, "rpi-kickstart: hello, world");
    let _ = writeln!(
        uart,
        "arch: {}",
        if cfg!(target_arch = "aarch64") {
            "aarch64"
        } else {
            "aarch32"
        }
    );

    // A heartbeat rather than a single line and a halt: one line proves
    // only that the image reached `kmain`, while a count that keeps rising
    // proves it is still running -- which is the difference between a
    // board that booted and a board that booted and then faulted.
    let mut seconds = 0u32;
    loop {
        timer.delay_ms(1000);
        seconds += 1;
        let _ = writeln!(uart, "up {seconds}s");
    }
}
