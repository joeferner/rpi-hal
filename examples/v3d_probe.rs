//! V3D 3D pipeline bring-up probe.
//!
//! Enables the V3D block via the mailbox and reads back its
//! identification registers — no control lists, no shaders, just "is
//! the block alive and clocked once enabled". The bare-metal
//! equivalent of `camera_probe.rs`'s "can we identify and talk to the
//! sensor" check, one step before any real 3D work.
//!
//! `ident()`'s raw values aren't checked against an expected magic
//! here: this crate's confidence in the exact `IDENT0`/`1`/`2` bit
//! layout isn't solid enough yet to assert against (see
//! `rpi_hal::v3d`'s doc comments) — compare the printed words by hand
//! against Broadcom's public 3D Architecture Reference Guide, or a
//! known-good capture from a real Pi 3 running Linux's `vc4` driver.

#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::halt;
use rpi_hal::mailbox::{ClockId, Mailbox};
use rpi_hal::pac;
use rpi_hal::uart::Uart;
use rpi_hal::v3d::V3d;

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
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // The 3D core clock has to be running before the block does real
    // work, and nothing brings it up on its own here -- see `rpi_hal::v3d`'s
    // module documentation. Harmless for a pure identification-register
    // read like this one (those answer without it), but this example is
    // the reference for the bring-up order, so it does it properly.
    match mailbox.set_clock_rate_hz(ClockId::V3d, 250_000_000) {
        Ok(hz) => {
            let _ = writeln!(uart, "v3d clock set to: {hz} Hz");
        }
        Err(e) => {
            let _ = writeln!(uart, "v3d clock set failed: {e:?}");
        }
    }

    match mailbox.set_enable_qpu(true) {
        Ok(()) => {
            let _ = writeln!(uart, "set_enable_qpu(true): ok");
        }
        Err(e) => {
            let _ = writeln!(uart, "set_enable_qpu(true): error {e:?}");
            let _ = writeln!(
                uart,
                "continuing anyway -- registers may just read back garbage"
            );
        }
    }

    // SAFETY: single-threaded `kmain`; only one `V3d` is constructed,
    // here.
    let v3d = unsafe { V3d::new() };
    let (ident0, ident1, ident2) = v3d.ident();
    let _ = writeln!(uart, "IDENT0: 0x{ident0:08x}");
    let _ = writeln!(uart, "IDENT1: 0x{ident1:08x}");
    let _ = writeln!(uart, "IDENT2: 0x{ident2:08x}");

    halt();
}
