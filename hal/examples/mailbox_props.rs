#![no_std]
#![no_main]

// Mailbox property-interface smoke test: queries a handful of
// VideoCore-reported facts (firmware revision, board identity, the
// ARM/VC memory split, and a couple of clock rates) and prints them
// over UART. Nothing here is timing-sensitive or expected to change
// between reads, but it re-queries every loop iteration anyway --
// simplest way to confirm the mailbox itself keeps working call after
// call, not just once at boot.

use core::fmt::Write;
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
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    loop {
        let _ = writeln!(uart, "-- VideoCore mailbox properties --");

        match mailbox.firmware_revision() {
            Ok(rev) => {
                let _ = writeln!(uart, "firmware revision: 0x{rev:08x}");
            }
            Err(e) => {
                let _ = writeln!(uart, "firmware revision: error {e:?}");
            }
        }

        match mailbox.board_revision() {
            Ok(rev) => {
                let _ = writeln!(uart, "board revision:    0x{rev:08x}");
            }
            Err(e) => {
                let _ = writeln!(uart, "board revision:    error {e:?}");
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
                    "ARM memory:        base 0x{:08x}, size 0x{:08x} ({} MiB)",
                    region.base_address,
                    region.size_bytes,
                    region.size_bytes / (1024 * 1024)
                );
            }
            Err(e) => {
                let _ = writeln!(uart, "ARM memory:        error {e:?}");
            }
        }

        match mailbox.vc_memory() {
            Ok(region) => {
                let _ = writeln!(
                    uart,
                    "VC memory:         base 0x{:08x}, size 0x{:08x} ({} MiB)",
                    region.base_address,
                    region.size_bytes,
                    region.size_bytes / (1024 * 1024)
                );
            }
            Err(e) => {
                let _ = writeln!(uart, "VC memory:         error {e:?}");
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

        match mailbox.clock_rate_hz(ClockId::Core) {
            Ok(hz) => {
                let _ = writeln!(uart, "core clock:        {} MHz", hz / 1_000_000);
            }
            Err(e) => {
                let _ = writeln!(uart, "core clock:        error {e:?}");
            }
        }

        let _ = writeln!(uart);
        delay(150_000_000);
    }
}

fn delay(cycles: u32) {
    for _ in 0..cycles {
        unsafe { core::arch::asm!("nop") };
    }
}
