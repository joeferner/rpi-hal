#![no_std]
#![no_main]

use core::fmt::Write;
use embedded_hal::i2c::I2c as _;
use rpi_hal::halt;
use rpi_hal::{i2c::I2c, pac, uart::Uart};

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
    let mut i2c = I2c::<pac::BSC1>::init(&peripherals.GPIO, peripherals.BSC1, 0x05dc);

    loop {
        let _ = writeln!(uart, "scanning I2C1 (GPIO2/3)...");

        // 0x03-0x77: the conventional 7-bit scan range, excluding the
        // reserved 0x00-0x02 and 0x78-0x7f blocks (general call,
        // 10-bit addressing, and other reserved patterns).
        let mut found = 0;
        for addr in 0x03..=0x77u8 {
            // A 1-byte read, not a 0-byte write: confirmed the hard
            // way on real hardware that BCM2835's BSC doesn't drive a
            // real bus transaction at all for `DLEN=0` -- it reported
            // every single address in this range as "found" (see
            // `i2c::Error::ZeroLengthUnsupported`'s doc comment, added
            // after exactly this happened). A real 1-byte transfer
            // forces the hardware through an actual address phase, so
            // a NAK gets detected correctly. The byte read back is
            // discarded -- this only cares whether the address ACKs.
            let mut probe = [0u8];
            if i2c.read(addr, &mut probe).is_ok() {
                let _ = writeln!(uart, "  found device at 0x{addr:02x}");
                found += 1;
            }
        }

        let _ = writeln!(uart, "done: {found} device(s) found\n");
        delay(150_000_000);
    }
}

fn delay(cycles: u32) {
    for _ in 0..cycles {
        unsafe { core::arch::asm!("nop") };
    }
}
