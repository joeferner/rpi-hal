#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::halt;
use rpi_hal::{pac, rng::Rng, timer::Timer, uart::Uart};

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
    let timer = Timer::new(peripherals.SYSTMR);
    let mut rng = Rng::new();

    loop {
        let word = rng.next_u32();
        let mut buf = [0u8; 8];
        rng.fill_bytes(&mut buf);
        let _ = writeln!(uart, "u32 = {word:#010x}  bytes = {buf:02x?}");
        timer.delay_ms(1000);
    }
}
