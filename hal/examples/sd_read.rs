#![no_std]
#![no_main]

// SD card bring-up smoke test: brings up the EMMC host controller,
// runs the card identification sequence, and reads block 0 (the boot
// sector / MBR, on any card that's ever been partitioned). Prints the
// card's addressing mode, the first 16 bytes, and the two bytes at the
// very end of the block -- 0x55 0xAA there is the standard boot
// signature, an easy human-verifiable check that a real block actually
// came back, not just zeros or noise.

use core::fmt::Write;
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::sd::Sd;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;

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
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    let _ = writeln!(uart, "initializing SD card...");
    let emmc = unsafe { Sd::steal_emmc() };
    let sd = match Sd::init(&peripherals.GPIO, emmc, &mut mailbox, &timer) {
        Ok(sd) => sd,
        Err(e) => {
            let _ = writeln!(uart, "SD init failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(
        uart,
        "SD card ready ({})",
        if sd.high_capacity() {
            "SDHC/SDXC, block addressing"
        } else {
            "SDSC, byte addressing"
        }
    );

    let mut block = [0u8; 512];
    match sd.read_block(0, &mut block, &timer) {
        Ok(()) => {
            let _ = writeln!(uart, "block 0 read OK");
            let _ = writeln!(uart, "first 16 bytes: {:02x?}", &block[..16]);
            let _ = writeln!(
                uart,
                "boot signature (bytes 510-511): {:02x} {:02x}{}",
                block[510],
                block[511],
                if block[510] == 0x55 && block[511] == 0xaa {
                    " -- matches the standard 0x55 0xAA signature"
                } else {
                    " -- does not match 0x55 0xAA (unpartitioned card, or something's wrong)"
                }
            );
        }
        Err(e) => {
            let _ = writeln!(uart, "block 0 read failed: {e:?}");
        }
    }

    halt();
}
