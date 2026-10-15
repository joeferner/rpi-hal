#![no_std]
#![no_main]

// Auxiliary-SPI (SPI1) self-test: jumper GPIO20 (SPI1 MOSI) to GPIO19
// (SPI1 MISO) so every byte written shifts straight back in as the byte
// read. Needs no external device — the aux-SPI counterpart of
// `spi_loopback.rs`, which does the same for SPI0.
//
// Reports PASS/FAIL over UART0 (already verified by `uart_hello.rs`, so
// trusted as the output channel here) and blinks the LED on GPIO4 as a
// redundant indicator: slow = pass, fast = fail, matching this project's
// established fast-blink-means-problem convention.

use core::fmt::Write;
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal::spi::{SpiBus, MODE_0};
use rpi_hal::aux_spi::{AuxSpi, ChipSelect};
use rpi_hal::gpio::{Input, Pin};
use rpi_hal::halt;
use rpi_hal::{pac, uart::Uart};

const TEST_PATTERN: [u8; 8] = [0x00, 0xff, 0xa5, 0x5a, 0x01, 0x80, 0x7e, 0x3c];

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
    let mut led = Pin::<4, Input>::new(unsafe { pac::GPIO::steal() }).into_output();

    // ChipSelect::None: a loopback wire has no device to select, so no CE
    // line is driven. `speed` is left deliberately unverified against a
    // real target frequency (see `AuxSpi::init_spi1`) — any value here
    // just needs to be slow enough for the loopback wire to settle.
    let mut spi = AuxSpi::init_spi1(
        &peripherals.GPIO,
        &peripherals.AUX,
        peripherals.SPI1,
        MODE_0,
        ChipSelect::None,
        250,
    );

    let mut buf = TEST_PATTERN;
    let _ = spi.transfer_in_place(&mut buf);

    // Per-byte comparison, not just a final verdict: an all-zero (or
    // all-0xff) `recv` column points at MISO floating (jumper missing or
    // not making contact) rather than a driver bug, since a real wiring
    // fault reads back a fixed idle level on every byte regardless of
    // what was sent. A scrambled but non-constant pattern would instead
    // point at the shift loop or FIFO timing in `src/aux_spi.rs`.
    for (i, (&sent, &recv)) in TEST_PATTERN.iter().zip(buf.iter()).enumerate() {
        let _ = writeln!(
            uart,
            "[{i}] sent=0x{sent:02x} recv=0x{recv:02x}{}",
            if sent == recv { "" } else { "  <-- mismatch" }
        );
    }

    let pass = buf == TEST_PATTERN;
    let _ = writeln!(
        uart,
        "aux SPI (SPI1) loopback: {}",
        if pass { "PASS" } else { "FAIL" }
    );

    let period_cycles = if pass { 500_000 } else { 100_000 };
    loop {
        let _ = led.toggle();
        delay(period_cycles);
    }
}

fn delay(cycles: u32) {
    for _ in 0..cycles {
        unsafe { core::arch::asm!("nop") };
    }
}
