#![no_std]
#![no_main]

// Aux-SPI (SPI1) CPOL/CPHA verification fixture for a logic analyzer.
//
// Emits the byte 0xB4 on SPI1 in each of the four SPI modes, back to back,
// so one short capture checks the CPOL/CPHA mapping in `AuxSpi::configure`
// against hardware. The whole 4-mode burst is packed into ~3ms (with
// microsecond inter-byte/inter-mode gaps) so it fits a short capture
// window, repeating every ~2s.
//
// Probe these SPI1 pins (ALT4):
//   SCLK = GPIO21, MOSI = GPIO20, CE0 = GPIO18.
// MISO/GPIO19 is unused — this is a transmit-only pattern.
//
// Each mode emits a distinct *number* of bytes so the captured group's
// byte count identifies the mode with no ambiguity about ordering:
//   mode 0 -> 1 byte, mode 1 -> 2, mode 2 -> 3, mode 3 -> 4.
// Each byte is one CE0-framed transfer, so count the CE0 pulses per group.
//
// The byte is 0xB4 = 0b1011_0100 (MSB first: 1,0,1,1,0,1,0,0).
//
// Expected result (this is what the capture confirmed, and why the driver
// documents CPHA=1 as unsupported): decode MOSI MSB-first, sampling on the
// mode's sampling edge (leading for CPHA=0, trailing for CPHA=1).
//
//   Mode | CPOL | CPHA | SCLK idle | expect | note
//   -----+------+------+-----------+--------+-----------------------------
//     0  |  0   |  0   |   low     |  0xB4  | CPHA=0 works
//     1  |  0   |  1   |   low     |  0x68  | CPHA=1: 0xB4 shifted one bit
//     2  |  1   |  0   |   high    |  0xB4  | CPHA=0 works
//     3  |  1   |  1   |   high    |  0x68  | CPHA=1: 0xB4 shifted one bit
//
// The 0x68 in modes 1/3 is the aux SPI's inability to generate CPHA=1
// (it presents the first bit at CS-assert, one bit early) — see
// `src/aux_spi.rs`. A change that made modes 1/3 read 0xB4 would mean that
// limitation had somehow been lifted; modes 0/2 must stay 0xB4.

use core::fmt::Write;
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal::spi::{Mode, SpiBus, MODE_0, MODE_1, MODE_2, MODE_3};
use rpi_hal::aux_spi::{AuxSpi, ChipSelect};
use rpi_hal::gpio::{Input, Pin};
use rpi_hal::timer::Timer;
use rpi_hal::{pac, uart::Uart};

/// The byte shifted out in every mode. 0b1011_0100.
const TEST_BYTE: u8 = 0xB4;

/// `CNTL0.SPEED` divider — SPI clock is `core_clock / (2 * (SPEED + 1))`,
/// so ~100kHz at the 250MHz default core clock. Deliberately slow so a
/// modest logic analyzer captures every edge cleanly with margin to
/// spare; the exact frequency doesn't matter for a CPOL/CPHA check.
const SPEED: u16 = 1249;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    rpi_hal::halt();
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut led = Pin::<4, Input>::new(unsafe { pac::GPIO::steal() }).into_output();

    let _ = writeln!(
        uart,
        "\naux-SPI mode probe: byte=0x{TEST_BYTE:02x}, pins SCLK=GPIO21 MOSI=GPIO20 CE0=GPIO18"
    );
    let _ = writeln!(
        uart,
        "burst per mode by byte count: m0=1 m1=2 m2=3 m3=4; expect 0xB4 in m0/m2, 0x68 in m1/m3"
    );

    // (label, mode, byte-count == mode index + 1) for each of the four.
    let modes: [(&str, Mode, usize); 4] = [
        ("MODE 0 (CPOL0/CPHA0)", MODE_0, 1),
        ("MODE 1 (CPOL0/CPHA1)", MODE_1, 2),
        ("MODE 2 (CPOL1/CPHA0)", MODE_2, 3),
        ("MODE 3 (CPOL1/CPHA1)", MODE_3, 4),
    ];

    // The whole 4-mode burst is packed into ~3ms with microsecond gaps so
    // it fits a short logic-analyzer capture window, and each mode emits a
    // distinct *number* of bytes (1/2/3/4) so the captured group's byte
    // count identifies which mode it is with no ambiguity about ordering.
    // A long idle between bursts lets the analyzer arm/trigger on one clean
    // burst.
    loop {
        let _ = writeln!(uart, "spi burst");

        for &(_label, mode, count) in &modes {
            // Reconfigure SPI1 for this mode. The controller's mode is
            // fixed at init, so re-steal the token and re-init per mode —
            // fine for a test fixture; re-muxing the pins each time is
            // idempotent. CE0 is driven so each byte is framed on the
            // analyzer.
            let spi1 = unsafe { pac::SPI1::steal() };
            let mut spi = AuxSpi::init_spi1(
                &peripherals.GPIO,
                &peripherals.AUX,
                spi1,
                mode,
                ChipSelect::Cs0,
                SPEED,
            );

            // `count` bytes for this mode (1/2/3/4 = mode + 1), each its
            // own write() so CE0 pulses once per byte and the pulse count
            // reads back the mode. ~150us between bytes keeps them separate.
            for _ in 0..count {
                let _ = spi.write(&[TEST_BYTE]);
                timer.delay_us(150);
            }

            let _ = led.toggle();
            // Inter-mode gap, clearly larger than the inter-byte gap.
            timer.delay_us(400);
        }

        // Long idle between bursts so the analyzer can trigger on the next
        // ~3ms burst and capture all four modes in one short window.
        timer.delay_ms(2000);
    }
}
