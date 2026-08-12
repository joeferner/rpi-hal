#![no_std]
#![no_main]

// Auxiliary-SPI (SPI1) full-duplex check against a real external SPI
// *slave* — the way to verify the MISO/receive path that a MOSI->MISO
// jumper loopback (`aux_spi_loopback.rs`) can't, since a jumper only ever
// echoes what this board just sent.
//
// Pi SPI1 (master) drives a 4-byte transfer: it shifts TX_BYTES out on
// MOSI and reads whatever the slave shifts back on MISO. With the slave
// armed to reply EXPECTED_MISO, a correct receive path reads exactly those
// bytes back. It retries every ~3s (fast-blinking, so there's time to arm
// the fixture) until it sees the armed reply, then latches to a steady
// slow blink and stops driving the bus. The fixture's `arm` is one-shot,
// so only the first transfer after each arm carries the real reply — the
// latch means that single clean PASS is the last thing printed rather than
// being buried under exhausted-reply noise.
//
// Wiring — Pi SPI1 (ALT4) to the slave:
//   Pi GPIO21 (SCLK) -> slave SCK
//   Pi GPIO20 (MOSI) -> slave MOSI (slave data in)
//   Pi GPIO19 (MISO) <- slave MISO (slave data out)
//   Pi GPIO18 (CE0)  -> slave NSS/CS
//   GND <-> GND
//
// Fixture side (e.g. bench-link's SPI-slave mode over its USB console):
//   m spis            # enter SPI-slave mode
//   0                 # SPI mode 0 (must match — the aux SPI only does
//                     #   CPHA=0, i.e. modes 0 and 2; this uses mode 0)
//   arm 4 DE AD BE EF # expect 4 bytes, reply 0xDE AD BE EF on MISO
// then watch this board's UART for the next transfer. Back on the fixture,
//   p                 # should report the 4 TX_BYTES this board sent (MOSI)
// verifying both directions. Re-`arm` before each subsequent attempt.
//
// Reports over UART0 (trusted from uart_hello.rs) and blinks GPIO4:
// slow = the received bytes matched EXPECTED_MISO, fast = mismatch.

use core::fmt::Write;
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal::spi::{SpiBus, MODE_0};
use rpi_hal::aux_spi::{AuxSpi, ChipSelect};
use rpi_hal::gpio::{Input, Pin};
use rpi_hal::timer::Timer;
use rpi_hal::{pac, uart::Uart};

/// Bytes this board shifts out on MOSI. Distinct, non-trivial values so
/// the fixture's `poll` dump is easy to eyeball.
const TX_BYTES: [u8; 4] = [0x01, 0x02, 0x03, 0x04];

/// Bytes the slave should shift back on MISO — must match what the fixture
/// was `arm`ed to reply. A correct receive path reads exactly these back.
const EXPECTED_MISO: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];

/// `CNTL0.SPEED` divider — ~100kHz at the 250MHz default core clock.
/// Deliberately slow so the slave comfortably keeps its MISO byte set up
/// between clocked bytes; raise it once the path is confirmed.
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

    // ChipSelect::Cs0 drives CE0 (GPIO18) as the slave's NSS, held asserted
    // for the whole 4-byte transfer. MODE_0 because the aux SPI can't
    // generate CPHA=1 (see `AuxSpi::init_spi1`); the fixture must match.
    let mut spi = AuxSpi::init_spi1(
        &peripherals.GPIO,
        &peripherals.AUX,
        peripherals.SPI1,
        MODE_0,
        ChipSelect::Cs0,
        SPEED,
    );

    let _ = writeln!(
        uart,
        "\naux-SPI slave check: TX={:02x?}, expecting MISO={:02x?}",
        TX_BYTES, EXPECTED_MISO
    );
    let _ = writeln!(
        uart,
        "arm the fixture with: m spis / 0 / arm 4 {:02x} {:02x} {:02x} {:02x}",
        EXPECTED_MISO[0], EXPECTED_MISO[1], EXPECTED_MISO[2], EXPECTED_MISO[3]
    );

    // Retry until the slave replies with the armed bytes.
    loop {
        let mut rx = [0u8; TX_BYTES.len()];
        let _ = spi.transfer(&mut rx, &TX_BYTES);

        if rx == EXPECTED_MISO {
            let _ = writeln!(
                uart,
                "TX={:02x?} MISO={:02x?} PASS — MISO receive verified",
                TX_BYTES, rx
            );
            break;
        }

        let _ = writeln!(
            uart,
            "TX={:02x?} MISO={:02x?} — waiting; (re-)arm the fixture",
            TX_BYTES, rx
        );
        // ~3s of fast blink between attempts, long enough to (re-)arm.
        for _ in 0..12 {
            let _ = led.toggle();
            timer.delay_ms(250);
        }
    }

    // Verified: steady slow blink, no more bus traffic.
    loop {
        let _ = led.toggle();
        timer.delay_ms(500);
    }
}
