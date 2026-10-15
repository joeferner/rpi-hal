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

/// Times the first few words out of the generator, then streams words
/// and byte fills forever.
///
/// The timing is the interesting half, and it is worth running twice:
/// once from a power-on, and once by reloading this image over a UART
/// loader onto a board that has already run it. The block stays enabled
/// across a warm reload, and the two runs should look the same.
///
/// `Rng::new` arms a discard of 262,144 samples, which takes the
/// hardware roughly 0.74 s, and empties the FIFO so that nothing queued
/// before that discard can escape. So the expected shape is a
/// constructor that returns in microseconds, a first word that carries
/// the whole warmup, and later words that come straight out of the FIFO.
///
/// Without that drain the warm run came out inverted — four queued words
/// in 4-6 µs each and the 0.74 s stall on the fifth. That is why the
/// `WORDS` loop below times each word separately instead of timing one
/// bulk fill: the stall is easy to see either way, but *which* word pays
/// it is the thing being measured.
#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);

    /// How many words to time individually. More than the FIFO can hold,
    /// so a stall parked behind queued words would show up here rather
    /// than after the loop had finished.
    const WORDS: usize = 8;

    let start = timer.now_micros();
    let mut rng = Rng::new();
    let construct_us = timer.now_micros() - start;
    let _ = writeln!(uart, "Rng::new: {construct_us}us");

    let mut first_us = 0;
    let mut worst_rest_us = 0;
    for i in 0..WORDS {
        let start = timer.now_micros();
        let word = rng.next_u32();
        let elapsed = timer.now_micros() - start;
        let _ = writeln!(uart, "word {i}: {elapsed}us ({word:#010x})");
        if i == 0 {
            first_us = elapsed;
        } else {
            worst_rest_us = worst_rest_us.max(elapsed);
        }
    }

    // A word after the first that takes as long as the first did means
    // the warmup had not been waited out when `next_u32` first returned,
    // which is the failure this example exists to catch.
    if worst_rest_us * 2 >= first_us {
        let _ = writeln!(
            uart,
            "FAIL: warmup did not land on word 0 (worst later word {worst_rest_us}us \
             vs {first_us}us)"
        );
    } else {
        let _ = writeln!(uart, "warmup landed on word 0, as documented");
    }

    loop {
        let word = rng.next_u32();
        let mut buf = [0u8; 8];
        rng.fill_bytes(&mut buf);
        let _ = writeln!(uart, "u32 = {word:#010x}  bytes = {buf:02x?}");
        timer.delay_ms(1000);
    }
}
