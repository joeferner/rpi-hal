#![no_std]
#![no_main]

// Hardware entropy, exercised both ways it is reached: `entropy::fill`
// directly, and `getrandom`, which resolves to the same place only if the
// registration linked.
//
// The second is the one that needs an image to prove. A `getrandom` call
// with no backend fails to *link*, so a build that gets this far has
// already shown something; but a build that links and then returns zeros
// is the failure this is really watching for. `rpi_hal::rng` addresses a
// peripheral at a `PERIPHERAL_BASE` offset, so a chip feature naming the
// wrong SoC points it at an address the MMU has not mapped -- which is a
// fault on a good day and a FIFO that never fills on a bad one.
//
// Hence the sanity checks below rather than just a hex dump. They are
// deliberately crude: anything strong enough to be a real randomness test
// would be a statistics library, and what is worth catching here is the
// gross failure -- all zeros, a stuck word, an obvious bias -- not a
// subtle one.
//
// Build it with `scripts/build-example.sh entropy` (kernel7.img) or
// `scripts/build-example64.sh entropy` (kernel8.img).

use core::fmt::Write;

use rpi_hal::{pac, timer::Timer, uart::Uart};
use rpi_kickstart::{console, entropy, logln};
use static_cell::StaticCell;

static UART: StaticCell<Uart> = StaticCell::new();

/// How many bytes the bit-balance check draws. Large enough that a fair
/// source lands within about a percent of half the bits set, small enough
/// that the whole run is over in well under a second.
const SAMPLE_BYTES: usize = 4096;

/// The clock the console stamps with — see `examples/console.rs` for why
/// this is a `fn` that steals the peripheral rather than a closure.
fn uptime() -> u64 {
    Timer::new(unsafe { pac::Peripherals::steal() }.SYSTMR).now_micros()
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    rpi_hal::halt();
}

#[unsafe(no_mangle)]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    console::init(
        UART.init(Uart::init(&peripherals.GPIO, peripherals.UART0)),
        uptime,
    );

    logln!("rpi-kickstart entropy");

    // One word at a time, timed individually, because *which* fill blocks
    // is the question. `Rng::new` arms a discard of 262,144 samples and
    // documents the first read as waiting it out, so the expected shape
    // is one long duration followed by short ones.
    //
    // On a board whose generator is already running it is the other way
    // round: four words arrive in microseconds and the fifth blocks for
    // the warmup. Arming the discard does not flush what is already
    // queued, so those four come from whatever state the block was left
    // in -- by the firmware, or by the previous image. They are not
    // necessarily bad; the hardware presents a word only after some
    // discard has completed. But which discard is unknowable from here,
    // and that is the point of measuring it: the stall is real, it lands
    // in an unpredictable place, and the words ahead of it are the ones
    // no one can account for.
    let mut first = [0u8; 16];
    for (i, chunk) in first.chunks_mut(4).enumerate() {
        let start = uptime();
        entropy::fill(chunk);
        logln!("word {i}: {}us", uptime() - start);
    }

    let start = uptime();
    let mut second = [0u8; 16];
    entropy::fill(&mut second);
    logln!("next 4 words: {}us", uptime() - start);

    log_bytes("first ", &first);
    log_bytes("second", &second);

    // Two fills of the same length that come back identical mean a FIFO
    // that is not advancing -- the single most likely way for this to be
    // broken while still appearing to work.
    if first == second {
        logln!("FAIL: two fills returned the same bytes");
    }
    if first == [0u8; 16] {
        logln!("FAIL: first fill was all zeros");
    }

    // The `getrandom` path. Nothing here calls into this module by name,
    // which is the point: a crypto crate would reach the same generator
    // through exactly this call, and it only arrives if
    // `register_custom_getrandom!` resolved.
    let mut through_getrandom = [0u8; 16];
    match getrandom::getrandom(&mut through_getrandom) {
        Ok(()) => log_bytes("getrand", &through_getrandom),
        Err(e) => logln!("FAIL: getrandom returned {e:?}"),
    }

    // Bit balance over a larger sample. A fair source sets close to half
    // the bits; a stuck-at-zero or stuck-at-one block is nowhere near.
    let mut sample = [0u8; SAMPLE_BYTES];
    entropy::fill(&mut sample);
    let ones: u32 = sample.iter().map(|b| b.count_ones()).sum();
    let total = (SAMPLE_BYTES * 8) as u32;
    // Tenths of a percent, computed in integers -- this target has no
    // hardware float worth using and the precision is not needed.
    let per_mille = ones as u64 * 1000 / total as u64;
    logln!(
        "{ones}/{total} bits set ({}.{}%), want about 50%",
        per_mille / 10,
        per_mille % 10
    );

    logln!("done");
    rpi_hal::halt();
}

/// Logs `bytes` as hex under a label, so two fills can be compared by eye
/// as well as by the equality check above.
fn log_bytes(label: &str, bytes: &[u8]) {
    // `heapless` would be the tidy way to build the line, but a fixed
    // buffer is one dependency for one line of output. Writing the label
    // and the bytes as separate `logln!` arguments is not an option --
    // each call is its own stamped line -- so the hex is assembled with
    // the formatter's own repetition instead.
    let mut hex = [0u8; 48];
    for (i, byte) in bytes.iter().take(16).enumerate() {
        const DIGITS: &[u8; 16] = b"0123456789abcdef";
        hex[i * 3] = DIGITS[usize::from(byte >> 4)];
        hex[i * 3 + 1] = DIGITS[usize::from(byte & 0xf)];
        hex[i * 3 + 2] = b' ';
    }
    // The buffer is built entirely from ASCII above, so this cannot fail;
    // `from_utf8` rather than an unchecked conversion because the check
    // costs nothing here and the unsafe would have to be justified.
    let text = core::str::from_utf8(&hex).unwrap_or("<not utf-8>");
    logln!("{label}: {}", text.trim_end());
}
