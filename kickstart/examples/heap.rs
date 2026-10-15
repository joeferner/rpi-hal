#![no_std]
#![no_main]

// The global heap: how much of the board there is to allocate, that
// `alloc` types work once it has been handed over, and that a second
// `init` reports instead of quietly corrupting the first.
//
// The size is the interesting number. It is not a constant of the board —
// the VideoCore takes its share first, set by `gpu_mem` in `config.txt`,
// and what is left is what the firmware reports. An image that hardcoded
// it would be wrong on a board with a different split, and wrong in the
// direction that hands the allocator memory the GPU is also using, which
// shows up as corruption somewhere else entirely.
//
// Build it with `scripts/build-example.sh heap` (kernel7.img) or
// `scripts/build-example64.sh heap` (kernel8.img).

extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::Write as _;

use rpi_hal::{mailbox::Mailbox, pac, timer::Timer, uart::Uart};
use rpi_kickstart::{console, heap, logln};
use static_cell::StaticCell;

static UART: StaticCell<Uart> = StaticCell::new();

/// The clock the console stamps with — see `examples/console.rs`.
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

    logln!("rpi-kickstart heap");

    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    let bytes = match heap::init(&mut mailbox) {
        Ok(bytes) => bytes,
        Err(e) => {
            logln!("FAIL: heap::init returned {e:?}");
            rpi_hal::halt();
        }
    };
    logln!("heap: {bytes} bytes ({} MiB)", bytes / (1024 * 1024));

    // Nothing above this line could have allocated — the allocator panics
    // on an empty region — so reaching here at all is half the result.
    let mut numbers: Vec<u32> = Vec::new();
    for i in 0..1000 {
        numbers.push(i * 3);
    }
    logln!(
        "vec: {} items, last {}, sum {}",
        numbers.len(),
        numbers[numbers.len() - 1],
        numbers.iter().copied().map(u64::from).sum::<u64>()
    );

    let mut text = String::new();
    let _ = write!(text, "formatted into {} bytes of heap", numbers.len() * 4);
    logln!("string: {text}");

    // Repeated mixed-size allocate and free, which is the workload TLSF
    // was chosen for. What this is watching for is not a wrong answer but
    // a hang: a fragmenting allocator degrades rather than fails, so the
    // evidence is that the last round is no slower than the first.
    for round in 0..4 {
        let start = uptime();
        let mut blocks: Vec<Vec<u8>> = Vec::new();
        for size in 1..=200 {
            blocks.push(alloc::vec![0u8; size * 17]);
        }
        // Drop every other one first, so the freed blocks are interleaved
        // with live ones rather than contiguous — a first-fit allocator
        // handles the contiguous case well and this one badly.
        blocks.retain(|b| b.len().is_multiple_of(2));
        for size in 1..=100 {
            blocks.push(alloc::vec![0u8; size * 23]);
        }
        let held: usize = blocks.iter().map(Vec::len).sum();
        logln!(
            "round {round}: {} blocks, {held} bytes held, {}us",
            blocks.len(),
            uptime() - start
        );
    }

    // The guard. A board with two pieces of bring-up that each believe
    // they own the heap gets an error rather than two live allocations
    // pointing at the same bytes.
    match heap::init(&mut mailbox) {
        Err(heap::Error::AlreadyInitialized) => logln!("second init: refused, as it should be"),
        Ok(n) => logln!("FAIL: second init handed over {n} bytes again"),
        Err(e) => logln!("FAIL: second init returned {e:?}"),
    }

    logln!("done");
    rpi_hal::halt();
}
