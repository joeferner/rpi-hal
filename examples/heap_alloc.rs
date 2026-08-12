#![no_std]
#![no_main]

//! Dynamic memory allocation on bare metal: wiring up a `#[global_allocator]`
//! so the `alloc` crate (`Box`, `Vec`, `String`, `BTreeMap`, ...) works.
//!
//! `rpi-hal` itself is `#![no_std]` with no allocator, and stays that way --
//! a HAL can't define a `#[global_allocator]`, because a program may only
//! have one and that choice belongs to the final binary. So the allocator
//! lives here, in the application:
//!
//!   1. Pick a heap allocator crate (`embedded-alloc` here) and register it
//!      with `#[global_allocator]`.
//!   2. Give it a region of RAM to hand out. This example uses everything
//!      from the end of `.bss` (`__bss_end`, from the linker script) up to
//!      the top of the ARM/VideoCore memory split, which the VideoCore
//!      firmware reports via the mailbox. The whole ARM region below the
//!      peripheral base (`0x3F00_0000`) is identity-mapped as cacheable
//!      Normal memory by `rpi-hal`'s MMU bring-up, so it's all safe to use.
//!   3. `extern crate alloc;` and go.
//!
//! `embedded-alloc` takes its lock through `critical-section`, for which
//! `rpi-hal` provides an implementation under its default `rt` feature --
//! so it drops in with no extra wiring.
//!
//! Output goes to UART0 (PL011) at 115200 8N1.

extern crate alloc;

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::Write;

use embedded_alloc::LlffHeap as Heap;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pac;
use rpi_hal::uart::Uart;

/// The global heap. `empty()` is a `const` constructor, so the allocator can
/// be a `static` -- but it hands out nothing until `init` gives it a region
/// (any allocation before that panics), which is why the first thing `kmain`
/// does is set it up.
#[global_allocator]
static HEAP: Heap = Heap::empty();

extern "C" {
    /// End of the `.bss` section, defined by `linker.ld`. Only its address
    /// is meaningful -- the byte itself is never read.
    static __bss_end: u8;
}

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

    // Heap starts just past everything the linker statically placed. Reading
    // the symbol's address (not the symbol) is the whole point; `&raw const`
    // avoids ever forming a reference to a byte we never actually own.
    let heap_start = &raw const __bss_end as usize;

    // The top of the heap is the top of the ARM side of the memory split.
    // Asking the firmware (rather than hardcoding, say, 1 GiB) means the same
    // binary sizes its heap correctly whatever `gpu_mem` the board is set to
    // and however much RAM it has.
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    let region = match mailbox.arm_memory() {
        Ok(region) => region,
        Err(e) => {
            let _ = writeln!(uart, "could not read ARM memory size: {e:?}");
            halt();
        }
    };
    let heap_end = (region.base_address + region.size_bytes) as usize;
    let heap_size = heap_end - heap_start;

    let _ = writeln!(uart, "heap: {} KiB at 0x{heap_start:08x}", heap_size / 1024);

    // Safe to call exactly once, before any allocation, with a region that
    // isn't used for anything else. `heap_start`..`heap_end` is above `.bss`
    // and below the peripheral base, and the main stack grows *down* from the
    // 0x8000 load address (below all of this), so nothing else claims it.
    unsafe { HEAP.init(heap_start, heap_size) };

    // From here on, `alloc` just works.

    // Box: a single heap-allocated value.
    let boxed = Box::new(0xC0FFEEu32);
    let _ = writeln!(uart, "Box<u32>        = 0x{:x}", *boxed);

    // Vec: a growable array. Push past its initial capacity to force a
    // reallocation, exercising the allocator's realloc path.
    let mut squares: Vec<u32> = Vec::new();
    for n in 1..=8u32 {
        squares.push(n * n);
    }
    let _ = writeln!(uart, "Vec<u32>        = {squares:?}");

    // String: owned, growable UTF-8, built with the `format!`-style writer.
    let mut greeting = String::new();
    let _ = write!(greeting, "sum of squares = {}", squares.iter().sum::<u32>());
    let _ = writeln!(uart, "String          = \"{greeting}\"");

    // BTreeMap: an ordered map, to show a non-trivial collection working.
    let mut counts: BTreeMap<char, u32> = BTreeMap::new();
    for c in "raspberry pi".chars().filter(|c| c.is_alphabetic()) {
        *counts.entry(c).or_insert(0) += 1;
    }
    let _ = writeln!(uart, "BTreeMap        = {counts:?}");

    // Trait objects need a heap (or an arena) to be stored uniformly; a
    // `Vec<Box<dyn Trait>>` is the canonical example of why alloc is useful.
    let shapes: Vec<Box<dyn Area>> =
        alloc::vec![Box::new(Circle { r: 2.0 }), Box::new(Square { side: 3.0 })];
    let total: f32 = shapes.iter().map(|s| s.area()).sum();
    let _ = writeln!(uart, "Vec<Box<dyn>>   = total area {total}");

    let _ = writeln!(uart, "done.");
    halt();
}

/// Toy trait to demonstrate a heterogeneous `Vec<Box<dyn Area>>`.
trait Area {
    fn area(&self) -> f32;
}

struct Circle {
    r: f32,
}

impl Area for Circle {
    fn area(&self) -> f32 {
        core::f32::consts::PI * self.r * self.r
    }
}

struct Square {
    side: f32,
}

impl Area for Square {
    fn area(&self) -> f32 {
        self.side * self.side
    }
}

fn halt() -> ! {
    loop {
        unsafe { core::arch::asm!("wfe") };
    }
}
