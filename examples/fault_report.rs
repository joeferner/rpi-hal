//! Faults on purpose, to show what the `fault-report` feature prints.
//!
//! Run with `--features bcm2837,fault-report`. Without the feature the
//! same program still faults and the vector table's weak handler parks
//! the core silently, which is the comparison worth seeing once: the
//! board goes quiet after `about to fault` and nothing else is ever
//! printed. That silence is indistinguishable from a hang in a driver,
//! and telling those apart is the whole reason the feature exists.
//!
//! Expected output on a Pi 3, AArch32, with the feature on:
//!
//! ```text
//! rpi-hal: fault_report example
//! stack 0x400000..0x500000, 1023 KiB free
//! about to fault: read from 0xf0000000, which is unmapped
//! FAULT: data abort on core 0
//!   pc     0x00008364
//!   addr   0xf0000000  read
//!   cause  translation fault, first level
//!   dfsr   0x00000005   spsr 0x600001d3
//!   stack  0x00400000..0x00500000
//! ```
//!
//! And the same program on AArch64, where `ELR_EL1` needs no un-biasing
//! and `ESR_EL1` carries the class and the fault status together:
//!
//! ```text
//! FAULT: synchronous exception at EL1 on core 0
//!   pc     0x0000000000080270
//!   addr   0x00000000f0000000  read
//!   cause  translation fault, level 2
//!   class  data abort
//!   esr    0x96000006   spsr 0x600003c5
//!   sp     0x00000000004fff90
//!   stack  0x00400000..0x00500000
//! ```
//!
//! The `sp` there is the faulting one, not the handler's: the entry stub
//! captures it before moving `sp` to a stack of its own, which is what
//! lets the report survive the overflow case that would otherwise fault
//! again on its first push.
//!
//! The `pc` is the faulting instruction itself, already un-biased, so it
//! can be looked up directly:
//!
//! ```text
//! arm-none-eabi-objdump -d target/.../examples/fault_report | grep 811c
//! ```
//!
//! # Choosing which fault to raise
//!
//! [`FAULT`] below selects one. Each reaches the handler through a
//! different vector slot, which is what the report's first line names —
//! and on AArch32 a data abort and a prefetch abort arrive in the *same*
//! CPU mode, so that first line is information the handler has no other
//! way to obtain.

#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::{halt, pac, uart::Uart};

/// Which fault the example raises. Edit and rebuild to see another.
const FAULT: Fault = Fault::UnmappedRead;

/// An address with no translation behind it.
///
/// `rpi_hal::mmu` identity-maps RAM up to the peripheral base, the
/// peripheral block above that, and the ARM-local block above that --
/// on a Pi 2/3 the last mapped byte is `0x400f_ffff`. Everything higher
/// is left as an invalid descriptor, which is what makes this a
/// translation fault rather than a read of something.
///
/// **Not address zero.** Reading through a null pointer does not fault
/// here: section 0 is ordinary identity-mapped RAM like any other, so a
/// null dereference quietly returns whatever the firmware left at
/// physical zero. There is no guard page, and nothing on these boards
/// arranges one -- a null pointer bug on this platform corrupts memory
/// instead of announcing itself.
const UNMAPPED: usize = 0xf000_0000;

/// The faults this example knows how to cause on purpose.
///
/// `dead_code` is allowed because [`FAULT`] names exactly one of them
/// and the others are then unconstructed by definition. Deleting the
/// unused ones is the alternative, and it would mean this example could
/// only ever demonstrate the fault someone last edited it to.
#[allow(dead_code)]
enum Fault {
    /// Reads [`UNMAPPED`]. Arrives as a data abort with a first-level
    /// translation fault.
    UnmappedRead,
    /// Calls into [`UNMAPPED`]. Arrives as a prefetch abort — the fetch
    /// faults, not a data access — which is the case the report can
    /// only distinguish by vector slot, both being CPSR mode 0x17.
    BadJump,
    /// Recurses without bound until `sp` runs off the end of the region
    /// the linker script reserved, which faults on the store that first
    /// reaches below `__stack_bottom` -- into the guard the `mmu`
    /// feature leaves unmapped there. The report's last line is the one
    /// that matters here.
    StackOverflow,
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt()
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);

    let _ = writeln!(uart, "rpi-hal: fault_report example");
    let _ = writeln!(
        uart,
        "stack {:#x}..{:#x}, {} KiB free",
        rpi_hal::stack::bottom(),
        rpi_hal::stack::top(),
        rpi_hal::stack::headroom().unwrap_or(0) / 1024
    );

    match FAULT {
        Fault::UnmappedRead => {
            let _ = writeln!(
                uart,
                "about to fault: read from {UNMAPPED:#x}, which is unmapped"
            );
            // The announcement has to have left the FIFO before the fault,
            // or the handler -- which re-initialises the peripheral, and
            // does not wait for a transmission in progress -- interleaves
            // its first bytes with it.
            uart.flush();
            // SAFETY: not safe, and deliberately so -- this is the fault
            // the example exists to raise.
            let value = unsafe { core::ptr::read_volatile(UNMAPPED as *const u32) };
            let _ = writeln!(uart, "unreachable: read {value:#x}");
        }
        Fault::BadJump => {
            let _ = writeln!(uart, "about to fault: call into {UNMAPPED:#x}");
            uart.flush();
            let target: extern "C" fn() = unsafe { core::mem::transmute(UNMAPPED) };
            target();
        }
        Fault::StackOverflow => {
            let _ = writeln!(uart, "about to fault: recurse until the stack runs out");
            uart.flush();
            let _ = consume(0);
        }
    }

    halt()
}

/// Recurses with a frame large enough to reach the end of a 1 MiB stack
/// in about a thousand calls.
///
/// Both `black_box` calls are load-bearing, and the second one more than
/// the first. Without something opaque touching `frame` *after* the
/// recursive call, the frame is dead across it: LLVM tail-call-eliminates
/// the recursion into a branch and shrinks the array to a few bytes, and
/// the result is a function that spins forever on one 16-byte frame
/// without ever growing the stack. That is not a hypothetical -- it is
/// what this example compiled to, and what it spent a hardware run
/// silently proving nothing with. An earlier `write_volatile` on one
/// element was not enough: it kept the store, not the frame.
///
/// `#[inline(never)]` is the easy half of the same requirement.
#[inline(never)]
#[allow(unconditional_recursion)]
fn consume(depth: u32) -> u32 {
    let mut frame = [0u32; 256];
    frame[0] = depth;
    // The frame has to really exist...
    core::hint::black_box(&mut frame);

    let deeper = consume(depth + 1);

    // ...and has to still be live here, which is what stops the call
    // above from becoming a branch.
    core::hint::black_box(&mut frame);
    deeper.wrapping_add(frame[0])
}
