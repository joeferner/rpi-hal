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
    /// the linker script reserved.
    ///
    /// **This currently produces no report at all**, and that is worth
    /// seeing rather than worth hiding. Nothing below the stack is
    /// unmapped -- `linker.ld` reserves `__stack_slack` beneath it and
    /// says the region is 2 MiB-aligned so the `mmu` feature *can later*
    /// leave it invalid, which it does not yet do -- so `sp` descending
    /// past `__stack_bottom` crosses no boundary the hardware objects
    /// to. It walks the slack, reaches `.text`/`.data`, and overwrites
    /// the running program. Observed: the announcement below, then
    /// silence, then a spontaneous reboot.
    ///
    /// That is the failure mode the whole report exists to replace, and
    /// it is the one case it cannot reach until an unmapped guard region
    /// exists. When one does, this becomes a data abort just below
    /// `__stack_bottom` and the report's last line names it.
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
/// quickly, and touches it so the compiler cannot elide it.
///
/// `#[inline(never)]` because the whole point is a call, and the
/// volatile write is what stops the recursion becoming a loop: LLVM will
/// otherwise turn a tail-recursive function with an unused frame into
/// something that never grows the stack at all, and the example silently
/// stops testing anything.
///
/// The `unconditional_recursion` lint is exactly right about this
/// function and exactly wrong about whether it is a mistake: running off
/// the end of the stack is the behaviour being demonstrated.
#[inline(never)]
#[allow(unconditional_recursion)]
fn consume(depth: u32) -> u32 {
    let mut frame = [0u32; 256];
    // SAFETY: `frame` is a live local; the write is volatile only to
    // keep it from being optimised away.
    unsafe { core::ptr::write_volatile(&mut frame[0], depth) };
    consume(depth + 1).wrapping_add(frame[0])
}
