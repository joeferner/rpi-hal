//! Turning an unhandled exception into a line on the console.
//!
//! A fault that parks silently is indistinguishable from a hang in a
//! driver, a deadlock, or a wedged peripheral, and the three are
//! debugged in completely different directions. This module is the weak
//! `__unhandled_exception` in `vectors.s`/`vectors64.s` replaced by one
//! that says what happened before it stops.
//!
//! Nothing here is public. The feature installs a symbol; there is no
//! API to call, and an application that wants its own report defines
//! `__unhandled_exception` itself with this feature off.
//!
//! # What it prints
//!
//! ```text
//! FAULT: data abort on core 0
//!   pc     0x0008a41c
//!   addr   0x000ffff8  write
//!   cause  permission fault, second level
//!   dfsr   0x0000080f   spsr 0x600001d3
//!   stack  0x00100000..0x00200000
//!   *** past the bottom of the stack: this is a stack overflow
//! ```
//!
//! The last line is the one a register dump cannot reach. A stack
//! overflow is the hardest fault to recognise from registers alone,
//! because every number in it looks ordinary -- the faulting address is
//! a plausible address, the fault is a plausible translation fault, and
//! nothing says which of the program's many pointers it was. Comparing
//! it against the region the linker script reserved answers that, and it
//! is a comparison the program can make and a person reading hex
//! generally cannot.
//!
//! It only fires if an overflow *faults*, which on these boards it does
//! not yet do: `mmu32`/`mmu64` identity-map all of RAM, so `sp`
//! descending past `__stack_bottom` walks into `__stack_slack` and then
//! into `.text` without the hardware objecting. `linker.ld` sizes and
//! 2 MiB-aligns that slack precisely so the `mmu` feature can leave it
//! as an invalid descriptor; until it does, an overflow is silent
//! corruption rather than a report. The check costs two comparisons and
//! is already correct for the day the guard exists.
//!
//! # The console it writes to
//!
//! UART0, re-initialised from scratch on GPIO14/15, because a fault
//! handler cannot assume the peripheral is in any particular state --
//! and the one program that most needs this is the one that faulted
//! while setting it up. The cost is that a board running UART0 somewhere
//! else, on a different pin set, gets its lines on GPIO14/15 instead.
//! That trade is only ever paid by a board that is already dead.
//!
//! Nothing here allocates, locks, or calls into a driver beyond putting
//! bytes in a FIFO. In particular it takes no critical section: the lock
//! guarding an application's console may well be held by the code that
//! just faulted, and blocking on it would replace the report with a
//! deadlock.

use core::fmt::Write as _;

use crate::pac;
use crate::uart::Uart;

/// Writes the report and parks the core.
///
/// Called from `fault.s`/`fault64.s`, which capture the state that does
/// not survive an ordinary function prologue and then tail-call here.
///
/// The arguments differ by architecture because what is perishable
/// differs: AArch32 has to hand over `lr` and the banked `spsr` before
/// anything can touch them, while AArch64 keeps the faulting address in
/// `ELR_EL1` and instead has to hand over the stack pointer it is about
/// to move off.
#[cfg(target_arch = "arm")]
#[no_mangle]
extern "C" fn rpi_hal_fault_report(kind: u32, lr: usize, spsr: u32) -> ! {
    let mut uart = console();

    // `lr` on entry is the faulting instruction plus a bias fixed by the
    // exception type: 8 for a data abort, 4 for everything else that
    // lands here. Undoing it here rather than printing the raw value and
    // a note is the difference between an address that can be looked up
    // in a disassembly and one that has to be adjusted first, correctly,
    // by whoever is reading at the time.
    let pc = lr.wrapping_sub(if kind == KIND_DATA_ABORT { 8 } else { 4 });

    let _ = writeln!(
        uart,
        "FAULT: {} on core {}",
        kind_name(kind),
        crate::cpu::core_id()
    );
    let _ = writeln!(uart, "  pc     {pc:#010x}");

    match kind {
        KIND_DATA_ABORT => {
            let (dfsr, dfar) = data_fault_registers();
            // Bit 11 says which direction the access was. Worth printing
            // in words: "write" immediately rules out every read in the
            // faulting line, which on a line with several is most of the
            // search.
            let direction = if dfsr & (1 << 11) != 0 {
                "write"
            } else {
                "read"
            };
            let _ = writeln!(uart, "  addr   {dfar:#010x}  {direction}");
            let _ = writeln!(uart, "  cause  {}", short_fault_name(status(dfsr)));
            let _ = writeln!(uart, "  dfsr   {dfsr:#010x}   spsr {spsr:#010x}");
            report_stack(&mut uart, &[dfar]);
        }
        KIND_PREFETCH_ABORT => {
            let (ifsr, ifar) = instruction_fault_registers();
            let _ = writeln!(uart, "  addr   {ifar:#010x}  fetch");
            let _ = writeln!(uart, "  cause  {}", short_fault_name(status(ifsr)));
            let _ = writeln!(uart, "  ifsr   {ifsr:#010x}   spsr {spsr:#010x}");
            report_stack(&mut uart, &[ifar]);
        }
        // An undefined instruction, an unexpected supervisor call or an
        // FIQ with nothing behind it. None of the three has a fault
        // address register to consult -- the address in `pc` above is
        // the whole of what the hardware recorded. The stack extent is
        // still worth printing: an overflow that corrupts a return
        // address arrives as an undefined instruction just as readily as
        // it arrives as a data abort.
        _ => {
            let _ = writeln!(uart, "  spsr   {spsr:#010x}");
            report_stack(&mut uart, &[]);
        }
    }

    crate::halt()
}

/// Writes the report and parks the core. See the AArch32 twin above.
#[cfg(target_arch = "aarch64")]
#[no_mangle]
extern "C" fn rpi_hal_fault_report(kind: u32, faulting_sp: usize) -> ! {
    let mut uart = console();

    let (esr, far, elr, spsr) = exception_registers();
    let class = (esr >> 26) & 0x3f;

    let _ = writeln!(
        uart,
        "FAULT: {} on core {}",
        kind_name(kind),
        crate::cpu::core_id()
    );
    // ELR_EL1 is the faulting instruction exactly, with none of
    // AArch32's per-exception bias to undo.
    let _ = writeln!(uart, "  pc     {elr:#018x}");

    // ESR is written by synchronous exceptions and by SError, and by
    // nothing else. An IRQ or FIQ arriving here leaves whatever the last
    // synchronous exception put in it, so printing it would be inventing
    // a diagnosis -- the slot number the vector passed is the only thing
    // that is actually known.
    let mut suspects = [faulting_sp, faulting_sp];
    if is_synchronous(kind) {
        if is_abort(class) {
            let direction = if class == EC_DATA_ABORT_SAME || class == EC_DATA_ABORT_LOWER {
                if esr & (1 << 6) != 0 {
                    "write"
                } else {
                    "read"
                }
            } else {
                "fetch"
            };
            let _ = writeln!(uart, "  addr   {far:#018x}  {direction}");
            let _ = writeln!(uart, "  cause  {}", long_fault_name(esr & 0x3f));
            // Only now is FAR_EL1 known to hold a faulting address.
            // Outside an abort it holds whatever the last one left, and
            // testing that against the stack would invent an overflow.
            suspects[0] = far as usize;
        }
        let _ = writeln!(uart, "  class  {}", class_name(class));
        let _ = writeln!(uart, "  esr    {esr:#010x}   spsr {spsr:#010x}");
    }

    let _ = writeln!(uart, "  sp     {faulting_sp:#018x}");
    report_stack(&mut uart, &suspects);

    crate::halt()
}

/// The UART the report goes to.
///
/// Stolen rather than borrowed, because a fault handler is reached from
/// anywhere and owns nothing. This is the one place in the crate where
/// that is unconditionally correct: every other user of these
/// peripherals has already stopped running and will never resume.
fn console() -> Uart {
    // SAFETY: the core is on its way to `halt` and nothing it was doing
    // will continue, so no other holder of these peripherals can observe
    // the aliasing.
    let peripherals = unsafe { pac::Peripherals::steal() };
    Uart::init(&peripherals.GPIO, peripherals.UART0)
}

/// Names the region the linker script reserved for the main stack, and
/// says so when one of `suspects` has left it.
///
/// The extent is printed for every fault rather than only for a
/// suspected overflow, because it is what makes an address meaningful: a
/// number a few KiB below `__stack_bottom` means something entirely
/// different from the same number in a program whose stack is elsewhere.
/// A reader who has the extent can reach the conclusion themselves on
/// the faults this does not reach it for.
///
/// `suspects` is whatever the architecture actually knows. AArch64 has
/// two — the faulting address and the stack pointer at the moment of the
/// fault — and they are the same event seen from both ends. AArch32 has
/// only the first: the handler runs on the banked abort stack, so its
/// own `sp` says nothing about the one that overflowed.
fn report_stack(uart: &mut Uart, suspects: &[usize]) {
    let (bottom, top) = (crate::stack::bottom(), crate::stack::top());
    let _ = writeln!(uart, "  stack  {bottom:#010x}..{top:#010x}");

    // Below the region and within a stack's worth of it. An address far
    // below is an ordinary wild pointer that happens to be numerically
    // smaller, and calling that an overflow would send a reader in the
    // wrong direction with confidence.
    let overflowed = suspects
        .iter()
        .any(|&address| address < bottom && bottom - address <= crate::stack::size());
    if overflowed {
        let _ = writeln!(
            uart,
            "  *** past the bottom of the stack: this is a stack overflow"
        );
    }
}

/// Vector index for an AArch32 data abort. See `vectors.s`.
#[cfg(target_arch = "arm")]
const KIND_DATA_ABORT: u32 = 4;
/// Vector index for an AArch32 prefetch abort. See `vectors.s`.
#[cfg(target_arch = "arm")]
const KIND_PREFETCH_ABORT: u32 = 3;

/// What the vector index means, in words.
#[cfg(target_arch = "arm")]
fn kind_name(kind: u32) -> &'static str {
    match kind {
        1 => "undefined instruction",
        2 => "supervisor call",
        3 => "prefetch abort",
        4 => "data abort",
        7 => "FIQ with no handler",
        _ => "unknown exception",
    }
}

/// What the vector index means, in words. `group << 2 | type`, as
/// `vectors64.s` encodes it.
#[cfg(target_arch = "aarch64")]
fn kind_name(kind: u32) -> &'static str {
    let group = match kind >> 2 {
        0 => "at EL1 on SP_EL0",
        1 => "at EL1",
        2 => "from a lower EL (AArch64)",
        _ => "from a lower EL (AArch32)",
    };
    // Written as one string per combination rather than assembled from
    // two, because `core::fmt` here would mean another frame on a stack
    // this may have very little of.
    match (kind & 3, group) {
        (0, g) => match g {
            "at EL1 on SP_EL0" => "synchronous exception at EL1 on SP_EL0",
            "at EL1" => "synchronous exception at EL1",
            "from a lower EL (AArch64)" => "synchronous exception from a lower EL (AArch64)",
            _ => "synchronous exception from a lower EL (AArch32)",
        },
        (1, _) => "IRQ with no handler",
        (2, _) => "FIQ with no handler",
        _ => "SError",
    }
}

/// Whether this vector slot is one the hardware wrote `ESR_EL1` for.
#[cfg(target_arch = "aarch64")]
fn is_synchronous(kind: u32) -> bool {
    // Synchronous (type 0) and SError (type 3) both set it; IRQ and FIQ
    // do not.
    matches!(kind & 3, 0 | 3)
}

/// Exception class for a data abort taken without a change in EL.
#[cfg(target_arch = "aarch64")]
const EC_DATA_ABORT_SAME: u64 = 0x25;
/// Exception class for a data abort taken from a lower EL.
#[cfg(target_arch = "aarch64")]
const EC_DATA_ABORT_LOWER: u64 = 0x24;

/// Whether this exception class carries a fault address in `FAR_EL1`
/// and a fault status in the low bits of `ESR_EL1`.
#[cfg(target_arch = "aarch64")]
fn is_abort(class: u64) -> bool {
    matches!(
        class,
        0x20 | 0x21 | EC_DATA_ABORT_LOWER | EC_DATA_ABORT_SAME
    )
}

/// `ESR_EL1.EC`, in words. Only the classes a kernel with no EL0 code
/// can realistically reach are named; the rest print as a number, which
/// is enough to look up.
#[cfg(target_arch = "aarch64")]
fn class_name(class: u64) -> &'static str {
    match class {
        0x00 => "unknown reason",
        0x0e => "illegal execution state",
        0x15 => "supervisor call",
        0x18 => "trapped system register access",
        0x20 | 0x21 => "instruction abort",
        0x22 => "PC alignment fault",
        0x24 | 0x25 => "data abort",
        0x26 => "SP alignment fault",
        0x2c => "floating-point exception",
        0x2f => "SError interrupt",
        0x3c => "breakpoint instruction",
        _ => "see ESR_EL1.EC",
    }
}

/// The AArch32 short-descriptor fault status, assembled from the two
/// places the encoding keeps it: `FS[3:0]` in the low bits and `FS[4]`
/// up at bit 10.
#[cfg(target_arch = "arm")]
fn status(fsr: u32) -> u32 {
    ((fsr >> 10) & 1) << 4 | (fsr & 0xf)
}

/// A short-descriptor fault status, in words.
///
/// The level matters as much as the kind: a first-level translation
/// fault is a wholly unmapped megabyte, which usually means a pointer
/// that was never valid, while a second-level one is a page missing from
/// a section that does exist.
#[cfg(target_arch = "arm")]
fn short_fault_name(fs: u32) -> &'static str {
    match fs {
        0x01 => "alignment fault",
        0x02 => "debug event",
        0x03 => "access flag fault, first level",
        0x04 => "instruction cache maintenance fault",
        0x05 => "translation fault, first level",
        0x06 => "access flag fault, second level",
        0x07 => "translation fault, second level",
        0x08 => "synchronous external abort",
        0x09 => "domain fault, first level",
        0x0b => "domain fault, second level",
        0x0c => "external abort on translation table walk, first level",
        0x0d => "permission fault, first level",
        0x0e => "external abort on translation table walk, second level",
        0x0f => "permission fault, second level",
        0x16 => "asynchronous external abort",
        0x18 => "asynchronous parity error",
        0x19 => "synchronous parity error",
        _ => "see DFSR/IFSR",
    }
}

/// An AArch64 `DFSC`/`IFSC`, in words.
#[cfg(target_arch = "aarch64")]
fn long_fault_name(status: u64) -> &'static str {
    match status {
        0x00..=0x03 => "address size fault",
        0x04 => "translation fault, level 0",
        0x05 => "translation fault, level 1",
        0x06 => "translation fault, level 2",
        0x07 => "translation fault, level 3",
        0x08..=0x0b => "access flag fault",
        0x0c => "permission fault, level 0",
        0x0d => "permission fault, level 1",
        0x0e => "permission fault, level 2",
        0x0f => "permission fault, level 3",
        0x10 => "synchronous external abort",
        0x14..=0x17 => "external abort on translation table walk",
        0x18 => "synchronous parity error",
        0x1c..=0x1f => "parity error on translation table walk",
        0x21 => "alignment fault",
        0x30 => "TLB conflict abort",
        _ => "see ESR_EL1.DFSC",
    }
}

/// `DFSR` and `DFAR`: why the data access faulted, and at what address.
#[cfg(target_arch = "arm")]
fn data_fault_registers() -> (u32, usize) {
    let (dfsr, dfar): (u32, usize);
    // SAFETY: two CP15 reads into general-purpose registers. Both exist
    // on every core this crate targets (ARMv7-A and the ARM1176), no
    // memory is touched and no state changes.
    unsafe {
        core::arch::asm!(
            "mrc p15, 0, {dfsr}, c5, c0, 0",
            "mrc p15, 0, {dfar}, c6, c0, 0",
            dfsr = out(reg) dfsr,
            dfar = out(reg) dfar,
            options(nomem, nostack, preserves_flags),
        );
    }
    (dfsr, dfar)
}

/// `IFSR` and `IFAR`: why the instruction fetch faulted, and at what
/// address.
///
/// Read only on the prefetch-abort path. The two abort types share a
/// CPSR mode and so used to share a report, which meant printing both
/// register pairs and leaving a reader to work out which half was
/// meaningful; the vector index in `vectors.s` is what made it possible
/// to read only the pair that is.
#[cfg(target_arch = "arm")]
fn instruction_fault_registers() -> (u32, usize) {
    let (ifsr, ifar): (u32, usize);
    // SAFETY: as above. `IFAR` (c6, c0, 2) is architectural from ARMv6
    // onwards, so it is present on the ARM1176 as well as on ARMv7-A.
    unsafe {
        core::arch::asm!(
            "mrc p15, 0, {ifsr}, c5, c0, 1",
            "mrc p15, 0, {ifar}, c6, c0, 2",
            ifsr = out(reg) ifsr,
            ifar = out(reg) ifar,
            options(nomem, nostack, preserves_flags),
        );
    }
    (ifsr, ifar)
}

/// `ESR_EL1`, `FAR_EL1`, `ELR_EL1` and `SPSR_EL1` — what happened, where,
/// which instruction, and the state it happened in.
#[cfg(target_arch = "aarch64")]
fn exception_registers() -> (u64, u64, u64, u64) {
    let (esr, far, elr, spsr): (u64, u64, u64, u64);
    // SAFETY: four system-register reads into general-purpose registers.
    // No memory is touched and no state changes.
    unsafe {
        core::arch::asm!(
            "mrs {esr}, esr_el1",
            "mrs {far}, far_el1",
            "mrs {elr}, elr_el1",
            "mrs {spsr}, spsr_el1",
            esr = out(reg) esr,
            far = out(reg) far,
            elr = out(reg) elr,
            spsr = out(reg) spsr,
            options(nomem, nostack, preserves_flags),
        );
    }
    (esr, far, elr, spsr)
}
