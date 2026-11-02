//! Where RP1 sits in the CPU's address map.
//!
//! Every RP1 driver in this crate reaches its registers through
//! [`pac::RP1_BASE`], the CPU address of RP1's peripheral window. That
//! window is not fixed by the hardware: it is whatever CPU range `pcie2`'s
//! outbound window 0 covers, and the bootloader in the board's EEPROM
//! decides that when it brings the link up. Bootloaders up to 2024-09-23
//! map CPU `0x1f_0000_0000` to PCIe address 0, where they also put RP1's
//! peripheral BAR. From 2024-10-10 on they start PCIe memory at 2 GiB
//! instead: RP1's BAR moves to PCIe `0x8000_0000` and the window to CPU
//! `0x1c_0000_0000`, so nothing answers at the old address and the first
//! access there stalls the core with no fault.
//!
//! [`init`] puts the window back at [`pac::RP1_BASE`], keeping the PCIe
//! address the bootloader chose and so RP1's BAR with it. `rt`'s boot
//! sequence calls it before `kmain`; a program supplying its own boot
//! sequence calls it before its first RP1 access.
//!
//! [`pac::RP1_BASE`]: crate::pac::RP1_BASE

use crate::pac::{RP1_BASE, SOC_BASE};

/// `pcie2`'s register file (`pcie2`'s `reg`, bus `0x0012_0000` on the SoC
/// bus), a Broadcom `brcm,bcm2712-pcie` controller (Linux's
/// `pcie-brcmstb`), whose register offsets the constants below follow.
const PCIE2: usize = SOC_BASE + 0x0012_0000;

/// Outbound window 0's CPU base and limit, in MiB: the low 12 bits of each,
/// base in bits 15:4 and limit in bits 31:20.
const MEM_WIN0_BASE_LIMIT: usize = 0x4070;
/// The bits of the base and limit, in MiB, above those in
/// [`MEM_WIN0_BASE_LIMIT`].
const MEM_WIN0_BASE_HI: usize = 0x4080;
const MEM_WIN0_LIMIT_HI: usize = 0x4084;

/// Reads the `pcie2` register at `offset`.
fn read(offset: usize) -> u32 {
    // SAFETY: an aligned register in `pcie2`'s register file, which is on
    // the SoC's own bus and always readable.
    unsafe { core::ptr::read_volatile((PCIE2 + offset) as *const u32) }
}

/// Writes the `pcie2` register at `offset`.
fn write(offset: usize, value: u32) {
    // SAFETY: as `read`.
    unsafe { core::ptr::write_volatile((PCIE2 + offset) as *mut u32, value) }
}

/// Moves `pcie2`'s outbound window 0, and with it RP1's peripheral window,
/// to [`pac::RP1_BASE`] if the bootloader put it anywhere else.
///
/// Keeps the window's size and the PCIe address it maps to, so RP1's BARs
/// stay where the bootloader assigned them. Does nothing if the window is
/// already there, or if it is disabled (limit below base), which is how
/// `pciex4_reset=1` (the default without that `config.txt` line) leaves
/// it: RP1 is unreachable then whatever this does.
///
/// Idempotent, and must run before the first access to anything behind
/// RP1. `rt`'s boot sequence calls it before `kmain`, so an application
/// using the default features never needs to.
///
/// [`pac::RP1_BASE`]: crate::pac::RP1_BASE
pub fn init() {
    let base_limit = read(MEM_WIN0_BASE_LIMIT);
    let base_mb = (read(MEM_WIN0_BASE_HI) << 12) | ((base_limit >> 4) & 0xfff);
    let limit_mb = (read(MEM_WIN0_LIMIT_HI) << 12) | (base_limit >> 20);
    let target_mb = (RP1_BASE >> 20) as u32;
    if base_mb == target_mb || limit_mb < base_mb {
        return;
    }
    let new_limit_mb = target_mb + (limit_mb - base_mb);
    // Disable the window (limit below base) while the high and low halves
    // disagree, so it never decodes a range that is half old, half new.
    // Nothing reaches RP1 before this runs, so nothing sees the gap.
    write(MEM_WIN0_BASE_LIMIT, 0xfff0);
    write(MEM_WIN0_BASE_HI, target_mb >> 12);
    write(MEM_WIN0_LIMIT_HI, new_limit_mb >> 12);
    write(
        MEM_WIN0_BASE_LIMIT,
        ((new_limit_mb & 0xfff) << 20) | ((target_mb & 0xfff) << 4),
    );
    // Make sure the window has moved before the caller's first RP1 access.
    // SAFETY: barriers only.
    unsafe { core::arch::asm!("dsb sy", "isb") };
}
