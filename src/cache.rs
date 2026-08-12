//! Internal D-cache maintenance for the DMA-driven drivers.
//!
//! The mailbox, SD, touch, and USB paths hand buffers to bus masters
//! (the VideoCore, the EMMC controller, the DWC2 USB core) that read and
//! write RAM directly, bypassing this core's data cache. Before telling
//! one of them to read an ARM-written buffer, [`clean_range`] writes the
//! covering lines back to RAM; before reading a buffer one of them wrote,
//! [`invalidate_range`] drops the stale cached copies so the read comes
//! from RAM. Each ends with a barrier so the maintenance has completed
//! before the DMA is kicked off or the data is read.
//!
//! Consolidated here (rather than a private copy per driver) so the one
//! architecture-specific piece — the cache-maintenance instructions —
//! lives in a single place.

use core::arch::asm;

/// Conservative minimum D-cache line size, in bytes.
///
/// The true line size is discoverable at runtime (the `CTR` register's
/// `DminLine` field), but walking a buffer in strides this small is
/// always safe: on hardware with a bigger line (this SoC's Cortex-A7 and
/// Cortex-A53 both use 64 bytes) a clean/invalidate-by-address op affects
/// the whole line containing the address, so a smaller stride just
/// touches each line more than once — harmless — where a stride larger
/// than the true line size could skip one.
pub(crate) const MIN_CACHE_LINE: u32 = 32;

/// Cleans (writes back, without invalidating) every cache line covering
/// `len` bytes from `address`, then barriers — makes an ARM-side write
/// actually visible in RAM before a bus master is told to read it.
pub(crate) fn clean_range(address: u32, len: usize) {
    let end = address + len as u32;
    let mut line = address & !(MIN_CACHE_LINE - 1);
    while line < end {
        unsafe { clean_line(line) };
        line += MIN_CACHE_LINE;
    }
    barrier();
}

/// Invalidates every cache line covering `len` bytes from `address`, then
/// barriers — forces the next ARM-side read to come from RAM instead of
/// whatever this core cached before a bus master wrote its response
/// there.
pub(crate) fn invalidate_range(address: u32, len: usize) {
    let end = address + len as u32;
    let mut line = address & !(MIN_CACHE_LINE - 1);
    while line < end {
        unsafe { invalidate_line(line) };
        line += MIN_CACHE_LINE;
    }
    barrier();
}

// The per-line ops and the barrier are the only architecture-specific
// part. AArch32 uses CP15 cache-maintenance coprocessor writes (DCCMVAC
// to clean, DCIMVAC to invalidate) and a bare `dsb`; AArch64 uses the
// `dc cvac` / `dc ivac` cache-maintenance instructions and `dsb sy`. Both
// operate to the point of coherency — the level shared with the bus
// masters. On AArch64 the `{0:x}` operand selects the 64-bit view of the
// (zero-extended, always < 4 GB on this SoC) address, since `dc` takes an
// X register.

/// Cleans the single cache line containing `line` to the point of
/// coherency.
#[cfg(target_arch = "arm")]
#[inline(always)]
unsafe fn clean_line(line: u32) {
    asm!("mcr p15, 0, {0}, c7, c10, 1", in(reg) line);
}

/// Cleans the single cache line containing `line` to the point of
/// coherency.
#[cfg(target_arch = "aarch64")]
#[inline(always)]
unsafe fn clean_line(line: u32) {
    asm!("dc cvac, {0:x}", in(reg) line);
}

/// Invalidates the single cache line containing `line` to the point of
/// coherency.
#[cfg(target_arch = "arm")]
#[inline(always)]
unsafe fn invalidate_line(line: u32) {
    asm!("mcr p15, 0, {0}, c7, c6, 1", in(reg) line);
}

/// Invalidates the single cache line containing `line` to the point of
/// coherency.
#[cfg(target_arch = "aarch64")]
#[inline(always)]
unsafe fn invalidate_line(line: u32) {
    asm!("dc ivac, {0:x}", in(reg) line);
}

/// Data synchronization barrier over the full system domain — completes
/// the cache maintenance above before the caller kicks off DMA.
#[cfg(target_arch = "arm")]
#[inline(always)]
fn barrier() {
    unsafe { asm!("dsb") };
}

/// Data synchronization barrier over the full system domain — completes
/// the cache maintenance above before the caller kicks off DMA.
#[cfg(target_arch = "aarch64")]
#[inline(always)]
fn barrier() {
    unsafe { asm!("dsb sy") };
}

#[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
compile_error!("rpi-hal supports only ARM (AArch32) and AArch64 targets");
