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

// Each build calls only the helpers its chip's drivers need -- the BCM2712
// has few DMA-driven drivers, the others don't touch RAM above 4 GiB --
// and the rest stay compiled rather than `cfg`'d out, ready for the first
// caller that arrives.
#![allow(dead_code)]

use crate::barrier::dsb;
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

/// Runs `op` on every cache line covering `len` bytes from `address`, then
/// barriers.
fn each_line(address: usize, len: usize, op: unsafe fn(usize)) {
    let end = address + len;
    let mut line = address & !(MIN_CACHE_LINE as usize - 1);
    while line < end {
        // SAFETY: a cache maintenance operation by address on memory the
        // caller has named; it changes no data a correct program relies
        // on, only where it is held.
        unsafe { op(line) };
        line += MIN_CACHE_LINE as usize;
    }
    dsb();
}

/// Cleans (writes back, without invalidating) every cache line covering
/// `len` bytes from `address`, then barriers — makes an ARM-side write
/// actually visible in RAM before a bus master is told to read it.
pub(crate) fn clean_range(address: u32, len: usize) {
    clean_bytes(address as usize, len);
}

/// Invalidates every cache line covering `len` bytes from `address`, then
/// barriers — forces the next ARM-side read to come from RAM instead of
/// whatever this core cached before a bus master wrote its response
/// there.
pub(crate) fn invalidate_range(address: u32, len: usize) {
    invalidate_bytes(address as usize, len);
}

/// [`clean_range`] at any address, including the BCM2712's RAM above
/// 4 GiB, which a `u32` can't name.
pub(crate) fn clean_bytes(address: usize, len: usize) {
    each_line(address, len, clean_line);
}

/// [`invalidate_range`] at any address.
pub(crate) fn invalidate_bytes(address: usize, len: usize) {
    each_line(address, len, invalidate_line);
}

/// [`clean_invalidate_range`] at any address.
pub(crate) fn clean_invalidate_bytes(address: usize, len: usize) {
    each_line(address, len, clean_invalidate_line);
}

/// Cleans *and* invalidates every cache line covering `len` bytes from
/// `address`, then barriers — writes back anything this core had dirty and
/// leaves nothing cached behind.
///
/// The one place this is needed rather than one of the two halves above is
/// when memory stops being cacheable at all: a region about to be remapped
/// Normal Non-cacheable (`crate::mmu::set_uncached`) must have no cached
/// copy left, dirty or clean. A plain invalidate would discard writes that
/// never reached RAM; a plain clean would leave clean lines that the new,
/// non-cacheable accesses bypass, so a later eviction could still hand back
/// pre-remap data.
///
/// It is also what a buffer a bus master is about to *write* needs first,
/// when its ends may share cache lines with other data: cleaning writes
/// those neighbours back, so the invalidate that follows the transfer
/// can't discard them.
pub(crate) fn clean_invalidate_range(address: u32, len: usize) {
    clean_invalidate_bytes(address as usize, len);
}

// The per-line ops are the only architecture-specific part left here;
// the trailing barrier comes from `crate::barrier`, which has its own
// three-way split. AArch32 uses CP15 cache-maintenance coprocessor
// writes (DCCMVAC to clean, DCIMVAC to invalidate, DCCIMVAC for both),
// with the same encodings on ARMv6 and ARMv7 — maintenance *by MVA* did
// not change, unlike the barriers; AArch64 uses the `dc cvac` /
// `dc ivac` / `dc civac` cache-maintenance instructions. All operate to
// the point of coherency — the level shared with the bus masters. The
// address is a `usize`: a 32-bit register on AArch32, and on AArch64 the
// X register `dc` takes, which reaches the BCM2712's RAM above 4 GiB.

/// Cleans the single cache line containing `line` to the point of
/// coherency.
#[cfg(target_arch = "arm")]
#[inline(always)]
unsafe fn clean_line(line: usize) {
    asm!("mcr p15, 0, {0}, c7, c10, 1", in(reg) line);
}

/// Cleans the single cache line containing `line` to the point of
/// coherency.
#[cfg(target_arch = "aarch64")]
#[inline(always)]
unsafe fn clean_line(line: usize) {
    asm!("dc cvac, {0}", in(reg) line);
}

/// Invalidates the single cache line containing `line` to the point of
/// coherency.
#[cfg(target_arch = "arm")]
#[inline(always)]
unsafe fn invalidate_line(line: usize) {
    asm!("mcr p15, 0, {0}, c7, c6, 1", in(reg) line);
}

/// Invalidates the single cache line containing `line` to the point of
/// coherency.
#[cfg(target_arch = "aarch64")]
#[inline(always)]
unsafe fn invalidate_line(line: usize) {
    asm!("dc ivac, {0}", in(reg) line);
}

/// Cleans and invalidates the single cache line containing `line` to the
/// point of coherency.
#[cfg(target_arch = "arm")]
#[inline(always)]
unsafe fn clean_invalidate_line(line: usize) {
    asm!("mcr p15, 0, {0}, c7, c14, 1", in(reg) line);
}

/// Cleans and invalidates the single cache line containing `line` to the
/// point of coherency.
#[cfg(target_arch = "aarch64")]
#[inline(always)]
unsafe fn clean_invalidate_line(line: usize) {
    asm!("dc civac, {0}", in(reg) line);
}

#[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
compile_error!("rpi-hal supports only ARM (AArch32) and AArch64 targets");
