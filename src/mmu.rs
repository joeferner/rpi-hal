//! MMU bring-up: enables address translation so RAM can be marked
//! Normal memory, needed for `core::sync::atomic`'s exclusive-access
//! instructions (`ldrex`/`strex` on AArch32, `ldxr`/`stxr` on AArch64)
//! to behave correctly -- those are architecturally UNPREDICTABLE on the
//! Strongly-Ordered/Device memory every address is with the MMU off.
//! Every peripheral MMIO region keeps behaving exactly as it does today
//! (Device memory, same ordering guarantees raw register access already
//! relies on everywhere else in this crate).
//!
//! Identity-mapped only: every virtual address equals its physical
//! address. This exists purely to change memory *attributes*, not
//! layout -- no relocation, no higher-half kernel.
//!
//! The data cache is enabled and RAM is mapped Cacheable -- confirmed
//! necessary on real hardware (AArch32), not a nicety: with RAM mapped
//! Normal-but-Non-cacheable, the exclusive monitor never let `strex`
//! succeed, because this core ties the monitor to cache-line state.
//!
//! The implementation differs enough between execution states to live in
//! two files (as `boot.s`/`boot64.s` do), selected by target
//! architecture:
//!
//! - [`mmu32`](mod@self) -> `mmu32.rs`: ARMv7-A VMSA short-descriptor
//!   first-level table (1MB sections), CP15-programmed, run at PL1.
//!   Called from `rt`'s `boot.s` after its HYP->SVC drop.
//! - `mmu64.rs`: VMSAv8-64 long-descriptor tables (2MB blocks),
//!   programmed via `MAIR_EL1`/`TCR_EL1`/`TTBR0_EL1`, run at EL1. The
//!   caller must already be at EL1 (a consumer's boot stub drops
//!   EL2->EL1 first).
//!
//! Both define the same `#[no_mangle] extern "C" fn rpi_hal_mmu_init`
//! entry point. It is a *strong* symbol; `rt`'s own boot sequence
//! (`boot.s`) calls it unconditionally, falling back to a weak no-op
//! (`mmu_fallback.s`, only included when the `mmu` feature is off -- see
//! `boot.rs`) when this feature is off. A consumer wanting a different
//! memory map turns this feature off and defines their own
//! `#[no_mangle] extern "C" fn rpi_hal_mmu_init()` instead.

// SoC memory map, shared by both implementations. The BCM2836/2837
// values are confirmed against this crate's own peripheral access types
// (e.g. the System Timer at `0x3f00_3000`, the legacy interrupt
// controller at `0x3f00_b000`, out to `0x3f98_0e00` for USB), not assumed
// from a datasheet. The BCM2711 values are the "low peripheral mode"
// addresses from Linux's `bcm2711.dtsi` (the peripheral block's `ranges`
// entry and `local_intc`/GIC-400 nodes), not yet cross-checked against
// this crate's own drivers the way the BCM2836/2837 base was -- there's
// no BCM2711 target to build until these consts exist, which is what
// they're for. `PERIPHERAL_BASE` itself lives in `crate::soc`, not here
// -- `crate::dma`/`crate::watchdog` need it regardless of whether this
// (`mmu`-gated) module is even compiled.

use crate::soc::PERIPHERAL_BASE;

/// End of the mapped peripheral block (inclusive) -- a round,
/// 1MB-aligned number comfortably covering every peripheral address this
/// crate can reach.
#[cfg(not(feature = "bcm2711"))]
const PERIPHERAL_END: u32 = 0x3FFF_FFFF;
#[cfg(feature = "bcm2711")]
const PERIPHERAL_END: u32 = 0xFEFF_FFFF;

/// Base of the ARM-local peripheral block (per-core timers, IRQ routing,
/// and the inter-core mailbox registers on BCM2836/2837; the same block
/// plus GIC-400 on BCM2711) -- a physically separate MMIO region from
/// [`PERIPHERAL_BASE`]. Needed device-mapped so [`crate::multicore`] can
/// reach the mailbox registers with the MMU on.
#[cfg(not(feature = "bcm2711"))]
const LOCAL_PERIPHERAL_BASE: u32 = 0x4000_0000;
/// BCM2711 low-peripheral-mode base (`bcm2711.dtsi`'s `ranges` entry
/// mapping bus address `0x4000_0000` to this physical address; GIC-400's
/// distributor/CPU-interface registers live at `0xFF84_1000`/`0xFF84_2000`
/// within this block).
#[cfg(feature = "bcm2711")]
const LOCAL_PERIPHERAL_BASE: u32 = 0xFF80_0000;

/// End of the ARM-local peripheral block (inclusive) -- one section
/// comfortably covers the whole register file (mailboxes and core control
/// live in the first few hundred bytes).
#[cfg(not(feature = "bcm2711"))]
const LOCAL_PERIPHERAL_END: u32 = 0x400F_FFFF;
/// BCM2711's ARM-local block is 8MB (`bcm2711.dtsi`'s `ranges` size for
/// this entry), running to the top of the 32-bit address space.
#[cfg(feature = "bcm2711")]
const LOCAL_PERIPHERAL_END: u32 = 0xFFFF_FFFF;

#[cfg(target_arch = "arm")]
#[path = "mmu32.rs"]
mod imp;

#[cfg(target_arch = "aarch64")]
#[path = "mmu64.rs"]
mod imp;

use crate::cache::clean_invalidate_range;

/// Granularity of [`set_uncached`]: 1MB (AArch32's section) or 2MB
/// (AArch64's level-2 block), whichever this build's table uses. A region
/// handed to that function must be aligned to this and a whole multiple of
/// it, since a translation table entry is the smallest thing whose memory
/// type can be changed.
///
/// Sized for whichever implementation is compiled, so a `static` intended to
/// be remapped has to be aligned and padded to the larger of the two to
/// build for both targets — 2MB, the value below on AArch64.
pub const UNCACHED_GRANULE: usize = imp::UNCACHED_GRANULE;

/// Why a [`set_uncached`] call was rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The base address or the length isn't a multiple of
    /// [`UNCACHED_GRANULE`]. A translation table entry covers exactly that
    /// much, so a partial one can't be given a different memory type
    /// without splitting the entry — which this map, deliberately flat,
    /// doesn't do.
    Misaligned,
    /// The region isn't entirely inside the RAM this table maps as Normal
    /// memory (below the peripheral block). Remapping MMIO space, or an
    /// address with no descriptor at all, is a caller bug rather than
    /// something to silently do.
    NotRam,
}

/// Remaps `len` bytes at `base` as Normal **Non-cacheable** memory, and
/// drops whatever this core had cached of it.
///
/// This is what makes a shared-memory protocol with the VideoCore possible.
/// The rest of this crate's bus-master traffic gets by with explicit cache
/// maintenance (`cache.rs`) because each buffer has one owner at a time: the
/// ARM writes it, cleans it, and hands it over. VCHIQ's shared state
/// (`crate::vchiq`) is not like that — both sides write *different fields of
/// the same cache line* concurrently, so there is no correct maintenance
/// sequence at all. Cleaning the line to publish this core's field writes
/// the stale copy of the peer's field back over it; invalidating it to read
/// the peer's field discards this core's not-yet-written-back one. Only
/// taking the line out of the picture fixes that, which is why Linux
/// allocates the same region with `dma_alloc_coherent`.
///
/// Both the descriptor write and the TLB invalidation are broadcast to the
/// inner-shareable domain, so secondary cores (which share this one table)
/// see the new memory type too rather than keeping a stale cached
/// translation.
///
/// # Safety
///
/// The region must be memory this caller owns outright — the entire
/// [`UNCACHED_GRANULE`]-sized block gets a new memory type, so anything else
/// that happens to live in it silently becomes non-cacheable and much
/// slower. Nothing may be concurrently accessing the region on any core
/// during the call: its cached copies are discarded as part of it.
pub unsafe fn set_uncached(base: usize, len: usize) -> Result<(), Error> {
    if !base.is_multiple_of(UNCACHED_GRANULE) || len == 0 || !len.is_multiple_of(UNCACHED_GRANULE) {
        return Err(Error::Misaligned);
    }
    let end = base
        .checked_add(len)
        .filter(|end| *end <= PERIPHERAL_BASE as usize)
        .ok_or(Error::NotRam)?;

    // Write back and drop every cached copy *before* the memory type
    // changes: afterwards these lines are no longer reachable by
    // maintenance-by-address through this (now non-cacheable) mapping, and a
    // dirty one left behind could be evicted over data written through the
    // new mapping.
    clean_invalidate_range(base as u32, len);

    for block in (base..end).step_by(UNCACHED_GRANULE) {
        // SAFETY: `block` is granule-aligned and inside the RAM range the
        // table maps as Normal memory, both checked above.
        unsafe { imp::set_uncached_block(block as u32) };
    }
    Ok(())
}
