//! AArch32 (ARMv7-A short-descriptor) MMU implementation -- see the
//! parent [`mmu`](super) module's doc comment for the overall design.
//!
//! A single flat first-level table of 1MB sections covering all 4GB,
//! CP15-programmed, run at PL1.

use super::{LOCAL_PERIPHERAL_BASE, LOCAL_PERIPHERAL_END, PERIPHERAL_BASE, PERIPHERAL_END};
use crate::cache::clean_range;
use core::arch::asm;
use core::cell::UnsafeCell;

/// Entries in a full first-level short-descriptor translation table:
/// one 1MB section per entry, covering the whole 4GB address space.
const SECTION_COUNT: usize = 4096;

/// Shift from a section index to its base physical address (each
/// section covers `1 << 20` = 1MB).
const SECTION_SHIFT: u32 = 20;

// Section descriptor fields (ARMv7-A VMSA short-descriptor format --
// see the ARM Architecture Reference Manual's translation table
// descriptor and memory region attribute tables).
/// Bits[1:0]: marks this entry as a Section descriptor with PXN
/// (bit 0) clear, i.e. privileged execution is allowed.
const DESCRIPTOR_SECTION: u32 = 0b10;
/// Bits[11:10] (AP\[1:0\]) with APX (bit 15) left clear: full
/// read/write access at any privilege level.
const AP_FULL_ACCESS: u32 = 0b11 << 10;
/// Bit[4]: execute-never.
const XN: u32 = 1 << 4;
/// Bit[2] (B) -- meaning depends on which `TEX`/`C` it's paired with;
/// see `SECTION_RAM`/`SECTION_DEVICE`'s comments.
const B_BIT: u32 = 1 << 2;
/// Bits[14:12] (TEX) = `0b001`: Normal memory, with C and B then selecting
/// its cacheability -- C=1/B=1 for Outer and Inner Write-Back
/// Write-Allocate, both clear for Outer and Inner Non-cacheable. See the
/// parent module's doc comment on why RAM needs to be Cacheable, not just
/// Shareable, for `ldrex`/`strex` to actually succeed on this core.
const TEX_NORMAL: u32 = 0b001 << 12;
/// Bit[3] (C), paired with `TEX_NORMAL`/`B_BIT` above.
const C_BIT: u32 = 1 << 3;
/// Bit[16] (S): Shareable. Necessary but not sufficient on its own on
/// this core (Cortex-A7) -- see the parent module's doc comment.
const S_BIT: u32 = 1 << 16;

/// RAM: Normal, Write-Back Write-Allocate Cacheable, Shareable, full
/// access, executable -- covers every address this crate's own code,
/// data, and stacks can occupy.
const SECTION_RAM: u32 = DESCRIPTOR_SECTION | AP_FULL_ACCESS | TEX_NORMAL | C_BIT | B_BIT | S_BIT;

/// RAM with the caches taken out of the picture: the same `TEX=001` Normal
/// memory as [`SECTION_RAM`] but with C and B clear, i.e. Outer and Inner
/// Non-cacheable. Still Shareable and still Normal (not Device), so
/// unaligned accesses and the compiler's usual load/store merging remain
/// legal -- only the caching is gone. Installed by
/// [`set_uncached_block`] over a region shared with the VideoCore; see
/// [`crate::mmu::set_uncached`] for why that is necessary rather than a
/// performance choice.
const SECTION_RAM_UNCACHED: u32 = DESCRIPTOR_SECTION | AP_FULL_ACCESS | TEX_NORMAL | S_BIT;

/// Peripherals: TEX=000/C=0/B=1 is "Shareable Device" memory -- the
/// same ordering/no-caching guarantees every raw register access in
/// this crate already depends on with the MMU off, now made explicit.
/// Execute-never, since nothing should ever jump into MMIO space.
const SECTION_DEVICE: u32 = DESCRIPTOR_SECTION | AP_FULL_ACCESS | B_BIT | XN;

/// Builds the identity map: RAM below the peripheral base as
/// [`SECTION_RAM`], the peripheral block and the ARM-local peripheral
/// block as [`SECTION_DEVICE`], and everything else left as an invalid
/// descriptor (bits\[1:0\] = `00`) -- touching genuinely unbacked address
/// space still faults instead of being silently redefined as valid.
const fn build_page_table() -> [u32; SECTION_COUNT] {
    let mut table = [0u32; SECTION_COUNT];
    let mut i = 0;
    while i < SECTION_COUNT {
        let base = (i as u32) << SECTION_SHIFT;
        if base < PERIPHERAL_BASE {
            table[i] = base | SECTION_RAM;
        } else if base <= PERIPHERAL_END || {
            // Under `bcm2711`, `LOCAL_PERIPHERAL_END` is `u32::MAX` (the
            // ARM-local block runs to the top of the address space), which
            // makes the upper bound below trivially true -- still the
            // right check for the BCM2836/2837 case, where it isn't.
            #[allow(clippy::absurd_extreme_comparisons)]
            let in_local_block = base >= LOCAL_PERIPHERAL_BASE && base <= LOCAL_PERIPHERAL_END;
            in_local_block
        } {
            table[i] = base | SECTION_DEVICE;
        }
        i += 1;
    }
    table
}

/// The first-level translation table itself, built entirely at compile
/// time (not populated by any runtime loop). `TTBR0` with `N=0` (a single
/// top-level table, no split) requires 16KB alignment --
/// `#[repr(align(16384))]` on a wrapper struct, since `align` can't be
/// attached directly to a `static`'s type when that type is a plain array.
///
/// `UnsafeCell` because the hardware page-table walker reads all of it
/// behind the compiler's back, and because [`set_uncached_block`] rewrites
/// individual descriptors after boot.
#[repr(align(16384))]
struct PageTable(UnsafeCell<[u32; SECTION_COUNT]>);

// SAFETY: the table is written only during early-boot MMU setup and by
// `set_uncached_block`, whose own safety contract requires the region it
// covers to be quiescent; otherwise it is read only by the hardware walker.
unsafe impl Sync for PageTable {}

static PAGE_TABLE: PageTable = PageTable(UnsafeCell::new(build_page_table()));

/// Bytes covered by one descriptor -- see [`crate::mmu::UNCACHED_GRANULE`].
pub(super) const UNCACHED_GRANULE: usize = 1 << SECTION_SHIFT;

/// Rewrites the section descriptor covering `base` as
/// [`SECTION_RAM_UNCACHED`], then makes the change take effect everywhere.
///
/// The descriptor write is cleaned out of this core's cache before the TLB
/// invalidation because `TTBR0` is programmed for non-cacheable table walks
/// (see [`rpi_hal_mmu_init`]) -- the walker reads RAM directly, so a
/// descriptor sitting dirty in the D-cache is one the hardware would never
/// see. The TLB operation is the inner-shareable variant (`TLBIALLIS`), so
/// secondary cores walking this same table drop their stale entries too.
///
/// # Safety
///
/// `base` must be 1MB-aligned and within the RAM this table maps as Normal
/// memory; see [`crate::mmu::set_uncached`], which checks both and is the
/// only caller.
pub(super) unsafe fn set_uncached_block(base: u32) {
    let index = (base >> SECTION_SHIFT) as usize;
    let entry = unsafe { (PAGE_TABLE.0.get() as *mut u32).add(index) };
    unsafe { entry.write_volatile(base | SECTION_RAM_UNCACHED) };

    clean_range(entry as u32, size_of::<u32>());

    unsafe {
        // TLBIALLIS: invalidate the entire TLB across the inner-shareable
        // domain. The operand is ignored (SBZ).
        asm!("mcr p15, 0, {0}, c8, c3, 0", in(reg) 0u32);
        asm!("dsb");
        asm!("isb");
    }
}

/// Builds the identity-mapped page table (above) and enables the MMU.
/// Called from `boot.s`, after `VBAR`/`SCTLR.V` setup and before `.bss`
/// zeroing/`kmain`: a fault during this sequence is at least catchable
/// once `VBAR` is live, and nothing before `kmain` needs the MMU already
/// on.
///
/// Safe to call once per core, not just once overall: with the
/// `multicore` feature on, every secondary core calls this again as part
/// of its own bring-up (see `boot.s`'s `__secondary_core_entry`).
/// TTBR0/DACR/SCTLR and the TLB are all per-core banked state, so each
/// call only ever reprograms the calling core's own copy against the
/// single, already-built [`PAGE_TABLE`] -- never a second build of the
/// table itself.
///
/// # Safety
///
/// Must only be called early in boot, by `rt`'s own boot sequence, on
/// each core before any code on that core relies on today's MMU-off
/// memory ordering guarantees changing underneath it.
#[no_mangle]
pub unsafe extern "C" fn rpi_hal_mmu_init() {
    let ttbr0 = PAGE_TABLE.0.get() as u32;

    unsafe {
        // ACTLR.SMP (bit 6): per the Cortex-A7 TRM, a core must set this
        // before enabling its caches for cache coherency (via the Snoop
        // Control Unit) to actually apply to it in a multiprocessor
        // system. Without it, this core's cacheable writes are not
        // guaranteed to ever become visible to another core no matter how
        // many dsb/dmb barriers follow. Harmless to set unconditionally
        // even when only core 0 ever runs.
        let mut actlr: u32;
        asm!("mrc p15, 0, {0}, c1, c0, 1", out(reg) actlr);
        actlr |= 1 << 6;
        asm!("mcr p15, 0, {0}, c1, c0, 1", in(reg) actlr);

        // TTBR0: point at the page table. Low attribute bits (RGN/S/IRGN,
        // meaningful for cached/shared page-table walks) left 0 --
        // irrelevant with caches off and a single core.
        asm!("mcr p15, 0, {0}, c2, c0, 0", in(reg) ttbr0);

        // DACR: domain 0 set to "client" (0b01) -- respects the page
        // table's own AP bits rather than bypassing them. Every other
        // domain left at 0 = "no access".
        asm!("mcr p15, 0, {0}, c3, c0, 0", in(reg) 0b01u32);

        // Invalidate the entire TLB -- this core's TLB state coming out of
        // whatever GPU firmware ran before us isn't something to assume is
        // clean.
        asm!("mcr p15, 0, {0}, c8, c7, 0", in(reg) 0u32);
        asm!("dsb");
        asm!("isb");

        // SCTLR: set M (bit 0) and C (bit 2, data cache) -- see the parent
        // module's doc comment on why C is needed for `ldrex`/`strex` to
        // work on this core. I (bit 12, instruction cache) stays clear --
        // unrelated to this.
        let mut sctlr: u32;
        asm!("mrc p15, 0, {0}, c1, c0, 0", out(reg) sctlr);
        sctlr |= 1 | (1 << 2);
        asm!("mcr p15, 0, {0}, c1, c0, 0", in(reg) sctlr);

        // Architecturally required right after enabling the MMU: the
        // pipeline may have already fetched ahead using the old (MMU-off)
        // address translation behavior.
        asm!("isb");
    }
}
