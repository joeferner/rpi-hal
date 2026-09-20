//! AArch32 (short-descriptor) MMU implementation -- see the parent
//! [`mmu`](super) module's doc comment for the overall design.
//!
//! A single flat first-level table of 1MB sections covering all 4GB,
//! CP15-programmed, run at PL1.
//!
//! One file for both 32-bit architectures, because the table itself is
//! the same on each: ARMv6 with `SCTLR.XP` set uses the descriptor format
//! ARMv7-A made the only one, down to the bit positions of `TEX`, `APX`
//! and `XN`. What differs is the CP15 programming around it, and each
//! difference is marked `cfg(armv6)` below with its reason -- the short
//! version being that ARMv6 has no Snoop Control Unit to enable, no
//! inner-shareable operations, nothing shareable worth marking on a
//! uniprocessor, and caches that come out of reset holding rubbish.

use super::{LOCAL_PERIPHERAL, PERIPHERAL_BASE, PERIPHERAL_END};
use crate::barrier::{dsb, isb};
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
#[cfg(not(armv6))]
const S_BIT: u32 = 1 << 16;
/// Zero on ARMv6, so RAM is mapped Normal Cacheable **Non-shareable**.
///
/// The one descriptor field that does not carry over from the ARMv7
/// constants, and the reasoning is the opposite of the ARMv7 one above.
/// The ARM1176JZF-S is a uniprocessor part with no coherent interconnect
/// and no external global monitor: there is nothing for a shareable
/// mapping to be shared *with*, and marking cacheable memory shareable on
/// a core that cannot coherently cache it is how a region quietly stops
/// being cached at all -- which would take `ldrex`/`strex` down with it,
/// this map's whole purpose (see the parent module's doc comment).
/// Non-shareable leaves the core's own local exclusive monitor to do the
/// job, which is what it is for.
///
/// Linux reaches the same arrangement from the other direction: its
/// ARMv6 section flags are `PMD_FLAGS_UP = PMD_SECT_WB` for
/// uniprocessor, against `PMD_FLAGS_SMP` which adds `PMD_SECT_S`, and it
/// forces the S bit on only when it knows it is running SMP.
#[cfg(armv6)]
const S_BIT: u32 = 0;

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
/// block (on a chip that has one) as [`SECTION_DEVICE`], and everything
/// else left as an invalid descriptor (bits\[1:0\] = `00`) -- touching
/// genuinely unbacked address space still faults instead of being
/// silently redefined as valid.
const fn build_page_table() -> [u32; SECTION_COUNT] {
    let mut table = [0u32; SECTION_COUNT];
    let mut i = 0;
    while i < SECTION_COUNT {
        let base = (i as u32) << SECTION_SHIFT;
        if base < PERIPHERAL_BASE {
            table[i] = base | SECTION_RAM;
        } else if base <= PERIPHERAL_END || {
            // Under `bcm2711`, the local block's end is `u32::MAX` (it runs
            // to the top of the address space), which makes the upper bound
            // below trivially true -- still the right check for the
            // BCM2836/2837 case, where it isn't.
            #[allow(clippy::absurd_extreme_comparisons)]
            let in_local_block = match LOCAL_PERIPHERAL {
                Some((local_base, local_end)) => base >= local_base && base <= local_end,
                None => false,
            };
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
/// see. On ARMv7 the TLB operation is the inner-shareable variant
/// (`TLBIALLIS`), so secondary cores walking this same table drop their
/// stale entries too.
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
        #[cfg(not(armv6))]
        asm!("mcr p15, 0, {0}, c8, c3, 0", in(reg) 0u32);

        // ARMv6 has no inner-shareable TLB operations -- they arrive with
        // the ARMv7 multiprocessing extensions -- so this is the plain
        // `TLBIALL`, which is all a uniprocessor needs. The branch
        // predictor is invalidated alongside it: the ARM1176 can hold
        // predictions made under the old translation, where ARMv7-A
        // discards them itself.
        #[cfg(armv6)]
        {
            asm!("mcr p15, 0, {0}, c8, c7, 0", in(reg) 0u32);
            asm!("mcr p15, 0, {0}, c7, c5, 6", in(reg) 0u32);
        }
    }
    dsb();
    isb();
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
        //
        // Not done on ARMv6, and this is not a case of "harmless to skip":
        // `ACTLR` is implementation-defined, the ARM1176 has no Snoop
        // Control Unit to turn on, and its bit 6 is something else
        // entirely. A read-modify-write left in place for symmetry would
        // be setting an unknown bit on a real register.
        #[cfg(not(armv6))]
        {
            let mut actlr: u32;
            asm!("mrc p15, 0, {0}, c1, c0, 1", out(reg) actlr);
            actlr |= 1 << 6;
            asm!("mcr p15, 0, {0}, c1, c0, 1", in(reg) actlr);
        }

        // Caches and branch predictor come out of reset with UNPREDICTABLE
        // contents on this core, and are about to be turned on: invalidate
        // them before they can be consulted. Invalidate rather than
        // clean+invalidate, deliberately -- a clean would write whatever
        // random lines reset left looking dirty *out* to RAM. Nothing can
        // be lost by discarding them, since the caches have been off since
        // reset.
        //
        // ARMv7-A needs none of this: the Cortex-A7 invalidates its caches
        // at reset.
        #[cfg(armv6)]
        {
            // Invalidate both caches (`c7, c7, 0`) and the branch target
            // cache (`c7, c5, 6`).
            asm!("mcr p15, 0, {0}, c7, c7, 0", in(reg) 0u32);
            asm!("mcr p15, 0, {0}, c7, c5, 6", in(reg) 0u32);

            // TTBCR = 0: every translation walks through TTBR0, with no
            // TTBR1 split. That is the reset value, but "whatever the GPU
            // firmware left" is not something this code assumes anywhere
            // else either (see SCTLR.V in boot6.s), and a non-zero N here
            // would send the top of the address space to a second table
            // that does not exist.
            asm!("mcr p15, 0, {0}, c2, c0, 2", in(reg) 0u32);
        }

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
        dsb();
        isb();

        // SCTLR: set M (bit 0) and C (bit 2, data cache) -- see the parent
        // module's doc comment on why C is needed for `ldrex`/`strex` to
        // work on this core. I (bit 12, instruction cache) stays clear --
        // unrelated to this.
        let mut sctlr: u32;
        asm!("mrc p15, 0, {0}, c1, c0, 0", out(reg) sctlr);
        sctlr |= 1 | (1 << 2);

        // Three more bits on ARMv6. Two of them are real and writable
        // only here -- ARMv7 dropped both, having made each the only
        // behaviour -- and the third is a judgement call this core's speed
        // changes.
        //
        // XP (bit 23) is the one line in this file that the whole ARMv6
        // map depends on. It selects the descriptor format that
        // `build_page_table` above emits. Leave it clear and the core
        // reads those same words in the legacy subpage-AP format instead,
        // where `TEX` is not a memory type, bit 15 is not `APX`, and bit
        // 16 is not `S` -- every attribute means something else, silently,
        // and the map that results is nonsense rather than absent.
        //
        // U (bit 22) selects the ARMv6 unaligned access model. With it
        // clear the core keeps the ARMv5 one, where an unaligned word load
        // does not fault but returns the addressed word *rotated* -- wrong
        // data, no diagnostic. Nothing this crate compiles emits an
        // unaligned access today (the target is strict-alignment), so this
        // is not fixing a live bug; it is closing off a silent failure in
        // favour of a loud one. Linux sets both bits on this core for the
        // same reasons (`v6_crval`'s `mmuset` = 0x00c0387d).
        //
        // I (bit 12, instruction cache) is the third, and is the one place
        // this arm deliberately differs from the ARMv7 one rather than
        // merely spelling the same intent differently. The ARM1176 fetches
        // from a 1GHz core over a memory system shared with the VideoCore,
        // and an uncached fetch of every instruction is a cost it feels in
        // a way the later cores do not. Safe here because the I-cache is
        // invalidated above before it is switched on, and nothing in this
        // crate writes instructions: a program that generates or relocates
        // code must invalidate the I-cache itself, which is true on every
        // architecture but only matters once the cache is on.
        #[cfg(armv6)]
        {
            sctlr |= (1 << 23) | (1 << 22) | (1 << 12);
        }

        asm!("mcr p15, 0, {0}, c1, c0, 0", in(reg) sctlr);
    }

    // Architecturally required right after enabling the MMU: the
    // pipeline may have already fetched ahead using the old (MMU-off)
    // address translation behavior.
    isb();
}
