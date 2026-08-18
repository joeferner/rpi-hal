//! AArch64 (VMSAv8-64 long-descriptor) MMU implementation, run at EL1 --
//! see the parent [`mmu`](super) module's doc comment for the overall
//! design.
//!
//! The AArch32 table uses 1MB sections in a single flat 4GB table.
//! AArch64 has no equivalent single-level format, so this builds a small
//! two-level tree with a 4KB granule and 39-bit input addresses
//! (`T0SZ`=25), covering the same low 4GB:
//!
//! ```text
//! level 1 (1GB/entry) -> the low four entries, one per 1GB region:
//!   [0] -> level-2 table: 0x0000_0000 ..= 0x3FFF_FFFF
//!   [1] -> level-2 table: 0x4000_0000 ..= 0x7FFF_FFFF
//!   [2] -> level-2 table: 0x8000_0000 ..= 0xBFFF_FFFF
//!   [3] -> level-2 table: 0xC000_0000 ..= 0xFFFF_FFFF
//! level 2 (2MB/block): RAM below PERIPHERAL_BASE as Normal, the
//!   peripheral block and the ARM-local peripheral block as Device.
//! ```
//!
//! Every region gets its own level-2 table rather than special-casing the
//! ones a peripheral block happens to fall in, because where those blocks
//! land varies by chip: on BCM2836/2837 they sit in regions 0
//! (`0x3F00_0000`) and 1 (`0x4000_0000`), but on BCM2711 both are in
//! region 3 (`0xFE00_0000` and `0xFF80_0000`). Populating a fixed pair of
//! level-1 entries is what an earlier version of this file did, and on
//! BCM2711 that left the whole peripheral block unmapped, so the first
//! register access after enabling translation faulted -- hanging boot
//! inside this module, before anything could report why. Covering all four
//! regions uniformly costs 16KB of `.bss` and removes the class of bug.
//!
//! Anything above 4GB, and anything within it not covered by a rule above,
//! is left as an invalid (zero) descriptor, so touching genuinely unbacked
//! address space faults -- same intent as the AArch32 map.

use super::{LOCAL_PERIPHERAL_BASE, LOCAL_PERIPHERAL_END, PERIPHERAL_BASE, PERIPHERAL_END};
use core::arch::asm;
use core::cell::UnsafeCell;

/// Number of descriptors in a 4KB-granule translation table.
const ENTRIES: usize = 512;

/// Bytes spanned by one level-2 descriptor (a 2MB block).
const BLOCK_2MB: u64 = 2 * 1024 * 1024;

/// Bytes spanned by one level-1 descriptor (a 1GB region).
const BLOCK_1GB: u64 = 1024 * 1024 * 1024;

/// Level-1 entries populated, covering the low 4GB -- the same span as the
/// AArch32 table, and enough for every physical address this crate can
/// reach (see `TCR_EL1`'s `IPS` below). The rest of the 39-bit input range
/// stays invalid.
const L1_REGIONS: usize = 4;

// Descriptor bits (see the ARMv8-A Architecture Reference Manual's
// VMSAv8-64 translation table descriptor formats).
/// Bits[1:0] = `0b11`: a table descriptor (points at the next level).
const DESC_TABLE: u64 = 0b11;
/// Bits[1:0] = `0b01`: a block descriptor (a leaf mapping at L1/L2).
const DESC_BLOCK: u64 = 0b01;
/// Bit[10] (AF, Access Flag): set so the first access doesn't fault.
const AF: u64 = 1 << 10;
/// Bits[9:8] (SH) = `0b11`: Inner Shareable -- see the parent module's
/// doc comment on why cacheable, shareable RAM matters for exclusive
/// accesses.
const SH_INNER: u64 = 0b11 << 8;
/// Bits[4:2] (AttrIndx) = 0: index into `MAIR_EL1` for Normal memory.
const ATTR_NORMAL: u64 = 0 << 2;
/// Bits[4:2] (AttrIndx) = 1: index into `MAIR_EL1` for Device memory.
const ATTR_DEVICE: u64 = 1 << 2;
/// Bits[4:2] (AttrIndx) = 2: index into `MAIR_EL1` for Normal
/// Non-cacheable memory.
const ATTR_NORMAL_UNCACHED: u64 = 2 << 2;
/// Bit[54] (UXN) and bit[53] (PXN): execute-never at every privilege --
/// nothing should fetch instructions from MMIO space.
const EXEC_NEVER: u64 = (1 << 54) | (1 << 53);

/// Flags for a Normal, cacheable, inner-shareable, executable RAM block.
/// `AP` left `0b00` (bits[7:6]): read/write at EL1, no EL0 access.
const RAM_BLOCK_FLAGS: u64 = DESC_BLOCK | ATTR_NORMAL | SH_INNER | AF;

/// Flags for a Device, execute-never block. `SH` left `0b00` -- Device
/// memory is not cached, so shareability is not meaningful.
const DEVICE_BLOCK_FLAGS: u64 = DESC_BLOCK | ATTR_DEVICE | AF | EXEC_NEVER;

/// Flags for a RAM block with the caches taken out of the picture: still
/// Normal memory (so unaligned accesses and the compiler's usual load/store
/// merging remain legal), just non-cacheable. Installed by
/// [`set_uncached_block`] over a region shared with the VideoCore; see
/// [`crate::mmu::set_uncached`] for why that is necessary rather than a
/// performance choice.
const UNCACHED_BLOCK_FLAGS: u64 = DESC_BLOCK | ATTR_NORMAL_UNCACHED | SH_INNER | AF;

/// A single 4KB-granule translation table. `UnsafeCell` because every
/// entry is filled in at runtime by [`rpi_hal_mmu_init`] (the level-1
/// entries hold the addresses of the level-2 tables, which are link-time
/// values rather than compile-time constants), and because the MMU's
/// hardware walker reads all of them behind the compiler's back.
#[repr(C, align(4096))]
struct Table(UnsafeCell<[u64; ENTRIES]>);

// SAFETY: these tables are mutated only once, during the single-threaded
// early-boot MMU setup, and thereafter read only by the hardware walker.
unsafe impl Sync for Table {}

/// An all-invalid table, giving the statics below a zero initializer so
/// they land in `.bss` instead of adding 20KB of descriptors to the binary
/// image. `boot64.s` zeroes `.bss` before calling [`rpi_hal_mmu_init`],
/// which then fills them in.
///
/// A `const` rather than a `static` because it is only ever an initializer
/// for the two items below -- including [`L2`]'s array-repeat, which a
/// `static` can't provide. `clippy::declare_interior_mutable_const` warns
/// that each use is a fresh copy of the `UnsafeCell` rather than shared
/// state, which is exactly the intent here: it names the all-zero
/// descriptor pattern, and the `static`s built from it are the things with
/// identity.
#[allow(clippy::declare_interior_mutable_const)]
const EMPTY_TABLE: Table = Table(UnsafeCell::new([0u64; ENTRIES]));

/// The level-1 table: one entry per 1GB region, the low [`L1_REGIONS`] of
/// which are pointed at [`L2`]'s tables.
static L1: Table = EMPTY_TABLE;

/// One level-2 table per covered 1GB region, indexed by region number.
static L2: [Table; L1_REGIONS] = [EMPTY_TABLE; L1_REGIONS];

/// Fills `table` with 2MB identity-mapping blocks for the 1GB region
/// starting at `region * 1GB`, applying the same per-block rule as the
/// AArch32 table: RAM below the peripheral base as Normal, the peripheral
/// block and the ARM-local peripheral block as Device, everything else left
/// invalid.
///
/// A block is classified by its base address, so a peripheral block whose
/// declared end isn't 2MB-aligned is rounded up to the enclosing block --
/// BCM2836/2837's 1MB ARM-local block maps 2MB, for instance. Over-mapping
/// MMIO space as Device is harmless; the alternative is a level-3 table for
/// no benefit.
fn fill_l2(table: &Table, region: usize) {
    let entries = table.0.get() as *mut u64;
    let region_base = region as u64 * BLOCK_1GB;

    for i in 0..ENTRIES {
        let base = region_base + i as u64 * BLOCK_2MB;
        let descriptor = if base < u64::from(PERIPHERAL_BASE) {
            base | RAM_BLOCK_FLAGS
        } else if base <= u64::from(PERIPHERAL_END)
            || (base >= u64::from(LOCAL_PERIPHERAL_BASE) && base <= u64::from(LOCAL_PERIPHERAL_END))
        {
            base | DEVICE_BLOCK_FLAGS
        } else {
            0
        };

        // Volatile because the walker, not this code, is the reader: these
        // are write-only stores as far as the compiler can tell.
        unsafe { entries.add(i).write_volatile(descriptor) };
    }
}

/// Builds the identity-mapped translation tables and enables the MMU.
///
/// Must be called at EL1 (a consumer's boot stub is responsible for the
/// EL2->EL1 drop first). Safe to call once per core: `TTBR0_EL1`/
/// `TCR_EL1`/`MAIR_EL1`/`SCTLR_EL1` and the TLB are per-core state, and
/// the tables are rebuilt identically every time.
///
/// # Safety
///
/// Must only be called early in boot, at EL1, on each core before any
/// code on that core relies on today's MMU-off memory ordering guarantees
/// changing underneath it.
#[no_mangle]
pub unsafe extern "C" fn rpi_hal_mmu_init() {
    unsafe {
        // Build the level-2 tables and point the level-1 table's live
        // entries at them. Identity-mapped and the MMU still off here, so a
        // table's physical address is just its own address.
        let l1 = L1.0.get() as *mut u64;
        for (region, l2) in L2.iter().enumerate() {
            fill_l2(l2, region);
            l1.add(region)
                .write_volatile(l2.0.get() as u64 | DESC_TABLE);
        }

        // MAIR_EL1: attr0 = Normal, Inner+Outer Write-Back non-transient
        // Read/Write-Allocate (0xFF); attr1 = Device-nGnRE (0x04); attr2 =
        // Normal, Inner+Outer Non-cacheable (0x44), used only by blocks
        // `set_uncached_block` rewrites. These are the AttrIndx values
        // baked into the descriptors above.
        let mair: u64 = 0xFF | (0x04 << 8) | (0x44 << 16);
        asm!("msr mair_el1, {0}", in(reg) mair);

        // TCR_EL1: T0SZ=25 (39-bit VA), walk memory Inner+Outer Write-Back
        // Write-Allocate (IRGN0/ORGN0=0b01) and Inner Shareable
        // (SH0=0b11), and EPD1=1 to disable TTBR1 walks (this map is
        // TTBR0-only). The zero-valued fields are left implicit: TG0=0b00
        // (4KB granule) and IPS=0 (32-bit intermediate PA -- every address
        // this crate reaches is well under 4GB).
        let tcr: u64 = 25
            | (0b01 << 8)   // IRGN0
            | (0b01 << 10)  // ORGN0
            | (0b11 << 12)  // SH0
            | (1 << 23); // EPD1
        asm!("msr tcr_el1, {0}", in(reg) tcr);

        // TTBR0_EL1: base of the level-1 table.
        asm!("msr ttbr0_el1, {0}", in(reg) L1.0.get() as u64);

        // Complete the table writes and system-register setup, make them
        // visible to the walker, then invalidate the (stale, firmware-era)
        // TLB before enabling translation.
        asm!("dsb sy");
        asm!("tlbi vmalle1");
        asm!("dsb sy");
        asm!("isb");

        // SCTLR_EL1: read-modify-write to preserve its RES1 bits, setting
        // M (bit 0, MMU), C (bit 2, data cache) and I (bit 12, instruction
        // cache -- safe here given the executable Normal RAM mapping,
        // unlike the AArch32 path which leaves it off).
        let mut sctlr: u64;
        asm!("mrs {0}, sctlr_el1", out(reg) sctlr);
        sctlr |= 1 | (1 << 2) | (1 << 12);
        asm!("msr sctlr_el1, {0}", in(reg) sctlr);

        // Architecturally required after enabling the MMU: the pipeline
        // may have fetched ahead using the old (MMU-off) translation.
        asm!("isb");
    }
}

/// Bytes covered by one level-2 descriptor -- see
/// [`crate::mmu::UNCACHED_GRANULE`].
pub(super) const UNCACHED_GRANULE: usize = BLOCK_2MB as usize;

/// Rewrites the level-2 block descriptor covering `base` with
/// [`UNCACHED_BLOCK_FLAGS`], then makes the change take effect everywhere.
///
/// No cache maintenance on the descriptor itself: `TCR_EL1` programs table
/// walks as Write-Back cacheable and Inner Shareable (see
/// [`rpi_hal_mmu_init`]), so the walker reads through the same coherent
/// caches this store lands in, and a `dsb` is all that is needed to order
/// it before the TLB operation. That operation is the inner-shareable,
/// by-address form (`tlbi vaae1is`, whose operand is the virtual address
/// shifted right by 12), so secondary cores walking these same tables drop
/// their stale entry for this block too.
///
/// # Safety
///
/// `base` must be 2MB-aligned and within the RAM these tables map as Normal
/// memory; see [`crate::mmu::set_uncached`], which checks both and is the
/// only caller.
pub(super) unsafe fn set_uncached_block(base: u32) {
    let base = u64::from(base);
    let region = (base / BLOCK_1GB) as usize;
    let index = ((base % BLOCK_1GB) / BLOCK_2MB) as usize;

    let entries = L2[region].0.get() as *mut u64;
    unsafe {
        entries
            .add(index)
            .write_volatile(base | UNCACHED_BLOCK_FLAGS)
    };

    unsafe {
        asm!("dsb ishst");
        asm!("tlbi vaae1is, {0}", in(reg) base >> 12);
        asm!("dsb ish");
        asm!("isb");
    }
}
