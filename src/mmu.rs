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
