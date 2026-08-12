//! The one piece of the SoC memory map every build needs regardless of
//! feature selection: the peripheral block's physical base address.
//!
//! Kept separate from `mmu.rs` (compiled only behind the `mmu` feature)
//! because [`crate::dma`] and [`crate::watchdog`] need this address
//! unconditionally -- their register blocks sit at a `PERIPHERAL_BASE`-
//! relative offset whether or not the MMU feature is in use. The rest of
//! the memory map (the peripheral block's end, and the ARM-local block)
//! is only ever consulted while building the MMU's identity map, so it
//! stays in `mmu.rs`.

/// Physical base address of the peripheral block: BCM2836/2837 unless
/// the `bcm2711` feature selects the BCM2711's relocated one. The
/// BCM2836/2837 value is confirmed against this crate's own peripheral
/// access types (e.g. the System Timer at `0x3f00_3000`, the legacy
/// interrupt controller at `0x3f00_b000`, out to `0x3f98_0e00` for USB),
/// not assumed from a datasheet.
#[cfg(not(feature = "bcm2711"))]
pub(crate) const PERIPHERAL_BASE: u32 = 0x3F00_0000;
/// BCM2711 low-peripheral-mode base, from Linux's `bcm2711.dtsi` (the
/// peripheral block's `ranges` entry mapping bus address `0x7e00_0000`
/// to this physical address) -- not yet cross-checked against this
/// crate's own drivers the way the BCM2836/2837 value above was, since
/// there's no BCM2711 target to build until this const exists.
#[cfg(feature = "bcm2711")]
pub(crate) const PERIPHERAL_BASE: u32 = 0xFE00_0000;
