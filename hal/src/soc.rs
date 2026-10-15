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

/// BCM2711 low-peripheral-mode base, from Linux's `bcm2711.dtsi` (the
/// peripheral block's `ranges` entry mapping bus address `0x7e00_0000`
/// to this physical address) -- not yet cross-checked against this
/// crate's own drivers the way the BCM2836/2837 value below was, since
/// there's no BCM2711 target to build until this const exists.
#[cfg(feature = "bcm2711")]
pub(crate) const PERIPHERAL_BASE: u32 = 0xFE00_0000;
/// BCM2836/2837 base, confirmed against this crate's own peripheral
/// access types (e.g. the System Timer at `0x3f00_3000`, the legacy
/// interrupt controller at `0x3f00_b000`, out to `0x3f98_0e00` for USB),
/// not assumed from a datasheet.
#[cfg(all(not(feature = "bcm2711"), feature = "bcm2837"))]
pub(crate) const PERIPHERAL_BASE: u32 = 0x3F00_0000;
/// BCM2835 base -- the original, un-relocated one every later chip moved
/// away from. Confirmed the same way as the BCM2836/2837 value above,
/// against `bcm2835-lpa`'s own addresses (System Timer at `0x2000_3000`,
/// legacy interrupt controller at `0x2000_b000`, out to `0x2098_0e00`
/// for USB).
///
/// This arm is not gated on `feature = "bcm2835"`, unlike the two above:
/// it is also what a build with *no* chip feature at all gets, so that
/// [`crate::dma`] and [`crate::watchdog`] still resolve this const and
/// the no-chip diagnostic stays the short one `lib.rs`'s
/// `compile_error!` is written to produce, rather than being buried under
/// every use of a missing address.
#[cfg(all(not(feature = "bcm2711"), not(feature = "bcm2837")))]
pub(crate) const PERIPHERAL_BASE: u32 = 0x2000_0000;
