//! Peripheral access for the BCM2712 (Pi 5), hand-written because no
//! generated PAC exists for it.
//!
//! Shaped like the `svd2rust` crates the other chips re-export as `pac` —
//! zero-sized ownership tokens, a [`Peripherals`] struct handing out one of
//! each, `PTR`/`ptr()`/`steal()` on every token — so code written as
//! `pac::Peripherals::take()` followed by a driver constructor that consumes
//! a token reads the same on every chip.
//!
//! Only what this crate's BCM2712 drivers use is here. Addresses come from
//! Raspberry Pi's `bcm2712.dtsi`/`rp1.dtsi`, translated to ARM physical
//! addresses through the bus `ranges` those files declare:
//!
//! - The SoC's own peripherals sit on a bus whose address 0 is physical
//!   [`SOC_BASE`] (`soc` node: `ranges = <0x0 0x10 0x0 0x80000000>`), so
//!   the System Timer at bus `0x7c00_3000` is physical `0x10_7c00_3000`.
//! - The RP1 south bridge is a PCIe device. Its peripheral window starts at
//!   physical [`RP1_BASE`] (`pcie2`'s outbound `ranges` put PCIe address 0
//!   there, and `&rp1`'s `ranges` put RP1's `0xc0_4000_0000` at PCIe 0), so
//!   an RP1 block at `0xc0_400d_0000` is physical `0x1f_000d_0000`.
//!
//! RP1 is only reachable if the firmware leaves the PCIe link it trained up:
//! `pciex4_reset=0` in `config.txt`. With the default (`1`) it resets the
//! controller before starting the kernel, and the first access to anything
//! behind [`RP1_BASE`] stalls the core with no fault to report it.

#![allow(non_camel_case_types, non_snake_case)]

use core::marker::PhantomData;
use core::ops::Deref;
use core::sync::atomic::{AtomicBool, Ordering};

/// The System Timer register block.
///
/// Re-exported from `bcm2711-lpa` rather than written out again: the
/// BCM2712's System Timer is the same `brcm,bcm2835-system-timer` block every
/// earlier chip has, at a new address, so only [`SYSTMR::PTR`] differs.
pub use bcm2711_lpa::systmr;

/// The PL011 UART register block.
///
/// Re-exported from `bcm2711-lpa` for the same reason as [`systmr`]: RP1's
/// UARTs are ARM PL011s (`arm,pl011-axi`), the IP every earlier Pi's UART0
/// is, so only [`UART0::PTR`] differs.
pub use bcm2711_lpa::uart0;

/// Physical address of bus address 0 on the SoC's peripheral bus.
pub const SOC_BASE: usize = 0x10_0000_0000;

/// Physical address of the start of RP1's peripheral window.
pub const RP1_BASE: usize = 0x1f_0000_0000;

/// The BCM System Timer: a free-running 1 MHz counter plus four compare
/// registers.
pub struct SYSTMR {
    _marker: PhantomData<*const ()>,
}

// SAFETY: a token is a capability, not shared state; moving it to another
// core moves the right to touch the block, which is what `Send` means here.
unsafe impl Send for SYSTMR {}

impl SYSTMR {
    /// Pointer to the register block.
    pub const PTR: *const systmr::RegisterBlock = (SOC_BASE + 0x7c00_3000) as *const _;

    /// Returns the pointer to the register block.
    #[inline(always)]
    pub const fn ptr() -> *const systmr::RegisterBlock {
        Self::PTR
    }

    /// Steals an instance of this peripheral.
    ///
    /// # Safety
    ///
    /// The new instance must not be used in a way that races with any
    /// existing one, and must not be handed to a driver that relies on
    /// being the only owner.
    pub unsafe fn steal() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl Deref for SYSTMR {
    type Target = systmr::RegisterBlock;

    /// The register block at [`SYSTMR::PTR`].
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        unsafe { &*Self::PTR }
    }
}

impl core::fmt::Debug for SYSTMR {
    /// Prints the token's name; it has no state to show.
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_struct("SYSTMR").finish()
    }
}

/// RP1's UART0, the PL011 that GPIO14/15 on the header route to.
pub struct UART0 {
    _marker: PhantomData<*const ()>,
}

// SAFETY: see `SYSTMR`'s.
unsafe impl Send for UART0 {}

impl UART0 {
    /// Pointer to the register block.
    pub const PTR: *const uart0::RegisterBlock = (RP1_BASE + 0x0003_0000) as *const _;

    /// Returns the pointer to the register block.
    #[inline(always)]
    pub const fn ptr() -> *const uart0::RegisterBlock {
        Self::PTR
    }

    /// Steals an instance of this peripheral.
    ///
    /// # Safety
    ///
    /// See [`SYSTMR::steal`].
    pub unsafe fn steal() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl Deref for UART0 {
    type Target = uart0::RegisterBlock;

    /// The register block at [`UART0::PTR`].
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        unsafe { &*Self::PTR }
    }
}

impl core::fmt::Debug for UART0 {
    /// Prints the token's name; it has no state to show.
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_struct("UART0").finish()
    }
}

/// RP1's user GPIO bank (bank 0: GPIO0-27, the 40-pin header), across the
/// three register blocks that together control one pin.
///
/// No `Deref`: unlike a generated PAC's GPIO block this is not one
/// contiguous register file, so the token exposes each block's address
/// instead and [`crate::gpio`] reaches the registers through them.
///
/// - [`IO_BANK0`](Self::IO_BANK0): per-pin `STATUS`/`CTRL` pairs, whose
///   `CTRL.FUNCSEL` picks what drives the pin.
/// - [`SYS_RIO0`](Self::SYS_RIO0): "registered I/O" — the `OUT`/`OE`/`IN`
///   bit-per-pin registers that software drives the pin through once
///   `FUNCSEL` selects it.
/// - [`PADS_BANK0`](Self::PADS_BANK0): per-pin pad control (pulls, input
///   enable, output disable, drive strength).
pub struct GPIO {
    _marker: PhantomData<*const ()>,
}

// SAFETY: see `SYSTMR`'s.
unsafe impl Send for GPIO {}

impl GPIO {
    /// Address of RP1's `IO_BANK0` block.
    pub const IO_BANK0: usize = RP1_BASE + 0x000d_0000;
    /// Address of RP1's `SYS_RIO0` block.
    pub const SYS_RIO0: usize = RP1_BASE + 0x000e_0000;
    /// Address of RP1's `PADS_BANK0` block.
    pub const PADS_BANK0: usize = RP1_BASE + 0x000f_0000;

    /// Steals an instance of this peripheral.
    ///
    /// # Safety
    ///
    /// See [`SYSTMR::steal`].
    pub unsafe fn steal() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl core::fmt::Debug for GPIO {
    /// Prints the token's name; it has no state to show.
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_struct("GPIO").finish()
    }
}

/// The BCM2712's always-on GPIO block (`gio_aon`): 23 lines in two banks
/// (17 + 6) of Broadcom's set-top-box GPIO design, wired to board
/// functions — on a Pi 5, line 9 is the green ACT LED.
pub struct GIO_AON {
    _marker: PhantomData<*const ()>,
}

// SAFETY: see `SYSTMR`'s.
unsafe impl Send for GIO_AON {}

impl GIO_AON {
    /// Pointer to the first bank's registers.
    pub const PTR: *const u32 = (SOC_BASE + 0x7d51_7c00) as *const _;

    /// Returns the pointer to the first bank's registers.
    #[inline(always)]
    pub const fn ptr() -> *const u32 {
        Self::PTR
    }

    /// Steals an instance of this peripheral.
    ///
    /// # Safety
    ///
    /// See [`SYSTMR::steal`].
    pub unsafe fn steal() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl core::fmt::Debug for GIO_AON {
    /// Prints the token's name; it has no state to show.
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_struct("GIO_AON").finish()
    }
}

/// Whether [`Peripherals`] has been handed out. Only ever loaded and
/// stored inside a critical section, so plain loads and stores suffice and
/// no exclusive-access instructions are needed.
static TAKEN: AtomicBool = AtomicBool::new(false);

/// All the peripherals.
pub struct Peripherals {
    /// The System Timer.
    pub SYSTMR: SYSTMR,
    /// RP1's UART0.
    pub UART0: UART0,
    /// RP1's header GPIO bank.
    pub GPIO: GPIO,
    /// The always-on GPIO block.
    pub GIO_AON: GIO_AON,
}

impl Peripherals {
    /// Returns all the peripherals once; `None` on every later call.
    pub fn take() -> Option<Self> {
        critical_section::with(|_| {
            if TAKEN.load(Ordering::Relaxed) {
                return None;
            }
            Some(unsafe { Self::steal() })
        })
    }

    /// Unchecked version of [`Peripherals::take`].
    ///
    /// # Safety
    ///
    /// Each of the returned peripherals must be used at most once.
    pub unsafe fn steal() -> Self {
        TAKEN.store(true, Ordering::Relaxed);
        Self {
            SYSTMR: unsafe { SYSTMR::steal() },
            UART0: unsafe { UART0::steal() },
            GPIO: unsafe { GPIO::steal() },
            GIO_AON: unsafe { GIO_AON::steal() },
        }
    }
}
