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

/// The VideoCore mailbox register block.
///
/// Re-exported from `bcm2711-lpa` for the same reason as [`systmr`]: the
/// BCM2712's mailbox is the same `brcm,bcm2835-mbox` block, so only
/// [`VCMAILBOX::PTR`] differs.
pub use bcm2711_lpa::vcmailbox;

/// The SD host controller register block.
///
/// Re-exported from `bcm2711-lpa`'s `EMMC`, whose layout is the standard
/// SDHCI register map the BCM2712's `brcm,bcm2712-sdhci` controller
/// follows (Linux drives it with the generic `sdhci` core plus a few
/// Broadcom-specific writes to a separate block, [`SDIO1::CFG`]).
pub use bcm2711_lpa::emmc;

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

/// The VideoCore mailbox, through which the firmware's property interface
/// is reached.
pub struct VCMAILBOX {
    _marker: PhantomData<*const ()>,
}

// SAFETY: see `SYSTMR`'s.
unsafe impl Send for VCMAILBOX {}

impl VCMAILBOX {
    /// Pointer to the register block.
    pub const PTR: *const vcmailbox::RegisterBlock = (SOC_BASE + 0x7c01_3880) as *const _;

    /// Returns the pointer to the register block.
    #[inline(always)]
    pub const fn ptr() -> *const vcmailbox::RegisterBlock {
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

impl Deref for VCMAILBOX {
    type Target = vcmailbox::RegisterBlock;

    /// The register block at [`VCMAILBOX::PTR`].
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        unsafe { &*Self::PTR }
    }
}

impl core::fmt::Debug for VCMAILBOX {
    /// Prints the token's name; it has no state to show.
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_struct("VCMAILBOX").finish()
    }
}

/// The SD host controller wired to the Pi 5's card slot (`sdio1` in
/// `bcm2712.dtsi`, `mmc@fff000`).
pub struct SDIO1 {
    _marker: PhantomData<*const ()>,
}

// SAFETY: see `SYSTMR`'s.
unsafe impl Send for SDIO1 {}

impl SDIO1 {
    /// Pointer to the SDHCI register block (the node's `host` region).
    pub const PTR: *const emmc::RegisterBlock = (SOC_BASE + 0x00ff_f000) as *const _;

    /// Address of the controller's Broadcom configuration block (the
    /// node's `cfg` region), outside the SDHCI register map: pin and
    /// timing selection, and a card-detect override.
    pub const CFG: usize = SOC_BASE + 0x00ff_f400;

    /// Returns the pointer to the register block.
    #[inline(always)]
    pub const fn ptr() -> *const emmc::RegisterBlock {
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

impl Deref for SDIO1 {
    type Target = emmc::RegisterBlock;

    /// The register block at [`SDIO1::PTR`].
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        unsafe { &*Self::PTR }
    }
}

impl core::fmt::Debug for SDIO1 {
    /// Prints the token's name; it has no state to show.
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_struct("SDIO1").finish()
    }
}

/// RP1's I2C1, the controller the header's GPIO2/3 (SDA1/SCL1) route to.
///
/// Named `BSC1` after the BCM2835 controller in that role on every earlier
/// Pi, so code naming the header bus builds on both, but the hardware is
/// different: a Synopsys DesignWare I2C (`snps,designware-i2c` in
/// `rp1.dtsi`), driven by [`crate::i2c`]. No `Deref`: the driver reaches
/// its registers through [`BSC1::ADDR`].
pub struct BSC1 {
    _marker: PhantomData<*const ()>,
}

// SAFETY: see `SYSTMR`'s.
unsafe impl Send for BSC1 {}

impl BSC1 {
    /// Address of the controller's register block.
    pub const ADDR: usize = RP1_BASE + 0x0007_4000;

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

impl core::fmt::Debug for BSC1 {
    /// Prints the token's name; it has no state to show.
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_struct("BSC1").finish()
    }
}

/// RP1's I2C0, the controller the header's GPIO0/1 (`ID_SD`/`ID_SC`, the
/// HAT ID EEPROM bus) route to. Named `BSC0` for the same reason as
/// [`BSC1`], and the same DesignWare hardware.
pub struct BSC0 {
    _marker: PhantomData<*const ()>,
}

// SAFETY: see `SYSTMR`'s.
unsafe impl Send for BSC0 {}

impl BSC0 {
    /// Address of the controller's register block.
    pub const ADDR: usize = RP1_BASE + 0x0007_0000;

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

impl core::fmt::Debug for BSC0 {
    /// Prints the token's name; it has no state to show.
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_struct("BSC0").finish()
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

/// RP1's PWM0, the PWM block whose four channels reach the header (GPIO12,
/// 13, 18 and 19), driven by [`crate::pwm`].
///
/// Not the BCM `PWM0` of the earlier chips: RP1's is a different design,
/// four channels with 32-bit counters, its registers set out in the RP1
/// Peripherals datasheet's PWM chapter. No `Deref`: the driver reaches its
/// registers through [`PWM0::ADDR`].
pub struct PWM0 {
    _marker: PhantomData<*const ()>,
}

// SAFETY: see `SYSTMR`'s.
unsafe impl Send for PWM0 {}

impl PWM0 {
    /// Address of the block's register file.
    pub const ADDR: usize = RP1_BASE + 0x0009_8000;

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

impl core::fmt::Debug for PWM0 {
    /// Prints the token's name; it has no state to show.
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_struct("PWM0").finish()
    }
}

/// RP1's PWM1, the same design as [`PWM0`]. Its only board use is the fan
/// connector: channel 3 drives `FAN_PWM` (GPIO45).
pub struct PWM1 {
    _marker: PhantomData<*const ()>,
}

// SAFETY: see `SYSTMR`'s.
unsafe impl Send for PWM1 {}

impl PWM1 {
    /// Address of the block's register file.
    pub const ADDR: usize = RP1_BASE + 0x0009_c000;

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

impl core::fmt::Debug for PWM1 {
    /// Prints the token's name; it has no state to show.
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_struct("PWM1").finish()
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
    /// The VideoCore mailbox.
    pub VCMAILBOX: VCMAILBOX,
    /// The System Timer.
    pub SYSTMR: SYSTMR,
    /// The SD card's host controller.
    pub SDIO1: SDIO1,
    /// RP1's UART0.
    pub UART0: UART0,
    /// RP1's I2C0 (the HAT ID bus).
    pub BSC0: BSC0,
    /// RP1's I2C1 (the header's I2C bus).
    pub BSC1: BSC1,
    /// RP1's header GPIO bank.
    pub GPIO: GPIO,
    /// RP1's PWM0 (the header's PWM channels).
    pub PWM0: PWM0,
    /// RP1's PWM1 (the fan connector).
    pub PWM1: PWM1,
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
            VCMAILBOX: unsafe { VCMAILBOX::steal() },
            SYSTMR: unsafe { SYSTMR::steal() },
            SDIO1: unsafe { SDIO1::steal() },
            UART0: unsafe { UART0::steal() },
            BSC0: unsafe { BSC0::steal() },
            BSC1: unsafe { BSC1::steal() },
            GPIO: unsafe { GPIO::steal() },
            PWM0: unsafe { PWM0::steal() },
            PWM1: unsafe { PWM1::steal() },
            GIO_AON: unsafe { GIO_AON::steal() },
        }
    }
}
