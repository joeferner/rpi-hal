//! Hardware abstraction layer for bare-metal Raspberry Pi.
//!
//! Built around the BCM2836/BCM2837 (Pi 2, Pi 3), with preliminary
//! support for the BCM2711 (Pi 4) and the BCM2835 (Pi 1, Pi Zero) —
//! see the crate's README for what is verified on each. Exactly one of
//! the `bcm2837`/`bcm2711`/`bcm2835` features selects the chip; none is
//! a default, since there is no sensible default target chip.

#![no_std]
#![deny(missing_docs)]
// docs.rs builds with `--cfg docsrs` (see `[package.metadata.docs.rs]` in
// Cargo.toml), which turns on rustdoc labelling every feature-gated item
// with the feature that provides it -- without it a reader sees modules
// like `gpio::asynch` with no indication that a feature is needed to get
// them. Gated on the cfg rather than unconditional because it's a
// nightly-only rustdoc feature.
#![cfg_attr(docsrs, feature(doc_cfg))]

/// Audio out through the VideoCore firmware's `ril.audio_render` MMAL
/// component — HDMI, or the analog jack.
#[cfg(feature = "mmal")]
pub mod audio_render;
/// Blocking driver for the auxiliary SPI controllers SPI1 and SPI2.
pub mod aux_spi;
mod barrier;
/// HCI bring-up for the on-board BCM43438 Bluetooth controller (Pi 3):
/// H4 transport, `.hcd` patchram firmware load, and local
/// version/address readback.
pub mod bluetooth;
#[cfg(feature = "rt")]
mod boot;
mod cache;
mod clock_manager;
/// Identity of the core executing this code — see the module's own doc
/// comment for why this is not part of `multicore`.
pub mod cpu;
#[cfg(feature = "rt")]
mod critical_section;
/// Blocking driver for the DMA controller.
pub mod dma;
mod emmc;
// Installs `__unhandled_exception` rather than offering an API, so there
// is nothing here to make public -- see the module's own documentation.
#[cfg(feature = "fault-report")]
mod fault;
/// Enabling the hardware floating-point / SIMD unit (VFP + NEON).
pub mod fpu;
/// Per-core ARM generic (architected) timer: monotonic counter, blocking
/// delays, and interrupt-driven deadlines.
///
/// Not built for ARMv6 (BCM2835): the generic timer is an ARMv7-A
/// extension, and the per-core control and routing registers this driver
/// programs live in an ARM-local peripheral block the BCM2835 does not
/// have. [`timer`] — the BCM System Timer, a peripheral rather than CPU
/// state — is the monotonic clock there.
#[cfg(not(armv6))]
pub mod generic_timer;
/// Typestated GPIO pin wrapper with `embedded-hal` `digital` trait
/// implementations.
pub mod gpio;
/// Transport-independent HID report descriptor parser, shared by the USB
/// and Bluetooth HID hosts — see [`hid_report::ReportDescriptor`].
pub mod hid_report;
/// Blocking driver for I2C1 (BSC1).
pub mod i2c;
#[cfg(feature = "rt")]
/// CPU-level IRQ enable/disable and the exception vector table.
pub mod irq;
/// Safe wrapper around the BCM2836/2837 legacy interrupt controller.
///
/// Not built under `bcm2711`: BCM2711 wires a different set of
/// peripherals through the surviving legacy IC block (its PAC's bit
/// names for the shared pending/enable registers genuinely differ, not
/// just cosmetically), and it's superseded there by GIC-400 anyway --
/// a real, separate driver this crate doesn't have yet. No interrupt
/// controller is available under `bcm2711` until that lands, so nothing
/// interrupt-driven works there either.
#[cfg(not(feature = "bcm2711"))]
pub mod lic;
/// Blocking driver for the VideoCore mailbox property interface.
pub mod mailbox;
/// Blocking driver for the mini UART (UART1).
pub mod mini_uart;
/// Client for the VideoCore firmware's MMAL multimedia framework, over
/// [`vchiq`] — components, ports, parameters, and buffer exchange.
#[cfg(feature = "mmal")]
pub mod mmal;
/// Identity-mapped MMU bring-up, plus the one thing a driver may need to
/// change about the map afterwards: taking a region of RAM out of the
/// caches ([`mmu::set_uncached`]), for memory a bus master writes
/// concurrently with this core.
#[cfg(feature = "mmu")]
pub mod mmu;
#[cfg(all(feature = "rt", feature = "multicore", not(armv6)))]
/// Bringing up secondary cores (1-3) — see the module's own doc
/// comment for the wake-up handoff this builds on.
///
/// Not built for ARMv6 (BCM2835): that chip has one core, and no
/// ARM-local mailbox registers for a second one to be woken through. The
/// `multicore` feature is rejected outright there rather than quietly
/// producing a build without this module — see the `compile_error!`
/// below.
pub mod multicore;
// A consumer who asked for `multicore` on a uniprocessor chip asked for
// something that cannot be delivered, and the failure to be loud about is
// the silent one: without this, the feature would simply compile away and
// the first sign would be `rpi_hal::multicore` not resolving.
#[cfg(all(feature = "multicore", armv6))]
compile_error!(
    "the `multicore` feature is not available on ARMv6: the BCM2835 (Pi 1, \
     Pi Zero) is a single-core part. Drop the feature -- note that it is \
     implied by nothing else, so nothing else has to change."
);
/// Reference I2C driver for the OV5647 (Raspberry Pi Camera v1) image
/// sensor — a third-party device (paired with [`unicam`]), not a SoC
/// peripheral.
pub mod ov5647;
/// Blocking bring-up for the PCM / I2S peripheral (digital audio out to an
/// external I2S DAC).
pub mod pcm;
/// The PMU cycle counter — a cheap per-core CPU-cycle clock for profiling
/// work too short for the System Timer to measure without disturbing it.
///
/// Not built for ARMv6 (BCM2835): the ARM1176 has performance counters,
/// but behind entirely different CP15 registers from the ARMv7-A PMU this
/// driver programs. [`timer`] is the clock available there.
#[cfg(not(armv6))]
pub mod pmu;
/// Board reboot and shutdown via the Power Management (PM) block.
pub mod power;
/// Blocking driver for the PWM controller.
pub mod pwm;
/// Blocking driver for the hardware random number generator.
pub mod rng;
/// Blocking driver for the on-board SD card slot (Arasan EMMC host
/// controller).
pub mod sd;
/// Blocking driver for the SD card slot over the *other* host
/// controller, SDHOST — the one that leaves the Arasan controller free
/// for [`sdio`], so a board can drive its card and its wireless chip at
/// the same time.
///
/// Not built for BCM2711: a Pi 4 drives its card from EMMC2, which is a
/// separate block again, and GPIO48-53 there are the Ethernet PHY's
/// RGMII interface rather than the card slot — so routing them here
/// would be actively wrong rather than merely useless.
#[cfg(not(feature = "bcm2711"))]
pub mod sdhost;
/// Blocking SDIO driver for the on-board BCM43438 wireless chip (Wi-Fi
/// side), via the same Arasan EMMC host controller.
pub mod sdio;
mod soc;
/// Blocking driver for SPI0.
pub mod spi;
/// Extent of the main stack and how much of it is left — see
/// [`stack::headroom`].
///
/// Needs the `rt` feature, since the region it reports on is the one
/// this crate's linker script reserves and its boot code points `sp`
/// at. A program supplying its own boot code and linker script owns
/// that layout instead.
#[cfg(feature = "rt")]
pub mod stack;
/// BCM System Timer driver: free-running microsecond counter and delays.
pub mod timer;
/// Driver for the DSI touchscreen's touch input (the firmware-mediated
/// shared touch buffer, not I2C).
pub mod touch;
/// Blocking driver for UART0 (PL011).
pub mod uart;
/// Blocking driver for the Unicam CSI-2 camera receiver.
pub mod unicam;
/// USB host support for the built-in DWC2 OTG controller.
pub mod usb;
/// Bring-up for the V3D 3D pipeline (VideoCore IV's QPU shader cores).
/// Pi 3 (BCM2836/BCM2837) only.
#[cfg(feature = "v3d")]
pub mod v3d;
/// VCHIQ: the shared-memory, doorbell-signalled message transport to the
/// VideoCore firmware — everything the simple request/response
/// [`mailbox`] can't carry.
#[cfg(feature = "vchiq")]
pub mod vchiq;
/// Hardware H.264 video decode, driven through the VideoCore firmware's
/// `ril.video_decode` MMAL component.
#[cfg(feature = "mmal")]
pub mod video_decode;
/// Blocking driver for the PM block's watchdog timer.
pub mod watchdog;
/// Host control protocol (SDPCM/CDC) for the on-board BCM43430 Wi-Fi
/// chip, on top of `sdio`.
pub mod wifi;

/// Re-export of the underlying peripheral access crate, for direct
/// register access alongside this crate's higher-level wrappers.
/// Whichever of `bcm2837`/`bcm2711`/`bcm2835` is enabled (see
/// `Cargo.toml`) -- with more than one on, the precedence is
/// `bcm2711` > `bcm2837` > `bcm2835`, newest chip first, and the same
/// order every other chip branch in this crate follows.
#[cfg(feature = "bcm2711")]
pub use bcm2711_lpa as pac;
#[cfg(all(
    not(feature = "bcm2711"),
    not(feature = "bcm2837"),
    feature = "bcm2835"
))]
pub use bcm2835_lpa as pac;
#[cfg(all(not(feature = "bcm2711"), feature = "bcm2837"))]
pub use bcm2837_lpa as pac;
// With neither feature on there is no `pac` at all, and every module that
// reaches for a register fails to resolve it -- 18 errors naming
// `crate::pac` and none of them saying a chip has to be chosen. This is
// the third arm of the re-export above, and it puts that sentence in
// front of them: expansion runs before name resolution, so this is the
// first error printed, and rustc gives up early enough afterwards that
// the tail shrinks to two (the unresolved `crate::pac`, and an
// `i2c.rs`/`init` duplicate-definition that only looks like one because
// the two impls' distinguishing types come from `pac`). Not silenced
// entirely -- that would mean stubbing out a fake `pac`, a worse trade
// than a short cascade under a clear headline.
#[cfg(not(any(feature = "bcm2837", feature = "bcm2711", feature = "bcm2835")))]
compile_error!(
    "no chip selected: rpi-hal needs exactly one of the `bcm2837` (Pi 2, Pi 3), \
     `bcm2711` (Pi 4) or `bcm2835` (Pi 1, Pi Zero) features. None is a default, \
     since there is no sensible default target chip -- add one to this crate's \
     dependency entry, e.g. \
     `rpi-hal = { version = \"0.1\", features = [\"bcm2837\"] }`."
);

/// Halts the calling core forever, parking it in a low-power
/// wait-for-event loop.
///
/// Useful as the terminal state of a bare-metal `kmain` or as the tail
/// of a `#[panic_handler]`, where there is nothing left to do and no OS
/// to return to.
pub fn halt() -> ! {
    loop {
        unsafe { core::arch::asm!("wfe") };
    }
}
