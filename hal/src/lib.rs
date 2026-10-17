//! Hardware abstraction layer for bare-metal Raspberry Pi.
//!
//! Built around the BCM2836/BCM2837 (Pi 2, Pi 3), with preliminary
//! support for the BCM2711 (Pi 4), the BCM2835 (Pi 1, Pi Zero) and the
//! BCM2712 (Pi 5) — see the crate's README for what is verified on each.
//! Exactly one of the `bcm2837`/`bcm2711`/`bcm2835`/`bcm2712` features
//! selects the chip; none is a default, since there is no sensible default
//! target chip.

#![no_std]
#![deny(missing_docs)]
// docs.rs builds with `--cfg docsrs` (see `[package.metadata.docs.rs]` in
// Cargo.toml), which turns on rustdoc labelling every feature-gated item
// with the feature that provides it -- without it a reader sees modules
// like `gpio::asynch` with no indication that a feature is needed to get
// them. Gated on the cfg rather than unconditional because it's a
// nightly-only rustdoc feature.
#![cfg_attr(docsrs, feature(doc_cfg))]

// The BCM2712 (Pi 5) builds only a handful of the modules below, and the
// `not(feature = "bcm2712")` on the rest is that list. Each one left out
// drives a peripheral the BCM2712 does not have where the driver expects
// it: most of the I/O moved onto the RP1 south bridge as different IP
// (RP1's GPIO, for one, shares nothing with the BCM283x block), and what
// stayed on the SoC moved address. What remains is CPU state (caches, MMU,
// FPU, PMU, exception vectors), the System Timer, and the two GPIO drivers
// written for this chip. A module rejoins once it has been checked against
// the BCM2712's own block, not before: one quietly built against the wrong
// register map would compile and then hang on its first access.

/// Audio out through the VideoCore firmware's `ril.audio_render` MMAL
/// component — HDMI, or the analog jack.
#[cfg(feature = "mmal")]
pub mod audio_render;
/// Blocking driver for the auxiliary SPI controllers SPI1 and SPI2.
#[cfg(not(feature = "bcm2712"))]
pub mod aux_spi;
mod barrier;
/// HCI bring-up for the on-board BCM43438 Bluetooth controller (Pi 3):
/// H4 transport, `.hcd` patchram firmware load, and local
/// version/address readback.
#[cfg(not(feature = "bcm2712"))]
pub mod bluetooth;
#[cfg(feature = "rt")]
mod boot;
mod cache;
#[cfg(not(feature = "bcm2712"))]
mod clock_manager;
/// Identity of the core executing this code — see the module's own doc
/// comment for why this is not part of `multicore`.
pub mod cpu;
#[cfg(feature = "rt")]
mod critical_section;
/// Blocking driver for the DMA controller.
#[cfg(not(feature = "bcm2712"))]
pub mod dma;
#[cfg(not(feature = "bcm2712"))]
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
///
/// Not built for BCM2712 yet either: the counter itself is CPU state, but
/// the per-core control registers this driver programs sit in the
/// BCM2836's ARM-local block, which the BCM2712 does not have.
#[cfg(not(any(armv6, feature = "bcm2712")))]
pub mod generic_timer;
/// The BCM2712's always-on GPIO block, which carries the Pi 5's ACT LED.
#[cfg(feature = "bcm2712")]
pub mod gio_aon;
/// Typestated GPIO pin wrapper with `embedded-hal` `digital` trait
/// implementations.
#[cfg(not(feature = "bcm2712"))]
pub mod gpio;
/// Typestated GPIO pin wrapper with `embedded-hal` `digital` trait
/// implementations, over the header pins on the Pi 5's RP1.
#[cfg(feature = "bcm2712")]
#[path = "gpio_rp1.rs"]
pub mod gpio;
/// Transport-independent HID report descriptor parser, shared by the USB
/// and Bluetooth HID hosts — see [`hid_report::ReportDescriptor`].
#[cfg(not(feature = "bcm2712"))]
pub mod hid_report;
/// Blocking driver for I2C1 (BSC1).
#[cfg(not(feature = "bcm2712"))]
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
#[cfg(not(any(feature = "bcm2711", feature = "bcm2712")))]
pub mod lic;
/// Blocking driver for the VideoCore mailbox property interface.
#[cfg(not(feature = "bcm2712"))]
pub mod mailbox;
/// Where the loaded image ends and free RAM begins — see
/// [`mem::heap_region`], which is how a program sizes a heap without
/// hardcoding a board's memory split.
///
/// Needs the `rt` feature, since the boundary it reports is the one this
/// crate's linker script places. A program supplying its own script owns
/// that layout instead.
#[cfg(all(feature = "rt", not(feature = "bcm2712")))]
pub mod mem;
/// Blocking driver for the mini UART (UART1).
#[cfg(not(feature = "bcm2712"))]
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
// The same loud failure for the features whose subsystems the BCM2712 does
// not have yet. Each is a feature a consumer opts into by name, so it is
// asking for something specific, and quietly compiling it away would leave
// them a missing module or a link error instead of a reason.
#[cfg(all(feature = "bcm2712", feature = "multicore"))]
compile_error!(
    "the `multicore` feature is not available on the BCM2712 (Pi 5) yet: its \
     secondary cores are started through PSCI calls to the boot firmware, \
     not the ARM-local mailboxes this crate's `multicore` uses."
);
#[cfg(all(feature = "bcm2712", feature = "fault-report"))]
compile_error!(
    "the `fault-report` feature is not available on the BCM2712 (Pi 5) yet: \
     its handler re-initialises UART0 from inside an exception, which has \
     not been checked against that chip's RP1-hosted UART."
);
#[cfg(all(feature = "bcm2712", any(feature = "vchiq", feature = "v3d")))]
compile_error!(
    "the `vchiq`, `mmal` and `v3d` features are not available on the BCM2712 \
     (Pi 5): they drive the VideoCore IV-era firmware interfaces and GPU."
);
/// Reference I2C driver for the OV5647 (Raspberry Pi Camera v1) image
/// sensor — a third-party device (paired with [`unicam`]), not a SoC
/// peripheral.
#[cfg(not(feature = "bcm2712"))]
pub mod ov5647;
/// Peripheral access for the BCM2712 (Pi 5): the hand-written stand-in for
/// the generated PAC the other chips re-export under this name.
#[cfg(feature = "bcm2712")]
#[path = "pac_bcm2712.rs"]
pub mod pac;
/// Blocking bring-up for the PCM / I2S peripheral (digital audio out to an
/// external I2S DAC).
#[cfg(not(feature = "bcm2712"))]
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
#[cfg(not(feature = "bcm2712"))]
pub mod power;
/// Blocking driver for the PWM controller.
#[cfg(not(feature = "bcm2712"))]
pub mod pwm;
/// Blocking driver for the hardware random number generator.
#[cfg(not(feature = "bcm2712"))]
pub mod rng;
/// Blocking driver for the on-board SD card slot (Arasan EMMC host
/// controller).
#[cfg(not(feature = "bcm2712"))]
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
#[cfg(not(any(feature = "bcm2711", feature = "bcm2712")))]
pub mod sdhost;
/// Blocking SDIO driver for the on-board BCM43438 wireless chip (Wi-Fi
/// side), via the same Arasan EMMC host controller.
#[cfg(not(feature = "bcm2712"))]
pub mod sdio;
#[cfg(not(feature = "bcm2712"))]
mod soc;
/// Blocking driver for SPI0.
#[cfg(not(feature = "bcm2712"))]
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
#[cfg(not(feature = "bcm2712"))]
pub mod touch;
/// Blocking driver for UART0 (PL011) — on the Pi 5, RP1's UART0, on the
/// same GPIO14/15 header pins.
pub mod uart;
/// Blocking driver for the Unicam CSI-2 camera receiver.
#[cfg(not(feature = "bcm2712"))]
pub mod unicam;
/// USB host support for the built-in DWC2 OTG controller.
#[cfg(not(feature = "bcm2712"))]
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
#[cfg(not(feature = "bcm2712"))]
pub mod watchdog;
/// Host control protocol (SDPCM/CDC) for the on-board BCM43430 Wi-Fi
/// chip, on top of `sdio`.
#[cfg(not(feature = "bcm2712"))]
pub mod wifi;

/// Re-export of the underlying peripheral access crate, for direct
/// register access alongside this crate's higher-level wrappers.
/// Whichever of `bcm2837`/`bcm2711`/`bcm2835` is enabled (see
/// `Cargo.toml`) -- with more than one on, the precedence is
/// `bcm2712` > `bcm2711` > `bcm2837` > `bcm2835`, newest chip first, and
/// the same order every other chip branch in this crate follows. The
/// `bcm2712` arm is the `pac` module above rather than a re-export, since
/// that chip has no generated PAC.
#[cfg(all(feature = "bcm2711", not(feature = "bcm2712")))]
pub use bcm2711_lpa as pac;
#[cfg(all(
    not(feature = "bcm2712"),
    not(feature = "bcm2711"),
    not(feature = "bcm2837"),
    feature = "bcm2835"
))]
pub use bcm2835_lpa as pac;
#[cfg(all(
    not(feature = "bcm2712"),
    not(feature = "bcm2711"),
    feature = "bcm2837"
))]
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
#[cfg(not(any(
    feature = "bcm2837",
    feature = "bcm2711",
    feature = "bcm2835",
    feature = "bcm2712"
)))]
compile_error!(
    "no chip selected: rpi-hal needs exactly one of the `bcm2837` (Pi 2, Pi 3), \
     `bcm2711` (Pi 4), `bcm2835` (Pi 1, Pi Zero) or `bcm2712` (Pi 5) features. \
     None is a default, since there is no sensible default target chip -- add \
     one to this crate's dependency entry, e.g. \
     `rpi-hal = { version = \"0.9\", features = [\"bcm2837\"] }`."
);
// The Pi 5's boot firmware starts 64-bit kernels only, so a 32-bit build
// would produce an image nothing can run -- and `pac`'s addresses, all
// above 4 GiB, would not even fit a pointer.
#[cfg(all(feature = "bcm2712", not(target_arch = "aarch64")))]
compile_error!(
    "the `bcm2712` feature (Pi 5) needs an AArch64 target, e.g. \
     `--target aarch64-unknown-none-softfloat`: the Pi 5 only boots 64-bit \
     kernels."
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
