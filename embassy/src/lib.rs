//! [Embassy](https://embassy.dev) support for Raspberry Pi boards using
//! the BCM2836/BCM2837 SoC (Pi 2, Pi 3), on top of the `rpi-hal` crate.
//!
//! Provides the two platform pieces Embassy needs and cannot supply
//! itself: an `embassy-time` driver over the BCM System Timer, and a
//! thread-mode executor for AArch32 and AArch64.
//!
//! This is a separate crate from `rpi-hal` rather than a feature of it
//! because an `embassy-time` driver is installed by *linkage*: the
//! `time_driver_impl!` macro defines `#[unsafe(no_mangle)]` symbols that
//! `embassy-time` resolves against, and a program links only if exactly
//! one driver exists in its crate graph. Behind a feature on the HAL, any
//! dependency enabling that feature would force this driver onto the whole
//! program and conflict with an application supplying its own — an opt-in
//! that cannot be opted out of.
//!
//! # What an application must provide
//!
//! - **A `critical-section` implementation**, which `rpi-hal`'s `rt`
//!   feature provides. The timer queue is guarded by one.
//! - **`rpi-hal`'s `mmu` feature** (on by default). The executor's run
//!   queue is built on atomic compare-exchange, and `ldrex`/`strex` are
//!   UNPREDICTABLE until RAM is mapped as cacheable Normal memory.
//! - **Interrupt dispatch.** `rpi-hal` leaves `__irq_handler` to the
//!   application, so this crate cannot claim it; the application routes
//!   the System Timer's interrupt here from its own handler.

#![no_std]
#![deny(missing_docs)]

/// `embassy-net` adapter over `rpi-hal`'s USB-Ethernet drivers, whichever
/// chip the board has — see the module's own documentation for the
/// interrupt wiring an application must provide.
#[cfg(feature = "embassy-net-driver")]
pub mod ethernet;
/// Thread-mode executor for AArch32 and AArch64 — see the module's own
/// documentation for why this isn't one of `embassy-executor`'s backends.
pub mod executor;
/// Getting interrupts to the code waiting on them — see [`irq::dispatch`],
/// and the `irq-dispatch` feature for the case where an application has no
/// sources of its own.
pub mod irq;
/// The LAN9514 spelling of [`ethernet`], kept so existing boards compile
/// unchanged.
#[cfg(feature = "embassy-net-driver")]
pub mod lan9514;
/// `embassy-time` driver over the BCM System Timer — see the module's own
/// documentation for the interrupt wiring an application must provide.
pub mod time_driver;
/// `embassy-net` adapter over `rpi-hal`'s Wi-Fi driver — see the module's
/// own documentation for why its runner polls where `lan9514`'s does not.
#[cfg(feature = "wifi")]
pub mod wifi;

/// Re-exported at the crate root because that is the path
/// `#[embassy_executor::main(executor = "rpi_hal_embassy::Executor")]`
/// expects to find it at.
pub use executor::Executor;

/// `embassy-net-driver-channel`, re-exported.
///
/// The queue pair both network adapters hand the stack is this crate's
/// `Device`, and [`ethernet::attach`]/[`wifi::attach`] take its `Runner`.
/// A board that owns its own queue pair — one that picks between Ethernet
/// and Wi-Fi at startup, say — needs `channel::State` and `channel::new`
/// to build it.
///
/// Re-exported rather than left for the board to depend on directly, so
/// there is no way to end up with two incompatible versions of a type that
/// crosses this crate's API. The version is whatever this crate was built
/// against.
#[cfg(any(feature = "embassy-net-driver", feature = "wifi"))]
pub use embassy_net_driver_channel as channel;
