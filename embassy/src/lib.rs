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

/// Thread-mode executor for AArch32 and AArch64 — see the module's own
/// documentation for why this isn't one of `embassy-executor`'s backends.
pub mod executor;
/// `embassy-net` adapter over `rpi-hal`'s LAN9514 USB-Ethernet driver —
/// see the module's own documentation for the interrupt wiring an
/// application must provide.
#[cfg(feature = "embassy-net-driver")]
pub mod lan9514;
/// `embassy-time` driver over the BCM System Timer — see the module's own
/// documentation for the interrupt wiring an application must provide.
pub mod time_driver;

/// Re-exported at the crate root because that is the path
/// `#[embassy_executor::main(executor = "rpi_hal_embassy::Executor")]`
/// expects to find it at.
pub use executor::Executor;
