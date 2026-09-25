//! The bring-up an application does before it is an application, for
//! bare-metal Raspberry Pi boards built on `rpi-hal`.
//!
//! `rpi-hal` stops at the peripheral: it hands out a `Uart`, a `Mailbox`,
//! an `Sd`. What every board then does with them is the same program
//! written again — a console the whole image can print to, a heap sized
//! from what the firmware reports, a card mounted and a settings file read
//! off it, a site served out of RAM, a clock that something has to set.
//! This crate is that program, once.
//!
//! # Everything is opt-in, and everything is replaceable
//!
//! Each capability is its own feature and its own module, and none of
//! them are on by default: a board states what it is, and pays for
//! nothing else. Where a piece has a plausible second implementation —
//! the time source above all, which may be the network on one board and a
//! DS3231 on the next — what this crate owns is the *sink* the rest of the
//! image reads through, and filling it is the application's to choose.
//!
//! The same principle draws the line at the executor. An
//! `#[embassy_executor::task]` cannot be generic, so this crate never
//! declares one; it exposes `async fn`s and the application wraps them in
//! tasks with its own concrete types. Routing, the global allocator and
//! the panic handler stay with the application for the same reason — they
//! are single, program-wide choices that a library taking them away
//! cannot give back.
//!
//! # What an application must provide
//!
//! - **`rpi-hal`'s `rt` feature**, which supplies the boot sequence,
//!   the vector table and the `critical-section` implementation.
//! - **`rpi-hal`'s `mmu` feature** (on by default) for anything using
//!   atomics, which includes every `embassy` executor.
//! - **A `#[global_allocator]` and a `#[panic_handler]`.** A program may
//!   have only one of each, so neither can come from a library.

#![no_std]
#![deny(missing_docs)]

/// The console every module logs to — see the module's own documentation
/// for why its sink is a trait object and its clock is passed in.
///
/// The macro that writes to it, [`logln!`], is at the crate root: an
/// exported `macro_rules!` lands there whatever file it is written in.
#[cfg(feature = "console")]
pub mod console;

// The rest of the modules this crate is being assembled from -- clock,
// entropy, heap, storage, config, site, web, metrics, mdns, ota -- arrive
// one at a time, each behind the feature named for it.
