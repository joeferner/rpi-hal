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

#[cfg(any(
    feature = "config",
    feature = "metrics",
    feature = "site",
    feature = "storage",
    feature = "tls",
    feature = "web",
    test
))]
extern crate alloc;

/// The wall clock — see the module's own documentation for why it is a
/// sink with no source of its own, and why it reads `None` until set.
#[cfg(feature = "clock")]
pub mod clock;
/// Reading a board's settings file — see the module's own documentation
/// for why the semantic checks run after parsing rather than inside it.
#[cfg(feature = "config")]
pub mod config;
/// The console every module logs to — see the module's own documentation
/// for why its sink is a trait object and its clock is passed in.
///
/// The macro that writes to it, [`logln!`], is at the crate root: an
/// exported `macro_rules!` lands there whatever file it is written in.
#[cfg(feature = "console")]
pub mod console;
/// Hardware entropy, and the `getrandom` backend the crypto stack needs —
/// see the module's own documentation for what enabling it decides for
/// the whole program.
#[cfg(feature = "entropy")]
pub mod entropy;
/// The global heap — see the module's own documentation for why it
/// declares the `#[global_allocator]` and what that decides for the rest
/// of the program.
#[cfg(feature = "heap")]
pub mod heap;
/// A minimal HTTPS client — see the module's own documentation for why it
/// is one request per connection, and why the body comes back whatever the
/// status.
#[cfg(feature = "https")]
pub mod https;
/// An mDNS responder for one name — see the module's own documentation
/// for the half of the multicast plumbing the board still has to do.
#[cfg(feature = "mdns")]
pub mod mdns;
/// Writing Prometheus's text exposition format — see the module's own
/// documentation for decimals without a float formatter, and why a label
/// value is only ever written escaped.
#[cfg(feature = "metrics")]
pub mod metrics;
/// Getting a board onto the network, whichever way it can — see the
/// module's own documentation for why "is Ethernet available" is two
/// questions and both are deadlines.
#[cfg(feature = "net")]
pub mod net;
/// Over-the-air updates: the upload route and the reboot after it — see
/// the module's own documentation for the one response shape every board
/// answers with.
#[cfg(feature = "ota")]
pub mod ota;
/// The board's web assets, read off the card into RAM — see the module's
/// own documentation for why the site lives on the card at all.
#[cfg(feature = "site")]
pub mod site;
/// Setting the clock from an NTP server — see the module's own
/// documentation for what its reply checks do and do not protect against.
#[cfg(feature = "sntp")]
pub mod sntp;
/// Mounting the card's FAT volume — see the module's own documentation
/// for what it costs in RAM and what time the files it writes carry.
#[cfg(feature = "storage")]
pub mod storage;
/// TLS client connections with real certificate verification — see the
/// module's own documentation for the three things a Pi has to supply,
/// one of them an unaudited crypto provider.
#[cfg(feature = "tls")]
pub mod tls;
/// Serving HTTP with `picoserve` — see the module's own documentation for
/// why this is pieces of a router rather than a server task.
#[cfg(feature = "web")]
pub mod web;

// The rest of the modules this crate is being assembled from -- notify,
// splash -- arrive one at a time, each behind the feature named for it.
