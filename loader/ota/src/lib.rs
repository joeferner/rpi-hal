//! The over-the-air update bundle a bare-metal Raspberry Pi installs on
//! itself: one container, packed on a host and validated on the device.
//!
//! A bundle carries a kernel image and the files that ship beside it —
//! a website, assets, whatever an application serves — under a single
//! checksum, so a board can reject a damaged transfer before it writes
//! anything to its card.
//!
//! The reason this is a crate rather than a file each project keeps a copy
//! of is that **the encoder and the decoder are the same code**. A wire
//! format described in two places is a format that has already drifted:
//! the two implementations this one was collected from agreed on every
//! byte and disagreed on what they enforced, each rejecting bundles the
//! other accepted.
//!
//! # How it is split
//!
//! | | Feature | Pulls in |
//! | --- | --- | --- |
//! | Reading and checking a bundle | *none* | nothing |
//! | Building one | `alloc` *(default)* | nothing |
//! | Installing one onto a FAT volume | `apply` | `resident-fat` |
//!
//! The split is not tidiness. `rpi-loader`, the host CLI that packs
//! bundles, never installs one — without the feature it would depend on a
//! filesystem implementation in order to write a file on a host. A device
//! that only installs bundles is the mirror image, compiling an encoder it
//! will never call.
//!
//! Reading needs no allocator at all: [`Bundle::parse`] validates in place
//! and borrows, so a device holds one buffer — the bundle as it arrived —
//! and nothing else.
//!
//! # Versions
//!
//! The container is at version 2 and version 1 is not readable here. The
//! two are different enough that supporting both would be a compatibility
//! path used twice and then deleted, which is worth less than the card
//! reader it would save.
//!
//! That awkwardness is inherent rather than accidental: **a bundle is
//! parsed by the firmware already running and installs the firmware that
//! replaces it**, so a board can never be sent a container its current
//! build does not understand. Changing the format means reaching the
//! board some other way once.
//!
//! # What an application still supplies
//!
//! **The transport and the reboot.** How a bundle arrives — an HTTP route,
//! a job handed to another core, a serial command — is application shaped,
//! and a crate that took it would be choosing the web framework.
//!
//! **The measurement.** What an update costs the card is the number these
//! projects care about most, and collecting it here would mean depending on
//! an async runtime for a clock and on whatever wrapper is counting device
//! commands. The install reports its progress and the caller times it.

// `std` under `cfg(test)` only, because the test harness needs it. The
// bare-metal clippy pass in `make clippy-ota` builds without `--all-targets`
// and so compiles this crate as the device sees it: `no_std`.
#![cfg_attr(not(test), no_std)]
#![deny(missing_docs)]

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod bundle;

pub use bundle::{Bundle, Entry, Error, Format, Role, checksum};

#[cfg(feature = "alloc")]
pub use bundle::encode;
