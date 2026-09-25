//! The global heap.
//!
//! Most bare-metal firmware has no heap at all, and a board reaching for
//! this one usually has a specific reason. `rustls` refuses to build
//! without `alloc` and allocates freely while it parses and validates a
//! certificate chain; `resident-fat` keeps the allocation table and every
//! directory it has read in memory, which is the whole reason it can move
//! a file in one transfer instead of a command per block. Both are
//! deliberate trades of memory for capability, and both are good trades
//! on a board with half a gigabyte of it.
//!
//! ```ignore
//! let bytes = heap::init(&mut mailbox)?;
//! logln!("heap: {} MiB", bytes / (1024 * 1024));
//! ```
//!
//! # Installed by linkage
//!
//! This module declares the `#[global_allocator]`, which is a decision for
//! the whole program rather than for whatever asked: a binary may contain
//! exactly one, and every crate in it allocates through this one.
//!
//! That is why it is behind a feature, off by default, the same as
//! [`entropy`](crate::entropy)'s `getrandom` backend. A board that wants
//! a different allocator — or a different region, or no heap at all —
//! leaves the feature alone and declares its own, and nothing here
//! competes with it. There is no way to make such a choice overridable;
//! the only honest thing a library can do is make taking it explicit.
//!
//! # Why TLSF rather than a linked list
//!
//! `embedded-alloc` offers both. A linked-list first-fit allocator is
//! simpler, but its allocation cost grows with the number of free blocks
//! and it fragments under repeated mixed-size allocate and free — which
//! is exactly the workload here, since every TLS handshake allocates and
//! frees a certificate chain. TLSF is bounded time for both operations
//! and resists fragmentation by design, which is what a board expected to
//! run for months without a power cycle wants: the cost of being wrong
//! about it is a device that gets slower and then stops, long after
//! anyone is watching the console.

use core::sync::atomic::{AtomicBool, Ordering};

use embedded_alloc::TlsfHeap;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::mem;

/// The heap itself.
///
/// `empty()` is a `const` constructor, so this can be a `static` — but it
/// hands out nothing until [`init`] gives it a region, and any allocation
/// before that point panics.
#[global_allocator]
static HEAP: TlsfHeap = TlsfHeap::empty();

/// Whether [`init`] has run, so that a second call reports rather than
/// corrupts.
///
/// `TlsfHeap::init` is `unsafe` precisely because calling it twice hands
/// the allocator a region it has already given parts of away, after which
/// two live allocations can overlap. That is not a failure anything
/// reports — it is silent memory corruption some distance from its cause.
/// Guarding it here is what lets [`init`] be a safe function at all.
static INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Why [`init`] could not hand the allocator a region.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The extent of free memory could not be established — the firmware
    /// would not answer, or it reported less memory than the image
    /// already occupies.
    Region(mem::Error),
    /// [`init`] has already run.
    ///
    /// Returned rather than ignored: a board reaching here has two pieces
    /// of bring-up that each believe they own the heap, and carrying on
    /// would leave whichever runs second believing it had sized one.
    AlreadyInitialized,
}

impl From<mem::Error> for Error {
    fn from(error: mem::Error) -> Self {
        Error::Region(error)
    }
}

/// Gives the allocator every byte between the end of the image and the
/// top of the ARM's share of RAM, returning the size of the region.
///
/// Call before anything allocates — an allocation before this point
/// panics, and on a board with no console yet that is a silent stop.
///
/// The bounds come from `rpi-hal`, which asks the firmware rather than
/// hardcoding them, so the same image sizes its heap correctly whatever
/// `gpu_mem` the board is set to and however much RAM it has.
///
/// # Errors
///
/// [`Error::Region`] if the firmware does not answer or leaves no room,
/// and [`Error::AlreadyInitialized`] on a second call.
pub fn init(mailbox: &mut Mailbox) -> Result<usize, Error> {
    // Claimed before the region is looked up, so that two callers racing
    // cannot both get past here — and released again on the error paths
    // below, since a caller that got no heap has not installed one and a
    // later attempt should be allowed to succeed.
    if INITIALIZED.swap(true, Ordering::AcqRel) {
        return Err(Error::AlreadyInitialized);
    }

    let region = match mem::heap_region(mailbox) {
        Ok(region) => region,
        Err(error) => {
            INITIALIZED.store(false, Ordering::Release);
            return Err(Error::Region(error));
        }
    };
    let size = region.len();

    // SAFETY: reached once, guarded above, and before anything has
    // allocated — nothing can have been handed out of an empty heap. The
    // region is `rpi-hal`'s: above the image and below the peripheral
    // base, identity-mapped as cacheable Normal memory by the `mmu`
    // feature's bring-up, and with this crate's stacks inside the image
    // rather than growing down into it.
    unsafe { HEAP.init(region.start, size) };

    Ok(size)
}
