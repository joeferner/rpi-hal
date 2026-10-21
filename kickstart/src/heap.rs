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
//! [`crate::entropy`]'s `getrandom` backend. A board that wants
//! a different allocator — or a different region, or no heap at all —
//! leaves the feature alone and declares its own, and nothing here
//! competes with it. There is no way to make such a choice overridable;
//! the only honest thing a library can do is make taking it explicit.
//!
//! # Why TLSF rather than a linked list
//!
//! A linked-list first-fit allocator is simpler, but its allocation cost
//! grows with the number of free blocks and it fragments under repeated
//! mixed-size allocate and free — which is exactly the workload here,
//! since every TLS handshake allocates and frees a certificate chain.
//! TLSF is bounded time for both operations and resists fragmentation by
//! design, which is what a board expected to run for months without a
//! power cycle wants: the cost of being wrong about it is a device that
//! gets slower and then stops, long after anyone is watching the console.
//!
//! # All of the RAM, in up to two regions
//!
//! On every board the heap is what [`rpi_hal::mem::heap_region`] gives:
//! from the end of the image to the top of the share of RAM the firmware
//! reports to the ARM. On a Pi 5, with the `bcm2712` feature, that share is
//! the first gigabyte whatever the board's size, and the rest of the
//! board's RAM is a second region above it (`rpi_hal::mem::high_ram_region`,
//! 7 GiB on an 8 GB board) with the firmware's own 4 MiB between the two. So the allocator
//! is `rlsf`'s TLSF driven directly, which takes any number of separate
//! regions, rather than through `embedded-alloc`'s wrapper of the same
//! allocator, which takes one.

use core::alloc::{GlobalAlloc, Layout};
use core::cell::RefCell;
use core::ops::Range;
use core::ptr::{self, NonNull};
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use critical_section::Mutex;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::mem;

/// `rlsf`'s TLSF, sized as `embedded-alloc` sizes it: `usize` bitmaps and a
/// first level as wide as an address, so no block is too large for it.
type Tlsf = rlsf::Tlsf<'static, usize, usize, { usize::BITS as usize }, { usize::BITS as usize }>;

/// The most separate regions the heap takes: the firmware's share of RAM,
/// and on a Pi 5 the rest of the board's above it.
const MAX_REGIONS: usize = 2;

/// The allocator's state: the TLSF structure, and the regions it was given,
/// which [`usage`] walks.
struct Pools {
    tlsf: Tlsf,
    regions: [Option<NonNull<[u8]>>; MAX_REGIONS],
}

// SAFETY: the regions are plain memory the allocator owns outright, and
// every access to them and to the TLSF structure is inside the `Mutex`
// that `Heap` wraps this in.
unsafe impl Send for Pools {}

/// The global allocator: [`Pools`] behind a critical section, so that every
/// core, and an interrupt handler, can allocate.
struct Heap(Mutex<RefCell<Pools>>);

/// The heap itself.
///
/// Built empty at compile time, so this can be a `static` — but it hands
/// out nothing until [`init`] gives it its regions, and any allocation
/// before that point fails, which `alloc` reports by panicking.
#[global_allocator]
static HEAP: Heap = Heap(Mutex::new(RefCell::new(Pools {
    tlsf: Tlsf::new(),
    regions: [None; MAX_REGIONS],
})));

// SAFETY: allocation and deallocation are `rlsf`'s, each in one critical
// section; `rlsf` returns blocks that satisfy the layout, and is given
// back only blocks it returned with the alignment they were allocated at.
unsafe impl GlobalAlloc for Heap {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        critical_section::with(|cs| self.0.borrow_ref_mut(cs).tlsf.allocate(layout))
            .map_or(ptr::null_mut(), NonNull::as_ptr)
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        critical_section::with(|cs| {
            // SAFETY: `ptr` came from `alloc` with this `layout`, per
            // `GlobalAlloc`'s contract, so it is non-null and `rlsf`'s.
            unsafe {
                self.0
                    .borrow_ref_mut(cs)
                    .tlsf
                    .deallocate(NonNull::new_unchecked(ptr), layout.align());
            }
        });
    }
}

/// Whether [`init`] has run, so that a second call reports rather than
/// corrupts.
///
/// Giving the allocator a region it has already given parts of away would
/// let two live allocations overlap. Guarding it here turns that into
/// [`Error::AlreadyInitialized`], which a board can report and carry on
/// from.
static INITIALIZED: AtomicBool = AtomicBool::new(false);

/// The bytes [`init`] gave the allocator, across its regions; zero before
/// then. What [`usage`] reckons `used` against.
static SIZE: AtomicUsize = AtomicUsize::new(0);

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

/// Gives the allocator all of the board's free RAM, returning how many
/// bytes that is: every byte between the end of the image and the top of
/// the ARM's share of RAM, and on a Pi 5 (the `bcm2712` feature) the rest
/// of the board's RAM above that.
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
    // Claimed before the regions are looked up, so that two callers racing
    // cannot both get past here — and released again on the error path
    // below, since a caller that got no heap has not installed one and a
    // later attempt should be allowed to succeed.
    if INITIALIZED.swap(true, Ordering::AcqRel) {
        return Err(Error::AlreadyInitialized);
    }
    let regions = match regions(mailbox) {
        Ok(regions) => regions,
        Err(error) => {
            INITIALIZED.store(false, Ordering::Release);
            return Err(Error::Region(error));
        }
    };

    let mut size = 0;
    critical_section::with(|cs| {
        let mut pools = HEAP.0.borrow_ref_mut(cs);
        let pools = &mut *pools;
        for (slot, region) in pools.regions.iter_mut().zip(regions.into_iter().flatten()) {
            let Some(start) = NonNull::new(region.start as *mut u8) else {
                continue;
            };
            let block = NonNull::slice_from_raw_parts(start, region.len());
            // SAFETY: reached once, guarded above, and before anything
            // has allocated. Each region is `rpi-hal`'s: free RAM, mapped
            // as cacheable Normal memory by the `mmu` feature's bring-up
            // (or, above 4 GiB, by `high_ram_region` itself), owned by
            // nothing else for the rest of the program, and with this
            // crate's stacks inside the image rather than growing down
            // into the first. The two don't overlap.
            unsafe { pools.tlsf.insert_free_block_ptr(block) };
            *slot = Some(block);
            size += region.len();
        }
    });
    SIZE.store(size, Ordering::Relaxed);

    Ok(size)
}

/// The free regions [`init`] gives the allocator: the firmware's share, and
/// on a Pi 5 the RAM above it when the board has any.
fn regions(mailbox: &mut Mailbox) -> Result<[Option<Range<usize>>; MAX_REGIONS], mem::Error> {
    let low = mem::heap_region(mailbox)?;
    #[cfg(feature = "bcm2712")]
    let high = Some(mem::high_ram_region(mailbox)?).filter(|high| !high.is_empty());
    #[cfg(not(feature = "bcm2712"))]
    let high = None;
    Ok([Some(low), high])
}

/// How the heap's bytes are split, as [`usage`] found them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Usage {
    /// Bytes not available to allocate: live allocations, and the
    /// allocator's own bookkeeping between them.
    pub used: usize,
    /// Bytes in free blocks. Not the largest allocation that would
    /// succeed — free space split across blocks is not one block — but the
    /// figure that, falling over weeks, says something is never freed.
    pub free: usize,
}

/// The heap's usage right now; both zero before [`init`].
///
/// Walks every block in the heap inside a critical section — interrupts
/// masked, and on a multicore board the other cores held off it — so the
/// cost grows with how many blocks there are, live and free, rather than
/// with the heap's size. Cheap enough for a metrics scrape every few
/// seconds; not something to call in a loop.
pub fn usage() -> Usage {
    // One walk rather than two, which could disagree if something
    // allocates between them. `used` is the regions less what is free.
    let free = critical_section::with(|cs| {
        let pools = HEAP.0.borrow_ref(cs);
        pools
            .regions
            .iter()
            .flatten()
            .map(|&region| {
                // SAFETY: each region is one `init` inserted, whole.
                unsafe { pools.tlsf.iter_blocks(region) }
                    .filter(|block| !block.is_occupied())
                    .map(|block| block.max_payload_size())
                    .sum::<usize>()
            })
            .sum::<usize>()
    });
    Usage {
        used: SIZE.load(Ordering::Relaxed).saturating_sub(free),
        free,
    }
}
