//! Where the image ends and free RAM begins.
//!
//! The companion to [`crate::stack`], which answers the same kind of
//! question about the other reserved region: this one is for a program
//! that wants a heap and has to decide how big it can be.
//!
//! Nothing here allocates or hands out memory. A HAL must not declare a
//! `#[global_allocator]` — a program may have only one, and that choice
//! belongs to the final binary — so what this provides is the *extent*,
//! and the binary passes it to whichever allocator it registered:
//!
//! ```ignore
//! let region = mem::heap_region(&mut mailbox)?;
//! // SAFETY: called once, before anything allocates.
//! unsafe { HEAP.init(region.start, region.len()) };
//! ```
//!
//! `examples/heap_alloc.rs` is that, whole.
//!
//! # Why the top comes from the firmware
//!
//! How much RAM the ARM has is not a property of the board alone. The
//! VideoCore takes its share first, set by `gpu_mem` in `config.txt`, and
//! what is left is what the firmware reports. Hardcoding a number means
//! an image that is wrong on a board with a different split — and wrong
//! in the direction that hands the allocator memory the GPU is also
//! using, which presents as corruption somewhere else entirely rather
//! than as a failure to allocate.

use core::ops::Range;

use crate::mailbox::{self, Mailbox};

extern "C" {
    /// End of `.bss`, placed by this crate's linker script. Only its
    /// address is meaningful — the byte itself is never read.
    static __bss_end: u8;
}

/// Alignment the heap region starts on.
///
/// The linker script promises `__bss_end` is word-aligned, which is all
/// the boot code's `.bss` zeroing needs. An allocator hands out blocks
/// aligned for `u64`, which is 8 even on AArch32 where a pointer is 4 —
/// so the start is raised to that here, next to the requirement, rather
/// than by asking the script for an alignment only one of its consumers
/// cares about.
const HEAP_ALIGN: usize = 8;

/// Why [`heap_region`] could not answer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The firmware would not say how much memory the ARM has.
    Mailbox(mailbox::Error),
    /// The image already ends at or above the top of the memory the
    /// firmware reported, so there is no region to describe.
    ///
    /// Both addresses are carried because the interesting part is which
    /// of the two is surprising: an `image_end` larger than expected is a
    /// kernel that has grown, while a small `memory_end` is a `gpu_mem`
    /// that has been raised. From a console line saying only "no heap",
    /// neither is visible.
    NoRoom {
        /// First address above the image, as [`image_end`] reports it.
        image_end: usize,
        /// First address above the ARM's share of RAM.
        memory_end: usize,
    },
}

impl From<mailbox::Error> for Error {
    fn from(error: mailbox::Error) -> Self {
        Error::Mailbox(error)
    }
}

/// First address above the loaded image, aligned for a heap to start on.
///
/// Everything this crate's linker script places — `.text`, `.rodata`,
/// `.data`, the stacks and `.bss` — is below this, so it is the lowest
/// address a program can claim without knowing what else it linked.
///
/// Useful on its own for a program placing something other than a heap
/// there. For a heap, [`heap_region`] pairs it with the upper bound.
pub fn image_end() -> usize {
    // Reading the symbol's *address* is the whole point; `&raw const`
    // avoids ever forming a reference to a byte this does not own.
    (&raw const __bss_end as usize).next_multiple_of(HEAP_ALIGN)
}

/// Every byte between the end of the image and the top of the ARM's share
/// of RAM.
///
/// The whole of what is free: this crate's linker script puts the stacks
/// inside the image rather than leaving them to grow down from the load
/// address, so nothing is reserved above [`image_end`] for anything else
/// to find.
///
/// The region is identity-mapped as cacheable Normal memory by the `mmu`
/// feature's bring-up — it is below the peripheral base, which is where
/// that map stops being RAM — so a program with the MMU on can hand the
/// whole of it to an allocator. With the MMU off it is still RAM, and
/// still Strongly Ordered like everything else, which is slow rather than
/// wrong.
///
/// # Errors
///
/// [`Error::Mailbox`] if the firmware does not answer, and
/// [`Error::NoRoom`] if the image ends at or above the top of the
/// reported memory — which is a board whose `gpu_mem` leaves less than
/// the kernel needs, not a condition to carry on past with a zero-length
/// heap.
pub fn heap_region(mailbox: &mut Mailbox) -> Result<Range<usize>, Error> {
    let start = image_end();

    let memory = mailbox.arm_memory()?;
    // `u64` first: on AArch32 both fields are `u32` and a board reporting
    // memory that reaches the top of the address space would wrap the sum
    // to zero, turning "all of it" into `NoRoom`.
    let end = u64::from(memory.base_address) + u64::from(memory.size_bytes);
    let end = usize::try_from(end).unwrap_or(usize::MAX);

    if end <= start {
        return Err(Error::NoRoom {
            image_end: start,
            memory_end: end,
        });
    }

    Ok(start..end)
}
