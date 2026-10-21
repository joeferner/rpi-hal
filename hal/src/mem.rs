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
    /// The board's revision code doesn't say how much RAM it has: an
    /// old-style code (bit 23 clear, the Pi 1's), or a memory-size field
    /// this crate has no size for. Carries the code.
    UnknownRam(u32),
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
/// On the BCM2712 (Pi 5) there is no peripheral base below 4 GiB; the map
/// is RAM up to there, and the firmware's answer is the first gigabyte
/// whatever the board's size (`0x3fc0_0000` bytes on an 8 GB board), so
/// the region is inside it the same way.
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

/// The board's RAM, in bytes, all of it: the VideoCore's share and, on a
/// Pi 5, what lies above the first gigabyte included.
///
/// From the revision code ([`Mailbox::board_revision`]), whose bits 20-22
/// give the size as 256 MiB shifted left by their value — 1 GB is 2, 8 GB
/// is 5, 16 GB is 6 — in every new-style code (bit 23 set), which is every
/// board since the Pi 2.
///
/// # Errors
///
/// [`Error::Mailbox`] if the firmware does not answer, and
/// [`Error::UnknownRam`] for a code that doesn't say.
pub fn ram_bytes(mailbox: &mut Mailbox) -> Result<u64, Error> {
    const NEW_STYLE: u32 = 1 << 23;
    let revision = mailbox.board_revision()?;
    let size = (revision >> 20) & 0b111;
    // 7 is unassigned; a board with it is newer than this code.
    if revision & NEW_STYLE == 0 || size == 7 {
        return Err(Error::UnknownRam(revision));
    }
    Ok((256 << 20) << size)
}

/// Where the Pi 5's RAM resumes above the firmware's: the first gigabyte
/// is what [`Mailbox::arm_memory`] reports (`0x3fc0_0000` bytes, the last
/// 4 MiB below 1 GiB being the firmware's own), and the rest starts here.
#[cfg(feature = "bcm2712")]
const HIGH_RAM_BASE: usize = 0x4000_0000;

/// The Pi 5's RAM above the first gigabyte: from 1 GiB to the end of the
/// board's RAM ([`ram_bytes`]), mapped as cacheable Normal memory and
/// ready for a program to use — a second heap region, or one large
/// buffer. Empty on a 1 GB board.
///
/// [`heap_region`] stops at the first gigabyte because that is all the
/// firmware reports to the ARM, whatever the board's size. The boot-time
/// map already makes RAM of everything up to 4 GiB; above that, on 8 GB
/// and 16 GB boards, this maps the rest first
/// ([`crate::mmu::map_ram_above_4g`]), when the `mmu` feature is on. Calling
/// it again maps the same entries again, which is harmless.
///
/// # Errors
///
/// As [`ram_bytes`].
#[cfg(feature = "bcm2712")]
pub fn high_ram_region(mailbox: &mut Mailbox) -> Result<Range<usize>, Error> {
    let end = usize::try_from(ram_bytes(mailbox)?).unwrap_or(usize::MAX);
    if end <= HIGH_RAM_BASE {
        return Ok(HIGH_RAM_BASE..HIGH_RAM_BASE);
    }
    #[cfg(feature = "mmu")]
    // SAFETY: `end` is the board's own RAM size, so everything from 4 GiB
    // to it is RAM; and every size a revision code gives is a whole number
    // of GiB, well below the peripheral windows, so the call can't fail.
    unsafe { crate::mmu::map_ram_above_4g(end as u64) }
        .expect("RAM sizes from a revision code are whole GiB below the device windows");
    Ok(HIGH_RAM_BASE..end)
}
