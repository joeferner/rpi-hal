//! The card: finding the FAT volume on it and mounting it.
//!
//! The filesystem is `resident-fat`, which keeps the allocation table and
//! every directory it has read in RAM. That is a trade of memory for
//! commands — a file moves in one multi-block transfer per contiguous run
//! rather than a command per block — and it is why this module needs a
//! heap. Everything here is generic over its
//! [`BlockDevice`](resident_fat::BlockDevice), so the controller the card
//! hangs off is the board's business: EMMC on one board, SDHOST on the
//! next, the same call.
//!
//! ```ignore
//! let mut volume = storage::mount(SdBlockDevice::new(sd, timer))?;
//! let loaded = config::load::<Settings, _>(&mut volume, "kickstart.toml")?;
//! ```
//!
//! # Timestamps
//!
//! With the `clock` feature, `mount` gives the volume a clock that reads
//! [`crate::clock`], so a file written once something has set the time
//! carries that time. Before then — and without the feature — it carries
//! the FAT epoch, 1980-01-01. That is not a concession: a write must not
//! depend on whether the network has answered, and a file dated 1980 beats
//! one that refuses to be written. It is also how a board reading the card
//! tells a file written by something that knew the time from one that did
//! not.
//!
//! The stamp is UTC. FAT timestamps carry no zone and operating systems
//! read them as local time, so a PC east or west of Greenwich shows these
//! files that many hours off; a board has no zone to convert with until a
//! `tz` one is read off the card, which is after the card is mounted.
//!
//! # One card, many tasks
//!
//! Once the executor runs, the card is wanted from several places at once —
//! a settings save from one web task, an update from another, a board's own
//! state from a third — and it cannot be driven from two of them.
//! [`Shared`](crate::storage::Shared) is where the mounted volume goes for
//! that: a `static` the board declares, holding the volume behind an async
//! mutex, reached with `with`.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use resident_fat::{BlockDevice, FileSystem};

use crate::logln;

/// Why [`mount`] found nothing to mount: `resident-fat`'s own error, which
/// has a case for each way a card goes wrong — including
/// `NoFatPartition`, a table with nothing FAT in it, told apart from a card
/// with no table at all — and prints as one line for the console.
pub type Error<E> = resident_fat::Error<E>;

/// A mounted FAT volume.
///
/// An alias rather than a wrapper: `resident-fat`'s own API is the one to
/// read and write files through, and hiding it would mean re-exporting all
/// of it.
pub type Volume<D> = FileSystem<D>;

/// A mounted volume that every task can reach, one at a time — declared by
/// the board as a `static`.
///
/// ```ignore
/// // SAFETY: one core runs the executor, no interrupt handler touches the
/// // card, and nothing reaches the volume except through `CARD`.
/// static CARD: storage::Shared<SdBlockDevice<'static>> = unsafe { storage::Shared::new() };
///
/// CARD.install(storage::mount(device)?);           // at bring-up
/// CARD.with(|volume| config::save(volume, FILE, &settings)).await  // from any task
/// ```
///
/// A type rather than a `static` of this crate's, because a `static` cannot
/// be generic and the block device is the board's.
pub struct Shared<D: BlockDevice> {
    volume: Mutex<CriticalSectionRawMutex, Option<Volume<D>>>,
}

// SAFETY: a volume holds its block device, and the devices a Pi has are not
// `Send` -- `rpi-hal` marks its peripheral handles against being shared
// across cores, and that marker is doing its job. What makes sharing one
// sound is the arrangement [`Shared::new`]'s caller promises: one core, no
// interrupt handler reaching the card, and every access through the mutex,
// so no two tasks are ever inside the driver at once.
unsafe impl<D: BlockDevice> Sync for Shared<D> {}

impl<D: BlockDevice> Shared<D> {
    /// An empty slot, for a `static`.
    ///
    /// # Safety
    ///
    /// The volume is handed between tasks although its block device is not
    /// `Send`, which is sound only if all of these hold for as long as the
    /// program runs:
    ///
    /// - **One core** runs every task that reaches this. A second core given
    ///   work that touches the card breaks it.
    /// - **No interrupt handler** reaches the card, through this or any
    ///   other way.
    /// - **Nothing reaches the volume except through this** — no second
    ///   handle to the same device kept aside.
    ///
    /// Each of those would break silently, as a corrupted transfer rather
    /// than a fault, which is why the promise is made here, once, where a
    /// reader of the board's code can see it.
    pub const unsafe fn new() -> Self {
        Shared {
            volume: Mutex::new(None),
        }
    }

    /// Hands the volume over. Call once, during bring-up.
    ///
    /// Not `async`, so the board can call it before the executor starts:
    /// nothing can be holding the slot then. If something is — a board
    /// installing from a task while another has the card — the volume comes
    /// back as the `Err` rather than being dropped, along with everything it
    /// holds in RAM.
    // The `Err` is the whole volume, and clippy would have it boxed. Handing
    // the caller's own value back is the point of it, on a path taken at
    // most once, at boot, and only by a board that got this wrong.
    #[allow(clippy::result_large_err)]
    pub fn install(&self, volume: Volume<D>) -> Result<(), Volume<D>> {
        match self.volume.try_lock() {
            Ok(mut slot) => {
                *slot = Some(volume);
                Ok(())
            }
            Err(_) => Err(volume),
        }
    }

    /// Runs `f` with the volume, waiting for whichever task has it.
    ///
    /// `None` means no volume was ever installed, which is distinct from `f`
    /// having failed: every caller has to say what it does about a board
    /// running without a card, and folding the two together would let "no
    /// card" be mistaken for "the write failed".
    ///
    /// `f` is synchronous by construction. A card driver on a Pi is blocking
    /// transfers, so the executor stalls for the length of the call whatever
    /// shape it has, and a closure that could `await` would only add ways to
    /// hold the card across an arbitrary wait.
    pub async fn with<R>(&self, f: impl FnOnce(&mut Volume<D>) -> R) -> Option<R> {
        self.volume.lock().await.as_mut().map(f)
    }
}

/// What the volume stamps on the entries it writes: [`crate::clock`]'s
/// time once something has set it, and the FAT epoch before that — see
/// the module documentation for why the fallback is the right answer.
///
/// Deliberately not uptime, which would stamp every file a few seconds
/// after 1980 with an air of precision it has not earned.
#[cfg(feature = "clock")]
fn fat_now() -> resident_fat::DateTime {
    match crate::clock::now_unix() {
        // Clamped into FAT's range by `from_unix_seconds` itself.
        Some(seconds) => resident_fat::DateTime::from_unix_seconds(seconds as i64),
        None => resident_fat::DateTime::EPOCH,
    }
}

/// Mounts the FAT volume on `device`, and says on the console what it
/// found.
///
/// The volume is the first partition whose type byte says FAT, whichever
/// slot that is — imaging tools usually put the boot partition in slot 0,
/// but not always, and a card with a separate data partition has more than
/// one. A card with no partition table at all is mounted as one bare
/// volume, which is how some tools format a small card.
///
/// Mounting reads the whole allocation table into RAM — four bytes per
/// cluster, for as long as the volume is mounted — so the heap has to be
/// up first. The console line gives the figure, since it is the resource
/// this spends.
pub fn mount<D: BlockDevice>(device: D) -> Result<Volume<D>, Error<D::Error>> {
    // Which of the two layouts the card has, and holding a partition's
    // volume to the partition's length, are `resident-fat`'s -- see
    // `mount_first_fat`. Where the volume starts is what says which it was.
    let volume = FileSystem::mount_first_fat(device)?;
    match volume.first_block() {
        0 => logln!("storage: FAT volume with no partition table"),
        block => logln!("storage: FAT volume at block {block}"),
    }

    #[cfg(feature = "clock")]
    let volume = {
        let mut volume = volume;
        volume.set_clock(alloc::boxed::Box::new(resident_fat::FnClock::new(fat_now)));
        volume
    };

    let clusters = volume.fat().cluster_count();
    logln!(
        "storage: {clusters} clusters of {} KiB, {} KiB of allocation table in RAM{}",
        volume.boot_sector().cluster_bytes() / 1024,
        // In `u64`: the sum is four bytes a cluster, and a large card's
        // count times four is close enough to `u32::MAX` not to trust it.
        (u64::from(clusters) + 2) * 4 / 1024,
        // Worth a line of its own: a volume that was not cleanly unmounted
        // may have an allocation table that disagrees with its directories,
        // and a board that starts writing to it is building on that.
        if volume.is_dirty() {
            " -- NOT CLEANLY UNMOUNTED"
        } else {
            ""
        }
    );
    Ok(volume)
}
