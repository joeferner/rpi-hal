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

use core::fmt;

use resident_fat::mbr::{MAX_PARTITIONS, PartitionTable};
use resident_fat::{BlockDevice, FileSystem};

use crate::logln;

/// Why [`mount`] found nothing to mount.
#[derive(Debug)]
pub enum Error<E> {
    /// The card has a partition table and none of its entries is FAT.
    ///
    /// Its own case rather than `resident-fat`'s `NoPartitionTable`,
    /// which would be the wrong diagnosis: there *is* a table, and the fix
    /// is a different one — the card was imaged with something other than
    /// a Pi's layout, or its FAT partition was retyped.
    NoFatPartition,
    /// The filesystem could not be mounted, or the device failed under it.
    Fat(resident_fat::Error<E>),
}

impl<E> From<resident_fat::Error<E>> for Error<E> {
    fn from(error: resident_fat::Error<E>) -> Self {
        Error::Fat(error)
    }
}

/// One line, for the console.
impl<E: fmt::Debug> fmt::Display for Error<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::NoFatPartition => f.write_str("no FAT partition on the card"),
            Error::Fat(error) => write!(f, "{error}"),
        }
    }
}

/// A mounted FAT volume.
///
/// An alias rather than a wrapper: `resident-fat`'s own API is the one to
/// read and write files through, and hiding it would mean re-exporting all
/// of it.
pub type Volume<D> = FileSystem<D>;

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
pub fn mount<D: BlockDevice>(mut device: D) -> Result<Volume<D>, Error<D::Error>> {
    let volume = match PartitionTable::read(&mut device)? {
        Some(table) => {
            // By slot rather than through `first_fat`, because the slot is
            // what `mount_partition` takes -- and it, unlike mounting at
            // the partition's first block, holds the volume to the
            // partition's length.
            let Some(slot) = (0..MAX_PARTITIONS)
                .find(|&slot| table.get(slot).is_some_and(|partition| partition.is_fat()))
            else {
                return Err(Error::NoFatPartition);
            };
            let volume = FileSystem::mount_partition(device, slot)?;
            logln!("storage: FAT volume in partition {slot}");
            volume
        }
        None => {
            let volume = FileSystem::mount(device)?;
            logln!("storage: FAT volume with no partition table");
            volume
        }
    };

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
