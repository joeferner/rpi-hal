//! Installing a validated bundle onto a FAT volume.
//!
//! This is the half that knows what a boot partition is. It takes the
//! entries [`Bundle::parse`] handed back and writes them where they say,
//! in an order chosen so that a failure part-way through leaves a board
//! that still boots.
//!
//! # What the caller still supplies
//!
//! The transport and the reboot: how a bundle arrived is application
//! shaped, and a crate that took it would be choosing the web framework.
//! And the measurement — [`Progress`] reports what happened and the caller
//! decides what to time, which is what keeps an async runtime's clock and
//! whatever counts device commands out of this crate.
//!
//! # Why it writes the way it does
//!
//! Two rules below look like implementation detail and are not: they are
//! the difference between an update that takes seconds and one that takes
//! minutes. The figures come from a Pi 2 writing a 1.6 MB kernel to an SD
//! card, and they are here rather than in an application because the rules
//! they justify are here.
//!
//! **The card charges per command, not per block.** A single-block write
//! cost 17.5 ms on that card; a 128-block write about 26 ms — one and a
//! half times the cost for a hundred and twenty-eight times the data, so
//! 62× cheaper per block. Nothing about the card changed between those two
//! numbers. It was being asked the wrong way.
//!
//! So: **one `write_file` per entry, with the length known up front.** The
//! chain is then allocated in one go and the file comes out contiguous,
//! which is what lets the layer underneath issue one long transfer instead
//! of thousands of short ones. A file grown a write at a time gets whatever
//! clusters happen to be spare at each step, and no amount of care further
//! down recovers from that. Through a filesystem that wrote single blocks,
//! the same kernel took 99,683 ms and 5,703 card commands; written this
//! way, 928 ms and 36. That is the whole of the 107×.
//!
//! And: **the read-back buffer is 64 KiB**, which is 128 blocks a command —
//! the same argument from the reading side.
//!
//! **Skipping costs a read, and a read is much cheaper than a write.**
//! Reading an entry back to decide whether to write it sounds like added
//! work and is how a bundle carrying 3 MB of Raspberry Pi firmware — which
//! changes about once a year — stays affordable to ship every time. A
//! 6-entry, 4.8 MB bundle whose contents the card already held cost 1055 ms
//! and **no write commands at all**, against 3724 ms to write the same
//! thing in full.
//!
//! **What is left is a floor of roughly 250 ms per entry, whatever its
//! size**, nearly all of it filesystem bookkeeping rather than data: a
//! rewrite frees the old chain and allocates a new one, each of which
//! flushes the allocation table, and FAT32 keeps two copies of it. So the
//! cost of an update scales with the *number* of entries far more than with
//! their bytes. Six files is nothing; several hundred small assets would be
//! the thing to think about.

use resident_fat::{BlockDevice, FileSystem};

use crate::bundle::{self, Bundle, Checksum, Entry, Format, Role};

/// Bytes read at a time when checking what is already on the card.
///
/// Not a stack buffer: this is allocated once per [`apply`] call and reused,
/// because the size is the whole point. A card charges per command far more
/// than per block, so reading a 3 MB file 512 bytes at a time would cost
/// thousands of commands and defeat the comparison it exists to make cheap.
/// 64 KiB is 128 blocks a call — enough that the per-command cost stops
/// mattering, small enough to be nothing beside the bundle already in
/// memory.
const SCRATCH: usize = 64 * 1024;

/// What installing a bundle touched.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Report {
    /// Size of the kernel installed, when the bundle carried one.
    ///
    /// `None` is not a failure: a bundle with no kernel is how a website or
    /// a settings file is updated without rewriting an image that has not
    /// changed.
    pub kernel_len: Option<u32>,
    /// Entries written.
    pub written: u32,
    /// Entries the card already held, and which were therefore not written.
    pub skipped: u32,
}

/// Told what the installer is doing, so that timing stays with the caller.
///
/// Every method does nothing by default, so an implementation names only
/// what it cares about, and `()` implements the whole thing for a caller
/// that cares about none of it.
///
/// The boundaries are where they are so that a caller can time the write
/// and the read-back separately. Those are not the same operation and do
/// not have the same fix — a write is erase-block program cycles and a read
/// is not — so a single figure covering both would hide which of them an
/// improvement had touched.
pub trait Progress {
    /// About to consider `entry`; nothing has been read or written yet.
    fn starting(&mut self, entry: Entry<'_>) {
        let _ = entry;
    }

    /// `entry` has been written and is about to be read back.
    fn wrote(&mut self, entry: Entry<'_>) {
        let _ = entry;
    }

    /// `entry` has been read back and matched.
    fn verified(&mut self, entry: Entry<'_>) {
        let _ = entry;
    }

    /// The card already held `entry`, so nothing was written. It has still
    /// been read in full — that is how this was established.
    fn skipped(&mut self, entry: Entry<'_>) {
        let _ = entry;
    }
}

impl Progress for () {}

/// Why an install was refused or failed.
///
/// Generic over the device error, unlike [`bundle::Error`], which is why the
/// two are separate types: a host that only ever builds bundles would
/// otherwise have to name a block-device error it does not have.
#[derive(Debug)]
pub enum Error<E> {
    /// The bundle was rejected before anything was written.
    Bundle(bundle::Error),
    /// The card or the filesystem failed.
    Storage(resident_fat::Error<E>),
    /// A file read back after writing did not match what was written.
    VerifyFailed,
}

impl<E> Error<E> {
    /// The HTTP status to answer this with, for a device that took the
    /// bundle over HTTP.
    ///
    /// A rejected bundle is the sender's fault and a card that failed
    /// mid-write is not, and answering the same to both sends whoever is
    /// updating to look at the wrong thing.
    pub fn http_status(&self) -> u16 {
        match self {
            Error::Bundle(error) => error.http_status(),
            Error::Storage(_) | Error::VerifyFailed => 500,
        }
    }
}

impl<E> From<bundle::Error> for Error<E> {
    fn from(error: bundle::Error) -> Error<E> {
        Error::Bundle(error)
    }
}

impl<E> From<resident_fat::Error<E>> for Error<E> {
    fn from(error: resident_fat::Error<E>) -> Error<E> {
        Error::Storage(error)
    }
}

impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Bundle(error) => write!(f, "{error}"),
            Error::Storage(error) => write!(f, "{error}"),
            Error::VerifyFailed => f.write_str("write verification failed"),
        }
    }
}

impl<E: core::fmt::Debug> core::error::Error for Error<E> {}

/// Installs `bundle` onto `volume`.
///
/// Does **not** reboot. The caller answers whoever sent the bundle first,
/// because a failure that nobody is told about is worse than a slow one.
///
/// # Order
///
/// Entries are written in role order rather than the order they were
/// packed, and the crate imposes it rather than trusting the bundle — the
/// same reasoning that made the kernel a role instead of a position:
///
/// 1. [`Role::File`] — assets, settings, blobs. Nothing here is loaded by
///    the firmware, so a failure leaves a board that still boots what it
///    booted last time.
/// 2. [`Role::Firmware`] — `start*.elf`, `fixup*.dat`, `bootcode.bin`.
/// 3. [`Role::Config`] — `config.txt`, which the firmware does read, but
///    which cannot on its own stop a board booting: lose it and the
///    firmware falls back to loading `kernel7.img`/`kernel8.img`.
/// 4. [`Role::Kernel`] — last, because with one boot image this write **is**
///    the commit: the board boots whatever it leaves behind.
///
/// Steps 3 and 4 swap once there are two kernel slots to write to. The
/// kernel then goes to whichever is not running — which is no longer the
/// live image, and no longer the commit — and rewriting `config.txt`'s
/// `kernel=` line takes both jobs over.
///
/// # `bootcode.bin` has no safety net
///
/// The ROM loads it by name, so there is no slot to write it to and no
/// `config.txt` line to point elsewhere: a failed write there means the
/// card comes out. Three things keep that acceptable rather than merely
/// unavoidable — it is about 50 KB, it changes almost never, and on the
/// Pi 4 it lives in SPI EEPROM and is not on the card at all. Every other
/// boot file has a way out.
pub fn apply<D, P>(
    volume: &mut FileSystem<D>,
    format: &Format,
    bundle: &[u8],
    progress: &mut P,
) -> Result<Report, Error<D::Error>>
where
    D: BlockDevice,
    P: Progress,
{
    // Every check that is a statement about the bundle happens here, before
    // a byte reaches the card.
    let parsed = Bundle::parse(format, bundle)?;
    let installed = install(volume, &parsed, progress);

    // Unconditional, and outside `install` so a failure syncs too. The
    // allocation table lives in memory, so nothing written above has
    // necessarily reached the card in full; what a failed install did manage
    // to write is on the card either way, and leaving the table behind is
    // the one thing that turns a half-finished update into a corrupt volume.
    let synced = volume.sync();
    installed.and_then(|report| synced.map(|()| report).map_err(Error::from))
}

/// The install itself, so [`apply`] can sync after it either way.
fn install<D, P>(
    volume: &mut FileSystem<D>,
    bundle: &Bundle<'_>,
    progress: &mut P,
) -> Result<Report, Error<D::Error>>
where
    D: BlockDevice,
    P: Progress,
{
    let mut scratch = alloc::vec![0u8; SCRATCH];
    let mut report = Report::default();

    for role in [Role::File, Role::Firmware, Role::Config] {
        for entry in bundle.iter().filter(|entry| entry.role == role) {
            record(
                &mut report,
                install_entry(volume, entry, progress, &mut scratch)?,
            );
        }
    }

    if let Some(entry) = bundle.kernel() {
        report.kernel_len = Some(entry.data.len() as u32);
        record(
            &mut report,
            install_entry(volume, entry, progress, &mut scratch)?,
        );
    }

    Ok(report)
}

/// Counts one entry as written or skipped.
fn record(report: &mut Report, written: bool) {
    if written {
        report.written += 1;
    } else {
        report.skipped += 1;
    }
}

/// Puts one entry where its path says. Returns whether it had to be written.
fn install_entry<D, P>(
    volume: &mut FileSystem<D>,
    entry: Entry<'_>,
    progress: &mut P,
    scratch: &mut [u8],
) -> Result<bool, Error<D::Error>>
where
    D: BlockDevice,
    P: Progress,
{
    progress.starting(entry);

    // Asked before writing, because the answer is often yes and a read costs
    // a fraction of a write. A bundle that carries the Raspberry Pi firmware
    // carries about 3 MB of it, and that changes roughly once a year.
    if on_card(volume, entry.path, entry.data, scratch)? {
        progress.skipped(entry);
        return Ok(false);
    }

    create_parents(volume, entry.path)?;
    // One call, and that is the point: the length is known before a byte is
    // written, so the whole cluster chain is allocated at once and the file
    // comes out contiguous. A file grown a write at a time gets whatever the
    // allocator had spare each time, which is how a transfer ends up as one
    // device command per fragment.
    volume.write_file(entry.path, entry.data)?;
    progress.wrote(entry);

    // The read-back is the point of the whole exercise: a write the card
    // accepted but did not durably store produces a board that will not
    // boot, discovered on the reboot that was supposed to complete the
    // update. No `sync` first -- what this crate keeps in memory is the
    // allocation table and the directories, and file data goes to the card
    // inside `write_file`, so a read comes off the card either way.
    if !on_card(volume, entry.path, entry.data, scratch)? {
        return Err(Error::VerifyFailed);
    }
    progress.verified(entry);
    Ok(true)
}

/// Whether the card already holds exactly `data` at `path`.
///
/// Used for both halves of the same question: before a write it decides
/// whether to write at all, and after one it is the verification. Sharing
/// the code is not only tidiness — it means a skipped entry has been
/// checked exactly as strictly as a written one.
///
/// The comparison is a checksum rather than the byte-for-byte compare an
/// in-memory copy would allow. That is a real if small weakening, and it
/// buys the ability to check a file of any size against a fixed scratch
/// buffer: the alternative is holding a 3 MB read-back beside the 3 MB
/// already in the bundle.
fn on_card<D>(
    volume: &mut FileSystem<D>,
    path: &str,
    data: &[u8],
    scratch: &mut [u8],
) -> Result<bool, Error<D::Error>>
where
    D: BlockDevice,
{
    let file = match volume.open(path) {
        Ok(file) => file,
        // Absent is an answer, not a failure. Any other error is the card
        // and must not be reported as "different".
        Err(resident_fat::Error::NotFound { .. }) => return Ok(false),
        Err(error) => return Err(Error::Storage(error)),
    };

    // Length first, because it is free and settles most of the cases a
    // checksum would have to read a whole file to settle.
    if file.len() as usize != data.len() {
        return Ok(false);
    }

    let mut checksum = Checksum::new();
    let mut at = 0u64;
    while at < file.len() as u64 {
        let read = volume.read_at(&file, at, scratch)?;
        if read == 0 {
            // Short of the length the directory entry claims. Whatever that
            // is, it is not the file being asked about.
            return Ok(false);
        }
        checksum.update(&scratch[..read]);
        at += read as u64;
    }

    Ok(checksum.finish() == bundle::checksum(data))
}

/// Creates every directory above `path` that is not already there.
///
/// A path in a bundle can be nested, and `write_file` resolves a parent
/// rather than making one, so a first update onto a card that has never held
/// the directory would otherwise fail at the first asset. One level at a
/// time, because that is what `create_dir` does.
fn create_parents<D>(volume: &mut FileSystem<D>, path: &str) -> Result<(), Error<D::Error>>
where
    D: BlockDevice,
{
    let mut at = 0;
    while let Some(offset) = path[at..].find('/') {
        let end = at + offset;
        let directory = &path[..end];
        if volume.open_dir(directory).is_err() {
            volume.create_dir(directory)?;
        }
        at = end + 1;
    }
    Ok(())
}
