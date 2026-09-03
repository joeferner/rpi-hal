//! The container: what a bundle is on the wire, and how to check one.
//!
//! # Layout
//!
//! Little-endian throughout.
//!
//! ```text
//! magic   : 4 bytes             per-application; see [`Format`]
//! version : u8      = 2
//! reserved: u8      = 0
//! count   : u16                 number of entries
//! entries : count x {
//!             role    : u8      see [`Role`]
//!             path_len: u8
//!             path    : [u8; path_len]
//!             size    : u32
//!             data    : [u8; size]
//!           }
//! crc32   : u32                 IEEE CRC-32 over every preceding byte
//! ```
//!
//! A bundle is a list of **(path, role, bytes)**. Where each entry lands is
//! the bundle's business, not the receiving firmware's — which is the whole
//! difference from the format this replaces, where a kernel came first and
//! everything after it went into one directory compiled into the device.
//!
//! # Why there is no per-entry checksum
//!
//! It was in the design and did not survive being written down. Its purpose
//! was to let a device skip rewriting a file whose bytes are already on the
//! card — but the entry's data is in memory beside the comparison, so the
//! expected checksum can be computed there with [`checksum`] for nothing.
//! A copy on the wire would be four bytes per entry that no reader needs,
//! and a second place for the same number to be wrong.
//!
//! The whole-bundle checksum stays, because that one covers something no
//! local computation can: that the bytes which arrived are the bytes that
//! were sent.

use crc::{CRC_32_ISO_HDLC, Crc};

/// Bundle format version this crate reads and writes.
///
/// Version 1 existed, carried a kernel plus a flat list of names, and is
/// deliberately not readable here — see the crate documentation.
pub const VERSION: u8 = 2;

/// Magic, version, reserved byte and entry count.
const HEADER_LEN: usize = 8;

/// The trailing whole-bundle CRC-32.
const TRAILER_LEN: usize = 4;

/// A header and a checksum with nothing between them.
const MIN_LEN: usize = HEADER_LEN + TRAILER_LEN;

/// IEEE CRC-32, the same one the host packer computes.
///
/// A `static` rather than a `const` because [`Checksum`] borrows it for the
/// life of the program: a `const` is a fresh temporary at every mention and
/// cannot be borrowed past the expression naming it.
static CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

/// The IEEE CRC-32 of `data`, as it appears in a bundle.
///
/// Exposed so that a device comparing a file on its card against an entry
/// checksums both the same way. Getting that wrong would not corrupt
/// anything — it would silently rewrite every file on every update, which
/// is worse, because nothing would ever report it.
pub fn checksum(data: &[u8]) -> u32 {
    CRC32.checksum(data)
}

/// The same checksum, over data that arrives in pieces.
///
/// [`checksum`] is what a caller holding the whole thing wants. This is for
/// the other side of the comparison — a file already on a card, read a chunk
/// at a time precisely so that checking it needs no second copy of it in
/// memory.
pub struct Checksum(crc::Digest<'static, u32>);

impl Checksum {
    /// Starts one.
    pub fn new() -> Checksum {
        Checksum(CRC32.digest())
    }

    /// Adds the next piece.
    pub fn update(&mut self, data: &[u8]) {
        self.0.update(data);
    }

    /// The checksum of everything added, comparable with [`checksum`] of the
    /// same bytes.
    pub fn finish(self) -> u32 {
        self.0.finalize()
    }
}

impl Default for Checksum {
    fn default() -> Checksum {
        Checksum::new()
    }
}

impl core::fmt::Debug for Checksum {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // `crc::Digest` is not `Debug`, and what it holds mid-stream is not
        // something a reader could act on anyway.
        f.write_str("Checksum(..)")
    }
}

/// What an application must agree with its packer about.
///
/// Everything else a bundle needs to describe is in the bundle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Format {
    /// Four bytes identifying which application a bundle was built for, so
    /// that one board rejects another's update.
    ///
    /// It is compiled into the firmware and written by the packer, and it
    /// is the only compatibility check there is: nothing else in a bundle
    /// says which board it belongs to. A project shipping more than one
    /// architecture should therefore vary it — `WAT7` against `WAT8` —
    /// since an image for the wrong architecture is otherwise a bundle that
    /// installs perfectly and does not boot.
    pub magic: [u8; 4],
    /// Most entries to accept.
    ///
    /// A bound on what a single upload can ask the device to write, checked
    /// before any entry is walked.
    pub max_entries: usize,
}

/// How the device should treat an entry.
///
/// The byte values are wire values and say nothing about the order entries
/// are written in — that order is the installer's, and it is not this one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    /// Anything the application owns: assets, settings, data.
    File,
    /// The boot image, to be routed to whichever kernel slot is not
    /// running. At most one per bundle.
    Kernel,
    /// A Raspberry Pi firmware file — `bootcode.bin`, `start*.elf`,
    /// `fixup*.dat`.
    Firmware,
    /// `config.txt`. At most one per bundle, because the installer edits
    /// the file this entry lands as.
    Config,
}

impl Role {
    /// The wire byte for this role.
    pub fn as_byte(self) -> u8 {
        match self {
            Role::File => 0,
            Role::Kernel => 1,
            Role::Firmware => 2,
            Role::Config => 3,
        }
    }

    /// The role a wire byte names, or `None` if it names none.
    pub fn from_byte(byte: u8) -> Option<Role> {
        match byte {
            0 => Some(Role::File),
            1 => Some(Role::Kernel),
            2 => Some(Role::Firmware),
            3 => Some(Role::Config),
            _ => None,
        }
    }
}

/// One file in a bundle, borrowed from the buffer it arrived in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Entry<'a> {
    /// How the device should treat it.
    pub role: Role,
    /// Where it goes, relative to the root of the volume. Always a valid
    /// relative path when it came from [`Bundle::parse`].
    pub path: &'a str,
    /// The file's contents.
    pub data: &'a [u8],
}

/// Why a bundle was rejected, reading one or building one.
///
/// Most variants can arise either way, because most of what makes a bundle
/// wrong is a property of its entries rather than of its bytes — and one
/// vocabulary for both directions is what keeps a packer from cheerfully
/// building something no device will take. The exceptions are marked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// Fewer bytes than a header and a checksum. Reading only.
    TooShort,
    /// The magic did not match the one this application expects. Reading
    /// only.
    BadMagic,
    /// A format version this build does not read. Carries the version seen.
    /// Reading only.
    BadVersion(u8),
    /// The trailing CRC-32 did not match the content. Reading only.
    BadChecksum,
    /// No entries at all, or more than [`Format::max_entries`].
    BadEntryCount(usize),
    /// An entry ran past the end of the bundle, or the entries did not fill
    /// it exactly. Reading only.
    Truncated,
    /// An entry named a role this build does not know. Carries the byte.
    /// Reading only.
    BadRole(u8),
    /// An entry's path was not valid UTF-8. Reading only — a path being
    /// built is already a `&str`.
    PathNotUtf8,
    /// A path longer than the 255 bytes its length field can hold. Building
    /// only.
    PathTooLong,
    /// A file larger than the 4 GiB its size field can hold. Building only.
    DataTooLarge,
    /// An entry's path was empty, absolute, or contained `.`, `..`, an
    /// empty component or a backslash.
    BadPath,
    /// Two entries wanted the same path.
    DuplicatePath,
    /// More than one entry claimed a role that allows only one.
    DuplicateRole(Role),
    /// A `start*.elf` arrived without its `fixup*.dat`, or the reverse.
    UnpairedFirmware,
}

impl Error {
    /// The HTTP status to answer this with, for a device that took the
    /// bundle over HTTP.
    ///
    /// Always 400, and that is the content of the method rather than a
    /// placeholder: every rejection in this module is a statement about the
    /// bytes that arrived, so none of them is ever the receiver's fault. An
    /// installer whose errors are not all client errors wraps this one and
    /// answers for the rest itself.
    pub fn http_status(&self) -> u16 {
        400
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let text = match self {
            Error::TooShort => "bundle too short",
            Error::BadMagic => "not a bundle for this device",
            Error::BadVersion(_) => "unsupported bundle version",
            Error::BadChecksum => "checksum mismatch",
            Error::BadEntryCount(_) => "bad entry count",
            Error::Truncated => "bundle truncated",
            Error::BadRole(_) => "unknown entry role",
            Error::PathNotUtf8 => "bad file path",
            Error::PathTooLong => "file path too long",
            Error::DataTooLarge => "file too large",
            Error::BadPath => "unsafe file path",
            Error::DuplicatePath => "two entries with the same path",
            Error::DuplicateRole(_) => "two entries with the same role",
            Error::UnpairedFirmware => "firmware file without its pair",
        };
        f.write_str(text)
    }
}

impl core::error::Error for Error {}

/// A validated bundle: a handle onto the buffer it was parsed from.
///
/// Holding one is the proof that every check in this module passed, which
/// is why walking it with [`iter`](Bundle::iter) cannot fail.
#[derive(Clone, Copy, Debug)]
pub struct Bundle<'a> {
    /// Exactly the entry region — header and trailing checksum removed.
    entries: &'a [u8],
    count: usize,
}

impl<'a> Bundle<'a> {
    /// Checks `bytes` against `format` and returns a handle to it.
    ///
    /// Allocates nothing: the result borrows the buffer.
    ///
    /// # Order of checks
    ///
    /// The checksum is verified **before** any entry is walked. A transfer
    /// that lost bytes produces lengths that still look plausible, and
    /// reporting the first one that happens not to fit describes a symptom
    /// rather than what went wrong.
    pub fn parse(format: &Format, bytes: &'a [u8]) -> Result<Bundle<'a>, Error> {
        if bytes.len() < MIN_LEN {
            return Err(Error::TooShort);
        }
        if bytes[..4] != format.magic {
            return Err(Error::BadMagic);
        }
        if bytes[4] != VERSION {
            return Err(Error::BadVersion(bytes[4]));
        }

        let split = bytes.len() - TRAILER_LEN;
        let stored = u32::from_le_bytes([
            bytes[split],
            bytes[split + 1],
            bytes[split + 2],
            bytes[split + 3],
        ]);
        if checksum(&bytes[..split]) != stored {
            return Err(Error::BadChecksum);
        }

        let count = u16::from_le_bytes([bytes[6], bytes[7]]) as usize;
        if count == 0 || count > format.max_entries {
            return Err(Error::BadEntryCount(count));
        }

        // Walked here, and again by every `iter()` afterwards. Two passes
        // over a buffer already in memory, in exchange for an iterator that
        // needs no error type and a handle that cannot name a bundle it has
        // not checked.
        let entries = &bytes[HEADER_LEN..split];
        let mut rest = entries;
        for _ in 0..count {
            let (_, next) = split_entry(rest)?;
            rest = next;
        }
        // The count and the bytes have to agree exactly. Anything left over
        // is a bundle whose header does not describe it.
        if !rest.is_empty() {
            return Err(Error::Truncated);
        }

        // Structure first, then meaning: everything above is about whether
        // there is a bundle here at all, and `check_entries` is about
        // whether the entries in it make sense. A torn transfer trips the
        // first kind, and saying so is more use than reporting whichever
        // rule the wreckage happened to break.
        let bundle = Bundle { entries, count };
        check_entries(bundle.iter())?;
        Ok(bundle)
    }

    /// How many entries it holds.
    pub fn count(&self) -> usize {
        self.count
    }

    /// The entries, in the order they were packed.
    ///
    /// Not the order they should be written in — see [`Role`].
    pub fn iter(&self) -> Iter<'a> {
        Iter {
            rest: self.entries,
            left: self.count,
        }
    }

    /// The entry carrying the boot image, if the bundle has one.
    ///
    /// A bundle without a kernel is valid and useful: it updates a website
    /// or a settings file without rewriting an image that has not changed.
    pub fn kernel(&self) -> Option<Entry<'a>> {
        self.iter().find(|entry| entry.role == Role::Kernel)
    }
}

/// Every rule about a set of entries that does not depend on which
/// direction they came from.
///
/// Both halves of this module run it: [`Bundle::parse`] after it has proved
/// there is a bundle there at all, and [`encode`] before it writes one. That
/// is the point — a packer that could build what a device rejects, or the
/// reverse, is the drift this crate exists to remove.
///
/// Takes a cloneable iterator rather than a slice so that a decoded bundle
/// can be checked without collecting it into one, which is what keeps
/// reading allocation-free.
fn check_entries<'a, I>(entries: I) -> Result<(), Error>
where
    I: Iterator<Item = Entry<'a>> + Clone,
{
    for entry in entries.clone() {
        check_path(entry.path)?;
    }
    check_unique_paths(entries.clone())?;
    check_single(entries.clone(), Role::Kernel)?;
    check_single(entries.clone(), Role::Config)?;
    check_firmware_pairs(entries)
}

/// Rejects two entries wanting the same destination.
///
/// The second would simply overwrite the first, so the file the packer
/// meant to ship is not the one that lands — a failure with no symptom at
/// the time and a confusing one later.
fn check_unique_paths<'a, I>(entries: I) -> Result<(), Error>
where
    I: Iterator<Item = Entry<'a>> + Clone,
{
    for (index, entry) in entries.clone().enumerate() {
        if entries
            .clone()
            .skip(index + 1)
            .any(|other| other.path == entry.path)
        {
            return Err(Error::DuplicatePath);
        }
    }
    Ok(())
}

/// Rejects a second entry claiming a role that admits only one.
fn check_single<'a, I>(entries: I, role: Role) -> Result<(), Error>
where
    I: Iterator<Item = Entry<'a>>,
{
    if entries.filter(|entry| entry.role == role).count() > 1 {
        return Err(Error::DuplicateRole(role));
    }
    Ok(())
}

/// Rejects a `start*.elf` without its `fixup*.dat`, or the reverse.
///
/// The two are released together and a mismatched pair does not boot, so
/// shipping one of them is a packing mistake whose cost is a board that
/// needs its card pulled. Checking it is a few lines here and impossible
/// anywhere later.
///
/// Only the pairing is checked, not the names: an entry marked
/// [`Role::Firmware`] may be called anything. A list of the files Raspberry
/// Pi currently ships would reject the one they add next, and this crate
/// has no way to be right about that list over time.
fn check_firmware_pairs<'a, I>(entries: I) -> Result<(), Error>
where
    I: Iterator<Item = Entry<'a>> + Clone,
{
    for entry in entries.clone().filter(|entry| entry.role == Role::Firmware) {
        let name = basename(entry.path);
        let paired = if let Some(suffix) = strip_around(name, "start", ".elf") {
            has_firmware(entries.clone(), "fixup", suffix, ".dat")
        } else if let Some(suffix) = strip_around(name, "fixup", ".dat") {
            has_firmware(entries.clone(), "start", suffix, ".elf")
        } else {
            // `bootcode.bin`, or anything else with no partner.
            true
        };
        if !paired {
            return Err(Error::UnpairedFirmware);
        }
    }
    Ok(())
}

/// Whether a firmware entry named `<prefix><middle><extension>` is present,
/// comparing the way FAT does.
fn has_firmware<'a, I>(entries: I, prefix: &str, middle: &str, extension: &str) -> bool
where
    I: Iterator<Item = Entry<'a>>,
{
    entries
        .filter(|entry| entry.role == Role::Firmware)
        .any(|entry| {
            strip_around(basename(entry.path), prefix, extension)
                .is_some_and(|found| found.eq_ignore_ascii_case(middle))
        })
}

/// Builds a bundle from `entries`.
///
/// The entries are written in the order given. That order carries no
/// meaning — an installer decides what to write when from
/// [`Role`], not from where an entry sits — so a packer is free to emit
/// them in whatever order is convenient to produce.
///
/// # Errors
///
/// Every rule [`Bundle::parse`] would apply to the result is applied here
/// first, so this cannot produce a bundle its own parser rejects, plus the
/// two limits of the container itself: [`Error::PathTooLong`] and
/// [`Error::DataTooLarge`].
#[cfg(feature = "alloc")]
pub fn encode(format: &Format, entries: &[Entry<'_>]) -> Result<alloc::vec::Vec<u8>, Error> {
    // A count that does not fit the header's `u16` is caught here rather
    // than by the cast below, which would otherwise write a plausible small
    // number and produce a bundle describing a fraction of itself.
    if entries.is_empty() || entries.len() > format.max_entries || entries.len() > u16::MAX as usize
    {
        return Err(Error::BadEntryCount(entries.len()));
    }
    check_entries(entries.iter().copied())?;

    let mut size = HEADER_LEN + TRAILER_LEN;
    for entry in entries {
        if entry.path.len() > u8::MAX as usize {
            return Err(Error::PathTooLong);
        }
        // Unreachable where `usize` is 32 bits, and unreachable in practice
        // anywhere else — a 4 GiB entry is not a thing anyone packs. Here
        // because the alternative is a silently truncated length field, and
        // a check that never fires costs one comparison per entry.
        if entry.data.len() as u64 > u32::MAX as u64 {
            return Err(Error::DataTooLarge);
        }
        size += 1 + 1 + entry.path.len() + 4 + entry.data.len();
    }

    // Sized up front. A bundle carrying a kernel and the Raspberry Pi
    // firmware is several megabytes, and growing a buffer to that from
    // nothing copies it a dozen times for no reason.
    let mut out = alloc::vec::Vec::with_capacity(size);
    out.extend_from_slice(&format.magic);
    out.push(VERSION);
    out.push(0);
    out.extend_from_slice(&(entries.len() as u16).to_le_bytes());
    for entry in entries {
        out.push(entry.role.as_byte());
        out.push(entry.path.len() as u8);
        out.extend_from_slice(entry.path.as_bytes());
        out.extend_from_slice(&(entry.data.len() as u32).to_le_bytes());
        out.extend_from_slice(entry.data);
    }
    out.extend_from_slice(&checksum(&out).to_le_bytes());
    Ok(out)
}

/// Walks the entries of a [`Bundle`].
#[derive(Clone, Debug)]
pub struct Iter<'a> {
    rest: &'a [u8],
    left: usize,
}

impl<'a> Iterator for Iter<'a> {
    type Item = Entry<'a>;

    fn next(&mut self) -> Option<Entry<'a>> {
        if self.left == 0 {
            return None;
        }
        // `Bundle::parse` walked this exact region and rejected everything
        // `split_entry` can fail on, so the failing arm is unreachable for
        // any `Bundle` that exists. Ending the walk is the conservative
        // response to a bug reaching it: a caller sees fewer entries than
        // `count()` promised, rather than an entry that is not there.
        let (entry, rest) = split_entry(self.rest).ok()?;
        self.rest = rest;
        self.left -= 1;
        Some(entry)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        // No lower bound, for the same reason `next` can stop early.
        (0, Some(self.left))
    }
}

/// Splits one entry off the front of `rest`, returning it and what follows.
fn split_entry(rest: &[u8]) -> Result<(Entry<'_>, &[u8]), Error> {
    let (&role, rest) = rest.split_first().ok_or(Error::Truncated)?;
    let role = Role::from_byte(role).ok_or(Error::BadRole(role))?;

    let (&path_len, rest) = rest.split_first().ok_or(Error::Truncated)?;
    let (path, rest) = rest
        .split_at_checked(path_len as usize)
        .ok_or(Error::Truncated)?;
    let path = core::str::from_utf8(path).map_err(|_| Error::PathNotUtf8)?;

    let (size, rest) = rest.split_at_checked(4).ok_or(Error::Truncated)?;
    let size = u32::from_le_bytes([size[0], size[1], size[2], size[3]]) as usize;
    let (data, rest) = rest.split_at_checked(size).ok_or(Error::Truncated)?;

    Ok((Entry { role, path, data }, rest))
}

/// Rejects a path that is not a plain relative one.
///
/// This is about a bundle escaping the volume it is being written to, and
/// is unrelated to whether writing a particular file is a good idea — a
/// bundle is allowed to replace `config.txt` and the boot firmware, because
/// being unable to is what made a card reader necessary.
fn check_path(path: &str) -> Result<(), Error> {
    if path.is_empty() {
        return Err(Error::BadPath);
    }
    // Redundant with the empty-component rule below, which a leading
    // separator also trips. Kept because "absolute paths are rejected"
    // should be readable here rather than derived.
    if path.starts_with('/') {
        return Err(Error::BadPath);
    }
    // A backslash is a separator to some tools and an ordinary character to
    // a FAT writer, so a path carrying one lands as a single strangely
    // named file instead of the two levels whoever packed it meant.
    if path.contains('\\') {
        return Err(Error::BadPath);
    }
    for component in path.split('/') {
        if component.is_empty() || component == "." || component == ".." {
            return Err(Error::BadPath);
        }
    }
    Ok(())
}

/// The last component of `path`.
fn basename(path: &str) -> &str {
    path.rsplit_once('/').map_or(path, |(_, name)| name)
}

/// The middle of `<prefix><middle><suffix>`, matching the ends the way FAT
/// compares names.
fn strip_around<'a>(name: &'a str, prefix: &str, suffix: &str) -> Option<&'a str> {
    if name.len() < prefix.len() + suffix.len() {
        return None;
    }
    let (head, rest) = name.split_at_checked(prefix.len())?;
    if !head.eq_ignore_ascii_case(prefix) {
        return None;
    }
    let (middle, tail) = rest.split_at_checked(rest.len() - suffix.len())?;
    if !tail.eq_ignore_ascii_case(suffix) {
        return None;
    }
    Some(middle)
}

#[cfg(test)]
mod tests {
    use super::*;

    const FORMAT: Format = Format {
        magic: *b"TEST",
        max_entries: 8,
    };

    /// Builds bundles, including malformed ones.
    ///
    /// Deliberately not [`encode`], which cannot produce most of what the
    /// decoder has to reject — and which, being the other half of the code
    /// under test, would agree with it about a format neither had got
    /// right. The golden vectors below are the check on that.
    struct Builder {
        magic: [u8; 4],
        version: u8,
        count: Option<u16>,
        entries: Vec<(u8, Vec<u8>, Vec<u8>)>,
        trailing: Vec<u8>,
        corrupt: bool,
    }

    impl Builder {
        fn new() -> Self {
            Builder {
                magic: *b"TEST",
                version: VERSION,
                count: None,
                entries: Vec::new(),
                trailing: Vec::new(),
                corrupt: false,
            }
        }

        fn entry(mut self, role: u8, path: &str, data: &[u8]) -> Self {
            self.entries
                .push((role, path.as_bytes().to_vec(), data.to_vec()));
            self
        }

        fn raw_path_entry(mut self, role: u8, path: &[u8], data: &[u8]) -> Self {
            self.entries.push((role, path.to_vec(), data.to_vec()));
            self
        }

        fn build(self) -> Vec<u8> {
            let mut out = Vec::new();
            out.extend_from_slice(&self.magic);
            out.push(self.version);
            out.push(0);
            let count = self.count.unwrap_or(self.entries.len() as u16);
            out.extend_from_slice(&count.to_le_bytes());
            for (role, path, data) in &self.entries {
                out.push(*role);
                out.push(path.len() as u8);
                out.extend_from_slice(path);
                out.extend_from_slice(&(data.len() as u32).to_le_bytes());
                out.extend_from_slice(data);
            }
            out.extend_from_slice(&self.trailing);
            let crc = checksum(&out) ^ if self.corrupt { 1 } else { 0 };
            out.extend_from_slice(&crc.to_le_bytes());
            out
        }
    }

    fn one_file() -> Vec<u8> {
        Builder::new().entry(0, "WWW/INDEX.HTM", b"hello").build()
    }

    fn parse(bytes: &[u8]) -> Result<Bundle<'_>, Error> {
        Bundle::parse(&FORMAT, bytes)
    }

    fn error(bytes: &[u8]) -> Error {
        parse(bytes).expect_err("expected this bundle to be rejected")
    }

    #[test]
    fn checksum_matches_the_standard_vector() {
        // The IEEE CRC-32 check value, so a change of polynomial or of
        // reflection is caught here rather than by a board that will not
        // boot.
        assert_eq!(checksum(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn a_well_formed_bundle_parses() {
        let bytes = Builder::new()
            .entry(1, "KERNEL7.IMG", b"kernel bytes")
            .entry(0, "WWW/INDEX.HTM", b"<html>")
            .entry(3, "CONFIG.TXT", b"arm_64bit=0\n")
            .build();
        let bundle = parse(&bytes).unwrap();

        assert_eq!(bundle.count(), 3);
        let entries: Vec<_> = bundle.iter().collect();
        assert_eq!(entries.len(), 3);
        assert_eq!(entries[0].role, Role::Kernel);
        assert_eq!(entries[0].path, "KERNEL7.IMG");
        assert_eq!(entries[0].data, b"kernel bytes");
        assert_eq!(entries[1].role, Role::File);
        assert_eq!(entries[1].path, "WWW/INDEX.HTM");
        assert_eq!(entries[2].role, Role::Config);
        assert_eq!(bundle.kernel().unwrap().data, b"kernel bytes");
    }

    #[test]
    fn a_bundle_with_no_kernel_is_valid() {
        let bytes = Builder::new().entry(0, "WATER.CFG", b"zone=1").build();
        let bundle = parse(&bytes).unwrap();
        assert_eq!(bundle.count(), 1);
        assert!(bundle.kernel().is_none());
    }

    #[test]
    fn an_empty_file_is_valid() {
        let bytes = Builder::new().entry(0, "EMPTY.TXT", b"").build();
        assert_eq!(parse(&bytes).unwrap().iter().next().unwrap().data, b"");
    }

    #[test]
    fn too_short() {
        assert_eq!(error(&[]), Error::TooShort);
        assert_eq!(error(&one_file()[..MIN_LEN - 1]), Error::TooShort);
    }

    #[test]
    fn bad_magic() {
        let mut bytes = one_file();
        bytes[0] = b'X';
        assert_eq!(error(&bytes), Error::BadMagic);
    }

    #[test]
    fn bad_version() {
        // The version this replaces, which is the one a stale bundle left
        // in a `target/` directory would carry.
        let mut builder = Builder::new().entry(0, "A.TXT", b"a");
        builder.version = 1;
        assert_eq!(error(&builder.build()), Error::BadVersion(1));
    }

    #[test]
    fn bad_checksum() {
        let mut bytes = one_file();
        let last = bytes.len() - TRAILER_LEN - 1;
        bytes[last] ^= 0xFF;
        assert_eq!(error(&bytes), Error::BadChecksum);
    }

    #[test]
    fn the_checksum_is_checked_before_the_entries() {
        // A count of zero and a broken checksum together: the checksum is
        // what a torn transfer actually looks like, and reporting the count
        // would send someone to look at the packer.
        let mut builder = Builder::new().entry(0, "A.TXT", b"a");
        builder.count = Some(0);
        builder.corrupt = true;
        assert_eq!(error(&builder.build()), Error::BadChecksum);
    }

    #[test]
    fn bad_entry_count() {
        let mut builder = Builder::new().entry(0, "A.TXT", b"a");
        builder.count = Some(0);
        assert_eq!(error(&builder.build()), Error::BadEntryCount(0));

        let mut builder = Builder::new();
        for index in 0..9 {
            builder = builder.entry(0, &format!("F{index}.TXT"), b"x");
        }
        assert_eq!(error(&builder.build()), Error::BadEntryCount(9));
    }

    #[test]
    fn an_entry_running_past_the_end_is_truncated() {
        let mut builder = Builder::new().entry(0, "A.TXT", b"abc");
        builder.count = Some(2);
        assert_eq!(error(&builder.build()), Error::Truncated);
    }

    #[test]
    fn slack_between_the_last_entry_and_the_checksum_is_truncated() {
        let mut builder = Builder::new().entry(0, "A.TXT", b"abc");
        builder.trailing = vec![0; 3];
        assert_eq!(error(&builder.build()), Error::Truncated);
    }

    #[test]
    fn bad_role() {
        let bytes = Builder::new().entry(4, "A.TXT", b"a").build();
        assert_eq!(error(&bytes), Error::BadRole(4));
    }

    #[test]
    fn path_not_utf8() {
        let bytes = Builder::new()
            .raw_path_entry(0, &[0xFF, 0xFE], b"a")
            .build();
        assert_eq!(error(&bytes), Error::PathNotUtf8);
    }

    #[test]
    fn unsafe_paths() {
        for path in [
            "",
            "/ABS.TXT",
            "../ESCAPE.TXT",
            "WWW/../ESCAPE.TXT",
            "./HERE.TXT",
            "WWW//DOUBLE.TXT",
            "TRAILING/",
            "WWW\\INDEX.HTM",
        ] {
            let bytes = Builder::new().entry(0, path, b"x").build();
            assert_eq!(error(&bytes), Error::BadPath, "path {path:?} was accepted");
        }
    }

    #[test]
    fn nested_paths_are_fine() {
        let bytes = Builder::new().entry(0, "CERTS/ROOTS/CA.PEM", b"x").build();
        assert_eq!(parse(&bytes).unwrap().count(), 1);
    }

    #[test]
    fn duplicate_path() {
        let bytes = Builder::new()
            .entry(0, "WWW/INDEX.HTM", b"one")
            .entry(0, "WWW/INDEX.HTM", b"two")
            .build();
        assert_eq!(error(&bytes), Error::DuplicatePath);
    }

    #[test]
    fn two_kernels() {
        let bytes = Builder::new()
            .entry(1, "KERNEL7.IMG", b"a")
            .entry(1, "KERNEL8.IMG", b"b")
            .build();
        assert_eq!(error(&bytes), Error::DuplicateRole(Role::Kernel));
    }

    #[test]
    fn two_configs() {
        let bytes = Builder::new()
            .entry(3, "CONFIG.TXT", b"a")
            .entry(3, "BOOT/CONFIG.TXT", b"b")
            .build();
        assert_eq!(error(&bytes), Error::DuplicateRole(Role::Config));
    }

    #[test]
    fn a_start_without_its_fixup_is_unpaired() {
        let bytes = Builder::new().entry(2, "START.ELF", b"a").build();
        assert_eq!(error(&bytes), Error::UnpairedFirmware);
    }

    #[test]
    fn a_fixup_without_its_start_is_unpaired() {
        let bytes = Builder::new().entry(2, "FIXUP.DAT", b"a").build();
        assert_eq!(error(&bytes), Error::UnpairedFirmware);
    }

    #[test]
    fn a_pair_with_mismatched_suffixes_is_unpaired() {
        let bytes = Builder::new()
            .entry(2, "START4.ELF", b"a")
            .entry(2, "FIXUP.DAT", b"b")
            .build();
        assert_eq!(error(&bytes), Error::UnpairedFirmware);
    }

    #[test]
    fn a_matched_pair_is_accepted_whatever_the_case_and_suffix() {
        for (start, fixup) in [
            ("START.ELF", "FIXUP.DAT"),
            ("start.elf", "fixup.dat"),
            ("Start4.Elf", "fixup4.DAT"),
            ("BOOT/start_x.elf", "FIXUP_X.DAT"),
        ] {
            let bytes = Builder::new()
                .entry(2, start, b"a")
                .entry(2, fixup, b"b")
                .build();
            assert!(parse(&bytes).is_ok(), "{start} + {fixup} was rejected");
        }
    }

    #[test]
    fn firmware_with_no_partner_needs_none() {
        let bytes = Builder::new().entry(2, "BOOTCODE.BIN", b"a").build();
        assert_eq!(parse(&bytes).unwrap().count(), 1);
    }

    #[test]
    fn every_rejection_is_the_senders_fault() {
        assert_eq!(Error::TooShort.http_status(), 400);
        assert_eq!(Error::BadChecksum.http_status(), 400);
        assert_eq!(Error::UnpairedFirmware.http_status(), 400);
    }

    #[test]
    fn roles_round_trip_through_their_wire_bytes() {
        for role in [Role::File, Role::Kernel, Role::Firmware, Role::Config] {
            assert_eq!(Role::from_byte(role.as_byte()), Some(role));
        }
        assert_eq!(Role::from_byte(4), None);
    }

    // --- Golden vectors -------------------------------------------------
    //
    // Laid out by hand from the format at the top of this module, byte by
    // byte, and never regenerated from `encode`. That is the whole value of
    // them: a round trip only proves the encoder and the decoder agree, and
    // they would agree just as happily about a container that had drifted.
    // These fail instead.
    //
    // The checksums were computed by Python's `zlib.crc32` — a third
    // implementation, so the constant in this crate is pinned to something
    // outside it.

    /// One file, nothing else. The smallest bundle that is not an error.
    const GOLDEN_ONE_FILE: &[u8] = &[
        b'T', b'E', b'S', b'T', // magic
        2,    // version
        0,    // reserved
        1, 0, // count = 1
        0, // [0] role = file
        5, // path_len
        b'A', b'.', b'T', b'X', b'T', // path
        3, 0, 0, 0, // size = 3
        b'a', b'b', b'c', // data
        0xE9, 0xB1, 0x6D, 0xB9, // crc32 of everything above
    ];

    /// A kernel, a nested asset and a `config.txt` — one entry of every
    /// role that an ordinary update carries.
    const GOLDEN_FULL: &[u8] = &[
        b'T', b'E', b'S', b'T', // magic
        2,    // version
        0,    // reserved
        3, 0,  // count = 3
        1,  // [0] role = kernel
        11, // path_len
        b'K', b'E', b'R', b'N', b'E', b'L', b'7', b'.', b'I', b'M', b'G', 4, 0, 0,
        0, // size = 4
        b'k', b'r', b'n', b'l', 0,  // [1] role = file
        13, // path_len
        b'W', b'W', b'W', b'/', b'I', b'N', b'D', b'E', b'X', b'.', b'H', b'T', b'M', 6, 0, 0,
        0, // size = 6
        b'<', b'h', b't', b'm', b'l', b'>', 3,  // [2] role = config
        10, // path_len
        b'C', b'O', b'N', b'F', b'I', b'G', b'.', b'T', b'X', b'T', 12, 0, 0, 0, // size = 12
        b'a', b'r', b'm', b'_', b'6', b'4', b'b', b'i', b't', b'=', b'0', b'\n', 0xB0, 0xBD, 0xAA,
        0xA1, // crc32
    ];

    /// A firmware pair and no kernel — an update that replaces the
    /// Raspberry Pi firmware and leaves the boot image alone.
    const GOLDEN_FIRMWARE: &[u8] = &[
        b'T', b'E', b'S', b'T', // magic
        2,    // version
        0,    // reserved
        2, 0, // count = 2
        2, // [0] role = firmware
        9, // path_len
        b'S', b'T', b'A', b'R', b'T', b'.', b'E', b'L', b'F', 2, 0, 0, 0, // size = 2
        b's', b'e', 2, // [1] role = firmware
        9, // path_len
        b'F', b'I', b'X', b'U', b'P', b'.', b'D', b'A', b'T', 2, 0, 0, 0, // size = 2
        b'f', b'd', 0x4F, 0xA4, 0x7B, 0xD1, // crc32
    ];

    /// Both directions against bytes neither direction produced.
    fn check_golden(bytes: &[u8], entries: &[Entry<'_>]) {
        let bundle = parse(bytes).expect("the golden vector should parse");
        assert_eq!(bundle.iter().collect::<Vec<_>>(), entries);
        assert_eq!(encode(&FORMAT, entries).unwrap(), bytes);
    }

    #[test]
    fn golden_one_file() {
        check_golden(
            GOLDEN_ONE_FILE,
            &[Entry {
                role: Role::File,
                path: "A.TXT",
                data: b"abc",
            }],
        );
    }

    #[test]
    fn golden_full() {
        check_golden(
            GOLDEN_FULL,
            &[
                Entry {
                    role: Role::Kernel,
                    path: "KERNEL7.IMG",
                    data: b"krnl",
                },
                Entry {
                    role: Role::File,
                    path: "WWW/INDEX.HTM",
                    data: b"<html>",
                },
                Entry {
                    role: Role::Config,
                    path: "CONFIG.TXT",
                    data: b"arm_64bit=0\n",
                },
            ],
        );
    }

    #[test]
    fn golden_firmware() {
        check_golden(
            GOLDEN_FIRMWARE,
            &[
                Entry {
                    role: Role::Firmware,
                    path: "START.ELF",
                    data: b"se",
                },
                Entry {
                    role: Role::Firmware,
                    path: "FIXUP.DAT",
                    data: b"fd",
                },
            ],
        );
    }

    // --- Encoding -------------------------------------------------------

    #[test]
    fn what_is_encoded_parses_back() {
        let big = vec![0xA5; 100_000];
        let entries = [
            Entry {
                role: Role::Kernel,
                path: "KERNEL8.IMG",
                data: &big,
            },
            Entry {
                role: Role::File,
                path: "CERTS/ROOTS/CA.PEM",
                data: b"-----BEGIN",
            },
            Entry {
                role: Role::File,
                path: "EMPTY.BIN",
                data: b"",
            },
        ];
        let bytes = encode(&FORMAT, &entries).unwrap();
        let bundle = parse(&bytes).unwrap();
        assert_eq!(bundle.iter().collect::<Vec<_>>(), entries);
        assert_eq!(bundle.kernel().unwrap().data.len(), 100_000);
    }

    #[test]
    fn encoding_reserves_exactly_what_it_writes() {
        // Not a style point: the buffer is megabytes in practice, and a
        // capacity that is merely close still reallocates and copies.
        let entries = [Entry {
            role: Role::File,
            path: "A.TXT",
            data: b"abc",
        }];
        let bytes = encode(&FORMAT, &entries).unwrap();
        assert_eq!(bytes.len(), bytes.capacity());
    }

    #[test]
    fn another_applications_bundle_is_rejected() {
        let entries = [Entry {
            role: Role::File,
            path: "A.TXT",
            data: b"abc",
        }];
        let bytes = encode(
            &Format {
                magic: *b"OTHR",
                max_entries: 8,
            },
            &entries,
        )
        .unwrap();
        assert_eq!(error(&bytes), Error::BadMagic);
    }

    #[test]
    fn encoding_applies_every_rule_the_parser_would() {
        let file = |path| Entry {
            role: Role::File,
            path,
            data: b"x" as &[u8],
        };

        assert_eq!(
            encode(&FORMAT, &[]).unwrap_err(),
            Error::BadEntryCount(0),
            "an empty bundle"
        );

        let many: Vec<_> = ["A", "B", "C", "D", "E", "F", "G", "H", "I"]
            .iter()
            .map(|path| file(path))
            .collect();
        assert_eq!(
            encode(&FORMAT, &many).unwrap_err(),
            Error::BadEntryCount(9),
            "more entries than the format allows"
        );

        assert_eq!(
            encode(&FORMAT, &[file("../ESCAPE.TXT")]).unwrap_err(),
            Error::BadPath
        );
        assert_eq!(
            encode(&FORMAT, &[file("A.TXT"), file("A.TXT")]).unwrap_err(),
            Error::DuplicatePath
        );
        assert_eq!(
            encode(
                &FORMAT,
                &[
                    Entry {
                        role: Role::Kernel,
                        path: "KERNEL7.IMG",
                        data: b"a"
                    },
                    Entry {
                        role: Role::Kernel,
                        path: "KERNEL8.IMG",
                        data: b"b"
                    },
                ]
            )
            .unwrap_err(),
            Error::DuplicateRole(Role::Kernel)
        );
        assert_eq!(
            encode(
                &FORMAT,
                &[Entry {
                    role: Role::Firmware,
                    path: "START.ELF",
                    data: b"a"
                }]
            )
            .unwrap_err(),
            Error::UnpairedFirmware
        );
    }

    #[test]
    fn a_path_longer_than_its_length_field() {
        let long = "A".repeat(256);
        assert_eq!(
            encode(
                &FORMAT,
                &[Entry {
                    role: Role::File,
                    path: &long,
                    data: b"x"
                }]
            )
            .unwrap_err(),
            Error::PathTooLong
        );

        // 255 is the largest a `u8` length field can describe, so it has to
        // work rather than being one past a limit nobody tested.
        let limit = "A".repeat(255);
        assert!(
            encode(
                &FORMAT,
                &[Entry {
                    role: Role::File,
                    path: &limit,
                    data: b"x"
                }]
            )
            .is_ok()
        );
    }
}
