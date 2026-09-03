//! `apply` against a real FAT32 volume, with `fsck.vfat` as the oracle.
//!
//! A unit test could only check this module against itself. What matters
//! here is whether the volume afterwards is one somebody else's filesystem
//! implementation agrees is well formed — a bundle that installs and leaves
//! a card `fsck` complains about is a bundle that installed and broke a
//! board. So the image is made by `mkfs.vfat`, written by this crate, and
//! judged by `fsck.vfat`.
//!
//! Requires `dosfstools`. The tests fail rather than skip when it is
//! missing: a suite that quietly checks nothing is worse than one that
//! cannot run.

use std::path::PathBuf;
use std::process::Command;

use resident_fat::{BlockDevice, FileSystem};
use rpi_loader_ota::apply::{Progress, Report, apply};
use rpi_loader_ota::{Entry, Format, Role, encode};

/// Thirty-four megabytes, which is the smallest that yields a *correct*
/// FAT32 volume rather than merely one `mkfs.vfat` will produce.
///
/// FAT32 requires at least 65,525 clusters; below that, `fsck.vfat` warns
/// that the filesystem "may lead to problems on some systems" and the image
/// is not really the thing being tested. At 512 bytes per cluster this is
/// the first size clear of that, and 68,528 clusters comes out silent.
/// `resident-fat` itself would accept a smaller one — it enforces only a
/// maximum — which is exactly why the test must not.
const IMAGE_BYTES: u64 = 34 * 1024 * 1024;

const FORMAT: Format = Format {
    magic: *b"TEST",
    max_entries: 16,
};

/// A volume in memory, so a test can hand the same bytes to this crate and
/// then to `fsck`.
struct Ram {
    blocks: Vec<u8>,
    /// Every device call, which is how the skip test proves a write did not
    /// happen rather than merely that the bytes are unchanged.
    reads: usize,
    writes: usize,
}

#[derive(Debug)]
struct OutOfRange;

impl BlockDevice for Ram {
    type Error = OutOfRange;

    fn read(&mut self, start_block: u64, blocks: &mut [u8]) -> Result<(), OutOfRange> {
        let at = start_block as usize * 512;
        let end = at + blocks.len();
        if end > self.blocks.len() {
            return Err(OutOfRange);
        }
        blocks.copy_from_slice(&self.blocks[at..end]);
        self.reads += 1;
        Ok(())
    }

    fn write(&mut self, start_block: u64, blocks: &[u8]) -> Result<(), OutOfRange> {
        let at = start_block as usize * 512;
        let end = at + blocks.len();
        if end > self.blocks.len() {
            return Err(OutOfRange);
        }
        self.blocks[at..end].copy_from_slice(blocks);
        self.writes += 1;
        Ok(())
    }

    fn block_count(&mut self) -> Result<Option<u64>, OutOfRange> {
        Ok(Some(self.blocks.len() as u64 / 512))
    }
}

/// A freshly formatted FAT32 volume, made by `mkfs.vfat`.
fn blank_volume(name: &str) -> Ram {
    let path = scratch(name);
    std::fs::write(&path, vec![0u8; IMAGE_BYTES as usize]).expect("creating the image");
    let made = Command::new("mkfs.vfat")
        .args(["-F", "32", "-n", "TEST"])
        .arg(&path)
        .output()
        .expect("running mkfs.vfat — is dosfstools installed?");
    assert!(
        made.status.success(),
        "mkfs.vfat failed: {}",
        String::from_utf8_lossy(&made.stderr)
    );
    let blocks = std::fs::read(&path).expect("reading the image back");
    let _ = std::fs::remove_file(&path);
    Ram {
        blocks,
        reads: 0,
        writes: 0,
    }
}

fn scratch(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("rpi-loader-ota-{name}-{}.img", std::process::id()))
}

/// Runs `fsck.vfat` over a volume and fails the test with what it said.
///
/// `-n` so it reports rather than repairs: a filesystem this crate wrote is
/// either right or it is a bug, and letting `fsck` quietly fix one would
/// hide exactly what the test is for.
///
/// # Why the exit status is not the oracle
///
/// Because it is not one. `fsck.vfat -n` exits **0** even when it has found
/// something: a volume whose two FAT copies disagree reports `FATs differ`
/// and still exits 0, since in no-change mode it has nothing to report the
/// *outcome* of. Asserting on the status would be a test that cannot fail,
/// which is worse than no test because it looks like one.
///
/// So the output is the oracle. A clean run prints exactly two lines — the
/// version banner and a `N files, X/Y clusters` summary — and every
/// complaint is an extra line. Filtering those two out and requiring
/// nothing to remain catches whatever `fsck` decides to say, rather than a
/// list of problem strings guessed at in advance.
fn fsck(volume: &Ram, name: &str) {
    let path = scratch(name);
    std::fs::write(&path, &volume.blocks).expect("writing the image out");
    let checked = Command::new("fsck.vfat")
        .arg("-n")
        .arg(&path)
        .output()
        .expect("running fsck.vfat — is dosfstools installed?");
    let _ = std::fs::remove_file(&path);

    let output = format!(
        "{}{}",
        String::from_utf8_lossy(&checked.stdout),
        String::from_utf8_lossy(&checked.stderr)
    );
    let complaints: Vec<&str> = output
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty())
        .filter(|line| !line.starts_with("fsck.fat "))
        .filter(|line| !(line.contains(" files, ") && line.ends_with(" clusters")))
        .collect();

    assert!(
        complaints.is_empty(),
        "fsck.vfat found fault with the volume:\n  {}",
        complaints.join("\n  ")
    );
}

/// Records the order entries were handled in, and how.
#[derive(Default)]
struct Order {
    events: Vec<String>,
}

impl Progress for Order {
    fn wrote(&mut self, entry: Entry<'_>) {
        self.events.push(format!("wrote {}", entry.path));
    }

    fn skipped(&mut self, entry: Entry<'_>) {
        self.events.push(format!("skipped {}", entry.path));
    }
}

fn file<'a>(path: &'a str, data: &'a [u8]) -> Entry<'a> {
    Entry {
        role: Role::File,
        path,
        data,
    }
}

/// Reads a file back off a volume the way anything else would.
fn read(volume: &mut FileSystem<Ram>, path: &str) -> Vec<u8> {
    let handle = volume
        .open(path)
        .unwrap_or_else(|e| panic!("opening {path}: {e:?}"));
    volume.read_all(&handle).expect("reading it")
}

#[test]
fn installs_every_entry_where_its_path_says() {
    let entries = [
        Entry {
            role: Role::Kernel,
            path: "KERNEL7.IMG",
            data: b"kernel bytes",
        },
        file("WWW/INDEX.HTM", b"<html>"),
        file("WWW/CSS/SITE.CSS", b"body{}"),
        Entry {
            role: Role::Config,
            path: "CONFIG.TXT",
            data: b"arm_64bit=0\n",
        },
    ];
    let bytes = encode(&FORMAT, &entries).unwrap();

    let mut volume = FileSystem::mount(blank_volume("install")).expect("mounting");
    let report = apply(&mut volume, &FORMAT, &bytes, &mut ()).expect("applying");

    assert_eq!(report.written, 4);
    assert_eq!(report.skipped, 0);
    assert_eq!(report.kernel_len, Some(12));

    assert_eq!(read(&mut volume, "KERNEL7.IMG"), b"kernel bytes");
    assert_eq!(read(&mut volume, "CONFIG.TXT"), b"arm_64bit=0\n");
    // Nested, on a volume that had no such directories a moment ago.
    assert_eq!(read(&mut volume, "WWW/INDEX.HTM"), b"<html>");
    assert_eq!(read(&mut volume, "WWW/CSS/SITE.CSS"), b"body{}");

    fsck(volume.device(), "install");
}

#[test]
fn the_kernel_is_written_last() {
    // The whole safety argument in one assertion: with one boot image, that
    // write is the commit, so everything that could fail has to have failed
    // already.
    let entries = [
        Entry {
            role: Role::Kernel,
            path: "KERNEL7.IMG",
            data: b"kernel",
        },
        Entry {
            role: Role::Firmware,
            path: "BOOTCODE.BIN",
            data: b"boot",
        },
        Entry {
            role: Role::Config,
            path: "CONFIG.TXT",
            data: b"x=1\n",
        },
        file("WWW/INDEX.HTM", b"<html>"),
    ];
    let bytes = encode(&FORMAT, &entries).unwrap();

    let mut volume = FileSystem::mount(blank_volume("order")).expect("mounting");
    let mut order = Order::default();
    apply(&mut volume, &FORMAT, &bytes, &mut order).expect("applying");

    assert_eq!(
        order.events,
        vec![
            "wrote WWW/INDEX.HTM",
            "wrote BOOTCODE.BIN",
            "wrote CONFIG.TXT",
            "wrote KERNEL7.IMG",
        ],
        "entries were not written in role order"
    );
}

#[test]
fn an_unchanged_entry_is_read_and_not_rewritten() {
    let entries = [
        Entry {
            role: Role::Firmware,
            path: "START.ELF",
            data: &[0xA5; 40_000],
        },
        Entry {
            role: Role::Firmware,
            path: "FIXUP.DAT",
            data: b"fixup",
        },
        file("SETTINGS.CFG", b"zone=1"),
    ];
    let bytes = encode(&FORMAT, &entries).unwrap();

    let mut volume = FileSystem::mount(blank_volume("skip")).expect("mounting");
    apply(&mut volume, &FORMAT, &bytes, &mut ()).expect("first install");

    // Same bundle again. Nothing has changed, so nothing should be written.
    let before = volume.device().writes;
    let mut order = Order::default();
    let report = apply(&mut volume, &FORMAT, &bytes, &mut order).expect("second install");

    assert_eq!(report.written, 0);
    assert_eq!(report.skipped, 3);
    assert!(
        order
            .events
            .iter()
            .all(|event| event.starts_with("skipped")),
        "{:?}",
        order.events
    );
    // The device is the witness, not the byte contents: an installer that
    // wrote the same bytes back would pass a content check and cost the card
    // the same erase cycles.
    assert_eq!(
        volume.device().writes,
        before,
        "the card was written to for an update that changed nothing"
    );

    fsck(volume.device(), "skip");
}

#[test]
fn a_changed_entry_beside_unchanged_ones_is_the_only_one_written() {
    let unchanged: &[u8] = &[0x5A; 20_000];
    let first = [
        Entry {
            role: Role::Firmware,
            path: "START.ELF",
            data: unchanged,
        },
        Entry {
            role: Role::Firmware,
            path: "FIXUP.DAT",
            data: b"fixup",
        },
        file("SETTINGS.CFG", b"zone=1"),
    ];
    let mut volume = FileSystem::mount(blank_volume("partial")).expect("mounting");
    apply(
        &mut volume,
        &FORMAT,
        &encode(&FORMAT, &first).unwrap(),
        &mut (),
    )
    .expect("first");

    let second = [
        Entry {
            role: Role::Firmware,
            path: "START.ELF",
            data: unchanged,
        },
        Entry {
            role: Role::Firmware,
            path: "FIXUP.DAT",
            data: b"fixup",
        },
        file("SETTINGS.CFG", b"zone=2"),
    ];
    let mut order = Order::default();
    let report = apply(
        &mut volume,
        &FORMAT,
        &encode(&FORMAT, &second).unwrap(),
        &mut order,
    )
    .expect("second");

    assert_eq!(report.written, 1);
    assert_eq!(report.skipped, 2);
    assert_eq!(read(&mut volume, "SETTINGS.CFG"), b"zone=2");
    fsck(volume.device(), "partial");
}

#[test]
fn a_same_length_change_is_still_noticed() {
    // The length check is the fast path, not the answer. Two files of equal
    // length and different content must not be mistaken for each other.
    let mut volume = FileSystem::mount(blank_volume("samelen")).expect("mounting");
    let before = [file("A.TXT", b"aaaaaaaa")];
    apply(
        &mut volume,
        &FORMAT,
        &encode(&FORMAT, &before).unwrap(),
        &mut (),
    )
    .expect("first");

    let after = [file("A.TXT", b"bbbbbbbb")];
    let report = apply(
        &mut volume,
        &FORMAT,
        &encode(&FORMAT, &after).unwrap(),
        &mut (),
    )
    .expect("second");

    assert_eq!(report.written, 1, "a same-length change was skipped");
    assert_eq!(read(&mut volume, "A.TXT"), b"bbbbbbbb");
}

#[test]
fn a_bundle_with_no_kernel_installs_and_reports_none() {
    let entries = [file("WWW/INDEX.HTM", b"<html>")];
    let bytes = encode(&FORMAT, &entries).unwrap();

    let mut volume = FileSystem::mount(blank_volume("nokernel")).expect("mounting");
    let report = apply(&mut volume, &FORMAT, &bytes, &mut ()).expect("applying");

    assert_eq!(
        report,
        Report {
            kernel_len: None,
            written: 1,
            skipped: 0,
        }
    );
    fsck(volume.device(), "nokernel");
}

#[test]
fn a_rejected_bundle_writes_nothing() {
    let mut volume = FileSystem::mount(blank_volume("rejected")).expect("mounting");
    let bytes = encode(&FORMAT, &[file("A.TXT", b"a")]).unwrap();

    let other = Format {
        magic: *b"OTHR",
        max_entries: 16,
    };
    let before = volume.device().writes;
    apply(&mut volume, &other, &bytes, &mut ()).expect_err("should be refused");

    assert_eq!(
        volume.device().writes,
        before,
        "a bundle refused at the header still touched the card"
    );
}

#[test]
fn a_failed_install_leaves_a_volume_fsck_accepts() {
    // The install runs out of room part-way through, which is the shape of
    // every real failure here: some entries are on the card and some are
    // not. What must not happen is that the volume itself is left corrupt,
    // because that is a card that has to be reformatted rather than an
    // update that has to be retried.
    // On the heap rather than a repeat-expression literal, which would be
    // promoted to a static of the same size and put it in the binary.
    let huge = vec![0x11u8; IMAGE_BYTES as usize + 1024 * 1024];
    let entries = [file("SMALL.TXT", b"fits"), file("HUGE.BIN", &huge)];
    let bytes = encode(&FORMAT, &entries).unwrap();

    let mut volume = FileSystem::mount(blank_volume("failed")).expect("mounting");
    apply(&mut volume, &FORMAT, &bytes, &mut ()).expect_err("should not fit");

    // The entry that did fit is there and readable; the one that did not is
    // not half-written into the directory.
    assert_eq!(read(&mut volume, "SMALL.TXT"), b"fits");
    fsck(volume.device(), "failed");
}
