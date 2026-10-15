//! Writing Prometheus's text exposition format.
//!
//! The writer, not the series: which families a board publishes is the
//! board's, and this is the part every board otherwise writes the same way —
//! the `# HELP` / `# TYPE` preamble, label escaping, and a decimal point
//! without a float formatter.
//!
//! ```ignore
//! let mut out = Exposition::with_capacity(4096);
//! out.build_info("water", &rpi_kickstart::build_info!());
//! out.uptime("water");
//! out.family("water_zone_wet", Kind::Gauge, "1 while a zone reads wet.");
//! for (index, zone) in zones.iter().enumerate() {
//!     out.sample("water_zone_wet", &[("zone", &(index + 1))], zone.wet);
//! }
//! // `picoserve`, with `web`:
//! out.into_body()
//! ```
//!
//! # Decimals without a float formatter
//!
//! Prometheus convention is base SI units — `_celsius`, not `_decicelsius` —
//! so values carry a decimal point, and a kernel with no float formatter
//! has no reason to gain one for this. What a board measures is already an
//! integer in a known sub-unit, so
//! [`sample_fixed`](crate::metrics::Exposition::sample_fixed) prints the
//! integer part, a point, and the remainder zero-padded. Nothing passes
//! through a float, which on a target with no hardware floating point is
//! also the cheap path.
//!
//! # Absent is not zero
//!
//! Not something this module can enforce, and the mistake worth naming: a
//! sensor that has not reported should be *omitted*, never written as `0`.
//! A missing thermometer published as zero reads as a freezing day, which is
//! plausible enough to alert on. An age beside the reading is what makes the
//! absence legible — a series that stops and an age that climbs are the same
//! event seen twice, and the age is the one a scraper can alert on.
//!
//! # A malformed document is refused whole
//!
//! Prometheus rejects the entire scrape over one bad line, so one quote in
//! one label value written from the card would take every series with it.
//! That is why label values are only ever written through
//! [`sample`](crate::metrics::Exposition::sample), which escapes them, and
//! why help text is escaped too.
//!
//! # What a board is running
//!
//! [`build_info`](crate::metrics::Exposition::build_info) publishes the image's identity — the board
//! crate's version, the commit it was built from and whether the tree had
//! uncommitted changes, and a hash of its `Cargo.lock` — so "which boards
//! still run the build with the advisory against it" is one query rather
//! than a visit to each console. The version alone does not answer it: two
//! images of one version differ after a `cargo update`, and only the lock
//! hash tells them apart.
//!
//! All of it is captured on the host, by the board's `build.rs`, behind the
//! `metrics-build` feature, which needs `std` and so is a build-dependency
//! only:
//!
//! ```toml
//! [dependencies]
//! rpi-kickstart = { version = "…", features = ["metrics"] }
//!
//! [build-dependencies]
//! rpi-kickstart = { version = "…", features = ["metrics-build"] }
//! ```
//!
//! ```ignore
//! // build.rs
//! fn main() {
//!     rpi_kickstart::metrics::build::emit();
//! }
//! ```
//!
//! and [`build_info!`](crate::build_info) reads what it set. Nothing is
//! computed on the device.

use alloc::string::String;
use core::fmt::{self, Write as _};

#[cfg(feature = "metrics-build")]
pub mod build;

/// What an image is, for [`Exposition::build_info`]: constant for its whole
/// life, captured when it was built.
///
/// Usually made by [`build_info!`](crate::build_info) rather than by hand.
/// Every field is text, label values being text, and any of the last three
/// may be `unknown` — an image built outside a git checkout, or with no
/// `Cargo.lock` to be found — which is written as such rather than guessed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BuildInfo {
    /// The board crate's version, `CARGO_PKG_VERSION`.
    pub version: &'static str,
    /// The commit it was built from, abbreviated as `git` abbreviates it.
    pub git: &'static str,
    /// `true` if tracked files differed from that commit — an image built
    /// from an uncommitted tree, which no commit describes — else `false`.
    /// Untracked files do not count, as `git describe --dirty` has it.
    pub dirty: &'static str,
    /// The first eight hex digits of `Cargo.lock`'s SHA-256: what
    /// `sha256sum Cargo.lock | cut -c1-8` prints for the same file.
    pub lock: &'static str,
}

/// A [`BuildInfo`](crate::metrics::BuildInfo) for the crate it is written
/// in: its version, and what `metrics::build::emit` — behind
/// `metrics-build`, in its `build.rs` — recorded.
///
/// A macro rather than a function so that `env!` is expanded in the board's
/// crate, which is the one whose version and build script it means. The
/// variable names are literals because `env!` takes nothing else; `emit`
/// writes the same three, and its tests hold it to them. A board
/// that never calls `emit` fails to compile here, naming it, rather than
/// publishing a blank.
#[macro_export]
macro_rules! build_info {
    () => {
        $crate::metrics::BuildInfo {
            version: env!("CARGO_PKG_VERSION"),
            git: env!(
                "KICKSTART_BUILD_GIT",
                "call rpi_kickstart::metrics::build::emit() from build.rs"
            ),
            dirty: env!(
                "KICKSTART_BUILD_DIRTY",
                "call rpi_kickstart::metrics::build::emit() from build.rs"
            ),
            lock: env!(
                "KICKSTART_BUILD_LOCK",
                "call rpi_kickstart::metrics::build::emit() from build.rs"
            ),
        }
    };
}

/// The exposition format's content type, version and all.
///
/// A bare `text/plain` is read as this same format, so the version is not
/// what makes a scrape work — it is what makes the format a statement
/// rather than a default.
pub const CONTENT_TYPE: &str = "text/plain; version=0.0.4; charset=utf-8";

/// What a family's samples mean over time — the `# TYPE` line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// Only ever goes up, and a drop is a reset: a reboot, for anything
    /// counted since boot. Named `_total` by convention when it counts
    /// events.
    Counter,
    /// Goes up and down.
    Gauge,
}

impl Kind {
    fn as_str(self) -> &'static str {
        match self {
            Kind::Counter => "counter",
            Kind::Gauge => "gauge",
        }
    }
}

/// A sample's value: any integer, or a `bool` as `1` or `0`.
///
/// Integers rather than floats for the reason in the module documentation;
/// a value with a fractional part goes through
/// [`Exposition::sample_fixed`] instead. Sealed, so the set stays what
/// prints correctly.
pub trait Value: sealed::Sealed {
    /// Writes the value as the format has it.
    fn write(&self, out: &mut String);
}

mod sealed {
    pub trait Sealed {}
}

macro_rules! integer_values {
    ($($t:ty),*) => {$(
        impl sealed::Sealed for $t {}
        impl Value for $t {
            fn write(&self, out: &mut String) {
                let _ = write!(out, "{self}");
            }
        }
    )*};
}
integer_values!(u8, u16, u32, u64, usize, i8, i16, i32, i64, isize);

impl sealed::Sealed for bool {}
impl Value for bool {
    fn write(&self, out: &mut String) {
        out.push(if *self { '1' } else { '0' });
    }
}

/// Labels on a sample: `(name, value)` pairs, written `{name="value",...}`.
///
/// Values are anything `Display` — a zone number, a name read off the card
/// — and are escaped as they are written. Names are not: they are the
/// board's own literals, and a name needing escaping is not a valid name.
pub type Labels<'a> = &'a [(&'a str, &'a dyn fmt::Display)];

/// An exposition document being written.
#[derive(Debug, Default)]
pub struct Exposition {
    text: String,
}

impl Exposition {
    /// An empty document.
    pub fn new() -> Self {
        Exposition::default()
    }

    /// An empty document with room for `bytes` before it grows. Size it a
    /// little over what the board's families come to, so a scrape is
    /// written without reallocating; nothing depends on it being right.
    pub fn with_capacity(bytes: usize) -> Self {
        Exposition {
            text: String::with_capacity(bytes),
        }
    }

    /// Starts a family: its `# HELP` and `# TYPE` lines.
    ///
    /// Both rather than samples alone: the type is what tells a scraper
    /// whether a drop is a counter restarting or a value going down, and
    /// the help is what a dashboard shows beside a series nobody wrote the
    /// dashboard for. `help` is escaped as the format requires — a
    /// backslash doubled, a newline written as `\n`.
    pub fn family(&mut self, name: &str, kind: Kind, help: &str) {
        let _ = write!(self.text, "# HELP {name} ");
        for c in help.chars() {
            match c {
                '\\' => self.text.push_str("\\\\"),
                '\n' => self.text.push_str("\\n"),
                c => self.text.push(c),
            }
        }
        let _ = writeln!(self.text, "\n# TYPE {name} {}", kind.as_str());
    }

    /// Writes one sample.
    pub fn sample(&mut self, name: &str, labels: Labels<'_>, value: impl Value) {
        self.head(name, labels);
        value.write(&mut self.text);
        self.text.push('\n');
    }

    /// Writes one sample whose value is an integer in a sub-unit, as a
    /// decimal in the base unit: `value` times ten to the minus `decimals`.
    /// Tenths of a degree with `decimals` 1 are degrees; microvolts with 6
    /// are volts.
    pub fn sample_fixed(&mut self, name: &str, labels: Labels<'_>, value: i64, decimals: u32) {
        self.head(name, labels);
        // The sign comes off the whole value before it is split, not from
        // the integer part. Between -1 and 0 the integer part is `0`, which
        // carries no sign, so a naive split prints -0.5 as "0.5" — a
        // plausible reading with the wrong sign, in exactly the range where
        // the sign is the difference between freezing and not.
        if value < 0 {
            self.text.push('-');
        }
        let magnitude = value.unsigned_abs();
        match 10u64.checked_pow(decimals) {
            Some(scale) if decimals > 0 => {
                let _ = writeln!(
                    self.text,
                    "{}.{:0width$}",
                    magnitude / scale,
                    magnitude % scale,
                    width = decimals as usize
                );
            }
            // No decimals is a plain integer, and more than a `u64` can
            // scale by is more than any `i64` has: every digit is after the
            // point.
            Some(_) => {
                let _ = writeln!(self.text, "{magnitude}");
            }
            None => {
                let _ = writeln!(
                    self.text,
                    "0.{:0width$}",
                    magnitude,
                    width = decimals as usize
                );
            }
        }
    }

    /// `<prefix>_build_info{version="…",git="…",dirty="…",lock="…"} 1`,
    /// with its family: what the board is running, as labels on a
    /// constant, so a dashboard can mark an update where it happened and a
    /// query can find every board still on a given build. See [`BuildInfo`]
    /// for the labels and the module documentation for where they come
    /// from.
    pub fn build_info(&mut self, prefix: &str, info: &BuildInfo) {
        let name = alloc::format!("{prefix}_build_info");
        self.family(
            &name,
            Kind::Gauge,
            "The running firmware: its version, the commit it was built from, whether that tree had uncommitted changes, and a hash of its Cargo.lock, as labels on a constant 1.",
        );
        self.sample(
            &name,
            &[
                ("version", &info.version),
                ("git", &info.git),
                ("dirty", &info.dirty),
                ("lock", &info.lock),
            ],
            1u8,
        );
    }

    /// `<prefix>_uptime_seconds`, with its family: seconds since boot.
    ///
    /// A counter, because a reboot is precisely a counter reset and that is
    /// what a scraper should see rather than a value that went backwards.
    /// No `_total`: that convention is for counts of events, and this is
    /// not one.
    pub fn uptime(&mut self, prefix: &str) {
        let name = alloc::format!("{prefix}_uptime_seconds");
        self.family(&name, Kind::Counter, "Seconds since this board booted.");
        self.sample(&name, &[], embassy_time::Instant::now().as_secs());
    }

    /// `<prefix>_heap_used_bytes` and `<prefix>_heap_free_bytes`, with
    /// their families: the [`heap`](crate::heap) allocator's split, read
    /// by [`heap::usage`](crate::heap::usage).
    ///
    /// The one to watch is free, over weeks: a heap whose free space falls
    /// and never recovers is something being allocated and not freed, and on
    /// a board meant to run for months that is a fault that arrives long
    /// after anyone is looking at the console. Reading it walks the heap —
    /// see `heap::usage` for the cost.
    #[cfg(feature = "heap")]
    pub fn heap(&mut self, prefix: &str) {
        let usage = crate::heap::usage();
        let used = alloc::format!("{prefix}_heap_used_bytes");
        self.family(
            &used,
            Kind::Gauge,
            "Heap bytes not available to allocate: live allocations and the allocator's own bookkeeping.",
        );
        self.sample(&used, &[], usage.used);
        let free = alloc::format!("{prefix}_heap_free_bytes");
        self.family(
            &free,
            Kind::Gauge,
            "Heap bytes in free blocks. Falling over weeks and never recovering means something is never freed.",
        );
        self.sample(&free, &[], usage.free);
    }

    /// `<prefix>_card_commands_total` and `<prefix>_card_blocks_total`, with
    /// their families, each split `op="read"` and `op="write"`: what the
    /// card has been asked to do since boot, from the `Counters` its
    /// `resident_fat::counted::Counted` device counts into.
    ///
    /// Both, because they say different things: blocks are the work, and
    /// commands are the overhead — a card charges per command, not per
    /// block, so blocks per command is the figure that says whether writes
    /// are going out in runs or one block at a time.
    ///
    /// Counters, so a reboot reads as the reset it is. The underlying counts
    /// are `u32` and wrap, which a scraper sees as a reset too.
    #[cfg(feature = "storage")]
    pub fn card(&mut self, prefix: &str, counters: &resident_fat::counted::Counters) {
        let counts = counters.now();
        let commands = alloc::format!("{prefix}_card_commands_total");
        self.family(
            &commands,
            Kind::Counter,
            "Commands sent to the card since boot. A card charges per command, not per block.",
        );
        self.sample(&commands, &[("op", &"read")], counts.read_calls);
        self.sample(&commands, &[("op", &"write")], counts.write_calls);
        let blocks = alloc::format!("{prefix}_card_blocks_total");
        self.family(
            &blocks,
            Kind::Counter,
            "512-byte blocks moved to or from the card since boot.",
        );
        self.sample(&blocks, &[("op", &"read")], counts.read_blocks);
        self.sample(&blocks, &[("op", &"write")], counts.write_blocks);
    }

    /// The document so far.
    pub fn as_str(&self) -> &str {
        &self.text
    }

    /// The finished document.
    pub fn into_string(self) -> String {
        self.text
    }

    /// The finished document as a response body with [`CONTENT_TYPE`].
    #[cfg(feature = "web")]
    pub fn into_body(self) -> crate::web::TextBody {
        crate::web::TextBody {
            body: self.text,
            content_type: CONTENT_TYPE,
        }
    }

    /// `name{labels} `, escaping each label value.
    fn head(&mut self, name: &str, labels: Labels<'_>) {
        self.text.push_str(name);
        if !labels.is_empty() {
            self.text.push('{');
            for (index, (label, value)) in labels.iter().enumerate() {
                if index > 0 {
                    self.text.push(',');
                }
                let _ = write!(self.text, "{label}=\"");
                let _ = write!(Escaped(&mut self.text), "{value}");
                self.text.push('"');
            }
            self.text.push('}');
        }
        self.text.push(' ');
    }
}

/// Writes through to `0`, escaped as the inside of a label value: a
/// backslash, a quote and a newline, which are the three the format
/// reserves there.
struct Escaped<'a>(&'a mut String);

impl fmt::Write for Escaped<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.chars() {
            match c {
                '\\' => self.0.push_str("\\\\"),
                '"' => self.0.push_str("\\\""),
                '\n' => self.0.push_str("\\n"),
                c => self.0.push(c),
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_family_and_its_samples() {
        let mut out = Exposition::new();
        out.family("board_zone_wet", Kind::Gauge, "1 while a zone reads wet.");
        out.sample("board_zone_wet", &[("zone", &1)], true);
        out.sample("board_zone_wet", &[("zone", &2)], false);
        out.family("board_joins_total", Kind::Counter, "Joins.");
        out.sample("board_joins_total", &[], u64::MAX);
        assert_eq!(
            out.as_str(),
            "# HELP board_zone_wet 1 while a zone reads wet.\n\
             # TYPE board_zone_wet gauge\n\
             board_zone_wet{zone=\"1\"} 1\n\
             board_zone_wet{zone=\"2\"} 0\n\
             # HELP board_joins_total Joins.\n\
             # TYPE board_joins_total counter\n\
             board_joins_total 18446744073709551615\n"
        );
    }

    #[test]
    fn label_values_are_escaped() {
        let mut out = Exposition::new();
        let name = "back\\slash \"quoted\"\nnext";
        out.sample("board_zone_info", &[("zone", &3), ("name", &name)], 1u8);
        assert_eq!(
            out.as_str(),
            "board_zone_info{zone=\"3\",name=\"back\\\\slash \\\"quoted\\\"\\nnext\"} 1\n"
        );
    }

    #[cfg(feature = "storage")]
    #[test]
    fn card_counts_are_split_by_operation() {
        use resident_fat::BlockDevice;
        use resident_fat::counted::{Counted, Counters};

        /// Accepts every transfer, so the counter in front of it has
        /// something to count.
        struct Nothing;
        impl BlockDevice for Nothing {
            type Error = ();
            fn read(&mut self, _: u64, _: &mut [u8]) -> Result<(), ()> {
                Ok(())
            }
            fn write(&mut self, _: u64, _: &[u8]) -> Result<(), ()> {
                Ok(())
            }
            fn block_count(&mut self) -> Result<Option<u64>, ()> {
                Ok(None)
            }
        }

        let counters = Counters::new();
        let mut card = Counted::new(Nothing, &counters);
        card.read(0, &mut [0; 512 * 3]).unwrap();
        card.write(0, &[0; 512 * 2]).unwrap();
        card.write(8, &[0; 512]).unwrap();

        let mut out = Exposition::new();
        out.card("board", &counters);
        let samples: alloc::vec::Vec<&str> = out
            .as_str()
            .lines()
            .filter(|line| !line.starts_with('#'))
            .collect();
        assert_eq!(
            samples,
            [
                "board_card_commands_total{op=\"read\"} 1",
                "board_card_commands_total{op=\"write\"} 2",
                "board_card_blocks_total{op=\"read\"} 3",
                "board_card_blocks_total{op=\"write\"} 3",
            ]
        );
        assert!(
            out.as_str()
                .contains("# TYPE board_card_blocks_total counter\n")
        );
    }

    #[test]
    fn help_is_escaped() {
        let mut out = Exposition::new();
        out.family("x", Kind::Gauge, "a \\ b\nc");
        assert_eq!(out.as_str(), "# HELP x a \\\\ b\\nc\n# TYPE x gauge\n");
    }

    #[test]
    fn fixed_point_keeps_the_sign_between_minus_one_and_zero() {
        let cases: &[(i64, u32, &str)] = &[
            (223, 1, "22.3"),
            (-5, 1, "-0.5"),
            (-15, 1, "-1.5"),
            (0, 1, "0.0"),
            (1_234_567, 6, "1.234567"),
            (7, 3, "0.007"),
            (42, 0, "42"),
            (-42, 0, "-42"),
            (i64::MIN, 1, "-922337203685477580.8"),
            (5, 20, "0.00000000000000000005"),
        ];
        for &(value, decimals, text) in cases {
            let mut out = Exposition::new();
            out.sample_fixed("t", &[], value, decimals);
            assert_eq!(
                out.as_str(),
                alloc::format!("t {text}\n"),
                "{value} {decimals}"
            );
        }
    }

    #[test]
    fn build_info_labels_the_build() {
        let mut out = Exposition::new();
        let info = BuildInfo {
            version: "1.2.3",
            git: "a1b2c3d",
            dirty: "false",
            lock: "9f8e7d6c",
        };
        out.build_info("board", &info);
        assert!(
            out.as_str().ends_with(
                "# TYPE board_build_info gauge\n\
                 board_build_info{version=\"1.2.3\",git=\"a1b2c3d\",dirty=\"false\",lock=\"9f8e7d6c\"} 1\n"
            ),
            "{}",
            out.as_str()
        );
    }
}
