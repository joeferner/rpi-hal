//! Timing an install: what each entry's write and read-back cost.
//!
//! The installer reports what it is doing through [`Progress`] and leaves
//! the measuring to its caller. This is the measuring every board was
//! writing for itself: start a phase when an entry starts, end it when the
//! write is done, end the next when the read-back is, and turn bytes and
//! milliseconds into a rate.
//!
//! It still knows nothing about the board. The clock is a function the
//! caller passes — `|| embassy_time::Instant::now().as_millis()`, or a
//! host's — so no runtime's clock reaches this crate, and nothing is
//! logged: what was measured is recorded, one [`Timed`] per entry, and the
//! caller prints it, or sends it back to whoever uploaded the bundle.
//!
//! ```ignore
//! let mut measure = Measure::new(|| Instant::now().as_millis());
//! let installed = apply::apply(volume, &FORMAT, bundle, &mut measure);
//! for entry in measure.entries() {
//!     logln!("ota: {entry}");
//! }
//! let report = installed?;
//! ```
//!
//! Recorded rather than logged as it goes, which costs nothing: an install
//! blocks whatever is driving the card for its whole length, so there is
//! nothing to watch live. And the record outlives a failure — the entries
//! that were done before one are still there to print.
//!
//! # Commands as well as time
//!
//! A card charges per command, so a time alone cannot say whether a slow
//! write moved too many bytes or asked too many times. Given the
//! `resident_fat::counted::Counters` the card's device counts into,
//! [`Measure::counting`] records each phase's calls and blocks beside its
//! milliseconds.
//!
//! [`Progress`]: crate::apply::Progress

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt;

#[cfg(target_has_atomic = "32")]
use resident_fat::counted::{Counters, Counts};

use crate::apply::Progress;
use crate::bundle::{Entry, Role};

/// One phase of one entry: writing it, or reading it back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Phase {
    /// How long it took, in milliseconds.
    pub ms: u64,
    /// The bytes it moved — the entry's length.
    pub bytes: usize,
    /// The card commands it took, when the caller supplied counters.
    #[cfg(target_has_atomic = "32")]
    pub counts: Option<Counts>,
}

impl Phase {
    /// Kibibytes per second, or 0 for a phase too quick to time.
    ///
    /// Integer throughout: a board doing this has no floating point to
    /// spare for it, and a card's rate needs no decimal place to read.
    pub fn rate_kib_s(&self) -> u64 {
        if self.ms == 0 {
            return 0;
        }
        self.bytes as u64 * 1000 / (self.ms * 1024)
    }
}

/// `412 ms (3961 KiB/s)`, and the commands after it when they were
/// counted.
impl fmt::Display for Phase {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} ms ({} KiB/s)", self.ms, self.rate_kib_s())?;
        #[cfg(target_has_atomic = "32")]
        if let Some(counts) = &self.counts {
            write!(f, ", {counts}")?;
        }
        Ok(())
    }
}

/// What happened to one entry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Outcome {
    /// Written, then read back and matched.
    Written {
        /// The write.
        write: Phase,
        /// The read-back.
        verify: Phase,
    },
    /// The card already held it, which a read established; nothing was
    /// written.
    Unchanged {
        /// The read that established it.
        read: Phase,
    },
    /// Started and never finished — the install failed on this entry. The
    /// write, if it completed, is here.
    Unfinished {
        /// The write, when that much was done before the failure.
        write: Option<Phase>,
    },
}

/// One entry of a bundle, and what installing it cost.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Timed {
    /// Where it was written, relative to the volume's root.
    pub path: String,
    /// What the bundle said it was.
    pub role: Role,
    /// Its length.
    pub bytes: usize,
    /// What happened to it.
    pub outcome: Outcome,
}

/// One line: `/kernel7.img (1671168 bytes): write 412 ms (3961 KiB/s),
/// verify 120 ms (13600 KiB/s)`.
impl fmt::Display for Timed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "/{} ({} bytes): ", self.path, self.bytes)?;
        match &self.outcome {
            Outcome::Written { write, verify } => write!(f, "write {write}, verify {verify}"),
            // Worth a line rather than silence: an update that rewrote
            // nothing and one that never started look the same in a log
            // that only lists writes.
            Outcome::Unchanged { read } => write!(f, "unchanged, not rewritten -- read in {read}"),
            Outcome::Unfinished { write: Some(write) } => {
                write!(f, "write {write}, read-back did not finish")
            }
            Outcome::Unfinished { write: None } => f.write_str("write did not finish"),
        }
    }
}

/// Times an install, entry by entry. Hand it to
/// [`apply::apply`](crate::apply::apply) as the [`Progress`].
pub struct Measure<'c, F> {
    now_ms: F,
    #[cfg(target_has_atomic = "32")]
    counters: Option<&'c Counters>,
    #[cfg(not(target_has_atomic = "32"))]
    counters: core::marker::PhantomData<&'c ()>,
    started_ms: u64,
    phase_ms: u64,
    #[cfg(target_has_atomic = "32")]
    started_counts: Counts,
    #[cfg(target_has_atomic = "32")]
    phase_counts: Counts,
    entries: Vec<Timed>,
}

impl<'c, F: FnMut() -> u64> Measure<'c, F> {
    /// Times an install with `now_ms`, a monotonic clock in milliseconds.
    /// The whole install is timed from here.
    pub fn new(mut now_ms: F) -> Self {
        let started_ms = now_ms();
        Measure {
            now_ms,
            #[cfg(target_has_atomic = "32")]
            counters: None,
            #[cfg(not(target_has_atomic = "32"))]
            counters: core::marker::PhantomData,
            started_ms,
            phase_ms: started_ms,
            #[cfg(target_has_atomic = "32")]
            started_counts: Counts::default(),
            #[cfg(target_has_atomic = "32")]
            phase_counts: Counts::default(),
            entries: Vec::new(),
        }
    }

    /// Times an install, and counts its card commands through `counters`
    /// — the ones the volume's `resident_fat::counted::Counted` device
    /// counts into.
    #[cfg(target_has_atomic = "32")]
    pub fn counting(now_ms: F, counters: &'c Counters) -> Self {
        let mut measure = Measure::new(now_ms);
        measure.counters = Some(counters);
        measure.started_counts = counters.now();
        measure.phase_counts = measure.started_counts;
        measure
    }

    /// Every entry the installer reached, in the order it reached them.
    pub fn entries(&self) -> &[Timed] {
        &self.entries
    }

    /// The kernel's write and read-back, when it was written.
    ///
    /// The figure worth comparing between updates: the kernel is the one
    /// entry whose size is set by the build rather than by whatever else a
    /// bundle carries.
    pub fn kernel(&self) -> Option<(Phase, Phase)> {
        self.entries.iter().find_map(|entry| match entry.outcome {
            Outcome::Written { write, verify } if entry.role == Role::Kernel => {
                Some((write, verify))
            }
            _ => None,
        })
    }

    /// Milliseconds since this was made — the whole install, once it is
    /// over.
    pub fn elapsed_ms(&mut self) -> u64 {
        (self.now_ms)().saturating_sub(self.started_ms)
    }

    /// The card commands since this was made, when counted.
    #[cfg(target_has_atomic = "32")]
    pub fn counts(&self) -> Option<Counts> {
        self.counters
            .map(|counters| counters.since(self.started_counts))
    }

    /// Starts a phase.
    fn begin(&mut self) {
        self.phase_ms = (self.now_ms)();
        #[cfg(target_has_atomic = "32")]
        if let Some(counters) = self.counters {
            self.phase_counts = counters.now();
        }
    }

    /// Ends the phase in progress over `bytes`, and starts the next.
    fn end(&mut self, bytes: usize) -> Phase {
        let now = (self.now_ms)();
        let phase = Phase {
            ms: now.saturating_sub(self.phase_ms),
            bytes,
            #[cfg(target_has_atomic = "32")]
            counts: self
                .counters
                .map(|counters| counters.since(self.phase_counts)),
        };
        self.begin();
        phase
    }

    /// The entry in progress, which `starting` pushed.
    fn current(&mut self) -> Option<&mut Timed> {
        self.entries.last_mut()
    }
}

impl<F: FnMut() -> u64> Progress for Measure<'_, F> {
    fn starting(&mut self, entry: Entry<'_>) {
        self.entries.push(Timed {
            path: String::from(entry.path),
            role: entry.role,
            bytes: entry.data.len(),
            outcome: Outcome::Unfinished { write: None },
        });
        self.begin();
    }

    fn wrote(&mut self, entry: Entry<'_>) {
        let write = self.end(entry.data.len());
        if let Some(current) = self.current() {
            current.outcome = Outcome::Unfinished { write: Some(write) };
        }
    }

    fn verified(&mut self, entry: Entry<'_>) {
        let verify = self.end(entry.data.len());
        // Not a let chain: this crate's MSRV predates them.
        if let Some(current) = self.current() {
            if let Outcome::Unfinished { write: Some(write) } = current.outcome {
                current.outcome = Outcome::Written { write, verify };
            }
        }
    }

    fn skipped(&mut self, entry: Entry<'_>) {
        let read = self.end(entry.data.len());
        if let Some(current) = self.current() {
            current.outcome = Outcome::Unchanged { read };
        }
    }
}
