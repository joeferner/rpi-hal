//! Wall-clock time, which a Pi does not otherwise have.
//!
//! There is no battery-backed RTC on any of these boards, so a board boots
//! not knowing the date and stays that way until something tells it. This
//! module is the **sink** for that: whatever learns the time — SNTP, a
//! DS3231, a GPS, a timestamp handed over some other protocol — calls
//! [`set`](crate::clock::set), and everything that wants a date reads it
//! back through [`now_unix`](crate::clock::now_unix) and friends. No source is compiled in, so a board with an
//! RTC pays nothing for a network one.
//!
//! Everything that wants a date has to cope with not having one yet, which
//! is why the readers return `Option` rather than a plausible-looking 1970.
//! That is what makes TLS fail closed instead of validating certificates
//! against the epoch, and what lets a file written before the time was known
//! be told apart — `storage::mount` stamps such files with the FAT epoch.
//!
//! ```ignore
//! clock::set(rtc.read_unix_millis()?);
//! if let Some(now) = clock::now() {
//!     logln!("clock: {now}");
//! }
//! ```
//!
//! # What is stored
//!
//! Not "the time now", which would need a tick to keep current, but **the
//! Unix time that `embassy-time`'s zero corresponds to**. Reading the clock
//! is then one atomic load plus the monotonic counter, with nothing running
//! in between, and a later [`set`](crate::clock::set) re-datums the offset — which is how
//! drift is corrected without anything else noticing.
//!
//! Milliseconds rather than seconds, because a second is a visible amount of
//! error on something whose reason to know the time is to display it. What
//! limits the accuracy is the source — a network round trip, an RTC's
//! one-second register — not this.
//!
//! The monotonic half is `embassy-time`'s `Instant`, through its API only:
//! which driver is underneath is the board's choice, and `rpi-hal-embassy`
//! supplies one over the System Timer.
//!
//! # Local time
//!
//! What is stored is Unix time, which is UTC and has no zone in it. With the
//! `tz` feature, `LocalTime` is the other half: a `tz-rs` zone — parsed
//! from a TZif file, typically off the card — says what offset was in force
//! at an instant, and this applies it. The zone is passed in rather than
//! kept here: the datum above changes, from whichever task syncs it, where a
//! zone is read once at boot, so an argument says plainly which parts of a
//! board have a local time and which do not.

use core::fmt;
use core::sync::atomic::{AtomicU64, Ordering};

use embassy_time::Instant;

/// Sentinel meaning the clock has never been set. `u64::MAX` cannot occur
/// as a real value: it would put the boot instant some 580 million years
/// after the epoch.
const UNSET: u64 = u64::MAX;

/// Unix milliseconds corresponding to `Instant` zero, or [`UNSET`].
static EPOCH_AT_BOOT: AtomicU64 = AtomicU64::new(UNSET);

/// Sets the wall clock from an authoritative Unix timestamp in
/// milliseconds.
///
/// Safe to call repeatedly: a re-sync simply moves the datum, and nothing
/// reads the two halves separately, so there is no torn value to see.
///
/// A time earlier than the board's own uptime would put the boot instant
/// before 1970. It is clamped to the epoch rather than wrapped, which leaves
/// a clock that is wrong and visibly so rather than one 580 million years
/// out.
pub fn set(unix_millis: u64) {
    let elapsed = Instant::now().as_millis();
    EPOCH_AT_BOOT.store(unix_millis.saturating_sub(elapsed), Ordering::Relaxed);
}

/// The current Unix time in milliseconds, or `None` if the clock has never
/// been set.
pub fn now_unix_millis() -> Option<u64> {
    match EPOCH_AT_BOOT.load(Ordering::Relaxed) {
        UNSET => None,
        base => Some(base.saturating_add(Instant::now().as_millis())),
    }
}

/// The current Unix time in whole seconds, or `None` if the clock has never
/// been set.
pub fn now_unix() -> Option<u64> {
    now_unix_millis().map(|millis| millis / 1_000)
}

/// The current UTC date and time, or `None` if the clock has never been
/// set.
pub fn now() -> Option<DateTime> {
    now_unix().map(DateTime::from_unix)
}

/// A UTC date and time, broken out for display.
///
/// `u64` fields, wider than any of them needs, because everything here is
/// arithmetic on Unix seconds and a narrower type would be a cast at every
/// use for nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DateTime {
    /// Year, e.g. 2026.
    pub year: u64,
    /// Month, 1-12.
    pub month: u64,
    /// Day of the month, 1-31.
    pub day: u64,
    /// Hour, 0-23.
    pub hour: u64,
    /// Minute, 0-59.
    pub minute: u64,
    /// Second, 0-59.
    pub second: u64,
}

impl DateTime {
    /// Converts Unix seconds to a UTC calendar date.
    ///
    /// Howard Hinnant's `civil_from_days`, which is exact over the whole
    /// proleptic Gregorian calendar and needs no lookup tables. Leap seconds
    /// do not exist in Unix time, so none are accounted for.
    ///
    /// UTC, and only UTC. A local time needs a zone and its daylight-saving
    /// rules, which are political data that change and therefore belong in a
    /// file rather than in this function — see [`LocalTime`], behind `tz`.
    pub fn from_unix(unix_seconds: u64) -> Self {
        let days = unix_seconds / 86_400;
        let seconds_of_day = unix_seconds % 86_400;

        // Shift the epoch to 0000-03-01, which moves the leap day to the end
        // of the year and makes the month-length pattern regular.
        let shifted = days + 719_468;
        let era = shifted / 146_097;
        let day_of_era = shifted - era * 146_097;
        let year_of_era =
            (day_of_era - day_of_era / 1460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
        let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
        let month_prime = (5 * day_of_year + 2) / 153;

        let day = day_of_year - (153 * month_prime + 2) / 5 + 1;
        let month = if month_prime < 10 {
            month_prime + 3
        } else {
            month_prime - 9
        };
        let mut year = year_of_era + era * 400;
        if month <= 2 {
            year += 1;
        }

        DateTime {
            year,
            month,
            day,
            hour: seconds_of_day / 3600,
            minute: (seconds_of_day % 3600) / 60,
            second: seconds_of_day % 60,
        }
    }

    /// Which day this date is, counted from 1970-01-01.
    ///
    /// Only the date part. A [`LocalTime`]'s date is already in its zone, so
    /// there this is the *local* day — the one a display means by "today".
    pub fn epoch_day(&self) -> i64 {
        days_from_civil(self.year as i64, self.month as i64, self.day as i64)
    }
}

/// `2026-09-28 14:05:09 UTC`.
impl fmt::Display for DateTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:04}-{:02}-{:02} {:02}:{:02}:{:02} UTC",
            self.year, self.month, self.day, self.hour, self.minute, self.second
        )
    }
}

/// Days since 1970-01-01 for a proleptic Gregorian date.
///
/// Howard Hinnant's `days_from_civil`, the exact inverse of the
/// `civil_from_days` in [`DateTime::from_unix`] — same era arithmetic, run
/// the other way, with no lookup tables and no validity check on the date
/// handed in.
///
/// This is the form a *calendar day* is compared in. Two dates are the same
/// day if this returns the same number for both, which a broken-out
/// year/month/day cannot answer without knowing how long each month is, and
/// "the day after" is this plus one, which a broken-out date cannot answer
/// at all without knowing about leap years.
pub fn days_from_civil(year: i64, month: i64, day: i64) -> i64 {
    // March-based year, so the leap day lands at the end and the
    // month-length pattern is regular.
    let year = if month <= 2 { year - 1 } else { year };
    // `div_euclid` and not `/`: the eras before 1970 are negative, and
    // integer division truncating toward zero would put every date before
    // 0000-03-01 in the wrong one.
    let era = year.div_euclid(400);
    let year_of_era = year - era * 400;
    let month_prime = if month > 2 { month - 3 } else { month + 9 };
    let day_of_year = (153 * month_prime + 2) / 5 + day - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    era * 146_097 + day_of_era - 719_468
}

#[cfg(feature = "tz")]
pub use local::{LocalTime, utc_offset};

/// Local time, behind `tz`.
#[cfg(feature = "tz")]
mod local {
    use core::fmt;

    use tz::TimeZone;

    use super::DateTime;

    /// How far ahead of UTC `zone` was at `unix_seconds`, in seconds.
    ///
    /// Zero with no zone, and zero for an instant the zone has nothing to
    /// say about — the same UTC fallback [`LocalTime::from_unix`] leaves.
    ///
    /// Separate from [`LocalTime`] because the two answer different
    /// questions. That one converts an instant for display; this one is for
    /// arithmetic that has to know which *local day* an instant falls in,
    /// which a broken-out date cannot be asked after the fact.
    pub fn utc_offset(unix_seconds: u64, zone: Option<&TimeZone>) -> i64 {
        zone.and_then(|zone| zone.find_local_time_type(unix_seconds as i64).ok())
            .map_or(0, |local| i64::from(local.ut_offset()))
    }

    /// An instant as a particular zone reads it.
    ///
    /// Borrows the zone, because the abbreviation is a string inside it.
    /// That costs nothing where the zone is the `&'static` one read at boot.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct LocalTime<'z> {
        /// The calendar date and clock time in the zone.
        pub at: DateTime,
        /// What the zone calls the offset in force — `CST`, `CDT`, `+0530`.
        ///
        /// The zone's own abbreviation rather than one worked out here: the
        /// difference between `CST` and `CDT` is the only thing on a
        /// displayed time that says which side of a transition it is on,
        /// and inventing it from the offset would get every zone that does
        /// not use one wrong.
        pub abbreviation: &'z str,
    }

    impl<'z> LocalTime<'z> {
        /// Converts Unix seconds to the date and time `zone` reads them as.
        ///
        /// `None` if the zone has nothing to say about that instant, which
        /// a well-formed file does not: a TZif carries a POSIX rule for
        /// everything after its last tabulated transition, so a zone file
        /// years out of date still answers — with the rule that was current
        /// when it was written. That is the failure worth knowing about,
        /// and it is a wrong hour rather than a `None`.
        pub fn from_unix(unix_seconds: u64, zone: &'z TimeZone) -> Option<Self> {
            let local = zone.find_local_time_type(unix_seconds as i64).ok()?;
            Some(LocalTime {
                // Signed, because half the world is behind UTC, and checked
                // because an offset applied to an instant near the epoch is
                // the one case that goes below zero.
                at: DateTime::from_unix(
                    unix_seconds.checked_add_signed(i64::from(local.ut_offset()))?,
                ),
                abbreviation: local.time_zone_designation(),
            })
        }
    }

    /// `2026-09-28 09:05:09 CDT`.
    impl fmt::Display for LocalTime<'_> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            let at = &self.at;
            write!(
                f,
                "{:04}-{:02}-{:02} {:02}:{:02}:{:02} {}",
                at.year, at.month, at.day, at.hour, at.minute, at.second, self.abbreviation
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use alloc::format;

    use super::*;

    fn at(year: u64, month: u64, day: u64, hour: u64, minute: u64, second: u64) -> DateTime {
        DateTime {
            year,
            month,
            day,
            hour,
            minute,
            second,
        }
    }

    #[test]
    fn known_instants() {
        assert_eq!(DateTime::from_unix(0), at(1970, 1, 1, 0, 0, 0));
        // FAT's epoch, and the start of what it can store.
        assert_eq!(DateTime::from_unix(315_532_800), at(1980, 1, 1, 0, 0, 0));
        // A leap day, and the second before the next day.
        assert_eq!(DateTime::from_unix(951_782_400), at(2000, 2, 29, 0, 0, 0));
        assert_eq!(
            DateTime::from_unix(951_868_799),
            at(2000, 2, 29, 23, 59, 59)
        );
        // i32 overflow.
        assert_eq!(
            DateTime::from_unix(2_147_483_648),
            at(2038, 1, 19, 3, 14, 8)
        );
        // The last second FAT can store.
        assert_eq!(
            DateTime::from_unix(4_354_819_199),
            at(2107, 12, 31, 23, 59, 59)
        );
    }

    #[test]
    fn century_years_are_not_leap_unless_divisible_by_400() {
        // 2100-02-28 is followed by 2100-03-01.
        let feb_28 = days_from_civil(2100, 2, 28);
        assert_eq!(
            DateTime::from_unix((feb_28 as u64 + 1) * 86_400),
            at(2100, 3, 1, 0, 0, 0)
        );
        // 2000-02-28 is followed by 2000-02-29.
        let feb_28 = days_from_civil(2000, 2, 28);
        assert_eq!(
            DateTime::from_unix((feb_28 as u64 + 1) * 86_400),
            at(2000, 2, 29, 0, 0, 0)
        );
    }

    /// Every day from 1970 to FAT's last one, both directions: the two
    /// halves are each other's inverse, which is the property everything
    /// else relies on.
    #[test]
    fn days_from_civil_inverts_from_unix() {
        let last = days_from_civil(2107, 12, 31);
        for day in 0..=last {
            let date = DateTime::from_unix(day as u64 * 86_400);
            assert_eq!(date.epoch_day(), day, "{date}");
        }
    }

    #[test]
    fn days_before_1970_are_negative() {
        assert_eq!(days_from_civil(1969, 12, 31), -1);
        assert_eq!(days_from_civil(1900, 1, 1), -25_567);
        assert_eq!(days_from_civil(0, 3, 1), -719_468);
    }

    #[test]
    fn display() {
        assert_eq!(
            format!("{}", DateTime::from_unix(1_790_000_000)),
            "2026-09-21 14:13:20 UTC"
        );
    }

    /// The only test that touches the stored datum, since it is one static
    /// shared by the whole process and tests run in parallel.
    #[test]
    fn set_and_read_back() {
        use embassy_time::{Duration, MockDriver};

        let driver = MockDriver::get();
        driver.advance(Duration::from_secs(10));
        assert_eq!(now_unix_millis(), None);
        assert_eq!(now(), None);

        set(1_790_000_000_123);
        assert_eq!(now_unix_millis(), Some(1_790_000_000_123));
        assert_eq!(now_unix(), Some(1_790_000_000));

        // The clock runs with the monotonic counter, not with calls to `set`.
        driver.advance(Duration::from_millis(1_500));
        assert_eq!(now_unix_millis(), Some(1_790_000_001_623));

        // A re-sync moves the datum.
        set(1_800_000_000_000);
        assert_eq!(now_unix(), Some(1_800_000_000));

        // A time before the board's own uptime clamps to the epoch.
        set(5_000);
        assert_eq!(now_unix(), Some(11));
    }

    #[cfg(feature = "tz")]
    #[test]
    fn local_time_applies_the_offset() {
        let zone = tz::TimeZone::fixed(-5 * 3600).unwrap();
        let local = LocalTime::from_unix(1_790_000_000, &zone).unwrap();
        assert_eq!(local.at, at(2026, 9, 21, 9, 13, 20));
        assert_eq!(utc_offset(1_790_000_000, Some(&zone)), -5 * 3600);
        assert_eq!(utc_offset(1_790_000_000, None), 0);
        // Behind UTC at the epoch goes below zero.
        assert!(LocalTime::from_unix(0, &zone).is_none());
    }
}
