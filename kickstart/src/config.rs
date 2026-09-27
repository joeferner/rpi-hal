//! Reading a board's settings file.
//!
//! The file is TOML and the schema is the application's own struct: serde's
//! derives do the parsing, and there is no trait to implement and no key
//! table to keep in step with the struct. What this module adds is the
//! part TOML cannot do and the part every board otherwise writes twice —
//! turning a byte offset into `BOARD.TOML:12:18`, and checking that a
//! string which TOML typed as a string is also a *hostname*.
//!
//! ```ignore
//! use rpi_kickstart::config::{self, value, At, Problem, Spanned};
//!
//! /// Everything this board reads from `/BOARD.TOML`.
//! #[derive(serde::Deserialize)]
//! struct Settings {
//!     hostname: Spanned<String>,
//!     ntp_resync_interval: Spanned<String>,   // "6h" -- TOML has no duration type
//!     #[serde(default)]
//!     zone: Vec<Zone>,                        // [[zone]]
//! }
//!
//! impl Settings {
//!     fn resync(&self) -> Result<Duration, Problem> {
//!         value::duration(self.ntp_resync_interval.as_ref()).at(&self.ntp_resync_interval)
//!     }
//! }
//!
//! let parsed = config::parse::<Settings>(text)?;
//! for key in &parsed.unknown {
//!     logln!("BOARD.TOML: ignoring unknown key `{key}`");
//! }
//! if let Err(problem) = parsed.settings.resync() {
//!     logln!("{}", problem.report("BOARD.TOML", text));
//! }
//! ```
//!
//! # Two halves, and why they are separate
//!
//! [`parse`](crate::config::parse) is the syntactic half: TOML's grammar
//! and the schema's types, so a `true` where a number belongs is refused
//! here. The semantic half is [`value`](crate::config::value), run by the
//! application after parsing — `"water sensor"` is a perfectly good TOML
//! string and a useless mDNS name, and only the board knows which of its
//! strings are names. Both produce the same
//! [`Problem`](crate::config::Problem), located the same way.
//!
//! The semantic checks are deliberately not wired into deserialization
//! (`#[serde(deserialize_with)]` could do it). Run at boot, once, they turn
//! a bad value into a console line pointing at the character that is
//! wrong; hidden inside a derive, they would turn it into a type error
//! whose message is serde's rather than the setting's.
//!
//! # Absent is a state; malformed is a mistake
//!
//! An empty file parses to the schema's `#[serde(default)]`s — and is an
//! error naming the first field that has none — so a board whose card has
//! no settings file boots on them — reading the file, and deciding that a
//! missing one is empty, is the caller's. A file that exists and does not
//! parse is an `Err`, and what to do about it is the caller's too: one
//! board halts, another boots on defaults, and neither policy belongs in a
//! library.
//!
//! # Unknown keys are reported and skipped
//!
//! A key the schema does not name is collected in
//! [`Parsed::unknown`](crate::config::Parsed::unknown)
//! rather than refused, so a card written for newer firmware still boots
//! older firmware. They are returned rather than logged because this
//! module needs no console: it is pure, which is what lets it be tested on
//! the host.
//!
//! A schema that would rather refuse them says so with
//! `#[serde(deny_unknown_fields)]`, and the refusal comes back as a
//! located [`Problem`](crate::config::Problem) like any other.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt;
use core::ops::Range;

use serde::de::DeserializeOwned;

/// A value with the byte range it was read from, so a check that fails
/// after parsing can still point at the text.
///
/// Re-exported from the `toml` crate so a schema can name it without
/// depending on `toml` itself. Deref to the inner value, or `as_ref()` /
/// `into_inner()`.
pub use toml::Spanned;

/// What [`parse`] produced from a file that was valid.
#[derive(Debug)]
pub struct Parsed<C> {
    /// The settings, as the schema describes them.
    pub settings: C,
    /// Every key in the file the schema does not name, as a dotted path —
    /// `zone.1.nmae` for a misspelt field in the second `[[zone]]`, since
    /// array positions count from zero.
    ///
    /// These have been skipped, not refused; see the module documentation
    /// for why.
    pub unknown: Vec<String>,
}

/// Parses `text` as TOML into the schema `C`.
///
/// Bytes rather than `&str` because that is what comes off a card, and a
/// file that is not UTF-8 is one more way for it to be malformed: it is
/// reported like any other, located at the first byte that is not.
pub fn parse<C: DeserializeOwned>(text: &[u8]) -> Result<Parsed<C>, Problem> {
    let text = core::str::from_utf8(text).map_err(|e| {
        let at = e.valid_up_to();
        Problem::new(at..at + 1, "not valid UTF-8")
    })?;
    let document = toml::de::Deserializer::parse(text).map_err(Problem::from_toml)?;
    let mut unknown = Vec::new();
    let settings = serde_ignored::deserialize(document, |path| unknown.push(path.to_string()))
        .map_err(Problem::from_toml)?;
    Ok(Parsed { settings, unknown })
}

/// What [`load`] read off the card.
#[cfg(feature = "storage")]
#[derive(Debug)]
pub struct Loaded<C> {
    /// The settings, and the keys the schema did not name.
    pub parsed: Parsed<C>,
    /// The file as it was on the card, or `None` if there was no file and
    /// the settings are the schema's defaults.
    ///
    /// Kept because a [`Problem`] found after loading — by a check in
    /// [`value`] — needs the text back to say which line it is on.
    pub text: Option<Vec<u8>>,
}

/// Reads and parses the settings file at `path` on `volume`.
///
/// **Absent is a state.** A missing file parses as empty text, so a
/// schema with `#[serde(default)]`s boots on its defaults, and
/// [`Loaded::text`] is `None` so the caller can say so. A file that exists
/// and does not parse is an [`LoadError::Invalid`], and what to do about
/// it — halt, or boot on defaults — is the caller's decision.
///
/// `path` is `/`-separated and matched case-insensitively against long
/// and 8.3 names alike, so `kickstart.toml` finds the file whichever
/// machine wrote the card.
#[cfg(feature = "storage")]
pub fn load<C, D>(
    volume: &mut crate::storage::Volume<D>,
    path: &str,
) -> Result<Loaded<C>, LoadError<D::Error>>
where
    C: DeserializeOwned,
    D: resident_fat::BlockDevice,
{
    let text = match volume.open(path) {
        Ok(file) => Some(volume.read_all(&file).map_err(LoadError::Storage)?),
        Err(resident_fat::Error::NotFound { .. }) => None,
        Err(error) => return Err(LoadError::Storage(error)),
    };
    match parse(text.as_deref().unwrap_or_default()) {
        Ok(parsed) => Ok(Loaded { parsed, text }),
        Err(problem) => Err(LoadError::Invalid {
            path: path.to_string(),
            text: text.unwrap_or_default(),
            problem,
        }),
    }
}

/// Why [`load`] produced no settings.
#[cfg(feature = "storage")]
#[derive(Debug)]
pub enum LoadError<E> {
    /// The card could not be read.
    Storage(resident_fat::Error<E>),
    /// The file was read and is not valid for the schema.
    ///
    /// Carries the path and the text so that printing it gives the
    /// located form, `kickstart.toml:3:12: …`, with nothing else to keep
    /// hold of.
    Invalid {
        /// The path the file was loaded from, as given to [`load`].
        path: String,
        /// The file's contents, which the problem's position is in.
        text: Vec<u8>,
        /// What is wrong, and where.
        problem: Problem,
    },
}

/// One line, for the console: the located problem, or the path and the
/// storage error.
#[cfg(feature = "storage")]
impl<E: fmt::Debug> fmt::Display for LoadError<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LoadError::Storage(error) => write!(f, "{error}"),
            LoadError::Invalid {
                path,
                text,
                problem,
            } => write!(f, "{}", problem.report(path, text)),
        }
    }
}

/// Something wrong with the file, and where.
///
/// Carries a byte range rather than a line and column, because the checks
/// in [`value`] run on a [`Spanned`] long after the text was parsed and
/// have only the range to go on. [`report`](Self::report) turns it into
/// `file:line:column` given the text back.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Problem {
    span: Option<Range<usize>>,
    message: String,
}

impl Problem {
    /// A problem with the text in `span`, described by `message`.
    ///
    /// For an application's own checks, where [`At::at`] on an [`Invalid`]
    /// does not fit — a pair of settings that must agree, say.
    pub fn new(span: Range<usize>, message: impl Into<String>) -> Self {
        Problem {
            span: Some(span),
            message: message.into(),
        }
    }

    /// Converts `toml`'s error, keeping its span.
    ///
    /// The message is `toml`'s own and is already written for a person
    /// ("invalid type: integer `5`, expected a string", "missing field
    /// `hostname`"). What is dropped is its multi-line rendering with a
    /// caret under the column, which is written for a terminal and would be
    /// four console lines per mistake.
    fn from_toml(error: toml::de::Error) -> Self {
        Problem {
            span: error.span(),
            message: error.message().to_string(),
        }
    }

    /// The byte range of the offending text, if the error has one.
    ///
    /// `toml` gives one for every error it raises from a document, but a
    /// schema's own `impl Deserialize` can raise one it has no position
    /// for.
    pub fn span(&self) -> Option<Range<usize>> {
        self.span.clone()
    }

    /// What is wrong, without the location.
    pub fn message(&self) -> &str {
        &self.message
    }

    /// Where the problem starts in `text`, which must be the text it was
    /// found in.
    pub fn locate(&self, text: &[u8]) -> Option<Location> {
        self.span
            .as_ref()
            .map(|span| Location::of(text, span.start))
    }

    /// The problem as one line, `file:line:column: message` — the form
    /// editors and terminals already know how to jump to.
    ///
    /// `text` must be the text the problem was found in; the location is
    /// only computed from it here, when there is something to print.
    pub fn report<'a>(&'a self, file: &'a str, text: &'a [u8]) -> Report<'a> {
        Report {
            problem: self,
            file,
            text,
        }
    }
}

/// The message alone. See [`Problem::report`] for the located form.
impl fmt::Display for Problem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl core::error::Error for Problem {}

/// A position in a file, as a person counts it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Location {
    /// Line number, from 1.
    pub line: usize,
    /// Column, from 1, in characters rather than bytes — so a line
    /// containing `°` is counted the way an editor shows it.
    pub column: usize,
}

impl Location {
    /// The location of byte `offset` in `text`.
    ///
    /// Columns count the bytes that start a UTF-8 character, which is a
    /// character count for valid text and still something sensible for
    /// text that is not — which is one of the problems this may be asked
    /// to locate.
    fn of(text: &[u8], offset: usize) -> Self {
        let before = &text[..offset.min(text.len())];
        let line_start = before
            .iter()
            .rposition(|&b| b == b'\n')
            .map_or(0, |i| i + 1);
        Location {
            line: before.iter().filter(|&&b| b == b'\n').count() + 1,
            column: before[line_start..]
                .iter()
                .filter(|&&b| b & 0xC0 != 0x80)
                .count()
                + 1,
        }
    }
}

/// A [`Problem`] with its file and text, for printing. See
/// [`Problem::report`].
#[derive(Debug)]
pub struct Report<'a> {
    problem: &'a Problem,
    file: &'a str,
    text: &'a [u8],
}

/// `file:line:column: message`, or `file: message` for a problem with no
/// position.
impl fmt::Display for Report<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.problem.locate(self.text) {
            Some(Location { line, column }) => {
                write!(f, "{}:{line}:{column}: {}", self.file, self.problem)
            }
            None => write!(f, "{}: {}", self.file, self.problem),
        }
    }
}

/// Why a value failed one of the checks in [`value`]: what was expected
/// instead.
///
/// Carries no position — the checks take a plain `&str` so they are usable
/// on anything — and [`At::at`] is what attaches one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Invalid {
    expected: &'static str,
}

impl Invalid {
    /// A failed check, described by what a valid value looks like:
    /// `Invalid::expected("a threshold between 1 and 4096000 µV")`.
    ///
    /// Phrased as the expectation rather than the fault because it is
    /// printed as `expected …`, and because what somebody fixing the file
    /// needs is what to write instead.
    pub const fn expected(expected: &'static str) -> Self {
        Invalid { expected }
    }

    /// What was expected, as given to [`expected`](Self::expected).
    pub fn expectation(&self) -> &'static str {
        self.expected
    }
}

/// `expected …`.
impl fmt::Display for Invalid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "expected {}", self.expected)
    }
}

/// Locates a failed check at the setting it was run on.
///
/// A trait only so that it reads as a method at the call site —
/// `value::label(name.as_ref()).at(&name)?` — which is the shape a
/// schema's `validate` is written in.
pub trait At<T> {
    /// Attaches `setting`'s span to a failure, turning it into a
    /// [`Problem`] that can say which line and column is wrong.
    fn at<S>(self, setting: &Spanned<S>) -> Result<T, Problem>;
}

impl<T> At<T> for Result<T, Invalid> {
    fn at<S>(self, setting: &Spanned<S>) -> Result<T, Problem> {
        self.map_err(|invalid| Problem::new(setting.span(), format!("{invalid}")))
    }
}

/// The semantic checks: what TOML cannot say about a string.
///
/// Each takes the value as TOML handed it over and returns it — or the
/// part of it that matters — if it is acceptable. None of them trims:
/// TOML strings are quoted, so leading or trailing whitespace in one was
/// put there on purpose and is almost certainly a mistake to report
/// rather than repair.
///
/// `bool` and whole numbers are not here. TOML types those itself, and
/// a range check on a number is one comparison the application can write
/// with an [`Invalid`] of its own.
pub mod value {
    use core::time::Duration;

    use super::Invalid;

    /// Longest DNS label there is, in bytes. The length is carried in one
    /// byte whose top two bits are reserved for compression pointers, so 63
    /// is the wire format's own limit rather than a choice made here.
    const LABEL_MAX: usize = 63;

    /// A whole number of seconds, minutes (`m`) or hours (`h`), with a bare
    /// number or an `s` suffix meaning seconds: `"30s"`, `"5m"`, `"6h"`.
    ///
    /// A string because TOML has no duration type, and a bare integer would
    /// leave the unit to be remembered.
    ///
    /// Zero is refused. Every duration a board reads paces a retry or a
    /// re-sync loop, and a zero-length wait turns one into a spin that
    /// hammers the network and starves whatever else the executor has to
    /// run.
    pub fn duration(v: &str) -> Result<Duration, Invalid> {
        const EXPECTED: Invalid =
            Invalid::expected("a duration: a whole number above zero, followed by `s`, `m` or `h`");
        let (digits, multiplier) = match v.as_bytes().last() {
            Some(b's') => (&v[..v.len() - 1], 1),
            Some(b'm') => (&v[..v.len() - 1], 60),
            Some(b'h') => (&v[..v.len() - 1], 60 * 60),
            _ => (v, 1),
        };
        // `u64::from_str` accepts a leading `+`, which nobody means here.
        if !digits.bytes().all(|b| b.is_ascii_digit()) {
            return Err(EXPECTED);
        }
        let count: u64 = digits.parse().map_err(|_| EXPECTED)?;
        if count == 0 {
            return Err(EXPECTED);
        }
        Ok(Duration::from_secs(
            count.checked_mul(multiplier).ok_or(EXPECTED)?,
        ))
    }

    /// A name this board will *answer* to over mDNS: letters, digits and
    /// hyphens, with the hyphens somewhere other than the ends, and at most
    /// 63 bytes. Returns the label.
    ///
    /// A trailing `.local` is accepted and dropped, case-insensitively.
    /// Someone typing a name into a form is far more likely to type the
    /// one they will use in a browser than the label underneath it, and
    /// answering to `water-sensor.local.local` because of it would be a
    /// puzzle with no clue attached.
    ///
    /// Stricter than [`host`], and for the opposite reason. That one reads
    /// a name the board is going to *ask* a resolver about, where the
    /// resolver decides what is real; this one reads a name the board is
    /// going to answer to, so anything a querier cannot ask for is a name
    /// that silently never resolves. RFC 6763 allows very nearly any UTF-8
    /// in a label, but this name is typed into address bars and resolved
    /// by three operating systems' resolvers, and the ones that normalize
    /// or refuse the interesting characters do it silently — so the rule
    /// is RFC 1123's host-name one.
    pub fn label(v: &str) -> Result<&str, Invalid> {
        const EXPECTED: Invalid = Invalid::expected(
            "a name of 1 to 63 letters, digits and hyphens, not starting or ending with a hyphen",
        );
        let bytes = v.as_bytes();
        let label = match bytes.len().checked_sub(".local".len()) {
            Some(at) if bytes[at..].eq_ignore_ascii_case(b".local") => &v[..at],
            _ => v,
        };
        if label.is_empty() || label.len() > LABEL_MAX {
            return Err(EXPECTED);
        }
        if label.starts_with('-') || label.ends_with('-') {
            return Err(EXPECTED);
        }
        if !label
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'-')
        {
            return Err(EXPECTED);
        }
        Ok(label)
    }

    /// A host name, or an address written out: a non-empty run of
    /// printable, non-space ASCII.
    ///
    /// Not a check against the DNS grammar. What decides whether a name is
    /// real is the resolution, which reports its own failure; this only
    /// refuses what could not be a name at all, and in particular what
    /// could not travel in a DNS question.
    pub fn host(v: &str) -> Result<&str, Invalid> {
        graphic(v).ok_or(Invalid::expected("a host name or address, with no spaces"))
    }

    /// A value carried verbatim into a URL path or an HTTP header — a
    /// topic, a token: non-empty, printable ASCII with no spaces.
    ///
    /// The no-spaces rule is not cosmetic. A value with a newline in it
    /// would end the header it was written into and begin a new one, which
    /// is header injection from the settings file; refusing anything
    /// non-graphic makes that unrepresentable rather than something to
    /// strip later.
    pub fn word(v: &str) -> Result<&str, Invalid> {
        graphic(v).ok_or(Invalid::expected("printable ASCII with no spaces"))
    }

    /// The parts of an `https://host[:port][/path]` URL.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Url<'a> {
        /// The host, as written: a name or an address.
        pub host: &'a str,
        /// The port, or 443 if none was written.
        pub port: u16,
        /// The path, with any trailing `/` removed, or empty. A base URL is
        /// usually written with a trailing slash, and joining one to a
        /// topic would otherwise produce a double slash and a 404.
        pub path: &'a str,
    }

    /// An `https://host[:port][/path]` URL.
    ///
    /// **`https://` is required.** A setting that asks for plain HTTP would
    /// send whatever it carries — a message, a bearer token — in clear
    /// text, and would quietly discard the reason a board carries a TLS
    /// stack; it is refused rather than honoured.
    ///
    /// Not a general URL parser: no userinfo, no query, and no bracketed
    /// IPv6 literal, since the port is taken as the text after the last
    /// colon. What a board talks to is a named host.
    pub fn url(v: &str) -> Result<Url<'_>, Invalid> {
        const EXPECTED: Invalid = Invalid::expected("a URL starting `https://`");
        let rest = v.strip_prefix("https://").ok_or(EXPECTED)?;
        let (authority, path) = match rest.find('/') {
            Some(at) => rest.split_at(at),
            None => (rest, ""),
        };
        let path = path.trim_end_matches('/');
        let (host, port) = match authority.rsplit_once(':') {
            Some((host, port)) => (host, port.parse().map_err(|_| EXPECTED)?),
            None => (authority, 443),
        };
        let host = graphic(host).ok_or(EXPECTED)?;
        if !path.bytes().all(|b| b.is_ascii_graphic()) {
            return Err(EXPECTED);
        }
        Ok(Url { host, port, path })
    }

    /// An email address, or a comma-separated list of them. Returns the
    /// value as written; split it on `,` and trim to get the addresses.
    ///
    /// Deliberately shallow: every address must have an `@` with something
    /// on either side and no whitespace, and that is all. Validating email
    /// addresses properly is famously not worth attempting, and the
    /// authority on whether one works is the send attempt — which reports
    /// its own failure.
    pub fn address(v: &str) -> Result<&str, Invalid> {
        const EXPECTED: Invalid =
            Invalid::expected("an email address, or a comma-separated list of them");
        if v.is_empty() {
            return Err(EXPECTED);
        }
        for address in v.split(',') {
            let address = address.trim();
            let (local, domain) = address.split_once('@').ok_or(EXPECTED)?;
            if local.is_empty() || domain.is_empty() || graphic(address).is_none() {
                return Err(EXPECTED);
            }
        }
        Ok(v)
    }

    /// A coordinate in decimal degrees, within `limit` of zero: 90 for a
    /// latitude, 180 for a longitude. North and east are positive.
    ///
    /// Decimal degrees and nothing else — no `41°52'41"N`, no trailing
    /// hemisphere letter — since that is what every map application hands
    /// out when asked where something is. A number rather than a string,
    /// because TOML has floats — and a whole `41` reads into an `f64` field
    /// as well as `41.0` does, so nobody has to know TOML tells them apart.
    ///
    /// Refused rather than wrapped when out of range: a longitude of 200 is
    /// a typo, and the place it would wrap to is a plausible-looking answer
    /// half a world from the one that was meant. NaN fails the range test
    /// too, so there is nothing extra to check for it.
    pub fn degrees(v: f64, limit: f64) -> Result<f64, Invalid> {
        if (-limit..=limit).contains(&v) {
            Ok(v)
        } else {
            Err(Invalid::expected("decimal degrees within range"))
        }
    }

    /// Longest SSID 802.11 allows, in bytes.
    const SSID_MAX: usize = 32;

    /// A Wi-Fi network name: 1 to 32 bytes, with no control characters.
    ///
    /// 802.11 treats an SSID as opaque bytes, so this is narrower than the
    /// standard allows. It has to be: the radio driver takes a `&str`, and
    /// a name that cannot be written in a text file is not one anybody
    /// could have put in this one.
    pub fn ssid(v: &str) -> Result<&str, Invalid> {
        if v.is_empty() || v.len() > SSID_MAX || v.chars().any(char::is_control) {
            return Err(Invalid::expected(
                "a network name of 1 to 32 bytes, with no control characters",
            ));
        }
        Ok(v)
    }

    /// A WPA2-PSK passphrase: 8 to 63 characters of printable ASCII,
    /// spaces included, which is what 802.11i defines one to be.
    ///
    /// Refused here rather than at the join because of what a bad one
    /// looks like there: the radio associates, the handshake fails, and
    /// the console says only that the join did not complete. Everything
    /// this rejects is a passphrase no access point could have been set
    /// to.
    pub fn passphrase(v: &str) -> Result<&str, Invalid> {
        if !(8..=63).contains(&v.len()) || !v.bytes().all(|b| b.is_ascii_graphic() || b == b' ') {
            return Err(Invalid::expected(
                "a WPA2 passphrase of 8 to 63 printable ASCII characters",
            ));
        }
        Ok(v)
    }

    /// `v` if it is non-empty printable ASCII with no spaces — the rule
    /// [`host`] and [`word`] share for different reasons.
    fn graphic(v: &str) -> Option<&str> {
        (!v.is_empty() && v.bytes().all(|b| b.is_ascii_graphic())).then_some(v)
    }
}

#[cfg(test)]
mod tests {
    use alloc::string::String;
    use alloc::vec;
    use alloc::vec::Vec;
    use core::time::Duration;

    use super::value::{self, Url};
    use super::*;

    #[derive(Debug, serde::Deserialize)]
    struct Settings {
        hostname: Spanned<String>,
        #[serde(default)]
        ntp_resync_interval: Option<Spanned<String>>,
        #[serde(default)]
        zone: Vec<Zone>,
    }

    #[derive(Debug, serde::Deserialize)]
    struct Zone {
        name: String,
        enabled: bool,
        presence_uv: u32,
    }

    #[derive(Debug, Default, serde::Deserialize)]
    #[serde(default)]
    struct Defaults {
        hostname: String,
        port: u16,
    }

    fn report(text: &str, problem: &Problem) -> String {
        format!("{}", problem.report("BOARD.TOML", text.as_bytes()))
    }

    const GOOD: &str = r#"
hostname = "water-sensor"
ntp_resync_interval = "6h"

[[zone]]
name = "Water heater"
enabled = true
presence_uv = 400000

[[zone]]
name = "Sump"
enabled = false
presence_uv = 350000
"#;

    #[test]
    fn parses_a_schema_with_tables() {
        let parsed = parse::<Settings>(GOOD.as_bytes()).unwrap();
        let s = parsed.settings;
        assert_eq!(s.hostname.as_ref(), "water-sensor");
        assert_eq!(s.zone.len(), 2);
        assert_eq!(s.zone[0].name, "Water heater");
        assert!(s.zone[0].enabled);
        assert_eq!(s.zone[1].presence_uv, 350_000);
        assert!(parsed.unknown.is_empty());
        let resync = s.ntp_resync_interval.unwrap();
        assert_eq!(
            value::duration(resync.as_ref()),
            Ok(Duration::from_secs(21_600))
        );
    }

    #[test]
    fn spans_point_at_the_value() {
        let s = parse::<Settings>(GOOD.as_bytes()).unwrap().settings;
        assert_eq!(&GOOD[s.hostname.span()], "\"water-sensor\"");
    }

    #[test]
    fn an_empty_file_is_the_defaults() {
        let parsed = parse::<Defaults>(b"").unwrap();
        assert_eq!(parsed.settings.hostname, "");
        assert_eq!(parsed.settings.port, 0);
    }

    #[test]
    fn unknown_keys_are_collected_not_refused() {
        let text = "hostname = \"a\"\nfuture = 1\n[[zone]]\nname = \"x\"\nenabled = true\npresence_uv = 1\nnmae = \"y\"\n";
        let parsed = parse::<Settings>(text.as_bytes()).unwrap();
        assert_eq!(parsed.unknown, vec!["future", "zone.0.nmae"]);
        assert_eq!(parsed.settings.hostname.as_ref(), "a");
    }

    #[test]
    fn a_syntax_error_is_located() {
        let text = "hostname = \"a\"\nbroken = \n";
        let problem = parse::<Settings>(text.as_bytes()).unwrap_err();
        assert!(report(text, &problem).starts_with("BOARD.TOML:2:"));
    }

    #[test]
    fn a_type_error_is_located_at_the_value() {
        let text =
            "hostname = \"a\"\n\n[[zone]]\nname = \"x\"\nenabled = \"yes\"\npresence_uv = 1\n";
        let problem = parse::<Settings>(text.as_bytes()).unwrap_err();
        let line = report(text, &problem);
        assert!(line.starts_with("BOARD.TOML:5:11: "), "{line}");
        assert!(line.contains("bool"), "{line}");
    }

    #[test]
    fn a_missing_field_is_an_error() {
        let problem = parse::<Settings>(b"ntp_resync_interval = \"6h\"\n").unwrap_err();
        assert!(problem.message().contains("hostname"), "{problem}");
    }

    #[test]
    fn invalid_utf8_is_located_at_the_first_bad_byte() {
        let text = b"hostname = \"ab\xffc\"\n";
        let problem = parse::<Settings>(text).unwrap_err();
        assert_eq!(problem.span(), Some(14..15));
        assert_eq!(
            problem.locate(text),
            Some(Location {
                line: 1,
                column: 15
            })
        );
    }

    #[test]
    fn an_integer_is_not_a_float() {
        #[derive(Debug, serde::Deserialize)]
        #[allow(dead_code)]
        struct Place {
            latitude: f64,
        }
        let place = |text: &str| parse::<Place>(text.as_bytes()).unwrap().settings.latitude;
        assert_eq!(place("latitude = 41.5"), 41.5);
        assert_eq!(place("latitude = 41"), 41.0);
    }

    #[test]
    fn denying_unknown_fields_is_the_schemas_choice() {
        #[derive(Debug, serde::Deserialize)]
        #[serde(deny_unknown_fields)]
        #[allow(dead_code)]
        struct Strict {
            a: u32,
        }
        let text = "a = 1\nb = 2\n";
        let problem = parse::<Strict>(text.as_bytes()).unwrap_err();
        assert!(report(text, &problem).starts_with("BOARD.TOML:2:1: "));
    }

    #[test]
    fn a_failed_check_is_located_at_the_setting() {
        let text = "hostname = \"water sensor\"\n";
        let s = parse::<Settings>(text.as_bytes()).unwrap().settings;
        let problem = value::label(s.hostname.as_ref())
            .at(&s.hostname)
            .unwrap_err();
        assert_eq!(
            report(text, &problem),
            "BOARD.TOML:1:12: expected a name of 1 to 63 letters, digits and hyphens, \
             not starting or ending with a hyphen"
        );
    }

    #[test]
    fn columns_count_characters() {
        let text = "# 20°C\nhostname = \"é x\"\n";
        let s = parse::<Settings>(text.as_bytes()).unwrap().settings;
        let problem = value::host(s.hostname.as_ref())
            .at(&s.hostname)
            .unwrap_err();
        assert_eq!(
            problem.locate(text.as_bytes()),
            Some(Location {
                line: 2,
                column: 12
            })
        );
        let text = "a = \"°\" ; b = \"x\"";
        assert_eq!(
            Location::of(text.as_bytes(), text.find(';').unwrap()),
            Location { line: 1, column: 9 }
        );
    }

    #[test]
    fn a_problem_without_a_span_names_only_the_file() {
        let problem = Problem {
            span: None,
            message: "nope".into(),
        };
        assert_eq!(report("", &problem), "BOARD.TOML: nope");
    }

    #[test]
    fn duration() {
        assert_eq!(value::duration("30"), Ok(Duration::from_secs(30)));
        assert_eq!(value::duration("30s"), Ok(Duration::from_secs(30)));
        assert_eq!(value::duration("5m"), Ok(Duration::from_secs(300)));
        assert_eq!(value::duration("6h"), Ok(Duration::from_secs(21_600)));
        for bad in ["", "0", "0h", "h", "+5m", "-5", "5 m", "5d", "1.5h", "5M"] {
            assert!(value::duration(bad).is_err(), "{bad:?}");
        }
        assert!(value::duration("18446744073709551615h").is_err());
    }

    #[test]
    fn label() {
        assert_eq!(value::label("water-sensor"), Ok("water-sensor"));
        assert_eq!(value::label("Water-Sensor.Local"), Ok("Water-Sensor"));
        assert_eq!(value::label("a"), Ok("a"));
        let longest = "a".repeat(63);
        assert_eq!(value::label(&longest), Ok(longest.as_str()));
        for bad in ["", ".local", "-a", "a-", "a b", "a.b", "a_b", "café"] {
            assert!(value::label(bad).is_err(), "{bad:?}");
        }
        assert!(value::label(&"a".repeat(64)).is_err());
    }

    #[test]
    fn host_and_word() {
        assert_eq!(value::host("pool.ntp.org"), Ok("pool.ntp.org"));
        assert_eq!(value::host("192.168.1.1"), Ok("192.168.1.1"));
        assert_eq!(value::word("tk_abc-123"), Ok("tk_abc-123"));
        for bad in ["", "a b", " a", "a\n", "é"] {
            assert!(value::host(bad).is_err(), "{bad:?}");
            assert!(value::word(bad).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn url() {
        assert_eq!(
            value::url("https://ntfy.sh"),
            Ok(Url {
                host: "ntfy.sh",
                port: 443,
                path: ""
            })
        );
        assert_eq!(
            value::url("https://example.com:8443/base/"),
            Ok(Url {
                host: "example.com",
                port: 8443,
                path: "/base"
            })
        );
        for bad in [
            "http://ntfy.sh",
            "https://",
            "https://:443",
            "https://a:port",
            "https://a:70000",
            "https://a b",
            "https://a/b c",
            "ntfy.sh",
        ] {
            assert!(value::url(bad).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn address() {
        assert_eq!(value::address("a@b.c"), Ok("a@b.c"));
        assert_eq!(value::address("a@b.c, d@e.f"), Ok("a@b.c, d@e.f"));
        for bad in ["", "a", "@b", "a@", "a@b,", "a b@c"] {
            assert!(value::address(bad).is_err(), "{bad:?}");
        }
    }

    /// The template in the repository has to parse, and pass the checks
    /// the `mdns` example runs, or the first thing somebody copies onto a
    /// card is a file the board refuses.
    #[test]
    fn the_example_settings_file_is_valid() {
        #[derive(Debug, serde::Deserialize)]
        struct Example {
            hostname: Option<Spanned<String>>,
            wifi: Option<Wifi>,
        }
        #[derive(Debug, serde::Deserialize)]
        struct Wifi {
            ssid: Spanned<String>,
            passphrase: Spanned<String>,
        }

        let text = include_bytes!("../kickstart.toml.example");
        let parsed = parse::<Example>(text).unwrap();
        assert!(parsed.unknown.is_empty(), "{:?}", parsed.unknown);
        let example = parsed.settings;
        assert_eq!(
            value::label(example.hostname.unwrap().as_ref()),
            Ok("kickstart")
        );
        let wifi = example.wifi.unwrap();
        assert!(value::ssid(wifi.ssid.as_ref()).is_ok());
        assert!(value::passphrase(wifi.passphrase.as_ref()).is_ok());
    }

    #[test]
    fn ssid() {
        assert_eq!(value::ssid("home"), Ok("home"));
        assert_eq!(value::ssid("Café Wi-Fi"), Ok("Café Wi-Fi"));
        assert_eq!(value::ssid(&"a".repeat(32)), Ok("a".repeat(32).as_str()));
        for bad in ["", "a\tb", "a\nb"] {
            assert!(value::ssid(bad).is_err(), "{bad:?}");
        }
        assert!(value::ssid(&"a".repeat(33)).is_err());
    }

    #[test]
    fn passphrase() {
        assert_eq!(value::passphrase("12345678"), Ok("12345678"));
        assert_eq!(
            value::passphrase("p@ss #word \"q\" \\b"),
            Ok("p@ss #word \"q\" \\b")
        );
        assert!(value::passphrase(&"a".repeat(63)).is_ok());
        for bad in ["", "1234567", "tab\there!", "café1234"] {
            assert!(value::passphrase(bad).is_err(), "{bad:?}");
        }
        assert!(value::passphrase(&"a".repeat(64)).is_err());
    }

    #[test]
    fn degrees() {
        assert_eq!(value::degrees(41.8781, 90.0), Ok(41.8781));
        assert_eq!(value::degrees(-180.0, 180.0), Ok(-180.0));
        assert!(value::degrees(90.1, 90.0).is_err());
        assert!(value::degrees(f64::NAN, 90.0).is_err());
        assert!(value::degrees(f64::INFINITY, 180.0).is_err());
    }
}
