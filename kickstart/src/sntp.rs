//! Setting the wall clock from an NTP server.
//!
//! SNTP (RFC 4330) is NTP's client half: one datagram out, one back, and the
//! server's transmit timestamp is the answer. The discipline loop that makes
//! a real NTP client — filtering several servers, steering a local
//! oscillator instead of stepping it — is what this deliberately is not. A
//! Pi's clock is a crystal-derived 1 MHz counter, so stepping it every few
//! hours is both simpler and more than good enough.
//!
//! One source for [`crate::clock`] among possible others: this calls
//! `clock::set` on every sync and nothing depends on it, so a board with an
//! RTC leaves the feature off.
//!
//! ```ignore
//! #[embassy_executor::task]
//! async fn sntp_task(stack: Stack<'static>) -> ! {
//!     sntp::run(stack, NtpConfig::default()).await
//! }
//! ```
//!
//! The packet handling is this module's own rather than a client crate's,
//! and it is forty lines: build 48 bytes, check four fields of the reply,
//! convert one timestamp. What a crate would add is a dependency and an
//! adapter layer between it and the stack.
//!
//! # What it protects against
//!
//! An off-path forgery, which for a plain UDP exchange is the realistic
//! attack: anything that knows an exchange is in flight can answer before
//! the real server does. So the reply has to come from the address the
//! request went to, and it has to echo the request's transmit timestamp — a
//! value this end picks and nobody off the path sees — in its originate
//! field. Neither is a defence against something on the path, and nothing
//! short of NTS would be. A board that must not be lied to about the time
//! needs a better source than this one; TLS in particular trusts it for
//! certificate validity.

use core::net::Ipv4Addr;
use core::time::Duration as StdDuration;

use embassy_net::dns::DnsQueryType;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpAddress, Stack};
use embassy_time::{Duration, Instant, Timer, with_timeout};

use crate::{clock, logln};

/// Where the clock comes from, and how often it is asked.
#[derive(Debug, Clone, Copy)]
pub struct NtpConfig<'a> {
    /// The server to ask: a name to resolve, or an IPv4 address written
    /// out, which needs no resolver and is how a router doubling as a time
    /// server is usually named. Default `pool.ntp.org` — a pool rather than
    /// a host, so a dead server resolves to a different one next time.
    pub server: &'a str,
    /// The longest to wait after a failed sync before trying again. Default
    /// 30 seconds.
    ///
    /// A cap rather than a fixed wait: the first retry comes after a second
    /// and each one after that doubles, up to this. A pool name resolves to
    /// a different volunteer server on each attempt, so one that does not
    /// answer is usually followed by one that does, and the board need not
    /// sit out the full interval clockless to find it.
    pub retry_interval: StdDuration,
    /// How long to wait after a successful one. Default 6 hours.
    ///
    /// The drift being corrected is a crystal's, so this is about bounding
    /// it rather than about accuracy, and asking a public server more often
    /// is discourteous rather than useful.
    pub resync_interval: StdDuration,
}

impl NtpConfig<'static> {
    /// The defaults, as a constant — so a board can start a `static` from
    /// them, which [`Default::default`] cannot do.
    pub const DEFAULT: Self = NtpConfig {
        server: "pool.ntp.org",
        retry_interval: StdDuration::from_secs(30),
        resync_interval: StdDuration::from_secs(6 * 60 * 60),
    };
}

/// [`NtpConfig::DEFAULT`].
impl Default for NtpConfig<'_> {
    fn default() -> Self {
        NtpConfig::DEFAULT
    }
}

/// The NTP port, at the server end.
const NTP_PORT: u16 = 123;

/// An SNTP packet, in bytes. Extension fields and an authenticator may
/// follow on a reply; this neither sends nor reads them.
const PACKET_LEN: usize = 48;

/// Receive buffer for a reply: [`PACKET_LEN`] plus room for the extension
/// fields of a server that appends them. Those are skipped, but a datagram
/// larger than the buffer is dropped whole rather than truncated, and
/// dropping the reply is worse than ignoring its tail.
const REPLY_MAX: usize = 256;

/// Datagrams the socket may hold queued in each direction. Two, and the
/// second one is the point: an exchange expects one reply, and the spare is
/// so a forged or stray datagram arriving first leaves room for the real
/// one behind it.
const QUEUE_DEPTH: usize = 2;

/// Seconds between the NTP epoch (1900-01-01) and the Unix one — 70 years
/// with 17 leap days.
const NTP_UNIX_OFFSET: u64 = 2_208_988_800;

/// How long to wait for the server name to resolve.
const DNS_TIMEOUT: Duration = Duration::from_secs(10);

/// How long to wait for a reply.
///
/// **Not optional.** A receive on a datagram socket waits forever, so
/// without a deadline a request that goes into a hole — a firewall eating
/// outbound port 123, a server that has stopped answering — parks the task
/// for good, and the retry that exists to recover from exactly that never
/// runs.
const REPLY_TIMEOUT: Duration = Duration::from_secs(10);

/// The wait before the first retry after a failure; each consecutive
/// failure doubles it, up to [`NtpConfig::retry_interval`].
///
/// Short is not discourteous here: a retry after a missing reply already
/// sits behind [`REPLY_TIMEOUT`], and is usually a different server's.
const FIRST_RETRY: Duration = Duration::from_secs(1);

/// Why a sync failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Error {
    /// The name could not be resolved, which usually means the lease
    /// supplied no usable resolver — worth telling apart from a server that
    /// is down.
    Dns,
    /// Resolution did not finish in time.
    DnsTimeout,
    /// The name resolved to nothing.
    NoAddress,
    /// The socket could not be bound, or the request could not be sent.
    Send,
    /// Nothing usable arrived before the deadline: carries how many
    /// datagrams did arrive and were ignored, each already logged, because
    /// "nothing came back" and "answers came back and were refused" are
    /// different faults that would otherwise read the same.
    Timeout {
        /// Datagrams received and not used.
        ignored: usize,
    },
}

/// The reason as it appears on the console.
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Dns => f.write_str("name did not resolve"),
            Error::DnsTimeout => f.write_str("name did not resolve in time"),
            Error::NoAddress => f.write_str("resolved to no address"),
            Error::Send => f.write_str("request could not be sent"),
            Error::Timeout { ignored: 0 } => f.write_str("no reply"),
            Error::Timeout { ignored } => write!(f, "{ignored} replies, none usable"),
        }
    }
}

/// Why a datagram that arrived was not used as the answer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Refusal {
    /// Shorter than a packet; carries its length.
    Short(usize),
    /// Not a server's reply: carries the mode it did have.
    NotAServerReply(u8),
    /// Leap indicator 3: the server says its own clock is unsynchronized.
    Unsynchronized,
    /// Stratum 0, a "kiss of death": carries the four-character code — a
    /// `RATE` asks the client to slow down, a `DENY` refuses it outright.
    KissOfDeath([u8; 4]),
    /// Stratum above 15, unsynchronized by definition; carries it.
    Stratum(u8),
    /// The originate timestamp is not the one sent: an answer to somebody
    /// else's request, a late answer to an earlier one of ours, or a
    /// forgery.
    NotOurRequest,
    /// The transmit timestamp is zero.
    NoTimestamp,
    /// The transmit timestamp is before 1970.
    Before1970,
}

impl core::fmt::Display for Refusal {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Refusal::Short(len) => write!(f, "{len} bytes, shorter than a packet"),
            Refusal::NotAServerReply(mode) => write!(f, "mode {mode}, not a server reply"),
            Refusal::Unsynchronized => f.write_str("server says its clock is unsynchronized"),
            Refusal::KissOfDeath(code) => {
                f.write_str("kiss of death ")?;
                // Printable ASCII as itself; anything else escaped, since
                // the code comes off the wire.
                for &byte in code {
                    if byte.is_ascii_graphic() {
                        write!(f, "{}", byte as char)?;
                    } else {
                        write!(f, "\\x{byte:02x}")?;
                    }
                }
                Ok(())
            }
            Refusal::Stratum(stratum) => write!(f, "stratum {stratum}, unsynchronized"),
            Refusal::NotOurRequest => f.write_str("not an answer to this request"),
            Refusal::NoTimestamp => f.write_str("no transmit timestamp"),
            Refusal::Before1970 => f.write_str("transmit timestamp before 1970"),
        }
    }
}

/// Keeps the wall clock set, forever.
///
/// Waits for an address, syncs, and repeats: after a success on the re-sync
/// interval, after a failure on a backoff that starts at a second and
/// doubles up to the retry interval. Every
/// sync and every failure is a console line: a clock that never sets is
/// otherwise indistinguishable from a clock nobody asked to set.
///
/// Needs a UDP socket, and a DNS one while `config.server` is a name, from
/// the stack's `StackResources`; and a resolver from the lease, unless the
/// server is written as an address.
pub async fn run(stack: Stack<'_>, config: NtpConfig<'_>) -> ! {
    let retry_max = to_embassy(config.retry_interval);
    let resync = to_embassy(config.resync_interval);
    let mut retry = FIRST_RETRY.min(retry_max);
    loop {
        // Re-checked every pass rather than only at startup: a lease can be
        // lost, and asking a server from an address the board no longer
        // holds is a request that cannot be answered.
        stack.wait_config_up().await;

        // Resolved here rather than inside `sync` so a failure can name the
        // address that did not answer. A pool name is a different
        // volunteer server on every lookup, and one of them not answering
        // is ordinary; the same failure against a different address every
        // time is the board, not the pool -- and only the address tells
        // the two apart.
        let result = match resolve(stack, config.server).await {
            Ok(address) => sync(stack, address).await.map_err(|e| (e, Some(address))),
            Err(e) => Err((e, None)),
        };
        let wait = match result {
            Ok(()) => {
                retry = FIRST_RETRY.min(retry_max);
                resync
            }
            Err((e, address)) => {
                let wait = retry;
                retry = next_retry(retry, retry_max);
                logln!(
                    "sntp: {} -- {e}; retrying in {} s",
                    Server(config.server, address),
                    wait.as_secs()
                );
                wait
            }
        };
        Timer::after(wait).await;
    }
}

/// The wait after `retry`'s: double it, but no more than `max`.
fn next_retry(retry: Duration, max: Duration) -> Duration {
    retry.checked_mul(2).unwrap_or(Duration::MAX).min(max)
}

/// `core`'s duration as `embassy-time`'s, saturating: an interval too long
/// for `embassy-time` to count is one that never elapses either way.
fn to_embassy(duration: StdDuration) -> Duration {
    Duration::try_from(duration).unwrap_or(Duration::MAX)
}

/// A server as a failure line names it: `pool.ntp.org (203.0.113.7)` once
/// the name has resolved, the name alone before that, and a server
/// written as an address only once rather than twice.
struct Server<'a>(&'a str, Option<Ipv4Addr>);

impl core::fmt::Display for Server<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.1 {
            Some(address) if self.0.parse::<Ipv4Addr>().is_err() => {
                write!(f, "{} ({address})", self.0)
            }
            _ => f.write_str(self.0),
        }
    }
}

/// Queries `address` once and sets the clock from the answer.
async fn sync(stack: Stack<'_>, address: Ipv4Addr) -> Result<(), Error> {
    let mut rx_meta = [PacketMetadata::EMPTY; QUEUE_DEPTH];
    let mut rx_buffer = [0u8; REPLY_MAX * QUEUE_DEPTH];
    let mut tx_meta = [PacketMetadata::EMPTY; QUEUE_DEPTH];
    let mut tx_buffer = [0u8; PACKET_LEN * QUEUE_DEPTH];
    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );
    // Port 0: a dynamic port from `embassy-net`, which starts each boot at an
    // offset drawn from the stack's random seed and counts up from there, so
    // every sync goes out from a different port than the last and the first
    // of a boot is not one an off-path forger can predict. That is a second
    // thing to guess besides the nonce, which is why DNS clients randomize
    // theirs.
    //
    // It replaced a fixed port, 12123, reused on every sync of every boot. A
    // fixed source port can land on a router's translation entry left from
    // the previous boot's exchange, and a first sync after a reboot failing
    // against a server that answers every other client is what that looks
    // like.
    socket.bind(0).map_err(|_| Error::Send)?;

    // The transmit timestamp is this exchange's nonce rather than a time:
    // the board does not know the time, that being the point, and what the
    // field is used for here is matching the reply. The monotonic counter
    // in microseconds is unguessable enough for that at the distance an
    // off-path forger works from.
    let sent_at = Instant::now();
    let nonce = sent_at.as_micros().to_be_bytes();

    socket
        .send_to(&request(nonce), (IpAddress::Ipv4(address), NTP_PORT))
        .await
        .map_err(|_| Error::Send)?;

    // Everything that arrives before the deadline, not just the first
    // datagram: a forgery arriving ahead of the real reply would otherwise
    // be the only one looked at, and the real one behind it never read.
    let deadline = sent_at + REPLY_TIMEOUT;
    let mut reply = [0u8; REPLY_MAX];
    // Every datagram set aside is logged as it arrives, with why: a server
    // answering in a way this refuses is otherwise indistinguishable from
    // one that did not answer.
    let mut ignored = 0;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.as_ticks() == 0 {
            return Err(Error::Timeout { ignored });
        }
        let Ok(Ok((len, meta))) = with_timeout(remaining, socket.recv_from(&mut reply)).await
        else {
            return Err(Error::Timeout { ignored });
        };
        if meta.endpoint.addr != IpAddress::Ipv4(address) {
            ignored += 1;
            logln!(
                "sntp: ignored a datagram from {}, not {address}",
                meta.endpoint
            );
            continue;
        }
        let server_millis = match parse(&reply[..len], nonce) {
            Ok(millis) => millis,
            Err(refusal) => {
                ignored += 1;
                logln!("sntp: ignored a reply from {address}: {refusal}");
                continue;
            }
        };

        let round_trip = Instant::now()
            .saturating_duration_since(sent_at)
            .as_millis();
        let unix_millis = corrected(server_millis, round_trip);

        let previous = clock::now_unix_millis();
        clock::set(unix_millis);

        let time = clock::DateTime::from_unix(unix_millis / 1_000);
        match previous {
            // The correction against the clock the board was already
            // keeping, which is the only measurement of its drift anyone
            // gets -- and the number that would say, if it grew, that
            // something is wrong with the timer rather than with the
            // network.
            Some(previous) => logln!(
                "sntp: {time} from {address} (round trip {round_trip} ms, {} ms correction)",
                unix_millis as i64 - previous as i64
            ),
            None => logln!("sntp: {time} from {address} (round trip {round_trip} ms)"),
        }
        return Ok(());
    }
}

/// Resolves `server`, or takes it as it stands if it is written as an
/// address.
///
/// An address is worth supporting for its own sake — a router is usually a
/// perfectly good time server and naming it by address needs no resolver —
/// and it is also the way out if DHCP supplies no usable DNS.
async fn resolve(stack: Stack<'_>, server: &str) -> Result<Ipv4Addr, Error> {
    if let Ok(address) = server.parse::<Ipv4Addr>() {
        return Ok(address);
    }
    let addresses = with_timeout(DNS_TIMEOUT, stack.dns_query(server, DnsQueryType::A))
        .await
        .map_err(|_| Error::DnsTimeout)?
        .map_err(|_| Error::Dns)?;
    match addresses.first() {
        Some(IpAddress::Ipv4(address)) => Ok(*address),
        None => Err(Error::NoAddress),
    }
}

/// A client request carrying `nonce` as its transmit timestamp.
///
/// Leap indicator 0, version 4, mode 3 (client); everything else a client
/// sends is ignored by the server.
fn request(nonce: [u8; 8]) -> [u8; PACKET_LEN] {
    let mut packet = [0u8; PACKET_LEN];
    packet[0] = 0b00_100_011;
    packet[40..48].copy_from_slice(&nonce);
    packet
}

/// The server's time, corrected for the half of the round trip that passed
/// between it reading its clock and this board reading the answer.
///
/// The round trip is assumed symmetric, which is what SNTP assumes and what
/// a LAN very nearly is.
fn corrected(server_millis: u64, round_trip_millis: u64) -> u64 {
    server_millis + round_trip_millis / 2
}

/// Reads a reply, returning the server's transmit timestamp as Unix
/// milliseconds, or why it is not a usable answer to this request.
///
/// Refused rather than used, and each for its own reason:
///
/// * anything shorter than a packet, or not a server's reply (mode 4), is
///   not an answer;
/// * an originate timestamp that is not the one sent is an answer to
///   somebody else's request, or a forgery — see the module documentation;
/// * leap indicator 3 is the server saying its own clock is not
///   synchronized, which is the one case where it is telling the truth
///   about being wrong;
/// * stratum 0 is a "kiss of death" packet, carrying a four-character
///   reason rather than a time, and stratum above 15 is unsynchronized by
///   definition;
/// * a zero transmit timestamp is a server that answered without filling
///   the field in.
fn parse(reply: &[u8], nonce: [u8; 8]) -> Result<u64, Refusal> {
    if reply.len() < PACKET_LEN {
        return Err(Refusal::Short(reply.len()));
    }
    let word =
        |at: usize| u32::from_be_bytes([reply[at], reply[at + 1], reply[at + 2], reply[at + 3]]);
    let leap = reply[0] >> 6;
    let mode = reply[0] & 0b111;
    let stratum = reply[1];
    if mode != 4 {
        return Err(Refusal::NotAServerReply(mode));
    }
    // Before the leap indicator: a kiss of death is often sent with LI 3
    // as well, and its code is the more useful of the two to report.
    if stratum == 0 {
        return Err(Refusal::KissOfDeath(word(12).to_be_bytes()));
    }
    if leap == 3 {
        return Err(Refusal::Unsynchronized);
    }
    if stratum > 15 {
        return Err(Refusal::Stratum(stratum));
    }
    if reply[24..32] != nonce {
        return Err(Refusal::NotOurRequest);
    }
    let (seconds, fraction) = (word(40), word(44));
    if seconds == 0 && fraction == 0 {
        return Err(Refusal::NoTimestamp);
    }
    to_unix_millis(seconds, fraction).ok_or(Refusal::Before1970)
}

/// An NTP timestamp — seconds since 1900 and a binary fraction of a second
/// — as Unix milliseconds, or `None` for a time before 1970.
///
/// NTP counts seconds in 32 bits, which run out in February 2036. RFC 4330
/// resolves that by the top bit: set means the current era, clear means the
/// one after it. Getting this wrong is not a subtle bug — it is a 136-year
/// error — and it costs one branch to get right.
///
/// Before 1970 is refused rather than subtracted: the top-bit rule puts
/// 1968–1970 in era 0, below the Unix epoch, and the subtraction would
/// underflow — a panic in a debug build, a date 580 million years out in a
/// release one. No server has a reason to send one, and the clock cannot
/// hold one.
fn to_unix_millis(seconds: u32, fraction: u32) -> Option<u64> {
    let unix_seconds = if seconds & 0x8000_0000 != 0 {
        u64::from(seconds).checked_sub(NTP_UNIX_OFFSET)?
    } else {
        u64::from(seconds) + (1 << 32) - NTP_UNIX_OFFSET
    };
    // A binary fraction of a second, so scaling it to milliseconds is a
    // multiply and a shift rather than a divide.
    let millis = (u64::from(fraction) * 1_000) >> 32;
    Some(unix_seconds * 1_000 + millis)
}

#[cfg(test)]
mod tests {
    use super::*;

    const NONCE: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];

    /// A well-formed reply to [`NONCE`]: leap 0, version 4, mode 4,
    /// stratum 2, transmitting `seconds`.`fraction`.
    fn reply(seconds: u32, fraction: u32) -> [u8; PACKET_LEN] {
        let mut packet = [0u8; PACKET_LEN];
        packet[0] = 0b00_100_100;
        packet[1] = 2;
        packet[24..32].copy_from_slice(&NONCE);
        packet[40..44].copy_from_slice(&seconds.to_be_bytes());
        packet[44..48].copy_from_slice(&fraction.to_be_bytes());
        packet
    }

    /// 2026-09-21 14:13:20 UTC, in NTP seconds.
    const SEPT_2026: u32 = (1_790_000_000 + NTP_UNIX_OFFSET) as u32;

    #[test]
    fn a_request_is_a_v4_client_packet_carrying_the_nonce() {
        let packet = request(NONCE);
        assert_eq!(packet[0] >> 6, 0, "leap");
        assert_eq!((packet[0] >> 3) & 0b111, 4, "version");
        assert_eq!(packet[0] & 0b111, 3, "mode");
        assert_eq!(packet[40..48], NONCE);
        assert!(packet[1..40].iter().all(|&b| b == 0));
    }

    #[test]
    fn a_good_reply_gives_the_transmit_time() {
        assert_eq!(parse(&reply(SEPT_2026, 0), NONCE), Ok(1_790_000_000_000));
        // Half a second, as a binary fraction.
        assert_eq!(
            parse(&reply(SEPT_2026, 0x8000_0000), NONCE),
            Ok(1_790_000_000_500)
        );
    }

    #[test]
    fn extension_fields_after_the_packet_are_ignored() {
        let mut long = [0u8; PACKET_LEN + 20];
        long[..PACKET_LEN].copy_from_slice(&reply(SEPT_2026, 0));
        assert_eq!(parse(&long, NONCE), Ok(1_790_000_000_000));
    }

    #[test]
    fn replies_that_are_not_answers_are_refused_and_say_why() {
        let good = reply(SEPT_2026, 0);
        assert_eq!(
            parse(&good[..PACKET_LEN - 1], NONCE),
            Err(Refusal::Short(PACKET_LEN - 1))
        );
        assert_eq!(parse(&good, [0; 8]), Err(Refusal::NotOurRequest));

        let mut client = good;
        client[0] = 0b00_100_011;
        assert_eq!(parse(&client, NONCE), Err(Refusal::NotAServerReply(3)));

        let mut unsynchronized = good;
        unsynchronized[0] |= 0b11 << 6;
        assert_eq!(parse(&unsynchronized, NONCE), Err(Refusal::Unsynchronized));

        let mut stratum_16 = good;
        stratum_16[1] = 16;
        assert_eq!(parse(&stratum_16, NONCE), Err(Refusal::Stratum(16)));

        assert_eq!(parse(&reply(0, 0), NONCE), Err(Refusal::NoTimestamp));
    }

    /// A kiss of death carries its reason in the reference id, and is
    /// often sent with leap indicator 3 too — the code is the one worth
    /// reporting.
    #[test]
    fn a_kiss_of_death_reports_its_code() {
        use alloc::format;
        let mut kiss = reply(SEPT_2026, 0);
        kiss[0] |= 0b11 << 6;
        kiss[1] = 0;
        kiss[12..16].copy_from_slice(b"RATE");
        let refusal = parse(&kiss, NONCE).unwrap_err();
        assert_eq!(refusal, Refusal::KissOfDeath(*b"RATE"));
        assert_eq!(format!("{refusal}"), "kiss of death RATE");
        assert_eq!(
            format!("{}", Refusal::KissOfDeath([b'X', 0, b'\n', b'Y'])),
            "kiss of death X\\x00\\x0aY"
        );
    }

    #[test]
    fn a_timeout_says_whether_anything_arrived() {
        use alloc::format;
        assert_eq!(format!("{}", Error::Timeout { ignored: 0 }), "no reply");
        assert_eq!(
            format!("{}", Error::Timeout { ignored: 2 }),
            "2 replies, none usable"
        );
    }

    #[test]
    fn the_2036_era_rollover() {
        // The last second of era 0: 2036-02-07 06:28:15 UTC.
        assert_eq!(to_unix_millis(u32::MAX, 0), Some(2_085_978_495_000));
        // The first second of era 1, one second later, with the top bit clear.
        assert_eq!(to_unix_millis(0, 0), Some(2_085_978_496_000));
        // The Unix epoch, which is in era 0.
        assert_eq!(to_unix_millis(NTP_UNIX_OFFSET as u32, 0), Some(0));
    }

    #[test]
    fn before_1970_is_refused_not_underflowed() {
        // Top bit set, so era 0 -- and 1968, before the Unix epoch.
        assert_eq!(to_unix_millis(0x8000_0000, 0), None);
        assert_eq!(to_unix_millis(NTP_UNIX_OFFSET as u32 - 1, 0), None);
        assert_eq!(
            parse(&reply(0x8000_0000, 0), NONCE),
            Err(Refusal::Before1970)
        );
    }

    #[test]
    fn a_failure_names_the_address_asked() {
        use alloc::format;
        let resolved = Some(Ipv4Addr::new(203, 0, 113, 7));
        assert_eq!(
            format!("{}", Server("pool.ntp.org", resolved)),
            "pool.ntp.org (203.0.113.7)"
        );
        assert_eq!(format!("{}", Server("pool.ntp.org", None)), "pool.ntp.org");
        assert_eq!(
            format!("{}", Server("203.0.113.7", resolved)),
            "203.0.113.7"
        );
    }

    #[test]
    fn half_the_round_trip_is_added() {
        assert_eq!(corrected(1_000, 40), 1_020);
        assert_eq!(corrected(1_000, 0), 1_000);
    }

    #[test]
    fn the_retry_doubles_up_to_the_cap() {
        let max = Duration::from_secs(30);
        let mut retry = FIRST_RETRY;
        let mut waits = [0; 7];
        for wait in &mut waits {
            *wait = retry.as_secs();
            retry = next_retry(retry, max);
        }
        assert_eq!(waits, [1, 2, 4, 8, 16, 30, 30]);
        assert_eq!(next_retry(Duration::MAX, Duration::MAX), Duration::MAX);
    }

    #[test]
    fn defaults() {
        let config = NtpConfig::default();
        assert_eq!(config.server, "pool.ntp.org");
        assert_eq!(config.retry_interval, StdDuration::from_secs(30));
        assert_eq!(config.resync_interval, StdDuration::from_secs(21_600));
        assert_eq!(to_embassy(StdDuration::MAX), Duration::MAX);
    }
}
