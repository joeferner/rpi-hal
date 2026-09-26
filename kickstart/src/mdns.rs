//! An mDNS responder for one name, so a board can be reached as
//! `<name>.local` instead of as an address.
//!
//! Purely a convenience, and deliberately built as one. It is a future a
//! board spawns and forgets, and one that can fail entirely without
//! taking anything else with it.
//!
//! ```ignore
//! #[embassy_executor::task]
//! async fn mdns_task(stack: Stack<'static>) -> ! {
//!     rpi_kickstart::mdns::run(stack, || "sensor").await
//! }
//! ```
//!
//! # A responder, not an implementation of mDNS
//!
//! It answers `A` queries for one name and announces that name
//! unsolicited, and that is the whole of it. Out of scope, each for its
//! own reason:
//!
//! * **Service discovery** (`SRV`, `TXT`, `_http._tcp`). Browsing would
//!   put a board in the list a Bonjour browser shows; nobody is looking
//!   for it there. The name is what someone types.
//! * **Reverse `PTR` lookups**, for the same reason.
//! * **Name-conflict probing** (RFC 6762 §8.1). This is the omission
//!   worth writing down rather than discovering: two boards configured
//!   with the same name will both answer, and every querier will believe
//!   whichever reply it heard last. A correct implementation probes for
//!   the name before claiming it and renames itself on a conflict. On a
//!   home LAN with one of these the conflict cannot arise, and the fix if
//!   it ever does is to change the name on one of them.
//! * **Known-answer suppression** (§7.1), where a querier lists the
//!   records it already holds and a responder stays quiet. Ignoring it
//!   costs one small multicast packet on a link where the querier already
//!   knew the answer.
//!
//! `edge-mdns` was the alternative, and it is a good crate: it fits
//! `no_std`, it fits `embassy-net` through `edge-nal-embassy`, and its
//! versions all line up with this graph's. It is not used because what it
//! implements is the whole of mDNS and DNS-SD on top of the `domain`
//! crate and a second major version of `heapless`, to answer one question
//! about one `A` record. The risk in this feature was never the DNS — it
//! was the multicast plumbing underneath, which no dependency avoids.
//!
//! # The multicast plumbing, which is the board's half
//!
//! Two things, and only one of them happens here:
//!
//! * `embassy-net`'s `multicast` feature and `Stack::join_multicast_group`.
//!   [`run`](crate::mdns::run) does this. Without the join, smoltcp drops
//!   an IPv4 packet addressed to a group it is not a member of before any
//!   socket sees it.
//! * **The interface's own receive filter**, which this cannot reach and
//!   the board must open before spawning. A LAN9514 passes
//!   unicast-to-us and broadcast and drops multicast until told
//!   otherwise; a Wi-Fi adapter wants `allmulti` set. DHCP is broadcast,
//!   which is why everything up to the first multicast works without
//!   touching it — and why forgetting this presents as a responder that
//!   announces perfectly and answers nothing.
//!
//! # Why the name is a function
//!
//! [`run`](crate::mdns::run) takes `fn() -> &'static str` rather than a
//! name, and reads it on every pass. The name a board answers to is
//! usually a setting, and a setting can be saved from a web form while
//! this is running; capturing it at spawn would mean a rename took effect
//! at the next boot rather than within [`POLL`](crate::mdns::POLL). A
//! board whose name is fixed passes `|| "sensor"` and pays nothing.
//!
//! # Why the loop wakes on a timer
//!
//! [`POLL`](crate::mdns::POLL) bounds how long the responder can go on
//! announcing a name or an address that has changed underneath it. Both
//! change without anything here being told: a DHCP renewal can move the
//! address, and a save can rename the board. Waking on a receive alone
//! would tie noticing either one to some other host on the link happening
//! to send a query.

use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpAddress, IpEndpoint, Ipv4Address, Stack};
use embassy_time::{Duration, Timer, with_timeout};

use crate::logln;

/// The IPv4 mDNS group, `224.0.0.251` (RFC 6762 §3).
const GROUP: Ipv4Address = Ipv4Address::new(224, 0, 0, 251);

/// The mDNS port. Both ends of the conversation use it: a query arrives
/// from port 5353 and the answer goes back to port 5353, which is why the
/// socket is bound rather than ephemeral.
const PORT: u16 = 5353;

/// Where an answer goes when the querier did not ask for a unicast one.
const GROUP_ENDPOINT: IpEndpoint = IpEndpoint {
    addr: IpAddress::Ipv4(GROUP),
    port: PORT,
};

/// The second label of every name this answers to.
const LOCAL: &[u8] = b"local";

/// `QTYPE`/`TYPE` for an `A` record.
const TYPE_A: u16 = 1;
/// `QTYPE` for "any record you have".
const TYPE_ANY: u16 = 255;
/// `CLASS` for the internet class.
const CLASS_IN: u16 = 1;
/// `QCLASS` for "any class".
const CLASS_ANY: u16 = 255;
/// The top bit of a question's `QCLASS`: the querier is asking for the
/// answer unicast rather than to the group (RFC 6762 §5.4).
const UNICAST_RESPONSE: u16 = 0x8000;
/// The top bit of an answer's `CLASS`: everything else cached under this
/// name and type is stale (RFC 6762 §10.2).
const CACHE_FLUSH: u16 = 0x8000;

/// Header flags on the responses this sends: `QR` (this is a response)
/// and `AA` (authoritative). Opcode and rcode are both zero.
const RESPONSE_FLAGS: u16 = 0x8400;

/// How long a resolver may cache the answer, in seconds.
///
/// Short, because the thing it maps is usually a DHCP address. Two
/// minutes bounds how long a stale entry can survive a lease change on a
/// resolver that missed the announcement, at the cost of a query every
/// two minutes from anything actively using the name.
const TTL: u32 = 120;

/// The TTL on an answer to a legacy query, capped at 10 seconds by
/// RFC 6762 §6.7 — those answers travel to one host that cannot see the
/// announcements that would correct them.
const LEGACY_TTL: u32 = 10;

/// How long the loop waits for a query before looking at the address and
/// the name again.
pub const POLL: Duration = Duration::from_secs(5);

/// How many unsolicited announcements go out when the name or the address
/// changes. RFC 6762 §8.3 asks for at least two.
const ANNOUNCEMENTS: usize = 2;

/// The gap between them, as §8.3 specifies.
const ANNOUNCE_GAP: Duration = Duration::from_secs(1);

/// Receive buffer, in bytes.
///
/// Generous for what this parses — a query for one name is under a
/// hundred bytes — because it is not the only thing that arrives. A link
/// with Apple devices on it carries large DNS-SD packets, and a datagram
/// too big for this buffer is reported as truncated and dropped whole.
/// Being dropped is the right outcome for those; being dropped is *not*
/// the right outcome for a query with the one question this cares about
/// buried in it.
const RX_BUFFER: usize = 1500;

/// Transmit buffer, in bytes. One answer is under 200.
const TX_BUFFER: usize = 512;

/// Largest response this builds.
const RESPONSE_MAX: usize = 256;

/// A response packet, built in place.
type Response = heapless::Vec<u8, RESPONSE_MAX>;

/// Answers `A` queries for the name `name` returns, and announces it.
///
/// Never returns, so an application wraps it in a task of its own:
/// `#[embassy_executor::task]` cannot be generic, which is why this is an
/// `async fn` and not a task itself.
///
/// Waits for the stack to have a configuration before doing anything, so
/// it can be spawned alongside everything else rather than after DHCP.
///
/// The board must have opened its interface's multicast receive filter
/// first — see the module documentation, since the failure is silent.
pub async fn run(stack: Stack<'static>, name: fn() -> &'static str) -> ! {
    stack.wait_config_up().await;

    // Before the bind, so the first announcement below goes out to a link
    // that is already forwarding the group to us. A failure here is not
    // fatal: the announcements still leave, so caches still populate, and
    // what is lost is the ability to answer a question anyone asks later.
    if let Err(e) = stack.join_multicast_group(GROUP) {
        logln!("mdns: could not join {GROUP} ({e:?}); queries will not be answered");
    }

    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx_buffer = [0u8; RX_BUFFER];
    let mut tx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_buffer = [0u8; TX_BUFFER];
    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );

    // 5353 rather than an ephemeral port: a responder has to be reachable
    // at the port every querier sends to, and RFC 6762 §5.1 requires
    // answers to come *from* it as well.
    if let Err(e) = socket.bind(PORT) {
        logln!("mdns: could not bind port {PORT} ({e:?}); the name will not resolve");
        // Parked rather than retried. Nothing releases the port later,
        // and a task that cannot do its job should stop rather than log
        // about it forever — the line above is the report.
        core::future::pending::<()>().await;
    }

    // What the last announcement claimed, so a rename or a new lease is
    // noticed by comparison rather than by being signalled.
    let mut announced: Option<(&'static str, Ipv4Address)> = None;

    loop {
        let name = name();
        let address = address(stack);

        match address {
            Some(address) if announced != Some((name, address)) => {
                announce(&socket, name, address).await;
                announced = Some((name, address));
            }
            // No lease: nothing to announce and nothing worth answering
            // with, and the next lease has to re-announce whatever it
            // turns out to be, even if it is the address this had before.
            None => announced = None,
            _ => {}
        }

        // The parse and the reply are built inside the callback so the
        // datagram never has to be copied out of the socket's own buffer.
        let answer = with_timeout(
            POLL,
            socket.recv_from_with(|query, from| reply(query, from.endpoint, name, address)),
        )
        .await;

        // A timeout is the ordinary case, and means only that it is time
        // to look at the address and the name again.
        if let Ok(Some((response, to))) = answer
            && let Err(e) = socket.send_to(&response, to).await
        {
            logln!("mdns: answer to {to} not sent ({e:?})");
        }
    }
}

/// The address to answer with, or `None` before DHCP has produced one.
fn address(stack: Stack<'static>) -> Option<Ipv4Address> {
    stack.config_v4().map(|config| config.address.address())
}

/// Sends the unsolicited announcements for `name` at `address`.
///
/// Unsolicited because a cache that already holds the old answer will not
/// ask again until its TTL runs out, and the [`CACHE_FLUSH`] bit on these
/// is what replaces it now instead of in two minutes. This is what makes
/// the name follow a DHCP change without a reboot.
async fn announce(socket: &UdpSocket<'_>, name: &str, address: Ipv4Address) {
    let Some(packet) = response(name, address, 0, None) else {
        return;
    };

    for round in 0..ANNOUNCEMENTS {
        if round > 0 {
            Timer::after(ANNOUNCE_GAP).await;
        }
        if let Err(e) = socket.send_to(&packet, GROUP_ENDPOINT).await {
            logln!("mdns: announcement not sent ({e:?})");
            return;
        }
    }
    logln!("mdns: announced {name}.local as {address}");
}

/// Works out what to say about `query`, if anything.
///
/// `None` covers everything this does not answer, which is nearly all
/// mDNS traffic on a busy link: responses, queries about other names,
/// queries for record types this does not hold, and anything malformed.
fn reply(
    query: &[u8],
    from: IpEndpoint,
    name: &str,
    address: Option<Ipv4Address>,
) -> Option<(Response, IpEndpoint)> {
    let address = address?;
    let (id, questions, mut at) = header(query)?;

    for _ in 0..questions {
        let (next, matched) = question_name(query, at, name)?;
        let qtype = be16(query, next)?;
        let qclass = be16(query, next + 2)?;
        let end = next + 4;

        if matched
            && (qtype == TYPE_A || qtype == TYPE_ANY)
            && matches!(qclass & !UNICAST_RESPONSE, CLASS_IN | CLASS_ANY)
        {
            // A querier that did not send from port 5353 is not a
            // Multicast DNS querier at all — it is `dig`, or a resolver
            // stub, asking a one-off question of this host. RFC 6762 §6.7
            // calls that a legacy query, and the answer has to look like
            // a conventional DNS reply: unicast back to the port it came
            // from, the query's own ID, its question echoed, no
            // cache-flush bit, and a short TTL.
            //
            // Worth handling for ten lines, because it is how this gets
            // tested from a shell without an mDNS client in the way.
            let legacy = from.port != PORT;
            let question = legacy.then(|| &query[at..end]);
            let to = if legacy || qclass & UNICAST_RESPONSE != 0 {
                from
            } else {
                GROUP_ENDPOINT
            };
            return Some((
                response(name, address, if legacy { id } else { 0 }, question)?,
                to,
            ));
        }

        at = end;
    }
    None
}

/// Reads the header of a query, returning its ID, its question count, and
/// the offset the questions start at.
///
/// `None` for anything that is not a standard query: a response (a
/// board's own announcements come back to it, and answering one's own
/// announcement is how a packet storm starts), an inverse query, an
/// update.
fn header(query: &[u8]) -> Option<(u16, u16, usize)> {
    let id = be16(query, 0)?;
    let flags = be16(query, 2)?;
    let questions = be16(query, 4)?;

    // Bit 15 is QR and bits 14..11 are the opcode; both must be zero.
    if flags & 0xF800 != 0 {
        return None;
    }
    Some((id, questions, 12))
}

/// Walks the `QNAME` at `at`, reporting where it ends and whether it is
/// the name this board answers to.
///
/// `None` means the packet cannot be read any further — it ran off the
/// end, or it used name compression. Compression is rejected rather than
/// implemented: a pointer can only refer to a name that appeared earlier
/// in the same message, and the first question of a query for one host
/// has nothing earlier to point at. Following pointers is also where a
/// DNS parser traditionally acquires its infinite loop.
fn question_name(query: &[u8], mut at: usize, name: &str) -> Option<(usize, bool)> {
    let expected = [name.as_bytes(), LOCAL];
    let mut labels = 0;
    let mut matched = true;

    loop {
        let length = usize::from(*query.get(at)?);
        at += 1;
        if length == 0 {
            // Exactly the two labels, so `sensor.local` does not match a
            // query for `sensor.local.example.com`.
            return Some((at, matched && labels == expected.len()));
        }
        if length & 0xC0 != 0 {
            return None;
        }
        let end = at.checked_add(length)?;
        let label = query.get(at..end)?;
        at = end;

        // DNS comparison is case-insensitive, and the case a querier uses
        // is whatever the person typed.
        matched =
            matched && labels < expected.len() && label.eq_ignore_ascii_case(expected[labels]);
        labels += 1;
    }
}

/// Builds a response carrying one `A` record for `name`.
///
/// `question` is the question bytes to echo, present only for a legacy
/// query; its presence is what makes this a conventional DNS reply rather
/// than an mDNS one. A multicast response must carry no questions at all
/// (RFC 6762 §6).
fn response(
    name: &str,
    address: Ipv4Address,
    id: u16,
    question: Option<&[u8]>,
) -> Option<Response> {
    let legacy = question.is_some();
    let mut out = Response::new();

    out.extend_from_slice(&id.to_be_bytes()).ok()?;
    out.extend_from_slice(&RESPONSE_FLAGS.to_be_bytes()).ok()?;
    out.extend_from_slice(&u16::from(legacy).to_be_bytes())
        .ok()?;
    // One answer, and no authority or additional records.
    out.extend_from_slice(&1u16.to_be_bytes()).ok()?;
    out.extend_from_slice(&[0, 0, 0, 0]).ok()?;

    if let Some(question) = question {
        out.extend_from_slice(question).ok()?;
    }

    // The name written out in full rather than as a pointer back into the
    // echoed question: both are legal, and a message this small has
    // nothing to gain from compression but a second thing to get wrong.
    for label in [name.as_bytes(), LOCAL] {
        out.push(u8::try_from(label.len()).ok()?).ok()?;
        out.extend_from_slice(label).ok()?;
    }
    out.push(0).ok()?;

    out.extend_from_slice(&TYPE_A.to_be_bytes()).ok()?;
    let class = if legacy {
        CLASS_IN
    } else {
        CLASS_IN | CACHE_FLUSH
    };
    out.extend_from_slice(&class.to_be_bytes()).ok()?;
    let ttl = if legacy { LEGACY_TTL } else { TTL };
    out.extend_from_slice(&ttl.to_be_bytes()).ok()?;
    // RDLENGTH, then the address itself.
    out.extend_from_slice(&4u16.to_be_bytes()).ok()?;
    out.extend_from_slice(&address.octets()).ok()?;

    Some(out)
}

/// The big-endian `u16` at `at`, or `None` if the packet is shorter than
/// that.
fn be16(bytes: &[u8], at: usize) -> Option<u16> {
    let pair = bytes.get(at..at.checked_add(2)?)?;
    Some(u16::from_be_bytes([pair[0], pair[1]]))
}
