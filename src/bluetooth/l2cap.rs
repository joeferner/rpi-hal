//! L2CAP over LE: the thin framing layer between raw ACL data and the LE
//! host protocols (ATT, SMP, and the L2CAP signaling channel).
//!
//! On an LE link, L2CAP runs only in *basic mode* on a set of fixed
//! channels — there's no channel-establishment handshake as on Classic
//! BR/EDR. Each L2CAP PDU is a "B-frame": a 4-byte basic header
//! (`length(2)` + `cid(2)`, both little-endian) followed by `length` bytes
//! of payload, where the CID names the protocol above ([`CID_ATT`],
//! [`CID_SMP`], [`CID_LE_SIGNALING`]).
//!
//! A single B-frame can be larger than one ACL data packet, so the
//! controller splits it: the first ACL fragment carries the basic header
//! and the start of the payload ([`super::AclData::first_fragment`] set),
//! and continuation fragments carry the rest. [`Reassembler`] rebuilds the
//! whole B-frame from those fragments; [`send`] does the reverse, prefixing
//! a payload with the basic header and handing it to
//! [`super::Bluetooth::send_acl`], which re-fragments it to the ACL packet
//! size on the way out.
//!
//! This layer does not interpret the payload beyond the basic header — a
//! reassembled [`Pdu`]'s bytes are handed up by CID to whichever protocol
//! owns that channel.

use super::{AclData, Bluetooth, Error};

/// L2CAP CID of the Attribute Protocol (ATT) fixed channel — the transport
/// for GATT.
pub const CID_ATT: u16 = 0x0004;
/// L2CAP CID of the LE signaling fixed channel (L2CAP control commands,
/// e.g. Connection Parameter Update).
pub const CID_LE_SIGNALING: u16 = 0x0005;
/// L2CAP CID of the Security Manager Protocol (SMP) fixed channel — the
/// transport for LE pairing.
pub const CID_SMP: u16 = 0x0006;

/// Length of the L2CAP basic (B-frame) header: a 2-byte payload length and
/// a 2-byte channel identifier.
pub const HEADER_LEN: usize = 4;
/// Largest L2CAP payload this layer buffers, in bytes — comfortably above
/// any ATT MTU a peripheral like a keyboard negotiates. A B-frame whose
/// payload exceeds this is dropped on receive ([`Reassembler::feed`]) and
/// rejected on send ([`send`] returns [`Error::PayloadTooLarge`]).
pub const MAX_PAYLOAD: usize = 512;

/// A fully reassembled L2CAP B-frame, borrowing its payload from the
/// [`Reassembler`] that produced it. The `cid` selects the protocol the
/// `payload` belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pdu<'a> {
    /// The channel identifier — which fixed channel (and so which protocol)
    /// this frame is for.
    pub cid: u16,
    /// The frame's information payload (the bytes after the basic header).
    pub payload: &'a [u8],
}

/// Reassembles L2CAP B-frames from the ACL fragments delivered by
/// [`super::Bluetooth::poll`].
///
/// Feed every inbound [`AclData`] to [`Self::feed`]; it accumulates
/// fragments and, once a complete B-frame has arrived, returns it as a
/// [`Pdu`]. A first fragment ([`AclData::first_fragment`]) starts a new
/// frame — on LE each ACL "first" fragment begins exactly one B-frame — so
/// receiving one always resets any partial frame in progress.
///
/// Sized for a single connection's inbound stream: the buffer holds one
/// in-progress B-frame at a time, which is all a point-to-point peripheral
/// needs.
pub struct Reassembler {
    /// Accumulation buffer: the basic header followed by payload bytes.
    buf: [u8; HEADER_LEN + MAX_PAYLOAD],
    /// Bytes accumulated in `buf` so far.
    len: usize,
    /// Total B-frame size (header + payload) once the header has arrived,
    /// or `None` while fewer than [`HEADER_LEN`] bytes are buffered.
    expected: Option<usize>,
    /// The connection handle the in-progress frame belongs to, so a
    /// continuation fragment for a different handle can be ignored.
    handle: u16,
}

impl Reassembler {
    /// Creates an empty reassembler with no frame in progress.
    pub fn new() -> Self {
        Self {
            buf: [0u8; HEADER_LEN + MAX_PAYLOAD],
            len: 0,
            expected: None,
            handle: 0,
        }
    }

    /// Discards any partially reassembled frame, returning to the empty
    /// state — e.g. after a disconnect, before the next connection reuses
    /// the reassembler.
    pub fn reset(&mut self) {
        self.len = 0;
        self.expected = None;
    }

    /// Feeds one ACL fragment in, returning the completed [`Pdu`] if this
    /// fragment finished a B-frame, or `None` if more fragments are needed
    /// (or the fragment was ignored).
    ///
    /// A [`first fragment`](AclData::first_fragment) resets accumulation and
    /// begins a new frame; a continuation fragment appends to the frame in
    /// progress, and is ignored if it names a different connection handle
    /// than the frame being assembled. A frame whose declared payload
    /// exceeds [`MAX_PAYLOAD`] never completes (its bytes are dropped) until
    /// the next first fragment resets the reassembler.
    pub fn feed(&mut self, acl: &AclData) -> Option<Pdu<'_>> {
        if acl.first_fragment {
            self.len = 0;
            self.expected = None;
            self.handle = acl.handle;
        } else if acl.handle != self.handle {
            // A continuation for a frame we aren't assembling — ignore it
            // rather than corrupting the one in progress.
            return None;
        }

        // Append the fragment, bounded by the buffer. If the frame is
        // larger than the buffer the tail is dropped and the frame will
        // never reach `expected`, so it's silently discarded until the next
        // first fragment — see the doc comment.
        let data = acl.data();
        let room = self.buf.len() - self.len;
        let take = data.len().min(room);
        self.buf[self.len..self.len + take].copy_from_slice(&data[..take]);
        self.len += take;

        // Once the basic header is present, the total frame size is known.
        if self.expected.is_none() && self.len >= HEADER_LEN {
            let length = u16::from_le_bytes([self.buf[0], self.buf[1]]) as usize;
            self.expected = Some(HEADER_LEN + length);
        }

        match self.expected {
            Some(total) if self.len >= total => {
                let cid = u16::from_le_bytes([self.buf[2], self.buf[3]]);
                // Leave `len`/`expected` as they are: the next first
                // fragment resets them. Extra bytes past `total` (not
                // expected on LE, where one first fragment is one B-frame)
                // are ignored by slicing to `total`.
                Some(Pdu {
                    cid,
                    payload: &self.buf[HEADER_LEN..total],
                })
            }
            _ => None,
        }
    }
}

impl Default for Reassembler {
    fn default() -> Self {
        Self::new()
    }
}

/// Sends an L2CAP B-frame: prefixes `payload` with the basic header
/// (`length` + `cid`) and hands the whole frame to
/// [`Bluetooth::send_acl`], which fragments it to the controller's ACL
/// packet size.
///
/// `payload` is the protocol PDU for the channel (an ATT PDU for
/// [`CID_ATT`], etc.); this builds the L2CAP header around it but does not
/// touch its contents. Returns [`Error::PayloadTooLarge`] if `payload`
/// exceeds [`MAX_PAYLOAD`], or a transport error from the underlying send.
pub fn send(bt: &mut Bluetooth, handle: u16, cid: u16, payload: &[u8]) -> Result<(), Error> {
    if payload.len() > MAX_PAYLOAD {
        return Err(Error::PayloadTooLarge);
    }
    let mut frame = [0u8; HEADER_LEN + MAX_PAYLOAD];
    frame[0..2].copy_from_slice(&(payload.len() as u16).to_le_bytes());
    frame[2..4].copy_from_slice(&cid.to_le_bytes());
    frame[HEADER_LEN..HEADER_LEN + payload.len()].copy_from_slice(payload);
    bt.send_acl(handle, &frame[..HEADER_LEN + payload.len()])
}
