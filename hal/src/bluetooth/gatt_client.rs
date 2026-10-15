//! A GATT client over ATT (L2CAP channel [`CID_ATT`]) for the central role —
//! the counterpart to the [`gatt`](super::gatt) server.
//!
//! Where [`Server`](super::gatt::Server) answers the requests a client sends,
//! [`Client`] *is* that client: once [`Bluetooth::connect`] has established a
//! link, it drives the ATT request/response exchanges that discover a
//! peripheral's services and characteristics, subscribes to notifications by
//! writing a characteristic's CCC descriptor, and surfaces the notifications
//! the peripheral then pushes. This is the path a host uses to talk to a HID
//! device (a mouse, a gamepad) over HID-over-GATT.
//!
//! # Model
//!
//! ATT on a single connection is strictly one request at a time: the client
//! sends a request and the server answers with exactly one response (or an
//! Error Response) before the next request goes out. [`Client`] follows that
//! model — each discovery method sends a request and pumps
//! [`Bluetooth::poll`] until the matching response (an ACL fragment
//! reassembled through the owned [`Reassembler`]) arrives, so the calls read
//! as straight-line synchronous operations.
//!
//! Discovery of a whole database is several such round trips: a Read By Group
//! Type walk for [`Service`]s, a Read By Type walk for each service's
//! [`Characteristic`]s, and a Find Information walk for the [`Descriptor`]s
//! (the CCC among them). The results land in caller-provided slices — the
//! crate is `no_std` with no allocator, so the caller owns the storage and
//! the method returns how many entries it filled.
//!
//! # Notifications
//!
//! After [`Client::subscribe`] writes a characteristic's CCC descriptor, the
//! peripheral pushes new values as unacknowledged Handle Value Notifications.
//! Those arrive asynchronously, interleaved with anything else, so the caller
//! runs its own [`Bluetooth::poll`] loop and hands each [`Event::Acl`] to
//! [`Client::feed`], which reassembles it and returns a [`Notification`] when
//! one completes.
//!
//! # Scope
//!
//! LE only, one connection per [`Client`], basic-mode ATT. Not implemented:
//! reading long values across Read Blob (only the first response's bytes are
//! kept), the Read By Type form of characteristic-value reads, and Handle
//! Value *Indications* (the acknowledged variant) — a notification-based HID
//! device needs none of these. Encryption/pairing, which HID-over-GATT
//! requires before a peripheral will notify, is a separate layer
//! ([`smp`](super::smp)).

use super::gatt::{Uuid, ATT_DEFAULT_MTU, ATT_MAX_MTU, UUID_CHARACTERISTIC, UUID_PRIMARY_SERVICE};
use super::l2cap::{self, Reassembler, CID_ATT};
use super::{AclData, Bluetooth, Error, Event};
use crate::timer::Timer;

/// ATT opcode `Error Response` — the server rejecting a request, carrying the
/// rejected opcode, the handle in question, and an error code.
const OP_ERROR_RESPONSE: u8 = 0x01;
/// ATT opcode `Exchange MTU Request`.
const OP_EXCHANGE_MTU_REQ: u8 = 0x02;
/// ATT opcode `Exchange MTU Response`.
const OP_EXCHANGE_MTU_RSP: u8 = 0x03;
/// ATT opcode `Find Information Request` — descriptor discovery.
const OP_FIND_INFORMATION_REQ: u8 = 0x04;
/// ATT opcode `Find Information Response`.
const OP_FIND_INFORMATION_RSP: u8 = 0x05;
/// ATT opcode `Read By Type Request` — characteristic discovery (type
/// [`UUID_CHARACTERISTIC`]).
const OP_READ_BY_TYPE_REQ: u8 = 0x08;
/// ATT opcode `Read By Type Response`.
const OP_READ_BY_TYPE_RSP: u8 = 0x09;
/// ATT opcode `Read By Group Type Request` — primary service discovery (type
/// [`UUID_PRIMARY_SERVICE`]).
const OP_READ_BY_GROUP_TYPE_REQ: u8 = 0x10;
/// ATT opcode `Read By Group Type Response`.
const OP_READ_BY_GROUP_TYPE_RSP: u8 = 0x11;
/// ATT opcode `Write Request` (client wants a Write Response) — used to write
/// a CCC descriptor when subscribing.
const OP_WRITE_REQ: u8 = 0x12;
/// ATT opcode `Write Response`.
const OP_WRITE_RSP: u8 = 0x13;
/// ATT opcode `Handle Value Notification` — a server-initiated, unacknowledged
/// push of a characteristic value to a subscribed client.
const OP_HANDLE_VALUE_NOTIFICATION: u8 = 0x1b;

/// ATT error code `Attribute Not Found` — returned by a discovery walk once no
/// more matching attributes remain in the requested range, which is how the
/// walk knows it is done (not a failure).
const ERR_ATTRIBUTE_NOT_FOUND: u8 = 0x0a;

/// CCC descriptor value bit that enables notifications for its characteristic
/// — the value [`Client::subscribe`] writes.
pub const CCC_NOTIFY: u16 = 0x0001;
/// CCC descriptor value bit that enables indications (acknowledged) for its
/// characteristic.
pub const CCC_INDICATE: u16 = 0x0002;

/// The full handle range `0x0001..=0xffff`, the span a discovery walk starts
/// from when scanning the whole database.
const HANDLE_MIN: u16 = 0x0001;
/// See [`HANDLE_MIN`].
const HANDLE_MAX: u16 = 0xffff;

/// Time budget for a single ATT request/response round trip, in
/// milliseconds. The peer is a directly-connected peripheral answering in
/// milliseconds; this is generous headroom, well under the 30 s ATT
/// transaction timeout the spec allows.
const ATT_REQUEST_TIMEOUT_MS: u32 = 5_000;
/// Upper bound on how long a single [`Bluetooth::poll`] inside a request
/// blocks before the round trip re-checks its deadline, in milliseconds.
const POLL_SLICE_MS: u32 = 500;

/// A primary service discovered by [`Client::discover_primary_services`]: its
/// attribute-handle range and type UUID.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Service {
    /// Handle of the service declaration — the start of the service's group.
    pub start_handle: u16,
    /// Handle of the last attribute in the service's group.
    pub end_handle: u16,
    /// The service's UUID (16- or 128-bit).
    pub uuid: Uuid,
}

/// A characteristic discovered by [`Client::discover_characteristics`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Characteristic {
    /// Handle of the characteristic declaration attribute.
    pub decl_handle: u16,
    /// The characteristic's property bits (e.g.
    /// [`CHAR_PROP_NOTIFY`](super::gatt::CHAR_PROP_NOTIFY)).
    pub properties: u8,
    /// Handle of the characteristic's *value* attribute — the handle a
    /// notification carries, and the anchor for finding its CCC descriptor.
    pub value_handle: u16,
    /// The characteristic's UUID (16- or 128-bit).
    pub uuid: Uuid,
}

/// A descriptor discovered by [`Client::discover_descriptors`]: a handle and
/// its type UUID (e.g. [`UUID_CCC_DESCRIPTOR`](super::gatt::UUID_CCC_DESCRIPTOR)
/// for the CCC that [`Client::subscribe`] writes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Descriptor {
    /// The descriptor's attribute handle.
    pub handle: u16,
    /// The descriptor's type UUID.
    pub uuid: Uuid,
}

/// An inbound Handle Value Notification surfaced by [`Client::feed`]: the
/// value handle it targets and the length of the value written into the
/// caller's buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Notification {
    /// The handle of the characteristic value the notification carries — match
    /// it against a [`Characteristic::value_handle`] to know which one fired.
    pub value_handle: u16,
    /// Number of value bytes written into the buffer passed to
    /// [`Client::feed`].
    pub len: usize,
}

/// Errors from the GATT client.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClientError {
    /// An underlying HCI/transport error (from [`Bluetooth::send_acl`] or
    /// [`Bluetooth::poll`]).
    Hci(Error),
    /// No ATT response arrived within the per-request time budget (5 s).
    Timeout,
    /// The link dropped mid-exchange; carries the HCI disconnect reason code.
    Disconnected(u8),
    /// The server answered with an ATT Error Response. An error code of
    /// Attribute Not Found (`0x0a`) on a discovery request is not surfaced as
    /// this error — it is the normal end of a walk — so any `Att` seen here is
    /// a genuine rejection.
    Att {
        /// The request opcode the server rejected.
        request: u8,
        /// The attribute handle the error refers to.
        handle: u16,
        /// The ATT error code.
        code: u8,
    },
    /// A response was malformed or was not the expected reply to the request
    /// just sent.
    Protocol,
    /// A discovery walk produced more entries than the caller's slice could
    /// hold. The slice is filled; grow it and retry to see the rest.
    Overflow,
}

impl From<Error> for ClientError {
    fn from(e: Error) -> Self {
        ClientError::Hci(e)
    }
}

/// A GATT client bound to one LE connection.
///
/// Create it with [`Self::new`] from the handle of an [`Event::Connected`]
/// (with [`Role::Central`](super::Role::Central)), optionally raise the MTU
/// with [`Self::exchange_mtu`], then discover and subscribe. It owns the
/// [`Reassembler`] for the connection's inbound ATT traffic, so both the
/// discovery round trips and the steady-state [`Self::feed`] share one
/// reassembly buffer.
pub struct Client {
    /// The connection handle this client talks over.
    handle: u16,
    /// Reassembles inbound ACL fragments into L2CAP frames for this
    /// connection.
    reasm: Reassembler,
    /// The ATT MTU in effect — [`ATT_DEFAULT_MTU`] until
    /// [`Self::exchange_mtu`] negotiates a larger one.
    mtu: u16,
}

impl Client {
    /// Creates a client for the connection identified by `connection_handle`
    /// (from an [`Event::Connected`]). The MTU starts at [`ATT_DEFAULT_MTU`].
    pub fn new(connection_handle: u16) -> Self {
        Self {
            handle: connection_handle,
            reasm: Reassembler::new(),
            mtu: ATT_DEFAULT_MTU,
        }
    }

    /// The connection handle this client is bound to.
    pub fn handle(&self) -> u16 {
        self.handle
    }

    /// The ATT MTU currently in effect.
    pub fn mtu(&self) -> u16 {
        self.mtu
    }

    /// Negotiates the ATT MTU (`Exchange MTU`), proposing [`ATT_MAX_MTU`] and
    /// adopting the smaller of that and the server's answer. Returns the MTU
    /// now in effect.
    ///
    /// Optional but worthwhile: the 23-byte default only carries 20 bytes of
    /// value per notification, and a larger MTU also lets service discovery
    /// pack more entries per response. A server that doesn't support the
    /// exchange answers with an Error Response, surfaced as
    /// [`ClientError::Att`]; the caller may treat that as "keep the default".
    pub fn exchange_mtu(&mut self, bt: &mut Bluetooth, timer: &Timer) -> Result<u16, ClientError> {
        let req = [
            OP_EXCHANGE_MTU_REQ,
            ATT_MAX_MTU.to_le_bytes()[0],
            ATT_MAX_MTU.to_le_bytes()[1],
        ];
        let mut resp = [0u8; ATT_MAX_MTU as usize];
        let n = self.request(bt, &req, OP_EXCHANGE_MTU_RSP, &mut resp, timer)?;
        if n < 3 {
            return Err(ClientError::Protocol);
        }
        let server_mtu = u16::from_le_bytes([resp[1], resp[2]]);
        self.mtu = server_mtu.clamp(ATT_DEFAULT_MTU, ATT_MAX_MTU);
        Ok(self.mtu)
    }

    /// Discovers all primary services on the peer, filling `out` and returning
    /// how many were found (a `Read By Group Type` walk over type
    /// [`UUID_PRIMARY_SERVICE`]).
    ///
    /// Returns [`ClientError::Overflow`] if there are more services than `out`
    /// holds (the slice is still filled). Any ATT error other than the
    /// walk-ending Attribute Not Found is surfaced as [`ClientError::Att`].
    pub fn discover_primary_services(
        &mut self,
        bt: &mut Bluetooth,
        out: &mut [Service],
        timer: &Timer,
    ) -> Result<usize, ClientError> {
        let mut resp = [0u8; ATT_MAX_MTU as usize];
        let mut count = 0;
        let mut start = HANDLE_MIN;
        loop {
            let ps = UUID_PRIMARY_SERVICE.to_le_bytes();
            let req = [
                OP_READ_BY_GROUP_TYPE_REQ,
                start.to_le_bytes()[0],
                start.to_le_bytes()[1],
                HANDLE_MAX.to_le_bytes()[0],
                HANDLE_MAX.to_le_bytes()[1],
                ps[0],
                ps[1],
            ];
            let n = match self.request(bt, &req, OP_READ_BY_GROUP_TYPE_RSP, &mut resp, timer) {
                Ok(n) => n,
                Err(ClientError::Att { code, .. }) if code == ERR_ATTRIBUTE_NOT_FOUND => {
                    return Ok(count)
                }
                Err(e) => return Err(e),
            };
            // [opcode, entry_len(1), entries...]; each entry is
            // handle(2), group_end(2), value(entry_len-4 = service UUID).
            if n < 2 {
                return Err(ClientError::Protocol);
            }
            let entry_len = resp[1] as usize;
            if entry_len < 6 || (n - 2) % entry_len != 0 {
                return Err(ClientError::Protocol);
            }
            let mut last_end = 0u16;
            for chunk in resp[2..n].chunks_exact(entry_len) {
                let start_handle = u16::from_le_bytes([chunk[0], chunk[1]]);
                let end_handle = u16::from_le_bytes([chunk[2], chunk[3]]);
                let Some(uuid) = uuid_from_raw(&chunk[4..entry_len]) else {
                    return Err(ClientError::Protocol);
                };
                if count >= out.len() {
                    return Err(ClientError::Overflow);
                }
                out[count] = Service {
                    start_handle,
                    end_handle,
                    uuid,
                };
                count += 1;
                last_end = end_handle;
            }
            // Continue past the last group; stop cleanly at the handle ceiling.
            if last_end == HANDLE_MAX {
                return Ok(count);
            }
            start = last_end + 1;
        }
    }

    /// Discovers the characteristics within a service's handle range
    /// (`start_handle..=end_handle`, from a [`Service`]), filling `out` and
    /// returning how many were found (a `Read By Type` walk over type
    /// [`UUID_CHARACTERISTIC`]).
    ///
    /// Returns [`ClientError::Overflow`] if there are more than `out` holds.
    pub fn discover_characteristics(
        &mut self,
        bt: &mut Bluetooth,
        start_handle: u16,
        end_handle: u16,
        out: &mut [Characteristic],
        timer: &Timer,
    ) -> Result<usize, ClientError> {
        let mut resp = [0u8; ATT_MAX_MTU as usize];
        let mut count = 0;
        let mut start = start_handle;
        loop {
            if start > end_handle {
                return Ok(count);
            }
            let ct = UUID_CHARACTERISTIC.to_le_bytes();
            let req = [
                OP_READ_BY_TYPE_REQ,
                start.to_le_bytes()[0],
                start.to_le_bytes()[1],
                end_handle.to_le_bytes()[0],
                end_handle.to_le_bytes()[1],
                ct[0],
                ct[1],
            ];
            let n = match self.request(bt, &req, OP_READ_BY_TYPE_RSP, &mut resp, timer) {
                Ok(n) => n,
                Err(ClientError::Att { code, .. }) if code == ERR_ATTRIBUTE_NOT_FOUND => {
                    return Ok(count)
                }
                Err(e) => return Err(e),
            };
            // [opcode, entry_len(1), entries...]; each entry is
            // decl_handle(2), value(entry_len-2), where the characteristic
            // declaration value is properties(1), value_handle(2), uuid(rest).
            if n < 2 {
                return Err(ClientError::Protocol);
            }
            let entry_len = resp[1] as usize;
            if entry_len < 5 || (n - 2) % entry_len != 0 {
                return Err(ClientError::Protocol);
            }
            let mut last_handle = 0u16;
            for chunk in resp[2..n].chunks_exact(entry_len) {
                let decl_handle = u16::from_le_bytes([chunk[0], chunk[1]]);
                let properties = chunk[2];
                let value_handle = u16::from_le_bytes([chunk[3], chunk[4]]);
                let Some(uuid) = uuid_from_raw(&chunk[5..entry_len]) else {
                    return Err(ClientError::Protocol);
                };
                if count >= out.len() {
                    return Err(ClientError::Overflow);
                }
                out[count] = Characteristic {
                    decl_handle,
                    properties,
                    value_handle,
                    uuid,
                };
                count += 1;
                last_handle = decl_handle;
            }
            if last_handle == HANDLE_MAX {
                return Ok(count);
            }
            start = last_handle + 1;
        }
    }

    /// Discovers the descriptors in a handle range (`start_handle..=end_handle`
    /// — typically a characteristic's value handle + 1 through the end of its
    /// characteristic), filling `out` and returning how many were found (a
    /// `Find Information` walk).
    ///
    /// The CCC descriptor a caller writes to subscribe has type
    /// [`UUID_CCC_DESCRIPTOR`](super::gatt::UUID_CCC_DESCRIPTOR); scan the
    /// returned slice for it. Returns [`ClientError::Overflow`] if there are
    /// more than `out` holds.
    pub fn discover_descriptors(
        &mut self,
        bt: &mut Bluetooth,
        start_handle: u16,
        end_handle: u16,
        out: &mut [Descriptor],
        timer: &Timer,
    ) -> Result<usize, ClientError> {
        let mut resp = [0u8; ATT_MAX_MTU as usize];
        let mut count = 0;
        let mut start = start_handle;
        loop {
            if start > end_handle {
                return Ok(count);
            }
            let req = [
                OP_FIND_INFORMATION_REQ,
                start.to_le_bytes()[0],
                start.to_le_bytes()[1],
                end_handle.to_le_bytes()[0],
                end_handle.to_le_bytes()[1],
            ];
            let n = match self.request(bt, &req, OP_FIND_INFORMATION_RSP, &mut resp, timer) {
                Ok(n) => n,
                Err(ClientError::Att { code, .. }) if code == ERR_ATTRIBUTE_NOT_FOUND => {
                    return Ok(count)
                }
                Err(e) => return Err(e),
            };
            // [opcode, format(1), entries...]; format 0x01 => handle(2) +
            // uuid16(2), format 0x02 => handle(2) + uuid128(16).
            if n < 2 {
                return Err(ClientError::Protocol);
            }
            let uuid_len = match resp[1] {
                0x01 => 2,
                0x02 => 16,
                _ => return Err(ClientError::Protocol),
            };
            let entry_len = 2 + uuid_len;
            if (n - 2) % entry_len != 0 {
                return Err(ClientError::Protocol);
            }
            let mut last_handle = 0u16;
            for chunk in resp[2..n].chunks_exact(entry_len) {
                let handle = u16::from_le_bytes([chunk[0], chunk[1]]);
                let Some(uuid) = uuid_from_raw(&chunk[2..entry_len]) else {
                    return Err(ClientError::Protocol);
                };
                if count >= out.len() {
                    return Err(ClientError::Overflow);
                }
                out[count] = Descriptor { handle, uuid };
                count += 1;
                last_handle = handle;
            }
            if last_handle == HANDLE_MAX {
                return Ok(count);
            }
            start = last_handle + 1;
        }
    }

    /// Writes `value` to the attribute at `handle` with a `Write Request` and
    /// waits for the `Write Response`.
    ///
    /// The generic form beneath [`Self::subscribe`]; also usable to write any
    /// writable characteristic (e.g. a gamepad's rumble/output report). The
    /// value is sent whole, so it must fit `mtu - 3` bytes.
    pub fn write(
        &mut self,
        bt: &mut Bluetooth,
        handle: u16,
        value: &[u8],
        timer: &Timer,
    ) -> Result<(), ClientError> {
        let mut req = [0u8; ATT_MAX_MTU as usize];
        req[0] = OP_WRITE_REQ;
        req[1..3].copy_from_slice(&handle.to_le_bytes());
        let vlen = value.len().min(self.mtu as usize - 3);
        req[3..3 + vlen].copy_from_slice(&value[..vlen]);
        let mut resp = [0u8; ATT_MAX_MTU as usize];
        self.request(bt, &req[..3 + vlen], OP_WRITE_RSP, &mut resp, timer)?;
        Ok(())
    }

    /// Subscribes to a characteristic by writing `bits`
    /// ([`CCC_NOTIFY`]/[`CCC_INDICATE`]) to its CCC descriptor at
    /// `ccc_handle` — turning on the server's Handle Value Notifications so
    /// they start arriving at [`Self::feed`]. A thin wrapper over
    /// [`Self::write`].
    pub fn subscribe(
        &mut self,
        bt: &mut Bluetooth,
        ccc_handle: u16,
        bits: u16,
        timer: &Timer,
    ) -> Result<(), ClientError> {
        self.write(bt, ccc_handle, &bits.to_le_bytes(), timer)
    }

    /// Feeds one inbound [`Event::Acl`] fragment (from the caller's own
    /// [`Bluetooth::poll`] loop) into reassembly, returning a [`Notification`]
    /// — with its value copied into `out` — when a Handle Value Notification
    /// completes.
    ///
    /// Returns `None` when the fragment belongs to another connection, doesn't
    /// complete a frame, or completes a non-notification ATT PDU (a stray
    /// response, an indication) — the steady-state loop only cares about
    /// notifications. The value is truncated to `out.len()`.
    pub fn feed(&mut self, acl: &AclData, out: &mut [u8]) -> Option<Notification> {
        if acl.handle != self.handle {
            return None;
        }
        let pdu = self.reasm.feed(acl)?;
        if pdu.cid != CID_ATT {
            return None;
        }
        let payload = pdu.payload;
        // [opcode, value_handle(2), value...].
        if payload.first() != Some(&OP_HANDLE_VALUE_NOTIFICATION) || payload.len() < 3 {
            return None;
        }
        let value_handle = u16::from_le_bytes([payload[1], payload[2]]);
        let value = &payload[3..];
        let len = value.len().min(out.len());
        out[..len].copy_from_slice(&value[..len]);
        Some(Notification { value_handle, len })
    }

    /// Sends one ATT request and pumps [`Bluetooth::poll`] until its response
    /// arrives, copying that response PDU into `resp` and returning its length.
    ///
    /// Only the reply that matches `expect` (or an Error Response naming the
    /// request we sent) ends the wait. Any other ATT PDU that arrives in
    /// between is skipped, because ATT is bidirectional: the peer interleaves
    /// its own asynchronous notifications/indications and — acting as a client
    /// toward us — its own requests/commands (Android sends an Exchange MTU
    /// Request on connect) on the same channel. A disconnect, or the 5 s
    /// per-request deadline, ends the wait with an error.
    fn request(
        &mut self,
        bt: &mut Bluetooth,
        req: &[u8],
        expect: u8,
        resp: &mut [u8],
        timer: &Timer,
    ) -> Result<usize, ClientError> {
        l2cap::send(bt, self.handle, CID_ATT, req)?;

        let deadline = timer.now_micros() + (ATT_REQUEST_TIMEOUT_MS as u64) * 1000;
        loop {
            let now = timer.now_micros();
            if now >= deadline {
                return Err(ClientError::Timeout);
            }
            let remaining_ms = (((deadline - now) / 1000) as u32 + 1).min(POLL_SLICE_MS);
            match bt.poll(timer, remaining_ms)? {
                Some(Event::Acl(acl)) => {
                    if acl.handle != self.handle {
                        continue;
                    }
                    let Some(pdu) = self.reasm.feed(&acl) else {
                        continue;
                    };
                    if pdu.cid != CID_ATT {
                        continue;
                    }
                    let payload = pdu.payload;
                    let Some(&op) = payload.first() else {
                        continue;
                    };
                    match op {
                        OP_ERROR_RESPONSE => {
                            // [opcode, req_opcode, handle(2), error_code]. An
                            // error carries which request it rejects; ignore
                            // one for a different opcode (it answers some other
                            // exchange, not ours).
                            if payload.len() < 5 {
                                return Err(ClientError::Protocol);
                            }
                            if payload[1] != req.first().copied().unwrap_or(0) {
                                continue;
                            }
                            return Err(ClientError::Att {
                                request: payload[1],
                                handle: u16::from_le_bytes([payload[2], payload[3]]),
                                code: payload[4],
                            });
                        }
                        _ if op == expect => {
                            let n = payload.len().min(resp.len());
                            resp[..n].copy_from_slice(&payload[..n]);
                            return Ok(n);
                        }
                        // Anything else is not this request's reply and is
                        // skipped, not an error: ATT is bidirectional, so the
                        // peer legitimately interleaves its own traffic on
                        // CID_ATT — asynchronous notifications/indications
                        // (0x1b/0x1d), and, acting as a *client* toward us, its
                        // own requests/commands (e.g. an Exchange MTU Request,
                        // which Android sends on connect). None of those answer
                        // our request; we keep waiting for the reply that does.
                        // A genuine wrong-opcode desync therefore surfaces as a
                        // Timeout rather than a false Protocol error.
                        _ => continue,
                    }
                }
                Some(Event::Disconnected { reason, .. }) => {
                    return Err(ClientError::Disconnected(reason))
                }
                // Other events (LTK request, encryption change) and quiet
                // windows don't end the wait; keep polling until the deadline.
                Some(_) | None => continue,
            }
        }
    }
}

/// Builds a [`Uuid`] from its raw little-endian wire bytes — 2 bytes for a
/// 16-bit UUID, 16 for a 128-bit one. `None` for any other length.
fn uuid_from_raw(raw: &[u8]) -> Option<Uuid> {
    match raw.len() {
        2 => Some(Uuid::Bit16(u16::from_le_bytes([raw[0], raw[1]]))),
        16 => {
            let mut bytes = [0u8; 16];
            bytes.copy_from_slice(raw);
            Some(Uuid::Bit128(bytes))
        }
        _ => None,
    }
}
