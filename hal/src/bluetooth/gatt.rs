//! A generic GATT server over the ATT protocol (L2CAP channel `CID_ATT`).
//!
//! This is transport- and application-agnostic: it answers the ATT
//! requests a GATT client (a phone, a laptop) sends while discovering and
//! reading a peripheral, against an attribute table **the consumer
//! supplies**. The crate does not bake in any particular device's services
//! — a keyboard, a sensor, and a beacon all build their own `&[Attribute]`
//! table and hand it to [`Server::new`]. The server itself only knows the
//! generic GATT declarations (primary/secondary service, characteristic,
//! descriptor) well enough to walk the table for discovery.
//!
//! # The attribute table
//!
//! A GATT database is a flat, handle-ordered list of [`Attribute`]s. Each
//! has a 16-bit handle, a type [`Uuid`] (what kind of attribute it is), and
//! a value. Services and characteristics are expressed as attributes with
//! well-known type UUIDs and structured values, in the standard layout:
//!
//! ```text
//! handle  type (uuid)            value
//! 0x0001  0x2800 Primary Service   <service UUID>
//! 0x0002  0x2803 Characteristic    <properties> <value handle> <char UUID>
//! 0x0003  <char UUID>              <the characteristic's value>
//! 0x0004  0x2902 CCC Descriptor    <notify/indicate enable bits>
//! ...
//! ```
//!
//! Attributes **must be sorted by ascending handle** — the discovery
//! walks assume it. Handles need not be contiguous but conventionally are.
//! The [`attr`] / [`primary_service`] / [`characteristic`] helpers build the
//! common entries; a characteristic's declaration value must be laid out by
//! the caller (its bytes need a `'static` home the no-alloc model can't
//! synthesize), which [`characteristic`] does from its arguments.
//!
//! # Scope
//!
//! Implements the ATT requests a client uses to discover, read, subscribe,
//! and receive from a server: Exchange MTU, Find Information, Find By Type
//! Value, Read By Type, Read By Group Type, Read, Read Blob (for values too
//! long for one response, like a HID Report Map), and Write (Request and
//! Command). A client subscribes by writing a characteristic's CCC
//! descriptor ([`UUID_CCC_DESCRIPTOR`]); the server tracks that state and
//! [`Server::notification`] builds a Handle Value Notification for a
//! subscribed characteristic to push a new value. Writes to attributes
//! other than CCC descriptors are answered `Write Not Permitted` (there's no
//! mutable backing store for arbitrary characteristic values yet). Anything
//! unrecognized gets a spec-compliant ATT Error Response, so a client always
//! makes forward progress rather than stalling.

/// ATT opcode `Error Response`.
const OP_ERROR_RESPONSE: u8 = 0x01;
/// ATT opcode `Exchange MTU Request`.
const OP_EXCHANGE_MTU_REQ: u8 = 0x02;
/// ATT opcode `Exchange MTU Response`.
const OP_EXCHANGE_MTU_RSP: u8 = 0x03;
/// ATT opcode `Find Information Request`.
const OP_FIND_INFORMATION_REQ: u8 = 0x04;
/// ATT opcode `Find Information Response`.
const OP_FIND_INFORMATION_RSP: u8 = 0x05;
/// ATT opcode `Find By Type Value Request`.
const OP_FIND_BY_TYPE_VALUE_REQ: u8 = 0x06;
/// ATT opcode `Find By Type Value Response`.
const OP_FIND_BY_TYPE_VALUE_RSP: u8 = 0x07;
/// ATT opcode `Read By Type Request`.
const OP_READ_BY_TYPE_REQ: u8 = 0x08;
/// ATT opcode `Read By Type Response`.
const OP_READ_BY_TYPE_RSP: u8 = 0x09;
/// ATT opcode `Read Request`.
const OP_READ_REQ: u8 = 0x0a;
/// ATT opcode `Read Response`.
const OP_READ_RSP: u8 = 0x0b;
/// ATT opcode `Read Blob Request` — reads a long attribute value from an
/// offset, for values that don't fit one Read Response.
const OP_READ_BLOB_REQ: u8 = 0x0c;
/// ATT opcode `Read Blob Response`.
const OP_READ_BLOB_RSP: u8 = 0x0d;
/// ATT opcode `Read By Group Type Request`.
const OP_READ_BY_GROUP_TYPE_REQ: u8 = 0x10;
/// ATT opcode `Read By Group Type Response`.
const OP_READ_BY_GROUP_TYPE_RSP: u8 = 0x11;
/// ATT opcode `Write Request` (client wants a Write Response).
const OP_WRITE_REQ: u8 = 0x12;
/// ATT opcode `Write Response`.
const OP_WRITE_RSP: u8 = 0x13;
/// ATT opcode `Write Command` (fire-and-forget; no response).
const OP_WRITE_CMD: u8 = 0x52;
/// ATT opcode `Handle Value Notification` — a server-initiated push of a
/// characteristic value to a subscribed client.
const OP_HANDLE_VALUE_NOTIFICATION: u8 = 0x1b;
/// ATT `Command` opcode flag (bit 6): set on client commands, which expect
/// no response.
const OP_COMMAND_FLAG: u8 = 0x40;

/// CCC descriptor bit enabling notifications for its characteristic.
const CCC_NOTIFY: u16 = 0x0001;

/// ATT error code `Invalid Handle`.
const ERR_INVALID_HANDLE: u8 = 0x01;
/// ATT error code `Write Not Permitted`.
const ERR_WRITE_NOT_PERMITTED: u8 = 0x03;
/// ATT error code `Read Not Permitted`.
const ERR_READ_NOT_PERMITTED: u8 = 0x02;
/// ATT error code `Request Not Supported`.
const ERR_REQUEST_NOT_SUPPORTED: u8 = 0x06;
/// ATT error code `Attribute Not Found`.
const ERR_ATTRIBUTE_NOT_FOUND: u8 = 0x0a;
/// ATT error code `Invalid Offset` — a Read Blob offset past the value's
/// length.
const ERR_INVALID_OFFSET: u8 = 0x07;

/// GATT declaration UUID `Primary Service` (`0x2800`).
pub const UUID_PRIMARY_SERVICE: u16 = 0x2800;
/// GATT declaration UUID `Secondary Service` (`0x2801`).
pub const UUID_SECONDARY_SERVICE: u16 = 0x2801;
/// GATT declaration UUID `Characteristic` (`0x2803`).
pub const UUID_CHARACTERISTIC: u16 = 0x2803;
/// GATT descriptor UUID `Client Characteristic Configuration` (`0x2902`) —
/// the CCC descriptor a client writes to enable notifications/indications.
pub const UUID_CCC_DESCRIPTOR: u16 = 0x2902;

/// Characteristic property bit `Read` — the value may be read.
pub const CHAR_PROP_READ: u8 = 0x02;
/// Characteristic property bit `Write Without Response`.
pub const CHAR_PROP_WRITE_NO_RESPONSE: u8 = 0x04;
/// Characteristic property bit `Write` — the value may be written with a
/// Write Request.
pub const CHAR_PROP_WRITE: u8 = 0x08;
/// Characteristic property bit `Notify` — the server may push the value via
/// a Handle Value Notification.
pub const CHAR_PROP_NOTIFY: u8 = 0x10;

/// The ATT default MTU (23), the size in effect before any Exchange MTU.
pub const ATT_DEFAULT_MTU: u16 = 23;
/// The largest ATT MTU this server advertises and accepts. Bounds the
/// response buffer a caller must provide to [`Server::handle`].
pub const ATT_MAX_MTU: u16 = 247;

/// How many distinct CCC-descriptor subscriptions a [`Server`] tracks at
/// once. A client enabling notifications on more than this many
/// characteristics has the excess silently dropped — comfortably above what
/// a fixed-purpose peripheral (a keyboard, a sensor) exposes.
pub const MAX_SUBSCRIPTIONS: usize = 8;

/// A Bluetooth UUID: 16-bit (an assigned/SIG UUID) or a full 128-bit
/// (a vendor-defined one). Stored and compared in little-endian wire order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Uuid {
    /// A 16-bit assigned UUID (e.g. `0x2800`, `0x2A00`).
    Bit16(u16),
    /// A 128-bit UUID, in little-endian wire order (LSB first).
    Bit128([u8; 16]),
}

impl Uuid {
    /// The UUID's width on the wire, in bytes (2 or 16).
    pub const fn len(&self) -> usize {
        match self {
            Uuid::Bit16(_) => 2,
            Uuid::Bit128(_) => 16,
        }
    }

    /// Returns `true` if the UUID has no bytes — never, for a valid UUID;
    /// present because Clippy expects `len` to have an `is_empty` companion.
    pub const fn is_empty(&self) -> bool {
        false
    }

    /// Writes the UUID into `out` in little-endian wire order, returning the
    /// number of bytes written ([`Self::len`]). `out` must have room.
    fn write(&self, out: &mut [u8]) -> usize {
        match self {
            Uuid::Bit16(v) => {
                out[..2].copy_from_slice(&v.to_le_bytes());
                2
            }
            Uuid::Bit128(bytes) => {
                out[..16].copy_from_slice(bytes);
                16
            }
        }
    }

    /// Returns `true` if this UUID equals the raw little-endian bytes from a
    /// request's type field (2 or 16 bytes).
    fn equals_raw(&self, raw: &[u8]) -> bool {
        match self {
            Uuid::Bit16(v) => raw.len() == 2 && raw == v.to_le_bytes(),
            Uuid::Bit128(bytes) => raw.len() == 16 && raw == bytes,
        }
    }
}

/// One entry in a GATT attribute table: a handle, a type [`Uuid`], and a
/// value, plus whether reads are permitted. Build these with [`attr`],
/// [`primary_service`], and [`characteristic`], or construct directly.
#[derive(Clone, Copy, Debug)]
pub struct Attribute<'a> {
    /// The attribute handle (unique, and the table must be sorted ascending
    /// by it).
    pub handle: u16,
    /// The attribute type — a GATT declaration UUID for a service or
    /// characteristic entry, or the characteristic's own UUID for its value
    /// entry.
    pub uuid: Uuid,
    /// The attribute value bytes.
    pub value: &'a [u8],
    /// Whether a client may read this attribute's value. Service and
    /// characteristic *declarations* are always readable; a value attribute
    /// follows its characteristic's properties.
    pub readable: bool,
}

/// Builds a readable attribute with a 16-bit type UUID and the given value —
/// e.g. a characteristic value entry, or a descriptor.
pub const fn attr(handle: u16, uuid16: u16, value: &[u8]) -> Attribute<'_> {
    Attribute {
        handle,
        uuid: Uuid::Bit16(uuid16),
        value,
        readable: true,
    }
}

/// Builds a Primary Service declaration attribute: type `0x2800`, value the
/// service's UUID bytes (`uuid_bytes` in little-endian wire order — 2 bytes
/// for a 16-bit service UUID, 16 for a 128-bit one).
pub const fn primary_service(handle: u16, uuid_bytes: &[u8]) -> Attribute<'_> {
    Attribute {
        handle,
        uuid: Uuid::Bit16(UUID_PRIMARY_SERVICE),
        value: uuid_bytes,
        readable: true,
    }
}

/// Builds a Characteristic declaration attribute: type `0x2803`, value
/// `[properties, value_handle(2, LE), char_uuid…]`. The caller supplies the
/// already-assembled `decl` bytes (via a `const` array), since the no-alloc
/// model can't synthesize the value slice here; this just tags it with the
/// declaration type and handle.
pub const fn characteristic(handle: u16, decl: &[u8]) -> Attribute<'_> {
    Attribute {
        handle,
        uuid: Uuid::Bit16(UUID_CHARACTERISTIC),
        value: decl,
        readable: true,
    }
}

/// Builds a Client Characteristic Configuration descriptor attribute (type
/// `0x2902`), the descriptor a client writes to enable notifications. Its
/// stored value is a placeholder — the [`Server`] tracks live subscription
/// state itself and serves reads of the CCC from that, not from here.
pub const fn cccd(handle: u16) -> Attribute<'static> {
    Attribute {
        handle,
        uuid: Uuid::Bit16(UUID_CCC_DESCRIPTOR),
        value: &[0, 0],
        readable: true,
    }
}

/// A generic GATT server: answers ATT requests against a borrowed attribute
/// table. Holds only the negotiated MTU as mutable state; the table itself
/// is immutable.
///
/// Feed each inbound ATT PDU (the payload of an L2CAP frame on
/// `CID_ATT`) to [`Self::handle`], and send back the response it writes.
pub struct Server<'a> {
    /// The attribute table, sorted ascending by handle.
    attributes: &'a [Attribute<'a>],
    /// The ATT MTU currently in effect (starts at [`ATT_DEFAULT_MTU`], is
    /// raised by an Exchange MTU exchange).
    mtu: u16,
    /// Client Characteristic Configuration state: for each CCC descriptor a
    /// client has written, its handle and the 2-byte value (notify/indicate
    /// enable bits). A `handle` of 0 marks an unused slot. This is the only
    /// mutable per-connection state; the attribute table stays immutable.
    subscriptions: [(u16, u16); MAX_SUBSCRIPTIONS],
}

impl<'a> Server<'a> {
    /// Creates a server over `attributes`, which must be sorted ascending by
    /// handle. The MTU starts at [`ATT_DEFAULT_MTU`] until a client
    /// negotiates a larger one, and no characteristics are subscribed.
    pub fn new(attributes: &'a [Attribute<'a>]) -> Self {
        Self {
            attributes,
            mtu: ATT_DEFAULT_MTU,
            subscriptions: [(0, 0); MAX_SUBSCRIPTIONS],
        }
    }

    /// The ATT MTU currently in effect.
    pub fn mtu(&self) -> u16 {
        self.mtu
    }

    /// Resets per-connection state (negotiated MTU and all subscriptions) to
    /// defaults — call on disconnect, before the next client connects.
    pub fn reset(&mut self) {
        self.mtu = ATT_DEFAULT_MTU;
        self.subscriptions = [(0, 0); MAX_SUBSCRIPTIONS];
    }

    /// Handles one inbound ATT PDU, writing the response into `out` and
    /// returning its length, or `None` if no response is warranted (an ATT
    /// command, which the caller need not answer).
    ///
    /// `out` should be at least [`ATT_MAX_MTU`] bytes; the response is capped
    /// to the negotiated MTU regardless. The returned bytes are a complete
    /// ATT PDU to hand to `l2cap::send` on the `CID_ATT` channel.
    pub fn handle(&mut self, request: &[u8], out: &mut [u8]) -> Option<usize> {
        let &opcode = request.first()?;

        // Cap the working response window at the negotiated MTU.
        let limit = out.len().min(self.mtu as usize);
        let out = &mut out[..limit];

        let n = match opcode {
            OP_EXCHANGE_MTU_REQ => self.exchange_mtu(request, out),
            OP_FIND_INFORMATION_REQ => self.find_information(request, out),
            OP_FIND_BY_TYPE_VALUE_REQ => self.find_by_type_value(request, out),
            OP_READ_BY_TYPE_REQ => self.read_by_type(request, out),
            OP_READ_BY_GROUP_TYPE_REQ => self.read_by_group_type(request, out),
            OP_READ_REQ => self.read(request, out),
            OP_READ_BLOB_REQ => self.read_blob(request, out),
            OP_WRITE_REQ => self.write(request, out),
            OP_WRITE_CMD => {
                // Fire-and-forget: apply the write, send nothing back.
                self.apply_write(request);
                return None;
            }
            _ if opcode & OP_COMMAND_FLAG != 0 => {
                // A command (e.g. Signed Write) expects no response.
                return None;
            }
            // A client-sent Error Response (unusual) needs no reply.
            OP_ERROR_RESPONSE => return None,
            // Anything else: a spec-compliant error so the client moves on.
            _ => error_response(opcode, 0x0000, ERR_REQUEST_NOT_SUPPORTED, out),
        };
        Some(n)
    }

    /// Exchange MTU: adopt the smaller of the client's MTU and ours, and
    /// report ours back.
    fn exchange_mtu(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        if req.len() < 3 {
            return error_response(OP_EXCHANGE_MTU_REQ, 0x0000, ERR_INVALID_HANDLE, out);
        }
        let client_mtu = u16::from_le_bytes([req[1], req[2]]);
        self.mtu = client_mtu.clamp(ATT_DEFAULT_MTU, ATT_MAX_MTU);
        out[0] = OP_EXCHANGE_MTU_RSP;
        out[1..3].copy_from_slice(&ATT_MAX_MTU.to_le_bytes());
        3
    }

    /// Find Information: list `(handle, type UUID)` for every attribute in
    /// the requested handle range — descriptor discovery.
    fn find_information(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        let Some((start, end)) = handle_range(req) else {
            return error_response(OP_FIND_INFORMATION_REQ, 0x0000, ERR_INVALID_HANDLE, out);
        };

        // Format is decided by the first matching attribute's UUID width and
        // all entries in one response must share it (0x01 = 16-bit, 0x02 =
        // 128-bit).
        let mut written = 0;
        let mut format = 0u8;
        for a in self.in_range(start, end) {
            let uuid_len = a.uuid.len();
            let this_format = if uuid_len == 2 { 0x01 } else { 0x02 };
            if written == 0 {
                format = this_format;
                out[0] = OP_FIND_INFORMATION_RSP;
                out[1] = format;
                written = 2;
            } else if this_format != format {
                break;
            }
            let entry = 2 + uuid_len;
            if written + entry > out.len() {
                break;
            }
            out[written..written + 2].copy_from_slice(&a.handle.to_le_bytes());
            a.uuid.write(&mut out[written + 2..]);
            written += entry;
        }

        if written == 0 {
            return error_response(OP_FIND_INFORMATION_REQ, start, ERR_ATTRIBUTE_NOT_FOUND, out);
        }
        written
    }

    /// Find By Type Value: find the attributes in range whose type and value
    /// match, returning `(found_handle, group_end_handle)` pairs — used to
    /// discover a service by its UUID (type `0x2800`, value the service
    /// UUID).
    fn find_by_type_value(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        // [opcode, start(2), end(2), type(2), value(N)]
        if req.len() < 7 {
            return error_response(OP_FIND_BY_TYPE_VALUE_REQ, 0x0000, ERR_INVALID_HANDLE, out);
        }
        let start = u16::from_le_bytes([req[1], req[2]]);
        let end = u16::from_le_bytes([req[3], req[4]]);
        let type_uuid = u16::from_le_bytes([req[5], req[6]]);
        let value = &req[7..];

        let mut written = 0;
        for (i, a) in self.attributes.iter().enumerate() {
            if a.handle < start || a.handle > end {
                continue;
            }
            if !a.uuid.equals_raw(&type_uuid.to_le_bytes()) || a.value != value {
                continue;
            }
            if written == 0 {
                out[0] = OP_FIND_BY_TYPE_VALUE_RSP;
                written = 1;
            }
            if written + 4 > out.len() {
                break;
            }
            let group_end = self.group_end_handle(i);
            out[written..written + 2].copy_from_slice(&a.handle.to_le_bytes());
            out[written + 2..written + 4].copy_from_slice(&group_end.to_le_bytes());
            written += 4;
        }

        if written == 0 {
            return error_response(
                OP_FIND_BY_TYPE_VALUE_REQ,
                start,
                ERR_ATTRIBUTE_NOT_FOUND,
                out,
            );
        }
        written
    }

    /// Read By Type: list `(handle, value)` for attributes in range whose
    /// type matches — characteristic discovery (type `0x2803`) and reading a
    /// value by type (e.g. Device Name `0x2A00`). All returned values share
    /// one length, per the ATT rule.
    fn read_by_type(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        let Some((start, end, type_raw)) = handle_range_and_type(req) else {
            return error_response(OP_READ_BY_TYPE_REQ, 0x0000, ERR_INVALID_HANDLE, out);
        };

        let mut written = 0;
        let mut entry_len = 0usize;
        for a in self.in_range(start, end) {
            if !a.uuid.equals_raw(type_raw) {
                continue;
            }
            if !a.readable {
                // First matching attribute is unreadable: report it.
                if written == 0 {
                    return error_response(
                        OP_READ_BY_TYPE_REQ,
                        a.handle,
                        ERR_READ_NOT_PERMITTED,
                        out,
                    );
                }
                break;
            }
            // Value length is fixed by the first match (truncated to MTU);
            // stop at the first differently-sized value.
            let vlen = a.value.len().min(self.mtu as usize - 4);
            if written == 0 {
                entry_len = 2 + vlen;
                out[0] = OP_READ_BY_TYPE_RSP;
                out[1] = entry_len as u8;
                written = 2;
            } else if 2 + vlen != entry_len {
                break;
            }
            if written + entry_len > out.len() {
                break;
            }
            out[written..written + 2].copy_from_slice(&a.handle.to_le_bytes());
            out[written + 2..written + entry_len].copy_from_slice(&a.value[..vlen]);
            written += entry_len;
        }

        if written == 0 {
            return error_response(OP_READ_BY_TYPE_REQ, start, ERR_ATTRIBUTE_NOT_FOUND, out);
        }
        written
    }

    /// Read By Group Type: list `(handle, group_end, value)` for service
    /// declarations in range — primary/secondary service discovery (type
    /// `0x2800`/`0x2801`).
    fn read_by_group_type(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        let Some((start, end, type_raw)) = handle_range_and_type(req) else {
            return error_response(OP_READ_BY_GROUP_TYPE_REQ, 0x0000, ERR_INVALID_HANDLE, out);
        };

        let mut written = 0;
        let mut entry_len = 0usize;
        for i in 0..self.attributes.len() {
            let a = &self.attributes[i];
            if a.handle < start || a.handle > end || !a.uuid.equals_raw(type_raw) {
                continue;
            }
            let vlen = a.value.len().min(self.mtu as usize - 6);
            if written == 0 {
                entry_len = 4 + vlen;
                out[0] = OP_READ_BY_GROUP_TYPE_RSP;
                out[1] = entry_len as u8;
                written = 2;
            } else if 4 + vlen != entry_len {
                break;
            }
            if written + entry_len > out.len() {
                break;
            }
            let group_end = self.group_end_handle(i);
            out[written..written + 2].copy_from_slice(&a.handle.to_le_bytes());
            out[written + 2..written + 4].copy_from_slice(&group_end.to_le_bytes());
            out[written + 4..written + entry_len].copy_from_slice(&a.value[..vlen]);
            written += entry_len;
        }

        if written == 0 {
            return error_response(
                OP_READ_BY_GROUP_TYPE_REQ,
                start,
                ERR_ATTRIBUTE_NOT_FOUND,
                out,
            );
        }
        written
    }

    /// Read: return one attribute's value by handle (truncated to the MTU).
    fn read(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        if req.len() < 3 {
            return error_response(OP_READ_REQ, 0x0000, ERR_INVALID_HANDLE, out);
        }
        let handle = u16::from_le_bytes([req[1], req[2]]);
        let Some(a) = self.attributes.iter().find(|a| a.handle == handle) else {
            return error_response(OP_READ_REQ, handle, ERR_INVALID_HANDLE, out);
        };
        if !a.readable {
            return error_response(OP_READ_REQ, handle, ERR_READ_NOT_PERMITTED, out);
        }
        out[0] = OP_READ_RSP;
        // A CCC descriptor reads back its live subscription state, not the
        // static placeholder value in the table.
        if is_ccc(&a.uuid) {
            let value = self.subscription(handle).to_le_bytes();
            let vlen = value.len().min(out.len() - 1);
            out[1..1 + vlen].copy_from_slice(&value[..vlen]);
            return 1 + vlen;
        }
        let vlen = a.value.len().min(out.len() - 1);
        out[1..1 + vlen].copy_from_slice(&a.value[..vlen]);
        1 + vlen
    }

    /// Read Blob: return one attribute's value from a byte offset (for a long
    /// value read across multiple responses — e.g. a HID Report Map larger
    /// than one Read Response).
    fn read_blob(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        if req.len() < 5 {
            return error_response(OP_READ_BLOB_REQ, 0x0000, ERR_INVALID_HANDLE, out);
        }
        let handle = u16::from_le_bytes([req[1], req[2]]);
        let offset = u16::from_le_bytes([req[3], req[4]]) as usize;
        let Some(a) = self.attributes.iter().find(|a| a.handle == handle) else {
            return error_response(OP_READ_BLOB_REQ, handle, ERR_INVALID_HANDLE, out);
        };
        if !a.readable {
            return error_response(OP_READ_BLOB_REQ, handle, ERR_READ_NOT_PERMITTED, out);
        }
        if offset > a.value.len() {
            return error_response(OP_READ_BLOB_REQ, handle, ERR_INVALID_OFFSET, out);
        }
        out[0] = OP_READ_BLOB_RSP;
        let vlen = (a.value.len() - offset).min(out.len() - 1);
        out[1..1 + vlen].copy_from_slice(&a.value[offset..offset + vlen]);
        1 + vlen
    }

    /// Write Request: apply the write and answer with a Write Response, or a
    /// spec-compliant error if the target can't be written.
    fn write(&mut self, req: &[u8], out: &mut [u8]) -> usize {
        if req.len() < 3 {
            return error_response(OP_WRITE_REQ, 0x0000, ERR_INVALID_HANDLE, out);
        }
        let handle = u16::from_le_bytes([req[1], req[2]]);
        match self.apply_write(req) {
            WriteOutcome::Ok => {
                out[0] = OP_WRITE_RSP;
                1
            }
            WriteOutcome::InvalidHandle => {
                error_response(OP_WRITE_REQ, handle, ERR_INVALID_HANDLE, out)
            }
            WriteOutcome::NotPermitted => {
                error_response(OP_WRITE_REQ, handle, ERR_WRITE_NOT_PERMITTED, out)
            }
        }
    }

    /// Applies a write request/command to an attribute, shared by
    /// [`Self::write`] and the Write Command path.
    ///
    /// Only CCC descriptors are writable today — a client enabling/disabling
    /// notifications. The new value is recorded in [`Self::subscriptions`].
    /// Writes to any other attribute report [`WriteOutcome::NotPermitted`]
    /// (there's no mutable backing store for arbitrary characteristic
    /// values yet), and an unknown handle reports
    /// [`WriteOutcome::InvalidHandle`].
    fn apply_write(&mut self, req: &[u8]) -> WriteOutcome {
        if req.len() < 3 {
            return WriteOutcome::InvalidHandle;
        }
        let handle = u16::from_le_bytes([req[1], req[2]]);
        let value = &req[3..];
        let Some(a) = self.attributes.iter().find(|a| a.handle == handle) else {
            return WriteOutcome::InvalidHandle;
        };
        if !is_ccc(&a.uuid) {
            return WriteOutcome::NotPermitted;
        }
        let bits = u16::from_le_bytes([
            value.first().copied().unwrap_or(0),
            value.get(1).copied().unwrap_or(0),
        ]);
        self.set_subscription(handle, bits);
        WriteOutcome::Ok
    }

    /// Records a CCC descriptor's subscription value, updating the existing
    /// slot for `ccc_handle` or claiming a free one. Silently drops the
    /// write if all [`MAX_SUBSCRIPTIONS`] slots are taken by other CCCs.
    fn set_subscription(&mut self, ccc_handle: u16, value: u16) {
        if let Some(slot) = self
            .subscriptions
            .iter_mut()
            .find(|(h, _)| *h == ccc_handle)
        {
            slot.1 = value;
            return;
        }
        if let Some(slot) = self.subscriptions.iter_mut().find(|(h, _)| *h == 0) {
            *slot = (ccc_handle, value);
        }
    }

    /// The current subscription value for a CCC descriptor handle, or `0`
    /// (no bits set) if the client never wrote it.
    fn subscription(&self, ccc_handle: u16) -> u16 {
        self.subscriptions
            .iter()
            .find(|(h, _)| *h == ccc_handle)
            .map(|(_, v)| *v)
            .unwrap_or(0)
    }

    /// The handle of the CCC descriptor belonging to the characteristic
    /// whose value is at `value_handle`: the first `0x2902` descriptor after
    /// the value attribute, before the next characteristic or service
    /// declaration. `None` if the characteristic has no CCC descriptor.
    fn ccc_handle_for_value(&self, value_handle: u16) -> Option<u16> {
        let start = self
            .attributes
            .iter()
            .position(|a| a.handle == value_handle)?;
        for a in &self.attributes[start + 1..] {
            if is_service_decl(&a.uuid) || is_characteristic_decl(&a.uuid) {
                break;
            }
            if is_ccc(&a.uuid) {
                return Some(a.handle);
            }
        }
        None
    }

    /// Returns `true` if a client has enabled notifications on the
    /// characteristic whose value is at `value_handle` (wrote its CCC
    /// descriptor's notify bit).
    pub fn is_subscribed(&self, value_handle: u16) -> bool {
        self.ccc_handle_for_value(value_handle)
            .map(|ccc| self.subscription(ccc) & CCC_NOTIFY != 0)
            .unwrap_or(false)
    }

    /// Marks the characteristic whose value is at `value_handle` as
    /// notify-subscribed, as if the client had written its CCC descriptor.
    /// Returns `false` if the characteristic has no CCC descriptor.
    ///
    /// Use it to restore a bonded client's subscription on reconnect: a HOGP
    /// host expects the server to persist the CCC configuration across a
    /// bond and may not re-write it, so without this the server would never
    /// notify a reconnected keyboard.
    pub fn subscribe(&mut self, value_handle: u16) -> bool {
        match self.ccc_handle_for_value(value_handle) {
            Some(ccc) => {
                self.set_subscription(ccc, CCC_NOTIFY);
                true
            }
            None => false,
        }
    }

    /// Builds a Handle Value Notification for `value_handle` carrying
    /// `value`, into `out`, returning its length — or `None` if the client
    /// hasn't subscribed to notifications for that characteristic (nothing
    /// should be sent).
    ///
    /// The value is truncated to the negotiated MTU. Send the returned bytes
    /// on the `CID_ATT` channel like any other ATT PDU; a notification is
    /// unacknowledged, so there's no response to wait for.
    pub fn notification(&self, value_handle: u16, value: &[u8], out: &mut [u8]) -> Option<usize> {
        if !self.is_subscribed(value_handle) {
            return None;
        }
        let limit = out.len().min(self.mtu as usize);
        out[0] = OP_HANDLE_VALUE_NOTIFICATION;
        out[1..3].copy_from_slice(&value_handle.to_le_bytes());
        let vlen = value.len().min(limit.saturating_sub(3));
        out[3..3 + vlen].copy_from_slice(&value[..vlen]);
        Some(3 + vlen)
    }

    /// Returns the attributes in `[start, end]` (inclusive), in handle order.
    fn in_range(&self, start: u16, end: u16) -> impl Iterator<Item = &Attribute<'a>> {
        self.attributes
            .iter()
            .filter(move |a| a.handle >= start && a.handle <= end)
    }

    /// The group-end handle for the service declaration at index `service`:
    /// the handle of the last attribute before the next service declaration,
    /// or the last attribute in the table. Assumes handle-ordered
    /// attributes.
    fn group_end_handle(&self, service: usize) -> u16 {
        let mut end = self.attributes[service].handle;
        for a in &self.attributes[service + 1..] {
            if is_service_decl(&a.uuid) {
                break;
            }
            end = a.handle;
        }
        end
    }
}

/// The result of applying a write to an attribute.
enum WriteOutcome {
    /// The write was accepted.
    Ok,
    /// No attribute has the target handle.
    InvalidHandle,
    /// The attribute exists but isn't writable.
    NotPermitted,
}

/// Returns `true` if a type UUID is a service declaration (primary or
/// secondary) — the boundary between attribute groups.
fn is_service_decl(uuid: &Uuid) -> bool {
    match uuid {
        Uuid::Bit16(v) => *v == UUID_PRIMARY_SERVICE || *v == UUID_SECONDARY_SERVICE,
        Uuid::Bit128(_) => false,
    }
}

/// Returns `true` if a type UUID is a characteristic declaration
/// (`0x2803`) — the boundary between characteristics within a service.
fn is_characteristic_decl(uuid: &Uuid) -> bool {
    matches!(uuid, Uuid::Bit16(v) if *v == UUID_CHARACTERISTIC)
}

/// Returns `true` if a type UUID is a Client Characteristic Configuration
/// descriptor (`0x2902`).
fn is_ccc(uuid: &Uuid) -> bool {
    matches!(uuid, Uuid::Bit16(v) if *v == UUID_CCC_DESCRIPTOR)
}

/// Parses `[opcode, start(2), end(2)]`, returning the handle range.
fn handle_range(req: &[u8]) -> Option<(u16, u16)> {
    if req.len() < 5 {
        return None;
    }
    Some((
        u16::from_le_bytes([req[1], req[2]]),
        u16::from_le_bytes([req[3], req[4]]),
    ))
}

/// Parses `[opcode, start(2), end(2), type(2 or 16)]`, returning the handle
/// range and the raw type-UUID bytes.
fn handle_range_and_type(req: &[u8]) -> Option<(u16, u16, &[u8])> {
    if req.len() < 7 {
        return None;
    }
    let start = u16::from_le_bytes([req[1], req[2]]);
    let end = u16::from_le_bytes([req[3], req[4]]);
    let type_raw = &req[5..];
    if type_raw.len() != 2 && type_raw.len() != 16 {
        return None;
    }
    Some((start, end, type_raw))
}

/// Writes an ATT Error Response `[0x01, req_opcode, handle(2), error]` into
/// `out`, returning its length (always 5).
fn error_response(req_opcode: u8, handle: u16, error: u8, out: &mut [u8]) -> usize {
    out[0] = OP_ERROR_RESPONSE;
    out[1] = req_opcode;
    out[2..4].copy_from_slice(&handle.to_le_bytes());
    out[4] = error;
    5
}
