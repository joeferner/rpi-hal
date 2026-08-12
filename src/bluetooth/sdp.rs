//! A minimal Bluetooth Classic SDP (Service Discovery Protocol) client — just
//! enough to read a HID device's **report descriptor** from its service
//! record.
//!
//! SDP is how a Classic device advertises its services and their attributes.
//! It runs over its own L2CAP channel (PSM [`PSM_SDP`], `0x0001`), reached
//! through [`bredr::L2cap`](super::bredr::L2cap). This issues one
//! `ServiceSearchAttributeRequest` for the HID service class
//! ([`HID_SERVICE_CLASS`]) asking for the HID Descriptor List attribute
//! ([`ATTR_HID_DESCRIPTOR_LIST`]), reassembles the response across SDP
//! continuations, and digs the Report descriptor
//! ([`HID_DESCRIPTOR_TYPE_REPORT`]) out of the nested data elements.
//!
//! The returned bytes are a standard **HID Report Descriptor** — the same
//! self-describing report layout an OS parses, so it feeds a transport-agnostic
//! descriptor parser rather than a hand-written per-device decoder.
//!
//! # Byte order
//!
//! Unlike HCI and L2CAP (little-endian), SDP is **big-endian** — its PDU header
//! fields and data-element values are network byte order. Both are handled
//! here.

use super::bredr::{L2cap, L2capError};
use super::Bluetooth;
use crate::timer::Timer;

/// L2CAP PSM for the SDP server.
pub const PSM_SDP: u16 = 0x0001;

/// SDP PDU ID `ServiceSearchAttributeRequest`.
const PDU_SSA_REQUEST: u8 = 0x06;
/// SDP PDU ID `ServiceSearchAttributeResponse`.
const PDU_SSA_RESPONSE: u8 = 0x07;
/// SDP PDU ID `ErrorResponse`.
const PDU_ERROR_RESPONSE: u8 = 0x01;

/// The 16-bit service class UUID for `HumanInterfaceDeviceService`.
pub const HID_SERVICE_CLASS: u16 = 0x1124;
/// SDP attribute ID `HIDDescriptorList` — the HID report/physical descriptors.
pub const ATTR_HID_DESCRIPTOR_LIST: u16 = 0x0206;
/// HID class-descriptor type `Report` — the report descriptor entry within the
/// [`ATTR_HID_DESCRIPTOR_LIST`].
pub const HID_DESCRIPTOR_TYPE_REPORT: u8 = 0x22;

/// Data-element type code `Unsigned Integer`.
const DE_UINT: u8 = 1;
/// Data-element type code `UUID`.
const DE_UUID: u8 = 3;
/// Data-element type code `Text String`.
const DE_STRING: u8 = 4;
/// Data-element type code `Sequence`.
const DE_SEQUENCE: u8 = 6;

/// Max attribute bytes we let the server return per response — kept below the
/// L2CAP reassembly limit so a response is always one reassembled frame; larger
/// records arrive across SDP continuations.
const MAX_ATTR_BYTES: u16 = 0x0190;
/// How long to allow the SDP channel to open, in ms.
const OPEN_TIMEOUT_MS: u32 = 8_000;
/// How long to wait for each SDP response, in ms.
const RESPONSE_TIMEOUT_MS: u32 = 5_000;
/// Buffer for one reassembled SDP response frame.
const RESPONSE_BUF: usize = 512;
/// Buffer accumulating attribute-list bytes across continuations.
const ATTR_ACC: usize = 1024;
/// Largest SDP continuation-state blob.
const CONT_MAX: usize = 17;

/// Errors from the SDP client.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SdpError {
    /// The underlying L2CAP layer failed.
    L2cap(L2capError),
    /// No SDP response arrived in time.
    Timeout,
    /// The response was malformed, or the server returned an Error Response.
    Protocol,
    /// The HID service or its Report descriptor wasn't found in the record.
    NotFound,
    /// The accumulated response exceeded the local buffer.
    TooLong,
}

impl From<L2capError> for SdpError {
    fn from(e: L2capError) -> Self {
        SdpError::L2cap(e)
    }
}

/// Reads the device's HID **Report Descriptor** over SDP, copying it into `out`
/// and returning its length.
///
/// Opens an SDP channel on `handle` (an established, paired ACL connection),
/// queries the HID service's descriptor list, and extracts the Report
/// descriptor. The result is a standard HID report descriptor ready for a
/// descriptor parser.
pub fn read_report_descriptor(
    bt: &mut Bluetooth,
    handle: u16,
    timer: &Timer,
    out: &mut [u8],
) -> Result<usize, SdpError> {
    let mut l2 = L2cap::new(handle);
    let cid = l2.open(bt, PSM_SDP, timer, OPEN_TIMEOUT_MS)?;

    let result = query_descriptor(&mut l2, bt, cid, timer, out);

    // Tear the SDP channel down cleanly so its CID returns to the pool — the
    // next profile on this link (HID) reuses the same dynamic CID, which a
    // spec-compliant peer would reject if the SDP channel were left dangling.
    let _ = l2.close(bt, cid, timer);
    result
}

/// Runs the `ServiceSearchAttributeRequest` exchange (across SDP
/// continuations) on an already-open channel `cid`, extracting the HID Report
/// descriptor into `out`.
fn query_descriptor(
    l2: &mut L2cap,
    bt: &mut Bluetooth,
    cid: u16,
    timer: &Timer,
    out: &mut [u8],
) -> Result<usize, SdpError> {
    let mut attr = [0u8; ATTR_ACC];
    let mut attr_len = 0usize;
    let mut cont = [0u8; CONT_MAX];
    let mut cont_len = 0usize;
    let mut tid: u16 = 1;
    let mut resp = [0u8; RESPONSE_BUF];

    loop {
        // Build and send the request with the current continuation state.
        let mut req = [0u8; 48];
        let req_len = build_request(tid, &cont[..cont_len], &mut req);
        l2.send(bt, cid, &req[..req_len])?;

        // Read the response for our channel.
        let n = loop {
            match l2.recv(bt, timer, RESPONSE_TIMEOUT_MS, &mut resp)? {
                Some((c, n)) if c == cid => break n,
                Some(_) => continue,
                None => return Err(SdpError::Timeout),
            }
        };
        let pdu = &resp[..n];
        if pdu.first() == Some(&PDU_ERROR_RESPONSE) {
            return Err(SdpError::Protocol);
        }
        // [pdu_id(1), tid(2), param_len(2), AttributeListsByteCount(2),
        //  AttributeLists(N), ContinuationState(1 len + bytes)].
        if pdu.len() < 7 || pdu[0] != PDU_SSA_RESPONSE {
            return Err(SdpError::Protocol);
        }
        let list_len = u16::from_be_bytes([pdu[5], pdu[6]]) as usize;
        let list_start = 7;
        let list_end = list_start + list_len;
        if list_end >= pdu.len() {
            return Err(SdpError::Protocol);
        }
        if attr_len + list_len > attr.len() {
            return Err(SdpError::TooLong);
        }
        attr[attr_len..attr_len + list_len].copy_from_slice(&pdu[list_start..list_end]);
        attr_len += list_len;

        // Continuation state: a length byte then that many bytes.
        let cs_len = pdu[list_end] as usize;
        if cs_len == 0 || list_end + 1 + cs_len > pdu.len() || cs_len > cont.len() {
            break;
        }
        cont[..cs_len].copy_from_slice(&pdu[list_end + 1..list_end + 1 + cs_len]);
        cont_len = cs_len;
        tid = tid.wrapping_add(1);
    }

    extract_report_descriptor(&attr[..attr_len], out).ok_or(SdpError::NotFound)
}

/// Builds a `ServiceSearchAttributeRequest` for the HID service's descriptor
/// list into `buf`, returning its length.
fn build_request(tid: u16, cont: &[u8], buf: &mut [u8]) -> usize {
    // Parameters:
    //   ServiceSearchPattern: DES { UUID16(HID_SERVICE_CLASS) }
    //   MaximumAttributeByteCount: uint16 (big-endian)
    //   AttributeIDList: DES { uint16(ATTR_HID_DESCRIPTOR_LIST) }
    //   ContinuationState: len byte + bytes
    let hid = HID_SERVICE_CLASS.to_be_bytes();
    let attr = ATTR_HID_DESCRIPTOR_LIST.to_be_bytes();
    let max = MAX_ATTR_BYTES.to_be_bytes();
    let mut params = [0u8; 32];
    let mut p = 0;
    // ServiceSearchPattern: 0x35 len | 0x19 uuid16
    params[p] = 0x35;
    params[p + 1] = 0x03;
    params[p + 2] = 0x19;
    params[p + 3] = hid[0];
    params[p + 4] = hid[1];
    p += 5;
    // MaximumAttributeByteCount.
    params[p] = max[0];
    params[p + 1] = max[1];
    p += 2;
    // AttributeIDList: 0x35 len | 0x09 uint16
    params[p] = 0x35;
    params[p + 1] = 0x03;
    params[p + 2] = 0x09;
    params[p + 3] = attr[0];
    params[p + 4] = attr[1];
    p += 5;
    // ContinuationState.
    params[p] = cont.len() as u8;
    p += 1;
    params[p..p + cont.len()].copy_from_slice(cont);
    p += cont.len();

    // PDU: id(1), tid(2 BE), param_len(2 BE), params.
    buf[0] = PDU_SSA_REQUEST;
    buf[1..3].copy_from_slice(&tid.to_be_bytes());
    buf[3..5].copy_from_slice(&(p as u16).to_be_bytes());
    buf[5..5 + p].copy_from_slice(&params[..p]);
    5 + p
}

/// Parses a data-element header at `data[i]`, returning `(type, value_start,
/// value_len, next_index)`, or `None` if it runs past the end.
fn de_header(data: &[u8], i: usize) -> Option<(u8, usize, usize, usize)> {
    let b = *data.get(i)?;
    let de_type = b >> 3;
    let size_index = b & 0x07;
    let (val_len, extra) = match size_index {
        0 => (if de_type == 0 { 0 } else { 1 }, 0),
        1 => (2, 0),
        2 => (4, 0),
        3 => (8, 0),
        4 => (16, 0),
        5 => (*data.get(i + 1)? as usize, 1),
        6 => (
            u16::from_be_bytes([*data.get(i + 1)?, *data.get(i + 2)?]) as usize,
            2,
        ),
        7 => (
            u32::from_be_bytes([
                *data.get(i + 1)?,
                *data.get(i + 2)?,
                *data.get(i + 3)?,
                *data.get(i + 4)?,
            ]) as usize,
            4,
        ),
        _ => return None,
    };
    let val_start = i + 1 + extra;
    let next = val_start + val_len;
    if next > data.len() {
        return None;
    }
    Some((de_type, val_start, val_len, next))
}

/// Walks a `ServiceSearchAttributeResponse` attribute-list blob to find the HID
/// Report descriptor, copying it into `out` and returning its length.
///
/// Structure: outer sequence of per-service sequences; each service sequence is
/// alternating `(attribute-id, value)`; the value of
/// [`ATTR_HID_DESCRIPTOR_LIST`] is a sequence of `(descriptor-type, data)`
/// sequences, of which the [`HID_DESCRIPTOR_TYPE_REPORT`] one holds the report
/// descriptor.
fn extract_report_descriptor(attr_lists: &[u8], out: &mut [u8]) -> Option<usize> {
    // Outer sequence of service records.
    let (t, vs, vl, _) = de_header(attr_lists, 0)?;
    if t != DE_SEQUENCE {
        return None;
    }
    let services = &attr_lists[vs..vs + vl];

    let mut i = 0;
    while i < services.len() {
        let (st, svs, svl, snext) = de_header(services, i)?;
        i = snext;
        if st != DE_SEQUENCE {
            continue;
        }
        // A service record: alternating attribute id / value elements.
        let record = &services[svs..svs + svl];
        let mut j = 0;
        while j < record.len() {
            let (idt, ids, idl, idnext) = de_header(record, j)?;
            if idt != DE_UINT || idl != 2 {
                j = idnext;
                continue;
            }
            let attr_id = u16::from_be_bytes([record[ids], record[ids + 1]]);
            let (_vt, vvs, vvl, vnext) = de_header(record, idnext)?;
            if attr_id == ATTR_HID_DESCRIPTOR_LIST {
                if let Some(n) = find_report(&record[vvs..vvs + vvl], out) {
                    return Some(n);
                }
            }
            j = vnext;
        }
    }
    None
}

/// Finds the Report descriptor within a HID Descriptor List value (a sequence
/// of `(type, data)` sequences), copying it into `out`.
fn find_report(list: &[u8], out: &mut [u8]) -> Option<usize> {
    let mut i = 0;
    while i < list.len() {
        let (et, evs, evl, enext) = de_header(list, i)?;
        i = enext;
        if et != DE_SEQUENCE {
            continue;
        }
        // Entry: [uint8 descriptor type, string descriptor data].
        let entry = &list[evs..evs + evl];
        let (tt, tvs, tvl, tnext) = de_header(entry, 0)?;
        if tt != DE_UINT || tvl < 1 {
            continue;
        }
        let dtype = entry[tvs];
        let (dt, dvs, dvl, _) = de_header(entry, tnext)?;
        if dtype == HID_DESCRIPTOR_TYPE_REPORT && (dt == DE_STRING || dt == DE_UUID) {
            let n = dvl.min(out.len());
            out[..n].copy_from_slice(&entry[dvs..dvs + n]);
            return Some(n);
        }
    }
    None
}
