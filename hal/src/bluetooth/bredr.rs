//! Bluetooth Classic (BR/EDR) connection-oriented L2CAP — the reusable channel
//! layer beneath the Classic profiles.
//!
//! LE L2CAP runs on fixed channels ([`l2cap`]); Classic L2CAP
//! opens *dynamic* channels to a PSM through a signalling handshake
//! (Connection Request/Response, then a two-way Configuration exchange) on the
//! signalling channel (CID `0x0001`). [`L2cap`] drives that handshake and then
//! carries data both ways, so a profile — [`sdp`](super::sdp) to read a
//! device's service records, or HID to read input reports — only deals in
//! "open a channel to this PSM, send/receive on it".
//!
//! # Model
//!
//! Build an [`L2cap`] over an established ACL handle, [`L2cap::open`] a channel
//! to a PSM (returning its local CID), [`L2cap::send`] payloads on it, drain
//! inbound data with [`L2cap::recv`], and [`L2cap::close`] it when done. It
//! answers the peer's Configuration and Information Requests itself while doing
//! so. Several channels can be open at once (e.g. HID's control + interrupt).
//!
//! # Scope
//!
//! Basic L2CAP mode (no retransmission/flow-control mode), which the HID and
//! SDP profiles use. This is the *initiator* side — it opens channels; it does
//! not currently accept channels a peer opens toward it (a HID device that
//! reconnects by opening its own channels needs that added).

use super::l2cap::{self, Reassembler, MAX_PAYLOAD};
use super::{AclData, Bluetooth, Error, Event};
use crate::timer::Timer;

/// L2CAP signalling channel CID for Bluetooth Classic.
const CID_SIGNALING: u16 = 0x0001;

// --- L2CAP signalling command codes ---
/// Command Reject.
const SIG_COMMAND_REJECT: u8 = 0x01;
/// Connection Request.
const SIG_CONNECTION_REQ: u8 = 0x02;
/// Connection Response.
const SIG_CONNECTION_RSP: u8 = 0x03;
/// Configuration Request.
const SIG_CONFIG_REQ: u8 = 0x04;
/// Configuration Response.
const SIG_CONFIG_RSP: u8 = 0x05;
/// Disconnection Request.
const SIG_DISCONNECTION_REQ: u8 = 0x06;
/// Disconnection Response.
const SIG_DISCONNECTION_RSP: u8 = 0x07;
/// Information Request.
const SIG_INFO_REQ: u8 = 0x0a;
/// Information Response.
const SIG_INFO_RSP: u8 = 0x0b;

/// L2CAP result "successful".
const L2CAP_SUCCESS: u16 = 0x0000;
/// L2CAP Connection Response result "pending".
const L2CAP_PENDING: u16 = 0x0001;

/// Configuration option type: Maximum Transmission Unit.
const CONFIG_OPT_MTU: u8 = 0x01;
/// The MTU we advertise as our receive limit — the reassembly buffer size, so
/// a peer may send us a whole [`MAX_PAYLOAD`] SDU in one B-frame.
const LOCAL_MTU: u16 = MAX_PAYLOAD as u16;

/// Information Request/Response type: Extended Features Mask.
const INFO_TYPE_EXTENDED_FEATURES: u16 = 0x0002;
/// Information Response result "success".
const INFO_RESULT_SUCCESS: u16 = 0x0000;
/// Information Response result "not supported".
const INFO_RESULT_NOT_SUPPORTED: u16 = 0x0001;

/// First dynamic (local) CID we assign; dynamic CIDs must be ≥ `0x0040`.
const FIRST_DYNAMIC_CID: u16 = 0x0040;
/// How many channels an [`L2cap`] can hold open at once.
pub const MAX_CHANNELS: usize = 4;

/// Largest signalling packet this layer builds.
const SIG_BUF: usize = 64;
/// How long each [`Bluetooth::poll`] blocks while driving the channel setup /
/// receive, in ms.
const POLL_SLICE_MS: u32 = 500;
/// How long [`L2cap::close`] waits for the peer's Disconnection Response before
/// freeing the slot regardless, in ms.
const DISCONNECT_TIMEOUT_MS: u32 = 1_000;

/// Errors from the Classic L2CAP layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum L2capError {
    /// An underlying HCI/transport error.
    Hci(Error),
    /// A channel didn't finish opening within the time budget.
    Timeout,
    /// The link dropped; carries the HCI disconnect reason.
    Disconnected(u8),
    /// The peer rejected the channel (Connection Response failure).
    Rejected,
    /// No free channel slot ([`MAX_CHANNELS`] already open), or an unknown CID.
    NoChannel,
}

impl From<Error> for L2capError {
    fn from(e: Error) -> Self {
        L2capError::Hci(e)
    }
}

/// One dynamic L2CAP channel's state.
#[derive(Clone, Copy)]
struct Channel {
    /// Whether this slot holds a live channel.
    in_use: bool,
    /// Our (source) CID.
    local_cid: u16,
    /// The peer's (destination) CID — where our data frames are addressed.
    remote_cid: u16,
    /// Connected (Connection Response success).
    connected: bool,
    /// Our Configuration Request was answered with success.
    config_rsp_received: bool,
    /// We answered the peer's Configuration Request.
    config_req_answered: bool,
}

impl Channel {
    /// An empty slot.
    const fn empty() -> Self {
        Self {
            in_use: false,
            local_cid: 0,
            remote_cid: 0,
            connected: false,
            config_rsp_received: false,
            config_req_answered: false,
        }
    }

    /// `true` once both configuration directions completed — ready for data.
    fn is_open(&self) -> bool {
        self.connected && self.config_rsp_received && self.config_req_answered
    }
}

/// A Classic L2CAP endpoint bound to one ACL connection, managing a set of
/// dynamic channels.
pub struct L2cap {
    /// The ACL connection handle.
    handle: u16,
    /// Reassembles inbound ACL fragments into L2CAP frames.
    reasm: Reassembler,
    /// The open (and opening) channels.
    channels: [Channel; MAX_CHANNELS],
    /// Rolling signalling identifier for requests we originate.
    ident: u8,
}

impl L2cap {
    /// Creates an L2CAP endpoint over `handle` (from
    /// [`Bluetooth::classic_connect`](super::Bluetooth::classic_connect)).
    pub fn new(handle: u16) -> Self {
        Self {
            handle,
            reasm: Reassembler::new(),
            channels: [Channel::empty(); MAX_CHANNELS],
            ident: 1,
        }
    }

    /// Opens a channel to `psm`, returning its local CID once it is fully
    /// configured (or a [`L2capError`] on failure/timeout). Drive
    /// [`Self::send`]/[`Self::recv`] on it afterwards.
    pub fn open(
        &mut self,
        bt: &mut Bluetooth,
        psm: u16,
        timer: &Timer,
        timeout_ms: u32,
    ) -> Result<u16, L2capError> {
        let slot = self.alloc_channel().ok_or(L2capError::NoChannel)?;
        let local_cid = self.channels[slot].local_cid;

        // Connection Request: psm(2), scid(2).
        let ident = self.next_ident();
        let mut req = [0u8; 4];
        req[0..2].copy_from_slice(&psm.to_le_bytes());
        req[2..4].copy_from_slice(&local_cid.to_le_bytes());
        self.send_sig(bt, SIG_CONNECTION_REQ, ident, &req)?;

        let deadline = timer.now_micros() + (timeout_ms as u64) * 1000;
        loop {
            if self.channels[slot].is_open() {
                return Ok(local_cid);
            }
            let now = timer.now_micros();
            if now >= deadline {
                return Err(L2capError::Timeout);
            }
            let remaining = (((deadline - now) / 1000) as u32 + 1).min(POLL_SLICE_MS);
            match bt.poll(timer, remaining)? {
                Some(Event::Acl(acl)) => {
                    // A rejected channel surfaces as the slot being freed.
                    self.on_acl_signaling(bt, &acl)?;
                    if !self.channels[slot].in_use {
                        return Err(L2capError::Rejected);
                    }
                }
                Some(Event::Disconnected { reason, .. }) => {
                    return Err(L2capError::Disconnected(reason))
                }
                Some(_) | None => {}
            }
        }
    }

    /// Sends a data payload on the channel with local CID `local_cid` (its
    /// bytes are addressed to the peer's CID). The payload must fit
    /// [`MAX_PAYLOAD`].
    pub fn send(
        &mut self,
        bt: &mut Bluetooth,
        local_cid: u16,
        payload: &[u8],
    ) -> Result<(), L2capError> {
        let ch = self
            .channel_by_local_cid(local_cid)
            .ok_or(L2capError::NoChannel)?;
        let remote_cid = self.channels[ch].remote_cid;
        l2cap::send(bt, self.handle, remote_cid, payload)?;
        Ok(())
    }

    /// Waits up to `timeout_ms` for the next inbound data frame on any open
    /// channel, copying it into `out` and returning `(local_cid, len)`, or
    /// `Ok(None)` if the window elapsed quietly. Answers signalling
    /// (configuration, information, disconnection) meanwhile.
    pub fn recv(
        &mut self,
        bt: &mut Bluetooth,
        timer: &Timer,
        timeout_ms: u32,
        out: &mut [u8],
    ) -> Result<Option<(u16, usize)>, L2capError> {
        let deadline = timer.now_micros() + (timeout_ms as u64) * 1000;
        loop {
            let now = timer.now_micros();
            if now >= deadline {
                return Ok(None);
            }
            let remaining = (((deadline - now) / 1000) as u32 + 1).min(POLL_SLICE_MS);
            match bt.poll(timer, remaining)? {
                Some(Event::Acl(acl)) => {
                    if let Some(hit) = self.on_acl_data(bt, &acl, out)? {
                        return Ok(Some(hit));
                    }
                }
                Some(Event::Disconnected { reason, .. }) => {
                    return Err(L2capError::Disconnected(reason))
                }
                Some(_) | None => {}
            }
        }
    }

    /// Closes the channel with local CID `local_cid`: sends a Disconnection
    /// Request, waits briefly for the peer's Disconnection Response, then frees
    /// the slot. Tearing a channel down cleanly returns its CID to the pool so a
    /// later channel on the same link can reuse it — without this, the abandoned
    /// CID stays allocated on the peer and a reopen with the same CID is
    /// rejected. A no-op (returns `Ok`) if no such channel is open.
    pub fn close(
        &mut self,
        bt: &mut Bluetooth,
        local_cid: u16,
        timer: &Timer,
    ) -> Result<(), L2capError> {
        let Some(ch) = self.channel_by_local_cid(local_cid) else {
            return Ok(());
        };
        let remote_cid = self.channels[ch].remote_cid;
        let ident = self.next_ident();
        // Disconnection Request: dcid(2)=peer's CID, scid(2)=our CID.
        let mut data = [0u8; 4];
        data[0..2].copy_from_slice(&remote_cid.to_le_bytes());
        data[2..4].copy_from_slice(&local_cid.to_le_bytes());
        self.send_sig(bt, SIG_DISCONNECTION_REQ, ident, &data)?;

        // Wait for the Disconnection Response (which frees the slot via
        // `on_disconnection_rsp`), but free it ourselves on timeout regardless —
        // the request has been sent and we won't send on this CID again.
        let deadline = timer.now_micros() + (DISCONNECT_TIMEOUT_MS as u64) * 1000;
        while timer.now_micros() < deadline && self.channel_by_local_cid(local_cid).is_some() {
            let now = timer.now_micros();
            let remaining = (((deadline - now) / 1000) as u32 + 1).min(POLL_SLICE_MS);
            match bt.poll(timer, remaining)? {
                Some(Event::Acl(acl)) => self.on_acl_signaling(bt, &acl)?,
                Some(Event::Disconnected { reason, .. }) => {
                    return Err(L2capError::Disconnected(reason))
                }
                Some(_) | None => {}
            }
        }
        if let Some(ch) = self.channel_by_local_cid(local_cid) {
            self.channels[ch] = Channel::empty();
        }
        Ok(())
    }

    /// Feeds an ACL fragment during channel setup: reassembles it and, if it's
    /// a signalling frame, processes it.
    fn on_acl_signaling(&mut self, bt: &mut Bluetooth, acl: &AclData) -> Result<(), L2capError> {
        let _ = self.on_acl_data(bt, acl, &mut [])?;
        Ok(())
    }

    /// Feeds an ACL fragment: reassembles it, dispatches signalling (CID
    /// `0x0001`), or reports inbound channel data into `out`.
    fn on_acl_data(
        &mut self,
        bt: &mut Bluetooth,
        acl: &AclData,
        out: &mut [u8],
    ) -> Result<Option<(u16, usize)>, L2capError> {
        if acl.handle != self.handle {
            return Ok(None);
        }
        // Copy the completed frame out so no borrow of `self.reasm` outlives
        // the signalling handling (which mutates `self`).
        let mut frame = [0u8; MAX_PAYLOAD];
        let (cid, len) = match self.reasm.feed(acl) {
            Some(pdu) => {
                let n = pdu.payload.len().min(frame.len());
                frame[..n].copy_from_slice(&pdu.payload[..n]);
                (pdu.cid, n)
            }
            None => return Ok(None),
        };
        if cid == CID_SIGNALING {
            self.handle_signaling(bt, &frame[..len])?;
            return Ok(None);
        }
        // Data on a dynamic channel: the CID is our local CID.
        if let Some(_ch) = self.channel_by_local_cid(cid) {
            let n = len.min(out.len());
            out[..n].copy_from_slice(&frame[..n]);
            return Ok(Some((cid, n)));
        }
        Ok(None)
    }

    /// Handles one signalling B-frame (which may pack several `code/ident/
    /// length/data` commands).
    fn handle_signaling(&mut self, bt: &mut Bluetooth, frame: &[u8]) -> Result<(), L2capError> {
        let mut i = 0;
        while i + 4 <= frame.len() {
            let code = frame[i];
            let ident = frame[i + 1];
            let len = u16::from_le_bytes([frame[i + 2], frame[i + 3]]) as usize;
            let start = i + 4;
            let end = start + len;
            if end > frame.len() {
                break;
            }
            let data = &frame[start..end];
            match code {
                SIG_CONNECTION_RSP => self.on_connection_rsp(bt, data)?,
                SIG_CONFIG_REQ => self.on_config_req(bt, ident, data)?,
                SIG_CONFIG_RSP => self.on_config_rsp(data),
                SIG_DISCONNECTION_REQ => self.on_disconnection_req(bt, ident, data)?,
                SIG_DISCONNECTION_RSP => self.on_disconnection_rsp(data),
                SIG_INFO_REQ => self.on_info_req(bt, ident, data)?,
                // A Command Reject aborts the channel it names — free the slot.
                SIG_COMMAND_REJECT => self.on_command_reject(),
                // Echoes of our own replies (Info Response) and anything else
                // need no action.
                _ => {}
            }
            i = end;
        }
        Ok(())
    }

    /// Our Connection Request was answered: record the peer's CID and, on
    /// success, send our Configuration Request; on failure, free the slot.
    fn on_connection_rsp(&mut self, bt: &mut Bluetooth, data: &[u8]) -> Result<(), L2capError> {
        if data.len() < 8 {
            return Ok(());
        }
        let dcid = u16::from_le_bytes([data[0], data[1]]);
        let scid = u16::from_le_bytes([data[2], data[3]]);
        let result = u16::from_le_bytes([data[4], data[5]]);
        let Some(ch) = self.channel_by_local_cid(scid) else {
            return Ok(());
        };
        match result {
            L2CAP_SUCCESS => {
                self.channels[ch].remote_cid = dcid;
                self.channels[ch].connected = true;
                self.send_config_req(bt, ch)
            }
            L2CAP_PENDING => Ok(()),
            _ => {
                self.channels[ch] = Channel::empty();
                Ok(())
            }
        }
    }

    /// The peer's Configuration Request: accept it (echo options, success) and
    /// mark that direction done.
    fn on_config_req(
        &mut self,
        bt: &mut Bluetooth,
        ident: u8,
        data: &[u8],
    ) -> Result<(), L2capError> {
        if data.len() < 4 {
            return Ok(());
        }
        let dcid = u16::from_le_bytes([data[0], data[1]]);
        let options = &data[4..];
        let Some(ch) = self.channel_by_local_cid(dcid) else {
            return Ok(());
        };
        let remote_cid = self.channels[ch].remote_cid;

        // Configuration Response: scid(2)=peer's CID, flags(2)=0,
        // result(2)=success, echoed options.
        let mut rsp = [0u8; SIG_BUF];
        rsp[0..2].copy_from_slice(&remote_cid.to_le_bytes());
        rsp[4..6].copy_from_slice(&L2CAP_SUCCESS.to_le_bytes());
        let opt_len = options.len().min(rsp.len() - 6);
        rsp[6..6 + opt_len].copy_from_slice(&options[..opt_len]);
        self.channels[ch].config_req_answered = true;
        self.send_sig(bt, SIG_CONFIG_RSP, ident, &rsp[..6 + opt_len])
    }

    /// Our Configuration Request was answered: on success, mark that direction
    /// done.
    fn on_config_rsp(&mut self, data: &[u8]) {
        if data.len() < 6 {
            return;
        }
        let scid = u16::from_le_bytes([data[0], data[1]]);
        let result = u16::from_le_bytes([data[4], data[5]]);
        if result == L2CAP_SUCCESS {
            if let Some(ch) = self.channel_by_local_cid(scid) {
                self.channels[ch].config_rsp_received = true;
            }
        }
    }

    /// The peer wants to close a channel: acknowledge and free the slot.
    fn on_disconnection_req(
        &mut self,
        bt: &mut Bluetooth,
        ident: u8,
        data: &[u8],
    ) -> Result<(), L2capError> {
        if data.len() < 4 {
            return Ok(());
        }
        let dcid = u16::from_le_bytes([data[0], data[1]]);
        self.send_sig(bt, SIG_DISCONNECTION_RSP, ident, &data[..4])?;
        if let Some(ch) = self.channel_by_local_cid(dcid) {
            self.channels[ch] = Channel::empty();
        }
        Ok(())
    }

    /// The peer answered our Disconnection Request: free the slot it names (by
    /// our source CID).
    fn on_disconnection_rsp(&mut self, data: &[u8]) {
        if data.len() < 4 {
            return;
        }
        let scid = u16::from_le_bytes([data[2], data[3]]);
        if let Some(ch) = self.channel_by_local_cid(scid) {
            self.channels[ch] = Channel::empty();
        }
    }

    /// A Command Reject frees any channel still mid-open (best effort — the
    /// reject may not name a CID, so drop the newest un-opened channel).
    fn on_command_reject(&mut self) {
        if let Some(ch) = self.channels.iter().rposition(|c| c.in_use && !c.is_open()) {
            self.channels[ch] = Channel::empty();
        }
    }

    /// An Information Request: report basic-mode capabilities.
    fn on_info_req(
        &mut self,
        bt: &mut Bluetooth,
        ident: u8,
        data: &[u8],
    ) -> Result<(), L2capError> {
        if data.len() < 2 {
            return Ok(());
        }
        let info_type = u16::from_le_bytes([data[0], data[1]]);
        let mut rsp = [0u8; 12];
        rsp[0..2].copy_from_slice(&info_type.to_le_bytes());
        let n = if info_type == INFO_TYPE_EXTENDED_FEATURES {
            // success, then a 4-byte feature mask of 0 (basic mode).
            rsp[2..4].copy_from_slice(&INFO_RESULT_SUCCESS.to_le_bytes());
            8
        } else {
            rsp[2..4].copy_from_slice(&INFO_RESULT_NOT_SUPPORTED.to_le_bytes());
            4
        };
        self.send_sig(bt, SIG_INFO_RSP, ident, &rsp[..n])
    }

    /// Sends our Configuration Request (advertising [`LOCAL_MTU`]) for channel
    /// `ch`.
    fn send_config_req(&mut self, bt: &mut Bluetooth, ch: usize) -> Result<(), L2capError> {
        let ident = self.next_ident();
        let dcid = self.channels[ch].remote_cid;
        // dcid(2), flags(2)=0, MTU option: type(1), len(1)=2, value(2).
        let mut data = [0u8; 8];
        data[0..2].copy_from_slice(&dcid.to_le_bytes());
        data[4] = CONFIG_OPT_MTU;
        data[5] = 0x02;
        data[6..8].copy_from_slice(&LOCAL_MTU.to_le_bytes());
        self.send_sig(bt, SIG_CONFIG_REQ, ident, &data)
    }

    /// Frames and sends one signalling command on [`CID_SIGNALING`].
    fn send_sig(
        &mut self,
        bt: &mut Bluetooth,
        code: u8,
        ident: u8,
        data: &[u8],
    ) -> Result<(), L2capError> {
        let mut buf = [0u8; SIG_BUF];
        buf[0] = code;
        buf[1] = ident;
        buf[2..4].copy_from_slice(&(data.len() as u16).to_le_bytes());
        let end = 4 + data.len();
        buf[4..end].copy_from_slice(data);
        l2cap::send(bt, self.handle, CID_SIGNALING, &buf[..end])?;
        Ok(())
    }

    /// Claims a free channel slot with a fresh local CID.
    fn alloc_channel(&mut self) -> Option<usize> {
        let slot = self.channels.iter().position(|c| !c.in_use)?;
        let local_cid = FIRST_DYNAMIC_CID + slot as u16;
        self.channels[slot] = Channel::empty();
        self.channels[slot].in_use = true;
        self.channels[slot].local_cid = local_cid;
        Some(slot)
    }

    /// The next signalling identifier for a request we originate (never 0).
    fn next_ident(&mut self) -> u8 {
        let id = self.ident;
        self.ident = self.ident.wrapping_add(1);
        if self.ident == 0 {
            self.ident = 1;
        }
        id
    }

    /// The index of the in-use channel with local CID `cid`.
    fn channel_by_local_cid(&self, cid: u16) -> Option<usize> {
        self.channels
            .iter()
            .position(|c| c.in_use && c.local_cid == cid)
    }
}
