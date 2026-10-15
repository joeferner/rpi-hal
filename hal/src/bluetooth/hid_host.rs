//! A Bluetooth Classic (BR/EDR) HID *host* — the L2CAP layer that carries a
//! game controller's / keyboard's / mouse's input reports over an encrypted
//! ACL link.
//!
//! Where the LE side reaches a HID device through GATT
//! ([`gatt_client`](super::gatt_client)), Classic HID rides two
//! connection-oriented L2CAP channels: **HID Control** ([`PSM_HID_CONTROL`],
//! `0x11`) and **HID Interrupt** ([`PSM_HID_INTERRUPT`], `0x13`). Unlike LE's
//! fixed channels, Classic L2CAP channels are opened by a signalling handshake
//! (Connection Request/Response, then a two-way Configuration exchange) on the
//! signalling channel (CID `0x0001`); [`HidHost`] drives that. Once both
//! channels are configured, input reports arrive on the interrupt channel as
//! L2CAP frames whose payload is a HID transaction header
//! ([`HIDP_INPUT_REPORT`], `0xA1`) followed by the report bytes.
//!
//! # Lifecycle
//!
//! After the ACL link is up and encrypted (page + SSP pairing —
//! [`Bluetooth::classic_connect`] / [`Bluetooth::classic_pair`] /
//! [`Bluetooth::classic_set_encryption`]), build a [`HidHost`] with the
//! connection handle, call [`HidHost::open`] to bring up both channels, then
//! loop [`HidHost::next_report`] to read input reports. Either side may
//! initiate the channels — a host in pairing mode opens them, but some devices
//! open them first — so [`HidHost`] both initiates and accepts them.
//!
//! # Scope
//!
//! Basic L2CAP mode only (no retransmission/flow-control mode, no
//! fragmentation beyond what ACL requires), which is what HID uses. It answers
//! the peer's Configuration Requests by accepting them, and Information
//! Requests by reporting basic mode. SDP is skipped — the well-known HID PSMs
//! are used directly. Output reports (rumble, LEDs) over the control channel
//! aren't sent here; this is receive-only.

use super::l2cap::{self, Reassembler};
use super::{AclData, Bluetooth, Error, Event};
use crate::timer::Timer;

/// L2CAP signalling channel CID for Bluetooth Classic (`0x0001`). (LE uses
/// `0x0005`; the two stacks number this channel differently.)
const CID_SIGNALING: u16 = 0x0001;

/// L2CAP PSM for the HID Control channel.
pub const PSM_HID_CONTROL: u16 = 0x0011;
/// L2CAP PSM for the HID Interrupt channel — the one input reports arrive on.
pub const PSM_HID_INTERRUPT: u16 = 0x0013;

/// HID transaction header for an input report on the interrupt channel:
/// `(DATA << 4) | INPUT` = `0xA0 | 0x01`. The report bytes follow it.
pub const HIDP_INPUT_REPORT: u8 = 0xa1;

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

/// L2CAP result "connection successful" (Connection/Configuration Response).
const L2CAP_SUCCESS: u16 = 0x0000;
/// L2CAP Connection Response result "pending" — the peer is still deciding; a
/// second response with the final result follows.
const L2CAP_PENDING: u16 = 0x0001;

/// Configuration option type: Maximum Transmission Unit.
const CONFIG_OPT_MTU: u8 = 0x01;
/// The MTU we advertise as our receive limit — comfortably above any HID
/// report, well within one ACL packet.
const LOCAL_MTU: u16 = 0x00c0;

/// Information Request/Response type: Extended Features Mask.
const INFO_TYPE_EXTENDED_FEATURES: u16 = 0x0002;
/// Information Response result "success".
const INFO_RESULT_SUCCESS: u16 = 0x0000;
/// Information Response result "not supported".
const INFO_RESULT_NOT_SUPPORTED: u16 = 0x0001;

/// Local (source) CID we assign to the HID Control channel. Dynamic CIDs must
/// be ≥ `0x0040`.
const LOCAL_CID_CONTROL: u16 = 0x0040;
/// Local (source) CID we assign to the HID Interrupt channel.
const LOCAL_CID_INTERRUPT: u16 = 0x0041;

/// Index of the control channel in [`HidHost::channels`].
const CH_CONTROL: usize = 0;
/// Index of the interrupt channel in [`HidHost::channels`].
const CH_INTERRUPT: usize = 1;

/// Largest signalling packet this layer builds (Connection/Configuration
/// commands are small).
const SIG_BUF: usize = 64;

/// How long each [`Bluetooth::poll`] blocks while driving the channel setup /
/// report wait, in ms.
const POLL_SLICE_MS: u32 = 500;

/// Errors from the Classic HID host.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HidError {
    /// An underlying HCI/transport error.
    Hci(Error),
    /// The channels didn't finish opening within the time budget.
    Timeout,
    /// The link dropped; carries the HCI disconnect reason.
    Disconnected(u8),
    /// The peer rejected a channel (Connection Response failure or Command
    /// Reject).
    Rejected,
}

impl From<Error> for HidError {
    fn from(e: Error) -> Self {
        HidError::Hci(e)
    }
}

/// One L2CAP channel's state during and after setup.
#[derive(Clone, Copy)]
struct Channel {
    /// The HID PSM this channel carries.
    psm: u16,
    /// Our (source) CID for the channel.
    local_cid: u16,
    /// The peer's (destination) CID, learned from the Connection
    /// Request/Response; `0` until known.
    remote_cid: u16,
    /// A Connection Request has gone out (so we don't send it twice).
    conn_requested: bool,
    /// The channel is connected (Connection Response success, or we accepted
    /// the peer's Connection Request).
    connected: bool,
    /// Our Configuration Request has been answered with success.
    config_rsp_received: bool,
    /// We have answered the peer's Configuration Request.
    config_req_answered: bool,
}

impl Channel {
    /// A fresh channel for `psm` with local CID `local_cid`.
    const fn new(psm: u16, local_cid: u16) -> Self {
        Self {
            psm,
            local_cid,
            remote_cid: 0,
            conn_requested: false,
            connected: false,
            config_rsp_received: false,
            config_req_answered: false,
        }
    }

    /// `true` once both configuration directions have completed — the channel
    /// is ready to carry data.
    fn is_open(&self) -> bool {
        self.connected && self.config_rsp_received && self.config_req_answered
    }
}

/// A Classic HID host bound to one encrypted ACL connection. Opens the HID
/// Control + Interrupt L2CAP channels and yields the input reports the device
/// sends on the interrupt channel.
pub struct HidHost {
    /// The ACL connection handle.
    handle: u16,
    /// Reassembles inbound ACL fragments into L2CAP frames.
    reasm: Reassembler,
    /// The two HID channels: [`CH_CONTROL`] then [`CH_INTERRUPT`].
    channels: [Channel; 2],
    /// Rolling signalling identifier for requests we originate.
    ident: u8,
}

impl HidHost {
    /// Creates a HID host for the connection identified by `connection_handle`
    /// (from [`Bluetooth::classic_connect`]). Call [`Self::open`] next.
    pub fn new(connection_handle: u16) -> Self {
        Self {
            handle: connection_handle,
            reasm: Reassembler::new(),
            channels: [
                Channel::new(PSM_HID_CONTROL, LOCAL_CID_CONTROL),
                Channel::new(PSM_HID_INTERRUPT, LOCAL_CID_INTERRUPT),
            ],
            ident: 1,
        }
    }

    /// Brings up both HID L2CAP channels, returning once the interrupt channel
    /// (the one reports arrive on) is open, or a [`HidError`] on
    /// failure/timeout.
    ///
    /// Initiates the control channel, then the interrupt channel once control
    /// is up, while also accepting channels the device opens toward us and
    /// answering its Configuration/Information Requests — so it works whether
    /// the host or the device drives the setup.
    pub fn open(
        &mut self,
        bt: &mut Bluetooth,
        timer: &Timer,
        timeout_ms: u32,
    ) -> Result<(), HidError> {
        // Kick off the control channel.
        self.request_connection(bt, CH_CONTROL)?;

        let deadline = timer.now_micros() + (timeout_ms as u64) * 1000;
        loop {
            if self.channels[CH_INTERRUPT].is_open() {
                return Ok(());
            }
            // Once control is up, open the interrupt channel (unless the peer
            // already started it).
            if self.channels[CH_CONTROL].is_open()
                && !self.channels[CH_INTERRUPT].conn_requested
                && !self.channels[CH_INTERRUPT].connected
            {
                self.request_connection(bt, CH_INTERRUPT)?;
            }

            let now = timer.now_micros();
            if now >= deadline {
                return Err(HidError::Timeout);
            }
            let remaining = (((deadline - now) / 1000) as u32 + 1).min(POLL_SLICE_MS);
            match bt.poll(timer, remaining)? {
                Some(Event::Acl(acl)) => self.on_acl(bt, &acl, timer)?,
                Some(Event::Disconnected { reason, .. }) => {
                    return Err(HidError::Disconnected(reason))
                }
                Some(_) | None => {}
            }
        }
    }

    /// Waits up to `timeout_ms` for the next input report on the interrupt
    /// channel, copying its bytes into `out` and returning the length
    /// (`Ok(Some(n))`), or `Ok(None)` if the window elapsed quietly. Keeps
    /// answering signalling (config, disconnect) meanwhile.
    ///
    /// The returned bytes are the report *after* the [`HIDP_INPUT_REPORT`]
    /// transaction header — the same raw report a HID Report Map describes.
    pub fn next_report(
        &mut self,
        bt: &mut Bluetooth,
        timer: &Timer,
        timeout_ms: u32,
        out: &mut [u8],
    ) -> Result<Option<usize>, HidError> {
        let deadline = timer.now_micros() + (timeout_ms as u64) * 1000;
        loop {
            let now = timer.now_micros();
            if now >= deadline {
                return Ok(None);
            }
            let remaining = (((deadline - now) / 1000) as u32 + 1).min(POLL_SLICE_MS);
            match bt.poll(timer, remaining)? {
                Some(Event::Acl(acl)) => {
                    if let Some(n) = self.on_acl_report(bt, &acl, timer, out)? {
                        return Ok(Some(n));
                    }
                }
                Some(Event::Disconnected { reason, .. }) => {
                    return Err(HidError::Disconnected(reason))
                }
                Some(_) | None => {}
            }
        }
    }

    /// Feeds an ACL fragment during setup: reassembles it and, if it's a
    /// signalling frame, processes the command.
    fn on_acl(&mut self, bt: &mut Bluetooth, acl: &AclData, timer: &Timer) -> Result<(), HidError> {
        if acl.handle != self.handle {
            return Ok(());
        }
        // Copy out the completed frame so no borrow of `self.reasm` outlives
        // the signalling handling (which mutates `self`).
        let mut frame = [0u8; l2cap::MAX_PAYLOAD];
        let (cid, len) = match self.reasm.feed(acl) {
            Some(pdu) => {
                let n = pdu.payload.len().min(frame.len());
                frame[..n].copy_from_slice(&pdu.payload[..n]);
                (pdu.cid, n)
            }
            None => return Ok(()),
        };
        if cid == CID_SIGNALING {
            self.handle_signaling(bt, &frame[..len], timer)?;
        }
        Ok(())
    }

    /// Feeds an ACL fragment during the report phase: reassembles it, delivers
    /// an interrupt-channel input report into `out` (returning its length), or
    /// keeps answering signalling.
    fn on_acl_report(
        &mut self,
        bt: &mut Bluetooth,
        acl: &AclData,
        timer: &Timer,
        out: &mut [u8],
    ) -> Result<Option<usize>, HidError> {
        if acl.handle != self.handle {
            return Ok(None);
        }
        let mut frame = [0u8; l2cap::MAX_PAYLOAD];
        let (cid, len) = match self.reasm.feed(acl) {
            Some(pdu) => {
                let n = pdu.payload.len().min(frame.len());
                frame[..n].copy_from_slice(&pdu.payload[..n]);
                (pdu.cid, n)
            }
            None => return Ok(None),
        };
        let payload = &frame[..len];
        if cid == CID_SIGNALING {
            self.handle_signaling(bt, payload, timer)?;
            return Ok(None);
        }
        // Input report on the interrupt channel: HIDP header then report bytes.
        if cid == self.channels[CH_INTERRUPT].local_cid
            && payload.first() == Some(&HIDP_INPUT_REPORT)
            && payload.len() > 1
        {
            let report = &payload[1..];
            let n = report.len().min(out.len());
            out[..n].copy_from_slice(&report[..n]);
            return Ok(Some(n));
        }
        Ok(None)
    }

    /// Sends a Connection Request for channel `ch` and marks it requested.
    fn request_connection(&mut self, bt: &mut Bluetooth, ch: usize) -> Result<(), HidError> {
        let ident = self.next_ident();
        let psm = self.channels[ch].psm;
        let scid = self.channels[ch].local_cid;
        let mut data = [0u8; 4];
        data[0..2].copy_from_slice(&psm.to_le_bytes());
        data[2..4].copy_from_slice(&scid.to_le_bytes());
        self.channels[ch].conn_requested = true;
        self.send_sig(bt, SIG_CONNECTION_REQ, ident, &data)
    }

    /// Sends our Configuration Request (advertising [`LOCAL_MTU`]) for channel
    /// `ch`.
    fn send_config_req(&mut self, bt: &mut Bluetooth, ch: usize) -> Result<(), HidError> {
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

    /// Handles one signalling B-frame — it may pack several commands, each a
    /// `code(1) ident(1) length(2) data(length)` block.
    fn handle_signaling(
        &mut self,
        bt: &mut Bluetooth,
        frame: &[u8],
        timer: &Timer,
    ) -> Result<(), HidError> {
        let _ = timer;
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
                SIG_CONNECTION_REQ => self.on_connection_req(bt, ident, data)?,
                SIG_CONNECTION_RSP => self.on_connection_rsp(bt, data)?,
                SIG_CONFIG_REQ => self.on_config_req(bt, ident, data)?,
                SIG_CONFIG_RSP => self.on_config_rsp(data),
                SIG_DISCONNECTION_REQ => self.on_disconnection_req(bt, ident, data)?,
                SIG_INFO_REQ => self.on_info_req(bt, ident, data)?,
                // A Command Reject needs no action.
                SIG_COMMAND_REJECT => {}
                // The echoes of our own replies (Disconnection/Info Response,
                // 0x07/0x0b) and anything else need no action either.
                _ => {}
            }
            i = end;
        }
        Ok(())
    }

    /// A device-initiated Connection Request: accept it on the matching HID
    /// channel and answer with success, then send our Configuration Request.
    fn on_connection_req(
        &mut self,
        bt: &mut Bluetooth,
        ident: u8,
        data: &[u8],
    ) -> Result<(), HidError> {
        if data.len() < 4 {
            return Ok(());
        }
        let psm = u16::from_le_bytes([data[0], data[1]]);
        let peer_scid = u16::from_le_bytes([data[2], data[3]]);
        let Some(ch) = self.channel_for_psm(psm) else {
            return Ok(());
        };
        self.channels[ch].remote_cid = peer_scid;
        self.channels[ch].connected = true;
        let dcid = self.channels[ch].local_cid;

        // Connection Response: dcid(2), scid(2), result(2), status(2).
        let mut rsp = [0u8; 8];
        rsp[0..2].copy_from_slice(&dcid.to_le_bytes());
        rsp[2..4].copy_from_slice(&peer_scid.to_le_bytes());
        rsp[4..6].copy_from_slice(&L2CAP_SUCCESS.to_le_bytes());
        self.send_sig(bt, SIG_CONNECTION_RSP, ident, &rsp)?;
        self.send_config_req(bt, ch)
    }

    /// Our Connection Request was answered: record the peer's CID and, on
    /// success, send our Configuration Request.
    fn on_connection_rsp(&mut self, bt: &mut Bluetooth, data: &[u8]) -> Result<(), HidError> {
        if data.len() < 8 {
            return Ok(());
        }
        let dcid = u16::from_le_bytes([data[0], data[1]]);
        let scid = u16::from_le_bytes([data[2], data[3]]);
        let result = u16::from_le_bytes([data[4], data[5]]);
        let Some(ch) = self.channel_for_local_cid(scid) else {
            return Ok(());
        };
        match result {
            L2CAP_SUCCESS => {
                self.channels[ch].remote_cid = dcid;
                self.channels[ch].connected = true;
                self.send_config_req(bt, ch)
            }
            // Still deciding — the final response follows.
            L2CAP_PENDING => Ok(()),
            _ => Err(HidError::Rejected),
        }
    }

    /// The peer's Configuration Request: accept it (echo its options with a
    /// success result) and mark that direction done.
    fn on_config_req(
        &mut self,
        bt: &mut Bluetooth,
        ident: u8,
        data: &[u8],
    ) -> Result<(), HidError> {
        if data.len() < 4 {
            return Ok(());
        }
        let dcid = u16::from_le_bytes([data[0], data[1]]);
        let options = &data[4..];
        let Some(ch) = self.channel_for_local_cid(dcid) else {
            return Ok(());
        };
        let remote_cid = self.channels[ch].remote_cid;

        // Configuration Response: scid(2)=peer's CID, flags(2)=0,
        // result(2)=success, then the accepted options echoed back.
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
            if let Some(ch) = self.channel_for_local_cid(scid) {
                self.channels[ch].config_rsp_received = true;
            }
        }
    }

    /// The peer wants to close a channel: acknowledge it.
    fn on_disconnection_req(
        &mut self,
        bt: &mut Bluetooth,
        ident: u8,
        data: &[u8],
    ) -> Result<(), HidError> {
        if data.len() < 4 {
            return Ok(());
        }
        // Disconnection Response echoes the request's dcid + scid.
        self.send_sig(bt, SIG_DISCONNECTION_RSP, ident, &data[..4])
    }

    /// An Information Request: report basic-mode capabilities so the peer
    /// settles on basic L2CAP.
    fn on_info_req(&mut self, bt: &mut Bluetooth, ident: u8, data: &[u8]) -> Result<(), HidError> {
        if data.len() < 2 {
            return Ok(());
        }
        let info_type = u16::from_le_bytes([data[0], data[1]]);
        let mut rsp = [0u8; 12];
        rsp[0..2].copy_from_slice(&info_type.to_le_bytes());
        let n = if info_type == INFO_TYPE_EXTENDED_FEATURES {
            // result=success, then a 4-byte feature mask of 0 (no extended
            // features → basic mode).
            rsp[2..4].copy_from_slice(&INFO_RESULT_SUCCESS.to_le_bytes());
            8
        } else {
            rsp[2..4].copy_from_slice(&INFO_RESULT_NOT_SUPPORTED.to_le_bytes());
            4
        };
        self.send_sig(bt, SIG_INFO_RSP, ident, &rsp[..n])
    }

    /// Frames and sends one L2CAP signalling command on [`CID_SIGNALING`].
    fn send_sig(
        &mut self,
        bt: &mut Bluetooth,
        code: u8,
        ident: u8,
        data: &[u8],
    ) -> Result<(), HidError> {
        let mut buf = [0u8; SIG_BUF];
        buf[0] = code;
        buf[1] = ident;
        buf[2..4].copy_from_slice(&(data.len() as u16).to_le_bytes());
        let end = 4 + data.len();
        buf[4..end].copy_from_slice(data);
        l2cap::send(bt, self.handle, CID_SIGNALING, &buf[..end])?;
        Ok(())
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

    /// The channel index carrying `psm`, if any.
    fn channel_for_psm(&self, psm: u16) -> Option<usize> {
        self.channels.iter().position(|c| c.psm == psm)
    }

    /// The channel index with local CID `cid`, if any.
    fn channel_for_local_cid(&self, cid: u16) -> Option<usize> {
        self.channels.iter().position(|c| c.local_cid == cid)
    }
}
