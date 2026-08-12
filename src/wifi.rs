//! Host-side control protocol for the on-board BCM43430 wireless chip,
//! on top of the [`crate::sdio`] link once its firmware is running.
//!
//! A [`Sdio`](crate::sdio::Sdio) that has completed
//! [`load_firmware`](crate::sdio::Sdio::load_firmware) is wrapped in a
//! [`Wifi`](crate::wifi::Wifi), which speaks Broadcom's SDIO protocol stack
//! over function 2
//! (the WLAN data path): the SDPCM framing layer, and, inside control
//! frames, the CDC (a.k.a. BCDC) command protocol that carries "iovars"
//! (named firmware variables) and numeric ioctls. Reading the firmware
//! version or the chip's MAC address is a plain CDC "get"
//! ([`get_iovar`](crate::wifi::Wifi::get_iovar)); joining a WPA2-PSK network
//! is a series of CDC
//! "set"s driving the chip's in-firmware supplicant
//! ([`join_wpa2`](crate::wifi::Wifi::join_wpa2)). Network data frames move
//! over the SDPCM data channel wrapped in a BDC header
//! ([`send_ethernet`](crate::wifi::Wifi::send_ethernet) /
//! [`recv_ethernet`](crate::wifi::Wifi::recv_ethernet)); with the `smoltcp`
//! feature, [`WifiPhy`](crate::wifi::WifiPhy) wraps
//! that as a `phy::Device` so a TCP/IP stack can run on top.
//!
//! The framing follows plan9/9front's `ether4330.c` (a self-contained
//! bare-metal SDPCM/CDC implementation for this exact chip), cross-
//! checked against Linux's `brcmfmac` (`sdio.c`/`bcdc.c`). Pi 3 only.

use crate::sdio::{self, Sdio};
use crate::timer::Timer;

/// Length of the SDPCM frame header (4-byte hardware frame tag + 8-byte
/// software header). Every frame over function 2 begins with this.
const SDPCM_HEADER_LEN: usize = 12;
/// Length of the CDC (BCDC) command header, which follows the SDPCM
/// header in a control frame.
const CDC_HEADER_LEN: usize = 16;
/// SDPCM channel for CDC control messages (ioctls/iovars).
const CHANNEL_CONTROL: u8 = 0;
/// SDPCM channel for asynchronous firmware events (join/link/etc.).
const CHANNEL_EVENT: u8 = 1;
/// SDPCM channel for network data frames (the BDC-wrapped Ethernet path).
const CHANNEL_DATA: u8 = 2;

/// Length of the BDC header that wraps each data-channel Ethernet frame
/// (flags, priority, flags2, data-offset).
const BDC_HEADER_LEN: usize = 4;
/// BDC header `flags` byte: protocol version 2 in the high nibble.
const BDC_FLAG_VERSION: u8 = 0x20;
/// Flow-control mask bit (in an SDPCM header's `fcmask`) that pauses the
/// data channel — the firmware sets it to throttle host transmits.
const DATA_FC_BIT: u8 = 1 << 2;

/// Firmware event: the result of a `WLC_SET_SSID` join attempt.
pub const EVENT_SET_SSID: u16 = 0;
/// Firmware event: 802.11 authentication result.
pub const EVENT_AUTH: u16 = 3;
/// Firmware event: 802.11 association result.
pub const EVENT_ASSOC: u16 = 7;
/// Firmware event: link up/down (its `flags` bit 0 = up).
pub const EVENT_LINK: u16 = 16;
/// Firmware event: in-firmware WPA supplicant progress (its `status`
/// reaches 6, "completed", on a successful 4-way handshake; 7 is a
/// handshake timeout, e.g. a wrong passphrase).
pub const EVENT_PSK_SUP: u16 = 46;
/// Firmware event: one scan result (a found AP), delivered during a scan
/// started by [`Wifi::scan`]. A final one with no BSS marks scan end.
pub const EVENT_ESCAN_RESULT: u16 = 69;
/// [`EVENT_LINK`] `flags` bit meaning the link is up.
pub const EVENT_LINK_UP: u16 = 0x01;
/// [`EVENT_PSK_SUP`] `status` meaning the 4-way handshake completed.
pub const EVENT_SUP_COMPLETED: u32 = 6;

/// Firmware command: bring the interface up.
const WLC_UP: u32 = 2;
/// Firmware command: select passive (`1`) vs active (`0`) scanning.
const WLC_SET_PASSIVE_SCAN: u32 = 49;
/// Firmware command: set infrastructure (BSS) mode.
const WLC_SET_INFRA: u32 = 20;
/// Firmware command: read the associated AP's BSSID (fails with a
/// not-associated status when the chip isn't on a network).
const WLC_GET_BSSID: u32 = 23;
/// Firmware command: set the SSID and join (given a `wlc_ssid_t`).
const WLC_SET_SSID: u32 = 26;
/// Firmware command: hand the WPA(2) passphrase to the in-firmware
/// supplicant (given a `wsec_pmk_t`).
const WLC_SET_WSEC_PMK: u32 = 268;
/// Firmware command to *get* a named variable (iovar); the variable
/// name, NUL-terminated, is the request payload.
const WLC_GET_VAR: u32 = 262;
/// Firmware command to *set* a named variable (iovar).
const WLC_SET_VAR: u32 = 263;

/// `wsec` value selecting AES-CCMP encryption (WPA2).
const WSEC_AES: u32 = 4;
/// `wpa_auth` value selecting WPA2-PSK authentication.
const WPA2_AUTH_PSK: u32 = 0x80;
/// `wsec_pmk_t` flag marking the key material as an ASCII passphrase
/// (the firmware derives the PMK itself) rather than a raw PMK.
const WSEC_PMK_PASSPHRASE: u16 = 0x0001;

/// Length of the `clmload` download header (flag, type, length, crc)
/// prefixed to each CLM chunk.
const CLM_HEADER_LEN: usize = 12;
/// Base `clmload` flag: download-handler version 1, CRC not in use.
const CLM_FLAG_BASE: u16 = (1 << 12) | 0x0001;
/// `clmload` flag marking the first chunk of the blob.
const CLM_FLAG_BEGIN: u16 = 0x0002;
/// `clmload` flag marking the last chunk of the blob.
const CLM_FLAG_END: u16 = 0x0004;
/// `clmload` download type selecting CLM (regulatory) data.
const CLM_DOWNLOAD_TYPE: u16 = 2;
/// Bytes of CLM blob per `clmload` chunk.
const CLM_CHUNK: usize = 1024;

/// CDC flags bit marking a command as a *set* (write) rather than a get.
const CDC_FLAG_SET: u16 = 0x02;
/// CDC flags bit set in a *response* whose command failed (the `status`
/// field then holds the firmware error code).
const CDC_FLAG_ERROR: u16 = 0x01;

/// Scratch capacity for an assembled iovar request (`name` + NUL +
/// value). The largest value this driver sends as an iovar is the scan
/// parameters; ioctls with larger payloads (the passphrase struct) go
/// through [`Wifi::ioctl_set`] directly, not this buffer.
const MAX_IOVAR_REQUEST: usize = 256;
/// Length of the `escan` parameters (see [`fill_escan_params`]): the
/// 132 bytes of fields plus a 4-byte tail the firmware's struct includes
/// (a shorter buffer is rejected as `BCME_BUFTOOSHORT`).
const ESCAN_PARAMS_LEN: usize = 136;

/// Length of the `event_msgs` bitmask (one bit per firmware event).
const EVENT_MASK_LEN: usize = 16;
/// Firmware event numbers left *disabled* in the `event_msgs` mask —
/// high-rate or uninteresting events (radio, probe req/resp, interface,
/// tx-fail) that would otherwise flood the receive path. Matches plan9's
/// choices; everything else stays on so the join events arrive.
const DISABLED_EVENTS: [usize; 6] = [40, 44, 54, 71, 20, 124];

/// SDIO device-core register offset: interrupt status (write-1-to-clear).
const SDIO_CORE_INTSTATUS: u32 = 0x20;
/// SDIO device-core register offset: interrupt mask.
const SDIO_CORE_INTMASK: u32 = 0x24;
/// `INTSTATUS`/`INTMASK` bit: the firmware raised a flow-control change.
const INT_FCCHANGE: u32 = 1 << 5;
/// `INTSTATUS`/`INTMASK` bit: a frame is ready to read.
const INT_FRAME: u32 = 1 << 6;
/// `INTSTATUS`/`INTMASK` bit: a mailbox (firmware-ready) event.
const INT_MAILBOX: u32 = 1 << 7;

/// CCCR interrupt-pending register (function 0): non-zero when the chip
/// has raised an interrupt (a per-function bitmap).
const CCCR_INT_PENDING: u32 = 0x05;
/// CCCR "I/O Enable" and interrupt-enable registers live in function 0;
/// this is the interrupt-enable register.
const CCCR_INT_ENABLE: u32 = 0x04;
/// CCCR function-2 block-size register (low byte; high byte is +1).
const CCCR_FBR2_BLOCKSIZE: u32 = 0x210;
/// SDIO function 2's block size, in bytes.
const F2_BLOCK_SIZE: u16 = 512;

/// SDIO function numbers, as `cmd52` addresses them.
const FN0: u32 = 0;

/// Largest SDPCM frame this driver builds or accepts, in bytes — the
/// SDPCM length field's practical ceiling. Sized to hold a control
/// command plus a reasonably large iovar reply (e.g. the version
/// string).
const MAX_FRAME: usize = 2048;

/// Errors from the Wi-Fi protocol layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// An error from the underlying SDIO link.
    Sdio(sdio::Error),
    /// A received SDPCM frame was malformed (its length-check word
    /// didn't match, or the length was out of range) — the function-2
    /// receive stream is out of sync.
    BadFrame,
    /// A CDC command came back with a non-zero firmware status code.
    CommandFailed(i32),
    /// No matching response arrived within the time budget.
    NoResponse,
    /// A join was issued but the chip didn't associate within the time
    /// budget (a wrong passphrase, an out-of-range or missing AP, …).
    NotAssociated,
    /// A data frame couldn't be sent right now: the firmware's transmit
    /// credit window is exhausted, or it has flow-controlled the data
    /// channel. Transient — retry once a received frame advances the
    /// window.
    TxBusy,
    /// A frame handed to [`Wifi::send_ethernet`] is larger than the driver
    /// can frame (see [`Wifi::MTU`]).
    FrameTooLong,
}

impl From<sdio::Error> for Error {
    fn from(error: sdio::Error) -> Self {
        Error::Sdio(error)
    }
}

/// A running-firmware wireless chip, reachable over the SDPCM/CDC
/// protocol.
///
/// Build one with [`Self::new`] from a [`Sdio`] whose firmware is
/// already loaded and running, then issue control commands such as
/// [`Self::get_iovar`].
pub struct Wifi {
    sdio: Sdio,
    /// Sequence number stamped into the next frame sent, incremented
    /// after each send (wrapping at 256).
    tx_seq: u8,
    /// Credit ceiling advertised by the firmware (the highest sequence
    /// number the host may use) — updated from every received frame.
    /// Data transmits ([`Wifi::send_ethernet`]) are gated on it; control
    /// frames are not.
    tx_window: u8,
    /// Per-channel flow-control bitmap from the last received frame; its
    /// [`DATA_FC_BIT`] pauses the data channel.
    fc_mask: u8,
    /// Request id stamped into the next CDC command, to match its
    /// response.
    req_id: u16,
    /// Scratch buffer for building an outgoing frame or holding an
    /// incoming one.
    frame: [u8; MAX_FRAME],
    /// Whether the receive FIFO is being drained: `true` between the
    /// frame-ready interrupt firing and the zero-length header that marks
    /// the FIFO empty. While set, [`Self::recv_frame`] reads the next
    /// frame directly rather than waiting for an interrupt that won't come
    /// for a frame already queued.
    rx_draining: bool,
}

impl Wifi {
    /// Wraps a firmware-loaded [`Sdio`] and readies the SDPCM protocol
    /// path: sets function 2's block size, unmasks the SDIO core's
    /// frame/mailbox interrupts, and enables the host-side SDIO
    /// interrupt. The chip's firmware must already be running (see
    /// [`Sdio::load_firmware`]).
    pub fn new(mut sdio: Sdio, timer: &Timer) -> Result<Self, Error> {
        // Function-2 block size = 512 (low byte then high byte).
        sdio.cmd52_write(FN0, CCCR_FBR2_BLOCKSIZE, F2_BLOCK_SIZE as u8, timer)?;
        sdio.cmd52_write(
            FN0,
            CCCR_FBR2_BLOCKSIZE + 1,
            (F2_BLOCK_SIZE >> 8) as u8,
            timer,
        )?;

        // Let the SDIO core raise frame-ready / mailbox / flow-change
        // interrupts, and enable the host-side interrupt for functions
        // 1 and 2 (bit 0 is the master enable).
        let core = sdio.sdio_core_base();
        sdio.backplane_write32(
            core + SDIO_CORE_INTMASK,
            INT_FRAME | INT_MAILBOX | INT_FCCHANGE,
            timer,
        )?;
        sdio.cmd52_write(FN0, CCCR_INT_ENABLE, 0b111, timer)?;

        Ok(Self {
            sdio,
            tx_seq: 0,
            tx_window: 0,
            fc_mask: 0,
            req_id: 0,
            frame: [0; MAX_FRAME],
            rx_draining: false,
        })
    }

    /// Loads the chip's CLM (country/regulatory) blob — the data file the
    /// Cypress firmware needs before it will bring the radio up in a
    /// valid regulatory domain. Without it the interface comes up but
    /// scanning and joining fail, and `country` reads back garbage. The
    /// blob is sent to the `clmload` iovar in chunks, each prefixed with
    /// a download header (flags marking the first and last chunk, the
    /// data type, and the length). Call this once, right after
    /// [`Self::new`], before any scan or join.
    pub fn load_clm(&mut self, clm: &[u8], timer: &Timer) -> Result<(), Error> {
        // "clmload" + NUL, then the download header, then the chunk data.
        let mut request = [0u8; 8 + CLM_HEADER_LEN + CLM_CHUNK];
        request[..8].copy_from_slice(b"clmload\0");

        let mut offset = 0;
        while offset < clm.len() {
            let end = (offset + CLM_CHUNK).min(clm.len());
            let chunk = &clm[offset..end];

            let mut flag = CLM_FLAG_BASE;
            if offset == 0 {
                flag |= CLM_FLAG_BEGIN;
            }
            if end == clm.len() {
                flag |= CLM_FLAG_END;
            }
            request[8..10].copy_from_slice(&flag.to_le_bytes());
            request[10..12].copy_from_slice(&CLM_DOWNLOAD_TYPE.to_le_bytes());
            request[12..16].copy_from_slice(&(chunk.len() as u32).to_le_bytes());
            request[16..20].copy_from_slice(&0u32.to_le_bytes()); // crc unused
            request[20..20 + chunk.len()].copy_from_slice(chunk);

            self.command(
                WLC_SET_VAR,
                true,
                &request[..20 + chunk.len()],
                &mut [],
                timer,
            )?;
            offset = end;
        }
        Ok(())
    }

    /// Reads the firmware variable `name` (an "iovar") into `out`,
    /// returning the number of bytes the firmware supplied. A CDC "get"
    /// on the control channel: the request payload is `name` plus a NUL,
    /// and the firmware replies with the value in the same buffer
    /// position.
    ///
    /// For example `get_iovar("cur_etheraddr", &mut mac)` fills six bytes
    /// with the chip's MAC address, and `get_iovar("ver", &mut buf)`
    /// returns the firmware version as an ASCII string.
    pub fn get_iovar(&mut self, name: &str, out: &mut [u8], timer: &Timer) -> Result<usize, Error> {
        let mut request = [0u8; MAX_IOVAR_REQUEST];
        let len = iovar_request(name, &[], &mut request);
        self.command(WLC_GET_VAR, false, &request[..len], out, timer)
    }

    /// Writes the firmware variable `name` to `value` (a CDC "set"
    /// iovar): the payload is `name`, a NUL, then the value bytes.
    pub fn set_iovar(&mut self, name: &str, value: &[u8], timer: &Timer) -> Result<(), Error> {
        let mut request = [0u8; MAX_IOVAR_REQUEST];
        let len = iovar_request(name, value, &mut request);
        self.command(WLC_SET_VAR, true, &request[..len], &mut [], timer)?;
        Ok(())
    }

    /// Writes a `u32`-valued iovar — the common case (little-endian).
    pub fn set_iovar_u32(&mut self, name: &str, value: u32, timer: &Timer) -> Result<(), Error> {
        self.set_iovar(name, &value.to_le_bytes(), timer)
    }

    /// Reads a numeric ioctl `cmd` into `out`, returning the byte count.
    pub fn ioctl_get(&mut self, cmd: u32, out: &mut [u8], timer: &Timer) -> Result<usize, Error> {
        self.command(cmd, false, &[], out, timer)
    }

    /// Writes a numeric ioctl `cmd` with `value` as its payload.
    pub fn ioctl_set(&mut self, cmd: u32, value: &[u8], timer: &Timer) -> Result<(), Error> {
        self.command(cmd, true, value, &mut [], timer)?;
        Ok(())
    }

    /// Writes a `u32`-valued numeric ioctl (little-endian).
    pub fn ioctl_set_u32(&mut self, cmd: u32, value: u32, timer: &Timer) -> Result<(), Error> {
        self.ioctl_set(cmd, &value.to_le_bytes(), timer)
    }

    /// Enables the firmware's async event delivery via the `event_msgs`
    /// bitmask: the default-on mask with a few high-rate events quieted
    /// (as plan9 does). Everything else — the escan results and join
    /// events we rely on — stays on.
    fn enable_events(&mut self, timer: &Timer) -> Result<(), Error> {
        let mut event_mask = [0xffu8; EVENT_MASK_LEN];
        for event in DISABLED_EVENTS {
            event_mask[event / 8] &= !(1u8 << (event % 8));
        }
        self.set_iovar("event_msgs", &event_mask, timer)
    }

    /// Scans the 2.4GHz band for access points, invoking `on_result`
    /// once per AP found. Starts an "escan", whose results arrive as
    /// async events (each carrying one AP's [`ScanResult`]); returns when
    /// the firmware signals the scan is complete or the time budget
    /// expires. The same AP may be reported more than once (seen on
    /// multiple channels/probes) — the caller can deduplicate by BSSID.
    pub fn scan(
        &mut self,
        timer: &Timer,
        mut on_result: impl FnMut(&ScanResult),
    ) -> Result<(), Error> {
        // escan results arrive as async EVENT_ESCAN_RESULT events, so the
        // event mask must be enabled first — the firmware's power-on
        // default does not deliver them.
        self.enable_events(timer)?;
        // The radio must be up before it can scan.
        self.ioctl_set_u32(WLC_UP, 0, timer)?;
        self.ioctl_set_u32(WLC_SET_PASSIVE_SCAN, 0, timer)?;
        let mut params = [0u8; ESCAN_PARAMS_LEN];
        fill_escan_params(&mut params);
        self.set_iovar("escan", &params, timer)?;

        let start = timer.now_micros();
        loop {
            if timer.now_micros() - start > 8_000_000 {
                return Ok(());
            }
            let Some(frame) = self.recv_frame(timer)? else {
                timer.delay_ms(10);
                continue;
            };
            if frame.channel != CHANNEL_EVENT {
                continue;
            }
            // Event message start (see `poll_event` for the offset math).
            let bdc = 4 + ((self.frame[frame.data_offset + 3] as usize) << 2);
            let msg = frame.data_offset + bdc + 14 + 10;
            if msg + 8 > frame.frame_len {
                continue;
            }
            let event_type = u16::from_be_bytes([self.frame[msg + 6], self.frame[msg + 7]]);
            if event_type != EVENT_ESCAN_RESULT {
                continue;
            }
            // The escan result data follows the 48-byte event message:
            // buflen, version, sync_id, then a BSS count and BSS records.
            let data = msg + 48;
            if data + 12 > frame.frame_len {
                continue;
            }
            let bss_count = u16::from_le_bytes([self.frame[data + 10], self.frame[data + 11]]);
            if bss_count == 0 {
                // A result with no BSS marks the scan complete.
                return Ok(());
            }
            if let Some(result) = self.parse_bss_info(data + 12, frame.frame_len) {
                on_result(&result);
            }
        }
    }

    /// Parses one `wl_bss_info` at frame offset `bss` into a
    /// [`ScanResult`] (BSSID, SSID, channel, RSSI), or `None` if it runs
    /// past `frame_len`.
    fn parse_bss_info(&self, bss: usize, frame_len: usize) -> Option<ScanResult> {
        if bss + 80 > frame_len {
            return None;
        }
        let mut result = ScanResult {
            bssid: [0; 6],
            ssid: [0; 32],
            ssid_len: 0,
            channel: 0,
            rssi: 0,
        };
        result.bssid.copy_from_slice(&self.frame[bss + 8..bss + 14]);
        let ssid_len = (self.frame[bss + 18] as usize).min(32);
        result.ssid_len = ssid_len;
        result.ssid[..ssid_len].copy_from_slice(&self.frame[bss + 19..bss + 19 + ssid_len]);
        // chanspec's low byte is the channel; RSSI is a signed dBm value.
        result.channel = self.frame[bss + 72];
        result.rssi = i16::from_le_bytes([self.frame[bss + 78], self.frame[bss + 79]]);
        Some(result)
    }

    /// Joins the WPA2-PSK network `ssid` using `passphrase`, returning
    /// the AP's BSSID once associated. Issues the configuration and join
    /// ([`Self::start_join`]) then polls until the association completes.
    ///
    /// `ssid` is at most 32 bytes; `passphrase` is the ASCII WPA2
    /// passphrase (8..63 bytes). Failing to associate within the budget
    /// — typically a wrong passphrase or an unreachable AP — is
    /// [`Error::NotAssociated`].
    pub fn join_wpa2(
        &mut self,
        ssid: &str,
        passphrase: &str,
        timer: &Timer,
    ) -> Result<[u8; 6], Error> {
        self.start_join(ssid, passphrase, timer)?;

        // Poll the associated BSSID rather than waiting for an E_LINK
        // event: this firmware runs the join to completion (E_SET_SSID and
        // E_PSK_SUP report success) without reliably emitting E_LINK.
        // `WLC_GET_BSSID` fails (BCME_NOTASSOCIATED) or reads back all
        // zeros until the join lands, then returns the AP's address; the
        // read also services the receive path, draining the join events.
        let start = timer.now_micros();
        loop {
            let mut bssid = [0u8; 6];
            if self.ioctl_get(WLC_GET_BSSID, &mut bssid, timer).is_ok()
                && bssid.iter().any(|&b| b != 0)
            {
                return Ok(bssid);
            }
            if timer.now_micros() - start > 15_000_000 {
                return Err(Error::NotAssociated);
            }
            timer.delay_ms(100);
        }
    }

    /// Configures the chip for the WPA2-PSK network `ssid`/`passphrase`
    /// and issues the join, without waiting for the result — the caller
    /// drives its own [`Self::poll_event`] loop (or use [`Self::join_wpa2`]
    /// for the wait). Enables the join events, brings the interface up in
    /// station mode, sets AES-CCMP/WPA2-PSK, hands the passphrase to the
    /// in-firmware supplicant, and sends the SSID.
    pub fn start_join(&mut self, ssid: &str, passphrase: &str, timer: &Timer) -> Result<(), Error> {
        // Ask the firmware to deliver the join-related async events.
        self.enable_events(timer)?;

        // Bring the interface up in infrastructure (station) mode.
        self.ioctl_set_u32(WLC_UP, 0, timer)?;
        self.ioctl_set_u32(WLC_SET_INFRA, 1, timer)?;

        // Security: AES-CCMP, WPA2-PSK, and the in-firmware supplicant so
        // the chip runs the 4-way handshake itself.
        self.set_iovar_u32("wsec", WSEC_AES, timer)?;
        self.set_iovar_u32("wpa_auth", WPA2_AUTH_PSK, timer)?;
        self.set_iovar_u32("sup_wpa", 1, timer)?;

        // Hand over the passphrase as a `wsec_pmk_t` (length, the
        // passphrase flag, then the ASCII passphrase). The whole struct
        // is sent; the firmware honors `key_len`.
        let mut pmk = [0u8; 4 + 256];
        pmk[0..2].copy_from_slice(&(passphrase.len() as u16).to_le_bytes());
        pmk[2..4].copy_from_slice(&WSEC_PMK_PASSPHRASE.to_le_bytes());
        pmk[4..4 + passphrase.len()].copy_from_slice(passphrase.as_bytes());
        self.ioctl_set(WLC_SET_WSEC_PMK, &pmk, timer)?;

        // Join: a `wlc_ssid_t` — length then the zero-padded SSID.
        let mut ssid_param = [0u8; 4 + 32];
        ssid_param[0..4].copy_from_slice(&(ssid.len() as u32).to_le_bytes());
        ssid_param[4..4 + ssid.len()].copy_from_slice(ssid.as_bytes());
        self.ioctl_set(WLC_SET_SSID, &ssid_param, timer)?;
        Ok(())
    }

    /// Reads one pending frame and, if it's an async firmware event,
    /// returns the parsed [`Event`]; returns `None` if nothing is waiting
    /// or the frame isn't an event. Drive this in a loop after
    /// [`Self::start_join`] to follow the join, or any time to service
    /// link-state changes.
    pub fn poll_event(&mut self, timer: &Timer) -> Result<Option<Event>, Error> {
        let Some(frame) = self.recv_frame(timer)? else {
            return Ok(None);
        };
        if frame.channel != CHANNEL_EVENT {
            return Ok(None);
        }
        // After the SDPCM header (at `data_offset`) comes the BDC header
        // (4 bytes + `data_offset[3]`×4 of firmware-signal TLV), then the
        // event packet: a 14-byte Ethernet header, a 10-byte Broadcom
        // header, then the big-endian event message.
        let bdc = 4 + ((self.frame[frame.data_offset + 3] as usize) << 2);
        let msg = frame.data_offset + bdc + 14 + 10;
        if msg + 12 > frame.frame_len {
            return Ok(None);
        }
        // event_type is the low 16 bits of a big-endian u32 at offset 4.
        Ok(Some(Event {
            flags: u16::from_be_bytes([self.frame[msg + 2], self.frame[msg + 3]]),
            event_type: u16::from_be_bytes([self.frame[msg + 6], self.frame[msg + 7]]),
            status: u32::from_be_bytes([
                self.frame[msg + 8],
                self.frame[msg + 9],
                self.frame[msg + 10],
                self.frame[msg + 11],
            ]),
        }))
    }

    /// Largest Ethernet frame [`Self::send_ethernet`]/[`Self::recv_ethernet`]
    /// move, in bytes: a 14-byte header plus a 1500-byte payload.
    pub const MTU: usize = 1514;

    /// Sends one Ethernet `frame` over the network data channel, wrapping
    /// it in the SDPCM data header and a BDC header. `frame` is a complete
    /// Ethernet frame (destination/source/ethertype then payload), at most
    /// [`Self::MTU`] bytes.
    ///
    /// The firmware gates the data channel with a credit window and a
    /// flow-control flag; when it's out of credit or has paused the
    /// channel this returns [`Error::TxBusy`] without sending. The window
    /// advances as received frames arrive (their SDPCM headers carry it),
    /// so a caller that also services receives will see the window reopen.
    pub fn send_ethernet(&mut self, frame: &[u8], timer: &Timer) -> Result<(), Error> {
        if frame.len() > Self::MTU {
            return Err(Error::FrameTooLong);
        }
        // Respect the firmware's flow control: no transmit credit left
        // (host sequence has caught up to the advertised ceiling) or the
        // data channel is paused.
        if self.tx_seq == self.tx_window || self.fc_mask & DATA_FC_BIT != 0 {
            return Err(Error::TxBusy);
        }

        let total = SDPCM_HEADER_LEN + BDC_HEADER_LEN + frame.len();
        let padded = total.next_multiple_of(4);
        self.frame[..padded].fill(0);
        // SDPCM header: length + complement, sequence, data channel, and
        // the data offset (start of the BDC header).
        self.frame[0..2].copy_from_slice(&(total as u16).to_le_bytes());
        self.frame[2..4].copy_from_slice(&(!(total as u16)).to_le_bytes());
        self.frame[4] = self.tx_seq;
        self.frame[5] = CHANNEL_DATA;
        self.frame[7] = SDPCM_HEADER_LEN as u8;
        // BDC header at offset 12: version-2 flags, zero priority/flags2,
        // and a zero data-offset (the Ethernet frame follows immediately).
        self.frame[SDPCM_HEADER_LEN] = BDC_FLAG_VERSION;
        // Ethernet frame after the SDPCM + BDC headers.
        let data_at = SDPCM_HEADER_LEN + BDC_HEADER_LEN;
        self.frame[data_at..data_at + frame.len()].copy_from_slice(frame);

        self.sdio.f2_write(&self.frame[..padded], timer)?;
        self.tx_seq = self.tx_seq.wrapping_add(1);
        Ok(())
    }

    /// Receives one Ethernet frame from the network data channel into
    /// `out`, returning its length, or `None` if nothing is waiting (or
    /// the next frame is control/event traffic, which this drops). Strips
    /// the SDPCM and BDC headers, leaving a complete Ethernet frame.
    pub fn recv_ethernet(&mut self, out: &mut [u8], timer: &Timer) -> Result<Option<usize>, Error> {
        let Some(frame) = self.recv_frame(timer)? else {
            return Ok(None);
        };
        if frame.channel != CHANNEL_DATA {
            return Ok(None);
        }
        // After the SDPCM header (at `data_offset`) comes the BDC header:
        // 4 bytes plus `data_offset[3]`×4 of optional firmware signalling.
        let bdc = BDC_HEADER_LEN + ((self.frame[frame.data_offset + 3] as usize) << 2);
        let start = frame.data_offset + bdc;
        if start >= frame.frame_len {
            return Ok(None);
        }
        let len = (frame.frame_len - start).min(out.len());
        out[..len].copy_from_slice(&self.frame[start..start + len]);
        Ok(Some(len))
    }

    /// Runs one CDC command round-trip: builds the SDPCM control frame
    /// with the CDC header (`cmd`, a set/get flag, and a fresh request
    /// id) and `request` as payload, sends it, and awaits the matching
    /// response, copying up to `reply.len()` value bytes back. On a get,
    /// the firmware overwrites the request payload with the value, so the
    /// payload region is sized to the larger of the two.
    fn command(
        &mut self,
        cmd: u32,
        set: bool,
        request: &[u8],
        reply: &mut [u8],
        timer: &Timer,
    ) -> Result<usize, Error> {
        let payload_len = if set {
            request.len()
        } else {
            request.len().max(reply.len())
        };
        let total = SDPCM_HEADER_LEN + CDC_HEADER_LEN + payload_len;
        let padded = total.next_multiple_of(4);
        if padded > MAX_FRAME {
            return Err(Error::NoResponse);
        }

        self.req_id = self.req_id.wrapping_add(1);
        let seq = self.tx_seq;
        let req_id = self.req_id;

        self.frame[..padded].fill(0);
        // SDPCM header: length + its complement, sequence, control
        // channel, and the data offset (start of the CDC header).
        self.frame[0..2].copy_from_slice(&(total as u16).to_le_bytes());
        self.frame[2..4].copy_from_slice(&(!(total as u16)).to_le_bytes());
        self.frame[4] = seq;
        self.frame[5] = CHANNEL_CONTROL;
        self.frame[7] = SDPCM_HEADER_LEN as u8;
        // CDC header at offset 12: command, payload length, flags
        // (bit 1 = set), request id.
        self.frame[12..16].copy_from_slice(&cmd.to_le_bytes());
        self.frame[16..20].copy_from_slice(&(payload_len as u32).to_le_bytes());
        self.frame[20..22].copy_from_slice(&(if set { CDC_FLAG_SET } else { 0 }).to_le_bytes());
        self.frame[22..24].copy_from_slice(&req_id.to_le_bytes());
        // Request payload at offset 28.
        let payload_at = SDPCM_HEADER_LEN + CDC_HEADER_LEN;
        self.frame[payload_at..payload_at + request.len()].copy_from_slice(request);

        self.sdio.f2_write(&self.frame[..padded], timer)?;
        self.tx_seq = self.tx_seq.wrapping_add(1);
        self.await_response(req_id, reply, timer)
    }

    /// Reads frames until one is a control response matching `req_id`,
    /// copying its value into `out` and returning the value length.
    /// Frames on other channels (async events, data) are read to keep
    /// the receive stream in sync — and their headers update flow
    /// control — but are otherwise ignored here.
    fn await_response(
        &mut self,
        req_id: u16,
        out: &mut [u8],
        timer: &Timer,
    ) -> Result<usize, Error> {
        let start = timer.now_micros();
        loop {
            if timer.now_micros() - start > 2_000_000 {
                return Err(Error::NoResponse);
            }
            let Some(frame) = self.recv_frame(timer)? else {
                timer.delay_ms(1);
                continue;
            };
            if frame.channel != CHANNEL_CONTROL {
                continue;
            }
            let cdc = frame.data_offset;
            let id = u16::from_le_bytes([self.frame[cdc + 10], self.frame[cdc + 11]]);
            if id != req_id {
                continue;
            }
            let flags = u16::from_le_bytes([self.frame[cdc + 8], self.frame[cdc + 9]]);
            let status = i32::from_le_bytes([
                self.frame[cdc + 12],
                self.frame[cdc + 13],
                self.frame[cdc + 14],
                self.frame[cdc + 15],
            ]);
            if flags & CDC_FLAG_ERROR != 0 || status != 0 {
                return Err(Error::CommandFailed(status));
            }
            let value_len = u32::from_le_bytes([
                self.frame[cdc + 4],
                self.frame[cdc + 5],
                self.frame[cdc + 6],
                self.frame[cdc + 7],
            ]) as usize;
            let value_at = cdc + CDC_HEADER_LEN;
            let available = frame.frame_len.saturating_sub(value_at);
            let n = value_len.min(out.len()).min(available);
            out[..n].copy_from_slice(&self.frame[value_at..value_at + n]);
            return Ok(n);
        }
    }

    /// Whether the firmware has a frame ready to read. The frame-ready
    /// interrupt must be checked before reading function 2 — reading an
    /// empty receive FIFO stalls. When the CCCR interrupt-pending flag is
    /// set, this reads and clears the SDIO core's interrupt status and
    /// reports whether the frame-ready bit was among the causes.
    fn frame_ready(&mut self, timer: &Timer) -> Result<bool, Error> {
        if self.sdio.cmd52_read(FN0, CCCR_INT_PENDING, timer)? == 0 {
            return Ok(false);
        }
        let core = self.sdio.sdio_core_base();
        let status = self
            .sdio
            .backplane_read32(core + SDIO_CORE_INTSTATUS, timer)?;
        self.sdio
            .backplane_write32(core + SDIO_CORE_INTSTATUS, status, timer)?;
        Ok(status & INT_FRAME != 0)
    }

    /// Reads one SDPCM frame into [`Self::frame`]: the 12-byte header
    /// first (whose length field says how much more to read, or `0` for
    /// "no frame pending"), then the rest. Updates flow control from the
    /// header. Returns `None` when nothing is waiting.
    fn read_frame(&mut self, timer: &Timer) -> Result<Option<FrameInfo>, Error> {
        self.sdio
            .f2_read(&mut self.frame[..SDPCM_HEADER_LEN], timer)?;

        let len = u16::from_le_bytes([self.frame[0], self.frame[1]]) as usize;
        if len == 0 {
            return Ok(None);
        }
        let len_check = u16::from_le_bytes([self.frame[2], self.frame[3]]);
        if len_check != !(len as u16) || !(SDPCM_HEADER_LEN..=MAX_FRAME).contains(&len) {
            return Err(Error::BadFrame);
        }

        // Adopt the firmware's advertised credit window and flow-control
        // mask. Guard against a garbage window byte the way brcmfmac
        // does: if it's implausibly far ahead of our sequence, allow
        // just one more frame instead.
        let window = self.frame[9];
        self.tx_window = if window.wrapping_sub(self.tx_seq) > 0x40 {
            self.tx_seq.wrapping_add(2)
        } else {
            window
        };
        self.fc_mask = self.frame[8];

        let channel = self.frame[5] & 0x0f;
        let data_offset = self.frame[7] as usize;

        // Read the rest of the frame (function-2 transfers are 4-byte
        // granular, so round up).
        if len > SDPCM_HEADER_LEN {
            let rest = (len - SDPCM_HEADER_LEN).next_multiple_of(4);
            self.sdio.f2_read(
                &mut self.frame[SDPCM_HEADER_LEN..SDPCM_HEADER_LEN + rest],
                timer,
            )?;
        }

        Ok(Some(FrameInfo {
            channel,
            data_offset,
            frame_len: len,
        }))
    }

    /// Receives the next SDPCM frame, or `None` when the receive FIFO is
    /// empty.
    ///
    /// The firmware raises the frame-ready interrupt only when the FIFO
    /// goes from empty to non-empty; frames queued behind that first one —
    /// a control response sitting behind an async event, say — get no
    /// interrupt of their own. So gating every read on the interrupt would
    /// strand them. Instead, once the interrupt has fired this reads
    /// frames back-to-back until a zero-length header signals the FIFO is
    /// drained, and only then waits for the next interrupt — the receive
    /// model brcmfmac and plan9 both use.
    fn recv_frame(&mut self, timer: &Timer) -> Result<Option<FrameInfo>, Error> {
        if !self.rx_draining {
            if !self.frame_ready(timer)? {
                return Ok(None);
            }
            self.rx_draining = true;
        }
        match self.read_frame(timer)? {
            Some(frame) => Ok(Some(frame)),
            None => {
                self.rx_draining = false;
                Ok(None)
            }
        }
    }

    /// Clears any pending SDIO-core interrupt status — the frame-ready
    /// bit is level-triggered off the receive FIFO, so this is only for
    /// draining stale mailbox/flow-change events. Exposed for callers
    /// that drive their own receive loop.
    pub fn clear_interrupts(&mut self, timer: &Timer) -> Result<(), Error> {
        let core = self.sdio.sdio_core_base();
        let status = self
            .sdio
            .backplane_read32(core + SDIO_CORE_INTSTATUS, timer)?;
        if status != 0 {
            self.sdio
                .backplane_write32(core + SDIO_CORE_INTSTATUS, status, timer)?;
        }
        Ok(())
    }

    /// Returns the wrapped [`Sdio`], e.g. to reuse the controller.
    pub fn free(self) -> Sdio {
        self.sdio
    }
}

/// Assembles an iovar request into `out`: the variable `name`, a NUL
/// terminator, then `value`. Returns the total length. `out` must be
/// large enough (see [`MAX_IOVAR_REQUEST`]).
fn iovar_request(name: &str, value: &[u8], out: &mut [u8]) -> usize {
    let n = name.len();
    out[..n].copy_from_slice(name.as_bytes());
    out[n] = 0;
    out[n + 1..n + 1 + value.len()].copy_from_slice(value);
    n + 1 + value.len()
}

/// An asynchronous event delivered by the firmware over the SDPCM event
/// channel — a join result, link change, supplicant progress, and so on.
/// Returned by [`Wifi::poll_event`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    /// The event code (e.g. [`EVENT_LINK`], [`EVENT_SET_SSID`]).
    pub event_type: u16,
    /// The event's status code — meaning depends on the event; `0` is
    /// generally success.
    pub status: u32,
    /// Event flags (e.g. [`EVENT_LINK_UP`] on an [`EVENT_LINK`]).
    pub flags: u16,
}

/// One access point found by [`Wifi::scan`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScanResult {
    /// The AP's BSSID (its MAC address).
    pub bssid: [u8; 6],
    /// The SSID bytes (not NUL-terminated); see [`Self::ssid`].
    pub ssid: [u8; 32],
    /// Number of valid bytes in [`Self::ssid`].
    pub ssid_len: usize,
    /// The 2.4GHz channel the AP was found on.
    pub channel: u8,
    /// Received signal strength, in dBm (negative; closer to 0 is
    /// stronger).
    pub rssi: i16,
}

impl ScanResult {
    /// The SSID as a byte slice (may not be valid UTF-8 or printable).
    pub fn ssid(&self) -> &[u8] {
        &self.ssid[..self.ssid_len]
    }
}

/// Fills `out` (length [`ESCAN_PARAMS_LEN`]) with the `escan` parameters
/// for an all-SSID active scan of the fourteen 2.4GHz channels — the
/// wildcard scan plan9's `wlscanstart` issues. Layout: an escan header
/// (version, start action, sync id) then a `wl_scan_params` (wildcard
/// SSID and BSSID, "any" BSS type, default timings, the channel list).
fn fill_escan_params(out: &mut [u8]) {
    out[..ESCAN_PARAMS_LEN].fill(0);
    out[0..4].copy_from_slice(&1u32.to_le_bytes()); // escan version
    out[4..6].copy_from_slice(&1u16.to_le_bytes()); // action = start
    out[6..8].copy_from_slice(&0x1234u16.to_le_bytes()); // sync id
                                                         // ssid_len (8) = 0, ssid[32] (12) = 0 → wildcard.
    out[44..50].fill(0xff); // bssid = wildcard
    out[50] = 2; // bss_type = any
    out[51] = 0; // scan_type = active
    out[52..68].fill(0xff); // nprobes / active / passive / home = -1 (defaults)
    out[68..70].copy_from_slice(&14u16.to_le_bytes()); // channel count
    out[70..72].copy_from_slice(&1u16.to_le_bytes()); // ssid count (one wildcard)
                                                      // Fourteen 2.4GHz chanspecs (channel | 20MHz-band bits), as plan9's.
    let chanspecs: [u16; 14] = [
        0x2b01, 0x2b02, 0x2b03, 0x2b04, 0x2e05, 0x2e06, 0x2e07, 0x2b08, 0x2b09, 0x2b0a, 0x2b0b,
        0x2b0c, 0x2b0d, 0x2b0e,
    ];
    for (i, chanspec) in chanspecs.iter().enumerate() {
        out[72 + i * 2..74 + i * 2].copy_from_slice(&chanspec.to_le_bytes());
    }
    // ssids[1][32] at 100..132 and the 4-byte tail (132..136) stay zero.
}

/// What [`Wifi::read_frame`] learned about the frame it just read.
struct FrameInfo {
    /// SDPCM channel (control, event, data, …).
    channel: u8,
    /// Byte offset within the frame where the payload (the CDC header,
    /// for control frames) begins.
    data_offset: usize,
    /// Total frame length, in bytes, including the SDPCM header.
    frame_len: usize,
}

#[cfg(feature = "smoltcp")]
use smoltcp::phy::{Device as PhyDevice, DeviceCapabilities, Medium, RxToken, TxToken};
#[cfg(feature = "smoltcp")]
use smoltcp::time::Instant;

/// A [`smoltcp`] [`Device`](smoltcp::phy::Device) over a joined [`Wifi`]:
/// it moves the stack's Ethernet frames through the chip's SDPCM data
/// channel. Construct it with [`WifiPhy::new`] from a [`Wifi`] that has
/// already [`join_wpa2`](Wifi::join_wpa2)'d a network, then hand it to
/// `smoltcp`'s [`Interface`](smoltcp::iface::Interface).
///
/// `smoltcp` hands out a receive and a transmit token together from one
/// `&mut self`; both would need the chip at once. The adapter sidesteps
/// this the same way [`Lan9514Phy`](crate::usb::lan9514::Lan9514Phy) does:
/// the receive is done up front and the frame copied into a buffer the RX
/// token owns outright, leaving the TX token the sole borrower.
///
/// Available only with the `smoltcp` feature enabled.
#[cfg(feature = "smoltcp")]
pub struct WifiPhy<'a> {
    wifi: Wifi,
    timer: &'a Timer,
    /// Scratch the TX token fills for smoltcp and hands to the driver.
    tx_scratch: [u8; Wifi::MTU],
}

#[cfg(feature = "smoltcp")]
impl<'a> WifiPhy<'a> {
    /// Wraps a joined [`Wifi`] as a smoltcp device, borrowing the timer it
    /// drives frames through.
    pub fn new(wifi: Wifi, timer: &'a Timer) -> Self {
        Self {
            wifi,
            timer,
            tx_scratch: [0; Wifi::MTU],
        }
    }

    /// Returns the wrapped [`Wifi`], e.g. to issue further control
    /// commands or tear the connection down.
    pub fn free(self) -> Wifi {
        self.wifi
    }
}

#[cfg(feature = "smoltcp")]
impl PhyDevice for WifiPhy<'_> {
    type RxToken<'t>
        = WifiRxToken
    where
        Self: 't;
    type TxToken<'t>
        = WifiTxToken<'t>
    where
        Self: 't;

    /// Pulls a data frame from the chip (if any) and returns it paired
    /// with a transmit token. The received bytes are copied into the RX
    /// token so the driver is free for the TX token returned alongside —
    /// see the type docs.
    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let mut rx = WifiRxToken {
            buffer: [0; Wifi::MTU],
            len: 0,
        };
        match self.wifi.recv_ethernet(&mut rx.buffer, self.timer) {
            Ok(Some(len)) => rx.len = len,
            Ok(None) => return None,
            Err(_) => return None,
        }
        let tx = WifiTxToken {
            wifi: &mut self.wifi,
            timer: self.timer,
            scratch: &mut self.tx_scratch,
        };
        Some((rx, tx))
    }

    /// Returns a transmit token borrowing the driver.
    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        Some(WifiTxToken {
            wifi: &mut self.wifi,
            timer: self.timer,
            scratch: &mut self.tx_scratch,
        })
    }

    /// Reports an Ethernet medium with [`Wifi::MTU`] and a one-frame burst
    /// (the driver's frame calls are synchronous and one-at-a-time).
    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.medium = Medium::Ethernet;
        caps.max_transmission_unit = Wifi::MTU;
        caps.max_burst_size = Some(1);
        caps
    }
}

/// An owned copy of one received frame, produced by [`WifiPhy::receive`].
/// Owning the bytes (instead of borrowing the driver's buffer) is what
/// lets the driver be handed to the TX token returned alongside it.
///
/// Available only with the `smoltcp` feature enabled.
#[cfg(feature = "smoltcp")]
pub struct WifiRxToken {
    buffer: [u8; Wifi::MTU],
    len: usize,
}

#[cfg(feature = "smoltcp")]
impl RxToken for WifiRxToken {
    /// Hands the received frame's bytes to `f`.
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        f(&self.buffer[..self.len])
    }
}

/// A pending transmit from [`WifiPhy`]: smoltcp fills the scratch buffer
/// via [`consume`](TxToken::consume), then the frame goes out the chip's
/// SDPCM data channel.
///
/// Available only with the `smoltcp` feature enabled.
#[cfg(feature = "smoltcp")]
pub struct WifiTxToken<'a> {
    wifi: &'a mut Wifi,
    timer: &'a Timer,
    scratch: &'a mut [u8],
}

#[cfg(feature = "smoltcp")]
impl TxToken for WifiTxToken<'_> {
    /// Lets `f` fill the frame buffer, then sends it. A failed send is
    /// dropped: smoltcp treats transmission as best-effort (retransmission
    /// is a higher layer's job), and a transient [`Error::TxBusy`] clears
    /// once the credit window reopens on the next received frame.
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let result = f(&mut self.scratch[..len]);
        let _ = self.wifi.send_ethernet(&self.scratch[..len], self.timer);
        result
    }
}
