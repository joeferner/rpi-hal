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
//! One received frame is not always one packet. Under load the firmware
//! coalesces several into a single *superframe* — its answer to the
//! per-frame cost of the bus — and does so whether or not the host asks
//! it to. `recv_ethernet` reads one of those once and hands the packets
//! inside it back one call at a time, so a caller sees no difference.
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
/// SDPCM channel carrying a *superframe*: several complete frames the
/// firmware has coalesced into one, to amortize the per-frame cost of
/// getting them across the bus. It does this whenever it has more than
/// one frame in hand, which in practice means under any sustained
/// download — and it does it whether or not the host asked (see
/// [`Wifi::set_rx_glom`]), so reading one is not optional.
const CHANNEL_GLOM: u8 = 3;

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
/// Firmware command: set the power-management mode (see
/// [`PowerManagement`]).
const WLC_SET_PM: u32 = 86;
/// Firmware command: read the associated AP's BSSID (fails with a
/// not-associated status when the chip isn't on a network).
const WLC_GET_BSSID: u32 = 23;
/// Firmware command: read the current link rate, in units of 500 kbit/s.
const WLC_GET_RATE: u32 = 12;

/// Room requested for the `counters` reply.
///
/// Sized against the largest layout rather than against the part that is
/// read, because the firmware checks the room it is offered against the
/// whole structure and refuses the command outright if it is short — a
/// buffer big enough for the fields wanted is not big enough to ask with.
/// The versions in [`COUNTERS_VERSIONS`] run to a few hundred bytes and
/// grow with each one, so this is generous on purpose; only the leading
/// [`COUNTERS_PREFIX`] bytes are decoded.
const COUNTERS_REPLY: usize = 1024;

/// Bytes of the `counters` reply [`Counters`] decodes.
///
/// The structure's leading fields — through `rxuflo` — are identical
/// across every layout version in [`COUNTERS_VERSIONS`]; what differs
/// between them is what follows.
const COUNTERS_PREFIX: usize = 156;

/// `counters` layout versions whose leading fields match what
/// [`Counters`] reads.
///
/// Later firmware answers this iovar with a tagged-and-length-prefixed
/// format instead of a flat structure, under its own much higher version
/// number — which is why this is a list of known-good layouts rather than
/// a minimum. Anything outside it is refused rather than misread.
const COUNTERS_VERSIONS: core::ops::RangeInclusive<u16> = 6..=11;
/// Firmware command: read the received signal strength, in dBm.
const WLC_GET_RSSI: u32 = 127;
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
/// The backplane function, which carries the chip's own control
/// registers — including the frame-control register [`resync_rx`](
/// Wifi::resync_rx) writes.
const FN1: u32 = 1;

/// Function-1 frame control: writing [`SFC_RF_TERM`] here abandons the
/// receive frame in progress.
const SBSDIO_FUNC1_FRAMECTRL: u32 = 0x1_000d;
/// "Read frame terminate", in [`SBSDIO_FUNC1_FRAMECTRL`].
const SFC_RF_TERM: u8 = 1 << 0;
/// Low byte of how much of the current receive frame the chip still
/// holds. Zero in both halves is how a terminated frame reports that it
/// has been flushed.
const SBSDIO_FUNC1_RFRAMEBCLO: u32 = 0x1_001b;
/// High byte of the same count.
const SBSDIO_FUNC1_RFRAMEBCHI: u32 = 0x1_001c;

/// How many times [`Wifi::resync_rx`] asks whether the abandoned frame
/// has drained before giving up.
///
/// Each pass is two `CMD52` reads, so this is tens of milliseconds
/// rather than a number of microseconds. It is a bound on a wait that
/// should take a handful of passes, not a budget anything is expected to
/// spend.
const RESYNC_POLLS: u32 = 1024;

/// Largest SDPCM frame this driver builds or accepts, in bytes.
///
/// The length field is 16 bits, so this is its ceiling rather than a
/// budget: no frame the chip can describe is too large to read, and
/// "the buffer was too small" is a failure that cannot happen.
///
/// It is sized that way because of superframes ([`CHANNEL_GLOM`]). A
/// coalesced frame is as large as however many packets the firmware had
/// in hand — 13,856 bytes, nine of them, measured on a 43430 — and one
/// that will not fit is not a frame lost but *every packet inside it*,
/// which is a stalled download rather than a dropped packet. Picking a
/// number smaller than the field would mean picking how many packets it
/// takes to break, and there is no answer to that worth having on a
/// board with hundreds of megabytes of RAM.
const MAX_FRAME: usize = 64 * 1024;

/// Largest outgoing data frame, in bytes: an Ethernet frame plus the two
/// headers in front of it, rounded up to the word the FIFO moves.
///
/// Its own buffer rather than a share of [`MAX_FRAME`] — see
/// [`Wifi::tx`].
const TX_FRAME_MAX: usize = (SDPCM_HEADER_LEN + BDC_HEADER_LEN + Wifi::MTU + 3) & !3;

/// Errors from the Wi-Fi protocol layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// An error from the underlying SDIO link.
    Sdio(sdio::Error),
    /// A received SDPCM frame header was malformed — the function-2
    /// receive stream is out of sync.
    ///
    /// **The stream has been resynchronized by the time this is
    /// returned** (see [`Wifi::resync_rx`]), so a caller's receive loop
    /// should report this and carry on rather than treat it as fatal.
    /// Without that, one bad header is permanent: the header has been
    /// consumed and its body has not, so every read after it starts
    /// mid-frame and fails the same way, forever.
    ///
    /// The two fields are what says *which* malformation it was, and
    /// they answer different questions:
    ///
    /// * `len_check` not the complement of `len` — both usually
    ///   nonsense — is a stream that has lost its place.
    /// * a valid complement means the stream is in step and the header
    ///   itself is unusable — a length past the end of the buffer, or
    ///   shorter than a header. A coalesced frame is *not* this case:
    ///   those are read and unpacked by [`Wifi::recv_ethernet`], and the
    ///   buffer is the length field's own ceiling, so there is no size
    ///   the chip can name that will not fit.
    BadFrame {
        /// The length word from the header.
        len: u16,
        /// The word that should be its complement.
        len_check: u16,
        /// The SDPCM channel the header names — 1 control, 2 data, 3 a
        /// glommed superframe. Meaningless when the complement is wrong,
        /// and the whole answer when it is right: channel 3 is the
        /// firmware coalescing frames, and anything else that size is
        /// not.
        channel: u8,
    },
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
    /// The firmware answered in a structure layout this driver does not
    /// know how to read — see [`Wifi::counters`], which is the only thing
    /// that returns this.
    UnsupportedFormat {
        /// The version the firmware stamped on the structure.
        version: u16,
    },
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
    /// Buffer holding the frame most recently read from the chip, and
    /// the one a control command is built in.
    frame: [u8; MAX_FRAME],
    /// Buffer an outgoing *data* frame is built in.
    ///
    /// Separate from [`Self::frame`] for one reason, and it is load
    /// bearing: a coalesced frame is handed out a packet at a time from
    /// that buffer, and a transmit in the middle of that would overwrite
    /// the packets still to come. Sharing it would mean every
    /// acknowledgement sent during a download threw away the rest of the
    /// superframe it was acknowledging — which is most of them.
    tx: [u8; TX_FRAME_MAX],
    /// Whether the receive FIFO is being drained: `true` between the
    /// frame-ready interrupt firing and the zero-length header that marks
    /// the FIFO empty. While set, [`Self::recv_frame`] reads the next
    /// frame directly rather than waiting for an interrupt that won't come
    /// for a frame already queued.
    rx_draining: bool,
    /// How far through a coalesced frame [`Self::recv_ethernet`] has
    /// got, or `None` when it is not part-way through one.
    ///
    /// A superframe is read from the bus once and handed out a packet at
    /// a time, so this is what makes the next call return the next
    /// packet instead of reading the bus again.
    glom: Option<Glom>,
}

/// Where [`Wifi::recv_ethernet`] has got to in a coalesced frame.
#[derive(Clone, Copy)]
struct Glom {
    /// Offset in [`Wifi::frame`] to resume searching from.
    at: usize,
    /// Offset one past the superframe's last byte.
    end: usize,
}

/// Finds the next subframe header in `frame` at or after `from`,
/// returning where it starts and the length it declares.
///
/// The recognition test is the SDPCM header's own: a length word
/// followed by that length's complement. See [`Wifi::next_subframe`] for
/// why the walk searches rather than striding by a fixed padding.
fn find_subframe(frame: &[u8], from: usize) -> Option<(usize, usize)> {
    // Four-byte steps because every SDPCM frame the chip emits starts on
    // a word boundary; nothing here would work if one did not.
    let mut at = from.next_multiple_of(4);

    while at + SDPCM_HEADER_LEN <= frame.len() {
        let len = u16::from_le_bytes([frame[at], frame[at + 1]]) as usize;
        let check = u16::from_le_bytes([frame[at + 2], frame[at + 3]]);
        // A length that does not run past the end, with its complement
        // beside it. Zero is padding rather than an answer -- inside a
        // superframe there is no "nothing more to read" to signal.
        if len != 0 && check == !(len as u16) && len >= SDPCM_HEADER_LEN && at + len <= frame.len()
        {
            return Some((at, len));
        }
        at += 4;
    }
    None
}

impl Wifi {
    /// Wraps a firmware-loaded [`Sdio`] and readies the SDPCM protocol
    /// path: sets function 2's block size, unmasks the SDIO core's
    /// frame/mailbox interrupts, enables the host-side SDIO interrupt,
    /// and asks the firmware not to coalesce received frames — a
    /// request some firmware grants and some ignores, see
    /// [`Self::set_rx_glom`]. The chip's firmware must already be
    /// running (see [`Sdio::load_firmware`]).
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

        let mut wifi = Self {
            sdio,
            tx_seq: 0,
            tx_window: 0,
            fc_mask: 0,
            req_id: 0,
            frame: [0; MAX_FRAME],
            tx: [0; TX_FRAME_MAX],
            rx_draining: false,
            glom: None,
        };

        // **Tell the firmware not to coalesce received frames.**
        //
        // With glomming on, the chip packs several Ethernet frames into
        // one SDPCM frame — measured at 13,856 bytes on a 43430 running
        // 7.45.98, nine packets in one — and this driver has no
        // deglomming path: it would read a well-formed header whose
        // length is several times [`MAX_FRAME`] and drop the whole
        // superframe, taking every packet in it. The symptom is a link
        // that works for small traffic and collapses under a download,
        // which is where the firmware starts having several frames to
        // coalesce in the first place.
        //
        // A preference, not a requirement: coalesced frames are unpacked
        // either way, and 7.45.98 answers this with success and
        // coalesces regardless. Asked because on a firmware that does
        // honour it, one frame per read is the cheaper arrangement for a
        // driver with no scatter-gather to spend. See
        // [`Self::set_rx_glom`].
        let _ = wifi.set_rx_glom(false, timer);

        Ok(wifi)
    }

    /// Asks the firmware to coalesce received frames, or not to.
    ///
    /// **Asking is all this does, and one firmware is known to say yes
    /// and carry on coalescing.** A 43430 running 7.45.98 returns
    /// success for `false` and then sends superframes anyway: channel 3,
    /// a well-formed header, and a length of `32 + n × 1536` — several
    /// Ethernet frames padded to the block size and packed into one.
    ///
    /// Which is why nothing depends on the answer. Coalesced frames are
    /// read and handed out a packet at a time either way (see
    /// [`Wifi::recv_ethernet`]), the same way
    /// `brcmfmac` handles them unconditionally — and that is why it has
    /// no "off" switch to copy: it sets this same iovar to *enable*
    /// coalescing, and supports it regardless.
    ///
    /// So this is here for the throughput question rather than the
    /// correctness one. Coalescing is the firmware's answer to the
    /// per-frame cost of the bus, and on a firmware that honours the
    /// request, turning it off trades that away.
    ///
    /// [`Self::new`] asks for `false` and ignores the answer. This is
    /// public so a caller can see what the firmware said, and ask again
    /// later — the answer can depend on how far bring-up has got.
    pub fn set_rx_glom(&mut self, enabled: bool, timer: &Timer) -> Result<(), Error> {
        self.set_iovar_u32("bus:rxglom", u32::from(enabled), timer)
    }

    /// The rate the link is currently running at, in kbit/s.
    ///
    /// The rate the two ends settled on, not the rate they are capable
    /// of: a link that has fallen back to the 802.11b rates reports one
    /// or two megabits here, and that is a ceiling nothing above it can
    /// argue with — a transfer that seems mysteriously slow is often
    /// just this number being small.
    ///
    /// Only meaningful while associated.
    pub fn link_rate_kbps(&mut self, timer: &Timer) -> Result<u32, Error> {
        let mut value = [0u8; 4];
        self.ioctl_get(WLC_GET_RATE, &mut value, timer)?;
        // The firmware counts in half-megabits.
        Ok(u32::from_le_bytes(value).saturating_mul(500))
    }

    /// The received signal strength, in dBm.
    ///
    /// Negative, and closer to zero is stronger: around -50 is a radio in
    /// the same room, around -80 is one that still associates and whose
    /// link rate has collapsed to keep it associated. The companion to
    /// [`Self::link_rate_kbps`] — the rate says what the link is doing
    /// and this says why.
    ///
    /// Only meaningful while associated.
    pub fn rssi_dbm(&mut self, timer: &Timer) -> Result<i32, Error> {
        let mut value = [0u8; 4];
        self.ioctl_get(WLC_GET_RSSI, &mut value, timer)?;
        Ok(i32::from_le_bytes(value))
    }

    /// The firmware's own MAC-layer counters.
    ///
    /// Everything else this driver reports is counted above the chip:
    /// frames the host managed to move, and errors the host could see. A
    /// frame the radio retried four times and then delivered is not an
    /// error anywhere in that picture — it cost air time and latency and
    /// arrived intact — so a link that is working hard and a link that is
    /// working well look identical from the host. These are the counters
    /// that tell them apart.
    ///
    /// Read [`Counters::txretrans`] against [`Counters::txframe`] for how
    /// much of the transmit effort is repeat work, and
    /// [`Counters::rxoflo`] for frames the chip took off the air and then
    /// dropped because the host had not emptied its receive FIFO — the
    /// one loss on this path that nothing else counts, because the frame
    /// never reaches the host to be counted.
    ///
    /// Cumulative since the firmware started, so what they are worth is
    /// the difference between two reads.
    pub fn counters(&mut self, timer: &Timer) -> Result<Counters, Error> {
        let mut reply = [0u8; COUNTERS_REPLY];
        let len = self.get_iovar("counters", &mut reply, timer)?;
        Counters::parse(&reply[..len])
    }

    /// Sets how aggressively the radio may sleep between frames.
    ///
    /// The firmware powers on in [`PowerManagement::Fast`], so a caller
    /// that never asks gets a radio that sleeps. That is the right default
    /// for something battery-powered and the wrong one for a board on a
    /// wall supply, which is why this is a decision rather than a default:
    /// `brcmfmac` and `cyw43` both set it explicitly after associating for
    /// the same reason.
    ///
    /// # What sleeping costs
    ///
    /// Not throughput directly — it costs *latency*, and only when the
    /// link goes briefly quiet. A round trip measured with a ping is a
    /// single packet against an otherwise idle radio, which is the one
    /// case a sleeping chip handles well, so an idle latency that looks
    /// healthy says nothing about this setting.
    ///
    /// Where it shows up is a window-limited bulk transfer, which is
    /// quiet by construction: the sender fills the receive window and
    /// waits, and a radio that treats that pause as idleness adds its
    /// wake-up to every round trip. Throughput is the window divided by
    /// the round trip, so the cost lands on the whole transfer rather
    /// than on the pauses.
    ///
    /// Call after joining. The setting does not survive a re-association.
    pub fn set_power_management(
        &mut self,
        mode: PowerManagement,
        timer: &Timer,
    ) -> Result<(), Error> {
        self.ioctl_set_u32(WLC_SET_PM, mode as u32, timer)
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
        // Built in the transmit buffer, not the receive one: a coalesced
        // frame may still be being handed out of that, and every packet
        // left in it would go with this write.
        self.tx[..padded].fill(0);
        // SDPCM header: length + complement, sequence, data channel, and
        // the data offset (start of the BDC header).
        self.tx[0..2].copy_from_slice(&(total as u16).to_le_bytes());
        self.tx[2..4].copy_from_slice(&(!(total as u16)).to_le_bytes());
        self.tx[4] = self.tx_seq;
        self.tx[5] = CHANNEL_DATA;
        self.tx[7] = SDPCM_HEADER_LEN as u8;
        // BDC header at offset 12: version-2 flags, zero priority/flags2,
        // and a zero data-offset (the Ethernet frame follows immediately).
        self.tx[SDPCM_HEADER_LEN] = BDC_FLAG_VERSION;
        // Ethernet frame after the SDPCM + BDC headers.
        let data_at = SDPCM_HEADER_LEN + BDC_HEADER_LEN;
        self.tx[data_at..data_at + frame.len()].copy_from_slice(frame);

        self.sdio.f2_write(&self.tx[..padded], timer)?;
        self.tx_seq = self.tx_seq.wrapping_add(1);
        Ok(())
    }

    /// Receives one Ethernet frame from the network data channel into
    /// `out`, returning its length, or `None` if nothing is waiting (or
    /// the next frame is control/event traffic, which this drops). Strips
    /// the SDPCM and BDC headers, leaving a complete Ethernet frame.
    pub fn recv_ethernet(&mut self, out: &mut [u8], timer: &Timer) -> Result<Option<usize>, Error> {
        // A superframe already read is emptied before the bus is touched
        // again: it holds several frames, and this hands back one per
        // call. See [`Self::next_subframe`].
        if let Some(len) = self.next_subframe(out) {
            return Ok(Some(len));
        }

        let Some(frame) = self.recv_frame(timer)? else {
            return Ok(None);
        };
        if frame.channel == CHANNEL_GLOM {
            self.glom = Some(Glom {
                at: frame.data_offset,
                end: frame.frame_len,
            });
            // `None` here is a superframe that held nothing this driver
            // wanted -- events, or padding -- which is the same answer a
            // control frame gives, and for the same reason.
            return Ok(self.next_subframe(out));
        }
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

    /// Hands back the next Ethernet frame from a superframe already in
    /// [`Self::frame`], or `None` when there are no more.
    ///
    /// # Finding the subframes
    ///
    /// A superframe is several complete SDPCM frames laid end to end,
    /// each padded so the next starts on a boundary the chip likes — and
    /// the firmware does not say which boundary that is. Measured on a
    /// 43430: a 1530-byte frame occupies 1536, which is consistent with
    /// alignment to 64 bytes, to 512, and to several values in between.
    ///
    /// Rather than guess, this searches. Every SDPCM header carries its
    /// length and that length's complement, so a header can be
    /// recognized on sight: the walk steps forward four bytes at a time
    /// from the end of the previous subframe until it finds one. Padding
    /// is skipped by not matching, whatever its size, and the pair of
    /// words makes a false positive on payload bytes vanishingly
    /// unlikely — with the length bounds below, it needs 32 bits to
    /// agree by chance in a region that is a few words long.
    ///
    /// All-zero words are skipped rather than ending the walk. Zero is
    /// how the chip says "nothing more" on the bus itself, but inside a
    /// superframe it is just padding.
    fn next_subframe(&mut self, out: &mut [u8]) -> Option<usize> {
        let Glom { mut at, end } = self.glom?;

        loop {
            let Some((header, len)) = find_subframe(&self.frame[..end], at) else {
                self.glom = None;
                return None;
            };
            // Recorded before anything can return, so a subframe this
            // does not want is one the next call starts past rather than
            // one it finds again.
            at = header + len;
            self.glom = Some(Glom { at, end });

            let channel = self.frame[header + 5] & 0x0f;
            let data_offset = self.frame[header + 7] as usize;
            // The offset has to name a byte inside this subframe, with
            // room for the BDC header that follows it. A subframe that
            // fails this is not one to reach into.
            if channel != CHANNEL_DATA
                || data_offset < SDPCM_HEADER_LEN
                || data_offset + BDC_HEADER_LEN > len
            {
                continue;
            }

            let bdc = BDC_HEADER_LEN + ((self.frame[header + data_offset + 3] as usize) << 2);
            let start = header + data_offset + bdc;
            let finish = header + len;
            if start >= finish {
                continue;
            }
            let copied = (finish - start).min(out.len());
            out[..copied].copy_from_slice(&self.frame[start..start + copied]);
            return Some(copied);
        }
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

        // The request is built in the receive buffer, so a coalesced
        // frame part-way through being handed out is overwritten here
        // rather than at the read below. Same trade as `read_frame`'s:
        // control traffic is rare and the packets are already lost.
        self.glom = None;
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
        // Whatever is read next lands on top of any coalesced frame not
        // yet handed out, so the walk through it ends here. Reaching
        // this with one still pending means a caller read the bus
        // without draining [`Self::recv_ethernet`] first — `poll_event`
        // and the control path both do — and the packets left in it are
        // gone either way. Dropping them deliberately beats walking a
        // buffer that has been written over.
        self.glom = None;
        self.sdio
            .f2_read(&mut self.frame[..SDPCM_HEADER_LEN], timer)?;

        let len = u16::from_le_bytes([self.frame[0], self.frame[1]]) as usize;
        if len == 0 {
            return Ok(None);
        }
        let len_check = u16::from_le_bytes([self.frame[2], self.frame[3]]);
        if len_check != !(len as u16) || !(SDPCM_HEADER_LEN..=MAX_FRAME).contains(&len) {
            // The header is gone from the FIFO and its body is not, so
            // the stream is now misaligned and every read after this one
            // would fail the same way. Put it back in step before
            // reporting, which is what makes this survivable: a caller
            // that logs the error and carries on gets a working receive
            // path rather than an endless flood of the same failure.
            //
            // Best effort by construction. If the resynchronization
            // itself fails there is nothing better to return than the
            // malformation that prompted it, which is also the more
            // useful of the two to read.
            let channel = self.frame[5] & 0x0f;
            let _ = self.resync_rx(timer);
            return Err(Error::BadFrame {
                len: len as u16,
                len_check,
                channel,
            });
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

    /// Abandons the receive frame in progress and waits for the chip to
    /// flush it, putting the function-2 stream back in step.
    ///
    /// This is the way out of a desynchronized receive stream, and there
    /// is no other: the FIFO is a byte stream with no frame boundary the
    /// host can search for, so once a read has stopped part-way through
    /// a frame, every subsequent read is offset by whatever is left of
    /// it. Writing "read frame terminate" tells the chip to drop the
    /// remainder, and the byte-count registers report when it has.
    ///
    /// Called automatically when a malformed header is read — see
    /// [`Error::BadFrame`] — so a caller's receive loop does not
    /// normally need it. It is public for one that drives the FIFO
    /// itself, and because a driver that finds its own reason to
    /// distrust the stream has nowhere else to turn.
    ///
    /// Modelled on `brcmfmac`'s `rxfail` path, which does the same two
    /// steps in the same order.
    pub fn resync_rx(&mut self, timer: &Timer) -> Result<(), Error> {
        // Cleared first, so that even a failure below leaves the next
        // receive waiting for a fresh frame-ready interrupt rather than
        // reading on from where it was.
        self.rx_draining = false;
        self.sdio
            .cmd52_write(FN1, SBSDIO_FUNC1_FRAMECTRL, SFC_RF_TERM, timer)?;

        for _ in 0..RESYNC_POLLS {
            let high = self.sdio.cmd52_read(FN1, SBSDIO_FUNC1_RFRAMEBCHI, timer)?;
            let low = self.sdio.cmd52_read(FN1, SBSDIO_FUNC1_RFRAMEBCLO, timer)?;
            if high == 0 && low == 0 {
                return Ok(());
            }
        }
        // The count never reached zero. Reported rather than waited on
        // forever: something is wrong with the chip's own receive path,
        // and the caller's next read failing says so more usefully than
        // this spinning does.
        Err(Error::NoResponse)
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

/// The firmware's MAC-layer counters, as [`Wifi::counters`] reads them.
///
/// A direct mirror of the leading fields of the chip's own counters
/// structure, names and all. They are cumulative since the firmware
/// started and several of them only ever move under load, so a single
/// reading says little — take two and subtract.
///
/// Fields the structure carries but this does not are the ones after
/// `rxuflo`, whose position moves between layout versions.
#[derive(Clone, Copy, Debug)]
pub struct Counters {
    /// Layout version the firmware stamped on the structure.
    pub version: u16,
    /// Length the firmware gave for the whole structure, which may be
    /// more than was read.
    pub length: u16,

    /// Data frames transmitted.
    pub txframe: u32,
    /// Data bytes transmitted.
    pub txbyte: u32,
    /// MAC-layer retransmissions — a frame the radio sent again because
    /// the first attempt was not acknowledged. Against
    /// [`Self::txframe`], the share of transmit effort spent repeating
    /// itself, and the clearest single measure of a marginal link.
    pub txretrans: u32,
    /// Transmit errors, the firmware's own sum of the failures below.
    pub txerror: u32,
    /// Management frames transmitted.
    pub txctl: u32,
    /// Frames transmitted with a short preamble.
    pub txprshort: u32,
    /// Frames whose transmit status came back an error.
    pub txserr: u32,
    /// Transmits abandoned for want of a buffer.
    pub txnobuf: u32,
    /// Transmits discarded because the chip was not associated.
    pub txnoassoc: u32,
    /// Runt frames transmitted.
    pub txrunt: u32,
    /// Transmit header cache hits — the firmware's fast path.
    pub txchit: u32,
    /// Transmit header cache misses.
    pub txcmiss: u32,
    /// Transmit FIFO underflows: the radio started a frame and ran out of
    /// data to send.
    pub txuflo: u32,
    /// Transmit errors the PHY reported.
    pub txphyerr: u32,
    /// Transmits deferred because the channel was busy.
    pub txphycrs: u32,

    /// Data frames received.
    pub rxframe: u32,
    /// Data bytes received.
    pub rxbyte: u32,
    /// Receive errors, the firmware's own sum of the failures below.
    pub rxerror: u32,
    /// Management frames received.
    pub rxctl: u32,
    /// Receives dropped for want of a buffer.
    pub rxnobuf: u32,
    /// Non-data frames arriving on the data channel.
    pub rxnondata: u32,
    /// Frames with a bad distribution-system field.
    pub rxbadds: u32,
    /// Malformed control or management frames.
    pub rxbadcm: u32,
    /// Fragmentation errors.
    pub rxfragerr: u32,
    /// Runt frames received.
    pub rxrunt: u32,
    /// Oversized frames received.
    pub rxgiant: u32,
    /// Frames for a station the firmware has no control block for.
    pub rxnoscb: u32,
    /// Frames rejected as invalid.
    pub rxbadproto: u32,
    /// Frames with an invalid source address.
    pub rxbadsrcmac: u32,
    /// Frames discarded for an invalid destination address.
    pub rxbadda: u32,
    /// Frames the firmware's own filters discarded.
    pub rxfilter: u32,
    /// **Receive FIFO overflows.** Frames the radio took off the air and
    /// then threw away because the host had not emptied the FIFO in
    /// time.
    ///
    /// The one drop on this path that nothing above the chip can see:
    /// the frame never reaches the host, so no driver counter moves, and
    /// the sender simply retransmits. A number that climbs during a bulk
    /// transfer means the bus or the poll cadence is not keeping up with
    /// the air, whatever the host-side counters say.
    pub rxoflo: u32,
    /// Per-FIFO receive DMA descriptor underflows.
    pub rxuflo: [u32; 6],
}

impl Counters {
    /// Decodes a `counters` reply, refusing a layout this does not know.
    fn parse(reply: &[u8]) -> Result<Counters, Error> {
        if reply.len() < COUNTERS_PREFIX {
            return Err(Error::NoResponse);
        }
        let at = |offset: usize| -> u32 {
            u32::from_le_bytes([
                reply[offset],
                reply[offset + 1],
                reply[offset + 2],
                reply[offset + 3],
            ])
        };

        let version = u16::from_le_bytes([reply[0], reply[1]]);
        if !COUNTERS_VERSIONS.contains(&version) {
            return Err(Error::UnsupportedFormat { version });
        }

        let mut rxuflo = [0u32; 6];
        for (index, fifo) in rxuflo.iter_mut().enumerate() {
            *fifo = at(132 + index * 4);
        }

        Ok(Counters {
            version,
            length: u16::from_le_bytes([reply[2], reply[3]]),
            txframe: at(4),
            txbyte: at(8),
            txretrans: at(12),
            txerror: at(16),
            txctl: at(20),
            txprshort: at(24),
            txserr: at(28),
            txnobuf: at(32),
            txnoassoc: at(36),
            txrunt: at(40),
            txchit: at(44),
            txcmiss: at(48),
            txuflo: at(52),
            txphyerr: at(56),
            txphycrs: at(60),
            rxframe: at(64),
            rxbyte: at(68),
            rxerror: at(72),
            rxctl: at(76),
            rxnobuf: at(80),
            rxnondata: at(84),
            rxbadds: at(88),
            rxbadcm: at(92),
            rxfragerr: at(96),
            rxrunt: at(100),
            rxgiant: at(104),
            rxnoscb: at(108),
            rxbadproto: at(112),
            rxbadsrcmac: at(116),
            rxbadda: at(120),
            rxfilter: at(124),
            rxoflo: at(128),
            rxuflo,
        })
    }
}

/// How aggressively the radio may sleep between frames, for
/// [`Wifi::set_power_management`].
///
/// The values are the firmware's own, so this is the whole of what the
/// chip offers rather than a selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PowerManagement {
    /// The receiver stays on. Lowest latency, highest current, and the
    /// right choice for anything on a wall supply.
    None = 0,
    /// The radio sleeps once a link has been idle briefly and wakes for
    /// beacons.
    Max = 1,
    /// As [`Self::Max`], but the radio stays awake while traffic is
    /// flowing and only sleeps after a longer idle period. The firmware's
    /// power-on default.
    Fast = 2,
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
