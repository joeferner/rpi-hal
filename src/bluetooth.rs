//! HCI bring-up for the on-board BCM43438 Bluetooth controller (Pi 3).
//!
//! The controller shares the BCM43438 combo chip with the Wi-Fi radio
//! ([`crate::wifi`]), but is reached over an entirely separate link: a
//! UART attachment carrying the Bluetooth HCI, wired internally to the
//! SoC's PL011 on GPIO30-33 (see [`crate::uart::Uart::init_bluetooth`]).
//! That is the *same* PL011 the GPIO14/15 debug console uses, so driving
//! Bluetooth means moving the console to the mini UART
//! ([`crate::mini_uart`]).
//!
//! This drives the H4 (UART) HCI transport: each packet is prefixed with
//! a one-byte type ([`H4_COMMAND`](crate::bluetooth::H4_COMMAND)/
//! [`H4_ACL`](crate::bluetooth::H4_ACL)/
//! [`H4_EVENT`](crate::bluetooth::H4_EVENT)), and HCI
//! commands are answered by Command Complete / Command Status events.
//! Flow control (RTS/CTS) is handled in hardware by the PL011, so the
//! simple H4 framing is enough — the reliability layer H5 adds is only
//! needed on a link without it.
//!
//! Out of reset the controller runs a minimal ROM HCI at 115200 baud and
//! does nothing useful until Broadcom's patch/config firmware — a `.hcd`
//! "patchram" blob — is downloaded into its RAM over HCI and launched
//! ([`Bluetooth::load_firmware`](crate::bluetooth::Bluetooth::load_firmware)).
//! This mirrors the Wi-Fi side's
//! [`crate::sdio::Sdio::load_firmware`]: the controller is inert until its
//! blob is running. After that the usual informational commands answer
//! with real data
//! ([`Bluetooth::read_local_version`](crate::bluetooth::Bluetooth::read_local_version)
//! / [`Bluetooth::read_bd_addr`](crate::bluetooth::Bluetooth::read_bd_addr)) —
//! the proof the controller is alive.
//!
//! The patchram sequence follows Linux's `hci_bcm`/`btbcm`
//! (`drivers/bluetooth/`) and Broadcom's `brcm_patchram_plus`: reset,
//! "download minidriver" vendor command, replay every HCI record in the
//! `.hcd` (a run of Write-RAM chunks terminated by a Launch-RAM), then
//! reset again to resync once the patched firmware restarts. Pi 3 only.
//!
//! On top of the controller bring-up, this module carries the LE
//! connection transport in both roles. As a *peripheral*, connectable
//! advertising yields a connection
//! ([`Bluetooth::poll`](crate::bluetooth::Bluetooth::poll) surfacing
//! [`Event::Connected`](crate::bluetooth::Event::Connected) with
//! [`Role::Peripheral`](crate::bluetooth::Role::Peripheral)); as a *central*,
//! [`Bluetooth::connect`](crate::bluetooth::Bluetooth::connect) initiates a
//! connection to an advertiser by address, which the controller reports the
//! same way (with [`Role::Central`](crate::bluetooth::Role::Central)). Either
//! way ACL data then flows both ways
//! ([`Bluetooth::send_acl`](crate::bluetooth::Bluetooth::send_acl) /
//! [`Event::Acl`](crate::bluetooth::Event::Acl)) with credit-based
//! host-to-controller flow control
//! ([`Bluetooth::le_read_buffer_size`](crate::bluetooth::Bluetooth::le_read_buffer_size)),
//! and links can be torn down
//! ([`Bluetooth::disconnect`](crate::bluetooth::Bluetooth::disconnect) /
//! [`Event::Disconnected`](crate::bluetooth::Event::Disconnected)). The
//! protocol layers *above* ACL — L2CAP, and above it SDP/RFCOMM for Classic
//! or ATT/GATT/SMP for LE — live in the
//! submodules: [`l2cap`](crate::bluetooth::l2cap) framing,
//! [`gatt`](crate::bluetooth::gatt) (a GATT server) and
//! [`gatt_client`](crate::bluetooth::gatt_client) (a GATT client, for the
//! central role), and [`smp`](crate::bluetooth::smp) pairing.
//! [`Event::Acl`](crate::bluetooth::Event::Acl) hands the raw L2CAP bytes to
//! whichever of those owns the channel.

use crate::mailbox::{Mailbox, EXPANDER_BT_ON};
use crate::timer::Timer;
use crate::uart::Uart;

pub mod bredr;
pub mod gatt;
pub mod gatt_client;
pub mod hid_host;
pub mod l2cap;
pub mod sdp;
pub mod smp;

/// H4 packet-type prefix for an outgoing HCI command.
pub const H4_COMMAND: u8 = 0x01;
/// H4 packet-type prefix for an ACL data packet (host↔controller data).
pub const H4_ACL: u8 = 0x02;
/// H4 packet-type prefix for an incoming HCI event.
pub const H4_EVENT: u8 = 0x04;

/// HCI event code: Command Complete — the controller finished a command
/// and (usually) returned parameters, the first of which is a status.
const EVT_COMMAND_COMPLETE: u8 = 0x0e;
/// HCI event code: Command Status — the controller accepted a command
/// that completes asynchronously; carries only a status, no parameters.
const EVT_COMMAND_STATUS: u8 = 0x0f;
/// HCI event code `Disconnection Complete` (`0x05`): a connection ended,
/// carrying the connection handle and the reason it dropped.
const EVT_DISCONNECTION_COMPLETE: u8 = 0x05;
/// HCI event code `Number Of Completed Packets` (`0x13`): the controller
/// reports how many previously-queued ACL packets it has finished sending,
/// per connection handle — the signal that replenishes TX credits.
const EVT_NUM_COMPLETED_PACKETS: u8 = 0x13;
/// HCI event code `Encryption Change` (`0x08`): link-layer encryption was
/// enabled or disabled on a connection (the result of pairing + key setup).
const EVT_ENCRYPTION_CHANGE: u8 = 0x08;

// --- Bluetooth Classic (BR/EDR) inquiry events ---
/// HCI event code `Inquiry Complete` (`0x01`): the inquiry (Classic device
/// discovery) finished its scan window.
const EVT_INQUIRY_COMPLETE: u8 = 0x01;
/// HCI event code `Inquiry Result` (`0x02`): one or more Classic devices
/// answered the inquiry — the standard form, without RSSI or extended data.
const EVT_INQUIRY_RESULT: u8 = 0x02;
/// HCI event code `Inquiry Result with RSSI` (`0x22`): like `Inquiry Result`
/// but each response carries a signal strength. Sent when the inquiry mode is
/// set to RSSI (`0x01`).
const EVT_INQUIRY_RESULT_WITH_RSSI: u8 = 0x22;
/// HCI event code `Extended Inquiry Result` (`0x2f`): one response carrying
/// RSSI plus a 240-byte Extended Inquiry Response (which often includes the
/// device name). Sent when the inquiry mode is set to extended (`0x02`).
const EVT_EXTENDED_INQUIRY_RESULT: u8 = 0x2f;

// --- Bluetooth Classic (BR/EDR) connection + SSP pairing events ---
/// HCI event code `Connection Complete` (`0x03`): a Classic ACL (or SCO) link
/// finished being established, carrying the status, connection handle, and the
/// peer's address.
const EVT_CONNECTION_COMPLETE: u8 = 0x03;
/// HCI event code `Authentication Complete` (`0x06`): the authentication (SSP)
/// procedure on a connection finished, with a status.
const EVT_AUTHENTICATION_COMPLETE: u8 = 0x06;
/// HCI event code `Link Key Request` (`0x17`): the controller asks the host
/// for a stored link key for a peer (a reconnect), answered with the key or a
/// negative reply that triggers fresh pairing.
const EVT_LINK_KEY_REQUEST: u8 = 0x17;
/// HCI event code `Link Key Notification` (`0x18`): pairing produced a link
/// key; the host stores it (with the peer address) to reconnect without
/// re-pairing.
const EVT_LINK_KEY_NOTIFICATION: u8 = 0x18;
/// HCI event code `IO Capability Request` (`0x31`): the controller needs the
/// host's IO capability to choose the SSP association model, answered with
/// `IO_Capability_Request_Reply`. (The peer's own capability arrives as the
/// unused `IO Capability Response`, `0x32`.)
const EVT_IO_CAPABILITY_REQUEST: u8 = 0x31;
/// HCI event code `User Confirmation Request` (`0x33`): the SSP numeric value
/// to confirm — auto-accepted for Just Works with `User_Confirmation_Request_Reply`.
const EVT_USER_CONFIRMATION_REQUEST: u8 = 0x33;
/// HCI event code `Simple Pairing Complete` (`0x36`): the SSP exchange
/// finished, with a status.
const EVT_SIMPLE_PAIRING_COMPLETE: u8 = 0x36;

/// HCI command opcode `HCI_Reset` (OGF 0x03, OCF 0x003): resets the
/// controller's link layer and baseband to a known state.
const OP_RESET: u16 = 0x0c03;
/// HCI command opcode `Read_Local_Version_Information` (OGF 0x04, OCF
/// 0x001): HCI/LMP version, manufacturer, and subversion.
const OP_READ_LOCAL_VERSION: u16 = 0x1001;
/// HCI command opcode `Read_BD_ADDR` (OGF 0x04, OCF 0x009): the
/// controller's public Bluetooth device address.
const OP_READ_BD_ADDR: u16 = 0x1009;
/// HCI command opcode `Read_Buffer_Size` (OGF 0x04, OCF 0x005): the
/// controller's (BR/EDR) ACL data packet length and how many such packets it
/// can buffer — the Classic counterpart to `LE_Read_Buffer_Size`, and the
/// basis for host-to-controller ACL flow control on a Classic link.
const OP_READ_BUFFER_SIZE: u16 = 0x1005;
/// HCI command opcode `Disconnect` (OGF 0x01, OCF 0x006): tears down an
/// existing connection by handle. Answered by a Command Status, with the
/// actual teardown reported later by a `Disconnection Complete` event.
const OP_DISCONNECT: u16 = 0x0406;
/// HCI command opcode `LE_Read_Buffer_Size` (OGF 0x08, OCF 0x002): the
/// controller's LE ACL data packet length and how many such packets it can
/// buffer — the basis for host-to-controller ACL flow control.
const OP_LE_READ_BUFFER_SIZE: u16 = 0x2002;
/// HCI command opcode `LE_Encrypt` (OGF 0x08, OCF 0x017): AES-128-encrypts a
/// 128-bit block with a 128-bit key in the controller. Used as the AES
/// primitive for the SMP pairing crypto ([`crate::bluetooth::smp`]) so the
/// host needs no software AES.
const OP_LE_ENCRYPT: u16 = 0x2017;
/// HCI command opcode `LE_Rand` (OGF 0x08, OCF 0x018): returns 8 bytes of
/// controller-generated randomness — the source for SMP pairing randoms.
const OP_LE_RAND: u16 = 0x2018;
/// HCI command opcode `LE_Start_Encryption` (OGF 0x08, OCF 0x019): as the
/// central, tells the controller to start (or restart) link encryption with a
/// given Long Term Key, identified by an EDIV/Rand. The central-role
/// counterpart to answering an `LE Long Term Key Request`: for an in-progress
/// LE Legacy pairing the key is the STK with EDIV/Rand both zero; on a bonded
/// reconnect it is the stored LTK with its distributed EDIV/Rand. Answered by
/// a Command Status, with the result arriving later as an `Encryption Change`
/// event.
const OP_LE_START_ENCRYPTION: u16 = 0x2019;
/// HCI command opcode `LE_Long_Term_Key_Request_Reply` (OGF 0x08, OCF
/// 0x01a): supplies the LTK/STK for a pending `LE Long Term Key Request`, so
/// the controller can turn on link encryption.
const OP_LE_LTK_REQUEST_REPLY: u16 = 0x201a;
/// HCI command opcode `LE_Long_Term_Key_Request_Negative_Reply` (OGF 0x08,
/// OCF 0x01b): rejects an `LE Long Term Key Request` when no matching key is
/// known (e.g. an unrecognized bonded EDIV/Rand), so the encryption attempt
/// fails cleanly instead of hanging.
const OP_LE_LTK_REQUEST_NEG_REPLY: u16 = 0x201b;

/// Broadcom vendor command `Download_Minidriver` (OGF 0x3F, OCF 0x02e):
/// puts the ROM into the state that accepts the Write-RAM patch stream.
const OP_BCM_DOWNLOAD_MINIDRIVER: u16 = 0xfc2e;
/// HCI command opcode `LE_Set_Advertising_Parameters` (OGF 0x08, OCF
/// 0x006): interval, advertising type, address types, channel map, and
/// filter policy for subsequent advertising.
const OP_LE_SET_ADV_PARAMS: u16 = 0x2006;
/// HCI command opcode `LE_Set_Advertising_Data` (OGF 0x08, OCF 0x008): the
/// AD structures broadcast in each advertising PDU (a length byte then a
/// fixed 31-byte payload).
const OP_LE_SET_ADV_DATA: u16 = 0x2008;
/// HCI command opcode `LE_Set_Advertising_Enable` (OGF 0x08, OCF 0x00a):
/// starts (`0x01`) or stops (`0x00`) advertising with the parameters and
/// data already set.
const OP_LE_SET_ADV_ENABLE: u16 = 0x200a;
/// HCI command opcode `LE_Set_Scan_Parameters` (OGF 0x08, OCF 0x00b): scan
/// type (passive/active), interval/window, and address/filter policy.
const OP_LE_SET_SCAN_PARAMS: u16 = 0x200b;
/// HCI command opcode `LE_Set_Scan_Enable` (OGF 0x08, OCF 0x00c): starts
/// (`0x01`) or stops (`0x00`) scanning, with a duplicate-filter flag.
const OP_LE_SET_SCAN_ENABLE: u16 = 0x200c;
/// HCI command opcode `LE_Create_Connection` (OGF 0x08, OCF 0x00d): as a
/// central, initiate a connection to an advertising peripheral by address.
/// Answered by a Command Status; the established link (or a failure) arrives
/// later as an `LE Connection Complete` event surfaced by [`Bluetooth::poll`].
const OP_LE_CREATE_CONNECTION: u16 = 0x200d;
/// HCI command opcode `Inquiry` (OGF 0x01, OCF 0x001): Bluetooth Classic
/// device discovery — the BR/EDR analog of an LE scan. Broadcasts on the
/// inquiry-access-code channels and gathers responses as `Inquiry Result`
/// events, ending with `Inquiry Complete`. Answered by a Command Status.
const OP_INQUIRY: u16 = 0x0401;
/// HCI command opcode `Inquiry_Cancel` (OGF 0x01, OCF 0x002): stops an inquiry
/// still in progress.
const OP_INQUIRY_CANCEL: u16 = 0x0402;
/// HCI command opcode `Write_Inquiry_Mode` (OGF 0x03, OCF 0x045): selects how
/// much detail inquiry responses carry — `0x00` standard, `0x01` with RSSI,
/// `0x02` extended (RSSI + a 240-byte Extended Inquiry Response that often
/// holds the device name).
const OP_WRITE_INQUIRY_MODE: u16 = 0x0c45;
/// HCI command opcode `Create_Connection` (OGF 0x01, OCF 0x005): as the
/// Classic master, page a device by address to establish an ACL link — the
/// BR/EDR analog of `LE_Create_Connection`. Answered by a Command Status, with
/// the link (or a page failure) reported later as a `Connection Complete`.
const OP_CREATE_CONNECTION: u16 = 0x0405;
/// HCI command opcode `Authentication_Requested` (OGF 0x01, OCF 0x011): starts
/// authentication (which triggers SSP pairing if no link key exists) on a
/// connection. Answered by a Command Status; the SSP handshake then plays out
/// as events, ending with `Authentication Complete`.
const OP_AUTHENTICATION_REQUESTED: u16 = 0x0411;
/// HCI command opcode `Set_Connection_Encryption` (OGF 0x01, OCF 0x013): turns
/// link encryption on (or off) for a connection. Answered by a Command Status,
/// with the result in an `Encryption Change` event.
const OP_SET_CONNECTION_ENCRYPTION: u16 = 0x0413;
/// HCI command opcode `Link_Key_Request_Negative_Reply` (OGF 0x01, OCF 0x00c):
/// tells the controller no link key is stored for the peer, so it proceeds to
/// fresh SSP pairing.
const OP_LINK_KEY_REQUEST_NEGATIVE_REPLY: u16 = 0x040c;
/// HCI command opcode `IO_Capability_Request_Reply` (OGF 0x01, OCF 0x02b):
/// gives the controller the host's IO capability, OOB presence, and
/// authentication requirements for the SSP association-model choice.
const OP_IO_CAPABILITY_REQUEST_REPLY: u16 = 0x042b;
/// HCI command opcode `User_Confirmation_Request_Reply` (OGF 0x01, OCF 0x02c):
/// confirms the SSP numeric value — the auto-accept that completes Just Works.
const OP_USER_CONFIRMATION_REQUEST_REPLY: u16 = 0x042c;
/// HCI command opcode `Set_Event_Mask` (OGF 0x03, OCF 0x001): selects which
/// HCI events the controller emits. The SSP events are masked off by default,
/// so Classic pairing needs this to unmask them.
const OP_SET_EVENT_MASK: u16 = 0x0c01;
/// HCI command opcode `Write_Simple_Pairing_Mode` (OGF 0x03, OCF 0x056):
/// enables Secure Simple Pairing on the controller (required before SSP
/// pairing can run).
const OP_WRITE_SIMPLE_PAIRING_MODE: u16 = 0x0c56;
/// HCI command opcode `LE_Create_Connection_Cancel` (OGF 0x08, OCF 0x00e):
/// aborts a `LE_Create_Connection` still searching for the peer, so a
/// connect attempt that never finds its target can be abandoned rather than
/// left running. Answered by a Command Complete; the cancelled attempt then
/// yields an `LE Connection Complete` with a non-zero status (which
/// [`Bluetooth::poll`] discards).
const OP_LE_CREATE_CONNECTION_CANCEL: u16 = 0x200e;

/// HCI event code `LE Meta` (`0x3e`): wraps all the Low Energy events,
/// distinguished by a subevent code in the first parameter byte.
const EVT_LE_META: u8 = 0x3e;
/// `LE Meta` subevent `LE Connection Complete` (`0x01`): a link was
/// established, carrying the connection handle, local role, and the peer's
/// address.
const LE_SUBEVENT_CONNECTION_COMPLETE: u8 = 0x01;
/// `LE Meta` subevent `LE Long Term Key Request` (`0x05`): the controller
/// (as peripheral) needs the LTK to complete an encryption start the central
/// initiated — answered with `LE_Long_Term_Key_Request_Reply`.
const LE_SUBEVENT_LTK_REQUEST: u8 = 0x05;
/// `LE Meta` subevent `LE Advertising Report` (`0x02`): one or more
/// advertising PDUs the controller received while scanning.
const LE_SUBEVENT_ADV_REPORT: u8 = 0x02;

/// `Connection_Handle` field width: the handle is the low 12 bits of the
/// 16-bit handle/flags field; the top 4 bits are packet-boundary and
/// broadcast flags in an ACL header, and reserved elsewhere.
const CONN_HANDLE_MASK: u16 = 0x0fff;

/// ACL header `PB` (packet-boundary) flag for a first, non-flushable L2CAP
/// fragment — the start of an L2CAP PDU. Shifted into bits 12-13 of the
/// ACL handle/flags field.
const ACL_PB_FIRST_NON_FLUSH: u16 = 0x00;
/// ACL header `PB` flag for a continuing fragment of an L2CAP PDU.
const ACL_PB_CONTINUATION: u16 = 0x01;
/// Largest ACL data payload this driver buffers for a single fragment.
/// Comfortably covers an LE controller's ACL packet length (27 bytes
/// classic, up to 251 with LE Data Length Extension).
const MAX_ACL_DATA: usize = 255;

/// Broadcom vendor command `Update_Baudrate` (OGF 0x3F, OCF 0x018):
/// switches the controller's HCI UART to a new baud rate. Its six-byte
/// parameter is a 2-byte encoded-rate field (zero — select the literal
/// rate) followed by the little-endian 32-bit baud. The controller sends
/// the Command Complete at the *old* rate and only then switches, so the
/// host reprograms its own UART afterward — see [`Bluetooth::set_baud`].
const OP_BCM_UPDATE_BAUDRATE: u16 = 0xfc18;

/// Milliseconds to let `BT_ON` settle after asserting it, before the
/// first HCI command — the analog of the Wi-Fi side's `WL_ON` settle.
const BT_ON_SETTLE_MS: u32 = 150;
/// Milliseconds to let the controller digest `Download_Minidriver`
/// before streaming the patch records, matching `brcm_patchram_plus`.
const MINIDRIVER_SETTLE_MS: u32 = 50;
/// Milliseconds to let the patched firmware restart after the `.hcd`'s
/// terminating Launch-RAM record, before resyncing with a reset.
const LAUNCH_SETTLE_MS: u32 = 250;
/// Milliseconds to let the controller settle at a freshly-set baud rate
/// after an `Update_Baudrate`, before the next command hits the wire.
const BAUD_SETTLE_MS: u32 = 10;

/// Default per-command timeout waiting for the matching Command
/// Complete / Command Status event, in milliseconds. Generous: the
/// controller answers most commands in well under a millisecond, but the
/// initial reset out of the ROM can lag.
const COMMAND_TIMEOUT_MS: u32 = 1000;

/// Largest HCI event this driver buffers: a 2-byte header is stripped by
/// the reader, leaving the 255-byte maximum parameter length.
const MAX_EVENT_PARAMS: usize = 255;

/// Fixed advertising interval used by [`Bluetooth::start_advertising`], in
/// units of 0.625 ms: `0x00a0` = 160 = 100 ms. Both the min and max
/// interval are set to this — a steady, moderately fast rate that a phone
/// scanner picks up promptly without flooding the air.
const ADV_INTERVAL: u16 = 0x00a0;
/// Advertising channel map passed to `LE_Set_Advertising_Parameters`:
/// `0x07` = all three primary advertising channels (37, 38, 39).
const ADV_CHANNEL_ALL: u8 = 0x07;
/// AD structure type `Flags` (Core Spec supplement).
const AD_TYPE_FLAGS: u8 = 0x01;
/// AD structure type `Shortened Local Name` — used when the name was
/// truncated to fit the 31-byte advertising payload.
const AD_TYPE_NAME_SHORT: u8 = 0x08;
/// AD structure type `Complete Local Name`.
const AD_TYPE_NAME_COMPLETE: u8 = 0x09;
/// `Flags` AD value: LE General Discoverable Mode + BR/EDR Not Supported —
/// the standard flags for an LE-only peripheral.
const AD_FLAGS_LE_GENERAL: u8 = 0x06;
/// Maximum advertising payload, in bytes — the fixed `LE_Set_Advertising_Data`
/// field width.
const ADV_DATA_MAX: usize = 31;

/// `LE_Scan_Type` value selecting active scanning: the scanner transmits
/// scan requests, so it also captures scan-response data (often a
/// connectable device's name) rather than only the advertising PDU.
const SCAN_TYPE_ACTIVE: u8 = 0x01;
/// Scan interval and window used by [`Bluetooth::start_scan`], both in
/// units of 0.625 ms: `0x0010` = 16 = 10 ms. Setting the window equal to
/// the interval scans continuously.
const SCAN_INTERVAL: u16 = 0x0010;
/// See `SCAN_INTERVAL` — window equal to interval = continuous scanning.
const SCAN_WINDOW: u16 = 0x0010;

/// General Inquiry Access Code (`0x9E8B33`) in HCI wire order (LSB first) —
/// the standard LAP an [`Bluetooth::start_inquiry`] broadcasts on to discover
/// any discoverable Classic device.
const GIAC_LAP: [u8; 3] = [0x33, 0x8b, 0x9e];
/// Inquiry duration passed to `Inquiry`, in units of 1.28 s: `0x08` = 8 ≈
/// 10.24 s of scanning before the controller sends `Inquiry Complete` (which
/// [`Bluetooth::next_inquiry_result`] then transparently restarts).
const INQUIRY_LENGTH: u8 = 0x08;
/// `Num_Responses` for `Inquiry`: `0x00` = unlimited (report every device
/// heard, don't stop after N).
const INQUIRY_UNLIMITED_RESPONSES: u8 = 0x00;
/// `Write_Inquiry_Mode` value `0x02` = extended: responses carry RSSI and a
/// 240-byte Extended Inquiry Response, from which a device name can be read.
const INQUIRY_MODE_EXTENDED: u8 = 0x02;
/// Largest device name [`InquiryResult`] keeps from an Extended Inquiry
/// Response's name AD structure.
const EIR_NAME_MAX: usize = 32;
/// Class-of-Device major device class `Peripheral` (`0x05`) — keyboards, mice,
/// and game controllers. Read from bits 8-12 of the 24-bit Class of Device.
const COD_MAJOR_PERIPHERAL: u8 = 0x05;

/// Packet types offered to `Create_Connection` (`0xcc18`): the DM1/DH1/DM3/
/// DH3/DM5/DH5 ACL packet types, the standard "all basic-rate" set a host
/// offers so the controller can pick the best.
const CLASSIC_PACKET_TYPES: u16 = 0xcc18;
/// `Page_Scan_Repetition_Mode` R1 passed to `Create_Connection` — the common
/// default; the controller retries paging across the peer's scan window.
const PAGE_SCAN_REPETITION_R1: u8 = 0x01;
/// SSP IO capability `NoInputNoOutput` (`0x03`) — selects the Just Works
/// association model (no passkey, no numeric comparison), the same choice the
/// LE side makes for [`smp`].
const IO_CAP_NO_INPUT_NO_OUTPUT: u8 = 0x03;
/// SSP `OOB_Data_Present` value `0x00` — no out-of-band pairing data.
const OOB_DATA_NONE: u8 = 0x00;
/// SSP authentication requirements `MITM Protection Not Required – General
/// Bonding` (`0x04`): no man-in-the-middle protection (Just Works) but persist
/// keys so a reconnect skips re-pairing.
const AUTH_REQ_GENERAL_BONDING: u8 = 0x04;
/// Encryption-enable value for `Set_Connection_Encryption`.
const CLASSIC_ENCRYPTION_ON: u8 = 0x01;
/// HCI event mask enabling every event through the Secure Simple Pairing set —
/// all ones, so the SSP events (masked off by default) are delivered.
const EVENT_MASK_ALL: u64 = 0xffff_ffff_ffff_ffff;
/// Time budget for a `Create_Connection` to page the peer and report
/// `Connection Complete`, in milliseconds — generous, as paging waits on the
/// peer's page-scan window.
const CLASSIC_CONNECT_TIMEOUT_MS: u32 = 15_000;
/// Time budget for the SSP pairing handshake (`Authentication_Requested`
/// through `Authentication Complete`), in milliseconds.
const CLASSIC_PAIR_TIMEOUT_MS: u32 = 15_000;
/// Time budget for `Set_Connection_Encryption` to report `Encryption Change`,
/// in milliseconds.
const CLASSIC_ENCRYPT_TIMEOUT_MS: u32 = 5_000;

/// Scan interval used while initiating a connection with
/// [`Bluetooth::connect`], in units of 0.625 ms: `0x0060` = 96 = 60 ms — how
/// often the initiator listens for the target's advertising.
const CONN_SCAN_INTERVAL: u16 = 0x0060;
/// Scan window used while initiating a connection, in units of 0.625 ms:
/// `0x0030` = 48 = 30 ms — how long it listens within each interval.
const CONN_SCAN_WINDOW: u16 = 0x0030;
/// Minimum connection interval requested by [`Bluetooth::connect`], in units
/// of 1.25 ms: `0x0018` = 24 = 30 ms. The controller negotiates a value in
/// `[min, max]` with the peripheral.
const CONN_INTERVAL_MIN: u16 = 0x0018;
/// Maximum connection interval requested by [`Bluetooth::connect`], in units
/// of 1.25 ms: `0x0028` = 40 = 50 ms.
const CONN_INTERVAL_MAX: u16 = 0x0028;
/// Peripheral latency requested by [`Bluetooth::connect`]: `0` — the
/// peripheral must respond every connection event (lowest latency, simplest
/// to reason about for an interactive input device).
const CONN_LATENCY: u16 = 0x0000;
/// Supervision timeout requested by [`Bluetooth::connect`], in units of
/// 10 ms: `0x00c8` = 200 = 2000 ms — how long without a packet before the
/// link is declared lost. Must comfortably exceed
/// `(1 + latency) * interval_max`.
const CONN_SUPERVISION_TIMEOUT: u16 = 0x00c8;

/// The kind of advertising [`Bluetooth::start_advertising`] performs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Advertising {
    /// Connectable undirected advertising (`ADV_IND`): a central may
    /// connect. Note that the controller stops advertising once a central
    /// connects, and — until a GATT server is implemented — a connected
    /// central finds no services and will disconnect. Useful mainly to
    /// exercise the connection path once that exists.
    Connectable,
    /// Non-connectable undirected advertising (`ADV_NONCONN_IND`): a pure
    /// broadcaster that stays continuously visible to scanners and accepts
    /// no connections. The honest choice while there's no connection
    /// handling above the controller.
    NonConnectable,
}

impl Advertising {
    /// The `Advertising_Type` byte this kind maps to in
    /// `LE_Set_Advertising_Parameters`.
    fn adv_type(self) -> u8 {
        match self {
            Advertising::Connectable => 0x00,
            Advertising::NonConnectable => 0x03,
        }
    }
}

/// One advertising PDU received while scanning, from a
/// [`Bluetooth::next_advertising_report`] — a single device's report out of
/// an `LE Advertising Report` event. Owns a copy of the advertising data,
/// so it outlives the read buffer.
#[derive(Clone, Copy, Debug)]
pub struct AdvReport {
    /// The advertising event type: `0x00` ADV_IND (connectable), `0x02`
    /// ADV_SCAN_IND, `0x03` ADV_NONCONN_IND (broadcaster), `0x04` SCAN_RSP
    /// (a scan response to an active scan), etc.
    pub event_type: u8,
    /// The advertiser's address type: `0x00` public, `0x01` random.
    pub address_type: u8,
    /// The advertiser's device address, little-endian (LSB first) as it
    /// arrives on the wire — print it MSB-first for the usual
    /// `AA:BB:CC:DD:EE:FF` form.
    pub address: [u8; 6],
    /// Received signal strength of this PDU, in dBm (negative; closer to
    /// zero is stronger).
    pub rssi: i8,
    /// Advertising data bytes, copied out of the event (≤ 31 bytes).
    data_buf: [u8; ADV_DATA_MAX],
    /// Significant length of `data_buf`.
    data_len: u8,
}

impl AdvReport {
    /// The raw advertising data (a sequence of `length/type/value` AD
    /// structures) carried by this report.
    pub fn data(&self) -> &[u8] {
        &self.data_buf[..self.data_len as usize]
    }

    /// The advertiser's local name from the AD structures — the complete
    /// name if present, else the shortened name — decoded as UTF-8.
    /// `None` if no name AD structure is present or it isn't valid UTF-8.
    pub fn name(&self) -> Option<&str> {
        let data = self.data();
        let bytes = find_ad_structure(data, AD_TYPE_NAME_COMPLETE)
            .or_else(|| find_ad_structure(data, AD_TYPE_NAME_SHORT))?;
        core::str::from_utf8(bytes).ok()
    }
}

/// One Bluetooth Classic (BR/EDR) device discovered by an inquiry, from
/// [`Bluetooth::next_inquiry_result`] — the Classic analog of [`AdvReport`].
/// Owns a copy of any name found in the Extended Inquiry Response, so it
/// outlives the read buffer.
#[derive(Clone, Copy, Debug)]
pub struct InquiryResult {
    /// The device's Bluetooth address, little-endian (LSB first) as it arrives
    /// on the wire — print it MSB-first for the usual `AA:BB:CC:DD:EE:FF` form.
    pub bd_addr: [u8; 6],
    /// The 24-bit Class of Device, little-endian as received — encodes the
    /// device's major/minor class and service classes (see
    /// [`Self::major_device_class`] / [`Self::is_gamepad`]).
    pub class_of_device: [u8; 3],
    /// Received signal strength in dBm, when the inquiry mode supplies it
    /// (RSSI or extended); `None` for a standard `Inquiry Result`.
    pub rssi: Option<i8>,
    /// Device name from the Extended Inquiry Response, copied out (≤
    /// [`EIR_NAME_MAX`] bytes).
    name_buf: [u8; EIR_NAME_MAX],
    /// Significant length of `name_buf`.
    name_len: u8,
}

impl InquiryResult {
    /// The device's name from its Extended Inquiry Response, if one was
    /// present and is valid UTF-8. Standard (non-extended) inquiry responses
    /// carry no name, so this is `None` there — fetch it separately with a
    /// remote-name request if needed.
    pub fn name(&self) -> Option<&str> {
        if self.name_len == 0 {
            return None;
        }
        core::str::from_utf8(&self.name_buf[..self.name_len as usize]).ok()
    }

    /// The 24-bit Class of Device as a single value.
    pub fn class_of_device_u24(&self) -> u32 {
        let c = self.class_of_device;
        u32::from(c[0]) | (u32::from(c[1]) << 8) | (u32::from(c[2]) << 16)
    }

    /// The major device class (bits 8-12 of the Class of Device) — e.g.
    /// `COD_MAJOR_PERIPHERAL` (`0x05`) for keyboards/mice/controllers.
    pub fn major_device_class(&self) -> u8 {
        ((self.class_of_device_u24() >> 8) & 0x1f) as u8
    }

    /// `true` if the Class of Device marks this as a game controller — a
    /// Peripheral whose minor class is joystick (`0x01`) or gamepad (`0x02`).
    /// The quick "is this the controller?" test during discovery.
    pub fn is_gamepad(&self) -> bool {
        // Peripheral minor device class occupies bits 2-7; its low nibble is
        // the device kind (1 = joystick, 2 = gamepad).
        let minor_kind = ((self.class_of_device_u24() >> 2) & 0x0f) as u8;
        self.major_device_class() == COD_MAJOR_PERIPHERAL
            && (minor_kind == 0x01 || minor_kind == 0x02)
    }
}

/// The local device's role on an established connection, from the
/// `LE Connection Complete` event.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    /// The local device initiated the connection (it was scanning/initiating
    /// and connected to an advertiser).
    Central,
    /// The local device was advertising and a central connected to it — the
    /// role a peripheral (e.g. a keyboard) plays.
    Peripheral,
}

impl Role {
    /// Decodes the `Role` byte from `LE Connection Complete` (`0x00`
    /// Central, `0x01` Peripheral); any other value is treated as
    /// [`Role::Peripheral`], the role this controller takes when a central
    /// connects to its advertising.
    fn from_byte(byte: u8) -> Self {
        match byte {
            0x00 => Role::Central,
            _ => Role::Peripheral,
        }
    }
}

/// An established LE connection, from a [`Bluetooth::poll`]
/// [`Event::Connected`]. The [`handle`](Self::handle) identifies it for
/// [`Bluetooth::send_acl`] and [`Bluetooth::disconnect`], and is echoed by
/// every inbound [`Event::Acl`] and [`Event::Disconnected`] for it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Connection {
    /// Controller-assigned connection handle (12 bits significant).
    pub handle: u16,
    /// The local device's role on this connection — [`Role::Peripheral`]
    /// when a central connected to our advertising.
    pub role: Role,
    /// The peer's address type: `0x00` public, `0x01` random.
    pub peer_address_type: u8,
    /// The peer's device address, little-endian (LSB first) as it arrives
    /// on the wire — print it MSB-first for the usual `AA:BB:CC:DD:EE:FF`
    /// form.
    pub peer_address: [u8; 6],
}

/// Inbound ACL-U data on a connection: the L2CAP-over-ACL payload of one
/// received ACL fragment, from a [`Bluetooth::poll`] [`Event::Acl`]. Owns a
/// copy of the fragment (like [`AdvReport`]) so it outlives the read
/// buffer. Interpreting it — L2CAP framing, reassembly of PDUs split across
/// fragments — is a higher layer's job; this is the raw fragment as the
/// controller delivered it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AclData {
    /// The handle of the connection the data arrived on (12 bits
    /// significant).
    pub handle: u16,
    /// `true` if this fragment starts a new L2CAP PDU (ACL `PB` flag),
    /// `false` if it continues the previous one.
    pub first_fragment: bool,
    /// Fragment payload bytes, copied out of the ACL packet.
    data_buf: [u8; MAX_ACL_DATA],
    /// Significant length of `data_buf`.
    data_len: u16,
}

impl AclData {
    /// The fragment's payload bytes (the L2CAP-over-ACL data).
    pub fn data(&self) -> &[u8] {
        &self.data_buf[..self.data_len as usize]
    }
}

/// An asynchronous event surfaced by [`Bluetooth::poll`] once a connection
/// is being established or is live. Command replies (Command Complete /
/// Status) and `Number Of Completed Packets` are consumed internally and
/// never surface here; advertising reports are delivered by the separate
/// [`Bluetooth::next_advertising_report`] instead.
// `Acl` is large because `AclData` owns an inline copy of its fragment
// (like `AdvReport`), which the usual fix — boxing — can't shrink here:
// this crate is `no_std` with no allocator. The owned copy is the
// deliberate design, so the size difference is expected.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// A connection was established (for a peripheral, a central connected
    /// to our advertising, which the controller then stops on its own).
    Connected(Connection),
    /// A connection ended.
    Disconnected {
        /// The handle of the connection that dropped.
        handle: u16,
        /// The HCI reason code the controller reported for the disconnect.
        reason: u8,
    },
    /// Inbound ACL-U data on a connection.
    Acl(AclData),
    /// The controller needs the Long Term Key to complete an encryption
    /// start the central initiated (an `LE Long Term Key Request`). Answer
    /// with [`Bluetooth::le_ltk_request_reply`] carrying the key: the STK for
    /// an in-progress LE Legacy pairing (`ediv`/`rand` both zero), or a
    /// bonded LTK identified by `ediv`/`rand` on a reconnect.
    LongTermKeyRequest {
        /// The handle of the connection encryption is starting on.
        handle: u16,
        /// The EDIV identifying which bonded LTK the central is using — zero
        /// for the STK of an in-progress pairing.
        ediv: u16,
        /// The random value identifying the bonded LTK — all zero for the STK
        /// of an in-progress pairing.
        rand: [u8; 8],
    },
    /// Link-layer encryption changed on a connection — the confirmation that
    /// pairing succeeded and the link is now encrypted (or was disabled).
    EncryptionChange {
        /// The handle of the connection whose encryption changed.
        handle: u16,
        /// `true` if encryption is now enabled.
        enabled: bool,
    },
}

/// Errors from the Bluetooth HCI layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// No matching event arrived within the command's time budget.
    Timeout,
    /// A command's Command Complete / Command Status came back with a
    /// non-zero HCI status code (`opcode` is the command that failed).
    CommandFailed {
        /// The opcode of the command the controller rejected.
        opcode: u16,
        /// The non-zero HCI status code the controller returned.
        status: u8,
    },
    /// A Command Complete returned fewer bytes than the issuing command's
    /// documented return parameters — the controller answered oddly.
    ShortResponse,
    /// A requested HCI baud rate the host PL011 can't represent at its
    /// reference clock (see [`crate::uart::Uart::set_baud`]). The
    /// controller may already have switched to it — the link is left in an
    /// indeterminate state and needs re-bringing-up.
    UnsupportedBaud(u32),
    /// The `.hcd` firmware blob is malformed: a record's length runs past
    /// the end of the blob, or trailing bytes don't form a whole record.
    BadFirmware,
    /// An ACL send found no free controller TX buffers (all credits from
    /// [`Bluetooth::le_read_buffer_size`] are outstanding, awaiting a
    /// `Number Of Completed Packets` event). Pump [`Bluetooth::poll`] to
    /// let credits replenish, then retry. Not seen at keyboard-scale
    /// traffic, where the controller's buffer count is never exhausted.
    NoAclBuffers,
    /// An L2CAP payload exceeded [`l2cap::MAX_PAYLOAD`] and could not be
    /// framed for sending (see [`l2cap::send`]).
    PayloadTooLarge,
    /// The SMP crypto self-test failed: the controller's `LE_Encrypt` did
    /// not reproduce the known AES test vector under either byte-order
    /// convention (see [`smp::self_test`]). Pairing crypto can't be trusted,
    /// so pairing must not proceed.
    CryptoSelfTest,
}

/// The controller's local version information, from
/// [`Bluetooth::read_local_version`] — a `Read_Local_Version_Information`
/// reply. For a genuine BCM43438 the `manufacturer` reads back as
/// Broadcom (`0x000f`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LocalVersion {
    /// HCI specification version the controller implements.
    pub hci_version: u8,
    /// Vendor-specific HCI revision.
    pub hci_revision: u16,
    /// LMP (link manager protocol) version.
    pub lmp_version: u8,
    /// Manufacturer (company) identifier — Broadcom is `0x000f`.
    pub manufacturer: u16,
    /// Vendor-specific LMP subversion.
    pub lmp_subversion: u16,
}

/// The on-board Bluetooth controller, reached over the H4 HCI transport.
///
/// Build one with [`Self::new`] from a [`Uart`] already routed to the
/// controller's pins ([`Uart::init_bluetooth`]); it asserts `BT_ON` and
/// takes ownership of the UART. Then [`Self::load_firmware`] downloads the
/// `.hcd` patchram blob, after which [`Self::read_local_version`] /
/// [`Self::read_bd_addr`] answer with real data.
pub struct Bluetooth {
    uart: Uart,
    /// Maximum ACL data payload the controller accepts per packet, from
    /// [`Self::le_read_buffer_size`]. [`Self::send_acl`] fragments larger
    /// payloads to this. Zero until queried, in which case `send_acl`
    /// sends the payload as a single fragment.
    acl_packet_len: u16,
    /// Free host-to-controller ACL TX buffers. Seeded by
    /// [`Self::le_read_buffer_size`], decremented per fragment sent, and
    /// replenished by the `Number Of Completed Packets` events [`Self::poll`]
    /// consumes. Zero here means all buffers are outstanding.
    acl_credits: u16,
}

/// One decoded HCI transport packet from [`Bluetooth::read_packet`],
/// borrowing its payload from the caller's read buffer. The transport
/// carries only events and ACL data inbound; the type byte selects which.
enum Packet<'a> {
    /// An HCI event: its event code and parameter bytes.
    Event {
        /// The HCI event code (e.g. [`EVT_COMMAND_COMPLETE`]).
        code: u8,
        /// The event's parameter bytes.
        params: &'a [u8],
    },
    /// An ACL data packet: the connection handle it arrived on, whether it
    /// starts a new L2CAP PDU, and its payload.
    Acl {
        /// The connection handle (12 bits significant).
        handle: u16,
        /// `true` if the ACL `PB` flag marks this as a first fragment.
        first_fragment: bool,
        /// The ACL payload bytes (an L2CAP fragment).
        data: &'a [u8],
    },
}

impl Bluetooth {
    /// Powers the controller up and takes ownership of its HCI UART.
    ///
    /// Asserts the `BT_ON` enable line through the VideoCore GPIO
    /// expander ([`EXPANDER_BT_ON`]) — the SoC has no direct GPIO to it —
    /// then waits for it to settle. As on the Wi-Fi side's `WL_ON`
    /// (see [`crate::sdio::Sdio::init`]), this is best-effort: the boot
    /// firmware often powers the Bluetooth core up already, and not every
    /// Pi 3 firmware answers the expander mailbox tag, so a rejection is
    /// tolerated rather than fatal — the [`Self::load_firmware`] handshake
    /// is the real proof the controller is alive.
    ///
    /// `uart` must already be configured for the controller's pins and
    /// flow control via [`Uart::init_bluetooth`].
    pub fn new(uart: Uart, mailbox: &mut Mailbox, timer: &Timer) -> Self {
        let _ = mailbox.set_expander_gpio(EXPANDER_BT_ON, true);
        timer.delay_ms(BT_ON_SETTLE_MS);
        Self {
            uart,
            acl_packet_len: 0,
            acl_credits: 0,
        }
    }

    /// Loads and launches the controller's `.hcd` patchram firmware blob.
    ///
    /// Runs the full Broadcom download sequence: reset the ROM HCI,
    /// `Download_Minidriver`, replay every HCI record in `hcd` (a run of
    /// Write-RAM chunks ending in a Launch-RAM), then reset again to
    /// resync once the patched firmware restarts. Each record carries its
    /// own opcode and length, so this driver replays them verbatim rather
    /// than knowing the Write-RAM/Launch-RAM opcodes itself.
    ///
    /// The controller stays at 115200 baud across the launch (the rate it
    /// boots at); raising the HCI baud is a separate step layered on top,
    /// not done here.
    ///
    /// Returns [`Error::BadFirmware`] if `hcd` isn't a clean sequence of
    /// whole `[opcode(2), len(1), params(len)]` records.
    pub fn load_firmware(&mut self, hcd: &[u8], timer: &Timer) -> Result<(), Error> {
        self.reset(timer)?;

        // Put the ROM into patch-download mode, then let it settle before
        // the Write-RAM stream — the ROM needs a beat here.
        self.command(OP_BCM_DOWNLOAD_MINIDRIVER, &[], &mut [], timer)?;
        timer.delay_ms(MINIDRIVER_SETTLE_MS);

        // Replay every record in the blob. Records are pre-framed HCI
        // commands — `[opcode(2, little-endian), len(1), params(len)]` —
        // so each is sent as-is and its Command Complete awaited. The
        // final record is the Launch-RAM that restarts the controller.
        let mut offset = 0;
        let mut discard = [0u8; MAX_EVENT_PARAMS];
        while offset + 3 <= hcd.len() {
            let opcode = u16::from_le_bytes([hcd[offset], hcd[offset + 1]]);
            let len = hcd[offset + 2] as usize;
            let params_start = offset + 3;
            let params_end = params_start + len;
            if params_end > hcd.len() {
                return Err(Error::BadFirmware);
            }
            self.command(opcode, &hcd[params_start..params_end], &mut discard, timer)?;
            offset = params_end;
        }
        if offset != hcd.len() {
            return Err(Error::BadFirmware);
        }

        // Give the patched firmware time to restart, then resync.
        timer.delay_ms(LAUNCH_SETTLE_MS);
        self.reset(timer)?;
        Ok(())
    }

    /// Switches the HCI link to a higher baud rate.
    ///
    /// The controller boots — and comes back from a firmware launch — at
    /// 115200; raising the rate cuts transfer time and lifts the ceiling
    /// on ACL/data throughput. Call this after [`Self::load_firmware`],
    /// once the patched firmware is running. Raspberry Pi OS runs this
    /// link at 3_000_000 (3 Mbaud), which the Pi's PL011 represents
    /// exactly at its 48MHz reference clock.
    ///
    /// The switch is two-sided and ordered: the Broadcom `Update_Baudrate`
    /// vendor command tells the controller to change, its Command Complete
    /// comes back at the *old* rate (so it's read cleanly), and only then
    /// is the host PL011 reprogrammed to match. A brief settle follows
    /// before control returns.
    ///
    /// Returns [`Error::UnsupportedBaud`] if the host UART can't represent
    /// `baud`; because the controller is told to switch first, a host that
    /// then can't follow leaves the link out of sync — treat that as fatal
    /// and re-bring-up rather than retrying at the old rate.
    pub fn set_baud(&mut self, baud: u32, timer: &Timer) -> Result<(), Error> {
        // Parameter: 2-byte encoded-rate field (0 = use the literal value)
        // then the little-endian 32-bit baud.
        let mut params = [0u8; 6];
        params[2..6].copy_from_slice(&baud.to_le_bytes());
        self.command(OP_BCM_UPDATE_BAUDRATE, &params, &mut [], timer)?;

        // The controller has acknowledged (at the old rate) and switched;
        // bring the host UART to the same rate to follow it.
        if !self.uart.set_baud(baud) {
            return Err(Error::UnsupportedBaud(baud));
        }
        timer.delay_ms(BAUD_SETTLE_MS);
        Ok(())
    }

    /// Starts BLE advertising as a peripheral named `name`.
    ///
    /// Sets the advertising parameters (`ADV_INTERVAL` on all three
    /// primary channels, `kind` selecting connectable vs a pure
    /// broadcaster), builds the advertising data (a `Flags` AD structure
    /// followed by the device name, truncated to fit the 31-byte payload —
    /// marked `Shortened Local Name` if it didn't fit), and enables
    /// advertising. After this returns the controller advertises on its
    /// own; nothing further from the host is required to stay visible.
    ///
    /// This is controller-level GAP broadcasting only — no connection or
    /// GATT handling exists above it yet, so [`Advertising::NonConnectable`]
    /// is the behaviour that fully works today.
    pub fn start_advertising(
        &mut self,
        name: &str,
        kind: Advertising,
        timer: &Timer,
    ) -> Result<(), Error> {
        // A `Flags` AD structure followed by the device name, into the
        // 31-byte payload.
        let mut adv_data = [0u8; ADV_DATA_MAX];
        let len = build_adv_data(name, &mut adv_data);
        self.advertise(&adv_data[..len], kind, timer)
    }

    /// Starts BLE advertising with caller-supplied advertising data.
    ///
    /// Like [`Self::start_advertising`] but the `adv_data` (a sequence of
    /// `length/type/value` AD structures, ≤ `ADV_DATA_MAX` bytes) is built
    /// by the caller — for advertising an Appearance, service-class UUIDs, or
    /// other AD types beyond the plain name. Data past 31 bytes is truncated.
    pub fn start_advertising_raw(
        &mut self,
        adv_data: &[u8],
        kind: Advertising,
        timer: &Timer,
    ) -> Result<(), Error> {
        self.advertise(adv_data, kind, timer)
    }

    /// Sets the advertising parameters and data, then enables advertising —
    /// the shared body of [`Self::start_advertising`] and
    /// [`Self::start_advertising_raw`].
    fn advertise(
        &mut self,
        adv_data: &[u8],
        kind: Advertising,
        timer: &Timer,
    ) -> Result<(), Error> {
        // Advertising parameters: min/max interval, type, own/peer address
        // types (public/unused), peer address (unused), channel map,
        // filter policy. The zero-initialized bytes cover own-address-type
        // (public), the peer address type and address, and an
        // allow-any filter policy.
        let mut params = [0u8; 15];
        params[0..2].copy_from_slice(&ADV_INTERVAL.to_le_bytes());
        params[2..4].copy_from_slice(&ADV_INTERVAL.to_le_bytes());
        params[4] = kind.adv_type();
        params[13] = ADV_CHANNEL_ALL;
        self.command(OP_LE_SET_ADV_PARAMS, &params, &mut [], timer)?;

        // Advertising data: a 1-byte significant-length prefix followed by
        // a fixed 31-byte payload.
        let len = adv_data.len().min(ADV_DATA_MAX);
        let mut data = [0u8; 1 + ADV_DATA_MAX];
        data[0] = len as u8;
        data[1..1 + len].copy_from_slice(&adv_data[..len]);
        self.command(OP_LE_SET_ADV_DATA, &data, &mut [], timer)?;

        // Enable.
        self.command(OP_LE_SET_ADV_ENABLE, &[0x01], &mut [], timer)?;
        Ok(())
    }

    /// Stops BLE advertising previously started with
    /// [`Self::start_advertising`].
    pub fn stop_advertising(&mut self, timer: &Timer) -> Result<(), Error> {
        self.command(OP_LE_SET_ADV_ENABLE, &[0x00], &mut [], timer)?;
        Ok(())
    }

    /// Starts BLE scanning as a central: sets the scan parameters (active
    /// scanning on `SCAN_INTERVAL`/`SCAN_WINDOW`) and enables scanning
    /// with duplicate filtering on, so each nearby device is reported once
    /// per scan session. After this returns, drain the results with
    /// [`Self::next_advertising_report`].
    pub fn start_scan(&mut self, timer: &Timer) -> Result<(), Error> {
        // Scan parameters: type, interval, window, own-address-type, and
        // scanning-filter-policy. The trailing zero bytes are a public own
        // address and an accept-all filter policy.
        let mut params = [0u8; 7];
        params[0] = SCAN_TYPE_ACTIVE;
        params[1..3].copy_from_slice(&SCAN_INTERVAL.to_le_bytes());
        params[3..5].copy_from_slice(&SCAN_WINDOW.to_le_bytes());
        self.command(OP_LE_SET_SCAN_PARAMS, &params, &mut [], timer)?;

        // Enable scanning (byte 0) with duplicate filtering (byte 1).
        self.command(OP_LE_SET_SCAN_ENABLE, &[0x01, 0x01], &mut [], timer)?;
        Ok(())
    }

    /// Stops BLE scanning previously started with [`Self::start_scan`].
    pub fn stop_scan(&mut self, timer: &Timer) -> Result<(), Error> {
        self.command(OP_LE_SET_SCAN_ENABLE, &[0x00, 0x00], &mut [], timer)?;
        Ok(())
    }

    /// Selects the extended inquiry mode (`Write_Inquiry_Mode` = `0x02`) so
    /// subsequent [`Self::start_inquiry`] responses carry RSSI and an Extended
    /// Inquiry Response (from which [`InquiryResult::name`] reads the device
    /// name). Optional — a controller that rejects it still answers inquiries
    /// in the standard form (address + Class of Device, no RSSI or name). Call
    /// once after the firmware is up, before inquiring.
    pub fn set_inquiry_mode_extended(&mut self, timer: &Timer) -> Result<(), Error> {
        self.command(
            OP_WRITE_INQUIRY_MODE,
            &[INQUIRY_MODE_EXTENDED],
            &mut [],
            timer,
        )?;
        Ok(())
    }

    /// Starts a Bluetooth Classic (BR/EDR) inquiry — device discovery, the
    /// counterpart to [`Self::start_scan`] for LE. Broadcasts on the general
    /// inquiry access code; discovered devices are drained with
    /// [`Self::next_inquiry_result`].
    ///
    /// The command is accepted with a Command Status; the responses arrive as
    /// asynchronous `Inquiry Result` events. The inquiry runs for
    /// `INQUIRY_LENGTH` and then completes, which
    /// [`Self::next_inquiry_result`] restarts transparently, so discovery
    /// streams continuously until the caller stops polling (or calls
    /// [`Self::inquiry_cancel`]).
    pub fn start_inquiry(&mut self, timer: &Timer) -> Result<(), Error> {
        self.issue_inquiry(timer)
    }

    /// Sends one `Inquiry` command (GIAC, `INQUIRY_LENGTH`, unlimited
    /// responses) and waits for its Command Status — the shared body of
    /// [`Self::start_inquiry`] and the transparent restart in
    /// [`Self::next_inquiry_result`].
    fn issue_inquiry(&mut self, timer: &Timer) -> Result<(), Error> {
        let mut params = [0u8; 5];
        params[0..3].copy_from_slice(&GIAC_LAP);
        params[3] = INQUIRY_LENGTH;
        params[4] = INQUIRY_UNLIMITED_RESPONSES;
        self.command(OP_INQUIRY, &params, &mut [], timer)?;
        Ok(())
    }

    /// Stops an inquiry started with [`Self::start_inquiry`]
    /// (`Inquiry_Cancel`).
    pub fn inquiry_cancel(&mut self, timer: &Timer) -> Result<(), Error> {
        self.command(OP_INQUIRY_CANCEL, &[], &mut [], timer)?;
        Ok(())
    }

    /// Waits up to `timeout_ms` for the next Classic device to answer the
    /// inquiry, returning it (`Ok(Some(..))`) or `Ok(None)` if the window
    /// elapsed quietly — the Classic analog of [`Self::next_advertising_report`].
    /// Call it in a loop to stream discovered devices.
    ///
    /// An `Inquiry Complete` (the scan window ending) is handled internally by
    /// restarting the inquiry, so discovery continues across it rather than
    /// stopping; only a genuinely quiet `timeout_ms` returns `Ok(None)`. All
    /// three response formats (standard, with-RSSI, extended) are parsed; a
    /// batched multi-device standard event yields its first device.
    pub fn next_inquiry_result(
        &mut self,
        timer: &Timer,
        timeout_ms: u32,
    ) -> Result<Option<InquiryResult>, Error> {
        let deadline = timer.now_micros() + (timeout_ms as u64) * 1000;
        let mut buf = [0u8; MAX_EVENT_PARAMS];
        loop {
            let (code, params) = match self.read_packet(&mut buf, timer, deadline) {
                Ok(Packet::Event { code, params }) => (code, params),
                Ok(Packet::Acl { .. }) => continue,
                Err(Error::Timeout) => return Ok(None),
                Err(other) => return Err(other),
            };
            match code {
                EVT_INQUIRY_COMPLETE => {
                    // The scan window ended; restart so discovery streams on.
                    self.issue_inquiry(timer)?;
                }
                EVT_INQUIRY_RESULT | EVT_INQUIRY_RESULT_WITH_RSSI | EVT_EXTENDED_INQUIRY_RESULT => {
                    if let Some(result) = parse_inquiry_result(code, params) {
                        return Ok(Some(result));
                    }
                }
                _ => {}
            }
        }
    }

    /// Sets the controller's HCI event mask (`Set_Event_Mask`). Pass
    /// `EVENT_MASK_ALL` before Classic pairing to unmask the Secure Simple
    /// Pairing events, which are masked off by default.
    pub fn set_event_mask(&mut self, mask: u64, timer: &Timer) -> Result<(), Error> {
        self.command(OP_SET_EVENT_MASK, &mask.to_le_bytes(), &mut [], timer)?;
        Ok(())
    }

    /// Enables (or disables) Secure Simple Pairing on the controller
    /// (`Write_Simple_Pairing_Mode`). Call with `true` before
    /// [`Self::classic_pair`]; SSP must be on for the pairing handshake to run.
    pub fn set_simple_pairing_mode(&mut self, enabled: bool, timer: &Timer) -> Result<(), Error> {
        self.command(
            OP_WRITE_SIMPLE_PAIRING_MODE,
            &[enabled as u8],
            &mut [],
            timer,
        )?;
        Ok(())
    }

    /// Establishes a Bluetooth Classic ACL link by paging `bd_addr`
    /// (`Create_Connection`), returning the connection handle — the BR/EDR
    /// counterpart to [`Self::connect`].
    ///
    /// `bd_addr` is the device address in HCI wire order (LSB first — exactly
    /// as [`InquiryResult::bd_addr`] delivers it). This pages the device (it
    /// must be connectable, i.e. doing page scan — a controller in pairing
    /// mode is) and waits for the `Connection Complete`. A non-zero completion
    /// status (e.g. page timeout) is returned as [`Error::CommandFailed`].
    /// After this, authenticate with [`Self::classic_pair`].
    pub fn classic_connect(&mut self, bd_addr: &[u8; 6], timer: &Timer) -> Result<u16, Error> {
        // Parameters (13 bytes): bd_addr(6), packet_type(2), page scan
        // repetition mode(1), reserved(1), clock offset(2), allow role switch(1).
        let mut params = [0u8; 13];
        params[0..6].copy_from_slice(bd_addr);
        params[6..8].copy_from_slice(&CLASSIC_PACKET_TYPES.to_le_bytes());
        params[8] = PAGE_SCAN_REPETITION_R1;
        // params[9] reserved = 0; params[10..12] clock offset = 0.
        params[12] = 0x01; // allow role switch
        self.command(OP_CREATE_CONNECTION, &params, &mut [], timer)?;

        // Wait for Connection Complete: status(1), handle(2), bd_addr(6), …
        let deadline = timer.now_micros() + (CLASSIC_CONNECT_TIMEOUT_MS as u64) * 1000;
        let mut buf = [0u8; MAX_EVENT_PARAMS];
        loop {
            let (code, params) = match self.read_packet(&mut buf, timer, deadline)? {
                Packet::Event { code, params } => (code, params),
                Packet::Acl { .. } => continue,
            };
            if code == EVT_CONNECTION_COMPLETE && params.len() >= 3 {
                if params[0] != 0 {
                    return Err(Error::CommandFailed {
                        opcode: OP_CREATE_CONNECTION,
                        status: params[0],
                    });
                }
                return Ok(u16::from_le_bytes([params[1], params[2]]) & CONN_HANDLE_MASK);
            }
        }
    }

    /// Authenticates and pairs a Classic connection with LE Legacy-equivalent
    /// "Just Works" Secure Simple Pairing, returning the 16-byte link key.
    ///
    /// Requires SSP enabled ([`Self::set_simple_pairing_mode`]); the SSP events
    /// (masked off by default) are unmasked here automatically. Issues
    /// `Authentication_Requested` and then answers the controller's SSP
    /// handshake automatically: no stored key (negative Link Key Request
    /// reply), IO capability `IO_CAP_NO_INPUT_NO_OUTPUT` (Just Works),
    /// user-confirmation auto-accepted. The returned key is the bond — store it
    /// (with `bd_addr`) to reconnect encrypted without re-pairing. Follow with
    /// [`Self::classic_set_encryption`].
    ///
    /// A failed `Simple Pairing Complete` or `Authentication Complete` is
    /// returned as [`Error::CommandFailed`].
    pub fn classic_pair(
        &mut self,
        bd_addr: &[u8; 6],
        handle: u16,
        timer: &Timer,
    ) -> Result<[u8; 16], Error> {
        // The SSP handshake events are masked off by default; unmask them so
        // the IO-capability / user-confirmation events below are delivered.
        self.set_event_mask(EVENT_MASK_ALL, timer)?;
        self.command(
            OP_AUTHENTICATION_REQUESTED,
            &(handle & CONN_HANDLE_MASK).to_le_bytes(),
            &mut [],
            timer,
        )?;

        let deadline = timer.now_micros() + (CLASSIC_PAIR_TIMEOUT_MS as u64) * 1000;
        let mut buf = [0u8; MAX_EVENT_PARAMS];
        let mut link_key = [0u8; 16];
        loop {
            let (code, params) = match self.read_packet(&mut buf, timer, deadline)? {
                Packet::Event { code, params } => (code, params),
                Packet::Acl { .. } => continue,
            };
            match code {
                // No stored key → negative reply, which drives fresh SSP.
                EVT_LINK_KEY_REQUEST => {
                    self.send_command(OP_LINK_KEY_REQUEST_NEGATIVE_REPLY, bd_addr);
                }
                // Give our IO capability: NoInputNoOutput = Just Works.
                EVT_IO_CAPABILITY_REQUEST => {
                    let mut reply = [0u8; 9];
                    reply[0..6].copy_from_slice(bd_addr);
                    reply[6] = IO_CAP_NO_INPUT_NO_OUTPUT;
                    reply[7] = OOB_DATA_NONE;
                    reply[8] = AUTH_REQ_GENERAL_BONDING;
                    self.send_command(OP_IO_CAPABILITY_REQUEST_REPLY, &reply);
                }
                // Auto-accept the numeric value (Just Works).
                EVT_USER_CONFIRMATION_REQUEST => {
                    self.send_command(OP_USER_CONFIRMATION_REQUEST_REPLY, bd_addr);
                }
                // The bond key; keep it to return once pairing completes.
                EVT_LINK_KEY_NOTIFICATION if params.len() >= 22 => {
                    link_key.copy_from_slice(&params[6..22]);
                }
                EVT_SIMPLE_PAIRING_COMPLETE if !params.is_empty() => {
                    if params[0] != 0 {
                        return Err(Error::CommandFailed {
                            opcode: OP_AUTHENTICATION_REQUESTED,
                            status: params[0],
                        });
                    }
                }
                EVT_AUTHENTICATION_COMPLETE if !params.is_empty() => {
                    if params[0] != 0 {
                        return Err(Error::CommandFailed {
                            opcode: OP_AUTHENTICATION_REQUESTED,
                            status: params[0],
                        });
                    }
                    return Ok(link_key);
                }
                // Everything else — the peer's IO Capability Response (0x32)
                // and the Command Completes for our replies — is consumed here
                // without action.
                _ => {}
            }
        }
    }

    /// Turns on encryption for a Classic connection
    /// (`Set_Connection_Encryption`) and waits for the `Encryption Change`
    /// confirming it. Call after [`Self::classic_pair`]. A non-zero status or
    /// encryption-disabled result is returned as [`Error::CommandFailed`].
    pub fn classic_set_encryption(&mut self, handle: u16, timer: &Timer) -> Result<(), Error> {
        let mut params = [0u8; 3];
        params[0..2].copy_from_slice(&(handle & CONN_HANDLE_MASK).to_le_bytes());
        params[2] = CLASSIC_ENCRYPTION_ON;
        self.command(OP_SET_CONNECTION_ENCRYPTION, &params, &mut [], timer)?;

        let deadline = timer.now_micros() + (CLASSIC_ENCRYPT_TIMEOUT_MS as u64) * 1000;
        let mut buf = [0u8; MAX_EVENT_PARAMS];
        loop {
            let (code, params) = match self.read_packet(&mut buf, timer, deadline)? {
                Packet::Event { code, params } => (code, params),
                Packet::Acl { .. } => continue,
            };
            if code == EVT_ENCRYPTION_CHANGE && params.len() >= 4 {
                let ev_handle = u16::from_le_bytes([params[1], params[2]]) & CONN_HANDLE_MASK;
                if ev_handle != (handle & CONN_HANDLE_MASK) {
                    continue;
                }
                if params[0] != 0 || params[3] == 0 {
                    return Err(Error::CommandFailed {
                        opcode: OP_SET_CONNECTION_ENCRYPTION,
                        status: params[0],
                    });
                }
                return Ok(());
            }
        }
    }

    /// Initiates a connection to an advertising peripheral as a central
    /// (`LE_Create_Connection`).
    ///
    /// `peer_address` is the target's device address in HCI wire order
    /// (little-endian, LSB first — exactly as [`AdvReport::address`] delivers
    /// it), and `peer_address_type` its address type (`0x00` public, `0x01`
    /// random — pass [`AdvReport::address_type`] straight through). The
    /// connection is opened with the fixed parameters
    /// `CONN_INTERVAL_MIN`/`CONN_INTERVAL_MAX` etc.
    ///
    /// Scanning must be **stopped** first ([`Self::stop_scan`]); a controller
    /// rejects `LE_Create_Connection` while a scan is active. This returns as
    /// soon as the controller accepts the command (its Command Status); the
    /// link itself arrives asynchronously as an [`Event::Connected`] (with
    /// [`Role::Central`]) from [`Self::poll`]. If the target is never found
    /// the attempt stays pending until [`Self::connect_cancel`] aborts it.
    pub fn connect(
        &mut self,
        peer_address_type: u8,
        peer_address: &[u8; 6],
        timer: &Timer,
    ) -> Result<(), Error> {
        // Parameters (25 bytes): scan interval/window, initiator filter
        // policy (0 = connect to this one address, not a whitelist), peer
        // address type + address, own address type (0 = public), connection
        // interval min/max, latency, supervision timeout, and min/max CE
        // length (0 = let the controller choose).
        let mut params = [0u8; 25];
        params[0..2].copy_from_slice(&CONN_SCAN_INTERVAL.to_le_bytes());
        params[2..4].copy_from_slice(&CONN_SCAN_WINDOW.to_le_bytes());
        params[4] = 0x00; // initiator filter policy: use the peer address
        params[5] = peer_address_type;
        params[6..12].copy_from_slice(peer_address);
        params[12] = 0x00; // own address type: public
        params[13..15].copy_from_slice(&CONN_INTERVAL_MIN.to_le_bytes());
        params[15..17].copy_from_slice(&CONN_INTERVAL_MAX.to_le_bytes());
        params[17..19].copy_from_slice(&CONN_LATENCY.to_le_bytes());
        params[19..21].copy_from_slice(&CONN_SUPERVISION_TIMEOUT.to_le_bytes());
        // params[21..25] (min/max CE length) stay zero.
        self.command(OP_LE_CREATE_CONNECTION, &params, &mut [], timer)?;
        Ok(())
    }

    /// Aborts a connection attempt still in progress
    /// (`LE_Create_Connection_Cancel`) — the target was never found within
    /// the time a caller was willing to wait.
    ///
    /// The controller then emits an `LE Connection Complete` with a non-zero
    /// status (Unknown Connection Identifier), which [`Self::poll`] discards
    /// as a failed connection, so no spurious [`Event::Connected`] results.
    /// Calling this when no attempt is pending fails with a Command Disallowed
    /// [`Error::CommandFailed`].
    pub fn connect_cancel(&mut self, timer: &Timer) -> Result<(), Error> {
        self.command(OP_LE_CREATE_CONNECTION_CANCEL, &[], &mut [], timer)?;
        Ok(())
    }

    /// Waits up to `timeout_ms` for the next advertising report while
    /// scanning, returning it (`Ok(Some(..))`) or `Ok(None)` if the window
    /// elapsed with no report — the normal "quiet air" outcome, not an
    /// error. Call it in a loop to stream results.
    ///
    /// Unlike a command's Command Complete, advertising reports arrive
    /// unsolicited as `LE Advertising Report` events; any other event
    /// received meanwhile is skipped, as is a batched event carrying more
    /// than one device's report (not seen on this controller, which sends
    /// one per event).
    pub fn next_advertising_report(
        &mut self,
        timer: &Timer,
        timeout_ms: u32,
    ) -> Result<Option<AdvReport>, Error> {
        let deadline = timer.now_micros() + (timeout_ms as u64) * 1000;
        let mut buf = [0u8; MAX_EVENT_PARAMS];
        loop {
            let (code, params) = match self.read_packet(&mut buf, timer, deadline) {
                Ok(Packet::Event { code, params }) => (code, params),
                Ok(Packet::Acl { .. }) => continue,
                Err(Error::Timeout) => return Ok(None),
                Err(other) => return Err(other),
            };
            // Only LE Advertising Report events carry a device report.
            if code != EVT_LE_META || params.first() != Some(&LE_SUBEVENT_ADV_REPORT) {
                continue;
            }

            // params: [subevent(1), num_reports(1), report…]. This
            // controller sends exactly one report per event; a batched
            // multi-report event lays its fields out as parallel arrays,
            // not one contiguous report, so it's skipped rather than
            // misparsed (not observed on this hardware).
            if params.get(1) != Some(&1) {
                continue;
            }

            // A single report is event_type(1), addr_type(1), addr(6),
            // data_len(1), data(N), rssi(1) — at least 10 bytes (data may
            // be empty). RSSI is the *last* byte and the advertising data
            // is everything between the length byte and it. Both are located
            // from the HCI frame length (authoritative — `read_event` framed
            // the event by its own length byte) rather than the report's
            // interior `data_len` field, which on this controller does not
            // always match the framing: trusting it put RSSI on a trailing
            // data byte (garbled, sometimes positive) for some reports.
            let report = &params[2..];
            if report.len() < 10 {
                continue;
            }
            let data = &report[9..report.len() - 1];
            let rssi = report[report.len() - 1] as i8;

            let copy = data.len().min(ADV_DATA_MAX);
            let mut data_buf = [0u8; ADV_DATA_MAX];
            data_buf[..copy].copy_from_slice(&data[..copy]);
            let mut address = [0u8; 6];
            address.copy_from_slice(&report[2..8]);
            return Ok(Some(AdvReport {
                event_type: report[0],
                address_type: report[1],
                address,
                rssi,
                data_buf,
                data_len: copy as u8,
            }));
        }
    }

    /// Reads the controller's LE ACL buffer sizing
    /// (`LE_Read_Buffer_Size`) and arms host-side ACL flow control.
    ///
    /// Records the maximum ACL data payload the controller accepts per
    /// packet (which [`Self::send_acl`] fragments to) and seeds the TX
    /// credit count with how many such packets it can buffer. From then on
    /// [`Self::send_acl`] spends a credit per fragment and [`Self::poll`]
    /// returns them as the controller's `Number Of Completed Packets`
    /// events report send completions. Returns `(acl_packet_len,
    /// total_packets)`.
    ///
    /// Call once after the firmware is up (and typically after
    /// [`Self::set_baud`]) and before sending ACL data. Some controllers
    /// report a zero LE packet length, meaning "no dedicated LE buffers, use
    /// the shared BR/EDR pool"; this driver then leaves fragmentation off
    /// (single-fragment sends) and uses the returned packet count as-is.
    pub fn le_read_buffer_size(&mut self, timer: &Timer) -> Result<(u16, u8), Error> {
        // Return parameters: le_acl_data_packet_length(2),
        // total_num_le_acl_data_packets(1) = 3 bytes.
        let mut ret = [0u8; 3];
        let n = self.command(OP_LE_READ_BUFFER_SIZE, &[], &mut ret, timer)?;
        if n < ret.len() {
            return Err(Error::ShortResponse);
        }
        let packet_len = u16::from_le_bytes([ret[0], ret[1]]);
        let total_packets = ret[2];
        self.acl_packet_len = packet_len;
        self.acl_credits = total_packets as u16;
        Ok((packet_len, total_packets))
    }

    /// Reads the controller's Classic (BR/EDR) ACL buffer sizing
    /// (`Read_Buffer_Size`) and arms host-side ACL flow control — the Classic
    /// counterpart to [`Self::le_read_buffer_size`].
    ///
    /// Records the maximum ACL data payload the controller accepts per packet
    /// (which [`Self::send_acl`] fragments to) and seeds the TX credit count
    /// with how many such packets it can buffer. Call once after the firmware
    /// is up (and typically after [`Self::set_baud`]) and before sending any
    /// ACL data on a Classic link — without it the credit count is zero and the
    /// first [`Self::send_acl`] fails with [`Error::NoAclBuffers`]. Returns
    /// `(acl_packet_len, total_packets)`.
    pub fn read_buffer_size(&mut self, timer: &Timer) -> Result<(u16, u16), Error> {
        // Return parameters: acl_data_packet_length(2), sco_data_packet_length(1),
        // total_num_acl_data_packets(2), total_num_sco_data_packets(2) = 7 bytes.
        let mut ret = [0u8; 7];
        let n = self.command(OP_READ_BUFFER_SIZE, &[], &mut ret, timer)?;
        if n < ret.len() {
            return Err(Error::ShortResponse);
        }
        let packet_len = u16::from_le_bytes([ret[0], ret[1]]);
        let total_packets = u16::from_le_bytes([ret[3], ret[4]]);
        self.acl_packet_len = packet_len;
        self.acl_credits = total_packets;
        Ok((packet_len, total_packets))
    }

    /// AES-128-encrypts one 128-bit block with a 128-bit key, in the
    /// controller (`LE_Encrypt`), returning the 16-byte ciphertext.
    ///
    /// This is the AES primitive the LE pairing crypto is built on
    /// ([`crate::bluetooth::smp`]) — offloading it to the controller means
    /// the host carries no software AES. `key` and `plaintext` are passed to
    /// the controller verbatim and the ciphertext is returned verbatim; the
    /// caller owns any byte-order convention (the controller's octet order
    /// is pinned by [`smp`]'s boot self-test against a known AES vector).
    pub fn le_encrypt(
        &mut self,
        key: &[u8; 16],
        plaintext: &[u8; 16],
        timer: &Timer,
    ) -> Result<[u8; 16], Error> {
        let mut params = [0u8; 32];
        params[..16].copy_from_slice(key);
        params[16..].copy_from_slice(plaintext);
        let mut ret = [0u8; 16];
        let n = self.command(OP_LE_ENCRYPT, &params, &mut ret, timer)?;
        if n < ret.len() {
            return Err(Error::ShortResponse);
        }
        Ok(ret)
    }

    /// Returns 8 bytes of controller-generated randomness (`LE_Rand`) — the
    /// entropy source for SMP pairing randoms.
    pub fn le_rand(&mut self, timer: &Timer) -> Result<[u8; 8], Error> {
        let mut ret = [0u8; 8];
        let n = self.command(OP_LE_RAND, &[], &mut ret, timer)?;
        if n < ret.len() {
            return Err(Error::ShortResponse);
        }
        Ok(ret)
    }

    /// Starts link encryption as the central (`LE_Start_Encryption`), the
    /// counterpart to a peripheral's [`Self::le_ltk_request_reply`].
    ///
    /// Once the SMP pairing exchange has derived the Short Term Key (the
    /// initiator side of [`crate::bluetooth::smp`]), the central commands the
    /// controller to turn on encryption with it: pass `ltk` = the STK and
    /// `ediv`/`rand` both zero. On a bonded reconnect, pass the stored LTK with
    /// the EDIV/Rand the peripheral distributed when the bond was made.
    ///
    /// The command is accepted with a Command Status; the result — the link
    /// becoming encrypted, or the attempt failing — arrives later as an
    /// [`Event::EncryptionChange`] from [`Self::poll`].
    pub fn le_start_encryption(
        &mut self,
        handle: u16,
        rand: [u8; 8],
        ediv: u16,
        ltk: &[u8; 16],
        timer: &Timer,
    ) -> Result<(), Error> {
        // Parameters: connection_handle(2), random(8), ediv(2), ltk(16).
        let mut params = [0u8; 28];
        params[0..2].copy_from_slice(&(handle & CONN_HANDLE_MASK).to_le_bytes());
        params[2..10].copy_from_slice(&rand);
        params[10..12].copy_from_slice(&ediv.to_le_bytes());
        params[12..28].copy_from_slice(ltk);
        self.command(OP_LE_START_ENCRYPTION, &params, &mut [], timer)?;
        Ok(())
    }

    /// Supplies the Long Term Key for a pending [`Event::LongTermKeyRequest`]
    /// (`LE_Long_Term_Key_Request_Reply`), letting the controller turn on
    /// link encryption. For LE Legacy pairing this key is the STK derived by
    /// the Security Manager ([`crate::bluetooth::smp`]).
    pub fn le_ltk_request_reply(
        &mut self,
        handle: u16,
        ltk: &[u8; 16],
        timer: &Timer,
    ) -> Result<(), Error> {
        let mut params = [0u8; 18];
        params[0..2].copy_from_slice(&(handle & CONN_HANDLE_MASK).to_le_bytes());
        params[2..18].copy_from_slice(ltk);
        self.command(OP_LE_LTK_REQUEST_REPLY, &params, &mut [], timer)?;
        Ok(())
    }

    /// Rejects a pending [`Event::LongTermKeyRequest`]
    /// (`LE_Long_Term_Key_Request_Negative_Reply`) — used when no key matches
    /// the requested `ediv`/`rand` (an unknown bond), so the encryption
    /// attempt fails cleanly rather than hanging.
    pub fn le_ltk_request_negative_reply(
        &mut self,
        handle: u16,
        timer: &Timer,
    ) -> Result<(), Error> {
        let params = (handle & CONN_HANDLE_MASK).to_le_bytes();
        self.command(OP_LE_LTK_REQUEST_NEG_REPLY, &params, &mut [], timer)?;
        Ok(())
    }

    /// Sends an L2CAP-over-ACL payload to a connection, fragmenting it to
    /// the controller's ACL packet length and spending one TX credit per
    /// fragment.
    ///
    /// `data` is a complete L2CAP PDU (or whatever payload a higher layer
    /// has framed); this splits it across ACL packets no larger than the
    /// [`Self::le_read_buffer_size`] length, tagging the first fragment as a
    /// PDU start and the rest as continuations. It does not build the L2CAP
    /// header — that's the caller's job.
    ///
    /// Returns [`Error::NoAclBuffers`] if there aren't enough free TX
    /// credits for every fragment (none are sent in that case, so a partial
    /// PDU never goes out); pump [`Self::poll`] to let the controller's
    /// completions replenish credits, then retry. At keyboard-scale traffic
    /// the controller's buffer count is never exhausted.
    pub fn send_acl(&mut self, handle: u16, data: &[u8]) -> Result<(), Error> {
        // A zero reported packet length means "no dedicated LE buffers";
        // fall back to a single fragment carrying the whole payload.
        let frag_len = if self.acl_packet_len == 0 {
            data.len().max(1)
        } else {
            self.acl_packet_len as usize
        };

        // Number of fragments, rounding up (at least one, so an empty
        // payload still sends a single zero-length ACL packet).
        let fragments = data.len().div_ceil(frag_len).max(1);
        if (fragments as u16) > self.acl_credits {
            return Err(Error::NoAclBuffers);
        }

        let mut offset = 0;
        let mut first = true;
        while offset < data.len() || first {
            let end = (offset + frag_len).min(data.len());
            let chunk = &data[offset..end];
            let pb = if first {
                ACL_PB_FIRST_NON_FLUSH
            } else {
                ACL_PB_CONTINUATION
            };
            self.send_acl_fragment(handle, pb, chunk);
            self.acl_credits -= 1;
            offset = end;
            first = false;
        }
        self.uart.flush();
        Ok(())
    }

    /// Writes one ACL data frame: the [`H4_ACL`] type byte, the 16-bit
    /// handle with the `PB`/`BC` flags in its top bits, the 16-bit data
    /// length, then the payload. Does not flush — [`Self::send_acl`] flushes
    /// once after the last fragment.
    fn send_acl_fragment(&mut self, handle: u16, pb: u16, data: &[u8]) {
        let handle_flags = (handle & CONN_HANDLE_MASK) | (pb << 12);
        let handle_flags = handle_flags.to_le_bytes();
        let len = (data.len() as u16).to_le_bytes();
        self.uart.write_byte(H4_ACL);
        self.uart.write_byte(handle_flags[0]);
        self.uart.write_byte(handle_flags[1]);
        self.uart.write_byte(len[0]);
        self.uart.write_byte(len[1]);
        for &byte in data {
            self.uart.write_byte(byte);
        }
    }

    /// Waits up to `timeout_ms` for the next asynchronous connection event,
    /// returning it (`Ok(Some(..))`) or `Ok(None)` if the window elapsed
    /// quietly. Call it in a loop to service a connection.
    ///
    /// This is the pump for connection-oriented traffic, the counterpart to
    /// [`Self::next_advertising_report`] for scanning. It reads HCI packets
    /// and surfaces the ones a connection handler cares about —
    /// [`Event::Connected`], [`Event::Disconnected`], and inbound
    /// [`Event::Acl`] data. `Number Of Completed Packets` events are
    /// consumed here to replenish [`Self::send_acl`] TX credits and never
    /// surface; a failed `LE Connection Complete` (non-zero status, no link
    /// formed) and any other unrelated event are skipped.
    pub fn poll(&mut self, timer: &Timer, timeout_ms: u32) -> Result<Option<Event>, Error> {
        let deadline = timer.now_micros() + (timeout_ms as u64) * 1000;
        // Sized for the larger of an event's params and an ACL fragment.
        let mut buf = [0u8; MAX_ACL_DATA];
        loop {
            let packet = match self.read_packet(&mut buf, timer, deadline) {
                Ok(packet) => packet,
                Err(Error::Timeout) => return Ok(None),
                Err(other) => return Err(other),
            };
            match packet {
                Packet::Acl {
                    handle,
                    first_fragment,
                    data,
                } => {
                    let mut data_buf = [0u8; MAX_ACL_DATA];
                    let copy = data.len().min(MAX_ACL_DATA);
                    data_buf[..copy].copy_from_slice(&data[..copy]);
                    return Ok(Some(Event::Acl(AclData {
                        handle,
                        first_fragment,
                        data_buf,
                        data_len: copy as u16,
                    })));
                }
                Packet::Event { code, params } => {
                    match code {
                        EVT_LE_META if params.first() == Some(&LE_SUBEVENT_CONNECTION_COMPLETE) => {
                            // params: subevent(1), status(1), handle(2),
                            // role(1), peer_addr_type(1), peer_addr(6), …
                            if params.len() < 12 || params[1] != 0 {
                                // Non-zero status means no link formed; wait
                                // for a real one rather than reporting it.
                                continue;
                            }
                            let handle =
                                u16::from_le_bytes([params[2], params[3]]) & CONN_HANDLE_MASK;
                            let mut peer_address = [0u8; 6];
                            peer_address.copy_from_slice(&params[6..12]);
                            return Ok(Some(Event::Connected(Connection {
                                handle,
                                role: Role::from_byte(params[4]),
                                peer_address_type: params[5],
                                peer_address,
                            })));
                        }
                        EVT_LE_META if params.first() == Some(&LE_SUBEVENT_LTK_REQUEST) => {
                            // params: subevent(1), handle(2), rand(8), ediv(2).
                            // rand/ediv are zero for an in-progress pairing's
                            // STK, or the bonded LTK's identifiers on reconnect.
                            if params.len() < 13 {
                                continue;
                            }
                            let handle =
                                u16::from_le_bytes([params[1], params[2]]) & CONN_HANDLE_MASK;
                            let mut rand = [0u8; 8];
                            rand.copy_from_slice(&params[3..11]);
                            let ediv = u16::from_le_bytes([params[11], params[12]]);
                            return Ok(Some(Event::LongTermKeyRequest { handle, ediv, rand }));
                        }
                        EVT_DISCONNECTION_COMPLETE if params.len() >= 4 => {
                            // params: status(1), handle(2), reason(1).
                            let handle =
                                u16::from_le_bytes([params[1], params[2]]) & CONN_HANDLE_MASK;
                            return Ok(Some(Event::Disconnected {
                                handle,
                                reason: params[3],
                            }));
                        }
                        EVT_ENCRYPTION_CHANGE if params.len() >= 4 => {
                            // params: status(1), handle(2), encryption_enabled(1).
                            let handle =
                                u16::from_le_bytes([params[1], params[2]]) & CONN_HANDLE_MASK;
                            return Ok(Some(Event::EncryptionChange {
                                handle,
                                enabled: params[3] != 0,
                            }));
                        }
                        EVT_NUM_COMPLETED_PACKETS => {
                            self.credit_completed_packets(params);
                            continue;
                        }
                        _ => continue,
                    }
                }
            }
        }
    }

    /// Adds the completions in a `Number Of Completed Packets` event back to
    /// the ACL TX credit pool. The event is `num_handles(1)` followed by
    /// that many `(handle(2), num_completed(2))` pairs; every completion
    /// frees one controller buffer regardless of which handle it was on.
    fn credit_completed_packets(&mut self, params: &[u8]) {
        let Some(&num_handles) = params.first() else {
            return;
        };
        for i in 0..num_handles as usize {
            // Each entry is 4 bytes, starting after the 1-byte count; the
            // completion count is the second half of the pair.
            let base = 1 + i * 4;
            if base + 4 > params.len() {
                break;
            }
            let completed = u16::from_le_bytes([params[base + 2], params[base + 3]]);
            self.acl_credits = self.acl_credits.saturating_add(completed);
        }
    }

    /// Tears down a connection by handle (`HCI_Disconnect`), asking the
    /// controller to drop the link with the given HCI reason code (`0x13`
    /// "remote user terminated" is the conventional local-initiated value).
    /// The command is accepted with a Command Status; the actual teardown
    /// arrives later as an [`Event::Disconnected`] from [`Self::poll`].
    pub fn disconnect(&mut self, handle: u16, reason: u8, timer: &Timer) -> Result<(), Error> {
        let mut params = [0u8; 3];
        params[0..2].copy_from_slice(&(handle & CONN_HANDLE_MASK).to_le_bytes());
        params[2] = reason;
        self.command(OP_DISCONNECT, &params, &mut [], timer)?;
        Ok(())
    }

    /// Issues `HCI_Reset`, resetting the controller's link layer and
    /// baseband. Used both to sync with the ROM before a firmware
    /// download and to resync after the patched firmware restarts.
    pub fn reset(&mut self, timer: &Timer) -> Result<(), Error> {
        self.command(OP_RESET, &[], &mut [], timer)?;
        Ok(())
    }

    /// Reads the controller's local version information
    /// (`Read_Local_Version_Information`). For a genuine BCM43438 the
    /// `manufacturer` field reads back as Broadcom (`0x000f`) — a good
    /// "the firmware is running and answering" check.
    pub fn read_local_version(&mut self, timer: &Timer) -> Result<LocalVersion, Error> {
        // Return parameters: hci_version(1), hci_revision(2),
        // lmp_version(1), manufacturer(2), lmp_subversion(2) = 8 bytes.
        let mut ret = [0u8; 8];
        let n = self.command(OP_READ_LOCAL_VERSION, &[], &mut ret, timer)?;
        if n < ret.len() {
            return Err(Error::ShortResponse);
        }
        Ok(LocalVersion {
            hci_version: ret[0],
            hci_revision: u16::from_le_bytes([ret[1], ret[2]]),
            lmp_version: ret[3],
            manufacturer: u16::from_le_bytes([ret[4], ret[5]]),
            lmp_subversion: u16::from_le_bytes([ret[6], ret[7]]),
        })
    }

    /// Reads the controller's public Bluetooth device address
    /// (`Read_BD_ADDR`). The six bytes are returned in HCI wire order —
    /// little-endian, least-significant byte first — so a human-readable
    /// `AA:BB:CC:DD:EE:FF` prints the returned array in reverse.
    pub fn read_bd_addr(&mut self, timer: &Timer) -> Result<[u8; 6], Error> {
        let mut ret = [0u8; 6];
        let n = self.command(OP_READ_BD_ADDR, &[], &mut ret, timer)?;
        if n < ret.len() {
            return Err(Error::ShortResponse);
        }
        Ok(ret)
    }

    /// Releases the underlying UART, consuming the driver. Lets a caller
    /// reclaim the PL011 (e.g. to hand it back to the console) once it's
    /// done with Bluetooth.
    pub fn free(self) -> Uart {
        self.uart
    }

    /// Sends one HCI command over H4 and waits for its matching Command
    /// Complete (or Command Status) event, copying up to `ret.len()`
    /// return-parameter bytes into `ret` and returning the number the
    /// controller actually supplied (which may exceed `ret.len()`).
    ///
    /// Events for other opcodes that arrive while waiting are skipped —
    /// the controller can interleave unrelated events — until the one for
    /// `opcode` shows up or the time budget runs out.
    fn command(
        &mut self,
        opcode: u16,
        params: &[u8],
        ret: &mut [u8],
        timer: &Timer,
    ) -> Result<usize, Error> {
        self.send_command(opcode, params);

        let deadline = timer.now_micros() + (COMMAND_TIMEOUT_MS as u64) * 1000;
        let mut buf = [0u8; MAX_EVENT_PARAMS];
        loop {
            // Only events answer a command; ACL data that arrives mid-wait
            // (a live connection's inbound traffic) is not this command's
            // reply, so it's skipped here. A caller expecting ACL uses
            // `poll` instead, which surfaces it.
            let (code, params) = match self.read_packet(&mut buf, timer, deadline)? {
                Packet::Event { code, params } => (code, params),
                Packet::Acl { .. } => continue,
            };
            match code {
                // [num_cmd_pkts(1), opcode(2), status(1), return_params…]
                EVT_COMMAND_COMPLETE if params.len() >= 3 => {
                    if u16::from_le_bytes([params[1], params[2]]) != opcode {
                        continue;
                    }
                    let status = *params.get(3).ok_or(Error::ShortResponse)?;
                    if status != 0 {
                        return Err(Error::CommandFailed { opcode, status });
                    }
                    let payload = &params[4..];
                    let copied = payload.len().min(ret.len());
                    ret[..copied].copy_from_slice(&payload[..copied]);
                    return Ok(payload.len());
                }
                // [status(1), num_cmd_pkts(1), opcode(2)]
                EVT_COMMAND_STATUS if params.len() >= 4 => {
                    if u16::from_le_bytes([params[2], params[3]]) != opcode {
                        continue;
                    }
                    if params[0] != 0 {
                        return Err(Error::CommandFailed {
                            opcode,
                            status: params[0],
                        });
                    }
                    // Accepted; the real result arrives in a later event,
                    // which the informational commands here never use.
                    return Ok(0);
                }
                _ => continue,
            }
        }
    }

    /// Writes one HCI command as an H4 frame: the [`H4_COMMAND`] type
    /// byte, the little-endian opcode, the parameter length, then the
    /// parameters. Flushes so the whole frame is on the wire before the
    /// reply is awaited.
    fn send_command(&mut self, opcode: u16, params: &[u8]) {
        let opcode = opcode.to_le_bytes();
        self.uart.write_byte(H4_COMMAND);
        self.uart.write_byte(opcode[0]);
        self.uart.write_byte(opcode[1]);
        self.uart.write_byte(params.len() as u8);
        for &byte in params {
            self.uart.write_byte(byte);
        }
        self.uart.flush();
    }

    /// Reads one complete HCI packet into `buf` by `deadline`, dispatching
    /// on its H4 type byte to an [`Packet::Event`] or [`Packet::Acl`].
    ///
    /// Reads the type byte first, tolerating (and skipping) any byte that is
    /// neither [`H4_EVENT`] nor [`H4_ACL`] — a stray leading byte, or a
    /// packet type this transport never carries. Then reads that packet's
    /// header and payload: an event's `code` + length-prefixed parameters,
    /// or an ACL packet's handle/flags + length-prefixed data. Every byte
    /// read is bounded by the shared `deadline` so a silent controller
    /// surfaces as [`Error::Timeout`] rather than hanging.
    ///
    /// A payload longer than `buf` (only possible for a misframed ACL
    /// length, since an event length is a `u8`) is truncated into `buf` with
    /// the excess drained off the wire, so framing stays aligned for the
    /// next packet rather than desyncing.
    fn read_packet<'a>(
        &mut self,
        buf: &'a mut [u8],
        timer: &Timer,
        deadline: u64,
    ) -> Result<Packet<'a>, Error> {
        loop {
            match self.read_byte_by(timer, deadline)? {
                H4_EVENT => {
                    let code = self.read_byte_by(timer, deadline)?;
                    let len = self.read_byte_by(timer, deadline)? as usize;
                    let params = self.read_payload(buf, len, timer, deadline)?;
                    return Ok(Packet::Event { code, params });
                }
                H4_ACL => {
                    let lo = self.read_byte_by(timer, deadline)?;
                    let hi = self.read_byte_by(timer, deadline)?;
                    let handle_flags = u16::from_le_bytes([lo, hi]);
                    let len_lo = self.read_byte_by(timer, deadline)?;
                    let len_hi = self.read_byte_by(timer, deadline)?;
                    let len = u16::from_le_bytes([len_lo, len_hi]) as usize;
                    let data = self.read_payload(buf, len, timer, deadline)?;
                    return Ok(Packet::Acl {
                        handle: handle_flags & CONN_HANDLE_MASK,
                        // PB flag is bits 12-13; bit 12 distinguishes a
                        // first (0b00/0b10) from a continuation (0b01)
                        // fragment for LE ACL-U data.
                        first_fragment: (handle_flags >> 12) & 0x03 != ACL_PB_CONTINUATION,
                        data,
                    });
                }
                _ => continue,
            }
        }
    }

    /// Reads `len` payload bytes off the wire, storing up to `buf.len()` of
    /// them in `buf` and draining any excess (so an over-long frame doesn't
    /// desync the stream), and returns the stored prefix.
    fn read_payload<'a>(
        &mut self,
        buf: &'a mut [u8],
        len: usize,
        timer: &Timer,
        deadline: u64,
    ) -> Result<&'a [u8], Error> {
        let stored = len.min(buf.len());
        for slot in buf[..stored].iter_mut() {
            *slot = self.read_byte_by(timer, deadline)?;
        }
        for _ in stored..len {
            self.read_byte_by(timer, deadline)?;
        }
        Ok(&buf[..stored])
    }

    /// Polls the UART for one byte until it arrives or `deadline`
    /// (an absolute [`Timer::now_micros`] value) passes, in which case it
    /// returns [`Error::Timeout`].
    fn read_byte_by(&mut self, timer: &Timer, deadline: u64) -> Result<u8, Error> {
        loop {
            if let Some(byte) = self.uart.try_read_byte() {
                return Ok(byte);
            }
            if timer.now_micros() >= deadline {
                return Err(Error::Timeout);
            }
        }
    }
}

/// Builds the advertising-data payload into `out` (which must be at least
/// `ADV_DATA_MAX` bytes), returning the number of significant bytes
/// written.
///
/// Emits two AD structures: `Flags` (LE-general / BR-EDR-not-supported)
/// and the device name. The name is truncated to whatever space is left in
/// the 31-byte payload and tagged `Complete`/`Shortened Local Name`
/// accordingly. Truncation is on raw bytes, so a name split mid-character
/// may render oddly in a scanner but is still valid on the wire.
fn build_adv_data(name: &str, out: &mut [u8]) -> usize {
    // Flags AD structure: length, type, value.
    out[0] = 2;
    out[1] = AD_TYPE_FLAGS;
    out[2] = AD_FLAGS_LE_GENERAL;
    let mut len = 3;

    // Name AD structure fills the remaining space (2 bytes of AD header
    // plus as much of the name as fits).
    let name = name.as_bytes();
    let room = ADV_DATA_MAX - len - 2;
    let take = name.len().min(room);
    let ad_type = if take < name.len() {
        AD_TYPE_NAME_SHORT
    } else {
        AD_TYPE_NAME_COMPLETE
    };
    out[len] = (take + 1) as u8;
    out[len + 1] = ad_type;
    out[len + 2..len + 2 + take].copy_from_slice(&name[..take]);
    len += 2 + take;

    len
}

/// Parses one inquiry-result event body into an [`InquiryResult`], or `None`
/// if it's too short to hold a response. Handles all three formats: the
/// with-RSSI (`0x22`) and extended (`0x2f`) events share a contiguous
/// per-response prefix (address, Class of Device, RSSI), with the extended
/// event adding an Extended Inquiry Response the name is read from; the
/// standard event (`0x02`) lays its fields out as parallel arrays, of which
/// the first device's are read.
fn parse_inquiry_result(code: u8, params: &[u8]) -> Option<InquiryResult> {
    match code {
        EVT_EXTENDED_INQUIRY_RESULT | EVT_INQUIRY_RESULT_WITH_RSSI => {
            // [num(1)=1, bd_addr(6), psrm(1), reserved(1), cod(3), clock(2),
            // rssi(1), (extended: eir(240))]. Offsets: bd_addr@1, cod@9, rssi@14.
            if params.len() < 15 {
                return None;
            }
            let mut bd_addr = [0u8; 6];
            bd_addr.copy_from_slice(&params[1..7]);
            let mut class_of_device = [0u8; 3];
            class_of_device.copy_from_slice(&params[9..12]);
            let rssi = Some(params[14] as i8);
            let (name_buf, name_len) = if code == EVT_EXTENDED_INQUIRY_RESULT && params.len() > 15 {
                extract_eir_name(&params[15..])
            } else {
                ([0u8; EIR_NAME_MAX], 0)
            };
            Some(InquiryResult {
                bd_addr,
                class_of_device,
                rssi,
                name_buf,
                name_len,
            })
        }
        EVT_INQUIRY_RESULT => {
            // [num(1), bd_addr(6*num), psrm(1*num), reserved(2*num),
            // cod(3*num), clock(2*num)]. Read the first device only.
            let num = *params.first()? as usize;
            if num == 0 {
                return None;
            }
            // Bytes per device before the Class-of-Device array: 6 + 1 + 2.
            let cod_off = 1 + 9 * num;
            if params.len() < cod_off + 3 {
                return None;
            }
            let mut bd_addr = [0u8; 6];
            bd_addr.copy_from_slice(&params[1..7]);
            let mut class_of_device = [0u8; 3];
            class_of_device.copy_from_slice(&params[cod_off..cod_off + 3]);
            Some(InquiryResult {
                bd_addr,
                class_of_device,
                rssi: None,
                name_buf: [0u8; EIR_NAME_MAX],
                name_len: 0,
            })
        }
        _ => None,
    }
}

/// Extracts a device name from an Extended Inquiry Response (the same
/// `length/type/value` AD structures as advertising data): the complete name
/// if present, else the shortened name, copied into a fixed buffer (truncated
/// to [`EIR_NAME_MAX`]). Returns the buffer and its significant length.
fn extract_eir_name(eir: &[u8]) -> ([u8; EIR_NAME_MAX], u8) {
    let mut buf = [0u8; EIR_NAME_MAX];
    let mut len = 0u8;
    if let Some(name) = find_ad_structure(eir, AD_TYPE_NAME_COMPLETE)
        .or_else(|| find_ad_structure(eir, AD_TYPE_NAME_SHORT))
    {
        let n = name.len().min(EIR_NAME_MAX);
        buf[..n].copy_from_slice(&name[..n]);
        len = n as u8;
    }
    (buf, len)
}

/// Finds the value of the first AD structure of type `ad_type` in an
/// advertising-data byte string, or `None` if absent.
///
/// Advertising data is a sequence of `[length, type, value…]` structures
/// where `length` counts the type byte plus the value. Walks them,
/// stopping at a zero length (padding) or a length that runs past the end
/// (malformed) rather than reading out of bounds.
fn find_ad_structure(data: &[u8], ad_type: u8) -> Option<&[u8]> {
    let mut i = 0;
    while i < data.len() {
        let len = data[i] as usize;
        if len == 0 {
            break;
        }
        let end = i + 1 + len;
        if end > data.len() {
            break;
        }
        if data[i + 1] == ad_type {
            return Some(&data[i + 2..end]);
        }
        i = end;
    }
    None
}
