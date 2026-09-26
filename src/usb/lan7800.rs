//! The Microchip LAN7800 USB Gigabit Ethernet controller — the Ethernet
//! half of the LAN7515 fitted to a Raspberry Pi 3 Model B+.
//!
//! The sibling of [`crate::usb::lan9514`], which drives the SMSC part on a
//! Pi 2B/3B. They are not variants of one driver: both are programmed
//! through vendor control transfers, and that is where the resemblance
//! ends. The register map, the PHY bring-up, and the headers wrapped
//! around a frame in each direction are all this part's own. What carries
//! over is the shape — configure from a [`Device`], find the bulk
//! endpoints, program the MAC, poll the link, move frames — so a consumer
//! written against one ports to the other by changing the type name and
//! the sizes it assumes.
//!
//! Reaching it at all needs the hub traversal in
//! [`Bus`](crate::usb::Bus): on a 3B+ this sits behind *two* cascaded
//! hubs, and it attaches seconds after power-on rather than at boot, so it
//! appears through [`Bus::poll`](crate::usb::Bus::poll) rather than in the
//! initial walk. An example that only calls
//! [`enumerate`](crate::usb::enumerate) will not find it, and the way that
//! failure looks is an empty bus rather than an error.
//!
//! Register access is a 32-bit read or write per vendor request, with the
//! register's offset in `wIndex` and its value little-endian in the data
//! stage — see
//! [`Lan7800::read_register`](crate::usb::lan7800::Lan7800::read_register).

use crate::timer::Timer;
use crate::usb::control::{get_configuration_descriptor, set_configuration, vendor_in, vendor_out};
use crate::usb::descriptor::{ConfigurationDescriptor, Descriptors, EndpointDescriptor};
use crate::usb::dwc2::{Channel, ControlEndpoint, TransferError};
use crate::usb::Device;

/// USB vendor ID of the LAN7800 (Microchip, formerly SMSC) — the value to
/// match a [`Device`] against before handing it to [`Lan7800::from_device`].
pub const VENDOR_ID: u16 = 0x0424;
/// USB product ID of the LAN7800 Ethernet function.
pub const PRODUCT_ID: u16 = 0x7800;

/// Vendor request reading one 32-bit register.
const READ_REGISTER: u8 = 0xA1;
/// Vendor request writing one 32-bit register.
const WRITE_REGISTER: u8 = 0xA0;

/// `ID_REV` — chip ID in the high half, silicon revision in the low.
const REG_ID_REV: u16 = 0x000;
/// `INT_STS` — interrupt status, write-one-to-clear.
const REG_INT_STS: u16 = 0x00C;
/// `HW_CFG` — device-level configuration, including the soft reset.
const REG_HW_CFG: u16 = 0x010;
/// `PMT_CTL` — power management, and the PHY reset.
const REG_PMT_CTL: u16 = 0x014;
/// `USB_CFG0` — USB-side behaviour, including the empty bulk-IN response.
const REG_USB_CFG0: u16 = 0x080;
/// `RFE_CTL` — receive filtering engine.
const REG_RFE_CTL: u16 = 0x0B0;
/// `FCT_RX_CTL` — receive FIFO controller.
const REG_FCT_RX_CTL: u16 = 0x0C0;
/// `FCT_TX_CTL` — transmit FIFO controller.
const REG_FCT_TX_CTL: u16 = 0x0C4;
/// `FCT_FLOW` — FIFO controller flow-control thresholds.
const REG_FCT_FLOW: u16 = 0x0D0;
/// `MAC_CR` — MAC configuration.
const REG_MAC_CR: u16 = 0x100;
/// `MAC_RX` — receiver enable and maximum frame size.
const REG_MAC_RX: u16 = 0x104;
/// `MAC_TX` — transmitter enable.
const REG_MAC_TX: u16 = 0x108;
/// `FLOW` — MAC flow control (pause frames).
const REG_FLOW: u16 = 0x10C;
/// `RX_ADDRH` — the station MAC address's high two bytes.
const REG_RX_ADDRH: u16 = 0x118;
/// `RX_ADDRL` — the station MAC address's low four bytes.
const REG_RX_ADDRL: u16 = 0x11C;
/// `MII_ACC` — MII address/command register, driving the PHY.
const REG_MII_ACC: u16 = 0x120;
/// `MII_DATA` — MII data register.
const REG_MII_DATA: u16 = 0x124;
/// `MAF_HI(0)` — high half of perfect address filter entry 0, plus its
/// valid bit.
const REG_MAF_HI0: u16 = 0x400;
/// `MAF_LO(0)` — low half of perfect address filter entry 0.
const REG_MAF_LO0: u16 = 0x404;

/// `HW_CFG.LRST` — device soft reset. Self-clearing: set it, then poll
/// until the chip clears it.
const HW_CFG_LRST: u32 = 0x0000_0002;
/// `HW_CFG.MEF` — Multiple Ethernet Frames per bulk-IN transfer.
///
/// Deliberately left clear, for the reason [`Frames`] gives: turning it on
/// without also programming `BURST_CAP` and `BULK_IN_DLY` stops receive
/// traffic dead rather than degrading it.
const HW_CFG_MEF: u32 = 0x0000_0010;
/// `HW_CFG.CLK125_EN` — enable the 125MHz clock the gigabit path needs.
const HW_CFG_CLK125_EN: u32 = 0x0200_0000;
/// `HW_CFG.REFCLK25_EN` — enable the 25MHz reference clock the internal
/// PHY runs from.
const HW_CFG_REFCLK25_EN: u32 = 0x0100_0000;
/// `HW_CFG.LED0_EN` — drive the chip's first LED pin.
///
/// Off unless something asks for it. Linux takes the answer from the
/// device tree (`lan78xx_configure_leds_from_dt`), clearing every LED bit
/// and setting one per LED the board declares — so with no device tree
/// there is nobody to ask and the jack stays dark whatever the link is
/// doing. Both of a Pi's two are enabled here, since the board has them
/// wired and an unlit socket is the first thing anyone checks.
const HW_CFG_LED0_EN: u32 = 0x0010_0000;
/// `HW_CFG.LED1_EN` — drive the chip's second LED pin; see
/// [`HW_CFG_LED0_EN`].
const HW_CFG_LED1_EN: u32 = 0x0020_0000;

/// `PMT_CTL.PHY_RST` — reset the internal PHY. Self-clearing, like
/// [`HW_CFG_LRST`].
const PMT_CTL_PHY_RST: u32 = 0x0000_0010;
/// `PMT_CTL.READY` — set once the chip is out of reset and answering.
const PMT_CTL_READY: u32 = 0x0000_0080;

/// `USB_CFG0.BIR` — Bulk In Response: how the chip answers a bulk-IN when
/// no frame is waiting. Set means NAK, clear means a zero-length packet.
///
/// **Cleared**, and the receive path depends on it being clear. This is
/// the opposite of [`crate::usb::lan9514`]'s bit of the same name, where
/// setting it is what *selects* the zero-length reply — carrying that
/// polarity across is a mistake that costs nothing at link level and
/// everything above it.
///
/// A NAK is not an answer this driver can poll against. The DWC2 retries
/// a NAK'd bulk channel rather than halting it, so an idle poll blocks for
/// the whole transfer timeout and then reports one — and since that is
/// most polls on a quiet link, the receive path spends its time wedged in
/// doomed transfers and drops the frames that arrive meanwhile. Linux sets
/// this bit because its URBs are asynchronous and a NAK there just means
/// "not yet".
const USB_CFG_BIR: u32 = 0x0000_0040;

/// `RFE_CTL.BCAST_EN` — accept broadcast frames.
const RFE_CTL_BCAST_EN: u32 = 0x0000_0400;
/// `RFE_CTL.DA_PERFECT` — filter unicast against the perfect address
/// table (see [`REG_MAF_HI0`]) rather than accepting everything.
const RFE_CTL_DA_PERFECT: u32 = 0x0000_0002;
/// `RFE_CTL.MCAST_EN` — accept every multicast frame.
const RFE_CTL_MCAST_EN: u32 = 0x0000_0200;
/// `RFE_CTL.UCAST_EN` — accept every unicast frame, whoever it is for
/// (promiscuous).
const RFE_CTL_UCAST_EN: u32 = 0x0000_0100;

/// `FCT_RX_CTL.EN` — enable the receive FIFO controller.
const FCT_RX_CTL_EN: u32 = 0x8000_0000;
/// `FCT_TX_CTL.EN` — enable the transmit FIFO controller.
const FCT_TX_CTL_EN: u32 = 0x8000_0000;

/// `MAC_CR.AUTO_DUPLEX` — take the duplex setting from the PHY's
/// auto-negotiation result rather than from this register.
const MAC_CR_AUTO_DUPLEX: u32 = 0x0000_1000;
/// `MAC_CR.AUTO_SPEED` — take the link speed from the PHY the same way.
const MAC_CR_AUTO_SPEED: u32 = 0x0000_0800;
/// `MAC_CR.EEE_EN` — Energy Efficient Ethernet. Cleared: a link that
/// sleeps between frames is a variable this driver has no way to
/// diagnose around.
const MAC_CR_EEE_EN: u32 = 0x0002_0000;
/// `MAC_CR.GMII_EN` — enable the GMII interface between the MAC and the
/// PHY.
///
/// The LAN7800's PHY is internal and reaches the MAC over GMII, which
/// Linux selects with `PHY_INTERFACE_MODE_GMII` and turns on in
/// `lan78xx_mac_config`. The LAN7801 is the variant this differs for: its
/// PHY is external on RGMII, and that is the case Linux clears this bit
/// for.
const MAC_CR_GMII_EN: u32 = 0x0008_0000;

/// `MAC_RX.RXEN` — enable the receiver.
const MAC_RX_RXEN: u32 = 0x0000_0001;
/// Bit position of `MAC_RX`'s maximum-frame-size field.
const MAC_RX_MAX_SIZE_SHIFT: u32 = 16;
/// Mask of `MAC_RX`'s maximum-frame-size field.
const MAC_RX_MAX_SIZE_MASK: u32 = 0x3FFF_0000;
/// `MAC_TX.TXEN` — enable the transmitter.
const MAC_TX_TXEN: u32 = 0x0000_0001;

/// `MAF_HI.VALID` — marks a perfect-filter entry as one to match against.
const MAF_HI_VALID: u32 = 0x8000_0000;

/// `MII_ACC.MII_BUSY` — set to start an MII access, clears when it is done.
const MII_ACC_BUSY: u32 = 0x0000_0001;
/// Bit position of the PHY address in `MII_ACC`.
const MII_ACC_PHY_ADDR_SHIFT: u32 = 11;
/// `MII_ACC.MII_WRITE` — write rather than read (read is the zero value).
const MII_ACC_WRITE: u32 = 0x0000_0002;
/// Bit position of the PHY register index in `MII_ACC`.
const MII_ACC_REG_SHIFT: u32 = 6;

/// MII address of the LAN7800's internal PHY.
const PHY_ID_INTERNAL: u32 = 1;
/// MII basic-mode control register (BMCR).
const PHY_REG_CONTROL: u8 = 0x00;
/// MII 1000BASE-T control register, which advertises gigabit ability.
const PHY_REG_GIGABIT_CONTROL: u8 = 0x09;
/// MII basic-mode status register (BMSR).
const PHY_REG_STATUS: u8 = 0x01;
/// BMCR bit 12 — auto-negotiation enabled.
const BMCR_ANEG_ENABLE: u16 = 1 << 12;
/// BMCR bit 9 — restart auto-negotiation. Self-clearing.
const BMCR_ANEG_RESTART: u16 = 1 << 9;
/// BMCR bit 11 — PHY powered down.
const BMCR_POWER_DOWN: u16 = 1 << 11;
/// BMCR bit 10 — PHY isolated from the MII interface.
const BMCR_ISOLATE: u16 = 1 << 10;
/// MII auto-negotiation advertisement register.
const PHY_REG_ADVERTISE: u8 = 0x04;
/// MII link-partner ability register.
const PHY_REG_LINK_PARTNER: u8 = 0x05;
/// BMSR bit 2 — link up. Latching low, so a transient drop is reported
/// until the register is read twice; see [`Lan7800::is_link_up`].
const BMSR_LINK_UP: u16 = 1 << 2;
/// Advertisement/partner bit for 100BASE-TX full duplex.
const AN_100_FULL: u16 = 1 << 8;
/// Advertisement/partner bit for 10BASE-T full duplex.
const AN_10_FULL: u16 = 1 << 6;

/// Bytes of header the chip puts in front of each received frame:
/// `RX_CMD_A` and `RX_CMD_B` (four each) then `RX_CMD_C` (two).
const RX_COMMAND_SIZE: usize = 10;
/// Mask of the received frame's length, in `RX_CMD_A`'s low bits. Counts
/// the 4-byte Ethernet CRC, which the caller doesn't want.
const RX_CMD_A_LEN_MASK: u32 = 0x0000_3FFF;
/// Every receive-error bit of `RX_CMD_A` together — any of them and the
/// frame is dropped rather than passed up.
const RX_CMD_A_ERRORS: u32 = 0xC03F_0000;
/// The two bytes the chip counts alongside a frame when working out how
/// much padding follows it. Not part of the frame, and not part of the
/// header this driver skips — purely an offset in the alignment sum; see
/// [`Frames`].
const RX_PADDING: usize = 2;

/// Bytes of header this driver puts in front of each transmitted frame:
/// `TX_CMD_A` and `TX_CMD_B`, four each.
const TX_COMMAND_SIZE: usize = 8;
/// Mask of the transmitted frame's length in `TX_CMD_A`.
const TX_CMD_A_LEN_MASK: u32 = 0x000F_FFFF;
/// `TX_CMD_A.FCS` — have the chip append the Ethernet CRC, so the caller
/// hands over a frame without one.
const TX_CMD_A_FCS: u32 = 0x0040_0000;

/// Largest Ethernet frame this driver carries, in bytes: the 14-byte
/// header plus a 1500-byte payload, without the CRC.
pub const MTU: usize = 1514;

/// The largest frame the receiver is told to accept: [`MTU`] plus the
/// 4-byte CRC and 4 bytes of VLAN tag, so a tagged frame isn't truncated
/// by a filter the caller never asked for.
const MAC_RX_MAX_FRAME_SIZE: u32 = (MTU + 4 + 4) as u32;

/// Size of each DMA frame buffer. A whole max-size frame plus its
/// header, rounded up to a cache line.
const FRAME_BUFFER_SIZE: usize = 2048;

/// How long an MII access may stay busy before it's called stuck, in
/// microseconds.
const MII_TIMEOUT_US: u64 = 1_000_000;

/// How long a self-clearing reset bit may stay set before it's called
/// stuck, in microseconds.
const RESET_TIMEOUT_US: u64 = 1_000_000;

/// A DMA-able frame buffer, aligned and sized as
/// [`Channel::bulk_in`](crate::usb::dwc2::Channel::bulk_in) requires: it
/// invalidates whole cache lines around the transfer, so the buffer has
/// to start on one and occupy a whole number of them.
#[repr(C, align(64))]
struct FrameBuffer([u8; FRAME_BUFFER_SIZE]);

/// One of the chip's bulk data endpoints, with the running data toggle
/// the driver must carry across transfers.
struct BulkEndpoint {
    number: u8,
    max_packet_size: u16,
    toggle: bool,
}

/// The LAN7800's `ID_REV` register, split into the chip ID and silicon
/// revision it packs into one 32-bit word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IdRevision {
    /// Chip ID (the register's high 16 bits) — `0x7800` for this part.
    pub id: u16,
    /// Silicon revision (the register's low 16 bits).
    pub revision: u16,
}

/// The receive half of a [`Lan7800`]: the bulk IN endpoint and the DMA
/// buffer frames land in. Split from the transmit half so the two
/// directions borrow disjointly, as [`crate::usb::lan9514`] is.
struct Rx {
    bulk_in: BulkEndpoint,
    buffer: FrameBuffer,
}

/// The transmit half of a [`Lan7800`] — the counterpart to [`Rx`].
struct Tx {
    bulk_out: BulkEndpoint,
    buffer: FrameBuffer,
}

/// A configured LAN7800 Ethernet controller: its endpoint 0 (for register
/// access) and bulk IN/OUT endpoints (for frame RX/TX), plus DMA frame
/// buffers. Build it with [`Self::from_device`], [`Self::start`] it, then
/// move frames with [`Self::send_frame`]/[`Self::receive_frames`].
pub struct Lan7800 {
    endpoint: ControlEndpoint,
    rx: Rx,
    tx: Tx,
}

/// Picks the bulk IN and OUT endpoints out of a configuration descriptor,
/// with their max packet sizes and a fresh DATA0 toggle. `None` unless
/// both directions are present.
fn find_bulk_endpoints(config: &[u8]) -> Option<(BulkEndpoint, BulkEndpoint)> {
    let mut bulk_in = None;
    let mut bulk_out = None;
    for descriptor in Descriptors::new(config) {
        if let Some(endpoint) = EndpointDescriptor::parse(descriptor) {
            if endpoint.is_bulk() {
                let info = BulkEndpoint {
                    number: endpoint.number(),
                    max_packet_size: endpoint.max_packet_size(),
                    toggle: false,
                };
                if endpoint.is_in() {
                    bulk_in = Some(info);
                } else {
                    bulk_out = Some(info);
                }
            }
        }
    }
    Some((bulk_in?, bulk_out?))
}

/// The received frames one bulk-IN transfer carried, in order.
///
/// # The layout it walks
///
/// Each frame arrives behind a 10-byte header — `RX_CMD_A`, `RX_CMD_B`,
/// `RX_CMD_C` — with the frame's length (CRC included) in `RX_CMD_A`'s
/// low 14 bits. What follows the frame is padding, and the rule is the
/// one oddity here: the chip aligns on the sum of the frame length and
/// two, not on the frame length alone, so the next header
/// starts at `length + 2` rounded up to a multiple of four, less those
/// two again. Getting that wrong doesn't lose one frame, it loses every
/// frame behind it in the transfer, because each header's position
/// depends on the last one's arithmetic.
///
/// # Why this currently only ever yields one frame
///
/// Coalescing is not on: [`Lan7800::start`] leaves `HW_CFG.MEF` clear,
/// so the chip ends a transfer after one frame and this iterator yields
/// one. The loop around it is inert, and kept for the same reason
/// [`crate::usb::lan9514`]'s is — it is the prerequisite for turning
/// coalescing on, and a receive path that structurally cannot return a
/// second frame is how a silent discarding bug gets built.
///
/// **`MEF` alone stops traffic dead.** Setting it makes a transfer
/// complete when `BURST_CAP` is reached rather than after one frame, and
/// `BULK_IN_DLY` decides how long the chip waits before flushing a
/// partial burst. Without both programmed, a lone frame on a quiet link —
/// a DHCP offer, say — sits in the chip waiting for a burst that never
/// comes, and the interface delivers nothing at all. That is what it did
/// on the sibling part, and it is not a failure mode worth rediscovering:
/// anything enabling `MEF` must set `BULK_IN_DLY` with it, and must be
/// tested somewhere the card can be pulled.
///
/// # Trailing bytes it will not guess at
///
/// Iteration stops at the first header that does not describe a plausible
/// frame — one whose length is impossible, or which claims more bytes
/// than the transfer delivered. That is the conservative end to be wrong
/// at: trusting a length and handing the stack whatever followed would
/// turn a padding-rule mistake into corrupt frames rather than into
/// missing ones.
pub struct Frames<'a> {
    /// The transfer's bytes — exactly what arrived, no more.
    buffer: &'a [u8],
    /// Where the next header should be.
    offset: usize,
}

impl<'a> Frames<'a> {
    /// Parses the frames out of one transfer's `buffer`.
    ///
    /// `buffer` must be the bytes the transfer actually delivered, not the
    /// whole receive buffer: the length is what says where the frames
    /// stop.
    pub fn new(buffer: &'a [u8]) -> Self {
        Frames { buffer, offset: 0 }
    }

    /// The raw transfer bytes these frames are parsed out of, for a caller
    /// that has to keep a whole batch rather than consume it in one pass —
    /// copy this, then parse the copy with [`Frames::new`].
    pub fn as_bytes(&self) -> &'a [u8] {
        self.buffer
    }
}

impl<'a> Iterator for Frames<'a> {
    type Item = &'a [u8];

    fn next(&mut self) -> Option<&'a [u8]> {
        loop {
            let remaining = self.buffer.len().checked_sub(self.offset)?;
            if remaining < RX_COMMAND_SIZE {
                return None;
            }

            let command = u32::from_le_bytes([
                self.buffer[self.offset],
                self.buffer[self.offset + 1],
                self.buffer[self.offset + 2],
                self.buffer[self.offset + 3],
            ]);
            // Counts the 4-byte Ethernet CRC, which the caller doesn't
            // want.
            let length = (command & RX_CMD_A_LEN_MASK) as usize;

            // Anything that cannot be a frame ends the iteration rather
            // than being skipped: past this point the offsets are guesses,
            // and a guess produces garbage rather than a gap.
            if length <= 4 || length > MTU + 4 || RX_COMMAND_SIZE + length > remaining {
                self.offset = self.buffer.len();
                return None;
            }

            let start = self.offset + RX_COMMAND_SIZE;
            // Dropping the CRC leaves the frame in `[start, start + length - 4)`.
            let end = start + length - 4;

            // The chip pads so that the frame plus `RX_PADDING` is a whole
            // number of dwords — the padding is computed on that sum, not
            // on the frame length. Stepped over even for a frame being
            // skipped, or every frame behind a bad one would be lost too.
            let padding = (4 - ((length + RX_PADDING) % 4)) % 4;
            self.offset = start + length + padding;

            if command & RX_CMD_A_ERRORS != 0 {
                continue;
            }
            return Some(&self.buffer[start..end]);
        }
    }
}

impl Rx {
    /// The bulk IN endpoint as a [`ControlEndpoint`]: the device's address
    /// and speed with this endpoint's max packet size.
    fn endpoint(&self, device: ControlEndpoint) -> ControlEndpoint {
        ControlEndpoint {
            max_packet_size: self.bulk_in.max_packet_size,
            ..device
        }
    }

    /// The frames the `received` bytes now in [`Self::buffer`] carry.
    fn frames(&self, received: usize) -> Frames<'_> {
        Frames::new(&self.buffer.0[..received.min(FRAME_BUFFER_SIZE)])
    }
}

impl Tx {
    /// The bulk OUT endpoint as a [`ControlEndpoint`] — see
    /// [`Rx::endpoint`].
    fn endpoint(&self, device: ControlEndpoint) -> ControlEndpoint {
        ControlEndpoint {
            max_packet_size: self.bulk_out.max_packet_size,
            ..device
        }
    }

    /// Lays `frame` out in [`Self::buffer`] behind the chip's 8-byte TX
    /// command header, and returns how many bytes of the buffer to send.
    fn stage(&mut self, frame: &[u8]) -> usize {
        debug_assert!(frame.len() <= FRAME_BUFFER_SIZE - TX_COMMAND_SIZE);
        let length = frame.len();

        // Length plus "append the CRC yourself". `TX_CMD_B` carries
        // checksum and segmentation options, none of which are asked for.
        let command_a = (length as u32 & TX_CMD_A_LEN_MASK) | TX_CMD_A_FCS;
        let command_b = 0u32;
        self.buffer.0[0..4].copy_from_slice(&command_a.to_le_bytes());
        self.buffer.0[4..8].copy_from_slice(&command_b.to_le_bytes());
        self.buffer.0[TX_COMMAND_SIZE..TX_COMMAND_SIZE + length].copy_from_slice(frame);
        TX_COMMAND_SIZE + length
    }
}

impl Lan7800 {
    /// Brings `device` up as a LAN7800 if its USB vendor/product ID
    /// ([`VENDOR_ID`]:[`PRODUCT_ID`]) matches, returning `Ok(None)` if it
    /// is something else so a caller can offer each enumerated device to
    /// several drivers in turn.
    ///
    /// Reads the configuration descriptor, locates the bulk IN/OUT pair,
    /// and activates the configuration. Call [`Self::start`] next.
    pub fn from_device(
        channel: &mut Channel,
        timer: &Timer,
        device: Device,
    ) -> Result<Option<Lan7800>, TransferError> {
        if device.descriptor.vendor_id != VENDOR_ID || device.descriptor.product_id != PRODUCT_ID {
            return Ok(None);
        }

        let mut config = [0u8; 64];
        let len = get_configuration_descriptor(channel, timer, device.endpoint, 0, &mut config)?;
        let Some(config_value) = ConfigurationDescriptor::parse(&config[..len]).map(|c| c.value())
        else {
            return Ok(None);
        };
        let Some((bulk_in, bulk_out)) = find_bulk_endpoints(&config[..len]) else {
            return Ok(None);
        };
        set_configuration(channel, timer, device.endpoint, config_value)?;

        Ok(Some(Lan7800::new(device.endpoint, bulk_in, bulk_out)))
    }

    /// Brings up a LAN7800 that something *else* has already addressed and
    /// configured, given its endpoint 0 — the counterpart to
    /// [`Self::from_device`] for a caller driving the bus itself.
    ///
    /// Only the configuration descriptor is read, and only to locate the
    /// bulk endpoints; the configuration is *not* re-activated, since
    /// re-issuing SET_CONFIGURATION would reset the device's endpoints out
    /// from under whoever configured it. No vendor/product check: a caller
    /// reaching for this has already identified the device.
    pub fn from_endpoint(
        channel: &mut Channel,
        timer: &Timer,
        endpoint: ControlEndpoint,
    ) -> Result<Option<Lan7800>, TransferError> {
        let mut config = [0u8; 64];
        let len = get_configuration_descriptor(channel, timer, endpoint, 0, &mut config)?;
        let Some((bulk_in, bulk_out)) = find_bulk_endpoints(&config[..len]) else {
            return Ok(None);
        };
        Ok(Some(Lan7800::new(endpoint, bulk_in, bulk_out)))
    }

    /// Assembles the driver around an endpoint and its bulk pair.
    fn new(endpoint: ControlEndpoint, bulk_in: BulkEndpoint, bulk_out: BulkEndpoint) -> Lan7800 {
        Lan7800 {
            endpoint,
            rx: Rx {
                bulk_in,
                buffer: FrameBuffer([0; FRAME_BUFFER_SIZE]),
            },
            tx: Tx {
                bulk_out,
                buffer: FrameBuffer([0; FRAME_BUFFER_SIZE]),
            },
        }
    }

    /// Reads one of the chip's 32-bit registers by offset.
    ///
    /// A vendor control-IN carrying the offset in `wIndex` and the value
    /// little-endian in a 4-byte data stage.
    pub fn read_register(
        &self,
        channel: &mut Channel,
        timer: &Timer,
        register: u16,
    ) -> Result<u32, TransferError> {
        let mut value = [0u8; 4];
        vendor_in(
            channel,
            timer,
            self.endpoint,
            READ_REGISTER,
            0,
            register,
            &mut value,
        )?;
        Ok(u32::from_le_bytes(value))
    }

    /// Writes one of the chip's 32-bit registers by offset — the mirror of
    /// [`Self::read_register`].
    pub fn write_register(
        &self,
        channel: &mut Channel,
        timer: &Timer,
        register: u16,
        value: u32,
    ) -> Result<(), TransferError> {
        vendor_out(
            channel,
            timer,
            self.endpoint,
            WRITE_REGISTER,
            0,
            register,
            &value.to_le_bytes(),
        )
    }

    /// Reads the chip's `ID_REV`. The ID is `0x7800` on a working part,
    /// which makes this the cheapest confirmation that register access is
    /// reaching the chip at all.
    pub fn id_revision(
        &self,
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<IdRevision, TransferError> {
        let id_rev = self.read_register(channel, timer, REG_ID_REV)?;
        Ok(IdRevision {
            id: (id_rev >> 16) as u16,
            revision: id_rev as u16,
        })
    }

    /// Sets the station MAC address: the address the receiver answers to,
    /// and entry 0 of the perfect address filter that
    /// `RFE_CTL.DA_PERFECT` matches against.
    ///
    /// Both have to be written. `RX_ADDR*` is what the MAC calls itself;
    /// the filter table is what actually decides which frames get through,
    /// and an unset entry 0 filters out the very frames addressed to this
    /// interface.
    pub fn set_mac_address(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        mac: [u8; 6],
    ) -> Result<(), TransferError> {
        let low = u32::from_le_bytes([mac[0], mac[1], mac[2], mac[3]]);
        let high = u16::from_le_bytes([mac[4], mac[5]]) as u32;

        self.write_register(channel, timer, REG_RX_ADDRL, low)?;
        self.write_register(channel, timer, REG_RX_ADDRH, high)?;

        self.write_register(channel, timer, REG_MAF_LO0, low)?;
        self.write_register(channel, timer, REG_MAF_HI0, high | MAF_HI_VALID)?;
        Ok(())
    }

    /// Reads back the station MAC address the chip is using.
    pub fn mac_address(
        &self,
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<[u8; 6], TransferError> {
        let low = self
            .read_register(channel, timer, REG_RX_ADDRL)?
            .to_le_bytes();
        let high = (self.read_register(channel, timer, REG_RX_ADDRH)? as u16).to_le_bytes();
        Ok([low[0], low[1], low[2], low[3], high[0], high[1]])
    }

    /// Resets the chip and its PHY, and puts the receive filter, clocks
    /// and MAC into a known state — everything [`Self::start`] does except
    /// setting the MAC address and opening the data paths.
    ///
    /// Both resets here are self-clearing bits that the chip takes real
    /// time over, so each is polled rather than waited out blindly, and
    /// each is bounded so a chip that never comes back reports
    /// [`TransferError::Timeout`] instead of hanging the caller.
    pub fn reset(&mut self, channel: &mut Channel, timer: &Timer) -> Result<(), TransferError> {
        let hw_cfg = self.read_register(channel, timer, REG_HW_CFG)?;
        self.write_register(channel, timer, REG_HW_CFG, hw_cfg | HW_CFG_LRST)?;
        self.wait_clear(channel, timer, REG_HW_CFG, HW_CFG_LRST)?;

        // A zero-length bulk-IN rather than a NAK when nothing is waiting;
        // the receive path is built on this (see `USB_CFG_BIR`).
        let usb_cfg = self.read_register(channel, timer, REG_USB_CFG0)?;
        self.write_register(channel, timer, REG_USB_CFG0, usb_cfg & !USB_CFG_BIR)?;

        // The internal PHY's reference clock, the gigabit path's clock,
        // and the two LED pins the board wires to the socket. `MEF` stays
        // clear — see `Frames`.
        let hw_cfg = self.read_register(channel, timer, REG_HW_CFG)?;
        self.write_register(
            channel,
            timer,
            REG_HW_CFG,
            (hw_cfg | HW_CFG_CLK125_EN | HW_CFG_REFCLK25_EN | HW_CFG_LED0_EN | HW_CFG_LED1_EN)
                & !HW_CFG_MEF,
        )?;

        self.write_register(channel, timer, REG_INT_STS, u32::MAX)?;

        // No pause frames in either direction. Flow control needs a peer
        // that honours it and a policy for when to assert it; without
        // both, an enabled pause is a way to stall a link rather than to
        // protect it.
        self.write_register(channel, timer, REG_FLOW, 0)?;
        self.write_register(channel, timer, REG_FCT_FLOW, 0)?;

        // Broadcast plus perfect-matched unicast, which is what an
        // interface needs before it knows anything about its own traffic.
        // Multicast is off and that is not neutral — see
        // `set_all_multicast`.
        self.write_register(
            channel,
            timer,
            REG_RFE_CTL,
            RFE_CTL_BCAST_EN | RFE_CTL_DA_PERFECT,
        )?;

        let pmt_ctl = self.read_register(channel, timer, REG_PMT_CTL)?;
        self.write_register(channel, timer, REG_PMT_CTL, pmt_ctl | PMT_CTL_PHY_RST)?;
        self.wait_phy_ready(channel, timer)?;

        // Let the MAC follow the PHY rather than being told the answer.
        // Speed and duplex are auto-negotiated and can change under a live
        // link; having the MAC track the PHY's result means there is one
        // place that knows, instead of a register this driver has to
        // remember to rewrite every time the link comes back.
        let mac_cr = self.read_register(channel, timer, REG_MAC_CR)?;
        self.write_register(
            channel,
            timer,
            REG_MAC_CR,
            (mac_cr | MAC_CR_AUTO_DUPLEX | MAC_CR_AUTO_SPEED | MAC_CR_GMII_EN) & !MAC_CR_EEE_EN,
        )?;

        let mac_rx = self.read_register(channel, timer, REG_MAC_RX)?;
        self.write_register(
            channel,
            timer,
            REG_MAC_RX,
            (mac_rx & !MAC_RX_MAX_SIZE_MASK)
                | ((MAC_RX_MAX_FRAME_SIZE << MAC_RX_MAX_SIZE_SHIFT) & MAC_RX_MAX_SIZE_MASK),
        )?;

        self.start_autonegotiation(channel, timer)?;
        Ok(())
    }

    /// Puts the PHY into service and starts auto-negotiation: gigabit
    /// withdrawn from what it advertises, out of power-down and out of
    /// isolate, negotiation enabled, and restarted.
    ///
    /// # Why gigabit is withdrawn
    ///
    /// **The link does not come up at all if it is left advertised**, and
    /// the failure is worth writing down because every visible part of it
    /// looks healthy. Base pages exchange correctly and the partner
    /// acknowledges them (`ANLPAR` comes back with bit 14 set); there is no
    /// parallel-detection fault and no master/slave configuration fault.
    /// What never happens is 1000BASE-T *training*: the remote-receiver-OK
    /// bit of the 1000BASE-T status register never sets, while the local
    /// one and the partner's gigabit ability flicker on and off, and
    /// auto-negotiation restarts and fails again indefinitely. It never
    /// falls back to 100BASE-TX either, because both ends keep agreeing on
    /// gigabit and then failing to train at it. From outside, the symptom
    /// is a link that is simply never up and a socket with no lights.
    ///
    /// Training needs analogue setup particular to this PHY that this
    /// driver does not do. Linux gets gigabit because phylib binds a
    /// dedicated driver to it — `drivers/net/phy/microchip.c`, for the
    /// LAN88xx core inside the LAN7800 — which programs DSP and MDIX
    /// registers that `lan78xx.c` itself never mentions. Porting that is
    /// its own piece of work.
    ///
    /// The cost of going without is smaller than it looks: this chip
    /// reaches the host over USB 2.0, whose 480 Mbit/s ceiling (nearer 300
    /// in practice) a gigabit line could not fill anyway. A 100BASE-TX
    /// full-duplex link that works beats a gigabit one that doesn't.
    fn start_autonegotiation(
        &self,
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<(), TransferError> {
        self.phy_write(channel, timer, PHY_REG_GIGABIT_CONTROL, 0)?;

        let control = self.phy_read(channel, timer, PHY_REG_CONTROL)?;
        let control =
            (control | BMCR_ANEG_ENABLE | BMCR_ANEG_RESTART) & !(BMCR_POWER_DOWN | BMCR_ISOLATE);
        self.phy_write(channel, timer, PHY_REG_CONTROL, control)
    }

    /// Resets the chip, gives it `mac`, and opens the data paths — the one
    /// call between [`Self::from_device`] and moving frames.
    ///
    /// The FIFO controllers and the MAC's two halves are enabled last and
    /// in that order: a MAC transmitting into a FIFO controller that isn't
    /// running has nowhere to put the frame.
    pub fn start(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        mac: [u8; 6],
    ) -> Result<(), TransferError> {
        self.reset(channel, timer)?;
        self.set_mac_address(channel, timer, mac)?;

        let fct_rx = self.read_register(channel, timer, REG_FCT_RX_CTL)?;
        self.write_register(channel, timer, REG_FCT_RX_CTL, fct_rx | FCT_RX_CTL_EN)?;
        let fct_tx = self.read_register(channel, timer, REG_FCT_TX_CTL)?;
        self.write_register(channel, timer, REG_FCT_TX_CTL, fct_tx | FCT_TX_CTL_EN)?;

        let mac_rx = self.read_register(channel, timer, REG_MAC_RX)?;
        self.write_register(channel, timer, REG_MAC_RX, mac_rx | MAC_RX_RXEN)?;
        let mac_tx = self.read_register(channel, timer, REG_MAC_TX)?;
        self.write_register(channel, timer, REG_MAC_TX, mac_tx | MAC_TX_TXEN)?;
        Ok(())
    }

    /// The bulk-IN (frame receive) endpoint's max packet size.
    pub fn bulk_in_max_packet_size(&self) -> u16 {
        self.rx.bulk_in.max_packet_size
    }

    /// The bulk-OUT (frame transmit) endpoint's max packet size.
    pub fn bulk_out_max_packet_size(&self) -> u16 {
        self.tx.bulk_out.max_packet_size
    }

    /// Whether the Ethernet link is up, read from the PHY's basic-mode
    /// status register over MII. `false` until the cable is connected and
    /// auto-negotiation completes.
    ///
    /// The link bit latches low, so a link that dropped and recovered
    /// between two calls still reads down on the first of them. A caller
    /// that cares about the present rather than the interval reads it
    /// twice.
    pub fn is_link_up(&self, channel: &mut Channel, timer: &Timer) -> Result<bool, TransferError> {
        Ok(self.phy_read(channel, timer, PHY_REG_STATUS)? & BMSR_LINK_UP != 0)
    }

    /// Whether auto-negotiation settled on full duplex — the highest
    /// common denominator of what this PHY advertised and what the partner
    /// did. Only meaningful once [`Self::is_link_up`] is `true`.
    ///
    /// Informational here, unlike on the sibling part: the MAC takes its
    /// duplex from the PHY directly (`MAC_CR.AUTO_DUPLEX`), so
    /// nothing has to act on this.
    pub fn is_full_duplex(
        &self,
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<bool, TransferError> {
        let ours = self.phy_read(channel, timer, PHY_REG_ADVERTISE)?;
        let theirs = self.phy_read(channel, timer, PHY_REG_LINK_PARTNER)?;
        Ok(ours & theirs & (AN_100_FULL | AN_10_FULL) != 0)
    }

    /// Whether to pass every multicast frame up to the host, or filter it.
    ///
    /// **Off is the chip's reset state, and it is not a neutral default.**
    /// The receiver comes up accepting unicast for its own address plus
    /// broadcast, and dropping multicast before the host ever sees it.
    /// Anything speaking only unicast or broadcast never notices — DHCP is
    /// broadcast — and anything else fails completely rather than
    /// partially: mDNS queries and announcements are multicast, so a
    /// responder binds a socket nothing ever arrives on, with no error to
    /// explain it.
    ///
    /// Passing all of it is the blunt option. The chip also offers a hash
    /// filter, which is a table to keep in step with the stack's group
    /// memberships; for a handful of groups, the traffic that gets past a
    /// pass-all filter only to be dropped by the stack is a few packets a
    /// second on a busy link, and the table is a second place for the
    /// membership list to be wrong.
    ///
    /// A read-modify-write, so it can be called after [`Self::start`] has
    /// opened the data paths without closing them again.
    pub fn set_all_multicast(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        pass: bool,
    ) -> Result<(), TransferError> {
        let rfe_ctl = self.read_register(channel, timer, REG_RFE_CTL)?;
        let rfe_ctl = if pass {
            rfe_ctl | RFE_CTL_MCAST_EN
        } else {
            rfe_ctl & !RFE_CTL_MCAST_EN
        };
        self.write_register(channel, timer, REG_RFE_CTL, rfe_ctl)
    }

    /// Whether to accept every unicast frame regardless of its
    /// destination, rather than only those matching this interface's
    /// address. Off by default, and what a bridge or a capture needs.
    pub fn set_promiscuous(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        pass: bool,
    ) -> Result<(), TransferError> {
        let rfe_ctl = self.read_register(channel, timer, REG_RFE_CTL)?;
        let rfe_ctl = if pass {
            rfe_ctl | RFE_CTL_UCAST_EN
        } else {
            rfe_ctl & !RFE_CTL_UCAST_EN
        };
        self.write_register(channel, timer, REG_RFE_CTL, rfe_ctl)
    }

    /// Sends one Ethernet frame (destination MAC through payload, without
    /// the CRC — the chip appends it). Prepends the chip's 8-byte TX
    /// command header and bulk-OUTs the lot. `frame` must be no larger
    /// than [`MTU`].
    pub fn send_frame(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        frame: &[u8],
    ) -> Result<(), TransferError> {
        let staged = self.tx.stage(frame);
        let endpoint = self.tx.endpoint(self.endpoint);
        let number = self.tx.bulk_out.number;
        channel.bulk_out(
            endpoint,
            number,
            &mut self.tx.bulk_out.toggle,
            &self.tx.buffer.0[..staged],
            timer,
        )?;
        Ok(())
    }

    /// Polls for received Ethernet frames, returning every one the
    /// transfer carried (destination MAC through payload, CRC stripped).
    /// The frames borrow this driver's RX buffer until the next call.
    ///
    /// An empty iterator means there was nothing to receive — the chip
    /// answers an empty receive FIFO with a zero-length packet rather than
    /// a NAK, which is what a cleared `USB_CFG0.BIR` buys and why this can be
    /// polled at all.
    ///
    /// **Drain it.** A transfer can carry several frames and the next call
    /// overwrites the buffer, so a caller that takes the first and calls
    /// again silently discards the others; see [`Frames`].
    pub fn receive_frames(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<Frames<'_>, TransferError> {
        let endpoint = self.rx.endpoint(self.endpoint);
        let number = self.rx.bulk_in.number;
        let received = match channel.bulk_in(
            endpoint,
            number,
            &mut self.rx.bulk_in.toggle,
            &mut self.rx.buffer.0,
            timer,
        ) {
            Ok(received) => received,
            // No frame waiting.
            Err(TransferError::Nak) => 0,
            Err(error) => return Err(error),
        };

        Ok(self.rx.frames(received))
    }

    /// Reads MII (PHY) register `index` of the internal PHY: point
    /// `MII_ACC` at it with the busy bit set, wait for busy to clear, then
    /// read `MII_DATA`.
    ///
    /// Public because a PHY that isn't answering is otherwise
    /// indistinguishable from a link that is down — both make
    /// [`Self::is_link_up`] return `false` forever, and neither raises an
    /// error, since `MII_ACC`'s busy bit clears whether or not anything
    /// replied. Registers 2 and 3 hold the PHY's identifier and are the
    /// cheap way to tell those apart.
    pub fn phy_read(
        &self,
        channel: &mut Channel,
        timer: &Timer,
        index: u8,
    ) -> Result<u16, TransferError> {
        self.phy_wait_not_busy(channel, timer)?;
        let access = (PHY_ID_INTERNAL << MII_ACC_PHY_ADDR_SHIFT)
            | ((index as u32) << MII_ACC_REG_SHIFT)
            | MII_ACC_BUSY;
        self.write_register(channel, timer, REG_MII_ACC, access)?;
        self.phy_wait_not_busy(channel, timer)?;
        Ok(self.read_register(channel, timer, REG_MII_DATA)? as u16)
    }

    /// Writes MII (PHY) register `index`. Unlike a read, the data goes in
    /// first: `MII_ACC`'s busy bit is what starts the access, so anything
    /// written after it is too late for this one.
    pub fn phy_write(
        &self,
        channel: &mut Channel,
        timer: &Timer,
        index: u8,
        value: u16,
    ) -> Result<(), TransferError> {
        self.phy_wait_not_busy(channel, timer)?;
        self.write_register(channel, timer, REG_MII_DATA, value as u32)?;
        let access = (PHY_ID_INTERNAL << MII_ACC_PHY_ADDR_SHIFT)
            | ((index as u32) << MII_ACC_REG_SHIFT)
            | MII_ACC_WRITE
            | MII_ACC_BUSY;
        self.write_register(channel, timer, REG_MII_ACC, access)?;
        self.phy_wait_not_busy(channel, timer)
    }

    /// Spins until the MII interface's busy bit clears, bounded by
    /// [`MII_TIMEOUT_US`] so a stuck PHY access can't wedge the caller.
    fn phy_wait_not_busy(&self, channel: &mut Channel, timer: &Timer) -> Result<(), TransferError> {
        let start = timer.now_micros();
        while self.read_register(channel, timer, REG_MII_ACC)? & MII_ACC_BUSY != 0 {
            if timer.now_micros() - start > MII_TIMEOUT_US {
                return Err(TransferError::Timeout);
            }
        }
        Ok(())
    }

    /// Spins until `bits` clear in `register`, bounded by
    /// [`RESET_TIMEOUT_US`] — the shape every self-clearing reset bit here
    /// is polled with.
    fn wait_clear(
        &self,
        channel: &mut Channel,
        timer: &Timer,
        register: u16,
        bits: u32,
    ) -> Result<(), TransferError> {
        let start = timer.now_micros();
        while self.read_register(channel, timer, register)? & bits != 0 {
            if timer.now_micros() - start > RESET_TIMEOUT_US {
                return Err(TransferError::Timeout);
            }
        }
        Ok(())
    }

    /// Waits for the PHY reset to finish: its bit self-clears *and* the
    /// chip raises `PMT_CTL.READY`. Both, because the reset bit clearing
    /// only says the pulse is over, while `READY` is the chip saying it
    /// will answer again.
    fn wait_phy_ready(&self, channel: &mut Channel, timer: &Timer) -> Result<(), TransferError> {
        let start = timer.now_micros();
        loop {
            let pmt_ctl = self.read_register(channel, timer, REG_PMT_CTL)?;
            if pmt_ctl & PMT_CTL_PHY_RST == 0 && pmt_ctl & PMT_CTL_READY != 0 {
                return Ok(());
            }
            if timer.now_micros() - start > RESET_TIMEOUT_US {
                return Err(TransferError::Timeout);
            }
        }
    }
}
