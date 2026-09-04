//! SMSC/Microchip LAN9514 USB-Ethernet controller support.
//!
//! On this project's target board (a Pi 2 Model B) the on-board Ethernet
//! jack is wired through a LAN9514 — a combined USB hub and 10/100
//! Ethernet controller — that hangs off the DWC2 root port. The hub half
//! is what [`crate::usb::enumerate`] brings up; the Ethernet half appears
//! as a separate vendor-specific device on one of the hub's ports (USB ID
//! [`VENDOR_ID`](crate::usb::lan9514::VENDOR_ID):[`PRODUCT_ID`](crate::usb::lan9514::PRODUCT_ID)),
//! enumerated and addressed like any other.
//!
//! [`Lan9514::from_device`](crate::usb::lan9514::Lan9514::from_device)
//! configures the device and finds its bulk endpoints;
//! [`start`](crate::usb::lan9514::Lan9514::start) programs the MAC and
//! enables the receiver/transmitter; then
//! [`receive_frames`](crate::usb::lan9514::Lan9514::receive_frames) and
//! [`send_frame`](crate::usb::lan9514::Lan9514::send_frame) move Ethernet
//! frames over the bulk endpoints (each frame carries the chip's TX
//! command header / RX status word, handled here — and a single receive
//! transfer carries as many frames as the chip had waiting, which is why
//! that one is plural; see [`Frames`](crate::usb::lan9514::Frames)). Register access is
//! over vendor control transfers, and the MII PHY is reachable for link
//! status ([`is_link_up`](crate::usb::lan9514::Lan9514::is_link_up)).
//!
//! Every method above has an interrupt-driven `async` twin under the
//! `async` feature —
//! [`send_frame_async`](crate::usb::lan9514::Lan9514::send_frame_async),
//! [`receive_frames_async`](crate::usb::lan9514::Lan9514::receive_frames_async)
//! and the register accessors behind them — so a driver running under an
//! executor gives the CPU back for the time a transfer spends on the bus
//! instead of busy-waiting on it. The blocking methods are not deprecated
//! by them: `smoltcp`'s `phy::Device` is synchronous by construction, so
//! the adapter below could not use the async ones even if it wanted to.
//! [`split`](crate::usb::lan9514::Lan9514::split) hands out the two
//! directions separately, for a caller that wants a receive parked on one
//! host channel while transmits go out on another.
//!
//! This driver only moves raw frames; a TCP/IP stack goes on top as
//! application code. With the `smoltcp` feature enabled,
//! [`Lan9514Phy`](crate::usb::lan9514::Lan9514Phy) wraps
//! [`send_frame`](crate::usb::lan9514::Lan9514::send_frame) /
//! [`receive_frames`](crate::usb::lan9514::Lan9514::receive_frames) as a
//! `smoltcp` `phy::Device` so a stack can run on top; the
//! `usb_ethernet_smoltcp` example uses it, leasing an address over DHCP
//! and running a poll loop that answers pings. For `embassy-net`, the
//! `rpi-hal-embassy` crate builds its adapter on
//! [`split`](crate::usb::lan9514::Lan9514::split) and the async methods.
//!
//! Register offsets, framing, and the bring-up sequence here follow
//! rsta2's `circle` `smsc951x` driver (in turn from the Linux `smsc95xx`
//! driver) for this same chip.

use crate::timer::Timer;
use crate::usb::control::{get_configuration_descriptor, set_configuration, vendor_in, vendor_out};
use crate::usb::descriptor::{ConfigurationDescriptor, Descriptors, EndpointDescriptor};
use crate::usb::dwc2::{Channel, ControlEndpoint, TransferError};
use crate::usb::Device;

/// The `async` twins of this driver's transfer methods.
#[cfg(feature = "async")]
mod asynch;

#[cfg(feature = "async")]
pub use asynch::{Lan9514Rx, Lan9514Tx};

/// USB vendor ID of the LAN9514 (SMSC/Microchip).
pub const VENDOR_ID: u16 = 0x0424;
/// USB product ID of the LAN9514's Ethernet function on this board.
pub const PRODUCT_ID: u16 = 0xEC00;

/// Vendor `bRequest` code that reads a device register (the register
/// offset goes in the request's `wIndex`).
const READ_REGISTER: u8 = 0xA1;
/// Vendor `bRequest` code that writes a device register.
const WRITE_REGISTER: u8 = 0xA0;

/// `ID_REV` register — chip ID (high half) and silicon revision (low).
const REG_ID_REV: u16 = 0x00;
/// `TX_CFG` register — transmitter configuration.
const REG_TX_CFG: u16 = 0x10;
/// `HW_CFG` register — hardware configuration.
const REG_HW_CFG: u16 = 0x14;
/// `RX_FIFO_INF` register — RX FIFO fill info; nonzero when the chip has
/// a received frame buffered (its used-space fields are all zero when the
/// receive FIFO is empty).
const REG_RX_FIFO_INF: u16 = 0x18;
/// `LED_GPIO_CFG` register — LED/GPIO pin configuration.
const REG_LED_GPIO_CFG: u16 = 0x24;
/// `MAC_CR` register — MAC control (RX/TX enables).
const REG_MAC_CR: u16 = 0x100;
/// `ADDRH` register — high 16 bits of the MAC address (bytes 4–5).
const REG_ADDRH: u16 = 0x104;
/// `ADDRL` register — low 32 bits of the MAC address (bytes 0–3).
const REG_ADDRL: u16 = 0x108;
/// `MII_ADDR` register — MII (PHY) access: PHY/register select + busy.
const REG_MII_ADDR: u16 = 0x114;
/// `MII_DATA` register — MII (PHY) read/write data.
const REG_MII_DATA: u16 = 0x118;

/// `HW_CFG.BIR` — make a bulk-IN with no frame available return a
/// zero-length packet instead of NAKing, so RX polling is deterministic.
const HW_CFG_BIR: u32 = 0x0000_1000;

/// `HW_CFG.RXDOFF` — bytes of padding the chip puts between a frame's RX
/// status word and the frame itself.
///
/// Cleared explicitly. [`Frames`] reads a frame as starting immediately
/// after its status word, and the next status word as dword-aligned from
/// there; a non-zero offset shifts every frame in a transfer. Zero is the
/// reset default, so this says the parser depends on it rather than
/// changing anything.
const HW_CFG_RXDOFF: u32 = 0x0000_0600;

/// `LED_GPIO_CFG` bit routing the speed LED to its pin.
const LED_GPIO_CFG_SPD_LED: u32 = 0x0100_0000;
/// `LED_GPIO_CFG` bit routing the link LED to its pin.
const LED_GPIO_CFG_LNK_LED: u32 = 0x0010_0000;
/// `LED_GPIO_CFG` bit routing the full-duplex LED to its pin.
const LED_GPIO_CFG_FDX_LED: u32 = 0x0001_0000;
/// `MAC_CR.RCVOWN` — receive own transmissions. Half duplex only; in full
/// duplex it must be clear, and [`MAC_CR_FDPX`] set instead.
const MAC_CR_RCVOWN: u32 = 0x0080_0000;

/// `MAC_CR.FDPX` — put the MAC in full duplex.
///
/// **Leaving this clear is expensive and does not look like a driver
/// fault.** A clear `FDPX` is a half-duplex MAC, which means CSMA/CD: the
/// MAC treats transmitting while receiving as a collision and discards the
/// frame. On a link the PHY has auto-negotiated as full duplex — which is
/// every switch — that loses frames precisely when traffic runs in both
/// directions at once, and only then.
///
/// What that cost while it was clear here: a page of eight files fetched
/// over eight concurrent connections stalled on 62% of requests, with a
/// median response time of 1.0 s against 3.9 ms for the same file fetched
/// one at a time. The stalls sat exactly on the peer's retransmission
/// timeout, and every layer above reported success — the driver had handed
/// each frame over, so its own send counter recorded no failure, and the
/// receive counters showed nothing wrong either. Loss inside the MAC is
/// invisible from both sides of it, which is what made it expensive to
/// find rather than expensive to fix.
///
/// [`Lan9514::set_duplex`] programs this from the negotiated duplex;
/// [`Lan9514::start`] assumes full, because half duplex needs a hub rather
/// than a switch.
const MAC_CR_FDPX: u32 = 0x0010_0000;
/// `MAC_CR.MCPAS` — pass every multicast frame up to the host instead of
/// filtering it.
///
/// The chip comes up filtering to unicast-for-this-MAC plus broadcast, and
/// drops multicast before it ever reaches the host. That is invisible to
/// anything speaking only unicast or broadcast — DHCP is broadcast — and
/// then total for anything that is not: every mDNS query and announcement
/// is multicast, so a responder binds a socket that nothing arrives on.
const MAC_CR_MCPAS: u32 = 0x0008_0000;
/// `MAC_CR.TXEN` — enable the transmitter.
const MAC_CR_TXEN: u32 = 0x0000_0008;
/// `MAC_CR.RXEN` — enable the receiver.
const MAC_CR_RXEN: u32 = 0x0000_0004;
/// `TX_CFG.ON` — turn the transmitter on.
const TX_CFG_ON: u32 = 0x0000_0004;

/// `MII_ADDR.BUSY` — set to start an MII access, clears when it's done.
const MII_BUSY: u32 = 0x01;
/// The internal 10/100 PHY's MII address.
const PHY_ID_INTERNAL: u32 = 0x01;
/// MII register 1, the PHY's basic-mode status register (`BMSR`).
const PHY_REG_STATUS: u8 = 0x01;
/// MII register 4 — what this PHY advertised in auto-negotiation.
const PHY_REG_ADVERTISE: u8 = 0x04;
/// MII register 5 — what the link partner advertised.
const PHY_REG_LINK_PARTNER: u8 = 0x05;
/// Auto-negotiation ability bit for 100BASE-TX full duplex.
const AN_100_FULL: u16 = 1 << 8;
/// Auto-negotiation ability bit for 10BASE-T full duplex.
const AN_10_FULL: u16 = 1 << 6;
/// `BMSR` bit 2 — link is up.
const BMSR_LINK_UP: u16 = 1 << 2;

/// `TX_CMD_A.FIRST_SEG` — this buffer holds the start of the frame.
const TX_CMD_A_FIRST_SEG: u32 = 0x0000_2000;
/// `TX_CMD_A.LAST_SEG` — this buffer holds the end of the frame.
const TX_CMD_A_LAST_SEG: u32 = 0x0000_1000;

/// `RX_STS` error-summary bit — the frame had a receive error.
const RX_STATUS_ERROR: u32 = 0x0000_8000;
/// Bytes of `RX_STS` in front of every received packet.
///
/// Also the boundary the next one is aligned to, which is why it is one
/// constant rather than two that happen to be equal.
const RX_STATUS_SIZE: usize = 4;
/// `RX_STS` frame-length field (bits 16–29), including the 4-byte CRC.
const RX_STATUS_FRAME_LENGTH: u32 = 0x3FFF_0000;

/// Bytes reserved for a frame buffer — comfortably over a max Ethernet
/// frame plus the chip's framing words, and a whole number of both cache
/// lines and 512-byte bulk max packets (`2048 = 32 × 64 = 4 × 512`) so a
/// bulk-IN transfer fills it exactly with no rounding waste (see
/// [`Channel::bulk_in`]).
const FRAME_BUFFER_SIZE: usize = 2048;

/// Timeout for an MII (PHY) access to complete, in microseconds.
const MII_TIMEOUT_US: u64 = 1_000_000;

/// A frame buffer sized and aligned for bulk DMA — cache-line aligned and
/// a whole number of cache lines, as [`Channel::bulk_in`] requires.
#[repr(C, align(64))]
struct FrameBuffer([u8; FRAME_BUFFER_SIZE]);

/// One of the chip's bulk data endpoints, with the running data toggle
/// the driver must carry across transfers.
struct BulkEndpoint {
    number: u8,
    max_packet_size: u16,
    toggle: bool,
}

/// The LAN9514's `ID_REV` register, split into the chip ID and silicon
/// revision it packs into one 32-bit word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IdRevision {
    /// Chip ID (the register's high 16 bits) — `0xEC00` for the LAN951x
    /// family.
    pub id: u16,
    /// Silicon revision (the register's low 16 bits).
    pub revision: u16,
}

/// The receive half of a [`Lan9514`]: the bulk IN endpoint and the DMA
/// buffer frames land in.
///
/// Split from the transmit half so the two directions borrow disjointly.
/// Nothing in the blocking API needs that, but the `embassy-net` runner
/// does: it keeps a receive parked on one host channel while transmits go
/// out on another, which is only expressible if the two futures don't
/// both want `&mut Lan9514`.
struct Rx {
    bulk_in: BulkEndpoint,
    buffer: FrameBuffer,
}

/// The transmit half of a [`Lan9514`] — the counterpart to [`Rx`], and
/// split out for the same reason.
struct Tx {
    bulk_out: BulkEndpoint,
    buffer: FrameBuffer,
}

/// A configured LAN9514 Ethernet function: its endpoint 0 (for register
/// access) and bulk IN/OUT endpoints (for frame RX/TX), plus DMA frame
/// buffers. Build it with [`Self::from_device`], [`Self::start`] it, then
/// move frames with [`Self::send_frame`]/[`Self::receive_frames`].
pub struct Lan9514 {
    endpoint: ControlEndpoint,
    rx: Rx,
    tx: Tx,
}

/// Picks the bulk IN and OUT endpoints out of a configuration
/// descriptor, with their max packet sizes and a fresh DATA0 toggle.
/// `None` unless both directions are present — a LAN9514 missing one of
/// them isn't one this driver can move frames through.
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

impl Rx {
    /// The bulk IN endpoint as a [`ControlEndpoint`]: the device's
    /// address and speed with this endpoint's max packet size.
    fn endpoint(&self, device: ControlEndpoint) -> ControlEndpoint {
        ControlEndpoint {
            max_packet_size: self.bulk_in.max_packet_size,
            ..device
        }
    }

    /// The frames the `received` bytes now in [`Self::buffer`] carry.
    ///
    /// **Plural.** One bulk IN can arrive holding several; see [`Frames`].
    fn frames(&self, received: usize) -> Frames<'_> {
        Frames::new(&self.buffer.0[..received.min(FRAME_BUFFER_SIZE)])
    }
}

/// The received frames one bulk-IN transfer carried, in order.
///
/// # Why a transfer carries more than one
///
/// The chip packs as many received packets as will fit into a single
/// transfer, each behind its own 4-byte RX status word and padded so that
/// the next status word starts on a 4-byte boundary. It is not an
/// optimisation the host asks for — it is simply what arrives when more
/// than one packet is waiting in the RX FIFO, and the receive buffer is
/// deliberately large enough to take a whole max-size frame — which means
/// it is large enough to take *dozens* of small ones. A bare TCP
/// acknowledgement is 54 bytes, plus 4 of CRC and 4 of status word and
/// padding to 64, so one 2048-byte transfer can hold 32 of them.
///
/// Parsing only the first and issuing another transfer therefore throws
/// the rest away. Nothing reports it: no error is raised, the discarded
/// packets simply never reach the stack, and the peer recovers by
/// retransmission timeout hundreds of milliseconds later. The loss scales
/// with load, because the busier the link the more packets are waiting to
/// be coalesced — which is the opposite of how a driver should behave and
/// very hard to see from outside. Hence an iterator: a caller either
/// drains it or visibly does not.
///
/// # Why this currently only ever yields one frame
///
/// Coalescing is not on. The chip does it only with `HW_CFG.MEF` (Multiple
/// Ethernet Frames) set, together with `HW_CFG.BCE` and a `BURST_CAP`
/// saying how much a transfer may carry, and [`Lan9514::start`]
/// deliberately sets none of them. So this iterator yields exactly one
/// frame per transfer today and the loop around it is inert — kept because
/// it is the prerequisite for turning coalescing on, and because a receive
/// path that structurally cannot return a second frame is how the
/// discarding bug would come back.
///
/// It is worth turning on. Measured on a Pi 3 with one frame per transfer,
/// the receive path handles a *sustained* 4,000 frames a second with no
/// loss at all, but cannot absorb a burst: eight frames arriving
/// back-to-back lose 10% of themselves, 64 lose 26%, and a line-rate burst
/// loses 71%. The frames die in the chip's RX FIFO during the gap between
/// one transfer completing and the next being submitted, which recurs per
/// frame, and nothing anywhere reports it. What it costs is a TCP peer
/// whose acknowledgements went missing waiting out a retransmission
/// timeout — a page that should load in 20 ms taking a second.
///
/// **`MEF` alone stops traffic dead, and this is the trap.** Setting it
/// makes a transfer complete when `BURST_CAP` is reached rather than after
/// one frame, and how long the chip waits before flushing a partial burst
/// is `BULK_IN_DLY`'s to say. Without that register programmed, a lone
/// frame on a quiet link — a DHCP offer, say — sits in the chip waiting
/// for a burst that never arrives, and the interface delivers nothing at
/// all. Tried on real hardware, that is exactly what it did: link up, no
/// address, no frames. Anything enabling `MEF` has to set `BULK_IN_DLY`
/// with it, and has to be tested somewhere the card can be pulled — an
/// interface this broken cannot be recovered over the network it broke.
///
/// # Trailing bytes it will not guess at
///
/// Iteration stops at the first status word that does not describe a
/// plausible frame — one whose length is impossible, or which claims more
/// bytes than the transfer actually delivered. That is the conservative
/// end to be wrong at: the alternative, trusting a length and handing the
/// stack whatever bytes followed, would turn a padding-rule mistake into
/// corrupt frames rather than into missing ones.
pub struct Frames<'a> {
    /// The transfer's bytes — exactly what arrived, no more.
    buffer: &'a [u8],
    /// Where the next status word should be.
    offset: usize,
}

impl<'a> Frames<'a> {
    /// Parses the frames out of one transfer's `buffer`.
    ///
    /// `buffer` must be the bytes the transfer actually delivered, not the
    /// whole receive buffer: the length is what says where the frames stop.
    pub fn new(buffer: &'a [u8]) -> Self {
        Frames { buffer, offset: 0 }
    }

    /// The raw transfer bytes these frames are parsed out of.
    ///
    /// For a caller that has to keep a whole batch rather than consume it
    /// in one pass — copy this, then parse the copy with [`Frames::new`].
    /// Cheaper than copying each frame separately, and the only way to
    /// hold a batch without borrowing the driver.
    pub fn as_bytes(&self) -> &'a [u8] {
        self.buffer
    }
}

impl<'a> Iterator for Frames<'a> {
    type Item = &'a [u8];

    fn next(&mut self) -> Option<&'a [u8]> {
        loop {
            let remaining = self.buffer.len().checked_sub(self.offset)?;
            if remaining < RX_STATUS_SIZE {
                return None;
            }

            let status = u32::from_le_bytes([
                self.buffer[self.offset],
                self.buffer[self.offset + 1],
                self.buffer[self.offset + 2],
                self.buffer[self.offset + 3],
            ]);
            // Counts the 4-byte Ethernet CRC, which the caller doesn't
            // want.
            let frame_length = ((status & RX_STATUS_FRAME_LENGTH) >> 16) as usize;

            // Anything that cannot be a frame ends the iteration rather
            // than being skipped: past this point the offsets are guesses,
            // and a guess produces garbage rather than a gap.
            if frame_length <= 4
                || frame_length > MTU + 4
                || RX_STATUS_SIZE + frame_length > remaining
            {
                self.offset = self.buffer.len();
                return None;
            }

            let start = self.offset + RX_STATUS_SIZE;
            // Frame data starts just past the status word, so dropping the
            // CRC leaves it in `[start, start + frame_length - 4)`.
            let end = start + frame_length - 4;

            // The next status word is dword-aligned, so the packet is
            // followed by 0-3 bytes of padding. Stepped over even for a
            // frame being skipped, or every frame behind a bad one would
            // be lost with it.
            self.offset = (start + frame_length).next_multiple_of(RX_STATUS_SIZE);

            if status & RX_STATUS_ERROR != 0 {
                continue;
            }
            return Some(&self.buffer[start..end]);
        }
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
        debug_assert!(frame.len() <= FRAME_BUFFER_SIZE - 8);
        let length = frame.len();

        // TX command: one whole segment, byte length in both words.
        let command_a = TX_CMD_A_FIRST_SEG | TX_CMD_A_LAST_SEG | length as u32;
        let command_b = length as u32;
        self.buffer.0[0..4].copy_from_slice(&command_a.to_le_bytes());
        self.buffer.0[4..8].copy_from_slice(&command_b.to_le_bytes());
        self.buffer.0[8..8 + length].copy_from_slice(frame);
        8 + length
    }
}

impl Lan9514 {
    /// Brings `device` up as a LAN9514 if its USB vendor/product ID
    /// ([`VENDOR_ID`]:[`PRODUCT_ID`]) matches, returning `Ok(None)`
    /// otherwise — so a caller can try it on every enumerated device
    /// (also `Ok(None)` if it matches but doesn't expose the expected
    /// bulk IN/OUT endpoints).
    ///
    /// Reads the configuration descriptor to locate the bulk endpoints
    /// and activates the configuration (`SET_CONFIGURATION`), which the
    /// chip requires before it accepts vendor register *writes* (a
    /// register read works unconfigured, but a write is STALLed until
    /// configured). Call [`Self::start`] next to program the MAC and
    /// enable RX/TX.
    pub fn from_device(
        channel: &mut Channel,
        timer: &Timer,
        device: Device,
    ) -> Result<Option<Lan9514>, TransferError> {
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

        Ok(Some(Lan9514::new(device.endpoint, bulk_in, bulk_out)))
    }

    /// Brings up a LAN9514 that something *else* has already addressed
    /// and configured, given its endpoint 0.
    ///
    /// The counterpart to [`Self::from_device`] for the case where this
    /// crate's [`enumerate`](crate::usb::enumerate) isn't the thing
    /// walking the bus — an external host stack that has done
    /// SET_ADDRESS and SET_CONFIGURATION itself, and can say which
    /// address, speed and split target the device ended up with. Only
    /// the configuration descriptor is read here, and only to locate the
    /// bulk endpoints; the configuration is *not* re-activated, since
    /// re-issuing SET_CONFIGURATION would reset the device's endpoints
    /// out from under whoever configured it.
    ///
    /// No vendor/product check: a caller reaching for this has already
    /// identified the device (that is how it knows to use this driver at
    /// all). `Ok(None)` means the expected bulk IN/OUT pair wasn't
    /// found. Call [`Self::start`] next, exactly as after
    /// [`Self::from_device`].
    pub fn from_endpoint(
        channel: &mut Channel,
        timer: &Timer,
        endpoint: ControlEndpoint,
    ) -> Result<Option<Lan9514>, TransferError> {
        let mut config = [0u8; 64];
        let len = get_configuration_descriptor(channel, timer, endpoint, 0, &mut config)?;
        let Some((bulk_in, bulk_out)) = find_bulk_endpoints(&config[..len]) else {
            return Ok(None);
        };

        Ok(Some(Lan9514::new(endpoint, bulk_in, bulk_out)))
    }

    /// Assembles the driver around endpoints already located by one of
    /// the constructors above.
    fn new(endpoint: ControlEndpoint, bulk_in: BulkEndpoint, bulk_out: BulkEndpoint) -> Self {
        Lan9514 {
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

    /// Reads one 32-bit device register at offset `register` via a vendor
    /// control-IN request (see [`control::vendor_in`](crate::usb::control::vendor_in)).
    /// Registers are little-endian on the wire.
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

    /// Writes one 32-bit device register at offset `register` via a
    /// vendor control-OUT request (see
    /// [`control::vendor_out`](crate::usb::control::vendor_out)).
    /// Registers are little-endian on the wire.
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

    /// Reads the `ID_REV` register and splits it into the chip ID and
    /// silicon revision (see [`IdRevision`]) — the first thing to check
    /// to confirm the LAN9514 is responding (its ID reads back `0xEC00`).
    pub fn id_revision(
        &self,
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<IdRevision, TransferError> {
        let value = self.read_register(channel, timer, REG_ID_REV)?;
        Ok(IdRevision {
            id: (value >> 16) as u16,
            revision: (value & 0xFFFF) as u16,
        })
    }

    /// Programs the chip's MAC address registers (`ADDRL`/`ADDRH`) with
    /// `mac`, in transmission byte order. The LAN9514 powers up without a
    /// MAC on this board (no EEPROM), so bringing it up means writing the
    /// firmware-provided address (see
    /// [`Mailbox::mac_address`](crate::mailbox::Mailbox::mac_address)) in
    /// here — [`Self::start`] does this.
    pub fn set_mac_address(
        &self,
        channel: &mut Channel,
        timer: &Timer,
        mac: [u8; 6],
    ) -> Result<(), TransferError> {
        let low = u32::from_le_bytes([mac[0], mac[1], mac[2], mac[3]]);
        let high = u16::from_le_bytes([mac[4], mac[5]]) as u32;
        self.write_register(channel, timer, REG_ADDRL, low)?;
        self.write_register(channel, timer, REG_ADDRH, high)?;
        Ok(())
    }

    /// Reads the MAC address currently in the chip's `ADDRL`/`ADDRH`
    /// registers, in transmission byte order — what [`Self::set_mac_address`]
    /// last programmed (all-zero on a freshly powered chip).
    pub fn mac_address(
        &self,
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<[u8; 6], TransferError> {
        let low = self.read_register(channel, timer, REG_ADDRL)?.to_le_bytes();
        let high = self.read_register(channel, timer, REG_ADDRH)?.to_le_bytes();
        Ok([low[0], low[1], low[2], low[3], high[0], high[1]])
    }

    /// Brings the interface up for traffic: programs `mac` into the MAC
    /// address registers, enables deterministic RX polling
    /// (`HW_CFG.BIR`), routes the status LEDs, and enables the MAC
    /// receiver and transmitter (`MAC_CR`, `TX_CFG`). The internal PHY
    /// auto-negotiates the link on its own; poll [`Self::is_link_up`] to
    /// wait for it before expecting frames.
    ///
    /// One frame arrives per receive transfer, which is what the chip does
    /// out of reset. [`Frames`] records what turning that off would buy,
    /// and why doing it needs more than the one bit it looks like.
    pub fn start(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        mac: [u8; 6],
    ) -> Result<(), TransferError> {
        self.set_mac_address(channel, timer, mac)?;

        // `RXDOFF` cleared explicitly because [`Frames`] depends on it
        // being zero, even though zero is the reset default. `MEF` is
        // deliberately *not* set here — see [`Frames`].
        let hw_cfg = self.read_register(channel, timer, REG_HW_CFG)?;
        self.write_register(
            channel,
            timer,
            REG_HW_CFG,
            (hw_cfg & !HW_CFG_RXDOFF) | HW_CFG_BIR,
        )?;

        self.write_register(
            channel,
            timer,
            REG_LED_GPIO_CFG,
            LED_GPIO_CFG_SPD_LED | LED_GPIO_CFG_LNK_LED | LED_GPIO_CFG_FDX_LED,
        )?;
        // Full duplex, not half. The link is not up yet — auto-negotiation
        // has not finished, so the negotiated duplex cannot be known here
        // — and this is the assumption to be wrong in: half duplex needs a
        // hub, and no switch has negotiated it in twenty years. A caller
        // that wants certainty polls `is_link_up` and then `set_duplex`
        // with `is_full_duplex`. See `MAC_CR_FDPX` for what leaving this
        // clear costs.
        self.write_register(
            channel,
            timer,
            REG_MAC_CR,
            MAC_CR_FDPX | MAC_CR_TXEN | MAC_CR_RXEN,
        )?;
        self.write_register(channel, timer, REG_TX_CFG, TX_CFG_ON)?;
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
    pub fn is_link_up(&self, channel: &mut Channel, timer: &Timer) -> Result<bool, TransferError> {
        Ok(self.phy_read(channel, timer, PHY_REG_STATUS)? & BMSR_LINK_UP != 0)
    }

    /// Whether auto-negotiation settled on full duplex.
    ///
    /// The highest common denominator of what this PHY advertised and what
    /// the link partner did, which is what auto-negotiation selects. Only
    /// meaningful once [`Self::is_link_up`] returns `true` — before that
    /// the partner's advertisement register holds nothing.
    ///
    /// Read from the two standard MII ability registers rather than from
    /// this PHY's vendor status register, so the arithmetic is the one IEEE
    /// defines rather than one particular chip's summary of it.
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
    /// enabled the receiver and transmitter without turning them back off.
    pub fn set_all_multicast(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        pass: bool,
    ) -> Result<(), TransferError> {
        let mac_cr = self.read_register(channel, timer, REG_MAC_CR)?;
        let mac_cr = if pass {
            mac_cr | MAC_CR_MCPAS
        } else {
            mac_cr & !MAC_CR_MCPAS
        };
        self.write_register(channel, timer, REG_MAC_CR, mac_cr)
    }

    /// Puts the MAC in full or half duplex.
    ///
    /// **This has to match what the link actually negotiated**, and getting
    /// it wrong does not look like a driver fault. A half-duplex MAC means
    /// CSMA/CD: it treats transmitting while receiving as a collision and
    /// discards the frame. On a link the PHY negotiated as full duplex that
    /// loses frames precisely when traffic runs both ways at once, and only
    /// then — and it reports nothing, because the driver handed the frame
    /// over successfully and the loss happened below it. Measured here, a
    /// half-duplex MAC on a full-duplex link stalled 62% of requests when
    /// eight files were fetched over eight concurrent connections, at a
    /// median of 1.0 s against 3.9 ms for the same file fetched alone.
    ///
    /// A read-modify-write, so it can be called after [`Self::start`] has
    /// enabled the receiver and transmitter without turning them off again.
    /// The intended sequence is: `start`, poll [`Self::is_link_up`], then
    /// `set_duplex` with what [`Self::is_full_duplex`] reports.
    pub fn set_duplex(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        full: bool,
    ) -> Result<(), TransferError> {
        let mac_cr = self.read_register(channel, timer, REG_MAC_CR)?;
        let mac_cr = if full {
            (mac_cr | MAC_CR_FDPX) & !MAC_CR_RCVOWN
        } else {
            (mac_cr | MAC_CR_RCVOWN) & !MAC_CR_FDPX
        };
        self.write_register(channel, timer, REG_MAC_CR, mac_cr)
    }

    /// Sends one Ethernet frame (destination MAC through payload, without
    /// the CRC — the chip appends it). Prepends the chip's 8-byte TX
    /// command header and bulk-OUTs the lot. `frame` must be no larger
    /// than a frame buffer less that header.
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
    /// An empty iterator means there was nothing to receive — an empty
    /// bulk-IN (see `HW_CFG.BIR`) or a NAK.
    ///
    /// **Drain it.** One transfer routinely carries several frames and
    /// the next call overwrites the buffer, so a caller that takes the
    /// first and calls again silently discards the others; see
    /// [`Frames`].
    pub fn receive_frames(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<Frames<'_>, TransferError> {
        // Only issue a bulk-IN when the chip actually has a frame
        // buffered. A bulk-IN against an empty RX FIFO just NAKs, and the
        // DWC2 doesn't halt a bulk channel on NAK (it retries), so it
        // would block for the full transfer timeout on every idle poll.
        if self.read_register(channel, timer, REG_RX_FIFO_INF)? == 0 {
            return Ok(self.rx.frames(0));
        }

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
            // No frame waiting -- an empty response or a NAK.
            Err(TransferError::Nak) => 0,
            Err(error) => return Err(error),
        };

        Ok(self.rx.frames(received))
    }

    /// Reads MII (PHY) register `index` (see [`Self::is_link_up`]). The
    /// MII interface is driven through the chip's `MII_ADDR`/`MII_DATA`
    /// registers: point `MII_ADDR` at the register and set busy, wait for
    /// busy to clear, then read `MII_DATA`.
    fn phy_read(
        &self,
        channel: &mut Channel,
        timer: &Timer,
        index: u8,
    ) -> Result<u16, TransferError> {
        self.phy_wait_not_busy(channel, timer)?;
        let mii_address = (PHY_ID_INTERNAL << 11) | ((index as u32) << 6);
        self.write_register(channel, timer, REG_MII_ADDR, mii_address | MII_BUSY)?;
        self.phy_wait_not_busy(channel, timer)?;
        Ok(self.read_register(channel, timer, REG_MII_DATA)? as u16)
    }

    /// Spins until the MII interface's busy bit clears, bounded by
    /// [`MII_TIMEOUT_US`] so a stuck PHY access can't wedge the caller.
    fn phy_wait_not_busy(&self, channel: &mut Channel, timer: &Timer) -> Result<(), TransferError> {
        let start = timer.now_micros();
        while self.read_register(channel, timer, REG_MII_ADDR)? & MII_BUSY != 0 {
            if timer.now_micros() - start > MII_TIMEOUT_US {
                return Err(TransferError::Timeout);
            }
        }
        Ok(())
    }
}

#[cfg(feature = "smoltcp")]
use smoltcp::phy::{Device as PhyDevice, DeviceCapabilities, Medium, RxToken, TxToken};
#[cfg(feature = "smoltcp")]
use smoltcp::time::Instant;

/// Largest Ethernet frame this driver carries, in bytes: the 14-byte
/// header plus a 1500-byte payload (the chip appends the 4-byte CRC
/// itself, so it isn't counted here).
///
/// Unconditional, and not tied to any one adapter's feature, because it
/// is a property of Ethernet and of this chip — an out-of-crate adapter
/// (`rpi-hal-embassy`'s `embassy-net` one, say) needs the same number and
/// should not have to restate it.
pub const MTU: usize = 1514;

/// A [`smoltcp`] [`Device`] over a [`Lan9514`]: it moves the stack's
/// Ethernet frames through the driver's bulk endpoints. Construct it with
/// [`Lan9514Phy::new`], then hand it to `smoltcp`'s
/// [`Interface`](smoltcp::iface::Interface).
///
/// The driver's frame calls each need `&mut Channel` and `&Timer`, so the
/// adapter carries both alongside the [`Lan9514`]. The channel is *owned*
/// rather than borrowed — it belongs to this device for as long as the
/// adapter lives, which is also what keeps the rest of the bus usable
/// while a network stack is running on it. `smoltcp` hands out an
/// RX and a TX token together from one `&mut self` borrow, and both would
/// otherwise need that shared bus at once; the adapter sidesteps this by
/// doing the receive up front and copying the frame into a buffer the RX
/// token owns outright, leaving the TX token the sole borrower of the
/// driver (see [`Lan9514Phy::receive`]).
///
/// Available only with the `smoltcp` feature enabled.
#[cfg(feature = "smoltcp")]
pub struct Lan9514Phy<'a, 'c> {
    lan9514: Lan9514,
    channel: Channel<'c>,
    timer: &'a Timer,
    /// Scratch the TX token fills for smoltcp and hands to the driver.
    tx_scratch: [u8; MTU + 2],
    /// The last transfer's bytes, copied out of the driver.
    ///
    /// `smoltcp` asks for one frame per [`PhyDevice::receive`] call, and a
    /// transfer carries several (see [`Frames`]). Without somewhere to
    /// keep the remainder, each call would issue a fresh transfer and
    /// discard every frame after the first — a silent loss that grows
    /// with load. Copied rather than borrowed because the driver has to
    /// be handed to the TX token returned alongside the RX one.
    rx_batch: [u8; FRAME_BUFFER_SIZE],
    /// Bytes of [`Self::rx_batch`] the last transfer filled.
    rx_filled: usize,
    /// How many of that batch's frames have been handed to `smoltcp`
    /// already. The batch is re-parsed from the start each time, which
    /// costs nothing at these sizes and avoids storing offsets.
    rx_served: usize,
}

#[cfg(feature = "smoltcp")]
impl<'a, 'c> Lan9514Phy<'a, 'c> {
    /// Wraps an already-[`started`](Lan9514::start) LAN9514 as a smoltcp
    /// device, taking the host channel it drives frames through and
    /// borrowing the timer.
    pub fn new(lan9514: Lan9514, channel: Channel<'c>, timer: &'a Timer) -> Self {
        Self {
            lan9514,
            channel,
            timer,
            tx_scratch: [0; MTU + 2],
            rx_batch: [0; FRAME_BUFFER_SIZE],
            rx_filled: 0,
            rx_served: 0,
        }
    }

    /// The next frame of the batch already in hand, or `None` once it is
    /// exhausted.
    fn next_pending(&self) -> Option<&[u8]> {
        Frames::new(&self.rx_batch[..self.rx_filled]).nth(self.rx_served)
    }
}

#[cfg(feature = "smoltcp")]
impl<'c> PhyDevice for Lan9514Phy<'_, 'c> {
    type RxToken<'t>
        = Lan9514RxToken
    where
        Self: 't;
    type TxToken<'t>
        = Lan9514TxToken<'t, 'c>
    where
        Self: 't;

    /// Pulls a frame from the chip (if any) and returns it paired with a
    /// transmit token. The received bytes are copied into the RX token so
    /// the driver is free for the TX token returned alongside — see the
    /// type docs.
    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        // Only ask the chip for more once the batch in hand is used up:
        // a transfer carries several frames and the next one overwrites
        // the buffer they live in.
        if self.next_pending().is_none() {
            let batch = match self.lan9514.receive_frames(&mut self.channel, self.timer) {
                Ok(frames) => frames.as_bytes(),
                Err(_) => return None,
            };
            let filled = batch.len().min(FRAME_BUFFER_SIZE);
            self.rx_batch[..filled].copy_from_slice(&batch[..filled]);
            self.rx_filled = filled;
            self.rx_served = 0;
        }

        let frame = self.next_pending()?;
        let len = frame.len().min(MTU);
        let mut rx = Lan9514RxToken {
            buffer: [0; MTU],
            len,
        };
        rx.buffer[..len].copy_from_slice(&frame[..len]);
        self.rx_served += 1;

        let tx = Lan9514TxToken {
            lan9514: &mut self.lan9514,
            channel: &mut self.channel,
            timer: self.timer,
            scratch: &mut self.tx_scratch,
        };
        Some((rx, tx))
    }

    /// Returns a transmit token borrowing the driver.
    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        Some(Lan9514TxToken {
            lan9514: &mut self.lan9514,
            channel: &mut self.channel,
            timer: self.timer,
            scratch: &mut self.tx_scratch,
        })
    }

    /// Reports an Ethernet medium with [`MTU`] and a one-frame burst
    /// (the driver's frame calls are synchronous and one-at-a-time).
    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.medium = Medium::Ethernet;
        caps.max_transmission_unit = MTU;
        caps.max_burst_size = Some(1);
        caps
    }
}

/// An owned copy of one received frame, produced by
/// [`Lan9514Phy::receive`]. Owning the bytes (instead of borrowing the
/// driver's RX buffer) is what lets the driver be handed to the TX token
/// returned alongside it — see [`Lan9514Phy`].
///
/// Available only with the `smoltcp` feature enabled.
#[cfg(feature = "smoltcp")]
pub struct Lan9514RxToken {
    buffer: [u8; MTU],
    len: usize,
}

#[cfg(feature = "smoltcp")]
impl RxToken for Lan9514RxToken {
    /// Hands the received frame's bytes to `f`.
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        f(&self.buffer[..self.len])
    }
}

/// A pending transmit from [`Lan9514Phy`]: smoltcp fills the scratch
/// buffer via [`consume`](TxToken::consume), then the frame goes out the
/// driver's bulk-OUT endpoint.
///
/// Available only with the `smoltcp` feature enabled.
#[cfg(feature = "smoltcp")]
pub struct Lan9514TxToken<'a, 'c> {
    lan9514: &'a mut Lan9514,
    channel: &'a mut Channel<'c>,
    timer: &'a Timer,
    scratch: &'a mut [u8],
}

#[cfg(feature = "smoltcp")]
impl TxToken for Lan9514TxToken<'_, '_> {
    /// Lets `f` fill the frame buffer, then sends it. A failed send is
    /// dropped: smoltcp treats transmission as best-effort (retransmission
    /// is a higher layer's job), and `consume` has no channel to report an
    /// error on anyway.
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let result = f(&mut self.scratch[..len]);
        let _ = self
            .lan9514
            .send_frame(self.channel, self.timer, &self.scratch[..len]);
        result
    }
}
