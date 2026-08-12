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
//! [`receive_frame`](crate::usb::lan9514::Lan9514::receive_frame) and
//! [`send_frame`](crate::usb::lan9514::Lan9514::send_frame) move Ethernet
//! frames over the bulk endpoints (each frame carries the chip's TX
//! command header / RX status word, handled here). Register access is
//! over vendor control transfers, and the MII PHY is reachable for link
//! status ([`is_link_up`](crate::usb::lan9514::Lan9514::is_link_up)).
//!
//! This driver only moves raw frames; a TCP/IP stack goes on top as
//! application code. With the `smoltcp` feature enabled,
//! [`Lan9514Phy`](crate::usb::lan9514::Lan9514Phy) wraps
//! [`send_frame`](crate::usb::lan9514::Lan9514::send_frame) /
//! [`receive_frame`](crate::usb::lan9514::Lan9514::receive_frame) as a
//! `smoltcp` `phy::Device` so a stack can run on top; the
//! `usb_ethernet_smoltcp` example uses it, leasing an address over DHCP
//! and running a poll loop that answers pings.
//!
//! Register offsets, framing, and the bring-up sequence here follow
//! rsta2's `circle` `smsc951x` driver (in turn from the Linux `smsc95xx`
//! driver) for this same chip.

use crate::timer::Timer;
use crate::usb::control::{get_configuration_descriptor, set_configuration, vendor_in, vendor_out};
use crate::usb::descriptor::{ConfigurationDescriptor, Descriptors, EndpointDescriptor};
use crate::usb::dwc2::{ControlEndpoint, Dwc2Host, TransferError};
use crate::usb::Device;

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
/// `LED_GPIO_CFG` bit routing the speed LED to its pin.
const LED_GPIO_CFG_SPD_LED: u32 = 0x0100_0000;
/// `LED_GPIO_CFG` bit routing the link LED to its pin.
const LED_GPIO_CFG_LNK_LED: u32 = 0x0010_0000;
/// `LED_GPIO_CFG` bit routing the full-duplex LED to its pin.
const LED_GPIO_CFG_FDX_LED: u32 = 0x0001_0000;
/// `MAC_CR.RCVOWN` — receive own transmissions (needed in half duplex).
const MAC_CR_RCVOWN: u32 = 0x0080_0000;
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
/// `BMSR` bit 2 — link is up.
const BMSR_LINK_UP: u16 = 1 << 2;

/// `TX_CMD_A.FIRST_SEG` — this buffer holds the start of the frame.
const TX_CMD_A_FIRST_SEG: u32 = 0x0000_2000;
/// `TX_CMD_A.LAST_SEG` — this buffer holds the end of the frame.
const TX_CMD_A_LAST_SEG: u32 = 0x0000_1000;

/// `RX_STS` error-summary bit — the frame had a receive error.
const RX_STATUS_ERROR: u32 = 0x0000_8000;
/// `RX_STS` frame-length field (bits 16–29), including the 4-byte CRC.
const RX_STATUS_FRAME_LENGTH: u32 = 0x3FFF_0000;

/// Bytes reserved for a frame buffer — comfortably over a max Ethernet
/// frame plus the chip's framing words, and a whole number of both cache
/// lines and 512-byte bulk max packets (`2048 = 32 × 64 = 4 × 512`) so a
/// bulk-IN transfer fills it exactly with no rounding waste (see
/// [`Dwc2Host::bulk_in`]).
const FRAME_BUFFER_SIZE: usize = 2048;

/// Host channel this driver's transfers use — fine as a fixed constant,
/// nothing else contends for channels (see [`crate::usb::control`]).
const CHANNEL: usize = 0;

/// Timeout for an MII (PHY) access to complete, in microseconds.
const MII_TIMEOUT_US: u64 = 1_000_000;

/// A frame buffer sized and aligned for bulk DMA — cache-line aligned and
/// a whole number of cache lines, as [`Dwc2Host::bulk_in`] requires.
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

/// A configured LAN9514 Ethernet function: its endpoint 0 (for register
/// access) and bulk IN/OUT endpoints (for frame RX/TX), plus DMA frame
/// buffers. Build it with [`Self::from_device`], [`Self::start`] it, then
/// move frames with [`Self::send_frame`]/[`Self::receive_frame`].
pub struct Lan9514 {
    endpoint: ControlEndpoint,
    bulk_in: BulkEndpoint,
    bulk_out: BulkEndpoint,
    rx_buffer: FrameBuffer,
    tx_buffer: FrameBuffer,
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
        dwc2: &mut Dwc2Host,
        timer: &Timer,
        device: Device,
    ) -> Result<Option<Lan9514>, TransferError> {
        if device.descriptor.vendor_id != VENDOR_ID || device.descriptor.product_id != PRODUCT_ID {
            return Ok(None);
        }

        let mut config = [0u8; 64];
        let len = get_configuration_descriptor(dwc2, timer, device.endpoint, 0, &mut config)?;
        let Some(config_value) = ConfigurationDescriptor::parse(&config[..len]).map(|c| c.value())
        else {
            return Ok(None);
        };

        let mut bulk_in = None;
        let mut bulk_out = None;
        for descriptor in Descriptors::new(&config[..len]) {
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
        let (Some(bulk_in), Some(bulk_out)) = (bulk_in, bulk_out) else {
            return Ok(None);
        };

        set_configuration(dwc2, timer, device.endpoint, config_value)?;

        Ok(Some(Lan9514 {
            endpoint: device.endpoint,
            bulk_in,
            bulk_out,
            rx_buffer: FrameBuffer([0; FRAME_BUFFER_SIZE]),
            tx_buffer: FrameBuffer([0; FRAME_BUFFER_SIZE]),
        }))
    }

    /// Reads one 32-bit device register at offset `register` via a vendor
    /// control-IN request (see [`control::vendor_in`](crate::usb::control::vendor_in)).
    /// Registers are little-endian on the wire.
    pub fn read_register(
        &self,
        dwc2: &mut Dwc2Host,
        timer: &Timer,
        register: u16,
    ) -> Result<u32, TransferError> {
        let mut value = [0u8; 4];
        vendor_in(
            dwc2,
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
        dwc2: &mut Dwc2Host,
        timer: &Timer,
        register: u16,
        value: u32,
    ) -> Result<(), TransferError> {
        vendor_out(
            dwc2,
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
        dwc2: &mut Dwc2Host,
        timer: &Timer,
    ) -> Result<IdRevision, TransferError> {
        let value = self.read_register(dwc2, timer, REG_ID_REV)?;
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
        dwc2: &mut Dwc2Host,
        timer: &Timer,
        mac: [u8; 6],
    ) -> Result<(), TransferError> {
        let low = u32::from_le_bytes([mac[0], mac[1], mac[2], mac[3]]);
        let high = u16::from_le_bytes([mac[4], mac[5]]) as u32;
        self.write_register(dwc2, timer, REG_ADDRL, low)?;
        self.write_register(dwc2, timer, REG_ADDRH, high)?;
        Ok(())
    }

    /// Reads the MAC address currently in the chip's `ADDRL`/`ADDRH`
    /// registers, in transmission byte order — what [`Self::set_mac_address`]
    /// last programmed (all-zero on a freshly powered chip).
    pub fn mac_address(
        &self,
        dwc2: &mut Dwc2Host,
        timer: &Timer,
    ) -> Result<[u8; 6], TransferError> {
        let low = self.read_register(dwc2, timer, REG_ADDRL)?.to_le_bytes();
        let high = self.read_register(dwc2, timer, REG_ADDRH)?.to_le_bytes();
        Ok([low[0], low[1], low[2], low[3], high[0], high[1]])
    }

    /// Brings the interface up for traffic: programs `mac` into the MAC
    /// address registers, enables deterministic RX polling
    /// (`HW_CFG.BIR`), routes the status LEDs, and enables the MAC
    /// receiver and transmitter (`MAC_CR`, `TX_CFG`). The internal PHY
    /// auto-negotiates the link on its own; poll [`Self::is_link_up`] to
    /// wait for it before expecting frames.
    pub fn start(
        &mut self,
        dwc2: &mut Dwc2Host,
        timer: &Timer,
        mac: [u8; 6],
    ) -> Result<(), TransferError> {
        self.set_mac_address(dwc2, timer, mac)?;

        let hw_cfg = self.read_register(dwc2, timer, REG_HW_CFG)?;
        self.write_register(dwc2, timer, REG_HW_CFG, hw_cfg | HW_CFG_BIR)?;

        self.write_register(
            dwc2,
            timer,
            REG_LED_GPIO_CFG,
            LED_GPIO_CFG_SPD_LED | LED_GPIO_CFG_LNK_LED | LED_GPIO_CFG_FDX_LED,
        )?;
        self.write_register(
            dwc2,
            timer,
            REG_MAC_CR,
            MAC_CR_RCVOWN | MAC_CR_TXEN | MAC_CR_RXEN,
        )?;
        self.write_register(dwc2, timer, REG_TX_CFG, TX_CFG_ON)?;
        Ok(())
    }

    /// The bulk-IN (frame receive) endpoint's max packet size.
    pub fn bulk_in_max_packet_size(&self) -> u16 {
        self.bulk_in.max_packet_size
    }

    /// The bulk-OUT (frame transmit) endpoint's max packet size.
    pub fn bulk_out_max_packet_size(&self) -> u16 {
        self.bulk_out.max_packet_size
    }

    /// Whether the Ethernet link is up, read from the PHY's basic-mode
    /// status register over MII. `false` until the cable is connected and
    /// auto-negotiation completes.
    pub fn is_link_up(&self, dwc2: &mut Dwc2Host, timer: &Timer) -> Result<bool, TransferError> {
        Ok(self.phy_read(dwc2, timer, PHY_REG_STATUS)? & BMSR_LINK_UP != 0)
    }

    /// Sends one Ethernet frame (destination MAC through payload, without
    /// the CRC — the chip appends it). Prepends the chip's 8-byte TX
    /// command header and bulk-OUTs the lot. `frame` must be no larger
    /// than a frame buffer less that header.
    pub fn send_frame(
        &mut self,
        dwc2: &mut Dwc2Host,
        timer: &Timer,
        frame: &[u8],
    ) -> Result<(), TransferError> {
        debug_assert!(frame.len() <= FRAME_BUFFER_SIZE - 8);
        let length = frame.len();

        // TX command: one whole segment, byte length in both words.
        let command_a = TX_CMD_A_FIRST_SEG | TX_CMD_A_LAST_SEG | length as u32;
        let command_b = length as u32;
        self.tx_buffer.0[0..4].copy_from_slice(&command_a.to_le_bytes());
        self.tx_buffer.0[4..8].copy_from_slice(&command_b.to_le_bytes());
        self.tx_buffer.0[8..8 + length].copy_from_slice(frame);

        let number = self.bulk_out.number;
        let endpoint = ControlEndpoint {
            max_packet_size: self.bulk_out.max_packet_size,
            ..self.endpoint
        };
        dwc2.bulk_out(
            CHANNEL,
            endpoint,
            number,
            &mut self.bulk_out.toggle,
            &self.tx_buffer.0[..8 + length],
            timer,
        )?;
        Ok(())
    }

    /// Polls for one received Ethernet frame. Returns `Ok(Some(frame))`
    /// with the frame bytes (destination MAC through payload, CRC
    /// stripped) when one arrived, or `Ok(None)` when there's nothing to
    /// receive (an empty bulk-IN — see `HW_CFG.BIR` — or a NAK) or the
    /// chip flagged the frame as errored. The returned slice borrows this
    /// driver's RX buffer until the next call.
    pub fn receive_frame(
        &mut self,
        dwc2: &mut Dwc2Host,
        timer: &Timer,
    ) -> Result<Option<&[u8]>, TransferError> {
        // Only issue a bulk-IN when the chip actually has a frame
        // buffered. A bulk-IN against an empty RX FIFO just NAKs, and the
        // DWC2 doesn't halt a bulk channel on NAK (it retries), so it
        // would block for the full transfer timeout on every idle poll.
        if self.read_register(dwc2, timer, REG_RX_FIFO_INF)? == 0 {
            return Ok(None);
        }

        let number = self.bulk_in.number;
        let endpoint = ControlEndpoint {
            max_packet_size: self.bulk_in.max_packet_size,
            ..self.endpoint
        };
        let received = match dwc2.bulk_in(
            CHANNEL,
            endpoint,
            number,
            &mut self.bulk_in.toggle,
            &mut self.rx_buffer.0,
            timer,
        ) {
            Ok(received) => received,
            // No frame waiting -- an empty response or a NAK.
            Err(TransferError::Nak) => return Ok(None),
            Err(error) => return Err(error),
        };

        // Every received frame is prefixed by a 4-byte RX status word.
        if received < 4 {
            return Ok(None);
        }
        let status = u32::from_le_bytes([
            self.rx_buffer.0[0],
            self.rx_buffer.0[1],
            self.rx_buffer.0[2],
            self.rx_buffer.0[3],
        ]);
        if status & RX_STATUS_ERROR != 0 {
            return Ok(None);
        }

        // The status word's frame length counts the 4-byte Ethernet CRC,
        // which the caller doesn't want. Frame data starts just past the
        // status word (index 4), so dropping the CRC leaves it in
        // `rx_buffer[4..4 + (frame_length - 4)]` == `rx_buffer[4..frame_length]`.
        let frame_length = ((status & RX_STATUS_FRAME_LENGTH) >> 16) as usize;
        if frame_length <= 4 {
            return Ok(None);
        }
        let end = frame_length.min(received);
        Ok(Some(&self.rx_buffer.0[4..end]))
    }

    /// Reads MII (PHY) register `index` (see [`Self::is_link_up`]). The
    /// MII interface is driven through the chip's `MII_ADDR`/`MII_DATA`
    /// registers: point `MII_ADDR` at the register and set busy, wait for
    /// busy to clear, then read `MII_DATA`.
    fn phy_read(
        &self,
        dwc2: &mut Dwc2Host,
        timer: &Timer,
        index: u8,
    ) -> Result<u16, TransferError> {
        self.phy_wait_not_busy(dwc2, timer)?;
        let mii_address = (PHY_ID_INTERNAL << 11) | ((index as u32) << 6);
        self.write_register(dwc2, timer, REG_MII_ADDR, mii_address | MII_BUSY)?;
        self.phy_wait_not_busy(dwc2, timer)?;
        Ok(self.read_register(dwc2, timer, REG_MII_DATA)? as u16)
    }

    /// Spins until the MII interface's busy bit clears, bounded by
    /// [`MII_TIMEOUT_US`] so a stuck PHY access can't wedge the caller.
    fn phy_wait_not_busy(&self, dwc2: &mut Dwc2Host, timer: &Timer) -> Result<(), TransferError> {
        let start = timer.now_micros();
        while self.read_register(dwc2, timer, REG_MII_ADDR)? & MII_BUSY != 0 {
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

/// Largest Ethernet frame the [`Lan9514Phy`] adapter moves, in bytes: the
/// 14-byte header plus a 1500-byte payload (the chip appends the 4-byte
/// CRC itself, so it isn't counted here).
#[cfg(any(feature = "smoltcp", feature = "embassy-net-driver"))]
pub const MTU: usize = 1514;

/// A [`smoltcp`] [`Device`] over a [`Lan9514`]: it moves the stack's
/// Ethernet frames through the driver's bulk endpoints. Construct it with
/// [`Lan9514Phy::new`], then hand it to `smoltcp`'s
/// [`Interface`](smoltcp::iface::Interface).
///
/// The driver's frame calls each need `&mut Dwc2Host` and `&Timer`, so the
/// adapter carries both alongside the [`Lan9514`]. `smoltcp` hands out an
/// RX and a TX token together from one `&mut self` borrow, and both would
/// otherwise need that shared bus at once; the adapter sidesteps this by
/// doing the receive up front and copying the frame into a buffer the RX
/// token owns outright, leaving the TX token the sole borrower of the
/// driver (see [`Lan9514Phy::receive`]).
///
/// Available only with the `smoltcp` feature enabled.
#[cfg(feature = "smoltcp")]
pub struct Lan9514Phy<'a> {
    lan9514: Lan9514,
    dwc2: &'a mut Dwc2Host,
    timer: &'a Timer,
    /// Scratch the TX token fills for smoltcp and hands to the driver.
    tx_scratch: [u8; MTU + 2],
}

#[cfg(feature = "smoltcp")]
impl<'a> Lan9514Phy<'a> {
    /// Wraps an already-[`started`](Lan9514::start) LAN9514 as a smoltcp
    /// device, borrowing the DWC2 host and timer it drives frames through.
    pub fn new(lan9514: Lan9514, dwc2: &'a mut Dwc2Host, timer: &'a Timer) -> Self {
        Self {
            lan9514,
            dwc2,
            timer,
            tx_scratch: [0; MTU + 2],
        }
    }
}

#[cfg(feature = "smoltcp")]
impl PhyDevice for Lan9514Phy<'_> {
    type RxToken<'t>
        = Lan9514RxToken
    where
        Self: 't;
    type TxToken<'t>
        = Lan9514TxToken<'t>
    where
        Self: 't;

    /// Pulls a frame from the chip (if any) and returns it paired with a
    /// transmit token. The received bytes are copied into the RX token so
    /// the driver is free for the TX token returned alongside — see the
    /// type docs.
    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let frame = match self.lan9514.receive_frame(self.dwc2, self.timer) {
            Ok(Some(frame)) => frame,
            Ok(None) => return None,
            Err(_) => return None,
        };
        let len = frame.len().min(MTU);
        let mut rx = Lan9514RxToken {
            buffer: [0; MTU],
            len,
        };
        rx.buffer[..len].copy_from_slice(&frame[..len]);

        let tx = Lan9514TxToken {
            lan9514: &mut self.lan9514,
            dwc2: self.dwc2,
            timer: self.timer,
            scratch: &mut self.tx_scratch,
        };
        Some((rx, tx))
    }

    /// Returns a transmit token borrowing the driver.
    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        Some(Lan9514TxToken {
            lan9514: &mut self.lan9514,
            dwc2: self.dwc2,
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
pub struct Lan9514TxToken<'a> {
    lan9514: &'a mut Lan9514,
    dwc2: &'a mut Dwc2Host,
    timer: &'a Timer,
    scratch: &'a mut [u8],
}

#[cfg(feature = "smoltcp")]
impl TxToken for Lan9514TxToken<'_> {
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
            .send_frame(self.dwc2, self.timer, &self.scratch[..len]);
        result
    }
}

/// Woken by [`wake_rx`] so a parked `embassy-net` runner re-polls
/// `Lan9514Driver`'s `receive`.
///
/// A single slot, not a table: there is one LAN9514 on this hardware.
#[cfg(feature = "embassy-net-driver")]
static RX_WAKER: critical_section::Mutex<core::cell::RefCell<Option<core::task::Waker>>> =
    critical_section::Mutex::new(core::cell::RefCell::new(None));

/// Tells a parked [`Lan9514Driver`] that a frame may have arrived.
///
/// **An application must call this periodically** — typically from a task
/// on an `embassy-time` ticker — or received frames are never picked up.
///
/// This exists because the chip is reached by polling, not by interrupt:
/// its frames come over USB bulk endpoints, and neither this crate nor the
/// SoC's interrupt controller has any USB interrupt wired up, so nothing
/// can tell the driver a packet landed. `embassy-net`'s `Driver` contract
/// requires waking the waker when one does, and this is the only honest
/// way to satisfy it. The polling interval is the application's to choose
/// because it is a latency-versus-wake-ups trade only it can make; this
/// crate takes no dependency on a time source to make it.
///
/// Cheap when idle: with the runner already awake or nothing parked, it
/// does nothing.
#[cfg(feature = "embassy-net-driver")]
pub fn wake_rx() {
    critical_section::with(|cs| {
        if let Some(waker) = RX_WAKER.borrow_ref_mut(cs).take() {
            waker.wake();
        }
    });
}

/// An [`embassy_net_driver::Driver`] over a [`Lan9514`], moving
/// `embassy-net`'s Ethernet frames through the driver's bulk endpoints.
/// Construct it with [`Lan9514Driver::new`] and hand it to
/// `embassy_net::new`.
///
/// The sibling of [`Lan9514Phy`], which does the same job for `smoltcp`.
/// They are deliberately parallel rather than shared: the two token
/// traits differ just enough — a waker in the signatures here, `&[u8]`
/// versus `&mut [u8]` in `RxToken::consume` — that unifying them would
/// need generics obscuring both, and the part actually worth sharing (the
/// frame moving itself) already is, in [`Lan9514::receive_frame`] and
/// [`Lan9514::send_frame`].
///
/// Requires the application to call [`wake_rx`] periodically; see its
/// documentation for why.
///
/// Available only with the `embassy-net-driver` feature enabled.
#[cfg(feature = "embassy-net-driver")]
pub struct Lan9514Driver<'a> {
    lan9514: Lan9514,
    dwc2: &'a mut Dwc2Host,
    timer: &'a Timer,
    mac: [u8; 6],
    /// Scratch the TX token fills for the stack and hands to the driver.
    tx_scratch: [u8; MTU + 2],
}

#[cfg(feature = "embassy-net-driver")]
impl<'a> Lan9514Driver<'a> {
    /// Wraps an already-[`started`](Lan9514::start) LAN9514 as an
    /// `embassy-net` device, borrowing the DWC2 host and timer it drives
    /// frames through.
    ///
    /// `mac` must be the address passed to [`Lan9514::start`]: the driver
    /// programs it into the chip but doesn't retain it, and
    /// `embassy-net` needs it to answer ARP.
    pub fn new(lan9514: Lan9514, dwc2: &'a mut Dwc2Host, timer: &'a Timer, mac: [u8; 6]) -> Self {
        Self {
            lan9514,
            dwc2,
            timer,
            mac,
            tx_scratch: [0; MTU + 2],
        }
    }
}

#[cfg(feature = "embassy-net-driver")]
impl embassy_net_driver::Driver for Lan9514Driver<'_> {
    type RxToken<'t>
        = Lan9514NetRxToken
    where
        Self: 't;
    type TxToken<'t>
        = Lan9514NetTxToken<'t>
    where
        Self: 't;

    /// Pulls a frame from the chip if one is waiting, pairing it with a
    /// transmit token so a reply can be built from it without allocating.
    ///
    /// With nothing to return, registers `cx`'s waker for [`wake_rx`] —
    /// the frame did not arrive on an interrupt, so only the
    /// application's polling can report the next one.
    ///
    /// The received bytes are copied into the RX token, leaving the TX
    /// token the sole borrower of the driver: both are handed out from
    /// one `&mut self`, and the bus underneath can only serve one.
    fn receive(
        &mut self,
        cx: &mut core::task::Context,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let frame = match self.lan9514.receive_frame(self.dwc2, self.timer) {
            Ok(Some(frame)) => frame,
            // A transfer error is reported the same way as an empty queue:
            // there is no frame to hand up either way, and the stack's
            // recovery for both is to ask again.
            Ok(None) | Err(_) => {
                critical_section::with(|cs| {
                    *RX_WAKER.borrow_ref_mut(cs) = Some(cx.waker().clone());
                });
                return None;
            }
        };

        let len = frame.len().min(MTU);
        let mut rx = Lan9514NetRxToken {
            buffer: [0; MTU],
            len,
        };
        rx.buffer[..len].copy_from_slice(&frame[..len]);

        let tx = Lan9514NetTxToken {
            lan9514: &mut self.lan9514,
            dwc2: self.dwc2,
            timer: self.timer,
            scratch: &mut self.tx_scratch,
        };
        Some((rx, tx))
    }

    /// Returns a transmit token borrowing the driver.
    ///
    /// Never `None`, so the waker goes unused: sending is synchronous
    /// here, with no queue that can fill up and later drain.
    fn transmit(&mut self, _cx: &mut core::task::Context) -> Option<Self::TxToken<'_>> {
        Some(Lan9514NetTxToken {
            lan9514: &mut self.lan9514,
            dwc2: self.dwc2,
            timer: self.timer,
            scratch: &mut self.tx_scratch,
        })
    }

    /// Always reports the link as up.
    ///
    /// Not because the real state is unavailable — [`Lan9514::is_link_up`]
    /// reads it from the PHY — but because reading it costs MII access
    /// over USB control transfers, and `embassy-net` calls this on every
    /// pass of its runner. Since those passes are driven by the
    /// application's [`wake_rx`] cadence, the cost would scale with the
    /// polling rate: at a millisecond interval the link check alone could
    /// crowd out the frame traffic it is meant to be supporting.
    ///
    /// The consequence is that an unplugged cable surfaces as transfers
    /// failing rather than as a link-down transition, and
    /// `Stack::wait_link_up` returns immediately. An application that
    /// needs better can call [`Lan9514::is_link_up`] on its own schedule,
    /// which is also the only place that knows how often is often enough.
    fn link_state(&mut self, _cx: &mut core::task::Context) -> embassy_net_driver::LinkState {
        embassy_net_driver::LinkState::Up
    }

    /// Reports [`MTU`] and a one-frame burst — the driver's frame calls
    /// are synchronous and one at a time.
    fn capabilities(&self) -> embassy_net_driver::Capabilities {
        let mut caps = embassy_net_driver::Capabilities::default();
        caps.max_transmission_unit = MTU;
        caps.max_burst_size = Some(1);
        caps
    }

    /// The Ethernet address this device was started with.
    fn hardware_address(&self) -> embassy_net_driver::HardwareAddress {
        embassy_net_driver::HardwareAddress::Ethernet(self.mac)
    }
}

/// An owned copy of one received frame, produced by
/// `Lan9514Driver`'s `receive`. Owning the bytes is what lets the driver
/// go to the TX token handed out alongside it.
///
/// Available only with the `embassy-net-driver` feature enabled.
#[cfg(feature = "embassy-net-driver")]
pub struct Lan9514NetRxToken {
    buffer: [u8; MTU],
    len: usize,
}

#[cfg(feature = "embassy-net-driver")]
impl embassy_net_driver::RxToken for Lan9514NetRxToken {
    /// Hands the received frame's bytes to `f`.
    fn consume<R, F>(mut self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        f(&mut self.buffer[..self.len])
    }
}

/// A pending transmit from [`Lan9514Driver`]: the stack fills the scratch
/// buffer via [`consume`](embassy_net_driver::TxToken::consume), then the
/// frame goes out the driver's bulk-OUT endpoint.
///
/// Available only with the `embassy-net-driver` feature enabled.
#[cfg(feature = "embassy-net-driver")]
pub struct Lan9514NetTxToken<'a> {
    lan9514: &'a mut Lan9514,
    dwc2: &'a mut Dwc2Host,
    timer: &'a Timer,
    scratch: &'a mut [u8],
}

#[cfg(feature = "embassy-net-driver")]
impl embassy_net_driver::TxToken for Lan9514NetTxToken<'_> {
    /// Lets `f` fill the frame buffer, then sends it. A failed send is
    /// dropped: `consume` has no channel to report an error on, and
    /// retransmission belongs to a higher layer.
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let result = f(&mut self.scratch[..len]);
        let _ = self
            .lan9514
            .send_frame(self.dwc2, self.timer, &self.scratch[..len]);
        result
    }
}
