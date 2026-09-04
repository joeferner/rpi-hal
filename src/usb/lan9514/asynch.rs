//! Interrupt-driven `async` twins of [`Lan9514`]'s transfer methods,
//! built on [`crate::usb::dwc2::asynch`]'s primitives rather than the
//! blocking ones.
//!
//! Every method here is a `_async` suffixed twin of one next door, doing
//! the same work over the same wire format; the difference is only that
//! the time a transfer spends on the bus is awaited rather than spun on.
//! The interrupt wiring that makes them resolve — `GINTMSK`, the
//! interrupt controller, the CPU mask, and an `__irq_handler` that calls
//! [`crate::usb::dwc2::on_irq`] — is the same wiring that module
//! documents, and without it nothing here completes.
//!
//! One thing genuinely differs, and it is the reason this exists:
//! [`Lan9514::receive_frames_async`] can afford to leave a bulk IN parked
//! on an empty receive FIFO, which the blocking twin cannot. See that
//! method.
//!
//! [`Lan9514::split`] is here for the same reason. The chip's two bulk
//! endpoints are independent pipes, and a driver that parks on the
//! receive one has to be able to transmit past it; that is only
//! expressible if the two directions can be borrowed apart.
//!
//! Enumeration ([`Lan9514::from_device`], [`Lan9514::from_endpoint`]) has
//! no twin. It runs once, during bring-up, before there is an executor
//! with anything else to do — the case the blocking path is already the
//! right shape for.

use super::{
    Frames, IdRevision, Lan9514, Rx, Tx, BMSR_LINK_UP, HW_CFG_BIR, HW_CFG_RXDOFF,
    LED_GPIO_CFG_FDX_LED, LED_GPIO_CFG_LNK_LED, LED_GPIO_CFG_SPD_LED, MAC_CR_FDPX, MAC_CR_MCPAS,
    MAC_CR_RXEN, MAC_CR_TXEN, MII_BUSY, MII_TIMEOUT_US, PHY_ID_INTERNAL, PHY_REG_STATUS,
    READ_REGISTER, REG_ADDRH, REG_ADDRL, REG_HW_CFG, REG_ID_REV, REG_LED_GPIO_CFG, REG_MAC_CR,
    REG_MII_ADDR, REG_MII_DATA, REG_TX_CFG, TX_CFG_ON, WRITE_REGISTER,
};
use crate::timer::Timer;
use crate::usb::control::{vendor_in_async, vendor_out_async};
use crate::usb::dwc2::{Channel, ControlEndpoint, TransferError};

/// The receive direction of a [`Lan9514`], borrowed from it by
/// [`Lan9514::split`]: its bulk IN endpoint and the frame buffer that
/// endpoint's transfers land in.
///
/// Available only with the `async` feature enabled.
pub struct Lan9514Rx<'a> {
    endpoint: ControlEndpoint,
    rx: &'a mut Rx,
}

/// The transmit direction of a [`Lan9514`] — the counterpart to
/// [`Lan9514Rx`], and produced by the same [`Lan9514::split`].
///
/// Available only with the `async` feature enabled.
pub struct Lan9514Tx<'a> {
    endpoint: ControlEndpoint,
    tx: &'a mut Tx,
}

impl Rx {
    /// Awaits one bulk IN and decodes what arrives. `device` is the
    /// chip's endpoint 0, which carries the address and speed the bulk
    /// endpoint shares.
    ///
    /// Shared by [`Lan9514::receive_frames_async`] and
    /// [`Lan9514Rx::receive_frames_async`] rather than either delegating
    /// to the other: the returned frame borrows this buffer, so a
    /// delegation through a temporary [`Lan9514Rx`] would tie it to the
    /// temporary.
    async fn receive_async(
        &mut self,
        device: ControlEndpoint,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<Frames<'_>, TransferError> {
        let endpoint = self.endpoint(device);
        let number = self.bulk_in.number;
        let received = match channel
            .bulk_in_async(
                endpoint,
                number,
                &mut self.bulk_in.toggle,
                &mut self.buffer.0,
                timer,
            )
            .await
        {
            Ok(received) => received,
            Err(TransferError::Nak) => 0,
            Err(error) => return Err(error),
        };
        Ok(self.frames(received))
    }
}

impl Tx {
    /// Stages `frame` behind the chip's TX command header and awaits the
    /// bulk OUT that sends it.
    async fn send_async(
        &mut self,
        device: ControlEndpoint,
        channel: &mut Channel<'_>,
        timer: &Timer,
        frame: &[u8],
    ) -> Result<(), TransferError> {
        let staged = self.stage(frame);
        let endpoint = self.endpoint(device);
        let number = self.bulk_out.number;
        channel
            .bulk_out_async(
                endpoint,
                number,
                &mut self.bulk_out.toggle,
                &self.buffer.0[..staged],
                timer,
            )
            .await?;
        Ok(())
    }
}

impl Lan9514Rx<'_> {
    /// Awaits received Ethernet frames on the bulk IN endpoint —
    /// [`Lan9514::receive_frames_async`] restricted to this direction,
    /// with the same behaviour and the same caveats. Read that method
    /// before using this one.
    pub async fn receive_frames_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<Frames<'_>, TransferError> {
        self.rx.receive_async(self.endpoint, channel, timer).await
    }
}

impl Lan9514Tx<'_> {
    /// Sends one Ethernet frame out the bulk OUT endpoint —
    /// [`Lan9514::send_frame_async`] restricted to this direction.
    pub async fn send_frame_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        frame: &[u8],
    ) -> Result<(), TransferError> {
        self.tx
            .send_async(self.endpoint, channel, timer, frame)
            .await
    }
}

impl Lan9514 {
    /// Async [`Lan9514::read_register`].
    pub async fn read_register_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        register: u16,
    ) -> Result<u32, TransferError> {
        let mut value = [0u8; 4];
        vendor_in_async(
            channel,
            timer,
            self.endpoint,
            READ_REGISTER,
            0,
            register,
            &mut value,
        )
        .await?;
        Ok(u32::from_le_bytes(value))
    }

    /// Async [`Lan9514::write_register`].
    pub async fn write_register_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        register: u16,
        value: u32,
    ) -> Result<(), TransferError> {
        vendor_out_async(
            channel,
            timer,
            self.endpoint,
            WRITE_REGISTER,
            0,
            register,
            &value.to_le_bytes(),
        )
        .await
    }

    /// Async [`Lan9514::id_revision`].
    pub async fn id_revision_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<IdRevision, TransferError> {
        let value = self.read_register_async(channel, timer, REG_ID_REV).await?;
        Ok(IdRevision {
            id: (value >> 16) as u16,
            revision: (value & 0xFFFF) as u16,
        })
    }

    /// Async [`Lan9514::set_mac_address`].
    pub async fn set_mac_address_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        mac: [u8; 6],
    ) -> Result<(), TransferError> {
        let low = u32::from_le_bytes([mac[0], mac[1], mac[2], mac[3]]);
        let high = u16::from_le_bytes([mac[4], mac[5]]) as u32;
        self.write_register_async(channel, timer, REG_ADDRL, low)
            .await?;
        self.write_register_async(channel, timer, REG_ADDRH, high)
            .await?;
        Ok(())
    }

    /// Async [`Lan9514::mac_address`].
    pub async fn mac_address_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<[u8; 6], TransferError> {
        let low = self
            .read_register_async(channel, timer, REG_ADDRL)
            .await?
            .to_le_bytes();
        let high = self
            .read_register_async(channel, timer, REG_ADDRH)
            .await?
            .to_le_bytes();
        Ok([low[0], low[1], low[2], low[3], high[0], high[1]])
    }

    /// Async [`Lan9514::start`], programming the same registers in the
    /// same order.
    pub async fn start_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        mac: [u8; 6],
    ) -> Result<(), TransferError> {
        self.set_mac_address_async(channel, timer, mac).await?;

        let hw_cfg = self.read_register_async(channel, timer, REG_HW_CFG).await?;
        self.write_register_async(
            channel,
            timer,
            REG_HW_CFG,
            (hw_cfg & !HW_CFG_RXDOFF) | HW_CFG_BIR,
        )
        .await?;

        self.write_register_async(
            channel,
            timer,
            REG_LED_GPIO_CFG,
            LED_GPIO_CFG_SPD_LED | LED_GPIO_CFG_LNK_LED | LED_GPIO_CFG_FDX_LED,
        )
        .await?;
        self.write_register_async(
            channel,
            timer,
            REG_MAC_CR,
            MAC_CR_FDPX | MAC_CR_TXEN | MAC_CR_RXEN,
        )
        .await?;
        self.write_register_async(channel, timer, REG_TX_CFG, TX_CFG_ON)
            .await?;
        Ok(())
    }

    /// Async [`Lan9514::is_link_up`].
    pub async fn is_link_up_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<bool, TransferError> {
        Ok(self.phy_read_async(channel, timer, PHY_REG_STATUS).await? & BMSR_LINK_UP != 0)
    }

    /// Async [`Lan9514::set_all_multicast`]. Read that one for why the
    /// chip's default is worth changing.
    pub async fn set_all_multicast_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        pass: bool,
    ) -> Result<(), TransferError> {
        let mac_cr = self.read_register_async(channel, timer, REG_MAC_CR).await?;
        let mac_cr = if pass {
            mac_cr | MAC_CR_MCPAS
        } else {
            mac_cr & !MAC_CR_MCPAS
        };
        self.write_register_async(channel, timer, REG_MAC_CR, mac_cr)
            .await
    }

    /// Borrows the two frame directions apart, so each can be driven
    /// independently — a receive parked on one host channel while
    /// transmits go out on another.
    ///
    /// That is not a convenience but the only way to express it: both
    /// [`Self::send_frame_async`] and [`Self::receive_frames_async`] take
    /// `&mut self`, so with the driver whole, a transmit can only happen
    /// by cancelling a parked receive — dropping a transfer the chip may
    /// be part-way through answering, and losing the frame with it. The
    /// two bulk endpoints are separate pipes and the controller has
    /// eight host channels to schedule them on; this lets a caller use
    /// them that way.
    ///
    /// Register access needs the driver whole, so anything reached that
    /// way ([`Self::start_async`], [`Self::is_link_up_async`]) happens
    /// either side of the split rather than during it.
    pub fn split(&mut self) -> (Lan9514Rx<'_>, Lan9514Tx<'_>) {
        let endpoint = self.endpoint;
        (
            Lan9514Rx {
                endpoint,
                rx: &mut self.rx,
            },
            Lan9514Tx {
                endpoint,
                tx: &mut self.tx,
            },
        )
    }

    /// Sends one Ethernet frame, awaiting the bulk OUT that carries it —
    /// the async twin of [`Lan9514::send_frame`], with the same framing
    /// and the same size limit on `frame`.
    pub async fn send_frame_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        frame: &[u8],
    ) -> Result<(), TransferError> {
        self.tx
            .send_async(self.endpoint, channel, timer, frame)
            .await
    }

    /// Awaits received Ethernet frames (destination MAC through payload,
    /// CRC stripped), returning every one the transfer carried. They
    /// borrow this driver's RX buffer until the next call.
    ///
    /// **Drain it.** One transfer routinely carries several frames and
    /// the next call overwrites the buffer, so taking the first and
    /// asking again silently discards the rest; see [`Frames`].
    ///
    /// Unlike [`Lan9514::receive_frames`] this does **not** read
    /// `RX_FIFO_INF` first to find out whether a frame is waiting, and
    /// that difference is the point of the whole async path. A bulk IN
    /// against an empty receive FIFO is answered with a NAK, and the
    /// DWC2 retries a NAK'd bulk transfer in hardware without halting
    /// the channel. For the blocking twin that is a disaster — it would
    /// spin out its entire transfer timeout on every idle poll, which is
    /// exactly why the pre-check is there. Here it is the mechanism: the
    /// transfer simply stays parked, costing nothing but a host channel,
    /// until the chip has a frame and the channel halts. The receive
    /// becomes interrupt-driven rather than polled, which is what lets
    /// the `embassy-net` adapter drop the periodic wake the application
    /// used to have to supply.
    ///
    /// It follows that this has no timeout and will wait indefinitely on
    /// an idle network — as every async transfer does; see
    /// [`crate::usb::dwc2::asynch`]. Impose a deadline by dropping the
    /// future. Doing so aborts the channel, which is safe, but a frame
    /// the chip was mid-way through handing over is lost with it.
    ///
    /// An empty iterator means the transfer produced nothing usable — a
    /// zero-length or truncated answer, or a frame the chip flagged as
    /// errored. Ask again.
    pub async fn receive_frames_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<Frames<'_>, TransferError> {
        self.rx.receive_async(self.endpoint, channel, timer).await
    }

    /// Async [`Lan9514::phy_read`].
    async fn phy_read_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        index: u8,
    ) -> Result<u16, TransferError> {
        self.phy_wait_not_busy_async(channel, timer).await?;
        let mii_address = (PHY_ID_INTERNAL << 11) | ((index as u32) << 6);
        self.write_register_async(channel, timer, REG_MII_ADDR, mii_address | MII_BUSY)
            .await?;
        self.phy_wait_not_busy_async(channel, timer).await?;
        Ok(self
            .read_register_async(channel, timer, REG_MII_DATA)
            .await? as u16)
    }

    /// Async [`Lan9514::phy_wait_not_busy`], bounded by the same
    /// [`MII_TIMEOUT_US`]. The wall clock stays here: this is waiting on
    /// the *PHY*, which reports its progress only in a register, so
    /// there is nothing to await but the next read of it.
    async fn phy_wait_not_busy_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<(), TransferError> {
        let start = timer.now_micros();
        while self
            .read_register_async(channel, timer, REG_MII_ADDR)
            .await?
            & MII_BUSY
            != 0
        {
            if timer.now_micros() - start > MII_TIMEOUT_US {
                return Err(TransferError::Timeout);
            }
        }
        Ok(())
    }
}
