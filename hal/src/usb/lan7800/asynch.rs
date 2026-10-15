//! Interrupt-driven `async` twins of [`Lan7800`]'s transfer methods,
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
//! [`Lan7800::start_async`] configures the chip's empty-receive response
//! the *opposite* way to [`Lan7800::start`], so that a receive can be left
//! parked on an idle link. See that method — it is the one place where
//! picking the wrong twin produces a working program that behaves badly
//! rather than one that fails.
//!
//! [`Lan7800::split`] is here for the same reason it is on the sibling
//! driver. The chip's two bulk endpoints are independent pipes, and a
//! driver that parks on the receive one has to be able to transmit past
//! it; that is only expressible if the two directions can be borrowed
//! apart.
//!
//! Enumeration ([`Lan7800::from_device`], [`Lan7800::from_endpoint`]) has
//! no twin. It runs once, during bring-up, before there is an executor
//! with anything else to do — the case the blocking path is already the
//! right shape for.

use super::{
    Frames, IdRevision, Lan7800, Rx, Tx, AN_100_FULL, AN_10_FULL, BMCR_ANEG_ENABLE,
    BMCR_ANEG_RESTART, BMCR_ISOLATE, BMCR_POWER_DOWN, BMSR_LINK_UP, FCT_RX_CTL_EN, FCT_TX_CTL_EN,
    HW_CFG_CLK125_EN, HW_CFG_LED0_EN, HW_CFG_LED1_EN, HW_CFG_LRST, HW_CFG_MEF, HW_CFG_REFCLK25_EN,
    MAC_CR_AUTO_DUPLEX, MAC_CR_AUTO_SPEED, MAC_CR_EEE_EN, MAC_CR_GMII_EN, MAC_RX_MAX_FRAME_SIZE,
    MAC_RX_MAX_SIZE_MASK, MAC_RX_MAX_SIZE_SHIFT, MAC_RX_RXEN, MAC_TX_TXEN, MAF_HI_VALID,
    MII_ACC_BUSY, MII_ACC_PHY_ADDR_SHIFT, MII_ACC_REG_SHIFT, MII_ACC_WRITE, MII_TIMEOUT_US,
    PHY_ID_INTERNAL, PHY_REG_ADVERTISE, PHY_REG_CONTROL, PHY_REG_GIGABIT_CONTROL,
    PHY_REG_LINK_PARTNER, PHY_REG_STATUS, PMT_CTL_PHY_RST, PMT_CTL_READY, READ_REGISTER,
    REG_FCT_FLOW, REG_FCT_RX_CTL, REG_FCT_TX_CTL, REG_FLOW, REG_HW_CFG, REG_ID_REV, REG_INT_STS,
    REG_MAC_CR, REG_MAC_RX, REG_MAC_TX, REG_MAF_HI0, REG_MAF_LO0, REG_MII_ACC, REG_MII_DATA,
    REG_PMT_CTL, REG_RFE_CTL, REG_RX_ADDRH, REG_RX_ADDRL, REG_USB_CFG0, RESET_TIMEOUT_US,
    RFE_CTL_BCAST_EN, RFE_CTL_DA_PERFECT, RFE_CTL_MCAST_EN, RFE_CTL_UCAST_EN, USB_CFG_BIR,
    WRITE_REGISTER,
};
use crate::timer::Timer;
use crate::usb::control::{vendor_in_async, vendor_out_async};
use crate::usb::dwc2::{Channel, ControlEndpoint, TransferError};

/// The receive direction of a [`Lan7800`], borrowed from it by
/// [`Lan7800::split`]: its bulk IN endpoint and the frame buffer that
/// endpoint's transfers land in.
///
/// Available only with the `async` feature enabled.
pub struct Lan7800Rx<'a> {
    endpoint: ControlEndpoint,
    rx: &'a mut Rx,
}

/// The transmit direction of a [`Lan7800`] — the counterpart to
/// [`Lan7800Rx`], and produced by the same [`Lan7800::split`].
///
/// Available only with the `async` feature enabled.
pub struct Lan7800Tx<'a> {
    endpoint: ControlEndpoint,
    tx: &'a mut Tx,
}

impl Rx {
    /// Awaits one bulk IN and decodes what arrives. `device` is the
    /// chip's endpoint 0, which carries the address and speed the bulk
    /// endpoint shares.
    ///
    /// Shared by [`Lan7800::receive_frames_async`] and
    /// [`Lan7800Rx::receive_frames_async`] rather than either delegating
    /// to the other: the returned frame borrows this buffer, so a
    /// delegation through a temporary [`Lan7800Rx`] would tie it to the
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

impl Lan7800Rx<'_> {
    /// Awaits received Ethernet frames on the bulk IN endpoint —
    /// [`Lan7800::receive_frames_async`] restricted to this direction,
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

impl Lan7800Tx<'_> {
    /// Sends one Ethernet frame out the bulk OUT endpoint —
    /// [`Lan7800::send_frame_async`] restricted to this direction.
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

impl Lan7800 {
    /// Async [`Lan7800::read_register`].
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

    /// Async [`Lan7800::write_register`].
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

    /// Async [`Lan7800::id_revision`].
    pub async fn id_revision_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<IdRevision, TransferError> {
        let id_rev = self.read_register_async(channel, timer, REG_ID_REV).await?;
        Ok(IdRevision {
            id: (id_rev >> 16) as u16,
            revision: id_rev as u16,
        })
    }

    /// Async [`Lan7800::set_mac_address`], writing both the station
    /// address and perfect-filter entry 0 for the same reason.
    pub async fn set_mac_address_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        mac: [u8; 6],
    ) -> Result<(), TransferError> {
        let low = u32::from_le_bytes([mac[0], mac[1], mac[2], mac[3]]);
        let high = u16::from_le_bytes([mac[4], mac[5]]) as u32;

        self.write_register_async(channel, timer, REG_RX_ADDRL, low)
            .await?;
        self.write_register_async(channel, timer, REG_RX_ADDRH, high)
            .await?;

        self.write_register_async(channel, timer, REG_MAF_LO0, low)
            .await?;
        self.write_register_async(channel, timer, REG_MAF_HI0, high | MAF_HI_VALID)
            .await?;
        Ok(())
    }

    /// Async [`Lan7800::mac_address`].
    pub async fn mac_address_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<[u8; 6], TransferError> {
        let low = self
            .read_register_async(channel, timer, REG_RX_ADDRL)
            .await?
            .to_le_bytes();
        let high = (self
            .read_register_async(channel, timer, REG_RX_ADDRH)
            .await? as u16)
            .to_le_bytes();
        Ok([low[0], low[1], low[2], low[3], high[0], high[1]])
    }

    /// Async [`Lan7800::reset`], with one deliberate difference: the
    /// chip's answer to a bulk IN with no frame waiting.
    ///
    /// The blocking path *clears* `USB_CFG0.BIR`, so an empty receive FIFO
    /// is answered with a zero-length packet and an idle poll returns at
    /// once. This sets it, so an empty FIFO NAKs instead — and that is
    /// what makes the async receive worth having. The DWC2 retries a
    /// NAK'd bulk transfer in hardware without halting the channel, so the
    /// transfer simply stays parked, costing nothing but a host channel,
    /// until the chip has a frame and the channel halts. The receive
    /// becomes interrupt-driven rather than polled.
    ///
    /// Picking the wrong twin is therefore not a compile error and not a
    /// failure either: [`Lan7800::start`] followed by
    /// [`Lan7800::receive_frames_async`] gives a future that resolves
    /// immediately with nothing, every time, and a caller looping on it
    /// spins the executor instead of sleeping.
    ///
    /// The sibling [`crate::usb::lan9514`] sets its bit of the same name
    /// in *both* of its paths, and its blocking receive pays for that by
    /// asking a FIFO-level register whether a frame is waiting before it
    /// issues a transfer at all. This chip offers no equivalent register
    /// this driver has definitions for, so the choice moves into the
    /// bring-up instead.
    pub async fn reset_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<(), TransferError> {
        let hw_cfg = self.read_register_async(channel, timer, REG_HW_CFG).await?;
        self.write_register_async(channel, timer, REG_HW_CFG, hw_cfg | HW_CFG_LRST)
            .await?;
        self.wait_clear_async(channel, timer, REG_HW_CFG, HW_CFG_LRST)
            .await?;

        // NAK an empty bulk IN rather than answering it, so a receive can
        // be parked on an idle link — the opposite of the blocking path.
        let usb_cfg = self
            .read_register_async(channel, timer, REG_USB_CFG0)
            .await?;
        self.write_register_async(channel, timer, REG_USB_CFG0, usb_cfg | USB_CFG_BIR)
            .await?;

        let hw_cfg = self.read_register_async(channel, timer, REG_HW_CFG).await?;
        self.write_register_async(
            channel,
            timer,
            REG_HW_CFG,
            (hw_cfg | HW_CFG_CLK125_EN | HW_CFG_REFCLK25_EN | HW_CFG_LED0_EN | HW_CFG_LED1_EN)
                & !HW_CFG_MEF,
        )
        .await?;

        self.write_register_async(channel, timer, REG_INT_STS, u32::MAX)
            .await?;

        self.write_register_async(channel, timer, REG_FLOW, 0)
            .await?;
        self.write_register_async(channel, timer, REG_FCT_FLOW, 0)
            .await?;

        self.write_register_async(
            channel,
            timer,
            REG_RFE_CTL,
            RFE_CTL_BCAST_EN | RFE_CTL_DA_PERFECT,
        )
        .await?;

        let pmt_ctl = self
            .read_register_async(channel, timer, REG_PMT_CTL)
            .await?;
        self.write_register_async(channel, timer, REG_PMT_CTL, pmt_ctl | PMT_CTL_PHY_RST)
            .await?;
        self.wait_phy_ready_async(channel, timer).await?;

        let mac_cr = self.read_register_async(channel, timer, REG_MAC_CR).await?;
        self.write_register_async(
            channel,
            timer,
            REG_MAC_CR,
            (mac_cr | MAC_CR_AUTO_DUPLEX | MAC_CR_AUTO_SPEED | MAC_CR_GMII_EN) & !MAC_CR_EEE_EN,
        )
        .await?;

        let mac_rx = self.read_register_async(channel, timer, REG_MAC_RX).await?;
        self.write_register_async(
            channel,
            timer,
            REG_MAC_RX,
            (mac_rx & !MAC_RX_MAX_SIZE_MASK)
                | ((MAC_RX_MAX_FRAME_SIZE << MAC_RX_MAX_SIZE_SHIFT) & MAC_RX_MAX_SIZE_MASK),
        )
        .await?;

        self.start_autonegotiation_async(channel, timer).await?;
        Ok(())
    }

    /// Async [`Lan7800::start`], programming the same registers in the
    /// same order — and inheriting [`Self::reset_async`]'s one difference,
    /// which is worth reading before choosing between the two.
    pub async fn start_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        mac: [u8; 6],
    ) -> Result<(), TransferError> {
        self.reset_async(channel, timer).await?;
        self.set_mac_address_async(channel, timer, mac).await?;

        let fct_rx = self
            .read_register_async(channel, timer, REG_FCT_RX_CTL)
            .await?;
        self.write_register_async(channel, timer, REG_FCT_RX_CTL, fct_rx | FCT_RX_CTL_EN)
            .await?;
        let fct_tx = self
            .read_register_async(channel, timer, REG_FCT_TX_CTL)
            .await?;
        self.write_register_async(channel, timer, REG_FCT_TX_CTL, fct_tx | FCT_TX_CTL_EN)
            .await?;

        let mac_rx = self.read_register_async(channel, timer, REG_MAC_RX).await?;
        self.write_register_async(channel, timer, REG_MAC_RX, mac_rx | MAC_RX_RXEN)
            .await?;
        let mac_tx = self.read_register_async(channel, timer, REG_MAC_TX).await?;
        self.write_register_async(channel, timer, REG_MAC_TX, mac_tx | MAC_TX_TXEN)
            .await?;
        Ok(())
    }

    /// Async [`Lan7800::is_link_up`].
    pub async fn is_link_up_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<bool, TransferError> {
        Ok(self.phy_read_async(channel, timer, PHY_REG_STATUS).await? & BMSR_LINK_UP != 0)
    }

    /// Async [`Lan7800::is_full_duplex`].
    pub async fn is_full_duplex_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<bool, TransferError> {
        let ours = self
            .phy_read_async(channel, timer, PHY_REG_ADVERTISE)
            .await?;
        let theirs = self
            .phy_read_async(channel, timer, PHY_REG_LINK_PARTNER)
            .await?;
        Ok(ours & theirs & (AN_100_FULL | AN_10_FULL) != 0)
    }

    /// Async [`Lan7800::set_all_multicast`]. Read that one for why the
    /// chip's default is worth changing.
    pub async fn set_all_multicast_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        pass: bool,
    ) -> Result<(), TransferError> {
        let rfe_ctl = self
            .read_register_async(channel, timer, REG_RFE_CTL)
            .await?;
        let rfe_ctl = if pass {
            rfe_ctl | RFE_CTL_MCAST_EN
        } else {
            rfe_ctl & !RFE_CTL_MCAST_EN
        };
        self.write_register_async(channel, timer, REG_RFE_CTL, rfe_ctl)
            .await
    }

    /// Async [`Lan7800::set_promiscuous`].
    pub async fn set_promiscuous_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        pass: bool,
    ) -> Result<(), TransferError> {
        let rfe_ctl = self
            .read_register_async(channel, timer, REG_RFE_CTL)
            .await?;
        let rfe_ctl = if pass {
            rfe_ctl | RFE_CTL_UCAST_EN
        } else {
            rfe_ctl & !RFE_CTL_UCAST_EN
        };
        self.write_register_async(channel, timer, REG_RFE_CTL, rfe_ctl)
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
    /// two bulk endpoints are separate pipes and the controller has eight
    /// host channels to schedule them on; this lets a caller use them that
    /// way.
    ///
    /// Register access needs the driver whole, so anything reached that
    /// way ([`Self::start_async`], [`Self::is_link_up_async`]) happens
    /// either side of the split rather than during it.
    pub fn split(&mut self) -> (Lan7800Rx<'_>, Lan7800Tx<'_>) {
        let endpoint = self.endpoint;
        (
            Lan7800Rx {
                endpoint,
                rx: &mut self.rx,
            },
            Lan7800Tx {
                endpoint,
                tx: &mut self.tx,
            },
        )
    }

    /// Sends one Ethernet frame, awaiting the bulk OUT that carries it —
    /// the async twin of [`Lan7800::send_frame`], with the same framing
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
    /// **Drain it.** A transfer can carry several frames and the next call
    /// overwrites the buffer, so taking the first and asking again
    /// silently discards the rest; see [`Frames`].
    ///
    /// This parks rather than polls, which is the whole point of the async
    /// path — but only if the chip was brought up by
    /// [`Lan7800::start_async`] rather than [`Lan7800::start`], because the
    /// two configure the empty-FIFO response differently. See
    /// [`Self::reset_async`], which is where that is decided and where the
    /// consequence of getting it wrong is spelled out.
    ///
    /// It follows that this has no timeout and will wait indefinitely on
    /// an idle network — as every async transfer does; see
    /// [`crate::usb::dwc2::asynch`]. Impose a deadline by dropping the
    /// future. Doing so aborts the channel, which is safe, but a frame the
    /// chip was mid-way through handing over is lost with it.
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

    /// Async [`Lan7800::phy_read`].
    pub async fn phy_read_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        index: u8,
    ) -> Result<u16, TransferError> {
        self.phy_wait_not_busy_async(channel, timer).await?;
        let access = (PHY_ID_INTERNAL << MII_ACC_PHY_ADDR_SHIFT)
            | ((index as u32) << MII_ACC_REG_SHIFT)
            | MII_ACC_BUSY;
        self.write_register_async(channel, timer, REG_MII_ACC, access)
            .await?;
        self.phy_wait_not_busy_async(channel, timer).await?;
        Ok(self
            .read_register_async(channel, timer, REG_MII_DATA)
            .await? as u16)
    }

    /// Async [`Lan7800::phy_write`].
    pub async fn phy_write_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        index: u8,
        value: u16,
    ) -> Result<(), TransferError> {
        self.phy_wait_not_busy_async(channel, timer).await?;
        self.write_register_async(channel, timer, REG_MII_DATA, value as u32)
            .await?;
        let access = (PHY_ID_INTERNAL << MII_ACC_PHY_ADDR_SHIFT)
            | ((index as u32) << MII_ACC_REG_SHIFT)
            | MII_ACC_WRITE
            | MII_ACC_BUSY;
        self.write_register_async(channel, timer, REG_MII_ACC, access)
            .await?;
        self.phy_wait_not_busy_async(channel, timer).await
    }

    /// Async twin of the blocking `start_autonegotiation`, withdrawing
    /// gigabit for the reason given there — it is not a tuning choice, the
    /// link does not come up at all with 1000BASE-T advertised.
    async fn start_autonegotiation_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<(), TransferError> {
        self.phy_write_async(channel, timer, PHY_REG_GIGABIT_CONTROL, 0)
            .await?;

        let control = self.phy_read_async(channel, timer, PHY_REG_CONTROL).await?;
        let control =
            (control | BMCR_ANEG_ENABLE | BMCR_ANEG_RESTART) & !(BMCR_POWER_DOWN | BMCR_ISOLATE);
        self.phy_write_async(channel, timer, PHY_REG_CONTROL, control)
            .await
    }

    /// Async `phy_wait_not_busy`, bounded by the same [`MII_TIMEOUT_US`].
    /// The wall clock stays here: this is waiting on the *PHY*, which
    /// reports its progress only in a register, so there is nothing to
    /// await but the next read of it.
    async fn phy_wait_not_busy_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<(), TransferError> {
        let start = timer.now_micros();
        while self
            .read_register_async(channel, timer, REG_MII_ACC)
            .await?
            & MII_ACC_BUSY
            != 0
        {
            if timer.now_micros() - start > MII_TIMEOUT_US {
                return Err(TransferError::Timeout);
            }
        }
        Ok(())
    }

    /// Async `wait_clear`, for the same self-clearing reset bits and with
    /// the same [`RESET_TIMEOUT_US`] bound.
    async fn wait_clear_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        register: u16,
        bits: u32,
    ) -> Result<(), TransferError> {
        let start = timer.now_micros();
        while self.read_register_async(channel, timer, register).await? & bits != 0 {
            if timer.now_micros() - start > RESET_TIMEOUT_US {
                return Err(TransferError::Timeout);
            }
        }
        Ok(())
    }

    /// Async `wait_phy_ready`: the reset bit self-clears *and* the chip
    /// raises `PMT_CTL.READY`, for the reason the blocking twin gives.
    async fn wait_phy_ready_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<(), TransferError> {
        let start = timer.now_micros();
        loop {
            let pmt_ctl = self
                .read_register_async(channel, timer, REG_PMT_CTL)
                .await?;
            if pmt_ctl & PMT_CTL_PHY_RST == 0 && pmt_ctl & PMT_CTL_READY != 0 {
                return Ok(());
            }
            if timer.now_micros() - start > RESET_TIMEOUT_US {
                return Err(TransferError::Timeout);
            }
        }
    }
}

impl crate::usb::ethernet::EthernetAsync for Lan7800 {
    type Rx<'a> = Lan7800Rx<'a>;
    type Tx<'a> = Lan7800Tx<'a>;

    fn split(&mut self) -> (Self::Rx<'_>, Self::Tx<'_>) {
        Lan7800::split(self)
    }

    async fn start_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        mac: [u8; 6],
    ) -> Result<(), TransferError> {
        Lan7800::start_async(self, channel, timer, mac).await
    }

    async fn is_link_up_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<bool, TransferError> {
        Lan7800::is_link_up_async(self, channel, timer).await
    }

    async fn set_all_multicast_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        pass: bool,
    ) -> Result<(), TransferError> {
        Lan7800::set_all_multicast_async(self, channel, timer, pass).await
    }
}

impl crate::usb::ethernet::EthernetRx for Lan7800Rx<'_> {
    type Frames<'a>
        = Frames<'a>
    where
        Self: 'a;

    async fn receive_frames_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<Self::Frames<'_>, TransferError> {
        Lan7800Rx::receive_frames_async(self, channel, timer).await
    }
}

impl crate::usb::ethernet::EthernetTx for Lan7800Tx<'_> {
    async fn send_frame_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        frame: &[u8],
    ) -> Result<(), TransferError> {
        Lan7800Tx::send_frame_async(self, channel, timer, frame).await
    }
}
