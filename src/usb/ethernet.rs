//! What a USB Ethernet controller looks like from above, independent of
//! which one a board has.
//!
//! Two chips here answer to the same description: the SMSC LAN9514 on a Pi
//! 2B/3B ([`crate::usb::lan9514`]) and the Microchip LAN7800 on a 3B+
//! ([`crate::usb::lan7800`]). They were written to the same shape
//! deliberately, so a consumer ports between them by changing a type name.
//! [`Ethernet`](crate::usb::ethernet::Ethernet) makes that a type rather
//! than a convention — code written
//! against it compiles for either board without being told which.
//!
//! The one thing that could not be expressed without this is the receive
//! iterator. Each driver returns its own `Frames`: the same item type,
//! different types, so a function handling both had nothing to return and
//! had to consume the frames where it produced them. A generic associated
//! type is what actually removes that.
//!
//! This is the blocking surface. The split into borrow-disjoint halves
//! that an executor needs is a separate extension, because one of the two
//! drivers does not have an async half yet.

use crate::timer::Timer;
use crate::usb::dwc2::{Channel, TransferError};

/// A chip's identity register, split into the chip ID and silicon
/// revision it packs into one 32-bit word.
///
/// Shared by both drivers rather than repeated, since it is the same two
/// numbers in the same places; what differs is only which ID means a
/// working part (`0xEC00` for the LAN951x family, `0x7800` for the
/// LAN7800).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IdRevision {
    /// Chip ID — the register's high 16 bits.
    pub id: u16,
    /// Silicon revision — the register's low 16 bits.
    pub revision: u16,
}

/// A USB Ethernet controller that has been found, configured, and can be
/// brought up and moved frames through.
///
/// Implemented by [`lan9514::Lan9514`](crate::usb::lan9514::Lan9514) and
/// [`lan7800::Lan7800`](crate::usb::lan7800::Lan7800). A board that knows
/// which chip it has does not need this; it is for code that should not
/// have to know, which on a Raspberry Pi means anything meant to run on
/// more than one model.
///
/// Every method takes the [`Channel`] to run its transfers on rather than
/// holding one, which is the same arrangement the drivers themselves use:
/// a channel is a scarce host resource, and a driver that kept one would
/// hold it for the life of the interface whether or not it was
/// transferring.
pub trait Ethernet {
    /// The received frames one transfer carried — this driver's own
    /// iterator, borrowing its receive buffer.
    ///
    /// Generic over the borrow because the frames live in the driver's DMA
    /// buffer until the next receive overwrites it. Nothing is copied, so
    /// the iterator cannot outlive the call that produced it.
    type Frames<'a>: Iterator<Item = &'a [u8]>
    where
        Self: 'a;

    /// Largest Ethernet frame this controller carries, in bytes: the
    /// 14-byte header plus payload, without the CRC.
    const MTU: usize;

    /// Reads the chip's identity register — the cheapest confirmation that
    /// register access is reaching it at all.
    fn id_revision(
        &self,
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<IdRevision, TransferError>;

    /// Programs `mac` into the chip and opens the data paths, leaving the
    /// interface ready to move frames once the link comes up.
    fn start(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        mac: [u8; 6],
    ) -> Result<(), TransferError>;

    /// Whether the Ethernet link is up, read from the PHY. `false` until
    /// the cable is connected and auto-negotiation completes, which takes
    /// a second or three from a standing start.
    fn is_link_up(&self, channel: &mut Channel, timer: &Timer) -> Result<bool, TransferError>;

    /// Whether auto-negotiation settled on full duplex. Only meaningful
    /// once [`Self::is_link_up`] returns `true`.
    fn is_full_duplex(&self, channel: &mut Channel, timer: &Timer) -> Result<bool, TransferError>;

    /// Whether to pass every multicast frame up to the host, or filter it.
    ///
    /// **Off is the reset state on both chips, and it is not a neutral
    /// default.** The receiver comes up taking unicast for its own address
    /// plus broadcast and dropping multicast before the host sees it.
    /// Anything speaking only unicast or broadcast never notices — DHCP is
    /// broadcast — and anything else fails completely rather than
    /// partially: mDNS is multicast, so a responder binds a socket nothing
    /// ever arrives on, with no error to explain it.
    fn set_all_multicast(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        pass: bool,
    ) -> Result<(), TransferError>;

    /// Sends one Ethernet frame — destination MAC through payload, without
    /// the CRC, which the chip appends. `frame` must be no longer than
    /// [`Self::MTU`].
    fn send_frame(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        frame: &[u8],
    ) -> Result<(), TransferError>;

    /// Polls for received frames, returning every one the transfer carried
    /// with its CRC stripped.
    ///
    /// **Drain the iterator.** A transfer can carry more than one frame and
    /// the next call overwrites the buffer they borrow, so taking the first
    /// and calling again discards the rest with nothing to report it.
    fn receive_frames(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<Self::Frames<'_>, TransferError>;
}

/// The `async` half of an Ethernet controller: the two frame directions
/// borrowed apart, so each can be driven independently.
///
/// Separate from [`Ethernet`] rather than part of it, because the two are
/// not the same promise. [`Ethernet`] describes a chip that can move
/// frames; this describes one whose transfers can be *awaited*, which
/// needs the `async` feature and the interrupt wiring that goes with it
/// (see [`crate::usb::dwc2::asynch`]).
///
/// The split is the reason this exists. Both halves' methods take
/// `&mut self`, so with the driver whole a transmit can only happen by
/// cancelling a parked receive — dropping a transfer the chip may be
/// part-way through answering, and losing the frame with it. The two bulk
/// endpoints are separate pipes and the controller has host channels to
/// spare; this is what lets a caller use them that way, and it is what an
/// `embassy-net` adapter is built on.
///
/// Register access needs the driver whole, so the methods here that reach
/// registers are on the driver rather than on either half, and happen
/// either side of a split rather than during one.
///
/// # Which bring-up to use
///
/// **Use [`Self::start_async`], not [`Ethernet::start`], with the methods
/// here.** A driver may configure the chip's answer to a bulk IN with no
/// frame waiting differently between the two — parking an idle receive
/// needs the chip to NAK, while a blocking poll needs it not to — and the
/// mismatch is neither a compile error nor a failure. It is a receive
/// future that resolves immediately with nothing, forever, and a caller
/// looping on it spinning its executor instead of sleeping. See
/// [`lan7800::Lan7800::reset_async`](crate::usb::lan7800::Lan7800::reset_async),
/// where that is decided for one of the two chips.
#[cfg(feature = "async")]
#[cfg_attr(docsrs, doc(cfg(feature = "async")))]
#[allow(
    async_fn_in_trait,
    reason = "the futures are polled on the core that owns the host channel, so \
              there is nothing for a `Send` bound to buy and adding one would \
              exclude the drivers that hold a raw peripheral handle"
)]
pub trait EthernetAsync: Ethernet {
    /// The receive direction, borrowed from the driver by
    /// [`Self::split`].
    type Rx<'a>: EthernetRx
    where
        Self: 'a;

    /// The transmit direction — the counterpart to [`Self::Rx`].
    type Tx<'a>: EthernetTx
    where
        Self: 'a;

    /// Borrows the two frame directions apart.
    fn split(&mut self) -> (Self::Rx<'_>, Self::Tx<'_>);

    /// Programs `mac` into the chip and opens the data paths, configured
    /// for the awaited transfers here rather than the blocking ones —
    /// see this trait's note on which bring-up to use.
    async fn start_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        mac: [u8; 6],
    ) -> Result<(), TransferError>;

    /// Whether the Ethernet link is up, read from the PHY.
    async fn is_link_up_async(
        &self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<bool, TransferError>;

    /// Whether to pass every multicast frame up to the host — see
    /// [`Ethernet::set_all_multicast`] for why the chip's default is not a
    /// neutral one.
    async fn set_all_multicast_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        pass: bool,
    ) -> Result<(), TransferError>;
}

/// The receive direction of an [`EthernetAsync`] driver.
#[cfg(feature = "async")]
#[cfg_attr(docsrs, doc(cfg(feature = "async")))]
#[allow(
    async_fn_in_trait,
    reason = "see `EthernetAsync`: no executor here moves these futures between cores"
)]
pub trait EthernetRx {
    /// The received frames one transfer carried, borrowing this half's
    /// buffer — the same arrangement as [`Ethernet::Frames`], and
    /// separate from it because this borrows the half rather than the
    /// whole driver.
    type Frames<'a>: Iterator<Item = &'a [u8]>
    where
        Self: 'a;

    /// Awaits received frames, parking until the chip has one rather than
    /// returning empty — which is the whole difference from
    /// [`Ethernet::receive_frames`], and depends on the bring-up this
    /// trait's note describes.
    ///
    /// **Drain the iterator**, for the reason
    /// [`Ethernet::receive_frames`] gives.
    ///
    /// No timeout: this waits indefinitely on an idle link. Impose a
    /// deadline by dropping the future, which aborts the channel safely
    /// but loses a frame the chip was part-way through handing over.
    async fn receive_frames_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
    ) -> Result<Self::Frames<'_>, TransferError>;
}

/// The transmit direction of an [`EthernetAsync`] driver.
#[cfg(feature = "async")]
#[cfg_attr(docsrs, doc(cfg(feature = "async")))]
#[allow(
    async_fn_in_trait,
    reason = "see `EthernetAsync`: no executor here moves these futures between cores"
)]
pub trait EthernetTx {
    /// Sends one Ethernet frame — destination MAC through payload,
    /// without the CRC, which the chip appends.
    async fn send_frame_async(
        &mut self,
        channel: &mut Channel<'_>,
        timer: &Timer,
        frame: &[u8],
    ) -> Result<(), TransferError>;
}
