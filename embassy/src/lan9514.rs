//! `embassy-net` adapter over `rpi-hal`'s LAN9514 USB-Ethernet driver: a
//! queue pair for the stack
//! ([`Lan9514Driver`](crate::lan9514::Lan9514Driver)) and a task that
//! moves frames between those queues and the chip's bulk endpoints
//! ([`Lan9514Runner`](crate::lan9514::Lan9514Runner)). Build both with
//! [`new`](crate::lan9514::new).
//!
//! # Why it is split in two
//!
//! `embassy_net_driver::Driver`'s `receive`/`transmit` are synchronous —
//! they return tokens and take a `Context`, so there is nowhere inside
//! them to `.await`. A `Driver` implemented directly over the chip
//! therefore could not use the driver's `async` methods at all, however
//! async those methods were; it would have to do its USB work with the
//! blocking ones, holding the executor for the length of every transfer.
//!
//! `embassy-net-driver-channel` is the way out, and the one `cyw43` and
//! friends take: the `Driver` the stack sees becomes a pair of packet
//! queues, which need no `.await` to serve, and the USB work moves into a
//! task on the far side of them, free to await as much as it likes.
//!
//! # Why two host channels
//!
//! [`new`](crate::lan9514::new) takes two `Channel`s, and the runner
//! keeps a receive parked on one for as long as the network is idle. A
//! single channel would have to be taken away from that parked receive
//! every time the stack wanted to transmit — cancelling a transfer the
//! chip may be part-way through answering, and losing the frame with it.
//! Two channels let the directions proceed independently, which is how
//! the endpoints are meant to be driven: they are separate pipes, and the
//! controller schedules eight of them.
//!
//! # What an application must provide
//!
//! The runner resolves nothing without the USB controller's interrupt.
//! On top of the wiring the crate root already asks for, an application
//! using this must enable that interrupt (`Lic::enable_usb_irq`,
//! `rpi_hal::irq::enable_irq`) and dispatch it from its `__irq_handler`
//! to `rpi_hal::usb::dwc2::on_irq`. Without it the runner's receive parks
//! forever and no frame is ever delivered.
//!
//! That is the whole of the contract. In particular there is nothing to
//! poll: the receive is interrupt-driven, so no ticker, no wake-up
//! interval, and no latency-versus-wake-ups trade for the application to
//! guess at.
//!
//! Available only with the `embassy-net-driver` feature enabled.

use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};
use embassy_futures::join::join;
use embassy_net_driver::{HardwareAddress, LinkState};
use embassy_net_driver_channel as ch;
use rpi_hal::timer::Timer;
use rpi_hal::usb::dwc2::{Channel, TransferError};
use rpi_hal::usb::lan9514::{Lan9514, Lan9514Rx, Lan9514Tx, MTU};

/// The packet queues [`new`] builds a
/// [`Lan9514Driver`] and [`Lan9514Runner`] out of, with room for `N_RX`
/// received and `N_TX` outbound frames.
///
/// An application owns this — typically in a `StaticCell` — because both
/// halves borrow from it for as long as the network stack runs. Depth is
/// the application's call: more buffers absorb a longer burst, at one
/// [`MTU`] each.
pub struct Lan9514State<const N_RX: usize, const N_TX: usize> {
    inner: ch::State<MTU, N_RX, N_TX>,
}

impl<const N_RX: usize, const N_TX: usize> Lan9514State<N_RX, N_TX> {
    /// Creates the queues, empty. `const`, so it can initialise a
    /// `static` directly.
    pub const fn new() -> Self {
        Self {
            inner: ch::State::new(),
        }
    }
}

impl<const N_RX: usize, const N_TX: usize> Default for Lan9514State<N_RX, N_TX> {
    /// The same as [`Lan9514State::new`].
    fn default() -> Self {
        Self::new()
    }
}

/// The `embassy_net_driver::Driver` half — what goes to
/// `embassy_net::new`.
///
/// It is the queue pair itself, and is synchronous by design: the frames
/// it hands the stack were fetched by [`Lan9514Runner`] before the stack
/// ever asked, which is the whole reason the split exists.
pub type Lan9514Driver<'d> = ch::Device<'d, MTU>;

/// The half that talks to the hardware: an endless task moving frames
/// between [`Lan9514Driver`]'s queues and the chip's bulk endpoints.
///
/// Created by [`new`] and driven by [`run`](Lan9514Runner::run), which an
/// application spawns as its own task.
pub struct Lan9514Runner<'d, 'c> {
    runner: ch::Runner<'d, MTU>,
    lan9514: Lan9514,
    rx_channel: Channel<'c>,
    tx_channel: Channel<'c>,
    timer: &'d Timer,
}

/// Wraps an already-started LAN9514 as an `embassy-net` device,
/// returning the [`Lan9514Driver`] to hand to `embassy_net::new` and the
/// [`Lan9514Runner`] to spawn.
///
/// `rx_channel` and `tx_channel` are two host channels the runner takes
/// for itself, one per direction — see this module's documentation for
/// why it is two and not one. They are owned rather than borrowed so the
/// controller's remaining channels stay free for other endpoints (a
/// keyboard polled from another task, say) while the stack runs.
///
/// `mac` must be the address `Lan9514::start` was given: the driver
/// programs it into the chip but doesn't retain it, and `embassy-net`
/// needs it to answer ARP.
pub fn new<'d, 'c, const N_RX: usize, const N_TX: usize>(
    state: &'d mut Lan9514State<N_RX, N_TX>,
    lan9514: Lan9514,
    rx_channel: Channel<'c>,
    tx_channel: Channel<'c>,
    timer: &'d Timer,
    mac: [u8; 6],
) -> (Lan9514Driver<'d>, Lan9514Runner<'d, 'c>) {
    let (runner, device) = ch::new(&mut state.inner, HardwareAddress::Ethernet(mac));
    (
        device,
        Lan9514Runner {
            runner,
            lan9514,
            rx_channel,
            tx_channel,
            timer,
        },
    )
}

impl Lan9514Runner<'_, '_> {
    /// Runs the two frame loops until the end of time.
    ///
    /// Receive and transmit are joined rather than selected between:
    /// each owns its own host channel, so neither has to be cancelled to
    /// let the other proceed, and a receive parked on a quiet network
    /// costs nothing while transmits go out past it.
    pub async fn run(self) -> ! {
        let Self {
            runner,
            mut lan9514,
            mut rx_channel,
            mut tx_channel,
            timer,
        } = self;

        let (state_runner, rx_runner, tx_runner) = runner.split();

        // Reported up once and left there. The real state is readable —
        // `Lan9514::is_link_up_async` asks the PHY over MII — but that
        // costs a pair of USB control transfers, and `embassy-net` would
        // want it on every pass of its runner. The consequence is that an
        // unplugged cable surfaces as transfers failing rather than as a
        // link-down transition, and `Stack::wait_link_up` returns
        // immediately. An application that needs better calls
        // `is_link_up_async` on its own schedule, which is also the only
        // place that knows how often is often enough.
        state_runner.set_link_state(LinkState::Up);

        let (rx, tx) = lan9514.split();
        join(
            receive_forever(rx_runner, rx, &mut rx_channel, timer),
            transmit_forever(tx_runner, tx, &mut tx_channel, timer),
        )
        .await
        .0
    }
}

/// Hands every frame the chip produces to the stack's receive queue.
///
/// A transfer that yields nothing usable — an empty answer, or an error —
/// is simply asked again. The stack's recovery is the same either way, and
/// stopping the loop would take the interface down for what is routinely a
/// transient.
///
/// Both outcomes are counted rather than merely ignored; see [`rx_stats`].
async fn receive_forever(
    mut runner: ch::RxRunner<'_, MTU>,
    mut rx: Lan9514Rx<'_>,
    channel: &mut Channel<'_>,
    timer: &Timer,
) -> ! {
    // When the previous transfer completed. Between that instant and the
    // next submission below, *no bulk IN is pending* and an arriving frame
    // has nowhere to go but the chip's FIFO — so the width of that window
    // is the thing that decides whether frames survive. See [`rx_stats`].
    let mut completed_at: Option<u64> = None;

    loop {
        if let Some(previous) = completed_at {
            record_gap(timer.now_micros().wrapping_sub(previous));
        }

        // One transfer, then every frame out of it. The slot is claimed
        // per frame rather than once before the transfer, because the
        // number of frames is not known until it arrives — and claiming
        // one in advance would cover only the first, leaving the rest to
        // be dropped exactly as they were before this loop was a loop.
        //
        // Backpressure still lands in the right place. A full queue parks
        // this task in `rx_buf`, no further transfer is issued while it
        // waits, and frames accumulate in the chip's FIFO — one
        // transfer's worth later than they used to, which is the whole
        // difference.
        let outcome = rx.receive_frames_async(channel, timer).await;
        completed_at = Some(timer.now_micros());

        match outcome {
            Ok(frames) => {
                let mut delivered = 0u32;
                for frame in frames {
                    let buf = runner.rx_buf().await;
                    let len = frame.len().min(buf.len());
                    buf[..len].copy_from_slice(&frame[..len]);
                    runner.rx_done(len);
                    delivered += 1;
                }
                if delivered == 0 {
                    RX_UNUSABLE.fetch_add(1, Ordering::Relaxed);
                } else {
                    RX_FRAMES.fetch_add(delivered, Ordering::Relaxed);
                    if delivered > 1 {
                        RX_BATCHED.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
            Err(error) => {
                RX_FAILURES.fetch_add(1, Ordering::Relaxed);
                RX_LAST_ERROR.store(error_code(error), Ordering::Relaxed);
            }
        }
    }
}

/// Frames [`receive_forever`] has delivered to the stack since boot.
static RX_FRAMES: AtomicU32 = AtomicU32::new(0);

/// Transfers that produced no usable frame — see [`RxStats::unusable`].
static RX_UNUSABLE: AtomicU32 = AtomicU32::new(0);

/// Transfers that carried more than one frame — see [`RxStats::batched`].
static RX_BATCHED: AtomicU32 = AtomicU32::new(0);

/// Widest gap yet seen with no receive transfer pending, in microseconds.
static GAP_MAX_US: AtomicU32 = AtomicU32::new(0);

/// Gaps wider than [`GAP_LONG_US`].
static GAP_LONG: AtomicU32 = AtomicU32::new(0);

/// Gaps at least this wide are counted, not just recorded as a maximum.
///
/// A hundred microseconds is about ten frames' worth of arrival time at
/// 100 Mbit for the small frames this matters for — acknowledgements. Below
/// that a gap cannot plausibly overrun the chip; above it, it can.
const GAP_LONG_US: u64 = 100;

/// Notes one gap between a transfer completing and the next being issued.
///
/// A maximum and a count rather than a mean: the loss this measures is
/// caused by the *worst* gaps, and an average dominated by thousands of
/// microsecond-wide ones would hide exactly the outliers that matter.
fn record_gap(microseconds: u64) {
    let gap = u32::try_from(microseconds).unwrap_or(u32::MAX);
    GAP_MAX_US.fetch_max(gap, Ordering::Relaxed);
    if microseconds >= GAP_LONG_US {
        GAP_LONG.fetch_add(1, Ordering::Relaxed);
    }
}

/// Receive transfers that failed outright.
static RX_FAILURES: AtomicU32 = AtomicU32::new(0);

/// [`error_code`] of the most recent receive failure, or [`NO_ERROR`].
static RX_LAST_ERROR: AtomicU8 = AtomicU8::new(NO_ERROR);

/// What the receive loop has managed since boot.
///
/// Returned by [`rx_stats`].
#[derive(Clone, Copy, Debug)]
pub struct RxStats {
    /// Frames handed to the stack.
    ///
    /// Worth comparing against what the peer believes it sent. A shortfall
    /// is frames that reached the chip and never reached the stack, and
    /// there is no other way to see them: the peer's retransmissions make
    /// the connection work, so nothing fails, and the only symptom is
    /// latency.
    pub frames: u32,
    /// Transfers that returned successfully carrying nothing usable — a
    /// zero-length or truncated answer, or a frame the chip itself flagged
    /// as errored.
    ///
    /// Distinct from [`Self::failures`], where the transfer itself did not
    /// complete.
    pub unusable: u32,
    /// Transfers that carried more than one frame.
    ///
    /// The number that says whether coalescing is happening at all, and so
    /// whether draining a transfer is doing anything. Every frame after the
    /// first in each of these used to be discarded silently — the chip had
    /// handed them over, and only the head was parsed.
    pub batched: u32,
    /// Receive transfers that failed outright.
    pub failures: u32,
    /// Why the most recent failure failed, or `None` if none has.
    pub last_error: Option<TransferError>,
    /// Widest gap seen with no receive transfer pending, in microseconds.
    ///
    /// **The number that decides whether inbound frames survive.** This
    /// adapter keeps exactly one bulk IN outstanding, so between a
    /// transfer completing and the next being issued the chip has nowhere
    /// to put an arriving frame but its own FIFO. Nothing reports an
    /// overrun of it: the frames simply never arrive, and the peer waits
    /// out a retransmission timeout.
    ///
    /// Worth comparing against how long the gap *should* be. The work
    /// between two transfers is a memcpy per frame, which is
    /// microseconds; anything much larger is this task waiting its turn
    /// on the executor behind other work, and no amount of reordering
    /// inside the loop will help — the fix for that is keeping more than
    /// one transfer in flight, as Linux's usbnet does with a queue of
    /// about forty.
    pub gap_max_us: u32,
    /// How many gaps were at least 100 µs wide — roughly ten small frames'
    /// arrival time at 100 Mbit, and so the point at which one can
    /// plausibly cost a frame.
    pub gaps_long: u32,
}

/// What [`receive_forever`] has managed since boot.
///
/// The counterpart to [`tx_stats`], and for the same reason: a frame lost on
/// the way in is as invisible as one lost on the way out, and it is the
/// harder of the two to reason about from outside. A lost acknowledgement
/// does not lose data — it stalls the sender until its retransmission timer
/// fires, so the cost lands on a connection that is working perfectly and
/// looks it.
pub fn rx_stats() -> RxStats {
    RxStats {
        frames: RX_FRAMES.load(Ordering::Relaxed),
        unusable: RX_UNUSABLE.load(Ordering::Relaxed),
        batched: RX_BATCHED.load(Ordering::Relaxed),
        failures: RX_FAILURES.load(Ordering::Relaxed),
        last_error: error_from_code(RX_LAST_ERROR.load(Ordering::Relaxed)),
        gap_max_us: GAP_MAX_US.load(Ordering::Relaxed),
        gaps_long: GAP_LONG.load(Ordering::Relaxed),
    }
}

/// Sends every frame the stack queues.
///
/// A failed send is dropped, and the buffer released either way, or the
/// queue would wedge on the first failure. Retransmission is left to the
/// layer above, which owns a timer for it.
///
/// That dropped frame is counted rather than merely discarded — see
/// [`tx_stats`] for why a silent one is worth this much bookkeeping.
async fn transmit_forever(
    mut runner: ch::TxRunner<'_, MTU>,
    mut tx: Lan9514Tx<'_>,
    channel: &mut Channel<'_>,
    timer: &Timer,
) -> ! {
    loop {
        let buf = runner.tx_buf().await;
        let outcome = tx.send_frame_async(channel, timer, buf).await;

        TX_FRAMES.fetch_add(1, Ordering::Relaxed);
        if let Err(error) = outcome {
            TX_FAILURES.fetch_add(1, Ordering::Relaxed);
            TX_LAST_ERROR.store(error_code(error), Ordering::Relaxed);
        }

        runner.tx_done();
    }
}

/// Frames [`transmit_forever`] has handed to the chip since boot.
static TX_FRAMES: AtomicU32 = AtomicU32::new(0);

/// How many of those the chip refused.
static TX_FAILURES: AtomicU32 = AtomicU32::new(0);

/// [`error_code`] of the most recent failure, or [`NO_ERROR`].
static TX_LAST_ERROR: AtomicU8 = AtomicU8::new(NO_ERROR);

/// [`TX_LAST_ERROR`] before anything has failed.
///
/// Zero rather than a variant's code so that "nothing has gone wrong" is the
/// value the counter starts at.
const NO_ERROR: u8 = 0;

/// What the transmit loop has managed since boot.
///
/// Returned by [`tx_stats`]. Counters, not a rate: an application decides
/// what interval to compare two readings over.
#[derive(Clone, Copy, Debug)]
pub struct TxStats {
    /// Frames the loop attempted to send.
    pub frames: u32,
    /// How many of those the chip refused, and which this adapter therefore
    /// dropped.
    ///
    /// **Not the same as frames lost on the network.** A drop here is a
    /// frame that never reached the wire at all, and nothing below TCP will
    /// notice: the peer waits out a retransmission timeout, which on a LAN
    /// is orders of magnitude longer than the transfer would have taken. A
    /// handful of these is enough to make a page load visibly stall while
    /// every other indicator — link state, throughput, error counters on the
    /// switch — looks perfectly healthy.
    pub failures: u32,
    /// Why the most recent failure failed, or `None` if none has.
    ///
    /// Which variant it is decides what to do about it, so it is worth more
    /// than the count on its own. A [`TransferError::Timeout`] or
    /// [`TransferError::NakTimeout`] is the chip declining to accept a frame
    /// it has no room for, and a caller that waited and asked again would
    /// have got it through. A [`TransferError::DataToggleError`] is host and
    /// device disagreeing about the bulk endpoint's toggle, which retrying
    /// cannot fix and which needs the endpoint resynchronised.
    pub last_error: Option<TransferError>,
}

/// What [`transmit_forever`] has managed since boot.
///
/// Exists because a frame this adapter drops is otherwise invisible. The
/// stack is told the send succeeded, the chip never sees the frame, and the
/// only trace is the peer's retransmission timeout some hundreds of
/// milliseconds later — so the symptom presents as latency with no
/// corresponding error anywhere, which is a hard thing to go looking for
/// without a number to look at.
///
/// Cheap enough to leave enabled: three relaxed atomic operations per frame,
/// against a USB transfer.
pub fn tx_stats() -> TxStats {
    TxStats {
        frames: TX_FRAMES.load(Ordering::Relaxed),
        failures: TX_FAILURES.load(Ordering::Relaxed),
        last_error: error_from_code(TX_LAST_ERROR.load(Ordering::Relaxed)),
    }
}

/// Packs a [`TransferError`] into a byte, so the last one can live in an
/// atomic.
///
/// A `match` rather than a `as` cast: [`TransferError`] carries no explicit
/// discriminants, so casting would silently renumber the codes if a variant
/// were ever inserted, and a stored code would then decode as the wrong
/// error.
fn error_code(error: TransferError) -> u8 {
    match error {
        TransferError::Stall => 1,
        TransferError::TransactionError => 2,
        TransferError::Babble => 3,
        TransferError::Nak => 4,
        TransferError::NakTimeout => 5,
        TransferError::FrameOverrun => 6,
        TransferError::DataToggleError => 7,
        TransferError::Halted => 8,
        TransferError::Timeout => 9,
    }
}

/// The inverse of [`error_code`]. Anything unrecognised — including
/// [`NO_ERROR`] — is `None`.
fn error_from_code(code: u8) -> Option<TransferError> {
    Some(match code {
        1 => TransferError::Stall,
        2 => TransferError::TransactionError,
        3 => TransferError::Babble,
        4 => TransferError::Nak,
        5 => TransferError::NakTimeout,
        6 => TransferError::FrameOverrun,
        7 => TransferError::DataToggleError,
        8 => TransferError::Halted,
        9 => TransferError::Timeout,
        _ => return None,
    })
}
