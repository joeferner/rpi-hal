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

use embassy_futures::join::join;
use embassy_net_driver::{HardwareAddress, LinkState};
use embassy_net_driver_channel as ch;
use rpi_hal::timer::Timer;
use rpi_hal::usb::dwc2::Channel;
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
/// is simply asked again. There is nowhere to report it to, the stack's
/// recovery is the same either way, and stopping the loop would take the
/// interface down for what is routinely a transient.
async fn receive_forever(
    mut runner: ch::RxRunner<'_, MTU>,
    mut rx: Lan9514Rx<'_>,
    channel: &mut Channel<'_>,
    timer: &Timer,
) -> ! {
    loop {
        // Claimed before the transfer is issued, so backpressure is taken
        // by leaving the frame in the chip's FIFO rather than by fetching
        // one there is nowhere to put.
        let buf = runner.rx_buf().await;
        if let Ok(Some(frame)) = rx.receive_frame_async(channel, timer).await {
            let len = frame.len().min(buf.len());
            buf[..len].copy_from_slice(&frame[..len]);
            runner.rx_done(len);
        }
    }
}

/// Sends every frame the stack queues.
///
/// A failed send is dropped: there is no channel to report it on, and
/// retransmission belongs to a higher layer. The buffer is released
/// either way, or the queue would wedge on the first failure.
async fn transmit_forever(
    mut runner: ch::TxRunner<'_, MTU>,
    mut tx: Lan9514Tx<'_>,
    channel: &mut Channel<'_>,
    timer: &Timer,
) -> ! {
    loop {
        let buf = runner.tx_buf().await;
        let _ = tx.send_frame_async(channel, timer, buf).await;
        runner.tx_done();
    }
}
