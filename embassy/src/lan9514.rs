//! The LAN9514 spelling of [`crate::ethernet`].
//!
//! That module used to be this one, and was named for the only chip it
//! could drive. It is now generic over
//! [`EthernetAsync`](rpi_hal::usb::ethernet::EthernetAsync) and serves a
//! Pi 3B+'s LAN7800 as well, so the name became wrong — but boards are
//! written against it, and renaming a module is not worth making every one
//! of them edit an import.
//!
//! Everything here is an alias or a one-line forward. New code should use
//! [`crate::ethernet`] directly; nothing is deprecated, because there is
//! nothing wrong with naming your chip when you know it.

use rpi_hal::timer::Timer;
use rpi_hal::usb::dwc2::Channel;
use rpi_hal::usb::lan9514::Lan9514;

pub use crate::ethernet::{EthernetConfig, MTU, RxStats, TxStats, rx_stats, start_error, tx_stats};

/// The packet queues, under the older name — see
/// [`ethernet::EthernetState`](crate::ethernet::EthernetState).
pub type Lan9514State<const N_RX: usize, const N_TX: usize> =
    crate::ethernet::EthernetState<N_RX, N_TX>;

/// The `embassy_net_driver::Driver` half — see
/// [`ethernet::EthernetDriver`](crate::ethernet::EthernetDriver).
pub type Lan9514Driver<'d> = crate::ethernet::EthernetDriver<'d>;

/// The task half, fixed to the LAN9514 — see
/// [`ethernet::EthernetRunner`](crate::ethernet::EthernetRunner).
pub type Lan9514Runner<'d, 'c> = crate::ethernet::EthernetRunner<'d, 'c, Lan9514>;

/// Wraps a LAN9514 as an `embassy-net` device — see
/// [`ethernet::new`](crate::ethernet::new), which this forwards to.
///
/// **`lan9514` must *not* be started**: the runner brings it up itself.
/// That changed when this module became a forward — callers used to call
/// `start` first — and it is the reason [`EthernetConfig`] carries the MAC and the
/// multicast setting rather than those being programmed beforehand. See
/// [`ethernet::new`](crate::ethernet::new) for why the bring-up moved.
pub fn new<'d, 'c, const N_RX: usize, const N_TX: usize>(
    state: &'d mut Lan9514State<N_RX, N_TX>,
    lan9514: Lan9514,
    rx_channel: Channel<'c>,
    tx_channel: Channel<'c>,
    timer: &'d Timer,
    config: EthernetConfig,
) -> (Lan9514Driver<'d>, Lan9514Runner<'d, 'c>) {
    crate::ethernet::new(state, lan9514, rx_channel, tx_channel, timer, config)
}
