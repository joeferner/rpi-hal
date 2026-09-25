//! Driving a USB hub's downstream ports, built on the hub-class control
//! requests in [`crate::usb::control`]. Wraps the raw `wPortStatus`
//! bitmap ([`PortStatus`](crate::usb::hub::PortStatus)) and the hub
//! bring-up / port-reset sequences ([`Hub`]) so [`enumerate`] — and any
//! caller reaching
//! devices behind a hub — works in named terms instead of bit masks and
//! descriptor offsets.

use crate::timer::Timer;
use crate::usb::control::{
    clear_port_feature, get_configuration_descriptor, get_hub_descriptor, get_port_status,
    set_configuration, set_port_power, set_port_reset, PORT_FEATURE_C_CONNECTION,
    PORT_FEATURE_C_RESET,
};
use crate::usb::descriptor::{ConfigurationDescriptor, Descriptors, EndpointDescriptor};
use crate::usb::dwc2::{Channel, ControlEndpoint, SplitTarget};
use crate::usb::EnumerationError;

/// How many times [`Hub::reset_port`] polls a port's status waiting for
/// it to enable after a reset, and the delay between polls — 20 × 10ms =
/// 200ms, comfortably longer than a hub takes to finish reset signaling
/// and enable the port.
const PORT_RESET_POLLS: u32 = 20;
/// Delay between the port-status polls of [`Hub::reset_port`].
const PORT_RESET_POLL_MS: u32 = 10;

/// How long [`Hub::configure`] waits for attached devices to show up
/// after powering the ports, on top of the hub's own `bPwrOn2PwrGood`.
///
/// The two waits are for different things and neither substitutes for
/// the other. `bPwrOn2PwrGood` is the hub's own power rail reaching the
/// port; only once it has does the attached device start up and drive
/// the pull-up that signals its speed, and USB 2.0 spec §7.1.7.3 gives
/// that up to 100ms (`TATTDB`) to settle before a hub is required to
/// report it. Reading a port's status the moment the power-good delay
/// expires therefore reads it before the device is there, and a hub with
/// a device on every port answers "nothing attached" for all of them.
const CONNECT_DEBOUNCE_MS: u32 = 100;

/// The `wPortStatus` bitmap from a hub GET_PORT_STATUS (USB 2.0 spec
/// §11.24.2.7) — a downstream port's connection, enable, power, and
/// attached-device speed, wrapped so callers read named bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PortStatus(pub u16);

impl PortStatus {
    /// Whether a device is connected to the port (`PORT_CONNECTION`,
    /// bit 0).
    pub fn connected(&self) -> bool {
        self.0 & (1 << 0) != 0
    }

    /// Whether the port is enabled (`PORT_ENABLE`, bit 1) — set by the
    /// hub once a device has successfully responded to a reset.
    pub fn enabled(&self) -> bool {
        self.0 & (1 << 1) != 0
    }

    /// Whether a low-speed device is attached (`PORT_LOW_SPEED`, bit 9).
    /// Only meaningful once the port is enabled.
    pub fn low_speed(&self) -> bool {
        self.0 & (1 << 9) != 0
    }

    /// Whether a high-speed device is attached (`PORT_HIGH_SPEED`, bit
    /// 10). Neither this nor [`Self::low_speed`] set means full speed.
    /// Only meaningful once the port is enabled.
    pub fn high_speed(&self) -> bool {
        self.0 & (1 << 10) != 0
    }
}

/// A configured USB hub: its addressed endpoint 0 plus the facts
/// needed to drive its downstream ports (how many there are, how long
/// to wait after powering one, whether the hub is running at high
/// speed and so has a transaction translator, and where its
/// status-change endpoint is). Build it with
/// [`Self::configure`], then reset and inspect individual ports through
/// it.
///
/// Plain data once built — every field is a fact read off the hub during
/// [`Self::configure`], so a `Hub` is `Copy` and can be kept in a table
/// the way [`Bus`](crate::usb::Bus) keeps one per hub on the bus.
#[derive(Clone, Copy)]
pub struct Hub {
    endpoint: ControlEndpoint,
    /// `bNbrPorts` — the number of downstream ports (1-based when
    /// addressing them).
    pub num_ports: u8,
    /// `bPwrOn2PwrGood` converted to milliseconds — how long to wait
    /// after powering a port before a device on it is stable.
    pub power_on_good_ms: u32,
    /// Whether the hub itself is operating at high speed, which is what
    /// decides whether it has a transaction translator of its own — see
    /// [`Self::split_target`].
    pub high_speed: bool,
    /// The endpoint number of the hub's status-change endpoint, and that
    /// endpoint's max packet size — see [`Self::status_endpoint`].
    status_endpoint: Option<(u8, u16)>,
}

impl Hub {
    /// Brings up the already-addressed hub at `endpoint`: activates its
    /// configuration, reads its class descriptor for the port count and
    /// power-on-good delay, powers every downstream port, and waits that
    /// delay plus the 100ms USB gives an attached device to announce
    /// itself before returning a [`Hub`] ready to drive those ports.
    ///
    /// `endpoint` must be the hub's endpoint 0 after SET_ADDRESS, with
    /// its real `bMaxPacketSize0` (see
    /// [`control::probe_and_address`](crate::usb::control::probe_and_address)).
    /// `high_speed` is whether the hub is itself running at high speed —
    /// `Dwc2Host::port_speed() == 0` for a hub on the root port, or
    /// [`PortStatus::high_speed`] of the upstream hub port it is plugged
    /// into. It can't be read back off `endpoint`, which distinguishes
    /// only low speed from the rest, and [`Self::split_target`] needs it.
    pub fn configure(
        channel: &mut Channel,
        timer: &Timer,
        endpoint: ControlEndpoint,
        high_speed: bool,
    ) -> Result<Hub, EnumerationError> {
        let mut config = [0u8; 64];
        let config_len = get_configuration_descriptor(channel, timer, endpoint, 0, &mut config)?;
        let config_value = ConfigurationDescriptor::parse(&config)
            .ok_or(EnumerationError::MalformedDescriptor)?
            .value();
        set_configuration(channel, timer, endpoint, config_value)?;
        let status_endpoint = find_status_endpoint(&config[..config_len]);

        let mut hub_descriptor = [0u8; 16];
        let len = get_hub_descriptor(channel, timer, endpoint, &mut hub_descriptor)?;
        // bNbrPorts is byte 2; bPwrOn2PwrGood (byte 5) is in 2ms units.
        if len < 6 {
            return Err(EnumerationError::MalformedDescriptor);
        }
        let num_ports = hub_descriptor[2];
        let power_on_good_ms = hub_descriptor[5] as u32 * 2;

        for port in 1..=num_ports {
            set_port_power(channel, timer, endpoint, port)?;
        }
        timer.delay_ms(power_on_good_ms + CONNECT_DEBOUNCE_MS);

        Ok(Hub {
            endpoint,
            num_ports,
            power_on_good_ms,
            high_speed,
            status_endpoint,
        })
    }

    /// The hub's status-change endpoint: its endpoint number and max
    /// packet size, or `None` if the hub's configuration didn't declare
    /// one (which no conforming hub does — USB 2.0 spec §11.12.4 requires
    /// exactly one interrupt-IN endpoint).
    ///
    /// Polling it with [`Channel::interrupt_in`]
    /// is how a full host hears about a device attached or removed after
    /// the bus was first walked. It answers with a bitmap — bit 0 the hub
    /// itself, bit *n* downstream port *n* — naming only what changed, and
    /// NAKs when nothing has, so watching a quiet hub costs one NAK'd
    /// transaction per poll rather than a status read per port.
    ///
    /// [`Bus::poll`](crate::usb::Bus::poll) deliberately does *not* use
    /// it, and reads the ports instead. This crate's DWC2 driver cannot
    /// yet schedule high-speed periodic transfers reliably, and the way it
    /// fails here is the dangerous kind: as well as
    /// [`TransferError::FrameOverrun`](crate::usb::dwc2::TransferError::FrameOverrun),
    /// the endpoint has been observed completing successfully with an
    /// all-zero bitmap while the hub's own port status showed a connection
    /// change outstanding — a device attached to that hub is then never
    /// seen at all. This is exposed for a caller that wants it anyway, and
    /// for `Bus` to build on once that is fixed.
    pub fn status_endpoint(&self) -> Option<(u8, u16)> {
        self.status_endpoint
    }

    /// The hub's own addressed endpoint 0 — what further control
    /// transfers to the hub (and the split target of anything below it)
    /// are built from.
    pub fn endpoint(&self) -> ControlEndpoint {
        self.endpoint
    }

    /// Reads downstream `port`'s current [`PortStatus`] (1-based).
    pub fn port_status(
        &self,
        channel: &mut Channel,
        timer: &Timer,
        port: u8,
    ) -> Result<PortStatus, EnumerationError> {
        let (status, _change) = get_port_status(channel, timer, self.endpoint, port)?;
        Ok(PortStatus(status))
    }

    /// Resets downstream `port` and returns its [`PortStatus`] once it
    /// enables (or once the poll budget runs out). The caller should
    /// check [`PortStatus::enabled`] on the result: a port that never
    /// enabled isn't an error here (nothing usable is attached), it just
    /// comes back not-enabled.
    ///
    /// Acknowledges the connection-change and reset-complete status bits
    /// around the reset (`C_PORT_CONNECTION` / `C_PORT_RESET`) so they
    /// don't linger; a failure to acknowledge is non-fatal and ignored.
    pub fn reset_port(
        &self,
        channel: &mut Channel,
        timer: &Timer,
        port: u8,
    ) -> Result<PortStatus, EnumerationError> {
        let endpoint = self.endpoint;

        let _ = clear_port_feature(channel, timer, endpoint, port, PORT_FEATURE_C_CONNECTION);
        set_port_reset(channel, timer, endpoint, port)?;

        let mut status = PortStatus(0);
        for _ in 0..PORT_RESET_POLLS {
            timer.delay_ms(PORT_RESET_POLL_MS);
            if let Ok((s, _)) = get_port_status(channel, timer, endpoint, port) {
                status = PortStatus(s);
                if status.enabled() {
                    break;
                }
            }
        }

        let _ = clear_port_feature(channel, timer, endpoint, port, PORT_FEATURE_C_RESET);
        Ok(status)
    }

    /// The [`SplitTarget`] for a device on downstream `port`, given its
    /// (post-reset) [`PortStatus`] — the nearest transaction translator
    /// upstream of that device, or `None` if it needs none.
    ///
    /// A transaction translator lives in a *high-speed* hub, one per
    /// downstream port (or one shared by all of them), and exists to
    /// relay a slower device's transfers onto the high-speed bus above.
    /// So which one a device needs depends on where the speed actually
    /// changes:
    ///
    /// - This hub is high speed and the device is too: no translation,
    ///   `None`.
    /// - This hub is high speed and the device is full/low speed: the
    ///   bus changes speed right here, so this hub's translator on
    ///   `port` is the target.
    /// - This hub is *not* high speed: it has no translator at all, and
    ///   everything below it sits on the same full/low-speed bus segment
    ///   the hub itself is on. The device therefore shares the hub's own
    ///   split target — the translator further upstream where that
    ///   segment began, or `None` if the whole chain is full speed down
    ///   from the root port.
    pub fn split_target(&self, port: u8, status: &PortStatus) -> Option<SplitTarget> {
        if !self.high_speed {
            return self.endpoint.split;
        }
        if status.high_speed() {
            None
        } else {
            Some(SplitTarget {
                hub_address: self.endpoint.address,
                port,
            })
        }
    }
}

/// Finds the status-change endpoint in a hub's configuration descriptor
/// block: the first interrupt-IN endpoint declared there, returned as its
/// endpoint number and max packet size.
///
/// A hub's configuration has exactly one endpoint besides endpoint 0 (USB
/// 2.0 spec §11.12.4), so "the first interrupt IN" is not a heuristic
/// standing in for a better match — it is the only candidate a conforming
/// hub offers.
fn find_status_endpoint(config: &[u8]) -> Option<(u8, u16)> {
    Descriptors::new(config)
        .filter_map(EndpointDescriptor::parse)
        .find(|endpoint| endpoint.is_interrupt() && endpoint.is_in())
        .map(|endpoint| (endpoint.number(), endpoint.max_packet_size()))
}
