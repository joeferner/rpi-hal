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
use crate::usb::descriptor::ConfigurationDescriptor;
use crate::usb::dwc2::{ControlEndpoint, Dwc2Host, SplitTarget};
use crate::usb::EnumerationError;

/// How many times [`Hub::reset_port`] polls a port's status waiting for
/// it to enable after a reset, and the delay between polls — 20 × 10ms =
/// 200ms, comfortably longer than a hub takes to finish reset signaling
/// and enable the port.
const PORT_RESET_POLLS: u32 = 20;
/// Delay between the port-status polls of [`Hub::reset_port`].
const PORT_RESET_POLL_MS: u32 = 10;

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

/// A configured USB hub: its addressed endpoint 0 plus the two facts
/// needed to drive its downstream ports (how many there are, and how
/// long to wait after powering one). Build it with [`Self::configure`],
/// then reset and inspect individual ports through it.
pub struct Hub {
    endpoint: ControlEndpoint,
    /// `bNbrPorts` — the number of downstream ports (1-based when
    /// addressing them).
    pub num_ports: u8,
    /// `bPwrOn2PwrGood` converted to milliseconds — how long to wait
    /// after powering a port before a device on it is stable.
    pub power_on_good_ms: u32,
}

impl Hub {
    /// Brings up the already-addressed hub at `endpoint`: activates its
    /// configuration, reads its class descriptor for the port count and
    /// power-on-good delay, powers every downstream port, and waits that
    /// delay before returning a [`Hub`] ready to drive those ports.
    ///
    /// `endpoint` must be the hub's endpoint 0 after SET_ADDRESS, with
    /// its real `bMaxPacketSize0` (see
    /// [`control::probe_and_address`](crate::usb::control::probe_and_address)).
    pub fn configure(
        dwc2: &mut Dwc2Host,
        timer: &Timer,
        endpoint: ControlEndpoint,
    ) -> Result<Hub, EnumerationError> {
        let mut config = [0u8; 64];
        get_configuration_descriptor(dwc2, timer, endpoint, 0, &mut config)?;
        let config_value = ConfigurationDescriptor::parse(&config)
            .ok_or(EnumerationError::MalformedDescriptor)?
            .value();
        set_configuration(dwc2, timer, endpoint, config_value)?;

        let mut hub_descriptor = [0u8; 16];
        let len = get_hub_descriptor(dwc2, timer, endpoint, &mut hub_descriptor)?;
        // bNbrPorts is byte 2; bPwrOn2PwrGood (byte 5) is in 2ms units.
        if len < 6 {
            return Err(EnumerationError::MalformedDescriptor);
        }
        let num_ports = hub_descriptor[2];
        let power_on_good_ms = hub_descriptor[5] as u32 * 2;

        for port in 1..=num_ports {
            set_port_power(dwc2, timer, endpoint.address, port, endpoint.low_speed)?;
        }
        timer.delay_ms(power_on_good_ms);

        Ok(Hub {
            endpoint,
            num_ports,
            power_on_good_ms,
        })
    }

    /// Reads downstream `port`'s current [`PortStatus`] (1-based).
    pub fn port_status(
        &self,
        dwc2: &mut Dwc2Host,
        timer: &Timer,
        port: u8,
    ) -> Result<PortStatus, EnumerationError> {
        let (status, _change) = get_port_status(
            dwc2,
            timer,
            self.endpoint.address,
            port,
            self.endpoint.low_speed,
        )?;
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
        dwc2: &mut Dwc2Host,
        timer: &Timer,
        port: u8,
    ) -> Result<PortStatus, EnumerationError> {
        let hub_address = self.endpoint.address;
        let low_speed = self.endpoint.low_speed;

        let _ = clear_port_feature(
            dwc2,
            timer,
            hub_address,
            port,
            PORT_FEATURE_C_CONNECTION,
            low_speed,
        );
        set_port_reset(dwc2, timer, hub_address, port, low_speed)?;

        let mut status = PortStatus(0);
        for _ in 0..PORT_RESET_POLLS {
            timer.delay_ms(PORT_RESET_POLL_MS);
            if let Ok((s, _)) = get_port_status(dwc2, timer, hub_address, port, low_speed) {
                status = PortStatus(s);
                if status.enabled() {
                    break;
                }
            }
        }

        let _ = clear_port_feature(
            dwc2,
            timer,
            hub_address,
            port,
            PORT_FEATURE_C_RESET,
            low_speed,
        );
        Ok(status)
    }

    /// The [`SplitTarget`] for a device on downstream `port`, given its
    /// (post-reset) [`PortStatus`]: `Some` for a full/low-speed device
    /// (its transfers go through this hub's transaction translator),
    /// `None` for a high-speed device (it reaches the host directly).
    pub fn split_target(&self, port: u8, status: &PortStatus) -> Option<SplitTarget> {
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
