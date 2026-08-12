//! USB host support for the BCM2836/2837's built-in DWC2 OTG
//! controller, built up in stages rather than attempted as one unit:
//! host controller bring-up first, enumeration and a HID class driver
//! next (the smallest real thing that validates the controller
//! actually works end to end), Ethernet only once those are solid
//! (its own class driver plus a full TCP/IP stack on top, far more
//! surface area than HID). So far there's controller/root-port
//! bring-up ([`dwc2::Dwc2Host`](crate::usb::dwc2::Dwc2Host)); the
//! standard control transfers that enumerate a device (reading its
//! [device](crate::usb::control::get_device_descriptor) and
//! [configuration](crate::usb::control::get_configuration_descriptor)
//! descriptors, [SET_ADDRESS](crate::usb::control::set_address),
//! [SET_CONFIGURATION](crate::usb::control::set_configuration)); and the
//! hub-class requests that drive the on-board hub's downstream ports
//! ([hub descriptor](crate::usb::control::get_hub_descriptor),
//! [`set_port_power`](crate::usb::control::set_port_power),
//! [`get_port_status`](crate::usb::control::get_port_status),
//! [`set_port_reset`](crate::usb::control::set_port_reset)). A
//! device behind the hub can be reset and its descriptor read through
//! it — at high speed directly, or at full/low speed via split
//! transactions through the hub's transaction translator (see
//! [`dwc2::SplitTarget`](crate::usb::dwc2::SplitTarget)) — and
//! interrupt-IN endpoints can be polled
//! ([`dwc2::Dwc2Host::interrupt_in`](crate::usb::dwc2::Dwc2Host::interrupt_in)),
//! directly (the hub's own status-change endpoint) or through the hub
//! at full/low speed via periodic split scheduling (the report endpoint
//! of a HID keyboard on a physical port). [`enumerate`](crate::usb::enumerate) ties the whole
//! bring-up together — root-port reset, root-hub configuration, and
//! per-port reset/probe/address — and hands each downstream device to a
//! callback (see the `usb_enum`/`usb_hid_keyboard` examples). On top of
//! that sit the class drivers — HID ([`hid`](crate::usb::hid)), turning a
//! device's report endpoint into input events: a boot-protocol keyboard
//! and mouse, and a gamepad decoded through its own HID report descriptor
//! ([`hid::gamepad`](crate::usb::hid::gamepad)) — and the beginnings of the
//! on-board LAN9514
//! USB-Ethernet controller ([`lan9514`](crate::usb::lan9514)), so far
//! reaching its registers over vendor control transfers. Still missing:
//! recursion into hubs plugged into the root hub's ports (only one level
//! is walked today), bulk transfers (which Ethernet frame RX/TX needs),
//! and the rest of the LAN9514 driver and a network stack on top. Each is
//! real, separate work still to come, layered on top of
//! [`dwc2`](crate::usb::dwc2)/[`control`](crate::usb::control).
//!
//! On this project's target board (a Pi 2 Model B rev 1.1), the
//! on-board USB port(s) and the Ethernet jack are both wired through
//! an SMSC LAN9514 (a combined USB hub + 10/100 Ethernet controller)
//! sitting on this DWC2 controller's single root port. That means the
//! very first "device" this stack will ever see attached, on real
//! hardware, is that hub itself — reaching anything actually plugged
//! into a physical port (a keyboard, a mouse, the Ethernet path) needs
//! hub traversal, not just enumerating one device directly on the
//! root port.
//!
//! All of the above is confirmed working on real hardware: this stack
//! enumerates the on-board LAN9514 hub, powers and resets its
//! downstream ports, reaches through it to read the descriptors of
//! both the hub's own high-speed Ethernet function and a full-speed
//! keyboard on a physical port (the latter via split transactions),
//! polls the hub's interrupt status-change endpoint to detect a
//! device being unplugged/replugged, and reads live key presses from
//! that keyboard's interrupt report endpoint over a periodic split.
//! Getting the initial bring-up
//! working depended on one thing that no amount of DWC2 register tuning
//! could substitute for — the controller must be
//! powered on through the VideoCore mailbox first (see
//! [`dwc2`](crate::usb::dwc2)'s module doc); before that the register
//! block responds normally but no transaction ever runs.

/// USB protocol-level control transfers built on [`dwc2`] — see
/// [`control::get_device_descriptor`].
pub mod control;
/// Typed views over the standard USB descriptors (device, configuration,
/// interface, endpoint) — see [`descriptor::DeviceDescriptor`].
pub mod descriptor;
/// Host-mode bring-up for the DWC2 core itself, plus the low-level
/// DMA-mode control-transfer channel primitives — see
/// [`dwc2::Dwc2Host`].
pub mod dwc2;
/// HID class drivers built on the enumeration above — boot-protocol
/// keyboard and mouse (see [`hid::keyboard::Keyboard`]) plus a
/// report-descriptor-driven gamepad (see [`hid::gamepad::Gamepad`]).
pub mod hid;
/// Driving a hub's downstream ports (bring-up, port reset, speed/split
/// detection) — see [`hub::Hub`].
pub mod hub;
/// The on-board LAN9514 USB-Ethernet controller — see
/// [`lan9514::Lan9514`].
pub mod lan9514;

use core::ops::ControlFlow;

use crate::mailbox::{Mailbox, PowerDeviceId};
use crate::timer::Timer;
use crate::usb::control::probe_and_address;
use crate::usb::descriptor::DeviceDescriptor;
use crate::usb::dwc2::{ControlEndpoint, Dwc2Host, TransferError};
use crate::usb::hub::Hub;

/// Address assigned to the on-board root hub during [`enumerate`].
/// Downstream devices are addressed from here upward.
const ROOT_HUB_ADDRESS: u8 = 1;

/// Powers the USB host controller on through the VideoCore mailbox — the
/// mandatory first step before [`dwc2::Dwc2Host::init`], since the
/// firmware hands the core off only partially powered (see [`dwc2`]'s
/// module doc: without this the register block responds normally but no
/// transaction ever runs). Returns `true` once the controller is powered
/// and present, `false` if the mailbox call fails or the firmware reports
/// the controller absent.
pub fn power_on(mailbox: &mut Mailbox) -> bool {
    matches!(
        mailbox.set_power_state(PowerDeviceId::UsbHcd, true),
        Ok(true)
    )
}

/// What went wrong bringing up the bus or the root hub in [`enumerate`].
/// Failures enumerating an individual downstream port are *not* reported
/// this way — [`enumerate`] skips a port that misbehaves and carries on —
/// so this only covers the shared bring-up every device depends on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EnumerationError {
    /// No device is connected on the root port (nothing to enumerate).
    NotConnected,
    /// The root port didn't enable after a reset — the root device never
    /// came up.
    PortNotEnabled,
    /// A descriptor (the root hub's configuration or class descriptor)
    /// was too short to parse.
    MalformedDescriptor,
    /// An underlying control transfer failed.
    Transfer(TransferError),
}

impl From<TransferError> for EnumerationError {
    fn from(error: TransferError) -> Self {
        EnumerationError::Transfer(error)
    }
}

/// A downstream device found and addressed by [`enumerate`], handed to
/// its per-device callback. Everything needed to talk to the device
/// further (read its configuration, configure it, poll its endpoints) is
/// here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Device {
    /// The hub downstream port (1-based) the device is on.
    pub port: u8,
    /// The device's addressed endpoint 0 — address, speed, endpoint-0
    /// max packet size, and split target all filled in.
    pub endpoint: ControlEndpoint,
    /// The device's parsed device descriptor.
    pub descriptor: DeviceDescriptor,
}

/// Enumerates every device behind the on-board root hub, calling
/// `on_device` once per successfully-addressed downstream device.
///
/// Given an already-[initialized](dwc2::Dwc2Host::init) controller with a
/// device connected on the root port (the caller having waited for
/// [`dwc2::Dwc2Host::port_connected`]), this resets the root port, brings
/// up and configures the root hub (the on-board SMSC LAN9514 on this
/// board — see this module's doc), powers its downstream ports, then for
/// each connected port resets it, reads the device's descriptor, and
/// assigns it an address before invoking `on_device`. Addresses are
/// handed out sequentially starting just above the root hub's, so each
/// device is addressed before the next port is touched (two devices must
/// never sit at address 0 at once).
///
/// `on_device` receives the controller and timer (to do further
/// transfers) and the [`Device`]; return [`ControlFlow::Break`] from it
/// to stop enumerating early (e.g. once the device of interest is found),
/// or [`ControlFlow::Continue`] to keep going. A port whose reset or
/// descriptor read fails is skipped rather than aborting the whole
/// enumeration; only a failure of the shared root-hub bring-up returns an
/// [`EnumerationError`].
///
/// This drives a single level of hubs (the root hub's own ports); a hub
/// plugged into one of those ports is reported as a device but not itself
/// recursed into.
pub fn enumerate<F>(
    dwc2: &mut Dwc2Host,
    timer: &Timer,
    mut on_device: F,
) -> Result<(), EnumerationError>
where
    F: FnMut(&mut Dwc2Host, &Timer, Device) -> ControlFlow<()>,
{
    if !dwc2.port_connected() {
        return Err(EnumerationError::NotConnected);
    }
    dwc2.reset_port(timer);
    if !dwc2.port_enabled() {
        return Err(EnumerationError::PortNotEnabled);
    }

    // Enumerate and address the root device (the on-board hub), then
    // configure it and power its downstream ports.
    let root = ControlEndpoint {
        address: 0,
        low_speed: dwc2.port_speed() == 2,
        max_packet_size: 8,
        split: None,
    };
    let (hub_endpoint, _root_descriptor) = probe_and_address(dwc2, timer, root, ROOT_HUB_ADDRESS)?;
    let hub = Hub::configure(dwc2, timer, hub_endpoint)?;

    // Enumerate each connected port's device, one at a time — addressing
    // each before touching the next so two just-reset devices don't both
    // sit at address 0. A port that misbehaves is skipped, not fatal.
    let mut next_address = ROOT_HUB_ADDRESS + 1;
    for port in 1..=hub.num_ports {
        let Ok(status) = hub.port_status(dwc2, timer, port) else {
            continue;
        };
        if !status.connected() {
            continue;
        }
        let Ok(status) = hub.reset_port(dwc2, timer, port) else {
            continue;
        };
        if !status.enabled() {
            continue;
        }

        let probe = ControlEndpoint {
            address: 0,
            low_speed: status.low_speed(),
            max_packet_size: 8,
            split: hub.split_target(port, &status),
        };
        let address = next_address;
        let Ok((endpoint, descriptor)) = probe_and_address(dwc2, timer, probe, address) else {
            continue;
        };
        next_address += 1;

        let device = Device {
            port,
            endpoint,
            descriptor,
        };
        if on_device(dwc2, timer, device).is_break() {
            break;
        }
    }

    Ok(())
}
