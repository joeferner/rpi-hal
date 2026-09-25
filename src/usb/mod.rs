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
//! ([`dwc2::Channel::interrupt_in`](crate::usb::dwc2::Channel::interrupt_in)),
//! directly (the hub's own status-change endpoint) or through the hub
//! at full/low speed via periodic split scheduling (the report endpoint
//! of a HID keyboard on a physical port). [`enumerate`](crate::usb::enumerate) ties the whole
//! bring-up together — root-port reset, root-hub configuration, and
//! per-port reset/probe/address, applied again to any port that turns
//! out to have a hub on it, so a device several hubs deep is found and
//! pointed at the right transaction translator for where it sits — and
//! hands each device to a
//! callback (see the `usb_enum`/`usb_hid_keyboard` examples).
//! The [bus type](crate::usb::Bus) is the same walk with its state kept
//! afterwards, so [a later sweep](crate::usb::Bus::poll) can bring up a
//! device that attaches after the walk and report one that goes away —
//! which a single pass cannot, and which matters on boards where a
//! soldered-on device takes longer to announce itself than the walk
//! takes to run.
//! On top of
//! that sit the class drivers — HID ([`hid`](crate::usb::hid)), turning a
//! device's report endpoint into input events: a boot-protocol keyboard
//! and mouse, and a gamepad decoded through its own HID report descriptor
//! ([`hid::gamepad`](crate::usb::hid::gamepad)) — and the beginnings of the
//! on-board LAN9514
//! USB-Ethernet controller ([`lan9514`](crate::usb::lan9514)), so far
//! reaching its registers over vendor control transfers. Still missing:
//! bulk transfers (which Ethernet frame RX/TX needs), and the rest of the
//! LAN9514 driver and a network stack on top. Each is
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

// rustdoc reports one `redundant explicit link target` in this module
// tree and gives no source location for it, in any error format, so
// there is nothing to point at and fix. It appeared when `Bus` was added
// and is not a link in this file: stripping every explicit link target
// from `mod.rs` leaves the warning, and restoring the previous `mod.rs`
// alongside the current `hub.rs` clears it. Allowed here rather than
// crate-wide so a locatable one elsewhere still fails the build.
#![allow(rustdoc::redundant_explicit_links)]

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
use crate::usb::control::{clear_port_feature, probe_and_address, PORT_FEATURE_C_CONNECTION};
use crate::usb::descriptor::DeviceDescriptor;
use crate::usb::dwc2::{Channel, ControlEndpoint, Dwc2Host, TransferError};
use crate::usb::hub::Hub;

/// Address assigned to the on-board root hub during [`enumerate`].
/// Downstream devices are addressed from here upward.
const ROOT_HUB_ADDRESS: u8 = 1;

/// Highest USB device address (USB 2.0 spec §9.4.6 — `wValue` of
/// SET_ADDRESS is 7 bits, and 0 is the unaddressed default). A bus with
/// more devices than this on it is one [`enumerate`] stops walking.
const MAX_ADDRESS: u8 = 127;

/// `bDeviceClass` identifying a device as a hub (USB 2.0 spec §11.23.1)
/// — what [`enumerate`] keys its recursion off. A hub declares this in
/// its *device* descriptor rather than per-interface, so a downstream
/// hub is recognizable from the descriptor read while addressing it,
/// with no configuration descriptor needed.
const CLASS_HUB: u8 = 9;

/// How deep below the root hub [`enumerate`] descends, as a
/// [`Device::depth`] — a device at this depth is reported, but a hub at
/// it is not recursed into.
///
/// USB 2.0 spec §4.1.1 caps a bus at seven tiers: the host controller,
/// up to five hubs in a chain, and the device. The board's root hub is
/// the first of those five, so four more may hang below it, which puts
/// the deepest reachable device four levels down.
const MAX_HUB_DEPTH: u8 = 4;

/// How many hubs a [`Bus`] tracks, the root hub included. Beyond this it
/// has nowhere to record the topology a device's split target is derived
/// from, so rather than guess it stops with
/// [`EnumerationError::TooManyHubs`].
///
/// Eight is far past any real arrangement — USB's own five-hub chain
/// limit means reaching it takes a *wide* tree of hubs, not a deep one —
/// and the table costs a few hundred bytes, so the cap is here to make
/// the storage finite rather than to ration anything.
pub const MAX_HUBS: usize = 8;

/// How many downstream ports per hub a [`Bus`] tracks. Ports past this on
/// a single hub are left alone: no device on one is brought up, so none
/// is reported as attached and none can go missing later.
///
/// A USB 2.0 hub descriptor can claim up to 255 ports and no physical hub
/// comes close; 15 covers every one built, including the 13-port
/// controllers inside monitor docks.
pub const MAX_PORTS: usize = 15;

/// Marks a downstream port that has something on it this couldn't bring
/// up, as against `0` for an empty port and a real address for a device.
///
/// Not a valid USB address (they stop at [`MAX_ADDRESS`]), so it can't be
/// confused with one. It exists to make "occupied" and "usable" separate
/// answers: without it a device that fails to enumerate looks like an
/// empty port, and [`Bus::poll`] resets it again on every sweep for as
/// long as it stays plugged in.
const PORT_FAILED: u8 = u8::MAX;

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
    /// Every host channel was already handed out, so enumeration
    /// couldn't get one to run its control transfers on — see
    /// [`Dwc2Host::alloc_channel`].
    OutOfChannels,
    /// All 127 USB device addresses are in use, so the device just reset
    /// cannot be given one. The devices already found are still reported;
    /// the walk stops at the one that didn't fit.
    OutOfAddresses,
    /// More hubs are on the bus than a [`Bus`] can track (see
    /// [`MAX_HUBS`]). Reported rather than passed over, because the
    /// topology a [`Bus`] keeps is what every device's split target is
    /// derived from: an untracked hub means devices below it would be
    /// addressed through the wrong transaction translator, which is a
    /// worse answer than stopping.
    TooManyHubs,
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
/// here, plus where on the bus it was found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Device {
    /// The hub downstream port (1-based) the device is on.
    pub port: u8,
    /// Address of the hub whose [`Self::port`] this is — the root hub
    /// for a device plugged straight into the board, or a downstream
    /// hub's own address for one found behind it. With [`Self::port`]
    /// it names the device's position on the bus, which survives a
    /// reboot in a way the assigned address doesn't.
    pub hub_address: u8,
    /// How many hubs below the root hub the device sits: `0` on one of
    /// the root hub's own ports, `1` behind a hub plugged into one of
    /// those, and so on — up to four, the deepest USB allows.
    pub depth: u8,
    /// The device's addressed endpoint 0 — address, speed, endpoint-0
    /// max packet size, and split target all filled in.
    pub endpoint: ControlEndpoint,
    /// The device's parsed device descriptor.
    pub descriptor: DeviceDescriptor,
}

/// What a [`Bus`] found happening, handed to the callback of
/// [`Bus::enumerate`] and [`Bus::poll`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// A device was brought up: reset, addressed, and ready to talk to.
    Attached(Device),
    /// A device is gone — the port it was on no longer reports it
    /// connected. Its address has already been freed and may be handed to
    /// something else on the next attach, so a driver holding this device
    /// should stop using it here rather than later.
    ///
    /// Unplugging a hub reports every device that was behind it, deepest
    /// first, and the hub itself last.
    Detached {
        /// Address of the hub whose port the device was on.
        hub_address: u8,
        /// That hub's downstream port (1-based).
        port: u8,
        /// The address the device had, now free.
        address: u8,
    },
}

/// One tracked hub: everything [`Bus`] needs to revisit its ports later,
/// which a [`Hub`] alone doesn't carry.
#[derive(Clone, Copy)]
struct HubRecord {
    /// The configured hub itself — endpoint, port count, speed, and the
    /// status-change endpoint.
    hub: Hub,
    /// [`Device::depth`] of the devices on this hub's ports.
    depth: u8,
    /// What is on each downstream port, indexed by port minus one: `0`
    /// for nothing, [`PORT_FAILED`] for something that wouldn't come up,
    /// otherwise the device's address. This is the memory the hub itself
    /// doesn't have — it reports only what is plugged in now — and
    /// comparing the two is what turns a port reading into an arrival or
    /// a departure.
    ports: [u8; MAX_PORTS],
}

/// The USB bus behind the controller: which devices are on it, where, and
/// at what addresses — kept across calls so devices that appear or vanish
/// after the first walk can still be brought up or reported.
///
/// [`enumerate`] is this with the state thrown away, which is enough when
/// everything is plugged in before the board boots and never moves. It
/// stops being enough as soon as something attaches late. A device that
/// takes seconds to come up is missed by a one-shot walk and there is no
/// second look, and the reason there can't be one is that the walk owns
/// the two things bringing a device up requires: the next free address,
/// and the hub topology every split target is derived from. A `Bus` holds
/// both, so [`Self::poll`] can do what [`Self::enumerate`] did, later and
/// for one port.
///
/// Typical use is [`Self::enumerate`] once at startup and [`Self::poll`]
/// from the main loop:
///
/// ```ignore
/// let mut bus = Bus::new(&dwc2);
/// bus.enumerate(&timer, |channel, timer, event| { .. })?;
/// loop {
///     bus.poll(&timer, |channel, timer, event| { .. })?;
///     timer.delay_ms(250);
/// }
/// ```
///
/// Both take the same callback, so the code that recognizes a device
/// doesn't care whether it was there at boot or arrived later — which is
/// the point, since on some boards that is a matter of timing rather than
/// of anything the user did.
pub struct Bus<'a> {
    dwc2: &'a Dwc2Host,
    /// One bit per USB device address, bit *n* set while address *n* is
    /// assigned. A bitmap rather than a counter because addresses come
    /// back when devices are unplugged, and a bus that is replugged all
    /// day would otherwise run out.
    addresses: u128,
    hubs: [Option<HubRecord>; MAX_HUBS],
    /// Set when a walk hits something it cannot carry on past, so the
    /// walk can stop the way a callback does — by returning
    /// [`ControlFlow::Break`] — and still have the reason surface as an
    /// error from the public method.
    fault: Option<EnumerationError>,
}

impl<'a> Bus<'a> {
    /// A bus with nothing on it yet, for an
    /// already-[initialized](dwc2::Dwc2Host::init) controller. Nothing is
    /// touched until [`Self::enumerate`].
    pub fn new(dwc2: &'a Dwc2Host) -> Self {
        Bus {
            dwc2,
            addresses: 0,
            hubs: [None; MAX_HUBS],
            fault: None,
        }
    }

    /// Walks the whole bus from the root port, reporting every device it
    /// brings up as [`Event::Attached`], and records the topology so
    /// [`Self::poll`] can pick up from here.
    ///
    /// The caller must have waited for
    /// [`dwc2::Dwc2Host::port_connected`]. This resets the root port,
    /// brings up and configures the root hub (soldered on, on every board
    /// this targets — see this module's doc), powers its downstream
    /// ports, then for each connected port resets it, reads the device's
    /// descriptor, and assigns it an address before reporting it. Each
    /// device is addressed before the next port is touched, since two
    /// just-reset devices must never sit at address 0 at once.
    ///
    /// A device that turns out to be a hub itself is brought up the same
    /// way the root hub was and walked in turn, depth first, so the whole
    /// bus is reported however many hubs deep it goes (up to the four
    /// levels below the root hub that USB allows). Each device carries
    /// the [`hub_address`](Device::hub_address)/[`port`](Device::port) it
    /// was found on and its [`depth`](Device::depth), and its
    /// [`endpoint`](Device::endpoint) already points at the right
    /// transaction translator for its position — the nearest high-speed
    /// hub above it, which for a device several levels down is not
    /// necessarily the hub it is plugged into (see
    /// [`Hub::split_target`]).
    ///
    /// `on_event` receives the [`Channel`] the walk ran on (to do further
    /// transfers), the timer, and the [`Event`]; return
    /// [`ControlFlow::Break`] from it to stop early, e.g. once the device
    /// of interest is found. A hub is reported before the devices behind
    /// it, so breaking on one stops before it is walked.
    ///
    /// A port whose reset or descriptor read fails is skipped rather than
    /// aborting the walk, as is a hub that fails to configure (it is
    /// still reported, just not descended into). Calling this again
    /// discards everything known and starts over, which is what a bus
    /// that has been reset out from under this type needs.
    pub fn enumerate<F>(&mut self, timer: &Timer, mut on_event: F) -> Result<(), EnumerationError>
    where
        F: FnMut(&mut Channel, &Timer, Event) -> ControlFlow<()>,
    {
        let dwc2 = self.dwc2;
        if !dwc2.port_connected() {
            return Err(EnumerationError::NotConnected);
        }
        dwc2.reset_port(timer);
        if !dwc2.port_enabled() {
            return Err(EnumerationError::PortNotEnabled);
        }

        self.addresses = 0;
        self.hubs = [None; MAX_HUBS];
        self.fault = None;

        let mut channel = dwc2
            .alloc_channel()
            .ok_or(EnumerationError::OutOfChannels)?;
        let channel = &mut channel;

        // Address the root device (the on-board hub), then configure it
        // and power its downstream ports. It is the first thing to take
        // an address, so it always lands on ROOT_HUB_ADDRESS.
        let root = ControlEndpoint {
            address: 0,
            low_speed: dwc2.port_speed() == 2,
            max_packet_size: 8,
            split: None,
        };
        let address = self
            .take_address()
            .ok_or(EnumerationError::OutOfAddresses)?;
        debug_assert_eq!(address, ROOT_HUB_ADDRESS);
        let (hub_endpoint, _root_descriptor) = probe_and_address(channel, timer, root, address)?;
        let hub = Hub::configure(channel, timer, hub_endpoint, dwc2.port_speed() == 0)?;
        let index = self
            .record_hub(hub, 0)
            .ok_or(EnumerationError::TooManyHubs)?;

        // Whether the walk ran to the end or a callback broke out of it
        // makes no difference here: stopping early is the caller's own
        // request, not something to report as a failure. A fault is, and
        // is picked up below.
        let _ = self.walk_hub(channel, timer, index, &mut on_event);

        match self.fault.take() {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }

    /// Asks every tracked hub what has changed since the last look,
    /// bringing up anything newly attached and reporting anything gone.
    /// One sweep, then it returns — pacing is the caller's.
    ///
    /// Every port of every tracked hub is read with GET_PORT_STATUS and
    /// compared against what this type last saw there, so the cost of a
    /// sweep scales with how many ports are on the bus rather than with
    /// how much is happening on it. On a board whose root hub has four
    /// ports and carries one four-port hub, that is eight control
    /// transfers per sweep.
    ///
    /// A hub also has a status-change endpoint that would name just the
    /// ports that moved (see
    /// [`Hub::status_endpoint`]), which is
    /// cheaper and is what a full host uses. It isn't used here, because
    /// this crate's DWC2 driver cannot yet schedule high-speed periodic
    /// transfers reliably: on the bench that endpoint returned
    /// [`TransferError::FrameOverrun`], and — worse — completed
    /// successfully with an all-zero bitmap while the hub's own port
    /// status showed a connection change outstanding. A device attached to
    /// such a hub is then never seen at all. Reading the ports says the
    /// same thing over transfers that do work.
    ///
    /// Linux makes the same arrangement from the other direction: its hub
    /// driver runs on that endpoint, but `hub_activate()` reads every port
    /// directly whenever a hub is initialized, resumed or reset, and ten
    /// consecutive endpoint errors force exactly that reset — so a hub
    /// whose endpoint misbehaves degrades into polling rather than going
    /// silent. Here polling is the floor; the endpoint becomes an
    /// optimization on top of it once the periodic scheduling is fixed,
    /// which changes nothing about what this method promises.
    ///
    /// A port that now reports a device gets the same bring-up
    /// [`Self::enumerate`] gives one — reset, address, report — including
    /// being walked in turn if it is a hub with devices already on it. A
    /// port that has lost one reports [`Event::Detached`] for it, and for
    /// everything that was behind it if it was a hub.
    ///
    /// Requires [`Self::enumerate`] to have run: with no hubs tracked
    /// there is nothing to read, and this returns `Ok(())` having done
    /// nothing. It does not notice the *root port* itself being
    /// disconnected — on every board this targets the root hub is
    /// soldered on, so that is a condition this type doesn't model.
    ///
    /// A port whose device fails to come up is tried once and then left
    /// alone until the port's connection changes again, so a device this
    /// can't talk to costs one attempt rather than a port reset on every
    /// sweep for as long as it stays plugged in.
    pub fn poll<F>(&mut self, timer: &Timer, mut on_event: F) -> Result<(), EnumerationError>
    where
        F: FnMut(&mut Channel, &Timer, Event) -> ControlFlow<()>,
    {
        let dwc2 = self.dwc2;
        let mut channel = dwc2
            .alloc_channel()
            .ok_or(EnumerationError::OutOfChannels)?;
        let channel = &mut channel;
        let on_event: &mut dyn FnMut(&mut Channel, &Timer, Event) -> ControlFlow<()> =
            &mut on_event;

        for index in 0..MAX_HUBS {
            let Some(record) = self.hubs[index] else {
                continue;
            };
            if self
                .sweep_hub(channel, timer, index, &record, on_event)
                .is_break()
            {
                break;
            }
        }

        match self.fault.take() {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }

    /// Reads every port of one hub and reconciles what it reports against
    /// what this type last recorded there, bringing up what has arrived
    /// and tearing down what has gone.
    ///
    /// The comparison is against the port's present state rather than its
    /// change bits, which makes a missed sweep harmless: a device plugged
    /// and still plugged reads the same however many sweeps ago it
    /// arrived, whereas a change bit is a one-shot that has to be caught.
    /// The two sides are complementary — the hub knows the present and
    /// remembers nothing, this type remembers and knows nothing of the
    /// present — and neither alone can tell an arrival from a departure.
    fn sweep_hub(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        index: usize,
        record: &HubRecord,
        on_event: &mut dyn FnMut(&mut Channel, &Timer, Event) -> ControlFlow<()>,
    ) -> ControlFlow<()> {
        for port in 1..=record.hub.num_ports.min(MAX_PORTS as u8) {
            let Ok(status) = record.hub.port_status(channel, timer, port) else {
                continue;
            };

            // Read live rather than from `record`: an attach earlier in
            // this same sweep may have walked a hub and filled in ports
            // the snapshot still shows as empty.
            let known = self.hubs[index].map_or(0, |r| r.ports[port as usize - 1]);

            let flow = match (status.connected(), known) {
                // Newly arrived.
                (true, 0) => self.attach(channel, timer, index, port, on_event),
                // Still there, or tried once and not worth retrying until
                // the port is disturbed again.
                (true, _) => ControlFlow::Continue(()),
                // Empty and known to be.
                (false, 0) => ControlFlow::Continue(()),
                // Gone. A port that only ever failed to come up was never
                // reported as attached, so nothing is reported now either
                // — clearing the marker is the whole job, and it is what
                // lets the next device on this port be tried afresh.
                (false, PORT_FAILED) => {
                    self.set_port(index, port, 0);
                    ControlFlow::Continue(())
                }
                (false, _) => self.detach(channel, timer, index, port, on_event),
            };

            if status.connected() != (known != 0) {
                // Acknowledge the connection change this sweep just acted
                // on. Nothing here reads that bit — the comparison above
                // is what decides — but leaving it latched would keep the
                // hub's status-change endpoint asserted forever, which
                // matters to whoever uses it next.
                let _ = clear_port_feature(
                    channel,
                    timer,
                    record.hub.endpoint(),
                    port,
                    PORT_FEATURE_C_CONNECTION,
                );
            }

            if flow.is_break() {
                return ControlFlow::Break(());
            }
        }
        ControlFlow::Continue(())
    }

    /// Walks every port of the hub at `index`, bringing up whatever is
    /// connected. The initial pass over a hub that has just been
    /// configured, where nothing is known yet and every connected port is
    /// therefore new.
    ///
    /// Taking `on_event` as a `dyn FnMut` rather than a generic is what
    /// makes the recursion through [`Self::attach`] possible at all: a
    /// generic callback would have to monomorphize into a distinct
    /// function per nesting level, which for a function that calls itself
    /// does not terminate.
    fn walk_hub(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        index: usize,
        on_event: &mut dyn FnMut(&mut Channel, &Timer, Event) -> ControlFlow<()>,
    ) -> ControlFlow<()> {
        let Some(record) = self.hubs[index] else {
            return ControlFlow::Continue(());
        };
        for port in 1..=record.hub.num_ports.min(MAX_PORTS as u8) {
            let Ok(status) = record.hub.port_status(channel, timer, port) else {
                continue;
            };
            if !status.connected() {
                continue;
            }
            if self
                .attach(channel, timer, index, port, on_event)
                .is_break()
            {
                return ControlFlow::Break(());
            }
        }
        ControlFlow::Continue(())
    }

    /// Resets `port` of the hub at `index`, addresses whatever answers,
    /// reports it, and — if it is a hub — configures it, records it, and
    /// walks it in turn.
    ///
    /// A port that fails anywhere along the way is marked
    /// [`PORT_FAILED`] and otherwise left alone rather than failing the
    /// whole walk: it holds something this can't talk to, which says
    /// nothing about the other ports. The marker is what stops
    /// [`Self::poll`] resetting it again on every sweep for as long as it
    /// stays plugged in; unplugging it clears the marker, so the next
    /// device there gets a fresh attempt.
    fn attach(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        index: usize,
        port: u8,
        on_event: &mut dyn FnMut(&mut Channel, &Timer, Event) -> ControlFlow<()>,
    ) -> ControlFlow<()> {
        let Some(record) = self.hubs[index] else {
            return ControlFlow::Continue(());
        };

        let Ok(status) = record.hub.reset_port(channel, timer, port) else {
            self.set_port(index, port, PORT_FAILED);
            return ControlFlow::Continue(());
        };
        if !status.enabled() {
            self.set_port(index, port, PORT_FAILED);
            return ControlFlow::Continue(());
        }

        let Some(address) = self.take_address() else {
            self.fault = Some(EnumerationError::OutOfAddresses);
            return ControlFlow::Break(());
        };
        let probe = ControlEndpoint {
            address: 0,
            low_speed: status.low_speed(),
            max_packet_size: 8,
            split: record.hub.split_target(port, &status),
        };
        let Ok((endpoint, descriptor)) = probe_and_address(channel, timer, probe, address) else {
            // The address was never taken up, so it goes straight back
            // rather than being burned by a device that didn't answer.
            self.release_address(address);
            self.set_port(index, port, PORT_FAILED);
            return ControlFlow::Continue(());
        };
        self.set_port(index, port, address);

        let device = Device {
            port,
            hub_address: record.hub.endpoint().address,
            depth: record.depth,
            endpoint,
            descriptor,
        };
        if on_event(channel, timer, Event::Attached(device)).is_break() {
            return ControlFlow::Break(());
        }

        // A hub declares itself in its device descriptor, so this is
        // already known without reading anything further. Configuring it
        // powers and settles its own ports, after which it is driven
        // exactly like the root hub was.
        if descriptor.device_class == CLASS_HUB && record.depth < MAX_HUB_DEPTH {
            let Ok(hub) = Hub::configure(channel, timer, endpoint, status.high_speed()) else {
                return ControlFlow::Continue(());
            };
            let Some(child) = self.record_hub(hub, record.depth + 1) else {
                self.fault = Some(EnumerationError::TooManyHubs);
                return ControlFlow::Break(());
            };
            return self.walk_hub(channel, timer, child, on_event);
        }
        ControlFlow::Continue(())
    }

    /// Reports the device that was on `port` of the hub at `index` as
    /// gone, along with everything that was behind it if it was a hub,
    /// and frees every address involved.
    ///
    /// The teardown runs to completion even if the callback breaks partway
    /// through: a half-freed topology would leave addresses that can never
    /// be handed out again and hubs that are polled forever. The break is
    /// remembered and returned once the state is consistent.
    fn detach(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        index: usize,
        port: u8,
        on_event: &mut dyn FnMut(&mut Channel, &Timer, Event) -> ControlFlow<()>,
    ) -> ControlFlow<()> {
        let Some(record) = self.hubs[index].as_mut() else {
            return ControlFlow::Continue(());
        };
        let address = core::mem::take(&mut record.ports[port as usize - 1]);
        if address == 0 {
            return ControlFlow::Continue(());
        }
        let hub_address = record.hub.endpoint().address;

        // Deepest first, so a driver hears about the device it holds
        // before the hub that device was behind.
        let mut stop = self.detach_subtree(channel, timer, address, on_event);
        self.release_address(address);
        if on_event(
            channel,
            timer,
            Event::Detached {
                hub_address,
                port,
                address,
            },
        )
        .is_break()
        {
            stop = ControlFlow::Break(());
        }
        stop
    }

    /// Tears down the hub at `address`, if the departed device was one:
    /// every device on its ports goes too, and theirs, and so on.
    fn detach_subtree(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
        address: u8,
        on_event: &mut dyn FnMut(&mut Channel, &Timer, Event) -> ControlFlow<()>,
    ) -> ControlFlow<()> {
        let Some(index) = (0..MAX_HUBS)
            .find(|i| self.hubs[*i].is_some_and(|record| record.hub.endpoint().address == address))
        else {
            return ControlFlow::Continue(());
        };
        // Out of the table first: nothing below can reach back up to a hub
        // that is already gone, and this bounds the recursion no matter
        // what the table says.
        let record = self.hubs[index].take();
        let Some(record) = record else {
            return ControlFlow::Continue(());
        };

        let mut stop = ControlFlow::Continue(());
        for (port, &below) in record.ports.iter().enumerate() {
            // A port that never came up was never reported as attached, so
            // there is nothing to report or free for it now — and
            // `PORT_FAILED` is not an address, so treating it as one would
            // free a bit of the address pool that belongs to somebody else.
            if below == 0 || below == PORT_FAILED {
                continue;
            }
            if self
                .detach_subtree(channel, timer, below, on_event)
                .is_break()
            {
                stop = ControlFlow::Break(());
            }
            self.release_address(below);
            if on_event(
                channel,
                timer,
                Event::Detached {
                    hub_address: address,
                    port: port as u8 + 1,
                    address: below,
                },
            )
            .is_break()
            {
                stop = ControlFlow::Break(());
            }
        }
        stop
    }

    /// Takes the lowest free USB device address, or `None` if all 127 are
    /// in use.
    fn take_address(&mut self) -> Option<u8> {
        let address = (1..=MAX_ADDRESS).find(|a| self.addresses & (1u128 << a) == 0)?;
        self.addresses |= 1u128 << address;
        Some(address)
    }

    /// Returns an address to the pool. It can be handed out again
    /// immediately, which is why [`Event::Detached`] carries it: a driver
    /// still talking to that address after being told is talking to
    /// whatever got plugged in next.
    ///
    /// Anything past [`MAX_ADDRESS`] is ignored rather than shifted by:
    /// the bitmap is 128 bits wide, so a stray [`PORT_FAILED`] reaching
    /// here would shift past the end of it — which is a panic in a debug
    /// build and, worse, silently wraps round and frees somebody else's
    /// address in a release one.
    fn release_address(&mut self, address: u8) {
        if address <= MAX_ADDRESS {
            self.addresses &= !(1u128 << address);
        }
    }

    /// Files a configured hub in the tracking table, returning its index,
    /// or `None` if the table is full.
    fn record_hub(&mut self, hub: Hub, depth: u8) -> Option<usize> {
        let index = (0..MAX_HUBS).find(|i| self.hubs[*i].is_none())?;
        self.hubs[index] = Some(HubRecord {
            hub,
            depth,
            ports: [0; MAX_PORTS],
        });
        Some(index)
    }

    /// Records what is on a downstream port of the hub at `index` — an
    /// address, `0` for nothing, or [`PORT_FAILED`]. A no-op if that hub
    /// is no longer tracked, which is what a port of a hub torn down
    /// mid-sweep looks like.
    fn set_port(&mut self, index: usize, port: u8, value: u8) {
        if let Some(record) = self.hubs[index].as_mut() {
            record.ports[port as usize - 1] = value;
        }
    }
}

/// Enumerates every device behind the on-board root hub, calling
/// `on_device` once per successfully-addressed downstream device.
///
/// [`Bus::enumerate`] without keeping the bus around afterwards — the
/// whole walk, including recursion into downstream hubs, with the
/// topology discarded when it returns. That is all a program needs when
/// everything is plugged in before the board boots and nothing moves,
/// which is most of them.
///
/// Reach for [`Bus`] instead when something can attach after startup, or
/// when a device being unplugged has to be noticed. A device slow enough
/// to come up is indistinguishable from an absent one here, and that is
/// a matter of the board's timing rather than of anything the user did:
/// see [`Bus::poll`].
pub fn enumerate<F>(
    dwc2: &Dwc2Host,
    timer: &Timer,
    mut on_device: F,
) -> Result<(), EnumerationError>
where
    F: FnMut(&mut Channel, &Timer, Device) -> ControlFlow<()>,
{
    Bus::new(dwc2).enumerate(timer, |channel, timer, event| match event {
        Event::Attached(device) => on_device(channel, timer, device),
        // Nothing can be gone from a bus this call has only just met.
        Event::Detached { .. } => ControlFlow::Continue(()),
    })
}
