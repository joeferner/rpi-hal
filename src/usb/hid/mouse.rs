//! Boot-protocol HID mouse driver.
//!
//! A mouse switched to boot protocol sends a fixed report (at least three
//! bytes — a button bitmap, then signed relative X and Y movement, and a
//! signed wheel delta on mice that include it) needing no HID
//! report-descriptor parsing. Unlike a keyboard's report (the absolute
//! *set* of held keys), a mouse's movement fields are already
//! per-report deltas; only the buttons are level state, so
//! `Mouse::poll` diffs consecutive button states into press/release
//! events while passing movement through as-is.
//!
//! Typical use, driven from a [`crate::usb::enumerate`] callback:
//!
//! ```ignore
//! use rpi_hal::usb::hid::mouse::{ButtonEvent, Mouse};
//!
//! if let Some(mut mouse) = Mouse::from_device(channel, timer, device)? {
//!     loop {
//!         timer.delay_ms(10); // pace polls -- see Mouse::poll
//!         if let Some(update) = mouse.poll(channel, timer)? {
//!             let report = update.report();
//!             // report.x / report.y / report.wheel are relative deltas
//!             for event in update.button_events() {
//!                 // ButtonEvent::Pressed(Button) / Released(Button)
//!             }
//!         }
//!     }
//! }
//! ```

use crate::timer::Timer;
use crate::usb::control::{
    get_configuration_descriptor, set_configuration, set_idle, set_protocol,
};
use crate::usb::descriptor::ConfigurationDescriptor;
use crate::usb::dwc2::{Channel, ControlEndpoint, TransferError};
use crate::usb::hid::{is_boot_interface, HidInterface, BOOT_PROTOCOL};
use crate::usb::Device;

/// `bInterfaceProtocol` value identifying a HID mouse (the boot
/// interface's mouse protocol).
const PROTOCOL_MOUSE: u8 = 2;

/// Number of mouse buttons a boot report carries in its button bitmap.
const BUTTON_COUNT: u8 = 3;

/// A mouse button.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Button {
    /// The left (primary) button.
    Left,
    /// The right (secondary) button.
    Right,
    /// The middle button (often the wheel click).
    Middle,
}

/// The mouse buttons held in a boot-mouse report (its first byte's
/// bitmap — USB HID spec Appendix B.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Buttons(
    /// The raw button bitmap byte.
    pub u8,
);

impl Buttons {
    /// Left (primary) button held (bit 0).
    pub fn left(&self) -> bool {
        self.0 & (1 << 0) != 0
    }

    /// Right (secondary) button held (bit 1).
    pub fn right(&self) -> bool {
        self.0 & (1 << 1) != 0
    }

    /// Middle button held (bit 2).
    pub fn middle(&self) -> bool {
        self.0 & (1 << 2) != 0
    }
}

/// A HID boot-mouse input report (USB HID spec Appendix B.2): the button
/// state plus relative movement since the last report. `x`/`y`/`wheel`
/// are signed deltas, not absolute positions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MouseReport {
    /// The buttons currently held.
    pub buttons: Buttons,
    /// Relative X movement since the last report (right is positive).
    pub x: i8,
    /// Relative Y movement since the last report (down is positive).
    pub y: i8,
    /// Relative wheel movement since the last report (`0` if the mouse
    /// has no wheel or reported only the minimal three bytes).
    pub wheel: i8,
}

impl MouseReport {
    /// Parses a boot mouse report (byte 0 the button bitmap, byte 1
    /// signed X, byte 2 signed Y, optional byte 3 signed wheel). Returns
    /// `None` if fewer than three bytes were received.
    pub fn parse(report: &[u8]) -> Option<Self> {
        if report.len() < 3 {
            return None;
        }
        Some(Self {
            buttons: Buttons(report[0]),
            x: report[1] as i8,
            y: report[2] as i8,
            wheel: if report.len() >= 4 {
                report[3] as i8
            } else {
                0
            },
        })
    }
}

/// A button transition between two consecutive reports.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ButtonEvent {
    /// A button was newly pressed.
    Pressed(Button),
    /// A button was released.
    Released(Button),
}

/// A newly-received mouse report together with the button changes since
/// the previous one, produced by [`Mouse::poll`]. Read the movement (and
/// current button state) from [`Self::report`], and iterate
/// [`Self::button_events`] for press/release transitions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MouseUpdate {
    previous_buttons: Buttons,
    report: MouseReport,
}

impl MouseUpdate {
    /// The current report — relative `x`/`y`/`wheel` movement and the
    /// buttons now held.
    pub fn report(&self) -> &MouseReport {
        &self.report
    }

    /// Iterates the button press/release events between the previous
    /// report and this one.
    pub fn button_events(&self) -> ButtonEventIter<'_> {
        ButtonEventIter {
            update: self,
            index: 0,
        }
    }
}

/// Iterator over the [`ButtonEvent`]s in a [`MouseUpdate`] — see
/// [`MouseUpdate::button_events`].
pub struct ButtonEventIter<'a> {
    update: &'a MouseUpdate,
    index: u8,
}

impl Iterator for ButtonEventIter<'_> {
    type Item = ButtonEvent;

    fn next(&mut self) -> Option<ButtonEvent> {
        while self.index < BUTTON_COUNT {
            let button = match self.index {
                0 => Button::Left,
                1 => Button::Right,
                _ => Button::Middle,
            };
            let mask = 1 << self.index;
            self.index += 1;

            let was_held = self.update.previous_buttons.0 & mask != 0;
            let is_held = self.update.report.buttons.0 & mask != 0;
            if is_held && !was_held {
                return Some(ButtonEvent::Pressed(button));
            }
            if was_held && !is_held {
                return Some(ButtonEvent::Released(button));
            }
        }
        None
    }
}

/// A configured HID boot mouse, polled for movement and button events.
///
/// Build one from an enumerated [`Device`] with [`Self::from_device`],
/// then call [`Self::poll`] repeatedly (paced — see that method). The
/// mouse has been switched to the boot protocol and set to report only on
/// change, so a still, unclicked mouse simply returns nothing.
pub struct Mouse {
    endpoint: ControlEndpoint,
    interface: u8,
    report_endpoint: u8,
    toggle: bool,
    previous_buttons: Buttons,
}

impl Mouse {
    /// Tries to bring `device` up as a boot mouse: reads its
    /// configuration descriptor, looks for a HID mouse interface with an
    /// interrupt-IN endpoint, and — if found — activates the
    /// configuration, switches the interface to the boot protocol, and
    /// sets it to report only on change. Returns `Ok(None)` if `device`
    /// isn't a HID mouse (leaving it addressed but otherwise untouched),
    /// so a caller can try it on every enumerated device.
    ///
    /// `SET_IDLE` is best-effort (some devices STALL it, harmlessly), but
    /// `SET_CONFIGURATION`/`SET_PROTOCOL` failing is reported as an error
    /// since the mouse wouldn't produce boot reports without them.
    pub fn from_device(
        channel: &mut Channel,
        timer: &Timer,
        device: Device,
    ) -> Result<Option<Mouse>, TransferError> {
        let mut config = [0u8; 64];
        let len = get_configuration_descriptor(channel, timer, device.endpoint, 0, &mut config)?;
        let Some(interface) = HidInterface::find(&config[..len], |iface| {
            is_boot_interface(iface, PROTOCOL_MOUSE)
        }) else {
            return Ok(None);
        };
        let Some(config_value) = ConfigurationDescriptor::parse(&config[..len]).map(|c| c.value())
        else {
            return Ok(None);
        };

        set_configuration(channel, timer, device.endpoint, config_value)?;
        set_protocol(
            channel,
            timer,
            device.endpoint,
            interface.interface,
            BOOT_PROTOCOL,
        )?;
        // Report-on-change only (idle duration 0); a STALL here is
        // harmless -- the mouse still reports on change by default.
        let _ = set_idle(channel, timer, device.endpoint, interface.interface, 0);

        Ok(Some(Mouse {
            endpoint: ControlEndpoint {
                max_packet_size: interface.max_packet_size,
                ..device.endpoint
            },
            interface: interface.interface,
            report_endpoint: interface.endpoint,
            toggle: false,
            previous_buttons: Buttons(0),
        }))
    }

    /// The interface number this mouse's report endpoint belongs to.
    pub fn interface(&self) -> u8 {
        self.interface
    }

    /// The report endpoint's number (its `bEndpointAddress` without the
    /// direction bit — OR in `0x80` for the full IN address).
    pub fn report_endpoint(&self) -> u8 {
        self.report_endpoint
    }

    /// The report endpoint's max packet size.
    pub fn max_packet_size(&self) -> u16 {
        self.endpoint.max_packet_size
    }

    /// Polls the report endpoint once, returning the movement and button
    /// changes since the previous poll — `Ok(Some(update))` when a new
    /// report arrived, `Ok(None)` when the mouse had nothing new (a NAK,
    /// the normal idle answer) or sent an unusable short report.
    ///
    /// The caller must pace polls (the endpoint's `bInterval`, ~10ms for
    /// a typical mouse): interrupt endpoints mustn't be hammered
    /// back-to-back, which wedges the controller's periodic scheduling —
    /// see [`Channel::interrupt_in`](crate::usb::dwc2::Channel::interrupt_in).
    pub fn poll(
        &mut self,
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<Option<MouseUpdate>, TransferError> {
        let mut buffer = [0u8; 8];
        let received = match channel.interrupt_in(
            self.endpoint,
            self.report_endpoint,
            &mut self.toggle,
            &mut buffer,
            timer,
        ) {
            Ok(received) => received,
            // NAK is normal flow control -- no new report this poll.
            Err(TransferError::Nak) => return Ok(None),
            Err(error) => return Err(error),
        };

        let Some(report) = MouseReport::parse(&buffer[..received]) else {
            // A short/partial report -- ignore it, keep the prior state.
            return Ok(None);
        };
        let previous_buttons = self.previous_buttons;
        self.previous_buttons = report.buttons;
        Ok(Some(MouseUpdate {
            previous_buttons,
            report,
        }))
    }
}
