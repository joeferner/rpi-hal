//! Boot-protocol HID keyboard driver.
//!
//! A keyboard switched to boot protocol sends a fixed 8-byte report (a
//! modifier bitmap, a reserved byte, then up to six held key usage IDs)
//! needing no HID report-descriptor parsing. A boot keyboard reports the
//! *set* of currently-held keys each time that set changes, not
//! press/release deltas; `Keyboard::poll` diffs consecutive reports into
//! `KeyEvent`s so callers see presses and releases directly.
//!
//! Typical use, driven from a [`crate::usb::enumerate`] callback:
//!
//! ```ignore
//! use rpi_hal::usb::hid::keyboard::{usage_to_ascii, KeyEvent, Keyboard};
//!
//! if let Some(mut keyboard) = Keyboard::from_device(dwc2, timer, device)? {
//!     loop {
//!         timer.delay_ms(10); // pace polls -- see Keyboard::poll
//!         if let Some(events) = keyboard.poll(dwc2, timer)? {
//!             let shift = events.report().modifiers.shift();
//!             for event in events.iter() {
//!                 if let KeyEvent::Pressed(usage) = event {
//!                     if let Some(c) = usage_to_ascii(usage, shift) { /* ... */ }
//!                 }
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
use crate::usb::dwc2::{ControlEndpoint, Dwc2Host, TransferError};
use crate::usb::hid::{is_boot_interface, HidInterface, BOOT_PROTOCOL, CHANNEL};
use crate::usb::Device;

/// `bInterfaceProtocol` value identifying a HID keyboard (the boot
/// interface's keyboard protocol).
const PROTOCOL_KEYBOARD: u8 = 1;

/// The lowest HID keyboard usage ID that is a real key. `0` is an empty
/// report slot and `1..=3` are error/rollover indicators (e.g. "too many
/// keys held at once"), never actual keys.
const FIRST_KEY_USAGE: u8 = 4;

/// Number of key slots in a boot keyboard report.
const KEY_SLOTS: usize = 6;

/// The modifier keys held in a boot-keyboard report (its first byte's
/// bitmap — USB HID spec Appendix B.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Modifiers(
    /// The raw modifier bitmap byte.
    pub u8,
);

impl Modifiers {
    /// Left Control held (bit 0).
    pub fn left_ctrl(&self) -> bool {
        self.0 & (1 << 0) != 0
    }

    /// Left Shift held (bit 1).
    pub fn left_shift(&self) -> bool {
        self.0 & (1 << 1) != 0
    }

    /// Left Alt held (bit 2).
    pub fn left_alt(&self) -> bool {
        self.0 & (1 << 2) != 0
    }

    /// Left GUI (Windows/Command) held (bit 3).
    pub fn left_gui(&self) -> bool {
        self.0 & (1 << 3) != 0
    }

    /// Right Control held (bit 4).
    pub fn right_ctrl(&self) -> bool {
        self.0 & (1 << 4) != 0
    }

    /// Right Shift held (bit 5).
    pub fn right_shift(&self) -> bool {
        self.0 & (1 << 5) != 0
    }

    /// Right Alt held (bit 6).
    pub fn right_alt(&self) -> bool {
        self.0 & (1 << 6) != 0
    }

    /// Right GUI (Windows/Command) held (bit 7).
    pub fn right_gui(&self) -> bool {
        self.0 & (1 << 7) != 0
    }

    /// Either Shift held.
    pub fn shift(&self) -> bool {
        self.left_shift() || self.right_shift()
    }

    /// Either Control held.
    pub fn ctrl(&self) -> bool {
        self.left_ctrl() || self.right_ctrl()
    }

    /// Either Alt held.
    pub fn alt(&self) -> bool {
        self.left_alt() || self.right_alt()
    }

    /// Either GUI (Windows/Command) held.
    pub fn gui(&self) -> bool {
        self.left_gui() || self.right_gui()
    }
}

/// A HID boot-keyboard input report (USB HID spec Appendix B.1): the
/// modifier bitmap plus up to six currently-held key usage IDs. This is
/// the *set* of keys held at report time, not a press/release delta — see
/// [`KeyEvents`] for those.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KeyboardReport {
    /// The modifier keys currently held.
    pub modifiers: Modifiers,
    /// Up to six held key usage IDs; `0` marks an unused slot.
    pub keys: [u8; KEY_SLOTS],
}

impl KeyboardReport {
    /// An all-released report (no modifiers, no keys) — the assumed state
    /// before the first poll, so the first real report's held keys read
    /// as freshly pressed.
    const fn empty() -> Self {
        Self {
            modifiers: Modifiers(0),
            keys: [0; KEY_SLOTS],
        }
    }

    /// Parses the 8-byte boot report (byte 0 the modifier bitmap, byte 1
    /// reserved, bytes 2..8 the six key slots). Returns `None` if fewer
    /// than 8 bytes were received.
    pub fn parse(report: &[u8]) -> Option<Self> {
        if report.len() < 8 {
            return None;
        }
        let mut keys = [0u8; KEY_SLOTS];
        keys.copy_from_slice(&report[2..8]);
        Some(Self {
            modifiers: Modifiers(report[0]),
            keys,
        })
    }

    /// Iterates the real keys currently held (usage IDs ≥ 4; empty slots
    /// and error/rollover codes are skipped).
    pub fn pressed_keys(&self) -> impl Iterator<Item = u8> + '_ {
        self.keys.iter().copied().filter(|&k| k >= FIRST_KEY_USAGE)
    }

    /// Serializes back to the 8-byte HID boot keyboard report (byte 0 the
    /// modifier bitmap, byte 1 reserved, bytes 2..8 the six key slots) — the
    /// inverse of [`Self::parse`]. Handy for forwarding a report unchanged,
    /// e.g. as a BLE HID Input Report.
    pub fn boot_report(&self) -> [u8; 8] {
        let mut report = [0u8; 8];
        report[0] = self.modifiers.0;
        report[2..8].copy_from_slice(&self.keys);
        report
    }

    /// Whether key usage `usage` appears in this report.
    fn holds(&self, usage: u8) -> bool {
        self.keys.contains(&usage)
    }
}

/// A key transition between two consecutive reports.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeyEvent {
    /// A key was newly pressed — its usage ID.
    Pressed(u8),
    /// A key was released — its usage ID.
    Released(u8),
}

/// The change from the previous report to a newly-received one, produced
/// by [`Keyboard::poll`]. Iterate it ([`Self::iter`]) for press/release
/// [`KeyEvent`]s, or read the current key state directly
/// ([`Self::report`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KeyEvents {
    previous: KeyboardReport,
    current: KeyboardReport,
}

impl KeyEvents {
    /// The full current report (held keys and modifiers).
    pub fn report(&self) -> &KeyboardReport {
        &self.current
    }

    /// Iterates the press/release events between the previous report and
    /// this one — releases first, then presses. Modifier changes are not
    /// events; read the current modifier state from [`Self::report`]'s
    /// [`modifiers`](KeyboardReport::modifiers) field.
    pub fn iter(&self) -> KeyEventIter<'_> {
        KeyEventIter {
            events: self,
            releases_done: false,
            index: 0,
        }
    }
}

/// Iterator over the [`KeyEvent`]s in a [`KeyEvents`] — see
/// [`KeyEvents::iter`].
pub struct KeyEventIter<'a> {
    events: &'a KeyEvents,
    releases_done: bool,
    index: usize,
}

impl Iterator for KeyEventIter<'_> {
    type Item = KeyEvent;

    fn next(&mut self) -> Option<KeyEvent> {
        // Releases: keys in the previous report no longer held now.
        if !self.releases_done {
            while self.index < KEY_SLOTS {
                let usage = self.events.previous.keys[self.index];
                self.index += 1;
                if usage >= FIRST_KEY_USAGE && !self.events.current.holds(usage) {
                    return Some(KeyEvent::Released(usage));
                }
            }
            self.releases_done = true;
            self.index = 0;
        }
        // Presses: keys held now that weren't in the previous report.
        while self.index < KEY_SLOTS {
            let usage = self.events.current.keys[self.index];
            self.index += 1;
            if usage >= FIRST_KEY_USAGE && !self.events.previous.holds(usage) {
                return Some(KeyEvent::Pressed(usage));
            }
        }
        None
    }
}

/// A configured HID boot keyboard, polled for key events.
///
/// Build one from an enumerated [`Device`] with [`Self::from_device`],
/// then call [`Self::poll`] repeatedly (paced — see that method). The
/// keyboard has been switched to the boot protocol and set to report only
/// on change, so an unchanged poll simply returns nothing.
pub struct Keyboard {
    endpoint: ControlEndpoint,
    interface: u8,
    report_endpoint: u8,
    toggle: bool,
    previous: KeyboardReport,
}

impl Keyboard {
    /// Tries to bring `device` up as a boot keyboard: reads its
    /// configuration descriptor, looks for a HID keyboard interface with
    /// an interrupt-IN endpoint, and — if found — activates the
    /// configuration, switches the interface to the boot protocol, and
    /// sets it to report only on change. Returns `Ok(None)` if `device`
    /// isn't a HID keyboard (leaving it addressed but otherwise
    /// untouched), so a caller can try it on every enumerated device.
    ///
    /// `SET_IDLE` is best-effort (some devices STALL it, harmlessly), but
    /// `SET_CONFIGURATION`/`SET_PROTOCOL` failing is reported as an error
    /// since the keyboard wouldn't produce boot reports without them.
    pub fn from_device(
        dwc2: &mut Dwc2Host,
        timer: &Timer,
        device: Device,
    ) -> Result<Option<Keyboard>, TransferError> {
        let mut config = [0u8; 64];
        let len = get_configuration_descriptor(dwc2, timer, device.endpoint, 0, &mut config)?;
        let Some(interface) = HidInterface::find(&config[..len], |iface| {
            is_boot_interface(iface, PROTOCOL_KEYBOARD)
        }) else {
            return Ok(None);
        };
        let Some(config_value) = ConfigurationDescriptor::parse(&config[..len]).map(|c| c.value())
        else {
            return Ok(None);
        };

        set_configuration(dwc2, timer, device.endpoint, config_value)?;
        set_protocol(
            dwc2,
            timer,
            device.endpoint,
            interface.interface,
            BOOT_PROTOCOL,
        )?;
        // Report-on-change only (idle duration 0); a STALL here is
        // harmless -- the keyboard still reports on change by default.
        let _ = set_idle(dwc2, timer, device.endpoint, interface.interface, 0);

        Ok(Some(Keyboard {
            endpoint: ControlEndpoint {
                max_packet_size: interface.max_packet_size,
                ..device.endpoint
            },
            interface: interface.interface,
            report_endpoint: interface.endpoint,
            toggle: false,
            previous: KeyboardReport::empty(),
        }))
    }

    /// The interface number this keyboard's report endpoint belongs to.
    pub fn interface(&self) -> u8 {
        self.interface
    }

    /// The report endpoint's number (its `bEndpointAddress` without the
    /// direction bit — OR in `0x80` for the full IN address).
    pub fn report_endpoint(&self) -> u8 {
        self.report_endpoint
    }

    /// The report endpoint's max packet size (a boot report is 8 bytes).
    pub fn max_packet_size(&self) -> u16 {
        self.endpoint.max_packet_size
    }

    /// Polls the report endpoint once, returning the key changes since
    /// the previous poll — `Ok(Some(events))` when a new report arrived,
    /// `Ok(None)` when the keyboard had nothing new (a NAK, the normal
    /// idle answer) or sent an unusable short report.
    ///
    /// The caller must pace polls (the endpoint's `bInterval`, ~10ms for
    /// a typical keyboard): interrupt endpoints mustn't be hammered
    /// back-to-back, which wedges the controller's periodic scheduling —
    /// see [`Dwc2Host::interrupt_in`](crate::usb::dwc2::Dwc2Host::interrupt_in).
    pub fn poll(
        &mut self,
        dwc2: &mut Dwc2Host,
        timer: &Timer,
    ) -> Result<Option<KeyEvents>, TransferError> {
        let mut buffer = [0u8; 8];
        let received = match dwc2.interrupt_in(
            CHANNEL,
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

        let Some(current) = KeyboardReport::parse(&buffer[..received]) else {
            // A short/partial report -- ignore it, keep the prior state.
            return Ok(None);
        };
        let previous = self.previous;
        self.previous = current;
        Ok(Some(KeyEvents { previous, current }))
    }
}

/// Translates a HID keyboard usage ID to its US-layout ASCII character,
/// honoring `shift` for letters and the shifted number-row/punctuation
/// symbols. Returns `None` for keys with no single-character US-layout
/// meaning (function keys, arrows, modifiers, etc.). US layout only —
/// other layouts map the same usage IDs to different characters, which
/// boot protocol alone can't distinguish.
pub fn usage_to_ascii(usage: u8, shift: bool) -> Option<char> {
    let character = match usage {
        // a-z / A-Z
        0x04..=0x1d => {
            let base = if shift { b'A' } else { b'a' };
            (base + (usage - 0x04)) as char
        }
        // 1-9 and their shifted symbols
        0x1e..=0x26 => {
            const SHIFTED: [u8; 9] = *b"!@#$%^&*(";
            if shift {
                SHIFTED[(usage - 0x1e) as usize] as char
            } else {
                (b'1' + (usage - 0x1e)) as char
            }
        }
        0x27 if shift => ')',
        0x27 => '0',
        0x28 => '\n',    // Enter
        0x2a => '\u{8}', // Backspace
        0x2b => '\t',    // Tab
        0x2c => ' ',     // Space
        0x2d if shift => '_',
        0x2d => '-',
        0x2e if shift => '+',
        0x2e => '=',
        0x2f if shift => '{',
        0x2f => '[',
        0x30 if shift => '}',
        0x30 => ']',
        0x31 if shift => '|',
        0x31 => '\\',
        0x33 if shift => ':',
        0x33 => ';',
        0x34 if shift => '"',
        0x34 => '\'',
        0x35 if shift => '~',
        0x35 => '`',
        0x36 if shift => '<',
        0x36 => ',',
        0x37 if shift => '>',
        0x37 => '.',
        0x38 if shift => '?',
        0x38 => '/',
        _ => return None,
    };
    Some(character)
}
