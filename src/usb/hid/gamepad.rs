//! HID gamepad driver, decoded through the device's own report
//! descriptor.
//!
//! Unlike the [boot keyboard](crate::usb::hid::keyboard) and
//! [boot mouse](crate::usb::hid::mouse) drivers here, this one can't rely on a
//! fixed report format: there is no boot protocol for game controllers, so
//! every controller declares its own report layout — which byte holds
//! which stick, how wide each axis is, where the buttons and the D-pad hat
//! sit — in a HID *report descriptor*. This driver reads that descriptor
//! ([`control::get_report_descriptor`](crate::usb::control::get_report_descriptor))
//! and parses it into a field map ([`crate::hid_report`]), so the *same*
//! code reads controllers whose report layouts have nothing in common.
//! That is also how it knows a gamepad when it sees one: a HID interface
//! outside the boot subclass declares nothing about what it is, and only
//! the descriptor's Application collection says "gamepad" (see
//! [`ReportDescriptor::is_gamepad`](crate::hid_report::ReportDescriptor::is_gamepad)).
//!
//! What each control *means physically* — which axis is the left stick and
//! which is a trigger, which button number is "A" — is a per-device
//! convention that genuinely isn't in the descriptor; an OS resolves it
//! with a quirk database. So this driver hands the caller the field map and
//! the live reports rather than pretending to a universal button layout.
//! The same split the Bluetooth side makes (see
//! [the Bluetooth HID host](crate::bluetooth::hid_host)), against the same
//! parser.
//!
//! Typical use, driven from a [`crate::usb::enumerate`] callback:
//!
//! ```ignore
//! use rpi_hal::hid_report::USAGE_PAGE_BUTTON;
//! use rpi_hal::usb::hid::gamepad::Gamepad;
//!
//! if let Some(mut gamepad) = Gamepad::from_device(dwc2, timer, device)? {
//!     let mut buf = [0u8; 64];
//!     loop {
//!         timer.delay_ms(gamepad.poll_interval_ms()); // pace polls
//!         if let Some(report) = gamepad.poll(dwc2, timer, &mut buf)? {
//!             for field in gamepad.report_descriptor().fields() {
//!                 if field.report_id != report.id {
//!                     continue;
//!                 }
//!                 let value = field.extract_signed(report.payload);
//!                 // interpret `value` by field.usage_page / field.usage
//!             }
//!         }
//!     }
//! }
//! ```

use crate::hid_report::ReportDescriptor;
use crate::timer::Timer;
use crate::usb::control::{
    get_configuration_descriptor, get_report_descriptor, set_configuration, set_idle,
};
use crate::usb::descriptor::ConfigurationDescriptor;
use crate::usb::dwc2::{ControlEndpoint, Dwc2Host, TransferError, MAX_TRANSFER_LEN};
use crate::usb::hid::{HidInterface, CHANNEL};
use crate::usb::Device;

/// Bytes of configuration descriptor read while looking for the gamepad's
/// HID interface. Capped at what one control transfer can carry; a
/// configuration longer than this is read truncated (see
/// [`get_configuration_descriptor`]), which would only hide an interface
/// declared past the cut in a large composite device.
const CONFIG_BUFFER_LEN: usize = MAX_TRANSFER_LEN;

/// Largest HID report descriptor this driver reads. A controller's is
/// typically 100–200 bytes, and the whole thing has to arrive in a single
/// transfer — GET_DESCRIPTOR has no way to ask for an offset into a
/// descriptor — so this is capped by what one transfer can carry.
const MAX_REPORT_DESCRIPTOR: usize = MAX_TRANSFER_LEN;

/// What went wrong bringing a device up as a gamepad in
/// [`Gamepad::from_device`] — which request failed, not just how.
///
/// Unlike the boot-protocol drivers (which report a bare
/// [`TransferError`]), setup here spans three different requests to two
/// different recipients, and they fail for unrelated reasons: a device that
/// rejects a descriptor read is a different problem from one that won't
/// activate its configuration. A bare "STALL" can't say which, and that
/// distinction is exactly what a caller needs to make sense of a controller
/// that refuses to come up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GamepadError {
    /// Reading the device's configuration descriptor failed (a standard
    /// request to the device — the same one every host issues during
    /// enumeration).
    ConfigurationDescriptor(TransferError),
    /// SET_CONFIGURATION failed: the device wouldn't activate the
    /// configuration whose `bConfigurationValue` its own configuration
    /// descriptor declared.
    SetConfiguration(TransferError),
    /// Reading the HID report descriptor of the interface with this number
    /// failed. A STALL is reported only if *no* interface turned out to be
    /// a gamepad: the device refusing that request for one interface is
    /// [`Gamepad::from_device`]'s cue to try the next one, and only when
    /// none is left does it mean the device's reports are unreadable.
    ReportDescriptor(u8, TransferError),
}

/// One input report received from a gamepad, split the way the report
/// descriptor's field offsets expect — produced by [`Gamepad::poll`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Report<'a> {
    /// The report ID, or `0` when the descriptor uses no report IDs (a
    /// controller with several report layouts tags each with an ID; the
    /// field map's [`report_id`](crate::hid_report::Field::report_id) says
    /// which fields belong to this one).
    pub id: u8,
    /// The report's payload — the bytes *after* the report-ID byte when the
    /// descriptor uses IDs, the whole report when it doesn't. Field bit
    /// offsets are relative to this, so it's what
    /// [`Field::extract`](crate::hid_report::Field::extract) takes.
    pub payload: &'a [u8],
}

/// A configured HID gamepad, polled for input reports and carrying the
/// parsed report descriptor that decodes them.
///
/// Build one from an enumerated [`Device`] with [`Self::from_device`], read
/// the layout from [`Self::report_descriptor`], then call [`Self::poll`]
/// repeatedly (paced — see that method).
pub struct Gamepad {
    /// The device's endpoint, with `max_packet_size` set to the *report*
    /// endpoint's — all this driver's remaining transfers are polls of that
    /// endpoint, the control transfers being done by [`Self::from_device`].
    endpoint: ControlEndpoint,
    interface: u8,
    report_endpoint: u8,
    poll_interval_ms: u32,
    toggle: bool,
    descriptor: ReportDescriptor,
    raw_descriptor: [u8; MAX_REPORT_DESCRIPTOR],
    raw_descriptor_len: usize,
}

impl Gamepad {
    /// Tries to bring `device` up as a gamepad: reads its configuration
    /// descriptor, looks for a HID interface with an interrupt-IN endpoint,
    /// activates the configuration, reads and parses the interface's report
    /// descriptor, and keeps the device only if that descriptor declares a
    /// game controller ([`ReportDescriptor::is_gamepad`]). Returns
    /// `Ok(None)` if `device` isn't one, so a caller can try it on every
    /// enumerated device.
    ///
    /// No `SET_PROTOCOL` is sent: that request selects between the boot and
    /// report protocols, and the report protocol — the one this driver
    /// parses — is what every HID device uses by default (a device outside
    /// the boot subclass has no other mode to be in). `SET_IDLE` is
    /// best-effort: with it the controller reports only on change, so an
    /// untouched one simply NAKs; devices that STALL it stream reports
    /// continuously instead, which [`Self::poll`] handles either way.
    ///
    /// A HID device that turns out not to be a gamepad (a keyboard, say) is
    /// left configured — harmless, and what the boot drivers do too when
    /// they claim a device.
    pub fn from_device(
        dwc2: &mut Dwc2Host,
        timer: &Timer,
        device: Device,
    ) -> Result<Option<Gamepad>, GamepadError> {
        let mut config = [0u8; CONFIG_BUFFER_LEN];
        let len = get_configuration_descriptor(dwc2, timer, device.endpoint, 0, &mut config)
            .map_err(GamepadError::ConfigurationDescriptor)?;
        // Nothing to do at all for a device with no HID report endpoint --
        // checked before configuring so a non-HID device is left untouched.
        if HidInterface::find(&config[..len], |_| true).is_none() {
            return Ok(None);
        }
        let Some(config_value) = ConfigurationDescriptor::parse(&config[..len]).map(|c| c.value())
        else {
            return Ok(None);
        };
        // Configure before reading any report descriptor: that request is
        // addressed to an interface, which only a configured device has.
        set_configuration(dwc2, timer, device.endpoint, config_value)
            .map_err(GamepadError::SetConfiguration)?;

        // Every HID interface with a report endpoint is a candidate -- what
        // kind of device each one is only becomes clear from its own report
        // descriptor, so they're tried in turn. A composite controller can
        // put its gamepad interface behind, say, a vendor or keyboard one.
        let mut from = 0u16;
        let mut stalled = None;
        while let Some(interface) =
            HidInterface::find(&config[..len], |iface| iface.number() as u16 >= from)
        {
            from = interface.interface as u16 + 1;
            if interface.report_descriptor_len == 0 {
                // No report descriptor declared, so nothing could decode
                // this interface's reports even if they arrived.
                continue;
            }

            // The read length must match what the interface declared -- see
            // `get_report_descriptor` -- so it's clamped to this driver's
            // cap rather than to the buffer, and an over-long descriptor is
            // read truncated (its trailing fields lost) instead of not at
            // all.
            let mut raw_descriptor = [0u8; MAX_REPORT_DESCRIPTOR];
            let to_read = (interface.report_descriptor_len as usize).min(MAX_REPORT_DESCRIPTOR);
            let raw_descriptor_len = match get_report_descriptor(
                dwc2,
                timer,
                device.endpoint,
                interface.interface,
                &mut raw_descriptor[..to_read],
            ) {
                Ok(received) => received,
                // A STALL is the device refusing the request for this one
                // interface, not a broken bus -- move on to the next
                // candidate rather than failing the whole probe, but keep
                // it in case no interface pans out (see below).
                Err(TransferError::Stall) => {
                    stalled.get_or_insert(interface.interface);
                    continue;
                }
                Err(error) => {
                    return Err(GamepadError::ReportDescriptor(interface.interface, error))
                }
            };
            let descriptor = ReportDescriptor::parse(&raw_descriptor[..raw_descriptor_len]);
            if !descriptor.is_gamepad() {
                continue;
            }

            // Report-on-change only (idle duration 0); a STALL here is
            // harmless -- see this method's doc.
            let _ = set_idle(dwc2, timer, device.endpoint, interface.interface, 0);

            return Ok(Some(Gamepad {
                endpoint: ControlEndpoint {
                    max_packet_size: interface.max_packet_size,
                    ..device.endpoint
                },
                interface: interface.interface,
                report_endpoint: interface.endpoint,
                poll_interval_ms: poll_interval_ms(&device.endpoint, interface.interval),
                toggle: false,
                descriptor,
                raw_descriptor,
                raw_descriptor_len,
            }));
        }
        // Nothing here was a gamepad. If an interface declared a report
        // descriptor and then refused to hand it over, say so rather than
        // reporting a plain "not a gamepad" -- unreadable is not the same
        // as read-and-it-wasn't-one, and the difference is the whole
        // difference between a broken device and an uninteresting one.
        match stalled {
            Some(interface) => Err(GamepadError::ReportDescriptor(
                interface,
                TransferError::Stall,
            )),
            None => Ok(None),
        }
    }

    /// The interface number this gamepad's report endpoint belongs to.
    pub fn interface(&self) -> u8 {
        self.interface
    }

    /// The report endpoint's number (its `bEndpointAddress` without the
    /// direction bit — OR in `0x80` for the full IN address).
    pub fn report_endpoint(&self) -> u8 {
        self.report_endpoint
    }

    /// The report endpoint's max packet size — the largest report the
    /// controller can send, and so the smallest buffer [`Self::poll`]
    /// should be given to not truncate one.
    pub fn max_packet_size(&self) -> u16 {
        self.endpoint.max_packet_size
    }

    /// How long to wait between [`Self::poll`] calls, in milliseconds —
    /// the report endpoint's `bInterval` converted to a period (at least
    /// `1`). Polling faster than the endpoint asks for gains nothing: it
    /// has at most one report per interval to give.
    pub fn poll_interval_ms(&self) -> u32 {
        self.poll_interval_ms
    }

    /// The parsed report descriptor — the field map that decodes the
    /// reports [`Self::poll`] returns.
    pub fn report_descriptor(&self) -> &ReportDescriptor {
        &self.descriptor
    }

    /// The raw report descriptor bytes, as read from the device. The parsed
    /// form ([`Self::report_descriptor`]) is what decodes reports; these are
    /// for dumping the descriptor when a control reads wrong and the
    /// question is whether the parse or the device is at fault.
    pub fn report_descriptor_bytes(&self) -> &[u8] {
        &self.raw_descriptor[..self.raw_descriptor_len]
    }

    /// Polls the report endpoint once, reading into `buf` — `Ok(Some(report))`
    /// when one arrived, `Ok(None)` when the controller had nothing new (a
    /// NAK, the normal idle answer, or an empty report). At most
    /// `buf.len()` bytes are read, so give it at least
    /// [`Self::max_packet_size`] bytes to not truncate a report.
    ///
    /// The caller must pace polls at [`Self::poll_interval_ms`]: interrupt
    /// endpoints mustn't be hammered back-to-back, which wedges the
    /// controller's periodic scheduling — see
    /// [`Dwc2Host::interrupt_in`](crate::usb::dwc2::Dwc2Host::interrupt_in).
    pub fn poll<'a>(
        &mut self,
        dwc2: &mut Dwc2Host,
        timer: &Timer,
        buf: &'a mut [u8],
    ) -> Result<Option<Report<'a>>, TransferError> {
        let to_read = buf
            .len()
            .min(self.endpoint.max_packet_size as usize)
            .min(MAX_TRANSFER_LEN);
        let received = match dwc2.interrupt_in(
            CHANNEL,
            self.endpoint,
            self.report_endpoint,
            &mut self.toggle,
            &mut buf[..to_read],
            timer,
        ) {
            Ok(received) => received,
            // NAK is normal flow control -- no new report this poll.
            Err(TransferError::Nak) => return Ok(None),
            Err(error) => return Err(error),
        };
        if received == 0 {
            return Ok(None);
        }

        // Split off the leading report-ID byte the field offsets assume.
        Ok(Some(if self.descriptor.uses_report_ids() {
            Report {
                id: buf[0],
                payload: &buf[1..received],
            }
        } else {
            Report {
                id: 0,
                payload: &buf[..received],
            }
        }))
    }
}

/// Converts a report endpoint's `bInterval` into a polling period in
/// milliseconds. The unit depends on the device's speed: frames (1ms) for a
/// full/low-speed device — every controller on this board, since they sit
/// behind the on-board high-speed hub — but an exponent of 125µs
/// microframes (`2^(bInterval-1)`) for a high-speed one, which is what a
/// device connected directly to the root port would be.
fn poll_interval_ms(endpoint: &ControlEndpoint, interval: u8) -> u32 {
    let high_speed = endpoint.split.is_none() && !endpoint.low_speed;
    let ms = if high_speed {
        // Exponent capped so the shift can't overflow; 2^15 microframes is
        // already 4 seconds, far past any real endpoint's interval.
        (1u32 << interval.clamp(1, 16).saturating_sub(1)) / 8
    } else {
        interval as u32
    };
    ms.max(1)
}
