//! HID (Human Interface Device) class support. The class covers
//! keyboards, mice, game controllers, and other input devices; there are
//! boot-protocol keyboard ([`keyboard`](crate::usb::hid::keyboard)) and
//! mouse ([`mouse`](crate::usb::hid::mouse)) drivers — the stage-2
//! milestone that validates the host controller, enumeration, and
//! interrupt transfers end to end by turning a real device's report
//! endpoint into input events — plus a gamepad driver
//! ([`gamepad`](crate::usb::hid::gamepad)) that goes through the full HID
//! report protocol instead.
//!
//! Those are the class's two ways of reading a device, and the drivers
//! here split along that line:
//!
//! - **Boot protocol** (keyboard, mouse): fixed-format reports defined by
//!   the HID spec itself, needing no report-descriptor parsing — the
//!   smallest real class driver, but available only from devices that
//!   declare the boot subclass, and limited to the fields the boot
//!   reports happen to carry.
//! - **Report protocol** (gamepad): the device declares its own report
//!   layout in a HID *report descriptor*, which the host parses
//!   ([`crate::hid_report`]) to learn where each control lives. More work,
//!   but it reads devices that support nothing else — which is every
//!   device outside the keyboard/mouse boot subclass.

/// Game controller driver, decoded through the HID report descriptor —
/// see [`gamepad::Gamepad`].
pub mod gamepad;
/// Boot-protocol keyboard driver — see [`keyboard::Keyboard`].
pub mod keyboard;
/// Boot-protocol mouse driver — see [`mouse::Mouse`].
pub mod mouse;

use crate::usb::descriptor::{Descriptors, EndpointDescriptor, InterfaceDescriptor};

/// `bInterfaceClass` value identifying a Human Interface Device.
const HID_CLASS: u8 = 3;

/// `bInterfaceSubClass` value for the HID "boot interface subclass" — a
/// device declares this only when it actually supports the boot protocol
/// (fixed-format reports). Required in addition to the class/protocol
/// match: a composite device can expose a non-boot interface that still
/// claims a keyboard/mouse `bInterfaceProtocol` (an illuminated keyboard
/// observed here advertises a protocol-2 "mouse" interface whose reports
/// are report-ID-prefixed, not boot format), and driving it as a boot
/// device reads garbage. Insisting on subclass 1 keeps to interfaces
/// whose reports really are the boot format these drivers parse.
const BOOT_INTERFACE_SUBCLASS: u8 = 1;

/// `SET_PROTOCOL` value selecting the boot protocol (fixed-format
/// reports), as opposed to the report protocol (`1`).
const BOOT_PROTOCOL: u8 = 0;

/// `bDescriptorType` of a HID class descriptor (HID spec §6.2.1) — the
/// per-interface descriptor that follows an interface descriptor in the
/// configuration block and lists the interface's subordinate descriptors
/// (chiefly the report descriptor and its length).
const HID_CLASS_DESCRIPTOR: u8 = 0x21;

/// `bDescriptorType` of a HID report descriptor, as listed in the HID
/// class descriptor's subordinate-descriptor table.
const HID_REPORT_DESCRIPTOR: u8 = 0x22;

/// Offset of the subordinate-descriptor table (pairs of
/// `bDescriptorType`, `wDescriptorLength`) within a HID class descriptor.
const SUBORDINATE_TABLE_OFFSET: usize = 6;

/// Size of one subordinate-descriptor table entry: a one-byte type and a
/// two-byte length.
const SUBORDINATE_ENTRY_LEN: usize = 3;

/// A HID interface and its interrupt-IN report endpoint, located in a
/// configuration descriptor — the common shape all three drivers key off,
/// differing only in which interfaces they accept (see [`Self::find`]).
struct HidInterface {
    /// `bInterfaceNumber` of the matched HID interface.
    interface: u8,
    /// Report endpoint number (without the direction bit).
    endpoint: u8,
    /// Report endpoint's max packet size.
    max_packet_size: u16,
    /// Report endpoint's `bInterval` — for the full/low-speed endpoints
    /// these devices have, its polling period in milliseconds.
    interval: u8,
    /// Length of the interface's HID report descriptor, from its HID class
    /// descriptor, or `0` if the interface declared none.
    report_descriptor_len: u16,
}

impl HidInterface {
    /// Finds the first HID interface in the configuration descriptor block
    /// `config` that `accept` approves of and that has an interrupt-IN
    /// endpoint. The class check (`bInterfaceClass == 3`) is applied here;
    /// `accept` refines it — the boot drivers insist on the boot subclass
    /// and their protocol ([`is_boot_interface`]), while the gamepad driver
    /// takes any HID interface and decides from its report descriptor.
    fn find(config: &[u8], accept: impl Fn(&InterfaceDescriptor) -> bool) -> Option<Self> {
        let mut interface = 0;
        let mut matches = false;
        let mut report_descriptor_len = 0;
        for descriptor in Descriptors::new(config) {
            if let Some(iface) = InterfaceDescriptor::parse(descriptor) {
                interface = iface.number();
                matches = iface.class() == HID_CLASS && accept(&iface);
                // Belongs to the interface just opened, not the last one.
                report_descriptor_len = 0;
            } else if matches && descriptor[1] == HID_CLASS_DESCRIPTOR {
                report_descriptor_len = report_descriptor_len_of(descriptor);
            } else if let Some(endpoint) = EndpointDescriptor::parse(descriptor) {
                if matches && endpoint.is_in() && endpoint.is_interrupt() {
                    return Some(Self {
                        interface,
                        endpoint: endpoint.number(),
                        max_packet_size: endpoint.max_packet_size(),
                        interval: endpoint.interval(),
                        report_descriptor_len,
                    });
                }
            }
        }
        None
    }
}

/// The report descriptor length a HID class descriptor declares, or `0` if
/// it lists no report descriptor. Its subordinate-descriptor table is
/// `bNumDescriptors` (type, length) pairs — normally just the report
/// descriptor, but a device may list optional physical descriptors too, so
/// the table is searched rather than indexed.
fn report_descriptor_len_of(descriptor: &[u8]) -> u16 {
    let mut offset = SUBORDINATE_TABLE_OFFSET;
    while offset + SUBORDINATE_ENTRY_LEN <= descriptor.len() {
        if descriptor[offset] == HID_REPORT_DESCRIPTOR {
            return u16::from_le_bytes([descriptor[offset + 1], descriptor[offset + 2]]);
        }
        offset += SUBORDINATE_ENTRY_LEN;
    }
    0
}

/// Whether `iface` is a HID boot interface with boot `protocol` (`1`
/// keyboard, `2` mouse) — the [`HidInterface::find`] filter the
/// boot-protocol drivers use. See [`BOOT_INTERFACE_SUBCLASS`] for why the
/// subclass has to match too.
fn is_boot_interface(iface: &InterfaceDescriptor, protocol: u8) -> bool {
    iface.subclass() == BOOT_INTERFACE_SUBCLASS && iface.protocol() == protocol
}
