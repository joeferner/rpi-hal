//! Typed, zero-copy views over the USB descriptors read during
//! enumeration (USB 2.0 spec §9.5–9.6), so callers work in terms of
//! named fields instead of raw byte offsets. Each view borrows the
//! bytes it was parsed from; none of them own or copy the descriptor
//! data (except [`DeviceDescriptor`], which extracts a few scalar
//! fields out of a fixed-size device descriptor).

/// `bDescriptorType` for a CONFIGURATION descriptor.
const DESCRIPTOR_TYPE_CONFIGURATION: u8 = 2;
/// `bDescriptorType` for an INTERFACE descriptor.
const DESCRIPTOR_TYPE_INTERFACE: u8 = 4;
/// `bDescriptorType` for an ENDPOINT descriptor.
const DESCRIPTOR_TYPE_ENDPOINT: u8 = 5;

/// The scalar fields of an 18-byte device descriptor (USB 2.0 spec
/// §9.6.1) this stack actually uses — enough to identify a device and
/// address its endpoint 0.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeviceDescriptor {
    /// `bDeviceClass` — the device's class code (`9` for a hub, `0` when
    /// the class is declared per-interface instead).
    pub device_class: u8,
    /// `bMaxPacketSize0` — endpoint 0's max packet size, needed to talk
    /// to the device with transfers larger than the 8-byte default.
    pub max_packet_size0: u8,
    /// `idVendor`.
    pub vendor_id: u16,
    /// `idProduct`.
    pub product_id: u16,
}

impl DeviceDescriptor {
    /// Extracts the fields above from a full 18-byte device descriptor
    /// (as returned by
    /// [`control::get_device_descriptor`](crate::usb::control::get_device_descriptor)).
    pub fn from_bytes(bytes: &[u8; 18]) -> Self {
        Self {
            device_class: bytes[4],
            max_packet_size0: bytes[7],
            vendor_id: u16::from_le_bytes([bytes[8], bytes[9]]),
            product_id: u16::from_le_bytes([bytes[10], bytes[11]]),
        }
    }
}

/// Iterates the individual descriptors packed back-to-back in a
/// configuration descriptor block (the configuration descriptor itself
/// followed by its interface, endpoint, and class-specific descriptors —
/// USB 2.0 spec §9.5). Each item is one descriptor, `bLength` bytes long,
/// sliced out of the block; parse it with [`ConfigurationDescriptor`],
/// [`InterfaceDescriptor`], or [`EndpointDescriptor`] as appropriate for
/// its `bDescriptorType`. Iteration stops at the first malformed length
/// (a `bLength` of 0, or one that would run past the end of the block).
pub struct Descriptors<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Descriptors<'a> {
    /// Starts iterating the descriptors in `buf` (a configuration
    /// descriptor block, e.g. the bytes filled in by
    /// [`control::get_configuration_descriptor`](crate::usb::control::get_configuration_descriptor)).
    pub fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }
}

impl<'a> Iterator for Descriptors<'a> {
    type Item = &'a [u8];

    fn next(&mut self) -> Option<&'a [u8]> {
        // Every descriptor starts with `bLength`, `bDescriptorType`, so
        // at least two bytes must remain.
        if self.pos + 2 > self.buf.len() {
            return None;
        }
        let len = self.buf[self.pos] as usize;
        if len < 2 || self.pos + len > self.buf.len() {
            return None;
        }
        let descriptor = &self.buf[self.pos..self.pos + len];
        self.pos += len;
        Some(descriptor)
    }
}

/// A view over a configuration descriptor (USB 2.0 spec §9.6.3).
pub struct ConfigurationDescriptor<'a>(&'a [u8]);

impl<'a> ConfigurationDescriptor<'a> {
    /// Interprets `descriptor` as a configuration descriptor, returning
    /// `None` unless it's long enough and its `bDescriptorType` says so.
    pub fn parse(descriptor: &'a [u8]) -> Option<Self> {
        if descriptor.len() >= 9 && descriptor[1] == DESCRIPTOR_TYPE_CONFIGURATION {
            Some(Self(descriptor))
        } else {
            None
        }
    }

    /// `bConfigurationValue` — the value to pass to
    /// [`control::set_configuration`](crate::usb::control::set_configuration)
    /// to activate this configuration (not its index in the list).
    pub fn value(&self) -> u8 {
        self.0[5]
    }
}

/// A view over an interface descriptor (USB 2.0 spec §9.6.5).
pub struct InterfaceDescriptor<'a>(&'a [u8]);

impl<'a> InterfaceDescriptor<'a> {
    /// Interprets `descriptor` as an interface descriptor, returning
    /// `None` unless it's long enough and its `bDescriptorType` says so.
    pub fn parse(descriptor: &'a [u8]) -> Option<Self> {
        if descriptor.len() >= 9 && descriptor[1] == DESCRIPTOR_TYPE_INTERFACE {
            Some(Self(descriptor))
        } else {
            None
        }
    }

    /// `bInterfaceNumber`.
    pub fn number(&self) -> u8 {
        self.0[2]
    }

    /// `bInterfaceClass` (e.g. `3` for HID).
    pub fn class(&self) -> u8 {
        self.0[5]
    }

    /// `bInterfaceSubClass` (for HID, `1` means the boot-interface
    /// subclass — the fixed-format reports needing no report-descriptor
    /// parsing).
    pub fn subclass(&self) -> u8 {
        self.0[6]
    }

    /// `bInterfaceProtocol` (for a HID boot interface, `1` is a keyboard
    /// and `2` is a mouse).
    pub fn protocol(&self) -> u8 {
        self.0[7]
    }
}

/// A view over an endpoint descriptor (USB 2.0 spec §9.6.6).
pub struct EndpointDescriptor<'a>(&'a [u8]);

impl<'a> EndpointDescriptor<'a> {
    /// Interprets `descriptor` as an endpoint descriptor, returning
    /// `None` unless it's long enough and its `bDescriptorType` says so.
    pub fn parse(descriptor: &'a [u8]) -> Option<Self> {
        if descriptor.len() >= 7 && descriptor[1] == DESCRIPTOR_TYPE_ENDPOINT {
            Some(Self(descriptor))
        } else {
            None
        }
    }

    /// The endpoint number (the low four bits of `bEndpointAddress`,
    /// without the direction bit).
    pub fn number(&self) -> u8 {
        self.0[2] & 0x0f
    }

    /// Whether this is an IN endpoint (device→host — bit 7 of
    /// `bEndpointAddress`).
    pub fn is_in(&self) -> bool {
        self.0[2] & 0x80 != 0
    }

    /// Whether this is an interrupt endpoint (`bmAttributes` transfer
    /// type == interrupt).
    pub fn is_interrupt(&self) -> bool {
        self.0[3] & 0x03 == 3
    }

    /// Whether this is a bulk endpoint (`bmAttributes` transfer type ==
    /// bulk).
    pub fn is_bulk(&self) -> bool {
        self.0[3] & 0x03 == 2
    }

    /// `wMaxPacketSize`.
    pub fn max_packet_size(&self) -> u16 {
        u16::from_le_bytes([self.0[4], self.0[5]])
    }

    /// `bInterval` — how often the host should service this endpoint. For a
    /// full/low-speed interrupt endpoint (what the HID devices here are) the
    /// unit is a 1ms frame, so the value is the polling period in
    /// milliseconds; for a high-speed interrupt endpoint it's an exponent
    /// (`2^(bInterval-1)` 125µs microframes) instead.
    pub fn interval(&self) -> u8 {
        self.0[6]
    }
}
