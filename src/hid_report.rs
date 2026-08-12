//! A HID Report Descriptor parser — turns the self-describing descriptor bytes
//! (from [`sdp`](crate::bluetooth::sdp) on Classic, a Report Map
//! characteristic on LE, or a USB
//! [GET_DESCRIPTOR(REPORT)](crate::usb::control::get_report_descriptor)) into a
//! flat [`Field`](crate::hid_report::Field) map, so reports can be decoded without a hand-written
//! per-device layout.
//!
//! It sits at the crate root rather than under one transport because it is the
//! *same* parser for all of them: HID's descriptor format is defined by the
//! USB HID spec and reused verbatim by HID over Bluetooth, so the USB and
//! Bluetooth hosts here decode reports through this one module.
//!
//! This is pure logic with no transport dependency: feed it `&[u8]`, get back
//! the input fields — each with its report ID, bit offset, size, HID
//! `usage_page`/`usage`, and logical range. A consumer then reads each field
//! out of a received report by [`Field::extract`](crate::hid_report::Field::extract) and interprets it by its
//! usage (X/Y axes, buttons, hat, …).
//!
//! # What it models
//!
//! A HID descriptor is a stream of items that build up *global* state (usage
//! page, logical min/max, report size/count, report ID) and *local* state
//! (the usages for the next field), which a **Main** item (Input/Output/
//! Feature) then commits into fields. This parser tracks that state and emits
//! one [`Field`](crate::hid_report::Field) per **Input** element (the device→host direction — buttons,
//! axes). Output (rumble/LEDs) and Feature items advance nothing here and are
//! skipped, as are constant (padding) fields.
//!
//! # Scope
//!
//! Enough for input devices: Input main items, the global/local items they
//! need, and both variable fields (one usage each — axes, individual buttons)
//! and the usage-range form (`Usage Minimum`/`Maximum`, e.g. Buttons 1–10).
//! Collections are followed only far enough to report the descriptor's
//! Application usage — what kind of device it is
//! ([`ReportDescriptor::application_usage`](crate::hid_report::ReportDescriptor::application_usage)) — since a field's meaning comes
//! from its own usage, not the collection nesting around it. Not handled:
//! Push/Pop of global state, array (non-variable) inputs like a keyboard
//! keycode array, and long items — none of which a game controller's input
//! reports use. Bit offsets are relative to the report *payload*; when
//! [`ReportDescriptor::uses_report_ids`](crate::hid_report::ReportDescriptor::uses_report_ids) is set, that payload starts after the
//! one-byte
//! report ID.

/// HID usage page `Generic Desktop` — axes, hat switches, the gamepad usage.
pub const USAGE_PAGE_GENERIC_DESKTOP: u16 = 0x01;
/// HID usage page `Button`.
pub const USAGE_PAGE_BUTTON: u16 = 0x09;

/// `Generic Desktop` usage `Mouse` (an application-collection usage — see
/// [`ReportDescriptor::application_usage`]).
pub const USAGE_MOUSE: u16 = 0x02;
/// `Generic Desktop` usage `Joystick` (an application-collection usage).
pub const USAGE_JOYSTICK: u16 = 0x04;
/// `Generic Desktop` usage `Gamepad` (an application-collection usage).
pub const USAGE_GAMEPAD: u16 = 0x05;
/// `Generic Desktop` usage `Keyboard` (an application-collection usage).
pub const USAGE_KEYBOARD: u16 = 0x06;
/// `Generic Desktop` usage `Multi-axis Controller` (an application-collection
/// usage — what some controllers declare instead of
/// [`USAGE_GAMEPAD`]/[`USAGE_JOYSTICK`]).
pub const USAGE_MULTI_AXIS: u16 = 0x08;

/// `Generic Desktop` usage `X`.
pub const USAGE_X: u16 = 0x30;
/// `Generic Desktop` usage `Y`.
pub const USAGE_Y: u16 = 0x31;
/// `Generic Desktop` usage `Z`.
pub const USAGE_Z: u16 = 0x32;
/// `Generic Desktop` usage `Rx`.
pub const USAGE_RX: u16 = 0x33;
/// `Generic Desktop` usage `Ry`.
pub const USAGE_RY: u16 = 0x34;
/// `Generic Desktop` usage `Rz`.
pub const USAGE_RZ: u16 = 0x35;
/// `Generic Desktop` usage `Hat Switch` (the D-pad).
pub const USAGE_HAT_SWITCH: u16 = 0x39;

/// Maximum number of input [`Field`]s a [`ReportDescriptor`] holds. Comfortably
/// covers a gamepad (a dozen or so axes/buttons across a few report IDs);
/// fields past this are dropped.
pub const MAX_FIELDS: usize = 64;
/// Maximum distinct report IDs whose bit offsets are tracked while parsing.
const MAX_REPORTS: usize = 8;
/// Maximum usages buffered from local items before a Main item consumes them.
const MAX_USAGES: usize = 16;

// --- HID item type codes (bits 3:2 of an item's prefix byte) ---
/// Main item (Input/Output/Feature/Collection).
const ITEM_TYPE_MAIN: u8 = 0;
/// Global item (usage page, logical range, report size/count/ID).
const ITEM_TYPE_GLOBAL: u8 = 1;
/// Local item (usage, usage min/max).
const ITEM_TYPE_LOCAL: u8 = 2;

// --- Main item tags (bits 7:4) ---
/// Input main item.
const MAIN_INPUT: u8 = 0x8;
/// Collection main item (opens a grouping of the items that follow).
const MAIN_COLLECTION: u8 = 0xa;

/// Collection type `Application` — the outermost grouping of a HID device's
/// controls, whose usage says what kind of device it is (a gamepad, a mouse,
/// …). See [`ReportDescriptor::application_usage`].
const COLLECTION_APPLICATION: u32 = 0x01;

// --- Global item tags ---
/// Usage Page.
const GLOBAL_USAGE_PAGE: u8 = 0x0;
/// Logical Minimum.
const GLOBAL_LOGICAL_MIN: u8 = 0x1;
/// Logical Maximum.
const GLOBAL_LOGICAL_MAX: u8 = 0x2;
/// Report Size (bits per field element).
const GLOBAL_REPORT_SIZE: u8 = 0x7;
/// Report ID.
const GLOBAL_REPORT_ID: u8 = 0x8;
/// Report Count (number of field elements).
const GLOBAL_REPORT_COUNT: u8 = 0x9;

// --- Local item tags ---
/// Usage.
const LOCAL_USAGE: u8 = 0x0;
/// Usage Minimum (start of a usage range).
const LOCAL_USAGE_MIN: u8 = 0x1;
/// Usage Maximum (end of a usage range).
const LOCAL_USAGE_MAX: u8 = 0x2;

/// Input item flag bit `Constant` — a padding field with no data.
const INPUT_CONSTANT: u32 = 0x01;
/// Input item flag bit `Variable` — each element is its own field (vs an
/// array of usage indices).
const INPUT_VARIABLE: u32 = 0x02;
/// Input item flag bit `Relative` — values are deltas, not absolute positions.
const INPUT_RELATIVE: u32 = 0x04;

/// One input field decoded from the descriptor: where it sits in a report and
/// what it means.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Field {
    /// The report ID this field belongs to (`0` if the descriptor uses no
    /// report IDs).
    pub report_id: u8,
    /// Bit offset of the field within the report payload (excluding the
    /// report-ID byte when [`ReportDescriptor::uses_report_ids`] is set).
    pub bit_offset: u16,
    /// Field width in bits.
    pub bit_size: u8,
    /// HID usage page (e.g. [`USAGE_PAGE_GENERIC_DESKTOP`],
    /// [`USAGE_PAGE_BUTTON`]).
    pub usage_page: u16,
    /// HID usage within the page (e.g. [`USAGE_X`], or a button number on the
    /// Button page).
    pub usage: u16,
    /// Logical minimum the device reports for this field.
    pub logical_min: i32,
    /// Logical maximum the device reports for this field.
    pub logical_max: i32,
    /// Whether the field is relative (a delta) rather than absolute.
    pub relative: bool,
}

impl Field {
    /// Reads this field's raw value out of a report `payload` (the report
    /// bytes *after* the report-ID byte, when IDs are used), as an unsigned
    /// integer. Returns `0` if the field runs past the payload.
    ///
    /// Little-endian bit packing (HID's convention): bit 0 of the field is the
    /// least-significant bit at [`Self::bit_offset`]. Widths up to 32 bits.
    pub fn extract(&self, payload: &[u8]) -> u32 {
        let mut value: u32 = 0;
        for i in 0..self.bit_size as usize {
            let bit = self.bit_offset as usize + i;
            let byte = bit / 8;
            if byte >= payload.len() {
                return 0;
            }
            if payload[byte] & (1 << (bit % 8)) != 0 {
                value |= 1 << i;
            }
        }
        value
    }

    /// Reads this field as a signed value, sign-extended from its width when
    /// [`Self::logical_min`] is negative; otherwise the same as
    /// [`Self::extract`] cast to `i32`.
    pub fn extract_signed(&self, payload: &[u8]) -> i32 {
        let raw = self.extract(payload);
        if self.logical_min < 0 && self.bit_size < 32 && raw & (1 << (self.bit_size - 1)) != 0 {
            // Negative: sign-extend above the field's top bit.
            (raw | (!0u32 << self.bit_size)) as i32
        } else {
            raw as i32
        }
    }
}

/// A parsed HID report descriptor: the input [`Field`]s it declares.
pub struct ReportDescriptor {
    /// The input fields, in descriptor order.
    fields: [Field; MAX_FIELDS],
    /// Number of valid entries in `fields`.
    count: usize,
    /// Whether the descriptor uses report IDs (so wire reports carry a leading
    /// ID byte).
    uses_report_ids: bool,
    /// `true` if there were more fields than [`MAX_FIELDS`] to store.
    overflow: bool,
    /// Usage page / usage of the first Application collection — the device's
    /// declared kind. See [`Self::application_usage`].
    application: Option<(u16, u16)>,
}

impl ReportDescriptor {
    /// Parses `desc` into its input fields.
    pub fn parse(desc: &[u8]) -> Self {
        let mut out = ReportDescriptor {
            fields: [Field {
                report_id: 0,
                bit_offset: 0,
                bit_size: 0,
                usage_page: 0,
                usage: 0,
                logical_min: 0,
                logical_max: 0,
                relative: false,
            }; MAX_FIELDS],
            count: 0,
            uses_report_ids: false,
            overflow: false,
            application: None,
        };

        // Global state.
        let mut usage_page: u16 = 0;
        let mut logical_min: i32 = 0;
        let mut logical_max: i32 = 0;
        let mut report_size: u8 = 0;
        let mut report_count: u16 = 0;
        let mut report_id: u8 = 0;
        // Local state (cleared after each Main item).
        let mut usages = [0u16; MAX_USAGES];
        let mut usage_len = 0usize;
        let mut usage_min: u16 = 0;
        let mut usage_max: u16 = 0;
        let mut have_usage_range = false;
        // Per-report-ID running bit offset for Input fields.
        let mut report_bits = [(0u8, 0u16); MAX_REPORTS];
        let mut report_bits_len = 0usize;

        let mut i = 0;
        while i < desc.len() {
            let prefix = desc[i];
            let size = match prefix & 0x03 {
                0 => 0,
                1 => 1,
                2 => 2,
                _ => 4,
            };
            let item_type = (prefix >> 2) & 0x03;
            let tag = prefix >> 4;
            let data_start = i + 1;
            if data_start + size > desc.len() {
                break;
            }
            let data = &desc[data_start..data_start + size];
            let uval = le_u32(data);
            let sval = sign_extend(data);

            match item_type {
                ITEM_TYPE_GLOBAL => match tag {
                    GLOBAL_USAGE_PAGE => usage_page = uval as u16,
                    GLOBAL_LOGICAL_MIN => logical_min = sval,
                    GLOBAL_LOGICAL_MAX => logical_max = sval,
                    GLOBAL_REPORT_SIZE => report_size = uval as u8,
                    GLOBAL_REPORT_ID => {
                        report_id = uval as u8;
                        out.uses_report_ids = true;
                    }
                    GLOBAL_REPORT_COUNT => report_count = uval as u16,
                    _ => {}
                },
                ITEM_TYPE_LOCAL => match tag {
                    LOCAL_USAGE => {
                        if usage_len < usages.len() {
                            usages[usage_len] = uval as u16;
                            usage_len += 1;
                        }
                    }
                    LOCAL_USAGE_MIN => {
                        usage_min = uval as u16;
                        have_usage_range = true;
                    }
                    LOCAL_USAGE_MAX => {
                        usage_max = uval as u16;
                        have_usage_range = true;
                    }
                    _ => {}
                },
                ITEM_TYPE_MAIN => {
                    if tag == MAIN_COLLECTION
                        && uval == COLLECTION_APPLICATION
                        && out.application.is_none()
                    {
                        // The collection's usage is the local usage in effect
                        // (`Usage (Gamepad)` immediately before it, in every
                        // real descriptor); it says what kind of device this is.
                        out.application = Some((
                            usage_page,
                            usages[..usage_len].first().copied().unwrap_or(0),
                        ));
                    }
                    if tag == MAIN_INPUT {
                        emit_input(
                            &mut out,
                            uval,
                            report_id,
                            report_size,
                            report_count,
                            usage_page,
                            logical_min,
                            logical_max,
                            &usages[..usage_len],
                            have_usage_range,
                            usage_min,
                            usage_max,
                            &mut report_bits,
                            &mut report_bits_len,
                        );
                    }
                    // Every Main item consumes and clears local state.
                    usage_len = 0;
                    usage_min = 0;
                    usage_max = 0;
                    have_usage_range = false;
                }
                _ => {}
            }
            i = data_start + size;
        }
        out
    }

    /// The parsed input fields, in descriptor order.
    pub fn fields(&self) -> &[Field] {
        &self.fields[..self.count]
    }

    /// Whether the descriptor uses report IDs — if so, each wire report begins
    /// with a one-byte report ID and the field bit offsets are into the bytes
    /// after it.
    pub fn uses_report_ids(&self) -> bool {
        self.uses_report_ids
    }

    /// `true` if the descriptor declared more input fields than [`MAX_FIELDS`]
    /// (the extras were dropped).
    pub fn overflowed(&self) -> bool {
        self.overflow
    }

    /// The `(usage_page, usage)` of the descriptor's first Application
    /// collection — what kind of device it declares itself to be
    /// ([`USAGE_GAMEPAD`], [`USAGE_MOUSE`], [`USAGE_KEYBOARD`], …, on
    /// [`USAGE_PAGE_GENERIC_DESKTOP`]), or `None` if it opened no Application
    /// collection.
    ///
    /// This is how a host tells a gamepad from a keyboard when the transport
    /// itself doesn't say (a USB HID interface outside the boot subclass
    /// declares neither in its interface descriptor — see
    /// [the USB gamepad driver](crate::usb::hid::gamepad)). Only the first
    /// collection is reported: a device exposing several distinct functions
    /// through one descriptor declares one Application collection each, and
    /// the first is its primary one.
    pub fn application_usage(&self) -> Option<(u16, u16)> {
        self.application
    }

    /// Whether this descriptor declares a game controller — an Application
    /// collection of [`USAGE_GAMEPAD`], [`USAGE_JOYSTICK`], or
    /// [`USAGE_MULTI_AXIS`] (which are interchangeable in practice; which one a
    /// controller picks is a vendor choice, not a difference in what its
    /// reports contain).
    pub fn is_gamepad(&self) -> bool {
        matches!(
            self.application,
            Some((USAGE_PAGE_GENERIC_DESKTOP, USAGE_GAMEPAD))
                | Some((USAGE_PAGE_GENERIC_DESKTOP, USAGE_JOYSTICK))
                | Some((USAGE_PAGE_GENERIC_DESKTOP, USAGE_MULTI_AXIS))
        )
    }

    /// Finds the first field for `report_id` with the given usage — a
    /// convenience for pulling a known control (an axis, the hat) out of the
    /// map. Buttons share a usage page with per-button usages, so match those
    /// by iterating [`Self::fields`] directly.
    pub fn find(&self, report_id: u8, usage_page: u16, usage: u16) -> Option<&Field> {
        self.fields()
            .iter()
            .find(|f| f.report_id == report_id && f.usage_page == usage_page && f.usage == usage)
    }
}

/// Commits one Input main item into fields, advancing the report's bit offset.
#[allow(clippy::too_many_arguments)]
fn emit_input(
    out: &mut ReportDescriptor,
    flags: u32,
    report_id: u8,
    report_size: u8,
    report_count: u16,
    usage_page: u16,
    logical_min: i32,
    logical_max: i32,
    usages: &[u16],
    have_usage_range: bool,
    usage_min: u16,
    usage_max: u16,
    report_bits: &mut [(u8, u16)],
    report_bits_len: &mut usize,
) {
    let base = report_bit_offset(report_bits, report_bits_len, report_id);
    let total_bits = report_count * report_size as u16;

    // Constant fields are padding — advance the offset but emit nothing.
    // Array (non-variable) inputs aren't modelled; advance and skip too.
    if flags & INPUT_CONSTANT == 0 && flags & INPUT_VARIABLE != 0 {
        let relative = flags & INPUT_RELATIVE != 0;
        for k in 0..report_count as usize {
            let usage = if have_usage_range {
                (usage_min as usize + k).min(usage_max as usize) as u16
            } else if !usages.is_empty() {
                usages[k.min(usages.len() - 1)]
            } else {
                0
            };
            let bit_offset = base + (k as u16) * report_size as u16;
            if out.count < out.fields.len() {
                out.fields[out.count] = Field {
                    report_id,
                    bit_offset,
                    bit_size: report_size,
                    usage_page,
                    usage,
                    logical_min,
                    logical_max,
                    relative,
                };
                out.count += 1;
            } else {
                out.overflow = true;
            }
        }
    }

    advance_report_bits(report_bits, report_bits_len, report_id, total_bits);
}

/// The current Input bit offset for `report_id` (0 the first time it's seen).
fn report_bit_offset(bits: &[(u8, u16)], len: &usize, report_id: u8) -> u16 {
    bits[..*len]
        .iter()
        .find(|(id, _)| *id == report_id)
        .map(|(_, b)| *b)
        .unwrap_or(0)
}

/// Advances the Input bit offset for `report_id` by `delta`, creating its slot
/// on first use.
fn advance_report_bits(bits: &mut [(u8, u16)], len: &mut usize, report_id: u8, delta: u16) {
    if let Some(slot) = bits[..*len].iter_mut().find(|(id, _)| *id == report_id) {
        slot.1 = slot.1.saturating_add(delta);
    } else if *len < bits.len() {
        bits[*len] = (report_id, delta);
        *len += 1;
    }
}

/// Reads up to 4 little-endian bytes as an unsigned integer.
fn le_u32(data: &[u8]) -> u32 {
    let mut v = 0u32;
    for (k, b) in data.iter().enumerate().take(4) {
        v |= (*b as u32) << (8 * k);
    }
    v
}

/// Reads up to 4 little-endian bytes as a signed integer, sign-extended from
/// the item's width (HID logical values are signed).
fn sign_extend(data: &[u8]) -> i32 {
    match data.len() {
        1 => data[0] as i8 as i32,
        2 => i16::from_le_bytes([data[0], data[1]]) as i32,
        4 => i32::from_le_bytes([data[0], data[1], data[2], data[3]]),
        _ => 0,
    }
}
