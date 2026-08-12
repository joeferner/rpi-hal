//! Shared HID gamepad decoding for the game-controller examples.
//!
//! `bt_hid_gamepad.rs` (a Classic Bluetooth controller) and `usb_gamepad.rs`
//! (a USB one) differ only in how they reach a controller and get its report
//! descriptor. Once they have one, decoding is identical — that is the whole
//! point of a report descriptor — so the decode/print half lives here, generic
//! over the console it writes to (the Bluetooth examples must use the mini
//! UART, since the PL011 is committed to the Bluetooth HCI; the USB one uses
//! the PL011).
//!
//! Nothing here knows any device: [`print_fields`] dumps the parsed field map,
//! and [`Decoder::print_changes`] turns each live report into a line of
//! calibrated axis deflections, a D-pad direction, and pressed button numbers,
//! printed only when the decoded state actually changes.
//!
//! This lives in a subdirectory so Cargo doesn't build it as its own example
//! binary (only top-level files in `examples/` become examples). Include it
//! with `#[path = "common/hid_gamepad.rs"] mod hid_gamepad;`.

// Not every example uses every helper — that's expected for shared support code.
#![allow(dead_code)]

use core::fmt::Write;
use rpi_hal::hid_report::{
    Field, ReportDescriptor, MAX_FIELDS, USAGE_HAT_SWITCH, USAGE_PAGE_BUTTON,
    USAGE_PAGE_GENERIC_DESKTOP, USAGE_RX, USAGE_RY, USAGE_RZ, USAGE_X, USAGE_Y, USAGE_Z,
};

/// Axis offsets within this fraction of full-scale (in the ±100% display units)
/// read as centered, so resting-stick jitter doesn't flood the console.
const AXIS_DEADZONE_PCT: i32 = 8;

/// HID usage page `Simulation Controls` — where some gamepads (e.g. the 8BitDo
/// SN30 Pro+) put their analog triggers, as `Accelerator`/`Brake` pedals.
const USAGE_PAGE_SIMULATION: u16 = 0x02;
/// `Simulation` usage `Throttle`.
const SIM_THROTTLE: u16 = 0xbb;
/// `Simulation` usage `Accelerator`.
const SIM_ACCELERATOR: u16 = 0xc4;
/// `Simulation` usage `Brake`.
const SIM_BRAKE: u16 = 0xc5;

/// Bytes of state digest kept per report: one per field, plus the report ID.
const DIGEST_LEN: usize = MAX_FIELDS + 1;

/// Whether a field is an analog axis we display as a deflection: anything a
/// nibble or wider that isn't a button and isn't the D-pad hat. A control's
/// physical role (which axis is a stick vs a trigger) isn't in the descriptor,
/// so axes are shown by HID usage, not a guessed role.
fn is_axis(f: &Field) -> bool {
    f.usage_page != USAGE_PAGE_BUTTON
        && !(f.usage_page == USAGE_PAGE_GENERIC_DESKTOP && f.usage == USAGE_HAT_SWITCH)
        && f.bit_size >= 4
}

/// Whether a field is the D-pad hat switch.
fn is_hat(f: &Field) -> bool {
    f.usage_page == USAGE_PAGE_GENERIC_DESKTOP && f.usage == USAGE_HAT_SWITCH
}

/// A short name for a control's HID usage — the Generic Desktop axes and the
/// Simulation-page pedals some pads put their analog triggers on. Anything else
/// falls back to its raw `page:usage` at the call site.
fn usage_label(page: u16, usage: u16) -> &'static str {
    match (page, usage) {
        (USAGE_PAGE_GENERIC_DESKTOP, USAGE_X) => "X",
        (USAGE_PAGE_GENERIC_DESKTOP, USAGE_Y) => "Y",
        (USAGE_PAGE_GENERIC_DESKTOP, USAGE_Z) => "Z",
        (USAGE_PAGE_GENERIC_DESKTOP, USAGE_RX) => "Rx",
        (USAGE_PAGE_GENERIC_DESKTOP, USAGE_RY) => "Ry",
        (USAGE_PAGE_GENERIC_DESKTOP, USAGE_RZ) => "Rz",
        (USAGE_PAGE_SIMULATION, SIM_THROTTLE) => "Throttle",
        (USAGE_PAGE_SIMULATION, SIM_ACCELERATOR) => "Accel",
        (USAGE_PAGE_SIMULATION, SIM_BRAKE) => "Brake",
        _ => "",
    }
}

/// A field's decoded value, interpreted signed or unsigned per its logical
/// range — the value already lies in `logical_min..=logical_max`.
fn field_value(f: &Field, payload: &[u8]) -> i32 {
    if f.logical_min < 0 {
        f.extract_signed(payload)
    } else {
        f.extract(payload) as i32
    }
}

/// Deflection of an axis toward its extremes as a signed percentage: `0` at the
/// axis's resting value `rest` (calibrated from the first report), `−100%` at
/// its logical minimum, `+100%` at its maximum. Because the rest point is
/// *measured* rather than assumed, a centered stick reads `±100%` and a trigger
/// — which rests at its minimum — reads `0%..100%`, with no per-device or
/// per-usage special-casing (which axis is a stick vs a trigger isn't in the
/// descriptor).
fn axis_deflection(f: &Field, v: i32, rest: i32) -> i32 {
    let span = if v >= rest {
        f.logical_max - rest
    } else {
        rest - f.logical_min
    };
    if span <= 0 {
        return 0;
    }
    ((v - rest) * 100 / span).clamp(-100, 100)
}

/// Names the D-pad direction from a HID hat value. Hats report `logical_min`
/// for north and increment clockwise; anything outside the range is centered.
fn hat_direction(f: &Field, v: i32) -> &'static str {
    if v < f.logical_min || v > f.logical_max {
        return "-";
    }
    match v - f.logical_min {
        0 => "N",
        1 => "NE",
        2 => "E",
        3 => "SE",
        4 => "S",
        5 => "SW",
        6 => "W",
        7 => "NW",
        _ => "-",
    }
}

/// Prints the parsed field map — every input field's report, bit position,
/// size, usage, and logical range — so the live decode below can be checked
/// against the descriptor when a control reads wrong.
pub fn print_fields<W: Write>(console: &mut W, rd: &ReportDescriptor) {
    let _ = writeln!(
        console,
        "parsed {} input field(s), report-ids={}, application={:?}:",
        rd.fields().len(),
        rd.uses_report_ids(),
        rd.application_usage(),
    );
    for f in rd.fields() {
        let name = if f.usage_page == USAGE_PAGE_BUTTON {
            "Button"
        } else if is_hat(f) {
            "Hat"
        } else {
            usage_label(f.usage_page, f.usage)
        };
        let _ = writeln!(
            console,
            "  rpt {:02X} @bit {:>3} size {:>2} {:#04x}:{:#04x} {:<6} log {}..{}{}",
            f.report_id,
            f.bit_offset,
            f.bit_size,
            f.usage_page,
            f.usage,
            name,
            f.logical_min,
            f.logical_max,
            if f.relative { " rel" } else { "" },
        );
    }
    if rd.overflowed() {
        let _ = writeln!(console, "  (field table overflowed -- some fields dropped)");
    }
}

/// Prints a controller's live input reports, decoded purely from its field map.
///
/// Holds the two pieces of state that needs: each field's *resting* value,
/// captured from the first report it appears in so deflection is measured from
/// rest rather than an assumed centre (see [`axis_deflection`]), and a digest of
/// the last decoded state so an untouched controller — which streams reports
/// continuously — doesn't flood the console.
pub struct Decoder {
    /// Each field's calibrated resting value, indexed by its position in
    /// `rd.fields()`.
    rest: [i32; MAX_FIELDS],
    /// Whether the field at that index has been calibrated yet.
    calibrated: [bool; MAX_FIELDS],
    /// Digest of the last state printed, and its length.
    last_digest: [u8; DIGEST_LEN],
    last_len: usize,
}

impl Decoder {
    /// A decoder with nothing calibrated and no state seen yet, so the first
    /// report of each kind calibrates its fields and prints.
    pub fn new() -> Self {
        Self {
            rest: [0; MAX_FIELDS],
            calibrated: [false; MAX_FIELDS],
            // No real digest starts with 0xff bytes, so the first report
            // always counts as a change.
            last_digest: [0xff; DIGEST_LEN],
            last_len: 0,
        }
    }

    /// Decodes one input report and prints it if the decoded state changed
    /// since the last printed one. `payload` is the report *after* its
    /// report-ID byte (`report_id` being `0` when the descriptor uses none) —
    /// what the field bit offsets are relative to.
    pub fn print_changes<W: Write>(
        &mut self,
        console: &mut W,
        rd: &ReportDescriptor,
        report_id: u8,
        payload: &[u8],
    ) {
        for (i, f) in rd.fields().iter().enumerate() {
            if f.report_id == report_id && !self.calibrated[i] {
                self.rest[i] = field_value(f, payload);
                self.calibrated[i] = true;
            }
        }

        let mut digest = [0u8; DIGEST_LEN];
        let len = self.digest(rd, report_id, payload, &mut digest);
        if digest[..len] == self.last_digest[..self.last_len] {
            return;
        }
        self.last_digest[..len].copy_from_slice(&digest[..len]);
        self.last_len = len;
        self.print(console, rd, report_id, payload);
    }

    /// Builds a coarse digest of the report's decoded state, so the console is
    /// only redrawn when something meaningful changes. Axes are bucketed by 10%
    /// of deflection so resting jitter doesn't register; buttons and the hat are
    /// kept exact. Returns the number of digest bytes used.
    fn digest(
        &self,
        rd: &ReportDescriptor,
        report_id: u8,
        payload: &[u8],
        out: &mut [u8; DIGEST_LEN],
    ) -> usize {
        out[0] = report_id;
        let mut n = 1;
        for (i, f) in rd.fields().iter().enumerate() {
            if f.report_id != report_id {
                continue;
            }
            if n >= out.len() {
                break;
            }
            let v = field_value(f, payload);
            out[n] = if is_axis(f) {
                (axis_deflection(f, v, self.rest[i]) / 10) as i8 as u8
            } else {
                v as u8
            };
            n += 1;
        }
        n
    }

    /// Prints one report: each analog axis as a calibrated deflection
    /// percentage (labelled by HID usage), the D-pad direction, and the set of
    /// pressed buttons. The layout comes entirely from `rd`.
    fn print<W: Write>(
        &self,
        console: &mut W,
        rd: &ReportDescriptor,
        report_id: u8,
        payload: &[u8],
    ) {
        let _ = write!(console, "rpt {report_id:02X}:");
        for (i, f) in rd.fields().iter().enumerate() {
            if f.report_id != report_id {
                continue;
            }
            let v = field_value(f, payload);
            if is_axis(f) {
                let d = axis_deflection(f, v, self.rest[i]);
                let d = if d.abs() <= AXIS_DEADZONE_PCT { 0 } else { d };
                let label = usage_label(f.usage_page, f.usage);
                if label.is_empty() {
                    let _ = write!(console, " {:#04x}:{:#04x}={d:>4}%", f.usage_page, f.usage);
                } else {
                    let _ = write!(console, " {label}={d:>4}%");
                }
            } else if is_hat(f) {
                let _ = write!(console, " dpad:{:<2}", hat_direction(f, v));
            }
        }
        // Pressed buttons, by their usage number (button 1, 2, ...).
        let _ = write!(console, " btn:[");
        let mut first = true;
        for f in rd
            .fields()
            .iter()
            .filter(|f| f.report_id == report_id && f.usage_page == USAGE_PAGE_BUTTON)
        {
            if f.extract(payload) != 0 {
                if !first {
                    let _ = write!(console, " ");
                }
                let _ = write!(console, "{}", f.usage);
                first = false;
            }
        }
        let _ = writeln!(console, "]");
    }
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}
