#![no_std]
#![no_main]

// Reads the attached display's EDID over the mailbox property interface
// and prints what it says about itself: who made it, which modes it
// supports, and which one it prefers.
//
// EDID is the display's own description of itself, which the firmware
// reads over the HDMI connector's I2C side-channel. That's strictly more
// than `Mailbox::display_size` reports -- that gives the one mode the
// firmware settled on at boot, this gives the menu it chose from. Useful
// for working out *why* a particular mode came up, and for finding out
// what else could be asked for instead.
//
// The parsing lives here rather than in the HAL on purpose. EDID is
// display-standard wire format with no Raspberry Pi anywhere in it, so
// it sits on the same side of the line as TCP/IP (smoltcp), FAT
// (embedded-sdmmc) and IR framing (the `infrared` crate): `rpi-hal`
// hands over the bytes with `Mailbox::edid_block`, and turning those
// bytes into meaning is the consumer's job. This example is that
// consumer, and covers the parts worth having -- see `parse_block0` and
// `parse_cea_extension`.
//
// A display with no EDID at all is a normal outcome, not a failure: a
// MIPI DSI panel has one fixed resolution and nothing to describe, and
// nothing plugged in has nobody to answer. Both report "no EDID".

use core::fmt::Write;
use rpi_hal::halt;
use rpi_hal::mailbox::{Mailbox, EDID_BLOCK_LEN};
use rpi_hal::{pac, uart::Uart};

/// The fixed 8-byte pattern every EDID starts with. Its whole purpose is
/// to be recognizable, so a block that doesn't begin with it isn't an
/// EDID and nothing further should be read out of it.
const EDID_HEADER: [u8; 8] = [0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00];

/// Extension-block tag marking a CEA-861 (now CTA-861) block -- the one
/// carrying the TV mode list this example is interested in. Other tags
/// exist (`0xF0` block map, `0x40` DisplayID, ...) and are reported but
/// not parsed.
const CEA_EXTENSION_TAG: u8 = 0x02;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // Print the mode actually in effect first, as the thing everything
    // below is context for. The overscan border has to be added back to
    // recover it -- see `examples/display_test_pattern.rs`, which
    // explains why at length.
    match (mailbox.display_size(), mailbox.overscan()) {
        (Ok(size), Ok(border)) => {
            let _ = writeln!(
                uart,
                "in effect now: {}x{} inside a border of {}/{}/{}/{} (t/b/l/r), \
                 so the mode is {}x{}",
                size.width,
                size.height,
                border.top,
                border.bottom,
                border.left,
                border.right,
                size.width + border.left + border.right,
                size.height + border.top + border.bottom
            );
        }
        (size, border) => {
            let _ = writeln!(
                uart,
                "in effect now: unknown (size {size:?}, overscan {border:?})"
            );
        }
    }

    let block0 = match mailbox.edid_block(0) {
        Ok(Some(block)) => block,
        Ok(None) => {
            let _ = writeln!(
                uart,
                "\nno EDID: the firmware has no block 0 to hand over. Expected \
                 on a DSI panel (fixed resolution, nothing to describe) or with \
                 nothing plugged in."
            );
            halt();
        }
        Err(e) => {
            let _ = writeln!(uart, "\nEDID block 0 could not be read: {e:?}");
            halt();
        }
    };

    let extensions = parse_block0(&mut uart, &block0);

    // Extension blocks are numbered from 1 up to the count block 0
    // declares. Read every one it claims rather than stopping at the
    // first CEA block: the mode list can be split across several, and a
    // block that isn't CEA still gets its tag reported so an unhandled
    // one is visible rather than silently skipped.
    for index in 1..=u32::from(extensions) {
        match mailbox.edid_block(index) {
            Ok(Some(block)) => parse_extension(&mut uart, index, &block),
            Ok(None) => {
                let _ = writeln!(
                    uart,
                    "\nblock {index}: declared by block 0 but the firmware has no \
                     such block"
                );
            }
            Err(e) => {
                let _ = writeln!(uart, "\nblock {index} could not be read: {e:?}");
            }
        }
    }

    let _ = writeln!(uart, "\nEDID read complete");
    halt();
}

/// Prints everything in the base block, returning the number of
/// extension blocks it declares (zero if the block doesn't validate, so
/// nothing further is read out of a block that isn't an EDID).
fn parse_block0(uart: &mut Uart, block: &[u8; EDID_BLOCK_LEN]) -> u8 {
    let _ = writeln!(uart, "\nblock 0 ({EDID_BLOCK_LEN} bytes):");
    hex_dump(uart, block);

    if block[..8] != EDID_HEADER {
        let _ = writeln!(
            uart,
            "not an EDID: the first 8 bytes aren't the fixed EDID header, so \
             nothing below would mean anything"
        );
        return 0;
    }

    // Every block carries a checksum byte chosen so the whole 128 bytes
    // sum to zero, mod 256. Report it but keep going: a display whose
    // checksum is wrong (they exist) still has readable timings, and
    // being unable to see them because of one byte would defeat the
    // point of a bring-up tool.
    let sum = block.iter().fold(0u8, |sum, &byte| sum.wrapping_add(byte));
    if sum == 0 {
        let _ = writeln!(uart, "header ok, checksum ok");
    } else {
        let _ = writeln!(
            uart,
            "header ok, checksum WRONG (bytes sum to 0x{sum:02x}, should be 0x00) \
             -- reading on anyway"
        );
    }

    // Manufacturer id: three 5-bit letters packed big-endian into bytes
    // 8..10, 1 = 'A'. This is the PNP vendor id ("DEL", "SAM", ...).
    let packed = u16::from_be_bytes([block[8], block[9]]);
    let letter = |shift: u16| b'A' + ((packed >> shift) & 0x1F) as u8 - 1;
    let manufacturer = [letter(10), letter(5), letter(0)];
    let _ = writeln!(
        uart,
        "manufacturer {}{}{}, product 0x{:04x}, serial 0x{:08x}",
        manufacturer[0] as char,
        manufacturer[1] as char,
        manufacturer[2] as char,
        u16::from_le_bytes([block[10], block[11]]),
        u32::from_le_bytes([block[12], block[13], block[14], block[15]])
    );
    // Byte 17 is the year offset from 1990. Byte 16 is the manufacture
    // week, or 0xFF to say byte 17 is a model year instead of a
    // manufacture date.
    if block[16] == 0xFF {
        let _ = writeln!(
            uart,
            "model year {}, EDID version {}.{}",
            1990 + u16::from(block[17]),
            block[18],
            block[19]
        );
    } else {
        let _ = writeln!(
            uart,
            "made week {} of {}, EDID version {}.{}",
            block[16],
            1990 + u16::from(block[17]),
            block[18],
            block[19]
        );
    }

    // Bytes 54..126 hold four 18-byte descriptors. Any of them can be a
    // detailed timing or a display descriptor (monitor name, serial
    // string, ...); a leading pixel clock of zero is what distinguishes
    // the latter. The first is special: it's the display's preferred
    // mode, which for a fixed-pixel panel is its native resolution.
    let _ = writeln!(uart, "\ndetailed timings (bytes 54..126):");
    let mut any = false;
    for (index, descriptor) in block[54..126].as_chunks::<18>().0.iter().enumerate() {
        if let Some(timing) = DetailedTiming::parse(descriptor) {
            any = true;
            let label = if index == 0 { "preferred" } else { "        " };
            let _ = writeln!(
                uart,
                "  [{}] {}x{}{} @ {}.{:03} Hz, pixel clock {} kHz, \
                 total {}x{}",
                label,
                timing.h_active,
                timing.height(),
                if timing.interlaced { "i" } else { "p" },
                timing.refresh_millihz() / 1000,
                timing.refresh_millihz() % 1000,
                timing.pixel_clock_khz,
                timing.h_total(),
                timing.v_total()
            );
        } else {
            // Report the descriptor kind rather than skipping silently,
            // so four descriptors always account for four lines.
            let _ = writeln!(
                uart,
                "  [        ] not a timing (display descriptor type 0x{:02x})",
                descriptor[3]
            );
        }
    }
    if !any {
        let _ = writeln!(uart, "  (none -- no descriptor carried a timing)");
    }

    print_established_timings(uart, &block[35..38]);
    print_standard_timings(uart, &block[38..54]);

    let extensions = block[126];
    let _ = writeln!(uart, "\nextension blocks declared: {extensions}");
    extensions
}

/// Prints the established-timings bitmap (bytes 35..38) -- a fixed menu
/// of legacy VESA/VGA modes, each present or absent as one bit. The last
/// byte's low 7 bits are manufacturer-reserved and carry no mode, so
/// they're not reported.
fn print_established_timings(uart: &mut Uart, bytes: &[u8]) {
    /// `(byte index within the bitmap, bit, width, height, refresh Hz,
    /// interlaced)`, in the order the bits are defined.
    const ESTABLISHED: [(usize, u8, u32, u32, u32, bool); 17] = [
        (0, 7, 720, 400, 70, false),
        (0, 6, 720, 400, 88, false),
        (0, 5, 640, 480, 60, false),
        (0, 4, 640, 480, 67, false),
        (0, 3, 640, 480, 72, false),
        (0, 2, 640, 480, 75, false),
        (0, 1, 800, 600, 56, false),
        (0, 0, 800, 600, 60, false),
        (1, 7, 800, 600, 72, false),
        (1, 6, 800, 600, 75, false),
        (1, 5, 832, 624, 75, false),
        (1, 4, 1024, 768, 87, true),
        (1, 3, 1024, 768, 60, false),
        (1, 2, 1024, 768, 70, false),
        (1, 1, 1024, 768, 75, false),
        (1, 0, 1280, 1024, 75, false),
        (2, 7, 1152, 870, 75, false),
    ];

    let _ = writeln!(uart, "\nestablished timings (bytes 35..38):");
    let mut any = false;
    for &(byte, bit, width, height, refresh, interlaced) in &ESTABLISHED {
        if bytes[byte] & (1 << bit) != 0 {
            any = true;
            let _ = writeln!(
                uart,
                "  {}x{}{} @ {} Hz",
                width,
                height,
                if interlaced { "i" } else { "p" },
                refresh
            );
        }
    }
    if !any {
        let _ = writeln!(uart, "  (none)");
    }
}

/// Prints the eight standard timing identifiers (bytes 38..54), two
/// bytes each: a horizontal size, an aspect ratio to derive the vertical
/// from, and a refresh rate. `0x01 0x01` marks an unused slot.
fn print_standard_timings(uart: &mut Uart, bytes: &[u8]) {
    let _ = writeln!(uart, "\nstandard timings (bytes 38..54):");
    let mut any = false;
    for pair in bytes.as_chunks::<2>().0 {
        // 0x01 0x01 is the defined "unused" filler; a zero first byte
        // would decode to a nonsense 248-pixel width, so treat it as
        // unused too rather than reporting it.
        if *pair == [0x01, 0x01] || pair[0] == 0x00 {
            continue;
        }
        any = true;
        let width = (u32::from(pair[0]) + 31) * 8;
        // Aspect ratio in the top two bits. The 0 encoding is 16:10 as
        // of EDID 1.3; it meant 1:1 in 1.2 and earlier, which no display
        // this could plausibly be attached to still reports.
        let height = match pair[1] >> 6 {
            0 => width * 10 / 16,
            1 => width * 3 / 4,
            2 => width * 4 / 5,
            _ => width * 9 / 16,
        };
        let _ = writeln!(
            uart,
            "  {}x{} @ {} Hz",
            width,
            height,
            u32::from(pair[1] & 0x3F) + 60
        );
    }
    if !any {
        let _ = writeln!(uart, "  (none)");
    }
}

/// Dispatches one extension block on its tag byte, parsing the CEA-861
/// ones and reporting the tag of anything else.
fn parse_extension(uart: &mut Uart, index: u32, block: &[u8; EDID_BLOCK_LEN]) {
    let _ = writeln!(uart, "\nblock {index} ({EDID_BLOCK_LEN} bytes):");
    hex_dump(uart, block);

    let sum = block.iter().fold(0u8, |sum, &byte| sum.wrapping_add(byte));
    if sum != 0 {
        let _ = writeln!(
            uart,
            "  checksum WRONG (bytes sum to 0x{sum:02x}) -- reading on anyway"
        );
    }

    if block[0] != CEA_EXTENSION_TAG {
        let _ = writeln!(
            uart,
            "  extension tag 0x{:02x}, not CEA-861 (0x{CEA_EXTENSION_TAG:02x}) -- \
             not parsed",
            block[0]
        );
        return;
    }
    parse_cea_extension(uart, block);
}

/// Prints a CEA-861 extension block: the TV modes it advertises as a
/// VIC list, and any detailed timings it carries of its own.
///
/// The block's layout is set by its third byte, the offset at which the
/// detailed timings start. Everything between byte 4 and there is the
/// "data block collection", a chain of length-prefixed blocks of which
/// the video data block (type 2) is the one carrying the mode list.
fn parse_cea_extension(uart: &mut Uart, block: &[u8; EDID_BLOCK_LEN]) {
    /// Data block collection tag for the video data block, whose payload
    /// is the list of short video descriptors (one mode each).
    const VIDEO_DATA_BLOCK: u8 = 2;

    let revision = block[1];
    let timings_at = usize::from(block[2]);
    let _ = writeln!(
        uart,
        "  CEA-861 extension, revision {revision}, native detailed timings {}",
        block[3] & 0x0F
    );

    // A timings offset of 0 means the block has neither a data block
    // collection nor detailed timings; 4 means no data block collection.
    // Anything outside the block is corrupt, and trusting it would slice
    // out of bounds below.
    if timings_at > EDID_BLOCK_LEN {
        let _ = writeln!(
            uart,
            "  timings offset {timings_at} is past the end of the block -- \
             not parsed"
        );
        return;
    }

    let _ = writeln!(uart, "  advertised TV modes:");
    let mut any = false;
    let mut at = 4;
    while at < timings_at {
        // Each data block starts with a header byte: type in the top
        // three bits, payload length in the low five.
        let tag = block[at] >> 5;
        let length = usize::from(block[at] & 0x1F);
        let payload_end = at + 1 + length;
        if payload_end > timings_at {
            let _ = writeln!(
                uart,
                "    data block at byte {at} claims {length} bytes, which runs \
                 past the timings offset -- stopping"
            );
            break;
        }
        if tag == VIDEO_DATA_BLOCK {
            for &svd in &block[at + 1..payload_end] {
                any = true;
                let (vic, native) = svd_to_vic(svd);
                match vic_mode(vic) {
                    Some((width, height, refresh, interlaced)) => {
                        let _ = writeln!(
                            uart,
                            "    VIC {}{}: {}x{}{} @ {} Hz",
                            vic,
                            if native { " (native)" } else { "" },
                            width,
                            height,
                            if interlaced { "i" } else { "p" },
                            refresh
                        );
                    }
                    None => {
                        let _ = writeln!(
                            uart,
                            "    VIC {}{}: not in this example's table",
                            vic,
                            if native { " (native)" } else { "" }
                        );
                    }
                }
            }
        }
        at = payload_end;
    }
    if !any {
        let _ = writeln!(uart, "    (none -- no video data block)");
    }

    // Detailed timings run from the declared offset to the checksum
    // byte, 18 at a time, ending early at an all-zero descriptor.
    if (4..EDID_BLOCK_LEN - 1).contains(&timings_at) {
        let _ = writeln!(uart, "  detailed timings (from byte {timings_at}):");
        let mut any = false;
        for descriptor in block[timings_at..EDID_BLOCK_LEN - 1].as_chunks::<18>().0 {
            match DetailedTiming::parse(descriptor) {
                Some(timing) => {
                    any = true;
                    let _ = writeln!(
                        uart,
                        "    {}x{}{} @ {}.{:03} Hz, pixel clock {} kHz, total {}x{}",
                        timing.h_active,
                        timing.height(),
                        if timing.interlaced { "i" } else { "p" },
                        timing.refresh_millihz() / 1000,
                        timing.refresh_millihz() % 1000,
                        timing.pixel_clock_khz,
                        timing.h_total(),
                        timing.v_total()
                    );
                }
                // A zero pixel clock here is padding to the end of the
                // block rather than a display descriptor, so stop.
                None => break,
            }
        }
        if !any {
            let _ = writeln!(uart, "    (none)");
        }
    }
}

/// One 18-byte detailed timing descriptor: an exact mode, given as
/// active and blanking counts plus a pixel clock, rather than as a name.
struct DetailedTiming {
    /// Pixel clock in kHz, stored as the descriptor's 10 kHz units
    /// multiplied out.
    pixel_clock_khz: u32,
    /// Visible pixels per line.
    h_active: u32,
    /// Non-visible pixels per line.
    h_blank: u32,
    /// Visible lines per frame.
    v_active: u32,
    /// Non-visible lines per frame.
    v_blank: u32,
    /// Whether the mode is interlaced.
    interlaced: bool,
}

impl DetailedTiming {
    /// Decodes one descriptor, or `None` if it isn't a timing at all.
    ///
    /// The four descriptor slots in a base block do double duty: a
    /// leading pixel clock of zero means the slot holds a *display*
    /// descriptor instead (monitor name, serial string, ...), which is a
    /// different layout entirely and carries no mode.
    ///
    /// The awkward bit is that the descriptor is packed to 18 bytes by
    /// splitting every field's high bits off into a shared byte -- so
    /// each value below is a low byte (or nibble) OR-ed with bits lifted
    /// out of bytes 4, 7, 10 and 11.
    fn parse(d: &[u8]) -> Option<Self> {
        if d.len() < 18 {
            return None;
        }
        let pixel_clock = u32::from(u16::from_le_bytes([d[0], d[1]]));
        if pixel_clock == 0 {
            return None;
        }
        Some(Self {
            pixel_clock_khz: pixel_clock * 10,
            h_active: u32::from(d[2]) | (u32::from(d[4] & 0xF0) << 4),
            h_blank: u32::from(d[3]) | (u32::from(d[4] & 0x0F) << 8),
            v_active: u32::from(d[5]) | (u32::from(d[7] & 0xF0) << 4),
            v_blank: u32::from(d[6]) | (u32::from(d[7] & 0x0F) << 8),
            interlaced: d[17] & 0x80 != 0,
        })
    }

    /// Pixels per line including blanking -- what the pixel clock is
    /// spent on.
    fn h_total(&self) -> u32 {
        self.h_active + self.h_blank
    }

    /// Visible lines per *frame*.
    ///
    /// An interlaced descriptor counts one field, so this doubles
    /// [`Self::v_active`] back into the frame height the mode is named
    /// for: 1080i stores 540, not 1080.
    fn height(&self) -> u32 {
        if self.interlaced {
            self.v_active * 2
        } else {
            self.v_active
        }
    }

    /// Lines per frame including blanking.
    ///
    /// An interlaced frame is two fields *plus* the half-line that makes
    /// the two fields fall between each other's lines, and the
    /// descriptor's counts leave that line out. Putting it back is what
    /// turns 1080i's 562-line field into the 1125-line frame its pixel
    /// clock is actually spread over -- without it the computed refresh
    /// comes out at 60.053 Hz instead of exactly 60.
    fn v_total(&self) -> u32 {
        let field = self.v_active + self.v_blank;
        if self.interlaced {
            field * 2 + 1
        } else {
            field
        }
    }

    /// Refresh rate in millihertz, computed rather than stored: the
    /// descriptor gives a pixel clock and the totals it's spread over,
    /// and the rate falls out as one divided by the other.
    ///
    /// Millihertz because whole Hz would round away the distinction that
    /// matters most in practice -- 59.94 Hz (the 1000/1001 rate) against
    /// a true 60 -- and the arithmetic runs in `u64` because a 1080p60
    /// clock scaled to millihertz overflows `u32` an order of magnitude
    /// over.
    ///
    /// For an interlaced mode this is the *field* rate, which is what
    /// such modes are named for -- 1080i60 is 60 fields per second, 30
    /// whole frames.
    fn refresh_millihz(&self) -> u32 {
        let total = u64::from(self.h_total()) * u64::from(self.v_total());
        if total == 0 {
            return 0;
        }
        let frames = u64::from(self.pixel_clock_khz) * 1_000_000 / total;
        (if self.interlaced { frames * 2 } else { frames }) as u32
    }
}

/// Splits a short video descriptor byte into its VIC and native flag.
///
/// The top bit means "this is the display's native mode", but only when
/// the low seven bits name a VIC in 1..=64. VICs above 127 exist and are
/// written as the whole byte, so treating bit 7 as a flag unconditionally
/// would halve every one of them into a different mode. This is the same
/// rule the kernel's EDID code applies.
fn svd_to_vic(svd: u8) -> (u8, bool) {
    if (1..=64).contains(&svd) || (129..=192).contains(&svd) {
        (svd & 0x7F, svd & 0x80 != 0)
    } else {
        (svd, false)
    }
}

/// Looks up a CEA-861 video identification code, returning `(width,
/// height, refresh in Hz, interlaced)`.
///
/// Covers VICs 1..=64 (every SD and HD mode) and 93..=107 (the 4K ones),
/// which is every code a display is likely to advertise. Deliberately
/// not covered, and reported as unrecognized rather than guessed at:
/// 65..=92, which are 64:27 anamorphic variants of modes already in the
/// table, and 108 and up, added by later revisions of the standard.
///
/// Refresh is the nominal rate. A VIC does not distinguish 60 Hz from
/// the 1000/1001 rate beside it (59.94) -- that follows from the pixel
/// clock the sink is actually driven at, so a mode listed here as 60 Hz
/// may run at 59.94. The detailed timings, which carry a real pixel
/// clock, are where that distinction survives.
fn vic_mode(vic: u8) -> Option<(u32, u32, u32, bool)> {
    /// `(VIC, width, height, refresh Hz, interlaced)`. Widths follow the
    /// convention that a mode written `720(1440)x480i` is listed at its
    /// 1440-sample width, the form it actually occupies on the wire.
    const VIC_MODES: [(u8, u32, u32, u32, bool); 79] = [
        (1, 640, 480, 60, false),
        (2, 720, 480, 60, false),
        (3, 720, 480, 60, false),
        (4, 1280, 720, 60, false),
        (5, 1920, 1080, 60, true),
        (6, 1440, 480, 60, true),
        (7, 1440, 480, 60, true),
        (8, 1440, 240, 60, false),
        (9, 1440, 240, 60, false),
        (10, 2880, 480, 60, true),
        (11, 2880, 480, 60, true),
        (12, 2880, 240, 60, false),
        (13, 2880, 240, 60, false),
        (14, 1440, 480, 60, false),
        (15, 1440, 480, 60, false),
        (16, 1920, 1080, 60, false),
        (17, 720, 576, 50, false),
        (18, 720, 576, 50, false),
        (19, 1280, 720, 50, false),
        (20, 1920, 1080, 50, true),
        (21, 1440, 576, 50, true),
        (22, 1440, 576, 50, true),
        (23, 1440, 288, 50, false),
        (24, 1440, 288, 50, false),
        (25, 2880, 576, 50, true),
        (26, 2880, 576, 50, true),
        (27, 2880, 288, 50, false),
        (28, 2880, 288, 50, false),
        (29, 1440, 576, 50, false),
        (30, 1440, 576, 50, false),
        (31, 1920, 1080, 50, false),
        (32, 1920, 1080, 24, false),
        (33, 1920, 1080, 25, false),
        (34, 1920, 1080, 30, false),
        (35, 2880, 480, 60, false),
        (36, 2880, 480, 60, false),
        (37, 2880, 576, 50, false),
        (38, 2880, 576, 50, false),
        (39, 1920, 1080, 50, true),
        (40, 1920, 1080, 100, true),
        (41, 1280, 720, 100, false),
        (42, 720, 576, 100, false),
        (43, 720, 576, 100, false),
        (44, 1440, 576, 100, true),
        (45, 1440, 576, 100, true),
        (46, 1920, 1080, 120, true),
        (47, 1280, 720, 120, false),
        (48, 720, 480, 120, false),
        (49, 720, 480, 120, false),
        (50, 1440, 480, 120, true),
        (51, 1440, 480, 120, true),
        (52, 720, 576, 200, false),
        (53, 720, 576, 200, false),
        (54, 1440, 576, 200, true),
        (55, 1440, 576, 200, true),
        (56, 720, 480, 240, false),
        (57, 720, 480, 240, false),
        (58, 1440, 480, 240, true),
        (59, 1440, 480, 240, true),
        (60, 1280, 720, 24, false),
        (61, 1280, 720, 25, false),
        (62, 1280, 720, 30, false),
        (63, 1920, 1080, 120, false),
        (64, 1920, 1080, 100, false),
        (93, 3840, 2160, 24, false),
        (94, 3840, 2160, 25, false),
        (95, 3840, 2160, 30, false),
        (96, 3840, 2160, 50, false),
        (97, 3840, 2160, 60, false),
        (98, 4096, 2160, 24, false),
        (99, 4096, 2160, 25, false),
        (100, 4096, 2160, 30, false),
        (101, 4096, 2160, 50, false),
        (102, 4096, 2160, 60, false),
        (103, 3840, 2160, 24, false),
        (104, 3840, 2160, 25, false),
        (105, 3840, 2160, 30, false),
        (106, 3840, 2160, 50, false),
        (107, 3840, 2160, 60, false),
    ];

    VIC_MODES
        .iter()
        .find(|&&(code, ..)| code == vic)
        .map(|&(_, width, height, refresh, interlaced)| (width, height, refresh, interlaced))
}

/// Dumps a block as hex, 16 bytes to a line with its byte offset.
///
/// Worth the eight lines of output: it's the raw evidence behind
/// everything else printed, so a display whose parsed output looks wrong
/// can be checked by hand (or pasted into `edid-decode`) without
/// reflashing.
fn hex_dump(uart: &mut Uart, block: &[u8]) {
    for (line, bytes) in block.chunks(16).enumerate() {
        let _ = write!(uart, "  {:3}:", line * 16);
        for byte in bytes {
            let _ = write!(uart, " {byte:02x}");
        }
        let _ = writeln!(uart);
    }
}
