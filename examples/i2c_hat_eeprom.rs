//! Reads the HAT ID EEPROM over BSC0's GPIO0/1 routing (`ID_SD`/`ID_SC`,
//! pins 27/28 of the 40-pin header) and prints what the board says it is:
//! the header's atom count and image length, then the vendor info atom's
//! UUID, product id/version, vendor string and product string.
//!
//! Wiring: none. The EEPROM is on the add-on board, at the fixed address
//! `0x50` the HAT specification assigns it, and the Pi fits 1.8k pull-ups
//! on both lines. A Pi with nothing plugged in has nothing to answer, and
//! this example says so rather than hanging.
//!
//! This is the other BSC0 routing to `camera_probe.rs`'s — one controller,
//! two pin pairs, selected by which constructor is called
//! ([`rpi_hal::i2c::I2c::<rpi_hal::pac::BSC0>::init_id`] here,
//! [`rpi_hal::i2c::I2c::<rpi_hal::pac::BSC0>::init`] there) — so the two cannot
//! both be live in one program.
//!
//! Reading a byte range from a 24C-series EEPROM is a two-byte address
//! write followed by a read, and this driver puts a STOP between them
//! rather than a repeated start (see [`rpi_hal::i2c::I2c`]). That is fine
//! *here*, and the reason is worth knowing before reusing the pattern: the
//! address write carries no data byte, so the part latches its address
//! counter without starting an internal write cycle, and the counter
//! survives the STOP. A device that resets its register pointer on a STOP
//! would need the repeated start this driver doesn't have yet.
#![no_std]
#![no_main]

use core::fmt::Write as _;
use embedded_hal::i2c::I2c as _;
use rpi_hal::halt;
use rpi_hal::{i2c::I2c, pac, timer::Timer, uart::Uart};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// The address the HAT specification fixes for the ID EEPROM. It is the
/// only address expected on this bus, which is why this example addresses
/// it directly instead of scanning: a scan built on 1-byte reads would
/// find it, but would also be the wrong habit to teach on a bus whose
/// other participants are unknown add-on boards.
const EEPROM_ADDR: u8 = 0x50;

/// How much of the image to pull in one go. The header plus a vendor info
/// atom with generous strings fits comfortably; the GPIO map and device
/// tree atoms that follow aren't parsed here.
const IMAGE_LEN: usize = 256;

/// `struct atom_header` and the header both start at fixed offsets, so
/// the layout is spelled out as constants rather than a packed struct:
/// the image is little-endian bytes off a wire, and reading it by index
/// keeps the alignment question from arising at all.
///
/// Header: signature\[4\], version, reserved, numatoms\[2\], eeplen\[4\].
const HEADER_LEN: usize = 12;

/// The header's magic, `"R-Pi"` — the one check that says the bytes came
/// from a formatted ID EEPROM rather than from an unprogrammed part (all
/// `0xff`) or from a device that merely happens to answer at `0x50`.
const SIGNATURE: [u8; 4] = *b"R-Pi";

/// Atom header: type\[2\], count\[2\], dlen\[4\], then `dlen` bytes of data
/// of which the last two are a CRC-16.
const ATOM_HEADER_LEN: usize = 8;

/// The vendor info atom — the one this example reads. `0x0002` is the
/// GPIO map, `0x0003` a device tree blob, `0x0004` manufacturer custom
/// data; all three are skipped over.
const ATOM_VENDOR_INFO: u16 = 0x0001;

/// Vendor info payload before the two variable-length strings:
/// uuid\[16\], pid\[2\], pver\[2\], vslen, pslen.
const VENDOR_INFO_FIXED: usize = 22;

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    // 0x05dc (1500) is the reset-default divider: 100kHz at the typical
    // 150MHz core clock, and 100kHz is what the HAT specification
    // requires the ID EEPROM to work at. Nothing is gained by going
    // faster on a bus whose device was designed by somebody else.
    let mut i2c = I2c::<pac::BSC0>::init_id(&peripherals.GPIO, peripherals.BSC0, 0x05dc, &timer);

    let mut image = [0u8; IMAGE_LEN];
    if let Err(error) = read_from(&mut i2c, 0, &mut image) {
        let _ = writeln!(uart, "hat: no EEPROM at 0x{EEPROM_ADDR:02x}: {error:?}");
        let _ = writeln!(uart, "hat: is an add-on board with an ID EEPROM fitted?");
        halt();
    }

    if image[..4] != SIGNATURE {
        let _ = writeln!(
            uart,
            "hat: something answered at 0x{EEPROM_ADDR:02x}, but the first four \
             bytes are {:02x} {:02x} {:02x} {:02x}, not \"R-Pi\"",
            image[0], image[1], image[2], image[3]
        );
        halt();
    }

    let numatoms = u16_at(&image, 6);
    let eeplen = u32_at(&image, 8);
    let _ = writeln!(
        uart,
        "hat: format version {}, {numatoms} atom(s), {eeplen} bytes",
        image[4]
    );

    match vendor_info(&image, numatoms) {
        Some(atom) => print_vendor_info(&mut uart, atom),
        None => {
            let _ = writeln!(
                uart,
                "hat: no vendor info atom in the first {IMAGE_LEN} bytes"
            );
        }
    }

    halt();
}

/// Reads `buffer.len()` bytes starting at `offset` in the EEPROM: a
/// two-byte big-endian address write (the 24C32-and-larger parts the HAT
/// specification calls for all address this way), then a read, which the
/// part answers from its address counter and auto-increments through.
fn read_from(
    i2c: &mut I2c<'_, pac::BSC0>,
    offset: u16,
    buffer: &mut [u8],
) -> Result<(), rpi_hal::i2c::Error> {
    i2c.write(EEPROM_ADDR, &offset.to_be_bytes())?;
    i2c.read(EEPROM_ADDR, buffer)
}

/// Walks the atom list for the vendor info atom, returning its data (the
/// `dlen` payload with the trailing CRC-16 already trimmed off).
///
/// The CRC is not checked. Getting it wrong would be worse than not
/// checking it — a bit-reversed CRC-16 implemented from memory reports a
/// mismatch on a perfectly good EEPROM, and the signature check plus the
/// bounds checks below already reject the failures that actually happen
/// on this bus (nothing fitted, an unprogrammed part, a misread length).
fn vendor_info(image: &[u8], numatoms: u16) -> Option<&[u8]> {
    let mut at = HEADER_LEN;
    for _ in 0..numatoms {
        // An atom whose header or payload runs past what was read is the
        // end of the road either way: the remaining atoms are beyond
        // `IMAGE_LEN`, not corrupt.
        if at + ATOM_HEADER_LEN > image.len() {
            return None;
        }
        let atom_type = u16_at(image, at);
        // `dlen` is a 32-bit length read out of a device that might be
        // unprogrammed or half-written, so every offset derived from it
        // is checked -- on AArch32 a plausible-looking 0xffffffff would
        // otherwise wrap `usize` and index somewhere real.
        let dlen = u32_at(image, at + 4) as usize;
        let start = at + ATOM_HEADER_LEN;
        let data = image.get(start..start.checked_add(dlen)?)?;

        // `dlen` counts the trailing CRC-16; the payload is what precedes
        // it.
        let payload = &data[..data.len().checked_sub(2)?];
        if atom_type == ATOM_VENDOR_INFO && payload.len() >= VENDOR_INFO_FIXED {
            return Some(payload);
        }
        at = start.checked_add(dlen)?;
    }
    None
}

/// Prints a vendor info payload: the 128-bit UUID, the product id and
/// version, and the two length-prefixed ASCII strings that follow the
/// fixed part.
fn print_vendor_info(uart: &mut Uart, payload: &[u8]) {
    // The UUID's 16 bytes are stored least significant first, which is the
    // reverse of the order its canonical text form is written in: an image
    // `eepmake` reports as 381a2d85-ae2d-40da-... starts `39 77 da 23` on
    // the wire. So the bytes are walked backwards here, and grouped 4-2-2-
    // 2-6, to print what `eepmake`, `eepdump` and (on a Linux Pi)
    // /proc/device-tree/hat/uuid all show.
    let _ = write!(uart, "hat: uuid ");
    for (position, byte) in payload[..16].iter().rev().enumerate() {
        if matches!(position, 4 | 6 | 8 | 10) {
            let _ = write!(uart, "-");
        }
        let _ = write!(uart, "{byte:02x}");
    }
    let _ = writeln!(uart);

    let _ = writeln!(
        uart,
        "hat: product 0x{:04x} version 0x{:04x}",
        u16_at(payload, 16),
        u16_at(payload, 18)
    );

    let vendor_len = payload[20] as usize;
    let product_len = payload[21] as usize;
    let strings = &payload[VENDOR_INFO_FIXED..];
    let _ = write!(uart, "hat: vendor \"");
    write_ascii(uart, strings.get(..vendor_len).unwrap_or_default());
    let _ = write!(uart, "\" product \"");
    write_ascii(
        uart,
        strings
            .get(vendor_len..vendor_len + product_len)
            .unwrap_or_default(),
    );
    let _ = writeln!(uart, "\"");
}

/// Writes EEPROM string bytes to the console, substituting `.` for
/// anything outside printable ASCII. The strings are whatever the board's
/// programmer put there, so nothing guarantees they are text at all, and
/// a stray control byte shouldn't be able to reprogram the terminal
/// reading this.
fn write_ascii(uart: &mut Uart, bytes: &[u8]) {
    for &byte in bytes {
        let printable = if (0x20..0x7f).contains(&byte) {
            byte as char
        } else {
            '.'
        };
        let _ = write!(uart, "{printable}");
    }
}

/// One little-endian `u16` out of the image at `at`.
fn u16_at(bytes: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([bytes[at], bytes[at + 1]])
}

/// One little-endian `u32` out of the image at `at`.
fn u32_at(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
}
