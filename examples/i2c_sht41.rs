//! Reads temperature and relative humidity from a Sensirion SHT41
//! (SHT4x family) over I2C1 (GPIO2 SDA1/GPIO3 SCL1): prints the
//! sensor's serial number once, then a measurement every second.
//!
//! Wiring: SHT41 SDA -> GPIO2, SCL -> GPIO3, VDD -> 3V3, GND -> GND.
//! No external pull-ups are needed — GPIO2/3 carry the Pi's own fixed
//! 1.8k pull-ups to 3V3.
//!
//! The SHT4x protocol is a good fit for this driver's transaction model
//! (each `embedded_hal::i2c::Operation` is its own complete START...STOP,
//! not a repeated start — see [`rpi_hal::i2c::I2c`]): a measurement is a
//! one-byte *write* of the command, a STOP, a wait while the sensor
//! converts, and then a separate six-byte *read*. Nothing here needs a
//! repeated start, and the wait between the two halves is mandatory
//! rather than incidental — the sensor NAKs its address while a
//! conversion is still in flight, which this driver reports as
//! [`rpi_hal::i2c::Error::NoAcknowledge`].
//!
//! That NAK-until-a-result-is-pending behavior is also why `i2c_scan.rs`
//! does *not* list an SHT4x. That scan probes each address with a 1-byte
//! read, and a sensor with no measurement pending refuses reads, so it's
//! reported absent even though it's on the bus and acknowledging
//! commands. (`i2cdetect` finds these because it probes with a
//! zero-length write, which BCM2835's BSC can't issue at all — see
//! [`rpi_hal::i2c::Error::ZeroLengthUnsupported`].) A scan built on reads
//! enumerates what answers reads, which isn't the same as what's on the
//! bus; this example addresses the sensor directly instead of scanning
//! for it.
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

/// The SHT4x's 7-bit I2C address. `0x44` is the SHT41's fixed address
/// (as well as the SHT40-A/SHT45's); the -B variants answer at `0x45`
/// and there is a `0x46` option, so a board built around a different
/// part number needs this changed rather than a scan.
const SHT4X_ADDR: u8 = 0x44;

/// "Measure T & RH with high repeatability" — the slowest and quietest
/// of the three repeatability settings (`0xf6` medium, `0xe0` low),
/// which is what a once-a-second reading wants. None of the three heat
/// the sensor; the separate heater commands (`0x39` and friends) do,
/// and aren't used here.
const CMD_MEASURE_HIGH: u8 = 0xfd;

/// "Read serial number" — answers with two 16-bit words (each followed
/// by its own CRC) forming a 32-bit unique id. Used here as a
/// communication check: it's the one command whose reply is a known
/// constant for a given part, so a plausible temperature that's
/// actually a wiring or framing artifact can't masquerade as success.
const CMD_READ_SERIAL: u8 = 0x89;

/// Soft reset. Cheap insurance at startup: a warm reboot of the Pi
/// leaves the sensor powered and possibly mid-sequence.
const CMD_SOFT_RESET: u8 = 0x94;

/// Worst-case conversion time for [`CMD_MEASURE_HIGH`], rounded up from
/// the datasheet's 8.3 ms maximum. The read must not be issued before
/// this elapses — an early read is NAK'd, not merely stale.
const MEASURE_DELAY_MS: u32 = 10;

/// Worst-case soft-reset time, rounded up from the datasheet's 1 ms.
const RESET_DELAY_MS: u32 = 2;

/// Both replies used here are the same shape: two 16-bit big-endian
/// words, each with a trailing CRC byte.
const REPLY_LEN: usize = 6;

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    // 0x05dc (1500) is the reset-default divider: 100kHz standard mode
    // at BCM2835's typical 150MHz core clock. The SHT4x also does
    // 400kHz fast mode, but there's nothing to gain here — six bytes a
    // second isn't bus-limited, and the slower clock is the more
    // forgiving one for jumper wiring.
    let mut i2c = I2c::<pac::BSC1>::init(&peripherals.GPIO, peripherals.BSC1, 0x05dc, &timer);

    let _ = writeln!(uart, "sht41: resetting");
    if let Err(error) = i2c.write(SHT4X_ADDR, &[CMD_SOFT_RESET]) {
        let _ = writeln!(uart, "sht41: no response at 0x{SHT4X_ADDR:02x}: {error:?}");
        let _ = writeln!(uart, "sht41: check SDA->GPIO2, SCL->GPIO3, VDD=3V3");
        halt();
    }
    timer.delay_ms(RESET_DELAY_MS);

    match command_response(&mut i2c, &timer, CMD_READ_SERIAL, 1) {
        Ok(reply) => {
            // Two 16-bit words, most significant first, concatenated
            // into the 32-bit serial.
            let serial = ((word(&reply[0..2]) as u32) << 16) | word(&reply[3..5]) as u32;
            let _ = writeln!(uart, "sht41: serial 0x{serial:08x}");
        }
        Err(error) => {
            let _ = writeln!(uart, "sht41: serial read failed: {error}");
        }
    }

    loop {
        match command_response(&mut i2c, &timer, CMD_MEASURE_HIGH, MEASURE_DELAY_MS) {
            Ok(reply) => {
                let temperature = temperature_millicelsius(word(&reply[0..2]));
                let humidity = humidity_millipercent(word(&reply[3..5]));
                let _ = write!(uart, "sht41: ");
                write_milli(&mut uart, temperature);
                let _ = write!(uart, " C, ");
                write_milli(&mut uart, humidity);
                let _ = writeln!(uart, " %RH");
            }
            // Reported rather than fatal: a single bad reading (a NAK
            // from a conversion that ran long, a CRC failure from a
            // noise hit on a long jumper) says nothing about whether
            // the next one will work, and the point of the loop is to
            // keep sampling.
            Err(error) => {
                let _ = writeln!(uart, "sht41: measurement failed: {error}");
            }
        }

        timer.delay_ms(1000);
    }
}

/// Issues a one-byte command, waits `delay_ms` for the sensor to have
/// an answer ready, reads the six-byte reply, and checks both CRCs.
///
/// The wait is the whole reason this is a helper rather than a
/// `write_read`: `embedded_hal`'s `write_read` would issue the read
/// immediately, and the SHT4x NAKs a read taken before its conversion
/// finishes.
fn command_response(
    i2c: &mut I2c<'_, pac::BSC1>,
    timer: &Timer,
    command: u8,
    delay_ms: u32,
) -> Result<[u8; REPLY_LEN], Failure> {
    i2c.write(SHT4X_ADDR, &[command])
        .map_err(Failure::Command)?;
    timer.delay_ms(delay_ms);

    let mut reply = [0u8; REPLY_LEN];
    i2c.read(SHT4X_ADDR, &mut reply).map_err(Failure::Reply)?;

    // Each word carries its own CRC, and a mismatch means the bytes
    // can't be trusted at all -- an SHT4x's raw ticks have no
    // implausible values to sanity-check against, since the full 16-bit
    // range maps onto a legitimate temperature or humidity.
    if crc8(&reply[0..2]) != reply[2] || crc8(&reply[3..5]) != reply[5] {
        return Err(Failure::Crc);
    }
    Ok(reply)
}

/// Why a command/response exchange didn't produce six trustworthy
/// bytes. The bus errors are carried through rather than flattened into
/// a message because they say quite different things about the
/// hardware: `NoAcknowledge` on the read is the sensor refusing while
/// it converts, `Incomplete { received, .. }` is a sensor that started
/// answering and stopped, and `Timeout` is a bus nobody is driving at
/// all (SDA shorted low, or a slave still holding it).
enum Failure {
    /// The command byte wasn't acknowledged.
    Command(rpi_hal::i2c::Error),
    /// The reply didn't arrive, or arrived short.
    Reply(rpi_hal::i2c::Error),
    /// Six bytes arrived, but at least one word failed its CRC.
    Crc,
}

impl core::fmt::Display for Failure {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Command(error) => write!(f, "command not acknowledged ({error:?})"),
            Self::Reply(error) => write!(f, "no usable reply ({error:?})"),
            Self::Crc => write!(f, "CRC mismatch"),
        }
    }
}

/// Assembles one big-endian 16-bit word from the wire.
fn word(bytes: &[u8]) -> u16 {
    ((bytes[0] as u16) << 8) | bytes[1] as u16
}

/// The datasheet's `T[C] = -45 + 175 * ticks / 65535`, in thousandths
/// of a degree so the whole example stays integer-only (the FPU isn't
/// enabled by default -- see `fpu_demo.rs`). The intermediate
/// `175000 * ticks` overflows 32 bits, hence the `i64`; the result is
/// bounded by -45000..130000 and always fits an `i32`.
fn temperature_millicelsius(ticks: u16) -> i32 {
    (-45_000 + (175_000i64 * ticks as i64) / 65_535) as i32
}

/// The datasheet's `RH[%] = -6 + 125 * ticks / 65535`, in thousandths
/// of a percent. That formula deliberately runs past both ends of the
/// physical range, so the datasheet also calls for clamping the result
/// to 0..100 -- the out-of-range values it produces near saturation are
/// a property of the linearization, not readings to report.
fn humidity_millipercent(ticks: u16) -> i32 {
    let raw = (-6_000 + (125_000i64 * ticks as i64) / 65_535) as i32;
    raw.clamp(0, 100_000)
}

/// Prints a thousandths-scaled value as a signed decimal with three
/// fraction digits.
fn write_milli(uart: &mut Uart, milli: i32) {
    // The sign is written separately rather than falling out of
    // `milli / 1000`: integer division of -500 gives 0, which would
    // print -0.5 C as "0.500".
    let sign = if milli < 0 { "-" } else { "" };
    let magnitude = milli.unsigned_abs();
    let _ = write!(uart, "{sign}{}.{:03}", magnitude / 1000, magnitude % 1000);
}

/// Sensirion's CRC-8: polynomial 0x31 (x^8 + x^5 + x^4 + 1), initial
/// value 0xff, MSB first, no final XOR.
fn crc8(bytes: &[u8]) -> u8 {
    let mut crc = 0xffu8;
    for &byte in bytes {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x31
            } else {
                crc << 1
            };
        }
    }
    crc
}
