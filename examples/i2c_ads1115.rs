//! Reads all four single-ended channels of a TI ADS1115 16-bit ADC
//! (the ADS1115IDGS, TSSOP-10 — same die and register map as every
//! other ADS1115 package) over I2C1 (GPIO2 SDA1/GPIO3 SCL1), printing
//! one sweep a second.
//!
//! Wiring: ADS1115 SDA -> GPIO2, SCL -> GPIO3, VDD -> 3V3, GND -> GND,
//! ADDR -> GND (see [`ADS1115_ADDR`]), and whatever is being measured
//! on AIN0..AIN3. No external pull-ups are needed — GPIO2/3 carry the
//! Pi's own fixed 1.8k pull-ups to 3V3. ALERT/RDY is left unconnected;
//! this example polls instead (see [`read_channel`]).
//!
//! Every exchange here is a register access: a write of the 1-byte
//! pointer register (optionally followed by the 2-byte value being
//! written), then a separate read of the 16-bit register it selects.
//! That suits this driver's transaction model, where each
//! `embedded_hal::i2c::Operation` is its own complete START...STOP
//! rather than a repeated start (see [`rpi_hal::i2c::I2c`]) — the
//! ADS1115 latches the pointer and keeps it across the STOP, so the
//! read that follows lands on the intended register.
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

/// 7-bit address with `ADDR` tied to GND. The pin picks one of four:
/// GND `0x48`, VDD `0x49`, SDA `0x4a`, SCL `0x4b` — it is not a
/// strap-on-reset, it's decoded continuously, so a floating `ADDR` pin
/// makes the part answer erratically or not at all.
const ADS1115_ADDR: u8 = 0x48;

/// Pointer value selecting the conversion register (read-only, holds
/// the last completed result as a signed 16-bit big-endian value).
const REG_CONVERSION: u8 = 0x00;

/// Pointer value selecting the config register.
const REG_CONFIG: u8 = 0x01;

/// Config bit 15. Written as 1 it starts a single conversion; read
/// back it means the *opposite* of busy — 1 is "no conversion in
/// progress", 0 is "still converting". [`read_channel`] polls for it
/// being set.
const CONFIG_OS_SINGLE: u16 = 1 << 15;

/// Config bits 11:9 = `001`, the ±4.096V full-scale range. Wider than
/// a 3V3 supply on purpose: the absolute input limit is VDD+0.3V, so
/// the top of the code range simply isn't reachable on a 3V3 board —
/// what this buys is that a rail-to-rail signal can't clip, at the
/// cost of using ~80% of the range. Narrow it (`010` = ±2.048V and so
/// on) for small signals, and change [`MICROVOLTS_PER_LSB`] with it.
const CONFIG_PGA_4V096: u16 = 0b001 << 9;

/// Config bit 8: single-shot mode. The part powers down between
/// conversions, which is why each reading has to be started explicitly
/// and waited for.
const CONFIG_MODE_SINGLE: u16 = 1 << 8;

/// Config bits 7:5 = `100`, 128 samples/second — the reset default.
/// [`CONVERSION_TIMEOUT_US`] is sized against this.
const CONFIG_DR_128SPS: u16 = 0b100 << 5;

/// Config bits 4:0: comparator disabled (`COMP_QUE = 11`), which
/// leaves ALERT/RDY high-impedance. The comparator is the only reason
/// to wire that pin, and this example polls instead.
const CONFIG_COMP_DISABLED: u16 = 0b11;

/// LSB size for [`CONFIG_PGA_4V096`]: the full-scale range spans
/// ±2^15 codes, so 4.096V / 32768 = 125uV exactly. Every PGA setting
/// has its own value — halving the range halves this.
const MICROVOLTS_PER_LSB: i32 = 125;

/// The config register's power-on value, per the datasheet. Read back
/// at startup as a communication check: the ADS1115 has no ID
/// register, so a known-constant register is the closest thing to one.
/// A part that has already been configured (a warm reboot of the Pi
/// leaves it powered) will legitimately read back something else.
const CONFIG_RESET_VALUE: u16 = 0x8583;

/// How long a single conversion is allowed to take before
/// [`read_channel`] gives up. At [`CONFIG_DR_128SPS`] one takes ~7.8ms,
/// plus the ~25us the part needs to wake from power-down; 50ms is a
/// wide margin that still fails fast enough to keep the sweep running.
const CONVERSION_TIMEOUT_US: u64 = 50_000;

/// The four single-ended MUX settings (config bits 14:12), AIN0..AIN3
/// each measured against GND. The other four settings are the
/// differential pairs (`000` = AIN0-AIN1, and so on), which this
/// example doesn't use — note that a differential reading is genuinely
/// signed, while a single-ended one below 0V isn't measurable at all
/// (the input can't go under GND-0.3V).
const SINGLE_ENDED_MUX: [u16; 4] = [0b100 << 12, 0b101 << 12, 0b110 << 12, 0b111 << 12];

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    // 0x05dc (1500) is the reset-default divider: 100kHz standard mode
    // at BCM2835's typical 150MHz core clock. The ADS1115 also does
    // 400kHz fast mode and 3.4MHz high-speed, but at 128 samples/second
    // the bus is nowhere near the bottleneck.
    let mut i2c = I2c::<pac::BSC1>::init(&peripherals.GPIO, peripherals.BSC1, 0x05dc, &timer);

    match read_register(&mut i2c, REG_CONFIG) {
        Ok(config) => {
            let _ = writeln!(uart, "ads1115: config 0x{config:04x}");
            if config != CONFIG_RESET_VALUE {
                // Not fatal, and not necessarily wrong -- see
                // `CONFIG_RESET_VALUE`.
                let _ = writeln!(
                    uart,
                    "ads1115: (not the 0x{CONFIG_RESET_VALUE:04x} reset default -- already configured?)"
                );
            }
        }
        Err(error) => {
            let _ = writeln!(
                uart,
                "ads1115: no response at 0x{ADS1115_ADDR:02x}: {error:?}"
            );
            let _ = writeln!(
                uart,
                "ads1115: check SDA->GPIO2, SCL->GPIO3, VDD=3V3, ADDR tied (not floating)"
            );
            halt();
        }
    }

    loop {
        for (channel, mux) in SINGLE_ENDED_MUX.iter().enumerate() {
            let _ = write!(uart, "ads1115: AIN{channel} = ");
            match read_channel(&mut i2c, &timer, *mux) {
                // A single-ended input is measured against GND, so a
                // negative code means noise around zero (or a signal
                // being pulled below GND, which the part isn't rated
                // for) rather than a real negative voltage.
                Ok(code) => {
                    write_millivolts(&mut uart, code as i32 * MICROVOLTS_PER_LSB);
                    let _ = writeln!(uart, " mV (raw {code})");
                }
                // Reported rather than fatal: one channel failing says
                // nothing about the next, and the point of the loop is
                // to keep sweeping.
                Err(error) => {
                    let _ = writeln!(uart, "failed: {error:?}");
                }
            }
        }

        let _ = writeln!(uart);
        timer.delay_ms(1000);
    }
}

/// Starts a single conversion on the channel `mux` selects, waits for
/// it, and returns the raw signed code.
///
/// The wait is a poll of the config register's `OS` bit rather than a
/// fixed delay. ALERT/RDY can signal the same thing on a wire, but that
/// needs the comparator configured and a pin routed to the Pi; polling
/// costs a couple of register reads and works with three wires. Note
/// that the conversion register holds the *previous* result until this
/// one lands, so returning early on a timeout would hand back a stale
/// reading dressed up as a fresh one -- hence the error.
fn read_channel(i2c: &mut I2c<'_, pac::BSC1>, timer: &Timer, mux: u16) -> Result<i16, Failure> {
    let config = CONFIG_OS_SINGLE
        | mux
        | CONFIG_PGA_4V096
        | CONFIG_MODE_SINGLE
        | CONFIG_DR_128SPS
        | CONFIG_COMP_DISABLED;
    write_register(i2c, REG_CONFIG, config)?;

    let deadline = timer.now_micros() + CONVERSION_TIMEOUT_US;
    while read_register(i2c, REG_CONFIG)? & CONFIG_OS_SINGLE == 0 {
        if timer.now_micros() > deadline {
            return Err(Failure::ConversionTimeout);
        }
    }

    Ok(read_register(i2c, REG_CONVERSION)? as i16)
}

/// Points the ADS1115 at `register` and reads its 16-bit big-endian
/// value.
fn read_register(i2c: &mut I2c<'_, pac::BSC1>, register: u8) -> Result<u16, Failure> {
    i2c.write(ADS1115_ADDR, &[register])
        .map_err(Failure::Pointer)?;

    let mut value = [0u8; 2];
    i2c.read(ADS1115_ADDR, &mut value).map_err(Failure::Value)?;
    Ok(u16::from_be_bytes(value))
}

/// Writes a 16-bit big-endian `value` to `register` — pointer byte and
/// value in one transaction, which is how the datasheet defines a
/// register write (unlike a read, this can't be split).
fn write_register(i2c: &mut I2c<'_, pac::BSC1>, register: u8, value: u16) -> Result<(), Failure> {
    let [hi, lo] = value.to_be_bytes();
    i2c.write(ADS1115_ADDR, &[register, hi, lo])
        .map_err(Failure::Pointer)
}

/// Why a register access or conversion didn't produce a usable result.
/// The bus errors are carried through rather than flattened into a
/// message because they say different things about the hardware:
/// `NoAcknowledge` is nothing listening at this address (a floating
/// `ADDR` pin, most often), `Incomplete { received, .. }` is a part
/// that started answering and stopped, and `Timeout` is a bus nobody
/// is driving at all.
enum Failure {
    /// The pointer byte (or a register write) wasn't acknowledged.
    Pointer(rpi_hal::i2c::Error),
    /// The register's two bytes didn't arrive, or arrived short.
    Value(rpi_hal::i2c::Error),
    /// The conversion never reported itself finished.
    ConversionTimeout,
}

impl core::fmt::Debug for Failure {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Pointer(error) => write!(f, "register select not acknowledged ({error:?})"),
            Self::Value(error) => write!(f, "no usable register value ({error:?})"),
            Self::ConversionTimeout => write!(f, "conversion did not complete"),
        }
    }
}

/// Prints a microvolt value as millivolts with three fraction digits.
fn write_millivolts(uart: &mut Uart, microvolts: i32) {
    // The sign is written separately rather than falling out of
    // `microvolts / 1000`: integer division of -500 gives 0, which
    // would print -0.5mV as "0.500".
    let sign = if microvolts < 0 { "-" } else { "" };
    let magnitude = microvolts.unsigned_abs();
    let _ = write!(uart, "{sign}{}.{:03}", magnitude / 1000, magnitude % 1000);
}
