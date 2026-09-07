//! Reference driver for the OmniVision OV5647 (Raspberry Pi Camera v1)
//! image sensor, over I2C.
//!
//! Unlike the rest of this crate — which drives the BCM2836/2837 SoC's own
//! peripherals — the OV5647 is a third-party device on an external bus, so
//! this is a sensor driver rather than a SoC HAL module. It's kept here to
//! pair with the SoC-side CSI-2 receiver ([`crate::unicam`]) that captures
//! its output, and is deliberately minimal: just enough to bring the sensor
//! up streaming so the receiver has something to receive.
//!
//! The register sequence is taken from mainline Linux's `ov5647.c` V4L2
//! driver (its `sensor_oe_enable`, `stream_stop`, `ov5647_common_regs`,
//! `ov5647_640x480_10bpp`, and `enable_streams` steps): 640×480 packed
//! RAW10, two CSI-2 data lanes. Registers are 16-bit addressed with 8-bit
//! data. The sensor's control bus on a Pi 3 is BSC0 on GPIO44/45, so this
//! drives an [`I2c<BSC0>`](crate::i2c::I2c).

use embedded_hal::i2c::I2c as _;

use crate::i2c::I2c;
use crate::pac::BSC0;
use crate::timer::Timer;

/// The OV5647's 7-bit I2C address.
pub const ADDR: u8 = 0x36;

/// Sensor output-enable pads (from the driver's power-on step).
const SENSOR_OE_ENABLE: &[(u16, u8)] = &[(0x3000, 0x0f), (0x3001, 0xff), (0x3002, 0xe4)];
/// Coax the CSI-2 lanes to LP-11 idle before configuration.
const STREAM_STOP: &[(u16, u8)] = &[(0x4800, 0x25), (0x4202, 0x0f), (0x300d, 0x01)];
/// Common clocking/analog/ISP baseline, applied after the software reset.
const COMMON_REGS: &[(u16, u8)] = &[
    (0x3034, 0x1a),
    (0x3035, 0x21),
    (0x303c, 0x11),
    (0x3106, 0xf5),
    (0x3827, 0xec),
    (0x370c, 0x03),
    (0x5000, 0x06),
    (0x5003, 0x08),
    (0x5a00, 0x08),
    (0x3000, 0x00),
    (0x3001, 0x00),
    (0x3002, 0x00),
    (0x3016, 0x08),
    (0x3017, 0xe0),
    (0x3018, 0x44),
    (0x301c, 0xf8),
    (0x301d, 0xf0),
    (0x3a18, 0x00),
    (0x3a19, 0xf8),
    (0x3c01, 0x80),
    (0x3b07, 0x0c),
    (0x3630, 0x2e),
    (0x3632, 0xe2),
    (0x3633, 0x23),
    (0x3634, 0x44),
    (0x3636, 0x06),
    (0x3620, 0x64),
    (0x3621, 0xe0),
    (0x3600, 0x37),
    (0x3704, 0xa0),
    (0x3703, 0x5a),
    (0x3715, 0x78),
    (0x3717, 0x01),
    (0x3731, 0x02),
    (0x370b, 0x60),
    (0x3705, 0x1a),
    (0x3f05, 0x02),
    (0x3f06, 0x10),
    (0x3f01, 0x0a),
    (0x3a08, 0x01),
    (0x3a0f, 0x58),
    (0x3a10, 0x50),
    (0x3a1b, 0x58),
    (0x3a1e, 0x50),
    (0x3a11, 0x60),
    (0x3a1f, 0x28),
    (0x4001, 0x02),
    (0x4000, 0x09),
    (0x3503, 0x03),
];
/// 640×480 SBGGR10 mode (2×2 binned + subsampled, full FOV).
const VGA_640X480: &[(u16, u8)] = &[
    (0x3036, 0x46),
    (0x3821, 0x03),
    (0x3820, 0x41),
    (0x3612, 0x59),
    (0x3618, 0x00),
    (0x3814, 0x35),
    (0x3815, 0x35),
    (0x3708, 0x64),
    (0x3709, 0x52),
    (0x3800, 0x00),
    (0x3801, 0x10),
    (0x3802, 0x00),
    (0x3803, 0x00),
    (0x3804, 0x0a),
    (0x3805, 0x2f),
    (0x3806, 0x07),
    (0x3807, 0x9f),
    (0x3808, 0x02),
    (0x3809, 0x80),
    (0x380a, 0x01),
    (0x380b, 0xe0),
    (0x3a09, 0x2e),
    (0x3a0a, 0x00),
    (0x3a0b, 0xfb),
    (0x3a0d, 0x02),
    (0x3a0e, 0x01),
    (0x4004, 0x02),
    (0x4800, 0x34),
    (0x0100, 0x01),
];
/// Bring the CSI-2 lanes out of idle and start frame output.
const STREAM_START: &[(u16, u8)] = &[(0x4800, 0x34), (0x4202, 0x00), (0x300d, 0x00)];

/// Writes one 8-bit `value` to the 16-bit `register`.
pub fn write_reg(
    i2c: &mut I2c<'_, BSC0>,
    register: u16,
    value: u8,
) -> Result<(), crate::i2c::Error> {
    let [hi, lo] = register.to_be_bytes();
    i2c.write(ADDR, &[hi, lo, value])
}

/// Reads the 8-bit value of the 16-bit `register`.
pub fn read_reg(i2c: &mut I2c<'_, BSC0>, register: u16) -> Result<u8, crate::i2c::Error> {
    let mut value = [0u8];
    i2c.write_read(ADDR, &register.to_be_bytes(), &mut value)?;
    Ok(value[0])
}

/// Applies a `(register, value)` table in order, returning `false` on the
/// first write that doesn't ACK.
fn apply(i2c: &mut I2c<'_, BSC0>, table: &[(u16, u8)]) -> bool {
    for &(register, value) in table {
        if write_reg(i2c, register, value).is_err() {
            return false;
        }
    }
    true
}

/// Returns `true` if an OV5647 answers on the bus (chip ID `0x5647`, from
/// registers `0x300a`/`0x300b`).
pub fn detect(i2c: &mut I2c<'_, BSC0>) -> bool {
    matches!(
        (read_reg(i2c, 0x300a), read_reg(i2c, 0x300b)),
        (Ok(0x56), Ok(0x47))
    )
}

/// Runs the full 640×480 packed-RAW10 streaming bring-up, returning `true`
/// if every register write ACKed. Enables auto exposure/gain; with
/// `test_pattern` set, also turns on the sensor's color-bar test pattern
/// (deterministic output, independent of the scene).
///
/// `timer` provides the required settle delay after the software reset.
pub fn start_streaming(i2c: &mut I2c<'_, BSC0>, timer: &Timer, test_pattern: bool) -> bool {
    let reset_ok = apply(i2c, SENSOR_OE_ENABLE)
        && apply(i2c, STREAM_STOP)
        && write_reg(i2c, 0x0100, 0x00).is_ok()
        && write_reg(i2c, 0x0103, 0x01).is_ok();
    if !reset_ok {
        return false;
    }
    // The sensor needs a moment after the software reset before further
    // register access.
    timer.delay_ms(5);

    if !apply(i2c, COMMON_REGS) || !apply(i2c, VGA_640X480) {
        return false;
    }

    // Auto exposure + auto gain: clear the AEC/AGC manual bits the common
    // table set so the sensor exposes itself to the scene.
    if write_reg(i2c, 0x3503, 0x00).is_err() {
        return false;
    }

    // ISP color-bar test pattern (0x503d), for a deterministic image.
    if test_pattern && write_reg(i2c, 0x503d, 0x80).is_err() {
        return false;
    }

    // MIPI virtual channel 0: clear bits [7:6] of MIPI_CTRL14.
    match read_reg(i2c, 0x4814) {
        Ok(ctrl14) => {
            if write_reg(i2c, 0x4814, ctrl14 & 0x3f).is_err() {
                return false;
            }
        }
        Err(_) => return false,
    }

    apply(i2c, STREAM_START)
}
