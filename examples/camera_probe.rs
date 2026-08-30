//! CSI camera sensor probe.
//!
//! Powers the camera module and reads the sensor's chip-ID register over
//! its I2C control bus, reporting which sensor (if any) answered. A
//! self-contained bring-up check for the camera path — no CSI-2 capture
//! yet, just "can we identify and talk to the sensor".
//!
//! Wiring facts, specific to a Pi 3:
//! - The sensor's master clock is an oscillator on the *camera module*
//!   board, gated by the module's enable line — the SoC generates no
//!   camera clock. That enable line is the VideoCore GPIO-expander pin
//!   [`EXPANDER_CAM_GPIO0`], reached over the mailbox; driving it high
//!   powers the module's regulators and starts its oscillator, after which
//!   the sensor will respond on I2C (OV5647 needs its clock running ≥1 ms
//!   before it accepts I2C; IMX219 likewise needs INCK before its control
//!   interface answers).
//! - The sensor's I2C bus is BSC0 on GPIO44/45 (see [`I2c`]).

#![no_std]
#![no_main]

use core::fmt::Write;
use embedded_hal::i2c::I2c as _;
use rpi_hal::halt;
use rpi_hal::i2c::I2c;
use rpi_hal::mailbox::{Mailbox, EXPANDER_CAM_GPIO0};
use rpi_hal::pac::{self, BSC0};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;

/// A camera sensor this probe knows how to recognise: its 7-bit I2C
/// address, the 16-bit register address of its chip-ID field, and the
/// two-byte value that field should read back.
struct Sensor {
    /// Human-readable name for the report.
    name: &'static str,
    /// 7-bit I2C address the sensor answers on.
    address: u8,
    /// 16-bit register address of the chip-ID field, big-endian as sent.
    id_register: [u8; 2],
    /// Expected two-byte chip-ID value.
    expected_id: [u8; 2],
}

/// The two official Raspberry Pi camera sensors.
const SENSORS: [Sensor; 2] = [
    Sensor {
        name: "OV5647 (Camera v1)",
        address: 0x36,
        id_register: [0x30, 0x0a],
        expected_id: [0x56, 0x47],
    },
    Sensor {
        name: "IMX219 (Camera v2)",
        address: 0x10,
        id_register: [0x00, 0x00],
        expected_id: [0x02, 0x19],
    },
];

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
    let timer = Timer::new(peripherals.SYSTMR);

    // Ask the firmware to assert the camera module's enable line (which
    // starts its on-board oscillator, the sensor's actual master clock).
    // Best-effort, exactly like sdio.rs does for the radio's WL_ON: not
    // every Pi 3 firmware answers the expander "Set GPIO State" tag, and on
    // those that don't, the firmware asserts the line itself when camera
    // auto-detect is enabled in config.txt. Either way we press on and let
    // the I2C probe below be the real test of whether the sensor is powered.
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    match mailbox.set_expander_gpio(EXPANDER_CAM_GPIO0, true) {
        Ok(()) => {
            let _ = writeln!(uart, "camera enable line asserted via mailbox (pin 133)");
        }
        Err(error) => {
            let _ = writeln!(
                uart,
                "mailbox expander tag not answered ({error:?}); relying on firmware to power the camera"
            );
        }
    }

    // Let the regulators, oscillator, and sensor reset settle before
    // touching I2C — comfortably past the sensors' clock-before-I2C minima.
    timer.delay_ms(50);

    // BSC0 on GPIO44/45 at ~100kHz (0x5dc = 1500 divider at the typical
    // core clock), the same conservative default the header-bus examples
    // use.
    let mut i2c = I2c::<BSC0>::init(&peripherals.GPIO, peripherals.BSC0, 0x05dc, &timer);

    let mut found = false;
    for sensor in &SENSORS {
        let mut id = [0u8; 2];
        match i2c.write_read(sensor.address, &sensor.id_register, &mut id) {
            Ok(()) if id == sensor.expected_id => {
                let _ = writeln!(
                    uart,
                    "detected {} at {:#04x}: chip ID {:02x?}",
                    sensor.name, sensor.address, id
                );
                found = true;
            }
            Ok(()) => {
                // The sensor ACKed but the ID didn't match. Most likely the
                // device is present but its register pointer didn't survive
                // the STOP between the address write and the read (this I2C
                // driver does not do a repeated start) — or it's a different
                // sensor at this address.
                let _ = writeln!(
                    uart,
                    "device ACKed at {:#04x} but chip ID = {:02x?} (expected {:02x?} for {})",
                    sensor.address, id, sensor.expected_id, sensor.name
                );
            }
            Err(error) => {
                let _ = writeln!(
                    uart,
                    "no response at {:#04x} ({}): {error:?}",
                    sensor.address, sensor.name
                );
            }
        }
    }

    if !found {
        let _ = writeln!(uart, "no known camera sensor identified");
    }

    halt();
}
