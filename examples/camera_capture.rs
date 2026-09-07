//! Headless single-frame camera capture self-test: OV5647 → Unicam → RAM.
//!
//! Powers and streams an OV5647 (Camera v1) at 640×480 packed RAW10 with
//! its color-bar test pattern on, captures one frame via the Unicam CSI-2
//! receiver ([`rpi_hal::unicam`]) into RAM, and reports over UART whether a
//! full frame landed (lines/status/errors, and how many of line 0's bytes
//! are non-zero). No display needed — the test pattern gives deterministic
//! data, so this doubles as a self-test of the capture path. For a live
//! image on the framebuffer, see `camera_display.rs`.
//!
//! Sensor bring-up uses the crate's [`rpi_hal::ov5647`] driver.

#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::halt;
use rpi_hal::i2c::I2c;
use rpi_hal::mailbox::{Mailbox, EXPANDER_CAM_GPIO0, POWER_DOMAIN_UNICAM1};
use rpi_hal::ov5647;
use rpi_hal::pac::{self, BSC0};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::unicam::Unicam;

/// Capture width in pixels.
const WIDTH: u32 = 640;
/// Capture height in pixels.
const HEIGHT: u32 = 480;
/// Packed-RAW10 line stride: 640 × 10 / 8 = 800 bytes.
const STRIDE: usize = WIDTH as usize * 10 / 8;
/// Full frame size in bytes (packed RAW10).
const FRAME_SIZE: usize = STRIDE * HEIGHT as usize;

/// A frame buffer aligned to a cache line so the receiver's cache
/// maintenance never spills onto neighbouring data.
#[repr(C, align(64))]
struct Frame([u8; FRAME_SIZE]);

/// The capture destination.
static mut FRAME: Frame = Frame([0; FRAME_SIZE]);

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

    // Power the camera and its Unicam analog power domain, settle, then
    // talk to the sensor.
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    let _ = mailbox.set_expander_gpio(EXPANDER_CAM_GPIO0, true);
    let _ = mailbox.set_power_domain(POWER_DOMAIN_UNICAM1, true);
    timer.delay_ms(50);

    let mut i2c = I2c::<BSC0>::init(&peripherals.GPIO, peripherals.BSC0, 0x05dc, &timer);

    if !ov5647::detect(&mut i2c) {
        let _ = writeln!(uart, "OV5647 not found; aborting");
        halt();
    }
    let _ = writeln!(uart, "OV5647 present; capturing one frame...");

    // Arm the receiver first (D-PHY idle, waiting), THEN start the sensor
    // streaming, so it locks onto the sensor's first frame.
    let frame = unsafe { &mut *core::ptr::addr_of_mut!(FRAME) };
    let mut unicam = Unicam::new();
    unicam.arm(&mut frame.0, WIDTH, HEIGHT, &timer);
    if !ov5647::start_streaming(&mut i2c, &timer, true) {
        let _ = writeln!(uart, "OV5647 streaming setup failed; aborting");
        halt();
    }

    let result = unicam.wait_frame(&timer, 500);

    let _ = writeln!(
        uart,
        "capture: lines={}/{} status={:#010x} ista={:#010x} timed_out={} error={}",
        result.lines_captured,
        HEIGHT,
        result.status,
        result.image_status,
        result.timed_out,
        result.had_error(),
    );

    // Sample the received data: the first packed bytes, and how many of
    // line 0's bytes are non-zero. With the color-bar test pattern this is a
    // mix of saturated and dark bytes, so all-zero would mean nothing came.
    let nonzero = frame.0[..STRIDE].iter().filter(|&&b| b != 0).count();
    let _ = writeln!(uart, "first bytes: {:02x?}", &frame.0[..16]);
    let _ = writeln!(uart, "non-zero bytes in line 0: {nonzero}/{STRIDE}");

    if result.lines_captured >= HEIGHT && !result.had_error() && nonzero > 0 {
        let _ = writeln!(uart, "CAPTURE OK: full frame received into RAM");
    } else {
        let _ = writeln!(uart, "capture incomplete or empty; see status above");
    }

    halt();
}
