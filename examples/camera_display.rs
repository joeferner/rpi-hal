//! Live camera preview: OV5647 → Unicam → demosaic → framebuffer.
//!
//! Captures frames from an OV5647 (Camera v1) over the Unicam CSI-2
//! receiver and draws them to the display's framebuffer in a loop, for a
//! live preview. The captured data is packed RAW10 Bayer; a cheap
//! 2×2-binning demosaic turns each Bayer quad into one RGB pixel, gamma-
//! corrected and scaled to the framebuffer.
//!
//! The payoff example for the camera path: it exercises the whole chain —
//! sensor bring-up ([`rpi_hal::ov5647`]) over BSC0, the Unicam receiver
//! ([`rpi_hal::unicam`], including the `PM_CAM1` analog-PHY power that makes
//! HS reception work), and the mailbox framebuffer.

#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::halt;
use rpi_hal::i2c::I2c;
use rpi_hal::mailbox::{Mailbox, PixelOrder, EXPANDER_CAM_GPIO0, POWER_DOMAIN_UNICAM1};
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
/// Capture buffer size (packed RAW10).
const FRAME_SIZE: usize = STRIDE * HEIGHT as usize;

/// Requested framebuffer resolution (the firmware may allocate a different
/// size; the blit uses the actual `Framebuffer::width`/`height`).
const FB_WIDTH: u32 = 640;
/// Requested framebuffer height.
const FB_HEIGHT: u32 = 480;

/// A cache-line-aligned packed-RAW10 capture buffer.
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

/// Reads the 8-bit Bayer sample at pixel `(x, y)` out of the packed-RAW10
/// buffer: 4 pixels share 5 bytes, and the first 4 of each group are the
/// high 8 bits of consecutive pixels (the 5th byte holds their low 2 bits,
/// which this preview drops).
#[inline]
fn bayer(frame: &[u8], x: u32, y: u32) -> u32 {
    let index = y as usize * STRIDE + (x as usize / 4) * 5 + (x as usize % 4);
    frame[index] as u32
}

/// Demosaics the packed Bayer `frame` into the framebuffer with a
/// 2×2-binning demosaic (each Bayer quad → one RGB pixel), nearest-neighbour
/// scaled to fill the framebuffer. Each channel is passed through `gamma`
/// (linear sensor data → display gamma), which is what turns the otherwise
/// very dark linear frame into a natural-looking image.
fn draw_frame(frame: &[u8], fb: &rpi_hal::mailbox::Framebuffer, gamma: &[u8; 256]) {
    let base = fb.address as *mut u32;
    let pitch_pixels = (fb.pitch_bytes / 4) as usize;
    // Binned image is WIDTH/2 × HEIGHT/2.
    let binned_w = WIDTH / 2;
    let binned_h = HEIGHT / 2;

    for fy in 0..fb.height {
        let cy = (fy * binned_h / fb.height).min(binned_h - 1);
        let by = cy * 2;
        for fx in 0..fb.width {
            let cx = (fx * binned_w / fb.width).min(binned_w - 1);
            let bx = cx * 2;
            // Bayer quad for this binned/flipped VGA mode: (even,even)=G,
            // (odd,even)=B, (odd,odd)=R. (Empirically — assuming BGGR here
            // came out with green and blue swapped on screen.)
            let g = gamma[bayer(frame, bx, by) as usize] as u32;
            let b = gamma[bayer(frame, bx + 1, by) as usize] as u32;
            let r = gamma[bayer(frame, bx + 1, by + 1) as usize] as u32;
            let pixel = (r << 16) | (g << 8) | b;
            let offset = fy as usize * pitch_pixels + fx as usize;
            unsafe { base.add(offset).write_volatile(pixel) };
        }
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);

    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
    let _ = mailbox.set_expander_gpio(EXPANDER_CAM_GPIO0, true);
    let _ = mailbox.set_power_domain(POWER_DOMAIN_UNICAM1, true);
    timer.delay_ms(50);

    let framebuffer = match mailbox.allocate_framebuffer(FB_WIDTH, FB_HEIGHT, 32, PixelOrder::Bgr) {
        Ok(fb) => fb,
        Err(error) => {
            let _ = writeln!(uart, "framebuffer allocation failed: {error:?}");
            halt();
        }
    };
    let _ = writeln!(
        uart,
        "framebuffer {}x{} pitch {} @ {:#010x}",
        framebuffer.width, framebuffer.height, framebuffer.pitch_bytes, framebuffer.address
    );

    let mut i2c = I2c::<BSC0>::init(&peripherals.GPIO, peripherals.BSC0, 0x05dc, &timer);

    // Gamma ~2.0 lookup (linear → display): out = sqrt(in × 255). Brightens
    // midtones without clipping highlights, the main fix for the dark
    // linear frame.
    let mut gamma = [0u8; 256];
    for (i, g) in gamma.iter_mut().enumerate() {
        *g = (i as u32 * 255).isqrt() as u8;
    }

    let mut unicam = Unicam::new();
    let frame = unsafe { &mut *core::ptr::addr_of_mut!(FRAME) };

    // Arm the receiver first, then start the sensor (the D-PHY must be idle
    // when the sensor begins transmitting), matching camera_capture.
    unicam.arm(&mut frame.0, WIDTH, HEIGHT, &timer);
    if !ov5647::start_streaming(&mut i2c, &timer, false) {
        let _ = writeln!(uart, "OV5647 streaming setup failed; aborting");
        halt();
    }
    let _ = writeln!(uart, "streaming; drawing live preview...");

    let mut frames: u32 = 0;
    loop {
        // Continuous capture: wait_frame re-latches each frame without
        // reconfiguring the D-PHY, so there's no per-frame re-arm. The
        // timeout is generous (1 s) to tolerate a long auto-exposure frame
        // in a dim scene rather than mistaking it for a dropped frame.
        let result = unicam.wait_frame(&timer, 1000);
        if !result.timed_out && result.lines_captured >= HEIGHT {
            draw_frame(&frame.0, &framebuffer, &gamma);
            framebuffer.flush();
            frames += 1;
            if frames.is_multiple_of(30) {
                let _ = writeln!(uart, "{frames} frames");
            }
        } else {
            // Only on an actual miss: fully re-arm (D-PHY reset) and restart
            // the sensor stream, in the arm-then-stream order, to resync.
            let _ = writeln!(
                uart,
                "frame missed (lines={}); resyncing",
                result.lines_captured
            );
            unicam.arm(&mut frame.0, WIDTH, HEIGHT, &timer);
            let _ = ov5647::start_streaming(&mut i2c, &timer, false);
        }
    }
}
