#![no_std]
#![no_main]

// Draws a filled square at each active touch point on the DSI
// touchscreen -- the framebuffer bring-up from `display_test_pattern`
// and the touch buffer from `rpi_hal::touch` used together: on every
// frame, clear to black and redraw a square under wherever a finger
// currently is. There's no bus to enumerate and no transfer to pace for
// touch -- the controller's current state lives in a small buffer the
// VideoCore firmware keeps refreshed on its own, and
// `rpi_hal::touch::TouchScreen` just re-reads it directly.

use core::fmt::Write;
use rpi_hal::halt;
use rpi_hal::mailbox::{Framebuffer, Mailbox, PixelOrder};
use rpi_hal::pac;
use rpi_hal::timer::Timer;
use rpi_hal::touch::{TouchEvent, TouchScreen};
use rpi_hal::uart::Uart;

/// Requested display resolution -- see `display_test_pattern`'s doc
/// comment on why this must match a DSI panel's fixed native
/// resolution (800x480 for the touchscreens this crate's been tried
/// against) rather than an arbitrary size.
const WIDTH: u32 = 800;
const HEIGHT: u32 = 480;
/// 32 bits per pixel (XRGB8888).
const DEPTH_BITS: u32 = 32;

/// Side length, in pixels, of the square drawn at each touch point.
const MARKER_SIZE: u32 = 40;

/// Background color: black.
const BACKGROUND: u32 = 0x0000_0000;
/// Marker color: white.
const MARKER: u32 = 0x00FF_FFFF;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // Best-effort: re-steal the UART (safe to re-`init` -- it's just
    // idempotent register writes) and print where this panic happened.
    // A silent halt is far harder to tell apart from a hardware hang
    // than a printed panic location, which is exactly the ambiguity
    // that made an out-of-bounds touch id look identical to a bus hang
    // during this driver's bring-up.
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
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // See `display_test_pattern` for why `Bgr`, not `Rgb`.
    let framebuffer = match mailbox.allocate_framebuffer(WIDTH, HEIGHT, DEPTH_BITS, PixelOrder::Bgr)
    {
        Ok(framebuffer) => framebuffer,
        Err(e) => {
            let _ = writeln!(uart, "framebuffer allocation failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(
        uart,
        "framebuffer: {}x{} @ {}bpp",
        framebuffer.width, framebuffer.height, framebuffer.depth_bits
    );

    let mut touchscreen = match TouchScreen::new(&mut mailbox) {
        Ok(touchscreen) => touchscreen,
        Err(e) => {
            let _ = writeln!(uart, "touch buffer lookup failed: {e:?}");
            halt();
        }
    };
    let _ = writeln!(
        uart,
        "touch buffer address: 0x{:08x}",
        touchscreen.address()
    );

    let _ = writeln!(uart, "touch the screen...");
    loop {
        // Pace the loop -- nothing here needs faster than a display frame.
        timer.delay_ms(16);

        let update = touchscreen.poll();
        for event in update.events() {
            match event {
                TouchEvent::Pressed(touch) => {
                    let _ = writeln!(uart, "id {} down at ({}, {})", touch.id, touch.x, touch.y);
                }
                TouchEvent::Released { id } => {
                    let _ = writeln!(uart, "id {id} up");
                }
            }
        }

        fill(&framebuffer, BACKGROUND);
        for touch in update.report().touches() {
            let _ = writeln!(uart, "id {} at ({}, {})", touch.id, touch.x, touch.y);
            fill_square(&framebuffer, touch.x, touch.y, MARKER_SIZE, MARKER);
        }
        framebuffer.flush();
    }
}

/// Fills the whole framebuffer with `color`, assuming 32-bit pixels
/// (this example only ever requests [`DEPTH_BITS`] = 32).
fn fill(framebuffer: &Framebuffer, color: u32) {
    let base = framebuffer.address as *mut u32;
    let pitch_pixels = framebuffer.pitch_bytes / 4;
    for y in 0..framebuffer.height {
        for x in 0..framebuffer.width {
            let offset = y * pitch_pixels + x;
            // Safety: `offset` is within the buffer the firmware
            // allocated for exactly this width/height/pitch.
            unsafe { base.add(offset as usize).write_volatile(color) };
        }
    }
}

/// Fills a `size`x`size` square centered on (`center_x`, `center_y`)
/// with `color`, clipped to the framebuffer's bounds.
fn fill_square(framebuffer: &Framebuffer, center_x: u16, center_y: u16, size: u32, color: u32) {
    let half = size / 2;
    let x0 = (center_x as u32).saturating_sub(half);
    let y0 = (center_y as u32).saturating_sub(half);
    let x1 = (x0 + size).min(framebuffer.width);
    let y1 = (y0 + size).min(framebuffer.height);

    let base = framebuffer.address as *mut u32;
    let pitch_pixels = framebuffer.pitch_bytes / 4;
    for y in y0..y1 {
        for x in x0..x1 {
            let offset = y * pitch_pixels + x;
            // Safety: `offset` is within the buffer the firmware
            // allocated for exactly this width/height/pitch (`x1`/`y1`
            // are clipped to `framebuffer.width`/`height` above).
            unsafe { base.add(offset as usize).write_volatile(color) };
        }
    }
}
