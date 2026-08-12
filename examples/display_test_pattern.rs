#![no_std]
#![no_main]

// Display bring-up smoke test: allocates a framebuffer over the mailbox
// property interface and draws a static test pattern (colored vertical
// bars) into it. Neither HDMI nor the MIPI DSI touchscreen have any
// ARM-side PHY this crate could program directly -- the VideoCore
// firmware owns both, and a mailbox framebuffer request is the only
// lever this driver has. Which physical output the buffer lands on is
// decided by firmware/`config.txt`, not by anything here -- the same
// code should show the pattern on whichever display is connected.

use core::fmt::Write;
use rpi_hal::halt;
use rpi_hal::mailbox::{Mailbox, PixelOrder};
use rpi_hal::{pac, uart::Uart};

/// Requested display resolution. 800x480 matches the fixed panel
/// resolution of the MIPI DSI touchscreens this crate's been tried
/// against (the official Raspberry Pi 7" touchscreen and its many
/// electrically-identical clones) -- unlike HDMI, a DSI panel has one
/// physical resolution, not a negotiated mode, so this must match it
/// rather than request something the firmware would need to scale.
/// Change this if testing against a different (HDMI or DSI) display;
/// always check [`rpi_hal::mailbox::Framebuffer::width`]/`height`
/// rather than assuming the request was allocated exactly as asked.
const WIDTH: u32 = 800;
const HEIGHT: u32 = 480;
/// 32 bits per pixel (XRGB8888) -- the depth every bare-metal Pi
/// framebuffer example uses, and simplest to index into.
const DEPTH_BITS: u32 = 32;

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

    // `Bgr`, not `Rgb`: on this little-endian core, writing pixels as
    // one `0x00RRGGBB` word (see `draw_color_bars`'s `COLORS`) lands in
    // memory as bytes `[BB, GG, RR, pad]` -- BGR byte order -- so that's
    // what to tell the firmware to expect. Requesting `Rgb` here made
    // the firmware read byte 0 as red and byte 2 as blue when the
    // memory layout actually has them the other way around, swapping
    // the two channels on screen.
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
        "framebuffer: {}x{} @ {}bpp, address 0x{:08x}, pitch {} bytes, size {} bytes",
        framebuffer.width,
        framebuffer.height,
        framebuffer.depth_bits,
        framebuffer.address,
        framebuffer.pitch_bytes,
        framebuffer.size_bytes
    );

    draw_color_bars(&framebuffer);
    framebuffer.flush();

    let _ = writeln!(uart, "pattern drawn -- check the display");
    halt();
}

/// Fills the framebuffer with eight equal-width vertical bars in the
/// classic SMPTE color-bar order, assuming 32-bit XRGB8888 pixels (this
/// example only ever requests [`DEPTH_BITS`] = 32).
fn draw_color_bars(framebuffer: &rpi_hal::mailbox::Framebuffer) {
    const COLORS: [u32; 8] = [
        0x00FF_FFFF, // white
        0x00FF_FF00, // yellow
        0x0000_FFFF, // cyan
        0x0000_FF00, // green
        0x00FF_00FF, // magenta
        0x00FF_0000, // red
        0x0000_00FF, // blue
        0x0000_0000, // black
    ];

    let base = framebuffer.address as *mut u32;
    let pitch_pixels = framebuffer.pitch_bytes / 4;
    let bar_width = framebuffer.width / COLORS.len() as u32;

    for y in 0..framebuffer.height {
        for x in 0..framebuffer.width {
            let bar = (x / bar_width).min(COLORS.len() as u32 - 1) as usize;
            let offset = y * pitch_pixels + x;
            // Safety: `offset` is within the buffer the firmware
            // allocated for exactly this width/height/pitch, and this
            // is the only code writing to it.
            unsafe { base.add(offset as usize).write_volatile(COLORS[bar]) };
        }
    }
}
