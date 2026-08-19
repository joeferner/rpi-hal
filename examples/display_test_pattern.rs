#![no_std]
#![no_main]

// Display bring-up smoke test: works out the resolution the firmware is
// driving (see `display_resolution`, which has to account for the
// overscan border to get it right), allocates a framebuffer that size
// over the mailbox property interface, and draws a static test pattern
// (colored vertical bars) into it, filling the screen edge to edge.
// Neither HDMI nor the MIPI DSI touchscreen have any
// ARM-side PHY this crate could program directly -- the VideoCore
// firmware owns both, and a mailbox framebuffer request is the only
// lever this driver has. Which physical output the buffer lands on is
// decided by firmware/`config.txt`, not by anything here -- the same
// code should show the pattern on whichever display is connected.

use core::fmt::Write;
use rpi_hal::halt;
use rpi_hal::mailbox::{Mailbox, Overscan, PixelOrder};
use rpi_hal::{pac, uart::Uart};

/// Resolution to fall back on when the firmware won't say what it is
/// driving (see [`rpi_hal::mailbox::Mailbox::display_size`]). 800x480
/// matches the fixed panel resolution of the MIPI DSI touchscreens this
/// crate's been tried against (the official Raspberry Pi 7"
/// touchscreen and its many electrically-identical clones), which is
/// the display most likely to be attached to a board running these
/// examples. Whatever resolution is used, always check
/// [`rpi_hal::mailbox::Framebuffer::width`]/`height` rather than
/// assuming the request was allocated exactly as asked.
const FALLBACK_WIDTH: u32 = 800;
const FALLBACK_HEIGHT: u32 = 480;
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

    let (width, height) = display_resolution(&mut mailbox, &mut uart);

    // `Bgr`, not `Rgb`: on this little-endian core, writing pixels as
    // one `0x00RRGGBB` word (see `draw_color_bars`'s `COLORS`) lands in
    // memory as bytes `[BB, GG, RR, pad]` -- BGR byte order -- so that's
    // what to tell the firmware to expect. Requesting `Rgb` here made
    // the firmware read byte 0 as red and byte 2 as blue when the
    // memory layout actually has them the other way around, swapping
    // the two channels on screen.
    let framebuffer = match mailbox.allocate_framebuffer(width, height, DEPTH_BITS, PixelOrder::Bgr)
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

/// Works out the resolution to allocate at, printing each step it took
/// to get there.
///
/// Two firmware behaviors make this more than one query. The firmware
/// keeps a blank overscan border by default -- 48 pixels on every edge,
/// for televisions that crop their input -- and "Get Physical
/// Width/Height" reports the image *inside* that border, so a 1920x1080
/// HDMI display answers 1824x984. Clearing the border does not resize
/// the framebuffer the firmware already made, so re-querying after the
/// clear still answers 1824x984; the border has to be added back
/// arithmetically to recover the mode. Allocating at that recovered
/// size is what covers the whole screen, since the allocation request
/// sets the physical size rather than being limited by the reported one.
///
/// A firmware that won't give up the border (`config.txt` pinning the
/// values) means the image genuinely has to stay inside it, so in that
/// case the reported size is already the right one and gets used as-is.
fn display_resolution(mailbox: &mut Mailbox, uart: &mut Uart) -> (u32, u32) {
    // Read the border *before* clearing it -- once it's zero, the width
    // it was hiding can't be recovered from the firmware any more.
    let border = match mailbox.overscan() {
        Ok(border) => {
            let _ = writeln!(
                uart,
                "overscan: top {} bottom {} left {} right {}",
                border.top, border.bottom, border.left, border.right
            );
            border
        }
        Err(e) => {
            let _ = writeln!(uart, "overscan: not reported ({e:?}), assuming none");
            Overscan {
                top: 0,
                bottom: 0,
                left: 0,
                right: 0,
            }
        }
    };

    let inner = match mailbox.display_size() {
        Ok(size) if size.width > 0 && size.height > 0 => {
            let _ = writeln!(
                uart,
                "display: firmware reports {}x{} inside the border",
                size.width, size.height
            );
            size
        }
        other => {
            // Zero width or height means firmware has no mode configured
            // (nothing plugged in), which is no more usable than no
            // answer at all.
            let _ = writeln!(
                uart,
                "display: no usable size from firmware ({other:?}), \
                 falling back to {FALLBACK_WIDTH}x{FALLBACK_HEIGHT}"
            );
            return (FALLBACK_WIDTH, FALLBACK_HEIGHT);
        }
    };

    if border.is_zero() {
        return (inner.width, inner.height);
    }

    let cleared = Overscan {
        top: 0,
        bottom: 0,
        left: 0,
        right: 0,
    };
    // Read the border back rather than trusting what `set_overscan`
    // echoed: the echo is this one call's answer, a fresh query is
    // independent evidence that the change actually took.
    if mailbox.set_overscan(cleared).is_err() || !mailbox.overscan().is_ok_and(|o| o.is_zero()) {
        let _ = writeln!(
            uart,
            "overscan: border stayed, drawing inside it at {}x{}",
            inner.width, inner.height
        );
        return (inner.width, inner.height);
    }

    let full = (
        inner.width + border.left + border.right,
        inner.height + border.top + border.bottom,
    );
    let _ = writeln!(
        uart,
        "overscan: cleared, full mode is {}x{}",
        full.0, full.1
    );
    full
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
