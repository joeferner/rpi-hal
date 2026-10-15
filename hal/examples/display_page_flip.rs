#![no_std]
#![no_main]

// Tearing, and how pages fix it. Sweeps a bright vertical bar across a
// dark screen at a steady 60 frames a second, alternating every few
// seconds between the two ways of putting it there:
//
// - **direct**: draw into the buffer the VideoCore is scanning out, the
//   way `Mailbox::allocate_framebuffer` hands it over. The redraw races
//   the scanout down the screen and overtakes it, so the display sends
//   the bar's old position above the crossing point and its new one
//   below: the bar appears **broken into two or three offset segments**,
//   with the horizontal joins between them drifting from frame to frame.
// - **paged**: draw into a page that isn't on screen and bring it on
//   with `Mailbox::set_virtual_offset`. Nothing is ever written to the
//   page being scanned out, so the bar stays a single unbroken column
//   however fast it moves.
//
// So: watch the bar, not the screen. One straight edge top to bottom is
// what "no tearing" looks like; a bar that looks chopped into offset
// pieces is a tear. Which mode is running is printed over the UART as it
// switches, so the two can be told apart without guessing.
//
// On a display that shows no difference between the modes, look at the
// page count the allocation reported: a firmware that wouldn't allocate
// the taller buffer leaves both modes drawing the same single page.

use core::fmt::Write;
use rpi_hal::halt;
use rpi_hal::mailbox::{Framebuffer, Mailbox, PixelOrder};
use rpi_hal::timer::Timer;
use rpi_hal::{pac, uart::Uart};

/// Requested display resolution. 800x480 matches the fixed panel
/// resolution of the MIPI DSI touchscreens this crate's been tried
/// against, the same as `display_test_pattern.rs` -- a DSI panel has one
/// physical resolution rather than a negotiated mode, so asking for
/// anything else leaves the firmware scaling. Change this if testing
/// against a different (HDMI or DSI) display; always check
/// [`rpi_hal::mailbox::Framebuffer::width`]/`height` rather than
/// assuming the request was allocated exactly as asked.
const WIDTH: u32 = 800;
const HEIGHT: u32 = 480;
/// 32 bits per pixel (XRGB8888) -- the depth every bare-metal Pi
/// framebuffer example uses, and simplest to index into.
const DEPTH_BITS: u32 = 32;

/// Full-screen pages to ask for. Three rather than two because a flip
/// doesn't take effect until the display's next vertical blank: with
/// two, the page just retired is still on screen while the next frame
/// is being drawn into it, which is the tear this is meant to avoid.
const PAGES: u32 = 3;

/// One frame, in microseconds -- about 60 a second, near enough to a
/// typical display's own rate that the two drift slowly past each other
/// rather than beating.
///
/// The animation is paced rather than run flat out for the same reason
/// anything real is: an unpaced redraw loop finishes a frame every few
/// milliseconds, and a bar crossing the screen in a fraction of a second
/// reads as a flickering screen instead of as motion. Nothing about the
/// tearing needs the speed — it comes from a redraw straddling a
/// refresh, which happens just as much at 60 fps.
const FRAME_PERIOD_US: u64 = 16_667;

/// How long each mode stays up before switching, in microseconds — long
/// enough to watch several passes of the bar before it changes.
const MODE_PERIOD_US: u64 = 8_000_000;

/// Width of the moving bar, in pixels. Wide enough to see the offset
/// between two segments of a torn one.
const BAR_WIDTH: u32 = 96;

/// Pixels the bar moves per frame: a pass across the screen every
/// ~40 frames, or two thirds of a second.
const STEP: u32 = 20;

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
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // `Bgr`, not `Rgb`: on this little-endian core, writing a pixel as
    // one `0x00RRGGBB` word lands in memory as bytes `[BB, GG, RR, pad]`
    // -- BGR byte order -- so that's what to tell the firmware to
    // expect.
    let framebuffer =
        match mailbox.allocate_framebuffer_paged(WIDTH, HEIGHT, PAGES, DEPTH_BITS, PixelOrder::Bgr)
        {
            Ok(framebuffer) => framebuffer,
            Err(e) => {
                let _ = writeln!(uart, "framebuffer allocation failed: {e:?}");
                halt();
            }
        };

    let _ = writeln!(
        uart,
        "framebuffer: {}x{} @ {}bpp, address 0x{:08x}, pitch {} bytes, {} pages",
        framebuffer.width,
        framebuffer.height,
        framebuffer.depth_bits,
        framebuffer.address,
        framebuffer.pitch_bytes,
        framebuffer.pages(),
    );
    if framebuffer.pages() < 2 {
        let _ = writeln!(
            uart,
            "only one page -- the firmware wouldn't allocate a taller buffer, \
             so both modes below will tear"
        );
    }
    let _ = writeln!(
        uart,
        "watch the bar: unbroken means no tearing, chopped into offset \
         segments means torn"
    );

    let mut paged = false;
    let mut page = 0;
    let mut bar_at = 0;
    let mut mode_started_at = timer.now_micros();
    let mut next_frame_at = timer.now_micros() + FRAME_PERIOD_US;
    let _ = writeln!(uart, "mode: direct -- expect a torn bar");

    loop {
        if timer.now_micros() - mode_started_at >= MODE_PERIOD_US {
            paged = !paged;
            mode_started_at = timer.now_micros();
            let _ = writeln!(
                uart,
                "mode: {}",
                if paged {
                    "paged -- expect an unbroken bar"
                } else {
                    "direct -- expect a torn bar"
                }
            );
            // Back to the page the display is showing, so "direct" is
            // genuinely drawing into the scanned-out buffer.
            if !paged {
                page = 0;
                let _ = mailbox.set_virtual_offset(0, 0);
            }
        }

        draw_bar(&framebuffer, page, bar_at);
        framebuffer.flush_page(page);

        if paged && framebuffer.pages() > 1 {
            // On screen at the display's next vertical blank, whole.
            let _ = mailbox.set_virtual_offset(0, page * framebuffer.height);
            page = (page + 1) % framebuffer.pages();
        }

        bar_at = (bar_at + STEP) % framebuffer.width;

        // Pace to `FRAME_PERIOD_US` from the last deadline rather than
        // from now, so the sweep keeps a steady rate instead of drifting
        // by however long the redraw took.
        let now = timer.now_micros();
        if now < next_frame_at {
            timer.delay_us((next_frame_at - now) as u32);
            next_frame_at += FRAME_PERIOD_US;
        } else {
            // A frame that overran its slot: start the next one from
            // here rather than trying to catch up on a debt that only
            // grows.
            next_frame_at = now + FRAME_PERIOD_US;
        }
    }
}

/// Fills page `page` with a dark background and one bright vertical bar
/// [`BAR_WIDTH`] wide, its left edge at column `bar_at` and wrapping
/// round the right-hand side.
///
/// The whole page is rewritten every frame, not just the columns that
/// changed: a full-screen redraw is what races the scanout, and a
/// two-column update would be over before the display noticed. This is
/// the shape of the work a real frontend does — a game, an emulator, a
/// GUI compositing a window — which is the thing worth showing.
fn draw_bar(framebuffer: &Framebuffer, page: u32, bar_at: u32) {
    /// The background. Not black, so the bar's edges are the only hard
    /// transition on screen and a tear has nothing else to hide behind.
    const BACKGROUND: u32 = 0x0010_1830;
    /// The bar.
    const BAR: u32 = 0x00FF_FFFF;

    let base = framebuffer.address as *mut u32;
    let pitch_pixels = framebuffer.pitch_bytes / 4;
    let origin = framebuffer.page_offset_bytes(page) / 4;

    for y in 0..framebuffer.height {
        for x in 0..framebuffer.width {
            // Distance right of the bar's left edge, wrapping — so the
            // bar reappears on the left as it leaves the right.
            let from_bar = (x + framebuffer.width - bar_at) % framebuffer.width;
            let color = if from_bar < BAR_WIDTH {
                BAR
            } else {
                BACKGROUND
            };
            let offset = origin + y * pitch_pixels + x;
            // Safety: `page` is within the allocation (it comes from
            // `pages()`), `origin` is that page's start, and the rest
            // indexes inside a page the firmware allocated for exactly
            // this width/height/pitch.
            unsafe { base.add(offset as usize).write_volatile(color) };
        }
    }
}
