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
use rpi_hal::mailbox::{DisplayId, Mailbox, Outcome, PixelOrder};
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

/// Which display to draw on, most preferred first.
///
/// Only decides anything on a board with more than one attached, and
/// there it decides a lot: which display the firmware enumerates as
/// number 0 is not stable from boot to boot, so without a preference the
/// bars land on a coin toss. An order rather than a single choice because
/// the same card is carried between boards -- the first one on the list
/// that is actually attached gets the picture.
///
/// The panel first because it is the deliberate one: HDMI is on every Pi
/// whether or not anything is plugged into it, while a DSI panel had to
/// be ribboned on by somebody who meant it. A board with only one of them
/// gets the picture either way.
///
/// Empty is a valid setting and means "whatever the firmware picked".
///
/// A Pi will not report more than one display until `max_framebuffers=2`
/// is set in `config.txt`, whatever is actually attached -- so a board
/// where this appears to do nothing should check there first. The
/// bring-up line below prints the count the firmware gave, which is what
/// distinguishes that from a firmware too old to know the tag.
const PREFERENCE: &[DisplayId] = &[DisplayId::MainLcd, DisplayId::Hdmi0];

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

    // Before `display_resolution`, not after: the mode query and the
    // allocation both act on the selected display, so asking first would
    // size the buffer from whichever screen the firmware started on.
    report_displays(&mut mailbox, &mut uart);

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

/// Points the framebuffer at [`PREFERENCE`] and says what it found.
///
/// The selecting is [`Mailbox::select_display`]'s; what is left here is
/// reporting, which the HAL cannot do -- it has no console. With two
/// displays attached, a picture on the wrong one is otherwise a mystery,
/// and which of these outcomes happened is the whole explanation.
fn report_displays(mailbox: &mut Mailbox, uart: &mut Uart) {
    let selection = mailbox.select_display(PREFERENCE);

    if selection.outcome() == Outcome::Sole {
        // Say which kind of "one display" this is. With two plugged in
        // and this still printing, the count is the whole diagnosis:
        // `None` is a firmware that does not know the tag, and a number
        // is a firmware that does and still says one -- which on a Pi
        // means `max_framebuffers` is at its default of 1 in
        // `config.txt`, and no amount of cabling will change it.
        let _ = match selection.reported_count() {
            Some(count) => writeln!(
                uart,
                "display: firmware reports {count}, using {:?}",
                selection.chosen()
            ),
            None => writeln!(
                uart,
                "display: firmware does not report a display count, using {:?}",
                selection.chosen()
            ),
        };
        return;
    }

    let _ = writeln!(
        uart,
        "display: firmware reports {:?} displays",
        selection.reported_count()
    );
    for (number, id) in selection.attached() {
        let _ = writeln!(uart, "display: {number} is {id:?}");
    }
    let _ = match selection.outcome() {
        Outcome::Preferred => writeln!(uart, "display: drawing on {:?}", selection.chosen()),
        Outcome::Firmwares => writeln!(
            uart,
            "display: no preference set, using whichever the firmware selected ({:?})",
            selection.chosen()
        ),
        Outcome::FellBack => writeln!(
            uart,
            "display: none of {PREFERENCE:?} is attached, falling back to display 0 ({:?})",
            selection.chosen()
        ),
        Outcome::Sole => unreachable!("handled above"),
    };
}

/// Works out the resolution to allocate at, printing each step it took
/// to get there.
///
/// The awkward part is [`Mailbox::display_mode`]'s: the firmware keeps a
/// blank overscan border for televisions that crop their input, "Get
/// Physical Width/Height" reports the image *inside* it, and clearing the
/// border does not resize a framebuffer already made -- so the border has
/// to be read first and added back arithmetically. What is left here is
/// saying what happened, and choosing a fallback when the firmware has no
/// mode at all, which is a policy rather than a fact about the hardware
/// and so is not the HAL's to pick.
fn display_resolution(mailbox: &mut Mailbox, uart: &mut Uart) -> (u32, u32) {
    let mode = match mailbox.display_mode() {
        Ok(mode) => mode,
        Err(rpi_hal::mailbox::Error::NoDisplayMode) => {
            // Nothing plugged in, or an output the firmware could not
            // bring up. Worth drawing at all, so that a board with a
            // display attached later has something on it.
            let _ = writeln!(
                uart,
                "display: firmware has no mode configured, \
                 falling back to {FALLBACK_WIDTH}x{FALLBACK_HEIGHT}"
            );
            return (FALLBACK_WIDTH, FALLBACK_HEIGHT);
        }
        Err(e) => {
            let _ = writeln!(
                uart,
                "display: no size from firmware ({e:?}), \
                 falling back to {FALLBACK_WIDTH}x{FALLBACK_HEIGHT}"
            );
            return (FALLBACK_WIDTH, FALLBACK_HEIGHT);
        }
    };

    if !mode.overscan.is_zero() {
        let _ = writeln!(
            uart,
            "overscan: top {} bottom {} left {} right {}",
            mode.overscan.top, mode.overscan.bottom, mode.overscan.left, mode.overscan.right
        );
    }
    if mode.overscan.is_zero() {
        let _ = writeln!(uart, "display: full mode is {}x{}", mode.width, mode.height);
    } else if mode.overscan_cleared {
        let _ = writeln!(
            uart,
            "overscan: cleared, full mode is {}x{}",
            mode.width, mode.height
        );
    } else {
        let _ = writeln!(
            uart,
            "overscan: border stayed, drawing inside it at {}x{}",
            mode.width, mode.height
        );
    }

    (mode.width, mode.height)
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
