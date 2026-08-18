#![no_std]
#![no_main]

// Plays an H.264 video, decoded in hardware, on the display.
//
// Reads `VIDEO.264` from the SD card's FAT partition, feeds it to the
// VideoCore's `ril.video_decode` component through `rpi_hal::video_decode`
// (VCHIQ transport, MMAL client), and puts each decoded frame on screen.
// Progress and an average frame rate go to UART0.
//
// The decoder hands back planar YUV (I420) and the mailbox framebuffer
// takes RGB, so the one piece between them is a colour conversion, done
// here on the ARM (`blit`, below) -- that pass, not the decode, is what
// sets the frame rate this reaches. The framebuffer is allocated once the
// first frame has revealed the stream's geometry, at the picture's own
// size and in pages, so the VideoCore scales it to whatever the display
// is and no half-drawn frame is ever scanned out.
//
// Preparing the card:
//
//   ffmpeg -i clip.mp4 -c:v libx264 -profile:v high -pix_fmt yuv420p \
//          -an -bsf:v h264_mp4toannexb -f h264 VIDEO.264
//
// and copy `VIDEO.264` to the boot partition. Any raw H.264 Annex B
// elementary stream works; `.mp4` and `.mkv` do not, since nothing here
// demuxes a container.
//
// `config.txt` needs enough memory on the GPU side for the codec to work
// in -- `gpu_mem=128` is a good value. (H.264 decode needs no license key;
// only the MPEG-2 and VC-1 decoders do.)
//
// The buffer sizes below cap the resolution this example can handle. They
// are sized for 720p; a larger clip fails with `BufferTooSmall` naming the
// size it needed, which is what to set `MAX_*` to.

use core::fmt::Write;
use embedded_sdmmc::{Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rpi_hal::halt;
use rpi_hal::mailbox::{Framebuffer, Mailbox, PixelOrder};
use rpi_hal::pac;
use rpi_hal::sd::{Sd, SdCard};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::vchiq::{SharedMemory, Vchiq};
use rpi_hal::video_decode::{Frame, VideoDecoder};

/// The elementary stream to decode, on the FAT partition (an 8.3 name).
const VIDEO_FILE: &str = "VIDEO.264";

/// Bits per pixel for the framebuffer -- 32 (XRGB8888), the depth every
/// framebuffer example here uses and the simplest to index into.
const DEPTH_BITS: u32 = 32;

/// Full-screen pages to allocate. Three rather than two because a flip
/// doesn't take effect until the display's next vertical blank: with two,
/// the page just retired is still on screen while the next frame is being
/// drawn into it.
const PAGES: u32 = 3;

/// Frames between progress lines. One line per frame would spend more
/// time in the UART than in the conversion.
const REPORT_EVERY: u32 = 30;

/// Largest frame this example has room for, in pixels. The decoder pads
/// frames out to whole macroblocks, so allow for that: multiples of 32
/// across and 16 down.
const MAX_WIDTH: usize = 1280;
/// See [`MAX_WIDTH`].
const MAX_HEIGHT: usize = 720;

/// Bytes one 4:2:0 frame at that size occupies: a full-size luma plane and
/// two quarter-size chroma planes.
const FRAME_BYTES: usize = MAX_WIDTH * MAX_HEIGHT * 3 / 2;

/// Bytes per compressed-data buffer. The firmware's own minimum for H.264
/// input is 80KB; this is comfortably above it and big enough that reading
/// the file isn't the bottleneck.
const INPUT_BYTES: usize = 128 * 1024;

/// Compressed-data buffers in flight. Three keeps the decoder fed while
/// the SD card fills the next one.
const INPUT_BUFFERS: usize = 3;

/// Frame buffers. Three lets the decoder work on one while another is
/// being reported and a third waits.
const OUTPUT_BUFFERS: usize = 3;

/// A buffer handed to the decoder. Cache-line aligned because the
/// firmware DMAs into and out of these, and that maintenance works in
/// whole 64-byte lines.
#[repr(C, align(64))]
struct Aligned<const BYTES: usize>([u8; BYTES]);

/// The compressed-data buffers.
static mut INPUT: [Aligned<INPUT_BYTES>; INPUT_BUFFERS] =
    [const { Aligned([0; INPUT_BYTES]) }; INPUT_BUFFERS];

/// The frame buffers.
static mut OUTPUT: [Aligned<FRAME_BYTES>; OUTPUT_BUFFERS] =
    [const { Aligned([0; FRAME_BYTES]) }; OUTPUT_BUFFERS];

/// The region VCHIQ shares with the VideoCore. Must outlive everything:
/// the firmware keeps reading it for as long as the board is up.
static mut VCHIQ_MEMORY: SharedMemory = SharedMemory::new();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// A fixed timestamp for `embedded-sdmmc`, only ever used for the
/// modification time of files being written -- irrelevant here, where the
/// card is read-only.
struct FixedTime;

impl TimeSource for FixedTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 56, // 2026
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    // The decode itself is much faster than the ARM core's default clock
    // makes the surrounding copying, so ask for full speed first.
    if let Ok(max) = mailbox.max_clock_rate_hz(rpi_hal::mailbox::ClockId::Arm) {
        let _ = mailbox.set_clock_rate_hz(rpi_hal::mailbox::ClockId::Arm, max);
    }

    let _ = writeln!(uart, "initializing SD card...");
    let sd = match Sd::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, &timer) {
        Ok(sd) => sd,
        Err(error) => {
            let _ = writeln!(uart, "SD init failed: {error:?}");
            halt();
        }
    };

    let _ = writeln!(uart, "bringing up VCHIQ...");
    // SAFETY: single-threaded bring-up, and this is the only reference
    // taken to the shared region -- it is handed straight to the driver,
    // which owns it from here on.
    let memory = unsafe { &mut *core::ptr::addr_of_mut!(VCHIQ_MEMORY) };
    let mut vchiq = match Vchiq::new(memory, &mut mailbox, &timer) {
        Ok(vchiq) => vchiq,
        Err(error) => {
            let _ = writeln!(uart, "VCHIQ init failed: {error:?}");
            halt();
        }
    };
    if let Err(error) = vchiq.connect(&timer) {
        let _ = writeln!(uart, "VCHIQ connect failed: {error:?}");
        halt();
    }

    let _ = writeln!(uart, "creating the decoder...");
    let mut decoder = match VideoDecoder::new(vchiq, &timer) {
        Ok(decoder) => decoder,
        Err(error) => {
            let _ = writeln!(uart, "decoder create failed: {error:?}");
            halt();
        }
    };

    // SAFETY: each buffer is handed to the decoder exactly once, and
    // nothing here touches the statics again -- ownership moves with the
    // slices.
    unsafe {
        for buffer in &mut *core::ptr::addr_of_mut!(INPUT) {
            let _ = decoder.add_input_buffer(&mut buffer.0);
        }
        for buffer in &mut *core::ptr::addr_of_mut!(OUTPUT) {
            let _ = decoder.add_output_buffer(&mut buffer.0);
        }
    }

    if let Err(error) = decoder.start(&timer) {
        let _ = writeln!(uart, "decoder start failed: {error:?}");
        halt();
    }

    // What the component settled on, which is not necessarily what it was
    // asked for -- and the first thing to check if nothing decodes.
    let _ = writeln!(uart, "input  {:?}", decoder.input_port());
    let _ = writeln!(uart, "output {:?}", decoder.output_port());

    if let Err(error) = decode(&mut decoder, sd, &mut mailbox, &timer, &mut uart) {
        let _ = writeln!(uart, "decode failed: {error:?}");
        // A stalled exchange with the firmware says nothing about itself,
        // so print what crossed the interface before it stopped -- sent
        // against returned is what identifies the half that went quiet.
        let mmal_stats = decoder.mmal().stats();
        let vchiq_stats = decoder.mmal().vchiq().stats();
        let _ = writeln!(uart, "mmal:  {mmal_stats:?}");
        let _ = writeln!(uart, "vchiq: {vchiq_stats:?}");
    }

    halt();
}

/// Errors the decode loop below can end on -- either side of it can fail,
/// so they get one type to return.
enum Error {
    /// The FAT layer or the card under it failed.
    Storage(embedded_sdmmc::Error<rpi_hal::sd::SdCardError>),
    /// The decoder failed.
    Decode(rpi_hal::video_decode::Error),
}

// Written out rather than derived: a derived `Debug` doesn't count as
// reading the wrapped errors, so the compiler warns that the fields are
// never used -- when printing them is the entire point of the type.
impl core::fmt::Debug for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Storage(error) => write!(f, "storage: {error:?}"),
            Error::Decode(error) => write!(f, "decoder: {error:?}"),
        }
    }
}

impl From<embedded_sdmmc::Error<rpi_hal::sd::SdCardError>> for Error {
    fn from(error: embedded_sdmmc::Error<rpi_hal::sd::SdCardError>) -> Self {
        Error::Storage(error)
    }
}

impl From<rpi_hal::video_decode::Error> for Error {
    fn from(error: rpi_hal::video_decode::Error) -> Self {
        Error::Decode(error)
    }
}

/// Streams the file through the decoder and puts every frame on screen.
fn decode(
    decoder: &mut VideoDecoder,
    sd: Sd,
    mailbox: &mut Mailbox,
    timer: &Timer,
    uart: &mut Uart,
) -> Result<(), Error> {
    let volume_mgr = VolumeManager::new(SdCard::new(sd, timer), FixedTime);
    let volume = volume_mgr.open_volume(VolumeIdx(0))?;
    let root = volume.open_root_dir()?;
    let file = root.open_file_in_dir(VIDEO_FILE, Mode::ReadOnly)?;
    let _ = writeln!(uart, "{VIDEO_FILE}: {} bytes", file.length());

    // One chunk of the file at a time. Sized to fit an input buffer, so a
    // read never has to be split across two of them.
    let mut chunk = [0u8; 32 * 1024];
    let mut pending = 0..0;
    let mut frames = 0u32;
    let mut finished = false;
    // Allocated on the first frame, once the stream has said how big its
    // pictures are; `None` until then.
    let mut display: Option<Framebuffer> = None;
    let mut page = 0;
    let start = timer.now_micros();

    while !decoder.end_of_stream() {
        // Top up the decoder with whatever it will take.
        if pending.is_empty() && !file.is_eof() {
            let read = file.read(&mut chunk)?;
            pending = 0..read;
        }
        if !pending.is_empty() {
            let taken = decoder.feed(
                &chunk[pending.clone()],
                0,
                rpi_hal::mmal::TIME_UNKNOWN,
                timer,
            )?;
            pending.start += taken;
        } else if file.is_eof() && !finished {
            // Nothing left to send: tell the decoder so it flushes the
            // frames it is still holding.
            decoder.finish(timer)?;
            finished = true;
        }

        while let Some(frame) = decoder.poll(timer)? {
            frames += 1;
            let format = frame.format;

            // The framebuffer is sized to the picture rather than to the
            // display: the VideoCore scales it to whatever is attached, so
            // this neither letterboxes by hand nor cares what the display
            // is. It can only be done here, since nothing knew the size
            // until this first frame arrived.
            if display.is_none() {
                let _ = writeln!(
                    uart,
                    "{}x{} (buffer {}x{}), {} bytes/frame",
                    format.crop_width,
                    format.crop_height,
                    format.width,
                    format.height,
                    frame.length
                );
                match mailbox.allocate_framebuffer_paged(
                    format.crop_width,
                    format.crop_height,
                    PAGES,
                    DEPTH_BITS,
                    // Pairs with the `0x00RRGGBB` words `blit` writes:
                    // little-endian puts those bytes in memory blue
                    // first, which is the order this names.
                    PixelOrder::Bgr,
                ) {
                    Ok(framebuffer) => {
                        let _ = writeln!(
                            uart,
                            "framebuffer {}x{}, {} page(s)",
                            framebuffer.width,
                            framebuffer.height,
                            framebuffer.pages()
                        );
                        display = Some(framebuffer);
                    }
                    Err(error) => {
                        let _ = writeln!(uart, "framebuffer alloc failed: {error:?}");
                    }
                }
            }

            if let Some(framebuffer) = &display {
                blit(&frame, framebuffer, page);
                framebuffer.flush_page(page);
                let _ = mailbox.set_virtual_offset(0, page * framebuffer.height);
                page = (page + 1) % framebuffer.pages();
            }

            decoder.recycle(frame, timer)?;

            if frames.is_multiple_of(REPORT_EVERY) {
                let elapsed_us = timer.now_micros() - start;
                let _ = writeln!(
                    uart,
                    "{frames} frames, {} frames/s",
                    ((frames as u64) * 1_000_000)
                        .checked_div(elapsed_us)
                        .unwrap_or(0)
                );
            }
        }
    }

    let elapsed_us = timer.now_micros() - start;
    let _ = writeln!(
        uart,
        "\ndecoded {frames} frames in {} ms ({} frames/s)",
        elapsed_us / 1000,
        ((frames as u64) * 1_000_000)
            .checked_div(elapsed_us)
            .unwrap_or(0)
    );
    Ok(())
}

/// Converts one decoded frame into `page` of the framebuffer.
///
/// The decoder's I420 output is three planes: full-resolution luma, then
/// blue- and red-difference chroma at half resolution in each direction,
/// all laid out to the frame's *padded* width. Only the top-left crop
/// region is the picture, which is what the framebuffer was sized to, so
/// the planes are indexed by the padded stride and read only that far.
///
/// The arithmetic is the standard integer form of the BT.601 conversion
/// for studio-swing video (luma 16-235), which is what an H.264 stream
/// carries unless it says otherwise: scale luma about 16, add the two
/// chroma differences about 128, and round. Integer rather than float not
/// because the FPU is missing -- this crate turns it on -- but because
/// this is the loop that decides the frame rate, running once per pixel.
fn blit(frame: &Frame, framebuffer: &Framebuffer, page: u32) {
    let format = frame.format;
    let stride = format.width as usize;
    let chroma_stride = stride / 2;
    let luma = &frame.buffer[..];
    let u_plane = format.u_offset();
    let v_plane = format.v_offset();

    let base = framebuffer.address as *mut u32;
    let pitch_pixels = framebuffer.pitch_bytes / 4;
    let origin = framebuffer.page_offset_bytes(page) / 4;

    let width = framebuffer.width.min(format.crop_width) as usize;
    let height = framebuffer.height.min(format.crop_height) as usize;

    for y in 0..height {
        let luma_row = y * stride;
        let chroma_row = (y / 2) * chroma_stride;
        let out_row = origin as usize + y * pitch_pixels as usize;

        for x in 0..width {
            let luma_value = (luma[luma_row + x] as i32 - 16) * 298;
            let blue_diff = luma[u_plane + chroma_row + x / 2] as i32 - 128;
            let red_diff = luma[v_plane + chroma_row + x / 2] as i32 - 128;

            let red = clamp8((luma_value + 409 * red_diff + 128) >> 8);
            let green = clamp8((luma_value - 100 * blue_diff - 208 * red_diff + 128) >> 8);
            let blue = clamp8((luma_value + 516 * blue_diff + 128) >> 8);

            // SAFETY: `page` came from `pages()`, `origin` is that page's
            // first pixel, and `width`/`height` are clamped to the
            // framebuffer's own, so this stays inside the allocation.
            unsafe {
                base.add(out_row + x)
                    .write_volatile((red << 16) | (green << 8) | blue)
            };
        }
    }
}

/// Saturates a conversion result to one 8-bit channel, in place in the
/// 32-bit pixel word.
fn clamp8(value: i32) -> u32 {
    value.clamp(0, 255) as u32
}
