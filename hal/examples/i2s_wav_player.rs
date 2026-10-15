#![no_std]
#![no_main]

//! Plays a WAV file off the SD card out to an I2S DAC — the SD/FAT read
//! path (`rpi_hal::sd` + `embedded-sdmmc`, as in `sd_fat_read.rs`) feeding
//! the digital-audio path (`rpi_hal::pcm`, as in `i2s_dac_tone.rs`).
//!
//! Reads `AUDIO.WAV` from the root of the card's first FAT partition and
//! streams it continuously (looping back to the start at end-of-file) to
//! the DAC via the double-buffered DMA stream: while the engine clocks one
//! buffer out, the CPU reads and converts the next chunk from the card.
//!
//! # Wiring
//!
//! Same I2S DAC wiring as `i2s_dac_tone.rs` (`BCK`→GPIO18, `LCK`→GPIO19,
//! `DIN`→GPIO21, and — crucially on a PCM5102A — `SCK`→GND, `XSMT`→High,
//! `FMT`→Low; see that example's header for the full table and why). The SD
//! card is the on-board slot the `rpi_hal::sd` EMMC driver already uses.
//! The two paths don't contend: SD runs off the EMMC controller (polled),
//! I2S off the PCM peripheral and one DMA channel.
//!
//! # WAV format requirements
//!
//! To keep this a player and not a WAV parser, it accepts only the
//! canonical 44-byte PCM header — `RIFF`/`WAVE`/`fmt `(16)/`data` laid out
//! contiguously with the audio data starting at byte 44 — and requires:
//!
//! - **PCM** (uncompressed, format code 1),
//! - **2 channels** (stereo — the fixed format `rpi_hal::pcm` drives),
//! - **16-bit** samples.
//!
//! The sample *rate* is read from the header and used to set the PCM clock,
//! so any rate works. Files with extra chunks (a `LIST`/`fact` chunk before
//! `data`, common in files that carry metadata) don't match the fixed
//! layout and are rejected with a clear message rather than mis-parsed —
//! produce a conforming file with e.g.
//! `ffmpeg -i in.mp3 -ar 44100 -ac 2 -c:a pcm_s16le -map_metadata -1 -fflags +bitexact AUDIO.WAV`.

use core::fmt::Write;
use embedded_sdmmc::{BlockDevice, File, Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rpi_hal::dma::Dma;
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::pcm::{pcm_sample, Pcm};
use rpi_hal::sd::{Sd, SdCard, SdCardError};
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::{pac, pac::CM_PCM, pac::GPIO};

/// The file played, in the root of the first FAT partition. 8.3 name so it
/// works on a plain FAT volume without long-filename support.
const WAV_NAME: &str = "AUDIO.WAV";
/// Size of the canonical PCM WAV header this player accepts, in bytes; the
/// audio data starts here.
const HEADER_LEN: u32 = 44;

/// Stereo frames per buffer chunk. A chunk is `CHUNK_FRAMES * 2` DMA words;
/// at 1024 that's 8 KiB per buffer (a whole number of cache lines) and
/// ~23 ms of audio at 44.1 kHz — enough that one SD read comfortably keeps
/// ahead of playback.
const CHUNK_FRAMES: usize = 1024;
/// DMA words per chunk buffer (interleaved L/R, one word per sample).
const CHUNK_WORDS: usize = CHUNK_FRAMES * 2;
/// Bytes of WAV data per chunk — two bytes per 16-bit sample.
const CHUNK_BYTES: usize = CHUNK_WORDS * 2;

/// A DMA chunk buffer, cache-line aligned so the stream's per-buffer clean
/// stays on its own lines.
#[repr(C, align(64))]
struct Chunk([u32; CHUNK_WORDS]);

/// The two ping-pong buffers the DMA engine alternates between.
static mut BUF0: Chunk = Chunk([0; CHUNK_WORDS]);
static mut BUF1: Chunk = Chunk([0; CHUNK_WORDS]);
/// CPU-side scratch holding the raw little-endian PCM bytes read from the
/// card before they're widened into a DMA buffer. Never touched by the DMA
/// engine, so it needs no special alignment.
static mut SCRATCH: [u8; CHUNK_BYTES] = [0; CHUNK_BYTES];

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// A fixed timestamp for `embedded-sdmmc` — only consulted when creating or
/// writing files, so irrelevant to this read-only player (see the note in
/// `sd_fat_read.rs`).
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

/// Everything that can go wrong playing the file: a filesystem/card error
/// from the FAT layer, or the WAV not matching the fixed format this player
/// requires.
enum PlayError {
    /// An error from the SD/FAT read path.
    Fat(embedded_sdmmc::Error<SdCardError>),
    /// `AUDIO.WAV` isn't the canonical stereo-16-bit PCM layout required
    /// (see this example's "WAV format requirements").
    BadWav(&'static str),
}

impl From<embedded_sdmmc::Error<SdCardError>> for PlayError {
    fn from(e: embedded_sdmmc::Error<SdCardError>) -> Self {
        PlayError::Fat(e)
    }
}

impl core::fmt::Display for PlayError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            PlayError::Fat(e) => write!(f, "SD/FAT error: {e:?}"),
            PlayError::BadWav(msg) => write!(f, "unsupported WAV ({msg})"),
        }
    }
}

/// Reads exactly `buf.len()` bytes from `file`, looping over the short
/// reads `File::read` can return at block boundaries. Returns the number of
/// bytes actually read and whether end-of-file was reached — a full buffer
/// with more data left returns `(len, false)`; a partial fill at the end
/// returns `(n, true)`.
fn read_fully<D, T, const MD: usize, const MF: usize, const MV: usize>(
    file: &File<'_, D, T, MD, MF, MV>,
    buf: &mut [u8],
) -> Result<(usize, bool), embedded_sdmmc::Error<D::Error>>
where
    D: BlockDevice,
    T: TimeSource,
{
    let mut filled = 0;
    while filled < buf.len() {
        let n = file.read(&mut buf[filled..])?;
        if n == 0 {
            return Ok((filled, true));
        }
        filled += n;
    }
    Ok((filled, file.is_eof()))
}

/// Fills one DMA buffer with the next chunk of audio: reads up to a chunk's
/// worth of little-endian 16-bit PCM into `scratch`, widens each sample
/// into a full DMA word ([`pcm_sample`]), and pads any tail past
/// end-of-file with silence. Returns whether end-of-file was reached.
fn fill_from_file<D, T, const MD: usize, const MF: usize, const MV: usize>(
    file: &File<'_, D, T, MD, MF, MV>,
    dma_buf: &mut [u32],
    scratch: &mut [u8],
) -> Result<bool, embedded_sdmmc::Error<D::Error>>
where
    D: BlockDevice,
    T: TimeSource,
{
    let bytes = &mut scratch[..dma_buf.len() * 2];
    let (filled, eof) = read_fully(file, bytes)?;
    let samples = filled / 2;
    for (i, word) in dma_buf.iter_mut().enumerate() {
        *word = if i < samples {
            pcm_sample(i16::from_le_bytes([bytes[i * 2], bytes[i * 2 + 1]]))
        } else {
            pcm_sample(0)
        };
    }
    Ok(eof)
}

/// Validates the fixed 44-byte canonical PCM WAV header and returns the
/// sample rate. See this example's "WAV format requirements".
fn parse_header(h: &[u8; 44]) -> Result<u32, PlayError> {
    if &h[0..4] != b"RIFF" || &h[8..12] != b"WAVE" || &h[12..16] != b"fmt " {
        return Err(PlayError::BadWav("not a RIFF/WAVE file"));
    }
    if &h[36..40] != b"data" {
        // The 'data' chunk isn't where the canonical layout puts it, so the
        // file has extra chunks (e.g. a metadata LIST) this player doesn't
        // skip -- see the format note.
        return Err(PlayError::BadWav(
            "non-canonical layout (data not at byte 36)",
        ));
    }
    let audio_format = u16::from_le_bytes([h[20], h[21]]);
    let channels = u16::from_le_bytes([h[22], h[23]]);
    let bits = u16::from_le_bytes([h[34], h[35]]);
    if audio_format != 1 {
        return Err(PlayError::BadWav("not uncompressed PCM"));
    }
    if channels != 2 {
        return Err(PlayError::BadWav("not stereo (2 channels required)"));
    }
    if bits != 16 {
        return Err(PlayError::BadWav("not 16-bit samples"));
    }
    Ok(u32::from_le_bytes([h[24], h[25], h[26], h[27]]))
}

/// Opens `AUDIO.WAV`, brings up I2S at the file's sample rate, and streams
/// it to the DAC forever (looping at end-of-file). Split from `kmain` so
/// the FAT/format steps can `?` against one error type.
fn run(
    sd: Sd,
    timer: &Timer,
    uart: &mut Uart,
    gpio: &GPIO,
    cm_pcm: CM_PCM,
) -> Result<(), PlayError> {
    let volume_mgr = VolumeManager::new(SdCard::new(sd, timer), FixedTime);
    let volume = volume_mgr.open_volume(VolumeIdx(0))?;
    let root = volume.open_root_dir()?;
    let file = root.open_file_in_dir(WAV_NAME, Mode::ReadOnly)?;

    let mut header = [0u8; 44];
    let (n, _) = read_fully(&file, &mut header)?;
    if n < header.len() {
        return Err(PlayError::BadWav("file shorter than a WAV header"));
    }
    let sample_rate = parse_header(&header)?;
    let _ = writeln!(
        uart,
        "{WAV_NAME}: {sample_rate} Hz stereo 16-bit, {} bytes of audio",
        file.length().saturating_sub(HEADER_LEN)
    );

    // SAFETY: single-threaded; these statics are used only here and stay
    // borrowed by the DMA stream / this function for the rest of the run.
    let buf0 = unsafe { &mut *core::ptr::addr_of_mut!(BUF0) };
    let buf1 = unsafe { &mut *core::ptr::addr_of_mut!(BUF1) };
    let scratch = unsafe { &mut *core::ptr::addr_of_mut!(SCRATCH) };

    // Prime both ping-pong buffers before the engine starts reading them.
    fill_from_file(&file, &mut buf0.0, scratch)?;
    fill_from_file(&file, &mut buf1.0, scratch)?;

    // Bring up the PCM/I2S peripheral at the file's sample rate.
    let divisor = Pcm::clock_divisor(sample_rate);
    let pcm = Pcm::init(cm_pcm, divisor);
    let i2s = pcm.i2s_out(gpio);

    // Start the ping-pong stream on a full DMA channel (0–6).
    let mut dma = Dma::new();
    let mut channel = dma.channel(5).expect("channel 5 available");
    let mut stream = channel
        .stream_peripheral(
            [&mut buf0.0, &mut buf1.0],
            i2s.dreq(),
            i2s.fifo_bus_address(),
        )
        .expect("start I2S stream");

    let _ = writeln!(uart, "playing (looping at end-of-file)...");

    let mut loops: u32 = 0;
    loop {
        // Refill whichever buffer the engine just finished; blocks until
        // it's free, so this loop is paced to the audio rate. Any read
        // error surfaces after the closure via `err`; end-of-file via `eof`.
        let mut eof = false;
        let mut err = None;
        stream.feed(|buf| match fill_from_file(&file, buf, scratch) {
            Ok(e) => eof = e,
            Err(e) => err = Some(e),
        });
        if let Some(e) = err {
            return Err(e.into());
        }
        if eof {
            // Rewind past the header to loop the track. The chunk that hit
            // EOF was silence-padded, so the seam is a few ms of silence.
            file.seek_from_start(HEADER_LEN)?;
            loops += 1;
            let _ = writeln!(uart, "looped ({loops})");
        }
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let timer = Timer::new(peripherals.SYSTMR);
    let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);

    let _ = writeln!(uart, "initializing SD card...");
    let sd = match Sd::init(&peripherals.GPIO, peripherals.EMMC, &mut mailbox, &timer) {
        Ok(sd) => sd,
        Err(e) => {
            let _ = writeln!(uart, "SD init failed: {e:?}");
            halt();
        }
    };

    // `run` only returns on error — it plays in an infinite loop otherwise.
    if let Err(e) = run(sd, &timer, &mut uart, &peripherals.GPIO, peripherals.CM_PCM) {
        let _ = writeln!(uart, "playback failed: {e}");
    }

    halt();
}
