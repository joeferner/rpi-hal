//! Streamed PWM analog audio — a swept-frequency siren.
//!
//! Unlike `pwm_audio.rs`, which loops one fixed buffer forever, this
//! example generates fresh samples continuously: a sine whose frequency
//! sweeps up and down, so the tone audibly changes over time — something
//! a fixed cyclic buffer can't do. It uses the double-buffered
//! (ping-pong) DMA stream: while the engine plays one buffer the CPU
//! fills the other, gaplessly.
//!
//! Observe on the analog-audio pins (GPIO40 / GPIO45) — a rising/falling
//! siren on headphones through the 3.5 mm jack, or a swept sine on a
//! scope after an RC low-pass. Self-contained: no external hardware.

#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::dma::Dma;
use rpi_hal::halt;
use rpi_hal::pwm::{Channel1Pin, Channel2Pin, Pwm};
use rpi_hal::{pac, uart::Uart};

/// Nominal sample rate (approximate — see `rpi_hal::pwm`'s clock docs).
const SAMPLE_RATE: u32 = 44_100;
/// Sample range / bit depth: each sample is a duty value in `0..=RANGE`.
const RANGE: u16 = 1024;
/// Stereo frames per buffer chunk. A chunk is `CHUNK_FRAMES * 2` words
/// (one channel-1 + one channel-2 word per frame); at 512 that's 4096
/// bytes, a whole number of cache lines. ~11.6 ms of audio per chunk at
/// 44.1 kHz.
const CHUNK_FRAMES: usize = 512;
/// Words per chunk buffer (interleaved L/R).
const CHUNK_WORDS: usize = CHUNK_FRAMES * 2;

/// Siren sweep bounds and per-chunk step, in Hz.
const FREQ_MIN: u32 = 220;
const FREQ_MAX: u32 = 1760;
const FREQ_STEP: u32 = 16;

/// A chunk buffer, aligned to a cache line so the DMA source clean stays
/// on its own lines.
#[repr(C, align(64))]
struct Chunk([u32; CHUNK_WORDS]);

/// The two ping-pong buffers the DMA engine alternates between.
static mut BUF0: Chunk = Chunk([0; CHUNK_WORDS]);
static mut BUF1: Chunk = Chunk([0; CHUNK_WORDS]);

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// Value of a sine at `deg` degrees (`0..360`), as an unsigned duty in
/// `0..=range` (a raised sine centred on `range / 2`). Bhaskara I's
/// integer approximation — no floating point or `libm`.
fn sine_deg(deg: i64, range: u16) -> u32 {
    let (x, sign) = if deg <= 180 {
        (deg, 1i64)
    } else {
        (deg - 180, -1i64)
    };
    let t = x * (180 - x);
    let num = 4 * t;
    let den = 40_500 - t;
    let half = range as i64 / 2;
    let duty = half + sign * half * num / den;
    duty.clamp(0, range as i64) as u32
}

/// A swept-sine oscillator. Holds a running phase (so successive chunks
/// join seamlessly) and a frequency that ramps between `FREQ_MIN` and
/// `FREQ_MAX`, reversing at each bound.
struct Siren {
    /// Phase in 1/256-degree units, `0..360*256`, advanced per sample.
    phase: u32,
    /// Current tone frequency in Hz.
    freq: u32,
    /// Whether the frequency is currently ramping up.
    rising: bool,
}

impl Siren {
    /// Fills `buf` with the next chunk of interleaved stereo samples
    /// (left == right), then steps the sweep frequency for the next chunk.
    fn fill(&mut self, buf: &mut [u32]) {
        let (frames, _) = buf.as_chunks_mut::<2>();
        for frame in frames {
            // Same value to channel 1 and channel 2 — mono content on both.
            let sample = sine_deg((self.phase >> 8) as i64, RANGE);
            frame[0] = sample; // channel 1
            frame[1] = sample; // channel 2
                               // Advance the phase one sample at the current frequency:
                               // 360°/cycle × freq / sample_rate, in the 1/256-degree fixed
                               // point `phase` uses.
            let inc = 360 * self.freq * 256 / SAMPLE_RATE;
            self.phase = (self.phase + inc) % (360 * 256);
        }
        // Ramp the frequency for the next chunk, bouncing at the bounds.
        if self.rising {
            self.freq += FREQ_STEP;
            if self.freq >= FREQ_MAX {
                self.freq = FREQ_MAX;
                self.rising = false;
            }
        } else {
            self.freq -= FREQ_STEP;
            if self.freq <= FREQ_MIN {
                self.freq = FREQ_MIN;
                self.rising = true;
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "Starting streamed PWM audio (siren)...");

    // SAFETY: single-threaded `kmain`; these statics are touched only here
    // and stay borrowed by the DMA stream for the rest of the program.
    let buf0 = unsafe { &mut *core::ptr::addr_of_mut!(BUF0) };
    let buf1 = unsafe { &mut *core::ptr::addr_of_mut!(BUF1) };

    let mut siren = Siren {
        phase: 0,
        freq: FREQ_MIN,
        rising: true,
    };
    // Prime both buffers with the first two chunks before the engine
    // starts reading them.
    siren.fill(&mut buf0.0);
    siren.fill(&mut buf1.0);

    // Bring up the PWM in FIFO/audio mode at the target sample rate.
    let divisor = Pwm::audio_clock_divisor(SAMPLE_RATE, RANGE);
    let pwm = Pwm::init(peripherals.PWM0, peripherals.CM_PWM, divisor);
    let audio = pwm.audio(
        &peripherals.GPIO,
        Channel1Pin::Gpio40,
        Channel2Pin::Gpio45,
        RANGE,
    );

    // Start the ping-pong stream. A full channel (0–6); any works on a
    // board this program has taken over (see `rpi_hal::dma` docs).
    let mut dma = Dma::new();
    let mut channel = dma.channel(5).expect("channel 5 available");
    let mut stream = channel
        .stream_peripheral(
            [&mut buf0.0, &mut buf1.0],
            audio.dreq(),
            audio.fifo_bus_address(),
        )
        .expect("start audio stream");

    let _ = writeln!(
        uart,
        "Streaming: sweep {FREQ_MIN}..{FREQ_MAX} Hz (divisor {divisor})"
    );

    let mut chunks: u32 = 0;
    loop {
        // Refill whichever buffer the engine has just finished. Blocks
        // until that buffer is free, so this loop is naturally paced to
        // the audio rate — no manual delay needed.
        stream.feed(|buf| siren.fill(buf));
        chunks += 1;
        // Heartbeat every ~0.5 s (a chunk is ~11.6 ms).
        if chunks.is_multiple_of(43) {
            let _ = writeln!(uart, "streaming ({chunks} chunks)");
        }
    }
}
