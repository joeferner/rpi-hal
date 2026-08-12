//! Digital audio out over I2S to an external DAC — a continuous stereo
//! tone (distinct left/right pitches so the channel separation is
//! audible).
//!
//! Wire an I2S DAC (this was brought up on a GY-PCM5102A board) to the I2S
//! pins on the Pi's 40-pin header. Signal wiring:
//!
//! | DAC pin | Pi        | Pi phys pin |
//! |---------|-----------|-------------|
//! | `BCK`   | GPIO18    | 12          |
//! | `LCK`   | GPIO19    | 35          |
//! | `DIN`   | GPIO21    | 40          |
//! | `GND`   | GND       | 6/9/14/…    |
//! | `VIN`   | 5V        | 2           |
//!
//! The critical, easy-to-miss part is the PCM5102A's control pins. The
//! BCM PCM peripheral is the clock master but emits **no** master/system
//! clock (only `BCK`/`LCK`/`DIN`), so the DAC must regenerate its system
//! clock internally — and that only happens with `SCK` grounded. Tie these
//! to the DAC's logic levels (3.3 V, *not* 5 V — use the Pi's 3.3 V rail or
//! the DAC's own, or its `HxL` solder bridges):
//!
//! - `SCK`  → **GND** — enables the internal PLL; floating = no clock = silence.
//! - `XSMT` → **High** — un-mute; low = silence.
//! - `FMT`  → **Low** — I2S format (matches this driver; high = left-justified).
//! - `FLT`  → Low — normal-latency filter (either level is fine).
//! - `DEMP` → Low — de-emphasis off.
//!
//! `ROUT`/`LOUT` (referenced to `AGND`) are the analog outputs — feed an
//! amp / powered speakers / headphones. With all that, you should hear a
//! steady A4 (440 Hz) in the left channel and E5 (660 Hz) in the right; a
//! scope on GPIO18/19/21 shows the I2S bit clock, frame clock, and serial
//! data even without a DAC attached.
//!
//! Uses the double-buffered (ping-pong) DMA stream: while the engine
//! clocks one buffer out to the DAC, the CPU fills the other, gaplessly —
//! the same shape as `pwm_audio_stream.rs`, but the samples are signed PCM
//! the DAC converts directly rather than PWM duty values.

#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::dma::Dma;
use rpi_hal::halt;
use rpi_hal::pcm::{pcm_sample, Pcm};
use rpi_hal::{pac, uart::Uart};

/// Nominal sample rate (approximate — see `rpi_hal::pcm`'s clock docs).
const SAMPLE_RATE: u32 = 48_000;
/// Tone amplitude, well below full scale (`i16::MAX`) to leave headroom.
const AMPLITUDE: i32 = 8_000;
/// Left- and right-channel tone frequencies, in Hz — deliberately
/// different so a working stereo path is audible as two distinct pitches.
const FREQ_LEFT: u32 = 440;
const FREQ_RIGHT: u32 = 660;

/// Stereo frames per buffer chunk. A chunk is `CHUNK_FRAMES * 2` words
/// (one left + one right word per frame); at 512 that's 4096 bytes, a
/// whole number of cache lines. ~10.7 ms of audio per chunk at 48 kHz.
const CHUNK_FRAMES: usize = 512;
/// Words per chunk buffer (interleaved L/R).
const CHUNK_WORDS: usize = CHUNK_FRAMES * 2;

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

/// Value of a sine at `deg` degrees (`0..360`), as a signed sample in
/// `-amp..=amp`. Bhaskara I's integer approximation — no floating point or
/// `libm` (same approximation `pwm_audio_stream.rs` uses, but signed and
/// centred on zero, the form an I2S DAC wants).
fn sine_deg(deg: i64, amp: i32) -> i32 {
    let (x, sign) = if deg <= 180 {
        (deg, 1i64)
    } else {
        (deg - 180, -1i64)
    };
    let t = x * (180 - x);
    let num = 4 * t;
    let den = 40_500 - t;
    (sign * amp as i64 * num / den) as i32
}

/// A two-tone stereo oscillator. Holds a running phase per channel (so
/// successive chunks join seamlessly).
struct Tones {
    /// Left/right phase in 1/256-degree units, `0..360*256`.
    phase_l: u32,
    phase_r: u32,
}

impl Tones {
    /// Fills `buf` with the next chunk of interleaved stereo samples:
    /// `FREQ_LEFT` on channel 1 (left), `FREQ_RIGHT` on channel 2 (right).
    fn fill(&mut self, buf: &mut [u32]) {
        let (frames, _) = buf.as_chunks_mut::<2>();
        for frame in frames {
            let left = sine_deg((self.phase_l >> 8) as i64, AMPLITUDE) as i16;
            let right = sine_deg((self.phase_r >> 8) as i64, AMPLITUDE) as i16;
            frame[0] = pcm_sample(left); // channel 1 (left)
            frame[1] = pcm_sample(right); // channel 2 (right)

            // Advance each phase one sample: 360°/cycle × freq / sample_rate,
            // in the 1/256-degree fixed point the phases use.
            let inc_l = 360 * FREQ_LEFT * 256 / SAMPLE_RATE;
            let inc_r = 360 * FREQ_RIGHT * 256 / SAMPLE_RATE;
            self.phase_l = (self.phase_l + inc_l) % (360 * 256);
            self.phase_r = (self.phase_r + inc_r) % (360 * 256);
        }
    }
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "Starting I2S stereo tone...");

    // SAFETY: single-threaded `kmain`; these statics are touched only here
    // and stay borrowed by the DMA stream for the rest of the program.
    let buf0 = unsafe { &mut *core::ptr::addr_of_mut!(BUF0) };
    let buf1 = unsafe { &mut *core::ptr::addr_of_mut!(BUF1) };

    let mut tones = Tones {
        phase_l: 0,
        phase_r: 0,
    };
    // Prime both buffers before the engine starts reading them.
    tones.fill(&mut buf0.0);
    tones.fill(&mut buf1.0);

    // Bring up the PCM/I2S peripheral in master mode at the target sample
    // rate and start transmission.
    let divisor = Pcm::clock_divisor(SAMPLE_RATE);
    let pcm = Pcm::init(peripherals.CM_PCM, divisor);
    let i2s = pcm.i2s_out(&peripherals.GPIO);

    // Start the ping-pong stream. A full channel (0–6); any works on a
    // board this program has taken over (see `rpi_hal::dma` docs).
    let mut dma = Dma::new();
    let mut channel = dma.channel(5).expect("channel 5 available");
    let mut stream = channel
        .stream_peripheral(
            [&mut buf0.0, &mut buf1.0],
            i2s.dreq(),
            i2s.fifo_bus_address(),
        )
        .expect("start I2S stream");

    let _ = writeln!(
        uart,
        "Streaming: L={FREQ_LEFT} Hz, R={FREQ_RIGHT} Hz (divisor {divisor})"
    );

    let mut chunks: u32 = 0;
    loop {
        // Refill whichever buffer the engine has just finished. Blocks
        // until that buffer is free, so this loop is naturally paced to the
        // audio rate — no manual delay needed.
        stream.feed(|buf| tones.fill(buf));
        chunks += 1;
        // Heartbeat every ~0.5 s (a chunk is ~10.7 ms).
        if chunks.is_multiple_of(47) {
            let _ = writeln!(uart, "streaming ({chunks} chunks)");
        }
    }
}
