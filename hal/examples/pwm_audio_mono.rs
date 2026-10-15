//! Mono PWM analog audio from signed 16-bit PCM.
//!
//! The counterpart to `pwm_audio.rs` (stereo): drives a single PWM
//! channel via `Pwm::audio_mono`, so every FIFO word is one sample with
//! no left/right interleaving. It also shows the `pcm_to_duty` helper —
//! the tone is generated as ordinary signed `i16` PCM (what a real audio
//! source produces) and converted to PWM duty values, rather than being
//! computed as duty directly.
//!
//! Observe on the driven analog-audio pin (GPIO40) — an ~441 Hz tone on
//! that jack contact, or a sine on a scope after an RC low-pass.
//! Self-contained: no external hardware.

#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::dma::Dma;
use rpi_hal::halt;
use rpi_hal::pwm::{pcm_to_duty, Channel1Pin, Pwm};
use rpi_hal::{pac, uart::Uart};

/// Nominal sample rate (approximate — see `rpi_hal::pwm`'s clock docs).
const SAMPLE_RATE: u32 = 44_100;
/// Sample range / bit depth: each duty value lands in `0..=RANGE`.
const RANGE: u16 = 1024;
/// Samples per sine period. Tone ≈ `SAMPLE_RATE / PERIOD_SAMPLES` (~441
/// Hz); a whole number of periods makes the cyclic DMA loop seamlessly.
const PERIOD_SAMPLES: usize = 100;

/// One period of mono samples (one word each, no interleaving), aligned
/// to a cache line so the DMA source clean stays on its own lines.
#[repr(C, align(64))]
struct Tone([u32; PERIOD_SAMPLES]);

/// One period of the tone, filled at startup and streamed forever.
static mut TONE: Tone = Tone([0; PERIOD_SAMPLES]);

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// Sample `index` of a sine period as signed 16-bit PCM (`i16::MIN..=
/// i16::MAX`, silence at 0) — the representation a real audio source
/// uses. Bhaskara I's integer sine approximation, scaled to `i16`; no
/// floating point or `libm`.
fn sine_pcm(index: usize) -> i16 {
    let degrees = (360 * index / PERIOD_SAMPLES) as i64;
    let (x, sign) = if degrees <= 180 {
        (degrees, 1i64)
    } else {
        (degrees - 180, -1i64)
    };
    let t = x * (180 - x);
    // sin ≈ 4t / (40500 - t), in 0..=1; scale by i16::MAX and sign.
    let amplitude = sign * i16::MAX as i64 * 4 * t / (40_500 - t);
    amplitude.clamp(i16::MIN as i64, i16::MAX as i64) as i16
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "Starting mono PWM audio (PCM)...");

    // SAFETY: single-threaded `kmain`; this static is touched only here,
    // and stays borrowed by the DMA transfer for the rest of the program.
    let tone = unsafe { &mut *core::ptr::addr_of_mut!(TONE) };
    for (i, word) in tone.0.iter_mut().enumerate() {
        // Generate as i16 PCM, then convert to a PWM duty value.
        *word = pcm_to_duty(sine_pcm(i), RANGE) as u32;
    }

    // Bring up a single channel in FIFO/audio mode at the target rate.
    let divisor = Pwm::audio_clock_divisor(SAMPLE_RATE, RANGE);
    let pwm = Pwm::init(peripherals.PWM0, peripherals.CM_PWM, divisor);
    let audio = pwm.audio_mono(&peripherals.GPIO, Channel1Pin::Gpio40, RANGE);

    // Stream the tone into the FIFO cyclically — plays indefinitely with
    // no further CPU work. A full channel (0–6); any works on a board this
    // program has taken over (see `rpi_hal::dma` docs).
    let mut dma = Dma::new();
    let mut channel = dma.channel(5).expect("channel 5 available");
    let _transfer = channel
        .write_peripheral(&tone.0, audio.dreq(), audio.fifo_bus_address(), true)
        .expect("start audio DMA");

    let _ = writeln!(
        uart,
        "Mono tone playing: ~{} Hz (divisor {divisor})",
        SAMPLE_RATE as usize / PERIOD_SAMPLES
    );

    let mut beat: u32 = 0;
    loop {
        delay(50_000_000);
        beat += 1;
        let _ = writeln!(uart, "still playing ({beat})");
    }
}

fn delay(cycles: u32) {
    for _ in 0..cycles {
        unsafe { core::arch::asm!("nop") };
    }
}
