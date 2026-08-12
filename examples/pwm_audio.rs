//! PWM analog-audio tone, streamed by DMA.
//!
//! Plays a sustained sine tone out both PWM channels (stereo) by feeding
//! the PWM FIFO from a RAM buffer over a cyclic DMA transfer — the CPU
//! sets it up once and then does nothing per-sample, which the UART
//! heartbeat proves by continuing to print while the tone keeps playing.
//!
//! Wiring / observing: drive the analog-audio pins (GPIO40 / GPIO45),
//! which reach the 3.5 mm jack on boards that break it out — the tone is
//! then audible on headphones. On any pin, a scope shows the PWM carrier
//! amplitude-following the sine; an RC low-pass recovers the clean tone.
//! Self-contained: needs no external hardware to run.

#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::dma::Dma;
use rpi_hal::halt;
use rpi_hal::pwm::{Channel1Pin, Channel2Pin, Pwm};
use rpi_hal::{pac, uart::Uart};

/// Nominal sample rate. Real rate is approximate — integer clock divisor
/// off a PLL-derived clock (see `rpi_hal::pwm`'s clock docs).
const SAMPLE_RATE: u32 = 44_100;
/// PWM period in clock ticks, and therefore the sample range: each sample
/// is a duty value in `0..=RANGE`. 1024 gives ~10-bit resolution.
const RANGE: u16 = 1024;
/// Samples per sine period (mono). The tone frequency is `SAMPLE_RATE /
/// PERIOD_SAMPLES` (~441 Hz), and holding a whole number of periods in
/// the buffer makes the cyclic DMA loop seamlessly.
const PERIOD_SAMPLES: usize = 100;
/// Interleaved stereo buffer length: one L and one R word per sample.
const STEREO_LEN: usize = PERIOD_SAMPLES * 2;

/// The sine period, interleaved L/R, aligned to a cache line so the DMA
/// source clean stays on its own lines.
#[repr(C, align(64))]
struct Tone([u32; STEREO_LEN]);

/// One period of the tone, filled at startup and then streamed forever.
static mut TONE: Tone = Tone([0; STEREO_LEN]);

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "PANIC: {info}");
    halt();
}

/// Sample `index` of a full sine period, as an unsigned duty in
/// `0..=range` (a raised sine centred on `range / 2`). Uses Bhaskara I's
/// integer sine approximation so no floating point or `libm` is needed.
fn sine_sample(index: usize, range: u16) -> u32 {
    // Phase in degrees, 0..360 across the period.
    let degrees = (360 * index / PERIOD_SAMPLES) as i64;
    // Bhaskara is defined on 0..=180; fold the second half and negate.
    let (x, sign) = if degrees <= 180 {
        (degrees, 1i64)
    } else {
        (degrees - 180, -1i64)
    };
    // sin(x°) ≈ 4x(180-x) / (40500 - x(180-x)), a value in 0..=1.
    let t = x * (180 - x);
    let num = 4 * t;
    let den = 40_500 - t;
    // duty = range/2 * (1 + sign * num/den), clamped into 0..=range.
    let half = range as i64 / 2;
    let duty = half + sign * half * num / den;
    duty.clamp(0, range as i64) as u32
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };
    let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);
    let _ = writeln!(uart, "Starting PWM audio tone...");

    // SAFETY: single-threaded `kmain`; this static is touched only here,
    // and stays borrowed by the DMA transfer for the rest of the program.
    let tone = unsafe { &mut *core::ptr::addr_of_mut!(TONE) };
    for i in 0..PERIOD_SAMPLES {
        let sample = sine_sample(i, RANGE);
        // Even word → channel 1, odd word → channel 2 (round-robin from
        // the shared FIFO). Same value to both, so this is mono content on
        // both channels regardless of which is the left/right contact.
        tone.0[i * 2] = sample; // channel 1
        tone.0[i * 2 + 1] = sample; // channel 2
    }

    // Clock the PWM so `pwm_clock_hz / RANGE` lands on the target sample
    // rate, then bring up both channels in FIFO/audio mode.
    let divisor = Pwm::audio_clock_divisor(SAMPLE_RATE, RANGE);
    let pwm = Pwm::init(peripherals.PWM0, peripherals.CM_PWM, divisor);
    let audio = pwm.audio(
        &peripherals.GPIO,
        Channel1Pin::Gpio40,
        Channel2Pin::Gpio45,
        RANGE,
    );

    // Stream the tone into the FIFO, cyclically, so it plays indefinitely
    // with no further CPU involvement. A full channel (0–6); any works on
    // a board this program has taken over (see `rpi_hal::dma` docs).
    let mut dma = Dma::new();
    let mut channel = dma.channel(5).expect("channel 5 available");
    let _transfer = channel
        .write_peripheral(&tone.0, audio.dreq(), audio.fifo_bus_address(), true)
        .expect("start audio DMA");

    let _ = writeln!(
        uart,
        "Tone playing: ~{} Hz, sample rate ~{} Hz (divisor {})",
        SAMPLE_RATE as usize / PERIOD_SAMPLES,
        SAMPLE_RATE,
        divisor
    );

    let mut beat: u32 = 0;
    loop {
        // Heartbeat: the DMA engine feeds the FIFO on its own, so the CPU
        // is free — these keep printing while the tone plays.
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
