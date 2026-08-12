//! Genuine-stereo PWM analog audio from embedded PCM.
//!
//! Where the other audio examples play identical content on both channels
//! (mono, so they don't actually exercise the shared FIFO's per-channel
//! split), this one plays *different* audio on each channel to prove real
//! stereo: a short clip that speaks "left" out one channel while the other
//! is silent, then "right" out the other. If the FIFO's round-robin
//! channel assignment is working, each word comes out its own speaker; if
//! it were mixing or swapping channels, you'd hear both words on both, or
//! on the wrong sides.
//!
//! The audio is embedded directly in the binary (`include_bytes!`) as
//! signed 16-bit interleaved PCM at 44.1 kHz — synthesised offline with
//! `espeak-ng` (so the asset is license-free), trimmed and laid out into
//! two channels. At startup each `i16` sample is converted to a PWM duty
//! value with `pcm_to_duty`, then the whole clip is streamed on a loop by
//! a cyclic DMA transfer.
//!
//! Channel mapping is board-dependent (see `rpi_hal::pwm`): on the board
//! this was made for, channel 1 (GPIO40) is the right contact and channel
//! 2 (GPIO45) the left, and the asset is arranged so "left" plays on the
//! left speaker and "right" on the right. Self-contained: no external
//! hardware beyond the 3.5 mm jack.

#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::dma::Dma;
use rpi_hal::halt;
use rpi_hal::pwm::{pcm_to_duty, Channel1Pin, Channel2Pin, Pwm};
use rpi_hal::{pac, uart::Uart};

/// Sample rate of the embedded PCM. Must stay near 44.1 kHz: in PWM FIFO
/// mode the carrier frequency equals the sample rate (one PWM period per
/// sample), so a lower rate would put an audible carrier whine in band.
const SAMPLE_RATE: u32 = 44_100;
/// Sample range / bit depth: each duty value lands in `0..=RANGE`.
const RANGE: u16 = 1024;

/// The stereo clip: signed 16-bit little-endian PCM, interleaved
/// channel 1 / channel 2, 44.1 kHz. See this file's docs for how it was
/// produced.
const PCM: &[u8] = include_bytes!("assets/stereo_lr.pcm");
/// One PWM duty word per `i16` sample (2 bytes each).
const WORDS: usize = PCM.len() / 2;

/// The clip converted to PWM duty words, aligned to a cache line so the
/// DMA source clean stays on its own lines.
#[repr(C, align(64))]
struct Samples([u32; WORDS]);

/// Storage for the converted clip, filled at startup and streamed forever.
static mut SAMPLES: Samples = Samples([0; WORDS]);

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
    let _ = writeln!(uart, "Starting stereo PWM audio ({WORDS} samples)...");

    // SAFETY: single-threaded `kmain`; this static is touched only here,
    // and stays borrowed by the DMA transfer for the rest of the program.
    let samples = unsafe { &mut *core::ptr::addr_of_mut!(SAMPLES) };
    // Convert each interleaved i16 PCM sample to a PWM duty word. The
    // interleaving (ch1, ch2, ch1, …) is preserved, so it lands back on
    // the FIFO in the order the two channels expect.
    let (pcm_samples, _) = PCM.as_chunks::<2>();
    for (word, bytes) in samples.0.iter_mut().zip(pcm_samples) {
        *word = pcm_to_duty(i16::from_le_bytes(*bytes), RANGE) as u32;
    }

    // Bring up both channels for stereo, at the clip's sample rate.
    let divisor = Pwm::audio_clock_divisor(SAMPLE_RATE, RANGE);
    let pwm = Pwm::init(peripherals.PWM0, peripherals.CM_PWM, divisor);
    let audio = pwm.audio(
        &peripherals.GPIO,
        Channel1Pin::Gpio40,
        Channel2Pin::Gpio45,
        RANGE,
    );

    // Stream the whole clip cyclically, so it repeats "left … right …"
    // (the clip's own lead/tail silence spaces the repeats). A full
    // channel (0–6); any works on a board this program has taken over.
    let mut dma = Dma::new();
    let mut channel = dma.channel(5).expect("channel 5 available");
    let _transfer = channel
        .write_peripheral(&samples.0, audio.dreq(), audio.fifo_bus_address(), true)
        .expect("start audio DMA");

    let _ = writeln!(uart, "Playing stereo clip on loop (divisor {divisor})");

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
