//! Audio out over HDMI — a continuous stereo tone (distinct left/right
//! pitches so the channel separation is audible), played through the
//! VideoCore firmware's audio renderer.
//!
//! Unlike the other audio examples here there is nothing to wire up:
//! `pwm_audio.rs` drives the 3.5 mm jack and `i2s_dac_tone.rs` an external
//! DAC, but HDMI audio travels inside the signal already going to the
//! display, so the only hardware needed is a display (or an HDMI audio
//! extractor) plugged into the HDMI connector, with speakers of its own.
//!
//! You should hear a steady A4 (440 Hz) in the left channel and E5 (660 Hz)
//! in the right, for [`PLAY_SECONDS`], out of whatever the display's audio
//! output is.
//!
//! Two things about the display side are worth knowing before concluding
//! the code is at fault:
//!
//! - The firmware only puts audio on the link when it is driving it as
//!   *HDMI* rather than DVI, which it decides from the display's own EDID.
//!   A display that doesn't announce itself as HDMI (or a DVI adapter in
//!   the path) gets no audio channel at all, and the fix is `hdmi_drive=2`
//!   in `config.txt`, which forces HDMI mode.
//! - Which formats work is negotiated between the firmware and the display,
//!   not chosen here. 48 kHz stereo, which this uses, is the one every HDMI
//!   sink supports.
//!
//! The lines printed at the end are the real check, and the ones worth
//! reading even when there are no speakers to hand: the renderer takes
//! samples no faster than it plays them, so the rate this example manages
//! to feed at *is* the rate the hardware is consuming them. Feeding much
//! faster than [`SAMPLE_RATE`] would mean the samples were going nowhere.
//!
//! Two rates are printed because the renderer queues about a third of a
//! second of audio of its own before it starts pacing anything. That
//! fill is real audio and counts, but it lands in a rush at the start, so
//! it lifts the whole-run average several percent above the sample rate on
//! a run this short. The steady-state figure — measured from
//! [`SETTLE_SECONDS`] in, by which time the queue is full — is the one that
//! should land within a fraction of a percent of [`SAMPLE_RATE`].
//!
//! Change [`DESTINATION`] to `Destination::Local` to send the same tone out
//! of the 3.5 mm jack instead — the same component, the same code, and a
//! useful A/B if HDMI stays silent, since it separates "the renderer isn't
//! running" from "the display isn't taking the audio".

#![no_std]
#![no_main]

use core::fmt::Write;
use rpi_hal::audio_render::{AudioRenderer, Destination, Format};
use rpi_hal::halt;
use rpi_hal::mailbox::Mailbox;
use rpi_hal::mmal;
use rpi_hal::pac;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;
use rpi_hal::vchiq::{SharedMemory, Vchiq};

/// Where the audio goes. `Destination::Local` is the 3.5 mm jack.
const DESTINATION: Destination = Destination::Hdmi;

/// Sample rate, in frames per second.
const SAMPLE_RATE: u32 = 48_000;

/// Seconds of tone to play before shutting the renderer down.
const PLAY_SECONDS: u64 = 10;

/// Seconds to let the renderer's own queue fill before starting the
/// steady-state measurement. Comfortably longer than the queue itself,
/// which holds about a third of a second.
const SETTLE_SECONDS: u64 = 2;

/// Tone amplitude, well below full scale (`i16::MAX`) to leave headroom.
const AMPLITUDE: i32 = 8_000;

/// Left- and right-channel tone frequencies, in Hz — deliberately
/// different so a working stereo path is audible as two distinct pitches.
const FREQ_LEFT: u32 = 440;
const FREQ_RIGHT: u32 = 660;

/// Stereo frames per buffer: 1024, which is 4096 bytes and ~21 ms of audio.
/// Also the smallest the renderer accepts — it reports a `buffer_size_min`
/// of 4096, and anything under that is refused at `start` rather than
/// played short.
const CHUNK_FRAMES: usize = 1024;

/// Samples per buffer — one per channel per frame.
const CHUNK_SAMPLES: usize = CHUNK_FRAMES * 2;

/// Bytes per buffer.
const CHUNK_BYTES: usize = CHUNK_SAMPLES * 2;

/// Buffers handed to the renderer. Together they are how far ahead of the
/// audio this can get: four of them is ~85 ms of slack, which is plenty for
/// a loop whose only other work is generating the next chunk.
const BUFFERS: usize = 4;

/// A buffer handed to the renderer. Cache-line aligned because the firmware
/// fetches these by DMA, and that maintenance works in whole 64-byte lines.
#[repr(C, align(64))]
struct Aligned([u8; CHUNK_BYTES]);

/// The sample buffers.
static mut SAMPLES: [Aligned; BUFFERS] = [const { Aligned([0; CHUNK_BYTES]) }; BUFFERS];

/// The region VCHIQ shares with the VideoCore. Must outlive everything: the
/// firmware keeps reading it for as long as the board is up.
static mut VCHIQ_MEMORY: SharedMemory = SharedMemory::new();

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

    let _ = writeln!(uart, "creating the renderer ({DESTINATION:?})...");
    let format = Format {
        sample_rate: SAMPLE_RATE,
        channels: 2,
    };
    let mut audio = match AudioRenderer::new(vchiq, DESTINATION, format, &timer) {
        Ok(audio) => audio,
        Err(error) => {
            let _ = writeln!(uart, "renderer create failed: {error:?}");
            halt();
        }
    };

    // SAFETY: each buffer is handed to the renderer exactly once, and
    // nothing here touches the static again -- ownership moves with the
    // slices.
    unsafe {
        for buffer in &mut *core::ptr::addr_of_mut!(SAMPLES) {
            let _ = audio.add_buffer(&mut buffer.0);
        }
    }

    if let Err(error) = audio.start(&timer) {
        let _ = writeln!(uart, "renderer start failed: {error:?}");
        halt();
    }

    // What the component settled on, which is not necessarily what it was
    // asked for -- and the first thing to check if nothing plays.
    let _ = writeln!(uart, "input {:?}", audio.input_port());

    if let Err(error) = play(&mut audio, &timer, &mut uart) {
        let _ = writeln!(uart, "playback failed: {error:?}");
        // A stalled exchange with the firmware says nothing about itself,
        // so print what crossed the interface before it stopped -- sent
        // against returned is what identifies the half that went quiet.
        let mmal_stats = audio.mmal().stats();
        let vchiq_stats = audio.mmal().vchiq().stats();
        let _ = writeln!(uart, "mmal:  {mmal_stats:?}");
        let _ = writeln!(uart, "vchiq: {vchiq_stats:?}");
    }

    halt();
}

/// Feeds the tone to the renderer for [`PLAY_SECONDS`], then waits for the
/// last of it to be played.
fn play(
    audio: &mut AudioRenderer,
    timer: &Timer,
    uart: &mut Uart,
) -> Result<(), rpi_hal::audio_render::Error> {
    let mut tones = Tones {
        phase_l: 0,
        phase_r: 0,
    };
    let mut chunk = [0i16; CHUNK_SAMPLES];
    let mut frames = 0u64;
    // The same two counts restarted once the renderer's own queue has
    // filled, which is what isolates the rate it plays at from the rush of
    // buffers it takes while filling.
    let mut steady_frames = 0u64;
    let mut steady_start = 0u64;

    let start = timer.now_micros();
    let settled = start + SETTLE_SECONDS * 1_000_000;
    let deadline = start + PLAY_SECONDS * 1_000_000;
    loop {
        let now = timer.now_micros();
        if now >= deadline {
            break;
        }
        if steady_start == 0 && now >= settled {
            steady_start = now;
        }

        tones.fill(&mut chunk);

        // Offer the chunk until it has all been taken. Every buffer being
        // with the firmware is the normal case rather than an error: it
        // means this loop has got as far ahead of the audio as the buffers
        // allow, and the wait is the renderer pacing it.
        let mut position = 0;
        while position < chunk.len() {
            audio.poll()?;
            position += audio.feed(&chunk[position..], mmal::TIME_UNKNOWN, timer)?;
        }
        frames += CHUNK_FRAMES as u64;
        if steady_start != 0 {
            steady_frames += CHUNK_FRAMES as u64;
        }
    }

    // Both rates are per second *of feeding*, so the clock stops here,
    // before the draining below: that waits on audio already handed over,
    // and counting it would report a consumption rate a percent or so under
    // the real one for no better reason than that the run ended.
    let end = timer.now_micros();

    // Wait for the renderer to take back every buffer, and then for what it
    // has queued of its own to finish playing -- there is no way to ask how
    // much that is, so this waits out several times what it has been
    // measured to hold. Halting before it drains cuts the tail of the audio
    // off.
    let drain_deadline = timer.now_micros() + DRAIN_TIMEOUT_US;
    while audio.buffers_in_flight() > 0 && timer.now_micros() < drain_deadline {
        audio.poll()?;
    }
    timer.delay_us(QUEUE_DRAIN_US);

    let elapsed_us = end - start;
    let steady_us = end - steady_start.max(start);
    let _ = writeln!(
        uart,
        "\nplayed {frames} frames in {} ms ({} frames/s overall, \
         {} frames/s steady-state, expected {SAMPLE_RATE})",
        elapsed_us / 1000,
        (frames * 1_000_000).checked_div(elapsed_us).unwrap_or(0),
        (steady_frames * 1_000_000)
            .checked_div(steady_us)
            .unwrap_or(0)
    );
    Ok(())
}

/// How long to wait for the renderer to hand back the buffers it still
/// holds, in microseconds. Longer than the audio they can hold, so reaching
/// it means the firmware has stopped rather than that the audio is still
/// going.
const DRAIN_TIMEOUT_US: u64 = 1_000_000;

/// How long to wait after that for the renderer's own queue to finish
/// playing, in microseconds. Nothing reports its depth; this is three
/// times the third of a second it has been measured to hold.
const QUEUE_DRAIN_US: u32 = 1_000_000;

/// Value of a sine at `deg` degrees (`0..360`), as a signed sample in
/// `-amp..=amp`. Bhaskara I's integer approximation — no floating point or
/// `libm` (the same one `i2s_dac_tone.rs` uses).
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
    /// See [`Self::phase_l`].
    phase_r: u32,
}

impl Tones {
    /// Fills `chunk` with the next block of interleaved stereo samples:
    /// `FREQ_LEFT` on the left channel, `FREQ_RIGHT` on the right.
    fn fill(&mut self, chunk: &mut [i16]) {
        let (frames, _) = chunk.as_chunks_mut::<2>();
        for frame in frames {
            frame[0] = sine_deg((self.phase_l >> 8) as i64, AMPLITUDE) as i16;
            frame[1] = sine_deg((self.phase_r >> 8) as i64, AMPLITUDE) as i16;

            // Advance each phase one sample: 360°/cycle × freq / sample_rate,
            // in the 1/256-degree fixed point the phases use.
            let inc_l = 360 * FREQ_LEFT * 256 / SAMPLE_RATE;
            let inc_r = 360 * FREQ_RIGHT * 256 / SAMPLE_RATE;
            self.phase_l = (self.phase_l + inc_l) % (360 * 256);
            self.phase_r = (self.phase_r + inc_r) % (360 * 256);
        }
    }
}
