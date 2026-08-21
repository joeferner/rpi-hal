//! Audio out through the VideoCore firmware: HDMI, or the analog jack.
//!
//! HDMI carries audio inside the same signal it carries video in, which on
//! these boards means inside a link this side never programs: the display's
//! resolution, timing and audio capabilities are negotiated by the closed
//! firmware, and the packets that carry samples are inserted by it too.
//! There is no register block to drive for it the way [`crate::pwm`] drives
//! the analog jack or [`crate::pcm`] drives an external I2S DAC. What is
//! reachable is the firmware's own audio sink — the `ril.audio_render` MMAL
//! component — so that is what this module drives, over [`crate::mmal`] and
//! [`crate::vchiq`], the same road [`crate::video_decode`] takes to the
//! H.264 block.
//!
//! The same component also feeds the 3.5 mm jack, as
//! [`Destination::Local`](crate::audio_render::Destination::Local). That
//! path is already reachable without the
//! firmware, by driving the PWM hardware directly ([`crate::pwm`]'s
//! `audio`), and the two are worth telling apart: the PWM route owns the
//! timing itself and costs a DMA channel, this one hands samples to the
//! firmware and costs a VCHIQ service. Only this one can reach HDMI.
//!
//! ## Using it
//!
//! ```ignore
//! let format = Format { sample_rate: 48_000, channels: 2 };
//! let mut audio = AudioRenderer::new(vchiq, Destination::Hdmi, format, &timer)?;
//! audio.add_buffer(buffer)?; // repeat for a few
//! audio.start(&timer)?;
//!
//! loop {
//!     audio.poll()?;
//!     // Whatever this doesn't take, offer again after the next poll.
//!     position += audio.feed(&samples[position..], mmal::TIME_UNKNOWN, &timer)?;
//! }
//! ```
//!
//! ## Pacing
//!
//! Nothing here has to keep time. The renderer takes samples at the rate it
//! plays them, so an application that feeds until
//! [`feed`](crate::audio_render::AudioRenderer::feed) returns zero and then
//! polls is paced by the audio clock rather than by any delay it chooses.
//!
//! Not from the first buffer, though. The renderer queues about a third of
//! a second of audio of its own — visible as it giving those first buffers
//! back far faster than real time, and so accepting that much more than it
//! has played — and only settles to the sample rate once that queue is
//! full. It is a fixed amount rather than a fast sample clock: a run twice
//! as long overshoots by the same 330 ms, not by twice as much. Two things
//! follow from it. A short run measures a feed rate a few percent above the
//! sample rate, all of it that initial fill. And
//! [`buffers_in_flight`](crate::audio_render::AudioRenderer::buffers_in_flight)
//! reaching zero means every buffer has been *taken*, not that every sample
//! has been *played* — there is still the firmware's queue to come out, so
//! something that stops the moment the last buffer returns loses the tail
//! of the audio.
//!
//! Latency is that queue plus whatever the caller's own buffers hold, which
//! is the reason to size them deliberately rather than as large as
//! convenient.
//!
//! ## Samples
//!
//! Signed 16-bit, in the platform's little-endian order, channels
//! interleaved sample by sample — the same shape [`crate::pcm`] and
//! [`crate::pwm`]'s audio paths take, and what
//! [`ENCODING_PCM_SIGNED_LE`](crate::mmal::ENCODING_PCM_SIGNED_LE) means at
//! the [`BITS_PER_SAMPLE`](crate::audio_render::BITS_PER_SAMPLE) this
//! module asks for. A stereo frame is two samples, left first.
//!
//! Buffers are the caller's, as `&'static mut [u8]`, and ownership moves
//! the way it does everywhere else in this stack: a buffer handed to the
//! firmware belongs to it until it comes back, which
//! [`poll`](crate::audio_render::AudioRenderer::poll)
//! is what notices. Each must be 64-byte (cache-line) aligned, since the
//! firmware fetches them by DMA.
//!
//! ## What the display allows
//!
//! Which sample rates and channel counts actually work is a property of the
//! attached display and the firmware's negotiation with it, not of anything
//! chosen here — 48 kHz stereo is what every HDMI sink supports, and the
//! safe choice. The firmware also has to be in HDMI mode rather than DVI
//! mode for there to be an audio channel at all, which for a display that
//! doesn't announce itself as HDMI means `hdmi_drive=2` in `config.txt`.

use crate::mmal::{self, Component, Event, Mmal, Pool, PortAction, PortInfo};
use crate::timer::Timer;
use crate::vchiq::Vchiq;

/// The firmware component this drives.
const COMPONENT: &str = "ril.audio_render";

/// The parameter naming where the renderer sends its output, as a
/// NUL-terminated string on the component's control port. MMAL numbers its
/// parameters in per-domain groups of 65536, audio being the fourth, and
/// the destination is the first parameter in it.
const PARAMETER_AUDIO_DESTINATION: u32 = 3 << 16;

/// Bits in one sample of one channel, which this module fixes at 16 — the
/// width the renderer takes, and the width the rest of this crate's audio
/// paths speak.
pub const BITS_PER_SAMPLE: u32 = 16;

/// Bytes one sample of one channel occupies.
const BYTES_PER_SAMPLE: usize = BITS_PER_SAMPLE as usize / 8;

/// Buffers this module will hold. The ceiling exists only because the pool
/// is a fixed-size array; how much audio they hold between them, which is
/// the playback latency, is the caller's choice of buffer size.
pub const MAX_BUFFERS: usize = mmal::POOL_CAPACITY;

/// Where the firmware should send the audio.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Destination {
    /// Out of the HDMI connector, in the blanking intervals of the video
    /// signal already going to the display.
    Hdmi,
    /// Out of the board's own 3.5 mm jack — the analog output
    /// [`crate::pwm`]'s audio path reaches by driving the hardware
    /// directly.
    Local,
}

impl Destination {
    /// The name the firmware knows this destination by.
    fn as_str(self) -> &'static str {
        match self {
            Destination::Hdmi => "hdmi",
            Destination::Local => "local",
        }
    }
}

/// What the samples handed to the renderer are.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Format {
    /// Frames per second — 48000 for the rate every HDMI sink supports.
    pub sample_rate: u32,
    /// Channels per frame, interleaved: 1 for mono, 2 for stereo.
    pub channels: u32,
}

impl Format {
    /// Samples in one frame, which is one per channel.
    fn samples_per_frame(&self) -> usize {
        (self.channels as usize).max(1)
    }
}

/// Errors from [`AudioRenderer`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The MMAL layer or the transport under it failed. An
    /// [`mmal::Error::Status`] from [`AudioRenderer::new`] is the firmware
    /// refusing the destination or the format — a display that never
    /// negotiated an audio channel is the usual reason for the first.
    Mmal(mmal::Error),
    /// More buffers were added than [`MAX_BUFFERS`] allows.
    TooManyBuffers,
    /// No buffers were added before [`AudioRenderer::start`].
    NoBuffers,
    /// A buffer is smaller than the firmware's own minimum for the port.
    BufferTooSmall {
        /// Bytes the buffer has.
        provided: usize,
        /// Bytes it needs.
        required: usize,
    },
    /// A call was made out of order — feeding a renderer that hasn't been
    /// started, or starting one twice.
    WrongState,
    /// The renderer component reported an error event.
    RendererError,
}

impl From<mmal::Error> for Error {
    /// Wraps an MMAL failure as [`Error::Mmal`].
    fn from(error: mmal::Error) -> Self {
        Error::Mmal(error)
    }
}

/// The firmware's audio sink.
pub struct AudioRenderer {
    /// The MMAL client, which owns the transport.
    mmal: Mmal,
    /// The renderer component.
    component: Component,
    /// Its control port, which is where the destination is set.
    control: PortInfo,
    /// Its sample input port.
    input: PortInfo,
    /// Buffers waiting to carry samples.
    pool: Pool,
    /// What the samples are.
    format: Format,
    /// Buffers the firmware currently holds.
    in_flight: usize,
    /// Whether [`Self::start`] has run.
    started: bool,
}

impl AudioRenderer {
    /// Creates the renderer component, points it at `destination`, and
    /// describes the samples it will be fed.
    ///
    /// Takes the transport, which must already be
    /// [`connected`](Vchiq::connect); everything below this owns it from
    /// here on.
    ///
    /// The destination is set before anything else happens to the
    /// component, because it decides which piece of hardware the port is
    /// then configured against — moving audio somewhere else once it is
    /// playing is not something this supports.
    pub fn new(
        vchiq: Vchiq,
        destination: Destination,
        format: Format,
        timer: &Timer,
    ) -> Result<Self, Error> {
        let mut mmal = Mmal::new(vchiq, timer)?;
        let component = mmal.component_create(COMPONENT, timer)?;

        // Where the audio goes is a property of the component rather than
        // of any one of its ports, so it is set on the control port -- the
        // port MMAL gives every component for exactly that.
        let control = mmal.port_info_get(&component, mmal::PORT_TYPE_CONTROL, 0, timer)?;
        mmal.parameter_set_string(
            &control,
            PARAMETER_AUDIO_DESTINATION,
            destination.as_str(),
            timer,
        )?;

        let mut input = mmal.port_info_get(&component, mmal::PORT_TYPE_INPUT, 0, timer)?;
        input.es_type = mmal::ES_TYPE_AUDIO;
        input.encoding = mmal::ENCODING_PCM_SIGNED_LE;
        input.encoding_variant = 0;
        input.audio = mmal::AudioFormat {
            channels: format.channels,
            sample_rate: format.sample_rate,
            bits_per_sample: BITS_PER_SAMPLE,
            // Meaningless for linear PCM, where a frame's size follows
            // from the two fields above; the firmware only reads it for the
            // block-based encodings.
            block_align: 0,
        };

        Ok(Self {
            mmal,
            component,
            control,
            input,
            pool: Pool::new(),
            format,
            in_flight: 0,
            started: false,
        })
    }

    /// Adds a buffer for samples. Call before [`Self::start`].
    ///
    /// Its size is the granularity playback is fed at, and the buffers
    /// together are how far ahead of the audio the application can get, so
    /// a few tens of milliseconds each is a reasonable starting point. The
    /// floor is the firmware's own minimum for the port, which
    /// [`Self::start`] checks against and reports as
    /// [`Error::BufferTooSmall`] — 4096 bytes, about 21 ms of 48 kHz
    /// stereo, on a Pi 3.
    pub fn add_buffer(&mut self, buffer: &'static mut [u8]) -> Result<(), Error> {
        if self.started {
            return Err(Error::WrongState);
        }
        if !self.pool.add(buffer) {
            return Err(Error::TooManyBuffers);
        }
        Ok(())
    }

    /// Configures and enables the component, after which it is ready to be
    /// fed.
    pub fn start(&mut self, timer: &Timer) -> Result<(), Error> {
        if self.started {
            return Err(Error::WrongState);
        }
        if self.pool.total() == 0 {
            return Err(Error::NoBuffers);
        }

        let smallest = self.pool.smallest();
        if smallest < self.input.buffer_size_min as usize {
            return Err(Error::BufferTooSmall {
                provided: smallest,
                required: self.input.buffer_size_min as usize,
            });
        }
        self.input.buffer_num = self.pool.total().max(self.input.buffer_num_min as usize) as u32;
        self.input.buffer_size = smallest as u32;
        self.mmal.port_info_set(&mut self.input, timer)?;

        self.mmal.component_enable(&self.component, timer)?;
        self.mmal
            .port_action(&self.input, PortAction::Enable, timer)?;
        self.input.enabled = true;

        self.started = true;
        Ok(())
    }

    /// Hands the renderer as many of `samples` as fit in one free buffer,
    /// returning how many it took — zero when every buffer is still with
    /// the firmware, which is the renderer asking the caller to
    /// [`poll`](Self::poll) and try again.
    ///
    /// Samples are interleaved by channel, left first. What this takes is
    /// always a whole number of frames, so a caller feeding a stereo stream
    /// in arbitrarily sized pieces never has its channels swap places
    /// part-way through — but by the same token, a piece shorter than one
    /// frame is not taken at all.
    ///
    /// `pts` is the presentation timestamp the firmware is told, or
    /// [`mmal::TIME_UNKNOWN`] for audio that is simply played as it
    /// arrives.
    pub fn feed(&mut self, samples: &[i16], pts: i64, timer: &Timer) -> Result<usize, Error> {
        if !self.started {
            return Err(Error::WrongState);
        }
        let Some(buffer) = self.pool.take() else {
            return Ok(0);
        };
        let frame = self.format.samples_per_frame();
        let capacity = buffer.len() / BYTES_PER_SAMPLE;
        let taken = (samples.len().min(capacity) / frame) * frame;
        if taken == 0 {
            self.pool.put(buffer);
            return Ok(0);
        }
        let (words, _) = buffer.as_chunks_mut::<BYTES_PER_SAMPLE>();
        for (bytes, sample) in words.iter_mut().zip(&samples[..taken]) {
            *bytes = sample.to_le_bytes();
        }

        self.mmal
            .send_buffer(&self.input, buffer, taken * BYTES_PER_SAMPLE, 0, pts, timer)?;
        self.in_flight += 1;
        Ok(taken)
    }

    /// Moves the renderer forward, taking back the buffers the firmware has
    /// finished playing.
    ///
    /// This must be called regularly: it is what returns buffers to the
    /// pool, so a caller that stops polling runs out of them and
    /// [`feed`](Self::feed) stops taking anything.
    pub fn poll(&mut self) -> Result<(), Error> {
        while let Some(event) = self.mmal.poll()? {
            match event {
                Event::Buffer { buffer, .. } => {
                    self.in_flight = self.in_flight.saturating_sub(1);
                    self.pool.put(buffer);
                }
                Event::PortEvent { cmd, .. } => {
                    if cmd == mmal::EVENT_ERROR {
                        return Err(Error::RendererError);
                    }
                }
            }
        }
        Ok(())
    }

    /// How many buffers the firmware still holds — i.e. how much fed audio
    /// has not been played yet.
    ///
    /// Zero after the last [`feed`](Self::feed) means everything handed
    /// over has been consumed, which is what to wait for before dropping
    /// the renderer or halting.
    pub fn buffers_in_flight(&self) -> usize {
        self.in_flight
    }

    /// What the samples being fed are, as [`Self::new`] was told.
    pub fn format(&self) -> Format {
        self.format
    }

    /// The MMAL client underneath, for parameters and components this
    /// wrapper doesn't cover.
    pub fn mmal(&mut self) -> &mut Mmal {
        &mut self.mmal
    }

    /// The sample input port, as the firmware last described it — which
    /// after [`Self::start`] is what it actually settled on, not what it
    /// was asked for.
    pub fn input_port(&self) -> &PortInfo {
        &self.input
    }

    /// The component's control port, for the parameters this wrapper
    /// doesn't set.
    pub fn control_port(&self) -> &PortInfo {
        &self.control
    }
}
