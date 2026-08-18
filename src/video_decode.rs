//! Hardware H.264 video decode.
//!
//! Every board this crate targets has a dedicated video decode block, but
//! there is no register-level driver to write for it: on BCM2836/2837 the
//! codec is driven entirely by the closed VideoCore firmware, with no
//! public documentation and no open-source driver that reverses it. What
//! *is* reachable is the firmware's own interface to it — the
//! `ril.video_decode` MMAL component — so that is what this module drives,
//! over [`crate::mmal`] and [`crate::vchiq`].
//!
//! That also means this is not a stateless decoder that needs to be fed
//! parsed slices: the firmware owns the whole bitstream front end. Feed it
//! an H.264 Annex B byte stream in whatever chunks are convenient — start
//! codes and all, split anywhere — and whole frames come back.
//!
//! ## Using it
//!
//! ```ignore
//! let mut decoder = VideoDecoder::new(vchiq, &timer)?;
//! decoder.add_input_buffer(input_buffer)?;   // repeat for a few
//! decoder.add_output_buffer(output_buffer)?; // and a few of these
//! decoder.start(&timer)?;
//!
//! loop {
//!     // Push more of the stream whenever the decoder has room.
//!     position += decoder.feed(&stream[position..], 0, mmal::TIME_UNKNOWN, &timer)?;
//!     if let Some(frame) = decoder.poll(&timer)? {
//!         // frame.buffer[..frame.length] is I420, laid out to
//!         // frame.format.width x frame.format.height.
//!         decoder.recycle(frame, &timer)?;
//!     }
//! }
//! ```
//!
//! ## Frame geometry
//!
//! The decoder does not know the picture size until it has parsed the
//! stream's sequence header, so
//! [`format`](crate::video_decode::VideoDecoder::format) is `None` until
//! the first frame is close to ready — and it can change again
//! mid-stream. Both cases are handled internally, by reconfiguring the
//! output port; the visible effect is that
//! [`poll`](crate::video_decode::VideoDecoder::poll) starts (or resumes)
//! producing frames with a new
//! [`FrameFormat`](crate::video_decode::FrameFormat).
//!
//! Output is [`ENCODING_I420`](crate::mmal::ENCODING_I420): a
//! full-resolution plane of luma followed by half-resolution blue- and
//! red-difference chroma planes, all laid out to the *padded*
//! [`width`](crate::video_decode::FrameFormat::width) and
//! [`height`](crate::video_decode::FrameFormat::height), with the picture
//! itself occupying the top-left
//! [`crop_width`](crate::video_decode::FrameFormat::crop_width) by
//! [`crop_height`](crate::video_decode::FrameFormat::crop_height) of it.
//! Getting that on screen is a separate job, and not one this module
//! does: the mailbox framebuffer path
//! ([`crate::mailbox::Mailbox::allocate_framebuffer`]) takes RGB, so
//! something has to convert.
//!
//! ## Buffers
//!
//! The caller provides the buffers, as `&'static mut [u8]`, because there
//! is no allocator here and because a decoded frame is large enough that
//! where it lives is an application decision. Ownership moves: a buffer
//! handed over belongs to the VideoCore until it comes back as a
//! [`Frame`](crate::video_decode::Frame), and
//! [`recycle`](crate::video_decode::VideoDecoder::recycle) hands it over
//! again.
//!
//! Output buffers must be big enough for a decoded frame *before* the
//! decoder has said how big that is — for 4:2:0, `width * height * 3 / 2`
//! rounded up to the decoder's macroblock alignment (multiples of 32
//! horizontally and 16 vertically). A buffer too small for the format the
//! stream turns out to need surfaces as
//! [`Error::BufferTooSmall`](crate::video_decode::Error::BufferTooSmall)
//! rather
//! than a truncated frame.

use crate::mmal::{self, Component, Event, Mmal, PortAction, PortInfo};
use crate::timer::Timer;
use crate::vchiq::Vchiq;

/// The firmware component this drives.
const COMPONENT: &str = "ril.video_decode";

/// Buffers this module will hold for one port. Four in and four out is
/// enough to keep the hardware busy; the ceiling exists only because the
/// pools are fixed-size arrays.
pub const MAX_PORT_BUFFERS: usize = 6;

/// Errors from [`VideoDecoder`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The MMAL layer or the transport under it failed.
    Mmal(mmal::Error),
    /// More buffers were added to one port than [`MAX_PORT_BUFFERS`]
    /// allows.
    TooManyBuffers,
    /// No buffers were added to a port before [`VideoDecoder::start`].
    NoBuffers,
    /// A buffer is smaller than the port needs: for an input port, than
    /// the firmware's own minimum; for the output port, than a decoded
    /// frame of the format the stream turned out to have.
    BufferTooSmall {
        /// Bytes the buffer has.
        provided: usize,
        /// Bytes it needs.
        required: usize,
    },
    /// A call was made out of order — feeding a decoder that hasn't been
    /// started, or starting one twice.
    WrongState,
    /// The decoder component reported an error event.
    DecoderError,
}

impl From<mmal::Error> for Error {
    /// Wraps an MMAL failure as [`Error::Mmal`].
    fn from(error: mmal::Error) -> Self {
        Error::Mmal(error)
    }
}

/// The geometry of the frames the decoder is currently producing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameFormat {
    /// Row stride of the luma plane, in pixels — the *padded* width, which
    /// is what the planes are laid out to.
    pub width: u32,
    /// Rows in the luma plane — the padded height.
    pub height: u32,
    /// Width of the picture within that buffer.
    pub crop_width: u32,
    /// Height of the picture within that buffer.
    pub crop_height: u32,
    /// Frame rate numerator, zero when the stream doesn't say.
    pub frame_rate_num: i32,
    /// Frame rate denominator.
    pub frame_rate_den: i32,
    /// Bytes one decoded frame occupies.
    pub frame_size: usize,
}

impl FrameFormat {
    /// Byte offset of the blue-difference chroma plane within a frame — it
    /// follows the luma plane, which is [`Self::width`] × [`Self::height`]
    /// bytes.
    pub fn u_offset(&self) -> usize {
        (self.width * self.height) as usize
    }

    /// Byte offset of the red-difference chroma plane, which follows the
    /// blue-difference one at half the width and half the height.
    pub fn v_offset(&self) -> usize {
        self.u_offset() + ((self.width / 2) * (self.height / 2)) as usize
    }
}

/// A decoded frame, and with it ownership of the buffer holding it. Hand
/// it back with [`VideoDecoder::recycle`] once it has been used.
#[derive(Debug)]
pub struct Frame {
    /// The buffer. Its first [`Self::length`] bytes are the frame.
    pub buffer: &'static mut [u8],
    /// Bytes of [`Self::buffer`] the decoder wrote.
    pub length: usize,
    /// Presentation timestamp, or [`mmal::TIME_UNKNOWN`] for a stream fed
    /// without timestamps.
    pub pts: i64,
    /// `mmal::BUFFER_FLAG_*` bits the decoder set.
    pub flags: u32,
    /// The geometry this frame is laid out to.
    pub format: FrameFormat,
}

/// A pool of buffers waiting to be handed to the firmware.
struct Pool {
    /// The free buffers.
    buffers: [Option<&'static mut [u8]>; MAX_PORT_BUFFERS],
    /// How many were ever added, which is what the port is told to expect.
    total: usize,
}

impl Pool {
    /// An empty pool.
    const fn new() -> Self {
        Self {
            buffers: [const { None }; MAX_PORT_BUFFERS],
            total: 0,
        }
    }

    /// Adds a buffer, failing once the pool is full.
    fn add(&mut self, buffer: &'static mut [u8]) -> Result<(), Error> {
        let slot = self
            .buffers
            .iter_mut()
            .find(|slot| slot.is_none())
            .ok_or(Error::TooManyBuffers)?;
        *slot = Some(buffer);
        self.total += 1;
        Ok(())
    }

    /// Puts a buffer back after the firmware has returned it.
    fn put(&mut self, buffer: &'static mut [u8]) {
        if let Some(slot) = self.buffers.iter_mut().find(|slot| slot.is_none()) {
            *slot = Some(buffer);
        }
    }

    /// Takes a free buffer, if there is one.
    fn take(&mut self) -> Option<&'static mut [u8]> {
        self.buffers.iter_mut().find_map(|slot| slot.take())
    }

    /// The smallest buffer in the pool, which is the size every buffer can
    /// be relied on to have.
    fn smallest(&self) -> usize {
        self.buffers
            .iter()
            .flatten()
            .map(|buffer| buffer.len())
            .min()
            .unwrap_or(0)
    }
}

/// The hardware H.264 decoder.
pub struct VideoDecoder {
    /// The MMAL client, which owns the transport.
    mmal: Mmal,
    /// The decoder component.
    component: Component,
    /// Its compressed-data input port.
    input: PortInfo,
    /// Its decoded-frame output port.
    output: PortInfo,
    /// Buffers waiting to carry compressed data in.
    input_pool: Pool,
    /// Buffers waiting to carry frames out.
    output_pool: Pool,
    /// Whether [`Self::start`] has run.
    started: bool,
    /// Whether the firmware currently has the output port enabled. True
    /// from [`Self::start`] onwards, since the port is enabled there
    /// before its format is known — which is what makes the firmware
    /// report the format at all.
    output_enabled: bool,
    /// Output buffers currently held by the firmware.
    output_in_flight: usize,
    /// The format frames are coming out in, once known.
    format: Option<FrameFormat>,
    /// Whether the end of the stream has come back out.
    eos: bool,
}

impl VideoDecoder {
    /// Creates the decoder component and points its input port at H.264.
    ///
    /// Takes the transport, which must already be
    /// [`connected`](Vchiq::connect); everything below this owns it from
    /// here on.
    pub fn new(vchiq: Vchiq, timer: &Timer) -> Result<Self, Error> {
        let mut mmal = Mmal::new(vchiq, timer)?;
        let component = mmal.component_create(COMPONENT, timer)?;

        let mut input = mmal.port_info_get(&component, mmal::PORT_TYPE_INPUT, 0, timer)?;
        input.es_type = mmal::ES_TYPE_VIDEO;
        input.encoding = mmal::ENCODING_H264;
        input.encoding_variant = 0;
        // Geometry is left exactly as the component reported it: the
        // firmware parses the stream's own sequence header, so anything
        // said here would only be a hint, and a wrong one is worse than
        // none.

        let output = mmal.port_info_get(&component, mmal::PORT_TYPE_OUTPUT, 0, timer)?;

        Ok(Self {
            mmal,
            component,
            input,
            output,
            input_pool: Pool::new(),
            output_pool: Pool::new(),
            started: false,
            output_enabled: false,
            output_in_flight: 0,
            format: None,
            eos: false,
        })
    }

    /// Adds a buffer for compressed data. Call before [`Self::start`].
    ///
    /// Bigger buffers mean fewer round trips; 64KB or so is a reasonable
    /// starting point, and the firmware enforces its own minimum (usually
    /// 80KB for H.264) at [`Self::start`].
    pub fn add_input_buffer(&mut self, buffer: &'static mut [u8]) -> Result<(), Error> {
        if self.started {
            return Err(Error::WrongState);
        }
        self.input_pool.add(buffer)
    }

    /// Adds a buffer for decoded frames. Call before [`Self::start`].
    ///
    /// It must be large enough for a whole frame of whatever the stream
    /// turns out to contain — see this module's doc comment — since the
    /// decoder cannot say how large that is until it has started decoding.
    pub fn add_output_buffer(&mut self, buffer: &'static mut [u8]) -> Result<(), Error> {
        if self.started {
            return Err(Error::WrongState);
        }
        self.output_pool.add(buffer)
    }

    /// Configures and enables the component, after which the decoder is
    /// ready to be fed.
    ///
    /// The output port is enabled here but not yet given buffers: until
    /// the stream says what size its frames are, there is no format to
    /// configure it for. It is enabled all the same because that is what
    /// makes the firmware report the format when it finds out — see
    /// [`Self::poll`].
    pub fn start(&mut self, timer: &Timer) -> Result<(), Error> {
        if self.started {
            return Err(Error::WrongState);
        }
        if self.input_pool.total == 0 || self.output_pool.total == 0 {
            return Err(Error::NoBuffers);
        }

        let smallest = self.input_pool.smallest();
        if smallest < self.input.buffer_size_min as usize {
            return Err(Error::BufferTooSmall {
                provided: smallest,
                required: self.input.buffer_size_min as usize,
            });
        }
        self.input.buffer_num = self
            .input_pool
            .total
            .max(self.input.buffer_num_min as usize) as u32;
        self.input.buffer_size = smallest as u32;
        self.mmal.port_info_set(&mut self.input, timer)?;

        // Ask for plain planar YUV out. Without this the component may
        // choose its opaque internal format, which is only useful to
        // another firmware component.
        self.output.es_type = mmal::ES_TYPE_VIDEO;
        self.output.encoding = mmal::ENCODING_I420;
        self.output.encoding_variant = 0;
        self.output.buffer_num = self
            .output_pool
            .total
            .max(self.output.buffer_num_min as usize) as u32;
        self.mmal.port_info_set(&mut self.output, timer)?;

        self.mmal.component_enable(&self.component, timer)?;
        self.mmal
            .port_action(&self.output, PortAction::Enable, timer)?;
        self.output.enabled = true;
        self.output_enabled = true;
        self.mmal
            .port_action(&self.input, PortAction::Enable, timer)?;
        self.input.enabled = true;

        self.started = true;
        Ok(())
    }

    /// Hands the decoder as much of `data` as fits in one free input
    /// buffer, returning how many bytes it took — zero when every input
    /// buffer is still with the firmware, which is the decoder asking the
    /// caller to poll and try again.
    ///
    /// `data` is a raw H.264 Annex B byte stream and may be split
    /// anywhere; `flags` and `pts` are passed through to the firmware (see
    /// `mmal::BUFFER_FLAG_*` and [`mmal::TIME_UNKNOWN`]).
    pub fn feed(
        &mut self,
        data: &[u8],
        flags: u32,
        pts: i64,
        timer: &Timer,
    ) -> Result<usize, Error> {
        if !self.started {
            return Err(Error::WrongState);
        }
        if data.is_empty() {
            return Ok(0);
        }
        let Some(buffer) = self.input_pool.take() else {
            return Ok(0);
        };
        let length = data.len().min(buffer.len());
        buffer[..length].copy_from_slice(&data[..length]);
        self.mmal
            .send_buffer(&self.input, buffer, length, flags, pts, timer)?;
        Ok(length)
    }

    /// Tells the decoder the stream has ended, so that it flushes out the
    /// frames it is still holding.
    ///
    /// Sends an empty buffer with the end-of-stream flag. Keep polling
    /// afterwards until [`Self::end_of_stream`] reports the flag has come
    /// back out the other side.
    pub fn finish(&mut self, timer: &Timer) -> Result<(), Error> {
        if !self.started {
            return Err(Error::WrongState);
        }
        let Some(buffer) = self.input_pool.take() else {
            return Err(Error::WrongState);
        };
        self.mmal.send_buffer(
            &self.input,
            buffer,
            0,
            mmal::BUFFER_FLAG_EOS,
            mmal::TIME_UNKNOWN,
            timer,
        )?;
        Ok(())
    }

    /// Moves the decoder forward, returning a decoded frame when one is
    /// ready.
    ///
    /// This is also where the output port gets (re)configured: the first
    /// time the firmware reports the stream's real frame geometry, and
    /// again if it changes mid-stream, this disables the output port,
    /// reconfigures it, and hands its buffers back over — so a caller only
    /// sees [`Self::format`] change.
    pub fn poll(&mut self, timer: &Timer) -> Result<Option<Frame>, Error> {
        while let Some(event) = self.mmal.poll()? {
            match event {
                Event::Buffer {
                    port,
                    buffer,
                    length,
                    flags,
                    pts,
                } => {
                    if port == self.input.handle {
                        self.input_pool.put(buffer);
                        continue;
                    }
                    self.output_in_flight = self.output_in_flight.saturating_sub(1);
                    if flags & mmal::BUFFER_FLAG_EOS != 0 {
                        self.eos = true;
                    }
                    match (length, self.format) {
                        // A frame. Ownership goes to the caller until it
                        // recycles it.
                        (1.., Some(format)) => {
                            return Ok(Some(Frame {
                                buffer,
                                length,
                                pts,
                                flags,
                                format,
                            }))
                        }
                        // An empty buffer — the end of the stream, or one
                        // coming back from a port being reconfigured.
                        _ => {
                            self.output_pool.put(buffer);
                            self.refill_output(timer)?;
                        }
                    }
                }
                Event::PortEvent { cmd, .. } => match cmd {
                    mmal::EVENT_FORMAT_CHANGED => self.reconfigure_output(timer)?,
                    mmal::EVENT_EOS => self.eos = true,
                    mmal::EVENT_ERROR => return Err(Error::DecoderError),
                    _ => {}
                },
            }
        }
        Ok(None)
    }

    /// Hands a frame's buffer back to the decoder.
    pub fn recycle(&mut self, frame: Frame, timer: &Timer) -> Result<(), Error> {
        self.output_pool.put(frame.buffer);
        self.refill_output(timer)
    }

    /// The geometry frames are currently coming out in, or `None` before
    /// the decoder has parsed enough of the stream to know.
    pub fn format(&self) -> Option<FrameFormat> {
        self.format
    }

    /// Whether the end of the stream has come back out of the decoder —
    /// i.e. every frame from the data fed in has now been returned.
    pub fn end_of_stream(&self) -> bool {
        self.eos
    }

    /// The MMAL client underneath, for parameters and components this
    /// wrapper doesn't cover.
    pub fn mmal(&mut self) -> &mut Mmal {
        &mut self.mmal
    }

    /// The compressed-data input port, as the firmware last described it —
    /// which after [`Self::start`] is what it actually settled on, not
    /// what it was asked for.
    pub fn input_port(&self) -> &PortInfo {
        &self.input
    }

    /// The decoded-frame output port; see [`Self::input_port`]. Its format
    /// is only meaningful once [`Self::format`] is `Some`.
    pub fn output_port(&self) -> &PortInfo {
        &self.output
    }

    /// Reconfigures the output port for the format the firmware has just
    /// announced, and hands its buffers over.
    ///
    /// The port has to be disabled to change its format, which is also
    /// what gets any buffers the firmware is holding returned — so this
    /// waits for those before re-enabling, or they would come back as
    /// frames in a layout that no longer matches what the caller is told.
    /// That includes the *first* format change: [`Self::start`] leaves the
    /// port enabled (with no buffers and no known format), so there is
    /// always something to disable here.
    fn reconfigure_output(&mut self, timer: &Timer) -> Result<(), Error> {
        let Some(changed) = mmal::parse_format_changed(self.mmal.event_data()) else {
            return Ok(());
        };

        if self.output_enabled {
            self.mmal
                .port_action(&self.output, PortAction::Disable, timer)?;
            self.output_enabled = false;
            self.output.enabled = false;
            self.drain_output(timer)?;
        }

        // Disabling rereads the port, so start from what the firmware says
        // now and apply the event's format on top of it.
        self.output = self
            .mmal
            .port_info_get(&self.component, mmal::PORT_TYPE_OUTPUT, 0, timer)?;
        self.output.es_type = mmal::ES_TYPE_VIDEO;
        self.output.encoding = mmal::ENCODING_I420;
        self.output.encoding_variant = 0;
        self.output.video = changed.video;
        self.output.buffer_num = self.output_pool.total.max(changed.buffer_num_min as usize) as u32;
        self.output.buffer_size = changed.buffer_size_min.max(changed.buffer_size_recommended);

        let smallest = self.output_pool.smallest();
        if smallest < self.output.buffer_size as usize {
            return Err(Error::BufferTooSmall {
                provided: smallest,
                required: self.output.buffer_size as usize,
            });
        }
        // The firmware sizes buffers for its own preferred stride; there is
        // no point telling it they are larger than they are.
        self.output.buffer_size = smallest as u32;

        self.mmal.port_info_set(&mut self.output, timer)?;
        self.mmal
            .port_action(&self.output, PortAction::Enable, timer)?;
        self.output.enabled = true;

        self.format = Some(FrameFormat {
            width: changed.video.width,
            height: changed.video.height,
            crop_width: changed.video.crop_width.max(0) as u32,
            crop_height: changed.video.crop_height.max(0) as u32,
            frame_rate_num: changed.video.frame_rate_num,
            frame_rate_den: changed.video.frame_rate_den,
            frame_size: (changed.video.width * changed.video.height * 3 / 2) as usize,
        });
        self.output_enabled = true;
        self.refill_output(timer)
    }

    /// Waits for every output buffer the firmware still holds to come
    /// back, discarding whatever is in them — called while the port is
    /// being reconfigured, when their contents are stale by definition.
    fn drain_output(&mut self, timer: &Timer) -> Result<(), Error> {
        let deadline = timer.now_micros() + DRAIN_TIMEOUT_US;
        while self.output_in_flight > 0 {
            if let Some(Event::Buffer { port, buffer, .. }) = self.mmal.poll()? {
                if port == self.input.handle {
                    self.input_pool.put(buffer);
                } else {
                    self.output_in_flight = self.output_in_flight.saturating_sub(1);
                    self.output_pool.put(buffer);
                }
            }
            if timer.now_micros() > deadline {
                // Whatever the firmware still has, it isn't giving back.
                // Carrying on with fewer buffers beats hanging here.
                self.output_in_flight = 0;
            }
        }
        Ok(())
    }

    /// Hands every free output buffer to the firmware, so it always has
    /// somewhere to decode into.
    fn refill_output(&mut self, timer: &Timer) -> Result<(), Error> {
        // Nothing to hand over until the port has a real format: the
        // buffers it is enabled with before then are the firmware's own
        // business, and giving it ours would size them for a frame
        // geometry neither side knows yet.
        if !self.output_enabled || self.format.is_none() {
            return Ok(());
        }
        while let Some(buffer) = self.output_pool.take() {
            self.mmal
                .send_buffer(&self.output, buffer, 0, 0, mmal::TIME_UNKNOWN, timer)?;
            self.output_in_flight += 1;
        }
        Ok(())
    }
}

/// How long [`VideoDecoder::drain_output`] waits for the firmware to
/// return the buffers it holds, in microseconds.
const DRAIN_TIMEOUT_US: u64 = 2_000_000;
