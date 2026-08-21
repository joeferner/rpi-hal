//! MMAL — the VideoCore firmware's multimedia framework, as a client.
//!
//! MMAL is how the closed firmware exposes the hardware blocks the ARM
//! cannot program directly: the camera ISP, the image encoders, the
//! resizer, and the H.264 video decoder this crate uses it for
//! ([`crate::video_decode`]). Its model is a graph of *components*, each
//! with numbered input, output and control *ports*; a port has an
//! elementary-stream format (an encoding four-character code plus, for
//! video, dimensions and cropping), and data moves through it as
//! *buffers* handed back and forth.
//!
//! Everything here is a message exchange over [`crate::vchiq`]'s `mmal`
//! service. Requests carry a context word the firmware echoes back, which
//! is what pairs a reply with its request and a returned buffer with the
//! memory it was sent from.
//!
//! ## Buffer flow
//!
//! Buffers move by *handing over ownership*, in both directions, and the
//! API mirrors that: [`Mmal::send_buffer`](crate::mmal::Mmal::send_buffer)
//! takes a `&'static mut [u8]`, and the same slice comes back out of
//! [`Mmal::poll`](crate::mmal::Mmal::poll) as
//! [`Event::Buffer`](crate::mmal::Event::Buffer) once the firmware is
//! finished with it. Between those
//! two points the memory belongs to the VideoCore and this core must not
//! touch it — which is exactly what moving the slice into the driver
//! enforces.
//!
//! What happens in between depends on direction. A buffer sent to an
//! *input* port carries data, so the message announcing it is followed by
//! a bulk transfer of the payload out of ARM memory, and the buffer comes
//! back empty once consumed. A buffer sent to an *output* port is empty,
//! and comes back with the firmware announcing a length, followed by a
//! bulk transfer *into* it — so [`Event::Buffer`](crate::mmal::Event::Buffer)
//! for an output port only appears once the data has actually landed and
//! been invalidated out of this core's cache.
//!
//! Ports also raise asynchronous
//! [`Event::PortEvent`](crate::mmal::Event::PortEvent)s — a decoder
//! discovering the real frame size mid-stream is the important one; see
//! [`EVENT_FORMAT_CHANGED`](crate::mmal::EVENT_FORMAT_CHANGED).
//!
//! ## Scope
//!
//! Enough of the protocol to drive a codec component: create/destroy,
//! enable/disable, port info get and set, port actions, parameters, and
//! buffer exchange. Zero-copy buffers (which need the firmware's shared
//! memory allocator, a second service this crate doesn't implement), port
//! connections, and the statistics messages are not here — none are
//! needed to decode.

use crate::timer::Timer;
use crate::vchiq::{self, ServiceId, Vchiq};

/// Largest MMAL message, in bytes — the protocol's own limit, and the
/// size of this driver's single message buffer.
const MSG_MAX_SIZE: usize = 512;

/// Bytes in an MMAL message header, before any per-type payload.
const HEADER_SIZE: usize = 24;

/// Magic word every message carries: `'m'`, `'m'`, `'a'`, `'l'`, in the
/// little-endian order MMAL's own `MMAL_FOURCC` builds.
const MAGIC: u32 = fourcc(*b"mmal");

/// Service version this client declares when opening the `mmal` service,
/// and the oldest firmware-side version it will accept — the pair Linux's
/// own client uses, which is what the firmware has been kept compatible
/// with.
const SERVICE_VERSION: u16 = 15;
/// See [`SERVICE_VERSION`].
const SERVICE_VERSION_MIN: u16 = 10;

// Message types. Only the ones this client sends or recognizes are named;
// the numbering is the firmware's, so the gaps are real.
/// Create a component by name.
const MSG_COMPONENT_CREATE: u32 = 4;
/// Destroy a component.
const MSG_COMPONENT_DESTROY: u32 = 5;
/// Enable a component.
const MSG_COMPONENT_ENABLE: u32 = 6;
/// Disable a component.
const MSG_COMPONENT_DISABLE: u32 = 7;
/// Read a port's configuration and format.
const MSG_PORT_INFO_GET: u32 = 8;
/// Write a port's configuration and format.
const MSG_PORT_INFO_SET: u32 = 9;
/// Enable, disable or flush a port.
const MSG_PORT_ACTION: u32 = 10;
/// Hand a buffer to a port.
const MSG_BUFFER_FROM_HOST: u32 = 11;
/// A buffer coming back from a port.
const MSG_BUFFER_TO_HOST: u32 = 12;
/// Write a port parameter.
const MSG_PORT_PARAMETER_SET: u32 = 14;
/// Read a port parameter.
const MSG_PORT_PARAMETER_GET: u32 = 15;
/// An asynchronous port event.
const MSG_EVENT_TO_HOST: u32 = 16;

/// Longest component name the create message carries.
const NAME_MAX: usize = 128;

/// Bytes of codec-specific extra data carried alongside a format.
const EXTRADATA_MAX: usize = 128;

/// Bytes of a buffer's payload that can travel inside the message itself
/// rather than by bulk transfer.
const SHORT_DATA_MAX: usize = 128;

/// Longest string [`Mmal::parameter_set_string`] can carry, including the
/// terminating NUL. The parameters shaped this way name a destination, a
/// source or a URI, so this is generous rather than a real constraint.
const PARAMETER_STRING_MAX: usize = 128;

/// Bytes of event payload carried inline in an event message.
const EVENT_DATA_MAX: usize = 256;

/// Buffers this driver can have handed to the firmware at once, across
/// every port. Two ports of a decoder with a handful of buffers each fit
/// comfortably; the cost of the ceiling is one small table entry apiece.
pub const MAX_BUFFERS: usize = 16;

// Port types, as `port_type` fields carry them.
/// A component's control port — parameters and events, no data.
pub const PORT_TYPE_CONTROL: u32 = 1;
/// A port data is fed into.
pub const PORT_TYPE_INPUT: u32 = 2;
/// A port data comes out of.
pub const PORT_TYPE_OUTPUT: u32 = 3;
/// A clock port (unused here).
pub const PORT_TYPE_CLOCK: u32 = 4;

// Elementary stream types, as a format's `es_type` carries them.
/// Control stream.
pub const ES_TYPE_CONTROL: u32 = 1;
/// Audio stream.
pub const ES_TYPE_AUDIO: u32 = 2;
/// Video stream.
pub const ES_TYPE_VIDEO: u32 = 3;
/// Sub-picture stream.
pub const ES_TYPE_SUBPICTURE: u32 = 4;

/// H.264 elementary stream, as an encoding four-character code.
pub const ENCODING_H264: u32 = fourcc(*b"H264");
/// Planar YUV 4:2:0 (a full-size Y plane then half-size U and V planes) —
/// the decoder's output encoding.
pub const ENCODING_I420: u32 = fourcc(*b"I420");
/// Semi-planar YUV 4:2:0 (a Y plane then one interleaved UV plane).
pub const ENCODING_NV12: u32 = fourcc(*b"NV12");
/// Signed linear PCM, little-endian, channels interleaved sample by
/// sample — the encoding the firmware's audio renderer takes
/// ([`crate::audio_render`]). How wide one sample is comes from the
/// format's [`AudioFormat::bits_per_sample`], not from the encoding.
pub const ENCODING_PCM_SIGNED_LE: u32 = fourcc(*b"pcms");

// Buffer header flags, as [`Event::Buffer`] reports them and
// [`Mmal::send_buffer`] accepts them.
/// This buffer is the end of the stream.
pub const BUFFER_FLAG_EOS: u32 = 1 << 0;
/// This buffer starts a frame.
pub const BUFFER_FLAG_FRAME_START: u32 = 1 << 1;
/// This buffer ends a frame.
pub const BUFFER_FLAG_FRAME_END: u32 = 1 << 2;
/// This buffer holds one or more whole frames.
pub const BUFFER_FLAG_FRAME: u32 = BUFFER_FLAG_FRAME_START | BUFFER_FLAG_FRAME_END;
/// This buffer is a keyframe — decodable on its own.
pub const BUFFER_FLAG_KEYFRAME: u32 = 1 << 3;
/// The stream is discontinuous here (after a seek, say).
pub const BUFFER_FLAG_DISCONTINUITY: u32 = 1 << 4;
/// This buffer is codec configuration data rather than stream data.
pub const BUFFER_FLAG_CONFIG: u32 = 1 << 5;
/// This buffer's contents are known to be damaged.
pub const BUFFER_FLAG_CORRUPTED: u32 = 1 << 9;
/// The firmware could not transfer this buffer's payload.
pub const BUFFER_FLAG_TRANSMISSION_FAILED: u32 = 1 << 10;

/// Timestamp value meaning "not known" — what to pass to
/// [`Mmal::send_buffer`] for data with no meaningful presentation time,
/// and what comes back on a buffer the firmware couldn't timestamp.
pub const TIME_UNKNOWN: i64 = i64::MIN;

/// A port's format changed: the payload of the [`Event::PortEvent`] is a
/// [`FormatChanged`]. A video decoder raises this on its output port once
/// it has parsed enough of the stream to know the real frame geometry —
/// which is why an output port cannot be usefully configured until it has
/// arrived.
pub const EVENT_FORMAT_CHANGED: u32 = fourcc(*b"EFCH");
/// The stream ended.
pub const EVENT_EOS: u32 = fourcc(*b"EEOS");
/// The component reported an error; the payload is a status word.
pub const EVENT_ERROR: u32 = fourcc(*b"ERRO");
/// A parameter changed.
pub const EVENT_PARAMETER_CHANGED: u32 = fourcc(*b"EPCH");

/// What a port action message asks a port to do.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum PortAction {
    /// Start accepting buffers.
    Enable = 1,
    /// Stop, returning every buffer the firmware still holds.
    Disable = 2,
    /// Discard buffered data without disabling the port.
    Flush = 3,
}

/// Errors from [`Mmal`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The transport underneath failed.
    Vchiq(vchiq::Error),
    /// The firmware answered with a nonzero MMAL status: 1 out of memory,
    /// 3 invalid argument, 4 not implemented, 5 no such component, and so
    /// on through MMAL's own list. `4` from
    /// [`Mmal::component_create`] means the firmware has no component of
    /// that name — a build of the firmware without that codec, or a
    /// misspelled name.
    Status(u32),
    /// The firmware answered a request with a reply to something else.
    UnexpectedReply,
    /// No reply arrived in time.
    Timeout,
    /// More buffers are outstanding than [`MAX_BUFFERS`] allows.
    TooManyBuffers,
    /// A returned buffer named a context this driver has no record of
    /// handing out.
    UnknownBuffer,
    /// A buffer was passed to [`Mmal::send_buffer`] with more data in it
    /// than it has room for, or an argument longer than the message that
    /// has to carry it (a component name, or a parameter value).
    TooLarge,
}

impl From<vchiq::Error> for Error {
    /// Wraps a transport failure as [`Error::Vchiq`].
    fn from(error: vchiq::Error) -> Self {
        Error::Vchiq(error)
    }
}

/// A component created on the VideoCore by [`Mmal::component_create`].
#[derive(Clone, Copy, Debug)]
pub struct Component {
    /// The firmware's handle for it, named in every later request.
    pub handle: u32,
    /// The index this client knows it by, echoed back in the events the
    /// component raises.
    pub client_handle: u32,
    /// How many input ports it has.
    pub inputs: u32,
    /// How many output ports it has.
    pub outputs: u32,
    /// How many clock ports it has.
    pub clocks: u32,
}

/// The video-specific half of an elementary stream format.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct VideoFormat {
    /// Buffer width in pixels, which for a decoder's output is the
    /// *padded* width — frames are laid out to this, not to
    /// [`Self::crop_width`].
    pub width: u32,
    /// Buffer height in pixel rows, likewise padded.
    pub height: u32,
    /// Left edge of the visible region within the buffer.
    pub crop_x: i32,
    /// Top edge of the visible region within the buffer.
    pub crop_y: i32,
    /// Width of the visible region — the picture's real width, which is
    /// what to display and is usually smaller than [`Self::width`].
    pub crop_width: i32,
    /// Height of the visible region.
    pub crop_height: i32,
    /// Frame rate numerator (zero when unknown).
    pub frame_rate_num: i32,
    /// Frame rate denominator.
    pub frame_rate_den: i32,
    /// Pixel aspect ratio numerator.
    pub par_num: i32,
    /// Pixel aspect ratio denominator.
    pub par_den: i32,
    /// Color space four-character code, or zero when unspecified.
    pub color_space: u32,
}

/// The audio-specific half of an elementary stream format.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AudioFormat {
    /// Channels per frame, interleaved sample by sample — 1 for mono, 2
    /// for stereo.
    pub channels: u32,
    /// Frames per second.
    pub sample_rate: u32,
    /// Bits in one sample of one channel.
    pub bits_per_sample: u32,
    /// Bytes in one block of the encoding, for the block-based encodings
    /// that have one. Zero for linear PCM, where a frame's size follows
    /// from the two fields above.
    pub block_align: u32,
}

/// A port's configuration and format, as
/// [`Mmal::port_info_get`] reads it and [`Mmal::port_info_set`] writes it
/// back.
///
/// The firmware fills in every field; only the buffer count and size, the
/// format, and the video geometry are writable, and everything else is
/// carried back unchanged. So the way to configure a port is to read this,
/// change what needs changing, and write it — never to build one from
/// scratch.
#[derive(Clone, Copy, Debug)]
pub struct PortInfo {
    /// The component this port belongs to.
    pub component: u32,
    /// The firmware's handle for the port, named in port actions and
    /// buffer messages.
    pub handle: u32,
    /// [`PORT_TYPE_INPUT`], [`PORT_TYPE_OUTPUT`], and so on.
    pub port_type: u32,
    /// Index of the port within its type.
    pub index: u32,
    /// Whether the firmware currently has it enabled.
    pub enabled: bool,
    /// Fewest buffers the port will work with (read-only).
    pub buffer_num_min: u32,
    /// Smallest buffer the port will accept, in bytes (read-only).
    pub buffer_size_min: u32,
    /// Required buffer alignment, zero for none (read-only).
    pub buffer_alignment_min: u32,
    /// Buffer count the component suggests for good performance
    /// (read-only).
    pub buffer_num_recommended: u32,
    /// Buffer size the component suggests (read-only).
    pub buffer_size_recommended: u32,
    /// Buffer count this client will actually use — writable.
    pub buffer_num: u32,
    /// Buffer size this client will actually use — writable.
    pub buffer_size: u32,
    /// Elementary stream type: [`ES_TYPE_VIDEO`] and friends — writable.
    pub es_type: u32,
    /// Encoding four-character code, e.g. [`ENCODING_H264`] — writable.
    pub encoding: u32,
    /// Encoding variant, zero for the default — writable.
    pub encoding_variant: u32,
    /// Stream bit rate in bits per second, zero when unknown — writable.
    pub bitrate: u32,
    /// Format flags — writable.
    pub flags: u32,
    /// Video geometry, meaningful when [`Self::es_type`] is
    /// [`ES_TYPE_VIDEO`] — writable.
    ///
    /// This and [`Self::audio`] are two readings of the same bytes: the
    /// message carries one type-specific union, not both halves, so
    /// [`Mmal::port_info_set`] writes whichever [`Self::es_type`] names
    /// and [`Mmal::port_info_get`] fills in both from the same union —
    /// only the one matching the type means anything.
    pub video: VideoFormat,
    /// Sample rate and channel layout, meaningful when [`Self::es_type`]
    /// is [`ES_TYPE_AUDIO`] — writable. Shares the message's union with
    /// [`Self::video`]; see there.
    pub audio: AudioFormat,
}

/// The payload of an [`EVENT_FORMAT_CHANGED`] event, as
/// [`parse_format_changed`] decodes it.
#[derive(Clone, Copy, Debug)]
pub struct FormatChanged {
    /// Smallest buffer the port will now accept.
    pub buffer_size_min: u32,
    /// Fewest buffers it will now work with.
    pub buffer_num_min: u32,
    /// Buffer size it now recommends.
    pub buffer_size_recommended: u32,
    /// Buffer count it now recommends.
    pub buffer_num_recommended: u32,
    /// The new elementary stream type.
    pub es_type: u32,
    /// The new encoding.
    pub encoding: u32,
    /// The new video geometry.
    pub video: VideoFormat,
}

/// Something that arrived from the VideoCore, returned by [`Mmal::poll`].
#[derive(Debug)]
pub enum Event {
    /// A buffer handed over with [`Mmal::send_buffer`] has come back, and
    /// with it ownership of the memory.
    Buffer {
        /// The port handle it was sent to.
        port: u32,
        /// The memory, back in this core's hands. For an output port its
        /// first `length` bytes are the data the firmware produced.
        buffer: &'static mut [u8],
        /// How many bytes of it are meaningful.
        length: usize,
        /// `BUFFER_FLAG_*` bits.
        flags: u32,
        /// Presentation timestamp, or [`TIME_UNKNOWN`].
        pts: i64,
    },
    /// A port raised an event. The payload is in [`Mmal::event_data`]
    /// until the next port event arrives.
    PortEvent {
        /// Which kind of port raised it.
        port_type: u32,
        /// Index of the port within its type.
        port_index: u32,
        /// What happened: [`EVENT_FORMAT_CHANGED`] and friends.
        cmd: u32,
        /// Length of the payload.
        length: usize,
    },
}

/// Running counts of what the firmware has done with the buffers handed
/// to it, from [`Mmal::stats`] — the MMAL-level counterpart to
/// [`vchiq::Stats`], and the same purpose: a protocol that stalls says
/// nothing about why, so the counts are what distinguish "the firmware
/// never answered" from "the firmware refused every buffer".
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// Buffers handed to the firmware.
    pub buffers_sent: u32,
    /// Buffers it has given back.
    pub buffers_returned: u32,
    /// Of those, ones it returned carrying an error status rather than
    /// data. Anything but zero means the component is rejecting what it
    /// is being given.
    pub buffers_failed: u32,
    /// The status on the most recent of those; see [`Error::Status`] for
    /// what the values mean.
    pub last_status: u32,
    /// The `BUFFER_FLAG_*` bits on the most recent returned buffer.
    /// [`BUFFER_FLAG_TRANSMISSION_FAILED`] here says the firmware wanted
    /// the payload and couldn't get it, which is a different fault from
    /// its never having asked.
    pub last_returned_flags: u32,
    /// The byte count on the most recent returned buffer. Zero on an
    /// input buffer means the firmware consumed it.
    pub last_returned_length: u32,
    /// Returned buffers whose data had to be fetched by bulk transfer.
    pub bulk_receives: u32,
    /// Returned buffers whose data came inline in the message.
    pub inline_receives: u32,
    /// Port events received.
    pub events: u32,
    /// The `cmd` of the most recent one.
    pub last_event: u32,
}

/// Where a registered buffer currently is.
#[derive(Clone, Copy, PartialEq, Eq)]
enum BufferState {
    /// The firmware has it.
    WithFirmware,
    /// The firmware has announced its return and a bulk transfer is
    /// filling it.
    Receiving,
    /// It is complete and waiting to be handed back to the caller.
    Returned,
}

/// One buffer this driver has handed to the firmware.
#[derive(Clone, Copy)]
struct BufferSlot {
    /// Address of the caller's memory.
    address: u32,
    /// Its total size.
    size: usize,
    /// The port it was sent to.
    port: u32,
    /// Where it is now.
    state: BufferState,
    /// Meaningful bytes, once the firmware has said.
    length: usize,
    /// `BUFFER_FLAG_*` bits from the firmware.
    flags: u32,
    /// Presentation timestamp from the firmware.
    pts: i64,
}

/// A client of the VideoCore's MMAL service.
pub struct Mmal {
    /// The transport, owned here: the `mmal` service is the only thing
    /// this crate opens on it.
    vchiq: Vchiq,
    /// The open service.
    service: ServiceId,
    /// Where outgoing messages are built. Separate from the receive
    /// buffer because a message can arrive while one is being sent —
    /// [`Self::send`] retries against back-pressure, and polling for the
    /// room to retry is exactly when that happens.
    tx: [u8; MSG_MAX_SIZE],
    /// The last message received, held for [`Self::request`]'s reply.
    rx: [u8; MSG_MAX_SIZE],
    /// Length of the message in [`Self::rx`].
    rx_len: usize,
    /// The last port event received, kept apart from [`Self::rx`] because
    /// an event can arrive in the middle of a request — the reply that
    /// ends the wait would otherwise overwrite the event's payload before
    /// anything had a chance to read it.
    event: [u8; MSG_MAX_SIZE],
    /// Context word of the request currently awaiting a reply, or zero.
    pending_context: u32,
    /// Set when a reply for [`Self::pending_context`] has landed in
    /// [`Self::rx`].
    reply_ready: bool,
    /// Set when a port event has landed in [`Self::event`] and has not
    /// been reported yet.
    pending_event: bool,
    /// Counter behind the context words handed to the firmware.
    next_context: u32,
    /// Buffers currently owned by the firmware, or finished and waiting to
    /// be handed back.
    buffers: [Option<BufferSlot>; MAX_BUFFERS],
    /// Indices of finished buffers, oldest first — the firmware returns
    /// them in order and so must this driver.
    returned: [usize; MAX_BUFFERS],
    /// How many entries of [`Self::returned`] are in use.
    returned_len: usize,
    /// What the firmware has done with the buffers so far; see [`Stats`].
    stats: Stats,
}

impl Mmal {
    /// Opens the firmware's `mmal` service on an already-connected
    /// transport.
    ///
    /// Takes ownership of `vchiq`: the service is the only client this
    /// crate has for it, and everything MMAL does needs the transport
    /// polled in step with its own state machine.
    pub fn new(mut vchiq: Vchiq, timer: &Timer) -> Result<Self, Error> {
        let service = vchiq.open_service(*b"mmal", SERVICE_VERSION, SERVICE_VERSION_MIN, timer)?;
        Ok(Self {
            vchiq,
            service,
            tx: [0; MSG_MAX_SIZE],
            rx: [0; MSG_MAX_SIZE],
            rx_len: 0,
            event: [0; MSG_MAX_SIZE],
            pending_context: 0,
            reply_ready: false,
            pending_event: false,
            next_context: 1,
            buffers: [None; MAX_BUFFERS],
            returned: [0; MAX_BUFFERS],
            returned_len: 0,
            stats: Stats::default(),
        })
    }

    /// Creates a component by name — `"ril.video_decode"` for the
    /// hardware video decoder, `"ril.video_encode"` for the encoder,
    /// `"ril.camera"` for the camera.
    ///
    /// The component starts disabled and with its ports in whatever
    /// default format it defines; configure them with
    /// [`Self::port_info_get`]/[`Self::port_info_set`] before
    /// [`Self::component_enable`].
    pub fn component_create(&mut self, name: &str, timer: &Timer) -> Result<Component, Error> {
        if name.len() >= NAME_MAX {
            return Err(Error::TooLarge);
        }
        // The client-side handle is echoed back on every event the
        // component raises. Nothing here multiplexes components, so a
        // fixed value is enough, and zero is what Linux's own first
        // component gets.
        let client_handle = 0;

        self.begin(MSG_COMPONENT_CREATE);
        self.put_u32(HEADER_SIZE, client_handle);
        self.tx[HEADER_SIZE + 4..HEADER_SIZE + 4 + NAME_MAX].fill(0);
        self.tx[HEADER_SIZE + 4..HEADER_SIZE + 4 + name.len()].copy_from_slice(name.as_bytes());
        self.put_u32(HEADER_SIZE + 4 + NAME_MAX, 0);
        let reply = self.request(HEADER_SIZE + 8 + NAME_MAX, MSG_COMPONENT_CREATE, timer)?;

        status(get_u32(reply, HEADER_SIZE))?;
        Ok(Component {
            handle: get_u32(reply, HEADER_SIZE + 4),
            client_handle,
            inputs: get_u32(reply, HEADER_SIZE + 8),
            outputs: get_u32(reply, HEADER_SIZE + 12),
            clocks: get_u32(reply, HEADER_SIZE + 16),
        })
    }

    /// Destroys a component, releasing the firmware-side resources it
    /// holds. Disable it first.
    pub fn component_destroy(&mut self, component: &Component, timer: &Timer) -> Result<(), Error> {
        self.component_request(MSG_COMPONENT_DESTROY, component, timer)
    }

    /// Enables a component, after which its ports can be enabled.
    pub fn component_enable(&mut self, component: &Component, timer: &Timer) -> Result<(), Error> {
        self.component_request(MSG_COMPONENT_ENABLE, component, timer)
    }

    /// Disables a component.
    pub fn component_disable(&mut self, component: &Component, timer: &Timer) -> Result<(), Error> {
        self.component_request(MSG_COMPONENT_DISABLE, component, timer)
    }

    /// Reads a port's current configuration and format.
    ///
    /// `port_type` is [`PORT_TYPE_INPUT`], [`PORT_TYPE_OUTPUT`] or
    /// [`PORT_TYPE_CONTROL`], and `index` numbers the port within that
    /// type. This is also how a port's handle is learned — the handle in
    /// the returned [`PortInfo`] is what every later call names it by.
    pub fn port_info_get(
        &mut self,
        component: &Component,
        port_type: u32,
        index: u32,
        timer: &Timer,
    ) -> Result<PortInfo, Error> {
        self.port_info_get_by_index(component.handle, port_type, index, timer)
    }

    /// [`Self::port_info_get`] against a bare component handle, for the
    /// read-back in [`Self::port_info_set`] where there is no
    /// [`Component`] in hand.
    fn port_info_get_by_index(
        &mut self,
        component: u32,
        port_type: u32,
        index: u32,
        timer: &Timer,
    ) -> Result<PortInfo, Error> {
        self.begin(MSG_PORT_INFO_GET);
        self.put_u32(HEADER_SIZE, component);
        self.put_u32(HEADER_SIZE + 4, port_type);
        self.put_u32(HEADER_SIZE + 8, index);
        let reply = self.request(HEADER_SIZE + 12, MSG_PORT_INFO_GET, timer)?;

        status(get_u32(reply, HEADER_SIZE))?;
        let port = HEADER_SIZE + 24;
        let format = port + PORT_FIELDS_SIZE;
        let es = format + FORMAT_FIELDS_SIZE;
        Ok(PortInfo {
            component,
            handle: get_u32(reply, HEADER_SIZE + 20),
            port_type: get_u32(reply, HEADER_SIZE + 8),
            index: get_u32(reply, HEADER_SIZE + 12),
            enabled: get_u32(reply, port + PORT_IS_ENABLED) != 0,
            buffer_num_min: get_u32(reply, port + PORT_BUFFER_NUM_MIN),
            buffer_size_min: get_u32(reply, port + PORT_BUFFER_SIZE_MIN),
            buffer_alignment_min: get_u32(reply, port + PORT_BUFFER_ALIGNMENT_MIN),
            buffer_num_recommended: get_u32(reply, port + PORT_BUFFER_NUM_RECOMMENDED),
            buffer_size_recommended: get_u32(reply, port + PORT_BUFFER_SIZE_RECOMMENDED),
            buffer_num: get_u32(reply, port + PORT_BUFFER_NUM),
            buffer_size: get_u32(reply, port + PORT_BUFFER_SIZE),
            es_type: get_u32(reply, format),
            encoding: get_u32(reply, format + 4),
            encoding_variant: get_u32(reply, format + 8),
            bitrate: get_u32(reply, format + 16),
            flags: get_u32(reply, format + 20),
            video: read_video_format(reply, es),
            audio: read_audio_format(reply, es),
        })
    }

    /// Writes a port's configuration and format back to the firmware, then
    /// reads it back into `port`.
    ///
    /// The read-back is not a courtesy: a component is free to clamp what
    /// it was asked for — a buffer count below its minimum, a size it
    /// rounds up to its own stride — and everything afterwards (enabling
    /// the port, sizing the buffers handed to it) has to work from what it
    /// settled on rather than what it was asked for. So `port` comes back
    /// describing the port as it now is.
    ///
    /// The port must be disabled: a decoder refuses a format change on a
    /// running port. Pass a [`PortInfo`] that came from
    /// [`Self::port_info_get`] with the writable fields adjusted — see
    /// that type's doc comment.
    pub fn port_info_set(&mut self, port: &mut PortInfo, timer: &Timer) -> Result<(), Error> {
        self.begin(MSG_PORT_INFO_SET);
        self.put_u32(HEADER_SIZE, port.component);
        self.put_u32(HEADER_SIZE + 4, port.port_type);
        self.put_u32(HEADER_SIZE + 8, port.index);

        let port_offset = HEADER_SIZE + 12;
        let format = port_offset + PORT_FIELDS_SIZE;
        let es = format + FORMAT_FIELDS_SIZE;
        self.tx[port_offset..es + ES_FIELDS_SIZE + EXTRADATA_MAX].fill(0);
        self.write_port_fields(port_offset, port);

        self.put_u32(format, port.es_type);
        self.put_u32(format + 4, port.encoding);
        self.put_u32(format + 8, port.encoding_variant);
        self.put_u32(format + 16, port.bitrate);
        self.put_u32(format + 20, port.flags);
        // One union, so only one of the two halves can go out; the stream
        // type is what says which of them the firmware will read.
        match port.es_type {
            ES_TYPE_AUDIO => self.write_audio_format(es, &port.audio),
            _ => self.write_video_format(es, &port.video),
        }

        let reply = self.request(
            es + ES_FIELDS_SIZE + EXTRADATA_MAX,
            MSG_PORT_INFO_SET,
            timer,
        )?;
        status(get_u32(reply, HEADER_SIZE))?;

        // What the component settled on, which is not necessarily what it
        // was asked for.
        let component = port.component;
        let port_type = port.port_type;
        let index = port.index;
        *port = self.port_info_get_by_index(component, port_type, index, timer)?;
        Ok(())
    }

    /// Enables, disables or flushes a port.
    ///
    /// Disabling returns every buffer the firmware still holds for that
    /// port first, so a caller draining a port should keep polling until
    /// it has them all back.
    pub fn port_action(
        &mut self,
        port: &PortInfo,
        action: PortAction,
        timer: &Timer,
    ) -> Result<(), Error> {
        self.begin(MSG_PORT_ACTION);
        self.put_u32(HEADER_SIZE, port.component);
        self.put_u32(HEADER_SIZE + 4, port.handle);
        self.put_u32(HEADER_SIZE + 8, action as u32);
        let port_offset = HEADER_SIZE + 12;
        self.tx[port_offset..port_offset + PORT_FIELDS_SIZE].fill(0);
        self.write_port_fields(port_offset, port);

        let reply = self.request(port_offset + PORT_FIELDS_SIZE, MSG_PORT_ACTION, timer)?;
        status(get_u32(reply, HEADER_SIZE))
    }

    /// Sets a port parameter. `value` is the parameter's own structure,
    /// which for the common boolean and integer parameters is a single
    /// little-endian 32-bit word.
    pub fn parameter_set(
        &mut self,
        port: &PortInfo,
        id: u32,
        value: &[u8],
        timer: &Timer,
    ) -> Result<(), Error> {
        if HEADER_SIZE + 16 + value.len() > MSG_MAX_SIZE {
            return Err(Error::TooLarge);
        }
        self.begin(MSG_PORT_PARAMETER_SET);
        self.put_u32(HEADER_SIZE, port.component);
        self.put_u32(HEADER_SIZE + 4, port.handle);
        self.put_u32(HEADER_SIZE + 8, id);
        // The size the firmware wants is the parameter header (its id and
        // this very size word) plus the value.
        self.put_u32(HEADER_SIZE + 12, 8 + value.len() as u32);
        self.tx[HEADER_SIZE + 16..HEADER_SIZE + 16 + value.len()].copy_from_slice(value);

        let reply = self.request(
            HEADER_SIZE + 16 + value.len(),
            MSG_PORT_PARAMETER_SET,
            timer,
        )?;
        status(get_u32(reply, HEADER_SIZE))
    }

    /// Sets a port parameter whose value is a string — the shape a
    /// handful of them take, the audio renderer's destination
    /// ([`crate::audio_render`]) among them.
    ///
    /// The value the firmware is given is the string *and* its
    /// terminating NUL, which is what the size in the parameter header
    /// counts. Sending the bytes without the terminator leaves the
    /// firmware reading a string that doesn't end where the parameter
    /// does.
    pub fn parameter_set_string(
        &mut self,
        port: &PortInfo,
        id: u32,
        value: &str,
        timer: &Timer,
    ) -> Result<(), Error> {
        let mut bytes = [0u8; PARAMETER_STRING_MAX];
        let Some(terminated) = bytes.get_mut(..value.len() + 1) else {
            return Err(Error::TooLarge);
        };
        terminated[..value.len()].copy_from_slice(value.as_bytes());
        let length = terminated.len();
        self.parameter_set(port, id, &bytes[..length], timer)
    }

    /// Reads a port parameter into `value`, returning how many bytes the
    /// firmware actually reported — which can exceed `value` if the
    /// parameter is larger than the caller expected, in which case only
    /// what fits was copied.
    pub fn parameter_get(
        &mut self,
        port: &PortInfo,
        id: u32,
        value: &mut [u8],
        timer: &Timer,
    ) -> Result<usize, Error> {
        self.begin(MSG_PORT_PARAMETER_GET);
        self.put_u32(HEADER_SIZE, port.component);
        self.put_u32(HEADER_SIZE + 4, port.handle);
        self.put_u32(HEADER_SIZE + 8, id);
        self.put_u32(HEADER_SIZE + 12, 8 + value.len() as u32);

        let reply = self.request(HEADER_SIZE + 16, MSG_PORT_PARAMETER_GET, timer)?;
        status(get_u32(reply, HEADER_SIZE))?;

        // As on the way out, the reported size counts the parameter's own
        // header, which isn't part of the value.
        let size = (get_u32(reply, HEADER_SIZE + 8) as usize).saturating_sub(8);
        let copied = size.min(value.len());
        value[..copied].copy_from_slice(&reply[HEADER_SIZE + 12..HEADER_SIZE + 12 + copied]);
        Ok(size)
    }

    /// Hands `buffer` to a port, transferring ownership of it to the
    /// VideoCore until it comes back out of [`Self::poll`].
    ///
    /// For an input port, `length` bytes of `buffer` are the data to
    /// process and are bulk-transferred to the firmware; `flags` and `pts`
    /// describe them ([`BUFFER_FLAG_EOS`] on the last one,
    /// [`TIME_UNKNOWN`] for a stream whose timestamps don't matter). For
    /// an output port the buffer is empty — pass a `length` of 0 — and
    /// comes back filled.
    ///
    /// `buffer` must be at least 64-byte (cache-line) aligned and its
    /// length a multiple of that, since the firmware's DMA into it is
    /// bracketed by whole-cache-line maintenance.
    pub fn send_buffer(
        &mut self,
        port: &PortInfo,
        buffer: &'static mut [u8],
        length: usize,
        flags: u32,
        pts: i64,
        timer: &Timer,
    ) -> Result<(), Error> {
        if length > buffer.len() {
            return Err(Error::TooLarge);
        }
        let index = self
            .buffers
            .iter()
            .position(|slot| slot.is_none())
            .ok_or(Error::TooManyBuffers)?;
        let address = buffer.as_ptr() as u32;
        let size = buffer.len();
        // Ownership of the memory now rests in the table; the slice is
        // rebuilt from these two fields when the firmware gives it back.
        self.buffers[index] = Some(BufferSlot {
            address,
            size,
            port: port.handle,
            state: BufferState::WithFirmware,
            length: 0,
            flags: 0,
            pts: TIME_UNKNOWN,
        });
        let context = buffer_context(index);

        self.begin(MSG_BUFFER_FROM_HOST);
        // A buffer's context is its table entry, not a request context: the
        // firmware quotes it back when it returns the buffer, which may be
        // long after any number of other exchanges.
        self.put_u32(12, context);
        let payload = HEADER_SIZE;
        self.tx[payload..payload + BUFFER_FROM_HOST_SIZE].fill(0);
        // The private area the firmware copies back verbatim, which is how
        // a returned buffer identifies itself.
        self.put_u32(payload, MAGIC);
        self.put_u32(payload + 4, port.component);
        self.put_u32(payload + 8, port.handle);
        self.put_u32(payload + 12, context);
        // The buffer header. `data` is only ever context for the firmware
        // here — the payload itself travels by bulk transfer — but it is
        // what the firmware logs, so it gets the real address.
        let header = payload + 32;
        self.put_u32(header + 12, address);
        self.put_u32(header + 16, size as u32);
        self.put_u32(header + 20, length as u32);
        self.put_u32(header + 24, 0);
        self.put_u32(header + 28, flags);
        self.put_i64(header + 32, pts);
        self.put_i64(header + 40, pts);

        self.send(HEADER_SIZE + BUFFER_FROM_HOST_SIZE, timer)?;
        self.stats.buffers_sent += 1;

        if length > 0 {
            // The payload follows its announcement immediately: the
            // firmware pairs the two by order, so nothing else may be sent
            // in between.
            //
            // SAFETY: the memory belongs to the table entry above until
            // the firmware returns it, and nothing here touches it in the
            // meantime.
            unsafe {
                self.vchiq
                    .bulk_transmit(self.service, address, (length + 3) & !3, context)?;
            }
        }
        Ok(())
    }

    /// Moves the driver forward, returning the next thing that needs the
    /// caller's attention.
    ///
    /// This must be called regularly: it is what notices returned buffers,
    /// starts and finishes the bulk transfers that fill them, and picks up
    /// port events.
    pub fn poll(&mut self) -> Result<Option<Event>, Error> {
        if let Some(event) = self.take_returned() {
            return Ok(Some(event));
        }
        self.pump()?;
        Ok(self.take_returned())
    }

    /// The payload of the [`Event::PortEvent`] [`Self::poll`] last
    /// returned, valid until the next port event arrives.
    pub fn event_data(&self) -> &[u8] {
        let length = (get_u32(&self.event, HEADER_SIZE + 16) as usize).min(EVENT_DATA_MAX);
        &self.event[HEADER_SIZE + 20..HEADER_SIZE + 20 + length]
    }

    /// The transport underneath, for a caller that needs to close the
    /// service, shut the whole thing down, or read [`vchiq::Stats`].
    pub fn vchiq(&mut self) -> &mut Vchiq {
        &mut self.vchiq
    }

    /// What the firmware has done with the buffers handed to it — see
    /// [`Stats`].
    pub fn stats(&self) -> Stats {
        self.stats
    }

    /// Sends a request and waits for its reply, checking that the reply is
    /// the type the request expects.
    ///
    /// Buffer traffic that arrives while waiting is processed rather than
    /// dropped — a port being disabled returns its buffers *before*
    /// acknowledging the disable, so this is the normal case, not an edge
    /// one.
    fn request(&mut self, len: usize, expected_type: u32, timer: &Timer) -> Result<&[u8], Error> {
        // The context word the firmware quotes back is what pairs this
        // reply with this request. Buffer contexts live in their own range
        // above `BUFFER_CONTEXT_BASE`, so the two can't be confused.
        self.next_context = self.next_context.wrapping_add(1) % BUFFER_CONTEXT_BASE;
        self.pending_context = self.next_context.max(1);
        self.put_u32(12, self.pending_context);
        self.reply_ready = false;

        self.send(len, timer)?;

        let deadline = timer.now_micros() + REQUEST_TIMEOUT_US;
        while !self.reply_ready {
            self.pump()?;
            if timer.now_micros() > deadline {
                self.pending_context = 0;
                return Err(Error::Timeout);
            }
        }
        self.reply_ready = false;
        self.pending_context = 0;

        if get_u32(&self.rx, 4) != expected_type {
            return Err(Error::UnexpectedReply);
        }
        Ok(&self.rx[..self.rx_len])
    }

    /// Sends the message built in [`Self::tx`], retrying while the
    /// transport is out of slots — which only means the firmware hasn't
    /// caught up yet, and polling is what lets it.
    fn send(&mut self, len: usize, timer: &Timer) -> Result<(), Error> {
        let deadline = timer.now_micros() + REQUEST_TIMEOUT_US;
        loop {
            match self.vchiq.send(self.service, &self.tx[..len]) {
                Ok(()) => return Ok(()),
                Err(vchiq::Error::OutOfSlots) => {}
                Err(error) => return Err(error.into()),
            }
            self.pump()?;
            if timer.now_micros() > deadline {
                return Err(Error::Timeout);
            }
        }
    }

    /// Processes one transport event, updating this driver's state.
    fn pump(&mut self) -> Result<(), Error> {
        let Some(event) = self.vchiq.poll(&mut self.rx)? else {
            return Ok(());
        };
        match event {
            vchiq::Event::Message { len, .. } => {
                self.rx_len = len.min(MSG_MAX_SIZE);
                self.on_message()?;
            }
            vchiq::Event::BulkReceiveDone { context, actual } => {
                // The data has landed and has been invalidated out of this
                // core's cache; the buffer is the caller's again.
                if let Some(index) = buffer_index(context) {
                    if let Some(slot) = &mut self.buffers[index] {
                        slot.length = slot.length.min(actual);
                        slot.state = BufferState::Returned;
                    }
                    self.push_returned(index);
                }
            }
            // Nothing to do: an input buffer's payload having been read
            // does not mean the firmware is finished with the buffer. That
            // is announced separately, as the buffer coming back.
            vchiq::Event::BulkTransmitDone { .. } => {}
            vchiq::Event::Connected
            | vchiq::Event::ServiceOpened(_)
            | vchiq::Event::ServiceClosed(_) => {}
        }
        Ok(())
    }

    /// Dispatches the message now in [`Self::rx`].
    fn on_message(&mut self) -> Result<(), Error> {
        match get_u32(&self.rx, 4) {
            MSG_BUFFER_TO_HOST => self.on_buffer_to_host(),
            // The firmware echoes these back as it takes them; the
            // buffer's real return is a separate `BUFFER_TO_HOST`.
            MSG_BUFFER_FROM_HOST => Ok(()),
            // Copied aside for `event_data` and flagged as the next thing
            // to report. An event that arrives while one is still
            // unreported replaces it — the same thing the firmware's own
            // clients do, since an event carries no state that a later one
            // doesn't supersede.
            MSG_EVENT_TO_HOST => {
                self.event.copy_from_slice(&self.rx);
                self.pending_event = true;
                self.stats.events += 1;
                self.stats.last_event = get_u32(&self.rx, HEADER_SIZE + 12);
                Ok(())
            }
            // A reply to whatever request is outstanding. Anything quoting
            // a context this driver has forgotten — a reply that arrived
            // after its request timed out — is dropped.
            _ => {
                if self.pending_context != 0 && get_u32(&self.rx, 12) == self.pending_context {
                    self.reply_ready = true;
                }
                Ok(())
            }
        }
    }

    /// Handles a buffer coming back from the firmware: either its data is
    /// already here, or a bulk transfer has to fetch it.
    fn on_buffer_to_host(&mut self) -> Result<(), Error> {
        let payload = HEADER_SIZE;
        if get_u32(&self.rx, payload) != MAGIC {
            return Err(Error::UnknownBuffer);
        }
        let context = get_u32(&self.rx, payload + 12);
        let index = buffer_index(context).ok_or(Error::UnknownBuffer)?;
        let Some(slot) = self.buffers[index] else {
            return Err(Error::UnknownBuffer);
        };

        let header = payload + 32;
        let length = (get_u32(&self.rx, header + 20) as usize).min(slot.size);
        let flags = get_u32(&self.rx, header + 28);
        let pts = get_i64(&self.rx, header + 32);
        let payload_in_message = get_u32(&self.rx, payload + 136) as usize;
        let message_status = get_u32(&self.rx, 16);

        if let Some(slot) = &mut self.buffers[index] {
            slot.length = length;
            slot.flags = flags;
            slot.pts = pts;
        }
        self.stats.buffers_returned += 1;
        self.stats.last_returned_flags = flags;
        self.stats.last_returned_length = length as u32;

        if message_status != 0 {
            // A failure is reported by returning the buffer with a status
            // rather than data. Hand it straight back rather than fetching
            // bytes the firmware has just said aren't there.
            self.stats.buffers_failed += 1;
            self.stats.last_status = message_status;
            self.finish_buffer(index, 0);
            return Ok(());
        }

        if payload_in_message > 0 {
            self.stats.inline_receives += 1;
            // Payloads small enough ride along inside the message.
            let copied = payload_in_message.min(slot.size);
            // SAFETY: the memory belongs to this table entry until it is
            // handed back, so writing it here is exclusive. `copied` is
            // capped at the size the buffer was registered with.
            let destination =
                unsafe { core::slice::from_raw_parts_mut(slot.address as *mut u8, copied) };
            destination.copy_from_slice(&self.rx[payload + 140..payload + 140 + copied]);
            self.finish_buffer(index, copied);
            return Ok(());
        }

        // A buffer with nothing to fetch comes straight back — except at
        // the end of the stream, where the firmware still expects a bulk
        // transfer, and a token one is what its own clients send to keep
        // the buffer ordering intact.
        let transfer = if length == 0 {
            if flags & BUFFER_FLAG_EOS == 0 {
                self.finish_buffer(index, 0);
                return Ok(());
            }
            8
        } else {
            // Bulk transfers move whole words.
            (length + 3) & !3
        };

        if let Some(slot) = &mut self.buffers[index] {
            slot.state = BufferState::Receiving;
        }
        self.stats.bulk_receives += 1;
        // SAFETY: the memory belongs to this table entry until it is handed
        // back, and the entry stays `Receiving` — untouched by everything
        // here — until the transfer completes.
        unsafe {
            self.vchiq
                .bulk_receive(self.service, slot.address, transfer, context)?;
        }
        Ok(())
    }

    /// Marks a buffer finished with `length` meaningful bytes and queues
    /// it to be handed back.
    fn finish_buffer(&mut self, index: usize, length: usize) {
        if let Some(slot) = &mut self.buffers[index] {
            slot.length = length;
            slot.state = BufferState::Returned;
        }
        self.push_returned(index);
    }

    /// Hands back the oldest finished buffer, or the port event that came
    /// in, if either is waiting.
    fn take_returned(&mut self) -> Option<Event> {
        if self.returned_len > 0 {
            let index = self.returned[0];
            self.returned.rotate_left(1);
            self.returned_len -= 1;
            let slot = self.buffers[index].take()?;
            // SAFETY: this rebuilds the slice `send_buffer` was given.
            // The firmware has returned the memory, and the table entry
            // has just been dropped, so this is the only reference again.
            let buffer =
                unsafe { core::slice::from_raw_parts_mut(slot.address as *mut u8, slot.size) };
            return Some(Event::Buffer {
                port: slot.port,
                buffer,
                length: slot.length,
                flags: slot.flags,
                pts: slot.pts,
            });
        }
        if self.pending_event {
            self.pending_event = false;
            return Some(Event::PortEvent {
                port_type: get_u32(&self.event, HEADER_SIZE + 4),
                port_index: get_u32(&self.event, HEADER_SIZE + 8),
                cmd: get_u32(&self.event, HEADER_SIZE + 12),
                length: (get_u32(&self.event, HEADER_SIZE + 16) as usize).min(EVENT_DATA_MAX),
            });
        }
        None
    }

    /// Queues a finished buffer for [`Self::take_returned`], preserving
    /// the order the firmware returned them in.
    fn push_returned(&mut self, index: usize) {
        if self.returned_len < MAX_BUFFERS {
            self.returned[self.returned_len] = index;
            self.returned_len += 1;
        }
    }

    /// Starts building an outgoing message: magic and type, with the
    /// context word left for [`Self::request`] (or, for a buffer, for
    /// [`Self::send_buffer`]) to fill in.
    fn begin(&mut self, message_type: u32) {
        self.tx[..HEADER_SIZE].fill(0);
        self.put_u32(0, MAGIC);
        self.put_u32(4, message_type);
    }

    /// Writes the port structure the info-set and action messages carry.
    /// Only the buffer count and size are writable; the rest is echoed
    /// back as read.
    fn write_port_fields(&mut self, offset: usize, port: &PortInfo) {
        self.put_u32(offset + PORT_TYPE, port.port_type);
        // 16-bit, with `index_all` in the half-word above it, left zero.
        self.tx[offset + PORT_INDEX..offset + PORT_INDEX + 2]
            .copy_from_slice(&(port.index as u16).to_le_bytes());
        self.put_u32(offset + PORT_IS_ENABLED, u32::from(port.enabled));
        self.put_u32(offset + PORT_BUFFER_NUM_MIN, port.buffer_num_min);
        self.put_u32(offset + PORT_BUFFER_SIZE_MIN, port.buffer_size_min);
        self.put_u32(
            offset + PORT_BUFFER_ALIGNMENT_MIN,
            port.buffer_alignment_min,
        );
        self.put_u32(
            offset + PORT_BUFFER_NUM_RECOMMENDED,
            port.buffer_num_recommended,
        );
        self.put_u32(
            offset + PORT_BUFFER_SIZE_RECOMMENDED,
            port.buffer_size_recommended,
        );
        self.put_u32(offset + PORT_BUFFER_NUM, port.buffer_num);
        self.put_u32(offset + PORT_BUFFER_SIZE, port.buffer_size);
    }

    /// Writes the video-specific half of a format.
    fn write_video_format(&mut self, offset: usize, video: &VideoFormat) {
        self.put_u32(offset, video.width);
        self.put_u32(offset + 4, video.height);
        self.put_u32(offset + 8, video.crop_x as u32);
        self.put_u32(offset + 12, video.crop_y as u32);
        self.put_u32(offset + 16, video.crop_width as u32);
        self.put_u32(offset + 20, video.crop_height as u32);
        self.put_u32(offset + 24, video.frame_rate_num as u32);
        self.put_u32(offset + 28, video.frame_rate_den as u32);
        self.put_u32(offset + 32, video.par_num as u32);
        self.put_u32(offset + 36, video.par_den as u32);
        self.put_u32(offset + 40, video.color_space);
    }

    /// Writes the audio-specific half of a format. Shorter than the video
    /// one it shares its union with, so the caller zeroes the whole union
    /// first and the tail stays zero.
    fn write_audio_format(&mut self, offset: usize, audio: &AudioFormat) {
        self.put_u32(offset, audio.channels);
        self.put_u32(offset + 4, audio.sample_rate);
        self.put_u32(offset + 8, audio.bits_per_sample);
        self.put_u32(offset + 12, audio.block_align);
    }

    /// Shared shape of the component messages, which all carry nothing but
    /// a component handle.
    fn component_request(
        &mut self,
        message_type: u32,
        component: &Component,
        timer: &Timer,
    ) -> Result<(), Error> {
        self.begin(message_type);
        self.put_u32(HEADER_SIZE, component.handle);
        let reply = self.request(HEADER_SIZE + 4, message_type, timer)?;
        status(get_u32(reply, HEADER_SIZE))
    }

    /// Writes a 32-bit field of the outgoing message.
    fn put_u32(&mut self, offset: usize, value: u32) {
        self.tx[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    /// Writes a 64-bit field of the outgoing message.
    fn put_i64(&mut self, offset: usize, value: i64) {
        self.tx[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }
}

/// Buffers one port's pool can hold — the ceiling on what a single port
/// can have in flight, shared by the drivers built on this client
/// ([`crate::video_decode::MAX_PORT_BUFFERS`],
/// [`crate::audio_render::MAX_BUFFERS`]). Enough to keep a port busy
/// while the application works on another buffer; the ceiling exists only
/// because the pool is a fixed-size array.
pub(crate) const POOL_CAPACITY: usize = 6;

/// A pool of buffers waiting to be handed to a port, holding them between
/// the firmware giving one back and the driver sending it out again.
pub(crate) struct Pool {
    /// The free buffers.
    buffers: [Option<&'static mut [u8]>; POOL_CAPACITY],
    /// How many were ever added, which is what the port is told to expect.
    total: usize,
}

impl Pool {
    /// An empty pool.
    pub(crate) const fn new() -> Self {
        Self {
            buffers: [const { None }; POOL_CAPACITY],
            total: 0,
        }
    }

    /// Adds a buffer, answering `false` — and keeping the buffer, which is
    /// `'static` and so leaks nothing — once the pool is full.
    pub(crate) fn add(&mut self, buffer: &'static mut [u8]) -> bool {
        let Some(slot) = self.buffers.iter_mut().find(|slot| slot.is_none()) else {
            return false;
        };
        *slot = Some(buffer);
        self.total += 1;
        true
    }

    /// Puts a buffer back after the firmware has returned it.
    pub(crate) fn put(&mut self, buffer: &'static mut [u8]) {
        if let Some(slot) = self.buffers.iter_mut().find(|slot| slot.is_none()) {
            *slot = Some(buffer);
        }
    }

    /// Takes a free buffer, if there is one.
    pub(crate) fn take(&mut self) -> Option<&'static mut [u8]> {
        self.buffers.iter_mut().find_map(|slot| slot.take())
    }

    /// How many buffers were ever added — what the port is told to expect,
    /// which is not how many are free right now.
    pub(crate) fn total(&self) -> usize {
        self.total
    }

    /// The smallest buffer in the pool, which is the size every buffer can
    /// be relied on to have.
    pub(crate) fn smallest(&self) -> usize {
        self.buffers
            .iter()
            .flatten()
            .map(|buffer| buffer.len())
            .min()
            .unwrap_or(0)
    }
}

/// Timeout for a request/reply exchange with the firmware, in
/// microseconds. Generous: the firmware answers in well under a
/// millisecond when healthy, so this only bounds the failure case.
const REQUEST_TIMEOUT_US: u64 = 2_000_000;

/// Context words at or above this identify a buffer table entry; below it
/// they identify a request awaiting a reply.
const BUFFER_CONTEXT_BASE: u32 = 0x1000_0000;

/// Bytes in the port structure a message carries.
const PORT_FIELDS_SIZE: usize = 64;

// Field offsets within that port structure. Spelled out, and spelled out
// *completely* — including the four fields nothing here reads — because
// the structure is easy to miscount: two of its fields are 16-bit (an
// index and an all-ports index, packed into one word between the type and
// the enabled flag), so the words do not fall where reading the field list
// suggests. Getting that wrong shifts every field after it by four bytes,
// and the failure is silent: the firmware accepts the message, keeps its
// own defaults for the values that landed in the wrong slots, and the only
// symptom is that nothing works. Listing all seventeen makes the offsets
// checkable against the structure at a glance, which is what the
// `PORT_CAPABILITIES` assertion below then confirms.
#[allow(dead_code)]
/// `priv`, a firmware-side pointer (read-only).
const PORT_PRIV: usize = 0;
#[allow(dead_code)]
/// `name`, a firmware-side pointer (read-only).
const PORT_NAME: usize = 4;
/// Port type: [`PORT_TYPE_INPUT`] and friends.
const PORT_TYPE: usize = 8;
/// 16-bit index within the type, with `index_all` in the half-word above.
const PORT_INDEX: usize = 12;
/// Whether the firmware has the port enabled.
const PORT_IS_ENABLED: usize = 16;
#[allow(dead_code)]
/// `format`, a firmware-side pointer (read-only).
const PORT_FORMAT: usize = 20;
/// Fewest buffers the port will work with (read-only).
const PORT_BUFFER_NUM_MIN: usize = 24;
/// Smallest buffer it will accept (read-only).
const PORT_BUFFER_SIZE_MIN: usize = 28;
/// Required buffer alignment (read-only).
const PORT_BUFFER_ALIGNMENT_MIN: usize = 32;
/// Buffer count the component suggests (read-only).
const PORT_BUFFER_NUM_RECOMMENDED: usize = 36;
/// Buffer size the component suggests (read-only).
const PORT_BUFFER_SIZE_RECOMMENDED: usize = 40;
/// Buffer count the client will use — writable.
const PORT_BUFFER_NUM: usize = 44;
/// Buffer size the client will use — writable.
const PORT_BUFFER_SIZE: usize = 48;
#[allow(dead_code)]
/// `component`, a firmware-side pointer (read-only).
const PORT_COMPONENT: usize = 52;
#[allow(dead_code)]
/// `userdata`, reserved for the client.
const PORT_USERDATA: usize = 56;
/// Capability flags (read-only).
const PORT_CAPABILITIES: usize = 60;
const _: () = assert!(PORT_CAPABILITIES + 4 == PORT_FIELDS_SIZE);

/// Bytes in the format structure a message carries.
const FORMAT_FIELDS_SIZE: usize = 32;

/// Bytes in the type-specific format union a message carries — sized by
/// its largest member, the video one.
const ES_FIELDS_SIZE: usize = 44;

/// Bytes in a buffer-from-host payload: two private areas, the buffer
/// header, its type-specific half, three words of flags, and the inline
/// short-data area — then rounded up to a multiple of 8.
///
/// That rounding is not cosmetic. The fields come to 268 bytes, but the
/// buffer header carries 64-bit timestamps, so the structure the firmware
/// was compiled against is 8-byte aligned and its `sizeof` — which is what
/// both the Linux and the userland client send as the message length — is
/// 272. Sending the unpadded 268 produces a message four bytes short of
/// what the firmware expects, and it answers by handing the buffer
/// straight back as if it were empty: never fetching the payload, never
/// completing the bulk transfer that was carrying it, and reporting no
/// error anywhere. Every other message here is all 32-bit fields, so this
/// is the only one where the distinction bites.
const BUFFER_FROM_HOST_SIZE: usize = (32 + 56 + 40 + 12 + SHORT_DATA_MAX).next_multiple_of(8);

/// Decodes an [`EVENT_FORMAT_CHANGED`] payload, i.e. what
/// [`Mmal::event_data`] holds after that event.
///
/// Returns `None` if the payload is too short to be one, which is what a
/// caller passing the payload of some other event gets.
pub fn parse_format_changed(data: &[u8]) -> Option<FormatChanged> {
    // The layout is four buffer-requirement words, a pointer the firmware
    // fills with its own address (ignored here), then a format and its
    // video-specific half — so the format starts at the sixth word.
    let format = 20;
    let es = format + FORMAT_FIELDS_SIZE;
    if data.len() < es + ES_FIELDS_SIZE {
        return None;
    }
    Some(FormatChanged {
        buffer_size_min: get_u32(data, 0),
        buffer_num_min: get_u32(data, 4),
        buffer_size_recommended: get_u32(data, 8),
        buffer_num_recommended: get_u32(data, 12),
        es_type: get_u32(data, format),
        encoding: get_u32(data, format + 4),
        video: read_video_format(data, es),
    })
}

/// The context word standing for a buffer table entry.
fn buffer_context(index: usize) -> u32 {
    BUFFER_CONTEXT_BASE + index as u32
}

/// The buffer table entry a context word refers to, if it is one rather
/// than a request's.
fn buffer_index(context: u32) -> Option<usize> {
    let index = context.checked_sub(BUFFER_CONTEXT_BASE)? as usize;
    (index < MAX_BUFFERS).then_some(index)
}

/// Turns a nonzero MMAL status into an error.
fn status(value: u32) -> Result<(), Error> {
    if value == 0 {
        Ok(())
    } else {
        Err(Error::Status(value))
    }
}

/// Reads the video-specific half of a format out of a message.
fn read_video_format(data: &[u8], offset: usize) -> VideoFormat {
    VideoFormat {
        width: get_u32(data, offset),
        height: get_u32(data, offset + 4),
        crop_x: get_u32(data, offset + 8) as i32,
        crop_y: get_u32(data, offset + 12) as i32,
        crop_width: get_u32(data, offset + 16) as i32,
        crop_height: get_u32(data, offset + 20) as i32,
        frame_rate_num: get_u32(data, offset + 24) as i32,
        frame_rate_den: get_u32(data, offset + 28) as i32,
        par_num: get_u32(data, offset + 32) as i32,
        par_den: get_u32(data, offset + 36) as i32,
        color_space: get_u32(data, offset + 40),
    }
}

/// Reads the audio-specific half of a format out of a message.
fn read_audio_format(data: &[u8], offset: usize) -> AudioFormat {
    AudioFormat {
        channels: get_u32(data, offset),
        sample_rate: get_u32(data, offset + 4),
        bits_per_sample: get_u32(data, offset + 8),
        block_align: get_u32(data, offset + 12),
    }
}

/// Reads a 32-bit field of a received message. Out-of-range offsets read
/// as zero rather than panicking: a truncated message from the firmware
/// should surface as a status or a mismatched reply, not as a fault in a
/// driver with no way to report one.
fn get_u32(data: &[u8], offset: usize) -> u32 {
    match data.get(offset..offset + 4) {
        Some(bytes) => u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
        None => 0,
    }
}

/// Reads a 64-bit field of a received message; see [`get_u32`]. Assembled
/// in `u64` and then reinterpreted, since a timestamp of
/// [`TIME_UNKNOWN`] has its top bit set and shifting the high word into
/// place as a signed value would overflow.
fn get_i64(data: &[u8], offset: usize) -> i64 {
    (u64::from(get_u32(data, offset)) | (u64::from(get_u32(data, offset + 4)) << 32)) as i64
}

/// MMAL's own four-character code packing: first character in the *low*
/// byte, the opposite order from VCHIQ's service codes.
const fn fourcc(code: [u8; 4]) -> u32 {
    u32::from_le_bytes(code)
}
