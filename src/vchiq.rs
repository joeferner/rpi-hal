//! VCHIQ — the shared-memory message transport to the VideoCore firmware.
//!
//! [`crate::mailbox`] is a doorbell with one 32-bit word: the ARM writes a
//! buffer address, the firmware answers in place, and that is the whole
//! protocol. Everything the firmware offers beyond simple queries — the
//! camera ISP, the video codecs, audio — is instead reached through
//! *services* running on the VideoCore, addressed by a four-character code
//! and spoken to with a stream of variable-length messages plus bulk
//! (DMA) transfers for payloads too big to inline. VCHIQ is the transport
//! that carries them, and this module is a from-scratch implementation of
//! its ARM (slave) side. [`crate::mmal`] is the one client of it here.
//!
//! ## How it works
//!
//! A single contiguous region of RAM
//! ([`SharedMemory`](crate::vchiq::SharedMemory)) is handed to the
//! firmware once, by bus address, through
//! [`Mailbox::vchiq_init`](crate::mailbox::Mailbox::vchiq_init). Its first
//! 4KB "slot zero" holds the protocol's shared bookkeeping — a magic
//! number and version, and one `SharedState` per side — and the slots
//! after it are the message ring: each side writes messages into the slots
//! it owns, advancing a byte position (`tx_pos`) the other side reads. A
//! message is an 8-byte header (a packed type/source-port/destination-port
//! id, and a length) followed by its data, padded so the next header stays
//! 8-byte aligned. A slot that has been fully consumed is handed back
//! through a recycle queue for the owner to write into again.
//!
//! Signalling is by doorbell: writing the VideoCore's bell register
//! (`BELL2`, at offset `0xB848` in the peripheral block, alongside the
//! property mailbox's own registers but not in either PAC's SVD, so poked
//! directly the way `dma.rs`/`v3d.rs` do) interrupts the VideoCore. The
//! reverse direction — the firmware ringing `BELL0` and interrupting the
//! ARM — is deliberately not used here: this driver never sets its
//! `armed` flags, which is exactly the condition the protocol defines for
//! "don't bother interrupting me", so the firmware quietly updates
//! `tx_pos` and this driver notices in
//! [`Vchiq::poll`](crate::vchiq::Vchiq::poll). That keeps the
//! whole subsystem interrupt-free, at the cost of the application having
//! to call `poll` regularly.
//!
//! ## Why the memory has to be non-cacheable
//!
//! Every other bus-master driver in this crate (`dma`, `sd`, `usb`, and
//! the mailbox itself) gets by with explicit cache maintenance because a
//! buffer has one owner at a time: the ARM fills it, cleans it, and hands
//! it over. VCHIQ's shared state is not like that — both sides write
//! *different fields of the same cache line* whenever they like. The
//! `trigger` event's `armed` (written here) and `fired` (written by the
//! firmware) are 4 bytes apart, and `tx_pos` shares a 32-byte line with
//! both. There is no clean/invalidate sequence that survives that:
//! cleaning the line to publish this core's field writes a stale copy of
//! the firmware's field back over it, and invalidating it to read the
//! firmware's field throws away this core's not-yet-written-back one.
//!
//! So [`Vchiq::new`](crate::vchiq::Vchiq::new) remaps the whole region
//! Normal Non-cacheable via
//! [`crate::mmu::set_uncached`] — the bare-metal equivalent of the
//! `dma_alloc_coherent` Linux's own driver uses for it — and every access
//! below is a volatile read or write with explicit barriers where the
//! protocol needs ordering. This is why the `vchiq` feature implies `mmu`:
//! without this crate's translation tables there is nothing to remap.
//!
//! Bulk transfer buffers are the *other* kind of memory — ordinary
//! cacheable RAM, exclusively owned for the duration of a transfer — so
//! those keep the usual clean-before-give, invalidate-after-take
//! maintenance, done by this module rather than by its caller.
//!
//! ## What is not implemented
//!
//! This is the slave (ARM) half only, which is all the firmware ever needs
//! from this side: bulk transfers are always mastered by the VideoCore, so
//! an inbound bulk *request* is a protocol error rather than something to
//! service. Synchronous-mode messages, service quotas, pause/resume, and
//! the "remote use" power-management handshake are all recognized on the
//! wire and skipped — no client here uses them.

use crate::cache::{barrier, clean_invalidate_range, clean_range, invalidate_range};
use crate::mailbox::Mailbox;
use crate::mmu;
use crate::soc::PERIPHERAL_BASE;
use crate::timer::Timer;
use core::mem::offset_of;

/// Bytes in one slot — the protocol's fixed unit of both message storage
/// and slot recycling.
const SLOT_SIZE: usize = 4096;

/// Mask for the byte offset within a slot.
const SLOT_MASK: u32 = SLOT_SIZE as u32 - 1;

/// Slots each side may own, and so the length of a `slot_queue`. Fixed by
/// the protocol (the firmware reads a `slot_queue` of exactly this length
/// out of slot zero), not a tuning choice.
const MAX_SLOTS_PER_SIDE: usize = 64;

/// Mask for indexing a `slot_queue`, which is circular.
const SLOT_QUEUE_MASK: u32 = MAX_SLOTS_PER_SIDE as u32 - 1;

/// Entries in slot zero's per-slot use/release table — the protocol's
/// ceiling on slots across both sides.
const MAX_SLOTS: usize = 128;

/// Debug words at the end of each side's `SharedState`. Not used here, but
/// they are part of the structure's size, and its size sets where the
/// firmware expects everything after it to be — so this must match the
/// value the firmware was built against (Linux's `DEBUG_MAX`, with its
/// debug entries compiled in, which is the configuration the firmware
/// interoperates with).
const DEBUG_ENTRIES: usize = 11;

/// Protocol magic in slot zero: `'V'`, `'C'`, `'H'`, `'I'`.
const MAGIC: u32 = u32::from_be_bytes(*b"VCHI");

/// Protocol version this implementation speaks, and the oldest it declares
/// compatibility with. Both are advertised in slot zero; the firmware
/// refuses to come up against a `version` below its own minimum.
const VERSION: u16 = 8;
/// See [`VERSION`].
const VERSION_MIN: u16 = 3;

// Message types, in the high 8 bits of a message id.
/// Filler to the end of a slot when the next message won't fit.
const MSG_PADDING: u32 = 0;
/// Connection handshake, sent by each side once slot zero is live.
const MSG_CONNECT: u32 = 1;
/// Open a service by four-character code.
const MSG_OPEN: u32 = 2;
/// The peer's acceptance of an [`MSG_OPEN`], carrying its port number.
const MSG_OPENACK: u32 = 3;
/// Close a service.
const MSG_CLOSE: u32 = 4;
/// An ordinary service message.
const MSG_DATA: u32 = 5;
/// Ask the peer to DMA into memory described by a page list.
const MSG_BULK_RX: u32 = 6;
/// Ask the peer to DMA out of memory described by a page list.
const MSG_BULK_TX: u32 = 7;
/// Completion of an [`MSG_BULK_RX`], carrying the byte count transferred.
const MSG_BULK_RX_DONE: u32 = 8;
/// Completion of an [`MSG_BULK_TX`], carrying the byte count transferred.
const MSG_BULK_TX_DONE: u32 = 9;

/// Shift of the message type within a message id.
const TYPE_SHIFT: u32 = 24;

/// Bytes in a message header (a message id and a length).
const HEADER_SIZE: u32 = 8;

/// Page size the firmware's bulk page lists are expressed in.
const PAGE_SIZE: u32 = 4096;

/// Bulk transfers this driver keeps in flight per direction. The protocol
/// allows four; the firmware completes them strictly in submission order,
/// which is what lets the queues below match a completion to a transfer by
/// position rather than by any identifier on the wire.
const MAX_BULKS: usize = 4;

/// Services this driver supports having open at once. The protocol's
/// ceiling is far higher (4096 ports); this is sized for its actual
/// clients — MMAL uses exactly one.
const MAX_SERVICES: usize = 4;

/// Largest message this driver will send or receive, header included: a
/// message may not straddle a slot boundary, so this is the protocol's own
/// limit rather than a choice.
pub const MAX_MESSAGE_SIZE: usize = SLOT_SIZE - HEADER_SIZE as usize;

/// Data slots in [`SharedMemory`], split evenly between the two sides —
/// 32 each, matching the count Linux's driver allocates.
const DATA_SLOTS: usize = 64;

/// Cache-line-sized scratch areas the firmware uses to stage the partial
/// head and tail lines of a bulk *receive* whose ends aren't cache-line
/// aligned, so that the ARM's own dirty lines can't be written back over
/// freshly transferred data. This driver never asks for that treatment —
/// its receive buffers are cache-line aligned and it owns every byte up to
/// the end of the covering line, so a direct transfer plus a whole-buffer
/// invalidate is correct — but the area is still declared to the firmware
/// exactly as Linux declares it, since it is part of what slot zero
/// advertises.
const MAX_FRAGMENTS: usize = 64;

/// Bytes per fragment area: two cache lines (head and tail), at the
/// 64-byte line size of both this SoC's cores.
const FRAGMENT_SIZE: usize = 128;

/// Bytes reserved for one page list. A page list is a header plus one
/// address word per physically contiguous run, and every buffer this
/// driver hands over is one identity-mapped, physically contiguous run —
/// so one address word is always enough, and the rest is padding that
/// keeps each page list in its own cache line.
const PAGELIST_SIZE: usize = 64;

/// Page lists live in the shared region, one per in-flight bulk transfer
/// in either direction.
const PAGELIST_COUNT: usize = 2 * MAX_BULKS;

/// Byte offset of the fragment area within [`SharedMemory`].
const FRAGMENTS_OFFSET: usize = (SLOT_ZERO_SLOTS + DATA_SLOTS) * SLOT_SIZE;

/// Byte offset of the page list area within [`SharedMemory`].
const PAGELISTS_OFFSET: usize = FRAGMENTS_OFFSET + MAX_FRAGMENTS * FRAGMENT_SIZE;

/// Slots occupied by slot zero itself, which the message slots follow.
const SLOT_ZERO_SLOTS: usize = size_of::<SlotZero>().div_ceil(SLOT_SIZE);

/// Size of [`SharedMemory`], which is one whole [`mmu::UNCACHED_GRANULE`]
/// on either target: the memory type can only be changed a granule at a
/// time, so anything sharing the granule would silently become
/// non-cacheable too.
const SHARED_MEMORY_SIZE: usize = 0x20_0000;

// The two shared structures are read and written by the firmware at
// offsets it was built with, so their layouts are not this crate's to
// choose. Both sizes are checked rather than trusted: a field of the wrong
// width or a stray alignment gap would move everything after it, and the
// symptom on hardware is silence rather than an error.
const _: () = assert!(size_of::<SharedState>() == 372);
const _: () = assert!(size_of::<SlotZero>() == 1288);
const _: () = assert!(SHARED_MEMORY_SIZE >= PAGELISTS_OFFSET + PAGELIST_COUNT * PAGELIST_SIZE);
const _: () = assert!(PAGELIST_SIZE >= size_of::<PageList>());
const _: () = assert!(SHARED_MEMORY_SIZE.is_multiple_of(mmu::UNCACHED_GRANULE));

/// A signal from one side to the other, living in shared memory.
///
/// `fired` is set by the sender; `armed` is set by a receiver that intends
/// to sleep and wants the doorbell rung. This driver polls, so it leaves
/// its own `armed` at zero — see the module doc comment — but it must
/// still honor the firmware's, which is what
/// [`Vchiq::signal_remote`] checks before ringing the bell.
#[repr(C)]
struct RemoteEvent {
    /// Set by the waiting side to request a doorbell.
    armed: u32,
    /// Set by the signalling side.
    fired: u32,
    /// Padding in the firmware's own definition of this structure.
    _unused: u32,
}

/// One side's half of the shared bookkeeping in slot zero.
#[repr(C)]
struct SharedState {
    /// Nonzero once the owning side has filled the rest in.
    initialised: u32,
    /// First and last slot index owned by this side, inclusive.
    slot_first: u32,
    /// See [`Self::slot_first`].
    slot_last: u32,
    /// Slot reserved for synchronous messages (unused by this driver).
    slot_sync: u32,
    /// Signalled when this side has written a message.
    trigger: RemoteEvent,
    /// Byte position of the next message this side will write. The low
    /// bits index within a slot, the rest index [`Self::slot_queue`].
    tx_pos: u32,
    /// Signalled when this side has returned a slot to its owner.
    recycle: RemoteEvent,
    /// Where the next recycled slot index will be written in
    /// [`Self::slot_queue`].
    slot_queue_recycle: u32,
    /// Synchronous-message events, initialized but never used here.
    sync_trigger: RemoteEvent,
    /// See [`Self::sync_trigger`].
    sync_release: RemoteEvent,
    /// Circular queue of slot indices: the slots this side may write to,
    /// in the order it will use them.
    slot_queue: [u32; MAX_SLOTS_PER_SIDE],
    /// Firmware-side debug counters; see [`DEBUG_ENTRIES`].
    debug: [u32; DEBUG_ENTRIES],
}

/// Per-slot reference counts, used to decide when a slot has been fully
/// consumed and can be recycled.
#[repr(C)]
struct SlotInfo {
    /// References taken on the slot's contents.
    use_count: u16,
    /// References released. Equal to [`Self::use_count`] means done.
    release_count: u16,
}

/// The head of the shared region: what the firmware reads first to decide
/// whether it is talking to something it understands.
#[repr(C)]
struct SlotZero {
    /// [`MAGIC`].
    magic: u32,
    /// [`VERSION`].
    version: u16,
    /// [`VERSION_MIN`].
    version_min: u16,
    /// `size_of::<SlotZero>()`, so the peer can check its own idea of the
    /// layout against this one.
    slot_zero_size: u32,
    /// [`SLOT_SIZE`].
    slot_size: u32,
    /// [`MAX_SLOTS`].
    max_slots: u32,
    /// [`MAX_SLOTS_PER_SIDE`].
    max_slots_per_side: u32,
    /// Bus address of the fragment area and the number of fragments in it.
    platform_data: [u32; 2],
    /// The VideoCore's half. It is the master; this driver never writes
    /// anything here except the fields the protocol says the peer writes:
    /// its recycle queue and its events.
    master: SharedState,
    /// This driver's half.
    slave: SharedState,
    /// Reference counts for every slot in the region.
    slots: [SlotInfo; MAX_SLOTS],
}

/// A bulk transfer's page list, as the firmware's DMA engine consumes it.
///
/// `addrs` is a run-length encoding: each word is a page-aligned bus
/// address with the number of consecutive pages *minus one* in its low 12
/// bits. One word covers up to 4096 pages (16MB), so every buffer this
/// driver can be handed fits in the single word declared here.
#[repr(C)]
struct PageList {
    /// Bytes to transfer.
    length: u32,
    /// [`PAGELIST_WRITE`] or [`PAGELIST_READ`].
    kind: u16,
    /// Byte offset of the data within the first page.
    offset: u16,
    /// The single run-length-encoded address word; see above.
    addrs: [u32; 1],
}

/// Page list direction: the firmware reads this memory (an ARM-to-VideoCore
/// transfer).
const PAGELIST_WRITE: u16 = 0;
/// Page list direction: the firmware writes this memory.
const PAGELIST_READ: u16 = 1;

/// The RAM shared with the VideoCore: slot zero, the message slots, the
/// bulk fragment area, and this driver's page lists.
///
/// A whole [`mmu::UNCACHED_GRANULE`] because [`Vchiq::new`] takes the
/// caches off it (see the module doc comment) and that can only be done a
/// granule at a time — anything else placed in the same granule would
/// silently become non-cacheable too. Most of it is genuinely used: 65
/// slots plus the fragment area is a little over 256KB.
///
/// The intended home is a `static`, since the firmware keeps a reference
/// to this memory forever after bring-up:
///
/// ```ignore
/// static mut VCHIQ_MEMORY: SharedMemory = SharedMemory::new();
/// ```
#[repr(C, align(0x20_0000))]
pub struct SharedMemory {
    /// Never accessed through this field — [`Vchiq`] works entirely in
    /// volatile accesses off its address, since the firmware writes it
    /// concurrently.
    #[allow(dead_code)]
    bytes: [u8; SHARED_MEMORY_SIZE],
}

impl SharedMemory {
    /// A new, zeroed region. `const` so it can initialize a `static`
    /// without the 2MB landing in the binary image.
    pub const fn new() -> Self {
        Self {
            bytes: [0; SHARED_MEMORY_SIZE],
        }
    }
}

impl Default for SharedMemory {
    /// Same as [`SharedMemory::new`].
    fn default() -> Self {
        Self::new()
    }
}

/// Errors from [`Vchiq`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The mailbox call that hands the firmware the shared region failed
    /// outright.
    Mailbox(crate::mailbox::Error),
    /// The firmware answered that call with a nonzero status, i.e. it
    /// looked at slot zero and refused it. A version or layout mismatch
    /// between this implementation and the running firmware is what that
    /// means in practice.
    SlotsRejected(u32),
    /// The firmware never filled in its own half of slot zero. Its VCHIQ
    /// side is either absent or wedged; on a Pi 3 this is what a firmware
    /// too old to implement the initialization tag looks like.
    FirmwareNotReady,
    /// Remapping the shared region non-cacheable was rejected — see
    /// [`mmu::Error`]. Only reachable if [`SharedMemory`] somehow isn't
    /// where its alignment says it should be.
    Mmu(mmu::Error),
    /// A handshake didn't complete in time: the firmware never answered a
    /// connect or service-open request.
    Timeout,
    /// A message was attempted before [`Vchiq::connect`] succeeded.
    NotConnected,
    /// No free service slot. This driver supports four open at once,
    /// against a protocol ceiling far higher — sized for its actual
    /// clients, MMAL being the only one and needing exactly one.
    TooManyServices,
    /// The firmware refused to open the requested service — nothing on
    /// the VideoCore is listening under that four-character code.
    ServiceRefused,
    /// A message longer than [`MAX_MESSAGE_SIZE`] was passed to
    /// [`Vchiq::send`].
    MessageTooLong,
    /// Every slot this side owns is full of messages the firmware hasn't
    /// consumed yet. The caller should [`Vchiq::poll`] (which returns
    /// consumed slots to the free list) and try again.
    OutOfSlots,
    /// A bulk transfer was submitted while the protocol's maximum of four
    /// were already in flight in that direction — `receive` says which.
    /// Since the firmware
    /// completes transfers it has been given, hitting this means its
    /// completions are not coming back: compare
    /// [`Stats::bulk_transmits`]/[`Stats::bulk_receives`] against their
    /// `_done` counterparts to see which direction stalled.
    TooManyBulks {
        /// `true` for a transfer into ARM memory, `false` for one out of
        /// it.
        receive: bool,
    },
    /// A bulk transfer's completion arrived with nothing outstanding to
    /// match it to, or the firmware sent a message type only a bulk
    /// *master* should ever receive. Either means this driver and the
    /// firmware have lost track of each other.
    Protocol,
}

impl From<crate::mailbox::Error> for Error {
    /// Wraps a mailbox failure as [`Error::Mailbox`].
    fn from(error: crate::mailbox::Error) -> Self {
        Error::Mailbox(error)
    }
}

/// An open service, as returned by [`Vchiq::open_service`] and named in
/// every call that acts on one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ServiceId(u16);

/// Running counts of what has crossed the transport, from
/// [`Vchiq::stats`].
///
/// A shared-memory protocol fails by going quiet — the peer simply stops
/// answering, with nothing on the wire to inspect and no error to report.
/// These are what makes that legible: comparing what was sent against what
/// came back says which half of an exchange stopped, which is the first
/// question to answer about any of it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// Messages written into this side's slots.
    pub messages_sent: u32,
    /// Messages read out of the firmware's slots, of every type.
    pub messages_received: u32,
    /// Service data messages among them.
    pub data_received: u32,
    /// Bulk transfers out of ARM memory submitted.
    pub bulk_transmits: u32,
    /// Completions received for them. Lagging behind
    /// [`Self::bulk_transmits`] by more than what is genuinely in flight
    /// means the firmware isn't finishing them.
    pub bulk_transmits_done: u32,
    /// Bulk transfers into ARM memory submitted.
    pub bulk_receives: u32,
    /// Completions received for them; see [`Self::bulk_transmits_done`].
    pub bulk_receives_done: u32,
    /// Messages of a type this driver doesn't act on (padding, and the
    /// modes listed in the module doc comment as unimplemented).
    pub messages_skipped: u32,
    /// The message type of the most recent of those — so a skipped
    /// message that turns out to matter can be identified.
    pub last_skipped_type: u32,
    /// Slots handed back to the firmware to write into again.
    pub slots_recycled: u32,
}

/// Something that arrived from the VideoCore, returned by
/// [`Vchiq::poll`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// The firmware completed the connection handshake.
    Connected,
    /// The firmware accepted a service open request.
    ServiceOpened(ServiceId),
    /// The firmware closed a service, either in answer to
    /// [`Vchiq::close_service`] or on its own initiative.
    ServiceClosed(ServiceId),
    /// A service message arrived and has been copied into the buffer
    /// passed to [`Vchiq::poll`].
    Message {
        /// Which service it is for.
        service: ServiceId,
        /// Length of the message. If this exceeds the buffer that was
        /// passed in, only that much was copied and the rest is lost —
        /// the message has already been consumed from its slot either
        /// way.
        len: usize,
    },
    /// A bulk transfer out of ARM memory finished.
    BulkTransmitDone {
        /// The `context` given to [`Vchiq::bulk_transmit`].
        context: u32,
        /// Bytes actually transferred.
        actual: usize,
    },
    /// A bulk transfer into ARM memory finished; the buffer has already
    /// been invalidated, so it can be read directly.
    BulkReceiveDone {
        /// The `context` given to [`Vchiq::bulk_receive`].
        context: u32,
        /// Bytes actually transferred.
        actual: usize,
    },
}

/// State of one of this driver's service ports.
#[derive(Clone, Copy, PartialEq, Eq)]
enum ServiceState {
    /// Unused port.
    Free,
    /// An open request has been sent, no answer yet.
    Opening,
    /// Open, with the peer's port number recorded.
    Open,
    /// Closed by either side.
    Closed,
}

/// One of this driver's service ports.
#[derive(Clone, Copy)]
struct Service {
    /// Where this port is in its life cycle.
    state: ServiceState,
    /// The peer's port number, learned from its open acknowledgement.
    remote_port: u16,
}

/// One in-flight bulk transfer, remembered so its completion — which
/// carries only a byte count — can be matched to the buffer it was for.
#[derive(Clone, Copy)]
struct Bulk {
    /// Address of the caller's buffer.
    address: u32,
    /// Its length in bytes.
    len: usize,
    /// The caller's own tag, handed back in the completion event.
    context: u32,
}

/// The VCHIQ transport.
pub struct Vchiq {
    /// Base address of the shared region. Identity-mapped, so this is both
    /// its ARM address and (after translation) its bus address.
    base: u32,
    /// Byte position this driver has consumed the firmware's message
    /// stream up to.
    rx_pos: u32,
    /// Byte position this driver has written its own message stream up to.
    /// The shared copy (`slave.tx_pos`) is only updated once a message is
    /// complete, so the firmware never sees a half-written one.
    local_tx_pos: u32,
    /// Address of the slot currently being written, or 0 before the first
    /// message.
    tx_data: u32,
    /// Address of the slot currently being read, or 0 between slots.
    rx_data: u32,
    /// Index of the slot [`Self::rx_data`] points into.
    rx_slot: usize,
    /// How far [`SharedState::slot_queue`] has been filled with slots this
    /// side may write — it grows as the firmware recycles them.
    slot_queue_available: u32,
    /// Whether the connection handshake has completed.
    connected: bool,
    /// This driver's service ports, indexed by local port number.
    services: [Service; MAX_SERVICES],
    /// Bulk transfers in flight, oldest first, in each direction. The
    /// firmware completes them in order, so the oldest entry is always the
    /// one a completion refers to.
    bulk_tx: [Option<Bulk>; MAX_BULKS],
    /// See [`Self::bulk_tx`].
    bulk_rx: [Option<Bulk>; MAX_BULKS],
    /// What has crossed the transport so far; see [`Stats`].
    stats: Stats,
    /// The shared region, held for as long as this driver lives. Accessed
    /// only through [`Self::base`]; the firmware writes it concurrently,
    /// so every access is volatile.
    _memory: &'static mut SharedMemory,
}

impl Vchiq {
    /// Brings the transport up: initializes the shared region, takes the
    /// caches off it, and hands it to the firmware.
    ///
    /// Returns once the firmware has filled in its own half of slot zero.
    /// [`Self::connect`] is the next step; nothing else works before it.
    ///
    /// `memory` must be a `static` (or otherwise permanent): the firmware
    /// keeps reading this region for as long as the board is up, so it can
    /// never be reused — which is also why this takes it by `&'static mut`
    /// rather than borrowing it.
    ///
    /// # Panics
    ///
    /// Never directly, but note that this is the point at which `memory`
    /// stops being cacheable. Code holding another reference into the same
    /// [`mmu::UNCACHED_GRANULE`] keeps working and merely gets slower; see
    /// [`mmu::set_uncached`]'s safety note.
    pub fn new(
        memory: &'static mut SharedMemory,
        mailbox: &mut Mailbox,
        timer: &Timer,
    ) -> Result<Self, Error> {
        let base = memory as *mut SharedMemory as u32;

        // Before anything is written through it: this region is shared with
        // a second bus master, so it must not be cached. See the module doc
        // comment for why maintenance-by-hand is not an option here.
        //
        // SAFETY: `memory` is a `&'static mut`, so this caller owns every
        // byte of it, and `SharedMemory` is exactly one granule, aligned to
        // one, so nothing else can be sharing it.
        unsafe { mmu::set_uncached(base as usize, SHARED_MEMORY_SIZE) }.map_err(Error::Mmu)?;

        let mut vchiq = Self {
            base,
            rx_pos: 0,
            local_tx_pos: 0,
            tx_data: 0,
            rx_data: 0,
            rx_slot: 0,
            slot_queue_available: 0,
            connected: false,
            services: [Service {
                state: ServiceState::Free,
                remote_port: 0,
            }; MAX_SERVICES],
            bulk_tx: [None; MAX_BULKS],
            bulk_rx: [None; MAX_BULKS],
            stats: Stats::default(),
            _memory: memory,
        };

        vchiq.init_slot_zero();

        // Everything above must be in memory before the firmware is told
        // where to look.
        barrier();

        let status = mailbox.vchiq_init(to_bus(base))?;
        if status != 0 {
            return Err(Error::SlotsRejected(status));
        }

        // The firmware fills in its own half asynchronously. A second is
        // far longer than it takes in practice; the point of the bound is
        // that a firmware without VCHIQ support fails rather than hangs.
        let deadline = timer.now_micros() + 1_000_000;
        while vchiq.read(vchiq.remote() + offset_of!(SharedState, initialised) as u32) == 0 {
            if timer.now_micros() > deadline {
                return Err(Error::FirmwareNotReady);
            }
        }

        Ok(vchiq)
    }

    /// Completes the connection handshake, after which services can be
    /// opened.
    ///
    /// Each side sends a connect message and waits for the other's. The
    /// firmware's answer is processed by the same [`Self::poll`] loop
    /// everything else goes through, so a message that arrives while
    /// waiting here is handled rather than dropped — though with no
    /// service open yet there is nothing that can arrive except the
    /// connect itself.
    pub fn connect(&mut self, timer: &Timer) -> Result<(), Error> {
        self.queue_message(MSG_CONNECT << TYPE_SHIFT, &[])?;

        let deadline = timer.now_micros() + 1_000_000;
        let mut scratch = [0u8; 0];
        while !self.connected {
            self.poll(&mut scratch)?;
            if timer.now_micros() > deadline {
                return Err(Error::Timeout);
            }
        }
        Ok(())
    }

    /// Opens the service named by `fourcc` — `*b"mmal"` for the
    /// multimedia framework — declaring the protocol version this client
    /// speaks and the oldest it can work with.
    ///
    /// The version pair is the *service's* own versioning, not VCHIQ's:
    /// the VideoCore refuses the open if its service is older than
    /// `version_min`, which is how a client finds out it is talking to
    /// firmware that predates whatever it needs.
    pub fn open_service(
        &mut self,
        fourcc: [u8; 4],
        version: u16,
        version_min: u16,
        timer: &Timer,
    ) -> Result<ServiceId, Error> {
        if !self.connected {
            return Err(Error::NotConnected);
        }

        let port = self
            .services
            .iter()
            .position(|service| service.state == ServiceState::Free)
            .ok_or(Error::TooManyServices)?;
        self.services[port] = Service {
            state: ServiceState::Opening,
            remote_port: 0,
        };

        // The open payload: four-character code, a client id used only for
        // the firmware's own logging, then the version pair.
        let mut payload = [0u8; 12];
        payload[0..4].copy_from_slice(&u32::from_be_bytes(fourcc).to_le_bytes());
        payload[4..8].copy_from_slice(&1u32.to_le_bytes());
        payload[8..10].copy_from_slice(&version.to_le_bytes());
        payload[10..12].copy_from_slice(&version_min.to_le_bytes());

        self.queue_message((MSG_OPEN << TYPE_SHIFT) | ((port as u32) << 12), &payload)?;

        let deadline = timer.now_micros() + 1_000_000;
        let mut scratch = [0u8; 0];
        loop {
            match self.services[port].state {
                ServiceState::Open => return Ok(ServiceId(port as u16)),
                // The firmware answers a service it doesn't have by
                // closing the port rather than by any explicit refusal.
                ServiceState::Closed => {
                    self.services[port].state = ServiceState::Free;
                    return Err(Error::ServiceRefused);
                }
                _ => {}
            }
            self.poll(&mut scratch)?;
            if timer.now_micros() > deadline {
                self.services[port].state = ServiceState::Free;
                return Err(Error::Timeout);
            }
        }
    }

    /// Closes a service. The firmware acknowledges with a close of its
    /// own, surfacing as [`Event::ServiceClosed`] from [`Self::poll`].
    pub fn close_service(&mut self, service: ServiceId) -> Result<(), Error> {
        let port = service.0 as usize;
        let remote = self.services[port].remote_port;
        self.services[port].state = ServiceState::Closed;
        self.queue_message(
            (MSG_CLOSE << TYPE_SHIFT) | ((port as u32) << 12) | u32::from(remote),
            &[],
        )
    }

    /// Sends a message to an open service.
    ///
    /// Fails with [`Error::OutOfSlots`] when every slot this side owns is
    /// still holding messages the firmware hasn't read. That is a
    /// back-pressure signal rather than a fault: [`Self::poll`] returns
    /// consumed slots to the free list, so the caller should poll and
    /// retry.
    pub fn send(&mut self, service: ServiceId, data: &[u8]) -> Result<(), Error> {
        let port = service.0 as usize;
        if self.services[port].state != ServiceState::Open {
            return Err(Error::NotConnected);
        }
        if data.len() > MAX_MESSAGE_SIZE {
            return Err(Error::MessageTooLong);
        }
        let remote = self.services[port].remote_port;
        self.queue_message(
            (MSG_DATA << TYPE_SHIFT) | ((port as u32) << 12) | u32::from(remote),
            data,
        )
    }

    /// Asks the firmware to DMA `len` bytes from `address` into its own
    /// memory, completing later as [`Event::BulkTransmitDone`] carrying
    /// `context`.
    ///
    /// This is how a payload too large to inline in a message is sent: the
    /// service protocol above (for MMAL, a "buffer from host" message)
    /// announces the transfer, and this carries the bytes.
    ///
    /// The buffer is cleaned out of this core's cache before the firmware
    /// is told about it, so a caller that has just filled it need do
    /// nothing further.
    ///
    /// # Safety
    ///
    /// The memory at `address` must stay valid, and must not be written by
    /// this core, until the matching [`Event::BulkTransmitDone`] arrives:
    /// the firmware reads it directly, at a time of its own choosing,
    /// after this returns.
    pub unsafe fn bulk_transmit(
        &mut self,
        service: ServiceId,
        address: u32,
        len: usize,
        context: u32,
    ) -> Result<(), Error> {
        clean_range(address, len);
        self.queue_bulk(service, address, len, context, PAGELIST_WRITE)
    }

    /// Asks the firmware to DMA `len` bytes into `address`, completing
    /// later as [`Event::BulkReceiveDone`] carrying `context`.
    ///
    /// The buffer's cache lines are dropped before the transfer starts and
    /// invalidated again when it completes, so by the time the event
    /// arrives the data can simply be read.
    ///
    /// # Safety
    ///
    /// The memory at `address` must stay valid, and must not be read or
    /// written by this core, until the matching
    /// [`Event::BulkReceiveDone`] arrives. It must also start on a
    /// 64-byte cache-line boundary and the caller must own every byte up
    /// to the end of the line containing `address + len`: the cache
    /// maintenance around the transfer works in whole lines, so a
    /// neighbour sharing the first or last line would lose writes.
    pub unsafe fn bulk_receive(
        &mut self,
        service: ServiceId,
        address: u32,
        len: usize,
        context: u32,
    ) -> Result<(), Error> {
        // Not just an invalidate: a dirty line anywhere in the buffer would
        // otherwise be written back over data the firmware has DMA'd in.
        clean_invalidate_range(address, len);
        self.queue_bulk(service, address, len, context, PAGELIST_READ)
    }

    /// Processes the firmware's message stream, returning the first thing
    /// that needs the caller's attention, or `None` when there is nothing
    /// pending.
    ///
    /// This is the only thing that moves the transport forward: it is what
    /// notices the firmware's messages, recycles the slots they arrived
    /// in, and completes bulk transfers. An application should call it in
    /// its main loop.
    ///
    /// A [`Event::Message`] is copied into `buffer`; anything longer than
    /// it is truncated, with the event still reporting the true length.
    pub fn poll(&mut self, buffer: &mut [u8]) -> Result<Option<Event>, Error> {
        // Slots the firmware has finished with become writable again.
        let recycle = self.read(self.local() + offset_of!(SharedState, slot_queue_recycle) as u32);
        self.slot_queue_available = recycle;

        loop {
            let tx_pos = self.read(self.remote() + offset_of!(SharedState, tx_pos) as u32);
            if self.rx_pos == tx_pos {
                return Ok(None);
            }

            if self.rx_data == 0 {
                // Starting a slot: which one is named by the firmware's own
                // queue, at the position the byte stream has reached.
                let queue_index = (self.rx_pos / SLOT_SIZE as u32) & SLOT_QUEUE_MASK;
                let slot = self.read(
                    self.remote() + offset_of!(SharedState, slot_queue) as u32 + queue_index * 4,
                ) as usize;
                self.rx_slot = slot;
                self.rx_data = self.base + (slot * SLOT_SIZE) as u32;
                // One reference for the slot as a whole, released once its
                // last message has been read — the protocol's guard against
                // recycling a slot mid-parse.
                self.write_slot_info(slot, 1, 0);
            }

            let header = self.rx_data + (self.rx_pos & SLOT_MASK);
            let msgid = self.read(header);
            let size = self.read(header + 4);
            let event = self.parse(msgid, size, header + HEADER_SIZE, buffer)?;

            self.rx_pos = self.rx_pos.wrapping_add(stride(size));
            if self.rx_pos & SLOT_MASK == 0 {
                self.release_slot(self.rx_slot);
                self.rx_data = 0;
            }

            if event.is_some() {
                return Ok(event);
            }
        }
    }

    /// What has crossed the transport since bring-up — see [`Stats`].
    pub fn stats(&self) -> Stats {
        self.stats
    }

    /// Acts on one received message, returning what (if anything) the
    /// caller should hear about.
    fn parse(
        &mut self,
        msgid: u32,
        size: u32,
        data: u32,
        buffer: &mut [u8],
    ) -> Result<Option<Event>, Error> {
        let message_type = msgid >> TYPE_SHIFT;
        let source_port = ((msgid >> 12) & 0xFFF) as u16;
        let destination_port = (msgid & 0xFFF) as usize;
        self.stats.messages_received += 1;

        match message_type {
            MSG_CONNECT => {
                self.connected = true;
                Ok(Some(Event::Connected))
            }
            MSG_OPENACK => {
                let Some(service) = self.services.get_mut(destination_port) else {
                    return Ok(None);
                };
                if service.state != ServiceState::Opening {
                    return Ok(None);
                }
                service.remote_port = source_port;
                service.state = ServiceState::Open;
                Ok(Some(Event::ServiceOpened(ServiceId(
                    destination_port as u16,
                ))))
            }
            MSG_CLOSE => {
                let Some(service) = self.services.get_mut(destination_port) else {
                    return Ok(None);
                };
                let was_open = service.state == ServiceState::Open;
                service.state = ServiceState::Closed;
                if was_open {
                    // Acknowledge a close the firmware started, so it can
                    // free its own end rather than waiting on this one.
                    self.queue_message(
                        (MSG_CLOSE << TYPE_SHIFT)
                            | ((destination_port as u32) << 12)
                            | u32::from(source_port),
                        &[],
                    )?;
                }
                Ok(Some(Event::ServiceClosed(ServiceId(
                    destination_port as u16,
                ))))
            }
            MSG_DATA => {
                if self.services.get(destination_port).map(|s| s.state) != Some(ServiceState::Open)
                {
                    return Ok(None);
                }
                let len = size as usize;
                let copied = len.min(buffer.len());
                for (index, byte) in buffer[..copied].iter_mut().enumerate() {
                    *byte = self.read_byte(data + index as u32);
                }
                self.stats.data_received += 1;
                Ok(Some(Event::Message {
                    service: ServiceId(destination_port as u16),
                    len,
                }))
            }
            MSG_BULK_TX_DONE | MSG_BULK_RX_DONE => {
                let receive = message_type == MSG_BULK_RX_DONE;
                let queue = if receive {
                    &mut self.bulk_rx
                } else {
                    &mut self.bulk_tx
                };
                // Completions arrive in submission order, so the oldest
                // outstanding transfer is always this one.
                let bulk = queue[0].take().ok_or(Error::Protocol)?;
                queue.rotate_left(1);

                // A negative count means the firmware aborted the transfer.
                let actual = self.read(data) as i32;
                let actual = if actual < 0 { 0 } else { actual as usize };

                if receive {
                    self.stats.bulk_receives_done += 1;
                    invalidate_range(bulk.address, bulk.len);
                    Ok(Some(Event::BulkReceiveDone {
                        context: bulk.context,
                        actual,
                    }))
                } else {
                    self.stats.bulk_transmits_done += 1;
                    Ok(Some(Event::BulkTransmitDone {
                        context: bulk.context,
                        actual,
                    }))
                }
            }
            // Bulk *requests* only ever go the other way: the VideoCore is
            // the bulk master, and this side has no DMA engine wired up to
            // service one. Receiving one means the two sides disagree about
            // which is which.
            MSG_BULK_TX | MSG_BULK_RX => Err(Error::Protocol),
            // Padding to the end of a slot, and the message types this
            // driver has no use for: synchronous-mode traffic, pause/resume,
            // and the power-management handshake. Skipped, not refused —
            // the firmware may send them regardless of whether anything
            // here acts on them.
            _ => {
                self.stats.messages_skipped += 1;
                self.stats.last_skipped_type = message_type;
                Ok(None)
            }
        }
    }

    /// Builds the page list for a bulk transfer, records it as
    /// outstanding, and tells the firmware to start it.
    fn queue_bulk(
        &mut self,
        service: ServiceId,
        address: u32,
        len: usize,
        context: u32,
        kind: u16,
    ) -> Result<(), Error> {
        let port = service.0 as usize;
        if self.services[port].state != ServiceState::Open {
            return Err(Error::NotConnected);
        }

        let receive = kind == PAGELIST_READ;
        let queue = if receive {
            &mut self.bulk_rx
        } else {
            &mut self.bulk_tx
        };
        let index = queue
            .iter()
            .position(|slot| slot.is_none())
            .ok_or(Error::TooManyBulks { receive })?;
        queue[index] = Some(Bulk {
            address,
            len,
            context,
        });
        if receive {
            self.stats.bulk_receives += 1;
        } else {
            self.stats.bulk_transmits += 1;
        }

        // One page list per in-flight transfer, receives after transmits.
        let list_index = if receive { MAX_BULKS + index } else { index };
        let list = self.base + (PAGELISTS_OFFSET + list_index * PAGELIST_SIZE) as u32;

        let offset = address & (PAGE_SIZE - 1);
        let pages = (len as u32 + offset).div_ceil(PAGE_SIZE);
        self.write(list + offset_of!(PageList, length) as u32, len as u32);
        // `kind` and `offset` are adjacent 16-bit fields.
        self.write(
            list + offset_of!(PageList, kind) as u32,
            u32::from(kind) | (offset << 16),
        );
        // Page-aligned bus address of the first page, with the number of
        // consecutive pages minus one packed into the low bits — the
        // encoding the firmware's DMA engine expects, and a single entry
        // because an identity-mapped buffer is physically contiguous.
        self.write(
            list + offset_of!(PageList, addrs) as u32,
            to_bus(address & !(PAGE_SIZE - 1)) | (pages - 1),
        );

        let remote = self.services[port].remote_port;
        let message_type = if receive { MSG_BULK_RX } else { MSG_BULK_TX };
        let mut payload = [0u8; 8];
        payload[0..4].copy_from_slice(&to_bus(list).to_le_bytes());
        payload[4..8].copy_from_slice(&(len as u32).to_le_bytes());
        self.queue_message(
            (message_type << TYPE_SHIFT) | ((port as u32) << 12) | u32::from(remote),
            &payload,
        )
    }

    /// Writes one message into this side's slot stream and rings the
    /// firmware's doorbell.
    fn queue_message(&mut self, msgid: u32, data: &[u8]) -> Result<(), Error> {
        let space = stride(data.len() as u32);
        let header = self.reserve_space(space)?;

        for (index, byte) in data.iter().enumerate() {
            self.write_byte(header + HEADER_SIZE + index as u32, *byte);
        }
        self.write(header, msgid);
        self.write(header + 4, data.len() as u32);

        // The message must be complete in memory before the position that
        // advertises it moves.
        barrier();
        self.write(
            self.local() + offset_of!(SharedState, tx_pos) as u32,
            self.local_tx_pos,
        );
        barrier();

        self.signal_remote(offset_of!(SharedState, trigger) as u32);
        self.stats.messages_sent += 1;
        Ok(())
    }

    /// Finds room for a `space`-byte message, moving to the next slot (and
    /// padding out the current one) when it won't fit in what's left.
    ///
    /// Returns the address to write the message header at.
    fn reserve_space(&mut self, space: u32) -> Result<u32, Error> {
        let mut tx_pos = self.local_tx_pos;
        let slot_space = SLOT_SIZE as u32 - (tx_pos & SLOT_MASK);

        if space > slot_space {
            // A message may not straddle slots, so the tail of this one
            // becomes a padding message the reader skips over.
            let header = self.tx_data + (tx_pos & SLOT_MASK);
            self.write(header, MSG_PADDING << TYPE_SHIFT);
            self.write(header + 4, slot_space - HEADER_SIZE);
            tx_pos = tx_pos.wrapping_add(slot_space);
        }

        if tx_pos & SLOT_MASK == 0 {
            if tx_pos / SLOT_SIZE as u32 == self.slot_queue_available {
                // Nothing free. Publish what has already been written and
                // nudge the firmware, so that polling and retrying has a
                // chance of finding a slot recycled.
                self.write(
                    self.local() + offset_of!(SharedState, tx_pos) as u32,
                    self.local_tx_pos,
                );
                barrier();
                self.signal_remote(offset_of!(SharedState, trigger) as u32);
                return Err(Error::OutOfSlots);
            }
            let queue_index = (tx_pos / SLOT_SIZE as u32) & SLOT_QUEUE_MASK;
            let slot = self
                .read(self.local() + offset_of!(SharedState, slot_queue) as u32 + queue_index * 4)
                as usize;
            self.tx_data = self.base + (slot * SLOT_SIZE) as u32;
        }

        self.local_tx_pos = tx_pos.wrapping_add(space);
        Ok(self.tx_data + (tx_pos & SLOT_MASK))
    }

    /// Hands a fully consumed slot back to the firmware to write into
    /// again.
    fn release_slot(&mut self, slot: usize) {
        let info = self.base + offset_of!(SlotZero, slots) as u32 + (slot * 4) as u32;
        let use_count = self.read_half(info);
        let release_count = self.read_half(info + 2) + 1;
        self.write_half(info + 2, release_count);
        if release_count != use_count {
            return;
        }

        let recycle_field = self.remote() + offset_of!(SharedState, slot_queue_recycle) as u32;
        let recycle = self.read(recycle_field);
        self.write(
            self.remote()
                + offset_of!(SharedState, slot_queue) as u32
                + (recycle & SLOT_QUEUE_MASK) * 4,
            slot as u32,
        );
        self.write(recycle_field, recycle.wrapping_add(1));
        barrier();
        self.signal_remote(offset_of!(SharedState, recycle) as u32);
        self.stats.slots_recycled += 1;
    }

    /// Fires the firmware's event at `event_offset` within its shared
    /// state, ringing the doorbell only if it asked to be interrupted.
    ///
    /// Skipping the bell when `armed` is clear is not an optimization this
    /// driver invented — it is the protocol's contract, and the same test
    /// keeps the firmware from interrupting *this* side, which never arms
    /// its own events (see the module doc comment).
    fn signal_remote(&mut self, event_offset: u32) {
        let event = self.remote() + event_offset;
        self.write(event + offset_of!(RemoteEvent, fired) as u32, 1);
        barrier();
        if self.read(event + offset_of!(RemoteEvent, armed) as u32) != 0 {
            // SAFETY: a plain MMIO store to the VideoCore's doorbell, which
            // takes any value; the address is fixed by the SoC memory map.
            unsafe { ((PERIPHERAL_BASE + DOORBELL_OFFSET) as *mut u32).write_volatile(0) };
        }
    }

    /// Fills in slot zero: the header the firmware validates, and this
    /// side's half of the shared state.
    fn init_slot_zero(&mut self) {
        for offset in (0..size_of::<SlotZero>() as u32).step_by(4) {
            self.write(self.base + offset, 0);
        }

        self.write(self.base + offset_of!(SlotZero, magic) as u32, MAGIC);
        // `version` and `version_min` are adjacent 16-bit fields.
        self.write(
            self.base + offset_of!(SlotZero, version) as u32,
            u32::from(VERSION) | (u32::from(VERSION_MIN) << 16),
        );
        self.write(
            self.base + offset_of!(SlotZero, slot_zero_size) as u32,
            size_of::<SlotZero>() as u32,
        );
        self.write(
            self.base + offset_of!(SlotZero, slot_size) as u32,
            SLOT_SIZE as u32,
        );
        self.write(
            self.base + offset_of!(SlotZero, max_slots) as u32,
            MAX_SLOTS as u32,
        );
        self.write(
            self.base + offset_of!(SlotZero, max_slots_per_side) as u32,
            MAX_SLOTS_PER_SIDE as u32,
        );
        self.write(
            self.base + offset_of!(SlotZero, platform_data) as u32,
            to_bus(self.base + FRAGMENTS_OFFSET as u32),
        );
        self.write(
            self.base + offset_of!(SlotZero, platform_data) as u32 + 4,
            MAX_FRAGMENTS as u32,
        );

        // The data slots are split down the middle, each side getting a
        // synchronous slot and then a run of ordinary ones. Which half
        // belongs to whom is fixed: the VideoCore is the master.
        let first_data_slot = SLOT_ZERO_SLOTS as u32;
        let half = DATA_SLOTS as u32 / 2;
        let master = self.remote();
        self.write(
            master + offset_of!(SharedState, slot_sync) as u32,
            first_data_slot,
        );
        self.write(
            master + offset_of!(SharedState, slot_first) as u32,
            first_data_slot + 1,
        );
        self.write(
            master + offset_of!(SharedState, slot_last) as u32,
            first_data_slot + half - 1,
        );

        let local = self.local();
        let slave_sync = first_data_slot + half;
        self.write(
            local + offset_of!(SharedState, slot_sync) as u32,
            slave_sync,
        );
        self.write(
            local + offset_of!(SharedState, slot_first) as u32,
            slave_sync + 1,
        );
        self.write(
            local + offset_of!(SharedState, slot_last) as u32,
            first_data_slot + DATA_SLOTS as u32 - 1,
        );

        // This side's write queue starts as every slot it owns, in order.
        for (position, slot) in (slave_sync + 1..first_data_slot + DATA_SLOTS as u32).enumerate() {
            self.write(
                local + offset_of!(SharedState, slot_queue) as u32 + (position as u32) * 4,
                slot,
            );
            self.slot_queue_available += 1;
        }
        self.write(
            local + offset_of!(SharedState, slot_queue_recycle) as u32,
            self.slot_queue_available,
        );

        // The synchronous slot starts empty, and its release event starts
        // fired — the state the protocol expects even though nothing here
        // sends synchronous messages.
        self.write(
            self.base + (slave_sync * SLOT_SIZE as u32),
            MSG_PADDING << TYPE_SHIFT,
        );
        self.write(
            local
                + offset_of!(SharedState, sync_release) as u32
                + offset_of!(RemoteEvent, fired) as u32,
            1,
        );

        // Last, and only once everything above is in place: the flag the
        // firmware reads to decide this side is ready.
        barrier();
        self.write(local + offset_of!(SharedState, initialised) as u32, 1);
    }

    /// Address of this side's half of the shared state.
    fn local(&self) -> u32 {
        self.base + offset_of!(SlotZero, slave) as u32
    }

    /// Address of the firmware's half of the shared state.
    fn remote(&self) -> u32 {
        self.base + offset_of!(SlotZero, master) as u32
    }

    /// Sets a slot's reference counts, which live packed as two 16-bit
    /// fields.
    fn write_slot_info(&mut self, slot: usize, use_count: u16, release_count: u16) {
        let info = self.base + offset_of!(SlotZero, slots) as u32 + (slot * 4) as u32;
        self.write(
            info,
            u32::from(use_count) | (u32::from(release_count) << 16),
        );
    }

    // The shared region is mapped non-cacheable (see the module doc
    // comment) but is still ordinary memory, so the compiler would happily
    // reorder, merge or elide accesses to it. Every one goes through these
    // volatile helpers instead. Addresses are always in the region, which
    // is identity-mapped and 2MB-aligned, so alignment is by construction.

    /// Reads a 32-bit field of the shared region.
    fn read(&self, address: u32) -> u32 {
        // SAFETY: `address` is within the shared region this driver owns,
        // and 32-bit fields in it are 4-byte aligned by layout.
        unsafe { (address as *const u32).read_volatile() }
    }

    /// Writes a 32-bit field of the shared region.
    fn write(&mut self, address: u32, value: u32) {
        // SAFETY: as `read`.
        unsafe { (address as *mut u32).write_volatile(value) };
    }

    /// Reads a 16-bit field of the shared region.
    fn read_half(&self, address: u32) -> u16 {
        // SAFETY: as `read`, for a 2-byte-aligned field.
        unsafe { (address as *const u16).read_volatile() }
    }

    /// Writes a 16-bit field of the shared region.
    fn write_half(&mut self, address: u32, value: u16) {
        // SAFETY: as `read_half`.
        unsafe { (address as *mut u16).write_volatile(value) };
    }

    /// Reads one byte of message data.
    fn read_byte(&self, address: u32) -> u8 {
        // SAFETY: as `read`, with no alignment requirement.
        unsafe { (address as *const u8).read_volatile() }
    }

    /// Writes one byte of message data.
    fn write_byte(&mut self, address: u32, value: u8) {
        // SAFETY: as `read_byte`.
        unsafe { (address as *mut u8).write_volatile(value) };
    }
}

/// Offset of the VideoCore's doorbell register (`BELL2`) within the
/// peripheral block — the same block the property mailbox's own registers
/// sit in, a few words below them. Not in either PAC's SVD, so this driver
/// pokes the address directly, as `dma.rs` and `v3d.rs` do for their own
/// unmodelled blocks.
const DOORBELL_OFFSET: u32 = 0x0000_B848;

/// Bytes one message occupies in a slot: its data plus a header, rounded
/// up so the next header stays 8-byte aligned.
fn stride(size: u32) -> u32 {
    (size + HEADER_SIZE + HEADER_SIZE - 1) & !(HEADER_SIZE - 1)
}

/// Translates a plain ARM physical address to the VideoCore's
/// `0xC000_0000` "direct, uncached" bus alias — the window a bus master
/// reads and writes RAM through without going via the VideoCore's own L2
/// cache. Mirrors `mailbox.rs`/`dma.rs`/`v3d.rs`'s own translations; kept
/// local here so this driver stays self-contained the way the other
/// physical-poke drivers do.
fn to_bus(physical_address: u32) -> u32 {
    (physical_address & 0x3FFF_FFFF) | 0xC000_0000
}
