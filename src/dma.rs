//! Blocking driver for the BCM2835/2836/2837 DMA controller.
//!
//! The SoC has a single DMA controller exposing 16 independent channels
//! that move data between memory and peripherals (or memory to memory)
//! without the CPU copying it word by word. Each channel is programmed by
//! a *control block* — an eight-word descriptor in RAM describing one
//! transfer (what to move, from where, to where, how much) — whose bus
//! address is written into the channel's `CONBLK_AD` register; setting the
//! channel `ACTIVE` then walks the control block (and any it chains to via
//! `NEXTCONBLK`) to completion.
//!
//! Channels come in two flavours. Channels 0–6 are full DMA engines
//! (30-bit transfer lengths, 2D strided mode, the widest bursts); channels
//! 7–14 are "DMA Lite" engines with a reduced feature set and a 16-bit
//! length limit. Channel 15 lives in a separate physical address region
//! and is not exposed here. This driver drives the common 1D transfer that
//! both flavours support; [`Channel::memcpy`](crate::dma::Channel::memcpy)
//! rejects a transfer too long
//! for the channel it runs on.
//!
//! # Addresses and cache coherence
//!
//! Like the mailbox, SD, and USB paths, the DMA engine is a bus master
//! that reads and writes RAM directly, bypassing this core's data cache.
//! Two consequences the driver handles for a memory-to-memory copy, and
//! that a caller must keep in mind:
//!
//! - Addresses inside a control block (and in `CONBLK_AD`) are *VideoCore
//!   bus* addresses, not plain ARM physical addresses — this driver
//!   translates through the `0xC000_0000` "direct, uncached" alias (see
//!   `to_bus`), the same window `mailbox.rs` uses, so the engine reaches
//!   RAM without going through the L2 the GPU owns.
//! - Before a transfer the driver cleans (writes back) the source buffer,
//!   the destination buffer, and the control block so the engine sees
//!   current data and no dirty line can later write back over the result;
//!   after it, it invalidates the destination so this core's next read
//!   comes from RAM rather than a stale cached copy. Buffers should
//!   therefore be cache-line aligned and padded — a clean or invalidate
//!   covers whole lines, so a buffer sharing a line with unrelated data
//!   could write that data back or drop it.
//!
//! # Sharing with the firmware
//!
//! On a running Raspberry Pi the VideoCore firmware owns some DMA channels
//! for its own use (framebuffer scanout, audio) and publishes the set left
//! for the ARM as a channel mask. This driver does not consult that mask —
//! nothing here reserves a channel against the firmware — so an
//! application that lets the firmware keep running should pick a channel
//! the firmware is not using. A bare-metal program that has taken the
//! machine over (the usual case for this crate) can use any channel.
//!
//! The controller isn't modelled in `bcm2837-lpa`'s SVD, so this pokes its
//! known physical addresses directly rather than going through the PAC —
//! the same approach `rng.rs`/`uart.rs` take for blocks SVD omits.
//! Register layout and the control-block format follow the BCM2835 ARM
//! Peripherals datasheet (§4, "DMA Controller").

use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use crate::cache;

/// DMA controller base address: `crate::soc::PERIPHERAL_BASE` (BCM2836/2837
/// or, under the `bcm2711` feature, BCM2711) plus the block's `0x7000`
/// offset, same on both -- the DMA controller is unchanged IP, just
/// relocated with the rest of the peripheral block.
const DMA_BASE: usize = crate::soc::PERIPHERAL_BASE as usize + 0x7000;
/// Stride between adjacent channels' register blocks.
const CHANNEL_STRIDE: usize = 0x100;
/// Global channel-enable register (`DMA_BASE + 0xFF0`): bit `N` powers up
/// channel `N`'s engine. A channel does nothing until its bit is set.
const DMA_ENABLE: *mut u32 = (DMA_BASE + 0xff0) as *mut u32;

/// Control and Status register offset within a channel's block.
const CS: usize = 0x00;
/// Control Block Address register offset — the bus address of the control
/// block the channel should execute.
const CONBLK_AD: usize = 0x04;
/// Debug register offset — carries the sticky per-channel error flags.
const DEBUG: usize = 0x20;

/// `CS.ACTIVE`: set by software to start the channel; cleared by hardware
/// when the transfer (control-block chain) completes.
const CS_ACTIVE: u32 = 1 << 0;
/// `CS.END`: set by hardware when a control block finishes; write-1 to
/// clear.
const CS_END: u32 = 1 << 1;
/// `CS.INT`: set by hardware on a control block whose `TI.INTEN` is set;
/// write-1 to clear.
const CS_INT: u32 = 1 << 2;
/// `CS.ERROR`: set by hardware when the channel has hit an error (details
/// in the debug register).
const CS_ERROR: u32 = 1 << 8;
/// `CS.RESET`: write-1 to reset the channel; self-clears when done.
const CS_RESET: u32 = 1 << 31;

/// The debug register's three write-1-to-clear error flags (read-last-not-
/// set, FIFO error, read error) as one mask, used both to check for and to
/// clear them.
const DEBUG_ERRORS: u32 = 0b111;

/// `TI.WAIT_RESP`: wait for the AXI write response of each write before
/// moving on — the safe default for a memory-to-memory copy so the
/// transfer isn't reported complete before the data has actually landed.
const TI_WAIT_RESP: u32 = 1 << 3;
/// `TI.DEST_INC`: increment the destination address after each transfer.
const TI_DEST_INC: u32 = 1 << 4;
/// `TI.DEST_DREQ`: gate each write on the destination peripheral's DREQ,
/// so the engine only pushes a word when the peripheral (e.g. the PWM
/// FIFO) signals it has room. This is what paces a memory-to-peripheral
/// transfer to the peripheral's own rate instead of running flat out.
const TI_DEST_DREQ: u32 = 1 << 6;
/// `TI.SRC_INC`: increment the source address after each transfer.
const TI_SRC_INC: u32 = 1 << 8;
/// `TI.SRC_DREQ`: gate each read on the source peripheral's DREQ, so the
/// engine only pulls a word when the peripheral (e.g. the EMMC read FIFO)
/// signals it has data ready. The read-side mirror of [`TI_DEST_DREQ`],
/// pacing a peripheral-to-memory transfer to the peripheral's own rate.
const TI_SRC_DREQ: u32 = 1 << 10;
/// Bit offset of `TI.PERMAP`, the 5-bit field naming which peripheral's
/// DREQ paces the transfer (used together with [`TI_DEST_DREQ`]).
const TI_PERMAP_SHIFT: u32 = 16;
/// `TI.NO_WIDE_BURSTS`: never combine writes into a wide (>1 word) burst.
/// A DREQ-paced peripheral write drains the FIFO a word at a time, so
/// there is nothing to gain from bursting and asking for one can stall
/// the transfer against a peripheral that only accepts single words.
const TI_NO_WIDE_BURSTS: u32 = 1 << 26;

/// Highest channel number this driver exposes. Channel 15 lives in a
/// separate address region and is deliberately left out.
const MAX_CHANNEL: u8 = 14;
/// First "DMA Lite" channel. Channels at or above this have a 16-bit
/// transfer-length limit; the full channels below it allow 30 bits.
const FIRST_LITE_CHANNEL: u8 = 7;
/// Largest single-transfer byte count a full channel (0–6) accepts —
/// `TXFR_LEN` is 30 bits wide in 1D mode.
const MAX_LEN_FULL: usize = (1 << 30) - 1;
/// Largest single-transfer byte count a lite channel (7–14) accepts —
/// `TXFR_LEN` is only 16 bits there.
const MAX_LEN_LITE: usize = (1 << 16) - 1;

/// One DMA channel's control block: the eight-word, 32-byte-aligned
/// descriptor the engine reads to run a transfer. The trailing two words
/// are reserved and must be zero. Alignment matters: `CONBLK_AD` ignores
/// the low five address bits, so the block must sit on a 32-byte boundary.
#[repr(C, align(32))]
struct ControlBlock {
    ti: u32,
    source_ad: u32,
    dest_ad: u32,
    txfr_len: u32,
    stride: u32,
    nextconblk: u32,
    _reserved: [u32; 2],
}

impl ControlBlock {
    /// An all-zero control block, used to initialise a channel's inline
    /// storage before its first transfer programs it.
    const EMPTY: ControlBlock = ControlBlock {
        ti: 0,
        source_ad: 0,
        dest_ad: 0,
        txfr_len: 0,
        stride: 0,
        nextconblk: 0,
        _reserved: [0; 2],
    };
}

/// Builds the control block for a memory-to-peripheral write: source
/// increments, destination fixed at `dest_bus`, each word gated on the
/// peripheral's `dreq`, chaining to `next_bus` (`0` ends the chain).
/// Shared by [`Channel::write_peripheral`] and [`Channel::stream_peripheral`].
fn peripheral_write_cb(
    src_bus: u32,
    len_bytes: u32,
    dreq: u8,
    dest_bus: u32,
    next_bus: u32,
) -> ControlBlock {
    ControlBlock {
        ti: TI_SRC_INC
            | TI_DEST_DREQ
            | TI_WAIT_RESP
            | TI_NO_WIDE_BURSTS
            | ((dreq as u32) << TI_PERMAP_SHIFT),
        source_ad: src_bus,
        dest_ad: dest_bus,
        txfr_len: len_bytes,
        stride: 0,
        nextconblk: next_bus,
        _reserved: [0; 2],
    }
}

/// Builds the control block for a peripheral-to-memory read: source fixed
/// at `src_bus` (the peripheral's data register), destination increments,
/// each word gated on the peripheral's `dreq`, no chaining. Used by
/// [`Channel::copy_from_peripheral`].
fn peripheral_read_cb(src_bus: u32, len_bytes: u32, dreq: u8, dest_bus: u32) -> ControlBlock {
    ControlBlock {
        ti: TI_DEST_INC
            | TI_SRC_DREQ
            | TI_WAIT_RESP
            | TI_NO_WIDE_BURSTS
            | ((dreq as u32) << TI_PERMAP_SHIFT),
        source_ad: src_bus,
        dest_ad: dest_bus,
        txfr_len: len_bytes,
        stride: 0,
        nextconblk: 0,
        _reserved: [0; 2],
    }
}

/// Errors from a DMA transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The source and destination slices had different lengths, so there
    /// is no unambiguous byte count to transfer.
    LengthMismatch,
    /// The requested byte count exceeds what the channel's `TXFR_LEN`
    /// field can express (30 bits on a full channel, 16 on a lite one).
    TooLong,
    /// The channel reported a hardware error (its `CS.ERROR` or one of the
    /// debug register's error flags was set) partway through the transfer.
    Transfer,
}

/// The DMA controller, handed out one [`Channel`] at a time.
///
/// There is no PAC singleton to take ownership of (the controller isn't in
/// `bcm2837-lpa`), so — like [`rng::Rng`](crate::rng::Rng) — `new` is safe
/// and constructs from nothing. Unlike the RNG, though, two owners of the
/// *same* channel would corrupt each other's transfers, so channels are
/// vended through [`channel`](Dma::channel), which tracks which are taken
/// and refuses to hand the same one out twice.
pub struct Dma {
    /// Bitmask of channels already vended by [`channel`](Dma::channel).
    taken: u16,
}

impl Dma {
    /// Creates a handle to the DMA controller.
    pub fn new() -> Self {
        Self { taken: 0 }
    }

    /// Takes ownership of channel `id`, powering up its engine.
    ///
    /// Returns `None` if `id` is above `MAX_CHANNEL` or the channel has
    /// already been handed out. Channels 0–6 are full engines; 7–14 are
    /// the reduced "DMA Lite" engines (see the module docs). See the module
    /// docs, too, on picking a channel the firmware isn't using.
    pub fn channel(&mut self, id: u8) -> Option<Channel> {
        if id > MAX_CHANNEL || self.taken & (1 << id) != 0 {
            return None;
        }
        self.taken |= 1 << id;
        // Power up this channel's engine (write-1 sets, and leaves any
        // other already-enabled channels untouched).
        unsafe {
            let enable = read_volatile(DMA_ENABLE);
            write_volatile(DMA_ENABLE, enable | (1 << id));
        }
        Some(Channel {
            id,
            cb: [ControlBlock::EMPTY; 2],
        })
    }
}

impl Default for Dma {
    /// Equivalent to [`Dma::new`].
    fn default() -> Self {
        Self::new()
    }
}

/// One owned DMA channel, carrying the control-block storage its transfers
/// execute from.
pub struct Channel {
    /// This channel's number (0–14), fixing which register block it drives.
    id: u8,
    /// Storage for the control block(s) programmed on each transfer. Held
    /// inline so their addresses are stable and the driver can hand the
    /// engine their bus addresses; the identity-mapped MMU makes these
    /// virtual addresses equal to the physical ones the engine needs.
    /// A single-block transfer ([`Channel::memcpy`]/
    /// [`Channel::write_peripheral`]) uses only `cb[0]`; the double-
    /// buffered [`Channel::stream_peripheral`] chains `cb[0]` and `cb[1]`.
    cb: [ControlBlock; 2],
}

impl Channel {
    /// Copies `src` into `dest` over this channel, blocking until the
    /// transfer completes.
    ///
    /// Returns [`Error::LengthMismatch`] if the slices differ in length and
    /// [`Error::TooLong`] if the byte count exceeds the channel's limit; an
    /// empty copy is a no-op. Handles cache maintenance around the transfer
    /// (see the module docs) — but the caller is responsible for buffer
    /// alignment: both slices should be cache-line aligned and padded so
    /// the destination invalidate doesn't discard neighbouring data.
    pub fn memcpy(&mut self, dest: &mut [u8], src: &[u8]) -> Result<(), Error> {
        if src.len() != dest.len() {
            return Err(Error::LengthMismatch);
        }
        let len = src.len();
        if len == 0 {
            return Ok(());
        }
        if len > self.max_len() {
            return Err(Error::TooLong);
        }

        let src_phys = src.as_ptr() as usize as u32;
        let dest_phys = dest.as_mut_ptr() as usize as u32;

        // Flush the source out of this core's cache so the engine reads the
        // data actually written, not a stale line still sitting in RAM.
        cache::clean_range(src_phys, len);
        // Flush the destination too, even though the engine is about to
        // overwrite it: if the caller had dirtied these lines (e.g. just
        // wrote the buffer), an opportunistic write-back landing *after* the
        // transfer would clobber the DMA's result in RAM with the stale
        // cached data. Cleaning here leaves no dirty line that can do that;
        // the post-transfer invalidate below then makes this core re-read
        // the engine's result from RAM.
        cache::clean_range(dest_phys, len);

        self.cb[0] = ControlBlock {
            ti: TI_SRC_INC | TI_DEST_INC | TI_WAIT_RESP,
            source_ad: to_bus(src_phys),
            dest_ad: to_bus(dest_phys),
            txfr_len: len as u32,
            stride: 0,
            nextconblk: 0,
            _reserved: [0; 2],
        };
        // The engine fetches the control block from RAM too, so flush it as
        // well before pointing the channel at it.
        let cb_phys = (&self.cb[0] as *const ControlBlock) as usize as u32;
        cache::clean_range(cb_phys, core::mem::size_of::<ControlBlock>());

        // Point the channel at the control block and start it, then block
        // until the engine drops ACTIVE (transfer done) or flags an error.
        self.start(to_bus(cb_phys));
        self.wait_blocking()?;

        // The engine wrote `dest` in RAM behind this core's cache; drop the
        // stale lines so the next read comes from RAM.
        cache::invalidate_range(dest_phys, len);
        Ok(())
    }

    /// Reads `dest.len()` bytes from the fixed peripheral register at *bus*
    /// address `src_bus`, paced by the peripheral's DREQ number `dreq`, into
    /// `dest` — blocking until the transfer completes. The peripheral-to-
    /// memory mirror of [`copy_to_peripheral`](Self::copy_to_peripheral), and
    /// the read side of the DREQ-paced FIFO transfers a block device like the
    /// SD/EMMC controller needs.
    ///
    /// The source address does not increment (every word is pulled from the
    /// same register) while the destination increments across `dest`; each
    /// read waits on `dreq`, so the transfer runs at the peripheral's rate
    /// rather than flat out. `src_bus` is a VideoCore bus address (e.g.
    /// `0x7E30_0020` for the EMMC `DATA` register), not an ARM physical
    /// address; the caller supplies it already translated so this method
    /// stays peripheral-agnostic.
    ///
    /// Handles cache maintenance around the transfer (cleaning `dest` before
    /// so a stale dirty line can't later write back over the engine's result,
    /// invalidating it after so this core re-reads from RAM). Like
    /// [`memcpy`](Self::memcpy), `dest` should be cache-line aligned and
    /// padded so that invalidate doesn't discard neighbouring data, and its
    /// length should be a multiple of the peripheral's word size (4 bytes for
    /// a 32-bit FIFO). Returns [`Error::TooLong`] if `dest` exceeds the
    /// channel's length limit; an empty `dest` is a no-op.
    pub fn copy_from_peripheral(
        &mut self,
        dest: &mut [u8],
        dreq: u8,
        src_bus: u32,
    ) -> Result<(), Error> {
        let len = dest.len();
        if len == 0 {
            return Ok(());
        }
        if len > self.max_len() {
            return Err(Error::TooLong);
        }

        let dest_phys = dest.as_mut_ptr() as usize as u32;
        // Clean the destination first: a dirty line the caller left could
        // otherwise write back over the engine's result after the transfer
        // (same reasoning as `memcpy`'s destination clean).
        cache::clean_range(dest_phys, len);

        self.cb[0] = peripheral_read_cb(src_bus, len as u32, dreq, to_bus(dest_phys));
        let cb_phys = (&self.cb[0] as *const ControlBlock) as usize as u32;
        cache::clean_range(cb_phys, core::mem::size_of::<ControlBlock>());

        self.start(to_bus(cb_phys));
        self.wait_blocking()?;

        cache::invalidate_range(dest_phys, len);
        Ok(())
    }

    /// Writes `src` to the fixed peripheral register at *bus* address
    /// `dest_bus`, paced by the peripheral's DREQ number `dreq` — blocking
    /// until the transfer completes. The blocking, one-shot counterpart to
    /// [`write_peripheral`](Self::write_peripheral) (which returns a
    /// background guard for cyclic/streaming audio): this is the shape a
    /// block device like the SD/EMMC controller needs, where the caller
    /// wants the whole transfer done before it moves on.
    ///
    /// The destination address does not increment (every word lands in the
    /// same register) while the source increments across `src`; each write
    /// waits on `dreq`. `dest_bus` is a VideoCore bus address, as for
    /// [`copy_from_peripheral`](Self::copy_from_peripheral). Cleans `src`
    /// before the transfer so the engine reads what the caller wrote; `src`
    /// should be cache-line aligned and its length a multiple of the
    /// peripheral's word size. Returns [`Error::TooLong`] if `src` exceeds
    /// the channel's length limit; an empty `src` is a no-op.
    pub fn copy_to_peripheral(&mut self, src: &[u8], dreq: u8, dest_bus: u32) -> Result<(), Error> {
        let len = src.len();
        if len == 0 {
            return Ok(());
        }
        if len > self.max_len() {
            return Err(Error::TooLong);
        }

        let src_phys = src.as_ptr() as usize as u32;
        // The engine reads `src` from RAM, so flush this core's cached copy
        // out first — same reasoning as `write_peripheral`'s source clean.
        cache::clean_range(src_phys, len);

        self.cb[0] = peripheral_write_cb(to_bus(src_phys), len as u32, dreq, dest_bus, 0);
        let cb_phys = (&self.cb[0] as *const ControlBlock) as usize as u32;
        cache::clean_range(cb_phys, core::mem::size_of::<ControlBlock>());

        self.start(to_bus(cb_phys));
        self.wait_blocking()
    }

    /// Blocks until the channel drops `CS.ACTIVE` (the control-block chain
    /// completed), returning [`Error::Transfer`] if it finished with
    /// `CS.ERROR` or a debug-register error flag set. Shared by the blocking
    /// transfers ([`memcpy`](Self::memcpy),
    /// [`copy_from_peripheral`](Self::copy_from_peripheral),
    /// [`copy_to_peripheral`](Self::copy_to_peripheral)).
    fn wait_blocking(&self) -> Result<(), Error> {
        unsafe {
            let cs = self.reg(CS);
            loop {
                let status = read_volatile(cs);
                if status & CS_ACTIVE == 0 {
                    if status & CS_ERROR != 0 || read_volatile(self.reg(DEBUG)) & DEBUG_ERRORS != 0
                    {
                        return Err(Error::Transfer);
                    }
                    return Ok(());
                }
                spin_loop();
            }
        }
    }

    /// Starts a memory-to-peripheral transfer, streaming `src` word by word
    /// into the fixed peripheral register at *bus* address `dest_bus`, paced
    /// by the peripheral's DREQ number `dreq`. Returns immediately with a
    /// [`Transfer`] guard while the engine runs in the background — unlike
    /// [`memcpy`](Self::memcpy), which blocks.
    ///
    /// The destination address does not increment (every word lands in the
    /// same register) and each write waits on `dreq`, so the transfer runs
    /// at the peripheral's rate rather than flat out — the shape a FIFO-fed
    /// peripheral like PWM audio or I2S needs. `dest_bus` is a VideoCore bus
    /// address (e.g. `0x7E20_C018` for the PWM FIFO), not an ARM physical
    /// address; the caller supplies it already translated so this method
    /// stays peripheral-agnostic.
    ///
    /// When `cyclic` is true the control block chains back to itself, so the
    /// engine replays `src` forever (a looping tone or ring buffer) until the
    /// returned guard is dropped or [`Transfer::stop`]ped; when false it runs
    /// once and then drops `CS.ACTIVE` (poll [`Transfer::is_complete`]).
    ///
    /// Returns [`Error::TooLong`] if `src` (in bytes) exceeds the channel's
    /// `TXFR_LEN` limit; an empty `src` returns `Ok` with an already-
    /// complete guard, there being nothing to stream. `src` should be
    /// cache-line aligned so cleaning it doesn't touch unrelated dirty
    /// lines — but unlike [`memcpy`](Self::memcpy) it needs no trailing
    /// padding: this path only *cleans* (writes back) the source and never
    /// invalidates it, so over-covering the last line is harmless.
    pub fn write_peripheral<'a>(
        &'a mut self,
        src: &'a [u32],
        dreq: u8,
        dest_bus: u32,
        cyclic: bool,
    ) -> Result<Transfer<'a>, Error> {
        let len = core::mem::size_of_val(src);
        if len == 0 {
            return Ok(Transfer { channel: self });
        }
        if len > self.max_len() {
            return Err(Error::TooLong);
        }

        let src_phys = src.as_ptr() as usize as u32;
        // The engine reads `src` from RAM, so flush this core's cached copy
        // out first — same reasoning as `memcpy`'s source clean.
        cache::clean_range(src_phys, len);

        // The control block's own bus address, needed up front so a cyclic
        // transfer can point `nextconblk` back at it.
        let cb_phys = (&self.cb[0] as *const ControlBlock) as usize as u32;
        let cb_bus = to_bus(cb_phys);

        self.cb[0] = peripheral_write_cb(
            to_bus(src_phys),
            len as u32,
            dreq,
            dest_bus,
            if cyclic { cb_bus } else { 0 },
        );
        // Flush the control block itself, then point the channel at it.
        cache::clean_range(cb_phys, core::mem::size_of::<ControlBlock>());
        self.start(cb_bus);

        Ok(Transfer { channel: self })
    }

    /// Starts a double-buffered ("ping-pong") memory-to-peripheral stream
    /// over the two `buffers`, paced by the peripheral's `dreq` into the
    /// fixed register at bus address `dest_bus`, and returns a [`Stream`]
    /// handle for feeding it. Like [`write_peripheral`](Self::write_peripheral)
    /// this returns immediately and runs in the background; unlike its
    /// cyclic mode — which replays one fixed buffer forever — a stream lets
    /// the caller supply fresh samples continuously, so the audio can
    /// change over time.
    ///
    /// The engine plays `buffers[0]`, then `buffers[1]`, then `buffers[0]`
    /// again, … chaining forever. While it reads one buffer the caller
    /// refills the other via [`Stream::feed`], so playback is gapless as
    /// long as each refill keeps ahead of the engine. `buffers` should hold
    /// the first two chunks of audio already — the engine starts playing
    /// immediately. Both buffers are cleaned before the transfer starts.
    ///
    /// `dest_bus` is a VideoCore bus address, as for
    /// [`write_peripheral`](Self::write_peripheral); each buffer should be
    /// cache-line aligned (same reasoning). Returns [`Error::TooLong`] if
    /// either buffer's byte length exceeds the channel's `TXFR_LEN` limit,
    /// or [`Error::LengthMismatch`] if either buffer is empty.
    pub fn stream_peripheral<'a>(
        &'a mut self,
        buffers: [&'a mut [u32]; 2],
        dreq: u8,
        dest_bus: u32,
    ) -> Result<Stream<'a>, Error> {
        let [len0, len1] = [
            core::mem::size_of_val(&*buffers[0]),
            core::mem::size_of_val(&*buffers[1]),
        ];
        if len0 == 0 || len1 == 0 {
            return Err(Error::LengthMismatch);
        }
        if len0 > self.max_len() || len1 > self.max_len() {
            return Err(Error::TooLong);
        }

        // Flush the initial contents of both buffers so the engine reads
        // what the caller wrote, not stale RAM.
        let src_bus = [
            to_bus(buffers[0].as_ptr() as usize as u32),
            to_bus(buffers[1].as_ptr() as usize as u32),
        ];
        cache::clean_range(buffers[0].as_ptr() as usize as u32, len0);
        cache::clean_range(buffers[1].as_ptr() as usize as u32, len1);

        // Each control block chains to the other, so the engine ping-pongs
        // between the two buffers indefinitely.
        let cb_bus = [
            to_bus((&self.cb[0] as *const ControlBlock) as usize as u32),
            to_bus((&self.cb[1] as *const ControlBlock) as usize as u32),
        ];
        self.cb[0] = peripheral_write_cb(src_bus[0], len0 as u32, dreq, dest_bus, cb_bus[1]);
        self.cb[1] = peripheral_write_cb(src_bus[1], len1 as u32, dreq, dest_bus, cb_bus[0]);
        cache::clean_range(
            (&self.cb as *const [ControlBlock; 2]) as usize as u32,
            core::mem::size_of::<[ControlBlock; 2]>(),
        );

        self.start(cb_bus[0]);

        Ok(Stream {
            channel: self,
            buffers,
            cb_bus,
            next: 0,
        })
    }

    /// Resets the channel to a clean state, clears any stale
    /// completion/error status, points it at the control block at bus
    /// address `cb_bus`, and sets it running. Shared startup sequence for
    /// every transfer kind; the caller decides whether to then block on
    /// completion (`memcpy`) or return a guard (`write_peripheral`/
    /// `stream_peripheral`).
    fn start(&self, cb_bus: u32) {
        unsafe {
            let cs = self.reg(CS);
            write_volatile(cs, CS_RESET);
            while read_volatile(cs) & CS_RESET != 0 {
                spin_loop();
            }
            write_volatile(cs, CS_END | CS_INT);
            write_volatile(self.reg(DEBUG), DEBUG_ERRORS);
            write_volatile(self.reg(CONBLK_AD), cb_bus);
            write_volatile(cs, CS_ACTIVE);
        }
    }

    /// Resets the channel, halting any in-flight (including cyclic or
    /// streaming) transfer so the engine stops reading its source
    /// buffer(s). Shared by the [`Transfer`] and [`Stream`] guards' `Drop`.
    fn halt(&self) {
        unsafe {
            let cs = self.reg(CS);
            write_volatile(cs, CS_RESET);
            while read_volatile(cs) & CS_RESET != 0 {
                spin_loop();
            }
        }
    }

    /// Largest byte count this channel's `TXFR_LEN` field can express.
    fn max_len(&self) -> usize {
        if self.id >= FIRST_LITE_CHANNEL {
            MAX_LEN_LITE
        } else {
            MAX_LEN_FULL
        }
    }

    /// Pointer to register `offset` within this channel's register block.
    fn reg(&self, offset: usize) -> *mut u32 {
        (DMA_BASE + self.id as usize * CHANNEL_STRIDE + offset) as *mut u32
    }
}

/// A running background transfer started by
/// [`Channel::write_peripheral`], holding the channel and the source
/// buffer borrowed for as long as the engine may still be reading them.
///
/// Dropping the guard stops the transfer (see [`Drop`]), so a cyclic
/// transfer keeps playing exactly as long as the guard is kept alive —
/// binding it to `_` (or letting it drop at the end of a statement)
/// would halt the transfer immediately. Both the channel and the source
/// slice stay borrowed here, so neither can be reused or freed while the
/// bus-master engine might still touch them.
pub struct Transfer<'a> {
    channel: &'a mut Channel,
}

impl Transfer<'_> {
    /// Whether a non-cyclic transfer has finished — true once the engine
    /// has dropped `CS.ACTIVE`. Always false in practice for a cyclic
    /// transfer, which only stops when the guard is dropped.
    pub fn is_complete(&self) -> bool {
        unsafe { read_volatile(self.channel.reg(CS)) & CS_ACTIVE == 0 }
    }

    /// Whether the channel flagged a hardware error (its `CS.ERROR` or a
    /// debug-register error flag is set) during the transfer.
    pub fn is_error(&self) -> bool {
        unsafe {
            read_volatile(self.channel.reg(CS)) & CS_ERROR != 0
                || read_volatile(self.channel.reg(DEBUG)) & DEBUG_ERRORS != 0
        }
    }

    /// Stops the transfer and releases the borrowed channel and buffer.
    /// Equivalent to dropping the guard, but reads as an explicit halt at
    /// the call site.
    pub fn stop(self) {}
}

impl Drop for Transfer<'_> {
    /// Halts the transfer so the engine stops reading the source buffer
    /// before the borrow ends and the buffer can be reused or freed.
    fn drop(&mut self) {
        self.channel.halt();
    }
}

/// A running double-buffered ("ping-pong") background stream started by
/// [`Channel::stream_peripheral`], holding the channel and both buffers
/// borrowed for as long as the engine may still be reading them.
///
/// The engine alternates between the two buffers forever; the caller
/// keeps it fed by calling [`Stream::feed`] in a loop, which refills
/// whichever buffer the engine is not currently reading. Dropping the
/// guard stops the stream (see [`Drop`]), so — like [`Transfer`] — it
/// must be kept alive for playback to continue.
pub struct Stream<'a> {
    channel: &'a mut Channel,
    /// The two sample buffers the engine ping-pongs between, kept borrowed
    /// so the caller can only touch them through [`Self::feed`].
    buffers: [&'a mut [u32]; 2],
    /// Bus addresses of `channel.cb[0]`/`cb[1]`, compared against the
    /// channel's live `CONBLK_AD` to tell which buffer is playing.
    cb_bus: [u32; 2],
    /// Index of the buffer [`Self::feed`] will refill next — the one the
    /// engine is expected to reach after the one it's reading now.
    next: usize,
}

impl Stream<'_> {
    /// Index (`0`/`1`) of the buffer the engine is currently reading, read
    /// live from the channel's `CONBLK_AD` (the control block it's
    /// executing). Defaults to `0` if the register matches neither block
    /// (e.g. momentarily between blocks).
    fn playing(&self) -> usize {
        let current = unsafe { read_volatile(self.channel.reg(CONBLK_AD)) };
        if current == self.cb_bus[1] {
            1
        } else {
            0
        }
    }

    /// Blocks until the next buffer to refill is safe to write (the engine
    /// has moved off it onto the other), calls `fill` to populate it with
    /// the next chunk of samples, flushes it so the engine sees the new
    /// data, and returns. Call in a loop for continuous playback.
    ///
    /// `fill` receives the whole buffer slice (the same length passed to
    /// [`Channel::stream_peripheral`]) and should fill it completely —
    /// whatever it leaves is what plays. If `fill` can't keep ahead of the
    /// engine the stream still plays, but the engine may replay a buffer's
    /// previous contents (an audible glitch) rather than stalling.
    pub fn feed(&mut self, fill: impl FnOnce(&mut [u32])) {
        let idx = self.next;
        // Wait until the engine is reading the *other* buffer, so refilling
        // `idx` can't race the read.
        while self.playing() == idx {
            spin_loop();
        }
        fill(self.buffers[idx]);
        let addr = self.buffers[idx].as_ptr() as usize as u32;
        let len = core::mem::size_of_val(&*self.buffers[idx]);
        cache::clean_range(addr, len);
        self.next = 1 - idx;
    }

    /// Whether the channel flagged a hardware error during the stream —
    /// same meaning as [`Transfer::is_error`].
    pub fn is_error(&self) -> bool {
        unsafe {
            read_volatile(self.channel.reg(CS)) & CS_ERROR != 0
                || read_volatile(self.channel.reg(DEBUG)) & DEBUG_ERRORS != 0
        }
    }

    /// Stops the stream and releases the borrowed channel and buffers.
    /// Equivalent to dropping the guard, but explicit at the call site.
    pub fn stop(self) {}
}

impl Drop for Stream<'_> {
    /// Halts the stream so the engine stops reading either buffer before
    /// the borrows end.
    fn drop(&mut self) {
        self.channel.halt();
    }
}

/// Translates a plain ARM physical address to the VideoCore's
/// `0xC000_0000`-based "direct, uncached" bus alias — the window a bus
/// master reads/writes RAM through without going via the L2 cache the GPU
/// owns. Mirrors `mailbox.rs`'s own translation; kept local here so the DMA
/// driver stays self-contained the way the other physical-poke drivers do.
fn to_bus(physical_address: u32) -> u32 {
    (physical_address & 0x3fff_ffff) | 0xc000_0000
}
