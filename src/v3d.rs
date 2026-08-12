//! Bring-up for the V3D 3D pipeline: VideoCore IV's QPU shader cores
//! and the tile-based binning/render hardware around them.
//!
//! Pi 3 (BCM2836/BCM2837) only. The Pi 4 ships a different GPU
//! generation under the same "V3D" name (different register layout,
//! successor ISA) that this module does not cover.
//!
//! Not modelled in `bcm2837-lpa`'s SVD, so this pokes its known
//! physical address directly — the same approach `dma.rs`/`rng.rs`
//! take for peripherals the SVD omits. `V3D_BASE` was confirmed
//! against a running Pi 3's device tree (the `v3d@7ec00000` node's
//! `reg` property, translated through the standard
//! `0x7Ennnnnn` -> `0x3Fnnnnnn` BCM283x bus-address alias) and
//! cross-checked against `/proc/iomem`'s live mapping — not assumed
//! from documentation alone.
//!
//! The block is power/clock-gated off by default, and needs *two*
//! mailbox calls before constructing [`V3d`](crate::v3d::V3d), in this
//! order:
//!
//! 1. [`crate::mailbox::Mailbox::set_clock_rate_hz`] with
//!    [`crate::mailbox::ClockId::V3d`] — the 3D core clock. The boot
//!    firmware never renders anything itself, so unlike every other
//!    clock this crate touches, it doesn't come up already running;
//!    under Linux the `vc4` driver's own `clk_prepare_enable` is what
//!    starts it, and a bare-metal build has no equivalent.
//! 2. [`crate::mailbox::Mailbox::set_enable_qpu`] — power/gating for
//!    the QPU shader cores. Not a substitute for the clock: with the
//!    clock unset, enough of the block answers to make this look like
//!    it worked (the identification registers read back correctly, and
//!    the control-list executor will even run a list to completion),
//!    but the shading pipeline behind it never processes a single
//!    primitive.
//!
//! Same shape as other GPU-adjacent blocks on this SoC (the camera
//! receiver's analog PHY, USB) needing explicit mailbox calls before
//! their registers do anything.
//!
//! Primary references for everything past this bring-up step (control
//! lists, QPU machine code, texture state): Broadcom's public
//! "VideoCore IV 3D Architecture Reference Guide" for the conceptual
//! model, and Mesa's open-source `vc4` gallium driver
//! (`src/gallium/drivers/vc4`) for exact packet/instruction encoding —
//! Broadcom's peripheral datasheet has no register-level documentation
//! for this block at all.

use core::ptr::{read_volatile, write_volatile};

use crate::cache::{clean_range, invalidate_range};

pub mod bcl;
pub mod rcl;
pub mod shader_record;
pub mod texture;
pub mod uniforms;

/// V3D register block base address — see the module docs for how this
/// was confirmed. Block size is `0x1000` (4KB).
const V3D_BASE: usize = 0x3fc0_0000;

/// Identification register 0 (`V3D_BASE + 0x000`): a technology-version
/// field and an ASCII identifier. Hardware-confirmed: a real read
/// returned `0x02443356`, which as little-endian bytes spells `"V3D"`
/// followed by technology-version byte `2` — and independently, the
/// real Linux `vc4` kernel driver's own `V3D_EXPECTED_IDENT0` constant
/// (`drivers/gpu/drm/vc4/vc4_regs.h`) computes to that exact same
/// value. Two independent sources agreeing, not a guess.
const IDENT0: *mut u32 = V3D_BASE as *mut u32;
/// Identification register 1 (`V3D_BASE + 0x004`): QPU/VPM
/// configuration fields.
const IDENT1: *mut u32 = (V3D_BASE + 0x004) as *mut u32;
/// Identification register 2 (`V3D_BASE + 0x008`): further
/// configuration fields (e.g. tile-buffer size).
const IDENT2: *mut u32 = (V3D_BASE + 0x008) as *mut u32;

/// L2 Cache Control — `V3D_BASE + 0x020` (`V3D_L2CACTL`). Bit 2
/// (`L2CCLR`) clears V3D's own L2 cache. That cache sits between V3D
/// and system memory and is *not* the ARM core's data cache, so
/// [`GpuBuffer::flush`]'s ARM-side clean doesn't touch it: a buffer
/// this core wrote and cleaned can still be shadowed by a stale line
/// here from whatever the firmware last did with the block.
const V3D_L2CACTL: *mut u32 = (V3D_BASE + 0x020) as *mut u32;
/// Slices Cache Control — `V3D_BASE + 0x024` (`V3D_SLCACTL`). Four
/// 4-bit fields, one bit per slice: `T1CC` (bits 27-24) and `T0CC`
/// (bits 19-16) are the two texture caches, `UCC` (bits 11-8) the
/// uniforms cache, `ICC` (bits 3-0) the QPU instruction cache. Writing
/// `0xf` to a field clears that cache across all four slices.
const V3D_SLCACTL: *mut u32 = (V3D_BASE + 0x024) as *mut u32;
/// VPM base (user) memory reservation — `V3D_BASE + 0x504`
/// (`V3D_VPMBASE`): how much Vertex Pipeline Memory is held back for
/// user (compute-style) QPU programs, in 256-byte units, and therefore
/// unavailable to the coordinate/vertex shading the 3D pipeline itself
/// needs. See [`V3d::new`] for why this driver forces it to zero.
const V3D_VPMBASE: *mut u32 = (V3D_BASE + 0x504) as *mut u32;

/// Performance Counter Clear — `V3D_BASE + 0x670` (`V3D_PCTRC`,
/// Table 83). One write-only bit per counter; writing `1` clears that
/// counter.
const V3D_PCTRC: *mut u32 = (V3D_BASE + 0x670) as *mut u32;
/// Performance Counter Enables — `V3D_BASE + 0x674` (`V3D_PCTRE`,
/// Table 84). Bits 0-15 enable individual counters; bit 31 is this
/// chip's global counter enable (`V3D_PCTRE_EN` in the kernel's
/// `vc4_regs.h`, which the datasheet's own table omits).
const V3D_PCTRE: *mut u32 = (V3D_BASE + 0x674) as *mut u32;
/// Base of the per-counter count registers — `V3D_BASE + 0x680`,
/// stride 8 (`V3D_PCTR(n)`, Table 85).
const V3D_PCTR0: usize = V3D_BASE + 0x680;
/// Base of the per-counter source-mapping registers — `V3D_BASE +
/// 0x684`, stride 8 (`V3D_PCTRS(n)`, Table 86). The low 5 bits select
/// which of the ~30 count sources in Table 82 that counter measures.
const V3D_PCTRS0: usize = V3D_BASE + 0x684;
/// `PCTRE` bit 31 — global performance-counter enable.
const PCTRE_EN: u32 = 1 << 31;

/// `L2CACTL.L2CCLR` — clear V3D's L2 cache.
const L2CCLR: u32 = 1 << 2;
/// `SLCACTL` value clearing all four slice caches (instruction,
/// uniforms, and both texture caches) — `0xf` in each of the four
/// 4-bit per-cache fields.
const SLCACTL_CLEAR_ALL: u32 = 0x0f0f_0f0f;
/// `SLCACTL` value clearing only the two texture caches (`T1CC`,
/// `T0CC`), leaving the instruction and uniforms caches alone.
const SLCACTL_CLEAR_TEXTURE: u32 = 0x0f0f_0000;

/// Control List Thread 0 (binning) Current Address — `V3D_BASE +
/// 0x110`, confirmed against the real Linux `vc4` kernel driver's
/// `vc4_regs.h` (`V3D_CT0CA`).
const CT0CA: *mut u32 = (V3D_BASE + 0x110) as *mut u32;
/// Control List Thread 0 (binning) End Address — `V3D_BASE + 0x108`
/// (`V3D_CT0EA`). Per the real kernel driver's `submit_cl`: "Writing
/// the end register is what starts the job" — `CT0CA` must be written
/// first, `CT0EA` second.
const CT0EA: *mut u32 = (V3D_BASE + 0x108) as *mut u32;
/// Control List Thread 0 (binning) Control/Status — `V3D_BASE + 0x100`
/// (`V3D_CT0CS`).
const CT0CS: *mut u32 = (V3D_BASE + 0x100) as *mut u32;
/// Control List Thread 1 (rendering) Current Address — `V3D_BASE +
/// 0x114` (`V3D_CT1CA`).
const CT1CA: *mut u32 = (V3D_BASE + 0x114) as *mut u32;
/// Control List Thread 1 (rendering) End Address — `V3D_BASE + 0x10c`
/// (`V3D_CT1EA`). Same write-order rule as [`CT0EA`]: `CT1CA` first,
/// `CT1EA` second.
const CT1EA: *mut u32 = (V3D_BASE + 0x10c) as *mut u32;
/// Control List Thread 1 (rendering) Control/Status — `V3D_BASE +
/// 0x104` (`V3D_CT1CS`).
const CT1CS: *mut u32 = (V3D_BASE + 0x104) as *mut u32;

/// Pipeline Control and Status — `V3D_BASE + 0x130` (`V3D_PCS`):
/// `BMOOM`/`RMBUSY`/`RMACTIVE`/`BMBUSY`/`BMACTIVE` (binner/render
/// machine busy/active, and binner-out-of-memory), confirmed against
/// the real Linux `vc4` kernel driver's `vc4_regs.h`.
const V3D_PCS: *mut u32 = (V3D_BASE + 0x130) as *mut u32;
/// Binner Flush Count — `V3D_BASE + 0x134` (`V3D_BFC`).
const V3D_BFC: *mut u32 = (V3D_BASE + 0x134) as *mut u32;
/// Render Flush Count — `V3D_BASE + 0x138` (`V3D_RFC`).
const V3D_RFC: *mut u32 = (V3D_BASE + 0x138) as *mut u32;
/// FEP (fragment pipe) overrun error signals — `V3D_BASE + 0xf04`
/// (`V3D_FDBGO`).
const V3D_FDBGO: *mut u32 = (V3D_BASE + 0xf04) as *mut u32;
/// FEP interface ready/stall signals — `V3D_BASE + 0xf08`
/// (`V3D_FDBGB`).
const V3D_FDBGB: *mut u32 = (V3D_BASE + 0xf08) as *mut u32;
/// Binner Memory Pool Current Address — `V3D_BASE + 0x300`
/// (`V3D_BPCA`), confirmed against the real Linux `vc4` kernel driver's
/// `vc4_regs.h`. Tracks where the binner is currently writing into the
/// tile allocation memory ([`bcl::BclParams::tile_alloc_address`]'s
/// buffer) as it bins primitives — reading this right after a bin pass
/// completes tells whether the binner ever advanced past the pool's
/// base address at all, independent of what ended up in any specific
/// tile's slot.
const V3D_BPCA: *mut u32 = (V3D_BASE + 0x300) as *mut u32;
/// Binner Memory Pool Current Size (remaining free bytes) — `V3D_BASE +
/// 0x304` (`V3D_BPCS`).
const V3D_BPCS: *mut u32 = (V3D_BASE + 0x304) as *mut u32;
/// Control List Executor Thread 0 (binning) List Counter — `V3D_BASE +
/// 0x120` (`V3D_CT0LC`), confirmed against Broadcom's public "VideoCore
/// IV 3D Architecture Reference Guide" (`docs.broadcom.com/doc/12358545`,
/// Table 53). Bits 31:16: count of `Flush` commands encountered so far
/// — direct confirmation that a `Flush` packet was actually fetched
/// (independent of [`V3D_BFC`], which only increments once the PTB
/// hardware has *finished* flushing, a separate and later condition).
/// Bits 15:0: count of `Return from sub-list` commands encountered.
const V3D_CT0LC: *mut u32 = (V3D_BASE + 0x120) as *mut u32;
/// Control List Executor Thread 1 (rendering) List Counter —
/// `V3D_BASE + 0x124` (`V3D_CT1LC`), same field layout as
/// [`V3D_CT0LC`].
const V3D_CT1LC: *mut u32 = (V3D_BASE + 0x124) as *mut u32;
/// Control List Executor Thread 0 (binning) Primitive List Counter —
/// `V3D_BASE + 0x128` (`V3D_CT0PC`), per the same reference guide
/// (Table 54): "Count of primitives remaining whilst processing a
/// primitive list." A nonzero, unmoving value here while the binning
/// pipeline is stalled (`V3D_PCS.BMACTIVE` stuck set) would mean the
/// coordinate shader itself is stuck partway through the draw's
/// vertices, not merely a control-list sequencing problem.
const V3D_CT0PC: *mut u32 = (V3D_BASE + 0x128) as *mut u32;
/// Control List Executor Thread 1 (rendering) Primitive List Counter —
/// `V3D_BASE + 0x12c` (`V3D_CT1PC`), same field layout as
/// [`V3D_CT0PC`].
const V3D_CT1PC: *mut u32 = (V3D_BASE + 0x12c) as *mut u32;

/// Miscellaneous error signals — `V3D_BASE + 0xf20` (`V3D_ERRSTAT`),
/// per Broadcom's reference guide Table 87. One bit per real internal
/// fault across the front-end blocks the 3D pipeline depends on: VPM
/// allocator errors (bits 0-3: allocating base while busy, request too
/// big, binner/renderer request over limit), VPM access errors (bits
/// 4-9: write/read range, read/write non-allocated, free non-allocated,
/// allocated size), VDW address overflow (bit 10), VCD FIFO pointers
/// out of sync (bit 11), VCD idle (bit 12), VCM binner/renderer errors
/// (bits 13-14), and L2C receive FIFO overrun (bit 15). All reset to
/// `0`, so any nonzero read is a real fault — the one register that
/// distinguishes "the binning front-end hit an error" from "the
/// binning front-end is merely waiting for something."
const V3D_ERRSTAT: *mut u32 = (V3D_BASE + 0xf20) as *mut u32;
/// PSE (primitive setup engine) error signals — `V3D_BASE + 0xf00`
/// (`V3D_DBGE`), Table 88. Bits 1-2 are "error a/b reading VPM", bits
/// 16-20 cover the primitive setup multipliers and interpolators.
const V3D_DBGE: *mut u32 = (V3D_BASE + 0xf00) as *mut u32;
/// QPU reservation settings for QPUs 0-7 — `V3D_BASE + 0x410`
/// (`V3D_SQRSV0`), Table 62. Four bits per QPU: bit 0 "do not use for
/// User Programs", bit 1 fragment shaders, bit 2 vertex shaders, bit 3
/// coordinate shaders. A QPU with its coordinate-shader bit set can't
/// be scheduled to run one — so if the firmware left every QPU
/// reserved against coordinate shaders, the binning pipeline would
/// stall forever with no primitive ever shaded, which is exactly the
/// symptom this driver is chasing.
const V3D_SQRSV0: *mut u32 = (V3D_BASE + 0x410) as *mut u32;
/// QPU reservation settings for QPUs 8-15 — `V3D_BASE + 0x414`
/// (`V3D_SQRSV1`), same per-QPU field layout as [`V3D_SQRSV0`].
const V3D_SQRSV1: *mut u32 = (V3D_BASE + 0x414) as *mut u32;

/// Control List Executor Thread 0 (binning) Return Address 0 —
/// `V3D_BASE + 0x118` (`V3D_CT00RA0`), Table 52: "Address on the return
/// address stack. (N.B. We only support a one-deep return address
/// stack.)" Meaningful when `CT0CS`'s `CTRTSD` field reports a nonzero
/// list-nesting depth — it says what address the thread believes it
/// still has to return to.
const V3D_CT00RA0: *mut u32 = (V3D_BASE + 0x118) as *mut u32;
/// Address of the overspill binning memory block — `V3D_BASE + 0x308`
/// (`V3D_BPOA`), Table 60: extra memory the PTB may take once the pool
/// set up by [`bcl::BclParams::tile_alloc_address`] runs out.
const V3D_BPOA: *mut u32 = (V3D_BASE + 0x308) as *mut u32;
/// Size of the overspill binning memory block — `V3D_BASE + 0x30c`
/// (`V3D_BPOS`), Table 61. Critically: "If this count is zero when the
/// PTB runs out of binning memory, the PTB will **halt, waiting for a
/// non-zero value to be written to this register**." This driver has
/// never written either overspill register, so both hold whatever the
/// firmware left — and a PTB halted this way is indistinguishable from
/// one that is merely idle, apart from these two registers.
const V3D_BPOS: *mut u32 = (V3D_BASE + 0x30c) as *mut u32;
/// Binner debug — `V3D_BASE + 0x310` (`V3D_BXCF`), Table 93: bit 0
/// `FWDDISA` (disable forwarding in state cache), bit 1 `CLIPDISA`
/// (disable clipping). Both reset to `0`; a stray `CLIPDISA` or
/// `FWDDISA` left set by the firmware would change binning behavior
/// with no error reported anywhere.
const V3D_BXCF: *mut u32 = (V3D_BASE + 0x310) as *mut u32;
/// VPM allocator control — `V3D_BASE + 0x500` (`V3D_VPACNTL`),
/// Table 70: allocation limits and timeouts for binning versus
/// rendering VPM requests. Reset `0` (limits disabled). If the firmware
/// left `VPALIMEN` (bit 12) set with a zero binning limit, the VPM
/// allocator would grant the binning pipeline nothing, forever, without
/// setting any error bit — a silent stall matching what this driver is
/// chasing.
const V3D_VPACNTL: *mut u32 = (V3D_BASE + 0x500) as *mut u32;

/// Iterations [`V3d::wait`] polls before giving up and returning
/// [`Error::Timeout`]. Not tied to a real time reference (this crate
/// has no cheap way to convert a cycle count to wall-clock time from
/// here) — just large enough that a genuinely completing job never
/// hits it, and small enough that a real hang doesn't look
/// indistinguishable from the board being off.
const WAIT_TIMEOUT_ITERATIONS: u32 = 50_000_000;

/// `CTnCS.CTRUN` — set while thread `n`'s control-list executor is
/// running, clear once it finishes (or before it's ever started).
const CTRUN: u32 = 1 << 5;
/// `CTnCS.CTERR` — set if thread `n`'s executor hit an error.
const CTERR: u32 = 1 << 3;

/// How many of V3D's 16 performance counters
/// [`V3d::configure_performance_counters`] drives. Four is enough to
/// cover the questions this driver's bring-up needs answered at once,
/// and keeps the printed diagnostics readable.
pub const PERFORMANCE_COUNTERS: usize = 4;

/// A handle to the V3D register block.
pub struct V3d {
    _private: (),
}

impl V3d {
    /// Constructs a handle to the V3D register block.
    ///
    /// Set the V3D clock rate and enable the QPU first — see this
    /// module's own documentation for both calls and why enabling the
    /// QPU alone isn't enough.
    ///
    /// Zeroes `V3D_VPMBASE`, matching the real Linux `vc4` driver's own
    /// hardware init (`vc4_v3d_init_hw` in `vc4_v3d.c`, run every time
    /// the block powers up): "Take all the memory that would have been
    /// reserved for user QPU programs, since we don't have an interface
    /// for running them, anyway." Whatever the firmware left this set to
    /// before handing the block over is memory the coordinate and vertex
    /// shaders can't use, and they need VPM to run at all — a nonzero
    /// value here starves the 3D pipeline rather than producing wrong
    /// output. This crate has no user-QPU-program interface either, so
    /// the same all-of-it-to-the-pipeline choice applies unconditionally.
    ///
    /// # Safety
    ///
    /// Unlike [`crate::rng::Rng`]'s benign shared entropy FIFO, this
    /// block's registers govern live GPU execution state (control-list
    /// submission, tile memory) — two owners writing to it concurrently
    /// could corrupt each other's jobs. There's no PAC singleton to
    /// enforce exclusivity (the block isn't in `bcm2837-lpa`'s SVD), so
    /// the caller must ensure at most one `V3d` exists at a time.
    pub unsafe fn new() -> Self {
        // SAFETY: `VPMBASE` is one of this block's own Device-mapped
        // registers, and the caller has guaranteed exclusive ownership.
        unsafe { write_volatile(V3D_VPMBASE, 0) };
        Self { _private: () }
    }

    /// Clears every V3D-side cache — its L2 plus all four slice caches
    /// (QPU instruction, uniforms, and both texture caches).
    ///
    /// These sit between V3D and system memory and are entirely
    /// separate from the ARM core's own data cache, so
    /// [`GpuBuffer::flush`]'s clean doesn't reach them: without this,
    /// V3D can fetch a stale shader instruction, uniform, or texel that
    /// this core's writes never invalidated. The real Linux `vc4`
    /// driver does exactly this before *every* binning submission
    /// (`vc4_flush_caches` in `vc4_gem.c`), not once at init — so this
    /// does too.
    fn flush_caches(&self) {
        // SAFETY: both are this block's own Device-mapped control
        // registers, exclusively owned by this `V3d`.
        unsafe {
            write_volatile(V3D_L2CACTL, L2CCLR);
            write_volatile(V3D_SLCACTL, SLCACTL_CLEAR_ALL);
        }
    }

    /// Clears V3D's L2 and the two texture caches only, leaving the
    /// instruction and uniforms caches alone — the render pass's
    /// counterpart to [`flush_caches`](Self::flush_caches), mirroring
    /// `vc4_flush_texture_caches` in `vc4_gem.c` and its reasoning: a
    /// previous render pass may have written one of this frame's
    /// textures after the bin-time flush already happened, but nothing
    /// writes shader code or uniforms from a render control list, so
    /// those two caches don't need re-clearing here.
    fn flush_texture_caches(&self) {
        // SAFETY: same as `flush_caches`.
        unsafe {
            write_volatile(V3D_L2CACTL, L2CCLR);
            write_volatile(V3D_SLCACTL, SLCACTL_CLEAR_TEXTURE);
        }
    }

    /// Reads the three identification registers as raw words — the
    /// cheapest possible check that the block is alive and clocked,
    /// before attempting anything harder to debug (a control-list
    /// submission that just hangs). Broadcom's public 3D Architecture
    /// Reference Guide documents `IDENT0` as encoding a
    /// technology-version number and an ASCII identifier, and
    /// `IDENT1`/`IDENT2` as further QPU/VPM/tile-buffer configuration
    /// fields; this driver doesn't decode those fields itself, just
    /// exposes the raw words for a caller to check against the
    /// reference guide (or a known-good capture from real hardware).
    pub fn ident(&self) -> (u32, u32, u32) {
        unsafe {
            (
                read_volatile(IDENT0),
                read_volatile(IDENT1),
                read_volatile(IDENT2),
            )
        }
    }

    /// Submits one frame — a binning pass over `bin_cl` followed by a
    /// render pass over `render_cl` — and blocks until both finish.
    /// Equivalent to [`submit_bin`](Self::submit_bin) then
    /// [`submit_render`](Self::submit_render); see those for the
    /// per-stage detail (and for why a caller might want to call them
    /// separately instead — e.g. to inspect what the binner produced
    /// before the render pass consumes it).
    pub fn submit_frame(&self, bin_cl: (u32, u32), render_cl: (u32, u32)) -> Result<(), Error> {
        self.submit_bin(bin_cl)?;
        self.submit_render(render_cl)
    }

    /// Submits the binning pass over `bin_cl` — `(start_bus_address,
    /// end_bus_address)`, matching [`crate::v3d::bcl::build`]'s return
    /// length added to its buffer's own [`GpuBuffer::bus_address`] —
    /// and blocks until it finishes. Polls rather than waiting for
    /// V3D's completion interrupts — matches how `dma.rs`/`sd.rs`
    /// start with polling before any IRQ path; wiring V3D's interrupts
    /// into `lic.rs` is a later, independent improvement.
    ///
    /// The end address must not coincide with the start of any buffer
    /// the control list branches *into*. The executor stops as soon as
    /// its current address reaches the end address, and that check
    /// applies inside sub-lists too — an `Indexed Primitive List` makes
    /// it branch to the index data and return, so an index buffer
    /// beginning exactly at `bin_cl.1` halts binning before a single
    /// index is read, silently: no error bit is set, and the only
    /// evidence is a nonzero `CTRTSD` nesting depth with `CT00RA0`
    /// pointing back into the control list. Leaving unused slack after
    /// the list, so its end address falls inside its own buffer, is the
    /// simple way to guarantee this (see `examples/gpu_cube.rs`).
    ///
    /// "Finishes" means two separate things here, and this waits for
    /// both. First the control-list *executor* stops (`CTRUN` clears),
    /// which only means it read to the end of `bin_cl` — the binning
    /// hardware behind it can still be working. Then `V3D_BFC`, the
    /// binning flush count, must actually change: per Broadcom's
    /// reference guide (Table 56) it increments "once the PTB has
    /// flushed all tile lists to memory and the PTB has finished with
    /// the tile state data array", which is the real completion
    /// condition, and the same event the Linux driver waits for as its
    /// `V3D_INT_FLDONE` interrupt. Waiting only on `CTRUN` would report
    /// success for a binning pass that never produced a single tile
    /// list — and the render pass's `Wait on Semaphore` would then
    /// block forever, since the semaphore increment is part of that
    /// same flush.
    pub fn submit_bin(&self, bin_cl: (u32, u32)) -> Result<(), Error> {
        self.flush_caches();
        // SAFETY: CT0CA/CT0EA/CT0CS/V3D_BFC are Device-mapped MMIO
        // registers for a block this `V3d` has exclusive ownership of
        // (see `new`'s safety doc); plain register accesses have no
        // other preconditions.
        let flush_count_before = unsafe { read_volatile(V3D_BFC) } & 0xff;
        unsafe {
            write_volatile(CT0CA, bin_cl.0);
            write_volatile(CT0EA, bin_cl.1);
        }
        self.wait(CT0CS, Error::BinFailed, Error::BinTimeout)?;

        // `BFC` is an 8-bit counter that wraps, so compare for
        // *change* rather than for a greater value.
        for _ in 0..WAIT_TIMEOUT_ITERATIONS {
            if unsafe { read_volatile(V3D_BFC) } & 0xff != flush_count_before {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(Error::BinFlushTimeout)
    }

    /// Submits the render pass over `render_cl`, same shape as
    /// [`submit_bin`](Self::submit_bin). Only meaningful after a
    /// matching `submit_bin` — matching the real Linux `vc4` driver's
    /// own ordering (`vc4_gem.c`'s `vc4_submit_next_bin_job`/
    /// `vc4_submit_next_render_job`), not submitted concurrently with
    /// it, even though the render pass's own `Wait on Semaphore`
    /// packet ([`crate::v3d::rcl`]'s doc comment) would likely make
    /// that safe too — an extra variable this crate's first real
    /// hardware test of this pipeline didn't need to take on.
    pub fn submit_render(&self, render_cl: (u32, u32)) -> Result<(), Error> {
        self.flush_texture_caches();
        // SAFETY: same as `submit_bin`, for thread 1.
        unsafe {
            write_volatile(CT1CA, render_cl.0);
            write_volatile(CT1EA, render_cl.1);
        }
        self.wait(CT1CS, Error::RenderFailed, Error::RenderTimeout)
    }

    /// Blocks until the control-list executor at `cs` (either
    /// [`CT0CS`] or [`CT1CS`]) drops `CTRUN`, returning `on_error` if
    /// `CTERR` was set, or `on_timeout` after
    /// [`WAIT_TIMEOUT_ITERATIONS`] if `CTRUN` never clears at all.
    /// Shared by both halves of [`submit_frame`](Self::submit_frame) —
    /// same poll-and-check shape as `dma.rs`'s `Channel::wait_blocking`,
    /// plus the timeout `dma.rs` doesn't need (a DMA transfer that
    /// never finishes there would mean the CPU itself has stopped
    /// making progress; a V3D job that never finishes here does not —
    /// the CPU is still alive and able to report it, so it should).
    fn wait(&self, cs: *mut u32, on_error: Error, on_timeout: Error) -> Result<(), Error> {
        for _ in 0..WAIT_TIMEOUT_ITERATIONS {
            // SAFETY: `cs` is one of this block's own Device-mapped
            // status registers.
            let status = unsafe { read_volatile(cs) };
            if status & CTRUN == 0 {
                return if status & CTERR != 0 {
                    Err(on_error)
                } else {
                    Ok(())
                };
            }
            core::hint::spin_loop();
        }
        Err(on_timeout)
    }

    /// Points this driver's [`PERFORMANCE_COUNTERS`] counters at
    /// `sources` — count-source ids from Broadcom's reference guide
    /// Table 82 — then clears and enables them.
    ///
    /// Unlike every other register this module exposes, these measure
    /// the *interior* of the pipeline rather than its control-list
    /// front end: whether the QPUs ran coordinate shading at all
    /// (source `14`), and how many primitives the PTB and PSE threw
    /// away for being outside the viewport (`10`), needing clipping
    /// (`11`), or facing backwards (`12`). That distinguishes "the
    /// geometry never reached the binner" from "the binner rejected
    /// it, for this specific reason" — a distinction none of the
    /// control-list status registers can make, since both look
    /// identical there: a clean flush with an empty tile list.
    pub fn configure_performance_counters(&self, sources: [u8; PERFORMANCE_COUNTERS]) {
        // SAFETY: all of these are this block's own Device-mapped
        // registers, exclusively owned by this `V3d`.
        unsafe {
            for (i, &source) in sources.iter().enumerate() {
                write_volatile((V3D_PCTRS0 + i * 8) as *mut u32, u32::from(source) & 0x1f);
            }
            write_volatile(V3D_PCTRC, 0xffff);
            let enabled = (1u32 << PERFORMANCE_COUNTERS) - 1;
            write_volatile(V3D_PCTRE, PCTRE_EN | enabled);
        }
    }

    /// Zeroes every performance counter, so the next read covers only
    /// what happened after this call.
    pub fn clear_performance_counters(&self) {
        // SAFETY: `PCTRC` is this block's own Device-mapped register.
        unsafe { write_volatile(V3D_PCTRC, 0xffff) };
    }

    /// Current values of the counters
    /// [`configure_performance_counters`](Self::configure_performance_counters)
    /// set up, in the same order as the `sources` given there.
    pub fn performance_counters(&self) -> [u32; PERFORMANCE_COUNTERS] {
        let mut counts = [0; PERFORMANCE_COUNTERS];
        for (i, count) in counts.iter_mut().enumerate() {
            // SAFETY: these are this block's own Device-mapped count
            // registers.
            *count = unsafe { read_volatile((V3D_PCTR0 + i * 8) as *const u32) };
        }
        counts
    }

    /// Raw diagnostic register values — for a caller to print when
    /// [`submit_frame`](Self::submit_frame) returns an error or a
    /// timeout and the plain `CTERR`/`CTRUN` bits aren't enough to
    /// tell what actually went wrong.
    pub fn debug_status(&self) -> DebugStatus {
        // SAFETY: all of these are this block's own Device-mapped
        // status registers.
        unsafe {
            DebugStatus {
                ct0ca: read_volatile(CT0CA),
                ct1ca: read_volatile(CT1CA),
                ct0cs: read_volatile(CT0CS),
                ct1cs: read_volatile(CT1CS),
                pcs: read_volatile(V3D_PCS),
                bfc: read_volatile(V3D_BFC),
                rfc: read_volatile(V3D_RFC),
                fdbgo: read_volatile(V3D_FDBGO),
                fdbgb: read_volatile(V3D_FDBGB),
                bpca: read_volatile(V3D_BPCA),
                bpcs: read_volatile(V3D_BPCS),
                ct0lc: read_volatile(V3D_CT0LC),
                ct1lc: read_volatile(V3D_CT1LC),
                ct0pc: read_volatile(V3D_CT0PC),
                ct1pc: read_volatile(V3D_CT1PC),
                errstat: read_volatile(V3D_ERRSTAT),
                dbge: read_volatile(V3D_DBGE),
                sqrsv0: read_volatile(V3D_SQRSV0),
                sqrsv1: read_volatile(V3D_SQRSV1),
                ct00ra0: read_volatile(V3D_CT00RA0),
                bpoa: read_volatile(V3D_BPOA),
                bpos: read_volatile(V3D_BPOS),
                bxcf: read_volatile(V3D_BXCF),
                vpacntl: read_volatile(V3D_VPACNTL),
                vpmbase: read_volatile(V3D_VPMBASE),
            }
        }
    }
}

/// Errors from [`V3d::submit_frame`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The binning pass's control-list executor reported `CTERR`.
    BinFailed,
    /// The binning pass never dropped `CTRUN` within
    /// `WAIT_TIMEOUT_ITERATIONS` — a hang, not a reported error.
    BinTimeout,
    /// The binning pass's control-list executor finished cleanly, but
    /// the binning *hardware* never completed: `V3D_BFC` never changed
    /// within `WAIT_TIMEOUT_ITERATIONS`, meaning the PTB never flushed
    /// its tile lists to memory. Distinct from [`Error::BinTimeout`],
    /// which is the executor itself hanging — see
    /// [`V3d::submit_bin`] for why the two are separate conditions.
    BinFlushTimeout,
    /// The render pass's control-list executor reported `CTERR`.
    RenderFailed,
    /// The render pass never dropped `CTRUN` within
    /// `WAIT_TIMEOUT_ITERATIONS` — a hang, not a reported error.
    RenderTimeout,
}

/// Raw V3D diagnostic register values — see
/// [`V3d::debug_status`].
#[derive(Clone, Copy, Debug)]
pub struct DebugStatus {
    /// `CT0CA` — the binning thread's current control-list read
    /// address. Equals [`bcl::BclParams`]'s control list's own end
    /// address once binning completes normally.
    pub ct0ca: u32,
    /// `CT1CA` — the rendering thread's current control-list read
    /// address: "the address of the current record ... or the first
    /// record to be processed when stopped" (Broadcom's reference
    /// guide, Table 51). When `CT1CS` reads `CTRUN | CTSUBS` with
    /// `CTRUN` set, `CTSUBS` means "stalled, waiting for other thread
    /// to complete in this mode" (Table 49) — i.e. blocked at
    /// `Wait on Semaphore`, not "currently inside a sub-list" as the
    /// bit's name alone suggests — so `ct1ca` in that state is the next
    /// record to process once unblocked, not necessarily an address
    /// [`crate::v3d::rcl`]'s `Branch to sub-list` ever reached.
    pub ct1ca: u32,
    /// `CT0CS` — the binning thread's own control/status word.
    pub ct0cs: u32,
    /// `CT1CS` — the rendering thread's own control/status word.
    pub ct1cs: u32,
    /// `V3D_PCS` ("Pipeline Control and Status"): binner/render machine
    /// active/busy bits, and binner-out-of-memory. Per Broadcom's
    /// reference guide (Table 55), `BMACTIVE`/`RMACTIVE` (bits 0/2) stay
    /// set until their respective pipeline is "completely empty" —
    /// still set long after a control list's own fetcher has stopped
    /// means the underlying binning/rendering *hardware pipeline*
    /// (not just the control-list executor) hasn't actually finished —
    /// a real stall `submit_bin`/`submit_render`'s own `CTRUN`/`CTERR`
    /// polling can't see, since it only watches the executor, not the
    /// pipeline behind it.
    pub pcs: u32,
    /// `V3D_BFC`: binner flush count — incremented "once the PTB has
    /// flushed all tile lists to memory and the PTB has finished with
    /// the tile state data array" (Table 56). This is later and
    /// stricter than the binning control list's own `Flush` packet
    /// merely having been *fetched*: use [`DebugStatus::ct0lc`]'s major
    /// count for that instead.
    pub bfc: u32,
    /// `V3D_RFC`: render flush count.
    pub rfc: u32,
    /// `V3D_FDBGO`: FEP (fragment pipe) overrun error signals.
    pub fdbgo: u32,
    /// `V3D_FDBGB`: FEP interface ready/stall signals.
    pub fdbgb: u32,
    /// `V3D_BPCA`: binner memory pool current address — where the
    /// binner is (or last was) writing into the tile allocation memory.
    /// Unchanged from [`bcl::BclParams::tile_alloc_address`] means the
    /// binner never wrote anything there at all.
    pub bpca: u32,
    /// `V3D_BPCS`: binner memory pool current size (remaining free
    /// bytes in the tile allocation memory).
    pub bpcs: u32,
    /// `V3D_CT0LC`: binning thread's list counter — high 16 bits are
    /// the count of `Flush` commands the control-list *executor* has
    /// fetched (regardless of whether the PTB hardware has actually
    /// finished flushing — see [`DebugStatus::bfc`]), low 16 bits are
    /// the count of `Return from sub-list` commands fetched.
    pub ct0lc: u32,
    /// `V3D_CT1LC`: rendering thread's list counter, same field layout
    /// as [`DebugStatus::ct0lc`].
    pub ct1lc: u32,
    /// `V3D_CT0PC`: binning thread's primitive list counter — "count of
    /// primitives remaining whilst processing a primitive list"
    /// (Table 54). A nonzero, unmoving value while [`DebugStatus::pcs`]
    /// shows the binning pipeline still active means the coordinate
    /// shader itself is stuck partway through the draw's vertices, not
    /// merely a control-list sequencing problem.
    pub ct0pc: u32,
    /// `V3D_CT1PC`: rendering thread's primitive list counter, same
    /// field layout as [`DebugStatus::ct0pc`].
    pub ct1pc: u32,
    /// `V3D_ERRSTAT`: VPM/VDW/VCD/VCM/L2C error signals — all reset to
    /// `0`, so any nonzero value is a real internal fault in the
    /// binning front-end. See `V3D_ERRSTAT`'s own constant for the
    /// per-bit meanings.
    pub errstat: u32,
    /// `V3D_DBGE`: PSE (primitive setup engine) error signals,
    /// including "error reading VPM".
    pub dbge: u32,
    /// `V3D_SQRSV0`: QPU 0-7 reservation settings — 4 bits per QPU,
    /// bit 3 of each nibble reserving that QPU *against* coordinate
    /// shaders. Nonzero means some shader type can't be scheduled onto
    /// the affected QPUs.
    pub sqrsv0: u32,
    /// `V3D_SQRSV1`: QPU 8-15 reservation settings, same per-QPU field
    /// layout as [`DebugStatus::sqrsv0`].
    pub sqrsv1: u32,
    /// `V3D_CT00RA0`: the binning thread's one-deep return address
    /// stack — what it thinks it has to return to when `CT0CS`'s
    /// `CTRTSD` reports nonzero list nesting.
    pub ct00ra0: u32,
    /// `V3D_BPOA`: address of the overspill binning memory block.
    pub bpoa: u32,
    /// `V3D_BPOS`: size of the overspill binning memory block. Zero
    /// here *and* a PTB that has run out of pool means a halted binner
    /// waiting for memory that will never arrive.
    pub bpos: u32,
    /// `V3D_BXCF`: binner debug — disable-clipping and
    /// disable-state-cache-forwarding bits, both normally `0`.
    pub bxcf: u32,
    /// `V3D_VPACNTL`: VPM allocator control — allocation limits and
    /// timeouts. Normally `0`; a nonzero limit-enable with a zero
    /// binning limit starves the binning pipeline silently.
    pub vpacntl: u32,
    /// `V3D_VPMBASE`: VPM reserved for user programs, read back after
    /// [`V3d::new`] forces it to zero — confirms the write took.
    pub vpmbase: u32,
}

/// Translates a plain ARM physical address to the VideoCore's
/// `0xC000_0000`-based "direct, uncached" bus alias — the window a bus
/// master reads/writes RAM through without going via the L2 cache the
/// GPU owns. Mirrors `mailbox.rs`/`dma.rs`'s own translation; kept
/// local here so this driver stays self-contained the way the other
/// physical-poke drivers do (see `dma.rs`'s own `to_bus`, whose doc
/// comment states this as a deliberate choice, not an oversight).
fn to_bus(physical_address: u32) -> u32 {
    (physical_address & 0x3fff_ffff) | 0xc000_0000
}

/// Public counterpart to this module's private `to_bus`, for a buffer
/// this crate doesn't own the type of — e.g. a texture, which needs
/// stronger (4096-byte) alignment than [`GpuBuffer`] provides and so
/// can't be one. [`GpuBuffer::bus_address`] calls this same
/// translation internally; this is that translation exposed directly
/// for anything else.
pub fn bus_address(physical_address: u32) -> u32 {
    to_bus(physical_address)
}

/// Public counterpart to [`GpuBuffer::flush`], for a buffer this crate
/// doesn't own the type of. See that method's doc comment for when to
/// call this.
///
/// `address` must be this core's own plain address for the buffer —
/// the same one [`bus_address`] takes as *input*, not the bus address
/// it returns. Cache maintenance is by-VA under the hood
/// (`DCCMVAC`/`dc cvac`), so it has to target an address the ARM MMU
/// actually maps; the `0xC000_0000` bus alias isn't identity-mapped
/// RAM from the ARM core's point of view, and targeting it with a
/// cache-maintenance instruction is expected to fault. Easy to get
/// backwards by reaching for `bus_address` out of habit, since
/// everything else handed to V3D goes through it — this doesn't.
pub fn flush(address: u32, len: usize) {
    clean_range(address, len);
}

/// Public counterpart to [`GpuBuffer::invalidate`], for a buffer this
/// crate doesn't own the type of. See that method's doc comment for
/// when to call this, and [`flush`]'s doc comment for why `address`
/// must be a plain address, never the result of [`bus_address`].
pub fn invalidate(address: u32, len: usize) {
    invalidate_range(address, len);
}

/// A cache-aligned, `BYTES`-byte buffer for data V3D reads or writes
/// directly over the bus: binning/render control lists, tile
/// state/allocation memory, vertex data, the uniform stream, texture
/// data. One generic type for all of them — mechanism, not a
/// per-purpose type — the same way `dma.rs`'s `Channel::memcpy` takes
/// a caller-owned slice rather than this crate guessing at scene-
/// specific sizes. How many tiles, how big a texture, is a decision
/// for whatever builds a real control list on top of this, not for
/// this driver.
///
/// V3D is a bus master like DMA/SD/USB, reading and writing RAM
/// outside this core's cache — [`flush`](Self::flush) and
/// [`invalidate`](Self::invalidate) are the same clean-before-give,
/// invalidate-after-take bookkeeping `dma.rs`'s transfers already need,
/// exposed here rather than done automatically since only the caller
/// knows when V3D is actually about to touch a given buffer.
///
/// Cache-line aligned (`align(32)`, matching this crate's `cache`
/// module's granularity) but not necessarily cache-line
/// *padded* — a `BYTES` not itself a multiple of 32 leaves the last
/// partial line shared with whatever memory follows, and
/// [`flush`](Self::flush)/[`invalidate`](Self::invalidate) operate on
/// whole lines. Same caller responsibility `dma.rs` already documents
/// for its own transfer buffers.
#[repr(C, align(32))]
pub struct GpuBuffer<const BYTES: usize> {
    bytes: [u8; BYTES],
}

impl<const BYTES: usize> GpuBuffer<BYTES> {
    /// A new, zeroed buffer.
    pub const fn new() -> Self {
        Self { bytes: [0; BYTES] }
    }

    /// The bus address to hand to V3D — e.g. a control list's start
    /// address, or a texture's address word in the uniform stream. See
    /// `to_bus` (this module's private translation helper) for why
    /// this isn't just the buffer's plain address.
    pub fn bus_address(&self) -> u32 {
        to_bus(self.bytes.as_ptr() as u32)
    }

    /// This buffer's contents, to write a control list, vertex data,
    /// or similar into before handing it to V3D.
    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        &mut self.bytes
    }

    /// This buffer's contents, to read back whatever V3D wrote (call
    /// [`invalidate`](Self::invalidate) first).
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Cleans (writes back) this buffer's cache lines. Call before
    /// handing its bus address to V3D for it to *read*, so V3D sees
    /// this core's latest writes instead of whatever was in RAM
    /// before them.
    pub fn flush(&self) {
        clean_range(self.bytes.as_ptr() as u32, BYTES);
    }

    /// Invalidates this buffer's cache lines. Call after V3D has
    /// *written* into it and before this core reads it back, so a
    /// stale cached copy from before the write isn't returned instead.
    pub fn invalidate(&self) {
        invalidate_range(self.bytes.as_ptr() as u32, BYTES);
    }
}

impl<const BYTES: usize> Default for GpuBuffer<BYTES> {
    /// Same as [`GpuBuffer::new`].
    fn default() -> Self {
        Self::new()
    }
}
