//! A minimal, polled NVMe driver: one namespace, one I/O queue pair,
//! blocking reads, writes and flushes.
//!
//! [`Nvme::new`] takes a controller [`crate::pcie::Pcie::enable`] has set
//! up, resets it, creates its admin queues, reads its identity and its
//! first namespace's size, and creates one I/O submission/completion queue
//! pair. Each [`read_blocks`](Nvme::read_blocks) /
//! [`write_blocks`](Nvme::write_blocks) call then submits one command per
//! chunk of up to [`max_transfer`](Nvme::max_transfer) bytes and polls the
//! completion queue for it. No interrupts are involved: the controller's
//! interrupts are masked, and the bus has none set up anyway (see
//! [`crate::pcie`]).
//!
//! ## Memory
//!
//! The queues, a PRP list for transfers that span more than two pages, and
//! a page for identify data and unaligned transfers live in a [`Memory`]
//! the caller provides — 24 KiB, page aligned, anywhere in RAM. The
//! controller reads and writes it directly, so it must outlive the
//! [`Nvme`] that borrows it, and nothing else may touch it meanwhile.
//!
//! The controller does not snoop this core's caches: every submission is
//! cleaned to RAM before its doorbell, every completion invalidated before
//! it is read, and a data buffer is cleaned before a write and
//! cleaned-and-invalidated around a read, the same as [`crate::sd`]'s DMA.
//!
//! ## Buffers
//!
//! Data moves straight between the device and the caller's buffer when the
//! buffer is 4-byte aligned (the NVMe PRP rule); otherwise it goes through
//! [`Memory`]'s page one page at a time, with a copy. A buffer is any
//! whole number of blocks; [`block_size`](Nvme::block_size) says how big a
//! block is (512 or 4096 bytes on most drives).
//!
//! Register and structure layouts are the NVM Express Base Specification's.

use core::mem::size_of;

use crate::cache;
use crate::pcie::{dma_address, Function};
use crate::timer::Timer;

/// The PCI class code of an NVMe controller: mass storage, non-volatile
/// memory, NVM Express.
pub const CLASS: u32 = 0x01_08_02;

/// The memory page size the controller is set up for (`CC.MPS` = 0), and
/// the PRP entry granularity.
const PAGE: usize = 4096;

// Controller registers (BAR0).
const CAP: usize = 0x00;
const CC: usize = 0x14;
const CSTS: usize = 0x1c;
const INTMS: usize = 0x0c;
const AQA: usize = 0x24;
const ASQ: usize = 0x28;
const ACQ: usize = 0x30;
const DOORBELLS: usize = 0x1000;

const CC_ENABLE: u32 = 1;
/// `CC.IOSQES` = 6 (64-byte submission entries), `CC.IOCQES` = 4 (16-byte
/// completion entries); `CSS` 0 (NVM command set), `MPS` 0 (4 KiB pages).
const CC_ENTRY_SIZES: u32 = (6 << 16) | (4 << 20);
/// `CC.SHN` = 1: normal shutdown.
const CC_SHUTDOWN_NORMAL: u32 = 1 << 14;
const CSTS_READY: u32 = 1;
const CSTS_FATAL: u32 = 1 << 1;
/// `CSTS.SHST`, bits 3:2: 2 = shutdown processing complete.
const CSTS_SHUTDOWN_MASK: u32 = 0b11 << 2;
const CSTS_SHUTDOWN_DONE: u32 = 0b10 << 2;

// Admin command opcodes.
const ADMIN_CREATE_IO_SQ: u8 = 0x01;
const ADMIN_CREATE_IO_CQ: u8 = 0x05;
const ADMIN_IDENTIFY: u8 = 0x06;

// NVM command opcodes.
const IO_FLUSH: u8 = 0x00;
const IO_WRITE: u8 = 0x01;
const IO_READ: u8 = 0x02;

/// Identify `CNS` values.
const IDENTIFY_NAMESPACE: u32 = 0;
const IDENTIFY_CONTROLLER: u32 = 1;

/// The namespace this driver uses. Drives as shipped have exactly one,
/// numbered 1.
const NAMESPACE: u32 = 1;

/// Submission queue entries per queue: one page of 64-byte entries.
const SQ_ENTRIES: usize = PAGE / size_of::<SubmissionEntry>();
/// Completion queue entries per queue: one page of 16-byte entries.
const CQ_ENTRIES: usize = PAGE / size_of::<CompletionEntry>();
/// PRP entries in the one-page list.
const PRP_ENTRIES: usize = PAGE / size_of::<u64>();

/// The largest transfer one command carries, whatever the controller
/// allows: the first, possibly partial, page through PRP1 and up to a full
/// list of [`PRP_ENTRIES`] pages after it never needs a chained list.
const MAX_TRANSFER: usize = PRP_ENTRIES * PAGE;

/// How long one command may take before [`Error::Timeout`]. Generous, for
/// a flush of a drive's whole write cache.
const COMMAND_TIMEOUT_US: u64 = 10_000_000;

/// A 64-byte submission queue entry.
#[derive(Clone, Copy, Default)]
#[repr(C)]
struct SubmissionEntry {
    /// Opcode in bits 7:0, command identifier in bits 31:16; PRPs
    /// (`PSDT` 0) for the data.
    cdw0: u32,
    nsid: u32,
    _reserved: [u32; 2],
    metadata: u64,
    prp1: u64,
    prp2: u64,
    cdw10: u32,
    cdw11: u32,
    cdw12: u32,
    cdw13: u32,
    cdw14: u32,
    cdw15: u32,
}

/// A 16-byte completion queue entry: command-specific result, reserved,
/// submission queue head and ID, then command identifier (bits 15:0),
/// phase (bit 16) and status (bits 31:17).
#[derive(Clone, Copy, Default)]
#[repr(C)]
struct CompletionEntry([u32; 4]);

/// One 4 KiB, page-aligned page.
#[derive(Clone, Copy)]
#[repr(C, align(4096))]
struct Page([u8; PAGE]);

/// The memory a controller reads and writes directly: admin and I/O queue
/// pairs, a PRP list and a scratch page. 24 KiB, page aligned.
///
/// Make one with [`Memory::new`] wherever suits — a `static`, the heap,
/// the stack — and lend it to [`Nvme::new`] for as long as the controller
/// runs.
#[repr(C, align(4096))]
pub struct Memory {
    admin_sq: [SubmissionEntry; SQ_ENTRIES],
    admin_cq: [CompletionEntry; CQ_ENTRIES],
    io_sq: [SubmissionEntry; SQ_ENTRIES],
    io_cq: [CompletionEntry; CQ_ENTRIES],
    prp_list: [u64; PRP_ENTRIES],
    scratch: Page,
}

impl Memory {
    /// Zeroed queue memory.
    pub const fn new() -> Self {
        const ZERO_SQ: SubmissionEntry = SubmissionEntry {
            cdw0: 0,
            nsid: 0,
            _reserved: [0; 2],
            metadata: 0,
            prp1: 0,
            prp2: 0,
            cdw10: 0,
            cdw11: 0,
            cdw12: 0,
            cdw13: 0,
            cdw14: 0,
            cdw15: 0,
        };
        Self {
            admin_sq: [ZERO_SQ; SQ_ENTRIES],
            admin_cq: [CompletionEntry([0; 4]); CQ_ENTRIES],
            io_sq: [ZERO_SQ; SQ_ENTRIES],
            io_cq: [CompletionEntry([0; 4]); CQ_ENTRIES],
            prp_list: [0; PRP_ENTRIES],
            scratch: Page([0; PAGE]),
        }
    }
}

impl Default for Memory {
    /// Same as [`Memory::new`].
    fn default() -> Self {
        Self::new()
    }
}

/// Why a controller could not be brought up, or a command failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The function is not an NVMe controller ([`CLASS`]), or has no BAR0.
    NotNvme,
    /// The controller does not support the NVM command set, or 4 KiB
    /// memory pages.
    Unsupported,
    /// Namespace 1 is missing, or is formatted with per-block metadata or
    /// a block size outside 512 B-4 KiB, which this driver does not handle.
    NoNamespace,
    /// The controller reported a fatal status (`CSTS.CFS`).
    Fatal,
    /// The controller did not become ready, finish shutting down, or
    /// complete a command in time.
    Timeout,
    /// The controller completed a command with an error status.
    Command {
        /// The command's opcode.
        opcode: u8,
        /// The completion status field: status code in bits 7:0, status
        /// code type in bits 10:8.
        status: u16,
    },
    /// The buffer is not a whole number of blocks.
    Length,
    /// The transfer runs past the end of the namespace.
    OutOfRange,
    /// [`NvmeBlockDevice::new`] was given a drive whose blocks are not the
    /// 512 bytes `resident-fat` moves.
    #[cfg(feature = "resident-fat")]
    BlockSize {
        /// The drive's block size, in bytes.
        bytes: usize,
    },
}

/// Which queue pair a command goes to.
#[derive(Clone, Copy)]
enum Queue {
    Admin,
    Io,
}

/// A queue pair's software state.
struct QueueState {
    /// The queue's ID: 0 for admin, 1 for I/O.
    id: u16,
    sq_entries: u16,
    cq_entries: u16,
    sq_tail: u16,
    cq_head: u16,
    /// The phase bit a new completion carries; flips each time the head
    /// wraps.
    phase: bool,
    /// The next command identifier.
    next_id: u16,
}

impl QueueState {
    fn new(id: u16, sq_entries: u16, cq_entries: u16) -> Self {
        Self {
            id,
            sq_entries,
            cq_entries,
            sq_tail: 0,
            cq_head: 0,
            phase: true,
            next_id: 0,
        }
    }
}

/// An NVMe controller, up and running, with namespace 1 ready for I/O.
pub struct Nvme<'m> {
    /// BAR0's CPU address.
    registers: usize,
    /// Bytes between doorbell registers (`4 << CAP.DSTRD`).
    doorbell_stride: usize,
    memory: &'m mut Memory,
    admin: QueueState,
    io: QueueState,
    block_size: usize,
    block_count: u64,
    max_transfer: usize,
    model: [u8; 40],
    serial: [u8; 20],
    firmware: [u8; 8],
}

impl<'m> Nvme<'m> {
    /// Brings up the NVMe controller `function`: resets it, enables it
    /// with admin queues in `memory`, identifies it and namespace 1, and
    /// creates one I/O queue pair.
    ///
    /// The controller's reset may take up to its own advertised timeout
    /// (`CAP.TO`, typically a few seconds at most); after that, about a
    /// millisecond.
    pub fn new(function: &Function, memory: &'m mut Memory, timer: &Timer) -> Result<Self, Error> {
        if function.info.class != CLASS {
            return Err(Error::NotNvme);
        }
        let bar = function.bars[0].ok_or(Error::NotNvme)?;

        let mut nvme = Self {
            registers: bar.address,
            doorbell_stride: 4,
            memory,
            admin: QueueState::new(0, 0, 0),
            io: QueueState::new(1, 0, 0),
            block_size: 0,
            block_count: 0,
            max_transfer: 0,
            model: [0; 40],
            serial: [0; 20],
            firmware: [0; 8],
        };

        let cap = u64::from(nvme.read(CAP)) | (u64::from(nvme.read(CAP + 4)) << 32);
        let max_entries = (cap & 0xffff) as usize + 1;
        // `CAP.TO` is in 500 ms units.
        let ready_timeout_us = ((cap >> 24) & 0xff).max(1) * 500_000;
        nvme.doorbell_stride = 4 << ((cap >> 32) & 0xf);
        let nvm_command_set = cap & (1 << 37) != 0;
        let page_min = (cap >> 48) & 0xf;
        if !nvm_command_set || page_min != 0 {
            return Err(Error::Unsupported);
        }

        // Disable (resetting the controller, and with it any queues a
        // bootloader set up) and wait for it to say so.
        nvme.write(CC, 0);
        nvme.wait_status(CSTS_READY, 0, ready_timeout_us, timer)?;

        // Fresh, zeroed queues, written back to RAM before the controller
        // first looks at them, with no stale lines left to be evicted over
        // what it writes.
        // SAFETY: all-zero is a valid `Memory` (it is what `Memory::new`
        // builds); written in place rather than assigned, so 24 KiB of
        // zeroes never pass through the stack.
        unsafe { core::ptr::write_bytes(nvme.memory as *mut Memory, 0, 1) };
        cache::clean_invalidate_bytes(nvme.memory as *const Memory as usize, size_of::<Memory>());

        let sq_entries = SQ_ENTRIES.min(max_entries) as u16;
        let cq_entries = CQ_ENTRIES.min(max_entries) as u16;
        nvme.admin = QueueState::new(0, sq_entries, cq_entries);
        nvme.io = QueueState::new(1, sq_entries, cq_entries);

        // Sizes are 0-based.
        nvme.write(
            AQA,
            (u32::from(cq_entries - 1) << 16) | u32::from(sq_entries - 1),
        );
        let admin_sq = dma_address(nvme.memory.admin_sq.as_ptr() as usize);
        let admin_cq = dma_address(nvme.memory.admin_cq.as_ptr() as usize);
        nvme.write(ASQ, admin_sq as u32);
        nvme.write(ASQ + 4, (admin_sq >> 32) as u32);
        nvme.write(ACQ, admin_cq as u32);
        nvme.write(ACQ + 4, (admin_cq >> 32) as u32);
        // Mask every interrupt vector: completions are polled.
        nvme.write(INTMS, u32::MAX);

        nvme.write(CC, CC_ENTRY_SIZES | CC_ENABLE);
        nvme.wait_status(CSTS_READY, CSTS_READY, ready_timeout_us, timer)?;

        nvme.identify(timer)?;
        nvme.create_io_queues(timer)?;
        Ok(nvme)
    }

    /// Bytes per logical block.
    pub fn block_size(&self) -> usize {
        self.block_size
    }

    /// The namespace's size, in blocks.
    pub fn block_count(&self) -> u64 {
        self.block_count
    }

    /// The most bytes one command moves: the controller's own limit
    /// (`MDTS`), or this driver's (2 MiB), whichever is smaller.
    /// [`read_blocks`](Self::read_blocks) and
    /// [`write_blocks`](Self::write_blocks) split anything longer.
    pub fn max_transfer(&self) -> usize {
        self.max_transfer
    }

    /// The model number the controller reports, trailing spaces trimmed.
    pub fn model(&self) -> &str {
        ascii_field(&self.model)
    }

    /// The serial number the controller reports, trailing spaces trimmed.
    pub fn serial(&self) -> &str {
        ascii_field(&self.serial)
    }

    /// The firmware revision the controller reports, trailing spaces
    /// trimmed.
    pub fn firmware(&self) -> &str {
        ascii_field(&self.firmware)
    }

    /// Reads `buffer.len() / block_size()` blocks starting at block `lba`.
    pub fn read_blocks(&mut self, lba: u64, buffer: &mut [u8], timer: &Timer) -> Result<(), Error> {
        self.check_range(lba, buffer.len())?;
        let address = buffer.as_mut_ptr() as usize;
        if address.is_multiple_of(4) {
            self.transfer(IO_READ, lba, address, buffer.len(), timer)
        } else {
            let blocks_per_page = (PAGE / self.block_size) as u64;
            for (i, chunk) in buffer.chunks_mut(PAGE).enumerate() {
                let scratch = self.memory.scratch.0.as_ptr() as usize;
                self.transfer(
                    IO_READ,
                    lba + i as u64 * blocks_per_page,
                    scratch,
                    chunk.len(),
                    timer,
                )?;
                chunk.copy_from_slice(&self.memory.scratch.0[..chunk.len()]);
            }
            Ok(())
        }
    }

    /// Writes `buffer.len() / block_size()` blocks starting at block `lba`.
    ///
    /// Done when the controller says so, which may be before the data is
    /// on flash: a drive with a volatile write cache holds it there until
    /// [`flush`](Self::flush) or [`shutdown`](Self::shutdown).
    pub fn write_blocks(&mut self, lba: u64, buffer: &[u8], timer: &Timer) -> Result<(), Error> {
        self.check_range(lba, buffer.len())?;
        let address = buffer.as_ptr() as usize;
        if address.is_multiple_of(4) {
            self.transfer(IO_WRITE, lba, address, buffer.len(), timer)
        } else {
            let blocks_per_page = (PAGE / self.block_size) as u64;
            for (i, chunk) in buffer.chunks(PAGE).enumerate() {
                self.memory.scratch.0[..chunk.len()].copy_from_slice(chunk);
                let scratch = self.memory.scratch.0.as_ptr() as usize;
                self.transfer(
                    IO_WRITE,
                    lba + i as u64 * blocks_per_page,
                    scratch,
                    chunk.len(),
                    timer,
                )?;
            }
            Ok(())
        }
    }

    /// Commits everything written so far to non-volatile media.
    pub fn flush(&mut self, timer: &Timer) -> Result<(), Error> {
        let command = SubmissionEntry {
            cdw0: u32::from(IO_FLUSH),
            nsid: NAMESPACE,
            ..Default::default()
        };
        self.submit(Queue::Io, command, timer).map(|_| ())
    }

    /// Shuts the controller down the way the specification asks before
    /// power is removed: it flushes its caches and stops. Consumes the
    /// driver, since the controller takes no more commands.
    pub fn shutdown(self, timer: &Timer) -> Result<(), Error> {
        self.write(CC, self.read(CC) | CC_SHUTDOWN_NORMAL);
        self.wait_status(
            CSTS_SHUTDOWN_MASK,
            CSTS_SHUTDOWN_DONE,
            COMMAND_TIMEOUT_US,
            timer,
        )
    }

    /// Reads the identify data this driver needs: the controller's names
    /// and transfer limit, then namespace 1's size and block format.
    fn identify(&mut self, timer: &Timer) -> Result<(), Error> {
        let data = self.identify_page(IDENTIFY_CONTROLLER, 0, timer)?;
        self.serial.copy_from_slice(&data[4..24]);
        self.model.copy_from_slice(&data[24..64]);
        self.firmware.copy_from_slice(&data[64..72]);
        // `MDTS`: a power of two of minimum pages (4 KiB, checked in
        // `new`), 0 for no limit.
        let mdts = data[77];
        self.max_transfer = if mdts == 0 || mdts >= 20 {
            MAX_TRANSFER
        } else {
            (PAGE << mdts).min(MAX_TRANSFER)
        };

        let data = self.identify_page(IDENTIFY_NAMESPACE, NAMESPACE, timer)?;
        let size = u64::from_le_bytes(data[0..8].try_into().unwrap());
        // `FLBAS`: the format in use, bits 3:0 (bits 6:5 extend it past 16).
        let flbas = data[26];
        let format = usize::from(flbas & 0xf) | (usize::from(flbas >> 5 & 0b11) << 4);
        let descriptor = 128 + 4 * format;
        if size == 0 || descriptor + 4 > PAGE {
            return Err(Error::NoNamespace);
        }
        let metadata = u16::from_le_bytes([data[descriptor], data[descriptor + 1]]);
        let block_shift = data[descriptor + 2];
        // Metadata either interleaves with the data (`FLBAS` bit 4) or is
        // moved through a separate pointer every command would need; a
        // drive formatted with none is the only kind handled.
        if metadata != 0 || !(9..=12).contains(&block_shift) {
            return Err(Error::NoNamespace);
        }
        self.block_count = size;
        self.block_size = 1 << block_shift;
        self.max_transfer -= self.max_transfer % self.block_size;
        Ok(())
    }

    /// Runs Identify with `cns` (and `nsid`) into the scratch page and
    /// returns it.
    fn identify_page(&mut self, cns: u32, nsid: u32, timer: &Timer) -> Result<[u8; PAGE], Error> {
        let scratch = self.memory.scratch.0.as_ptr() as usize;
        cache::clean_invalidate_bytes(scratch, PAGE);
        let command = SubmissionEntry {
            cdw0: u32::from(ADMIN_IDENTIFY),
            nsid,
            prp1: dma_address(scratch),
            cdw10: cns,
            ..Default::default()
        };
        self.submit(Queue::Admin, command, timer)?;
        cache::invalidate_bytes(scratch, PAGE);
        Ok(self.memory.scratch.0)
    }

    /// Creates I/O completion queue 1 and then submission queue 1 feeding
    /// it, both physically contiguous, with interrupts off.
    fn create_io_queues(&mut self, timer: &Timer) -> Result<(), Error> {
        let id = u32::from(self.io.id);
        let cq = SubmissionEntry {
            cdw0: u32::from(ADMIN_CREATE_IO_CQ),
            prp1: dma_address(self.memory.io_cq.as_ptr() as usize),
            cdw10: (u32::from(self.io.cq_entries - 1) << 16) | id,
            // `PC` (bit 0): physically contiguous; `IEN` (bit 1) clear.
            cdw11: 1,
            ..Default::default()
        };
        self.submit(Queue::Admin, cq, timer)?;
        let sq = SubmissionEntry {
            cdw0: u32::from(ADMIN_CREATE_IO_SQ),
            prp1: dma_address(self.memory.io_sq.as_ptr() as usize),
            cdw10: (u32::from(self.io.sq_entries - 1) << 16) | id,
            // The completion queue in bits 31:16, `PC` in bit 0.
            cdw11: (id << 16) | 1,
            ..Default::default()
        };
        self.submit(Queue::Admin, sq, timer).map(|_| ())
    }

    /// Checks that `len` bytes is whole blocks, all inside the namespace
    /// from `lba`.
    fn check_range(&self, lba: u64, len: usize) -> Result<(), Error> {
        if !len.is_multiple_of(self.block_size) {
            return Err(Error::Length);
        }
        let blocks = (len / self.block_size) as u64;
        match lba.checked_add(blocks) {
            Some(end) if end <= self.block_count => Ok(()),
            _ => Err(Error::OutOfRange),
        }
    }

    /// Moves `len` bytes (whole blocks) between the device and the
    /// 4-byte-aligned buffer at `address`, one command per
    /// [`max_transfer`](Self::max_transfer) bytes.
    fn transfer(
        &mut self,
        opcode: u8,
        mut lba: u64,
        mut address: usize,
        len: usize,
        timer: &Timer,
    ) -> Result<(), Error> {
        let end = address + len;
        while address < end {
            let chunk = (end - address).min(self.max_transfer);
            let (prp1, prp2) = self.prps(address, chunk);

            // Before a read, clean as well as invalidate, so a dirty
            // neighbour sharing an end line reaches RAM before the device
            // writes and the invalidate afterwards cannot discard it.
            if opcode == IO_WRITE {
                cache::clean_bytes(address, chunk);
            } else {
                cache::clean_invalidate_bytes(address, chunk);
            }

            let blocks = (chunk / self.block_size) as u32;
            let command = SubmissionEntry {
                cdw0: u32::from(opcode),
                nsid: NAMESPACE,
                prp1,
                prp2,
                cdw10: lba as u32,
                cdw11: (lba >> 32) as u32,
                // Number of blocks, 0-based.
                cdw12: blocks - 1,
                ..Default::default()
            };
            self.submit(Queue::Io, command, timer)?;
            if opcode == IO_READ {
                cache::invalidate_bytes(address, chunk);
            }

            address += chunk;
            lba += u64::from(blocks);
        }
        Ok(())
    }

    /// The two PRP entries for `len` bytes at `address`: the first page
    /// (from `address` to its page boundary) is PRP1; PRP2 is the second
    /// page if that is all there is, or else the list in [`Memory`] naming
    /// every page after the first. `len` is at most [`MAX_TRANSFER`], which
    /// the list always covers.
    fn prps(&mut self, address: usize, len: usize) -> (u64, u64) {
        let first_page_len = PAGE - address % PAGE;
        let second_page = address - address % PAGE + PAGE;
        let prp2 = if len <= first_page_len {
            0
        } else if len <= first_page_len + PAGE {
            dma_address(second_page)
        } else {
            let pages = (len - first_page_len).div_ceil(PAGE);
            for (i, entry) in self.memory.prp_list[..pages].iter_mut().enumerate() {
                *entry = dma_address(second_page + i * PAGE);
            }
            let list = self.memory.prp_list.as_ptr() as usize;
            cache::clean_bytes(list, pages * size_of::<u64>());
            dma_address(list)
        };
        (dma_address(address), prp2)
    }

    /// Submits `command` to `queue`, rings the doorbell and polls for its
    /// completion. Returns the completion's result dword.
    fn submit(
        &mut self,
        queue: Queue,
        mut command: SubmissionEntry,
        timer: &Timer,
    ) -> Result<u32, Error> {
        let (state, sq, cq) = match queue {
            Queue::Admin => (
                &mut self.admin,
                self.memory.admin_sq.as_mut_ptr(),
                self.memory.admin_cq.as_ptr(),
            ),
            Queue::Io => (
                &mut self.io,
                self.memory.io_sq.as_mut_ptr(),
                self.memory.io_cq.as_ptr(),
            ),
        };
        let opcode = command.cdw0 as u8;
        let command_id = state.next_id;
        state.next_id = state.next_id.wrapping_add(1);
        command.cdw0 |= u32::from(command_id) << 16;

        // SAFETY: `sq_tail` is always below `sq_entries`, which is at most
        // the array's length.
        let slot = unsafe { sq.add(usize::from(state.sq_tail)) };
        unsafe { slot.write_volatile(command) };
        cache::clean_bytes(slot as usize, size_of::<SubmissionEntry>());
        state.sq_tail = (state.sq_tail + 1) % state.sq_entries;
        let sq_doorbell = DOORBELLS + 2 * usize::from(state.id) * self.doorbell_stride;
        let cq_doorbell = sq_doorbell + self.doorbell_stride;
        // SAFETY: the controller's doorbell for this queue.
        unsafe {
            ((self.registers + sq_doorbell) as *mut u32).write_volatile(u32::from(state.sq_tail))
        };

        // SAFETY: as for `slot`, for the completion queue.
        let entry = unsafe { cq.add(usize::from(state.cq_head)) };
        let start = timer.now_micros();
        let completion = loop {
            cache::invalidate_bytes(entry as usize, size_of::<CompletionEntry>());
            // SAFETY: the completion slot, in memory this driver borrows.
            let completion = unsafe { entry.read_volatile() };
            if (completion.0[3] & (1 << 16) != 0) == state.phase {
                break completion;
            }
            if timer.now_micros() - start > COMMAND_TIMEOUT_US {
                return Err(Error::Timeout);
            }
            core::hint::spin_loop();
        };

        state.cq_head += 1;
        if state.cq_head == state.cq_entries {
            state.cq_head = 0;
            state.phase = !state.phase;
        }
        let cq_head = u32::from(state.cq_head);
        // SAFETY: the controller's doorbell for this queue.
        unsafe { ((self.registers + cq_doorbell) as *mut u32).write_volatile(cq_head) };

        let status = (completion.0[3] >> 17) as u16;
        // One command in flight at a time, so the completion is this one;
        // a mismatch means an earlier timed-out command finished late.
        debug_assert_eq!(state.id, (completion.0[2] >> 16) as u16);
        if status & 0x7ff != 0 || completion.0[3] as u16 != command_id {
            return Err(Error::Command { opcode, status });
        }
        Ok(completion.0[0])
    }

    /// Polls `CSTS` until `(CSTS & mask) == value`, or the controller
    /// reports a fatal error, or `timeout_us` passes.
    fn wait_status(
        &self,
        mask: u32,
        value: u32,
        timeout_us: u64,
        timer: &Timer,
    ) -> Result<(), Error> {
        let start = timer.now_micros();
        loop {
            let status = self.read(CSTS);
            if status & mask == value {
                return Ok(());
            }
            // A controller that has gone away reads all-ones, which would
            // otherwise look fatal: treat it as not answering.
            if status != u32::MAX && status & CSTS_FATAL != 0 {
                return Err(Error::Fatal);
            }
            if timer.now_micros() - start > timeout_us {
                return Err(Error::Timeout);
            }
            timer.delay_us(100);
        }
    }

    fn read(&self, offset: usize) -> u32 {
        // SAFETY: a register in the controller's BAR0.
        unsafe { ((self.registers + offset) as *const u32).read_volatile() }
    }

    fn write(&self, offset: usize, value: u32) {
        // SAFETY: a register in the controller's BAR0.
        unsafe { ((self.registers + offset) as *mut u32).write_volatile(value) }
    }
}

/// An identify string field, as text: space-padded ASCII, so trimmed; and
/// anything that isn't ASCII cut off at the first such byte.
fn ascii_field(field: &[u8]) -> &str {
    let ascii = field
        .iter()
        .position(|b| !b.is_ascii())
        .unwrap_or(field.len());
    // ASCII is always valid UTF-8.
    core::str::from_utf8(&field[..ascii])
        .unwrap_or("")
        .trim_end_matches([' ', '\0'])
}

/// An [`Nvme`] drive wrapped as a `resident-fat`
/// [`BlockDevice`](resident_fat::BlockDevice), so that crate's FAT32
/// filesystem can be layered on top — the NVMe counterpart of
/// [`crate::sd::SdBlockDevice`], bundling the drive with a borrow of the
/// [`Timer`] its commands need for their timeouts.
///
/// Transfers go straight to [`Nvme::read_blocks`] and
/// [`Nvme::write_blocks`], so a 4-byte-aligned buffer moves without a copy.
/// [`max_transfer_blocks`] is [`Nvme::max_transfer`] in blocks, the most one
/// command carries, so `resident-fat` sees what each transfer costs rather
/// than the driver splitting longer ones behind its back. [`block_count`]
/// is the namespace's size, which the driver knows from identify.
///
/// Only drives formatted with 512-byte blocks: `resident-fat` moves
/// 512-byte blocks, and translating them onto a 4 KiB-block drive would
/// mean a read-modify-write for every write smaller than a page.
/// [`new`](Self::new) refuses anything else with [`Error::BlockSize`].
///
/// `resident-fat` uses `alloc`, so a binary that enables this feature must
/// register a `#[global_allocator]`.
///
/// Available only with the `resident-fat` feature enabled.
///
/// [`max_transfer_blocks`]: resident_fat::BlockDevice::max_transfer_blocks
/// [`block_count`]: resident_fat::BlockDevice::block_count
#[cfg(feature = "resident-fat")]
pub struct NvmeBlockDevice<'m, 't> {
    nvme: Nvme<'m>,
    timer: &'t Timer,
}

#[cfg(feature = "resident-fat")]
impl<'m, 't> NvmeBlockDevice<'m, 't> {
    /// Wraps a running [`Nvme`] and the [`Timer`] its commands need, or
    /// hands the drive's block size back as [`Error::BlockSize`] if it is
    /// not 512 bytes.
    pub fn new(nvme: Nvme<'m>, timer: &'t Timer) -> Result<Self, Error> {
        let bytes = nvme.block_size();
        if bytes != resident_fat::BLOCK_SIZE {
            return Err(Error::BlockSize { bytes });
        }
        Ok(Self { nvme, timer })
    }

    /// The wrapped drive, borrowed.
    ///
    /// `resident-fat` owns the device once a volume is mounted and lends it
    /// back through its own accessors, so this is the way to reach the
    /// driver's own methods — its identity, say — without unmounting.
    pub fn inner(&self) -> &Nvme<'m> {
        &self.nvme
    }

    /// Unwraps back to the drive, dropping the timer borrow — to
    /// [`shutdown`](Nvme::shutdown) it before power goes, for one.
    pub fn into_inner(self) -> Nvme<'m> {
        self.nvme
    }
}

#[cfg(feature = "resident-fat")]
impl resident_fat::BlockDevice for NvmeBlockDevice<'_, '_> {
    type Error = Error;

    /// Reads a run of consecutive blocks, one command per
    /// [`Nvme::max_transfer`] bytes. A length that is not a whole number of
    /// blocks is [`Error::Length`], and a run past the end of the namespace
    /// [`Error::OutOfRange`].
    fn read(&mut self, start_block: u64, blocks: &mut [u8]) -> Result<(), Self::Error> {
        self.nvme.read_blocks(start_block, blocks, self.timer)
    }

    /// Writes a run of consecutive blocks — the mirror of
    /// [`read`](resident_fat::BlockDevice::read), with the same errors.
    ///
    /// Done when the controller has the data, which on a drive with a
    /// volatile write cache may be before it is on flash: see
    /// [`Nvme::write_blocks`].
    fn write(&mut self, start_block: u64, blocks: &[u8]) -> Result<(), Self::Error> {
        self.nvme.write_blocks(start_block, blocks, self.timer)
    }

    /// The namespace's size, from the controller's identify data.
    fn block_count(&mut self) -> Result<Option<u64>, Self::Error> {
        Ok(Some(self.nvme.block_count()))
    }

    /// [`Nvme::max_transfer`] in blocks: the most one command moves.
    fn max_transfer_blocks(&self) -> u64 {
        (self.nvme.max_transfer() / resident_fat::BLOCK_SIZE) as u64
    }
}
