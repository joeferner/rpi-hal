//! Blocking driver for the SDHOST controller — the *other* SD host on
//! this SoC, and the one that lets a board drive its card and its
//! wireless chip at the same time.
//!
//! # Why this exists when [`crate::sd`] already works
//!
//! BCM2835/6/7 has two SD host controllers:
//!
//! * the Arasan/SDHCI part ([`crate::sd`], [`crate::sdio`]), reachable
//!   at GPIO48-53 (ALT3, the card slot) **or** GPIO34-39 (ALT3, the
//!   wireless chip) — one or the other, never both, because it is one
//!   controller behind a mux;
//! * SDHOST, this one: Broadcom's own, simpler part, reachable only at
//!   GPIO48-53 (ALT0, the card slot).
//!
//! A program that wants Wi-Fi *and* the card therefore has to put the
//! card here and leave the Arasan controller for [`crate::sdio`]. That
//! is the split Raspberry Pi OS makes on a Pi 3 and a Zero W, and it is
//! the only arrangement in which both work at once.
//!
//! It does not go the other way round: SDHOST has no usable SDIO
//! support, so the wireless chip cannot be moved here to free the Arasan
//! controller for the card.
//!
//! [`crate::sdio`] already routes GPIO48-53 to ALT0 when it takes the
//! wireless pins — it has to, or the one controller would be wired to
//! both pin groups — so on a board that has brought Wi-Fi up, the card
//! slot is already pointed at this controller and nothing is driving it.
//!
//! # What this is not
//!
//! Not a replacement for [`crate::sd`]. That driver is the one to use
//! when the card is all a program wants: it has the DMA and
//! interrupt-driven paths, and it has been run against far more cards.
//! This one is for the case above.
//!
//! The card-side protocol — `CMD0`, `CMD8`, `ACMD41`, `CMD2`, `CMD3`,
//! `CMD7`, the 4-bit negotiation — is the same sequence [`crate::sd`]
//! performs, and is deliberately written out again here rather than
//! shared. What differs between the two is every register that sequence
//! is expressed in; what they have in common is a page of the SD
//! specification. Factoring that out would mean a trait over two
//! controllers' command mechanics to save repeating a list of command
//! indices, and it would put a verified driver's bring-up path at the
//! mercy of edits made for this one.
//!
//! # Register map
//!
//! Not in the SVD — like the RNG and the legacy GPIO pull registers,
//! SDHOST is absent from the peripheral access crate — so this pokes
//! its physical addresses directly. The register layout, the FIFO
//! thresholds, the clock divisor and the state-machine codes follow
//! Linux's `bcm2835-sdhost` driver, which is the reference for this
//! controller: the BCM2835 datasheet does not document it at all.
//!
//! # Blocking, polled, PIO
//!
//! Every transfer is the CPU moving words to and from `SDDATA`, paced
//! against the FIFO level that `SDEDM` reports. No DMA and no
//! interrupts: what this driver is for is a program whose card access is
//! occasional — settings at boot, an update now and then — while the
//! Arasan controller does the work that has to keep up.

use crate::mailbox::{ClockId, Mailbox, PowerDeviceId};
use crate::pac::GPIO;
use crate::timer::Timer;

/// SDHOST register block, ARM physical (bus `0x7E20_2000`).
///
/// The peripheral base comes from [`crate::soc`], which is the one place
/// in this crate that knows which chip's memory map is being built for.
/// A driver that writes the address out instead compiles for every chip
/// and works on one — see the comments in [`crate::power`] and
/// [`crate::rng`], which were both that bug.
const BASE: usize = crate::soc::PERIPHERAL_BASE as usize + 0x0020_2000;

/// Command register: index, flags, and the "new command" bit that
/// starts it.
const SDCMD: *mut u32 = BASE as *mut u32;
/// Argument for the command in `SDCMD`. Written first.
const SDARG: *mut u32 = (BASE + 0x04) as *mut u32;
/// Data-line timeout, in clocks of the current SD clock.
const SDTOUT: *mut u32 = (BASE + 0x08) as *mut u32;
/// Clock divisor: the SD clock is the core clock over `SDCDIV + 2`.
const SDCDIV: *mut u32 = (BASE + 0x0c) as *mut u32;
/// Response word 0: bits 31:0 of a short response, or of a long one.
///
/// A long (136-bit) response's other three words follow at `0x14`,
/// `0x18` and `0x1c`. Nothing here reads them — the one long response
/// this driver asks for is `CMD2`'s, which is issued to move the card
/// into identification state rather than for the card identity it
/// carries — so they are named here and not declared.
const SDRSP0: *mut u32 = (BASE + 0x10) as *mut u32;
/// Host status: the error and completion flags, write-1-to-clear.
const SDHSTS: *mut u32 = (BASE + 0x20) as *mut u32;
/// Card power control: 1 turns the bus on.
const SDVDD: *mut u32 = (BASE + 0x30) as *mut u32;
/// Extended data mode: FIFO thresholds, the FIFO level, and the data
/// state machine's current state.
const SDEDM: *mut u32 = (BASE + 0x34) as *mut u32;
/// Host configuration: bus width and interrupt enables.
const SDHCFG: *mut u32 = (BASE + 0x38) as *mut u32;
/// Byte count of one block, for a data command.
const SDHBCT: *mut u32 = (BASE + 0x3c) as *mut u32;
/// The data FIFO. One 32-bit word per access, either direction.
const SDDATA: *mut u32 = (BASE + 0x40) as *mut u32;
/// Number of blocks, for a data command.
const SDHBLC: *mut u32 = (BASE + 0x50) as *mut u32;

/// `SDCMD`: set to start the command, cleared by the controller when it
/// finishes. Polled as the completion flag.
const SDCMD_NEW_FLAG: u32 = 0x8000;
/// `SDCMD`: the command failed. Set alongside the clearing of
/// [`SDCMD_NEW_FLAG`], with `SDHSTS` saying how.
const SDCMD_FAIL_FLAG: u32 = 0x4000;
/// `SDCMD`: wait for the card to release the busy signal before
/// reporting completion (`R1b` responses).
const SDCMD_BUSYWAIT: u32 = 0x800;
/// `SDCMD`: this command has no response to collect.
const SDCMD_NO_RESPONSE: u32 = 0x400;
/// `SDCMD`: collect a 136-bit response rather than a 48-bit one.
const SDCMD_LONG_RESPONSE: u32 = 0x200;
/// `SDCMD`: a data command writing to the card.
const SDCMD_WRITE_CMD: u32 = 0x80;
/// `SDCMD`: a data command reading from the card.
const SDCMD_READ_CMD: u32 = 0x40;

/// `SDHSTS`: the card has released the busy signal it held after an
/// `R1b` command. Write-1-to-clear, like the error bits.
const SDHSTS_BUSY_IRPT: u32 = 0x400;
/// `SDHSTS`: the card's data-line timeout expired.
const SDHSTS_REW_TIME_OUT: u32 = 0x80;
/// `SDHSTS`: the card did not answer the command.
const SDHSTS_CMD_TIME_OUT: u32 = 0x40;
/// `SDHSTS`: a data block failed its CRC.
const SDHSTS_CRC16_ERROR: u32 = 0x20;
/// `SDHSTS`: a response failed its CRC.
const SDHSTS_CRC7_ERROR: u32 = 0x10;
/// `SDHSTS`: the data FIFO overran or underran.
const SDHSTS_FIFO_ERROR: u32 = 0x08;
/// Everything in `SDHSTS` that means a transfer went wrong.
const SDHSTS_ERROR_MASK: u32 = SDHSTS_CMD_TIME_OUT
    | SDHSTS_REW_TIME_OUT
    | SDHSTS_CRC16_ERROR
    | SDHSTS_CRC7_ERROR
    | SDHSTS_FIFO_ERROR;
/// Every write-1-to-clear bit in `SDHSTS`, for wiping it between
/// commands.
const SDHSTS_CLEAR_MASK: u32 = 0x7f8;

/// `SDHCFG`: report the card's busy signal.
const SDHCFG_BUSY_IRPT_EN: u32 = 1 << 10;
/// `SDHCFG`: never switch to the fast data-mode clock divisor.
///
/// Set unconditionally, and it is not an optimisation to remove. The
/// data-mode divisor is three bits wide, which cannot divide a core
/// clock above 250 MHz down to anything a card will accept — so the
/// controller's automatic switch is a way to overclock the bus past what
/// the card agreed to, on exactly the boards whose core clock is fastest.
const SDHCFG_SLOW_CARD: u32 = 1 << 3;
/// `SDHCFG`: drive the card over four data lines rather than one.
const SDHCFG_WIDE_EXT_BUS: u32 = 1 << 2;
/// `SDHCFG`: the controller's internal bus is wide. Always set.
const SDHCFG_WIDE_INT_BUS: u32 = 1 << 1;

/// `SDEDM`: force the state machine back to data mode, which is how a
/// finished transfer is released from its read-wait/write-start state.
const SDEDM_FORCE_DATA_MODE: u32 = 1 << 19;
/// `SDEDM`: mask of the data state machine's current state.
///
/// The states, for reading one out of an [`Error::TransferFailed`]:
/// 0 ident mode, 1 data mode, 2 read data, 3 write data, 4 read wait,
/// 5 read CRC, 6 write CRC, 7 write wait 1, 8 power down, 9 power up,
/// a write start 1, b write start 2, c generate pulses, d write wait 2,
/// f start power down. Only the four named below are acted on; the rest
/// are named here because what a stalled transfer was doing is the whole
/// of the diagnosis, and a bare nibble in an error is no use without
/// them.
const SDEDM_FSM_MASK: u32 = 0xf;
/// `SDEDM` state: identification mode, which counts as idle.
const SDEDM_FSM_IDENTMODE: u32 = 0x0;
/// `SDEDM` state: data mode, idle between transfers.
const SDEDM_FSM_DATAMODE: u32 = 0x1;
/// `SDEDM` state: a read has stopped with the FIFO full — where a
/// finished read parks until it is forced back to data mode.
const SDEDM_FSM_READWAIT: u32 = 0x4;
/// `SDEDM` state: a write is starting — where a finished write parks,
/// for the same reason.
const SDEDM_FSM_WRITESTART1: u32 = 0xa;
/// `SDEDM`: shift of the FIFO level field.
const SDEDM_FIFO_LEVEL_SHIFT: u32 = 4;
/// `SDEDM`: mask of the FIFO level field, once shifted down.
const SDEDM_FIFO_LEVEL_MASK: u32 = 0x1f;
/// `SDEDM`: shift of the write threshold field.
const SDEDM_WRITE_THRESHOLD_SHIFT: u32 = 9;
/// `SDEDM`: shift of the read threshold field.
const SDEDM_READ_THRESHOLD_SHIFT: u32 = 14;
/// `SDEDM`: mask of either threshold field, once shifted down.
const SDEDM_THRESHOLD_MASK: u32 = 0x1f;

/// Words the data FIFO holds.
const FIFO_WORDS: u32 = 16;
/// Words moved between checks of the FIFO level. Reading the level costs
/// a register read, so a burst amortises it; the burst is half the FIFO
/// so that neither direction can run it dry or overfull between checks.
const FIFO_BURST_WORDS: u32 = 8;
/// FIFO thresholds programmed at reset.
///
/// Four words, well under the FIFO's sixteen, because the controller has
/// a silicon erratum around a full FIFO — Linux's driver limits them the
/// same way and calls it out as a bug workaround rather than tuning.
const FIFO_THRESHOLD: u32 = 4;

/// Clock during card identification. The SD specification caps this at
/// 400 kHz until the card has been selected.
const SETUP_CLOCK_HZ: u32 = 400_000;
/// Clock for data transfer once the card is selected — SD default speed,
/// which every card supports without a mode switch.
const TRANSFER_CLOCK_HZ: u32 = 25_000_000;
/// Widest value `SDCDIV` holds.
const SDCDIV_MAX: u32 = 0x7ff;

/// How long to wait for a command to complete, in microseconds. Generous
/// because it also covers an `R1b` command's busy wait, which is the
/// card doing an erase rather than answering a question.
const COMMAND_TIMEOUT_US: u64 = 1_000_000;
/// How long to wait for the FIFO to move, in microseconds.
const FIFO_TIMEOUT_US: u64 = 500_000;
/// How long to wait for the data state machine to return to idle after a
/// transfer, in microseconds.
const TRANSFER_END_TIMEOUT_US: u64 = 500_000;

/// How long to let a card finish committing a write, in microseconds.
///
/// Generous because this is the card's own programming time, which is
/// its slowest operation by a wide margin and varies by orders of
/// magnitude between cards — a cheap one erasing a block it has to
/// reclaim first is nothing like a fast one overwriting a clean page.
/// What this bounds is a card that has stopped answering, not one that
/// is merely slow.
const PROGRAMMING_TIMEOUT_US: u64 = 5_000_000;

/// Card status: the card is ready to accept data (`READY_FOR_DATA`).
const CARD_STATUS_READY_FOR_DATA: u32 = 1 << 8;
/// Card status: shift of the current-state field.
const CARD_STATUS_STATE_SHIFT: u32 = 9;
/// Card status: mask of the current-state field, once shifted down.
const CARD_STATUS_STATE_MASK: u32 = 0xf;
/// Card state: transfer — the card has finished whatever it was doing
/// and will take another command. A write that is still committing
/// reports "programming" (7) instead.
const CARD_STATE_TRANSFER: u32 = 4;

/// GPIO alternate function (ALT0) routing GPIO48-53 to this controller.
///
/// The card slot's pins are ALT3 for the Arasan controller and ALT0 for
/// this one, which is the whole of the choice between them.
const GPIO_ALT_SDHOST: u8 = 0b100;

/// One 512-byte block, the unit every read and write here moves.
pub type Block = [u8; 512];

/// Errors from [`Sdhost::init`] and the block read and write methods.
///
/// `#[non_exhaustive]` for the reason [`crate::sd::Error`] is: a driver
/// learning to tell two failures apart should not be a breaking change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error {
    /// No card is in the slot: the first command that expects an answer
    /// got none.
    ///
    /// A conclusion rather than a reading — no Raspberry Pi wires a
    /// card-detect line anywhere this crate can look, so asking the card
    /// is the only detection there is.
    NoCard,
    /// The controller never finished a command.
    ///
    /// Distinct from the card refusing one: this is the `SDCMD` register
    /// still holding its start bit after the budget expired, which is the
    /// controller itself hung rather than an answer that did not come.
    ControllerBusy,
    /// A command completed with the controller's failure flag set.
    CommandFailed {
        /// The command index that failed.
        command: u8,
        /// `SDHSTS` when it failed: which of the timeout and CRC bits
        /// fired.
        status: u32,
    },
    /// A data transfer did not finish in time, or finished with an error
    /// flagged.
    TransferFailed {
        /// `SDHSTS` at the point it was abandoned.
        status: u32,
        /// `SDEDM` at the same moment — its low nibble is the data state
        /// machine's state, which says whether the controller was still
        /// moving data, waiting on the card, or idle when the transfer
        /// was given up on.
        edm: u32,
    },
    /// The card would not accept the voltage window offered during
    /// identification (`ACMD41`) — an unsupported or non-SD card.
    UnusableCard,
    /// The firmware's "set power state" call for the SD card domain
    /// failed, or the core clock could not be read.
    PowerOnFailed,
    /// A multi-block transfer asked for more blocks than `SDHBLC`'s
    /// 16-bit field can express. Split it.
    TooManyBlocks,
}

/// A configured card on the SDHOST controller, ready for block reads and
/// writes.
pub struct Sdhost {
    /// Whether the card addresses blocks rather than bytes (SDHC/SDXC).
    high_capacity: bool,
    /// The card's relative address, as the argument `CMD7` and the
    /// application-command prefix want it: already in the top 16 bits.
    rca: u32,
    /// Whether the four-bit bus was negotiated.
    four_bit_bus: bool,
    /// The core clock this controller divides, read once at bring-up.
    base_clock_hz: u32,
}

impl Sdhost {
    /// Brings the card up on the SDHOST controller: routes GPIO48-53 to
    /// it, resets it, and runs the SD identification sequence.
    ///
    /// **Leaves the Arasan controller alone**, which is the point — see
    /// the module documentation. A program that also wants Wi-Fi brings
    /// that up through [`crate::sdio`], in either order.
    ///
    /// Takes no peripheral token because there is none to take: SDHOST
    /// is not in the SVD, so this reaches its registers directly. The
    /// consequence is that nothing stops two of these existing at once,
    /// and a caller must not make two.
    pub fn init(gpio: &GPIO, mailbox: &mut Mailbox, timer: &Timer) -> Result<Self, Error> {
        // The card's power domain comes up off, the same as it does for
        // the Arasan controller: register access works regardless,
        // because the register bus is on an always-on rail, so skipping
        // this produces a controller that answers and a bus that never
        // moves.
        if !matches!(
            mailbox.set_power_state(PowerDeviceId::SdCard, true),
            Ok(true)
        ) {
            return Err(Error::PowerOnFailed);
        }

        // This controller is clocked from the core clock, not from the
        // EMMC clock the Arasan part uses -- and that clock moves with
        // config.txt and with the firmware's own scaling, so it is asked
        // for rather than assumed. Assuming a slower clock than the real
        // one produces a *faster* bus than the card agreed to.
        let base_clock_hz = mailbox
            .clock_rate_hz(ClockId::Core)
            .map_err(|_| Error::PowerOnFailed)?;

        route_gpio_to_sdhost(gpio);
        reset(timer);

        let sdhost = Self {
            high_capacity: false,
            rca: 0,
            four_bit_bus: false,
            base_clock_hz,
        };
        // Configuration before clock, which is the order the reference
        // driver resets in: the slow-card bit decides which divisor the
        // controller obeys, so choosing it after a divisor has been
        // programmed would leave one command's worth of bus running at
        // the other one.
        write_reg(
            SDHCFG,
            SDHCFG_BUSY_IRPT_EN | SDHCFG_WIDE_INT_BUS | SDHCFG_SLOW_CARD,
        );
        sdhost.set_clock(SETUP_CLOCK_HZ);

        sdhost.command(CMD_GO_IDLE, 0, timer)?;

        // `CMD0` expects no response, so `CMD8` is the first command
        // that can tell an empty slot from a populated one.
        if sdhost
            .command(CMD_SEND_IF_COND, 0x0000_01aa, timer)
            .is_err()
        {
            return Err(Error::NoCard);
        }

        // Poll until the card reports its power-up complete. The SD
        // specification allows it up to a second in the worst case.
        let start = timer.now_micros();
        let response = loop {
            let response = sdhost.app_command(CMD_SEND_OP_COND, ACMD41_ARG_HC, timer)?;
            if response & ACMD41_COMPLETE != 0 {
                break response;
            }
            if timer.now_micros() - start > 1_000_000 {
                return Err(Error::NoCard);
            }
            timer.delay_ms(10);
        };
        if response & ACMD41_VOLTAGE_WINDOW == 0 {
            return Err(Error::UnusableCard);
        }
        let high_capacity = response & ACMD41_HIGH_CAPACITY != 0;

        sdhost.command(CMD_ALL_SEND_CID, 0, timer)?;
        let rca = sdhost.command(CMD_SEND_REL_ADDR, 0, timer)? & 0xffff_0000;

        let sdhost = Self {
            high_capacity,
            rca,
            ..sdhost
        };
        sdhost.command(CMD_CARD_SELECT, rca, timer)?;
        sdhost.set_clock(TRANSFER_CLOCK_HZ);

        // Optional, and not something correctness rests on: every modern
        // card has the wide bus and it is a pure throughput gain, so a
        // card that will not negotiate it stays at one bit rather than
        // failing bring-up.
        let four_bit_bus = sdhost.negotiate_four_bit_bus(timer).unwrap_or(false);

        Ok(Self {
            four_bit_bus,
            ..sdhost
        })
    }

    /// Whether the card addresses blocks rather than bytes (SDHC/SDXC),
    /// which is what decides the form of a read or write argument.
    pub fn high_capacity(&self) -> bool {
        self.high_capacity
    }

    /// Whether [`Self::init`] negotiated the four-bit bus. A throughput
    /// fact, not something a caller branches on.
    pub fn four_bit_bus(&self) -> bool {
        self.four_bit_bus
    }

    /// Reads one 512-byte block.
    pub fn read_block(
        &self,
        block_index: u32,
        buf: &mut Block,
        timer: &Timer,
    ) -> Result<(), Error> {
        self.read_blocks(block_index, core::slice::from_mut(buf), timer)
    }

    /// Reads a run of consecutive blocks in one command.
    ///
    /// One `CMD18` with an automatic stop rather than a command per
    /// block: the card charges per command, so a run of any length costs
    /// about what a single block does plus the data.
    pub fn read_blocks(
        &self,
        block_index: u32,
        blocks: &mut [Block],
        timer: &Timer,
    ) -> Result<(), Error> {
        if blocks.is_empty() {
            return Ok(());
        }
        let count = u32::try_from(blocks.len()).map_err(|_| Error::TooManyBlocks)?;
        if count > u32::from(u16::MAX) {
            return Err(Error::TooManyBlocks);
        }

        self.start_transfer(count, timer)?;
        let command = if count == 1 {
            CMD_READ_SINGLE
        } else {
            CMD_READ_MULTI
        };
        self.command_with_data(command, self.address(block_index), true, timer)?;

        for block in blocks.iter_mut() {
            read_fifo(block, timer)?;
        }
        self.end_transfer(count, true, timer)
    }

    /// Writes one 512-byte block.
    pub fn write_block(&self, block_index: u32, buf: &Block, timer: &Timer) -> Result<(), Error> {
        self.write_blocks(block_index, core::slice::from_ref(buf), timer)
    }

    /// Writes a run of consecutive blocks in one command.
    pub fn write_blocks(
        &self,
        block_index: u32,
        blocks: &[Block],
        timer: &Timer,
    ) -> Result<(), Error> {
        if blocks.is_empty() {
            return Ok(());
        }
        let count = u32::try_from(blocks.len()).map_err(|_| Error::TooManyBlocks)?;
        if count > u32::from(u16::MAX) {
            return Err(Error::TooManyBlocks);
        }

        self.start_transfer(count, timer)?;
        let command = if count == 1 {
            CMD_WRITE_SINGLE
        } else {
            CMD_WRITE_MULTI
        };
        self.command_with_data(command, self.address(block_index), false, timer)?;

        for block in blocks {
            write_fifo(block, timer)?;
        }
        self.end_transfer(count, false, timer)
    }

    /// Ends a data transfer whose bytes have all moved, which is a
    /// different job in each direction.
    ///
    /// **Reading: stop first.** A multi-block read is open-ended — the
    /// card streams until `CMD12` — so the data state machine cannot
    /// leave `READDATA` on its own and the FIFO sits full behind it.
    /// Waiting for the machine to go idle *before* sending the stop is a
    /// deadlock, and it presents as a transfer that moved every byte
    /// correctly and then timed out, with `SDEDM` reporting a full FIFO
    /// and no error bit anywhere. A single-block read has no stop, so it
    /// is the one case that needs forcing back to data mode.
    ///
    /// **Writing: drain first.** The opposite, and for the opposite
    /// reason: the last words are still in the FIFO and the card has not
    /// seen them. Sending the stop there truncates the transfer — the
    /// card is told the data ended before it did, and the tail of what
    /// was written is whatever was in those blocks before. That is a
    /// write which reports success and fails its read-back, which is
    /// exactly how it was found.
    ///
    /// Then the card has to finish programming. Nothing after a write
    /// may touch the bus until it has, and the card is the only thing
    /// that knows — see [`Self::wait_ready`].
    fn end_transfer(&self, count: u32, read: bool, timer: &Timer) -> Result<(), Error> {
        if read {
            if count > 1 {
                self.command(CMD_STOP_TRANSMISSION, 0, timer)?;
            } else {
                force_data_mode(true, timer)?;
            }
            return check_transfer();
        }

        force_data_mode(false, timer)?;
        if count > 1 {
            self.command(CMD_STOP_TRANSMISSION, 0, timer)?;
        }
        check_transfer()?;
        self.wait_ready(timer)
    }

    /// Waits for the card to finish programming what was just written.
    ///
    /// A card pulls `DAT0` low while it commits a write and answers
    /// nothing else until it is done. The controller's busy-wait covers
    /// the stop command that follows a multi-block write, but not a
    /// single-block one — which has no stop — and not the rest of the
    /// programming time either way.
    ///
    /// So this asks the card directly, which is what the specification
    /// says to do: `CMD13` until it reports itself ready for data and
    /// back in the transfer state. Skipping it means the next command
    /// lands on a card that is still writing, and what that reads back
    /// is not what was written.
    fn wait_ready(&self, timer: &Timer) -> Result<(), Error> {
        let start = timer.now_micros();
        loop {
            let status = self.command(CMD_SEND_STATUS, self.rca, timer)?;
            // Ready for data, and in the transfer state rather than
            // still programming.
            if status & CARD_STATUS_READY_FOR_DATA != 0
                && (status >> CARD_STATUS_STATE_SHIFT) & CARD_STATUS_STATE_MASK
                    == CARD_STATE_TRANSFER
            {
                return Ok(());
            }
            if timer.now_micros() - start > PROGRAMMING_TIMEOUT_US {
                return Err(Error::TransferFailed {
                    status: read_reg(SDHSTS),
                    edm: read_reg(SDEDM),
                });
            }
        }
    }

    /// The argument a read or write command takes for `block_index`:
    /// a block number on a high-capacity card, a byte offset on an old
    /// one.
    fn address(&self, block_index: u32) -> u32 {
        if self.high_capacity {
            block_index
        } else {
            block_index * 512
        }
    }

    /// Programs the block size and count for the data command about to
    /// be issued, once the previous transfer has let go of the bus.
    fn start_transfer(&self, blocks: u32, timer: &Timer) -> Result<(), Error> {
        wait_command_idle(timer)?;
        write_reg(SDHBCT, 512);
        write_reg(SDHBLC, blocks);
        Ok(())
    }

    /// Sends a command with no data phase, returning the card's short
    /// response.
    fn command(&self, command: Command, arg: u32, timer: &Timer) -> Result<u32, Error> {
        self.issue(command, arg, None, timer)
    }

    /// Sends a data-bearing command, leaving the FIFO to the caller.
    fn command_with_data(
        &self,
        command: Command,
        arg: u32,
        read: bool,
        timer: &Timer,
    ) -> Result<u32, Error> {
        self.issue(command, arg, Some(read), timer)
    }

    /// Sends an application command: `CMD55` addressed to this card,
    /// then the command itself.
    fn app_command(&self, command: Command, arg: u32, timer: &Timer) -> Result<u32, Error> {
        self.command(CMD_APP_CMD, self.rca, timer)?;
        self.command(command, arg, timer)
    }

    /// The whole of issuing a command: clear the status, write the
    /// argument, write the command, wait for it to complete, and collect
    /// the response.
    fn issue(
        &self,
        command: Command,
        arg: u32,
        data: Option<bool>,
        timer: &Timer,
    ) -> Result<u32, Error> {
        wait_command_idle(timer)?;
        // Write-1-to-clear, so this wipes whatever the last command
        // left behind -- without it, a stale error bit is read back as
        // this command's.
        write_reg(SDHSTS, SDHSTS_CLEAR_MASK);

        let mut sdcmd = u32::from(command.index);
        match command.response {
            Response::None => sdcmd |= SDCMD_NO_RESPONSE,
            Response::Short => {}
            Response::ShortBusy => sdcmd |= SDCMD_BUSYWAIT,
            Response::Long => sdcmd |= SDCMD_LONG_RESPONSE,
        }
        match data {
            Some(true) => sdcmd |= SDCMD_READ_CMD,
            Some(false) => sdcmd |= SDCMD_WRITE_CMD,
            None => {}
        }

        write_reg(SDARG, arg);
        write_reg(SDCMD, sdcmd | SDCMD_NEW_FLAG);

        // The controller clears the start bit when it is done with the
        // command -- including, for a busy-wait command, when the card
        // has released `DAT0`.
        let start = timer.now_micros();
        let sdcmd = loop {
            let sdcmd = read_reg(SDCMD);
            if sdcmd & SDCMD_NEW_FLAG == 0 {
                break sdcmd;
            }
            if timer.now_micros() - start > COMMAND_TIMEOUT_US {
                return Err(Error::ControllerBusy);
            }
        };

        let status = read_reg(SDHSTS);
        if sdcmd & SDCMD_FAIL_FLAG != 0 || status & SDHSTS_ERROR_MASK != 0 {
            return Err(Error::CommandFailed {
                command: command.index,
                status,
            });
        }

        // An `R1b` command is not finished when the start bit clears.
        // The card holds `DAT0` low afterwards and the controller
        // reports the release separately, in `SDHSTS`, which is the only
        // thing that says the card is done. Returning at the start bit
        // hands back a card that is still working, and the next command
        // is then issued into a bus that is busy.
        if matches!(command.response, Response::ShortBusy) {
            self.wait_not_busy(command.index, timer)?;
        }

        Ok(match command.response {
            Response::None => 0,
            // Only the first word. A long response's other three are in
            // SDRSP1-3, and nothing in this driver's sequence reads the
            // CID or CSD -- `CMD2` is issued for its side effect of
            // moving the card into identification state, not for what it
            // says.
            _ => read_reg(SDRSP0),
        })
    }

    /// Waits for the controller to report that the card has released the
    /// busy signal after an `R1b` command.
    fn wait_not_busy(&self, command: u8, timer: &Timer) -> Result<(), Error> {
        let start = timer.now_micros();
        loop {
            let status = read_reg(SDHSTS);
            if status & SDHSTS_ERROR_MASK != 0 {
                return Err(Error::CommandFailed { command, status });
            }
            if status & SDHSTS_BUSY_IRPT != 0 {
                // Write-1-to-clear, so the next command does not read
                // this one's completion as its own.
                write_reg(SDHSTS, SDHSTS_BUSY_IRPT);
                return Ok(());
            }
            if timer.now_micros() - start > PROGRAMMING_TIMEOUT_US {
                return Err(Error::CommandFailed { command, status });
            }
        }
    }

    /// Asks the card whether it supports the four-bit bus and, if it
    /// does, switches both ends to it.
    ///
    /// The answer is in the SCR register, which arrives as an 8-byte
    /// data transfer — the only read here that is not a block.
    fn negotiate_four_bit_bus(&self, timer: &Timer) -> Result<bool, Error> {
        wait_command_idle(timer)?;
        write_reg(SDHBCT, 8);
        write_reg(SDHBLC, 1);

        self.command(CMD_APP_CMD, self.rca, timer)?;
        self.command_with_data(CMD_SEND_SCR, 0, true, timer)?;

        let mut scr = [0u8; 8];
        read_fifo(&mut scr, timer)?;
        // One block as far as the controller is concerned -- eight bytes
        // of it -- so it ends the way every other single-block read does.
        force_data_mode(true, timer)?;
        check_transfer()?;

        // Bit 50 of the SCR, which is bit 18 of its first big-endian
        // word: "4 bit bus width supported".
        if u32::from_be_bytes([scr[0], scr[1], scr[2], scr[3]]) & (1 << 18) == 0 {
            return Ok(false);
        }

        self.command(CMD_APP_CMD, self.rca, timer)?;
        self.command(CMD_SET_BUS_WIDTH, 2, timer)?;
        write_reg(SDHCFG, read_reg(SDHCFG) | SDHCFG_WIDE_EXT_BUS);
        Ok(true)
    }

    /// Sets the SD clock to the fastest the divisor can manage at or
    /// below `target_hz`, and the data timeout to half a second's worth
    /// of it.
    ///
    /// The divisor register holds `divider - 2`, so the slowest clock
    /// available is the core clock over 2049 — about 122 kHz from
    /// 250 MHz, comfortably inside the identification window.
    fn set_clock(&self, target_hz: u32) {
        let mut divider = self.base_clock_hz / target_hz.max(1);
        if divider < 2 {
            divider = 2;
        }
        // Round the divisor up rather than down: dividing up produces a
        // clock *above* the target, and the target is a ceiling the card
        // agreed to rather than a preference.
        if self.base_clock_hz / divider > target_hz {
            divider += 1;
        }
        let cdiv = (divider - 2).min(SDCDIV_MAX);
        write_reg(SDCDIV, cdiv);

        // The data timeout counts in SD clocks, so it is written from
        // whatever clock was just selected -- half a second of it.
        let actual_hz = self.base_clock_hz / (cdiv + 2);
        write_reg(SDTOUT, actual_hz / 2);
    }
}

/// Which response a command produces, which is all the controller needs
/// to be told about one.
#[derive(Clone, Copy)]
enum Response {
    /// No response at all (`CMD0`).
    None,
    /// A 48-bit response.
    Short,
    /// A 48-bit response, after which the card holds `DAT0` low until it
    /// is ready (`R1b`, e.g. `CMD12`).
    ShortBusy,
    /// A 136-bit response (`CMD2`).
    Long,
}

/// One SD command: its index, and what it answers with.
#[derive(Clone, Copy)]
struct Command {
    /// The command index, 0-63.
    index: u8,
    /// What the card answers with.
    response: Response,
}

/// `CMD0`, go idle.
const CMD_GO_IDLE: Command = Command {
    index: 0,
    response: Response::None,
};
/// `CMD2`, send the card identification.
const CMD_ALL_SEND_CID: Command = Command {
    index: 2,
    response: Response::Long,
};
/// `CMD3`, publish a relative address.
const CMD_SEND_REL_ADDR: Command = Command {
    index: 3,
    response: Response::Short,
};
/// `ACMD6`, set the bus width.
const CMD_SET_BUS_WIDTH: Command = Command {
    index: 6,
    response: Response::Short,
};
/// `CMD7`, select this card.
const CMD_CARD_SELECT: Command = Command {
    index: 7,
    response: Response::ShortBusy,
};
/// `CMD8`, send the interface condition — and, in practice, ask whether
/// there is a card there at all.
const CMD_SEND_IF_COND: Command = Command {
    index: 8,
    response: Response::Short,
};
/// `CMD12`, stop a multi-block transfer.
const CMD_STOP_TRANSMISSION: Command = Command {
    index: 12,
    response: Response::ShortBusy,
};
/// `CMD13`, ask the card for its status — which is how a host finds out
/// that a write has finished committing.
const CMD_SEND_STATUS: Command = Command {
    index: 13,
    response: Response::Short,
};
/// `CMD17`, read one block.
const CMD_READ_SINGLE: Command = Command {
    index: 17,
    response: Response::Short,
};
/// `CMD18`, read a run of blocks.
const CMD_READ_MULTI: Command = Command {
    index: 18,
    response: Response::Short,
};
/// `CMD24`, write one block.
const CMD_WRITE_SINGLE: Command = Command {
    index: 24,
    response: Response::Short,
};
/// `CMD25`, write a run of blocks.
const CMD_WRITE_MULTI: Command = Command {
    index: 25,
    response: Response::Short,
};
/// `ACMD41`, send the operating condition.
const CMD_SEND_OP_COND: Command = Command {
    index: 41,
    response: Response::Short,
};
/// `ACMD51`, send the SD configuration register.
const CMD_SEND_SCR: Command = Command {
    index: 51,
    response: Response::Short,
};
/// `CMD55`, the prefix that turns the next command into an application
/// command.
const CMD_APP_CMD: Command = Command {
    index: 55,
    response: Response::Short,
};

/// `ACMD41` argument: the 2.7-3.6 V window, with the host declaring it
/// understands high-capacity cards.
const ACMD41_ARG_HC: u32 = 0x51ff_8000;
/// `ACMD41` response: the card has finished powering up.
const ACMD41_COMPLETE: u32 = 0x8000_0000;
/// `ACMD41` response: the voltage window the card accepted. Zero means
/// it accepted none of what was offered.
const ACMD41_VOLTAGE_WINDOW: u32 = 0x00ff_8000;
/// `ACMD41` response: the card addresses blocks rather than bytes.
const ACMD41_HIGH_CAPACITY: u32 = 0x4000_0000;

/// Routes GPIO48-53 to SDHOST (ALT0).
///
/// The counterpart of [`crate::sd`]'s routing, which points the same six
/// pins at the Arasan controller instead. Whichever ran last owns the
/// slot, so a program must not do both.
fn route_gpio_to_sdhost(gpio: &GPIO) {
    gpio.gpfsel4().modify(|_, w| {
        w.fsel48()
            .bits(GPIO_ALT_SDHOST)
            .fsel49()
            .bits(GPIO_ALT_SDHOST)
    });
    gpio.gpfsel5().modify(|_, w| {
        w.fsel50()
            .bits(GPIO_ALT_SDHOST)
            .fsel51()
            .bits(GPIO_ALT_SDHOST)
            .fsel52()
            .bits(GPIO_ALT_SDHOST)
            .fsel53()
            .bits(GPIO_ALT_SDHOST)
    });

    // The SD bus idles high and `CMD` in particular needs a pull-up --
    // the same requirement `sd.rs` meets for the same six pins, and for
    // the same reason: the card drives them open-drain until the bus is
    // fully claimed.
    crate::gpio::set_pull_bank(gpio, 1, 0x003f_0000, crate::gpio::Pull::Up);
}

/// Resets the controller and powers the bus.
///
/// The order is the reference driver's and is not arbitrary: power off,
/// clear every register the controller latches, program the FIFO
/// thresholds, *then* power on. Programming thresholds with the bus live
/// leaves the FIFO in a state the state machine does not expect.
fn reset(timer: &Timer) {
    write_reg(SDVDD, 0);
    write_reg(SDCMD, 0);
    write_reg(SDARG, 0);
    // The reset default, in clocks. Replaced with a real figure by
    // `set_clock` as soon as a clock is chosen.
    write_reg(SDTOUT, 0x00f0_0000);
    write_reg(SDCDIV, 0);
    write_reg(SDHSTS, SDHSTS_CLEAR_MASK);
    write_reg(SDHCFG, 0);
    write_reg(SDHBCT, 0);
    write_reg(SDHBLC, 0);

    let edm = read_reg(SDEDM);
    let cleared = edm
        & !((SDEDM_THRESHOLD_MASK << SDEDM_READ_THRESHOLD_SHIFT)
            | (SDEDM_THRESHOLD_MASK << SDEDM_WRITE_THRESHOLD_SHIFT));
    write_reg(
        SDEDM,
        cleared
            | (FIFO_THRESHOLD << SDEDM_READ_THRESHOLD_SHIFT)
            | (FIFO_THRESHOLD << SDEDM_WRITE_THRESHOLD_SHIFT),
    );

    timer.delay_ms(10);
    write_reg(SDVDD, 1);
    timer.delay_ms(10);
}

/// Waits for the controller to finish whatever it is doing.
fn wait_command_idle(timer: &Timer) -> Result<(), Error> {
    let start = timer.now_micros();
    while read_reg(SDCMD) & SDCMD_NEW_FLAG != 0 {
        if timer.now_micros() - start > COMMAND_TIMEOUT_US {
            return Err(Error::ControllerBusy);
        }
    }
    Ok(())
}

/// Pulls `buf` out of the FIFO, a burst at a time.
///
/// `buf`'s length must be a multiple of four, which every caller's is: a
/// block is 512 bytes and the SCR is 8.
fn read_fifo(buf: &mut [u8], timer: &Timer) -> Result<(), Error> {
    let start = timer.now_micros();
    let mut at = 0;

    while at < buf.len() {
        let remaining = ((buf.len() - at) / 4) as u32;
        let burst = remaining.min(FIFO_BURST_WORDS);
        let edm = read_reg(SDEDM);
        let available = (edm >> SDEDM_FIFO_LEVEL_SHIFT) & SDEDM_FIFO_LEVEL_MASK;

        if available < burst {
            // Not enough words yet, which is either the controller
            // still filling the FIFO or a transfer that has stopped. The
            // status says whether the card gave a reason; the state
            // machine, carried in the error, says what it was doing when
            // the budget ran out.
            let status = read_reg(SDHSTS);
            if status & SDHSTS_ERROR_MASK != 0 || timer.now_micros() - start > FIFO_TIMEOUT_US {
                return Err(Error::TransferFailed { status, edm });
            }
            continue;
        }

        let words = available.min(remaining);
        for _ in 0..words {
            let word = read_reg(SDDATA);
            buf[at..at + 4].copy_from_slice(&word.to_le_bytes());
            at += 4;
        }
    }
    Ok(())
}

/// Pushes `buf` into the FIFO, a burst at a time.
fn write_fifo(buf: &[u8], timer: &Timer) -> Result<(), Error> {
    let start = timer.now_micros();
    let mut at = 0;

    while at < buf.len() {
        let remaining = ((buf.len() - at) / 4) as u32;
        let burst = remaining.min(FIFO_BURST_WORDS);
        let edm = read_reg(SDEDM);
        // The level field counts what is *in* the FIFO, so room is
        // whatever is left of it.
        let room = FIFO_WORDS - ((edm >> SDEDM_FIFO_LEVEL_SHIFT) & SDEDM_FIFO_LEVEL_MASK);

        if room < burst {
            let status = read_reg(SDHSTS);
            if status & SDHSTS_ERROR_MASK != 0 || timer.now_micros() - start > FIFO_TIMEOUT_US {
                return Err(Error::TransferFailed { status, edm });
            }
            continue;
        }

        let words = room.min(remaining);
        for _ in 0..words {
            let word = u32::from_le_bytes([buf[at], buf[at + 1], buf[at + 2], buf[at + 3]]);
            write_reg(SDDATA, word);
            at += 4;
        }
    }
    Ok(())
}

/// Waits for the data state machine to come back to idle once the last
/// word of a *single-block* transfer has moved, forcing it if it has
/// parked.
///
/// A transfer that has moved all its data does not necessarily end
/// there: a read stops in `READWAIT` and a write in `WRITESTART1`, and
/// the controller sits in that state until it is told to go back to data
/// mode. Leaving it there is what makes the *next* command hang, which
/// is a failure that looks nothing like the transfer that caused it.
///
/// Only for transfers with no stop command — see [`Sdhost::end_transfer`]
/// for why calling this on a multi-block transfer deadlocks instead.
fn force_data_mode(read: bool, timer: &Timer) -> Result<(), Error> {
    let parked = if read {
        SDEDM_FSM_READWAIT
    } else {
        SDEDM_FSM_WRITESTART1
    };

    let start = timer.now_micros();
    loop {
        let edm = read_reg(SDEDM);
        let fsm = edm & SDEDM_FSM_MASK;
        if fsm == SDEDM_FSM_IDENTMODE || fsm == SDEDM_FSM_DATAMODE {
            return Ok(());
        }
        if fsm == parked {
            write_reg(SDEDM, edm | SDEDM_FORCE_DATA_MODE);
            return Ok(());
        }
        if timer.now_micros() - start > TRANSFER_END_TIMEOUT_US {
            return Err(Error::TransferFailed {
                status: read_reg(SDHSTS),
                edm,
            });
        }
    }
}

/// Reports anything the card flagged during a transfer that has
/// otherwise finished.
///
/// Read after the transfer is over rather than during it, because the
/// bits are latched: a CRC failure on the first block is still set when
/// the last one has moved.
fn check_transfer() -> Result<(), Error> {
    let status = read_reg(SDHSTS);
    if status & SDHSTS_ERROR_MASK != 0 {
        return Err(Error::TransferFailed {
            status,
            edm: read_reg(SDEDM),
        });
    }
    Ok(())
}

/// A [`Sdhost`] as a `resident-fat` block device, paired with the
/// [`Timer`] its transfers are paced against.
///
/// The counterpart of [`crate::sd::SdBlockDevice`], and the reason to
/// have one at all: a filesystem hands this whole runs of blocks, and a
/// run reaches the card as one command rather than as a command per
/// block.
///
/// Available only with the `resident-fat` feature enabled.
#[cfg(feature = "resident-fat")]
pub struct SdhostBlockDevice<'t> {
    /// The card.
    sdhost: Sdhost,
    /// What its transfers time themselves against.
    timer: &'t Timer,
}

#[cfg(feature = "resident-fat")]
impl<'t> SdhostBlockDevice<'t> {
    /// Wraps an initialized [`Sdhost`] and the [`Timer`] its transfers
    /// need.
    pub fn new(sdhost: Sdhost, timer: &'t Timer) -> Self {
        Self { sdhost, timer }
    }

    /// The wrapped card, borrowed.
    ///
    /// `resident-fat` owns the device once a volume is mounted and lends
    /// it back through its own accessors, so this is how to reach the
    /// driver's own methods without unmounting.
    pub fn inner(&self) -> &Sdhost {
        &self.sdhost
    }

    /// Unwraps back to the card, dropping the timer borrow.
    pub fn into_inner(self) -> Sdhost {
        self.sdhost
    }
}

/// Error type for [`SdhostBlockDevice`]'s
/// [`BlockDevice`](resident_fat::BlockDevice) implementation.
///
/// Available only with the `resident-fat` feature enabled.
#[cfg(feature = "resident-fat")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SdhostBlockDeviceError {
    /// A block read or write failed at the driver level; carries the
    /// underlying [`Error`].
    Sdhost(Error),
    /// The transfer started past block 2^32, which the controller's
    /// 32-bit block address cannot reach.
    ///
    /// Refused rather than truncated: letting the high bits fall off
    /// turns an unreachable address into a reachable one and writes to
    /// the wrong place on the card.
    BlockOutOfRange {
        /// The first block of the refused transfer.
        start_block: u64,
    },
}

#[cfg(feature = "resident-fat")]
impl From<Error> for SdhostBlockDeviceError {
    /// Wraps a driver [`Error`] as [`SdhostBlockDeviceError::Sdhost`].
    fn from(e: Error) -> Self {
        SdhostBlockDeviceError::Sdhost(e)
    }
}

/// Length of one block, as `resident-fat` counts it — the same 512 this
/// controller moves, asserted below rather than assumed.
#[cfg(feature = "resident-fat")]
const BLOCK_LEN: usize = resident_fat::BLOCK_SIZE;

#[cfg(feature = "resident-fat")]
const _: () = assert!(BLOCK_LEN == core::mem::size_of::<Block>());

#[cfg(feature = "resident-fat")]
impl resident_fat::BlockDevice for SdhostBlockDevice<'_> {
    type Error = SdhostBlockDeviceError;

    /// Reads a run of consecutive blocks in a single (multi-block, when
    /// longer than one) transfer.
    ///
    /// # Panics
    ///
    /// If `blocks.len()` is not a multiple of 512, which the trait
    /// forbids. Asserted rather than rounded down: ignoring an odd tail
    /// would fill part of the caller's buffer, return `Ok`, and leave the
    /// rest holding whatever it held before.
    fn read(&mut self, start_block: u64, blocks: &mut [u8]) -> Result<(), Self::Error> {
        let index = u32::try_from(start_block)
            .map_err(|_| SdhostBlockDeviceError::BlockOutOfRange { start_block })?;
        // Zero-copy, and safely so: `Block` is a type alias for
        // `[u8; 512]` rather than a newtype, so this is a plain reborrow
        // of the caller's buffer with no layout assumption behind it.
        let (blocks, rest) = blocks.as_chunks_mut::<BLOCK_LEN>();
        assert!(rest.is_empty(), "transfer length must be a multiple of 512");
        self.sdhost.read_blocks(index, blocks, self.timer)?;
        Ok(())
    }

    /// Writes a run of consecutive blocks in a single transfer — the
    /// mirror of [`read`](resident_fat::BlockDevice::read), with the same
    /// length rule and the same reason for it.
    ///
    /// # Panics
    ///
    /// If `blocks.len()` is not a multiple of 512.
    fn write(&mut self, start_block: u64, blocks: &[u8]) -> Result<(), Self::Error> {
        let index = u32::try_from(start_block)
            .map_err(|_| SdhostBlockDeviceError::BlockOutOfRange { start_block })?;
        let (blocks, rest) = blocks.as_chunks::<BLOCK_LEN>();
        assert!(rest.is_empty(), "transfer length must be a multiple of 512");
        self.sdhost.write_blocks(index, blocks, self.timer)?;
        Ok(())
    }

    /// Always `Ok(None)` — this driver does not read the card's capacity
    /// (its CSD), so it does not know it.
    fn block_count(&mut self) -> Result<Option<u64>, Self::Error> {
        Ok(None)
    }

    /// 65535 — the largest run `SDHBLC`'s 16-bit block count can
    /// express, and so the longest transfer one command can carry.
    fn max_transfer_blocks(&self) -> u64 {
        u64::from(u16::MAX)
    }
}

/// Reads one of this controller's registers.
fn read_reg(reg: *mut u32) -> u32 {
    // SAFETY: `reg` is one of the constants above, each a fixed
    // peripheral address in the identity-mapped device region.
    unsafe { reg.read_volatile() }
}

/// Writes one of this controller's registers.
fn write_reg(reg: *mut u32, value: u32) {
    // SAFETY: as `read_reg`.
    unsafe { reg.write_volatile(value) }
}
