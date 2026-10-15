//! Interrupt-driven, `async` twins of [`Sd`]'s block transfers, plus the
//! handler ([`on_irq`]) that drives them.
//!
//! The blocking driver spins on `INTERRUPT` for every handshake, which
//! costs the CPU the card's own thinking time as well as the bus time.
//! Most of that is one wait: the `DATA_DONE` ending a write only arrives
//! once the card has programmed an entire internal erase block, which is
//! milliseconds per command on a cheap card. Under an executor that stall
//! takes every other task with it. Here it is an await.
//!
//! # Which waits are awaited, and which are not
//!
//! Not every handshake is worth a wakeup:
//!
//! - **`DATA_DONE`** at the end of a transfer — the card programming a
//!   write, or the auto-`CMD12` closing a multi-block read. This is the
//!   expensive one, and the reason this module exists.
//! - **`READ_RDY`/`WRITE_RDY`** per block — the card shipping or
//!   accepting data. Awaited too, so a long multi-block transfer parks
//!   between blocks rather than at the end only. The *first* `WRITE_RDY`
//!   of a write is already set when the command completes, so awaiting it
//!   costs nothing but a register read.
//! - **`CMD_DONE`** — microseconds. Spun on, exactly as the blocking path
//!   does: parking would cost more in executor overhead than the wait
//!   itself.
//! - **`STATUS.DAT_INHIBIT`**, waited for before each command, has no
//!   interrupt of its own and so cannot be awaited. It doesn't need to
//!   be: it is the previous write's card-busy time, and awaiting that
//!   write's `DATA_DONE` has already absorbed it, so the spin finds the
//!   line free and returns immediately.
//! - **The FIFO word loop** is not a wait at all and stays inline.
//!
//! # Wiring
//!
//! Nothing here resolves until the controller's interrupt reaches
//! [`on_irq`]. That means the same three gates every interrupt source in
//! this crate goes through — the peripheral (`IRPT_EN`, opened by each
//! wait as it parks, so an application configures nothing), the interrupt
//! controller (`crate::lic::Lic::enable_emmc_irq`), and the CPU mask
//! ([`enable_irq`](crate::irq::enable_irq)) — plus a call to [`on_irq`]
//! from the application's `__irq_handler`. A library crate cannot claim
//! that symbol, so dispatch stays the application's.
//!
//! # Two registers, not one
//!
//! The controller gates its interrupt twice, and confusing the two is the
//! easiest mistake to make here. `IRPT_MASK` decides whether a condition
//! becomes visible in `INTERRUPT` at all; [`Sd::init`] opens it wide so
//! the blocking path can poll. `IRPT_EN` decides which of those visible
//! bits assert the line to the ARM core, and it is left at zero until a
//! wait here opens exactly the bits it is about to park on — closing them
//! again the moment it stops waiting. Nothing else is ever enabled, so a
//! condition nobody is servicing cannot leave a level source asserted,
//! which on this controller is a hang rather than a wasted interrupt.
//!
//! # Masking, not acknowledging
//!
//! [`on_irq`] therefore clears `IRPT_EN` and leaves `INTERRUPT` alone.
//! It has to: which bits may be cleared, and when, is subtle enough on
//! this controller that the driver keeps that rule in exactly one place
//! (`Sd::poll_interrupt`, whose doc comment explains what a careless
//! clear costs), and the future reads the status for itself when it is
//! polled.
//!
//! # Cancellation
//!
//! Dropping a transfer future part-way — what `embassy_time::with_timeout`
//! and `select!` do — stops the card and resets the controller's data
//! circuit before the drop returns. That is not tidiness. An abandoned
//! data phase leaves the host FIFO holding part of an aborted block and
//! the card still in a data-transfer state; the *next* transfer would
//! then drain those stale words and return wrong bytes with no error bit
//! set anywhere. Silent bad data is a poor thing to hand a filesystem, so
//! cleanup is code rather than a warning in a doc comment.
//!
//! The abort itself is short: `CMD12` if the transfer was multi-block,
//! then a reset of the command and data circuits, and no wait at all for
//! the card to finish whatever it was programming. That last part is
//! deliberate — that wait is the millisecond-scale one, and the next
//! transfer's existing `DAT_INHIBIT` check absorbs it just as well. The
//! same guard runs when a transfer returns an error, so a failed async
//! transfer also leaves the controller clean.
//!
//! # Timeouts
//!
//! The idiomatic bound is the caller's own timer —
//! `embassy_time::with_timeout(Duration::from_secs(1), sd.write_blocks_async(..))`
//! — which puts the number where the application's judgement is, and
//! needs nothing from this driver but that dropping a transfer be safe.
//! Each wait still carries the blocking path's one-second backstop, but
//! it is only *observed* when the future is polled: a card that stops
//! answering raises no interrupt, so nothing polls the future and the
//! backstop never fires. It catches the case where something else in the
//! program polls the task; it is not a substitute for a real timeout.

use core::cell::RefCell;
use core::future::poll_fn;
use core::task::{Poll, Waker};

use critical_section::Mutex;

use super::{
    checked_block_count, wait_for, Block, Error, Sd, CMD_READ_MULTI, CMD_READ_SINGLE,
    CMD_WRITE_MULTI, CMD_WRITE_SINGLE, DATA_REG_BUS_ADDRESS, DMA_DREQ_EMMC, INT_DATA_DONE,
    INT_ERROR_MASK, INT_READ_RDY, INT_WRITE_RDY,
};
use crate::dma::Channel;
use crate::timer::Timer;

/// `CMDTM` value for CMD12 (`STOP_TRANSMISSION`) — R1b (48-bit + busy)
/// response, no data phase, and `CMD_TYPE = abort` (`0b11` in bits 22:23,
/// `0x00c0_0000`). Only ever sent by [`Abort`]: a multi-block transfer
/// that runs to completion is stopped by the controller's own auto-`CMD12`
/// (see `CMD_READ_MULTI`), and one that is cancelled part-way never
/// reaches it, leaving the card to be stopped by hand.
///
/// The abort command type is the part that matters and the part that
/// differs from the `0x0c03_0000` a driver sending `CMD12` the ordinary
/// way (bztsrc's `sd.c`, after a data phase has already finished) uses:
/// the SD host controller specification allows a command to be issued
/// while the data line is still busy only if it is marked as an abort,
/// which — a transfer still in flight being exactly the situation here —
/// is what this is for.
const CMD_STOP_TRANSMISSION: u32 = 0x0cc3_0000;

/// `CMDTM.TM_MULTI_BLOCK`, the bit distinguishing `CMD18`/`CMD25` from
/// their single-block counterparts — and so whether an aborted transfer
/// has a card to stop with [`CMD_STOP_TRANSMISSION`]. A single-block
/// command is not open-ended and needs no stop command at all.
const TM_MULTI_BLOCK: u32 = 0x0000_0020;

/// Budget for [`Abort`]'s circuit reset to self-clear, in microseconds.
/// Far shorter than anything the transfer path allows: a controller that
/// hasn't finished resetting two of its own circuits in 10ms is wedged
/// past what an abort can fix, and a `Drop` is the wrong place to keep
/// hoping. (The stop command sent before it carries `Sd::command`'s own
/// budgets instead, being an ordinary command.)
const ABORT_BUDGET_US: u64 = 10_000;

/// Budget for the DMA engine to finish moving what the card has already
/// transferred, in microseconds. This is bus time with the card no longer
/// involved — microseconds in practice — so the budget is a guard against
/// a wedged engine rather than a real allowance.
const DMA_DRAIN_BUDGET_US: u64 = 10_000;

/// Backstop for each await, in microseconds — the same budget
/// `Sd::wait_interrupt` spins for. See this module's "Timeouts" section
/// for why it is a backstop rather than the mechanism.
const WAIT_BACKSTOP_US: u64 = 1_000_000;

/// Handshake between a parked transfer and [`on_irq`].
///
/// One slot, because there is one controller: the async methods take
/// `&mut Sd`, so a second transfer cannot be in flight to overwrite a
/// waker this one is relying on.
static WAKER: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));

/// Services the EMMC controller's interrupt: closes the gate that raised
/// it and wakes the transfer parked behind it.
///
/// Call this from the application's `__irq_handler` when
/// `crate::lic::Lic::is_emmc_pending` reports the source. Harmless to
/// call spuriously, and harmless to a card being driven by the blocking
/// API in the same program: that path enables no interrupt, so the check
/// below finds `IRPT_EN` clear and returns having touched nothing —
/// leaving `INTERRUPT` latched for its polling loop to read. The same
/// check is what keeps this off the [`crate::sdio`] driver's back, which
/// drives this same controller for the on-board WiFi chip and shares its
/// interrupt line.
///
/// What it deliberately does *not* do is acknowledge anything in
/// `INTERRUPT`. Clearing `IRPT_EN` is what ends the interrupt; the status
/// bits are the woken future's to read and clear, for the reason
/// `Sd::poll_interrupt` documents.
pub fn on_irq() {
    // Safe to steal: this touches only `IRPT_EN`, and only when a parked
    // transfer has opened it -- and that transfer holds the `&mut Sd`
    // borrow for as long as it is parked.
    let emmc = unsafe { Sd::steal_emmc() };
    if emmc.irpt_en().read().bits() == 0 {
        return;
    }
    emmc.irpt_en().write(|w| unsafe { w.bits(0) });

    critical_section::with(|cs| {
        if let Some(waker) = WAKER.borrow_ref_mut(cs).take() {
            waker.wake();
        }
    });
}

/// Cleans up after a transfer that ends anywhere other than its last
/// line — dropped part-way by a `with_timeout`, or returning early with
/// an error.
///
/// Held for the whole data phase and disarmed only on success, so the
/// invariant is simply that an [`Sd`] is clean between transfers: no
/// interrupt enabled, no waker stranded in the slot, no card left
/// streaming into a FIFO nobody will drain. See this module's
/// "Cancellation" section for what the alternative costs.
struct Abort<'a> {
    sd: &'a Sd,
    timer: &'a Timer,
    /// The `CMDTM` code of the transfer in flight — read only for its
    /// [`TM_MULTI_BLOCK`] bit, which says whether the card needs a stop
    /// command.
    command: u32,
    /// Cleared once the transfer has completed and has nothing left to
    /// abort. The rest of the cleanup happens either way.
    armed: bool,
}

impl<'a> Abort<'a> {
    /// Arms cleanup for the transfer `command` just started.
    fn new(sd: &'a Sd, timer: &'a Timer, command: u32) -> Self {
        Self {
            sd,
            timer,
            command,
            armed: true,
        }
    }

    /// Marks the transfer complete: the controller is idle by its own
    /// account, so [`Drop`] should tidy up without aborting anything.
    fn disarm(&mut self) {
        self.armed = false;
    }
}

impl Drop for Abort<'_> {
    fn drop(&mut self) {
        // Unconditional, and the part that cannot be skipped: an enabled
        // interrupt nobody is waiting on re-enters the handler forever,
        // and a waker left in the slot points at a future that no longer
        // exists. A wait that finished normally has already done this;
        // one abandoned part-way has not.
        self.sd.end_wait();

        if self.armed {
            self.sd.abort_transfer(self.command, self.timer);
        }
    }
}

impl Sd {
    /// Reads the 512-byte block at `block_index` into `buf`, parking on
    /// the controller's interrupt rather than spinning. The `async` twin
    /// of [`Sd::read_block`] (`CMD17`), and identical to it in what
    /// reaches the card.
    pub async fn read_block_async(
        &mut self,
        block_index: u32,
        buf: &mut Block,
        timer: &Timer,
    ) -> Result<(), Error> {
        self.read_blocks_pio_async(block_index, 1, core::iter::once(buf), timer)
            .await
    }

    /// Reads `blocks.len()` consecutive 512-byte blocks starting at
    /// `block_index`, parking on the controller's interrupt between
    /// blocks rather than spinning. The `async` twin of
    /// [`Sd::read_blocks`]: the same single `CMD18` (plus auto-`CMD12`)
    /// for a run longer than one block, the same [`Error::TooManyBlocks`]
    /// past the controller's 16-bit block count, and the same no-op for
    /// an empty slice.
    ///
    /// Dropping the returned future mid-transfer stops the card and
    /// resets the controller's data circuit before the drop returns, so
    /// the next transfer starts clean — see the note on cancellation
    /// below.
    ///
    /// # Cancellation
    ///
    /// A cancelled read cannot leave stale words in the FIFO for the next
    /// transfer to mistake for its own data: the drop issues `CMD12` (for
    /// a multi-block run) and resets the command and data circuits. What
    /// it does not do is wait for the card to become ready again — that
    /// wait is absorbed by the next transfer, which checks
    /// `STATUS.DAT_INHIBIT` before issuing anything, exactly as it always
    /// has. The contents of `blocks` after a cancellation are
    /// unspecified: some prefix of it holds real data and the rest is
    /// whatever was there before.
    pub async fn read_blocks_async(
        &mut self,
        block_index: u32,
        blocks: &mut [Block],
        timer: &Timer,
    ) -> Result<(), Error> {
        if blocks.is_empty() {
            return Ok(());
        }
        let count = checked_block_count(blocks.len())?;
        self.read_blocks_pio_async(block_index, count, blocks.iter_mut(), timer)
            .await
    }

    /// Reads `blocks.len()` consecutive 512-byte blocks starting at
    /// `block_index` over the system DMA controller (`channel`), parking
    /// on the controller's interrupt while the engine works. The `async`
    /// twin of [`Sd::read_blocks_dma`], and the cheapest of these paths
    /// per byte: the whole data phase costs one wakeup rather than one
    /// per block, and no CPU cycles at all move the data.
    ///
    /// Same commands, same channel and alignment requirements, and the
    /// same [`Error::Dma`] for a channel length limit or a hardware error
    /// as the blocking version. Cancellation behaves as
    /// [`Sd::read_blocks_async`] describes, with the DMA channel halted
    /// as well.
    pub async fn read_blocks_dma_async(
        &mut self,
        block_index: u32,
        blocks: &mut [Block],
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<(), Error> {
        if blocks.is_empty() {
            return Ok(());
        }
        let count = checked_block_count(blocks.len())?;
        let cmd = if count == 1 {
            CMD_READ_SINGLE
        } else {
            CMD_READ_MULTI
        };
        self.start_transfer(cmd, block_index, count, timer)?;
        let mut abort = Abort::new(self, timer, cmd);

        let transfer = channel
            .start_from_peripheral(
                blocks.as_flattened_mut(),
                DMA_DREQ_EMMC,
                DATA_REG_BUS_ADDRESS,
            )
            .map_err(Error::Dma)?;
        self.wait_interrupt_async(INT_DATA_DONE, cmd, timer).await?;
        // The card and controller are finished; the engine may still have
        // a word or two of the FIFO left to move, which takes bus cycles
        // rather than card time. Spinning that out is the right call for
        // the same reason `CMD_DONE` is spun on.
        wait_for(timer, DMA_DRAIN_BUDGET_US, || transfer.is_complete())?;
        if transfer.is_error() {
            return Err(Error::Dma(crate::dma::Error::Transfer));
        }
        // Dropping the guard is what invalidates `blocks` -- until then
        // this core could still be reading the pre-transfer contents out
        // of its own cache.
        drop(transfer);

        abort.disarm();
        Ok(())
    }

    /// Writes `buf` to the 512-byte block at `block_index`, parking on
    /// the controller's interrupt rather than spinning. The `async` twin
    /// of [`Sd::write_block`] (`CMD24`), and the single-block case of the
    /// wait this module exists for: a successful return still means the
    /// card has committed the data, but the milliseconds it spent
    /// programming went to the executor.
    pub async fn write_block_async(
        &mut self,
        block_index: u32,
        buf: &Block,
        timer: &Timer,
    ) -> Result<(), Error> {
        self.write_blocks_pio_async(block_index, 1, core::iter::once(buf), timer)
            .await
    }

    /// Writes `blocks.len()` consecutive 512-byte blocks starting at
    /// `block_index`, parking on the controller's interrupt rather than
    /// spinning. The `async` twin of [`Sd::write_blocks`]: the same
    /// single `CMD25` (plus auto-`CMD12`) for a run longer than one
    /// block, the same commitment on return, the same
    /// [`Error::TooManyBlocks`], and the same no-op for an empty slice.
    ///
    /// # Cancellation
    ///
    /// As [`Sd::read_blocks_async`], with one thing a caller has to
    /// decide for itself: a cancelled write may have committed some
    /// prefix of `blocks` to the card and left the block it was in the
    /// middle of in an unspecified state. The controller is clean
    /// afterwards, but the card's contents are not what either outcome
    /// would have left, so a cancelled write wants re-issuing rather than
    /// ignoring.
    pub async fn write_blocks_async(
        &mut self,
        block_index: u32,
        blocks: &[Block],
        timer: &Timer,
    ) -> Result<(), Error> {
        if blocks.is_empty() {
            return Ok(());
        }
        let count = checked_block_count(blocks.len())?;
        self.write_blocks_pio_async(block_index, count, blocks.iter(), timer)
            .await
    }

    /// Writes `blocks.len()` consecutive 512-byte blocks starting at
    /// `block_index` over the system DMA controller (`channel`), parking
    /// on the controller's interrupt while the engine works — the write
    /// mirror of [`Sd::read_blocks_dma_async`] and the `async` twin of
    /// [`Sd::write_blocks_dma`]. Waits for transfer-complete before
    /// returning, so success means the data is committed to the card.
    pub async fn write_blocks_dma_async(
        &mut self,
        block_index: u32,
        blocks: &[Block],
        channel: &mut Channel,
        timer: &Timer,
    ) -> Result<(), Error> {
        if blocks.is_empty() {
            return Ok(());
        }
        let count = checked_block_count(blocks.len())?;
        let cmd = if count == 1 {
            CMD_WRITE_SINGLE
        } else {
            CMD_WRITE_MULTI
        };
        self.start_transfer(cmd, block_index, count, timer)?;
        let mut abort = Abort::new(self, timer, cmd);

        let transfer = channel
            .start_to_peripheral(blocks.as_flattened(), DMA_DREQ_EMMC, DATA_REG_BUS_ADDRESS)
            .map_err(Error::Dma)?;
        // `DATA_DONE` is the card's word, not the engine's, and the card
        // cannot have finished programming data the engine had yet to
        // push -- so the drain wait here is a formality where the read's
        // is real.
        self.wait_interrupt_async(INT_DATA_DONE, cmd, timer).await?;
        wait_for(timer, DMA_DRAIN_BUDGET_US, || transfer.is_complete())?;
        if transfer.is_error() {
            return Err(Error::Dma(crate::dma::Error::Transfer));
        }
        drop(transfer);

        abort.disarm();
        Ok(())
    }

    /// PIO core shared by [`Self::read_block_async`]/
    /// [`Self::read_blocks_async`] — the await-per-block twin of
    /// `Sd::read_blocks_pio`, which it otherwise matches command for
    /// command (see there on the FIFO drain and on why a single-block
    /// read skips the closing `DATA_DONE`). `blocks` must yield exactly
    /// `count` blocks.
    async fn read_blocks_pio_async<'a>(
        &self,
        block_index: u32,
        count: u16,
        blocks: impl Iterator<Item = &'a mut Block>,
        timer: &Timer,
    ) -> Result<(), Error> {
        let cmd = if count == 1 {
            CMD_READ_SINGLE
        } else {
            CMD_READ_MULTI
        };
        self.start_transfer(cmd, block_index, count, timer)?;
        let mut abort = Abort::new(self, timer, cmd);

        for block in blocks {
            self.wait_interrupt_async(INT_READ_RDY, cmd, timer).await?;
            for chunk in block.as_chunks_mut::<4>().0 {
                *chunk = self.emmc.data().read().bits().to_le_bytes();
            }
        }
        if count > 1 {
            self.wait_interrupt_async(INT_DATA_DONE, cmd, timer).await?;
        }

        abort.disarm();
        Ok(())
    }

    /// PIO core shared by [`Self::write_block_async`]/
    /// [`Self::write_blocks_async`] — the await-per-block twin of
    /// `Sd::write_blocks_pio`, matching it command for command. `blocks`
    /// must yield exactly `count` blocks.
    async fn write_blocks_pio_async<'a>(
        &self,
        block_index: u32,
        count: u16,
        blocks: impl Iterator<Item = &'a Block>,
        timer: &Timer,
    ) -> Result<(), Error> {
        let cmd = if count == 1 {
            CMD_WRITE_SINGLE
        } else {
            CMD_WRITE_MULTI
        };
        self.start_transfer(cmd, block_index, count, timer)?;
        let mut abort = Abort::new(self, timer, cmd);

        for block in blocks {
            self.wait_interrupt_async(INT_WRITE_RDY, cmd, timer).await?;
            for &chunk in block.as_chunks::<4>().0 {
                self.emmc
                    .data()
                    .write(|w| unsafe { w.bits(u32::from_le_bytes(chunk)) });
            }
        }
        self.wait_interrupt_async(INT_DATA_DONE, cmd, timer).await?;

        abort.disarm();
        Ok(())
    }

    /// Parks until `mask` (or any error bit) appears in `INTERRUPT`, and
    /// consumes it — the await-shaped `Sd::wait_interrupt`, sharing
    /// `Sd::poll_interrupt` with it so that the two agree exactly on what
    /// a completion means and which bits it clears.
    ///
    /// The order inside each poll is what makes the wakeup safe: check
    /// the status, store the waker, open `IRPT_EN`, then check again. An
    /// interrupt that lands anywhere in that sequence either finds the
    /// waker already stored, or is caught by the second check.
    async fn wait_interrupt_async(
        &self,
        mask: u32,
        command: u32,
        timer: &Timer,
    ) -> Result<u32, Error> {
        let start = timer.now_micros();

        poll_fn(|cx| {
            if let Some(result) = self.poll_interrupt(mask, command) {
                self.end_wait();
                return Poll::Ready(result);
            }

            critical_section::with(|cs| {
                WAKER.borrow_ref_mut(cs).replace(cx.waker().clone());
            });
            self.set_irq_enable(mask | INT_ERROR_MASK);

            if let Some(result) = self.poll_interrupt(mask, command) {
                self.end_wait();
                return Poll::Ready(result);
            }

            // Only ever observed on a poll that some *other* wakeup
            // caused -- see this module's "Timeouts" section.
            if timer.now_micros() - start > WAIT_BACKSTOP_US {
                self.end_wait();
                return Poll::Ready(Err(Error::WaitTimeout {
                    waiting_for: mask,
                    interrupt: self.emmc.interrupt().read().bits(),
                    status: self.emmc.status().read().bits(),
                    command,
                }));
            }

            Poll::Pending
        })
        .await
    }

    /// Ends a wait's interest in the controller: the interrupt closed
    /// again, and the waker slot emptied.
    ///
    /// Emptying the slot matters less than closing the gate — the next
    /// wait overwrites it before arming anything, and [`Abort`] clears it
    /// at the end of the transfer either way — but a waker left behind
    /// while the transfer moves on to its FIFO loop is a waker that can be
    /// woken for a wait that has already finished, and there is no reason
    /// to leave that lying around.
    fn end_wait(&self) {
        self.set_irq_enable(0);
        critical_section::with(|cs| {
            WAKER.borrow_ref_mut(cs).take();
        });
    }

    /// Opens exactly `bits` on the controller's interrupt line, closing
    /// everything else — see this module's "Two registers, not one".
    fn set_irq_enable(&self, bits: u32) {
        self.emmc.irpt_en().write(|w| unsafe { w.bits(bits) });
    }

    /// Puts the controller back in a state the next transfer can use
    /// after `command`'s data phase was abandoned part-way.
    ///
    /// Best-effort by nature: every step is bounded and every failure
    /// ignored, because this runs from a [`Drop`] that has nobody to
    /// report to and a next transfer that will fail loudly enough on its
    /// own if the controller really is wedged.
    fn abort_transfer(&self, command: u32, timer: &Timer) {
        // A card still clocking out (or waiting to take) data has to be
        // told to stop -- but only a multi-block transfer is open-ended
        // enough to need it, and only if the data line is actually still
        // busy. Skipping the command when it isn't keeps the common case
        // -- a future dropped a moment after its last await -- down to
        // the reset below.
        if command & TM_MULTI_BLOCK != 0 && self.emmc.status().read().dat_inhibit().bit_is_set() {
            let _ = self.command(CMD_STOP_TRANSMISSION, 0, timer);
        }

        // Reset the command and data circuits: this is what empties the
        // FIFO of the aborted block's leftovers and drops the
        // controller's own idea that a transfer is in flight. Not
        // `SRST_HC`, which would take the clock configuration and the
        // bus width with it and leave the card unusable.
        self.emmc
            .control1()
            .modify(|_, w| w.srst_cmd().set_bit().srst_data().set_bit());
        let _ = wait_for(timer, ABORT_BUDGET_US, || {
            let control1 = self.emmc.control1().read();
            !control1.srst_cmd().bit_is_set() && !control1.srst_data().bit_is_set()
        });

        // Put `IRPT_MASK` back the way `Sd::init` left it. The datasheet
        // is silent on whether a circuit reset takes the interrupt
        // registers with it (the SD host controller specification resets
        // them only on a *full* reset, which this deliberately isn't), and
        // a cleared `IRPT_MASK` is invisible until the next transfer hangs
        // waiting for a status bit that can no longer appear.
        self.emmc
            .irpt_mask()
            .write(|w| unsafe { w.bits(0xffff_ffff) });

        // Now -- and only now, with no transfer in flight to strand --
        // every latched status bit can go. The care `Sd::poll_interrupt`
        // takes not to clear more than it waited for is about a live
        // transfer; here the point is to leave nothing behind that the
        // next one could mistake for its own.
        let latched = self.emmc.interrupt().read().bits();
        self.emmc.interrupt().write(|w| unsafe { w.bits(latched) });
    }
}
