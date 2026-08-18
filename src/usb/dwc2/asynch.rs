//! Interrupt-driven, `async` twins of [`Channel`]'s transfer
//! primitives, plus the handler ([`on_irq`]) that drives them.
//!
//! The blocking methods on [`Channel`] busy-wait in three places: on
//! `HCINT.CHH` for a channel to halt, on `HFNUM` for the microframe a
//! periodic split must be scheduled in, and on the wall clock between
//! complete-split polls. Each of those becomes an await here, resolved
//! by an interrupt rather than by spinning, so an executor gets the CPU
//! back for the (comparatively enormous) intervals a USB transfer spends
//! waiting on the bus.
//!
//! # No clock is needed
//!
//! Nothing here depends on a time crate, which is why it can live in the
//! HAL rather than beside an executor. The two things the blocking path
//! used a clock for are both really *bus* time, and the bus provides it:
//! a transfer completing is a channel-halt interrupt, and every delay
//! that matters to split scheduling is a whole number of 125µs
//! microframes, which is exactly what start-of-frame counts. SOF is the
//! clock.
//!
//! The one thing that follows from that: an async transfer has **no
//! timeout**. The blocking twins give up after 50ms; these wait until
//! the hardware says something. Impose a deadline from outside by
//! dropping the future — `embassy_time::with_timeout` and friends — and
//! see the cancellation note below.
//!
//! # Wiring
//!
//! Nothing here resolves until the controller's interrupt reaches
//! [`on_irq`], which means the same three gates every interrupt source
//! in this crate goes through — the peripheral (`GINTMSK`, set up by
//! [`Dwc2Host::init`] and managed from here), the interrupt controller
//! (`crate::lic::Lic::enable_usb_irq`), and the
//! CPU mask ([`enable_irq`](crate::irq::enable_irq)) — plus a call to
//! [`on_irq`] from the application's `__irq_handler`. A library crate
//! can't claim that symbol, so dispatch stays the application's.
//!
//! # Coexisting with the blocking API
//!
//! [`on_irq`] only touches channels an async transfer is currently
//! waiting on. A channel being driven by the blocking methods is left
//! entirely alone — its `HCINT` stays latched for that channel's own
//! polling loop to read — so both styles can share one controller, one
//! interrupt, and one handler. That is what lets, say, a network driver
//! keep using the blocking calls on its own channel while an async
//! stack runs on others.
//!
//! # Cancellation
//!
//! Dropping a transfer future while it is parked aborts the channel
//! (`CHDIS`, then wait for `CHH`) before returning it to the caller.
//! This is not optional tidiness: re-arming a channel that is still live
//! from an abandoned transfer is a documented failure mode of this core
//! — every subsequent transfer on that channel fails, each a different
//! way, because each new start reprograms a channel that never stopped.
//! Aborting is the reason the async methods still take a [`Timer`]; it
//! is the only thing they use it for.

use core::cell::RefCell;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, Waker};

use critical_section::Mutex;

use super::{
    Channel, ControlEndpoint, DataPid, Dwc2Host, EndpointType, Transaction, TransferError,
    DMA_BUFFER_LEN, HCINT_ACK, HCINT_BBERR, HCINT_CHH, HCINT_NAK, HCINT_NYET, HCINT_STALL,
    HCINT_TXERR, HCINT_XFRC, MAX_CHANNELS, MAX_CSPLIT_POLLS,
};
use crate::cache::{clean_range, invalidate_range};
use crate::pac::{USB_OTG_GLOBAL, USB_OTG_HOST};
use crate::timer::Timer;

/// How many microframes apart the complete-split polls of a *non*-
/// periodic split are spaced — the async spelling of the blocking path's
/// `CSPLIT_RETRY_DELAY_US`, which is the same 5 × 125µs expressed as a
/// wall-clock delay. Waiting for the microframe instead of for elapsed
/// time is both cheaper and better aligned: the transaction translator's
/// progress is itself measured in microframes.
const CSPLIT_RETRY_MICROFRAMES: u8 = 5;

/// `GINTSTS.SOF` — start of frame, write-1-to-clear. As a raw mask
/// because clearing it must touch nothing else in the register; see
/// [`on_irq`].
const GINTSTS_SOF: u32 = 1 << 3;

/// What a channel's owner is currently parked on, if anything.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Wait {
    /// Nothing. [`on_irq`] leaves this channel completely alone, which
    /// is what lets the blocking path keep servicing its own `HCINT`.
    None,
    /// For the channel to halt (`HCINT.CHH`).
    Halt,
    /// For `HFNUM`'s microframe counter to reach this value.
    Frame(u8),
}

/// Per-channel handshake between a parked future and [`on_irq`].
struct Slot {
    /// What the future is waiting for.
    wait: Wait,
    /// Set by [`on_irq`] once [`Self::wait`] is satisfied: the raw
    /// `HCINT` captured at the halt, or `0` for a microframe wait.
    ///
    /// Needed as a separate field rather than inferred from `wait`
    /// returning to [`Wait::None`], because acking `HCINT` destroys the
    /// only evidence of what the halt reported — the future has to be
    /// handed the value the handler saw.
    ready: Option<u32>,
    /// The parked task, once it has polled at least once.
    ///
    /// Optional because a transfer arms its slot *before* starting the
    /// channel — the interrupt can arrive before the first poll, and a
    /// handler that found no waker and gave up would leave the line
    /// asserted with nothing to ack it.
    waker: Option<Waker>,
    /// The last `HCINT` [`on_irq`] captured for this channel, kept
    /// after [`Self::ready`] has been consumed — see
    /// [`last_interrupt`].
    last_hcint: u32,
    /// Every `HCINT` bit seen on this channel since boot, OR'd
    /// together — see [`seen_interrupts`].
    seen_hcint: u32,
}

impl Slot {
    const fn new() -> Self {
        Self {
            wait: Wait::None,
            ready: None,
            waker: None,
            last_hcint: 0,
            seen_hcint: 0,
        }
    }
}

static SLOTS: Mutex<RefCell<[Slot; MAX_CHANNELS]>> =
    Mutex::new(RefCell::new([const { Slot::new() }; MAX_CHANNELS]));

/// Root-port attach/detach handshake between [`on_irq`] and
/// [`Dwc2Host::wait_for_port_change`].
///
/// A single slot rather than a table: there is one root port. It is also
/// deliberately *latching* — an event that arrives before anyone is
/// waiting is remembered rather than dropped, because a device plugged
/// in a moment before the application got round to asking would
/// otherwise never be noticed.
struct PortSlot {
    pending: bool,
    waker: Option<Waker>,
}

static PORT: Mutex<RefCell<PortSlot>> = Mutex::new(RefCell::new(PortSlot {
    pending: false,
    waker: None,
}));

/// Unmasks or masks `GINTMSK.SOFM`.
///
/// Start-of-frame is left masked by [`Dwc2Host::init`] and opened only
/// while some channel is waiting on a microframe. At high speed SOF
/// fires every 125µs — 8000 times a second — and it is a level source,
/// so leaving it unmasked with nothing to service it doesn't merely
/// waste cycles, it re-enters the handler forever.
fn set_sof_mask(global: &USB_OTG_GLOBAL, unmask: bool) {
    global.gintmsk().modify(|_, w| w.sofm().bit(unmask));
}

/// True if any channel is still parked on a microframe.
fn any_frame_waiter(slots: &[Slot; MAX_CHANNELS]) -> bool {
    slots.iter().any(|slot| matches!(slot.wait, Wait::Frame(_)))
}

/// Lets `channel`'s `HCINT` reach `GINTSTS` (and so the CPU), or stops
/// it.
///
/// This is per *wait*, not per transfer, and that is the whole point:
/// only a channel an async transfer is parked on should be able to
/// assert the controller's interrupt line. A channel driven by the
/// blocking API polls its own `HCINT`, and [`on_irq`] must leave that
/// latched for the polling loop to read — so if such a channel were
/// unmasked, its halt would raise an interrupt the handler is obliged
/// not to acknowledge. The line is level triggered, so the core would
/// re-enter the handler forever and the polling loop would never
/// resume: a hang, in any program that mixes the two styles on one
/// controller.
fn set_channel_interrupt(index: usize, enable: bool) {
    // Safe to steal: this touches only `HAINTMSK`, and only the one bit
    // belonging to a channel whose slot is being armed or disarmed here.
    let host = unsafe { USB_OTG_HOST::steal() };
    unsafe {
        host.haintmsk().modify(|r, w| {
            let bits = r.haintm().bits();
            w.haintm().bits(if enable {
                bits | (1 << index)
            } else {
                bits & !(1 << index)
            })
        });
    }
}

/// Services the DWC2 controller's interrupt for channels an async
/// transfer is waiting on: captures and acknowledges each halt, notes
/// each awaited microframe, and wakes the futures concerned.
///
/// Call this from the application's `__irq_handler` when
/// `crate::lic::Lic::is_usb_pending` reports the
/// USB line. Harmless to call spuriously — with nothing waiting it does
/// nothing at all, and in particular it never touches a channel the
/// blocking API is driving.
///
/// Acknowledging inside the handler is mandatory, not housekeeping: both
/// sources here are level-triggered, so an interrupt returned from
/// without clearing `HCINT` (or `GINTSTS.SOF`) is re-entered
/// immediately and forever.
pub fn on_irq() {
    // Safe to steal: this only reads channel status and clears the
    // interrupt flags of channels the async layer armed. A channel with
    // a `Channel` handle driving it through the blocking path is skipped
    // entirely, below, so no register another owner is using is touched.
    let global = unsafe { USB_OTG_GLOBAL::steal() };
    let host = unsafe { USB_OTG_HOST::steal() };

    let halted = host.haint().read().haint().bits() as u32;
    let gintsts = global.gintsts().read();
    let sof = gintsts.sof().bit_is_set();

    // Root port first, and unconditionally: `GINTSTS.HPRTINT` is
    // read-only and stays asserted until `HPRT`'s change bits are
    // cleared, so this has to happen whether or not anyone is waiting —
    // an unacknowledged port change is a hang, not a missed event.
    if gintsts.hprtint().bit_is_set() {
        // `.write()`, not `.modify()`: `PENA` *disables* an enabled port
        // when written 1, so re-writing what was read would knock the
        // port down. Setting `PPWR` keeps it powered; the three change
        // bits are write-1-to-clear.
        host.hprt().write(|w| {
            w.ppwr().set_bit();
            w.pcdet().set_bit();
            w.penchng().set_bit();
            w.pocchng().set_bit()
        });
        critical_section::with(|cs| {
            let mut port = PORT.borrow_ref_mut(cs);
            port.pending = true;
            if let Some(waker) = port.waker.take() {
                waker.wake();
            }
        });
    }

    if sof {
        // Write-1-to-clear, and exactly one bit. `.modify()` would write
        // back every other status bit it read, clearing flags this
        // handler never looked at; even `.write()` would OR in this
        // register's non-zero reset value. Raw bits say what is meant.
        unsafe {
            global.gintsts().write(|w| w.bits(GINTSTS_SOF));
        }
    }
    if halted == 0 && !sof {
        return;
    }

    // Read once, outside the loop: every channel waiting on a microframe
    // is comparing against the same counter.
    let microframe = (host.hfnum().read().frnum().bits() & 7) as u8;

    critical_section::with(|cs| {
        let mut slots = SLOTS.borrow_ref_mut(cs);

        for index in 0..MAX_CHANNELS {
            let slot = &mut slots[index];
            match slot.wait {
                Wait::Halt if halted & (1 << index) != 0 => {
                    let ch = host.host_channel(index);
                    let hcint = ch.hcint().read().bits();
                    // `HCINTMSK` is programmed to `CHH` alone for a
                    // channel start (see `Channel::start_channel`), so
                    // this is the halt — but check rather than assume,
                    // and leave anything else latched for the transfer
                    // to read.
                    if hcint & HCINT_CHH == 0 {
                        continue;
                    }
                    unsafe {
                        ch.hcint().write(|w| w.bits(0xffff_ffff));
                    }
                    slot.wait = Wait::None;
                    slot.ready = Some(hcint);
                    slot.last_hcint = hcint;
                    slot.seen_hcint |= hcint;
                    if let Some(waker) = slot.waker.take() {
                        waker.wake();
                    }
                }
                Wait::Frame(frame) if sof && frame == microframe => {
                    slot.wait = Wait::None;
                    slot.ready = Some(0);
                    if let Some(waker) = slot.waker.take() {
                        waker.wake();
                    }
                }
                _ => {}
            }
        }

        if sof && !any_frame_waiter(&slots) {
            set_sof_mask(&global, false);
        }
    });
}

/// The raw `HCINT` captured at the most recent interrupt-driven halt on
/// `channel`, or `0` if that channel has not halted since boot.
///
/// [`Channel::last_interrupt`] reports the same thing and is the one to
/// reach for normally. This exists for the case that one cannot cover:
/// a driver stack built on these pipes *owns* the [`Channel`], several
/// layers down, so when it reports a transfer failure there is no way to
/// ask the channel what the hardware actually said. Indexing by channel
/// number needs nothing but the number, which an application always has.
///
/// Out-of-range indices read `0` rather than panicking — this is a
/// diagnostic, and one that faults while reporting a fault is worse than
/// useless.
pub fn last_interrupt(channel: usize) -> u32 {
    if channel >= MAX_CHANNELS {
        return 0;
    }
    critical_section::with(|cs| SLOTS.borrow_ref(cs)[channel].last_hcint)
}

/// Every `HCINT` bit seen on `channel` since boot, OR'd together.
///
/// [`last_interrupt`] reports one halt, which is the wrong tool for a
/// transfer that halts many times: a multi-packet transfer, or a split
/// one polling its complete-split, ends on whichever condition happened
/// last and every condition before it is gone. That matters when the
/// last one is a consequence rather than a cause — a device answering
/// `STALL` says nothing about the toggle error or overrun three packets
/// earlier that provoked it.
///
/// Sticky, and never cleared, so a bit set here means "this happened at
/// some point", not "this is happening now".
pub fn seen_interrupts(channel: usize) -> u32 {
    if channel >= MAX_CHANNELS {
        return 0;
    }
    critical_section::with(|cs| SLOTS.borrow_ref(cs)[channel].seen_hcint)
}

/// Arms `index`'s slot to wait for `wait`, discarding anything stale.
///
/// Called *before* the hardware is poked, so an interrupt that arrives
/// between arming and the first poll is still recognised as ours.
fn arm(index: usize, wait: Wait) {
    critical_section::with(|cs| {
        let mut slots = SLOTS.borrow_ref_mut(cs);
        let slot = &mut slots[index];
        slot.wait = wait;
        slot.ready = None;
        slot.waker = None;

        // Recompute the mask from the whole table rather than just
        // opening it for a frame wait. Arming is also how a channel
        // *stops* waiting on a microframe — a periodic split alternates
        // between frame waits and halt waits on one channel — so an
        // arm that only ever unmasked would leave start-of-frame on
        // after the last frame waiter became a halt waiter. That is not
        // merely untidy: SOF is 8000 interrupts a second on a
        // high-speed port, and leaving it running with nothing to
        // service steals enough time from the executor to push other
        // endpoints' transfers past their deadlines.
        let global = unsafe { USB_OTG_GLOBAL::steal() };
        set_sof_mask(&global, any_frame_waiter(&slots));

        // Only a halt wait needs the channel's interrupt: a microframe
        // wait resolves on start-of-frame, and the channel is not even
        // running.
        set_channel_interrupt(index, matches!(wait, Wait::Halt));
    });
}

/// Returns `index`'s slot to rest, reporting whether it was still
/// waiting (i.e. the wait was abandoned rather than satisfied).
fn disarm(index: usize) -> bool {
    critical_section::with(|cs| {
        let mut slots = SLOTS.borrow_ref_mut(cs);
        let slot = &mut slots[index];
        let was_waiting = slot.wait != Wait::None;
        slot.wait = Wait::None;
        slot.ready = None;
        slot.waker = None;

        if !any_frame_waiter(&slots) {
            let global = unsafe { USB_OTG_GLOBAL::steal() };
            set_sof_mask(&global, false);
        }

        // Nothing is waiting on this channel any more, so it must stop
        // asserting the interrupt line — including for whoever uses it
        // next, which may well be the blocking API.
        set_channel_interrupt(index, false);
        was_waiting
    })
}

/// A pending wait on one channel event, already armed by [`arm`].
///
/// Its `Drop` is the cancellation path: an abandoned wait must not leave
/// the slot claimed (the next interrupt would ack a halt nobody is
/// reading) nor the channel live (the next transfer would re-arm a
/// running channel — see [`Channel::abort_channel`]).
struct SlotWait<'a, 'c> {
    channel: &'a Channel<'c>,
    /// Only meaningful for a halt wait: a microframe wait leaves no
    /// hardware running, so there is nothing to abort.
    abort_on_drop: bool,
    timer: &'a Timer,
}

impl Future for SlotWait<'_, '_> {
    type Output = u32;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<u32> {
        let this = self.get_mut();
        critical_section::with(|cs| {
            let mut slots = SLOTS.borrow_ref_mut(cs);
            let slot = &mut slots[this.channel.index];

            if let Some(value) = slot.ready.take() {
                this.abort_on_drop = false;
                return Poll::Ready(value);
            }
            // Re-register every poll: a task can be moved between
            // wakers, and the one stored on the previous poll may no
            // longer be the one that reaches it.
            slot.waker = Some(cx.waker().clone());
            Poll::Pending
        })
    }
}

impl Drop for SlotWait<'_, '_> {
    fn drop(&mut self) {
        let abandoned = disarm(self.channel.index);
        if abandoned && self.abort_on_drop {
            self.channel.abort_channel(self.timer);
        }
    }
}

/// A pending wait for the next root-port change — see
/// [`Dwc2Host::wait_for_port_change`].
struct PortChange;

impl Future for PortChange {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        critical_section::with(|cs| {
            let mut port = PORT.borrow_ref_mut(cs);
            if core::mem::take(&mut port.pending) {
                return Poll::Ready(());
            }
            port.waker = Some(cx.waker().clone());
            Poll::Pending
        })
    }
}

impl Dwc2Host {
    /// Waits for the root port to report a change — a device attaching
    /// or detaching, or the port's enable state changing.
    ///
    /// Read [`port_connected`](Dwc2Host::port_connected) afterwards to
    /// see which it was; the interrupt says *that* something changed,
    /// and `HPRT`'s change bits are consumed by [`on_irq`] to keep the
    /// line from re-asserting, so they aren't available to inspect
    /// here.
    ///
    /// Latching, so a device that turned up before this was first
    /// awaited is still reported. That means the first call after
    /// bring-up may well return immediately — which is correct, since
    /// powering the port is itself enough to make a device attach.
    pub async fn wait_for_port_change(&self) {
        PortChange.await;
    }

    /// Discards a latched root-port change without waiting for one.
    ///
    /// The latch in [`Self::wait_for_port_change`] is what stops an
    /// attach being missed, but it cannot tell an attach apart from a
    /// change *the caller itself caused*: [`Self::reset_port`] sets
    /// `PCDET` and `PENCHNG`, so a reset leaves a change pending and the
    /// next wait returns instantly. A caller that has just reset the
    /// port calls this to say "that one was mine".
    ///
    /// Without it, a loop of the shape "wait for a change, reset, do
    /// something, repeat" never actually waits after its first pass, and
    /// the resets pile up on each other — the second landing while the
    /// device is still in its recovery time, so the transfer after it
    /// times out. That alternation (works, times out, works, …) is the
    /// signature of a missing call here.
    pub fn clear_port_change(&self) {
        critical_section::with(|cs| {
            PORT.borrow_ref_mut(cs).pending = false;
        });
    }
}

impl Channel<'_> {
    /// Arms this channel's slot, runs `start`, then waits for the halt
    /// interrupt and returns the captured `HCINT`.
    ///
    /// The order matters: the slot is claimed before the channel is
    /// enabled, because the transfer can complete before this function's
    /// caller is ever polled, and an interrupt that finds an unclaimed
    /// slot would leave `HCINT` set and the line asserted.
    async fn start_and_wait(
        &self,
        endpoint: ControlEndpoint,
        txn: &Transaction,
        complete_split: bool,
        odd_frame: Option<bool>,
        timer: &Timer,
    ) -> u32 {
        arm(self.index, Wait::Halt);
        let wait = SlotWait {
            channel: self,
            abort_on_drop: true,
            timer,
        };
        self.start_channel(endpoint, txn, complete_split, odd_frame);
        let hcint = wait.await;
        self.last_hcint.set(hcint);
        hcint
    }

    /// Waits for `HFNUM`'s microframe counter to reach `frame`, via the
    /// start-of-frame interrupt.
    ///
    /// Returns immediately if the counter is already there — the whole
    /// point of the wait is to *be* in that microframe, and having
    /// arrived is not a reason to sit out a further seven.
    async fn wait_for_microframe_async(&self, frame: u8, timer: &Timer) {
        if self.current_microframe() == frame {
            return;
        }
        arm(self.index, Wait::Frame(frame));
        SlotWait {
            channel: self,
            abort_on_drop: false,
            timer,
        }
        .await;
    }

    /// Waits `microframes` microframes from now. At most 7 — the counter
    /// is three bits wide, so a wait of 8 or more is indistinguishable
    /// from no wait at all. [`Self::wait_microframes`] is the version
    /// that handles longer intervals.
    async fn delay_microframes(&self, microframes: u8, timer: &Timer) {
        let target = (self.current_microframe() + microframes) & 7;
        self.wait_for_microframe_async(target, timer).await;
    }

    /// Waits `microframes` 125µs microframes of *bus* time, driven by the
    /// controller's start-of-frame interrupt rather than a wall clock.
    ///
    /// Offered because pacing is the one thing an interrupt endpoint's
    /// caller still has to do for itself (see
    /// [`Self::interrupt_in_async`]), and `bInterval` is denominated in
    /// exactly these units on a high-speed bus. Using it keeps polls on
    /// the same clock the bus schedule runs on, and — like everything
    /// else here — needs no time crate. An application that already has
    /// one loses nothing by using that instead; this is not a general
    /// -purpose delay, and it resolves no faster than the next SOF.
    ///
    /// Long waits are split into hops of at most 7 microframes, since
    /// `HFNUM`'s counter only distinguishes eight.
    pub async fn wait_microframes(&self, microframes: u32, timer: &Timer) {
        let mut remaining = microframes;
        while remaining > 0 {
            let step = remaining.min(7) as u8;
            self.delay_microframes(step, timer).await;
            remaining -= u32::from(step);
        }
    }

    /// The async twin of [`Channel::run_transaction`]: same split
    /// handling, same packet-at-a-time loop, same PID advance — see that
    /// method for why each is shaped the way it is.
    async fn run_transaction_async(
        &self,
        endpoint: ControlEndpoint,
        txn: &Transaction,
        timer: &Timer,
    ) -> Result<usize, TransferError> {
        if endpoint.split.is_none() {
            let requested = txn.transfer_size as usize;

            // A *periodic* transfer has to start on a microframe
            // boundary; a non-periodic one can start whenever.
            //
            // The core runs a periodic transaction within the microframe
            // it is armed in, and reports `HCINT.FRMOR` if the microframe
            // ends first. Arming one at an arbitrary moment therefore
            // works only by luck — with however much of the current
            // microframe happens to be left. That luck holds while a
            // single endpoint is running and the timing is effectively
            // self-clocked, and runs out as soon as something else shares
            // the bus: two interrupt endpoints polling at once, and the
            // second is armed at whatever point in the microframe the
            // first one's completion left the executor. Waiting for the
            // next boundary hands the transaction a whole 125µs instead
            // of a sliver.
            //
            // `ODDFRM` has to name that same microframe, not the current
            // one, or the core schedules the transaction into a frame
            // parity that has already gone.
            //
            // The blocking twin does not do this. It is not that it is
            // exempt — the hardware is the same — but that it drives one
            // periodic endpoint at a time, which is the case the luck
            // covers, and it is verified that way on real hardware.
            let odd_frame = if txn.endpoint_type == EndpointType::Interrupt {
                let next_frame = (self.current_microframe() + 1) & 7;
                self.wait_for_microframe_async(next_frame, timer).await;
                Some(next_frame & 1 == 1)
            } else {
                None
            };

            let hcint = self
                .start_and_wait(endpoint, txn, false, odd_frame, timer)
                .await;
            return self.interpret_halt(hcint, requested);
        }

        let periodic = txn.endpoint_type == EndpointType::Interrupt;
        let max_packet_size = endpoint.max_packet_size.max(1) as u32;
        let mut offset = 0;
        let mut pid = txn.pid;
        loop {
            let this_packet = (txn.transfer_size - offset).min(max_packet_size);
            let packet = Transaction {
                transfer_size: this_packet,
                dma_address: txn.dma_address + offset,
                pid,
                ..*txn
            };
            let received = if periodic {
                self.run_periodic_split_packet_async(endpoint, &packet, timer)
                    .await? as u32
            } else {
                self.run_split_packet_async(endpoint, &packet, timer)
                    .await? as u32
            };
            offset += received;
            pid = pid.toggled();
            if received < this_packet || offset >= txn.transfer_size {
                break;
            }
        }
        Ok(offset as usize)
    }

    /// The async twin of [`Channel::run_split_packet`]. The only
    /// difference is how the gap between complete-split polls is taken:
    /// [`CSPLIT_RETRY_MICROFRAMES`] of bus time awaited rather than the
    /// equivalent microseconds spun.
    async fn run_split_packet_async(
        &self,
        endpoint: ControlEndpoint,
        packet: &Transaction,
        timer: &Timer,
    ) -> Result<usize, TransferError> {
        let requested = packet.transfer_size as usize;

        let hcint = self
            .start_and_wait(endpoint, packet, false, None, timer)
            .await;
        if hcint & HCINT_ACK == 0 {
            return self.interpret_halt(hcint, requested);
        }

        let mut hcint = 0;
        for attempt in 0..MAX_CSPLIT_POLLS {
            if attempt > 0 {
                self.delay_microframes(CSPLIT_RETRY_MICROFRAMES, timer)
                    .await;
            }
            hcint = self
                .start_and_wait(endpoint, packet, true, None, timer)
                .await;
            if hcint & (HCINT_NYET | HCINT_NAK) != 0 && hcint & HCINT_XFRC == 0 {
                continue;
            }
            return self.interpret_halt(hcint, requested);
        }
        self.interpret_halt(hcint, requested)
    }

    /// The async twin of [`Channel::run_periodic_split_packet`], keeping
    /// its scheduling verbatim — start-split in the next microframe
    /// (skipping 6), complete-splits from +2 onward one microframe at a
    /// time, the same 3-or-2 `NYET` retry budget. See that method for
    /// where the schedule comes from; the only change is that each
    /// microframe is awaited on the start-of-frame interrupt instead of
    /// spun on `HFNUM`.
    async fn run_periodic_split_packet_async(
        &self,
        endpoint: ControlEndpoint,
        packet: &Transaction,
        timer: &Timer,
    ) -> Result<usize, TransferError> {
        let requested = packet.transfer_size as usize;

        let mut next_frame = (self.current_microframe() + 1) & 7;
        if next_frame == 6 {
            next_frame = 7;
        }
        self.wait_for_microframe_async(next_frame, timer).await;
        let hcint = self
            .start_and_wait(endpoint, packet, false, Some(next_frame & 1 == 1), timer)
            .await;
        if hcint & HCINT_ACK == 0 {
            return self.interpret_halt(hcint, requested);
        }

        let mut tries: i32 = if next_frame != 5 { 3 } else { 2 };
        next_frame = (next_frame + 2) & 7;
        loop {
            self.wait_for_microframe_async(next_frame, timer).await;
            let hcint = self
                .start_and_wait(endpoint, packet, true, Some(next_frame & 1 == 1), timer)
                .await;

            if hcint & (HCINT_STALL | HCINT_TXERR | HCINT_BBERR | HCINT_XFRC) != 0 {
                return self.interpret_halt(hcint, requested);
            }
            if hcint & (HCINT_NYET | HCINT_ACK) != 0 {
                if tries == 0 {
                    return Err(TransferError::Nak);
                }
                tries -= 1;
                next_frame = (next_frame + 1) & 7;
                continue;
            }
            if hcint & HCINT_NAK != 0 {
                return Err(TransferError::Nak);
            }
            return self.interpret_halt(hcint, requested);
        }
    }

    /// Async [`Channel::control_setup`].
    pub async fn control_setup_async(
        &mut self,
        endpoint: ControlEndpoint,
        setup: &[u8; 8],
        timer: &Timer,
    ) -> Result<(), TransferError> {
        self.dma_buffer.0[..8].copy_from_slice(setup);
        let address = self.dma_buffer.0.as_ptr() as u32;
        clean_range(address, 8);

        let txn = Transaction {
            endpoint_number: 0,
            endpoint_type: EndpointType::Control,
            direction_in: false,
            pid: DataPid::Setup,
            transfer_size: 8,
            dma_address: address,
        };
        self.run_transaction_async(endpoint, &txn, timer).await?;
        Ok(())
    }

    /// Async [`Channel::control_data_in`]. `buf.len()` must be at most
    /// [`MAX_TRANSFER_LEN`](super::MAX_TRANSFER_LEN).
    pub async fn control_data_in_async(
        &mut self,
        endpoint: ControlEndpoint,
        buf: &mut [u8],
        timer: &Timer,
    ) -> Result<usize, TransferError> {
        debug_assert!(buf.len() <= DMA_BUFFER_LEN);

        let address = self.dma_buffer.0.as_ptr() as u32;
        let txn = Transaction {
            endpoint_number: 0,
            endpoint_type: EndpointType::Control,
            direction_in: true,
            pid: DataPid::Data1,
            transfer_size: buf.len() as u32,
            dma_address: address,
        };
        let received = self.run_transaction_async(endpoint, &txn, timer).await?;

        invalidate_range(address, buf.len());
        buf[..received].copy_from_slice(&self.dma_buffer.0[..received]);
        Ok(received)
    }

    /// Async [`Channel::control_data_out`]. `data.len()` must be at most
    /// [`MAX_TRANSFER_LEN`](super::MAX_TRANSFER_LEN).
    pub async fn control_data_out_async(
        &mut self,
        endpoint: ControlEndpoint,
        data: &[u8],
        timer: &Timer,
    ) -> Result<(), TransferError> {
        debug_assert!(data.len() <= DMA_BUFFER_LEN);

        self.dma_buffer.0[..data.len()].copy_from_slice(data);
        let address = self.dma_buffer.0.as_ptr() as u32;
        clean_range(address, data.len());

        let txn = Transaction {
            endpoint_number: 0,
            endpoint_type: EndpointType::Control,
            direction_in: false,
            pid: DataPid::Data1,
            transfer_size: data.len() as u32,
            dma_address: address,
        };
        self.run_transaction_async(endpoint, &txn, timer).await?;
        Ok(())
    }

    /// Async [`Channel::control_status_in`].
    pub async fn control_status_in_async(
        &mut self,
        endpoint: ControlEndpoint,
        timer: &Timer,
    ) -> Result<(), TransferError> {
        let address = self.dma_buffer.0.as_ptr() as u32;
        let txn = Transaction {
            endpoint_number: 0,
            endpoint_type: EndpointType::Control,
            direction_in: true,
            pid: DataPid::Data1,
            transfer_size: 0,
            dma_address: address,
        };
        self.run_transaction_async(endpoint, &txn, timer).await?;
        Ok(())
    }

    /// Async [`Channel::control_status_out`].
    pub async fn control_status_out_async(
        &mut self,
        endpoint: ControlEndpoint,
        timer: &Timer,
    ) -> Result<(), TransferError> {
        let address = self.dma_buffer.0.as_ptr() as u32;
        let txn = Transaction {
            endpoint_number: 0,
            endpoint_type: EndpointType::Control,
            direction_in: false,
            pid: DataPid::Data1,
            transfer_size: 0,
            dma_address: address,
        };
        self.run_transaction_async(endpoint, &txn, timer).await?;
        Ok(())
    }

    /// Async [`Channel::interrupt_in`], with the same `data_toggle`
    /// contract: start it `false` when the endpoint is configured, pass
    /// the same `&mut bool` back on every poll, and note that a
    /// [`TransferError::Nak`] leaves it untouched so the poll can simply
    /// be retried.
    ///
    /// Pacing polls to the endpoint's `bInterval` is still the caller's
    /// job — this waits for *a* result, not for the right moment to ask.
    pub async fn interrupt_in_async(
        &mut self,
        endpoint: ControlEndpoint,
        endpoint_number: u8,
        data_toggle: &mut bool,
        buf: &mut [u8],
        timer: &Timer,
    ) -> Result<usize, TransferError> {
        debug_assert!(buf.len() <= DMA_BUFFER_LEN);

        let address = self.dma_buffer.0.as_ptr() as u32;
        let txn = Transaction {
            endpoint_number,
            endpoint_type: EndpointType::Interrupt,
            direction_in: true,
            pid: if *data_toggle {
                DataPid::Data1
            } else {
                DataPid::Data0
            },
            transfer_size: buf.len() as u32,
            dma_address: address,
        };
        let received = self.run_transaction_async(endpoint, &txn, timer).await?;
        *data_toggle = !*data_toggle;

        invalidate_range(address, buf.len());
        buf[..received].copy_from_slice(&self.dma_buffer.0[..received]);
        Ok(received)
    }

    /// Async [`Channel::bulk_out`], with the same `data_toggle`
    /// contract. DMA reads straight from `buf`, so there is no size cap
    /// from the channel's scratch buffer.
    pub async fn bulk_out_async(
        &mut self,
        endpoint: ControlEndpoint,
        endpoint_number: u8,
        data_toggle: &mut bool,
        buf: &[u8],
        timer: &Timer,
    ) -> Result<usize, TransferError> {
        let address = buf.as_ptr() as u32;
        clean_range(address, buf.len());

        let txn = Transaction {
            endpoint_number,
            endpoint_type: EndpointType::Bulk,
            direction_in: false,
            pid: Channel::toggle_pid(*data_toggle),
            transfer_size: buf.len() as u32,
            dma_address: address,
        };
        let sent = self.run_transaction_async(endpoint, &txn, timer).await?;
        *data_toggle = self.next_toggle();
        Ok(sent)
    }

    /// Async [`Channel::bulk_in`]. `buf` must be cache-line aligned and
    /// occupy whole cache lines, and is rounded down to a whole number
    /// of max-packet packets — both for the reasons the blocking twin
    /// documents.
    pub async fn bulk_in_async(
        &mut self,
        endpoint: ControlEndpoint,
        endpoint_number: u8,
        data_toggle: &mut bool,
        buf: &mut [u8],
        timer: &Timer,
    ) -> Result<usize, TransferError> {
        let address = buf.as_mut_ptr() as u32;
        debug_assert!(
            address.is_multiple_of(crate::cache::MIN_CACHE_LINE),
            "bulk IN buffer must be cache-line aligned"
        );

        let max_packet_size = endpoint.max_packet_size.max(1) as u32;
        let transfer_size = buf.len() as u32 / max_packet_size * max_packet_size;

        let txn = Transaction {
            endpoint_number,
            endpoint_type: EndpointType::Bulk,
            direction_in: true,
            pid: Channel::toggle_pid(*data_toggle),
            transfer_size,
            dma_address: address,
        };
        let received = self.run_transaction_async(endpoint, &txn, timer).await?;
        *data_toggle = self.next_toggle();

        invalidate_range(address, buf.len());
        Ok(received)
    }
}
