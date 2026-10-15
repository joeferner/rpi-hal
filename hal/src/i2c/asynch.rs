//! `embedded-hal-async`'s [`I2c`] for the BSC controllers, driven by the
//! controller's own `DONE`/`TXW`/`RXR` interrupts.
//!
//! The async counterpart to the blocking `embedded-hal` impl on the same
//! type, and the same transfer model: each
//! [`Operation`](embedded_hal::i2c::Operation) is its own complete
//! START...STOP, not a repeated start. What changes is what happens while
//! the bus is busy — a six-byte read at 100kHz is most of a millisecond,
//! which the blocking driver spends spinning on `S` and this one spends
//! parked, letting the executor run something else.
//!
//! # Wiring
//!
//! Nothing here resolves until the controller's interrupt reaches
//! [`on_irq`]. That means the same three gates every interrupt source in
//! this crate goes through — the peripheral condition (opened by the
//! transfer itself), the interrupt controller
//! (`Lic::enable_i2c_irq`), and the CPU
//! mask ([`enable_irq`](crate::irq::enable_irq)) — plus a call to
//! [`on_irq`] from the application's `__irq_handler`. A library crate
//! cannot claim that symbol, so dispatch stays the application's.
//!
//! # Masking, not just acknowledging
//!
//! All three conditions are level-driven off the `S` register: `DONE`
//! stays set until written back, and `TXW`/`RXR` stay asserted while the
//! FIFO is past its threshold. Acknowledging without removing the cause
//! would re-enter the handler immediately, so [`on_irq`] *masks*
//! `C.INTD`/`INTT`/`INTR` and the transfer re-arms them each time it
//! parks again. The handler moves no data and clears no status — the
//! transfer needs both when it is polled.
//!
//! # Timeouts, which work differently here
//!
//! The blocking driver has to carry its own deadline: it owns the core
//! while it spins, so nothing else could impose one. An async transfer is
//! a future, and the idiomatic bound is the caller's own timer —
//! `embassy_time::with_timeout(Duration::from_millis(50), i2c.read(addr,
//! &mut buf))` — which puts the number where the application's judgement
//! is, and needs nothing from this driver but that dropping a transfer
//! part-way be safe. It is: the drop leaves the controller masked, its
//! FIFOs cleared and its status clean, ready for the next transfer.
//!
//! The stored [`Timer`](crate::timer::Timer) deadline is still applied,
//! as a backstop rather than the primary mechanism, and the distinction
//! matters: it is only *observed* when the future is polled. A transfer
//! that stalls in a way that raises no further interrupt — nothing at all
//! answering after the address was acknowledged — is not woken by it, and
//! stays pending until something else polls the task. That is the case to
//! use `with_timeout` for.
//!
//! [`I2c`]: embedded_hal_async::i2c::I2c
//! [`on_irq`]: crate::i2c::on_irq

use core::cell::RefCell;
use core::future::poll_fn;
use core::ops::Deref;
use core::ptr;
use core::task::{Poll, Waker};

use critical_section::Mutex;

use super::{Error, I2c};
use crate::pac::{bsc0, BSC0, BSC1};

/// One waker per controller: BSC0 and BSC1 share an interrupt line but
/// are independent buses, and a program can have a transfer in flight on
/// each. A single slot would have the second transfer to park overwrite
/// the first's waker and strand it.
static WAKERS: [Mutex<RefCell<Option<Waker>>>; 2] = [
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
];

/// Services the I2C interrupt: masks whichever controller raised it and
/// wakes the transfer parked on that one.
///
/// Call this from the application's `__irq_handler` when
/// `Lic::is_i2c_pending` reports the
/// source. Harmless to call spuriously, and harmless to a controller
/// being driven by the blocking API in the same program: that one arms
/// none of these interrupt enables, so the checks below skip it and its
/// status is left untouched for its own polling loop to read.
pub fn on_irq() {
    // Safe to steal: this touches only the interrupt-enable bits of a
    // controller that a parked transfer armed, and that transfer holds
    // the `I2c` borrow for as long as it is parked. Both are checked
    // rather than assumed — the two share one interrupt line, so either
    // may be the one asserting.
    let bsc0 = unsafe { BSC0::steal() };
    let bsc1 = unsafe { BSC1::steal() };

    let woken = [mask_if_asserted(&bsc0), mask_if_asserted(&bsc1)];

    critical_section::with(|cs| {
        for (slot, fired) in WAKERS.iter().zip(woken) {
            if fired {
                if let Some(waker) = slot.borrow_ref_mut(cs).take() {
                    waker.wake();
                }
            }
        }
    });
}

/// Masks `regs`'s interrupt enables if one of the conditions it has armed
/// is actually asserted, reporting whether it was.
///
/// The armed check comes first so that a controller nobody is waiting on
/// — one being driven by the blocking API, or idle — is left completely
/// alone. Once masked, the condition itself is left standing: `DONE` is
/// still set, the FIFO still holds its bytes, and the transfer reads both
/// when it is polled.
fn mask_if_asserted(regs: &bsc0::RegisterBlock) -> bool {
    let control = regs.c().read();
    let (done_armed, tx_armed, rx_armed) = (
        control.intd().bit_is_set(),
        control.intt().bit_is_set(),
        control.intr().bit_is_set(),
    );
    if !(done_armed || tx_armed || rx_armed) {
        return false;
    }

    let status = regs.s().read();
    // `ERR` and `CLKT` have no interrupt enable of their own, so a fault
    // reaches the ARM core only alongside one of the three that do. They
    // are tested anyway: whichever enable carried it, the transfer needs
    // waking, and this way the answer does not depend on knowing which
    // of them the controller happens to raise on a NAK.
    let asserted = status.err().bit_is_set()
        || status.clkt().bit_is_set()
        || (done_armed && status.done().bit_is_set())
        || (tx_armed && status.txw().bit_is_set())
        || (rx_armed && status.rxr().bit_is_set());
    if !asserted {
        return false;
    }

    mask(regs);
    true
}

/// Closes all three interrupt conditions. `ST` and `CLEAR` read back as
/// zero, so a read-modify-write here cannot restart a transfer or clear a
/// FIFO as a side effect — the rest of `C` (`I2CEN`, `READ`) is preserved.
fn mask(regs: &bsc0::RegisterBlock) {
    regs.c()
        .modify(|_, w| w.intd().clear_bit().intt().clear_bit().intr().clear_bit());
}

/// Returns the controller to a state the next transfer can start from,
/// for a transfer that is being abandoned rather than completed: masked,
/// both FIFOs cleared, status clean.
///
/// The same best-effort as the blocking driver's `abandon` — nothing here
/// can make a slave that is holding SDA let go — with the interrupt
/// enables closed as well, since an abandoned transfer has no one left to
/// wake.
fn abandon(regs: &bsc0::RegisterBlock) {
    mask(regs);
    regs.c().write(|w| {
        unsafe { w.clear().bits(0b11) };
        w.i2cen().bit(true)
    });
    regs.s()
        .write(|w| w.done().bit(true).err().bit(true).clkt().bit(true));
}

/// Cleans up after a transfer that is dropped part-way through — the
/// cancellation the `with_timeout` pattern in this module's docs depends
/// on, and equally what happens when the task holding it is dropped.
///
/// Without this a cancelled transfer would leave the controller mid-burst
/// with its interrupts armed and a stale waker in its slot: the next
/// transfer on that bus would inherit whatever was in the FIFOs, and the
/// next interrupt would wake a future that no longer exists.
struct OnCancel {
    /// The controller's registers. A raw pointer rather than a reference
    /// because the future holds `&mut I2c` for as long as it is parked,
    /// and this has to survive alongside that borrow; it is an MMIO
    /// address, so it is valid for as long as the program runs.
    regs: *const bsc0::RegisterBlock,
    /// Cleared once the transfer has finished and tidied up after itself.
    armed: bool,
}

impl Drop for OnCancel {
    fn drop(&mut self) {
        if self.armed {
            // SAFETY: `regs` is the MMIO block of the controller this
            // transfer owns, valid for the whole program, and the
            // transfer being dropped is what gives up that ownership.
            abandon(unsafe { &*self.regs });
        }
    }
}

impl<I: Deref<Target = bsc0::RegisterBlock>> I2c<'_, I> {
    /// Which of [`WAKERS`] this instance parks in, by register-block
    /// address. Comparing addresses rather than adding a trait keeps the
    /// blocking driver's `Deref` bound as the only thing an instance has
    /// to satisfy.
    fn waker_slot(&self) -> usize {
        usize::from(ptr::eq(ptr::from_ref(&*self.bsc), BSC1::PTR))
    }

    /// Ends a transfer's interest in the controller: interrupts masked,
    /// and the waker slot emptied.
    ///
    /// Emptying the slot is tidiness rather than correctness — the next
    /// transfer on this bus overwrites it before arming anything, and
    /// `on_irq` only wakes a controller whose enables are set — but a
    /// finished transfer holding a live `Waker` clone for an arbitrary
    /// time afterwards is not something to leave lying around.
    fn unpark(&self) {
        mask(&self.bsc);
        critical_section::with(|cs| {
            *WAKERS[self.waker_slot()].borrow_ref_mut(cs) = None;
        });
    }

    /// Parks until the controller interrupts, arming `INTD` plus whichever
    /// of `INTT`/`INTR` this direction needs.
    ///
    /// Re-checks after arming rather than only before: a condition that
    /// becomes true in the gap between the caller's test and the unmask
    /// has already raised and cleared its interrupt, and waiting on it
    /// would wait forever.
    fn park(&self, waker: &Waker, reading: bool) -> bool {
        critical_section::with(|cs| {
            *WAKERS[self.waker_slot()].borrow_ref_mut(cs) = Some(waker.clone());
        });
        self.bsc.c().modify(|_, w| {
            w.intd().set_bit();
            if reading {
                w.intr().set_bit()
            } else {
                w.intt().set_bit()
            }
        });

        let status = self.bsc.s().read();
        let ready = status.done().bit_is_set()
            || status.err().bit_is_set()
            || status.clkt().bit_is_set()
            || if reading {
                status.rxd().bit_is_set()
            } else {
                status.txd().bit_is_set()
            };
        if ready {
            mask(&self.bsc);
        }
        ready
    }

    /// Writes `bytes` as one complete transaction, yielding whenever the
    /// transmit FIFO is full rather than spinning on it.
    ///
    /// The async twin of the blocking `write_one`, and it reports the same
    /// errors for the same reasons — see that method.
    async fn write_one_async(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error> {
        if bytes.is_empty() {
            return Err(Error::ZeroLengthUnsupported);
        }

        let mut cancel = OnCancel {
            regs: ptr::from_ref(&*self.bsc),
            armed: true,
        };
        self.one_shot(address, false, bytes.len());
        let deadline = self.timer.now_micros() + Self::timeout_us(bytes.len());

        let mut sent = 0;
        let outcome = poll_fn(|cx| {
            loop {
                let status = self.bsc.s().read();
                if status.err().bit_is_set() {
                    self.unpark();
                    self.clear_status();
                    return Poll::Ready(Err(Error::NoAcknowledge));
                }
                // A clock-stretch timeout means the controller gave up on
                // a slave holding SCL down: the bus was held, which is
                // what `Timeout` says, and the transfer is over either way.
                if status.clkt().bit_is_set() {
                    self.unpark();
                    self.abandon();
                    return Poll::Ready(Err(Error::Timeout));
                }
                if sent < bytes.len() && status.txd().bit_is_set() {
                    unsafe {
                        self.bsc.fifo().write(|w| w.data().bits(bytes[sent]));
                    }
                    sent += 1;
                    continue;
                }
                if status.done().bit_is_set() {
                    self.unpark();
                    self.clear_status();
                    return Poll::Ready(Ok(()));
                }
                if self.timer.now_micros() > deadline {
                    self.unpark();
                    self.abandon();
                    return Poll::Ready(Err(Error::Timeout));
                }
                if !self.park(cx.waker(), false) {
                    return Poll::Pending;
                }
            }
        })
        .await;

        cancel.armed = false;
        outcome
    }

    /// Reads into `buffer` as one complete transaction, yielding whenever
    /// the receive FIFO is empty rather than spinning on it.
    ///
    /// The async twin of the blocking `read_one`, including the part that
    /// is not obvious there: `DONE` can assert with bytes still in the
    /// FIFO, so a full buffer — not `DONE` — is what ends the read, and a
    /// transfer that finished having delivered fewer bytes than asked for
    /// is [`Error::Incomplete`] rather than a wait that never ends.
    async fn read_one_async(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error> {
        if buffer.is_empty() {
            return Err(Error::ZeroLengthUnsupported);
        }

        let mut cancel = OnCancel {
            regs: ptr::from_ref(&*self.bsc),
            armed: true,
        };
        self.one_shot(address, true, buffer.len());
        let deadline = self.timer.now_micros() + Self::timeout_us(buffer.len());

        let mut received = 0;
        let outcome = poll_fn(|cx| loop {
            let status = self.bsc.s().read();
            if status.err().bit_is_set() {
                self.unpark();
                self.clear_status();
                return Poll::Ready(Err(Error::NoAcknowledge));
            }
            if status.clkt().bit_is_set() {
                self.unpark();
                self.abandon();
                return Poll::Ready(Err(Error::Timeout));
            }
            if received < buffer.len() && status.rxd().bit_is_set() {
                buffer[received] = self.bsc.fifo().read().data().bits();
                received += 1;
                continue;
            }
            if status.done().bit_is_set() && received >= buffer.len() {
                self.unpark();
                self.clear_status();
                return Poll::Ready(Ok(()));
            }
            if self.timer.now_micros() > deadline {
                let complete = status.done().bit_is_set();
                self.unpark();
                self.abandon();
                return Poll::Ready(Err(if complete {
                    Error::Incomplete {
                        received,
                        requested: buffer.len(),
                    }
                } else {
                    Error::Timeout
                }));
            }
            if !self.park(cx.waker(), true) {
                return Poll::Pending;
            }
        })
        .await;

        cancel.armed = false;
        outcome
    }
}

impl<I: Deref<Target = bsc0::RegisterBlock>> embedded_hal_async::i2c::I2c for I2c<'_, I> {
    /// `read`/`write`/`write_read` all forward here via
    /// `embedded_hal_async::i2c::I2c`'s default implementations. As in the
    /// blocking impl, each [`Operation`](embedded_hal::i2c::Operation)
    /// gets its own complete START...STOP, not a true repeated start.
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        for operation in operations {
            match operation {
                embedded_hal::i2c::Operation::Read(buffer) => {
                    self.read_one_async(address, buffer).await?
                }
                embedded_hal::i2c::Operation::Write(bytes) => {
                    self.write_one_async(address, bytes).await?
                }
            }
        }
        Ok(())
    }
}
