//! `embedded-io-async`'s [`Read`] and [`Write`] for UART0, driven by the
//! PL011's receive and transmit interrupts.
//!
//! These are the async counterparts to the blocking `embedded-io` impls
//! on the same type — the byte-stream traits, not `embedded-hal-async`,
//! which has nothing for serial ports.
//!
//! # Wiring
//!
//! As everywhere else in this crate, three gates and an application-owned
//! dispatch: the peripheral interrupt (opened by the futures themselves),
//! `Lic::enable_uart_irq`, the CPU
//! mask ([`enable_irq`](crate::irq::enable_irq)), and a call to
//! [`on_irq`](crate::uart::on_irq) from the application's
//! `__irq_handler`.
//!
//! # Masking, not just acknowledging
//!
//! Both PL011 conditions are level-driven: RX asserts while the FIFO
//! holds data, TX asserts while it has room. Acknowledging either without
//! removing the cause leaves it asserted, and the handler re-enters
//! immediately. So [`on_irq`](crate::uart::on_irq) *masks* whichever
//! source fired and the waiting future unmasks it again next time it
//! needs to block. The data itself is left in the FIFO for the future to
//! read — the handler moves no bytes.
//!
//! [`Read`]: embedded_io_async::Read
//! [`Write`]: embedded_io_async::Write

use core::cell::RefCell;
use core::future::poll_fn;
use core::task::{Poll, Waker};

use critical_section::Mutex;

use super::Uart;
use crate::pac::UART0;

/// Woken when the receive interrupt fires; there is one UART0, so one
/// slot rather than a table.
static RX_WAKER: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));
/// Woken when the transmit interrupt fires.
static TX_WAKER: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));

/// Services the UART0 interrupt: masks whichever source fired and wakes
/// the future waiting on it.
///
/// Call this from the application's `__irq_handler` when
/// `Lic::is_uart_pending` reports the
/// source. Harmless to call spuriously.
///
/// Deliberately moves no data. A handler that drained the FIFO into a
/// buffer would be imposing a buffering policy — capacity, overflow
/// behaviour — that belongs to the application, the same reason the
/// blocking driver exposes `try_read_byte` rather than a ring buffer.
/// Masking is enough to stop the interrupt re-asserting, and the future
/// reads the bytes when it is polled.
pub fn on_irq() {
    // Safe to steal: this touches only the interrupt mask and status
    // registers, and only for sources a future has armed — a `Uart` whose
    // future is parked is borrowed by that future.
    let uart0 = unsafe { UART0::steal() };
    let status = uart0.mis().read();

    let rx = status.rxmis().bit_is_set() || status.rtmis().bit_is_set();
    let tx = status.txmis().bit_is_set();

    if rx {
        uart0
            .imsc()
            .modify(|_, w| w.rxim().clear_bit().rtim().clear_bit());
        uart0.icr().write(|w| w.rxic().set_bit().rtic().set_bit());
    }

    if tx {
        uart0.imsc().modify(|_, w| w.txim().clear_bit());
        uart0.icr().write(|w| w.txic().set_bit());
    }

    critical_section::with(|cs| {
        if rx {
            if let Some(waker) = RX_WAKER.borrow_ref_mut(cs).take() {
                waker.wake();
            }
        }
        if tx {
            if let Some(waker) = TX_WAKER.borrow_ref_mut(cs).take() {
                waker.wake();
            }
        }
    });
}

impl Uart {
    /// Waits until at least one byte is readable.
    async fn wait_rx(&mut self) {
        poll_fn(|cx| {
            if self.byte_available() {
                return Poll::Ready(());
            }

            critical_section::with(|cs| {
                *RX_WAKER.borrow_ref_mut(cs) = Some(cx.waker().clone());
            });
            self.enable_rx_irq();

            // Re-check after arming, not before only: a byte landing in
            // the gap between the test above and unmasking would
            // otherwise be waited on forever if the interrupt had already
            // come and gone.
            if self.byte_available() {
                self.disable_rx_irq();
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }

    /// Waits until the transmit FIFO has room for at least one byte.
    async fn wait_tx(&mut self) {
        poll_fn(|cx| {
            if !self.tx_full() {
                return Poll::Ready(());
            }

            critical_section::with(|cs| {
                *TX_WAKER.borrow_ref_mut(cs) = Some(cx.waker().clone());
            });
            self.enable_tx_irq();

            if !self.tx_full() {
                self.disable_tx_irq();
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }
}

impl embedded_io_async::Read for Uart {
    /// Waits for at least one byte, then takes as many more as are
    /// already in the FIFO and fit in `buf`.
    ///
    /// Returning a short read is the contract, not a shortcoming: a
    /// caller wanting `buf` filled uses `read_exact`, which loops.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        self.wait_rx().await;

        let mut n = 0;
        while n < buf.len() {
            match self.try_read_byte() {
                Some(byte) => {
                    buf[n] = byte;
                    n += 1;
                }
                None => break,
            }
        }

        Ok(n)
    }
}

impl embedded_io_async::Write for Uart {
    /// Writes as many bytes as the transmit FIFO will take right now,
    /// waiting first only if it is completely full.
    ///
    /// Never busy-waits on the wire: the point of this impl over the
    /// blocking one is that a full FIFO yields the core to other tasks
    /// instead of spinning for the ~87us each byte takes at 115200 baud.
    /// Like `read`, a short write is the contract — `write_all` loops.
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        self.wait_tx().await;

        let mut n = 0;
        while n < buf.len() && self.try_write_byte(buf[n]) {
            n += 1;
        }

        Ok(n)
    }

    /// Waits for the transmitter to fall idle, FIFO and shift register
    /// both.
    ///
    /// The FIFO is drained by yielding, but the last character is not:
    /// the PL011 raises no "transmit complete" interrupt, only "there is
    /// room", so once the FIFO empties there is nothing left to wait on
    /// but the shift register's `BUSY` bit. That tail is one character
    /// time — about 87us at 115200 baud — and it is spun on. Callers that
    /// cannot afford it should not flush; dropping bytes into the FIFO
    /// and letting it drain on its own needs no flush at all.
    async fn flush(&mut self) -> Result<(), Self::Error> {
        while self.tx_full() {
            self.wait_tx().await;
        }
        self.wait_tx_idle();
        Ok(())
    }
}
