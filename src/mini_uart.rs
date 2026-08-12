//! Blocking driver for the mini UART (UART1), one of the three
//! sub-peripherals behind the AUX block (alongside SPI1/SPI2 — see
//! [`crate::aux_spi`]).
//!
//! Distinct from UART0 (PL011, [`crate::uart`]): the mini UART is a
//! cut-down 16550-style device with shallower FIFOs, no fractional baud
//! divisor, and — the important operational difference — a reference
//! clock tied to the VPU/core clock rather than a fixed peripheral
//! clock. Its main use here is a second serial console: the on-board
//! wireless chip's Bluetooth side is hardwired to PL011, so once PL011
//! is committed to Bluetooth the debug console has to move to the mini
//! UART on GPIO14/15.

use crate::pac::{AUX, GPIO, UART1};
use core::fmt;

/// GPIO peripheral base address, matching `bcm2837_lpa::GPIO::PTR`.
/// Kept in sync manually since the pull-control registers below aren't
/// in that PAC's SVD at all — same situation as [`crate::uart`], which
/// pokes these same addresses for the same reason.
#[cfg(not(feature = "bcm2711"))]
const GPIO_BASE: usize = 0x3f20_0000;
/// GPIO Pull-up/down Enable (BCM2835 ARM Peripherals datasheet §6.1).
#[cfg(not(feature = "bcm2711"))]
const GPPUD: *mut u32 = (GPIO_BASE + 0x94) as *mut u32;
/// GPIO Pull-up/down Enable Clock 0, covers GPIO0-31.
#[cfg(not(feature = "bcm2711"))]
const GPPUDCLK0: *mut u32 = (GPIO_BASE + 0x98) as *mut u32;

/// The mini UART's reference clock, in Hz.
///
/// Unlike PL011's fixed 48MHz reference, the mini UART is clocked from
/// the VPU/core clock, which by default is *not fixed*: the firmware
/// scales it dynamically (nominally 250MHz at idle, higher under load),
/// the same not-firmware-guaranteed clock [`crate::spi`] and [`crate::i2c`]
/// warn about for their dividers. Two consequences worth knowing:
///
/// * Baud rates computed here (see [`MiniUart::init`]/[`MiniUart::set_baud`])
///   assume this value; if the core clock isn't 250MHz — or won't hold
///   still — the link garbles. **Add `core_freq=250` to `config.txt`** to
///   pin it: that stops the firmware scaling it, so the mini UART's baud
///   stays put. Note `enable_uart=1` alone is *not* reliable here — it
///   only pins the clock when the mini UART is the firmware's *primary*
///   UART; if PL011 is primary (e.g. Bluetooth disabled, so PL011 drives
///   GPIO14/15), the firmware leaves the core clock free and the mini
///   UART still garbles. Confirmed on hardware: `enable_uart=1` didn't
///   fix it, `core_freq=250` did.
/// * A one-shot mailbox query of the core clock at init would *not* be a
///   real fix, since the clock can scale afterward and the snapshot goes
///   stale — pinning the clock is the actual requirement, and once pinned
///   this constant is simply correct.
pub const CORE_CLOCK_HZ: u32 = 250_000_000;

/// Disables the pull resistor on GPIO14/15 via the legacy
/// GPPUD/GPPUDCLK0 sequence — see [`crate::uart`]'s equivalent for why
/// this pokes physical addresses directly rather than going through the
/// PAC (the BCM2836/2837 pull registers aren't modeled in
/// `bcm2837-lpa`'s BCM2711-shaped SVD). `gpio` is unused on this side —
/// taken anyway so both branches share one call site; see the
/// `bcm2711` branch below for the side that actually needs it.
#[cfg(not(feature = "bcm2711"))]
fn disable_gpio14_15_pull(_gpio: &GPIO) {
    const GPIO14_15_MASK: u32 = (1 << 14) | (1 << 15);
    unsafe {
        core::ptr::write_volatile(GPPUD, 0);
        spin_delay(150);
        core::ptr::write_volatile(GPPUDCLK0, GPIO14_15_MASK);
        spin_delay(150);
        core::ptr::write_volatile(GPPUD, 0);
        core::ptr::write_volatile(GPPUDCLK0, 0);
    }
}

/// BCM2711 counterpart — see [`crate::uart`]'s identical `disable_pull`
/// for the full rationale (real `GPIO_PUP_PDN_CNTRL_REG0` scheme,
/// modeled correctly in `bcm2711-lpa`, so this goes through the PAC
/// instead of poking raw addresses). Both GPIO14 and GPIO15 fall in
/// `_REG0` (pins 0-15).
#[cfg(feature = "bcm2711")]
fn disable_gpio14_15_pull(gpio: &GPIO) {
    gpio.gpio_pup_pdn_cntrl_reg0().modify(|_, w| {
        w.gpio_pup_pdn_cntrl14()
            .none()
            .gpio_pup_pdn_cntrl15()
            .none()
    });
}

#[cfg(not(feature = "bcm2711"))]
fn spin_delay(cycles: u32) {
    for _ in 0..cycles {
        unsafe { core::arch::asm!("nop") };
    }
}

/// Converts a target baud rate to the mini UART's `BAUD` register value,
/// or `None` if it can't be represented.
///
/// The mini UART divides its reference clock by `8 * (baud_reg + 1)`, so
/// `baud_reg = round(clk / (8 * baud)) - 1`. A `baud` so high the ratio
/// rounds to 0, or so low it overflows the 16-bit register, is rejected.
fn baud_to_reg(baud: u32) -> Option<u16> {
    if baud == 0 {
        return None;
    }
    let denom = 8 * baud;
    let ratio = (CORE_CLOCK_HZ + denom / 2) / denom;
    if ratio == 0 {
        return None;
    }
    let reg = ratio - 1;
    if reg > u16::MAX as u32 {
        return None;
    }
    Some(reg as u16)
}

/// Blocking driver for the mini UART (UART1).
pub struct MiniUart {
    uart1: UART1,
}

impl MiniUart {
    /// Wraps an already-initialized `UART1` without touching hardware —
    /// unlike [`init`](Self::init), this doesn't enable the peripheral,
    /// remux GPIO, reconfigure baud, or drain the RX FIFO. Needed for
    /// contexts (like an IRQ handler) that only have a freshly-stolen
    /// `UART1` token, not the original `MiniUart` instance `init`
    /// returned; mirrors [`crate::uart::Uart::from_initialized`].
    pub fn from_initialized(uart1: UART1) -> Self {
        Self { uart1 }
    }

    /// Enables the mini UART in the shared AUX block, routes GPIO14/15
    /// to it, and brings it up at 115200 8N1.
    ///
    /// `aux` is taken by reference, not consumed, because the `AUX`
    /// block's `ENABLES` register is shared with SPI1/SPI2: this sets
    /// only the UART1 bit (via `modify`), leaving any already-enabled
    /// aux SPI untouched, so the caller keeps `AUX` to lend to
    /// [`crate::aux_spi`] as well. Enabling UART1 here is also what makes
    /// the rest of its registers respond at all — they read back garbage
    /// until the block is enabled — so it happens before anything else.
    ///
    /// Baud assumes [`CORE_CLOCK_HZ`]; see that constant on the core-clock
    /// caveat and how to work around it. The 8-bit word size is written
    /// as the two-bit `0b11` encoding the hardware actually requires (a
    /// documented BCM2835 quirk — bit 0 alone does not select 8-bit).
    pub fn init(gpio: &GPIO, aux: &AUX, uart1: UART1) -> Self {
        // Must precede any other UART1 register access: the sub-peripheral
        // registers don't respond until their AUX_ENABLES bit is set.
        aux.enables().modify(|_, w| w.uart_1().set_bit());

        disable_gpio14_15_pull(gpio);
        gpio.gpfsel1()
            .modify(|_, w| w.fsel14().txd1().fsel15().rxd1());

        // Hold TX/RX disabled while (re)configuring framing and baud.
        uart1
            .cntl()
            .write(|w| w.tx_enable().clear_bit().rx_enable().clear_bit());
        // No interrupts by default — polled console. `enable_rx_irq`
        // opts in.
        uart1
            .ier()
            .write(|w| w.data_ready().clear_bit().tx_ready().clear_bit());
        // 8N1: 8-bit words. Writing the IIR's FIFO-clear bits empties
        // both FIFOs for a clean slate (the mini UART's FIFOs are always
        // enabled and can't be turned off).
        uart1.lcr().write(|w| w.data_size()._8bit());
        uart1
            .iir()
            .write(|w| w.data_ready().set_bit().tx_ready().set_bit());

        // 115200 is representable at the assumed clock, so the unwrap
        // can't fire; fall back to leaving the reset divisor in place if
        // that ever changes rather than panicking in a HAL constructor.
        if let Some(reg) = baud_to_reg(115_200) {
            uart1.baud().write(|w| unsafe { w.bits(reg) });
        }

        uart1
            .cntl()
            .write(|w| w.tx_enable().set_bit().rx_enable().set_bit());

        // Drain any stray byte(s) already latched in the RX FIFO, bounded
        // at the FIFO's own depth (8 bytes) rather than "until empty", for
        // the same reason as [`crate::uart::Uart::init`]: a genuinely
        // continuous RX stream would make "until empty" hang init forever.
        let me = Self { uart1 };
        for _ in 0..8 {
            if me.try_read_byte().is_none() {
                break;
            }
        }
        me
    }

    /// Blocks until the transmit FIFO has room, then writes one byte.
    pub fn write_byte(&mut self, byte: u8) {
        while self.uart1.lsr().read().tx_empty().bit_is_clear() {}
        unsafe {
            self.uart1.io().write(|w| w.data().bits(byte));
        }
    }

    /// Blocks until the transmitter is fully idle: both the TX FIFO
    /// drained *and* the last bit shifted out onto the line (`LSR`'s
    /// `TX_IDLE`). `write_byte` only waits for FIFO room, which says
    /// nothing about bytes still queued or shifting — flush before
    /// anything that would disturb an in-flight byte, e.g. reprogramming
    /// the baud divisor via [`set_baud`](Self::set_baud).
    pub fn flush(&mut self) {
        while self.uart1.lsr().read().tx_idle().bit_is_clear() {}
    }

    /// Reprograms the baud rate on the fly, returning `false` (leaving
    /// the current rate untouched) if `baud` can't be represented at
    /// [`CORE_CLOCK_HZ`].
    ///
    /// Briefly disables TX/RX to change the divisor, so a caller with a
    /// byte the other end still needs to read cleanly (e.g. an ack)
    /// should [`flush`](Self::flush) first — otherwise its tail is sent
    /// at the new rate and garbled.
    #[must_use]
    pub fn set_baud(&mut self, baud: u32) -> bool {
        let Some(reg) = baud_to_reg(baud) else {
            return false;
        };
        self.uart1
            .cntl()
            .modify(|_, w| w.tx_enable().clear_bit().rx_enable().clear_bit());
        self.uart1.baud().write(|w| unsafe { w.bits(reg) });
        self.uart1
            .cntl()
            .modify(|_, w| w.tx_enable().set_bit().rx_enable().set_bit());
        true
    }

    /// Blocks until a byte is available, then reads and returns it.
    pub fn read_byte(&mut self) -> u8 {
        while self.uart1.lsr().read().data_ready().bit_is_clear() {}
        self.uart1.io().read().data().bits()
    }

    /// Non-blocking: true if a byte is waiting in the receive FIFO.
    pub fn byte_available(&self) -> bool {
        self.uart1.lsr().read().data_ready().bit_is_set()
    }

    /// Non-blocking sibling of `read_byte`: `None` if the receive FIFO is
    /// empty, otherwise reads and returns one byte. No buffering — a
    /// caller holding bytes across calls (e.g. an IRQ handler feeding a
    /// queue) owns that itself, same as [`crate::uart::Uart::try_read_byte`].
    pub fn try_read_byte(&self) -> Option<u8> {
        if self.uart1.lsr().read().data_ready().bit_is_clear() {
            return None;
        }
        Some(self.uart1.io().read().data().bits())
    }

    /// Unmasks the receive interrupt (`IER`'s `DATA_READY`), raised
    /// whenever the RX FIFO holds at least one byte. Route the source
    /// through the interrupt controller separately
    /// (`Lic::enable_aux_irq`) and
    /// unmask IRQ at the CPU ([`crate::irq::enable_irq`]) — all three
    /// gates have to be open, same as the other IRQ paths in this crate.
    ///
    /// There's no separate acknowledge step: the mini UART clears this
    /// interrupt on its own once the handler has drained the RX FIFO (via
    /// [`try_read_byte`](Self::try_read_byte)) below the trigger level,
    /// unlike PL011's write-1-to-clear `ICR`.
    pub fn enable_rx_irq(&self) {
        self.uart1.ier().modify(|_, w| w.data_ready().set_bit());
    }

    /// Masks the receive interrupt — the inverse of `enable_rx_irq`.
    pub fn disable_rx_irq(&self) {
        self.uart1.ier().modify(|_, w| w.data_ready().clear_bit());
    }
}

impl fmt::Write for MiniUart {
    /// Writes a string, translating `\n` to `\r\n` so plain terminals
    /// display it correctly (matches this crate's other UART output).
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for byte in s.bytes() {
            if byte == b'\n' {
                self.write_byte(b'\r');
            }
            self.write_byte(byte);
        }
        Ok(())
    }
}

impl embedded_io::ErrorType for MiniUart {
    /// `MiniUart`'s operations are infallible busy-waits — this is never
    /// actually constructed.
    type Error = core::convert::Infallible;
}

impl embedded_io::Read for MiniUart {
    /// Blocks until at least one byte is available, then reads as many
    /// more already-buffered bytes as fit without blocking further.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        buf[0] = self.read_byte();
        let mut n = 1;
        while n < buf.len() && self.byte_available() {
            buf[n] = self.read_byte();
            n += 1;
        }
        Ok(n)
    }
}

impl embedded_io::Write for MiniUart {
    /// Writes every byte in `buf`, blocking as needed; always succeeds
    /// and reports the full length written.
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for &byte in buf {
            self.write_byte(byte);
        }
        Ok(buf.len())
    }

    /// Blocks until the transmit shift register is idle (`LSR`'s
    /// `TX_IDLE`) — a real transmit barrier, not a no-op.
    fn flush(&mut self) -> Result<(), Self::Error> {
        while self.uart1.lsr().read().tx_idle().bit_is_clear() {}
        Ok(())
    }
}
