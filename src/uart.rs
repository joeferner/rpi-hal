use crate::pac::{GPIO, UART0};

#[cfg(feature = "async")]
mod asynch;
#[cfg(feature = "async")]
pub use asynch::on_irq;
use core::fmt;

/// Disables the pull resistor on the GPIO0-31 pins selected by
/// `pin_mask` (bank 0, whose bits map one-to-one to GPIO0-31 — the only
/// pins this driver muxes: GPIO14/15 for the console route, GPIO30-33
/// for the Bluetooth route).
///
/// The register access itself lives in [`crate::gpio`], which is where
/// the BCM2836/2837-vs-BCM2711 split is handled; this exists only to
/// name what the UART wants of it.
fn disable_pull(gpio: &GPIO, pin_mask: u32) {
    crate::gpio::set_pull_bank(gpio, 0, pin_mask, crate::gpio::Pull::None);
}

/// Blocking driver for UART0 (PL011).
pub struct Uart {
    uart0: UART0,
}

impl Uart {
    /// Wraps an already-initialized `UART0` without touching hardware —
    /// unlike `init`, this doesn't remux GPIO, reconfigure the baud
    /// rate, or drain the RX FIFO. Needed for contexts (like an IRQ
    /// handler) that only have a freshly-stolen `UART0` token, not the
    /// original `Uart` instance `init` returned; mirrors `Timer::new`/
    /// `Lic::new`'s existing shape.
    pub fn from_initialized(uart0: UART0) -> Self {
        Self { uart0 }
    }

    /// Routes GPIO14/15 to UART0 and brings it up at 115200 8N1.
    ///
    /// Assumes a 48MHz UART reference clock. Confirmed empirically on
    /// this hardware by measuring a transmitted bit period on a scope
    /// (~0.52-0.54µs, matching 48MHz/(16*divisor) — not the 3MHz often
    /// quoted in older bare-metal Pi tutorials, which is 16x too slow
    /// and would garble everything at the intended 115200 baud). If
    /// this ever needs to be robust across firmware versions, query
    /// the real clock via the VideoCore mailbox instead of assuming it.
    pub fn init(gpio: &GPIO, uart0: UART0) -> Self {
        const GPIO14_15_MASK: u32 = (1 << 14) | (1 << 15);
        disable_pull(gpio, GPIO14_15_MASK);

        gpio.gpfsel1()
            .modify(|_, w| w.fsel14().txd0().fsel15().rxd0());

        uart0.cr().write(|w| w.uarten().clear_bit());

        // 115200 baud @ 48MHz UART clock: divisor = 48_000_000 / (16 * 115200) = 26.04.
        unsafe {
            uart0.ibrd().write(|w| w.bauddivint().bits(26));
            uart0.fbrd().write(|w| w.bauddivfrac().bits(3));
        }

        // 8 bits, FIFOs enabled, no parity, one stop bit.
        uart0
            .lcr_h()
            .write(|w| unsafe { w.wlen().bits(0b11) }.fen().set_bit());

        uart0
            .cr()
            .write(|w| w.uarten().set_bit().txe().set_bit().rxe().set_bit());

        // Drain any stray byte(s) that may already be sitting in the RX
        // FIFO — e.g. a transient latched in while GPIO14/15 were still
        // being muxed to the UART's ALT function with an external cable
        // already driving the line. Callers should be able to assume a
        // clean slate rather than needing to know about this themselves.
        //
        // Bounded at the FIFO's own depth (16 bytes) rather than looped
        // "until empty" — if RX is attached to a genuinely continuous
        // stream (not just a boot-time transient), "until empty" would
        // never terminate and hang init forever. This drains at most
        // one FIFO's worth and moves on regardless.
        for _ in 0..16 {
            if uart0.fr().read().rxfe().bit_is_set() {
                break;
            }
            let _ = uart0.dr().read().data().bits();
        }

        Self { uart0 }
    }

    /// Brings UART0 up on the on-board Bluetooth controller's HCI route:
    /// GPIO30-33 (ALT3) with hardware RTS/CTS flow control enabled, at
    /// 115200 8N1.
    ///
    /// This is the *same* PL011 [`init`](Self::init) drives, but on the
    /// pins wired internally to the BCM43438's HCI UART (GPIO32/33 =
    /// TXD0/RXD0, GPIO30/31 = CTS0/RTS0) rather than the GPIO14/15 header
    /// route. A Pi can only route the one PL011 to one of the two at a
    /// time, so committing it to Bluetooth means the debug console has to
    /// move to the mini UART ([`crate::mini_uart`]) on GPIO14/15.
    ///
    /// Unlike [`init`](Self::init), flow control (`CTSEN`/`RTSEN`) is
    /// enabled: the Broadcom controller asserts CTS to throttle the host
    /// during firmware download and at the higher HCI baud rates, and
    /// drops bytes without it. 115200 is the rate the controller boots at
    /// before any firmware is loaded; raise it with
    /// [`set_baud`](Self::set_baud) only after the controller has been
    /// told to change rate (an HCI vendor command), never before.
    pub fn init_bluetooth(gpio: &GPIO, uart0: UART0) -> Self {
        // GPPUDCLK0 reaches only GPIO0-31, which covers the two
        // flow-control lines GPIO30 (CTS0) and GPIO31 (RTS0). GPIO32/33
        // (TXD0/RXD0) live in GPPUDCLK1's range, but a UART data line
        // doesn't need its pull cleared to work — the flow-control inputs
        // are what matter — so this clears the pulls only on the pins
        // GPPUDCLK0 can reach.
        const BT_FLOW_MASK: u32 = (1 << 30) | (1 << 31);
        disable_pull(gpio, BT_FLOW_MASK);

        gpio.gpfsel3().modify(|_, w| {
            w.fsel30()
                .cts0()
                .fsel31()
                .rts0()
                .fsel32()
                .txd0()
                .fsel33()
                .rxd0()
        });

        uart0.cr().write(|w| w.uarten().clear_bit());

        // 115200 baud @ 48MHz UART clock, same divisor as `init`.
        unsafe {
            uart0.ibrd().write(|w| w.bauddivint().bits(26));
            uart0.fbrd().write(|w| w.bauddivfrac().bits(3));
        }

        // 8 bits, FIFOs enabled, no parity, one stop bit.
        uart0
            .lcr_h()
            .write(|w| unsafe { w.wlen().bits(0b11) }.fen().set_bit());

        // Set the RX FIFO trigger level to 1/8 (2 of 16 bytes). With
        // auto-RTS (`RTSEN` below), the PL011 de-asserts nRTS — telling the
        // controller to stop — when the RX FIFO fills to this level, so a
        // lower level leaves more headroom to absorb the bytes already in
        // the controller's transmit pipeline before it stops. At the reset
        // default of 1/2 (8 bytes) only 8 bytes of headroom remain, which
        // at 3MHz HCI baud (~26µs) the controller occasionally overshoots
        // if the host stalls mid-stream, overrunning the FIFO and
        // corrupting a frame; 1/8 leaves 14 bytes and eliminated the
        // overruns in a deliberate-stall test. RX interrupts aren't used on
        // this path (the Bluetooth driver polls), so the trigger level only
        // affects the auto-RTS de-assert point.
        //
        // SAFETY: `RXIFLSEL` is a 3-bit field; `0b000` selects 1/8.
        uart0
            .ifls()
            .modify(|_, w| unsafe { w.rxiflsel().bits(0b000) });

        // Enable the UART with TX, RX, and both flow-control lines.
        uart0.cr().write(|w| {
            w.uarten()
                .set_bit()
                .txe()
                .set_bit()
                .rxe()
                .set_bit()
                .ctsen()
                .set_bit()
                .rtsen()
                .set_bit()
        });

        // Drain any stray RX bytes, bounded at the FIFO depth — same
        // rationale as `init`.
        for _ in 0..16 {
            if uart0.fr().read().rxfe().bit_is_set() {
                break;
            }
            let _ = uart0.dr().read().data().bits();
        }

        Self { uart0 }
    }

    /// Blocks until the transmit FIFO has room, then writes one byte.
    pub fn write_byte(&mut self, byte: u8) {
        while self.uart0.fr().read().txff().bit_is_set() {}
        unsafe {
            self.uart0.dr().write(|w| w.data().bits(byte));
        }
    }

    /// Blocks until the transmitter is fully idle: both the TX FIFO
    /// drained *and* the last bit shifted out onto the line.
    ///
    /// `write_byte` only waits for FIFO room (`TXFF`), which says
    /// nothing about bytes still in the FIFO or the one currently in
    /// the shift register. The PL011's `BUSY` bit stays set until the
    /// final stop bit has left the shifter, so this waits on that.
    /// Needed before anything that would disturb an in-flight byte —
    /// notably reprogramming the baud divisor (`set_baud`): change it
    /// while the last byte is still shifting and its tail is sent at the
    /// new rate, garbling it for the receiver.
    pub fn flush(&mut self) {
        while self.uart0.fr().read().busy().bit_is_set() {}
    }

    /// Reprograms the baud rate on the fly, returning `false` (leaving
    /// the current rate untouched) if `baud` can't be represented.
    ///
    /// Uses the same 48MHz reference clock `init` assumes. The PL011
    /// divisor is `48_000_000 / (16 * baud)`, split into a 16-bit
    /// integer part (`IBRD`) and a 6-bit fractional part (`FBRD`,
    /// sixty-fourths); this computes both as `192_000_000 / baud` in
    /// 64ths, rounded. A `baud` so high that the integer part rounds to
    /// 0, or so low it overflows `IBRD`, is rejected — hence the bool.
    /// For example 1_500_000 baud gives an exact divisor of 2.0
    /// (`IBRD = 2`, `FBRD = 0`).
    ///
    /// Caller must `flush` first if a byte the other end still needs to
    /// read cleanly (e.g. an acknowledgement) is in flight — this
    /// disables the UART briefly to change the divisor, which would
    /// otherwise corrupt it. The FIFOs and 8N1 framing from `init` are
    /// preserved: the `LCR_H` write here re-latches the new divisor
    /// (required on the PL011) without changing those.
    #[must_use]
    pub fn set_baud(&mut self, baud: u32) -> bool {
        if baud == 0 {
            return false;
        }
        let div_x64 = (192_000_000 + baud / 2) / baud;
        let ibrd = div_x64 / 64;
        let fbrd = (div_x64 % 64) as u8;
        if ibrd == 0 || ibrd > u16::MAX as u32 {
            return false;
        }

        self.uart0.cr().modify(|_, w| w.uarten().clear_bit());
        unsafe {
            self.uart0
                .ibrd()
                .write(|w| w.bauddivint().bits(ibrd as u16));
            self.uart0.fbrd().write(|w| w.bauddivfrac().bits(fbrd));
        }
        // Rewriting LCR_H is what latches the new IBRD/FBRD on the
        // PL011; keep the same 8N1 + FIFOs-enabled config init set up.
        self.uart0
            .lcr_h()
            .write(|w| unsafe { w.wlen().bits(0b11) }.fen().set_bit());
        self.uart0.cr().modify(|_, w| w.uarten().set_bit());
        true
    }

    /// Blocks until a byte is available, then reads and returns it.
    pub fn read_byte(&mut self) -> u8 {
        while self.uart0.fr().read().rxfe().bit_is_set() {}
        self.uart0.dr().read().data().bits()
    }

    /// Non-blocking: true if a byte is waiting in the receive FIFO.
    pub fn byte_available(&self) -> bool {
        self.uart0.fr().read().rxfe().bit_is_clear()
    }

    /// Non-blocking sibling of `read_byte`: `None` if the receive FIFO
    /// is empty, otherwise reads and returns one byte. No buffering —
    /// callers that need to hold onto bytes across calls (e.g. an IRQ
    /// handler feeding a queue) own that themselves.
    pub fn try_read_byte(&self) -> Option<u8> {
        if self.uart0.fr().read().rxfe().bit_is_set() {
            return None;
        }
        Some(self.uart0.dr().read().data().bits())
    }

    /// Unmasks both RX-FIFO-level and RX-timeout interrupts (`IMSC`'s
    /// `RXIM`/`RTIM`) — both, not just one, so a byte or two trickling
    /// in below the FIFO's trigger level isn't left stuck waiting for
    /// more to arrive. Route the source through the interrupt
    /// controller separately (`Lic::enable_uart_irq`) and unmask IRQ at
    /// the CPU (`crate::irq::enable_irq`) — all three gates have to be
    /// open, same as the timer IRQ path.
    pub fn enable_rx_irq(&self) {
        self.uart0
            .imsc()
            .modify(|_, w| w.rxim().set_bit().rtim().set_bit());
    }

    /// Masks both RX-FIFO-level and RX-timeout interrupts — the inverse
    /// of `enable_rx_irq`.
    pub fn disable_rx_irq(&self) {
        self.uart0
            .imsc()
            .modify(|_, w| w.rxim().clear_bit().rtim().clear_bit());
    }

    /// Acknowledges both the RX-FIFO-level and RX-timeout interrupts at
    /// the peripheral level. Call this from your IRQ handler after
    /// draining whatever `try_read_byte` currently has to offer.
    pub fn clear_rx_irq(&self) {
        self.uart0
            .icr()
            .write(|w| w.rxic().set_bit().rtic().set_bit());
    }

    /// True if the transmit FIFO has no room for another byte (`FR`'s
    /// `TXFF`) — what [`write_byte`](Self::write_byte) busy-waits on,
    /// exposed so a caller can decide for itself rather than block.
    pub fn tx_full(&self) -> bool {
        self.uart0.fr().read().txff().bit_is_set()
    }

    /// Queues `byte` if the transmit FIFO has room, reporting whether it
    /// did. The non-blocking counterpart to
    /// [`write_byte`](Self::write_byte), mirroring
    /// [`try_read_byte`](Self::try_read_byte).
    pub fn try_write_byte(&mut self, byte: u8) -> bool {
        if self.tx_full() {
            return false;
        }
        unsafe {
            self.uart0.dr().write(|w| w.data().bits(byte));
        }
        true
    }

    /// Busy-waits until the transmitter is completely idle — FIFO drained
    /// *and* shift register empty (`FR`'s `BUSY` clear), so the last bit
    /// has left the pin.
    ///
    /// A real barrier, not a FIFO-space check: needed before changing
    /// baud rate or handing the pins to another function, where a byte
    /// still in flight would be corrupted.
    pub fn wait_tx_idle(&self) {
        while self.uart0.fr().read().busy().bit_is_set() {}
    }

    /// Unmasks the transmit interrupt (`IMSC`'s `TXIM`), which asserts
    /// while the transmit FIFO sits at or below its trigger level — that
    /// is, while there is room to write.
    ///
    /// Being level-driven rather than edge-driven has a consequence worth
    /// knowing: with the FIFO already drained, this asserts immediately
    /// and keeps asserting, so a handler must mask it (or refill the
    /// FIFO past the trigger) rather than merely acknowledge it. Enabling
    /// it only while genuinely waiting for room is the simplest way to
    /// stay out of that trap.
    pub fn enable_tx_irq(&self) {
        self.uart0.imsc().modify(|_, w| w.txim().set_bit());
    }

    /// Masks the transmit interrupt — the inverse of
    /// [`enable_tx_irq`](Self::enable_tx_irq).
    pub fn disable_tx_irq(&self) {
        self.uart0.imsc().modify(|_, w| w.txim().clear_bit());
    }

    /// Acknowledges the transmit interrupt at the peripheral level.
    ///
    /// On its own this does not stop the interrupt re-asserting: the
    /// condition is "there is room in the FIFO", which stays true until
    /// something fills it. See [`enable_tx_irq`](Self::enable_tx_irq).
    pub fn clear_tx_irq(&self) {
        self.uart0.icr().write(|w| w.txic().set_bit());
    }
}

impl fmt::Write for Uart {
    /// Writes a string, translating `\n` to `\r\n` so plain terminals
    /// display it correctly (matches this project's other UART output).
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

impl embedded_io::ErrorType for Uart {
    /// `Uart`'s operations are infallible busy-waits — this is never
    /// actually constructed.
    type Error = core::convert::Infallible;
}

impl embedded_io::Read for Uart {
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

impl embedded_io::Write for Uart {
    /// Writes every byte in `buf`, blocking as needed; always succeeds
    /// and reports the full length written.
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for &byte in buf {
            self.write_byte(byte);
        }
        Ok(buf.len())
    }

    /// Blocks until the transmit shift register is empty (FR's `BUSY`
    /// bit clear) — a real transmit barrier, not a no-op.
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.wait_tx_idle();
        Ok(())
    }
}
