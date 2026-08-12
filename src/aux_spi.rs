//! Blocking driver for the auxiliary SPI controllers SPI1 and SPI2 —
//! two of the three sub-peripherals behind the AUX block (alongside the
//! mini UART, [`crate::mini_uart`]).
//!
//! Distinct from SPI0 ([`crate::spi`]): the aux SPI is Broadcom's
//! "Universal SPI Master", a different core with a different register
//! layout. It gives two more SPI buses without giving up SPI0, at the
//! cost of shallower FIFOs and a reference clock tied to the VPU/core
//! clock (see
//! [`AuxSpi::init_spi1`](crate::aux_spi::AuxSpi::init_spi1)'s `speed` note).
//!
//! Generic over the instance: [`AuxSpi<SPI1>`](crate::aux_spi::AuxSpi) drives
//! SPI1 on GPIO16-21 (ALT4), which are broken out on the 40-pin header;
//! [`AuxSpi<SPI2>`](crate::aux_spi::AuxSpi) drives SPI2 on GPIO40-45 (ALT4),
//! which are *not* broken out on a Pi 2/3 board (they collide with the analog
//! PWM audio and the camera/display BSC0 bus) — SPI2 is provided for
//! completeness and custom-board use, not as a usable header bus. The
//! register layout is identical across instances (both deref to the same
//! PAC register block); only the pin mux and the AUX_ENABLES bit differ,
//! so the transfer logic is shared and each instance gets its own `init`.
//!
//! **CPHA=1 is not supported.** Broadcom's aux SPI presents the first
//! data bit the moment CS asserts (CPHA=0 timing) and cannot delay it to
//! the first clock edge, so it can only drive the two **CPHA=0** modes (0
//! and 2). A CPHA=1 request (mode 1 or 3) is applied faithfully but comes
//! out shifted by one bit on the wire — confirmed on hardware. Use modes 0
//! and 2 only; see [`AuxSpi::init_spi1`](crate::aux_spi::AuxSpi::init_spi1).
//!
//! One `bcm2837-lpa` bug is worked around here: the PAC places the
//! `IO`/`TXHOLD` FIFO registers at offsets `0x10`/`0x20`, but the real
//! hardware (and Linux's `spi-bcm2835aux`) has them at `0x20`/`0x30`.
//! Driving the PAC's `io()` accessor wrote a reserved word and enqueued
//! nothing, so no transfer ran at all. The FIFO is therefore accessed at
//! raw offsets from the register-block base (see `IO_OFFSET`); the other
//! registers, whose PAC offsets are correct, still go through the PAC.

use core::ops::Deref;

use crate::pac::{spi1, AUX, GPIO, SPI1, SPI2};
use embedded_hal::spi::{Phase, Polarity};

/// The number of bits the aux SPI shifts per FIFO word in this driver.
///
/// The driver runs the controller in *variable-width* mode, matching
/// Linux's `spi-bcm2835aux`: each FIFO word carries both the data and the
/// shift length (at [`SHIFT_LENGTH_POS`] upward). The transmit shift
/// register is 24 bits wide and shifts **out from bit 23** (the MS bit of
/// the 24-bit data field) for `length` clocks, so the outgoing data must
/// be *left-justified* — its MSB at bit 23. For a length of 8 that puts
/// the byte in bits `[23:16]` (see [`DATA_POS`]). The received bits shift
/// in at the bottom, so the response comes back *right-justified* in
/// `[7:0]` — an asymmetry the transmit/receive halves each account for.
///
/// Placing the byte in the low bits instead (as fixed-width mode and a
/// first cut of the variable-width path both did) clocked correctly but
/// drove MOSI low the whole time: the shifter took its MS bits from the
/// all-zero top of the word.
const SHIFT_LENGTH_BITS: u8 = 8;

/// Bit position of the shift-length field within a variable-width FIFO
/// word (`DATA[28:24]`), per the BCM2835 aux-SPI FIFO format.
const SHIFT_LENGTH_POS: u32 = 24;

/// Bit position the transmit byte is left-justified to within a
/// variable-width FIFO word, so its MSB lands at bit 23 (the MS bit of the
/// 24-bit data field the shifter reads from). For an 8-bit word that's
/// bit 16, i.e. the byte occupies `[23:16]`. See [`SHIFT_LENGTH_BITS`].
const DATA_POS: u32 = SHIFT_LENGTH_POS - SHIFT_LENGTH_BITS as u32;

/// Byte offset of the aux-SPI `IO` FIFO register from the peripheral base.
///
/// This works around a `bcm2837-lpa` bug: the PAC models the `IO` and
/// `TXHOLD` FIFO registers at offsets `0x10`/`0x20`, but on real BCM2835
/// hardware — and in Linux's `spi-bcm2835aux` — they live at `0x20`/`0x30`
/// (the `CNTL0`/`CNTL1`/`STAT`/`PEEK` registers at `0x00`..`0x0c` are
/// correct, so only the FIFO pair is off). Going through the PAC's `io()`
/// accessor hit a reserved word at `0x10`, so writes silently enqueued
/// nothing and no transfer ever ran — the whole reason a first cut of this
/// driver hung. So the FIFO is accessed at these raw offsets from the
/// register-block base instead of via the (mis-placed) PAC fields.
///
/// Writing `IO` deasserts CS at the end of the word; reading it pops the
/// RX FIFO.
const IO_OFFSET: usize = 0x20;

/// Byte offset of the aux-SPI `TXHOLD` FIFO register from the peripheral
/// base — like [`IO_OFFSET`], relocated to the hardware-correct address
/// the PAC gets wrong. Writing `TXHOLD` enqueues a word that *keeps* CS
/// asserted afterward, for all but the last byte of a multi-byte transfer.
const TXHOLD_OFFSET: usize = 0x30;

/// Which chip-select line the aux SPI drives automatically around every
/// `SpiBus` call.
///
/// Unlike SPI0's two CE lines, the aux SPI has three (CE0/CE1/CE2), all
/// broken out for SPI1 (GPIO18/17/16). The controller drives them as an
/// active-low pattern; the encodings here are that pattern with the
/// selected line pulled low.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChipSelect {
    /// CE0 — SPI1 GPIO18 / SPI2 GPIO43.
    Cs0,
    /// CE1 — SPI1 GPIO17 / SPI2 GPIO44.
    Cs1,
    /// CE2 — SPI1 GPIO16 / SPI2 GPIO45.
    Cs2,
    /// No hardware CE line is asserted. The transfer still runs, but no
    /// physical CE moves — use this when chip select is managed
    /// externally instead (e.g. wrapping `AuxSpi` in `embedded-hal-bus`'s
    /// `ExclusiveDevice` with a plain [`crate::gpio::Pin`], or toggling
    /// one by hand), same role as [`crate::spi::ChipSelect::None`]. The
    /// matching CE pin is left unmuxed so the caller can use it as GPIO.
    None,
}

impl ChipSelect {
    /// The 3-bit active-low pattern written to `CNTL0.CHIP_SELECTS`: the
    /// selected line low, the rest high (`None` leaves all three high, so
    /// nothing is asserted).
    fn pattern(self) -> u8 {
        match self {
            ChipSelect::Cs0 => 0b110,
            ChipSelect::Cs1 => 0b101,
            ChipSelect::Cs2 => 0b011,
            ChipSelect::None => 0b111,
        }
    }
}

/// Blocking driver for an aux SPI controller, generic over the instance
/// `S`. Construct with [`AuxSpi::init_spi1`] for SPI1 on GPIO16-21, or
/// [`AuxSpi::init_spi2`] for SPI2 on GPIO40-45.
///
/// Implements `embedded_hal::spi::SpiBus` (like [`crate::spi::Spi`], not
/// `SpiDevice`): with a hardware `ChipSelect` the controller owns and
/// drives one CE line for the duration of each bus operation; with
/// [`ChipSelect::None`] it leaves the CE lines alone and CS becomes the
/// caller's responsibility.
///
/// **Only SPI modes 0 and 2 (CPHA=0) work correctly.** The aux SPI cannot
/// generate CPHA=1 waveforms, so modes 1 and 3 come out shifted one bit on
/// MOSI — a hardware limitation, verified on a logic analyzer, described in
/// this module's documentation.
pub struct AuxSpi<S> {
    spi: S,
}

impl AuxSpi<SPI1> {
    /// Enables SPI1 in the shared AUX block, routes its pins on GPIO16-21
    /// (ALT4), and configures clock polarity/phase, the clock divider,
    /// and the chip select.
    ///
    /// Always muxes GPIO19/20/21 (MISO/MOSI/SCLK); the CE pin for the
    /// chosen `chip_select` (GPIO18/17/16 for CS0/CS1/CS2) is muxed too,
    /// while [`ChipSelect::None`] leaves all three CE pins as GPIO for the
    /// caller — matching [`crate::spi::Spi::init`]'s handling of SPI0's
    /// CE lines.
    ///
    /// `aux` is taken by reference, not consumed, because the AUX
    /// `ENABLES` register is shared with the mini UART and SPI2: this
    /// sets only the SPI1 bit (via `modify`), so the caller keeps `AUX`
    /// to lend to those as well. Enabling SPI1 here is also what makes
    /// its registers respond at all.
    ///
    /// `speed` is passed straight through to `CNTL0.SPEED` (the SPI clock
    /// is `core_clock / (2 * (speed + 1))`) rather than computed from a
    /// requested frequency, for the same reason as
    /// [`crate::spi::Spi::init`]'s `clock_divider`: the core clock isn't
    /// a fixed, firmware-guaranteed value, so a divider computed against
    /// an assumed clock would silently be wrong on a differently-clocked
    /// board. Query the real core clock via the VideoCore mailbox and
    /// compute `speed` from that if an exact frequency is needed.
    pub fn init_spi1(
        gpio: &GPIO,
        aux: &AUX,
        spi1: SPI1,
        mode: embedded_hal::spi::Mode,
        chip_select: ChipSelect,
        speed: u16,
    ) -> Self {
        aux.enables().modify(|_, w| w.spi_1().set_bit());

        gpio.gpfsel1().modify(|_, w| w.fsel19().spi1_miso());
        gpio.gpfsel2()
            .modify(|_, w| w.fsel20().spi1_mosi().fsel21().spi1_sclk());
        match chip_select {
            ChipSelect::Cs0 => gpio.gpfsel1().modify(|_, w| w.fsel18().spi1_ce0_n()),
            ChipSelect::Cs1 => gpio.gpfsel1().modify(|_, w| w.fsel17().spi1_ce1_n()),
            ChipSelect::Cs2 => gpio.gpfsel1().modify(|_, w| w.fsel16().spi1_ce2_n()),
            ChipSelect::None => {}
        }

        AuxSpi::configure(spi1, mode, chip_select, speed)
    }
}

impl AuxSpi<SPI2> {
    /// Enables SPI2 in the shared AUX block, routes its pins on GPIO40-45
    /// (ALT4), and configures the controller — the SPI2 counterpart of
    /// [`AuxSpi::init_spi1`], same parameters and same reasoning for
    /// `aux` being borrowed and `speed` being a raw divider.
    ///
    /// **SPI2's pins are not broken out on a Pi 2/3 board.** GPIO40-45
    /// route to the analog audio (GPIO40/41/45) and the camera/display
    /// BSC0 bus (GPIO44/45), not the 40-pin header, so this muxing takes
    /// those functions over and there's nowhere to attach a device on a
    /// stock board. Provided for completeness and custom hardware; the
    /// PAC doesn't even name these ALT4 functions (they're `reserved4`),
    /// so they're written as the raw ALT4 selection here.
    pub fn init_spi2(
        gpio: &GPIO,
        aux: &AUX,
        spi2: SPI2,
        mode: embedded_hal::spi::Mode,
        chip_select: ChipSelect,
        speed: u16,
    ) -> Self {
        aux.enables().modify(|_, w| w.spi_2().set_bit());

        // ALT4 on GPIO40-45. The PAC models these ALT4 functions only as
        // `reserved4` (the SVD didn't name SPI2), but `reserved4` *is* the
        // ALT4 encoding, which is the SPI2 routing on this SoC.
        gpio.gpfsel4().modify(|_, w| {
            w.fsel40().reserved4(); // SPI2_MISO
            w.fsel41().reserved4(); // SPI2_MOSI
            w.fsel42().reserved4() // SPI2_SCLK
        });
        match chip_select {
            ChipSelect::Cs0 => gpio.gpfsel4().modify(|_, w| w.fsel43().reserved4()),
            ChipSelect::Cs1 => gpio.gpfsel4().modify(|_, w| w.fsel44().reserved4()),
            ChipSelect::Cs2 => gpio.gpfsel4().modify(|_, w| w.fsel45().reserved4()),
            ChipSelect::None => {}
        }

        AuxSpi::configure(spi2, mode, chip_select, speed)
    }
}

impl<S: Deref<Target = spi1::RegisterBlock>> AuxSpi<S> {
    /// Shared tail of the per-instance `init`s: program `CNTL0`/`CNTL1`
    /// and leave the interface enabled with clean FIFOs, once the caller
    /// has done the instance's own AUX_ENABLES bit and pin mux.
    ///
    /// The aux SPI has no single CPHA bit; it exposes the two clock edges
    /// directly (`OUT_RISING`/`IN_RISING` — which edge data is shifted
    /// out on and sampled in on) plus `INVERT_CLK` for idle level. Mapped
    /// from the standard SPI mode: `INVERT_CLK = CPOL`, data is shifted
    /// out on one edge and sampled on the other, with the out edge being
    /// rising exactly when `CPOL XOR CPHA` (and the in edge the opposite).
    ///
    /// This is correct for the two **CPHA=0** modes (0 and 2) but **cannot
    /// produce a correct CPHA=1 waveform** (modes 1 and 3): the aux SPI
    /// presents the first data bit the instant CS asserts (CPHA=0 timing)
    /// and has no way to hold it until the first clock edge, which is what
    /// CPHA=1 needs. So a CPHA=1 transfer shifts MOSI out one bit early and
    /// the receiver loses the MSB — e.g. `0xB4` arrives as `0x68`. This was
    /// confirmed on hardware with a logic analyzer (modes 0/2 read back the
    /// sent byte, modes 1/3 read it shifted by one bit); `dout_hold_time`
    /// does not fix it. It's a documented limitation of Broadcom's aux SPI,
    /// not a mapping bug — see this type's `init` docs. The mode is still
    /// applied faithfully; a CPHA=1 request just won't work on this
    /// hardware, so callers should stick to modes 0 and 2.
    fn configure(
        spi: S,
        mode: embedded_hal::spi::Mode,
        chip_select: ChipSelect,
        speed: u16,
    ) -> Self {
        let cpol = mode.polarity == Polarity::IdleHigh;
        let cpha = mode.phase == Phase::CaptureOnSecondTransition;
        let out_rising = cpol ^ cpha;

        spi.cntl0().write(|w| {
            let w = w
                .enable()
                .set_bit()
                // Hold the FIFOs cleared for this write; released just
                // below so the first transfer starts empty.
                .clear_fifos()
                .set_bit()
                // Variable-width: take the shift length from each FIFO
                // word (`CNTL0.SHIFT_LENGTH` is ignored), so the data byte
                // is placed unambiguously — see `SHIFT_LENGTH_BITS`.
                .variable_width()
                .set_bit()
                .msb_first()
                .set_bit()
                .invert_clk()
                .bit(cpol)
                .out_rising()
                .bit(out_rising)
                .in_rising()
                .bit(!out_rising);
            unsafe {
                w.chip_selects()
                    .bits(chip_select.pattern())
                    .speed()
                    .bits(speed)
            }
        });
        // Release the FIFO-clear hold, leaving the rest of CNTL0 as set.
        spi.cntl0().modify(|_, w| w.clear_fifos().clear_bit());
        // Shift the received MSB in first, matching MSB_FIRST on the out
        // side; the rest of CNTL1 (flow/interrupt/CS-timing) stays at
        // reset.
        spi.cntl1().write(|w| w.msb_first().set_bit());

        Self { spi }
    }

    /// Raw pointer to the FIFO register at `offset` from the register
    /// block base. Used only for the `IO`/`TXHOLD` FIFO ports, which the
    /// PAC places at the wrong offsets — see [`IO_OFFSET`]. Every other
    /// register is reached through the (correctly-placed) PAC accessors.
    fn fifo_ptr(&self, offset: usize) -> *mut u32 {
        let base = core::ptr::from_ref(&*self.spi) as usize;
        (base + offset) as *mut u32
    }

    /// Enqueues one byte into the TX FIFO as a variable-width word: the
    /// byte left-justified at [`DATA_POS`] (MSB at bit 23) and
    /// [`SHIFT_LENGTH_BITS`] as the shift length at [`SHIFT_LENGTH_POS`].
    /// `hold_cs` selects `TXHOLD` (keeps CS asserted after the word)
    /// versus `IO` (deasserts it), which is how a multi-byte transfer
    /// holds CS across all its bytes and drops it only after the last.
    fn fifo_write(&self, byte: u8, hold_cs: bool) {
        let word = (byte as u32) << DATA_POS | (SHIFT_LENGTH_BITS as u32) << SHIFT_LENGTH_POS;
        let offset = if hold_cs { TXHOLD_OFFSET } else { IO_OFFSET };
        unsafe { core::ptr::write_volatile(self.fifo_ptr(offset), word) };
    }

    /// Pops one byte from the RX FIFO (reading `IO` consumes an entry).
    /// The received byte is right-justified in the low 8 bits.
    fn fifo_read(&self) -> u8 {
        (unsafe { core::ptr::read_volatile(self.fifo_ptr(IO_OFFSET)) } & 0xff) as u8
    }

    /// Full-duplex shift of `len` bytes: `tx(i)` supplies the byte to
    /// send for index `i`, `rx(i, byte)` receives the byte shifted in for
    /// index `i`. Every `SpiBus` method except `transfer_in_place` boils
    /// down to this, since the hardware always shifts a byte out and in
    /// together.
    ///
    /// Each byte is written as one variable-width FIFO word (see
    /// [`fifo_write`](Self::fifo_write)). Every byte but the last goes to
    /// `TXHOLD`, which keeps CS asserted after the word; the final byte
    /// goes to `IO`, which deasserts CS — so a multi-byte transfer holds
    /// CS for its whole duration instead of pulsing it per byte. The
    /// received byte comes back right-justified in `IO[7:0]`.
    fn shift(
        &mut self,
        len: usize,
        mut tx: impl FnMut(usize) -> u8,
        mut rx: impl FnMut(usize, u8),
    ) {
        let mut sent = 0;
        let mut received = 0;
        while received < len {
            if sent < len && self.spi.stat().read().tx_full().bit_is_clear() {
                let is_last = sent == len - 1;
                self.fifo_write(tx(sent), !is_last);
                sent += 1;
            }
            if self.spi.stat().read().rx_empty().bit_is_clear() {
                let byte = self.fifo_read();
                rx(received, byte);
                received += 1;
            }
        }
    }
}

impl<S: Deref<Target = spi1::RegisterBlock>> embedded_hal::spi::ErrorType for AuxSpi<S> {
    /// Infallible — every operation here is a direct busy-wait on
    /// hardware FIFO flags, no failure path exists.
    type Error = core::convert::Infallible;
}

impl<S: Deref<Target = spi1::RegisterBlock>> embedded_hal::spi::SpiBus<u8> for AuxSpi<S> {
    /// Reads `words.len()` bytes, shifting out `0x00` for each.
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.shift(words.len(), |_| 0, |i, byte| words[i] = byte);
        Ok(())
    }

    /// Writes every byte in `words`, discarding whatever shifts in.
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.shift(words.len(), |i| words[i], |_, _| {});
        Ok(())
    }

    /// Shifts `max(read.len(), write.len())` bytes: indices beyond
    /// `write`'s end send `0x00`; indices beyond `read`'s end discard the
    /// response.
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let len = read.len().max(write.len());
        self.shift(
            len,
            |i| write.get(i).copied().unwrap_or(0),
            |i, byte| {
                if let Some(slot) = read.get_mut(i) {
                    *slot = byte;
                }
            },
        );
        Ok(())
    }

    /// Shifts `words` out and overwrites it in place with the response.
    ///
    /// Not implemented via `shift` for the same reason as
    /// [`crate::spi::Spi`]'s: `shift`'s two closures would both need to
    /// capture `words`, which the borrow checker rejects even though a
    /// given index is always read before it's overwritten.
    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let len = words.len();
        let mut sent = 0;
        let mut received = 0;
        while received < len {
            if sent < len && self.spi.stat().read().tx_full().bit_is_clear() {
                let is_last = sent == len - 1;
                self.fifo_write(words[sent], !is_last);
                sent += 1;
            }
            if self.spi.stat().read().rx_empty().bit_is_clear() {
                words[received] = self.fifo_read();
                received += 1;
            }
        }
        Ok(())
    }

    /// This driver never buffers beyond the hardware FIFOs and every
    /// other method drains the RX FIFO to `len` before returning, so
    /// there's nothing left in flight by the time `flush` could be
    /// called — a no-op.
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
