//! Blocking driver for the BCM2835 BSC I2C controllers.
//!
//! Generic over the BSC instance: [`I2c<BSC1>`](crate::i2c::I2c) drives I2C1 on
//! GPIO2 (SDA1)/GPIO3 (SCL1), the general-purpose bus on the 40-pin header,
//! and [`I2c<BSC0>`](crate::i2c::I2c) drives BSC0 on GPIO44 (SDA0)/GPIO45
//! (SCL0), the routing the camera/display connectors use on a Pi 3. The
//! register layout is identical across instances (both deref to the same PAC
//! register block); only the pin mux and the peripheral token differ, so
//! the transfer logic is shared and each instance gets its own `init`.

use core::ops::Deref;

use crate::pac::{bsc0, BSC0, BSC1, GPIO};

/// Errors surfaced by [`I2c`]'s `embedded_hal::i2c::I2c` methods.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The addressed slave didn't acknowledge (`S.ERR`) — typically
    /// means nothing is listening at that address, or it NAK'd a data
    /// byte.
    NoAcknowledge,
    /// A zero-length read/write was requested. Confirmed on real
    /// hardware (not assumed): BCM2835's BSC doesn't drive a real bus
    /// transaction at all for `DLEN=0` -- it reports `DONE` with no
    /// error immediately, for *every* address, whether or not
    /// anything is actually listening. A first version of this driver
    /// treated `DLEN=0` as a valid "probe this address" primitive
    /// (it's honestly how the BCM2835 register spec reads); wiring
    /// that up as `i2c_scan.rs`'s bus-scan technique reported all 58
    /// addresses in the scan range as present, which is what exposed
    /// this. Refusing zero-length operations here rather than
    /// silently reporting a false `Ok(())` -- see `i2c_scan.rs` for
    /// the real (1-byte) probe technique instead.
    ZeroLengthUnsupported,
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            // `S.ERR` doesn't distinguish an address NAK from a data
            // NAK -- reported as `NoAcknowledgeSource::Unknown` rather
            // than guessing which.
            Self::NoAcknowledge => embedded_hal::i2c::ErrorKind::NoAcknowledge(
                embedded_hal::i2c::NoAcknowledgeSource::Unknown,
            ),
            Self::ZeroLengthUnsupported => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

/// Blocking driver for a BCM2835 BSC I2C controller, generic over the
/// instance `I` (defaulting to [`BSC1`] so `I2c` alone means the
/// general-purpose header bus). Construct with [`I2c::<BSC1>::init`] for
/// I2C1 on GPIO2/3, or [`I2c::<BSC0>::init`] for BSC0 on GPIO44/45.
///
/// A note on BSC0: it can be pin-muxed to two different routings —
/// GPIO0/1 (reserved for HAT EEPROM ID detection) and GPIO44/45 (the
/// camera/display connector bus on a Pi 3). This driver only ever drives
/// the GPIO44/45 routing; it never touches GPIO0/1. BSC0 is also nominally
/// owned by the VideoCore firmware, which arbitrates the camera/display
/// bus — so a program using this should have taken the machine over rather
/// than leaving the firmware's camera stack running to race on the bus.
///
/// Implements `embedded_hal::i2c::I2c` via its single required
/// `transaction` method — every [`embedded_hal::i2c::Operation`] in a
/// transaction gets its own complete START...STOP cycle. **This is not
/// a true repeated start**: BCM2835's hardware does support chaining
/// a write directly into a read with a repeated start (by rewriting
/// `C.READ`/`DLEN` while the first transfer is still in flight), but
/// that "combined transactions" technique is exactly the kind of
/// undocumented, erratum-prone hardware trick this crate has hit
/// real, silent-corruption-shaped bugs from before (see `spi.rs`'s
/// `REN` writeup) — implementing it without real hardware to verify
/// against would be guessing, not confirming. If a driver you're
/// writing against this needs a genuine repeated start (some devices
/// reset their internal register pointer on a STOP between the
/// address-write and the read), this isn't there yet.
pub struct I2c<I = BSC1> {
    bsc: I,
}

impl I2c<BSC1> {
    /// Routes GPIO2/3 to BSC1 (ALT0: SDA1, SCL1), sets the clock
    /// divider, and enables the peripheral (idle, no transfer yet).
    ///
    /// `clock_divider` is passed straight through to `DIV.CDIV` rather
    /// than computed from a target frequency, for the same reason as
    /// `spi::Spi::init`'s `clock_divider`: the core clock isn't a
    /// fixed, firmware-guaranteed value the way UART0's reference
    /// clock is (see `uart.rs`'s `init`) — a divider computed against
    /// an assumed core clock would silently be wrong on a board
    /// configured differently. The reset default (`0x5dc` = 1500)
    /// gives 100kHz standard mode at BCM2835's typical 150MHz core
    /// clock; halve it for 400kHz fast mode at that same clock, or
    /// compute against the real core clock (queried via the
    /// VideoCore mailbox) if precision matters.
    pub fn init(gpio: &GPIO, bsc1: BSC1, clock_divider: u16) -> Self {
        gpio.gpfsel0()
            .modify(|_, w| w.fsel2().sda1().fsel3().scl1());
        Self::configure(bsc1, clock_divider)
    }
}

impl I2c<BSC0> {
    /// Routes GPIO44/45 to BSC0 (ALT1: SDA0, SCL0) — the camera/display
    /// connector bus on a Pi 3, *not* BSC0's GPIO0/1 HAT-EEPROM routing —
    /// sets the clock divider, and enables the peripheral.
    ///
    /// See [`I2c::<BSC1>::init`] on why `clock_divider` is a raw `CDIV`
    /// value rather than a target frequency; the same reasoning applies.
    /// See this type's doc comment on BSC0 being firmware-arbitrated.
    pub fn init(gpio: &GPIO, bsc0: BSC0, clock_divider: u16) -> Self {
        gpio.gpfsel4()
            .modify(|_, w| w.fsel44().sda0().fsel45().scl0());
        Self::configure(bsc0, clock_divider)
    }
}

impl<I: Deref<Target = bsc0::RegisterBlock>> I2c<I> {
    /// Shared tail of the per-instance `init`s: program the divider and
    /// enable the controller, once the caller has done the instance's own
    /// pin mux.
    fn configure(bsc: I, clock_divider: u16) -> Self {
        unsafe {
            bsc.div().write(|w| w.cdiv().bits(clock_divider));
        }
        bsc.c().write(|w| w.i2cen().bit(true));
        Self { bsc }
    }

    /// Clears `DONE`/`ERR` (both write-1-to-clear) ahead of a new
    /// transfer — same "clean baseline before starting" approach as
    /// `spi::Spi::reset_hw`, just scoped to what the BSC actually needs
    /// between transfers (its FIFOs are cleared via `C.CLEAR` in
    /// [`Self::one_shot`] instead, since that's a `C`-register field,
    /// not `S`).
    fn clear_status(&self) {
        self.bsc.s().write(|w| w.done().bit(true).err().bit(true));
    }

    /// One complete BSC transfer: START, `address`, then either
    /// shifting `write` out or filling `read` in (never both — see
    /// this module's doc comment on why a single call here can't
    /// combine them into one repeated-start transaction), then STOP.
    /// `len` must be at least 1 — see [`Error::ZeroLengthUnsupported`]'s
    /// doc comment for why `DLEN=0` is refused rather than attempted;
    /// callers ([`Self::write_one`]/[`Self::read_one`]) check this
    /// before calling in.
    fn one_shot(&mut self, address: u8, is_read: bool, len: usize) {
        self.bsc.a().write(|w| unsafe { w.addr().bits(address) });
        self.clear_status();
        unsafe {
            self.bsc.dlen().write(|w| w.dlen().bits(len as u16));
        }
        self.bsc.c().write(|w| {
            unsafe { w.clear().bits(0b11) };
            w.i2cen().bit(true).read().bit(is_read).st().set_bit()
        });
    }

    /// Writes `bytes` as one complete transaction (START, address,
    /// `bytes`, STOP), feeding the TX FIFO as `TXD` (space available)
    /// allows and bailing out early with [`Error::NoAcknowledge`] the
    /// instant `S.ERR` is seen.
    fn write_one(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error> {
        if bytes.is_empty() {
            return Err(Error::ZeroLengthUnsupported);
        }
        self.one_shot(address, false, bytes.len());

        let mut sent = 0;
        loop {
            let status = self.bsc.s().read();
            if status.err().bit_is_set() {
                self.clear_status();
                return Err(Error::NoAcknowledge);
            }
            if sent < bytes.len() && status.txd().bit_is_set() {
                unsafe {
                    self.bsc.fifo().write(|w| w.data().bits(bytes[sent]));
                }
                sent += 1;
            }
            if status.done().bit_is_set() {
                break;
            }
        }

        self.clear_status();
        Ok(())
    }

    /// Reads `buffer.len()` bytes as one complete transaction (START,
    /// address, `buffer.len()` bytes, STOP), draining the RX FIFO as
    /// `RXD` (data available) allows. `DONE` can assert slightly
    /// before every byte has actually been drained from the FIFO, so
    /// this keeps draining until `buffer` is actually full rather than
    /// stopping at the first sight of `DONE`.
    fn read_one(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error> {
        if buffer.is_empty() {
            return Err(Error::ZeroLengthUnsupported);
        }
        self.one_shot(address, true, buffer.len());

        let mut received = 0;
        loop {
            let status = self.bsc.s().read();
            if status.err().bit_is_set() {
                self.clear_status();
                return Err(Error::NoAcknowledge);
            }
            if received < buffer.len() && status.rxd().bit_is_set() {
                buffer[received] = self.bsc.fifo().read().data().bits();
                received += 1;
            }
            if status.done().bit_is_set() && received >= buffer.len() {
                break;
            }
        }

        self.clear_status();
        Ok(())
    }
}

impl<I: Deref<Target = bsc0::RegisterBlock>> embedded_hal::i2c::ErrorType for I2c<I> {
    /// See [`Error`].
    type Error = Error;
}

impl<I: Deref<Target = bsc0::RegisterBlock>> embedded_hal::i2c::I2c for I2c<I> {
    /// `read`/`write`/`write_read` all forward here via
    /// `embedded_hal::i2c::I2c`'s default implementations. See this
    /// struct's doc comment: each [`embedded_hal::i2c::Operation`]
    /// gets its own complete START...STOP, not a true repeated start.
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        for operation in operations {
            match operation {
                embedded_hal::i2c::Operation::Read(buffer) => self.read_one(address, buffer)?,
                embedded_hal::i2c::Operation::Write(bytes) => self.write_one(address, bytes)?,
            }
        }
        Ok(())
    }
}
