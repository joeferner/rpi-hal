//! Blocking driver for the BCM2835 BSC I2C controllers.
//!
//! Generic over the BSC instance: [`I2c<BSC1>`](crate::i2c::I2c) drives I2C1 on
//! GPIO2 (SDA1)/GPIO3 (SCL1), the general-purpose bus on the 40-pin header,
//! and [`I2c<BSC0>`](crate::i2c::I2c) drives BSC0, on either of its two
//! routings — GPIO44 (SDA0)/GPIO45 (SCL0), the one the camera/display
//! connectors use on a Pi 3 ([`I2c::<BSC0>::init`](crate::i2c::I2c::init)),
//! or GPIO0 (ID_SD)/GPIO1 (ID_SC), the HAT ID EEPROM bus on the 40-pin
//! header ([`I2c::<BSC0>::init_id`](crate::i2c::I2c::init_id)). The
//! register layout is identical across instances (both deref to the same PAC
//! register block); only the pin mux and the peripheral token differ, so
//! the transfer logic is shared and each routing gets its own constructor.
//!
//! With the `async` feature the same type also implements
//! `embedded_hal_async::i2c::I2c`, driven by the controller's own
//! interrupts rather than by polling — see [`on_irq`](crate::i2c::on_irq)
//! for what the application has to wire up, and note that a timeout there
//! is the caller's `with_timeout` rather than the deadline below.
//!
//! Every blocking transfer is bounded against the System Timer, which is why
//! [`I2c::init`](crate::i2c::I2c::init) takes a [`Timer`](crate::timer::Timer). I2C is the one
//! bus in this crate where a *foreign* device — not silicon on the same
//! die — decides whether a transfer ever finishes, and a device that
//! acknowledges its address and then stops driving sets neither `S.ERR`
//! nor `S.DONE`. An unbounded poll of `S` is then infinite, and since this
//! is a blocking driver it takes the rest of the program with it (an
//! executor, a network stack, a watchdog kick). Bounding the wait turns
//! that into an [`Error::Timeout`](crate::i2c::Error::Timeout) the caller
//! can log, retry, or ignore.

use core::ops::Deref;

use crate::pac::{bsc0, BSC0, BSC1, GPIO};
use crate::timer::Timer;

#[cfg(feature = "async")]
mod asynch;
#[cfg(feature = "async")]
pub use asynch::on_irq;

/// Fixed allowance for START, the address byte, and the controller's own
/// setup, on top of [`TIMEOUT_PER_BYTE_US`].
const TIMEOUT_BASE_US: u64 = 5_000;

/// Allowance per byte of the transfer. At the reset-default divider
/// (100kHz) a byte and its acknowledge take ~90us, so this is more than
/// tenfold margin and still tolerates a slave that stretches the clock.
/// The exact number matters much less than it being finite.
const TIMEOUT_PER_BYTE_US: u64 = 1_000;

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
    ///
    /// The consequence for bus scans is worth stating outright, since
    /// it costs real debugging time: a scan built on 1-byte reads
    /// enumerates what answers *reads*, which is not the same as what
    /// is on the bus. A device that only answers a read while it has a
    /// result pending -- every Sensirion SHT4x, among others -- NAKs
    /// the probe and is reported absent while happily acknowledging
    /// commands. `i2cdetect` finds those because it probes with a
    /// zero-length write, which this hardware cannot issue at all.
    ZeroLengthUnsupported,
    /// The transfer neither completed (`S.DONE`) nor failed (`S.ERR`)
    /// within its deadline: the bus is being held, or a slave stopped
    /// driving mid-transfer. The controller has been left in a state a
    /// subsequent transfer can start from (FIFOs cleared, status
    /// cleared), but that is best-effort: a bus a slave is still holding
    /// will simply time out again.
    ///
    /// Also reported for the controller's *own* clock-stretch timeout
    /// (`S.CLKT`), where a slave held SCL down past the `CLKT` register's
    /// allowance and the hardware cut the transfer short. Same meaning —
    /// the bus was held — reached one level lower down, and quicker.
    Timeout,
    /// The transfer completed, but the slave delivered fewer bytes than
    /// were asked for -- it NAK'd mid-read, reset, or was asked for
    /// more than it had. Carries both counts, since how many arrived is
    /// what says whether the device is mute, truncating, or was simply
    /// over-read; `buffer[received..]` is untouched.
    Incomplete {
        /// Bytes actually drained from the RX FIFO into the buffer.
        received: usize,
        /// Bytes the transfer asked for (`buffer.len()`, the `DLEN` the
        /// controller was programmed with).
        requested: usize,
    },
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
            // `embedded-hal` 1.0 has no closer variant for either of
            // the last two: `Overrun` means the *receive buffer* was
            // overrun, which is a different failure.
            Self::ZeroLengthUnsupported | Self::Timeout | Self::Incomplete { .. } => {
                embedded_hal::i2c::ErrorKind::Other
            }
        }
    }
}

/// The `DIV.CDIV` value that clocks SCL at or below `target_hz`, given
/// the SoC core clock `core_hz` — the arithmetic behind `init`'s
/// `clock_divider`, in one place rather than in every application.
///
/// SCL is `core_hz / CDIV`, so this is that division rounded *up*, and
/// then up again to the even value the hardware requires (it rounds an
/// odd `CDIV` down, so asking for 1 would land on 0, which means 32768 —
/// the slowest rate on the bus, not the fastest). Rounding up means the
/// result never clocks faster than asked, which is the direction that
/// matters: an I2C part states a maximum bus rate, so erring low makes a
/// 400kHz device run slightly slow, while erring high makes it fail
/// intermittently at some temperature you were not testing at. The rate
/// actually produced is `core_hz / returned`.
///
/// `core_hz` has to come from the firmware rather than a constant, which
/// is the whole reason `init` does not take a frequency itself:
///
/// ```ignore
/// let core_hz = mailbox.clock_rate_hz(ClockId::Core)?;
/// let i2c = I2c::init(gpio, bsc1, i2c::divider_for(core_hz, 100_000), &timer);
/// ```
///
/// The core clock moves with `config.txt` and with the firmware's own
/// scaling, and the gap is not academic: the reset default of 1500 is
/// documented as 100kHz because the datasheet assumes a 150MHz core, and
/// is 166kHz on a board running 250MHz.
///
/// The result is clamped to what the field can usefully express, 2 to
/// 65534. `CDIV = 0` would be one step slower still, but a zero handed
/// back from a function like this reads as an error or a divide-by-zero
/// everywhere it is subsequently used, and `core_hz / 65534` is under
/// 4kHz on any Pi — far below anything an I2C part will answer.
pub fn divider_for(core_hz: u32, target_hz: u32) -> u16 {
    // `target_hz.max(1)` rather than a `Result` or a panic: zero is not a
    // frequency, and the useful reading of "as slow as possible" is the
    // largest divider, which is exactly what the clamp below produces.
    let exact = core_hz.div_ceil(target_hz.max(1));
    // Round up to even by adding the odd bit back, rather than
    // `next_multiple_of(2)`, which overflows on an odd `u32::MAX` — a
    // nonsense argument, but a public function should clamp it rather
    // than panic in a debug build and wrap in a release one.
    exact.saturating_add(exact % 2).clamp(2, 65534) as u16
}

/// Blocking driver for a BCM2835 BSC I2C controller, generic over the
/// instance `I` (defaulting to [`BSC1`] so `I2c` alone means the
/// general-purpose header bus). Construct with [`I2c::<BSC1>::init`] for
/// I2C1 on GPIO2/3, [`I2c::<BSC0>::init`] for BSC0 on GPIO44/45, or
/// [`I2c::<BSC0>::init_id`] for BSC0 on GPIO0/1.
///
/// A note on BSC0: one controller, two routings this crate can select
/// between — GPIO44/45 (the camera/display connector bus on a Pi 3) and
/// GPIO0/1 (`ID_SD`/`ID_SC`, the HAT ID EEPROM bus on the 40-pin header).
/// They are electrically different buses sharing one peripheral, so the
/// choice is a separate constructor rather than an argument that is easy
/// to skim past, and only one of them can be live at a time. BSC0 is also
/// nominally owned by the VideoCore firmware, which arbitrates the
/// camera/display bus — so a program driving the GPIO44/45 routing should
/// have taken the machine over rather than leaving the firmware's camera
/// stack running to race on the bus. The GPIO0/1 routing is quieter; see
/// [`I2c::<BSC0>::init_id`] for why.
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
///
/// Every transfer is bounded against the borrowed [`Timer`] (see this
/// module's doc comment on why). The timer is stored rather than passed
/// per call because transfers are reached through
/// `embedded_hal::i2c::I2c::transaction`, whose signature this crate
/// doesn't control.
pub struct I2c<'a, I = BSC1> {
    bsc: I,
    timer: &'a Timer,
}

impl<'a> I2c<'a, BSC1> {
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
    /// clock; halve it for 400kHz fast mode at that same clock.
    ///
    /// To pick one from a frequency instead — which is the only way to
    /// get the rate you asked for on a board whose core clock is not
    /// 150MHz — ask the mailbox for the real core clock and hand it to
    /// [`divider_for`].
    ///
    /// `timer` bounds every transfer this driver performs — see this
    /// type's doc comment.
    pub fn init(gpio: &GPIO, bsc1: BSC1, clock_divider: u16, timer: &'a Timer) -> Self {
        gpio.gpfsel0()
            .modify(|_, w| w.fsel2().sda1().fsel3().scl1());
        Self::configure(bsc1, clock_divider, timer)
    }
}

impl<'a> I2c<'a, BSC0> {
    /// Routes GPIO44/45 to BSC0 (ALT1: SDA0, SCL0) — the camera/display
    /// connector bus on a Pi 3, not BSC0's GPIO0/1 HAT-EEPROM routing
    /// ([`init_id`](Self::init_id)) — sets the clock divider, and enables
    /// the peripheral.
    ///
    /// See [`I2c::<BSC1>::init`] on why `clock_divider` is a raw `CDIV`
    /// value rather than a target frequency; the same reasoning applies.
    /// See this type's doc comment on BSC0 being firmware-arbitrated,
    /// and on `timer` bounding every transfer.
    pub fn init(gpio: &GPIO, bsc0: BSC0, clock_divider: u16, timer: &'a Timer) -> Self {
        gpio.gpfsel4()
            .modify(|_, w| w.fsel44().sda0().fsel45().scl0());
        Self::configure(bsc0, clock_divider, timer)
    }

    /// Routes GPIO0/1 to BSC0 (ALT0: SDA0, SCL0) — `ID_SD`/`ID_SC` on
    /// pins 27/28 of the 40-pin header, the HAT ID EEPROM bus — sets the
    /// clock divider, and enables the peripheral.
    ///
    /// The *same* controller as [`init`](Self::init), on the other
    /// routing: one of the two, not both, since enabling a pin's ALT
    /// function is what connects it to the peripheral. Taking this one
    /// therefore gives up the camera/display bus, and a driver written
    /// against `I2c<BSC0>` (say [`crate::ov5647`]) will be talking to
    /// whichever pair was muxed last.
    ///
    /// Despite the "reserved for HAT ID EEPROM detection" warning these
    /// pins carry in Raspberry Pi's own documentation, a bare-metal
    /// program is free to use them: the VideoCore firmware reads the
    /// EEPROM (address 0x50, per the HAT specification) early in boot,
    /// before the kernel image runs at all, and then leaves the pins
    /// alone — the warning is aimed at Linux userspace, where the ID
    /// bus is also how an add-on board's overlay gets loaded. The board
    /// fits 1.8kΩ pull-ups on both lines, so nothing external is needed
    /// to make the bus work either. What the warning does still mean is
    /// that anything a fitted HAT put on this bus is shared with that
    /// boot-time probe, and that a HAT may itself expect to be the only
    /// thing here.
    ///
    /// See [`I2c::<BSC1>::init`] on why `clock_divider` is a raw `CDIV`
    /// value rather than a target frequency; the same reasoning applies.
    /// The HAT specification requires the ID EEPROM to work at 100kHz,
    /// and a HAT's designer had no reason to design for more, so this is
    /// the routing least worth overclocking. See this type's doc comment
    /// on `timer` bounding every transfer.
    pub fn init_id(gpio: &GPIO, bsc0: BSC0, clock_divider: u16, timer: &'a Timer) -> Self {
        // Pulls are left as they are, as in the other two `init`s. GPIO0/1
        // power up pulled high, which is the direction an I2C line wants,
        // and the board's 1.8kΩ external pull-ups are what actually hold
        // the bus up regardless.
        gpio.gpfsel0()
            .modify(|_, w| w.fsel0().sda0().fsel1().scl0());
        Self::configure(bsc0, clock_divider, timer)
    }
}

impl<'a, I: Deref<Target = bsc0::RegisterBlock>> I2c<'a, I> {
    /// Shared tail of the per-instance `init`s: program the divider and
    /// enable the controller, once the caller has done the instance's own
    /// pin mux.
    fn configure(bsc: I, clock_divider: u16, timer: &'a Timer) -> Self {
        unsafe {
            bsc.div().write(|w| w.cdiv().bits(clock_divider));
        }
        bsc.c().write(|w| w.i2cen().bit(true));
        Self { bsc, timer }
    }

    /// How long a transfer of `len` bytes is allowed to take: a fixed
    /// setup allowance plus a per-byte one, rather than one flat number,
    /// since a 1-byte probe and a 32-byte register dump differ by more
    /// than an order of magnitude.
    fn timeout_us(len: usize) -> u64 {
        TIMEOUT_BASE_US + TIMEOUT_PER_BYTE_US * len as u64
    }

    /// Best-effort return to a known baseline after a transfer that
    /// didn't finish: clear both FIFOs (`C.CLEAR`) with `I2CEN` kept and
    /// `ST` left clear, then clear `DONE`/`ERR`, so the *next* transfer
    /// starts from a defined state rather than inheriting stale FIFO
    /// contents.
    ///
    /// Best-effort is the honest description: the BSC has no documented
    /// abort, and nothing here can make a slave that is holding SDA let
    /// go — that transfer will time out too, which is the correct
    /// outcome and is now survivable. Walking a stuck slave off the bus
    /// needs nine manual clock pulses, which means muxing GPIO2/3 back
    /// to outputs and bit-banging them; the BSC owns the pins while it
    /// is enabled, so that isn't something this method can do.
    fn abandon(&mut self) {
        self.bsc.c().write(|w| {
            unsafe { w.clear().bits(0b11) };
            w.i2cen().bit(true)
        });
        self.clear_status();
    }

    /// Clears `DONE`/`ERR`/`CLKT` (all three write-1-to-clear) ahead of a
    /// new transfer — same "clean baseline before starting" approach as
    /// `spi::Spi::reset_hw`, just scoped to what the BSC actually needs
    /// between transfers (its FIFOs are cleared via `C.CLEAR` in
    /// [`Self::one_shot`] instead, since that's a `C`-register field,
    /// not `S`).
    ///
    /// `CLKT` matters as much as the other two even though no transfer
    /// sets out to produce one: it latches, so a single clock-stretch
    /// timeout left uncleared would be read as a fault by every transfer
    /// afterwards, on a bus that had recovered.
    fn clear_status(&self) {
        self.bsc
            .s()
            .write(|w| w.done().bit(true).err().bit(true).clkt().bit(true));
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
    /// instant `S.ERR` is seen, or [`Error::Timeout`] if the transfer
    /// neither finishes nor fails within [`Self::timeout_us`].
    fn write_one(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error> {
        if bytes.is_empty() {
            return Err(Error::ZeroLengthUnsupported);
        }
        self.one_shot(address, false, bytes.len());

        let deadline = self.timer.now_micros() + Self::timeout_us(bytes.len());
        let mut sent = 0;
        loop {
            let status = self.bsc.s().read();
            if status.err().bit_is_set() {
                self.clear_status();
                return Err(Error::NoAcknowledge);
            }
            // The controller's own clock-stretch timeout fired: a slave
            // held SCL down past `CLKT` and the transfer was cut short.
            // Reported as `Timeout` because that is what happened — the
            // bus was held — and whatever the FIFO holds after one is not
            // the transfer that was asked for.
            if status.clkt().bit_is_set() {
                self.abandon();
                return Err(Error::Timeout);
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
            // A write has no `Incomplete` counterpart to report: `DONE`
            // is checked above, so reaching here means the transfer is
            // still in flight and the deadline has passed.
            if self.timer.now_micros() > deadline {
                self.abandon();
                return Err(Error::Timeout);
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
    ///
    /// That "until the buffer is full" condition is also why the
    /// deadline matters here more than anywhere else in this driver: a
    /// transfer that finishes having delivered *fewer* bytes than `DLEN`
    /// asked for makes the exit condition permanently unreachable, so
    /// without a deadline it would spin forever with `DONE` already set.
    /// `DONE` is exactly what separates the two failures on expiry —
    /// set means the transfer finished short ([`Error::Incomplete`]),
    /// clear means it never finished at all ([`Error::Timeout`]).
    fn read_one(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error> {
        if buffer.is_empty() {
            return Err(Error::ZeroLengthUnsupported);
        }
        self.one_shot(address, true, buffer.len());

        let deadline = self.timer.now_micros() + Self::timeout_us(buffer.len());
        let mut received = 0;
        loop {
            let status = self.bsc.s().read();
            if status.err().bit_is_set() {
                self.clear_status();
                return Err(Error::NoAcknowledge);
            }
            // See `write_one`: a clock-stretch timeout is the bus having
            // been held, and the bytes that did arrive are not the read
            // that was asked for.
            if status.clkt().bit_is_set() {
                self.abandon();
                return Err(Error::Timeout);
            }
            if received < buffer.len() && status.rxd().bit_is_set() {
                buffer[received] = self.bsc.fifo().read().data().bits();
                received += 1;
            }
            if status.done().bit_is_set() && received >= buffer.len() {
                break;
            }
            if self.timer.now_micros() > deadline {
                let complete = self.bsc.s().read().done().bit_is_set();
                self.abandon();
                return Err(if complete {
                    Error::Incomplete {
                        received,
                        requested: buffer.len(),
                    }
                } else {
                    Error::Timeout
                });
            }
        }

        self.clear_status();
        Ok(())
    }
}

impl<I: Deref<Target = bsc0::RegisterBlock>> embedded_hal::i2c::ErrorType for I2c<'_, I> {
    /// See [`Error`].
    type Error = Error;
}

impl<I: Deref<Target = bsc0::RegisterBlock>> embedded_hal::i2c::I2c for I2c<'_, I> {
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
