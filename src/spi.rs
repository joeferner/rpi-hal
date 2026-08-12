//! Blocking driver for SPI0.

use crate::pac::{GPIO, SPI0};
use embedded_hal::spi::{Phase, Polarity};

/// Which chip-select line `Spi` drives automatically (via `TA`) around
/// every `SpiBus` call.
///
/// The register field is 2 bits wide (four encodings), but only CE0
/// (GPIO8) and CE1 (GPIO7) are actually wired to a pin on this board —
/// there's no GPIO broken out for the third hardware-driven encoding,
/// so it isn't offered here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChipSelect {
    /// CE0, GPIO8.
    Cs0 = 0,
    /// CE1, GPIO7.
    Cs1 = 1,
    /// Neither CE0 nor CE1 is asserted; `TA` still gates the transfer,
    /// but no physical line moves as a result. Use this when chip
    /// select is managed externally instead — e.g. wrapping `Spi` in
    /// `embedded-hal-bus`'s `ExclusiveDevice` together with a plain
    /// `embedded_hal::digital::OutputPin` (this crate's own
    /// `gpio::Pin` works), or toggling one by hand. Without this,
    /// `Spi`'s automatic per-call CS assertion would fight a
    /// separately-managed CS pin.
    None = 0b11,
}

/// Blocking driver for SPI0 (GPIO9-11 always; GPIO7/8 only when using
/// a hardware-driven `ChipSelect`).
///
/// Implements `embedded_hal::spi::SpiBus` rather than `SpiDevice`. With
/// `ChipSelect::Cs0`/`Cs1` this peripheral ties chip-select assertion
/// to its own `TA` (transfer active) bit in hardware, so a `Spi`
/// instance already owns and drives one dedicated CS line for the
/// duration of each bus operation — there's nothing to compose in via
/// `embedded-hal-bus` in that mode. With `ChipSelect::None`, `Spi`
/// leaves both hardware CE lines alone and CS becomes the caller's
/// responsibility (e.g. via `embedded-hal-bus`'s `ExclusiveDevice` and
/// a plain `gpio::Pin`), same as HALs where `SpiBus` is CS-agnostic.
pub struct Spi {
    spi0: SPI0,
}

impl Spi {
    /// Routes GPIO9-11 to SPI0 (ALT0: MISO, MOSI, SCLK) — and, only for
    /// `ChipSelect::Cs0`/`Cs1`, GPIO7/8 too (ALT0: CE1, CE0) — then
    /// configures clock polarity/phase and the clock divider.
    /// `ChipSelect::None` deliberately leaves GPIO7/8 untouched so the
    /// caller can use them (or any other pin) as a plain GPIO chip
    /// select instead.
    ///
    /// `clock_divider` is passed straight through to the `CLK` register
    /// rather than computed from a requested frequency: the SPI core
    /// clock varies with `core_freq`/overclock settings in
    /// `config.txt` and isn't a fixed, firmware-guaranteed value the
    /// way UART0's reference clock is (see `uart.rs`'s `init` for that
    /// comparison) — a divider computed against an assumed core clock
    /// would silently be wrong on a board configured differently. Must
    /// be even (0 means divide by 65536); the hardware rounds odd
    /// values down. If an exact frequency is needed, query the real
    /// core clock via the VideoCore mailbox and compute the divider
    /// from that instead of guessing here.
    pub fn init(
        gpio: &GPIO,
        spi0: SPI0,
        mode: embedded_hal::spi::Mode,
        chip_select: ChipSelect,
        clock_divider: u16,
    ) -> Self {
        gpio.gpfsel0().modify(|_, w| w.fsel9().spi0_miso());
        gpio.gpfsel1()
            .modify(|_, w| w.fsel10().spi0_mosi().fsel11().spi0_sclk());

        if chip_select != ChipSelect::None {
            gpio.gpfsel0()
                .modify(|_, w| w.fsel7().spi0_ce1_n().fsel8().spi0_ce0_n());
        }

        let cpol = mode.polarity == Polarity::IdleHigh;
        let cpha = mode.phase == Phase::CaptureOnSecondTransition;

        spi0.cs().write(|w| {
            unsafe { w.cs().bits(chip_select as u8) }
                .cpol()
                .bit(cpol)
                .cpha()
                .bit(cpha)
                .clear()
                .both()
                // REN ("Read Enable") is `1` at reset and `.write()`
                // would otherwise leave it there untouched. The
                // register spec calls REN relevant only in 3-wire/LoSSI
                // mode (LEN=1, which this driver never sets) -- but on
                // real hardware, leaving REN=1 here was the actual
                // cause of a bug this crate hit during bring-up: SCLK
                // toggled correctly, but MOSI stayed low and every
                // received byte read back `0x00` regardless of what was
                // sent, in plain 4-wire mode. Clearing REN fixed it
                // completely (confirmed via a per-byte register/pin
                // trace in `examples/spi_loopback.rs`'s history).
                .ren()
                .clear_bit()
        });

        spi0.clk()
            .write(|w| unsafe { w.cdiv().bits(clock_divider) });

        let spi = Self { spi0 };
        spi.reset_hw();
        spi
    }

    /// Resets the peripheral to a clean baseline: disables interrupts/
    /// DMA/`TA`, clears both FIFOs, and clears `DLEN` — matching Linux's
    /// `spi-bcm2835` driver's `bcm2835_spi_reset_hw`, including its
    /// documented erratum: despite `DONE` (bit 16) being modeled
    /// read-only in the register spec — and in this PAC, which doesn't
    /// generate a writer for it at all — the Linux driver writes 1 back
    /// to it here regardless, on the theory that it's actually RW1C.
    /// Needs a raw `.bits()` write since the safe per-field writer has
    /// no way to touch `done`.
    ///
    /// Note: writing `DONE` here was *not* what fixed this crate's own
    /// MOSI-stuck-low bug during bring-up — a register
    /// trace showed it made no observable difference at all; the real
    /// cause was `REN` being left at its reset default in `init`. This
    /// reset is kept anyway since it's cheap, harmless, and matches the
    /// reference driver's general hygiene, not because it's known to
    /// matter on this specific hardware.
    fn reset_hw(&self) {
        const INTR: u32 = 1 << 10;
        const INTD: u32 = 1 << 9;
        const DMAEN: u32 = 1 << 8;
        const TA: u32 = 1 << 7;
        const CLEAR_TX: u32 = 1 << 4;
        const CLEAR_RX: u32 = 1 << 5;
        const DONE: u32 = 1 << 16;

        let cs = self.spi0.cs().read().bits();
        let cs = (cs & !(INTR | INTD | DMAEN | TA)) | DONE | CLEAR_TX | CLEAR_RX;
        unsafe {
            self.spi0.cs().write(|w| w.bits(cs));
            self.spi0.dlen().write(|w| w.bits(0));
        }
    }

    /// Asserts the configured chip-select line (`TA` set). Relies on
    /// `reset_hw` (called at `init` and at the end of every prior
    /// transfer) having already left both FIFOs clear.
    fn begin_transfer(&self) {
        self.spi0.cs().modify(|_, w| w.ta().set_bit());
    }

    /// Waits for the current transfer to finish shifting (`DONE`), then
    /// resets the peripheral back to a clean baseline for the next one.
    fn end_transfer(&self) {
        while self.spi0.cs().read().done().bit_is_clear() {}
        self.reset_hw();
    }

    /// Full-duplex shift of `len` bytes: `tx(i)` supplies the byte to
    /// send for index `i`, `rx(i, byte)` receives the byte shifted in
    /// for index `i`. Every `SpiBus` method except `transfer_in_place`
    /// boils down to this, since the hardware always shifts a byte out
    /// and in together — there's no separate write-only/read-only mode
    /// at the protocol level, only at the API level.
    fn shift(
        &mut self,
        len: usize,
        mut tx: impl FnMut(usize) -> u8,
        mut rx: impl FnMut(usize, u8),
    ) {
        self.begin_transfer();

        let mut sent = 0;
        let mut received = 0;
        while received < len {
            if sent < len && self.spi0.cs().read().txd().bit_is_set() {
                let byte = tx(sent);
                unsafe {
                    self.spi0.fifo().write(|w| w.data().bits(byte as u32));
                }
                sent += 1;
            }
            if self.spi0.cs().read().rxd().bit_is_set() {
                let byte = self.spi0.fifo().read().data().bits() as u8;
                rx(received, byte);
                received += 1;
            }
        }

        self.end_transfer();
    }
}

impl embedded_hal::spi::ErrorType for Spi {
    /// Infallible — every operation here is a direct busy-wait on
    /// hardware flags, no failure path exists.
    type Error = core::convert::Infallible;
}

impl embedded_hal::spi::SpiBus<u8> for Spi {
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
    /// `write`'s end send `0x00`; indices beyond `read`'s end discard
    /// the response.
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
    /// Not implemented via `shift`: that takes two closures which would
    /// both need to capture `words` (one to read the byte still to be
    /// sent, one to write the byte just received), which Rust's borrow
    /// checker won't allow even though the access pattern is safe in
    /// practice (a given index is always read before it's overwritten).
    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.begin_transfer();

        let len = words.len();
        let mut sent = 0;
        let mut received = 0;
        while received < len {
            if sent < len && self.spi0.cs().read().txd().bit_is_set() {
                let byte = words[sent];
                unsafe {
                    self.spi0.fifo().write(|w| w.data().bits(byte as u32));
                }
                sent += 1;
            }
            if self.spi0.cs().read().rxd().bit_is_set() {
                words[received] = self.spi0.fifo().read().data().bits() as u8;
                received += 1;
            }
        }

        self.end_transfer();
        Ok(())
    }

    /// This driver never buffers beyond the hardware FIFOs and every
    /// other method already waits for `DONE` before returning, so
    /// there's nothing left in flight by the time `flush` could be
    /// called — a no-op.
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
