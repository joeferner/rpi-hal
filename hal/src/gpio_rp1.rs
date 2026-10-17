//! GPIO pin wrapper with compile-time input/output mode enforcement
//! (typestate) and `embedded-hal` `digital` trait implementations — the
//! Pi 5 build, over the RP1 south bridge's GPIO bank 0.
//!
//! Same API as the other chips' `gpio` module (`Pin::<N, Input>::new`,
//! `into_output`, the pulls, the `embedded-hal` traits), so code driving
//! header pins ports across; only interrupts are missing, since there is no
//! interrupt controller driver for this chip yet.
//!
//! A pin is driven through three RP1 register blocks (see
//! [`pac::GPIO`](crate::pac::GPIO)): `IO_BANK0`'s `CTRL.FUNCSEL` hands the
//! pin to `SYS_RIO`, `SYS_RIO`'s `OUT`/`OE`/`IN` registers then drive and
//! sense it one bit per pin, and `PADS_BANK0` owns its pulls and its input
//! and output buffers. Register layout from Linux's `pinctrl-rp1.c`.
//!
//! Only bank 0 — GPIO0-27, the 40-pin header — is covered. RP1's banks 1
//! and 2 (GPIO28-53) are wired to on-board functions (Ethernet PHY reset,
//! the fan, the PWR LED, the camera/display connectors' control lines) and
//! are not general-purpose on a Pi 5.

use crate::pac::GPIO;
use core::marker::PhantomData;

/// Marker type: pin configured as output. See [`Pin`].
pub struct Output;

/// Marker type: pin configured as input. See [`Pin`].
pub struct Input;

/// Which internal resistor holds a pin at a defined level while nothing
/// external drives it — selected per pin via [`Pin::set_pull`] and the
/// `into_*_input` converters.
///
/// The resistors are weak (tens of kΩ), so anything actively driving the
/// pin wins; the pull only decides the level when the line is left
/// floating.
///
/// This is not typestate: a pull protects no invariant, so it is a plain
/// runtime setting on `Pin<N, MODE>` and does not appear in the pin's type.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Pull {
    /// Pull-up: the pin idles high.
    Up,
    /// Pull-down: the pin idles low.
    Down,
    /// No pull: the pin floats, and an undriven input reads whatever
    /// leakage and nearby switching leave it at.
    None,
}

/// A single header GPIO pin (0-27), typestated on whether it's currently
/// `Input` or `Output` — `into_output`/`into_input` are the only way to
/// change mode, and `embedded_hal::digital`'s traits are only implemented
/// for the mode they apply to, so e.g. `set_high` doesn't compile against a
/// pin still in `Input` mode.
pub struct Pin<const N: u8, MODE> {
    _gpio: GPIO,
    _mode: PhantomData<MODE>,
}

impl<const N: u8> Pin<N, Input> {
    /// Wraps an already-stolen `GPIO` token as pin `N`, without touching
    /// hardware. Typed `Input` to match the other chips' constructor, but
    /// unlike there that is not necessarily what the hardware is doing:
    /// RP1 pins come out of reset with no function selected at all, and the
    /// firmware may have claimed some since. Call `into_input` or
    /// `into_output` to actually configure the pin.
    pub fn new(gpio: GPIO) -> Self {
        const { Self::VALID_PIN };
        Self {
            _gpio: gpio,
            _mode: PhantomData,
        }
    }
}

impl<const N: u8, MODE> Pin<N, MODE> {
    /// Compile-time check that `N` is in bank 0 — evaluated (forced) from
    /// `Pin::new`, so an out-of-range `N` fails to build rather than
    /// misbehaving at runtime.
    const VALID_PIN: () = assert!(N <= 27, "the Pi 5 header is RP1 GPIO0..=27");

    /// Reconfigures this pin as an output: hands it to `SYS_RIO`, enables
    /// its pad's output buffer, then sets its output enable.
    ///
    /// The level it starts driving is whatever `OUT` already held for it.
    pub fn into_output(self) -> Pin<N, Output> {
        select_rio(N);
        rio_write(RIO_OE + SET_ALIAS, 1 << N);
        Pin {
            _gpio: self._gpio,
            _mode: PhantomData,
        }
    }

    /// Reconfigures this pin as an input: hands it to `SYS_RIO` and clears
    /// its output enable.
    ///
    /// Does not touch the pin's pull resistor — see [`set_pull`] on why
    /// "input" here does not mean "floating".
    ///
    /// [`set_pull`]: Self::set_pull
    pub fn into_input(self) -> Pin<N, Input> {
        select_rio(N);
        rio_write(RIO_OE + CLR_ALIAS, 1 << N);
        Pin {
            _gpio: self._gpio,
            _mode: PhantomData,
        }
    }

    /// Selects this pin's internal pull resistor (see [`Pull`]).
    ///
    /// Independent of direction, and legal on an output as well as an
    /// input. A pin's pull on arrival here is whatever reset and the
    /// firmware left — call this (or one of the `into_*_input` converters)
    /// rather than assuming a fresh input floats.
    pub fn set_pull(&self, pull: Pull) {
        set_pull(N, pull);
    }

    /// Reads back this pin's currently selected pull resistor.
    ///
    /// `None` — not [`Pull::None`], which is a *selected* setting — means
    /// both the pull-up and the pull-down were enabled at once, which this
    /// API never writes.
    pub fn pull(&self) -> Option<Pull> {
        let pad = pad_read(N);
        match (pad & PAD_PUE != 0, pad & PAD_PDE != 0) {
            (false, false) => Some(Pull::None),
            (true, false) => Some(Pull::Up),
            (false, true) => Some(Pull::Down),
            (true, true) => None,
        }
    }

    /// Reconfigures this pin as an input with its internal pull-up
    /// enabled — the pin idles high, so a switch to ground reads as low
    /// with no external resistor.
    pub fn into_pull_up_input(self) -> Pin<N, Input> {
        let pin = self.into_input();
        pin.set_pull(Pull::Up);
        pin
    }

    /// Reconfigures this pin as an input with its internal pull-down
    /// enabled — the pin idles low, so a switch to 3V3 reads as high
    /// with no external resistor.
    pub fn into_pull_down_input(self) -> Pin<N, Input> {
        let pin = self.into_input();
        pin.set_pull(Pull::Down);
        pin
    }

    /// Reconfigures this pin as an input with no pull resistor, leaving
    /// it floating. Only useful where something external always drives
    /// the line (or provides its own pull).
    pub fn into_floating_input(self) -> Pin<N, Input> {
        let pin = self.into_input();
        pin.set_pull(Pull::None);
        pin
    }

    /// Wraps an already-stolen `GPIO` token as pin `N` in mode `MODE`,
    /// without touching hardware. Needed for contexts that only have a
    /// freshly-stolen `GPIO` token, not the original `Pin` that was
    /// already configured.
    ///
    /// # Safety
    ///
    /// `MODE` must correctly describe pin `N`'s actual current
    /// configuration — the compiler can't check this the way it can for
    /// `into_output`/`into_input`.
    pub unsafe fn assume_mode(gpio: GPIO) -> Self {
        const { Self::VALID_PIN };
        Self {
            _gpio: gpio,
            _mode: PhantomData,
        }
    }
}

impl<const N: u8> embedded_hal::digital::ErrorType for Pin<N, Output> {
    /// Infallible — every operation here is a direct, always-succeeding
    /// register access.
    type Error = core::convert::Infallible;
}

impl<const N: u8> embedded_hal::digital::OutputPin for Pin<N, Output> {
    /// Drives the pin low.
    fn set_low(&mut self) -> Result<(), Self::Error> {
        rio_write(RIO_OUT + CLR_ALIAS, 1 << N);
        Ok(())
    }

    /// Drives the pin high.
    fn set_high(&mut self) -> Result<(), Self::Error> {
        rio_write(RIO_OUT + SET_ALIAS, 1 << N);
        Ok(())
    }
}

impl<const N: u8> embedded_hal::digital::StatefulOutputPin for Pin<N, Output> {
    /// Reads back the level this pin is being told to drive (`OUT`), not
    /// the level sensed on the pad, so a pin held down by a short still
    /// reports what was asked of it.
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(rio_read(RIO_OUT) & (1 << N) != 0)
    }

    /// The inverse of `is_set_high`.
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(rio_read(RIO_OUT) & (1 << N) == 0)
    }
}

impl<const N: u8> embedded_hal::digital::ErrorType for Pin<N, Input> {
    /// Infallible — every operation here is a direct, always-succeeding
    /// register access.
    type Error = core::convert::Infallible;
}

impl<const N: u8> embedded_hal::digital::InputPin for Pin<N, Input> {
    /// True if the pin currently reads high.
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(read_level(N))
    }

    /// The inverse of `is_high`.
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!read_level(N))
    }
}

impl<const N: u8> Pin<N, Input> {
    /// Blocks until the pin reads high, busy-polling `IN`. Returns
    /// immediately if it is already high; a high pulse shorter than the
    /// loop's sampling can be missed.
    pub fn wait_for_high(&self) {
        while !read_level(N) {
            core::hint::spin_loop();
        }
    }

    /// Blocks until the pin reads low, busy-polling `IN` — the inverse of
    /// [`wait_for_high`](Self::wait_for_high).
    pub fn wait_for_low(&self) {
        while read_level(N) {
            core::hint::spin_loop();
        }
    }
}

/// `SYS_RIO` output level register, one bit per pin.
const RIO_OUT: usize = 0x00;
/// `SYS_RIO` output enable register, one bit per pin.
const RIO_OE: usize = 0x04;
/// `SYS_RIO` synchronised input level register, one bit per pin.
const RIO_IN: usize = 0x08;

/// Offset of every RP1 register's atomic bit-set alias: a write here sets
/// the written bits and leaves the rest alone, so two pins never race over
/// a read-modify-write of a shared register.
const SET_ALIAS: usize = 0x2000;
/// Offset of every RP1 register's atomic bit-clear alias.
const CLR_ALIAS: usize = 0x3000;

/// `CTRL` offset within pin `n`'s 8-byte `IO_BANK0` slot (`STATUS` is at 0).
const CTRL: usize = 0x04;
/// `CTRL.FUNCSEL` (bits 4:0), the pin's function select.
const CTRL_FUNCSEL: u32 = 0x1f;
/// `CTRL.OUTOVER` (bits 13:12): 0 lets the selected function drive the
/// level.
const CTRL_OUTOVER: u32 = 0b11 << 12;
/// `CTRL.OEOVER` (bits 15:14): 0 lets the selected function drive the
/// output enable.
const CTRL_OEOVER: u32 = 0b11 << 14;
/// `FUNCSEL` value selecting `SYS_RIO` — software-driven GPIO.
const FUNCSEL_SYS_RIO: u32 = 5;

/// Pad register bit 2: pull-down enable.
const PAD_PDE: u32 = 1 << 2;
/// Pad register bit 3: pull-up enable.
const PAD_PUE: u32 = 1 << 3;
/// Pad register bit 6: input buffer enable.
const PAD_IE: u32 = 1 << 6;
/// Pad register bit 7: output buffer *disable*.
const PAD_OD: u32 = 1 << 7;

/// Hands pin `n` to `SYS_RIO`. See [`select_function`].
///
/// Enabling the output buffer there does not start driving the pin: whether
/// it drives is `SYS_RIO`'s `OE` bit, which the caller sets or clears next.
fn select_rio(n: u8) {
    select_function(n, FUNCSEL_SYS_RIO);
}

/// Hands bank-0 pin `n` to function `funcsel` (`CTRL.FUNCSEL`, the column
/// of RP1's pin function table), with both its input and output buffers
/// enabled at the pad and the function itself in charge of the level and
/// output enable — the same sequence Linux's `rp1_set_fsel` uses.
///
/// For the drivers in this crate that mux their own pins, as
/// [`crate::uart`] does with GPIO14/15.
pub(crate) fn select_function(n: u8, funcsel: u32) {
    pad_modify(n, PAD_IE | PAD_OD, PAD_IE);

    let ctrl = (GPIO::IO_BANK0 + 8 * n as usize + CTRL) as *mut u32;
    unsafe {
        let value = ctrl.read_volatile() & !(CTRL_FUNCSEL | CTRL_OUTOVER | CTRL_OEOVER);
        ctrl.write_volatile(value | funcsel);
    }
}

/// Sets bank-0 pin `n`'s output drive strength to 12mA, the strongest the
/// pad offers (`DRIVE`, bits 5:4, `0b11`). For drivers that mux their own
/// pins onto a bus with external pull-ups and capacitance to drive, as
/// [`crate::i2c`] does: the strength Linux's `rp1.dtsi` gives its I2C pins.
pub(crate) fn set_drive_12ma(n: u8) {
    const PAD_DRIVE: u32 = 0b11 << 4;
    pad_modify(n, PAD_DRIVE, PAD_DRIVE);
}

/// Applies `pull` to bank-0 pin `n`. The implementation behind
/// [`Pin::set_pull`], for drivers that mux their own pins.
pub(crate) fn set_pull(n: u8, pull: Pull) {
    let field = match pull {
        Pull::None => 0,
        Pull::Down => PAD_PDE,
        Pull::Up => PAD_PUE,
    };
    pad_modify(n, PAD_PDE | PAD_PUE, field);
}

/// Address of pin `n`'s pad register: after the bank's 4-byte
/// `VOLTAGE_SELECT`, one word per pin.
fn pad_register(n: u8) -> *mut u32 {
    (GPIO::PADS_BANK0 + 4 + 4 * n as usize) as *mut u32
}

/// Reads pin `n`'s pad register.
fn pad_read(n: u8) -> u32 {
    unsafe { pad_register(n).read_volatile() }
}

/// Replaces the `mask` bits of pin `n`'s pad register with `value`.
///
/// A plain read-modify-write rather than the set/clear aliases, since a
/// field like the pulls needs one bit set and the other cleared together.
/// It is per-pin, so the only thing it can race with is another owner of
/// the same pin.
fn pad_modify(n: u8, mask: u32, value: u32) {
    let pad = pad_register(n);
    unsafe { pad.write_volatile((pad.read_volatile() & !mask) | value) };
}

/// Writes `value` to the `SYS_RIO0` register at `offset` (which may
/// include an alias offset).
fn rio_write(offset: usize, value: u32) {
    unsafe { ((GPIO::SYS_RIO0 + offset) as *mut u32).write_volatile(value) };
}

/// Reads the `SYS_RIO0` register at `offset`.
fn rio_read(offset: usize) -> u32 {
    unsafe { ((GPIO::SYS_RIO0 + offset) as *const u32).read_volatile() }
}

/// Reads pin `n`'s sensed level from `IN`.
fn read_level(n: u8) -> bool {
    rio_read(RIO_IN) & (1 << n) != 0
}
