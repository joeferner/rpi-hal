//! The BCM2712's always-on GPIO block (`gio_aon`), whose lines carry
//! board functions rather than header pins — on a Pi 5, line 9 is the
//! green ACT LED ([`ACT_LED`]).
//!
//! Useful beyond the LED itself as a bring-up instrument: it is on the
//! SoC's own bus, so it works before, and independently of, the RP1 link
//! that every header pin ([`crate::gpio`]) depends on.
//!
//! The block is Broadcom's set-top-box GPIO design (Linux's
//! `gpio-brcmstb.c`), not the BCM283x GPIO of earlier Pis: per bank, eight
//! consecutive 32-bit registers, of which this driver uses `DATA` (level,
//! read and written) and `IODIR` (direction, a set bit meaning input). Its
//! 23 lines are numbered contiguously across two banks of 17 and 6.
//!
//! Typestated the same way as [`crate::gpio::Pin`], with the same
//! [`Input`]/[`Output`] markers and `embedded-hal` traits. There are no
//! pulls and no interrupts: the firmware watches the PMIC's interrupt
//! through this block, so this crate stays out of its interrupt registers.

use crate::gpio::{Input, Output};
use crate::pac::GIO_AON;
use core::marker::PhantomData;

/// The line driving the Pi 5's green ACT LED. Active-low: drive it low to
/// light the LED (`bcm2712-rpi-5-b.dts`: `gpios = <&gio_aon 9
/// GPIO_ACTIVE_LOW>`).
pub const ACT_LED: u8 = 9;

/// A single always-on GPIO line (0-22), typestated on whether it's
/// currently `Input` or `Output`.
pub struct Pin<const N: u8, MODE> {
    _gio: GIO_AON,
    _mode: PhantomData<MODE>,
}

impl<const N: u8> Pin<N, Input> {
    /// Wraps an already-stolen `GIO_AON` token as line `N`, without
    /// touching hardware. Typed `Input` to match [`crate::gpio::Pin::new`];
    /// call `into_output` or `into_input` to actually set the direction.
    pub fn new(gio: GIO_AON) -> Self {
        const { Self::VALID_LINE };
        Self {
            _gio: gio,
            _mode: PhantomData,
        }
    }
}

impl<const N: u8, MODE> Pin<N, MODE> {
    /// Compile-time check that `N` is a real line — evaluated (forced) from
    /// `Pin::new`.
    const VALID_LINE: () = assert!(N <= 22, "gio_aon only has lines 0..=22");

    /// Reconfigures this line as an output (clears its `IODIR` bit). The
    /// level it starts driving is whatever `DATA` already held for it.
    pub fn into_output(self) -> Pin<N, Output> {
        modify(N, IODIR, false);
        Pin {
            _gio: self._gio,
            _mode: PhantomData,
        }
    }

    /// Reconfigures this line as an input (sets its `IODIR` bit).
    pub fn into_input(self) -> Pin<N, Input> {
        modify(N, IODIR, true);
        Pin {
            _gio: self._gio,
            _mode: PhantomData,
        }
    }

    /// Wraps an already-stolen `GIO_AON` token as line `N` in mode `MODE`,
    /// without touching hardware.
    ///
    /// # Safety
    ///
    /// `MODE` must correctly describe line `N`'s actual current direction.
    pub unsafe fn assume_mode(gio: GIO_AON) -> Self {
        const { Self::VALID_LINE };
        Self {
            _gio: gio,
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
    /// Drives the line low.
    fn set_low(&mut self) -> Result<(), Self::Error> {
        modify(N, DATA, false);
        Ok(())
    }

    /// Drives the line high.
    fn set_high(&mut self) -> Result<(), Self::Error> {
        modify(N, DATA, true);
        Ok(())
    }
}

impl<const N: u8> embedded_hal::digital::StatefulOutputPin for Pin<N, Output> {
    /// Reads back the line's `DATA` bit, which on an output is the level
    /// being driven.
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(read(N, DATA))
    }

    /// The inverse of `is_set_high`.
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!read(N, DATA))
    }
}

impl<const N: u8> embedded_hal::digital::ErrorType for Pin<N, Input> {
    /// Infallible — every operation here is a direct, always-succeeding
    /// register access.
    type Error = core::convert::Infallible;
}

impl<const N: u8> embedded_hal::digital::InputPin for Pin<N, Input> {
    /// True if the line currently reads high.
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(read(N, DATA))
    }

    /// The inverse of `is_high`.
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!read(N, DATA))
    }
}

/// Drives line `n` as an output at `high`, setting the level before the
/// direction so the line never glitches to a stale level on the way.
///
/// For drivers in this crate that own a board line outright, as
/// [`crate::sd`] does the SD card's power and I/O-voltage switches.
pub(crate) fn drive(n: u8, high: bool) {
    modify(n, DATA, high);
    modify(n, IODIR, false);
}

/// Lines in bank 0; bank 1 holds the rest (`brcm,gpio-bank-widths = <17
/// 6>`).
const BANK0_WIDTH: u8 = 17;
/// Bytes per bank: eight 32-bit registers.
const BANK_BYTES: usize = 0x20;
/// `DATA` offset within a bank.
const DATA: usize = 0x04;
/// `IODIR` offset within a bank.
const IODIR: usize = 0x08;

/// Address of `register` in the bank holding line `n`, and `n`'s bit in it.
fn locate(n: u8, register: usize) -> (*mut u32, u32) {
    let (bank, bit) = if n < BANK0_WIDTH {
        (0, n)
    } else {
        (1, n - BANK0_WIDTH)
    };
    let address = GIO_AON::PTR as usize + bank * BANK_BYTES + register;
    (address as *mut u32, 1 << bit)
}

/// Reads line `n`'s bit of `register`.
fn read(n: u8, register: usize) -> bool {
    let (address, bit) = locate(n, register);
    unsafe { address.read_volatile() & bit != 0 }
}

/// Sets or clears line `n`'s bit of `register`.
///
/// A read-modify-write — the block has no set/clear aliases — so the other
/// lines in the bank keep whatever the firmware left them at. It races only
/// with another writer of the same bank register. The firmware's PMIC
/// monitoring on this block is an interrupt, so it should be working the
/// interrupt registers rather than these two.
fn modify(n: u8, register: usize, set: bool) {
    let (address, bit) = locate(n, register);
    unsafe {
        let value = address.read_volatile();
        address.write_volatile(if set { value | bit } else { value & !bit });
    }
}
