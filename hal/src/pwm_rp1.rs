//! Blocking driver for RP1's PWM blocks — the Pi 5 build of [`crate::pwm`].
//!
//! RP1 has two instances of one PWM design, each with four independent
//! channels: [`PWM0`] reaches the header (GPIO12, 13, 18 and 19), and
//! [`PWM1`] drives the fan connector (channel 3 on GPIO45, `FAN_PWM`). It is
//! not the BCM PWM the earlier chips have, so this module's API is its own
//! rather than [`crate::pwm`]'s on those chips: one generic [`Pwm`] over
//! either block, and a [`Channel`] per output implementing
//! [`embedded_hal::pwm::SetDutyCycle`], the trait code written against
//! `embedded-hal` drives.
//!
//! Register layout from the RP1 Peripherals datasheet's PWM chapter, and
//! the sequence from Linux's `pwm-rp1.c`.
//!
//! ## Clock
//!
//! Each block runs from its own RP1 clock (`clk_pwm0`, `clk_pwm1`), which
//! [`Pwm::init`] sets to the 50MHz crystal through a 16-bit integer
//! divider — like the UART's, set rather than assumed, since what the
//! firmware leaves there is not documented. Linux runs both at 50MHz too
//! (`rp1.dtsi`'s `assigned-clock-rates`).
//!
//! ## Mode and duty
//!
//! Channels run in trailing-edge mark-space mode: the counter counts up
//! from 0 and the output is high while the count is below the duty value,
//! a single contiguous pulse per period — the waveform `SetDutyCycle`
//! callers expect, and the one Linux uses.
//!
//! Duty and range writes need no latching: the datasheet gives them their
//! own update strobe, taking effect at the end of the current period. Only
//! the enables, the mode and the inversion go through `GLOBAL_CTRL`'s
//! `SET_UPDATE`, which applies them all on one PWM clock edge.
//!
//! ## The fan
//!
//! The Pi 5's fan connector takes an inverted signal: a low `FAN_PWM` is
//! full speed. The firmware hands over GPIO45 as a software GPIO
//! (`FUNCSEL` 5) with its pad's output buffer disabled and its pull-down
//! on, so the pin floats low and the fan runs flat out — which is what it
//! does from power-on under any kernel that leaves it alone. Linux drives
//! it at a 41,566 ns period (about 24kHz) with the output inverted; the
//! `pwm_fan` example does the same, so a duty fraction there is a speed
//! fraction.
//!
//! The official fan does not start from a standstill at low duty: at 25%
//! it sat at 0 RPM for about three seconds before creeping up, where 50%
//! reached 1,800 RPM within one. Linux's lowest fan level is 75/255, about
//! 29%. Starting it at a higher duty and then dropping avoids the wait.

use core::marker::PhantomData;

use crate::pac::{GPIO, PWM0, PWM1};

/// `GLOBAL_CTRL`: the four channel enables (bits 3:0) and `SET_UPDATE`.
const GLOBAL_CTRL: usize = 0x00;
/// `GLOBAL_CTRL.SET_UPDATE` (bit 31): applies every pending enable, mode
/// and inversion change on one PWM clock edge. Self-clearing.
const SET_UPDATE: u32 = 1 << 31;
/// The four `GLOBAL_CTRL.CHANn_EN` bits.
const ALL_CHANNELS: u32 = 0b1111;

/// `CHANn_CTRL` offset for channel `n`. Each channel's four registers
/// (`CTRL`, `RANGE`, `PHASE`, `DUTY`) are a 16-byte group from `0x14`.
const fn chan_ctrl(n: usize) -> usize {
    0x14 + 16 * n
}
/// `CHANn_RANGE` offset for channel `n`.
const fn chan_range(n: usize) -> usize {
    0x18 + 16 * n
}
/// `CHANn_DUTY` offset for channel `n`.
const fn chan_duty(n: usize) -> usize {
    0x20 + 16 * n
}

/// `CHANn_CTRL.MODE` (bits 2:0) = 1, trailing-edge mark-space modulation.
const MODE_TRAILING_EDGE: u32 = 1;
/// `CHANn_CTRL.INVERT` (bit 3).
const INVERT: u32 = 1 << 3;
/// `CHANn_CTRL.FIFO_POP_MASK` (bit 8), its reset value. Kept set, as Linux
/// does, though nothing here uses the FIFO.
const FIFO_POP_MASK: u32 = 1 << 8;

/// Frequency of RP1's crystal, the source [`Pwm::init`] selects (the same
/// 50MHz the UART runs from).
const CRYSTAL_HZ: u32 = 50_000_000;

/// One of RP1's two PWM blocks: [`PWM0`] or [`PWM1`]. Sealed; it exists
/// to make [`Pwm`] generic over the two.
pub trait Instance: sealed::Instance {
    /// The pins this block's channels can drive.
    type Pin: ChannelPin;
}

/// A pin a PWM channel can drive. Sealed; implemented by [`Pwm0Pin`] and
/// [`Pwm1Pin`], each of which knows its channel and pin function.
pub trait ChannelPin: sealed::ChannelPin + Copy {}

mod sealed {
    /// The private half of [`super::Instance`].
    pub trait Instance {
        /// Address of the block's register file.
        const ADDR: usize;
        /// Offset of the block's `CLK_PWMn_CTRL` in RP1's `clocks_main`.
        const CLK_CTRL: usize;
    }

    /// The private half of [`super::ChannelPin`].
    pub trait ChannelPin {
        /// The GPIO number, the channel (0-3) and the `FUNCSEL` value, in
        /// that order.
        fn route(self) -> (u8, usize, u32);
    }
}

impl sealed::Instance for PWM0 {
    const ADDR: usize = PWM0::ADDR;
    const CLK_CTRL: usize = 0x74;
}
impl Instance for PWM0 {
    type Pin = Pwm0Pin;
}

impl sealed::Instance for PWM1 {
    const ADDR: usize = PWM1::ADDR;
    const CLK_CTRL: usize = 0x84;
}
impl Instance for PWM1 {
    type Pin = Pwm1Pin;
}

/// Which header pin a [`PWM0`] channel drives, from RP1's pin function
/// table. Each pin belongs to one channel, which [`Pwm::channel`] picks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pwm0Pin {
    /// GPIO12, channel 0 (function 0).
    Gpio12,
    /// GPIO13, channel 1 (function 0).
    Gpio13,
    /// GPIO18, channel 2 (function 3).
    Gpio18,
    /// GPIO19, channel 3 (function 3).
    Gpio19,
}

impl ChannelPin for Pwm0Pin {}
impl sealed::ChannelPin for Pwm0Pin {
    fn route(self) -> (u8, usize, u32) {
        match self {
            Self::Gpio12 => (12, 0, 0),
            Self::Gpio13 => (13, 1, 0),
            Self::Gpio18 => (18, 2, 3),
            Self::Gpio19 => (19, 3, 3),
        }
    }
}

/// Which pin a [`PWM1`] channel drives. Only the fan's: PWM1 reaches no
/// header pin.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pwm1Pin {
    /// GPIO45, `FAN_PWM` on the fan connector: channel 3 (function 0), as
    /// the board's device tree wires it (`pwms = <&rp1_pwm1 3 ...>`).
    Gpio45,
}

impl ChannelPin for Pwm1Pin {}
impl sealed::ChannelPin for Pwm1Pin {
    fn route(self) -> (u8, usize, u32) {
        match self {
            Self::Gpio45 => (45, 3, 0),
        }
    }
}

/// Blocking driver for one RP1 PWM block's clock and channels.
pub struct Pwm<P: Instance> {
    _pwm: P,
    /// What [`Self::init`] programmed the block's clock to.
    clock_hz: u32,
}

impl<P: Instance> Pwm<P> {
    /// The largest divisor [`Self::init`] can program: `DIV_INT` is 16 bits.
    pub const MAX_CLOCK_DIVISOR: u16 = u16::MAX;

    /// The PWM clock rate [`Self::init`] produces for `clock_divisor`, with
    /// the same treatment of 0 that `init` gives it (as 1).
    ///
    /// Exact, unlike the BCM chips' PLL-derived PWM clock: the source is a
    /// crystal.
    pub const fn clock_hz(clock_divisor: u16) -> u32 {
        CRYSTAL_HZ / clamp_divisor(clock_divisor) as u32
    }

    /// The [`Self::init`] divisor giving the fastest PWM clock at or below
    /// `target_hz`, clamped to what the hardware can hold.
    pub const fn divisor_for(target_hz: u32) -> u16 {
        if target_hz == 0 {
            return Self::MAX_CLOCK_DIVISOR;
        }
        let divisor = CRYSTAL_HZ.div_ceil(target_hz);
        if divisor > Self::MAX_CLOCK_DIVISOR as u32 {
            Self::MAX_CLOCK_DIVISOR
        } else {
            clamp_divisor(divisor as u16)
        }
    }

    /// Sets the block's clock to the 50MHz crystal divided by
    /// `clock_divisor` (0 counts as 1) and disables all four of its
    /// channels. Doesn't touch any pin: those are routed one channel at a
    /// time, by [`Self::channel`].
    ///
    /// The channels are disabled first because a warm restart (such as
    /// `rpi-loader` jumping to a freshly loaded kernel) can leave a previous
    /// run's channels enabled, and the clock mux is not glitch-free, so the
    /// clock is stopped across the change.
    pub fn init(pwm: P, clock_divisor: u16) -> Self {
        let global = reg::<P>(GLOBAL_CTRL);
        unsafe {
            global.write_volatile(global.read_volatile() & !ALL_CHANNELS);
            global.write_volatile(global.read_volatile() | SET_UPDATE);
        }

        set_clock(P::CLK_CTRL, clamp_divisor(clock_divisor));

        Self {
            _pwm: pwm,
            clock_hz: Self::clock_hz(clock_divisor),
        }
    }

    /// The rate [`Self::init`] set the block's clock to.
    pub fn clock_rate_hz(&self) -> u32 {
        self.clock_hz
    }

    /// Starts `pin`'s channel with a period of `range` PWM clock ticks and
    /// a duty of 0 (output low), then routes `pin` to it. `range` becomes
    /// the returned [`Channel`]'s `max_duty_cycle`, so duty `range` is
    /// always high and duty 0 always low.
    ///
    /// The channel is running before the pin is switched over to it, so the
    /// pin goes straight from whatever it was doing to the channel's
    /// output, low at first.
    ///
    /// # Panics
    ///
    /// If `range` is 0.
    pub fn channel(&self, _gpio: &GPIO, pin: P::Pin, range: u16) -> Channel<'_, P> {
        use sealed::ChannelPin as _;

        assert!(range > 0, "a PWM range must be at least 1");
        let (gpio_n, n, funcsel) = pin.route();

        // The datasheet has the counter count up until it "reaches RANGE"
        // and then reload at 0, with the output high while the count is
        // below `DUTY` -- so `RANGE = range - 1` gives a period of `range`
        // ticks in which duty `range` is high on every tick. Writing
        // `range` itself, as Linux does, would leave a full duty one tick
        // short of always-on.
        unsafe {
            reg::<P>(chan_ctrl(n)).write_volatile(FIFO_POP_MASK | MODE_TRAILING_EDGE);
            reg::<P>(chan_range(n)).write_volatile(u32::from(range) - 1);
            reg::<P>(chan_duty(n)).write_volatile(0);
            let global = reg::<P>(GLOBAL_CTRL);
            global.write_volatile(global.read_volatile() | (1 << n));
            global.write_volatile(global.read_volatile() | SET_UPDATE);
        }

        crate::gpio::select_function(gpio_n, funcsel);

        Channel {
            n,
            _pwm: PhantomData,
        }
    }
}

impl Pwm<PWM1> {
    /// The fan's speed in RPM, as something other than this crate measures
    /// it.
    ///
    /// The value is read from offset `0x3c` of PWM1, which the RP1
    /// datasheet lists as `CHAN2_PHASE` — a channel nothing on the board
    /// uses. The Pi 5's device tree points Linux's `pwm-fan` driver at it
    /// as an RPM readout (`rpm-regmap = <&rp1_pwm1>`, `rpm-offset =
    /// <0x3c>`), and that driver only reads it: something outside the
    /// kernel counts the tachometer pulses (`FAN_TACH`, GPIO29) and writes
    /// the result there. It keeps doing so with no Linux running — the
    /// `pwm_fan` example reads it tracking each speed step, settling around
    /// 4,100-4,350 RPM at full speed — so nothing needs setting up first.
    ///
    /// It lags the fan by a second or so: a step reads low for its first
    /// sample.
    pub fn fan_rpm(&self) -> u32 {
        const FAN_RPM: usize = 0x3c;
        unsafe { reg::<PWM1>(FAN_RPM).read_volatile() }
    }
}

/// A live handle to one PWM channel, borrowed from [`Pwm`] — see
/// [`Pwm::channel`].
pub struct Channel<'a, P: Instance> {
    /// The channel number, 0-3.
    n: usize,
    _pwm: PhantomData<&'a Pwm<P>>,
}

impl<P: Instance> Channel<'_, P> {
    /// Inverts the channel's output: low for the duty fraction of each
    /// period and high for the rest. What the fan connector wants, where a
    /// low signal is full speed — so with this set, a duty fraction is a
    /// speed fraction.
    pub fn set_inverted(&mut self) {
        self.modify_ctrl(INVERT, INVERT);
    }

    /// Undoes [`Self::set_inverted`].
    pub fn clr_inverted(&mut self) {
        self.modify_ctrl(INVERT, 0);
    }

    /// Replaces the `mask` bits of this channel's `CTRL` with `value` and
    /// latches the change with `SET_UPDATE`.
    fn modify_ctrl(&mut self, mask: u32, value: u32) {
        let ctrl = reg::<P>(chan_ctrl(self.n));
        let global = reg::<P>(GLOBAL_CTRL);
        unsafe {
            ctrl.write_volatile((ctrl.read_volatile() & !mask) | value);
            global.write_volatile(global.read_volatile() | SET_UPDATE);
        }
    }
}

impl<P: Instance> embedded_hal::pwm::ErrorType for Channel<'_, P> {
    /// Infallible: every operation is a register write that always lands.
    type Error = core::convert::Infallible;
}

impl<P: Instance> embedded_hal::pwm::SetDutyCycle for Channel<'_, P> {
    /// The `range` [`Pwm::channel`] started this channel with, read back
    /// from the hardware (which holds one less — see `channel`).
    fn max_duty_cycle(&self) -> u16 {
        let range = unsafe { reg::<P>(chan_range(self.n)).read_volatile() };
        (range + 1) as u16
    }

    /// Writes the channel's `DUTY`. It takes effect at the end of the
    /// current period; no latching or delay is needed (see this module's
    /// "Mode and duty").
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        unsafe { reg::<P>(chan_duty(self.n)).write_volatile(u32::from(duty)) };
        Ok(())
    }
}

/// `clock_divisor`, with 0 taken as 1 (an undivided clock).
const fn clamp_divisor(clock_divisor: u16) -> u16 {
    if clock_divisor == 0 {
        1
    } else {
        clock_divisor
    }
}

/// Address of the register at `offset` in block `P`.
fn reg<P: sealed::Instance>(offset: usize) -> *mut u32 {
    (P::ADDR + offset) as *mut u32
}

/// Sets the RP1 clock whose `CTRL` is at `ctrl_offset` in `clocks_main` to
/// the crystal divided by `divisor`, and enables it.
///
/// The same sequence as the UART's (see `uart.rs`), plus the divider:
/// `CTRL` holds the enable (bit 11) and the source mux (`AUXSRC`, bits 9:5,
/// where 2 is `xosc` for the PWM clocks too), and `DIV_INT` and `DIV_FRAC`
/// follow it. Layout from Linux's `clk-rp1.c`.
fn set_clock(ctrl_offset: usize, divisor: u16) {
    const CLOCKS_MAIN: usize = crate::pac::RP1_BASE + 0x0001_8000;
    const ENABLE: u32 = 1 << 11;
    const AUXSRC: u32 = 0x1f << 5;
    const AUXSRC_XOSC: u32 = 2 << 5;

    let ctrl = (CLOCKS_MAIN + ctrl_offset) as *mut u32;
    let div_int = (CLOCKS_MAIN + ctrl_offset + 4) as *mut u32;
    let div_frac = (CLOCKS_MAIN + ctrl_offset + 8) as *mut u32;
    unsafe {
        let base = ctrl.read_volatile() & !(ENABLE | AUXSRC);
        ctrl.write_volatile(base);
        div_int.write_volatile(u32::from(divisor));
        div_frac.write_volatile(0);
        ctrl.write_volatile(base | AUXSRC_XOSC);
        ctrl.write_volatile(base | AUXSRC_XOSC | ENABLE);
    }
}
