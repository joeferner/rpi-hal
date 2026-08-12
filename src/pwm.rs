//! Blocking driver for the BCM System PWM controller — two independent
//! duty-cycle channels sharing one clock. Named `RNG1`/`DAT1`/`CTL`'s
//! `*1` bits and `RNG2`/`DAT2`/`CTL`'s `*2` bits in the PAC (matching
//! this module's [`Channel1`](crate::pwm::Channel1)/
//! [`Channel2`](crate::pwm::Channel2)); the BCM2835 ARM
//! Peripherals datasheet calls the same two channels "PWM0"/"PWM1"
//! (0-indexed) instead — same hardware, different numbering.
//!
//! ## Clock
//!
//! Both channels are driven from one shared PWM clock, configured via
//! the separate `CM_PWM` clock manager peripheral (not part of the
//! `PWM0` register block itself) in [`Pwm::init`](crate::pwm::Pwm::init).
//! Sourced from `PLLD_per` (a PLL-derived clock most bare-metal Pi PWM
//! audio implementations also use in practice — this PAC's own enum
//! happens to label the same encoding `pllc()`, a naming quirk of this
//! crate's SVD, not a different clock), not the board's crystal
//! oscillator. This is a weaker claim than it might look: real
//! hardware testing did show the oscillator failing to produce any
//! output while `PLLD_per` worked, but every oscillator attempt during
//! that testing also happened to be a single `CTL` write, and every
//! working `PLLD_per` attempt happened to include a second one (see
//! "Enable sequence" below) — so "the source matters" and "the double
//! write matters" were never cleanly isolated from each other.
//! `PLLD_per` is kept here because it's confirmed working in
//! combination with the double write, not because the oscillator is
//! confirmed broken on its own. Because of the PLL, `Pwm::init`'s
//! `clock_divisor` can't be treated as yielding an exactly-knowable
//! rate the way a crystal would (see `spi.rs`'s/`i2c.rs`'s "core
//! clock" caution for the same reasoning) — `PLLD_per`'s frequency is
//! commonly cited as 500MHz but isn't independently confirmed by this
//! crate, so treat the resulting PWM clock rate as nominal, not exact.
//!
//! Each channel's `range` (its
//! [`embedded_hal::pwm::SetDutyCycle::max_duty_cycle`]) divides that
//! PWM clock further into the channel's actual output period —
//! `pwm_clock_hz / range`.
//!
//! ## Mode
//!
//! Both channels run in the hardware's default "PDM/balanced
//! algorithm" mode (`MSEN` clear) — not M/S (mark:space) mode, which
//! an earlier version of this driver used instead. `MSEN` looked like
//! the obviously-correct choice (a plain, contiguous high-then-low
//! pulse per period, matching what `SetDutyCycle` callers expect), but
//! real hardware produced no output at all with it set even though
//! every relevant register (`CM_PWM`'s `CS`/`DIV`, `PWM0`'s `CTL`/
//! `RNG`/`DAT`, GPIO's ALT function) read back exactly as configured.
//! Clearing it, with nothing else changed at the time, got output
//! moving — though see "Enable sequence" below for a wrinkle in what
//! "nothing else changed" turned out to mean. The tradeoff: this mode
//! still averages to `DAT`/`RNG` (so `SetDutyCycle`'s duty-cycle
//! semantics still hold), but spreads the high/low ticks across the
//! period instead of one contiguous run — fine for dimming an LED or
//! driving an RC-filtered analog level, wrong for anything needing a
//! single clean pulse of a specific width per period (e.g. a hobby
//! servo).
//!
//! Also FIFO-less (`USEF` clear) either way: `DAT` drives the output
//! directly, so a [`embedded_hal::pwm::SetDutyCycle::set_duty_cycle`]
//! call takes effect on the very next period with nothing to drain
//! first.
//!
//! ## Enable sequence
//!
//! `channel1`/`channel2` write `CTL` twice — the exact same field
//! values both times — with `settle_delay` in between. Confirmed on
//! real hardware, not derivable from the register spec: a single write
//! left `STA.STA1`/`STA2` (the peripheral's own internal per-channel
//! output state) never advancing at all even after tens of millions of
//! polled samples with nothing else touched, regardless of clock
//! source or `MSEN`. Re-issuing the identical write once the clock had
//! been running a while is what actually got the counter moving. The
//! true minimum delay isn't characterized — `settle_delay` is a
//! generously long busy-wait, not a calibrated one.
//!
//! ## Audio
//!
//! [`Pwm::channel1`](crate::pwm::Pwm::channel1)/
//! [`Pwm::channel2`](crate::pwm::Pwm::channel2) drive a single held duty
//! cycle —
//! the CPU writes `DAT` and the output stays there. Audio instead needs
//! a *stream* of values changing at the sample rate, far faster than the
//! CPU can poll a register. [`Pwm::audio`](crate::pwm::Pwm::audio) configures
//! both channels in
//! the hardware's FIFO mode (`USEF` set): each channel takes its next
//! output value from the shared 16-entry FIFO rather than from `DAT`,
//! and the FIFO raises a DMA request (DREQ) whenever it has room. Feeding
//! that DREQ from the DMA controller (`crate::dma`) streams samples
//! straight from a RAM buffer into the FIFO with no per-sample CPU work.
//!
//! The FIFO is shared between the two channels: with both enabled, the
//! hardware hands successive FIFO words to channel 1, channel 2, channel
//! 1, … so an interleaved two-channel sample buffer plays as stereo
//! (which channel is the left/right jack contact is board-dependent — see
//! [`Pwm::audio`](crate::pwm::Pwm::audio)). Each sample is an unsigned duty
//! value in `0..=range`; a caller converts signed PCM to that range (see
//! [`pcm_to_duty`](crate::pwm::pcm_to_duty)) and
//! picks `range` for the bit depth it wants.
//!
//! The sample rate is not set directly — it falls out of the shared PWM
//! clock and `range` as `pwm_clock_hz / range` (one FIFO word is consumed
//! per channel period).
//! [`Pwm::audio_clock_divisor`](crate::pwm::Pwm::audio_clock_divisor) inverts
//! that to pick the [`Pwm::init`](crate::pwm::Pwm::init) divisor for a target
//! sample rate; like every rate in
//! this module it's nominal, not exact (integer divisor, PLL-derived
//! clock — see the "Clock" section above).
//!
//! Because the analog output is still PDM/balanced and unfiltered (see
//! "Mode"), the raw pin carries the high-frequency PWM carrier on top of
//! the audio; it needs an RC low-pass (or the board's own analog-audio
//! filter on the 3.5 mm jack pins —
//! [`Channel1Pin::Gpio40`](crate::pwm::Channel1Pin::Gpio40)/
//! [`Channel2Pin::Gpio45`](crate::pwm::Channel2Pin::Gpio45)) to recover a
//! clean signal.

use crate::pac::{CM_PWM, GPIO, PWM0};

/// The DMA DREQ (pacing) number for the PWM controller, passed to
/// [`crate::dma::Channel::write_peripheral`] so the DMA engine only
/// pushes a sample when the PWM FIFO has room. Fixed by the SoC.
pub const AUDIO_DREQ: u8 = 5;

/// VideoCore *bus* address of the PWM FIFO input register (`FIF1`), the
/// fixed destination a DMA channel streams audio samples into — pass it
/// as `dest_bus` to [`crate::dma::Channel::write_peripheral`]. This is
/// the PWM0 block's bus base `0x7E20_C000` plus `FIF1`'s `0x18` offset;
/// it's the bus alias of ARM physical `0x3F20_C018`, the address a bus
/// master (the DMA engine) must use rather than the ARM physical one.
pub const AUDIO_FIFO_BUS_ADDRESS: u32 = 0x7e20_c018;

/// Converts a signed 16-bit PCM sample (the usual audio representation,
/// `i16::MIN..=i16::MAX` with silence at `0`) to an unsigned PWM duty in
/// `0..range`, the form the FIFO expects. The full `i16` span maps
/// linearly onto the duty span, so silence lands at `range / 2` (the
/// mid-rail the PDM output averages to). Feed the result into an audio
/// fill loop as the sample value.
///
/// `range` is the value passed to [`Pwm::audio`]/[`Pwm::audio_mono`]
/// (a channel's `max_duty_cycle`). The result never quite reaches `range`
/// itself — full positive scale maps to `range - 1` — which keeps it a
/// valid duty and is inaudible at any realistic `range`.
pub const fn pcm_to_duty(sample: i16, range: u16) -> u16 {
    // Shift the signed sample into `0..=65535`, then scale that 16-bit
    // span down to `0..range` (a right shift by 16 = divide by 65536).
    let unsigned = (sample as i32 + 32768) as u32;
    ((unsigned * range as u32) >> 16) as u16
}

/// Which GPIO pin [`Pwm::channel1`] (or channel 1 of [`Pwm::audio`])
/// routes its output to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channel1Pin {
    /// GPIO12 (ALT0).
    Gpio12,
    /// GPIO18 (ALT5).
    Gpio18,
    /// GPIO40 (ALT0). Not on the 40-pin header — this is one of the pins
    /// wired internally to the board's analog audio circuit, so it's the
    /// pin to pick for driving the 3.5 mm jack (see [`Pwm::audio`]). On the
    /// board this was brought up on it feeds the **right** jack contact (a
    /// mono stream on channel 1 alone came out the right speaker); the
    /// exact GPIO40/41/45→contact mapping can still vary by board revision.
    Gpio40,
}

/// Which GPIO pin [`Pwm::channel2`] (or channel 2 of [`Pwm::audio`])
/// routes its output to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channel2Pin {
    /// GPIO13 (ALT0).
    Gpio13,
    /// GPIO19 (ALT5).
    Gpio19,
    /// GPIO45 (ALT0). Like [`Channel1Pin::Gpio40`], an internal analog-
    /// audio pin rather than a header pin — the channel-2 counterpart for
    /// the 3.5 mm jack (see [`Pwm::audio`]). Since channel 1 (GPIO40) was
    /// observed on the **right** contact, this (channel 2) is the **left**
    /// contact — inferred as the complementary contact, not independently
    /// confirmed; same board-revision caveat.
    Gpio45,
}

/// Blocking driver for the PWM controller's shared clock and both its
/// channels.
pub struct Pwm {
    pwm0: PWM0,
}

impl Pwm {
    /// Configures `CM_PWM` to run from `PLLD_per` at (nominally)
    /// `500_000_000 / clock_divisor` Hz (the fractional divider stays
    /// 0 — integer division only) and enables it — see this module's
    /// doc comment on why `PLLD_per`, not the oscillator. Doesn't touch
    /// GPIO or either channel — unlike `spi.rs`'s/`i2c.rs`'s single
    /// fixed pin set, each channel here has two possible GPIO pins and
    /// a caller may only want one channel at all, so pin muxing and
    /// channel setup are deferred to [`Self::channel1`]/
    /// [`Self::channel2`].
    ///
    /// Kills any clock already running on `CM_PWM` first — the
    /// datasheet requires disabling the generator before changing its
    /// source or divisor, and this crate can't assume GPU firmware
    /// left it disabled.
    ///
    /// Waits for `BUSY` to actually assert before returning, not just
    /// for it to clear after `KILL` — `channel1`/`channel2` enable
    /// `PWEN` within a handful of instructions of this returning, and
    /// doing that before the clock has genuinely started ticking risks
    /// the channel latching into a state where its internal counter
    /// never advances even once the clock catches up a few cycles
    /// later.
    pub fn init(pwm0: PWM0, cm_pwm: CM_PWM, clock_divisor: u16) -> Self {
        cm_pwm.cs().write(|w| w.kill().set_bit().passwd().passwd());
        while cm_pwm.cs().read().busy().bit_is_set() {}

        unsafe {
            cm_pwm.div().write(|w| {
                w.divi().bits(clock_divisor);
                w.divf().bits(0);
                w.passwd().passwd()
            });
        }
        unsafe {
            cm_pwm.cs().write(|w| {
                w.src().pllc();
                w.mash().bits(0);
                w.passwd().passwd()
            });
        }
        cm_pwm.cs().write(|w| {
            w.src().pllc();
            w.enab().set_bit();
            w.passwd().passwd()
        });
        while cm_pwm.cs().read().busy().bit_is_clear() {}

        Self { pwm0 }
    }

    /// Routes `pin` to channel 1 (ALT function `PWM0_0` either way —
    /// see this module's doc comment on the datasheet's 0-indexed
    /// naming), sets its period to `range` PWM clock ticks, and
    /// enables it in the default PDM/balanced, FIFO-less mode (see
    /// this module's doc comment on why not M/S mode). `range` becomes
    /// the returned [`Channel1`]'s `max_duty_cycle`.
    pub fn channel1(&self, gpio: &GPIO, pin: Channel1Pin, range: u16) -> Channel1<'_> {
        route_channel1_pin(gpio, pin);

        unsafe {
            self.pwm0.rng1().write(|w| w.bits(range as u32));
        }
        // `.modify()`, not `.write()`, so channel 2's bits in this same
        // register survive -- but that also means `MSEN1` must be
        // *cleared* explicitly, not just left unmentioned: `CTL` isn't
        // reset by anything this crate controls, and a warm reboot
        // (e.g. `rpi-loader` jumping straight to a freshly loaded
        // kernel with no power cycle) can leave a previous run's
        // `MSEN1=1` still latched. Merely omitting `.msen1()` here
        // would silently inherit that stale bit and reintroduce the
        // exact M/S-mode bug this module's doc comment describes.
        //
        // Written twice, with `settle_delay` in between -- confirmed
        // on real hardware, not derivable from the register spec: a
        // single write here (even with every bit exactly as below)
        // left the channel's internal counter never advancing at all.
        // Re-writing the *same* values again, once the clock has had
        // a while to run, is what actually gets it moving; see this
        // module's doc comment.
        self.pwm0.ctl().modify(|_, w| {
            w.mode1().pwm();
            w.msen1().clear_bit();
            w.pwen1().set_bit()
        });
        settle_delay();
        self.pwm0.ctl().modify(|_, w| {
            w.mode1().pwm();
            w.msen1().clear_bit();
            w.pwen1().set_bit()
        });

        Channel1 { pwm0: &self.pwm0 }
    }

    /// Channel 2 counterpart of [`Self::channel1`] (ALT function
    /// `PWM0_1`).
    pub fn channel2(&self, gpio: &GPIO, pin: Channel2Pin, range: u16) -> Channel2<'_> {
        route_channel2_pin(gpio, pin);

        unsafe {
            self.pwm0.rng2().write(|w| w.bits(range as u32));
        }
        // See `channel1`'s equivalent comments: `MSEN2` must be cleared
        // explicitly, and the write done twice with `settle_delay` in
        // between, for the same reasons.
        self.pwm0.ctl().modify(|_, w| {
            w.mode2().pwm();
            w.msen2().clear_bit();
            w.pwen2().set_bit()
        });
        settle_delay();
        self.pwm0.ctl().modify(|_, w| {
            w.mode2().pwm();
            w.msen2().clear_bit();
            w.pwen2().set_bit()
        });

        Channel2 { pwm0: &self.pwm0 }
    }

    /// Configures both channels for DMA-fed stereo audio playback and
    /// returns a [`PwmAudio`] handle. Routes `channel1` and `channel2` to
    /// their pins (the analog-jack pins [`Channel1Pin::Gpio40`]/
    /// [`Channel2Pin::Gpio45`] for the 3.5 mm output), sets both channels'
    /// period to `range` ticks, and puts both in FIFO mode (`USEF`) so
    /// each takes its output from the shared FIFO — see this module's
    /// "Audio" doc section for how the interleaved stream and sample rate
    /// work.
    ///
    /// The FIFO round-robins words to channel 1 then channel 2, so an
    /// interleaved sample buffer plays as stereo: even words → channel 1,
    /// odd words → channel 2. Which physical jack contact each channel
    /// drives is board-dependent — on the board this was brought up on,
    /// channel 1 ([`Channel1Pin::Gpio40`]) is the right contact and
    /// channel 2 the left. The parameters are named by channel, not
    /// left/right, for that reason.
    ///
    /// Clears any stale FIFO contents, enables the PWM DMA interface, and
    /// starts both channels. The caller still owns the actual sample
    /// streaming: pair the returned handle's [`PwmAudio::fifo_bus_address`]
    /// and [`PwmAudio::dreq`] with a DMA channel
    /// ([`crate::dma::Channel::write_peripheral`]) to feed the FIFO.
    ///
    /// Uses the same double-`CTL`-write-with-settle enable sequence as
    /// [`Self::channel1`]/[`Self::channel2`], for the reason this module's
    /// "Enable sequence" doc section describes.
    pub fn audio(
        &self,
        gpio: &GPIO,
        channel1: Channel1Pin,
        channel2: Channel2Pin,
        range: u16,
    ) -> PwmAudio<'_> {
        route_channel1_pin(gpio, channel1);
        route_channel2_pin(gpio, channel2);
        self.start_audio(range, true);
        PwmAudio { pwm0: &self.pwm0 }
    }

    /// Mono counterpart of [`Self::audio`]: configures only channel 1 (on
    /// `pin`) for DMA-fed audio and leaves channel 2 alone. With a single
    /// channel in FIFO mode the hardware routes *every* FIFO word to it, so
    /// the caller streams one duty value per sample with no left/right
    /// interleaving — half the data of the stereo path. Otherwise identical
    /// to [`Self::audio`]: same `range` semantics, same FIFO/DMA wiring,
    /// same [`PwmAudio`] handle to pair with a DMA channel.
    ///
    /// Only one of the analog-jack contacts is driven (whichever `pin`
    /// feeds); to hear the same mono signal on both, use [`Self::audio`]
    /// with the sample duplicated to left and right instead.
    pub fn audio_mono(&self, gpio: &GPIO, pin: Channel1Pin, range: u16) -> PwmAudio<'_> {
        route_channel1_pin(gpio, pin);
        self.start_audio(range, false);
        PwmAudio { pwm0: &self.pwm0 }
    }

    /// Shared FIFO/audio bring-up for [`Self::audio`] and
    /// [`Self::audio_mono`]: sets the channel range(s), enables the PWM DMA
    /// interface, clears the FIFO, and starts channel 1 (and channel 2 when
    /// `stereo`) in FIFO mode.
    ///
    /// When not `stereo`, channel 2's `USEF`/`PWEN` are cleared
    /// *explicitly* rather than left untouched: `CTL` isn't reset by
    /// anything this crate controls, so a warm reboot could leave a
    /// previous stereo run's channel-2 bits set, and a still-enabled
    /// channel 2 would steal alternate FIFO words from the mono stream
    /// (same warm-reboot reasoning as `channel1`'s `MSEN` handling).
    fn start_audio(&self, range: u16, stereo: bool) {
        unsafe {
            self.pwm0.rng1().write(|w| w.bits(range as u32));
            if stereo {
                self.pwm0.rng2().write(|w| w.bits(range as u32));
            }
        }

        // Enable the PWM's DMA interface: raise a DREQ (and PANIC) when the
        // shared FIFO drops to these fill levels so the DMA engine keeps it
        // topped up. 7/7 are the reset defaults and leave ample slack in the
        // 16-entry FIFO.
        self.pwm0.dmac().write(|w| {
            w.enab().set_bit();
            unsafe {
                w.panic().bits(7);
                w.dreq().bits(7);
            }
            w
        });

        // Clear the FIFO, put the enabled channel(s) in FIFO mode, and start
        // them. `MSEN` stays clear (PDM/balanced) exactly as the duty-cycle
        // path does, and must be cleared explicitly for the same warm-reboot
        // reason `channel1` documents. Written twice with `settle_delay`
        // between — see this module's "Enable sequence" section.
        for _ in 0..2 {
            self.pwm0.ctl().modify(|_, w| {
                w.clrf1().set_bit();
                w.mode1().pwm();
                w.msen1().clear_bit();
                w.usef1().set_bit();
                w.pwen1().set_bit();
                w.mode2().pwm();
                w.msen2().clear_bit();
                if stereo {
                    w.usef2().set_bit();
                    w.pwen2().set_bit();
                } else {
                    w.usef2().clear_bit();
                    w.pwen2().clear_bit();
                }
                w
            });
            settle_delay();
        }
    }

    /// Picks the [`Self::init`] `clock_divisor` that yields (nominally) a
    /// `sample_rate`-Hz audio stream at the given `range`, inverting the
    /// `sample_rate = pwm_clock_hz / range` relationship this module's
    /// "Audio" section describes: `divisor = 500_000_000 / (sample_rate *
    /// range)`, using `PLLD_per`'s nominal 500MHz.
    ///
    /// Integer division makes the result — and therefore the real sample
    /// rate — approximate, not exact (see the "Clock" section). Clamped to
    /// at least 1 so a too-high `sample_rate * range` product can't yield a
    /// zero divisor.
    pub const fn audio_clock_divisor(sample_rate: u32, range: u16) -> u16 {
        let product = sample_rate as u64 * range as u64;
        if product == 0 {
            return 1;
        }
        let divisor = 500_000_000u64 / product;
        if divisor < 1 {
            1
        } else if divisor > u16::MAX as u64 {
            u16::MAX
        } else {
            divisor as u16
        }
    }
}

/// Busy-wait used only as the settling delay between `channel1`'s/
/// `channel2`'s first and second `CTL` writes (see their doc
/// comments). Not calibrated against a real time base — no `Timer`
/// reference is available here — just a generously long instruction
/// count matching what was confirmed sufficient on real hardware; the
/// true minimum required delay hasn't been characterized.
fn settle_delay() {
    for _ in 0..20_000_000u32 {
        unsafe { core::arch::asm!("nop") };
    }
}

/// Muxes `pin` to channel 1's output (ALT function `PWM0_0`). Shared by
/// [`Pwm::channel1`] and [`Pwm::audio`] so the pin table lives in one
/// place.
fn route_channel1_pin(gpio: &GPIO, pin: Channel1Pin) {
    match pin {
        Channel1Pin::Gpio12 => gpio.gpfsel1().modify(|_, w| w.fsel12().pwm0_0()),
        Channel1Pin::Gpio18 => gpio.gpfsel1().modify(|_, w| w.fsel18().pwm0_0()),
        // Same ALT0 encoding (value 4) on both chips, but BCM2711's PAC
        // names it PWM1_0, not PWM0_0 -- confirmed by diffing
        // `bcm2711-lpa` against `bcm2837-lpa`'s generated source, not
        // assumed. Only the encoding is established that way, though:
        // whether GPIO40 reaches the audio jack on a real Pi 4 board is
        // untested, so this routing is a claim about the register, not
        // about where the signal comes out.
        #[cfg(not(feature = "bcm2711"))]
        Channel1Pin::Gpio40 => gpio.gpfsel4().modify(|_, w| w.fsel40().pwm0_0()),
        #[cfg(feature = "bcm2711")]
        Channel1Pin::Gpio40 => gpio.gpfsel4().modify(|_, w| w.fsel40().pwm1_0()),
    }
}

/// Muxes `pin` to channel 2's output (ALT function `PWM0_1`). Channel-2
/// counterpart of [`route_channel1_pin`].
fn route_channel2_pin(gpio: &GPIO, pin: Channel2Pin) {
    match pin {
        Channel2Pin::Gpio13 => gpio.gpfsel1().modify(|_, w| w.fsel13().pwm0_1()),
        Channel2Pin::Gpio19 => gpio.gpfsel1().modify(|_, w| w.fsel19().pwm0_1()),
        // Unlike Gpio40 above, GPIO45's ALT0 keeps the `pwm0_1` name on
        // both chips -- BCM2711 only renamed GPIO41's (which this crate
        // doesn't use), confirmed by diffing the generated PAC source.
        Channel2Pin::Gpio45 => gpio.gpfsel4().modify(|_, w| w.fsel45().pwm0_1()),
    }
}

/// A live handle to PWM channel 1, borrowed from [`Pwm`] — see
/// [`Pwm::channel1`].
pub struct Channel1<'a> {
    pwm0: &'a PWM0,
}

impl embedded_hal::pwm::ErrorType for Channel1<'_> {
    /// Infallible — every operation here is a direct, always-succeeding
    /// register write.
    type Error = core::convert::Infallible;
}

impl embedded_hal::pwm::SetDutyCycle for Channel1<'_> {
    /// The `range` [`Pwm::channel1`] configured this channel with —
    /// read back from `RNG1` rather than cached separately, so it can
    /// never drift out of sync with the hardware.
    fn max_duty_cycle(&self) -> u16 {
        self.pwm0.rng1().read().bits() as u16
    }

    /// Writes `DAT1` directly.
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        unsafe {
            self.pwm0.dat1().write(|w| w.bits(duty as u32));
        }
        Ok(())
    }
}

/// A live handle to PWM channel 2, borrowed from [`Pwm`] — see
/// [`Pwm::channel2`].
pub struct Channel2<'a> {
    pwm0: &'a PWM0,
}

impl embedded_hal::pwm::ErrorType for Channel2<'_> {
    /// Infallible — same as [`Channel1`]'s.
    type Error = core::convert::Infallible;
}

impl embedded_hal::pwm::SetDutyCycle for Channel2<'_> {
    /// See [`Channel1`]'s `max_duty_cycle`.
    fn max_duty_cycle(&self) -> u16 {
        self.pwm0.rng2().read().bits() as u16
    }

    /// See [`Channel1`]'s `set_duty_cycle`.
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        unsafe {
            self.pwm0.dat2().write(|w| w.bits(duty as u32));
        }
        Ok(())
    }
}

/// A live handle to both PWM channels configured for DMA-fed audio,
/// borrowed from [`Pwm`] — see [`Pwm::audio`].
///
/// It doesn't stream samples itself; it exposes the FIFO destination and
/// DREQ number a DMA channel needs
/// ([`crate::dma::Channel::write_peripheral`]). Dropping it tears the
/// audio setup back down (see [`Drop`]).
pub struct PwmAudio<'a> {
    pwm0: &'a PWM0,
}

impl PwmAudio<'_> {
    /// The bus address of the PWM FIFO, the fixed DMA destination for
    /// samples — the value of [`AUDIO_FIFO_BUS_ADDRESS`], offered here so
    /// the handle carries everything the DMA side needs.
    pub fn fifo_bus_address(&self) -> u32 {
        AUDIO_FIFO_BUS_ADDRESS
    }

    /// The PWM's DMA DREQ number, which paces the transfer — the value of
    /// [`AUDIO_DREQ`].
    pub fn dreq(&self) -> u8 {
        AUDIO_DREQ
    }
}

impl Drop for PwmAudio<'_> {
    /// Stops both channels and disables the PWM DMA interface, so the FIFO
    /// stops draining and stops raising DREQs once audio playback is done.
    fn drop(&mut self) {
        self.pwm0.ctl().modify(|_, w| {
            w.pwen1().clear_bit();
            w.usef1().clear_bit();
            w.pwen2().clear_bit();
            w.usef2().clear_bit()
        });
        self.pwm0.dmac().modify(|_, w| w.enab().clear_bit());
    }
}
