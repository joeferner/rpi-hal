//! When a zero duty stops a PWM channel, and when it does not.
//!
//! # What is being chased
//!
//! An application driving a piezo through this channel found that its buzzer
//! kept sounding a loud continuous tone after the pattern had ended — for over
//! thirty seconds, with `DAT1` reading back as `0` the whole time. Writing the
//! same zero a second time, with a few milliseconds in between, silenced it.
//!
//! The first reading of that was "one zero-duty write is ignored". **This case
//! measured the pin and falsified it**: a single write stops the channel, and
//! `pwm_zero_duty_stops_output` is the proof.
//!
//! # The defect, as measured
//!
//! **Two `DAT` writes too close together are both lost, and the channel goes
//! on emitting the duty that was in force before either of them.** One write
//! is fine. Two 15 ms apart are fine.
//!
//! **There is no sharp boundary.** Two single-trial sweeps disagreed about the
//! two gaps either side of where the edge appeared to be — 2 µs wedged and
//! 4 µs was clear on one run, and they swapped on the next — so each looked
//! like a clean threshold in isolation and they cannot both be one.
//!
//! Read as a *probability* the picture is consistent: near the boundary the
//! outcome turns on where the second write lands within the PWM clock cycle,
//! which a gap measured from the first write does not determine. That is what
//! a clock-domain crossing looks like from software, and it is why
//! [`sweep_gap`] runs [`GAP_TRIALS`] trials per gap and reports a rate. A
//! single trial measures the phase it happened to draw.
//!
//! It is swept at two clocks ([`TARGET_CLOCK_HZ`] and [`FAST_CLOCK_HZ`], 4×
//! apart) because a risk zone measured in clock cycles scales with the clock
//! and one measured in time does not — and a driver's wait has to be computed
//! from the divisor in the first case and can be a constant in the second.
//! Getting that backwards is only visibly wrong at a clock far from the one it
//! was measured at, which is the kind of bug that ships.
//!
//! # The measurement
//!
//! Failures out of eight trials per gap:
//!
//! ```text
//! gap      125kHz (tick 8us)   500kHz (tick 2us)
//! 0us      7/8                 6/8
//! 1us      7/8                 3/8
//! 2us      6/8                 1/8
//! 3us      5/8                 0/8
//! 4us      5/8                 0/8
//! 6us      2/8                 0/8
//! 8us      0/8                 0/8
//! 12-24us  0/8                 0/8
//! ```
//!
//! **It scales with the clock.** Clean from 8 µs at 125 kHz and from 3 µs at
//! 500 kHz; a zone fixed in time would clear at the same microsecond count at
//! both. In ticks that is 1.0 and 1.5, so the risk zone runs to somewhere
//! between one and two PWM clock cycles — which is the documented BCM2835
//! figure, once "within two peripheral clock cycles" is read as *at risk*
//! rather than *always lost*.
//!
//! **The rate decays rather than falling off a cliff**, and that matters more
//! than where the last failure sits: 0/8 is not 0 %, so a wait placed at the
//! first clean gap would sit at the edge of a distribution with a tail. Two
//! clock cycles clears the last observed failure at both clocks and has the
//! precedent behind it, which is what a fix should use.
//!
//! That is what the application's failing path did — it wrote zero once on
//! leaving its pattern and again microseconds later on consuming the stop
//! request that had interrupted it. `pwm_rapid_second_zero_write_stops_output`
//! reproduces it with two zeroes:
//!
//! ```text
//! 152 transitions in 20ms, 1960 of 64512 samples high
//! ```
//!
//! Both numbers are the *previous* duty exactly. 1960/64512 is 3.038 % against
//! a commanded 38/1250 = 3.04 %, and 152 transitions in 20 ms is 76 pulses,
//! which is 38 × 125000 / 1250 = 3800 Hz. The channel is not degraded or
//! half-updated; it is still running the duty it was told to abandon.
//!
//! `pwm_rapid_differing_writes_stop_output` then shows the values have nothing
//! to do with it. Writing 19 and then 0 back to back leaves the channel
//! emitting **38** — 1959/64511 = 3.036 %, neither of the two values written.
//! So a driver cannot dedupe its way out: nothing about *what* is written
//! distinguishes the failing case from the working one, only *when*.
//!
//! # This is not a silencing bug
//!
//! Silence is only where it was noticed, because a stuck tone is audible and a
//! stuck duty is not. Stated generally: **duty updates issued too quickly are
//! dropped, and they take the update before them with them.** Any consumer
//! ramping a duty — an LED fade, a soft start, a sweep — is exposed to being
//! frozen on a stale value, with no error and nothing to detect it by: `DAT`
//! reads back exactly what was written the whole time.
//!
//! # It is a known family of fault on this chip
//!
//! BCM2835 peripherals clocked separately from the CPU have a documented
//! clock-domain-crossing hazard on register writes. Mainline's SD host driver
//! carries a workaround for the Arasan controller losing "the content of
//! successive writes to registers that are within two SD-card clock cycles of
//! each other", using a `udelay` between them — the same shape as what is
//! measured here, on a different peripheral.
//!
//! Read as "writes within about two peripheral clock cycles are **at risk**"
//! rather than "are lost", that matches what is measured here: at
//! [`TARGET_CLOCK_HZ`] two cycles is 16 µs and every gap tested below it has
//! failed on some run, while at [`FAST_CLOCK_HZ`] two cycles is 4 µs and the
//! failures stop much sooner.
//!
//! It remains a prior on the *kind* of fault rather than evidence about this
//! register — no equivalent constraint on `DAT` is documented anywhere found —
//! but it is the reason to expect the remedy to be a few microseconds of
//! separation, which is what makes this fixable behind `set_duty_cycle` rather
//! than something every caller has to know.
//!
//! Note that [`Pwm::channel1`] already needs a write-settle-write at the other
//! end of the channel's life: it writes `CTL` twice with a settling delay
//! between, because a single write "left the channel's internal counter never
//! advancing at all". Same peripheral, and a fault in the same family — this
//! one is a register write inside a period being *swallowed* rather than one
//! failing to start the counter.
//!
//! # No fixture, and why that is not a compromise
//!
//! The question is entirely on-chip — *is this pin still switching?* — so the
//! board can answer it about itself. `GPLEV0` reports a pin's electrical
//! level regardless of which alternate function is driving it, so this samples
//! the PWM output without disturbing the mux that the PWM needs.
//!
//! That makes this self-checking like `hil_smoke`: no wire, no fixture
//! capability, nothing on the bench to arrange. The fixture's PIO timestamping
//! would give a better *waveform*, but it would need the marker line moved
//! onto a PWM-capable pin, and it would not answer anything this cannot.
//!
//! The measurement is a transition count rather than a level. A stuck channel
//! and a silent one are both "a pin", and which level a stuck one happens to
//! rest at is not something to depend on; whether the level *changes* is
//! unambiguous in both directions.
//!
//! # The fix, and what this case now guards
//!
//! `rpi_hal::pwm`'s `set_duty_cycle` holds off for two PWM clock cycles after
//! writing `DAT`, sized from the clock the channel was configured with. So the
//! five assertions below all go through the driver and all pass: they are the
//! regression test for the guarantee a caller gets, which is that two duty
//! writes in a row both take effect however close together they are issued.
//!
//! The gap sweep at the end cannot go through the driver — preventing exactly
//! that gap is the driver's job — so it writes `DAT1` through the PAC
//! directly. It is therefore still a measurement of the *hardware*, and it is
//! what says whether the two cycles the driver waits are still enough.
//!
//! # What the five cases mean together
//!
//! 1. `pwm_output_switches` — a tone duty makes the pin switch. If this fails,
//!    the rest are measuring nothing, so it exists to make that visible rather
//!    than to be interesting.
//! 2. `pwm_zero_duty_stops_output` — one write, then silence. Passes; this is
//!    the case that killed the original explanation.
//! 3. `pwm_rapid_second_zero_write_stops_output` — two `set_duty_cycle` calls
//!    back to back with nothing between them. This is the one that failed
//!    before the driver settled its writes, and the one to watch: it is what
//!    a caller does without thinking about it.
//! 4. `pwm_spaced_second_zero_write_stops_output` — two writes 15 ms apart.
//!    Passed even with the defect present, so it is the control that says a
//!    failure of case 3 is about spacing rather than about writing zero twice.
//!
//! Each of 2-4 re-arms the tone first and confirms the channel is switching
//! before it does anything, so every one of them is an experiment on a running
//! channel rather than on whatever the previous case left behind. Without that
//! a case following a successful silence would be writing zero to an already
//! stopped channel and reporting a pass for it.
//!
//! # GPIO12
//!
//! Channel 1's ALT0 pin, header pin 32, and nothing on the bench is wired to
//! it — see the pin map in the bench README, which uses GPIO4, 14 and 15 and
//! nothing else. It is also the pin the application that found this drives,
//! so the case reproduces that configuration rather than a near equivalent.
//!
//! The clock and range are that application's too: a 125 kHz PWM clock with a
//! range of 1250, giving a 10 ms period. Whether the defect depends on those
//! is **not established** — it has only ever been reproduced at these
//! settings, and a case that quietly changed them would be a different
//! experiment wearing this one's name.

#![no_std]
#![no_main]

use embedded_hal::pwm::SetDutyCycle as _;
use hil_cases::{hil_panic_handler, Session};
use rpi_hal::pac;
use rpi_hal::pwm::{Channel1, Channel1Pin, Pwm};
use rpi_hal::timer::Timer;

hil_panic_handler!();

/// The channel 1 output being watched. See the module docs.
const PWM_PIN: u8 = 12;

/// The PWM clock this case runs at, matching the configuration the defect was
/// found under.
const TARGET_CLOCK_HZ: u32 = 125_000;

/// `RNG1`, the period in PWM clock ticks. With [`TARGET_CLOCK_HZ`] this is a
/// 10 ms period, which is what [`SETTLE_MS`] is chosen against.
const RANGE: u16 = 1250;

/// The duty that produces a tone: 38 pulses spread across [`RANGE`] ticks, so
/// the pin switches roughly 3,800 times a second with each pulse one tick —
/// 8 µs — wide.
///
/// Well inside what the sampler below can resolve, and deliberately not 50 %:
/// the balanced algorithm spreads its pulses, so a low duty is the case that
/// makes a sampler's resolution matter, and it is the one the application
/// uses.
const TONE_DUTY: u16 = 38;

/// A second, different duty, for the case that writes two unequal values back
/// to back. Any value distinct from both [`TONE_DUTY`] and [`SILENT_DUTY`]
/// would do; half the tone is one whose loss is obvious in the numbers,
/// because a channel that had honoured it would read 1.5 % high rather than
/// 3 %.
const HALF_TONE_DUTY: u16 = TONE_DUTY / 2;

/// The duty that is supposed to mean silence.
const SILENT_DUTY: u16 = 0;

/// How long to wait after a duty write before believing the pin has settled.
///
/// One and a half PWM periods. A channel that merely needed a period boundary
/// to act on a new duty would be finished well inside this, so anything still
/// switching afterwards is not a write that has yet to take effect — it is a
/// write that has been ignored.
const SETTLE_MS: u32 = 15;

/// How long each measurement window is.
///
/// Long enough to hold 50 whole PWM periods, so a channel emitting anything
/// periodic at all cannot hide between samples.
const WINDOW_MS: u32 = 20;

/// How many gaps [`GAP_SWEEP_US`] tries.
const GAP_COUNT: usize = 10;

/// Gaps to try between the two writes.
///
/// Dense at the bottom and out to 24 µs, which is three PWM clock cycles at
/// [`TARGET_CLOCK_HZ`] and twelve at [`FAST_CLOCK_HZ`] — far enough past the
/// at-risk zone at both clocks to show the failures stop rather than thin out.
/// The long tail the first sweep carried is gone: gaps of milliseconds were
/// never in question once 16 µs was clear, and the trials below are worth more
/// than the reach.
const GAP_SWEEP_US: [u32; GAP_COUNT] = [0, 1, 2, 3, 4, 6, 8, 12, 16, 24];

/// Note keys for the per-gap failure counts at [`TARGET_CLOCK_HZ`], and
/// [`FAST_GAP_KEYS`] for the other clock.
///
/// Spelled out because [`Session::note`] takes a `&str` and this crate has no
/// allocator to format one in. The alternative is one key reused per gap, and
/// the host collects notes into a dict — so every gap but the last would be
/// silently dropped from the report while still looking fine on the wire.
const SLOW_GAP_KEYS: [&str; GAP_COUNT] = [
    "slow_gap_0us",
    "slow_gap_1us",
    "slow_gap_2us",
    "slow_gap_3us",
    "slow_gap_4us",
    "slow_gap_6us",
    "slow_gap_8us",
    "slow_gap_12us",
    "slow_gap_16us",
    "slow_gap_24us",
];

/// See [`SLOW_GAP_KEYS`].
const FAST_GAP_KEYS: [&str; GAP_COUNT] = [
    "fast_gap_0us",
    "fast_gap_1us",
    "fast_gap_2us",
    "fast_gap_3us",
    "fast_gap_4us",
    "fast_gap_6us",
    "fast_gap_8us",
    "fast_gap_12us",
    "fast_gap_16us",
    "fast_gap_24us",
];

/// How many times each gap is tried.
///
/// One trial per gap is what produced two runs that disagreed about 2 µs and
/// 4 µs, each looking like a clean threshold on its own. Near the boundary the
/// outcome depends on where the second write lands within the PWM clock cycle
/// rather than only on the gap, so a single sample measures the phase it
/// happened to draw. Eight makes a survival visible as a rate.
///
/// Eight rather than more because the case has to finish inside the runner's
/// timeout: each trial re-arms the tone and takes two measurement windows, so
/// this is already ~6 s per clock.
const GAP_TRIALS: u32 = 8;

/// The second PWM clock the sweep is repeated at, and the whole reason for
/// repeating it.
///
/// Four times [`TARGET_CLOCK_HZ`], so a boundary measured in *clock cycles*
/// moves to a quarter of where it was and one measured in *time* does not.
/// Those two answers want different fixes: a driver can compute a cycle-based
/// wait from the divisor it already holds, whereas a fixed few microseconds is
/// a constant — and picking the wrong one is only visibly wrong at a clock far
/// from the one it was measured at, which is the kind of bug that ships.
///
/// Fast enough to separate the answers (1 µs against 4 µs) and slow enough
/// that the pulse stays resolvable: 2 µs wide here, against a sampler measured
/// at 0.31 µs.
const FAST_CLOCK_HZ: u32 = 500_000;

/// How many samples must land inside one pulse before a transition count of
/// zero is allowed to mean "the channel stopped".
///
/// Four, so a pulse would have to be missed by every one of four attempts to
/// go unseen. Nyquist alone would say two, which is the rate at which a
/// *periodic* signal is exactly as likely to be sampled at its two extremes as
/// missed entirely — and this is asserting an absence, where being marginal is
/// indistinguishable from being right.
///
/// Checked rather than assumed because the sample rate is two peripheral reads
/// on device memory per iteration, which is not a number that can be reasoned
/// to from the source.
const MIN_SAMPLES_PER_PULSE: u32 = 4;

/// What one measurement window saw.
///
/// The sample count travels with the result because the whole case turns on
/// [`Window::transitions`] being zero, and zero is also what a sampler too slow
/// to catch a pulse reports. A verdict of "off" is only worth as much as the
/// rate that produced it, so the rate is measured rather than assumed and
/// [`Window::us_per_sample`] is asserted on before any of it is believed.
#[derive(Clone, Copy)]
struct Window {
    /// Level changes seen. Zero means a steady pin — not necessarily a *low*
    /// one, which is why this counts changes rather than reading a level once.
    transitions: u32,
    /// Samples taken, for the rate.
    samples: u32,
    /// Samples that found the pin high. A second, independent signal: a
    /// channel emitting [`TONE_DUTY`] should be high for about 3 % of them,
    /// and one that has stopped for none or all.
    highs: u32,
    /// How long the window actually ran, by the System Timer.
    elapsed_us: u32,
}

impl Window {
    /// Mean microseconds between samples, rounded down.
    ///
    /// Zero would mean sub-microsecond sampling, which the System Timer cannot
    /// resolve — so the assertion built on this is deliberately expressed the
    /// other way round, in samples per pulse.
    fn us_per_sample(&self) -> u32 {
        self.elapsed_us / self.samples.max(1)
    }

    /// How many samples land inside one pulse, at `pulse_us` wide.
    ///
    /// The number that says whether a zero transition count means anything.
    /// One or fewer is aliasing and the window's verdict is worthless.
    fn samples_per_pulse(&self, pulse_us: u32) -> u32 {
        self.samples.saturating_mul(pulse_us) / self.elapsed_us.max(1)
    }
}

/// Writes `DAT1` straight through the PAC, with none of the settling delay
/// [`SetDutyCycle::set_duty_cycle`] now performs.
///
/// Only [`sweep_gap`] uses this, and it has to: the driver's whole job is to
/// stop two writes landing within a couple of PWM clock cycles of each other,
/// so a sweep of the gap between two writes cannot be run through it. Every
/// gap would come back clean and the notes would read as though the hardware
/// hazard had gone away.
///
/// The assertions above deliberately do the opposite and go through the
/// driver, because what they check is the guarantee a caller gets.
fn write_dat1_raw(duty: u16) {
    // SAFETY: writes the duty register of the channel this case owns, which
    // is the same register `set_duty_cycle` writes and nothing else touches.
    unsafe {
        pac::Peripherals::steal()
            .PWM0
            .dat1()
            .write(|w| w.bits(u32::from(duty)));
    }
}

/// Watches the pin for `window_ms` and reports what it saw.
///
/// Polled rather than driven by an edge interrupt, so it can undercount a
/// switching pin — the transition count is a floor, not a frequency. What it
/// cannot do is miss a *steady* pin, which is the state every assertion here
/// actually turns on.
///
/// Reads `GPLEV0` directly rather than taking a `Pin`, because constructing an
/// input would rewrite the pin's function select and unmux the PWM this is
/// trying to observe — the case would then measure the pin it had just
/// disconnected, and report silence with total confidence.
fn watch(timer: &Timer, window_ms: u32) -> Window {
    // SAFETY: reads one GPIO register. Nothing here writes, so it cannot
    // disturb the channel driving the pin.
    let gpio = unsafe { pac::Peripherals::steal() }.GPIO;
    let mask = 1u32 << PWM_PIN;

    let started = timer.now_micros();
    let deadline = started + u64::from(window_ms) * 1000;
    let mut previous = gpio.gplev0().read().bits() & mask != 0;
    let mut window = Window {
        transitions: 0,
        samples: 0,
        highs: 0,
        elapsed_us: 0,
    };

    while timer.now_micros() < deadline {
        let level = gpio.gplev0().read().bits() & mask != 0;
        window.samples += 1;
        if level {
            window.highs += 1;
        }
        if level != previous {
            window.transitions += 1;
            previous = level;
        }
    }

    window.elapsed_us = timer.now_micros().wrapping_sub(started) as u32;
    window
}

/// Writes `duty`, waits [`SETTLE_MS`], and reports what the pin did next.
fn write_and_measure(channel: &mut Channel1<'_>, timer: &Timer, duty: u16) -> Window {
    let _ = channel.set_duty_cycle(duty);
    timer.delay_us(SETTLE_MS * 1000);
    watch(timer, WINDOW_MS)
}

/// Starts the tone again and returns whether the channel is switching.
///
/// Called before each silencing experiment so that each one acts on a running
/// channel. A case that inherited a stopped channel from the case before it
/// would write zero to something already silent and report a pass for having
/// changed nothing — the most flattering possible way to be wrong.
fn rearm(channel: &mut Channel1<'_>, timer: &Timer) -> bool {
    write_and_measure(channel, timer, TONE_DUTY).transitions > 0
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let mut session = Session::start(5);

    let timer = Timer::new(unsafe { pac::Peripherals::steal() }.SYSTMR);
    let peripherals = unsafe { pac::Peripherals::steal() };

    let divisor = Pwm::divisor_for(TARGET_CLOCK_HZ);
    let clock_hz = Pwm::clock_hz(divisor);
    let pwm = Pwm::init(peripherals.PWM0, peripherals.CM_PWM, divisor);
    let mut channel = pwm.channel1(&peripherals.GPIO, Channel1Pin::Gpio12, RANGE);

    // The clock the hardware will actually run at, not the one asked for:
    // `divisor_for` is integer division and `init` clamps, so a target below
    // the reachable floor comes back as a working divisor for the wrong rate.
    // Printed because every duration in this case is reasoned from the period.
    session.note("pwm_gpio", format_args!("{PWM_PIN}"));
    session.note(
        "pwm_clock",
        format_args!("{clock_hz} Hz (divisor {divisor}), range {RANGE}"),
    );
    session.note(
        "pwm_period_us",
        format_args!("{}", u32::from(RANGE) * 1_000_000 / clock_hz),
    );

    // The narrowest thing the sampler has to catch. In balanced mode the
    // algorithm spreads *single* ticks rather than widening one pulse, so this
    // is the pulse width at any low duty — the duty changes how many there
    // are, not how wide they are.
    let pulse_us = 1_000_000 / clock_hz;

    // 1. The channel is driving, and the sampler is fast enough to say so.
    //    Two assertions' worth of meaning in one case, because they are the
    //    same question: everything below reads a transition count of zero as
    //    "stopped", and zero is equally what a sampler that aliases every
    //    pulse would report. The rate is measured rather than reasoned about,
    //    because two peripheral reads per iteration on device memory is a
    //    guess until a board has run it.
    let tone = write_and_measure(&mut channel, &timer, TONE_DUTY);
    let per_pulse = tone.samples_per_pulse(pulse_us);
    session.note("pulse_us", format_args!("{pulse_us}"));
    session.note(
        "sampler",
        format_args!(
            "{} samples in {}us, {}us/sample, {per_pulse} per {pulse_us}us pulse",
            tone.samples,
            tone.elapsed_us,
            tone.us_per_sample()
        ),
    );
    session.note(
        "tone_window",
        format_args!(
            "{} transitions, {} of {} samples high",
            tone.transitions, tone.highs, tone.samples
        ),
    );
    session.check_fmt(
        "pwm_output_switches",
        tone.transitions > 0 && per_pulse >= MIN_SAMPLES_PER_PULSE,
        format_args!(
            "{} transitions on GPIO{PWM_PIN} at duty {TONE_DUTY}/{RANGE}, sampling every \
             {}us = {per_pulse} per {pulse_us}us pulse (want >= {MIN_SAMPLES_PER_PULSE}); \
             a zero count below this line would not distinguish a stopped channel from \
             a sampler that cannot see one",
            tone.transitions,
            tone.us_per_sample()
        ),
    );

    // 2. One write, on its own. This is what the original explanation said
    //    would leave the channel driving, and it does not.
    //
    //    The high-sample count is reported beside the transition count as an
    //    independent check on the same window: a channel still emitting
    //    TONE_DUTY is high for about 3% of samples, so a window with no
    //    transitions *and* no highs is a pin resting low rather than one whose
    //    edges were missed.
    let after_single = write_and_measure(&mut channel, &timer, SILENT_DUTY);
    session.note(
        "after_single_zero",
        format_args!(
            "{} transitions, {} of {} samples high",
            after_single.transitions, after_single.highs, after_single.samples
        ),
    );
    session.check_fmt(
        "pwm_zero_duty_stops_output",
        after_single.transitions == 0,
        format_args!(
            "GPIO{PWM_PIN} changed level {} times in {WINDOW_MS}ms ({} of {} samples high), \
             {SETTLE_MS}ms after a single write of DAT1={SILENT_DUTY}",
            after_single.transitions, after_single.highs, after_single.samples
        ),
    );

    // 3. The suspect: a second write with nothing between it and the first,
    //    which is the shape the application's failing path had. Deliberately
    //    back to back with no delay, because the whole question is whether a
    //    write landing inside the same PWM period as the last one wedges it.
    let rearmed = rearm(&mut channel, &timer);
    let _ = channel.set_duty_cycle(SILENT_DUTY);
    let after_rapid = write_and_measure(&mut channel, &timer, SILENT_DUTY);
    session.note(
        "after_rapid_pair",
        format_args!(
            "{} transitions, {} of {} samples high",
            after_rapid.transitions, after_rapid.highs, after_rapid.samples
        ),
    );
    session.check_fmt(
        "pwm_rapid_second_zero_write_stops_output",
        rearmed && after_rapid.transitions == 0,
        format_args!(
            "rearmed={rearmed}: GPIO{PWM_PIN} changed level {} times in {WINDOW_MS}ms \
             ({} of {} samples high) after two writes of DAT1={SILENT_DUTY} back to back",
            after_rapid.transitions, after_rapid.highs, after_rapid.samples
        ),
    );

    // 4. Two writes a whole period apart, which is what the application
    //    shipped. Also the control for case 3: same sampler, same pin, one
    //    variable changed.
    let rearmed = rearm(&mut channel, &timer);
    let _ = channel.set_duty_cycle(SILENT_DUTY);
    timer.delay_us(SETTLE_MS * 1000);
    let after_spaced = write_and_measure(&mut channel, &timer, SILENT_DUTY);
    session.note(
        "after_spaced_pair",
        format_args!(
            "{} transitions, {} of {} samples high",
            after_spaced.transitions, after_spaced.highs, after_spaced.samples
        ),
    );
    session.check_fmt(
        "pwm_spaced_second_zero_write_stops_output",
        rearmed && after_spaced.transitions == 0,
        format_args!(
            "rearmed={rearmed}: GPIO{PWM_PIN} changed level {} times in {WINDOW_MS}ms \
             ({} of {} samples high) after two writes of DAT1={SILENT_DUTY} \
             {SETTLE_MS}ms apart",
            after_spaced.transitions, after_spaced.highs, after_spaced.samples
        ),
    );

    // 5. The scope of the defect, and the case that decides how it can be
    //    fixed. Two writes back to back again, but carrying *different*
    //    values, so a channel that only objected to a redundant write would
    //    pass this.
    //
    //    It does not: the channel comes out emitting neither value but
    //    TONE_DUTY, the one in force before either write. Both are lost, so
    //    `set_duty_cycle` cannot dedupe its way out of this — nothing about
    //    the values written distinguishes the failing case from the working
    //    one, only when they were written.
    let rearmed = rearm(&mut channel, &timer);
    let _ = channel.set_duty_cycle(HALF_TONE_DUTY);
    let after_differing = write_and_measure(&mut channel, &timer, SILENT_DUTY);
    session.note(
        "after_rapid_differing_pair",
        format_args!(
            "rearmed {rearmed}, {} transitions, {} of {} samples high after writing \
             {HALF_TONE_DUTY} then {SILENT_DUTY} back to back",
            after_differing.transitions, after_differing.highs, after_differing.samples
        ),
    );
    session.check_fmt(
        "pwm_rapid_differing_writes_stop_output",
        rearmed && after_differing.transitions == 0,
        format_args!(
            "rearmed={rearmed}: GPIO{PWM_PIN} changed level {} times in {WINDOW_MS}ms \
             ({} of {} samples high) after writing {HALF_TONE_DUTY} then {SILENT_DUTY} \
             back to back; both writes were lost and the channel kept {TONE_DUTY}",
            after_differing.transitions, after_differing.highs, after_differing.samples
        ),
    );

    // Nothing below uses this channel: the sweeps claim their own, because the
    // clock is what they vary and only `Pwm::init` sets it. `Channel1` holds
    // no resource to release — it is a borrow of the register block — so there
    // is nothing to drop, and the guarantee that matters is simply that this
    // one is not touched again.
    let _ = channel;

    // Where the boundary actually is, at two clocks. Notes rather than
    // assertions: this is measuring an unknown rather than checking a known,
    // and asserting on an unmeasured expectation is how two wrong mechanisms
    // became documentation earlier in this case's life.
    sweep_gap(
        &mut session,
        &timer,
        TARGET_CLOCK_HZ,
        "gap_boundary_slow",
        &SLOW_GAP_KEYS,
    );
    sweep_gap(
        &mut session,
        &timer,
        FAST_CLOCK_HZ,
        "gap_boundary_fast",
        &FAST_GAP_KEYS,
    );

    session.finish()
}

/// Measures how often each gap between two `DAT` writes loses them, at a PWM
/// clock of `target_hz`, and reports the result under `key`.
///
/// A *rate* rather than a verdict, because the fault is not deterministic in
/// the gap alone. Two single-trial sweeps disagreed about 2 µs and 4 µs, each
/// looking like a clean threshold in isolation — near the boundary the outcome
/// turns on where the second write falls within the PWM clock cycle, which a
/// gap measured from the first write does not fix.
///
/// The number reported is the shortest gap from which **every** trial, at that
/// gap and every longer one, left the channel stopped. Contiguity is the point:
/// the lowest gap with a clean run of its own means nothing when a longer one
/// still fails, and that is exactly the shape the earlier sweeps produced.
///
/// Takes its own `Pwm` because the clock is what it is varying and `Pwm::init`
/// is the only thing that sets the divisor. The caller must have finished with
/// any channel of its own first.
fn sweep_gap(
    session: &mut Session,
    timer: &Timer,
    target_hz: u32,
    key: &'static str,
    keys: &[&'static str; GAP_COUNT],
) {
    // SAFETY: the caller has finished with its channel, so this is the only
    // live handle to PWM0 and CM_PWM for the length of this function.
    let peripherals = unsafe { pac::Peripherals::steal() };
    let divisor = Pwm::divisor_for(target_hz);
    let clock_hz = Pwm::clock_hz(divisor);
    let pwm = Pwm::init(peripherals.PWM0, peripherals.CM_PWM, divisor);
    let mut channel = pwm.channel1(&peripherals.GPIO, Channel1Pin::Gpio12, RANGE);

    let tick_us = 1_000_000 / clock_hz;
    let mut failures = [0u32; GAP_COUNT];
    let mut rearm_failed = false;

    for (index, gap_us) in GAP_SWEEP_US.iter().enumerate() {
        for _ in 0..GAP_TRIALS {
            // Re-armed through the driver — this only has to get the tone
            // running, and the settling it does is harmless here.
            if !rearm(&mut channel, timer) {
                rearm_failed = true;
            }
            // The pair itself goes straight to the register. See
            // `write_dat1_raw`: routed through the driver, the gap under test
            // could never be shorter than the driver's own settling delay.
            write_dat1_raw(SILENT_DUTY);
            timer.delay_us(*gap_us);
            write_dat1_raw(SILENT_DUTY);
            timer.delay_us(SETTLE_MS * 1000);
            if watch(timer, WINDOW_MS).transitions != 0 {
                failures[index] += 1;
            }
        }
    }

    // Down from the top, so the answer is the start of an unbroken clean run
    // rather than the first gap that happened to survive its trials.
    let mut safe_from = None;
    for index in (0..GAP_COUNT).rev() {
        if failures[index] != 0 {
            break;
        }
        safe_from = Some(GAP_SWEEP_US[index]);
    }

    // The longest gap that lost a write at all, which is where the at-risk
    // zone really ends — `safe_from` is only the next step up from it.
    let last_failing = GAP_SWEEP_US
        .iter()
        .zip(failures.iter())
        .filter(|(_, count)| **count > 0)
        .map(|(gap, _)| *gap)
        .next_back();

    session.note(
        key,
        format_args!(
            "clock {clock_hz}Hz, tick {tick_us}us, {GAP_TRIALS} trials/gap: clean from {}us \
             ({} ticks), worst surviving failure at {}us, rearm_failed {rearm_failed}",
            // `u32::MAX` for "no gap in the sweep was reliably clean", which
            // would be a different fault from the one being measured and must
            // not read as a plausible boundary.
            safe_from.unwrap_or(u32::MAX),
            safe_from.unwrap_or(u32::MAX) / tick_us.max(1),
            last_failing.unwrap_or_default()
        ),
    );

    // Every gap, including the clean ones: the shape of the risk zone is what
    // says how much margin a fix needs, and a histogram with the zeroes
    // removed cannot show whether the failures fall off a cliff or trail away.
    for (index, gap_us) in GAP_SWEEP_US.iter().enumerate() {
        session.note(
            keys[index],
            format_args!("{gap_us}us failed {}/{GAP_TRIALS}", failures[index]),
        );
    }
}
