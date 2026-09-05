//! GPIO shadowing: the fixture drives a board pin and the board reads it.
//!
//! The other direction from `hil_console`, and the one the HAT's whole 1:1
//! header plan rests on. If the fixture can stand in for whatever a board pin
//! is wired to — pull it high, pull it low, and be believed — then one
//! shadowed header covers every GPIO test the suite will ever want. If it
//! cannot, the board needs per-signal circuitry instead of a uniform pin
//! interface, which is a different schematic rather than a different value.
//!
//! Only GPIO14/15 are wired on the breadboard fixture, so that is what this
//! proves. The technique does not depend on the count — 2 pins or 28, it is
//! the same output driver against the same input buffer — which is why this
//! is worth running long before the header exists.
//!
//! Both pins are read every phase and driven to *opposite* levels for two of
//! the three. That is deliberate: a case that only ever saw the two agree
//! would pass just as happily if it were reading one pin twice, or if the
//! wires were crossed, and crossed wires are the single most likely mistake
//! in a 28-way ribbon.
//!
//! The last phase drives both high, which is not arbitrary. The window closes
//! with the board re-muxing GPIO14 to its UART transmitter while the fixture
//! may still be driving for the few milliseconds it takes the announcement to
//! reach the host. High is what an idle UART line sits at, so that overlap has
//! both ends driving the *same* level and no current flows. Ending on a low
//! phase would put a short there instead — brief, survivable, and exactly the
//! thing a bench should not be built out of.

#![no_std]
#![no_main]

use embedded_hal::digital::InputPin;
use hil_cases::{hil_panic_handler, Handoff, Session};
use rpi_hal::gpio::{Input, Pin};
use rpi_hal::pac;
use rpi_hal::timer::Timer;

hil_panic_handler!();

/// What the fixture drives, phase by phase, as `(gpio14, gpio15)`.
///
/// The host reads this same shape off the announcement's schedule rather than
/// being told the pattern, so the two are kept in step by the test that
/// compares them, not by a constant duplicated on each side.
const PATTERN: [(bool, bool); 3] = [(true, false), (false, true), (true, true)];

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let mut session = Session::start(3);

    let timer = Timer::new(unsafe { pac::Peripherals::steal() }.SYSTMR);

    let plan = Handoff::DEFAULT;
    let phase_us = (plan.hold_ms / PATTERN.len() as u32) * 1_000;

    // Sampled inside the window, reported after it. `[false; _]` rather than
    // uninitialised: a phase that somehow never ran then reads low, which
    // fails against a pattern expecting high rather than passing on whatever
    // the stack happened to hold.
    let mut seen = [(false, false); PATTERN.len()];

    session.release_console(plan);
    {
        // Explicitly re-asserted rather than inherited from
        // `release_console`, which has already put both pins in input mode.
        // The redundant write costs nothing and means this block states the
        // configuration it depends on instead of documenting it.
        let mut p14 = Pin::<14, Input>::new(unsafe { pac::Peripherals::steal() }.GPIO).into_input();
        let mut p15 = Pin::<15, Input>::new(unsafe { pac::Peripherals::steal() }.GPIO).into_input();

        // The window opens the moment `release_console` returns, which is the
        // origin both ends measure their phases from.
        let opened = timer.now_micros();

        for (index, slot) in seen.iter_mut().enumerate() {
            // Sampled at the *midpoint* of each phase, not at its edge. The
            // host's transitions are offset from the board's clock by however
            // long the announcement took to reach it, and the midpoint is
            // simply where that offset has the most room — half a phase in
            // either direction, against a delay measured in single-figure
            // milliseconds.
            let at = opened + (index as u64) * phase_us as u64 + (phase_us / 2) as u64;
            while timer.now_micros() < at {}
            *slot = (
                p14.is_high().unwrap_or(false),
                p15.is_high().unwrap_or(false),
            );
        }

        // Hold the pins for the full announced window even though the last
        // sample is taken before it ends. The host's schedule runs to
        // `hold_ms` and giving the console back early would have the board
        // re-muxing GPIO14 while the fixture is still driving it.
        let closes = opened + (plan.hold_ms as u64) * 1_000;
        while timer.now_micros() < closes {}
    }
    session.reclaim_console();

    // Console back.

    let want_14 = PATTERN.map(|(gpio14, _)| gpio14);
    let want_15 = PATTERN.map(|(_, gpio15)| gpio15);
    let got_14 = seen.map(|(gpio14, _)| gpio14);
    let got_15 = seen.map(|(_, gpio15)| gpio15);

    session.check_fmt(
        "shadow_gpio14_follows_fixture",
        got_14 == want_14,
        format_args!(
            "the fixture drove {} on GPIO14 and the board read {}",
            Levels(&want_14),
            Levels(&got_14)
        ),
    );
    session.check_fmt(
        "shadow_gpio15_follows_fixture",
        got_15 == want_15,
        format_args!(
            "the fixture drove {} on GPIO15 and the board read {}",
            Levels(&want_15),
            Levels(&got_15)
        ),
    );

    // The two pins were driven to opposite levels for the first two phases, so
    // reading them equal there means the board is not seeing two independent
    // wires. Its own case because the diagnosis is specific and nothing like
    // the two above: crossed wiring, or a read that resolves to the wrong pin.
    let independent = seen
        .iter()
        .zip(PATTERN.iter())
        .all(|(&(got14, got15), &(want14, want15))| (got14 == got15) == (want14 == want15));
    session.check_fmt(
        "shadow_pins_are_independent",
        independent,
        format_args!(
            "GPIO14 read {} and GPIO15 read {} against a drive of {} and {}; \
             the two pins are not being read independently",
            Levels(&got_14),
            Levels(&got_15),
            Levels(&want_14),
            Levels(&want_15)
        ),
    );

    session.note("shadow_phase_ms", format_args!("{}", phase_us / 1_000));
    session.note("shadow_gpio14", format_args!("{}", Levels(&got_14)));
    session.note("shadow_gpio15", format_args!("{}", Levels(&got_15)));

    session.finish()
}

/// Renders a phase-by-phase level sequence as `HLH`.
///
/// A wrapper rather than a formatted loop at each call site: these appear in
/// failure details, which are read by someone who does not have the code open,
/// and `HLH` against `LHH` says what went wrong at a glance where
/// `[true, false, true]` needs decoding first.
struct Levels<'a>(&'a [bool]);

impl core::fmt::Display for Levels<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for &level in self.0 {
            f.write_str(if level { "H" } else { "L" })?;
        }
        Ok(())
    }
}
