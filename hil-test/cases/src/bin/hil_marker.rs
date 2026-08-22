//! Marker-pin stimulus: the board emits known edges for the fixture to time.
//!
//! The convention every later timing assertion is built on. A case toggles a
//! designated GPIO around whatever it cares about and the fixture, watching
//! that one wire, says when each edge arrived. This binary is the calibration
//! of that convention rather than a use of it — it emits patterns whose
//! timing the board already knows, so the two clocks can be compared before
//! either is trusted to measure the other.
//!
//! Unlike `hil_console` and `hil_shadow` there is no blind window here, and
//! that is the entire argument for spending a pin on this: the marker is its
//! own wire, so the console stays up and the case can narrate what it is
//! doing while the measurement happens.
//!
//! Three segments, each answering a different question:
//!
//! 1. **A square wave at a known rate.** Compares the Pi's System Timer
//!    against the fixture's PIO timebase — two crystals, and the disagreement
//!    between them is the floor under every tolerance the suite will ever set.
//! 2. **The fastest edges the board can produce.** Not a controlled stimulus,
//!    deliberately: what it measures is the shortest pulse the bench can
//!    resolve *and* how fast a Pi can actually toggle a GPIO, both of which
//!    are numbers nobody here has had.
//! 3. **A long burst.** Fills enough of the capture buffer to show the depth
//!    is real and that nothing is dropped at a rate a real driver might hit.
//!
//! Segments are separated by gaps far longer than any interval inside one, so
//! the host can split them apart without being told where they are.

#![no_std]
#![no_main]

use embedded_hal::digital::OutputPin;
use hil_cases::{hil_panic_handler, Session};
use rpi_hal::gpio::{Input, Output, Pin};
use rpi_hal::pac;
use rpi_hal::timer::Timer;

hil_panic_handler!();

/// The marker line. GPIO4 is header pin 7, is not claimed by any peripheral
/// this crate drives, and sits well away from GPIO14/15 so a marker
/// measurement and a console handoff can never be the same experiment by
/// accident.
const MARKER: u8 = 4;

/// How long the runner has to arm its capture after the announcement. Same
/// role as the console handoff's grace: the board's line reaches the host
/// later than it was printed, and nothing may be emitted before the fixture
/// is listening.
const GRACE_MS: u32 = 400;

/// Between segments. Two orders of magnitude longer than the longest interval
/// inside a segment, so splitting the capture on gaps needs no threshold
/// anyone has to keep in step with the pattern.
const GAP_MS: u32 = 20;

/// Segment 1: cycles and half-period of the calibration square wave.
const SQUARE_CYCLES: u32 = 100;
/// Half-period of the calibration square wave, in microseconds.
const SQUARE_HALF_US: u32 = 500;

/// Segment 2: pulses at the narrowest width a case can actually ask for.
///
/// One microsecond is the System Timer's own resolution, so it is the floor
/// on any *deliberate* width — and 62 ticks of the fixture's timebase, which
/// should be measured exactly. This is the segment that says whether the
/// marker convention is usable for short events.
const NARROW_PULSES: u32 = 20;
/// Half-period of the narrow pulses, in microseconds.
const NARROW_HALF_US: u32 = 1;

/// Segment 3: cycles and half-period of the depth burst.
const BURST_CYCLES: u32 = 500;
/// Half-period of the depth burst, in microseconds.
const BURST_HALF_US: u32 = 50;

/// Segment 4: toggles with no delay at all, to find where the bench stops
/// seeing them.
///
/// Not a controlled stimulus and not meant to be: writes to `GPSET`/`GPCLR`
/// are posted, so the pulse width here is whatever the peripheral bus makes
/// it, and the useful output is what *fraction* of these the fixture manages
/// to resolve. Enough of them that the answer is a statistic rather than an
/// anecdote, and that the segment is still findable if most are lost.
const RUNT_PULSES: u32 = 200;

/// Emits `cycles` square-wave cycles and returns how long the board thinks it
/// took, in microseconds of System Timer.
///
/// Returning the board's own measurement is the point. The fixture will say
/// how long it took by *its* clock, and a bench that cannot state the
/// disagreement between the two has no business quoting either to a percent.
fn square_wave(marker: &mut Pin<MARKER, Output>, timer: &Timer, cycles: u32, half_us: u32) -> u64 {
    let start = timer.now_micros();
    for _ in 0..cycles {
        let _ = marker.set_high();
        timer.delay_us(half_us);
        let _ = marker.set_low();
        timer.delay_us(half_us);
    }
    timer.now_micros().wrapping_sub(start)
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let mut session = Session::start(3);

    let timer = Timer::new(unsafe { pac::Peripherals::steal() }.SYSTMR);
    let mut marker =
        Pin::<MARKER, Input>::new(unsafe { pac::Peripherals::steal() }.GPIO).into_output();
    // Driven low before anything else. The pin floats until something claims
    // it, and a floating input on the fixture's end is an edge generator —
    // the capture would open on noise rather than on the first real edge.
    let _ = marker.set_low();

    session.note("marker_gpio", format_args!("{MARKER}"));
    session.note(
        "marker_plan",
        format_args!(
            "square={SQUARE_CYCLES}x{SQUARE_HALF_US}us \
             narrow={NARROW_PULSES}x{NARROW_HALF_US}us \
             burst={BURST_CYCLES}x{BURST_HALF_US}us runts={RUNT_PULSES} \
             gap={GAP_MS}ms"
        ),
    );

    // The announcement the runner arms on. Flushed, because the grace period
    // starts when the host sees this line and an unflushed one sits in the
    // FIFO while the clock runs.
    session.marker_ready(GRACE_MS);

    // -- 1: calibration square wave -----------------------------------------
    let square_us = square_wave(&mut marker, &timer, SQUARE_CYCLES, SQUARE_HALF_US);
    timer.delay_us(GAP_MS * 1_000);

    // -- 2: the narrowest pulse a case can deliberately ask for -------------
    let narrow_us = square_wave(&mut marker, &timer, NARROW_PULSES, NARROW_HALF_US);
    timer.delay_us(GAP_MS * 1_000);

    // -- 3: depth ------------------------------------------------------------
    let burst_us = square_wave(&mut marker, &timer, BURST_CYCLES, BURST_HALF_US);
    timer.delay_us(GAP_MS * 1_000);

    // -- 4: runts, with no delay at all --------------------------------------
    // Writes to GPSET/GPCLR are posted, so these come out far narrower than
    // anything above and the fixture is expected to miss most of them. That
    // is the measurement: how many survive is where the marker convention
    // stops working, and a case that needs an event seen has to hold the pin
    // wider than whatever this turns out to be.
    let runt_start = timer.now_micros();
    for _ in 0..RUNT_PULSES {
        let _ = marker.set_high();
        let _ = marker.set_low();
    }
    let runt_us = timer.now_micros().wrapping_sub(runt_start);

    // Left low, so the capture ends on a defined level rather than on
    // whatever the last edge happened to be.
    let _ = marker.set_low();

    // The board's own view of what it emitted. Everything the fixture
    // measured is checked against these, so they are notes rather than cases:
    // the board cannot see the marker wire, and a PASS for something it did
    // not observe would be the bench asserting its own intentions.
    session.note("marker_square_us", format_args!("{square_us}"));
    session.note("marker_narrow_us", format_args!("{narrow_us}"));
    session.note("marker_burst_us", format_args!("{burst_us}"));
    session.note("marker_runt_us", format_args!("{runt_us}"));
    session.note("marker_runt_pulses", format_args!("{RUNT_PULSES}"));
    session.note(
        "marker_edges",
        format_args!(
            "{}",
            2 * (SQUARE_CYCLES + NARROW_PULSES + BURST_CYCLES + RUNT_PULSES)
        ),
    );

    // What the board *can* judge: whether its own System Timer delays came
    // out the length it asked for. A failure here is `Timer::delay_us`, not
    // the marker convention, and separating the two is why this is checked
    // here rather than left to the host's cross-clock comparison — which
    // would fail identically for either cause.
    let square_want = (SQUARE_CYCLES * SQUARE_HALF_US * 2) as u64;
    session.check_fmt(
        "marker_square_duration",
        square_us.abs_diff(square_want) < square_want / 20,
        format_args!("emitted a {square_want}us square wave in {square_us}us"),
    );

    let burst_want = (BURST_CYCLES * BURST_HALF_US * 2) as u64;
    session.check_fmt(
        "marker_burst_duration",
        burst_us.abs_diff(burst_want) < burst_want / 20,
        format_args!("emitted a {burst_want}us burst in {burst_us}us"),
    );

    // The runt segment has no target to hit — how narrow those pulses come
    // out *is* the measurement — but it must not be absurd. Zero would mean
    // the loop was optimised away and nothing reached the wire at all, and a
    // millisecond would mean a register write is stalling somewhere nobody
    // expects one to.
    session.check_fmt(
        "marker_runts_are_short",
        (1..2_000).contains(&runt_us),
        format_args!("{RUNT_PULSES} back-to-back pulses took {runt_us}us"),
    );

    session.finish()
}
