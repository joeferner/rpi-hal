//! The console handoff: borrowing GPIO14/15 from the console and giving them
//! back.
//!
//! UART0 reaches GPIO14/15, 32/33 and 36/37, and the mini-UART 32/33 and
//! 40/41, but none of GPIO32-41 come out on the 40-pin header. So on a Pi the
//! console and any test of those two pins are the same two pins, and every
//! later case that wants them — a GPIO sweep, a second UART, anything shadowed
//! across the whole header — depends on this working first.
//!
//! Structurally this is the awkward case in the suite, because for most of it
//! there is no console to report on. What it does instead:
//!
//! 1. Announce the schedule and flush it, so the runner knows how long the
//!    window is before the window starts.
//! 2. Drop the pins out of their alt function, drive a pattern on GPIO14 that
//!    the fixture can witness independently, and keep the findings in
//!    registers.
//! 3. Bring the console back and print them.
//!
//! Only step 3 proves the handoff, and it cannot prove itself: a board that
//! never returned prints nothing, and a board with a broken console prints
//! nothing, and from here those are the same. That is why the pattern in step
//! 2 exists — the fixture sampling GPIO14 from the other end is the half of
//! the evidence this board cannot produce.

#![no_std]
#![no_main]

use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use hil_cases::{hil_panic_handler, Handoff, Session};
use rpi_hal::gpio::{Input, Pin};
use rpi_hal::pac;
use rpi_hal::timer::Timer;

hil_panic_handler!();

/// GPFSEL1 field value selecting ALT0, which is TXD0/RXD0 for GPIO14/15.
const FSEL_ALT0: u32 = 0b100;
/// GPFSEL1 field value selecting an input.
const FSEL_INPUT: u32 = 0b000;
/// GPFSEL1 field value selecting an output.
const FSEL_OUTPUT: u32 = 0b001;

/// Reads GPIO14's and GPIO15's 3-bit GPFSEL1 fields.
///
/// Read raw rather than tracked in a variable on purpose: the question this
/// case asks is whether the *hardware* mux moved, and a mirror of what the
/// code intended would answer a different one — it would still read "input"
/// if the write never landed.
fn console_pin_functions(gpio: &pac::GPIO) -> (u32, u32) {
    let bits = gpio.gpfsel1().read().bits();
    ((bits >> 12) & 0b111, (bits >> 15) & 0b111)
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let mut session = Session::start(4);

    let peripherals = unsafe { pac::Peripherals::steal() };
    let timer = Timer::new(peripherals.SYSTMR);
    let gpio = unsafe { pac::Peripherals::steal() }.GPIO;

    let plan = Handoff::DEFAULT;
    // A third of the hold each, so the fixture sees high, low, high. Three
    // phases rather than two: a single edge could be a pin that was already
    // at the level it ended on, while a pin that goes both ways and comes
    // back is being driven.
    let phase_ms = plan.hold_ms / 3;

    // Everything from here to `reclaim_console` runs with no console. Results
    // accumulate in these, and nothing in the window may print, panic or
    // block on the UART.
    let released_fsel;
    let drives_high;
    let drives_low;

    session.release_console(plan);
    {
        let mut tx = Pin::<14, Input>::new(unsafe { pac::Peripherals::steal() }.GPIO).into_output();

        // The mux, read back from GPFSEL1. This is the assertion the whole
        // handoff rests on: if the pins never left ALT0 then the PL011 is
        // still driving GPIO14, and every measurement after it — here and in
        // every later case that borrows these pins — is of the UART, not of
        // the test.
        released_fsel = console_pin_functions(&gpio);

        // Drive the pattern the fixture is watching for. Read back through
        // GPLEV as well, which for an output-configured pin reflects the pad
        // rather than the output register — so a pin held down by something
        // external fails here rather than passing on the strength of a write
        // that went nowhere.
        let _ = tx.set_high();
        timer.delay_us(phase_ms * 1_000);
        drives_high = tx.is_set_high().unwrap_or(false);

        let _ = tx.set_low();
        timer.delay_us(phase_ms * 1_000);
        drives_low = tx.is_set_low().unwrap_or(false);

        let _ = tx.set_high();
        timer.delay_us(phase_ms * 1_000);
    }
    session.reclaim_console();

    // Console back. Everything below is ordinary reporting again.

    session.check_fmt(
        "console_pins_released",
        released_fsel == (FSEL_OUTPUT, FSEL_INPUT),
        format_args!(
            "with the console released GPFSEL1 should read \
             fsel14={FSEL_OUTPUT} fsel15={FSEL_INPUT}, not fsel14={} fsel15={}",
            released_fsel.0, released_fsel.1
        ),
    );

    session.check(
        "gpio14_drives_high",
        drives_high,
        "GPIO14 was set high but GPLEV0 read it low; something is holding the pin down",
    );
    session.check(
        "gpio14_drives_low",
        drives_low,
        "GPIO14 was set low but GPLEV0 read it high; something is holding the pin up",
    );

    // The console is demonstrably back — this line is arriving on it — so the
    // remaining question is whether it came back *correctly*, i.e. through
    // the alt function rather than by some accident of the pins never having
    // moved. GPFSEL1 answers that, and the pair with the check above is what
    // makes either meaningful.
    let (fsel14, fsel15) = console_pin_functions(&gpio);
    session.check_fmt(
        "console_restored_to_alt0",
        (fsel14, fsel15) == (FSEL_ALT0, FSEL_ALT0),
        format_args!(
            "after reclaiming, GPFSEL1 reads fsel14={fsel14} fsel15={fsel15}, \
             not ALT0 ({FSEL_ALT0}) on both"
        ),
    );

    // What the pattern was, for the host's side of the assertion to be
    // diagnosed against. A note rather than a case: the board cannot judge
    // whether the fixture saw these edges, and a case that reports PASS for
    // something it did not check is worse than no case at all.
    session.note("handoff_phase_ms", format_args!("{phase_ms}"));
    session.note("handoff_pattern", format_args!("high,low,high"));

    session.finish()
}
