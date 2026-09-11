//! Shared scaffolding for hardware-in-the-loop case binaries.
//!
//! A case binary should be its assertions and nothing else, so the session
//! banner, the result protocol and the panic handling live here.
//!
//! # The output protocol
//!
//! Everything a case prints that the runner cares about is a single line
//! beginning `#HIL`, so ordinary console noise, boot chatter and a partially
//! overwritten line can never be mistaken for a result:
//!
//! ```text
//! #HIL session board=0x00a02082 arch=aarch64 cases=3
//! #HIL case=timer_monotonic status=PASS
//! #HIL case=timer_resolution status=FAIL detail=jitter 412us exceeds 100us
//! #HIL end pass=2 fail=1
//! ```
//!
//! The board revision comes from the mailbox rather than from a build-time
//! constant, so the runner can check it is talking to the board it thinks it
//! is. A rig that silently runs the Pi 3 suite against a Pi 4 reports
//! nonsense with total confidence, and that is the failure mode this line
//! exists to prevent.

#![no_std]

use core::fmt::Write;

use rpi_hal::gpio::{Input, Pin};
use rpi_hal::mailbox::Mailbox;
use rpi_hal::timer::Timer;
use rpi_hal::uart::Uart;

/// The board's console pins, and the only header pins with a UART alt
/// function on BCM283x.
///
/// UART0 also reaches GPIO32/33 and 36/37, and the mini-UART 32/33 and 40/41,
/// but none of GPIO32-41 come out on the 40-pin header — so a case that
/// tests these two cannot move its console aside, it has to take turns with
/// it.
const CONSOLE_TX: u8 = 14;
/// The other half of the console pair. See [`CONSOLE_TX`].
const CONSOLE_RX: u8 = 15;

/// The schedule for borrowing GPIO14/15 from the console.
///
/// Every duration here exists because the window is blind in both directions:
/// with the pins reassigned there is no console to coordinate over, so the
/// two ends cannot agree on anything *during* it. They agree beforehand
/// instead, by the board printing this and the runner reading it — one
/// authority, published, rather than a constant duplicated on each side that
/// drifts the first time either is tuned.
#[derive(Clone, Copy)]
pub struct Handoff {
    /// From printing the announcement to moving the pins. The runner sees the
    /// line later than the board sent it — USB scheduling, the CDC bridge and
    /// a subprocess pipe are all in between — and it has to have released its
    /// own end before the board reassigns anything.
    pub grace_ms: u32,
    /// How long the pins stay borrowed.
    pub hold_ms: u32,
    /// From restoring the console to printing on it again. The runner
    /// reattaches inside this window, so it is what decides whether the first
    /// line back is captured or lost.
    pub settle_ms: u32,
}

impl Handoff {
    /// A schedule with enough slack for a host that is not watching closely.
    ///
    /// Generous rather than tight. What the margins buy is that a late runner
    /// costs nothing; what they cost is a second of wall clock in one case.
    /// Tightening them trades a real failure mode for a saving nobody
    /// measures.
    pub const DEFAULT: Handoff = Handoff {
        grace_ms: 400,
        hold_ms: 900,
        settle_ms: 500,
    };
}

/// Which execution state this binary was built for, reported in the banner
/// so the runner can confirm the board actually came up in the state its
/// `config.txt` asked for.
pub const ARCH: &str = if cfg!(target_arch = "aarch64") {
    "aarch64"
} else {
    "arm"
};

/// A case's outcome.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// The assertion held.
    Pass,
    /// The assertion did not hold. Always accompanied by a detail string:
    /// a bare FAIL forces whoever reads the report to reproduce it by hand.
    Fail,
    /// The case cannot run on this board or in this configuration. Distinct
    /// from `Fail` so absent hardware never looks like a defect.
    Skip,
}

impl Status {
    fn as_str(self) -> &'static str {
        match self {
            Status::Pass => "PASS",
            Status::Fail => "FAIL",
            Status::Skip => "SKIP",
        }
    }
}

/// Owns the console and tallies results for one run.
///
/// Takes the UART for the whole session rather than reopening it per line,
/// because a case that hands GPIO14/15 to the fixture has to tear the
/// console down and bring it back deliberately — see [`Session::release_console`].
pub struct Session {
    uart: Uart,
    passed: u32,
    failed: u32,
    skipped: u32,
    /// The schedule announced by [`Session::release_console`], held so
    /// [`Session::reclaim_console`] waits out the same `settle_ms` the runner
    /// was told about. Keeping it here rather than asking the caller for it
    /// twice removes the one way the two halves could disagree.
    handoff: Option<Handoff>,
}

impl Session {
    /// Opens the console and prints the session banner.
    ///
    /// `expected` is how many cases this binary intends to report. The runner
    /// compares it against what actually arrives, so a binary that hangs
    /// halfway is distinguishable from one that legitimately ran fewer —
    /// otherwise a truncated run looks like a clean one.
    pub fn start(expected: u32) -> Self {
        let peripherals = unsafe { rpi_hal::pac::Peripherals::steal() };
        let mut uart = Uart::init(&peripherals.GPIO, peripherals.UART0);

        let mut mailbox = Mailbox::new(peripherals.VCMAILBOX);
        let board = mailbox.board_revision().unwrap_or(0);

        let _ = writeln!(
            uart,
            "#HIL session board={board:#010x} arch={ARCH} cases={expected}"
        );

        Self {
            uart,
            passed: 0,
            failed: 0,
            skipped: 0,
            handoff: None,
        }
    }

    /// Reports one result.
    pub fn report(&mut self, case: &str, status: Status, detail: &str) {
        match status {
            Status::Pass => self.passed += 1,
            Status::Fail => self.failed += 1,
            Status::Skip => self.skipped += 1,
        }
        let _ = write!(self.uart, "#HIL case={case} status={}", status.as_str());
        if !detail.is_empty() {
            let _ = write!(self.uart, " detail={detail}");
        }
        let _ = writeln!(self.uart);
    }

    /// Reports `Pass` or `Fail` from a boolean, with the detail used only on
    /// failure.
    pub fn check(&mut self, case: &str, ok: bool, detail: &str) {
        if ok {
            self.report(case, Status::Pass, "");
        } else {
            self.report(case, Status::Fail, detail);
        }
    }

    /// As [`Session::check`], with a formatted detail.
    ///
    /// Worth having as its own method rather than pushing callers to a fixed
    /// string: a numeric assertion that fails without reporting the numbers
    /// leaves nothing to diagnose from, and the next step is always another
    /// run just to print them. Use `format_args!` at the call site.
    pub fn check_fmt(&mut self, case: &str, ok: bool, detail: core::fmt::Arguments<'_>) {
        if ok {
            self.report(case, Status::Pass, "");
            return;
        }
        self.failed += 1;
        let _ = writeln!(
            self.uart,
            "#HIL case={case} status={} detail={detail}",
            Status::Fail.as_str()
        );
    }

    /// Reports a case that could not run, and why.
    pub fn skip(&mut self, case: &str, reason: &str) {
        self.report(case, Status::Skip, reason);
    }

    /// Records an observation without asserting on it.
    ///
    /// For numbers worth having in the transcript that no assertion should
    /// depend on: register contents, measured rates, anything whose expected
    /// value is not yet known. Notes do not count towards the tallies, so a
    /// diagnostic cannot turn a run red — which is the point. Investigating a
    /// failure by adding an assertion for a value nobody has measured yet
    /// just produces a second, differently wrong failure.
    pub fn note(&mut self, key: &str, value: core::fmt::Arguments<'_>) {
        let _ = writeln!(self.uart, "#HIL note {key}={value}");
    }

    /// Announces that marker-pin edges are about to start, and waits
    /// `grace_ms` for the runner to arm its capture.
    ///
    /// The same shape as [`Session::release_console`] and for the same
    /// reason — the runner sees the line later than it was printed, so the
    /// board has to wait rather than assume — but without a blind window,
    /// because the marker is its own wire. That is the whole return on
    /// spending a pin on it: the console keeps working throughout, so a case
    /// can report while it is being measured.
    ///
    /// Flushed before the wait starts, or the grace period would be running
    /// while the announcement was still sitting in the PL011's FIFO.
    pub fn marker_ready(&mut self, grace_ms: u32) {
        let _ = writeln!(self.uart, "#HIL marker=ready grace_ms={grace_ms}");
        self.uart.flush();
        delay_ms(grace_ms);
    }

    /// Gives up GPIO14/15 so a case can drive them as test pins.
    ///
    /// Announces `plan` on the console, waits for its flush to leave the
    /// wire, holds the console up for `grace_ms` while the runner releases
    /// its end, and only then drops both pins out of their UART alt function.
    /// Returns with the pins as plain inputs, for the case to configure
    /// however it needs.
    ///
    /// The grace period comes *before* the pins move, not after. Unmuxing
    /// first would leave the board's TX floating while the fixture's UART is
    /// still receiving on it, which frames noise into the transcript at the
    /// exact moment the runner is reading the schedule out of it. Holding the
    /// line in its alt function — where it idles high — until the fixture has
    /// let go costs nothing and removes the window entirely.
    ///
    /// The announcement is the last thing the runner sees before the window,
    /// which is what makes a case that hangs inside it distinguishable from
    /// one that never got that far — those look identical otherwise, and want
    /// different investigations.
    ///
    /// The flush is not a nicety. `writeln!` only reaches the PL011's FIFO,
    /// and the pins are unmuxed a few instructions later; without waiting for
    /// the shift register to empty, the tail of the very line carrying the
    /// schedule is cut off, and the runner then has no schedule to follow.
    ///
    /// Nothing is buffered while released — anything a case wants to say has
    /// to wait until [`Session::reclaim_console`].
    pub fn release_console(&mut self, plan: Handoff) {
        let _ = writeln!(
            self.uart,
            "#HIL console=release grace_ms={} hold_ms={} settle_ms={}",
            plan.grace_ms, plan.hold_ms, plan.settle_ms
        );
        self.uart.flush();
        self.handoff = Some(plan);

        delay_ms(plan.grace_ms);
        release_console_pins();
    }

    /// Takes GPIO14/15 back and resumes reporting.
    ///
    /// Returns the pins to inputs before re-muxing them, so a case that left
    /// one driving is not fighting the fixture's transmitter for the moment
    /// between the two writes.
    ///
    /// Then waits out the schedule's `settle_ms` *before* printing. That
    /// order is the point: the runner reattaches its bridge somewhere inside
    /// that window, and a line printed into a console nobody is bridging yet
    /// is simply gone — which, for the first line after a blind window, reads
    /// exactly like a board that never came back.
    pub fn reclaim_console(&mut self) {
        release_console_pins();

        let peripherals = unsafe { rpi_hal::pac::Peripherals::steal() };
        self.uart = Uart::init(&peripherals.GPIO, peripherals.UART0);

        if let Some(plan) = self.handoff.take() {
            delay_ms(plan.settle_ms);
        }
        let _ = writeln!(self.uart, "#HIL console=reclaim");
    }

    /// Prints the trailer and reboots, handing the board back to the loader.
    ///
    /// Rebooting rather than halting is what lets a run load more than one
    /// case binary. A halted board has no resident loader left, so the next
    /// image has nothing to talk to and the runner has to reset the board
    /// between every binary — which on a bench without a load switch means a
    /// person pressing a switch once per binary, a cost that grows with every
    /// case added.
    ///
    /// The trailer is flushed before the reset is triggered, so the verdict
    /// has already left the board by the time it goes away — an unflushed
    /// FIFO would lose the very line the runner is waiting for.
    pub fn finish(mut self) -> ! {
        let _ = writeln!(
            self.uart,
            "#HIL end pass={} fail={} skip={}",
            self.passed, self.failed, self.skipped
        );
        self.uart.flush();
        rpi_hal::power::reboot()
    }

    /// Prints the trailer and parks the core without rebooting.
    ///
    /// For a case that has deliberately left the board in a state worth
    /// inspecting, or one testing the reset path itself, where rebooting here
    /// would destroy the evidence or duplicate the thing under test.
    pub fn finish_and_halt(mut self) -> ! {
        let _ = writeln!(
            self.uart,
            "#HIL end pass={} fail={} skip={}",
            self.passed, self.failed, self.skipped
        );
        rpi_hal::halt()
    }
}

/// Drops GPIO14/15 out of whatever function they were in, back to inputs.
///
/// Deliberately not a method: it runs on both sides of the window, and one of
/// those is after a case may have reconfigured the pins behind the
/// `Session`'s back — which is exactly what a case borrowing them is for.
///
/// Each pin needs its own `GPIO` token because `Pin::new` takes one by value,
/// and stealing is how every other helper here gets at the peripherals.
fn release_console_pins() {
    let peripherals = unsafe { rpi_hal::pac::Peripherals::steal() };
    let _ = Pin::<CONSOLE_TX, Input>::new(peripherals.GPIO).into_input();
    let peripherals = unsafe { rpi_hal::pac::Peripherals::steal() };
    let _ = Pin::<CONSOLE_RX, Input>::new(peripherals.GPIO).into_input();
}

/// Blocks for `ms` against the System Timer.
///
/// The System Timer rather than a spin loop: the handoff's margins are only
/// worth anything if both ends measure them the same way, and a cycle-counted
/// delay changes length with the core clock — which other cases in the same
/// run are free to move.
fn delay_ms(ms: u32) {
    let peripherals = unsafe { rpi_hal::pac::Peripherals::steal() };
    Timer::new(peripherals.SYSTMR).delay_us(ms * 1_000);
}

/// Emits a panic as a result line before halting.
///
/// Without this a panic is indistinguishable from a hang: both end in
/// silence, but one is a defect in the case and the other is a defect in the
/// driver, and they want different investigations.
#[macro_export]
macro_rules! hil_panic_handler {
    () => {
        #[panic_handler]
        fn panic(info: &core::panic::PanicInfo) -> ! {
            use core::fmt::Write;
            let peripherals = unsafe { $crate::pac::Peripherals::steal() };
            let mut uart = $crate::uart::Uart::init(&peripherals.GPIO, peripherals.UART0);
            let _ = writeln!(uart, "#HIL panic detail={info}");
            $crate::halt()
        }
    };
}

// Re-exported so `hil_panic_handler!` expands without the case binary having
// to name rpi-hal itself.
#[doc(hidden)]
pub use rpi_hal::halt;
#[doc(hidden)]
pub use rpi_hal::pac;
#[doc(hidden)]
pub use rpi_hal::uart;
