"""When a zero duty stops a PWM channel, and when it does not.

`hil_pwm` measures its own PWM output pin through `GPLEV0` and reports four
things: that a tone duty makes the pin switch, and then what a single zero-duty
write, a rapid pair of them, and a spaced pair each do to it.

The defect these were written for is fixed: `set_duty_cycle` now holds off for
two PWM clock cycles after writing `DAT`, so two writes issued back to back
both take effect. Every case is a plain assertion again.

Worth keeping the history, because the shape of it is the argument for the
bench existing at all. The defect this began as — "one zero-duty write is
ignored" — was diagnosed by ear from an application's stuck buzzer, and
measured and disproved by this case on its first run. What replaced it took two
more corrections: a rapid *second* write is what loses both writes, and the
window it has to fall in has no sharp edge, because whether a write survives
depends on where it lands within the PWM clock cycle. Single-trial sweeps
produced clean-looking thresholds that disagreed run to run.

None of that was reachable by reading the driver, and none of it was reachable
by listening to a piezo.

The four cases are ordered so that each is a control for the next. If
`pwm_output_switches` fails, nothing below it measured a running channel. If a
silencing case fails while a later one passes, the sampler can evidently see a
stopped channel and the failure is the channel's.

See `cases/src/bin/hil_pwm.rs` for the history and for why this needs no
fixture.
"""

from __future__ import annotations

import pytest

from hilbench.loader import load_addr_for

#: Matches the smoke tier's. The case is a handful of measurement windows of
#: tens of milliseconds either side of the two `settle_delay()` busy-waits
#: inside `Pwm::channel1`, so it is nowhere near this — the timeout is here to
#: end a hung board, which is the only way a bare-metal case fails to finish.
BOOT_TIMEOUT = 60.0

#: Every case the binary reports, in the order it reports them.
#:
#: `pwm_rapid_second_zero_write_stops_output` carried an `xfail(strict=True)`
#: while the defect stood, and it is a plain assertion again because the driver
#: now settles for two PWM clock cycles after each duty write. That is the
#: lifecycle the marker existed for: green while the bug was present, red on
#: the run where it was fixed, then gone.
CASES = [
    "pwm_output_switches",
    "pwm_zero_duty_stops_output",
    "pwm_rapid_second_zero_write_stops_output",
    "pwm_spaced_second_zero_write_stops_output",
    "pwm_rapid_differing_writes_stop_output",
]


pytestmark = pytest.mark.board


@pytest.fixture(scope="module")
def pwm_run(request, loader, case_image, case_target):
    """Boots `hil_pwm` once and shares its report across the assertions below.

    Module-scoped for the smoke tier's reason: these are several questions
    about one run rather than one question each, and a boot per assertion would
    both cost more and let them disagree about what happened.
    """
    request.getfixturevalue("board_arch")
    request.getfixturevalue("board_ready")()

    image = case_image("hil_pwm")
    result = loader.boot(str(image), load_addr_for(case_target), timeout=BOOT_TIMEOUT)
    if result.timed_out:
        pytest.fail(
            f"hil_pwm did not finish within {BOOT_TIMEOUT}s. Transcript:\n"
            f"{result.output.decode('utf-8', 'replace')}",
            pytrace=False,
        )
    return result


def test_run_completed(pwm_run) -> None:
    """The binary ran to its trailer without panicking.

    First, because every assertion below reads a verdict out of the report, and
    a truncated report can supply a missing verdict that looks like an answer.
    The transition counts are printed here rather than only on failure: the run
    that records an unexpected number is usually not the run that fails on it,
    and these numbers are the whole point of the case.
    """
    report = pwm_run.report
    assert not report.panic, f"case panicked: {report.panic}"
    assert report.complete, report.summary()
    print(f"\n{report.summary()} in {pwm_run.elapsed:.1f}s")
    for key, value in report.notes.items():
        print(f"  note {key} = {value}")


@pytest.mark.parametrize("name", CASES)
def test_case_passed(pwm_run, name: str) -> None:
    """Each measurement in turn.

    Parametrised rather than one assertion over all of them, so a report names
    which measurement disagreed instead of the first one that did.
    """
    case = next((c for c in pwm_run.report.cases if c.name == name), None)
    # A missing case is not a failing one — it means the binary did not get
    # that far — and collapsing the two would report a defect on a board that
    # never measured it.
    assert case is not None, (
        f"{name} did not report. Cases seen: "
        f"{', '.join(c.name for c in pwm_run.report.cases) or 'none'}"
    )
    assert case.status == "PASS", f"{name}: {case.detail}"
