"""The console handoff: a case borrows GPIO14/15, and both ends survive it.

The last structural unknown before the HAT can be designed. GPIO14/15 are the
only header pins with a UART alt function on a BCM283x, so console and test
pin are physically the same wires and every case that wants those pins has to
take turns with its own reporting channel. If the two ends cannot hand them
back and forth reliably, the 1:1 header shadow the whole board plan rests on
needs a different topology — which is a respin, not a rework.

Three separate questions here, kept as separate tests because they fail for
unrelated reasons:

- Does the fixture actually let go, and does its own USB link survive it?
  (`test_detach_releases_the_pins`, no board needed.)
- Does the board come back with its transcript intact?
- Did the board really drive the pins while it was gone, witnessed from the
  other end rather than taken on its own word?
"""

from __future__ import annotations

import contextlib
import time

import pytest

from hilbench import Fixture, ProtocolError
from hilbench.console import Handoff, parse_handoff
from hilbench.loader import load_addr_for

#: The case's own blind window is about 1.8s; the rest is the image transfer
#: and boot. Generous, because a timeout here leaves the board halted and
#: costs a power cycle to recover.
BOOT_TIMEOUT = 90.0

#: How often to sample the fixture's view of GPIO14 during the window. Each
#: sample is a USB control round trip, so the real rate is whatever the bus
#: allows; this only stops the loop spinning flat out.
SAMPLE_INTERVAL = 0.005

#: How far a witnessed phase may be from the schedule and still count.
#: Loose — a third of the phase — because it absorbs USB scheduling on the
#: sampling side and the board's own console teardown on the other. This asks
#: whether the edges happened roughly when they were promised, not how
#: precisely; edge timing to a real tolerance is what `MARKER_TIMESTAMP`
#: exists for, and a bound tight enough to be a measurement here would just be
#: a flaky test measuring USB.
PHASE_TOLERANCE = 0.12


def _stable_release_state(bench: Fixture) -> None:
    """Puts the console back, whatever a previous failure left behind.

    Errors are swallowed because this runs on the way out of a test that may
    already be failing for its own reason, and that reason is the one worth
    reporting. A fixture too broken to reattach will fail the next test's
    first call anyway.
    """
    with contextlib.suppress(ProtocolError):
        bench.console_attach()


def test_detach_and_attach_are_supported(bench: Fixture) -> None:
    """The fixture must be new enough to release the pins at all.

    A fixture that predates the handoff answers `UNSUPPORTED`, and every test
    below would then fail at its own first step with a message about pin
    levels rather than about the firmware being stale.
    """
    try:
        bench.console_detach()
    except ProtocolError as exc:
        pytest.fail(
            f"the fixture refused CONSOLE_DETACH ({exc}). Reflash it: "
            "make flash-rp2040",
            pytrace=False,
        )
    finally:
        _stable_release_state(bench)


def test_detach_releases_the_pins(bench: Fixture) -> None:
    """Detach reports detached, attach reports attached, and both stick.

    Needs no board: it asks only whether the fixture's own state machine and
    its USB link survive the pins moving. Worth isolating, because that is the
    half of the handoff that can be cleared without anyone holding a Pi.
    """
    try:
        bench.console_detach()
        assert not bench.console_attached(), (
            "the fixture still reports the console attached after a detach"
        )
        # The vendor channel has to keep working through the window: it is the
        # only way to end one. A fixture whose USB fell over when the UART
        # pins moved would hang here rather than fail, which is why the
        # capability is exercised rather than merely reported.
        bench.ping()
        bench.console_pins()
    finally:
        _stable_release_state(bench)

    assert bench.console_attached(), (
        "the fixture did not come back attached; later cases have no console"
    )


def test_console_survives_a_detach_with_no_board_involved(
    bench: Fixture, console_port: str
) -> None:
    """The CDC endpoint stays open across a detach/attach cycle.

    Distinct from the test above: that one asks whether the *fixture* is
    healthy, this one whether the *host's* serial port is. If the CDC
    interface dropped and re-enumerated when the bridge let go, the loader's
    open file descriptor would go stale mid-run and the port might come back
    under a different name — a failure that presents as the board dying.
    """
    import serial

    try:
        with serial.Serial(console_port, 115200, timeout=0.1) as port:
            # Drain before detaching. Bytes the board sent *before* the
            # detach are real transcript sitting in the fixture's receive
            # ring, and the bridge is right to deliver them — a previous case
            # ending in a reboot leaves the loader's banner in there, which is
            # how this test first failed. What is being asserted is that
            # nothing *new* crosses while the pins are released.
            port.reset_input_buffer()
            port.read(4096)

            bench.console_detach()
            assert port.read(16) == b"", "bytes arrived on a detached console"
    except serial.SerialException as exc:
        pytest.skip(f"{console_port} is not openable right now: {exc}")
    finally:
        _stable_release_state(bench)


@pytest.fixture(scope="module")
def handoff_run(request, loader, case_image, case_target, bench: Fixture):
    """Runs `hil_console`, answering its handoff announcement as it goes.

    The interesting part is the callback. The board publishes its schedule on
    the console and then goes silent, so the runner cannot be told anything
    more once the window opens — it has to have read the plan and committed to
    a timeline before the pins move. Everything the fixture witnesses is
    gathered inside that commitment.
    """
    request.getfixturevalue("board_arch")
    request.getfixturevalue("board_ready")()

    #: (seconds since the window opened, GPIO14 level) as the fixture saw it.
    samples: list[tuple[float, bool]] = []
    plan: dict[str, Handoff] = {}
    failures: list[str] = []

    def on_line(line: str) -> None:
        announced = parse_handoff(line)
        if announced is None:
            return
        if "plan" in plan:
            failures.append("the case announced two handoffs; expected one")
            return
        plan["plan"] = announced

        opened = time.monotonic()
        reattach_at = opened + announced.reattach_after_ms / 1000
        try:
            with bench.console_released():
                while time.monotonic() < reattach_at:
                    samples.append(
                        (time.monotonic() - opened, bench.console_pins().gpio14)
                    )
                    time.sleep(SAMPLE_INTERVAL)
        except ProtocolError as exc:
            # Recorded rather than raised: raising out of the callback aborts
            # the boot, and the transcript of the run that went wrong is the
            # thing most worth having when it does.
            failures.append(f"the fixture failed mid-window: {exc}")

    result = loader.boot(
        str(case_image("hil_console")),
        load_addr_for(case_target),
        timeout=BOOT_TIMEOUT,
        on_line=on_line,
    )

    # Belt and braces. `console_released` reattaches on its own way out, but
    # a run that never reached the announcement never entered it, and a bench
    # left detached breaks every later test in the session.
    _stable_release_state(bench)

    return result, plan.get("plan"), samples, failures


@pytest.mark.board
def test_the_case_announced_a_handoff(handoff_run) -> None:
    """The runner saw the schedule, in one piece, before the window."""
    result, plan, _samples, failures = handoff_run
    assert not failures, "; ".join(failures)
    assert plan is not None, (
        "no handoff announcement was seen while the case ran, so the fixture "
        "never released the pins. Transcript:\n"
        + result.output.decode("utf-8", "replace")
    )
    print(
        f"\ngrace={plan.grace_ms}ms hold={plan.hold_ms}ms "
        f"settle={plan.settle_ms}ms (blind for {plan.blind_ms}ms)"
    )


@pytest.mark.board
def test_the_console_came_back(handoff_run) -> None:
    """The board reported after the window, and reported everything.

    The assertion the whole item exists to answer. A board that cannot get its
    console back after borrowing these pins makes every later case that wants
    them unreportable — the test would run and nobody would ever hear the
    result.
    """
    result, _plan, _samples, _failures = handoff_run
    report = result.report
    transcript = result.output.decode("utf-8", "replace")

    assert not result.timed_out, (
        f"the case never finished; it is probably still in the blind window. "
        f"Transcript:\n{transcript}"
    )
    assert report.handoff is not None, f"no release line in:\n{transcript}"
    assert report.reclaimed, (
        "the case announced the handoff but never said it had the console "
        f"back. Transcript:\n{transcript}"
    )
    assert not report.panic, f"case panicked: {report.panic}"
    assert report.complete, f"{report.summary()}\n{transcript}"


@pytest.mark.board
def test_board_side_checks_pass(handoff_run) -> None:
    """The board's own view: the pins moved, and moved back."""
    result, _plan, _samples, _failures = handoff_run
    report = result.report
    failed = [c for c in report.cases if c.status == "FAIL"]
    assert not failed, "\n".join(f"{c.name}: {c.detail}" for c in failed)
    for key, value in report.notes.items():
        print(f"  note {key} = {value}")


@pytest.mark.board
def test_the_fixture_witnessed_the_pattern(handoff_run) -> None:
    """GPIO14 really was driven, seen from the other end of the wire.

    The board reporting that it drove a pin is not evidence — a pin still in
    its alt function reads back from the board's side exactly as one under
    GPIO control would, so the whole case could pass with the mux never having
    moved. This is the half only the fixture can supply.

    Asserted as a run-length pattern rather than as levels at instants,
    because the sampling rate is whatever USB allowed and a fixed instant may
    land either side of an edge.
    """
    _result, plan, samples, _failures = handoff_run
    assert plan is not None, "no schedule to check the samples against"
    assert samples, "the fixture was never sampled during the window"

    runs: list[tuple[bool, float, float]] = []
    for at, level in samples:
        if runs and runs[-1][0] == level:
            runs[-1] = (level, runs[-1][1], at)
        else:
            runs.append((level, at, at))

    shape = "".join("H" if level else "L" for level, _, _ in runs)
    detail = f"saw {shape} over {len(samples)} samples: " + ", ".join(
        f"{'H' if lvl else 'L'} {start:.3f}-{end:.3f}s" for lvl, start, end in runs
    )
    print(f"\n{detail}")

    # The pins float once released and the fixture may sample before the
    # board has driven anything, so a leading run of either level is noise
    # rather than a result. What must be there is high, low, high after it.
    assert "HLH" in shape, (
        "the fixture never saw GPIO14 driven high, low and high again, so the "
        f"board's pin was not under its control during the window. {detail}"
    )

    expected = plan.hold_ms / 3 / 1000
    low = next(
        (end - start for lvl, start, end in runs if not lvl and end > start), None
    )
    assert low is not None, f"no measurable low phase. {detail}"
    assert abs(low - expected) <= PHASE_TOLERANCE, (
        f"the low phase lasted {low:.3f}s, not the {expected:.3f}s the case "
        f"announced. {detail}"
    )
