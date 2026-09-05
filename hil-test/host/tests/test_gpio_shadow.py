"""GPIO shadowing: the fixture stands in for whatever a board pin is wired to.

The other direction from ``test_console_handoff``, and the one the HAT's 1:1
header plan actually rests on. Reading a board-driven pin was settled by
``CONSOLE_PINS``; this asks whether the fixture can *drive* into a board pin
and be believed. If it can, one shadowed header covers every GPIO test the
suite will ever want, and the board's job is a connector plus a resistor per
line. If it cannot, each signal needs its own circuitry and that is a
different schematic, not a different component value.

Only GPIO14/15 are wired on the breadboard fixture. The technique does not
depend on the count — it is the same output driver against the same input
buffer whether there are two lines or twenty-eight — which is the whole reason
this is worth answering before anything is laid out.
"""

from __future__ import annotations

import contextlib
import time

import pytest

from hilbench import Cmd, Fixture, ProtocolError
from hilbench.console import Handoff, parse_handoff
from hilbench.loader import load_addr_for

#: Must match `PATTERN` in `cases/src/bin/hil_shadow.rs`, as `(gpio14, gpio15)`
#: per phase. Duplicated rather than negotiated because the window is blind:
#: the board cannot be told what to expect once it is inside one. The case
#: reports the levels it read, so a drift between these two shows up as a
#: named failure with both sequences printed, not as a mystery.
PATTERN = [(True, False), (False, True), (True, True)]

#: See the case's own note on this. The last phase drives both lines high
#: because the board re-muxes GPIO14 to its UART transmitter as the window
#: closes, and an idle UART line is high — so the brief overlap has both ends
#: driving the same level and no current flows. A low final phase would put a
#: short there instead.
assert PATTERN[-1] == (True, True), "the final phase must match an idle UART line"

BOOT_TIMEOUT = 90.0


def _release(bench: Fixture) -> None:
    """Hands both wires back and reattaches, whatever went wrong."""
    with contextlib.suppress(ProtocolError):
        bench.console_drive()
    with contextlib.suppress(ProtocolError):
        bench.console_attach()


def _wait_for_idle_board(bench: Fixture, timeout: float = 20.0) -> None:
    """Waits for the board's TXD0 to sit high, i.e. a live idle UART.

    Every test that needs a *second* driver on the line needs the board to be
    up and holding it. A case binary ends by rebooting, so in a full run these
    land while the Pi is still coming back, and sampling once made them fail
    depending on what ran before — which is a property of the schedule, not of
    the bench.

    Skips rather than fails if it never settles: an unpowered board is absent
    hardware, and absent hardware is the one thing this suite is careful never
    to report as a defect.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if bench.console_pins().gpio14:
            return
        time.sleep(0.25)
    pytest.skip(
        "the board's TXD0 never went high, so nothing is driving that line. "
        "The Pi is unpowered, held in reset, or still booting."
    )


def _require_drive(bench: Fixture) -> None:
    """Fails with the actual fix if the fixture predates `CONSOLE_DRIVE`.

    Without this, an old fixture answers `BAD_COMMAND` to the first call and
    every test below fails against whatever it happened to be asserting —
    "expected BAD_STATE, got BAD_COMMAND" sends the reader to the interlock
    logic when the problem is that the board on the desk is running last
    week's firmware.
    """
    try:
        bench.request(Cmd.CONSOLE_DRIVE, b"\x00\x00")
    except ProtocolError as exc:
        if "BAD_COMMAND" in str(exc):
            pytest.fail(
                f"the fixture does not know CONSOLE_DRIVE ({exc}); it reports "
                f"firmware {'.'.join(str(n) for n in bench.hello().firmware_version)}. "
                "Reflash it: make flash-rp2040",
                pytrace=False,
            )
        # Anything else — BAD_STATE in particular — means the command exists,
        # which is all this is asking.


def test_drive_is_refused_while_the_console_is_attached(bench: Fixture) -> None:
    """The interlock that makes shadowing safe to build on.

    Driving a pad the bridge's UART is still muxed onto puts two drivers on
    one net, which is a short whenever they disagree. The fixture cannot see
    whether the *board* has let go, but it knows whether it has itself, so it
    enforces the half it can — and this is the test that it really does,
    rather than the comment merely claiming so.
    """
    _require_drive(bench)
    assert bench.console_attached(), "precondition: the bridge should be attached"
    with pytest.raises(ProtocolError, match="BAD_STATE"):
        bench.console_drive(gpio14=True)


def test_the_fixture_drives_the_board_s_input_line(bench: Fixture) -> None:
    """The fixture's own output driver, checked without a case running.

    Isolated deliberately: a failure here is the fixture's driver or the SIO
    mux, while a failure in the board test below could be either end or the
    wire between them. Clearing this one first halves that search.

    **Only GPIO15.** That is the board's RXD0 — an input on the Pi even with
    no case running, so the fixture is the only thing driving it. GPIO14 is
    the board's TXD0 and its UART holds it high the whole time the loader is
    resident; driving that from here would be two output drivers on one net,
    which on the current direct wiring is a short. Detaching the fixture's
    bridge does not help, because the *board* is the other driver and it has
    not been asked to let go. That is what the handoff exists for, and it is
    why the only test that drives GPIO14 is the one that runs inside a window.
    """
    _require_drive(bench)
    try:
        bench.console_detach()
        for level in (False, True, False, True):
            bench.console_drive(gpio15=level)
            # A beat for the pad to settle through whatever is in line — a
            # series resistor into the board's pin capacitance is an RC, and
            # slowing the edge is the point of the resistor.
            time.sleep(0.002)
            pins = bench.console_pins()
            assert pins.gpio15 == level, f"drove gpio15={level} and read back {pins}"
    finally:
        _release(bench)


def test_the_board_holds_its_transmit_line_idle(bench: Fixture) -> None:
    """GPIO14 sits high, which is what a live board's idle UART looks like.

    Not really an assertion about the HAL — it is the precondition every test
    below depends on, and the one that is invisible otherwise. A Pi that has
    lost power reads low here rather than absent, so without this a whole
    board run fails at the loader handshake and looks like a protocol fault.
    """
    _wait_for_idle_board(bench)


def test_a_series_resistor_isolates_two_drivers(
    request: pytest.FixtureRequest, bench: Fixture
) -> None:
    """The resistor turns a short into a divider — measured, not assumed.

    This is the check that justifies the component. It drives the fixture's
    end of GPIO14 low while the board's UART is holding its end high, which
    is precisely the accident a 1:1 header invites: a case that forgets to
    release a pin, or a runner that detaches a moment late.

    With a resistor in line each end owns its own side and the fixture reads
    back the level it drove. Without one the two pads are shorted, the board's
    driver wins, and the fixture reads back *high* — which is how this was
    first discovered, by running it on direct wiring by mistake.

    Opt-in, and this is the one flag in the suite that is a claim about the
    soldering rather than about what is plugged in. Run against direct wiring
    it is a genuine short: brief and survivable, but not something to do on a
    schedule, so it stays off until someone asserts the bench is built for it.
    """
    if not request.config.getoption("--series-resistor"):
        pytest.skip(
            "needs --series-resistor: this drives against the board's UART on "
            "purpose, and without a resistor in line that is a short"
        )
    _require_drive(bench)
    # Without a second driver on the line there is nothing to contend with,
    # and the measurement would pass for the wrong reason.
    _wait_for_idle_board(bench)

    try:
        bench.console_detach()
        bench.console_drive(gpio14=False)
        time.sleep(0.002)
        held_low = not bench.console_pins().gpio14
    finally:
        _release(bench)

    assert held_low, (
        "the fixture drove GPIO14 low against the board's idle UART and read "
        "it back high: the two pads are fighting directly, so either there is "
        "no series resistor in that line or it is far too small"
    )


@pytest.fixture(scope="module")
def shadow_run(request, loader, case_image, case_target, bench: Fixture):
    """Runs `hil_shadow`, driving the pattern into the board as it goes.

    The mirror of `handoff_run`: there the board drove and the fixture
    watched, here the fixture drives and the board watches. The schedule comes
    off the board's announcement either way, because the board is the side
    with the deadline.
    """
    request.getfixturevalue("board_arch")
    request.getfixturevalue("board_ready")()

    plan: dict[str, Handoff] = {}
    failures: list[str] = []
    #: (seconds since the window opened, gpio14, gpio15) actually asked for.
    driven: list[tuple[float, bool, bool]] = []

    def on_line(line: str) -> None:
        announced = parse_handoff(line)
        if announced is None:
            return
        if "plan" in plan:
            failures.append("the case announced two handoffs; expected one")
            return
        plan["plan"] = announced

        seen_at = time.monotonic()
        phase = announced.hold_ms / len(PATTERN) / 1000
        # Measured from when the runner *saw* the line, which is at or after
        # the board printed it. So `grace` here always lands at or after the
        # board's own release — the fixture never starts driving into a pad
        # the board still has muxed to its UART.
        opens = seen_at + announced.grace_ms / 1000

        try:
            with bench.console_released():
                for index, (gpio14, gpio15) in enumerate(PATTERN):
                    _sleep_until(opens + index * phase)
                    bench.console_drive(gpio14=gpio14, gpio15=gpio15)
                    driven.append((time.monotonic() - seen_at, gpio14, gpio15))
                _sleep_until(opens + len(PATTERN) * phase)
                bench.console_drive()
                _sleep_until(seen_at + announced.reattach_after_ms / 1000)
        except ProtocolError as exc:
            # Recorded, not raised: raising out of the callback aborts the
            # boot, and the transcript of the run that went wrong is the thing
            # most worth having when it does.
            failures.append(f"the fixture failed mid-window: {exc}")

    result = loader.boot(
        str(case_image("hil_shadow")),
        load_addr_for(case_target),
        timeout=BOOT_TIMEOUT,
        on_line=on_line,
    )
    _release(bench)

    return result, plan.get("plan"), driven, failures


def _sleep_until(deadline: float) -> None:
    remaining = deadline - time.monotonic()
    if remaining > 0:
        time.sleep(remaining)


@pytest.mark.board
def test_the_runner_drove_the_whole_pattern(shadow_run) -> None:
    """Every phase was actually driven, and on the board's schedule.

    Checked before the board's own verdict, because a case that reports the
    wrong levels because the runner never drove them is a bench failure, not a
    HAL one, and reading it as the latter is how an afternoon disappears.
    """
    result, plan, driven, failures = shadow_run
    assert not failures, "; ".join(failures)
    assert plan is not None, (
        "no handoff announcement was seen, so nothing was ever driven. "
        "Transcript:\n" + result.output.decode("utf-8", "replace")
    )
    assert len(driven) == len(PATTERN), (
        f"drove {len(driven)} of {len(PATTERN)} phases: {driven}"
    )

    phase = plan.hold_ms / len(PATTERN) / 1000
    for index, (at, gpio14, gpio15) in enumerate(driven):
        want = plan.grace_ms / 1000 + index * phase
        assert abs(at - want) < 0.05, (
            f"phase {index} was driven at {at:.3f}s, not {want:.3f}s; the "
            f"board sampled it half a phase ({phase / 2:.3f}s) later"
        )
        print(
            f"\nphase {index} at {at:.3f}s: gpio14={int(gpio14)} gpio15={int(gpio15)}"
        )


@pytest.mark.board
def test_the_board_read_what_the_fixture_drove(shadow_run) -> None:
    """The answer the HAT design is waiting for.

    A pass means one shadowed header is a viable interface for every GPIO test
    the suite will want. A fail means per-signal circuitry, which is a respin
    rather than a rework — which is why this runs before anything is laid out.
    """
    result, _plan, _driven, _failures = shadow_run
    report = result.report
    transcript = result.output.decode("utf-8", "replace")

    assert not result.timed_out, f"the case never finished. Transcript:\n{transcript}"
    assert not report.panic, f"case panicked: {report.panic}"
    assert report.reclaimed, f"the board never got its console back:\n{transcript}"
    assert report.complete, f"{report.summary()}\n{transcript}"

    failed = [c for c in report.cases if c.status == "FAIL"]
    assert not failed, "\n".join(f"{c.name}: {c.detail}" for c in failed)
    for key, value in report.notes.items():
        print(f"  note {key} = {value}")
