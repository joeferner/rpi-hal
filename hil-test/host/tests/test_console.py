"""Parsing the `#HIL` protocol out of a console transcript.

These need no hardware, which matters more than it sounds: the parser is
what decides whether a run passed, so a bug here misreports every case on
the bench at once. It should be the best-tested thing in the suite.
"""

from __future__ import annotations

from hilbench import console

COMPLETE = """\
Some boot chatter that is not ours
#HIL session board=0x00a02082 arch=arm cases=3
#HIL case=timer_advances status=PASS
#HIL case=timer_rate status=FAIL detail=50ms measured outside the window
#HIL case=pcm_i2s status=SKIP detail=no DAC on this rig
#HIL end pass=1 fail=1 skip=1
"""


def test_parses_a_complete_run() -> None:
    report = console.parse(COMPLETE)
    assert report.board == 0x00A02082
    assert report.arch == "arm"
    assert report.expected == 3
    assert [(c.name, c.status) for c in report.cases] == [
        ("timer_advances", "PASS"),
        ("timer_rate", "FAIL"),
        ("pcm_i2s", "SKIP"),
    ]
    assert (report.passed, report.failed, report.skipped) == (1, 1, 1)
    assert report.complete
    assert not report.ok


def test_details_survive_spaces() -> None:
    """A detail runs to end of line, so it must not stop at the first space.

    Truncating it would strip exactly the part that explains the failure.
    """
    report = console.parse(COMPLETE)
    failed = next(c for c in report.cases if c.status == "FAIL")
    assert failed.detail == "50ms measured outside the window"


def test_ignores_non_protocol_lines() -> None:
    """Boot chatter shares this console and must never become a result."""
    report = console.parse(COMPLETE)
    assert len(report.cases) == 3


def test_a_passing_run_is_ok() -> None:
    report = console.parse(
        "#HIL session board=0x1 arch=aarch64 cases=1\n"
        "#HIL case=only status=PASS\n"
        "#HIL end pass=1 fail=0 skip=0\n"
    )
    assert report.ok


def test_skips_alone_still_count_as_ok() -> None:
    """A case that could not run is not a case that failed."""
    report = console.parse(
        "#HIL session board=0x1 arch=arm cases=1\n"
        "#HIL case=needs_dac status=SKIP detail=no DAC\n"
        "#HIL end pass=0 fail=0 skip=1\n"
    )
    assert report.ok


def test_truncated_run_is_not_ok() -> None:
    """A hang mid-run must not read as a short but clean run.

    This is the case that makes the trailer and the declared count worth
    having: without them, a board that dies after its first passing case
    reports green.
    """
    report = console.parse(
        "#HIL session board=0x1 arch=arm cases=4\n#HIL case=first status=PASS\n"
    )
    assert not report.complete
    assert not report.ok
    assert "1 of 4" in report.summary()


def test_missing_trailer_is_not_ok() -> None:
    """All cases reported but no trailer still means it did not finish."""
    report = console.parse(
        "#HIL session board=0x1 arch=arm cases=1\n#HIL case=only status=PASS\n"
    )
    assert not report.complete


def test_panic_is_not_ok() -> None:
    report = console.parse(
        "#HIL session board=0x1 arch=arm cases=2\n"
        "#HIL case=first status=PASS\n"
        "#HIL panic detail=attempt to divide by zero\n"
    )
    assert report.panic == "attempt to divide by zero"
    assert not report.ok
    assert "panicked" in report.summary()


def test_empty_transcript_is_not_ok() -> None:
    """Silence is the most common bare-metal failure and must not pass."""
    report = console.parse("")
    assert not report.ok
    assert not report.cases


def test_accepts_bytes() -> None:
    """Transcripts arrive from a serial port as bytes."""
    report = console.parse(COMPLETE.encode())
    assert report.expected == 3


def test_tolerates_undecodable_bytes() -> None:
    """A wrong baud produces garbage, and the parser must not crash on it.

    Line noise is a normal thing to read off a console mid-bring-up; raising
    here would replace a useful "no results found" with a stack trace.
    """
    report = console.parse(b"\xff\xfe garbage \x00\n#HIL case=a status=PASS\n")
    assert [c.name for c in report.cases] == ["a"]


HANDOFF = """\
#HIL session board=0x00a22082 arch=aarch64 cases=1
#HIL console=release grace_ms=400 hold_ms=900 settle_ms=500
#HIL console=reclaim
#HIL case=console_pins_released status=PASS
#HIL end pass=1 fail=0 skip=0
"""


def test_parses_a_handoff_announcement() -> None:
    """The schedule is the one thing the runner must read while it can."""
    plan = console.parse_handoff(
        "#HIL console=release grace_ms=400 hold_ms=900 settle_ms=500"
    )
    assert plan == console.Handoff(grace_ms=400, hold_ms=900, settle_ms=500)


def test_handoff_ignores_other_lines() -> None:
    assert console.parse_handoff("#HIL case=a status=PASS") is None
    assert console.parse_handoff("chatter") is None


def test_partial_announcement_is_not_a_handoff() -> None:
    """A truncated line must not parse as a schedule with default numbers.

    The tail of this line is the first casualty of an unflushed console, and
    a half-read schedule would send the runner into the window on timings
    nobody chose — which is worse than not entering it at all.
    """
    assert console.parse_handoff("#HIL console=release grace_ms=400 hold") is None


def test_handoff_window_arithmetic() -> None:
    """The reattach point sits between the board's restore and its first line."""
    plan = console.Handoff(grace_ms=400, hold_ms=900, settle_ms=500)
    assert plan.blind_ms == 1800
    assert plan.grace_ms + plan.hold_ms < plan.reattach_after_ms < plan.blind_ms


def test_report_carries_the_handoff() -> None:
    report = console.parse(HANDOFF)
    assert report.handoff == console.Handoff(400, 900, 500)
    assert report.reclaimed
    assert report.ok


def test_a_handoff_with_no_reclaim_is_visible() -> None:
    """A board that never came back is a distinct, nameable state.

    Without this the transcript just stops, which looks the same as any other
    hang — and the recovery is different: this one has left the fixture
    detached, so the next case has no console either.
    """
    report = console.parse(
        "#HIL session board=0x1 arch=arm cases=1\n"
        "#HIL console=release grace_ms=400 hold_ms=900 settle_ms=500\n"
    )
    assert report.handoff is not None
    assert not report.reclaimed
    assert not report.ok
