"""When the runner decides a case has finished talking.

No hardware needed. This logic decides when to stop reading the console, so
getting it wrong truncates the transcript — and a truncated transcript is
indistinguishable from a board that died mid-run, which is the one thing the
`#HIL` protocol was shaped to detect.
"""

from __future__ import annotations

from hilbench import console
from hilbench.loader import (
    arch_from_config,
    config_for_arch,
    run_finished,
    split_new_lines,
)


def test_partial_trailer_is_not_finished() -> None:
    """The marker alone must not end the read.

    Regression: stopping at `#HIL end` cut the tallies off the very line
    being waited for, and the run then parsed as incomplete despite having
    finished perfectly. Both terminators carry their payload *after* the
    marker.
    """
    assert not run_finished(b"#HIL case=a status=PASS\r\n#HIL end pa")


def test_complete_trailer_is_finished() -> None:
    assert run_finished(b"#HIL end pass=4 fail=0 skip=0\n")


def test_carriage_returns_do_not_hide_the_newline() -> None:
    """The board's console emits CRLF, so the check must not assume bare LF."""
    assert run_finished(b"#HIL end pass=1 fail=0 skip=0\r\n")


def test_partial_panic_is_not_finished() -> None:
    assert not run_finished(b"#HIL panic detail=index out")


def test_complete_panic_is_finished() -> None:
    """A panic ends the run: waiting for a trailer that will never come would
    turn every panic into a timeout."""
    assert run_finished(b"#HIL panic detail=index out of bounds\n")


def test_no_terminator_is_not_finished() -> None:
    assert not run_finished(b"#HIL session board=0x1 arch=arm cases=2\n")


def test_truncated_read_round_trips_as_incomplete() -> None:
    """The two halves agree: a cut transcript parses as an unfinished run.

    Ties the reader to the parser, so a change to either that reintroduces
    the truncation bug fails here rather than on the bench.
    """
    cut = (
        "#HIL session board=0x00a22082 arch=aarch64 cases=1\r\n"
        "#HIL case=only status=PASS\r\n"
        "#HIL end pa"
    )
    assert not run_finished(cut.encode())
    assert not console.parse(cut).complete


def test_full_read_round_trips_as_complete() -> None:
    whole = (
        "#HIL session board=0x00a22082 arch=aarch64 cases=1\r\n"
        "#HIL case=only status=PASS\r\n"
        "#HIL end pass=1 fail=0 skip=0\r\n"
    )
    assert run_finished(whole.encode())
    report = console.parse(whole)
    assert report.complete and report.ok


def test_config_round_trip_preserves_other_settings() -> None:
    """An arch switch must not quietly drop the board's own settings.

    `core_freq` in particular changes how the mini-UART behaves, so losing it
    would alter what is under test rather than just how it boots.
    """
    original = "core_freq=250\narm_64bit=1\ndtoverlay=disable-bt\n"
    switched = config_for_arch(original, "arm")
    assert arch_from_config(switched) == "arm"
    assert "core_freq=250" in switched
    assert "dtoverlay=disable-bt" in switched


def test_config_absent_line_means_32_bit() -> None:
    """The firmware's own default, so an absent line is a statement."""
    assert arch_from_config("core_freq=250\n") == "arm"


def test_config_appends_when_absent() -> None:
    added = config_for_arch("core_freq=250\n", "aarch64")
    assert arch_from_config(added) == "aarch64"
    assert "core_freq=250" in added


def test_config_ignores_commented_settings() -> None:
    """A commented-out line does not select anything."""
    assert arch_from_config("# arm_64bit=1\n") == "arm"


def test_split_delivers_only_whole_lines() -> None:
    lines, at = split_new_lines(b"one\ntw", 0)
    assert lines == ["one"]
    assert at == 4


def test_split_does_not_repeat_or_halve_a_line() -> None:
    """A line arriving in two reads must reach the callback once, entire.

    The failure this guards is specific: the handoff schedule is a single
    line, and half of it does not parse, so a runner delivered the halves
    separately never enters the window the board has already committed to.
    """
    buf = b"#HIL console=release grace_ms=400 hold_ms=900 "
    lines, at = split_new_lines(buf, 0)
    assert lines == [] and at == 0

    buf += b"settle_ms=500\n#HIL case=a status=PASS\n"
    lines, at = split_new_lines(buf, at)
    assert lines == [
        "#HIL console=release grace_ms=400 hold_ms=900 settle_ms=500",
        "#HIL case=a status=PASS",
    ]
    assert at == len(buf)

    assert split_new_lines(buf, at) == ([], at)
