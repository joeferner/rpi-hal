"""The loop closed: load a case onto a board, run it, read the verdict back.

Everything before this tested a piece of the rig. This tests the rig — the
first thing here that would catch a regression in rpi-hal itself rather than
in the bench.
"""

from __future__ import annotations

import pytest

from hilbench.loader import load_addr_for

#: Long enough for a 32 KB transfer plus the case, short enough that a hung
#: board fails the case rather than stalling the run. Bare-metal cases hang
#: instead of failing, so this is the only thing that ends them.
BOOT_TIMEOUT = 60.0

#: Every case binary that self-checks with nothing attached but a console.
#: Adding a name here runs it — a binary that is built and never loaded is
#: worse than one that does not exist, because the build passing looks like
#: coverage.
SMOKE_BINARIES = ["hil_smoke", "hil_core"]


# Every test here needs a wired-up board, so they are excluded from the
# default run rather than left to time out on a bench that has none.
pytestmark = pytest.mark.board


@pytest.fixture(scope="module", params=SMOKE_BINARIES)
def smoke_run(request, loader, case_image, case_target):
    """Boots one case binary and shares its report across the assertions.

    Module-scoped so each binary is loaded once and the assertions below are
    several questions about that one run, rather than a boot each — which
    would multiply a slow operation and, worse, let them disagree about what
    happened. Parametrised so each binary still gets its own boot, because
    they are separate images and cannot share one.
    """
    binary = request.param
    # The execution state is settled once per session, since switching it
    # costs a reboot and every binary in a run wants the same one.
    request.getfixturevalue("board_arch")

    # Liveness, however, is checked before *every* boot. A case binary ends
    # by handing the board back to the loader, but one that panics or hangs
    # does not — and then the next binary in the run has nothing to talk to
    # and fails for a reason that has nothing to do with what it tests.
    request.getfixturevalue("board_ready")()

    image = case_image(binary)
    result = loader.boot(str(image), load_addr_for(case_target), timeout=BOOT_TIMEOUT)
    if result.timed_out:
        pytest.fail(
            f"{binary} did not finish within {BOOT_TIMEOUT}s. Transcript:\n"
            f"{result.output.decode('utf-8', 'replace')}",
            pytrace=False,
        )
    return result


def test_case_binary_runs_and_reports(smoke_run) -> None:
    """The board accepted an image, ran it, and reported a verdict.

    The single most important assertion in the suite: it is the difference
    between a bench that can answer questions about the HAL and one that can
    only answer questions about itself.
    """
    report = smoke_run.report
    assert report.cases, (
        "no #HIL lines came back. Transcript:\n"
        + smoke_run.output.decode("utf-8", "replace")
    )
    print(f"\n{report.summary()} in {smoke_run.elapsed:.1f}s")
    # Notes are diagnostics, so they are printed unconditionally rather than
    # only on failure: the run that records an unexpected value is usually
    # not the run that fails on it.
    for key, value in report.notes.items():
        print(f"  note {key} = {value}")


def test_run_was_not_truncated(smoke_run) -> None:
    """Every declared case reported, and the binary reached its trailer.

    Separate from the pass/fail check because they mean different things: a
    truncated run is a board that died mid-suite, while a failing run is a
    driver that misbehaved. Collapsing them would let the first masquerade
    as the second.
    """
    report = smoke_run.report
    assert not report.panic, f"case panicked: {report.panic}"
    assert report.complete, report.summary()


def test_board_identity_matches_expectation(smoke_run, case_target: str) -> None:
    """The banner must describe the board we think we are driving.

    Read from the mailbox on the device, so this catches running the wrong
    build against the wrong board — which otherwise produces confident
    nonsense rather than an error.
    """
    report = smoke_run.report
    assert report.board, "no session banner; cannot confirm which board answered"

    expected_arch = "aarch64" if "aarch64" in case_target else "arm"
    assert report.arch == expected_arch, (
        f"built for {expected_arch} but the board reports {report.arch}. "
        "The board booted in the other execution state — check arm_64bit in "
        "its config.txt."
    )
    print(f"\nboard={report.board:#010x} arch={report.arch}")


def test_all_smoke_cases_pass(smoke_run) -> None:
    """No case failed.

    Last, and deliberately so: the checks above distinguish "the rig could
    not run this" from "the HAL is wrong", and only once they hold does a
    failure here mean anything about rpi-hal.
    """
    report = smoke_run.report
    failures = [c for c in report.cases if c.status == "FAIL"]
    assert not failures, "\n".join(f"{c.name}: {c.detail}" for c in failures)
