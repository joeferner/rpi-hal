"""Reading the board's console: waiting for readiness, parsing results.

Everything here works on the tunnelled console port, so it is equally usable
against a fixture bridge or a plain USB-serial adapter on the smoke tier.
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass, field

#: What `rpi-loader` prints once it is relocated and accepting commands.
#:
#: Deliberately a substring of the full banner ("rpi-loader: relocated,
#: waiting for host") rather than the whole line: the loader is a separate
#: project, and matching the distinctive half survives cosmetic edits to the
#: prefix while still being specific enough not to fire on anything else.
LOADER_READY = "waiting for host"


class Timeout(RuntimeError):
    """A pattern did not appear on the console in time.

    Carries what *did* arrive, because triaging a bare-metal timeout without
    the byte stream is guesswork — the partial output usually names the
    problem.
    """

    def __init__(self, needle: str, seconds: float, seen: bytes) -> None:
        self.seen = seen
        shown = seen.decode("utf-8", "replace") if seen else "(nothing)"
        super().__init__(
            f"{needle!r} did not appear within {seconds}s. Console said: {shown}"
        )


def wait_for(port, needle: str, timeout: float = 15.0) -> bytes:
    """Reads until `needle` appears, returning everything read up to then.

    Polls rather than relying on a read timeout so the wait ends the moment
    the pattern lands. A fixed sleep would be either too short on a slow boot
    or wasted on every fast one, and the difference is multiplied by every
    power cycle in a run.
    """
    deadline = time.monotonic() + timeout
    buf = bytearray()
    target = needle.encode()
    while time.monotonic() < deadline:
        waiting = port.in_waiting
        if waiting:
            buf += port.read(waiting)
            if target in buf:
                return bytes(buf)
        else:
            time.sleep(0.02)
    raise Timeout(needle, timeout, bytes(buf))


def wait_for_loader(port, timeout: float = 15.0) -> bytes:
    """Waits for `rpi-loader` to announce it is ready for commands."""
    return wait_for(port, LOADER_READY, timeout)


@dataclass
class Case:
    """One reported case result."""

    name: str
    status: str
    detail: str = ""


@dataclass(frozen=True)
class Handoff:
    """The schedule a case announces before it borrows GPIO14/15.

    Those are the only header pins with a UART alt function, so a case that
    drives them has no console while it does. The window is therefore blind in
    both directions, and the two ends cannot negotiate their way through it —
    they have to have agreed beforehand. This is that agreement, published by
    the side that owns the deadline.

    Putting it on the wire rather than in a constant on each side is what
    keeps them in step: a case that changes its own timing changes the
    runner's with it, instead of the two drifting apart into a race that
    reproduces once a fortnight.
    """

    #: From printing the announcement to actually moving the pins. Covers the
    #: runner's reaction time, which includes USB scheduling and the loader
    #: subprocess's pipe — the announcement is seen later than it was sent.
    grace_ms: int
    #: How long the pins stay borrowed.
    hold_ms: int
    #: From restoring the console to the first line printed on it. The runner
    #: reattaches inside this, so it is the margin that decides whether the
    #: first line back is captured or lost.
    settle_ms: int

    @property
    def blind_ms(self) -> int:
        """Total time the console is unusable, from the announcement."""
        return self.grace_ms + self.hold_ms + self.settle_ms

    @property
    def reattach_after_ms(self) -> int:
        """When the runner should reattach, measured from *seeing* the line.

        Half a settle period after the board is due to have restored its end.
        The margin has to absorb the delay between the board printing the
        announcement and the runner reading it, in both directions: reattach
        too early and the fixture is bridging a UART the board has not brought
        back yet, too late and the board's first line is gone.
        """
        return self.grace_ms + self.hold_ms + self.settle_ms // 2


@dataclass
class Report:
    """Everything a case binary said, parsed.

    Kept as data rather than assertions so a caller can decide what matters:
    the runner wants a pass/fail verdict, while a human triaging a failure
    wants the banner and the raw text.
    """

    board: int | None = None
    arch: str | None = None
    expected: int | None = None
    cases: list[Case] = field(default_factory=list)
    passed: int | None = None
    failed: int | None = None
    skipped: int | None = None
    panic: str | None = None
    #: Observations a case recorded without asserting on them.
    notes: dict[str, str] = field(default_factory=dict)
    #: The handoff schedule, if the case announced one.
    handoff: Handoff | None = None
    #: Whether the case said it had its console back. Absent after a
    #: `handoff` means the board never returned from the blind window — the
    #: one failure the console itself cannot report.
    reclaimed: bool = False
    raw: str = ""

    @property
    def complete(self) -> bool:
        """True if the binary ran to its own trailer.

        A run that stops early is not a run with fewer results — it is a
        hang, and without this check a truncated transcript reports green.
        """
        return self.failed is not None and (
            self.expected is None or len(self.cases) == self.expected
        )

    @property
    def ok(self) -> bool:
        """True only if the binary finished and nothing failed or panicked."""
        return self.complete and not self.panic and not self.failed

    def summary(self) -> str:
        if self.panic:
            return f"panicked: {self.panic}"
        if not self.complete:
            got = len(self.cases)
            want = "?" if self.expected is None else self.expected
            return f"incomplete: {got} of {want} cases, no trailer"
        return f"{self.passed} passed, {self.failed} failed, {self.skipped} skipped"


_SESSION = re.compile(
    r"#HIL session board=(?P<board>0x[0-9a-fA-F]+) arch=(?P<arch>\S+) cases=(?P<n>\d+)"
)
_CASE = re.compile(
    r"#HIL case=(?P<name>\S+) status=(?P<status>PASS|FAIL|SKIP)"
    r"(?: detail=(?P<detail>.*))?"
)
_END = re.compile(r"#HIL end pass=(?P<p>\d+) fail=(?P<f>\d+) skip=(?P<s>\d+)")
_PANIC = re.compile(r"#HIL panic detail=(?P<detail>.*)")
_NOTE = re.compile(r"#HIL note (?P<key>\S+)=(?P<value>.*)")
_RELEASE = re.compile(
    r"#HIL console=release grace_ms=(?P<grace>\d+) hold_ms=(?P<hold>\d+) "
    r"settle_ms=(?P<settle>\d+)"
)
_RECLAIM = re.compile(r"#HIL console=reclaim")


def parse_handoff(line: str | bytes) -> Handoff | None:
    """Reads a handoff announcement, or returns `None` for any other line.

    Split out from :func:`parse` because the runner needs it *live*: the
    schedule has to be acted on while the case is running, not recovered from
    the transcript once it is over.
    """
    if isinstance(line, bytes):
        line = line.decode("utf-8", "replace")
    m = _RELEASE.search(line)
    if m is None:
        return None
    return Handoff(
        grace_ms=int(m.group("grace")),
        hold_ms=int(m.group("hold")),
        settle_ms=int(m.group("settle")),
    )


def parse(text: str | bytes) -> Report:
    """Extracts the `#HIL` protocol from a console transcript.

    Non-matching lines are ignored rather than rejected: boot chatter, driver
    logging and a partially overwritten line all share this console, which is
    exactly why every protocol line carries the `#HIL` prefix.
    """
    if isinstance(text, bytes):
        text = text.decode("utf-8", "replace")

    report = Report(raw=text)
    for line in text.splitlines():
        line = line.strip()

        if m := _SESSION.search(line):
            report.board = int(m.group("board"), 16)
            report.arch = m.group("arch")
            report.expected = int(m.group("n"))
        elif m := _CASE.search(line):
            report.cases.append(
                Case(m.group("name"), m.group("status"), m.group("detail") or "")
            )
        elif m := _END.search(line):
            report.passed = int(m.group("p"))
            report.failed = int(m.group("f"))
            report.skipped = int(m.group("s"))
        elif m := _PANIC.search(line):
            report.panic = m.group("detail")
        elif m := _NOTE.search(line):
            report.notes[m.group("key")] = m.group("value").strip()
        elif handoff := parse_handoff(line):
            report.handoff = handoff
        elif _RECLAIM.search(line):
            report.reclaimed = True

    return report
