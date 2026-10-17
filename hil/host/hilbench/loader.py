"""Driving `rpi-loader` to put an image on a board and read the result back.

Shells out to the loader's CLI rather than reimplementing its wire protocol.
That protocol has a baud negotiation and a checksum scheme that already work
and are tested in their own project; a second implementation here would be a
second thing to keep in step, and its bugs would look like HAL bugs.
"""

from __future__ import annotations

import contextlib
import io
import os
import selectors
import shutil
import subprocess
import sys
import tempfile
import termios
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import cast

from . import console

#: Where an image is linked, per execution state. The firmware would load a
#: kernel to the same place, so a case runs where a real one would.
LOAD_ADDR = {
    "armv7a-none-eabi": 0x8000,
    "aarch64-unknown-none-softfloat": 0x80000,
}

#: The line a case binary prints last. Reaching it means the run is over and
#: the subprocess can be stopped, rather than waiting out the full timeout.
END_MARKER = b"#HIL end"

#: A panic also ends the run, and waiting for a trailer that will never come
#: would turn every panic into a timeout.
PANIC_MARKER = b"#HIL panic"


def run_finished(buf: bytes | bytearray) -> bool:
    """True once a transcript contains a *complete* terminating line.

    Takes a `bytearray` as well as `bytes` because the read loop accumulates
    into one and checks it on every chunk; copying the whole transcript per
    chunk to satisfy a narrower annotation would be quadratic for nothing.

    The marker alone is not enough. Both terminators carry their payload
    after the marker — the tallies on `#HIL end`, the message on `#HIL panic`
    — so stopping at the marker truncates the very line being waited for, and
    the run then reads as incomplete despite having finished perfectly.
    """
    for marker in (END_MARKER, PANIC_MARKER):
        start = buf.find(marker)
        if start != -1 and b"\n" in buf[start:]:
            return True
    return False


def split_new_lines(buf: bytes, delivered: int) -> tuple[list[str], int]:
    """Returns the complete lines in `buf` after byte `delivered`, and the new
    offset.

    Only whole lines, and each one only once. A serial read lands wherever the
    USB packet boundary fell, so a line routinely arrives in two pieces —
    delivering the first piece and then the second as separate lines would
    hand the runner half a handoff schedule, which parses as no schedule at
    all and sends it into a blind window it has no timings for.
    """
    cut = buf.rfind(b"\n") + 1
    if cut <= delivered:
        return [], delivered
    fresh = buf[delivered:cut].decode("utf-8", "replace")
    return fresh.splitlines(), cut


@contextlib.contextmanager
def _terminal_preserved():
    """Restores this process's terminal settings whatever the child does.

    A belt to the `start_new_session` braces. Detaching should stop the child
    reaching /dev/tty at all, but termios damage is silent, persists after the
    run, and is confusing enough to debug that it is worth being certain —
    the symptom (no echo, staircased output) looks like a broken program
    rather than a broken terminal.
    """
    try:
        fd = sys.stdin.fileno()
        saved = termios.tcgetattr(fd)
    except (AttributeError, ValueError, termios.error, io.UnsupportedOperation):
        # Not a terminal: nothing to protect, and nothing to restore.
        yield
        return
    try:
        yield
    finally:
        # Suppressed rather than reported: this runs on the way out of a run
        # that may already be failing, and a terminal that cannot be restored
        # is not worth replacing the real error with.
        with contextlib.suppress(termios.error):
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)


class LoaderNotFound(RuntimeError):
    """The `rpi-loader` CLI is not on PATH."""

    def __init__(self, binary: str) -> None:
        super().__init__(
            f"{binary!r} not found on PATH. Install it from "
            "https://github.com/joeferner/rpi-loader, or point at it with "
            "RPI_LOADER."
        )


@dataclass
class BootResult:
    """What came back from booting an image."""

    #: Everything the loader CLI and the board printed, interleaved.
    output: bytes
    #: True if the run was cut short rather than reaching its own end.
    timed_out: bool
    #: Seconds from launching the loader to the end marker.
    elapsed: float

    @property
    def report(self) -> console.Report:
        """The `#HIL` protocol parsed out of the transcript."""
        return console.parse(self.output)


class Loader:
    """The `rpi-loader` CLI, pointed at one board's console."""

    def __init__(self, port: str, binary: str | None = None) -> None:
        self.port = port
        self.binary = binary or os.environ.get("RPI_LOADER", "rpi-loader")
        if shutil.which(self.binary) is None:
            raise LoaderNotFound(self.binary)

    def boot(
        self,
        image: str,
        load_addr: int,
        timeout: float = 60.0,
        baud: int | None = None,
        on_line: Callable[[str], None] | None = None,
    ) -> BootResult:
        """Writes `image` to memory, jumps to it, and captures what it says.

        Uses the loader's `boot` subcommand — mem-write, exec, then terminal —
        rather than running those steps separately. The terminal keeps the
        port held continuously, so there is no window between the jump and the
        host reattaching in which the board's first output could be lost. That
        output includes the session banner, so losing it would lose the board
        identity check.

        Returns as soon as the case reports its trailer or panics, rather than
        waiting out `timeout`. The timeout is the backstop for a hang, which
        is how bare-metal cases fail.

        `on_line` is called with each complete line as it arrives, for a case
        the runner has to *answer* rather than only record — the console
        handoff being the one that exists: the board announces it is about to
        take GPIO14/15 and the fixture has to let go of them before it does.
        A callback is free to block, and the handoff one does for the length
        of the blind window; nothing arrives on this console meanwhile, by
        construction, so there is nothing for it to hold up. Exceptions
        propagate rather than being swallowed — a runner that failed to answer
        has produced a transcript that means nothing, and reporting the run
        instead of the failure would hide why.
        """
        argv = [
            self.binary,
            "--device",
            self.port,
            "boot",
            "--load-addr",
            hex(load_addr),
        ]
        if baud is not None:
            argv += ["--baud", str(baud)]
        argv.append(image)

        started = time.monotonic()
        with _terminal_preserved():
            proc = subprocess.Popen(
                argv,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                # A new session, so the child has no controlling terminal.
                #
                # The loader's terminal mode enables raw mode through
                # crossterm, which on Linux does not use stdin: it falls back
                # to opening /dev/tty, i.e. *this* terminal, even though stdin
                # here is /dev/null. Since this side stops the loader with a
                # signal, its restore-on-drop guard never runs, and the
                # terminal is left with no echo and no ONLCR — every later
                # line staircases and nothing typed appears. Detaching means
                # there is no /dev/tty for it to grab in the first place.
                start_new_session=True,
            )

            # `Popen.stdout` is typed `IO[bytes] | None` — None being the case
            # where the caller asked for no pipe, which is not this one — and
            # `IO[bytes]` has no `read1`. With `bufsize` left at its default
            # the object really is a `BufferedReader`, and `read1` is what
            # keeps the loop below from blocking for a full buffer when the
            # board has printed a partial line. Narrowed once here rather than
            # at each of the three uses.
            stdout = cast("io.BufferedReader", proc.stdout)

            buf = bytearray()
            timed_out = True
            # Where the last complete line handed to `on_line` ended, so a
            # chunk that splits a line mid-way does not deliver half of it and
            # then the other half as a second line. The handoff announcement
            # carries its whole schedule, so half of it is unusable.
            delivered = 0
            try:
                sel = selectors.DefaultSelector()
                sel.register(stdout, selectors.EVENT_READ)
                deadline = started + timeout
                while time.monotonic() < deadline:
                    if not sel.select(timeout=0.1):
                        if proc.poll() is not None:
                            # The loader exited on its own, which for `boot` means
                            # it failed before reaching terminal mode.
                            timed_out = False
                            break
                        continue
                    chunk = stdout.read1(4096)
                    if not chunk:
                        timed_out = False
                        break
                    buf += chunk
                    if on_line is not None:
                        lines, delivered = split_new_lines(bytes(buf), delivered)
                        for line in lines:
                            on_line(line)
                    if run_finished(buf):
                        timed_out = False
                        break
            finally:
                # `boot` ends in a terminal that never returns, so it is always
                # this side's job to stop it. Terminate first so the loader can
                # release the port cleanly; kill only if it will not.
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=5)
                stdout.close()

        return BootResult(
            output=bytes(buf),
            timed_out=timed_out,
            elapsed=time.monotonic() - started,
        )

    def _run(self, *args: str, timeout: float = 60.0) -> subprocess.CompletedProcess:
        """Runs a loader subcommand that terminates on its own.

        A timeout is reported as a failed run rather than raised. Every
        caller here is asking a yes/no question about a board that may well
        be halted — which is exactly when the loader hangs — so an exception
        would force each of them to handle the most likely outcome
        separately.
        """
        argv = [self.binary, "--device", self.port, *args]
        try:
            return subprocess.run(
                argv, capture_output=True, timeout=timeout, check=False
            )
        except subprocess.TimeoutExpired as expired:
            return subprocess.CompletedProcess(
                argv,
                returncode=-1,
                stdout=expired.stdout or b"",
                stderr=(
                    f"no response within {timeout}s; the board is probably not "
                    "running the loader".encode()
                ),
            )

    def sd_read(self, remote: str, timeout: float = 60.0) -> bytes:
        """Reads a file off the board's SD card."""
        with tempfile.TemporaryDirectory() as tmp:
            local = os.path.join(tmp, "sd_read")
            done = self._run("sd-read", remote, local, timeout=timeout)
            if done.returncode != 0 or not os.path.exists(local):
                raise SdError(f"sd-read {remote}", done)
            with open(local, "rb") as handle:
                return handle.read()

    def sd_write(self, remote: str, data: bytes, timeout: float = 60.0) -> None:
        """Writes a file to the board's SD card, creating or truncating it.

        The caller should read it back and compare before relying on it: this
        is the only operation in the rig that can make a board unreachable,
        since a corrupt `config.txt` leaves no loader to fix it with.
        """
        with tempfile.TemporaryDirectory() as tmp:
            local = os.path.join(tmp, "sd_write")
            with open(local, "wb") as handle:
                handle.write(data)
            done = self._run("sd-write", local, remote, timeout=timeout)
            if done.returncode != 0:
                raise SdError(f"sd-write {remote}", done)


class SdError(RuntimeError):
    """An `sd-*` subcommand failed, with whatever the loader said."""

    def __init__(self, what: str, done: subprocess.CompletedProcess) -> None:
        detail = (done.stderr or done.stdout or b"").decode("utf-8", "replace").strip()
        super().__init__(f"{what} failed (exit {done.returncode}): {detail}")


#: Execution state the firmware boots into, keyed by what `config.txt` says.
#: `arm_64bit` absent means 32-bit: that is the firmware's own default on
#: these boards, so an absent line is a statement, not an unknown.
def arch_from_config(config: str | bytes) -> str:
    """Reads the execution state a `config.txt` selects.

    Returns the same vocabulary a case binary reports in its banner, so the
    two can be compared directly.
    """
    if isinstance(config, bytes):
        config = config.decode("utf-8", "replace")
    arch = "arm"
    for line in config.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        key, _, value = line.partition("=")
        if key.strip() == "arm_64bit":
            arch = "aarch64" if value.strip() not in ("0", "") else "arm"
    return arch


def config_for_arch(config: str | bytes, arch: str) -> str:
    """Returns `config` edited to select `arch`, preserving everything else.

    Rewrites the existing `arm_64bit` line in place where there is one, and
    appends otherwise. Keeping the rest of the file byte-identical matters:
    it holds the board's own settings — `core_freq`, overlays, the console
    UART — and losing those to an arch switch would change what is under
    test.
    """
    if isinstance(config, bytes):
        config = config.decode("utf-8", "replace")
    want = "1" if arch == "aarch64" else "0"

    lines = config.splitlines()
    for i, line in enumerate(lines):
        body = line.split("#", 1)[0]
        if body.partition("=")[0].strip() == "arm_64bit":
            lines[i] = f"arm_64bit={want}"
            break
    else:
        lines.append(f"arm_64bit={want}")

    return "\n".join(lines) + "\n"


def load_addr_for(target: str) -> int:
    """Load address for a Rust target triple."""
    try:
        return LOAD_ADDR[target]
    except KeyError:
        raise ValueError(
            f"no load address known for {target!r}; add it to LOAD_ADDR"
        ) from None
