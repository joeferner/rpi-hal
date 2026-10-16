"""Shared pytest fixtures for the HIL suite.

Rig hardware maps onto pytest fixtures literally, which is the reason for
using pytest here: capability-driven skipping comes for free, and the
board/arch matrix becomes parametrisation rather than bespoke loop code.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import pytest

from hilbench import Cap, Fixture, FixtureNotFound, FixturePermissionError


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--fixture-serial",
        default=None,
        help="USB serial of the bench fixture to drive (required with a rack)",
    )
    parser.addoption(
        "--console-loopback",
        action="store_true",
        help=(
            "the fixture's console TX and RX are jumpered together "
            "(GP0 to GP1 on a Pico); enables the bridge self-test"
        ),
    )
    parser.addoption(
        "--series-resistor",
        action="store_true",
        help=(
            "each signal line between fixture and board has a series resistor "
            "in it, as the HAT will; enables the contention check, which "
            "deliberately drives against the board's UART and needs the "
            "resistor to be safe"
        ),
    )
    parser.addoption(
        "--case-target",
        default="armv7a-none-eabi",
        help="Rust target of the case images to drive; must match how the board booted",
    )
    parser.addoption(
        "--case-chip",
        default="bcm2837",
        help="chip feature the case images were built with (bcm2837 or bcm2711)",
    )
    parser.addoption(
        "--image-dir",
        default="cases/target/images",
        help="where `make images` writes the raw case images",
    )
    parser.addoption(
        "--manual-reset",
        action="store_true",
        help=(
            "a human is present and will power-cycle the board when asked; "
            "stands in for the HAT's load switch. Never use unattended"
        ),
    )


@pytest.fixture(scope="session")
def bench(request: pytest.FixtureRequest) -> Fixture:
    """The attached bench fixture, or a skip if there isn't one.

    Session-scoped: the USB connection is stateless between cases, and
    reopening it per test would cost a re-enumeration for nothing.

    Absent hardware *skips*. A missing fixture is "this rig cannot answer
    that question", not a defect in the crate, and a suite that fails on it
    is a suite nobody runs on a laptop.
    """
    serial = request.config.getoption("--fixture-serial")
    try:
        fixture = Fixture.open(serial)
    except FixtureNotFound as exc:
        pytest.skip(f"no bench fixture: {exc}")

    # Probe access once, here, rather than letting every case discover it
    # independently. A fixture that is attached but unreachable is a setup
    # problem the operator has to fix, so it *fails* rather than skipping —
    # skipping would quietly report green on a bench that is not working.
    try:
        fixture.ping()
    except FixturePermissionError as exc:
        fixture.close()
        pytest.fail(str(exc), pytrace=False)

    request.addfinalizer(fixture.close)
    return fixture


@pytest.fixture(scope="session")
def console_port(bench: Fixture) -> str:
    """Path of the tunnelled board console, for handing to ``rpi-loader``."""
    try:
        return bench.console_port
    except FixtureNotFound as exc:
        pytest.skip(str(exc))


@pytest.fixture(scope="session")
def reset_board(request: pytest.FixtureRequest, bench: Fixture):
    """Returns a callable that power-cycles the board under test.

    One interface over three backings, so a case never learns how the reset
    happened: the fixture's load switch once the HAT exists, a human with a
    switch in the meantime, or a skip. When the electrical path arrives,
    nothing that calls this changes.

    Power-cycling rather than a warm reset is the primitive worth having,
    because it is the one that also resets the LAN9514, the wireless chip and
    anything on USB — a `RUN` reset leaves those in whatever state they
    wedged in, which is exactly the state a recovery path must not inherit.

    The caller is responsible for re-establishing the loader afterwards; this
    only guarantees the board went away and came back.
    """
    if bench.has(Cap.POWER_SWITCH):
        pytest.fail(
            "the fixture reports POWER_SWITCH but the host has no command to "
            "drive it — add it to the control protocol and wire it in here",
            pytrace=False,
        )

    if not request.config.getoption("--manual-reset"):
        pytest.skip(
            "no power switch on the fixture; pass --manual-reset to be "
            "prompted, or wait for the HAT"
        )

    # Refuse rather than block. An unattended run that reaches an `input()`
    # hangs until its job times out, with no indication of why — so require
    # a terminal and fail loudly without one.
    if not sys.stdin.isatty():
        pytest.fail(
            "--manual-reset needs a terminal to prompt on, and stdin is not "
            "one. Drop the flag for unattended runs.",
            pytrace=False,
        )

    capman = request.config.pluginmanager.getplugin("capturemanager")

    def reset(reason: str = "") -> None:
        why = f" ({reason})" if reason else ""
        with capman.global_and_fixture_disabled():
            print(f"\n>>> Power-cycle the board{why}, then press Enter: ", end="")
            input()

    return reset


@pytest.fixture(scope="session")
def board_ready(request: pytest.FixtureRequest, loader):
    """Ensures the board is answering the loader, resetting it if it is not.

    A case that jumps into a bad image leaves the board running whatever
    those bytes decoded to — usually nothing — and every later test then
    fails at the handshake for a reason unrelated to what it was testing.
    That turns one real failure into a run of noise, so state is established
    before each load rather than assumed.

    Liveness is a handshake, never a wait for the loader's boot banner. Two
    reasons, and the second is the one that bites. The banner prints once at
    startup, so a board that has been idle in the loader says nothing at all
    and would be declared dead every run. And it is one-shot: whoever waits
    for it must already hold the console when the board boots, which is
    impossible to guarantee around a manual power cycle — press the switch
    before pressing Enter and the banner is gone. A resident loader answers a
    command whenever it is asked, so asking has no ordering to get wrong.
    """

    def alive(timeout: float = 8.0) -> bool:
        return loader._run("sd-list", "/", timeout=timeout).returncode == 0

    def wait_alive(seconds: float = 30.0) -> bool:
        """Retries across a boot, rather than assuming one attempt lands.

        A board mid-boot fails the handshake without being broken, so a
        single attempt after a reset would report a healthy board dead and
        ask for another power cycle.
        """
        deadline = time.monotonic() + seconds
        while True:
            if alive(timeout=6.0):
                return True
            if time.monotonic() >= deadline:
                return False

    def ensure() -> None:
        if alive():
            return
        # Ask the option directly rather than letting `reset_board` skip:
        # `pytest.skip` raises through `BaseException`, so it cannot be
        # caught as an ordinary error, and the skip would replace this
        # message with a vaguer one.
        if not request.config.getoption("--manual-reset"):
            pytest.fail(
                "the board is not answering the loader. It is probably "
                "halted after a previous case; power-cycle it, or pass "
                "--manual-reset so the runner can ask you to.",
                pytrace=False,
            )
        request.getfixturevalue("reset_board")("it is not answering the loader")
        if not wait_alive():
            pytest.fail(
                "the board still does not answer after a power cycle. Check "
                "it is powered, that rpi-loader is on its card, and that the "
                "console pins reach the fixture.",
                pytrace=False,
            )

    return ensure


@pytest.fixture(scope="session")
def board_arch(
    request: pytest.FixtureRequest,
    loader,
    case_target: str,
    board_ready,
):
    """Makes the board's execution state match the images about to be run.

    Depends on `board_ready` rather than trusting a caller to sequence the
    two, because everything below talks to the board through the loader: on
    a halted board the first `sd-read` just waits out its timeout, and the
    reset that would have fixed it never gets offered. Expressing it as a
    dependency makes the order impossible to get wrong.

    Read first, write only on a mismatch. Reading is safe and usually enough,
    while writing `config.txt` is the one operation here that can make a board
    unreachable — a corrupt one leaves no loader to repair it with. So the
    order is read, compare, and only then write, verify, and power-cycle.

    This is also the *only* way to catch an arch mismatch before it wastes a
    run. Loading a 32-bit image into a board running AArch64 does not fail: the
    transfer succeeds, the jump succeeds, and the bytes decode as garbage, so
    the board goes quiet with no error anywhere. There is no banner left to
    compare against, which is why the check has to happen before the load.
    """
    from hilbench.loader import SdError, arch_from_config, config_for_arch

    board_ready()

    want = "aarch64" if "aarch64" in case_target else "arm"

    try:
        current = loader.sd_read("/config.txt")
    except SdError as exc:
        pytest.skip(f"cannot read config.txt to confirm the board's arch: {exc}")

    if arch_from_config(current) == want:
        return want

    if not request.config.getoption("--manual-reset"):
        pytest.skip(
            f"board is in {arch_from_config(current)} but the images are "
            f"{want}. Switching needs a power cycle — pass --manual-reset, "
            f"or run with --case-target for the state it is already in."
        )
    reset = request.getfixturevalue("reset_board")

    updated = config_for_arch(current, want)
    loader.sd_write("/config.txt", updated.encode())

    # Read back *before* power-cycling, while there is still a loader to fix
    # it with. After the reset a bad write is unrecoverable without a card
    # reader, so this is the last safe moment to check.
    verify = loader.sd_read("/config.txt")
    if arch_from_config(verify) != want:
        pytest.fail(
            "config.txt did not read back as written; the board has NOT been "
            f"rebooted, so it is still usable. Wanted {want}, got "
            f"{arch_from_config(verify)}.",
            pytrace=False,
        )

    reset(f"switching the board to {want}")

    # Reuse the readiness check rather than waiting for a banner here: same
    # handshake, same retry across the boot, and no second place to get the
    # ordering wrong.
    board_ready()

    # Confirm the switch took, from the card rather than from a case binary.
    # A mismatched image runs as garbage and says nothing, so there would be
    # no banner to compare against afterwards.
    settled = arch_from_config(loader.sd_read("/config.txt"))
    if settled != want:
        pytest.fail(
            f"config.txt says {settled} after switching to {want}", pytrace=False
        )
    return want


@pytest.fixture(scope="session")
def case_target(request: pytest.FixtureRequest) -> str:
    """Which build of the case binaries this run drives.

    Execution state is fixed by the board's firmware at reset, so this has to
    match how the board actually booted rather than being chosen freely — the
    session banner's `arch` field is what confirms it did.
    """
    return request.config.getoption("--case-target")


@pytest.fixture(scope="session")
def image_dir(request: pytest.FixtureRequest) -> Path:
    root = Path(request.config.getoption("--image-dir"))
    if not root.is_absolute():
        root = (Path(__file__).parent.parent.parent / root).resolve()
    return root


@pytest.fixture(scope="session")
def loader(console_port: str):
    """The `rpi-loader` CLI pointed at the board behind the fixture."""
    from hilbench.loader import Loader, LoaderNotFound

    try:
        return Loader(console_port)
    except LoaderNotFound as exc:
        pytest.skip(str(exc))


@pytest.fixture(scope="session")
def case_image(request: pytest.FixtureRequest, image_dir: Path, case_target: str):
    """Resolves a case name to its built raw image, or skips.

    Skips rather than fails when the image is missing: not having run `make
    images` is a workflow gap, not a defect in the crate, and a suite that
    fails on it trains people to ignore red.
    """
    chip = request.config.getoption("--case-chip")

    def resolve(name: str) -> Path:
        path = image_dir / f"{name}-{case_target}-{chip}.img"
        if not path.exists():
            pytest.skip(
                f"{path.name} not built — run: make images "
                f"TARGET={case_target} CHIP={chip}"
            )
        return path

    return resolve


def requires(*caps: Cap):
    """Marks a test as needing fixture capabilities, skipping if absent.

    Written as a decorator taking the capability set rather than a plain
    ``skipif``, so the skip reason names exactly which capability was
    missing — a report that only says "skipped" cannot be acted on.
    """

    def decorator(func):
        @pytest.mark.usefixtures("bench")
        def wrapper(bench: Fixture, *args, **kwargs):
            absent = bench.missing(*caps)
            if absent:
                pytest.skip(f"fixture lacks {', '.join(absent)}")
            return func(bench, *args, **kwargs)

        wrapper.__name__ = func.__name__
        wrapper.__doc__ = func.__doc__
        return wrapper

    return decorator
