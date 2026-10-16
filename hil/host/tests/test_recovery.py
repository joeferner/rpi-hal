"""The recovery primitive: can the rig get a board back?

Bare-metal cases hang rather than fail, so every unattended run depends on
being able to power-cycle a wedged board and carry on. That makes reset
infrastructure rather than a convenience, and infrastructure that has never
been exercised is a guess.

These are opt-in because the only reset available right now is a human with
a switch. They become part of the ordinary suite the moment the HAT's load
switch lands, with no change to what they assert.
"""

from __future__ import annotations

import time

import pytest


def _drain(port, seconds: float) -> bytes:
    """Reads whatever arrives within a window, rather than a fixed count.

    A boot produces an unknown number of bytes, so asking for a specific
    length would either block past the end or truncate the evidence.
    """
    deadline = time.monotonic() + seconds
    chunks = []
    while time.monotonic() < deadline:
        waiting = port.in_waiting
        if waiting:
            chunks.append(port.read(waiting))
        else:
            time.sleep(0.05)
    return b"".join(chunks)


def test_board_talks_after_power_cycle(console_port: str, reset_board) -> None:
    """A power-cycled board must come back and say something.

    This is the assertion the whole recovery loop rests on. Without it, a
    timeout handler can power-cycle into a board that never returns and the
    run continues reporting failures that are really one dead board.

    Asserts only that bytes arrive, not what they say: what a board emits on
    boot depends on which loader and which firmware, and pinning that here
    would make the recovery check fail for reasons unrelated to recovery.
    """
    import serial

    with serial.Serial(console_port, 115200, timeout=0.2) as port:
        port.reset_input_buffer()
        reset_board("so the console shows a fresh boot")

        boot = _drain(port, seconds=10.0)

    assert boot, (
        "nothing arrived on the console within 10s of the power cycle. "
        "The board did not come back, is not running the loader, or the "
        "console pins are not wired to the fixture."
    )
    print(f"\n{len(boot)} bytes after reset: {boot[:120]!r}")


def test_loader_ready_after_power_cycle(console_port: str, reset_board) -> None:
    """The board must come back *usable*, not merely alive.

    A separate claim from the test above, and the one the recovery loop
    actually needs: bytes on the console prove the board booted, while the
    loader's banner proves it will accept the next command. A runner that
    resumed on the first would race the second.

    Waiting for the banner rather than sleeping also makes recovery as fast
    as the board allows, which matters once a run power-cycles many times.
    """
    import serial

    from hilbench import console

    with serial.Serial(console_port, 115200, timeout=0.2) as port:
        port.reset_input_buffer()
        reset_board("waiting for the loader to announce itself")

        try:
            seen = console.wait_for_loader(port, timeout=15.0)
        except console.Timeout as exc:
            pytest.fail(
                f"the loader never became ready: {exc}. The board booted into "
                "something else, or rpi-loader is not on its card.",
                pytrace=False,
            )

    print(f"\nloader ready after {len(seen)} bytes")


def test_console_survives_power_cycle(console_port: str, reset_board) -> None:
    """The fixture's own USB link must outlive the board's power cycle.

    The point of powering the board from the bench rather than the fixture:
    if a reset took the console with it, the runner would lose its own
    transport at the exact moment it needs to report why. Cutting board
    power must be invisible from the host side.
    """
    import serial

    reset_board("checking the fixture stays up across it")

    # Reopening proves the port still exists; the fixture never re-enumerated.
    with serial.Serial(console_port, 115200, timeout=0.2) as port:
        assert port.is_open


@pytest.mark.parametrize("cycle", [1, 2])
def test_repeated_power_cycles(console_port: str, reset_board, cycle: int) -> None:
    """Reset has to be repeatable, not a one-off.

    A recovery loop may power-cycle the same board many times in a run, and a
    path that works once but leaves rails partly charged the second time is
    the classic cause of a hung half-reset. Two cycles is not proof, but it
    catches the case that only ever worked from cold.
    """
    import serial

    with serial.Serial(console_port, 115200, timeout=0.2) as port:
        port.reset_input_buffer()
        reset_board(f"cycle {cycle} of 2")
        boot = _drain(port, seconds=10.0)

    assert boot, f"no console output after power cycle {cycle}"
