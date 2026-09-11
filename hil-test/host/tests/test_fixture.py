"""M1: the fixture answers, identifies itself, and exposes a console port.

These are tests of the *bench*, not of rpi-hal. They exist because a rig
whose own liveness is unverified turns every real failure into a two-sided
mystery — the first thing a run should establish is that the witness works.
"""

from __future__ import annotations

import pytest

from hilbench import Cap, Fixture, ProtocolError, proto
from hilbench.proto import Cmd


def test_protocol_constants_match_firmware() -> None:
    """Guards the hand-maintained mirror of the firmware's wire format.

    The firmware and this module are not generated from a shared source, so
    the numbers are duplicated. A renumbered capability bit would otherwise
    show up as an unrelated case mysteriously skipping.
    """
    assert proto.PROTOCOL_VERSION == 1
    assert proto.MAX_BODY == 62
    assert (proto.USB_VID, proto.USB_PID) == (0x1209, 0x0001)
    assert Cmd.PING == 0x01 and Cmd.HELLO == 0x02
    assert Cmd.CONSOLE_DETACH == 0x10 and Cmd.CONSOLE_ATTACH == 0x11
    assert Cmd.CONSOLE_STATUS == 0x12 and Cmd.CONSOLE_PINS == 0x13
    assert Cmd.CONSOLE_DRIVE == 0x14
    assert [s.value for s in proto.Status] == [0, 1, 2, 3, 4]
    assert Cap.CONSOLE_BRIDGE == 1
    # Bit positions are wire format; renumbering breaks every fixture in the
    # field at once, so pin the whole set rather than spot-checking.
    assert [c.value for c in Cap] == [1 << n for n in range(13)]


def test_encode_rejects_oversized_body() -> None:
    with pytest.raises(ValueError, match="MAX_BODY"):
        proto.encode(Cmd.PING, b"x" * (proto.MAX_BODY + 1))


def test_decode_raises_on_short_packet() -> None:
    with pytest.raises(ProtocolError, match="too short"):
        proto.decode(b"\x00")


def test_decode_raises_on_truncated_body() -> None:
    with pytest.raises(ProtocolError, match="carries"):
        proto.decode(bytes([proto.Status.OK, 4, 1, 2]))


def test_console_pins_decode() -> None:
    """Bit 0 is the board's GPIO14, bit 1 its GPIO15 — not the fixture's own.

    Pinned because the translation happens in exactly one place, and getting
    it backwards would swap the pin a case drives for the pin it does not,
    producing a confident report about the wrong wire.
    """
    assert proto.parse_console_pins(b"\x00") == proto.ConsolePins(False, False)
    assert proto.parse_console_pins(b"\x01") == proto.ConsolePins(True, False)
    assert proto.parse_console_pins(b"\x02") == proto.ConsolePins(False, True)
    assert proto.parse_console_pins(b"\x03") == proto.ConsolePins(True, True)


def test_console_pins_rejects_a_wrong_length_body() -> None:
    with pytest.raises(ProtocolError, match="1 byte"):
        proto.parse_console_pins(b"")


def test_decode_surfaces_error_status() -> None:
    with pytest.raises(ProtocolError, match="UNSUPPORTED"):
        proto.decode(bytes([proto.Status.UNSUPPORTED, 0]))


def test_ping(bench: Fixture) -> None:
    bench.ping()


def test_hello_identifies_the_fixture(bench: Fixture) -> None:
    hello = bench.hello()
    assert hello.protocol_version == proto.PROTOCOL_VERSION
    # Whatever else it reports, a fixture with no capabilities at all is
    # misconfigured rather than merely limited.
    assert hello.capabilities, "fixture reports no capabilities"
    print(f"\n{hello}")


def test_console_bridge_is_present(bench: Fixture) -> None:
    """The bridge is the one capability M1 is about."""
    assert bench.has(Cap.CONSOLE_BRIDGE), "fixture does not bridge the console"


def test_console_port_resolves(console_port: str) -> None:
    """The console must appear as a serial device for the loader CLI."""
    assert console_port.startswith("/dev/")
    print(f"\nconsole: {console_port}")


def test_console_port_is_openable(console_port: str) -> None:
    """Resolving the path is not enough — the loader has to be able to open it.

    Split from the test above because they fail for unrelated reasons and
    want unrelated fixes: a path that does not resolve means the CDC driver
    did not bind, while a path that resolves but will not open means
    permissions. Asserting only the former reports green on a console the
    loader cannot use, which is the rig lying about itself.
    """
    import errno as errno_mod

    import serial

    try:
        with serial.Serial(console_port, 115200, timeout=0.1):
            pass
    except serial.SerialException as exc:
        code = exc.errno or getattr(exc.args[0] if exc.args else None, "errno", None)
        if code == errno_mod.EACCES or "Permission denied" in str(exc):
            pytest.fail(
                f"{console_port} exists but cannot be opened: {exc}. "
                'The udev rule needs its SUBSYSTEM=="tty" line installed '
                "(and a replug), or join the serial group — uucp on Arch, "
                "dialout on Debian.",
                pytrace=False,
            )
        if code == errno_mod.EBUSY:
            # Someone else holds the port, which is a normal state during a
            # real run. It also answers the question this test asks: an
            # exclusive open gets EACCES before EBUSY, so reaching "busy" at
            # all proves permissions are fine.
            pytest.skip(f"{console_port} is open in another process")
        raise


def test_unknown_opcode_is_rejected(bench: Fixture) -> None:
    """An unknown command must be refused, not silently accepted.

    A fixture that answers OK to anything would make every capability probe
    succeed and every skip disappear.
    """
    unallocated = 0x7F
    assert unallocated not in {int(c) for c in Cmd}
    with pytest.raises(ProtocolError, match="BAD_COMMAND"):
        bench.request(unallocated)


@pytest.mark.parametrize("baud", [115200, 921600, 1500000])
def test_console_loopback(
    request: pytest.FixtureRequest, console_port: str, baud: int
) -> None:
    """With TX jumpered to RX, the bridge must echo bytes back unchanged.

    This is the fixture's self-test, and the first thing to reach for when
    the loader cannot talk to a board: it exercises the whole firmware path
    — CDC out, UART TX, UART RX, CDC in — with the board and the wiring
    taken out of the picture. Passing here means a HELLO timeout is
    downstream; failing here means the bridge itself.

    Parametrised over the rates the loader actually uses. It idles at 115200
    and negotiates up to 1.5 Mbaud for bulk transfers, so a `SET_LINE_CODING`
    bug shows up as "handshake fine, large transfers corrupt" — testing only
    the idle rate would miss exactly the case that matters.
    """
    if not request.config.getoption("--console-loopback"):
        pytest.skip("needs --console-loopback and a TX-to-RX jumper on the fixture")

    import serial

    # A pattern with alternating bit runs: 0x00/0xFF catch a stuck line, 0x55/
    # 0xAA catch a bit-order or framing error, and the ramp catches a rate
    # error that only corrupts some symbols.
    payload = bytes([0x00, 0xFF, 0x55, 0xAA]) + bytes(range(256))

    with serial.Serial(console_port, baud, timeout=2.0) as port:
        port.reset_input_buffer()
        port.write(payload)
        port.flush()
        echoed = port.read(len(payload))

    assert len(echoed) == len(payload), (
        f"at {baud} baud, sent {len(payload)} bytes and got {len(echoed)} back"
    )
    assert echoed == payload, f"at {baud} baud, the echo differs from what was sent"


def test_console_status_reports_attached_at_rest(bench: Fixture) -> None:
    """The resting state of a fixture is a working console.

    Its own test rather than a line in the handoff suite: this is what every
    other test in the session assumes on entry, so when it is wrong the
    failure worth seeing is this one, not the twenty downstream of it. A
    fixture found detached here means something earlier crashed inside the
    blind window and left the bench without a console.
    """
    assert bench.console_attached(), (
        "the fixture is sitting detached, so nothing can reach the board. "
        "Something left the console released; power-cycle the fixture."
    )
