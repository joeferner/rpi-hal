"""The fixture control-interface wire format.

Mirrors ``firmware/src/proto.rs`` and ``firmware/src/board.rs``. The two are
not generated from a shared source, so ``PROTOCOL.md`` is the contract and
this module is checked against it by ``test_protocol_constants``.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass

#: Wire format version. The runner refuses a fixture reporting anything else
#: rather than guessing at a mismatched layout.
PROTOCOL_VERSION = 1

#: Largest request or response body: one full-speed bulk packet less the
#: two-byte header, so every exchange is a single transfer.
MAX_BODY = 62

#: pid.codes test allocation. Matching on VID/PID is how the control
#: interface is found without caring which ttyACM number the console got.
USB_VID = 0x1209
USB_PID = 0x0001

#: USB class of the control interface. Deliberately vendor-specific rather
#: than a second CDC, so capture buffers travel as binary with no terminal
#: layer in the path.
CONTROL_CLASS = 0xFF


class Cmd(enum.IntEnum):
    """Request opcodes."""

    PING = 0x01
    HELLO = 0x02
    CONSOLE_DETACH = 0x10
    CONSOLE_ATTACH = 0x11
    CONSOLE_STATUS = 0x12
    CONSOLE_PINS = 0x13
    CONSOLE_DRIVE = 0x14
    MARKER_ARM = 0x20
    MARKER_STATUS = 0x21
    MARKER_READ = 0x22
    MARKER_PULSE = 0x23


class Status(enum.IntEnum):
    """Response status codes."""

    OK = 0x00
    BAD_COMMAND = 0x01
    BAD_ARGS = 0x02
    #: Understood, but this fixture lacks the capability. Distinct from
    #: BAD_COMMAND so the runner can skip rather than error.
    UNSUPPORTED = 0x03
    #: Understood and well-formed, but not valid right now — driving the
    #: console pins while the bridge still owns them, for instance. Distinct
    #: from BAD_ARGS because the same request is correct a moment later, so a
    #: sequencing bug should not send anyone to read the codec.
    BAD_STATE = 0x04


class Board(enum.IntEnum):
    """Which fixture board answered."""

    RP2040_PICO = 1
    #: Olimex PICO2-XL or PICO2-XXL. One value for both: they are the same
    #: PCB and the same pin map, and nothing on either reports which it is.
    RP2350_PICO2X = 2


class Cap(enum.IntFlag):
    """Capability bits reported by :attr:`Cmd.HELLO`.

    Bit positions are wire format: allocated once, never renumbered. An
    absent bit means the runner skips the cases that need it — never that it
    fails them.
    """

    CONSOLE_BRIDGE = 1 << 0
    GPIO_SHADOW = 1 << 1
    MARKER_TIMESTAMP = 1 << 2
    SPI_SLAVE = 1 << 3
    I2C_SLAVE = 1 << 4
    LOGIC_CAPTURE = 1 << 5
    I2S_CAPTURE = 1 << 6
    AUDIO_ADC = 1 << 7
    POWER_SWITCH = 1 << 8
    RAIL_SENSE = 1 << 9
    CURRENT_SENSE = 1 << 10
    USB_VBUS_SWITCH = 1 << 11
    RUN_RESET = 1 << 12


class ProtocolError(RuntimeError):
    """A response that was malformed, or a status other than OK."""


@dataclass(frozen=True)
class ConsolePins:
    """Levels on the two console pins, named for the *board's* numbering.

    The fixture's own GP0/GP1 are an artefact of how the bench happens to be
    wired; what a case asserts about is its own GPIO14/15, so the translation
    happens once, at the fixture, rather than at every call site.
    """

    #: The board's TXD0, which the fixture receives on.
    gpio14: bool
    #: The board's RXD0, which the fixture transmits on.
    gpio15: bool

    def __str__(self) -> str:
        return f"gpio14={int(self.gpio14)} gpio15={int(self.gpio15)}"


def parse_console_pins(body: bytes) -> ConsolePins:
    """Decodes a CONSOLE_PINS body: one byte, bit 0 GPIO14, bit 1 GPIO15."""
    if len(body) != 1:
        raise ProtocolError(f"CONSOLE_PINS body should be 1 byte, got {len(body)}")
    return ConsolePins(gpio14=bool(body[0] & 1), gpio15=bool(body[0] & 2))


@dataclass(frozen=True)
class MarkerStatus:
    """State of the fixture's marker-pin capture."""

    #: Edges recorded so far. Derived from what the DMA has left to do, so it
    #: cannot disagree with what is actually in the buffer.
    captured: int
    #: The state machine stalled at some point, so edges are missing from the
    #: middle of the capture. The intervals either side of a gap still look
    #: entirely plausible, which is why this has to be checked rather than
    #: inferred from the data.
    overflowed: bool
    #: Ticks per second of the capture timebase. Reported rather than assumed:
    #: it follows the fixture's system clock, and hardcoding it here would let
    #: a firmware clock change silently rescale every measurement.
    tick_hz: int
    #: How many edges one capture can hold.
    capacity: int
    #: Which fixture GPIO is being watched.
    pin: int

    def __str__(self) -> str:
        return (
            f"{self.captured}/{self.capacity} edges on GP{self.pin} at "
            f"{self.tick_hz / 1e6:.1f} MHz"
            + (" (OVERFLOWED)" if self.overflowed else "")
        )


def parse_marker_status(body: bytes) -> MarkerStatus:
    """Decodes a MARKER_STATUS body."""
    if len(body) != 10:
        raise ProtocolError(f"MARKER_STATUS body should be 10 bytes, got {len(body)}")
    return MarkerStatus(
        captured=int.from_bytes(body[0:2], "little"),
        overflowed=bool(body[2] & 1),
        tick_hz=int.from_bytes(body[3:7], "little"),
        capacity=int.from_bytes(body[7:9], "little"),
        pin=body[9],
    )


@dataclass(frozen=True)
class Hello:
    """What the fixture reports about itself."""

    protocol_version: int
    board: Board
    capabilities: Cap
    firmware_version: tuple[int, int, int]

    def __str__(self) -> str:
        major, minor, patch = self.firmware_version
        caps = ", ".join(c.name for c in Cap if c in self.capabilities) or "none"
        return (
            f"{self.board.name} fw {major}.{minor}.{patch} "
            f"proto {self.protocol_version} caps: {caps}"
        )


def encode(cmd: Cmd | int, body: bytes = b"") -> bytes:
    """Frames a request. Raises if the body will not fit one packet.

    Accepts a bare `int` as well as a :class:`Cmd`, because sending an opcode
    this module does not know is a real operation rather than a mistake:
    checking that a fixture answers `BAD_COMMAND` to an unallocated one is how
    you establish it is not saying OK to everything, and a client that could
    only send allocated opcodes could not ask that question.
    """
    if len(body) > MAX_BODY:
        raise ValueError(f"body of {len(body)} exceeds MAX_BODY ({MAX_BODY})")
    return bytes([int(cmd), len(body)]) + body


def decode(packet: bytes) -> bytes:
    """Unwraps a response body, raising :class:`ProtocolError` unless OK."""
    if len(packet) < 2:
        raise ProtocolError(f"response of {len(packet)} bytes is too short")
    status, length = packet[0], packet[1]
    body = packet[2 : 2 + length]
    if len(body) != length:
        raise ProtocolError(
            f"response claims a {length}-byte body but carries {len(body)}"
        )
    if status != Status.OK:
        try:
            name = Status(status).name
        except ValueError:
            name = f"unknown status 0x{status:02x}"
        raise ProtocolError(name)
    return body


def parse_hello(body: bytes) -> Hello:
    """Decodes a HELLO body: version, board, 32-bit caps, 3-byte firmware."""
    if len(body) != 9:
        raise ProtocolError(f"HELLO body should be 9 bytes, got {len(body)}")
    return Hello(
        protocol_version=body[0],
        board=Board(body[1]),
        capabilities=Cap(int.from_bytes(body[2:6], "little")),
        firmware_version=(body[6], body[7], body[8]),
    )
