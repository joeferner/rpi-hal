"""Client for the bench fixture.

One USB device, two interfaces: a vendor-specific bulk pair carrying this
protocol, and a CDC ACM interface carrying the board's console. This module
owns the former and merely *locates* the latter, because the console is
handed to ``rpi-loader``'s CLI as a device path rather than driven here.
"""

from __future__ import annotations

import contextlib
import errno
from dataclasses import dataclass

import usb.core
import usb.util

from . import proto
from .proto import Cap, Cmd, Hello, ProtocolError

#: Long enough to cross a USB frame and the firmware's turnaround, short
#: enough that a wedged fixture fails a case instead of hanging the suite.
TIMEOUT_MS = 1000


class FixtureNotFound(RuntimeError):
    """No fixture matching the requested serial is attached."""


class FixturePermissionError(RuntimeError):
    """A fixture is attached but this user cannot talk to it.

    Its own error class because libusb reports the condition as a bare
    ``Errno 13`` or, worse, as ``ValueError: The device has no langid`` from
    deep inside a string-descriptor read — neither of which names the cause
    or the fix.
    """

    HINT = (
        "install the udev rule -- sudo cp host/udev/99-hil-fixture.rules "
        "/etc/udev/rules.d/ && sudo udevadm control --reload-rules && "
        "sudo udevadm trigger -- then replug the fixture. If it is already "
        "installed, check you are in the plugdev group (`groups`): the rule's "
        "uaccess tag only covers an active local seat, not SSH, a remote "
        "display, or an unattended runner"
    )

    def __init__(self, detail: str) -> None:
        super().__init__(f"cannot access the fixture ({detail}); {self.HINT}")


@dataclass
class _Endpoints:
    interface: int
    out_addr: int
    in_addr: int


class Fixture:
    """An attached bench fixture."""

    def __init__(self, device: usb.core.Device, endpoints: _Endpoints) -> None:
        self._dev = device
        self._ep = endpoints
        self._hello: Hello | None = None

    # -- discovery ----------------------------------------------------------

    @classmethod
    def find_all(cls) -> list[Fixture]:
        """Returns every attached fixture, in whatever order libusb reports."""
        found = []
        for dev in usb.core.find(
            find_all=True, idVendor=proto.USB_VID, idProduct=proto.USB_PID
        ):
            endpoints = cls._locate_control(dev)
            if endpoints is not None:
                found.append(cls(dev, endpoints))
        return found

    @classmethod
    def open(cls, serial: str | None = None) -> Fixture:
        """Opens one fixture, optionally selecting it by USB serial number.

        Selecting by serial is how a rack addresses a specific board: the
        fixture is permanently mated to one Pi, so its serial *is* the board's
        identity, and the runner cross-references it against ``bench.toml``.
        """
        candidates = cls.find_all()
        if serial is not None:
            candidates = [f for f in candidates if f.serial == serial]
        if not candidates:
            which = f" with serial {serial!r}" if serial else ""
            raise FixtureNotFound(f"no HIL fixture{which} attached")
        if len(candidates) > 1 and serial is None:
            serials = ", ".join(repr(f.serial) for f in candidates)
            raise FixtureNotFound(
                f"{len(candidates)} fixtures attached ({serials}); pass a serial"
            )
        return candidates[0]

    @staticmethod
    def _locate_control(dev: usb.core.Device) -> _Endpoints | None:
        """Finds the vendor-class interface and its bulk endpoint pair.

        Located by interface class rather than by index, so adding another
        interface to the firmware later cannot silently repoint this at the
        console.
        """
        for cfg in dev:
            for intf in cfg:
                if intf.bInterfaceClass != proto.CONTROL_CLASS:
                    continue
                out_ep = in_ep = None
                for ep in intf:
                    is_bulk = (
                        usb.util.endpoint_type(ep.bmAttributes)
                        == usb.util.ENDPOINT_TYPE_BULK
                    )
                    if not is_bulk:
                        continue
                    if usb.util.endpoint_direction(ep.bEndpointAddress) == (
                        usb.util.ENDPOINT_OUT
                    ):
                        out_ep = ep.bEndpointAddress
                    else:
                        in_ep = ep.bEndpointAddress
                if out_ep is not None and in_ep is not None:
                    return _Endpoints(intf.bInterfaceNumber, out_ep, in_ep)
        return None

    # -- identity -----------------------------------------------------------

    @property
    def serial(self) -> str | None:
        """The USB serial number string, or `None` if the device omits one.

        Reading it pulls a string descriptor, which is the first thing to
        fail without permission — so this is usually where a missing udev
        rule surfaces.
        """
        try:
            return self._dev.serial_number
        except ValueError as exc:  # pyusb's "no langid" for a denied read
            raise FixturePermissionError(str(exc)) from exc
        except usb.core.USBError as exc:
            raise FixturePermissionError(str(exc)) from exc

    def __repr__(self) -> str:
        # Never raises: a repr that throws turns a readable test failure into
        # an unreadable one, which is exactly what a permission problem did.
        try:
            return f"<Fixture serial={self.serial!r}>"
        except FixturePermissionError:
            return "<Fixture (inaccessible: see udev rule)>"

    @property
    def console_port(self) -> str:
        """Path of the CDC interface carrying the board's console.

        This is what gets passed to ``rpi-loader --device``. Resolved through
        pyserial's port list by matching VID/PID and serial, so it survives
        the ``ttyACM`` numbering changing between boots.
        """
        from serial.tools import list_ports

        for port in list_ports.comports():
            if port.vid != proto.USB_VID or port.pid != proto.USB_PID:
                continue
            if self.serial is not None and port.serial_number != self.serial:
                continue
            return port.device
        raise FixtureNotFound(
            f"fixture {self.serial!r} has no CDC console port; "
            "is the cdc-acm driver bound?"
        )

    # -- protocol -----------------------------------------------------------

    def request(self, cmd: Cmd | int, body: bytes = b"") -> bytes:
        """Sends one request and returns the response body.

        Raises :class:`ProtocolError` on any status other than OK, so a caller
        that wants to *tolerate* `UNSUPPORTED` has to say so explicitly rather
        than by ignoring a return value.

        `cmd` may be a bare opcode: probing an unallocated one is how a fixture
        is shown to reject what it does not understand. See :func:`proto.encode`.
        """
        try:
            self._dev.write(self._ep.out_addr, proto.encode(cmd, body), TIMEOUT_MS)
            packet = self._dev.read(self._ep.in_addr, 64, TIMEOUT_MS)
        except usb.core.USBError as exc:
            if exc.errno == errno.EACCES:
                raise FixturePermissionError(str(exc)) from exc
            raise
        return proto.decode(bytes(packet))

    def ping(self) -> None:
        """Liveness check."""
        self.request(Cmd.PING)

    def hello(self, refresh: bool = False) -> Hello:
        """Identifies the fixture, caching the answer.

        Cached because capabilities cannot change without the firmware
        restarting, and every skip decision consults them.
        """
        if self._hello is None or refresh:
            self._hello = proto.parse_hello(self.request(Cmd.HELLO))
            if self._hello.protocol_version != proto.PROTOCOL_VERSION:
                raise ProtocolError(
                    f"fixture speaks protocol {self._hello.protocol_version}, "
                    f"host speaks {proto.PROTOCOL_VERSION}"
                )
        return self._hello

    def has(self, *caps: Cap) -> bool:
        """True only if the fixture reports every one of `caps`."""
        have = self.hello().capabilities
        return all(cap in have for cap in caps)

    # -- the console handoff ------------------------------------------------

    def console_detach(self) -> None:
        """Releases GPIO14/15 so a case can drive them.

        The bridge stops there and then: bytes the host writes while detached
        are dropped rather than queued, because a queued byte would arrive at
        the board the instant the console came back — which is precisely when
        the board is re-establishing its own end of it.
        """
        self.request(Cmd.CONSOLE_DETACH)

    def console_attach(self) -> None:
        """Resumes the console bridge, at whatever baud was last set."""
        self.request(Cmd.CONSOLE_ATTACH)

    def console_attached(self) -> bool:
        """Whether the bridge currently owns the pins."""
        body = self.request(Cmd.CONSOLE_STATUS)
        if len(body) != 1:
            raise ProtocolError(
                f"CONSOLE_STATUS body should be 1 byte, got {len(body)}"
            )
        return bool(body[0])

    def console_pins(self) -> proto.ConsolePins:
        """Samples the two console pins as the fixture sees them.

        Answers whether the board actually drove what it claimed to during the
        blind window — the board's own report of that is not evidence, since a
        pin that never left its alt function reads back exactly the same way
        from the board's side.
        """
        return proto.parse_console_pins(self.request(Cmd.CONSOLE_PINS))

    def console_drive(
        self, gpio14: bool | None = None, gpio15: bool | None = None
    ) -> None:
        """Drives the console pins from the fixture, or lets go of them.

        `True`/`False` drive the named board pin high or low; `None` — the
        default for both — releases it to high-impedance. Defaulting to
        released rather than to "unchanged" means `console_drive()` with no
        arguments hands both wires back, which is the call a caller most needs
        to be able to make without thinking.

        Only valid while the console is detached; the fixture answers
        `BAD_STATE` otherwise rather than putting a second driver on a net its
        own UART is still muxed onto.
        """
        oe = (0 if gpio14 is None else 1) | (0 if gpio15 is None else 2)
        levels = (1 if gpio14 else 0) | (2 if gpio15 else 0)
        self.request(Cmd.CONSOLE_DRIVE, bytes([oe, levels]))

    @contextlib.contextmanager
    def console_released(self):
        """Detaches the console for the body, and reattaches whatever happens.

        A context manager rather than a pair of calls because the failure mode
        of forgetting the second one is not a failed test — it is a bench left
        with no console, where every later case fails at the loader handshake
        for a reason that has nothing to do with what it tested. Reattaching
        on the exception path is the whole point.
        """
        self.console_detach()
        try:
            yield self
        finally:
            self.console_attach()

    # -- marker-pin edge timestamping ---------------------------------------

    def marker_arm(self) -> None:
        """Starts a capture, discarding whatever the last one held."""
        self.request(Cmd.MARKER_ARM)

    def marker_status(self) -> proto.MarkerStatus:
        """How full the current capture is, and the timebase it is in."""
        return proto.parse_marker_status(self.request(Cmd.MARKER_STATUS))

    def marker_pulse(self, count: int, half_period_us: int) -> None:
        """Has the fixture drive a pulse train on its own marker pin.

        The bench testing its own capture path — PIO program, FIFO, DMA,
        readout — with no board, no wire and no case in the picture. When a
        board measurement comes out wrong, this is the call that says whose
        fault it is.

        The fixture busy-waits for the duration and refuses anything over
        25 ms, because it is starving its own USB stack meanwhile.
        """
        self.request(
            Cmd.MARKER_PULSE,
            count.to_bytes(2, "little") + half_period_us.to_bytes(2, "little"),
        )

    def marker_read(self, count: int | None = None) -> list[int]:
        """Reads the capture back as a list of ascending timestamps, in ticks.

        Chunked over as many round trips as it takes — 15 timestamps fit in a
        64-byte packet — because the control channel is one request per packet
        by design and reassembly at this end is cheaper than continuation
        state at the other.

        Ascending: the state machine's counter descends, since `jmp x--` is
        PIO's only single-cycle decrement, and the fixture inverts on the way
        out so nothing downstream has to remember that.
        """
        status = self.marker_status()
        want = status.captured if count is None else min(count, status.captured)

        per_packet = proto.MAX_BODY // 4
        stamps: list[int] = []
        while len(stamps) < want:
            chunk = min(per_packet, want - len(stamps))
            body = self.request(
                Cmd.MARKER_READ,
                len(stamps).to_bytes(2, "little") + bytes([chunk]),
            )
            if len(body) != chunk * 4:
                raise ProtocolError(
                    f"asked for {chunk} timestamps and got {len(body)} bytes"
                )
            stamps.extend(
                int.from_bytes(body[i : i + 4], "little")
                for i in range(0, len(body), 4)
            )
        return stamps

    def missing(self, *caps: Cap) -> list[str]:
        """Names the requested capabilities this fixture lacks.

        Returned rather than raised so a test can put them in its skip
        reason — "why did this not run" has to be answerable from the report
        alone.
        """
        have = self.hello().capabilities
        return [cap.name for cap in caps if cap not in have]

    def close(self) -> None:
        """Releases the USB device."""
        usb.util.dispose_resources(self._dev)

    def __enter__(self) -> Fixture:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()
