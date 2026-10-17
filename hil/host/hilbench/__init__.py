"""Host-side client for the rpi-hal hardware-in-the-loop bench."""

from . import console
from .console import LOADER_READY, Case, Report, Timeout, wait_for, wait_for_loader
from .fixture import Fixture, FixtureNotFound, FixturePermissionError
from .proto import (
    Board,
    Cap,
    Cmd,
    ConsolePins,
    Hello,
    MarkerStatus,
    ProtocolError,
    Status,
)

__all__ = [
    "LOADER_READY",
    "Board",
    "Cap",
    "Case",
    "Cmd",
    "ConsolePins",
    "Fixture",
    "FixtureNotFound",
    "FixturePermissionError",
    "Hello",
    "MarkerStatus",
    "ProtocolError",
    "Report",
    "Status",
    "Timeout",
    "console",
    "wait_for",
    "wait_for_loader",
]
