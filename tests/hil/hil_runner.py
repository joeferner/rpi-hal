#!/usr/bin/env python3
"""Hardware-in-the-loop test runner for rpi-hal's SPI0 and GPIO
drivers, using a `bench-link` agent on each side (see
../../../bench-link/PROTOCOL.md for the command grammar spoken here)
-- the Pi running rpi-hal, an STM32 Nucleo-F103RB as an independent
fixture. For SPI, the Pi is the real master and the STM32 a slave
fixture; unlike examples/spi_loopback.rs's MOSI-MISO jumper self-test,
this catches a real clock polarity/phase mismatch, since the two ends
are genuinely different hardware rather than the same peripheral
looped back on itself. For GPIO, either side can drive or read, so
both directions are exercised.

Wiring (once -- no rewiring needed between test runs, only reflashing
if the agent firmware itself changes):
  Pi GPIO11 (SCLK) <-> STM32 PB13 (SCK)
  Pi GPIO9  (MISO) <-> STM32 PB14 (MISO)
  Pi GPIO10 (MOSI) <-> STM32 PB15 (MOSI)
  Pi GPIO17        <-> STM32 PA0     (GPIO loopback, Pi drives)
  Pi GPIO27        <-> STM32 PA1     (GPIO loopback, STM32 drives)
No NSS/CS wire -- see PROTOCOL.md's note on software slave management.
The two GPIO wires aren't a `bench-link` requirement -- `GPIO SET`/`GET`
takes any pin (see PROTOCOL.md) -- they're just this harness's own
choice of spare pins to test with.

Usage:
  python3 hil_runner.py <pi-serial-port> <stm32-serial-port>
e.g.
  python3 hil_runner.py /dev/serial/by-id/usb-FTDI_TTL232R-3V3_* /dev/ttyACM0
"""
from __future__ import annotations

import sys
from collections.abc import Callable

import serial

BAUD_RATE = 115200
RESPONSE_TIMEOUT_S = 2.0

Scenario = Callable[["AgentLink", "AgentLink"], None]


class AgentLink:
    """One `bench-link` agent's serial connection: send a command line,
    read back exactly one response line."""

    def __init__(self, port: str) -> None:
        self.serial = serial.Serial(port, BAUD_RATE, timeout=RESPONSE_TIMEOUT_S)

    def command(self, line: str) -> str:
        self.serial.reset_input_buffer()
        self.serial.write(line.encode("ascii") + b"\n")
        response = self.serial.readline().decode("ascii", errors="replace").strip()
        if not response:
            raise TimeoutError(f"no response to {line!r} within {RESPONSE_TIMEOUT_S}s")
        return response

    def close(self) -> None:
        self.serial.close()


class ScenarioFailure(AssertionError):
    """Raised by a scenario function to report a failed assertion."""


def expect_ok(response: str, context: str) -> str:
    if not response.startswith("OK"):
        raise ScenarioFailure(f"{context}: expected OK, got {response!r}")
    return response


def poll_until_done(stm32: AgentLink, max_attempts: int = 50) -> str:
    """Repeatedly issues `SPI POLL` until the slave reports `DONE`,
    returning the hex payload. Bounded, not unbounded -- a real bug
    (or a genuinely idle bus) should fail loudly, not hang the runner
    forever."""
    for _ in range(max_attempts):
        response = expect_ok(stm32.command("SPI POLL"), "SPI POLL")
        if response.startswith("OK DONE"):
            return response.removeprefix("OK DONE").strip()
    raise ScenarioFailure(f"SPI POLL never reported DONE after {max_attempts} attempts")


def scenario_ping(pi: AgentLink, stm32: AgentLink) -> None:
    """Both agents respond to a bare liveness check."""
    for name, link in (("pi", pi), ("stm32", stm32)):
        response = link.command("PING")
        if response != "OK PONG":
            raise ScenarioFailure(f"{name}: expected 'OK PONG', got {response!r}")


def scenario_round_trip(pi: AgentLink, stm32: AgentLink, mode: int, pattern: str) -> None:
    """Pi sends `pattern`; the STM32 slave, armed beforehand, must
    capture exactly those bytes -- the real point of this harness over
    the MOSI-MISO loopback test, since a CPOL/CPHA mismatch between two
    genuinely independent devices would corrupt this and a loopback
    structurally cannot."""
    expect_ok(pi.command(f"SPI INIT MODE{mode} CDIV=250 CS=0"), "pi SPI INIT")
    expect_ok(stm32.command(f"SPI INIT MODE{mode}"), "stm32 SPI INIT")
    expect_ok(stm32.command(f"SPI ARM TX=NONE RX={len(pattern) // 2}"), "stm32 SPI ARM")
    expect_ok(pi.command(f"SPI SEND {pattern}"), "pi SPI SEND")

    received = poll_until_done(stm32)
    if received.upper() != pattern.upper():
        raise ScenarioFailure(
            f"MODE{mode}: sent {pattern}, stm32 captured {received} instead"
        )


def scenario_reverse_round_trip(pi: AgentLink, stm32: AgentLink, mode: int, pattern: str) -> None:
    """The reverse direction: the STM32 slave preloads `pattern` into
    its shift-out buffer, the Pi master reads it back via SPI RECV."""
    expect_ok(pi.command(f"SPI INIT MODE{mode} CDIV=250 CS=0"), "pi SPI INIT")
    expect_ok(stm32.command(f"SPI INIT MODE{mode}"), "stm32 SPI INIT")
    expect_ok(stm32.command(f"SPI ARM TX={pattern} RX={len(pattern) // 2}"), "stm32 SPI ARM")

    response = expect_ok(pi.command(f"SPI RECV {len(pattern) // 2}"), "pi SPI RECV")
    received = response.removeprefix("OK").strip()
    if received.upper() != pattern.upper():
        raise ScenarioFailure(
            f"MODE{mode} reverse: stm32 preloaded {pattern}, pi read {received} instead"
        )


def scenario_gpio_pi_drives(pi: AgentLink, stm32: AgentLink) -> None:
    """The Pi drives GPIO17, wired to the STM32's PA0; the STM32 reads
    it back at both levels."""
    for level in ("HIGH", "LOW"):
        expect_ok(pi.command(f"GPIO SET 17 {level}"), "pi GPIO SET")
        response = expect_ok(stm32.command("GPIO GET PA0"), "stm32 GPIO GET")
        if response != f"OK {level}":
            raise ScenarioFailure(f"pi set GPIO17 {level}, stm32 PA0 read {response!r}")


def scenario_gpio_stm32_drives(pi: AgentLink, stm32: AgentLink) -> None:
    """The reverse direction: the STM32 drives PA1, wired to the Pi's
    GPIO27; the Pi reads it back at both levels."""
    for level in ("HIGH", "LOW"):
        expect_ok(stm32.command(f"GPIO SET PA1 {level}"), "stm32 GPIO SET")
        response = expect_ok(pi.command("GPIO GET 27"), "pi GPIO GET")
        if response != f"OK {level}":
            raise ScenarioFailure(f"stm32 set PA1 {level}, pi GPIO27 read {response!r}")


TEST_PATTERN = "AABBCC5A017E3C"

SCENARIOS: list[tuple[str, Scenario]] = [
    ("ping", lambda pi, stm32: scenario_ping(pi, stm32)),
    ("gpio_pi_drives", lambda pi, stm32: scenario_gpio_pi_drives(pi, stm32)),
    ("gpio_stm32_drives", lambda pi, stm32: scenario_gpio_stm32_drives(pi, stm32)),
]
for mode in range(4):
    SCENARIOS.append((
        f"round_trip_mode{mode}",
        lambda pi, stm32, mode=mode: scenario_round_trip(pi, stm32, mode, TEST_PATTERN),
    ))
for mode in range(4):
    SCENARIOS.append((
        f"reverse_round_trip_mode{mode}",
        lambda pi, stm32, mode=mode: scenario_reverse_round_trip(pi, stm32, mode, TEST_PATTERN),
    ))


def main() -> int:
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <pi-serial-port> <stm32-serial-port>", file=sys.stderr)
        return 1

    pi = AgentLink(sys.argv[1])
    stm32 = AgentLink(sys.argv[2])

    passed = 0
    failed = 0
    try:
        for name, scenario in SCENARIOS:
            try:
                scenario(pi, stm32)
                print(f"PASS  {name}")
                passed += 1
            except (ScenarioFailure, TimeoutError) as error:
                print(f"FAIL  {name}: {error}")
                failed += 1
    finally:
        pi.close()
        stm32.close()

    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
