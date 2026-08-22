# Hardware-in-the-loop tests

CI can only prove that this crate compiles. Whether a driver *works* is
established here, on real boards, with an independent fixture watching the
wires.

This tree holds everything that makes that repeatable: firmware for the
bench fixture, test binaries for the board under test, and the host-side
runner that drives both.

## Layout

Three separate Cargo projects for three different targets, plus a Python
package. None of them share a workspace — each is detached deliberately, so
building one never drags in the others' targets or lockfiles.

| Directory | Target | What it is |
| --- | --- | --- |
| `firmware/` | `thumbv6m` / `thumbv8m` | fixture firmware for the RP2040 (Pico) and RP2350 (Olimex PICO2-XL) |
| `cases/` | `armv7a` / `aarch64` | the self-reporting test binaries that run on the Pi |
| `host/` | host Python | the protocol client, pytest fixtures and the runner |
| `hardware/` | — | the bench design: what to build and why, see [hardware/README.md](hardware/README.md) |

`cases/` depends on `rpi-hal` by path, as an ordinary consumer rather than
as more `examples/`. That is deliberate: it exercises the published API
surface, the `rpi-link.x` linker script the crate puts on the linker search
path, and the feature flags, the same way an external user would. Test
scaffolding also stays out of the human-facing examples, which exist to be
read.

`cases/` builds four ways — `bcm2837` and `bcm2711` × AArch32 and AArch64 —
and `firmware/` builds two, one per fixture chip.

## The two tiers

| Tier | What it is | What it owns |
| --- | --- | --- |
| **Orchestrator** | Linux SBC or mini PC | builds images, drives the loader, owns the capture/audio/Bluetooth/Ethernet dongles, the isolated network, pcap witnesses, report generation |
| **Fixture** | one MCU on a HAT | anything needing microsecond accuracy or real electrical presence: pin shadowing, bus slave roles, logic analysis, I2S capture, audio ADC, board power, USB VBUS switching |

The rule that decides where anything goes: **if the orchestrator can do it
in Python with a cheap dongle, it does not go into MCU firmware.** The
fixture has to be more reliable than the thing it is testing, or the time
goes into debugging the bench instead of the HAL.

One external project supplies the rest:
[`rpi-loader`](https://github.com/joeferner/rpi-loader), a resident UART
command agent on the board. Flashed once, after which every build is
`mem-write` + `exec` over serial. Its `sd-read`/`sd-write` also give the
host an independent view of the card.

## Methodology

### Self-reporting binaries

Every line the runner cares about begins `#HIL`, so console noise, boot
chatter and a half-overwritten line can never be mistaken for a result:

```text
#HIL session board=0x00a02082 arch=aarch64 cases=4
#HIL case=timer_advances status=PASS
#HIL case=timer_rate status=FAIL detail=50ms delay measured outside the window
#HIL case=pcm_i2s status=SKIP detail=no DAC on this rig
#HIL end pass=1 fail=1 skip=1
```

Four decisions in that shape, each there to stop a specific kind of false
report:

- **The board revision comes from the mailbox**, not a build-time constant,
  so the runner can confirm it is talking to the board it thinks it is. A
  rig that silently runs the Pi 3 suite against a Pi 4 reports nonsense
  with complete confidence.
- **The banner declares how many cases to expect.** A binary that hangs
  halfway is then distinguishable from one that legitimately ran fewer —
  without it, a truncated run looks like a clean one.
- **`SKIP` is distinct from `FAIL`**, so absent hardware never reads as a
  defect.
- **`FAIL` always carries a `detail`.** A bare failure forces whoever reads
  the report to reproduce it by hand.

A panic is reported as `#HIL panic detail=…` rather than being left to end
in silence. Silence is ambiguous: a hang and a panic look identical from
the host but want different investigations.

The shared helper lives in `cases/src/lib.rs`, so a case binary is a
`main` plus assertions and nothing else. `cases/src/bin/hil_smoke.rs` is
the whole `hil-smoke` target — self-checking, so it needs no fixture.

### Two channels, split by what they carry

The fixture is one USB device presenting two interfaces:

- **CDC ACM** — the Pi's console, bridged from the board's UART with the
  baud following the host's `SET_LINE_CODING`. It has to look like an
  ordinary serial port because the loader's CLI opens one.
- **Vendor bulk** — commands and capture data, spoken over libusb. Binary
  framing, no terminal layer to mangle bytes, and bulk transfers carry
  captures without hex-encoding them. The device is matched by VID/PID, so
  there is no guessing which `ttyACM` is which.

`PROTOCOL.md` is the authoritative wire format for the control interface.

### Capabilities, not assumptions

Every case declares what it needs. The runner asks each fixture what it
has — `HELLO` returns a capability list — and cross-references
`bench.toml`, the inventory of what this particular rig owns. Missing
hardware **skips with a reason**; it never fails. That is what lets one
suite serve a bare board on a desk and a full rack.

| Target | Needs | Coverage |
| --- | --- | --- |
| `hil-smoke` | a board and a USB-serial cable | boot, MMU, multicore, timers, RNG, mailbox, SD, framebuffer checksum |
| `hil-bench` | + the HAT | GPIO sweep, SPI/I2C/PWM/PCM, IRQ latency, IR, USB |
| `hil-av` | + capture and audio dongles | HDMI pixels, tearing, analog and I2S audio |
| `hil-net` | + isolated network, AP, BT dongle | Ethernet, Wi-Fi, TLS, BLE |
| `hil` | everything present | the full matrix |

A drive-by contributor with one board and a serial cable runs `hil-smoke`
and gets real signal. Nobody needs the whole rig to contribute.

### Assert from both ends

A device reporting its own success is not evidence. Wherever the
orchestrator or the fixture can observe the same event independently, it
does — a pcap catches bad checksums, a missing ARP and wrong window
behaviour that are invisible from the board's point of view; the loader's
`sd-read` checks from outside what the write path claimed to write; the
fixture watching the 3V3 rail plus a re-`HELLO` proves a watchdog reset
really happened rather than the code merely reaching the line.

### The console and the test pin are the same two wires

GPIO14/15 are the only header pins with a UART alt function on a BCM283x, so
a case that tests them has no console while it does. Both ends have to let go
and take hold again, and neither can talk to the other in between.

The way out is that the board publishes its timings before the window opens
and the runner commits to them, rather than the two sides negotiating through
a channel that no longer exists. `PROTOCOL.md` has the exact schedule;
`cases/src/bin/hil_console.rs` is the case that exercises it, and it is worth
reading before writing any other case that borrows these pins.

Two consequences worth knowing before relying on it:

- **A hang inside the window leaves the bench with no console**, not just a
  failed case. Every later test then fails at the loader handshake for an
  unrelated-looking reason, which is why the runner reattaches from a `finally`
  and `CONSOLE_ATTACH` is idempotent.
- **The board's own report of what it drove is not evidence.** A pin left in
  its alt function reads back from the board's side exactly as one under GPIO
  control would, so `hil_console` passes in full if the mux never moved. The
  fixture sampling the same pin through `CONSOLE_PINS` is the half that closes
  it.
- **Releasing is not optional just because there is a resistor.** The fixture
  refuses `CONSOLE_DRIVE` while its own bridge holds the pins, but it cannot
  see the board's end. If a case drives a pin the fixture is also driving, the
  series resistor keeps the current harmless and both ends read back their own
  side — so the test does not fail, it lies. `hil_shadow` is the worked
  example of doing it in the right order.

### One timebase

The fixture timestamps marker-pin edges, captured waveforms *and* console
bytes against the same clock, because the console arrives through it. So
"the board printed this 4.2 ms after the marker edge" is a measurement.
Host-side timestamps carry milliseconds of USB scheduling jitter and cannot
make that claim.

### Bounded and self-healing

Bare-metal cases hang rather than fail, so every case has a hard timeout.
On timeout the runner power-cycles the board, re-establishes the loader,
records the transcript and continues — that recovery loop is what makes
unattended runs possible at all, not a refinement. Retries are allowed but
**recorded**: a case that only passes on retry is reported flaky, not
silently green. Every raw transcript is kept, because triaging a
bare-metal failure without the byte stream is hopeless.

### Execution state is an outer loop

Whether the board runs AArch32 or AArch64 is fixed by firmware at reset, so
switching means rewriting `arm_64bit` in `config.txt` and rebooting. Both
loader images sit on the card as `kernel7.img` and `kernel8.img`, and the
flip is a same-length in-place write over the loader's `sd-write`, verified
by reading it back **before** power-cycling — the one operation in the rig
that can make a board unreachable if it goes wrong.

Each flip therefore costs a reboot and carries risk, so it is a
session-scoped fixture with the case order sorted to minimise flips, never
a per-case parametrisation.

## How a case runs

1. The runner reads `bench.toml`, opens each fixture's control and console
   interfaces, and `HELLO`s for capabilities.
2. Cases whose declared needs are unmet are skipped with a reason.
3. If the required execution state differs from the board's, `config.txt` is
   rewritten, read back, verified, and only then is the board power-cycled.
4. The board comes up in the loader; the runner re-`HELLO`s it.
5. `mem-write` + `exec` loads the case binary.
6. The banner is checked against the expected board revision and state.
7. If the case touches GPIO14/15 it announces its handoff schedule on the
   console; the runner reads that, sends `console detach`, and the fixture
   releases those pins.
8. The case runs. The fixture supplies stimulus and records witnesses over
   the control channel.
9. `console attach`, then the runner parses the `#HIL` lines.
10. Artifacts — transcript, waveform plots, captured frames, audio spectra —
    are collected into an HTML report, so the questions that genuinely need
    a human eye get answered by looking at pictures once, after the run,
    rather than at a monitor during it.

## Wiring the Pico fixture

The bench starts as a bare Pico on a breadboard, before any HAT exists. Only
the console bridge is wired, which is enough to prove the architecture and
to make every subsequent build load through the fixture.

Three wires, plus a USB cable from the Pico to the host.

| Pico signal | Pico pin | Pi signal | Pi header pin | Direction |
| --- | --- | --- | --- | --- |
| GP0 — UART0 TX | 1 | GPIO15 / RXD0 | 10 | fixture → Pi |
| GP1 — UART0 RX | 2 | GPIO14 / TXD0 | 8 | Pi → fixture |
| GP2 — marker | 4 | GPIO4 | 7 | Pi → fixture |
| GND | 3 | GND | 6 | — |

The marker line is what `hil_marker` and every later timing assertion use. It
is a separate wire precisely so it is not GPIO14/15: the console stays up
while a measurement runs, which is the whole return on spending a pin.

Both boards are 3.3 V, so no level shifting is involved.

Put **1 kΩ in series in each of the three signal lines** — not in the ground.
It is what the HAT will have on every shadowed line, so having it here means
the breadboard is testing the same circuit; it costs nothing measurable (the
loader still runs at 1.5 Mbaud through it); and it bounds a contention to
3.3 mA, which is the difference between a forgotten pin release being a
failed test and being a dead pad. Pass `--series-resistor` once it is in and
the suite will check it is doing its job.

Three things to get right, in decreasing order of how much time they cost
when wrong:

- **Cross the pair.** The fixture's transmit goes to the board's *receive*.
  Straight-through wiring is silent — no error surfaces anywhere, on either
  side.
- **Common ground is mandatory**, not optional tidiness. Without it the two
  boards' references float apart and the link misbehaves in a way that looks
  intermittent rather than broken.
- **Do not connect the Pico's VBUS or 3V3 to the Pi.** Each board keeps its
  own supply for now, and a fixture output driving an unpowered Pi pushes
  current through that pin's protection diodes into the Pi's 3V3 rail — the
  classic cause of a board that will not cold-boot cleanly. Power switching
  arrives with the HAT, along with the series resistors that make this safe
  to get wrong.

### Pin allocation

What the firmware claims so far, so the next thing added does not collide:

| Pico GPIO | Header pin | Use |
| --- | --- | --- |
| GP0, GP1 | 1, 2 | console bridge to the board's UART, and SIO inputs while it is detached |
| GP2 | 4 | marker-pin edge timestamping, watched by PIO0 state machine 0 |
| GP25 | none | on-board status LED — not brought out, nothing to wire |
| everything else | — | unclaimed |

DMA channel 0 is claimed too, draining the marker capture into RAM. Nothing
else here uses DMA; a second user has to say so, because the channel is
referenced by number rather than held as a peripheral token — the capture is
deliberately fire-and-forget, and `embassy-rp`'s `Transfer` aborts on drop.

GP23, GP24, GP25 and GP29 never reach the header on a Pico; they are the
SMPS mode control, VBUS sense, the LED and the VSYS divider respectively.
That is why a Pico exposes 26 of the RP2040's 30 GPIO, and part of why the
HAT fixture is an RP2350B.

### Reading the status LED

The LED is the fixture's only output that does not depend on USB, which is
what makes it worth a pin: if the USB stack fails to come up, or the host
never opens the port, there is otherwise no way to tell a hung board from an
unplugged cable from a wrong VID/PID. It starts blinking before USB or UART
are set up, so a hang during either still leaves a signal.

| Pattern | Meaning |
| --- | --- |
| Dark | the firmware is not running — suspect the flash, or see the Pico W note below |
| Fast even blink | running, but no host has spoken to the control interface. Suspect cabling, the udev rule, or the runner |
| Short pulse once a second | the runner has talked to us and the fixture is idle — the healthy resting state |
| Double pulse | console bytes are moving, i.e. the board under test is talking through the bridge |

So the first flash should give a fast even blink, and `make test` should
settle it into the once-a-second pulse.

**This assumes a plain Pico, not a Pico W.** On the W the LED is not on
GP25 at all — it hangs off the CYW43 wireless chip as `WL_GPIO0` and is
only reachable through that driver. The firmware would drive a pin
connected to nothing, leaving the LED permanently dark and therefore
indistinguishable from a board that never booted. On a W the status output
has to move to a spare header pin and an external LED.

## What to run, and when

**Before committing anything here**: `make pre-commit`. It formats both Rust
trees and the Python one, runs clippy over the firmware and the cases, `ruff`
and `ty` over the host package, builds all four case targets and the RP2040
firmware, and runs the host suite. About six seconds warm, and it needs no
hardware.

`rpi-hal`'s own `make pre-commit` calls this, so committing from the crate
root covers the bench too and there is nothing separate to remember.

`ruff` and `ty` are fetched on demand by `uvx` rather than being listed as
project dependencies — a contributor with one board and a serial cable should
not have to install a linter to run a test. Their rule selection and target
Python version live in `host/pyproject.toml`, pinned rather than left to
whatever version `uvx` fetches: a check that runs on every commit must not
start failing on untouched code the day the tool updates.

**Before touching hardware**, and on every change to either side of the
protocol: `make test-all`. It builds the firmware, runs clippy over it, and
runs the host suite. Everything needing hardware that is not attached skips
with a reason naming what was missing, so this is a real check even with no
bench on the desk — the protocol codec and the capability logic are covered
either way.

**Once per machine**, two host-side installs that are not project
dependencies. `make tools` reports what is present:

- The flashing tool cargo invokes as the target runner — `cargo install
  elf2uf2-rs` for the RP2040, `picotool` for the RP2350. The flash targets
  check for it first, because cargo's own failure is a bare "No such file
  or directory" that names neither the tool nor how to get it.
- The udev rule in `host/udev/`, so the runner can claim the fixture's
  vendor interface without root. Copy it to `/etc/udev/rules.d/`, reload,
  and **replug the fixture** — rules apply at enumeration, not
  retroactively.

  The rule grants the device to the `plugdev` group as well as tagging it
  `uaccess`, and the group is the part that matters here. `uaccess` only
  hands the device to the user of an active *local seat*, so it does
  nothing over SSH, nothing on a remote display, and nothing for an
  unattended runner — which is the rig's intended mode, not an edge case.
  Check with `groups`; add yourself with `sudo usermod -aG plugdev $USER`
  and log back in.

  The same distinction explains a password prompt when flashing: `udisksctl`
  mounts without authentication only from an active local session, and
  escalates to admin auth otherwise. Using `picotool` avoids the mount, and
  therefore the prompt, entirely.

**When bringing up a fixture for the first time**, or after changing the
firmware: hold BOOTSEL while plugging the Pico in, so it appears as a mass
storage device, then `make flash-rp2040` (`make flash-rp2350` for the
PICO2-XL). Flashing a board that is *not* in BOOTSEL fails at the deploy
step rather than doing nothing, so the failure is loud.

**With a fixture attached**: `make test` again. The cases that skipped
before should now run, and `test_hello_identifies_the_fixture` prints what
the fixture reports it can do — the fastest way to see whether the bench is
what you think it is.

**When only cross-compiling**, without flashing: `make firmware-rp2040` or
`make firmware-rp2350` for the fixture, `make cases` for all four builds of
the device-under-test binaries. Worth having in CI, since these catch a
build break with no hardware at all.

**When the bridge cannot reach a board**: `make test-loopback`, with the
fixture's TX jumpered to its RX and nothing else attached. It proves or
clears the bridge firmware — including baud following at 1.5 Mbaud — with
the board and the wiring out of the picture, which is the fastest way to
halve the search space.

**To run a case on a real board**: `make test-board64` or
`make test-board32`, which build the images, put the board in the matching
execution state, load the case through the fixture and parse the verdict.
Both ask you to power-cycle when a reset is needed, so keep a hand on the
switch. Use `make test-board` instead when the board is already in the right
state and you want no prompts.

Getting the execution state wrong is not an error you can see: a mismatched
image transfers, jumps, and decodes as garbage, leaving the board silent
with nothing to report. That is why the runner reads `config.txt` before
loading anything — the only check that works before the case runs — and
rewrites it, verifies the read-back, and power-cycles when it disagrees.

**To exercise the console handoff on its own**: `make test-handoff`. Split out
from `test-board` because it is the one case that can leave the bench
unusable — a board that hangs inside the blind window never restores its
console, and every later test then fails at the loader handshake for a reason
that has nothing to do with it. Recovery is a power cycle.

**When you have a hand on the power switch**: `make test-manual`. It
exercises the recovery primitive the unattended loop will depend on —
that a power-cycled board actually comes back, that the fixture's own USB
link survives the board losing power, and that a second cycle works as well
as the first. It waits on a person, so never run it unattended.

## Status

Nothing here is finished. Current milestone:

- **M1 — console passthrough. Done.** An RP2040 Pico enumerates, answers
  `HELLO` with `CONSOLE_BRIDGE`, exposes a `/dev/ttyACM*`, and `rpi-loader`
  drives a board through it. The loopback self-test passes at 115200,
  921600 and 1.5 Mbaud, so baud following is covered rather than assumed.
- **M2 — closing the loop. Done.** `make test-board64` builds the image,
  confirms the board's execution state from `config.txt`, checks it is
  sitting in the loader, loads the case through the fixture, runs it and
  parses the verdict. Verified on a Pi 3B in AArch64: four smoke cases
  passing, identity `0x00a22082`, about seven seconds for the lot.
- **M2.6 — GPIO shadowing. Done.** `CONSOLE_DRIVE` lets the fixture drive the
  two wired lines while a case has released them, and `hil_shadow` has the
  board read back what was driven: `HLH` on GPIO14 and `LHH` on GPIO15,
  complementary for two of the three phases so the case cannot pass by
  reading one pin twice. Repeated with 1 kΩ in series in each line — the
  value the HAT now commits to — with no change to either the shadow or the
  console, the loader's 1.5 Mbaud transfers included.

  The finding worth carrying forward is a negative one: **a series resistor
  does not let the fixture override a pin the board is driving.** It converts
  a short into a divider, and each end then reads its own side. Shadowing
  still needs the board to release the pin first. See
  [hardware/README.md](hardware/README.md) for the measurement and the value.
- **M2.5 — the console handoff. Done.** `CONSOLE_DETACH`/`ATTACH` release and
  restore GPIO14/15 rather than answering `UNSUPPORTED`, `CONSOLE_PINS` lets
  the fixture witness what the board drove on them, and `hil_console` runs the
  whole cycle. On a Pi 3B in AArch64 the board announced a 1.8s blind window,
  drove high/low/high on GPIO14, and came back with a transcript containing no
  stray bytes at all; the fixture's samples put the first edge at 699ms against
  a predicted 700ms and measured the low phase at 292ms against 300ms. This
  was the last structural unknown blocking the HAT design — the 1:1 header
  shadow needs the console to be able to step aside, and it can.
- **M3 — marker pins and edge timestamping. Done.** A PIO state machine on
  GP2 timestamps both edges of a marker line into a DMA-filled buffer;
  `hil_marker` emits patterns the board already knows the timing of, and the
  runner arms the capture off the case's announcement. Measured on a Pi 3B:

  | | |
  | --- | --- |
  | Resolution | **16 ns** (two system clocks, 62.5 MHz timebase) |
  | Depth | **4096 edges**, 1277 used with no overflow at 20 kHz |
  | Agreement with the Pi's System Timer | **44 ppm** over 99 periods |
  | Narrowest deliberate pulse | 1 µs, measured at a 960 ns median |
  | Where it stops | pulses under ~16 ns are lost or collapse to zero width |

  44 ppm is two crystals doing what two crystals do, and it is the floor under
  every timing tolerance the suite will ever quote.

  The one constraint a case author has to know: **a marker has to be held wide
  enough to see.** Writes to `GPSET`/`GPCLR` are posted, so a bare
  set-then-clear pair produces roughly an 8 ns pulse and the fixture resolved
  only 9% of 400 such edges. Everything on the wish list — PWM, UART baud, SPI
  clock, IRQ latency, DMA completion, page flips — is hundreds of nanoseconds
  or more, so this is a footnote rather than a limit, but it is a footnote
  that would otherwise be discovered as a flaky test.
- **M4 — `bench.toml`** and the board/arch matrix as parametrisation.

Board reset is **manual for now**. There is no load switch until the HAT
exists, so `make test-manual` prompts a human to power-cycle the board with
an inline USB switch. It is one interface over three backings — the
fixture's load switch when it exists, a person meanwhile, or a skip — so
cases never learn which one they got, and swapping the human for the switch
changes nothing that calls it. The prompt refuses to appear when stdin is
not a terminal, because an unattended run that reaches an `input()` hangs
until its job times out with no indication of why.

One gap left: the fixture's activity LED cannot distinguish transmit from
receive, so it can report bytes moving without saying whether the board ever
answered. Byte counters on the control interface would fix that, and would
have shortened at least one debugging session already.

The fixture starts on an RP2040 Pico with a reduced pin map, since 26 GPIO
cannot shadow the whole header and there is no power switching yet. Those
capabilities are simply absent from `HELLO`, and the cases needing them
skip.
