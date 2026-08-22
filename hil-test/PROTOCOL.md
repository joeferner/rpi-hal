# Fixture control protocol

The authoritative wire format between the host runner and the bench fixture.
`firmware/src/proto.rs` and `host/hilbench/proto.py` are both written against
this document, and `host/tests/test_fixture.py` pins the constants so a
renumbering cannot pass silently.

## USB layout

The fixture is one composite device at VID `0x1209`, PID `0x0001` — the
pid.codes test allocation, which is fine for a fixture that never ships.

| Interface | Class | Carries |
| --- | --- | --- |
| CDC ACM | `0x02` | the board's console, bridged from its UART |
| Vendor | `0xFF` | this protocol |

Both share one cable. Two reasons for the split rather than a second CDC:

- The console **must** look like an ordinary serial port, because
  `rpi-loader`'s CLI takes a device path. Its baud follows the host's
  `SET_LINE_CODING`, so the loader's negotiation from 115200 up to 1.5 Mbaud
  and back needs no cooperation from the fixture and no knowledge of the
  loader's own framing.
- This channel carries binary bodies and, later, capture buffers. A tty
  would add a line discipline with opinions about the bytes, and hex or
  base64 would double the size of every capture for the benefit of a human
  who is not reading it.

The control interface is found by **interface class**, never by index, so
adding an interface to the firmware later cannot silently repoint the runner
at the console.

## Framing

One request per bulk OUT packet, one response per bulk IN packet. Full-speed
bulk endpoints are 64 bytes, so every exchange is a single transfer and there
is no continuation state to get wrong.

```
request:   [u8 opcode][u8 body_len][body_len bytes]
response:  [u8 status][u8 body_len][body_len bytes]
```

`body_len` is at most **62** (`MAX_BODY`), one packet less the header. A
request declaring more is answered `BAD_ARGS` rather than being reassembled.

Multi-byte integers are **little-endian**, matching both ends natively.

## Status codes

| Value | Name | Meaning |
| --- | --- | --- |
| `0x00` | `OK` | succeeded; body holds whatever the command returns |
| `0x01` | `BAD_COMMAND` | opcode not recognised by this firmware |
| `0x02` | `BAD_ARGS` | opcode understood, body malformed or wrong length |
| `0x03` | `UNSUPPORTED` | understood, but this fixture cannot do it |
| `0x04` | `BAD_STATE` | understood and well-formed, but not valid right now |

`UNSUPPORTED` is deliberately distinct from `BAD_COMMAND`. The first means
*skip this case*; the second means the host and firmware disagree about the
protocol, which is a defect. Collapsing them would turn a version mismatch
into a suite that quietly skips everything and reports green.

`BAD_STATE` is distinct from `BAD_ARGS` for the same kind of reason: bad
arguments mean the caller built the request wrong and retrying cannot help,
while `BAD_STATE` means the identical request is correct a moment later.
Collapsing them would have a sequencing bug present as a malformed packet,
sending whoever debugs it to read the codec.

## Commands

### `0x01` PING

Liveness. Empty body both ways.

### `0x02` HELLO

Identifies the fixture. Empty request body; 9-byte response:

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 1 | protocol version |
| 1 | 1 | board id |
| 2 | 4 | capability bitmap, little-endian |
| 6 | 3 | firmware version, major/minor/patch |

The runner's first call. It refuses a fixture whose protocol version it does
not know rather than guessing at a mismatched layout, and every skip
decision keys off the capability bitmap.

#### Board ids

| Value | Board |
| --- | --- |
| 1 | RP2040 on a Raspberry Pi Pico |
| 2 | RP2350B on an Olimex PICO2-XL |

#### Capability bits

Bit positions are wire format: allocated once, never renumbered, because a
renumbering breaks every fixture in the field simultaneously and presents as
unrelated cases mysteriously skipping.

| Bit | Name | Meaning |
| --- | --- | --- |
| 0 | `CONSOLE_BRIDGE` | bridges the board's console UART to the host |
| 1 | `GPIO_SHADOW` | drives and reads the board's GPIO header 1:1 |
| 2 | `MARKER_TIMESTAMP` | timestamps marker-pin edges against the fixture clock |
| 3 | `SPI_SLAVE` | plays the SPI slave role in any of the four modes |
| 4 | `I2C_SLAVE` | programmable I2C slave, including NAK and clock stretch |
| 5 | `LOGIC_CAPTURE` | samples several lines at once into a buffer |
| 6 | `I2S_CAPTURE` | receives I2S and returns frames bit-exactly |
| 7 | `AUDIO_ADC` | samples the analog audio output |
| 8 | `POWER_SWITCH` | switches the board's 5V, i.e. can power-cycle it |
| 9 | `RAIL_SENSE` | reads the board's 3V3 rail |
| 10 | `CURRENT_SENSE` | measures current into the board's rail |
| 11 | `USB_VBUS_SWITCH` | switches VBUS to individual USB devices |
| 12 | `RUN_RESET` | pulls the board's `RUN` pad low for a warm reset |

A fixture claims a bit only when it can actually deliver it. An
optimistically-set bit is worse than a missing one: the runner stops skipping
and starts reporting false failures against hardware that isn't there.

### `0x10` CONSOLE_DETACH / `0x11` CONSOLE_ATTACH

Releases and resumes the console bridge, so a case can drive GPIO14/15. Empty
body both ways.

Those are the only header pins with UART alt functions on BCM283x — UART0 is
also on GPIO32/33 and 36/37, the mini-UART on 32/33 and 40/41, and none of
GPIO32–41 reach the 40-pin header — so the console cannot move aside. It has
to be time-multiplexed, and both sides have to agree explicitly rather than
inferring it.

`CONSOLE_DETACH` moves the fixture's console pins from the UART to SIO as
high-impedance inputs, clearing the output enable before the mux moves so the
pad is never briefly driven from a register nothing has set.
`CONSOLE_ATTACH` puts them back. The UART peripheral itself is untouched
throughout, so reattaching restores the link at whatever baud the host last
set through CDC line coding rather than at the default — which matters
because a loader session negotiates up to 1.5 Mbaud and would be garbled by a
silent reset to 115200.

Both are **idempotent**. The runner's recovery path reattaches a console it is
not certain it detached, and a second detach must not be an error there.

While detached, host bytes arriving on the CDC interface are **discarded, not
queued**. Queuing them would put them in the UART's transmit FIFO, where they
would sit with the pad muxed away and then transmit the instant the bridge
came back — injecting stale bytes into the board's receiver at exactly the
moment it is re-establishing its own console.

#### The window is blind, so the schedule goes on the wire

Neither end can coordinate during the handoff: the only channel between them
is the pins being borrowed. So the board publishes its timings *before* the
window, as part of its announcement, and the runner commits to them:

```text
#HIL console=release grace_ms=400 hold_ms=900 settle_ms=500
```

1. The case prints that line and **flushes** it — `writeln!` only reaches the
   PL011 FIFO, and a truncated schedule is worse than none, since half a line
   parses as no announcement at all.
2. `grace_ms` later the case reassigns the pins. The runner has that long to
   see the line and send `CONSOLE_DETACH`; the delay covers USB scheduling and
   the loader subprocess's pipe, because the announcement is always seen later
   than it was sent.
3. For `hold_ms` the case owns the pins, and accumulates results in RAM.
4. The case restores the console and waits `settle_ms` before printing. The
   runner sends `CONSOLE_ATTACH` partway through that, so the reattach lands
   after the board's UART is back and before its first line.

Steps 2–4 are a window with no console. A case that hangs there never restores
it, which is why the per-case timeout and power-cycle recovery loop is
mandatory infrastructure rather than a refinement.

The margins are deliberately loose. A late runner then costs nothing, while
tightening them trades a real failure mode for a saving nobody measures.

### `0x12` CONSOLE_STATUS

Reports the bridge state. Empty request; 1-byte response, `1` if attached and
`0` if detached.

### `0x13` CONSOLE_PINS

Samples the two console pins. Empty request; 1-byte response, bit 0 the
board's GPIO14 (its TXD0) and bit 1 its GPIO15 (its RXD0).

Named for the **board's** pins, not the fixture's: which fixture GPIO they
land on is a property of this wiring, while GPIO14/15 is what a case asserts
about, so the translation happens once here rather than at every call site.

This is what makes the handoff assertable from both ends. A board reporting
that it drove a pin is not evidence — a pin still in its alt function reads
back from the board's side exactly as one under GPIO control would, so a case
can pass in full with the mux never having moved.

Read-only, and valid whether or not the pins are released: `GPIO_IN` always
reflects the pad, whichever peripheral is muxed onto it. Deliberately not
gated on the detach state, because a level read that silently returned nothing
while attached would make a misordered handoff look like a dead pin rather
than a sequencing bug.

Not `GPIO_SHADOW`, and not gated on it. That capability is about driving and
reading the whole header 1:1; this reads two pins the fixture already owns,
and a fixture that bridges the console can always do it.

### `0x14` CONSOLE_DRIVE

Drives the two console pins from the fixture, so a case can read what was put
on them. Two-byte request body, empty response:

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 1 | output enable, bit 0 GPIO14, bit 1 GPIO15 |
| 1 | 1 | levels, same bit assignment |

A pin whose enable bit is clear goes high-impedance, which is the resting
state and how a caller hands the wire back. `00 00` therefore releases both,
and is what the runner sends on its way out of a window.

Answered `BAD_STATE` unless the console is detached. **This interlock is what
makes 1:1 shadowing safe to build on**: driving a pad the bridge's UART is
still muxed onto puts two output drivers on one net, which is a short
whenever they disagree. The fixture cannot see whether the *board* has let go
of its end, but it knows perfectly well whether it has itself, so it enforces
the half it can rather than trusting the caller with both.

The other half is the case's job, and it is not optional. A series resistor
bounds the damage but does not arbitrate — with one in line each end simply
owns its own side of it, so a fixture driving against a board that never
released the pin reads back its own level and the board reads back *its* own,
and neither is the truth. Shadowing a pin the board drives requires the board
to release it, which is what the handoff above is for.

### `0x20` MARKER_ARM

Starts a marker-pin capture, discarding whatever the last one held. Empty body
both ways.

Everything is torn down before anything is started, so a capture cannot open
with edges that arrived while the host was still setting it up — their
timestamps are against the *previous* counter and would read as one enormous
first interval rather than as an error.

### `0x21` MARKER_STATUS

Empty request; 10-byte response:

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 2 | edges captured, little-endian |
| 2 | 1 | flags: bit 0 set if the capture overflowed |
| 3 | 4 | ticks per second of the timebase, little-endian |
| 7 | 2 | capacity in edges, little-endian |
| 9 | 1 | fixture GPIO being watched |

The tick rate is **reported, never assumed**. It follows the fixture's system
clock, so a host that hardcoded it would go on quoting confident measurements
after a firmware clock change had silently rescaled every one of them.

The overflow flag matters more than it looks. A capture that dropped edges in
the middle still has entirely plausible intervals either side of the gap, so
there is nothing in the data itself to notice.

### `0x22` MARKER_READ

Reads timestamps out of the capture. Three-byte request — a little-endian
`u16` start index and a `u8` count — answering with `count` little-endian
`u32`s. At most 15 fit a packet; a larger count, or a start past the end, is
`BAD_ARGS` rather than being clamped, because zeroes decode as perfectly good
timestamps and a readout that ran off the end would produce a plausible
capture instead of a complaint.

Timestamps ascend. The state machine's own counter *descends* — `jmp x--` is
PIO's only single-cycle decrement — and the fixture inverts on the way out so
nothing downstream has to remember that.

### `0x23` MARKER_PULSE

Drives a pulse train on the fixture's own marker pin, for testing the capture
path with no board attached. Four-byte request: a little-endian `u16` count
and a `u16` half-period in microseconds. Empty response.

The fixture busy-waits for the duration, starving the USB stack it shares an
executor with, so a train longer than 25 ms is `BAD_ARGS`. Bounded in firmware
rather than trusted to the caller: the failure mode of an over-long one is a
fixture that stops answering for longer than the runner's own timeout, which
presents as dead hardware rather than as a bad argument.

## Adding a command

- Allocate the next opcode in its range — `0x0x` housekeeping, `0x1x`
  console, and a fresh range per subsystem after that. Never reuse a
  retired opcode.
- Add it here first, then to both `proto` modules, then pin any new constant
  in `test_protocol_constants_match_firmware`.
- Give it a capability bit if a fixture could plausibly lack it, and return
  `UNSUPPORTED` rather than failing when the hardware is absent.
