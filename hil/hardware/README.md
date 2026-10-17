# HIL bench hardware

The physical rig the hardware-in-the-loop tests run on: what it is made
of, why each part was chosen, and what it deliberately cannot do.

This document covers hardware only. The runner, the wire protocol between
runner and fixture, and the per-test assertions live alongside the test
code.

## Two tiers

| Tier | What it is | What it owns |
| --- | --- | --- |
| **Orchestrator** | Linux SBC or mini PC | builds images, drives `rpi-loader`, owns the USB capture/audio/Bluetooth/Ethernet dongles, the isolated network, pcap witnesses, report generation |
| **Fixture** | one MCU on a HAT | anything needing microsecond accuracy or real electrical presence: pin shadowing, SPI/I2C slave roles, logic analysis, I2S capture, audio ADC, Pi power control, USB VBUS switching |

The rule that decides where anything goes: **if the orchestrator can do
it in Python with a cheap dongle, it does not go into MCU firmware.** The
fixture has to be more reliable than the thing it is testing, or the time
goes into debugging the bench instead of the HAL.

The software either side of this split:

- [`rpi-loader`](https://github.com/joeferner/rpi-loader) — a resident
  UART command agent on the board. Flash once per board, then every
  subsequent build is `mem-write` + `exec` over serial. `sd-read`/
  `sd-write` also give the host an independent view of the card, so the
  Pi's own claims about what it wrote can be checked from outside.
- The fixture firmware and the host runner, both in this tree. The
  firmware presents one USB device with a CDC interface carrying the
  board's console and a vendor-class interface carrying commands and
  capture data.

## Bill of materials

| Part | Approx. | Tier | Purpose |
| --- | --- | --- | --- |
| Olimex PICO2-XXL (RP2350B) | €9 | fixture | the fixture MCU |
| Harness board rev A + passives | $25–40 | fixture | wiring, power switching, real devices |
| Header adapter PCB + connectors | ~$8 each | per board | keyed ribbon header and a power connector; stays on the Pi |
| 40-way IDC ribbon, ~15 cm | ~$3 | fixture | the signal path, and the consumable |
| TPS22958DGN load switch | ~$2 | fixture | Pi power control — 14 mΩ, 6 A, HVSSOP-8 |
| INA226 + 10 mΩ shunt | ~$2 | fixture | rail current sense, and the overcurrent alert |
| USB-serial adapter | ~$5 | orchestrator | smoke tier, and recovery — the bench-tier console is tunnelled through the fixture |
| USB VBUS switch board | ~$5 | fixture | USB attach/detach stimulus |
| USB flash drive, keyboard, small hub | ~$15 | — | the real USB devices under test |
| UVC HDMI capture stick | ~$15 | orchestrator | video pixel assertions |
| USB audio dongle | ~$10 | orchestrator | analog audio capture |
| USB Ethernet NIC | ~$10 | orchestrator | isolated wired network |
| Wi-Fi AP or dongle | ~$20 | orchestrator | isolated wireless network |
| Bluetooth dongle | ~$10 | orchestrator | scripted BLE peer |

Excluding the Pis, which the test matrix requires anyway. Around $140 for
the whole bench with every witness present, plus ~$8 of adapter per board;
a useful subset is far
cheaper, since tests skip on absent capabilities rather than failing. That
is a total, not a per-board figure: there is **one** harness, and the board
under test is swapped into it. See [One harness, boards swapped through
it](#one-harness-boards-swapped-through-it).

## Fixture MCU: Olimex PICO2-XXL

An [Olimex PICO2-XXL](https://www.olimex.com/Products/RaspberryPi/PICO/PICO2-XXL/open-source-hardware)
(RP2350B, 48 GPIO, 16 MB flash, 8 MB PSRAM, microSD, €9). Development
starts on RP2040 boards, since those are already on hand and the firmware
keeps its board differences in one pin-map module, so the move is a
feature flag rather than a port.

The board is the reason there is no fixture PCB to design. It arrives as
a finished, breadboardable module with all 48 GPIO, its own regulator,
USB-C and a bootloader, so the bench can be wired up and running before
any HAT exists — and when the HAT does exist, it is socketed rather than
reflowed onto it.

### Why not a Pico or Pico 2

The package, not the chip, is the constraint. RP2350 ships in two:
RP2350**A** (QFN-60, 30 GPIO, 4 ADC) and RP2350**B** (QFN-80, 48 GPIO,
8 ADC). A **Pico 2 is the A part**, and like a Pico it breaks out only 26
of its 30 GPIO — GP23/24/25/29 go to SMPS mode, VBUS sense, the LED and
the VSYS divider. Raspberry Pi has never shipped a B-package Pico, so 48
GPIO means a third-party board or a bare QFN-80 on the HAT itself.

| | Pico (RP2040) | Pico 2 (RP2350A) | PICO2-XXL (RP2350B) |
| --- | --- | --- | --- |
| SRAM (capture depth) | 264 KB | 520 KB | 520 KB + 8 MB PSRAM |
| Flash | 2 MB | 4 MB | 16 MB |
| Soft peripherals | 8 PIO SMs | 12 PIO SMs | 12 PIO SMs |
| GPIO on the die | 30 | 30 | **48** |
| GPIO exposed | 26 | 26 | **48** |
| ADC channels free | 3 | 3 | 8 |
| Control link | native USB CDC | same | same, USB-C |
| Local storage | none | none | microSD |

PIO is why the family is right at all: SPI-slave in all four modes, I2S
receive, IR decode, multi-channel logic analysis, odd-baud UART and edge
timestamping all become soft peripherals instead of a
rewire-per-test-group exercise. Capture depth is why a smaller MCU is
not — half the value of this rig is measuring the Pi's timing precisely,
and 520 KB is ~130k 32-bit samples, i.e. tens of milliseconds at 10 MSPS,
which covers every burst measurement worth making.

### Why this board out of the RP2350B boards

- **All 48 GPIO** on two 2×20 0.1" header positions, nothing lost to
  board housekeeping. This is the entire reason to leave the Pico form
  factor, and what makes the 1:1 header shadow below possible.
- **8 ADC channels** rather than 4. Analog audio L and R and the 3V3-rail
  sense each want their own, and a USB VBUS rail sense per port is the
  obvious next claim on them; that set does not fit in the 3 a Pico leaves
  free. Rail *current* is not among them — that goes to the INA226 over
  I2C rather than a shunt into an ADC channel, for the resolution reasons
  in [Power control](#power-control-the-harness-supplies-the-pis-5v).
- **Open-source hardware.** KiCad sources, schematic and Gerbers are
  published, so the footprint drops into the HAT layout, and if a later
  revision absorbs the fixture instead of socketing it, the reference
  design for the QFN-80 support circuitry is already in hand. Every pin
  fact in this document is read off that schematic rather than off a
  vendor pinout picture.
- **It powers itself.** USB VBUS reaches VSYS through an SS34 Schottky,
  and VSYS feeds a TPS62A02A buck (2 A, 3 A peak) that makes the 3V3 rail.
  The fixture's USB-C is attached to the orchestrator anyway — it carries
  the console and the control interface — so that same cable powers the
  module, and **the harness does not feed VSYS at all**. One less
  connection, and the module sources 3V3 onto the header pins rather than
  consuming it, which is where the INA226's supply comes from.

  The harness *could* feed VSYS from the always-on rail instead, and the
  two even coexist safely: a direct 5 V feed sits above what USB delivers
  through the diode's ~0.4 V drop, so the harness would win and the diode
  would simply reverse-bias. It is not worth the wire. The fixture draws
  on the order of 100 mA, well inside a host port, and a fixture is no
  use without its orchestrator regardless — so tying its life to the USB
  cable costs nothing real.

  Two consequences worth carrying. The always-on 5 V rail still has to
  exist, because the load switch's input needs it whether or not the
  fixture is powered from it. And the `ON` pull-down becomes
  load-bearing rather than tidy: the inlet can be live while the host is
  off, with no fixture powered up to hold `ON` low.

  Grounding `3V3_EN` (EXT2 pin 12, otherwise pulled to VSYS by 1 MΩ)
  shuts the regulator down, which is worth knowing before wiring
  anything to that pin. The buck's input range is 2.7–6 V, narrower than
  the 1.8–5.5 V a Pico's VSYS accepts — irrelevant on USB, but it rules
  out running the fixture from a discharging cell.
- **16 MB flash, 8 MB PSRAM and a microSD slot.** The €5 PICO2-XL is the
  same PCB, same silkscreen and same pin map with 2 MB of flash and none
  of the three, and 2 MB is ample for the firmware — so this is bought
  for capture depth, not for code. 520 KB of SRAM is tens of
  milliseconds at 10 MSPS; PSRAM is where a capture goes when a
  measurement wants seconds of it, and the card is where a golden
  waveform lives without a host in the loop.

  The two variants are indistinguishable to firmware — nothing on the
  board reports which one it is — which is why `HELLO` reports a single
  board id covering both.

### Consequences for the layout

It is **not castellated** — 2.54 mm through-hole pads, headers
unpopulated — so rev A sockets it rather than reflowing it. That is the
better choice for a first spin regardless: the fixture stays replaceable,
and the same board moves between the breadboard and the HAT.

At 51 × 29 mm it occupies roughly a third of a 65 × 56.5 mm HAT, with
header rows down both long edges. Its placement, and the stack height
above the Pi's own header, are the dominant mechanical constraints.

**The pads are four columns, not two.** EXT1 and EXT2 are 2×20
positions, 900 mil apart centre to centre, so the columns sit at 0, 0.1,
0.9 and 1.0 inch across a 1.14 inch board. Two consequences, and the
second is the one that costs an afternoon:

- On the HAT the whole 1.0 inch span is available, so all four columns
  get sockets and all 48 GPIO are reachable.
- **On a breadboard only the two outer columns are usable.** The two
  columns of one connector are 0.1 inch apart on the same side of the
  centre channel, which is one tie-point strip — populating both rows of
  either connector shorts each GPIO to the one opposite it. The outer
  columns are EXT1's even pins (GPIO0–15, `+3.3V`, `GND`) and EXT2's odd
  pins (GPIO32–47, `+3.3V`, `GND`), 1.0 inch apart, which straddles the
  channel with four free holes per strip on each side. That is 32 GPIO
  on a breadboard, and it is why the header pins for the inner columns
  are better left unsoldered until there is a HAT to plug into.

Erratum **RP2350-E9** — GPIO inputs latching part-way high rather than
following a weakly driven pad — applies to both packages, and this fixture
is its worst case rather than merely near it: high-impedance observation of
a line reached through a series resistor is the arrangement the erratum
describes. It is not theoretical here. The marker line hit it on the first
board bring-up, and cost a day of suspecting the firmware, the timebase and
the wire before the pad itself. The external pull-down the errata sheet
prescribes is therefore a committed part of every observed line, not a
contingency — see [Rest of the HAT](#rest-of-the-hat) for the value and what
it costs.

Two details that make it expensive to diagnose rather than merely to fix.
`embassy-rp` configures a PIO pin with **both pulls disabled**, so the
"internal pull-down" the erratum names is not even enabled — the pad still
misbehaves, and anyone checking the pull configuration to rule the erratum
out will rule it out wrongly. And the failure is partial: slow edges survive
and fast ones do not, so the bench keeps reporting plausible numbers.

### Control links

The fixture presents a **composite USB device with two interfaces** over
its single USB-C cable:

- **Control** — a vendor-class bulk pair carrying the fixture's own
  command protocol and, later, capture buffers.
- **Pi console passthrough** — a CDC ACM interface, because
  `rpi-loader`'s CLI opens a serial device. Bridged from the Pi's UART on
  GPIO14/15, with the baud following the host's line coding.

Nothing else may claim this controller (see
[USB strategy](#usb-strategy)).

Tunneling the console rather than wiring a separate USB-serial adapter to
GPIO14/15 buys three things. It removes a cable and a second USB device
from the bench, and takes the console with the harness rather than leaving
an adapter to be re-attached to whichever board is mated. It lets the
fixture timestamp console bytes against the
*same* clock as the marker-pin edges and the logic capture, so "the Pi
printed this 4.2 ms after the marker edge" becomes a measurement rather
than a guess — host-side timestamps carry milliseconds of USB scheduling
jitter and cannot do this. And it removes a second driver from the
console net, which is what makes the
[GPIO14/15 handoff](#the-gpio1415-handoff) tractable at all.

**The bridge is baud-transparent, so it needs no knowledge of the loader.**
`rpi-loader` idles at 115200 and negotiates up to 1.5 Mbaud in-band for
bulk transfers, then drops back. A CDC ACM interface receives the host's
`tcsetattr` as a standard `SET_LINE_CODING` request, exactly as a CP2102
would, and the fixture reprograms its UART to match. No parsing of the
loader's wire format, no coupling between the two projects.

Throughput has margin — 1.5 Mbaud is 187 KB/s against roughly 700–1000
KB/s for full-speed bulk CDC — so the risk is dropped bytes, not
bandwidth. The RP2350's 32-byte UART FIFO fills in about 213 µs at that
rate, so the passthrough wants DMA into a ring buffer and belongs on the
**second core**, leaving core 0 for control and PIO capture. In
practice the two rarely contend: a bulk load finishes, then `exec` runs,
then the capture starts.

**Keep a dumb path anyway.** A 3-pin console header (GND/TXD/RXD) on the
HAT, *not* permanently wired, takes a hand-plugged USB-serial adapter for
first-flashing `rpi-loader` onto a new board and for sessions where the
fixture firmware is wedged or being reflashed. Unconnected during normal
runs, so it contends with nothing. The smoke tier also uses a plain
adapter directly on GPIO14/15 with no HAT at all — "a Pi and a USB-serial
cable gets you real signal" has to stay true.

## Mechanical: an adapter, a ribbon and a power lead

The harness board is **not** a HAT and does not touch the Pi. Between them
sit two things:

- **A small adapter PCB that stays on each Pi's header**, mated once and
  left there. It carries a 2×20 socket underneath, a **shrouded, keyed**
  2×20 IDC header for the signal ribbon, and a 2-pin locking power
  connector fed from header pins 2/4 with wide copper.
- **A 40-way IDC ribbon** to the harness board, plus a short power lead.

So the harness sits anywhere on the bench, and a swap is: unplug the
ribbon, unplug the power lead, lift the board out. The adapter never
comes off.

### Why not spring probes

The obvious alternative was a bed of cup-tip spring probes pressing up
onto the tails of the Pi's through-hole header, with the Pi clamped down
on top. It works, and the electrical numbers are fine, but the mechanics
are most of a project on their own. Measured on the real probes: **100 gf
at 1 mm of compression rising to 200 gf near full stroke**, with movement
starting around 30 gf. Forty of those is **4–8 kgf**, which is not a
clamp you improvise:

- A Pi weighs about 45 g, under 2% of that, so gravity is not a fastening
  method — it needs a screwed bar over the header line.
- 6 kgf distributed over the 50 mm header span bends the Pi's own 1.6 mm
  PCB by roughly 0.5–1.7 mm, against 4.3 mm of probe stroke. The probes at
  the ends of the row compress hard while the middle barely touches, which
  presents as flaky GPIO rather than as a mechanical fault.
- The harness board sees the same load downward and needs standoffs
  flanking the probe field, plus a drilled guide plate above it to locate
  40 tips to ±0.2 mm, plus ~30 mm of clearance underneath for receptacle
  bodies.

None of that buys anything the ribbon does not, and the ribbon deletes all
of it. The probes remain the right answer for a genuine bed-of-nails
fixture against bare pads; they are the wrong answer against a header that
already has a connector on it.

### What the adapter is for

Three jobs, and each replaces a problem rather than adding one.

**It makes misplugging impossible.** A bare 2×20 header is unkeyed, so a
female IDC can be seated offset by a position or reversed, putting 5V and
3V3 onto GPIOs. A shrouded header on the adapter cannot. This is the one
place the ribbon was genuinely *less* safe than probes, and it is a
$2 fix.

**It keeps the power on pins 2/4.** The whole back-powering design
survives — no per-model power cable, and the adapter is identical on every
board, unlike the Pi's own power connector which is micro-USB on a Pi 3
and USB-C on a Pi 4.

**It moves the wear off the Pi.** The adapter's socket mates once per
board. What gets repeated is the keyed IDC and a locking power connector,
both cheap and both replaceable without touching anything soldered. A worn
ribbon is a few dollars, against reworking a field of receptacles.

### The two power paths are in parallel, which is the good part

The ribbon's conductors 2 and 4 land on the same pins the power lead
feeds, so tying them to the switched rail at the harness end puts the two
in parallel — same net, no conflict. For a 15 cm ribbon in 28 AWG against
a 20 AWG pigtail:

| Path | Resistance | Share of 3 A |
| --- | --- | --- |
| Power pigtail | ~5 mΩ | ~86% |
| Ribbon pair + 4 IDC contacts | ~31 mΩ | ~14% |
| **Parallel** | **~4.3 mΩ** | |

That leaves the ribbon carrying about 0.42 A across two conductors,
**0.21 A each against a ~1 A rating**. Ribbon current was the objection
that ruled out a ribbon-only design; splitting the load removes it, and
the parallel path is lower resistance than either wire alone.

End to end from the harness to the Pi at 3 A: the adapter's socket on
pins 2/4 (~21 mV), its copper (~6 mV), the parallel feed (~13 mV) and the
ground return (~15 mV) — about **55 mV**, against ~45 mV for two spring
probes. Electrically a wash, mechanically a different world.

**A standard 45 cm IDE cable will not do**, for the avoidance of doubt:
its 5V pair alone is ~150 mV at 3 A. This is a short cable, and since IDC
is the one connector system that assembles in a vice, making one to length
is easier than sourcing one.

## Power control: the harness supplies the Pi's 5V

The HAT switches 5V into header pins 2 and 4. This is the primary reset
mechanism, in preference to the `RUN` pad.

Why: the HAT plugs onto the header and that is the *entire* physical
connection to the board — no solder step, no flying leads, and nothing
board-revision-specific, since 5V is pins 2/4 on every model whereas
`RUN` is a 2-pin pad on Pi 2/3 and `GLOBAL_EN` on Pi 4, neither of them on
the 40-pin header. That matters more here than it would on a rack of
permanently-mated HATs: this one HAT gets unmated and re-mated every time
the board under test changes, so anything model-specific would be a
soldering job per swap rather than a one-off. Back-powering the 5V rail
through the header is
permitted by the HAT specification and is what UPS and PoE HATs do. It
also resets the LAN9514, the Wi-Fi chip and attached USB devices, which a
`RUN` reset leaves in whatever state they wedged in.

Five things this has to get right.

1. **The harness needs its own upstream supply.** This inverts the usual
   HAT relationship: the fixture must stay alive while the Pi is dead, so
   nothing here can be powered from the Pi's 3V3 pins. A dedicated 5V
   inlet → always-on 5V rail → the load switch → header pins 2 **and** 4
   in parallel with several GND pins. The fixture itself is not on this
   rail; it takes its power from the USB-C cable that already carries its
   console, so the inlet exists purely to supply the board under test and
   the harness's own devices. Budget 6A: a Pi 4 under load with peripherals
   draws upwards of 1.5A, the switch is specified to pass 6A, and the rig's
   own supply must not be a variable. That budget rules out a passive USB-C
   breakout with 5.1k CC resistors, which without PD negotiation is
   entitled to 500–900mA and reaches 3A only if the source advertises it.
   See [Where the 5V comes from](#where-the-5v-comes-from).
2. **Switch the high side, never the ground.** Low-side switching floats
   the Pi's ground against the fixture's, and then every shadowed GPIO
   finds a path through the SoC's ESD diodes. Common ground always.
3. **Backfeed through the shadowed GPIOs.** Pi off, fixture on: any
   fixture output driving a Pi GPIO pushes current through that pin's
   protection diodes into the Pi's 3V3 rail, partially powering the SoC —
   a classic cause of "won't cold-boot cleanly". Three layers of defence:
   the series resistors below; firmware discipline (all fixture pins Hi-Z
   while the Pi is unpowered); and, for a later revision, a hardware
   interlock — bus switches or level translators whose output-enable is
   gated on a sense of the Pi's real 3V3 rail, making backfeed impossible
   regardless of firmware bugs.
4. **The user's own PSU still plugged in.** Then the switch is bypassed
   and power cycling silently does nothing, or two supplies back-feed each
   other. Detect rather than document: the fixture reads the Pi's 3V3 rail
   while its own switch is off, and the runner refuses to start with "Pi
   still powered — unplug its own supply".
5. **Cold boot needs the rail to actually collapse.** After cutting power,
   wait until 3V3 is below ~0.3V before re-enabling. Cutting and restoring
   too fast leaves the rails partly charged and produces a hung half-reset —
   exactly the flake an unattended run cannot absorb. Cut 5V at the header
   and restore it a second later and a Pi 3 cold-boots cleanly, so a second
   is a floor that is known to work; the sense is what the wait should
   actually be driven by, with a ceiling around three seconds that fails
   loudly rather than a fixed delay that returns success while the rail is
   still at two volts.

   The rail is **high-impedance while it collapses**, and that is the part
   with consequences. Taking a second to fall from 3.3V implies a residual
   sink of only a couple of milliamps, which is the entire current available
   to pull it down — so one fixture pin left driving a Pi GPIO through a 1k
   series resistor injects a comparable current through the SoC's clamp and
   the collapse stalls rather than slows. One shadowed pin out of 28 is
   enough. That is why the Hi-Z discipline in item 3 is load-bearing rather
   than tidy, and why the interlock there is worth having in the first
   revision.

   The switched 5V rail takes care of itself: the TPS22958 ties a 135 Ω
   discharge resistor from VOUT to ground whenever it is disabled, so the
   rail it feeds is pulled down rather than left to leak away. That is the
   *upstream* rail, though. It removes what supplies 3V3 without draining
   3V3, whose own capacitance still discharges through whatever load
   happens to remain.

   So a switched bleeder on 3V3 is still the cheap insurance: an N-FET and
   ~150–330 Ω from a 3V3 header pin to ground on a fixture GPIO, enabled
   only while the switch is off. Ten or twenty milliamps swamps any
   plausible clamp injection and collapses the rail in tens of
   milliseconds. Two things it does not buy:
   3V3 is a *proxy*, since the SoC's core rails come off their own switchers
   and draining 3V3 does not prove those fell, so any shortened wait has to
   be validated against actual cold boots rather than against the sense
   reading; and it has to be interlocked off while the board runs, or it
   corrupts every current measurement taken through the shunt.

### Where the 5V comes from

A bench brick into a barrel jack is the obvious answer and it works. The
better one, when the orchestrator is a PC, is **the orchestrator's own ATX
supply** — a 4-pin Molex peripheral tail gives red +5V and two grounds on
18 AWG, and one harness drawing at most 6A is a load any ATX unit's 5V
column carries without arithmetic. Avoid SATA power for this — three
+5V pins, but contacts rated around 1.5A each, which 6A exceeds — and
avoid +5VSB, which is always live but only good for 2–3A.

Take care with the yellow wire on that Molex tail: it is +12V, and the
inlet has a 6V absolute maximum behind it. That is the strongest argument
for the clamp being fitted rather than optional, and for the inlet
connector being something that resists being wired wrong.

The reason to prefer it is not the cable count. **It collapses the rig onto
one ground reference.** With a separate brick, HAT ground is bonded to the
fixture's through the header and the fixture's to the orchestrator's through
the USB cable, so the brick's floating output ground reaches the PC chassis
by way of a USB ground conductor, and whatever its Y-caps leak flows down
that path. Small, usually harmless, and a recurring explanation for benches
that misbehave in ways nobody can pin down — and the plug/unplug transient
along that path is the classic way to kill a USB port. One supply and one
earth removes the whole class of problem, which is the same argument as
"common ground always" one item up.

Four things it changes:

- **Fuse the inlet, and fuse the Pi branch too.** A 4A brick self-limits
  into a fault; a 5V rail rated 15–20A does not. The inlet wants a fuse
  because the always-on rail feeds the fixture and the HAT's own devices —
  a low-resistance 6A blade fuse rather than a polyfuse, whose 25 mΩ or so
  would cost more of the drop budget than the switch does. The Pi's branch
  needs its own, because the switch has no current limit of its own to
  interrupt a fault; size it just above the design current so a short opens
  it rather than cooking the part.
- **The load switch's `ON` must default off.** At PC power-on the rail
  comes up hard across every HAT at once; if `ON` floats or pulls high,
  every Pi gets hard-switched with no soft-start before any fixture
  firmware is running, which is the uncontrolled cold start item 5 is
  about. The datasheet says not to leave `ON` floating in any case, so pull
  it down: the rail then arrives with the Pi off and the fixture turns it
  on deliberately.
- **Budget the whole path, including the connectors.** Every term, at 5A
  with the naive choices: 18 AWG is roughly 21 mΩ/m, so a metre out and
  back is 42 mΩ (**210mV**); the switch is 14 mΩ (70mV); a 10 mΩ shunt is
  50mV; a low-resistance fuse about 25mV; and the adapter path — its
  socket on pins 2/4, its copper, the parallel ribbon-and-pigtail feed and
  the ground return, ~18 mΩ together — is 92mV. That totals 447mV and
  lands a nominal 5.0V rail at **4.55V**, well under the 4.75V the Pi
  wants. At **3A the same path costs 268mV and lands at 4.73V**, which
  also fails.

  So the naive wiring does not work at any current, and the fix is the
  wiring rather than the parts. 16 AWG over half a metre each way is
  13 mΩ instead of 42, and a 5 mΩ shunt halves that term:

  | | 16 AWG + 5 mΩ shunt | lands at |
  | --- | --- | --- |
  | 3A | 39 + 42 + 15 + 15 + 55 = **166mV** | **4.83V** ✓ |
  | 5A | 65 + 70 + 25 + 25 + 92 = **277mV** | 4.72V — peak only |

  Two things worth carrying forward. **The connectors belong in this
  budget** and were missing from it: any 40-way interface contributes
  something on the parallel 5V pair, and it is the second-largest term
  after the cable. And the reason 3A now clears comfortably rather than
  scraping is the parallel feed — a single path, ribbon or pigtail alone,
  would put it back near the floor. This is still a measurement to take at
  the header under load rather than a figure to trust on paper.
- **The rail dies when the PC does.** +5V is only live in S0, so shutting
  down or rebooting the orchestrator drops the board under test — and the
  fixture goes with it anyway, since its USB host is the same machine.
  Almost certainly what you want, since a fixture is
  no use without its orchestrator — but "always-on" now means "always on
  while the PC is up", which is better known in advance than diagnosed as
  a fault.

Noise is not the concern it looks like: ATX specifies +5V at ±5% with 50mV
ripple, inside what a Pi tolerates, and the inlet wants local bulk anyway.
The ±5% is worth carrying downstream, though — it means the rail can sit
legitimately at 5.25V, and the load switch on it has a 6V absolute
maximum on every pin. That is well under a volt of headroom, so the rail
wants a clamp at the inlet alongside the fuse. The fixture is not exposed
to this, since it powers itself from USB rather than from this rail.

**Part choice**: **TPS22958DGN**, a high-side load switch IC rather than a
discrete FET or a relay. One package gives a 14 mΩ pass element good for
6A continuous, an active-high `ON` input a 3V3 GPIO drives directly
(V<sub>IH</sub> 1.2V), a **capacitor-programmed rise time** on the `CT`
pin — hard-switching into the Pi's discharged bulk capacitance is an
inrush spike that can brown out the shared supply — and **quick output
discharge**, a 135 Ω resistor tied across the output whenever the switch
is off, which is exactly the rail-collapse behaviour the cold-boot recipe
needs. Relays work and the cycle count is a non-issue, but they are
bulkier, slower, need a coil driver and flyback diode, and offer no
soft-start.

The pinout is 1 `VIN`, 2 `ON`, 3 `VBIAS`, 4 `VIN`, 5 `VOUT`, 6 `GND`,
7 `CT`, 8 `VOUT`, plus the thermal pad. `VBIAS` is the device's own supply
and wants to sit at or above `VIN`, so it ties to the same always-on rail
with its own 100nF. Two ordering notes, both of which produce a board that
looks right and behaves wrong:

- **`DGN`, not `DGK`.** Same pinout, same 0.65mm-pitch leads; `DGK` has no
  thermal pad and is rated 4A rather than 6A. The pad is what earns the
  difference — R<sub>θJA</sub> is 67 °C/W with it against 185.7 °C/W
  without, so the 350mW the part dissipates at 5A is a 23 °C rise on `DGN`
  and a 65 °C rise on `DGK`. At 5A the pad is load-bearing, not
  decorative, and it has to be soldered down to copper with vias.
- **`TPS22958`, not `TPS22958N`.** The `N` variant drops the quick output
  discharge and nothing else. The rail-collapse help disappears silently.

What this part does **not** have is a current limit or a fault output, and
that is a deliberate trade for a leaded package and 14 mΩ. Two
consequences:

- **Overcurrent detection moves to the INA226**, which is already on this
  rail. Its alert output and limit registers give a threshold settable in
  firmware and an actual current reading, rather than a resistor and a
  binary flag — better for a bench, where the useful artifact is the
  number.
- **Overcurrent *limiting* is gone**, and nothing gives it back. An
  overloaded board no longer holds its rail up in constant-current mode,
  so a fault is bounded only by a fuse and by firmware reacting to the
  alert. The switched branch therefore needs its own fast fuse rather than
  leaning on the inlet fuse, and a hard short becomes a consumable instead
  of a self-recovering limit. The part is 6A continuous and 8A pulsed for
  under 300µs, which a short exceeds.

The 6V absolute maximum on every pin is the other cost. A rail that can
legitimately sit at 5.25V leaves 0.75V of headroom, so the inlet clamp
stops being advisable and becomes required — though the fixture's own buck
already put it in the bill of materials, so the switch joins that
constraint rather than adding one.

### Committing it to the board rather than to a prototype

The switch has one setting, and it is not waiting on a bench measurement.
The rise time follows from the `CT` capacitor as
SR = 0.146·C<sub>T</sub> + 14.78 µs/V with C<sub>T</sub> in pF, so **4.7nF
gives 700 µs/V — about a 3.5ms ramp at 5V**, which draws roughly 310mA of
inrush into the ~220µF the Pi's bulk capacitance and a modest output cap
come to. 2.2nF is the faster alternative at 1.7ms and 650mA. Both are
inside a window that is wide at both ends: a 500mA-limited supply boots the
board, so a slow ramp is safe, and any ramp at all beats hard-switching
into discharged bulk capacitance.

What the datasheet does change is the framing. `CT` **is not optional** —
left floating the ramp is 79µs, which into the same capacitance asks for
about 14A, so the pad is always populated and the *value* is what is
adjustable. The bulk also belongs on the input rather than the output: the
switch's body diode conducts `VOUT` to `VIN`, and TI asks for
C<sub>IN</sub> above C<sub>L</sub> — ideally 10:1 — to keep charge from
running back through it, so the electrolytic sits at `VIN` and the
switched side carries only what it needs.

So the board carries the adjustment rather than a prototype deriving it —
a swappable `CT` capacitor, test points either side of the shunt and on
3V3, `ON`, and the INA226's alert line — and every pin gets checked
against the datasheet before it becomes a footprint. A wrong value is a
tweezer; a wrong pinout is a respin.

Measuring the inrush is worth doing when there is a symptom to aim it at:
if the rail sags on restore enough to reset the fixture or drop a USB port,
which un-soft-started hand switching has done. It needs no differential
probe — the shunt goes in the *ground* return and is probed single-ended,
with every other cable off the board, since each one is a parallel return
that bypasses it.

[`power-switch.svg`](power-switch.svg) draws the whole power path — supply,
always-on rail, switch, shunt, header feed, 3V3 sense, common ground — with
the switch built from discrete parts, which is what a bench build before
the board exists would use.

**Whatever the switch, the power path is not breadboarded.** Breadboard
springs are rated around 1A and carry tens of milliohms each; add
jumper-wire resistance and a Pi 4 at 1.5–2A sees a couple of hundred
millivolts of droop and a softened inrush edge, which is measuring the
breadboard rather than the Pi. Soldered — strip board, screw terminals, or
short 20 AWG between the switch and a header socket — with the breadboard
for the gate-drive side only, and ground bonded throughout.

A discrete high-side P-FET cannot be driven from a 3V3 GPIO directly: the
gate has to be pulled within a few hundred millivolts of the 5V source to
turn *off*, so it takes 10k from gate to source and a small N-FET pulling
the gate down through a 1k. Gate drive is then only −5V, so the part must
be specified at V<sub>GS</sub> = −4.5V — which rules out most through-hole
parts, IRF9540 and FQP27P06 class devices being −10V parts that run hot
here — leaving a 20V, 4A-class SOT-23 around 30mΩ for a bench build at a
couple of amps, or a SOIC-8 or DPAK in the low tens of milliohms to reach
the 5A the finished design allows. Note that 30mΩ discrete against the
switch's 14 mΩ costs another 80mV at 5A, which the drop budget above does
not have spare — a discrete is a way to get a bench running early, not a
way to match the part.

Whichever switches it, the pass element's body diode conducts
load-to-supply — the TPS22958's datasheet says so directly, warning about
current flowing from `VOUT` back to `VIN` through it — which is what makes
"the user's own PSU is still plugged in" a runtime check rather than a
documented warning.

**Current sense** is nearly free once the whole Pi rail passes through one
point, and it buys assertions that are otherwise impossible — idle versus
`wfe`-parked versus four-cores-spinning current, i.e. *did the code
actually park the cores*; power-domain control taking effect rather than
the mailbox call merely returning success; and a regression guard on the
boot path's power behaviour.

The part is an **INA226** (`INA226AIDGSR`, VSSOP-10, no thermal pad)
across a **10 mΩ** shunt in the switched leg, not a shunt into a fixture
ADC channel. Three reasons it is worth the I2C traffic over the simpler
option:

- **The resolution is in a different class.** Shunt full scale is 81.92mV
  with a 2.5µV LSB, so 10 mΩ gives a range to 8.19A — above the switch's
  6A ceiling — at a 250µA step. Distinguishing parked cores from spinning
  ones is a tens-of-milliamps question on a rail carrying hundreds, which
  a fixture ADC reading a 50mV shunt drop does not resolve.
- **It measures a 5V rail while running from the fixture's 3V3.** The
  common-mode input range is 0–36V independent of the supply pin, which
  takes 2.7–5.5V. Powering `VS` from the *fixture's* 3V3 rather than the
  Pi's is what keeps the monitor alive while the board under test is dead
  — the whole point of it being the overcurrent watchdog.
- **The Alert pin replaces the fault output the switch does not have.** It
  is an open-drain output driven by programmable limit registers, so
  Shunt Voltage Over-Limit becomes a hardware interrupt into a fixture
  GPIO with a threshold set in firmware, latching or transparent as
  configured.

`A0` and `A1` both to ground put it at address 0x40. The shunt dissipates
250mW at 5A, so a 2512 part; and 50mV of the drop budget, which the 5 mΩ
alternative halves at half the resolution.

**`RUN` stays as a test stimulus, not as infrastructure.** A warm reset
that does *not* reset the peripherals is a distinct and useful stimulus,
particularly for the watchdog and reset-cause paths. So: an optional 2-pin
header with an open-drain FET — pull to GND only, Hi-Z otherwise, since
`RUN` has an internal pull-up and must never be driven high — and a jumper
in the box. Tests needing it declare it as a capability and skip when it
is absent. Every board gets reliable power cycling with zero wiring;
boards worth soldering also get warm-reset coverage.

## Pin budget

The Pi's 40-pin header carries 28 GPIO (GPIO0–27). At 48 fixture pins, a
**1:1 shadow of the whole header** fits with room left for housekeeping:
load-switch `ON` and the INA226's alert line, the 3V3-rail sense, INA226
I2C, the `RUN` open-drain, analog audio in, marker pins, and an
enable/fault pair per USB VBUS switch.

Six of the 48 are spoken for by the board itself, and they are all
brought out to the headers, so nothing stops a design from using them —
it just inherits what is already hanging off them:

| Fixture GPIO | Committed to | What it drags along |
| --- | --- | --- |
| GPIO8 | PSRAM chip select (`QMI_CS1n`) | the PSRAM die; unusable for anything else if PSRAM is used |
| GPIO9, 10, 11, 24 | microSD on SPI1 (hardware rev B and later) | 10 kΩ pull-ups and 33 Ω series into the card socket |
| GPIO25 | the status LED | 2.2 kΩ to an LED to ground — a load, not a conflict |

So 42 pins are unencumbered, 28 of which the header shadow claims. That
still covers the housekeeping list, and the SD and PSRAM pins come back
if a build uses neither — but a *high-impedance observation* pin is the
one job none of the six can do, because the pull-ups and the LED both
show up as the thing being measured.

This is what 26 pins cannot do. The union of every bus under test is only
17 pins — I2C1 (2/3), SPI0 (7–11), PWM (12/13/18/19), PCM (18–21), aux
SPI1 (16–21), UART0 and mini-UART (14/15) — so a Pico-class fixture covers
the bus tests perfectly well. What it cannot cover is a sweep of the GPIO
driver across the whole header, which would have to split into halves.
Shadowing everything at once also means adding a pin to a future test is a
firmware change rather than a respin.

### The pin map

Read off the Olimex schematic's own netlist and the RP2350 function mux, not
off a pinout picture. Four constraints shape it, and only the first two are
obvious:

- **The 28 shadow lines avoid all six committed pins.** GP8–11 and GP24 are
  PSRAM and SD, GP25 is the LED; none can do high-impedance observation.
- **The console pair must be a real UART0 TX/RX pair**, because the fixture
  is the Pi's console peer. GP12/GP13 are one, so board GPIO15 (the Pi's
  RXD0, our transmit) lands on GP12 and GPIO14 on GP13.
- **The I2C slave role must sit on real I2C pins** — that role is hardware,
  unlike the SPI, I2S and UART peer roles, which all come from PIO and so
  place no constraint on their pins at all. Board GPIO2/3 therefore land on
  GP14/GP15, an I2C1 SDA/SCL pair. The INA226, where the fixture is master
  instead, goes on the *other* instance.
- **Analog needs the ADC pins**, which on the RP2350B are only GP26–29 and
  GP40–47. The shadow claims nine of those twelve, which is why the audio
  and rail sense sit at the top of EXT2.

No 28-pin contiguous run of free GPIO exists — GP24/25 sit in the middle of
every candidate window — so the shadow is two blocks, GP12–23 and GP26–41.
That matters only for PIO logic capture, which reads a contiguous base plus
count: the larger block covers sixteen header lines in one pass and the rest
needs a second.

| Pi GPIO | Pi pin | Pi alt | Fixture GP | PICO2-XXL | Mux used |
| --- | --- | --- | --- | --- | --- |
| GPIO0 | 27 | ID_SD | GP16 | EXT1.3 | PIO / SIO |
| GPIO1 | 28 | ID_SC | GP17 | EXT1.5 | PIO / SIO |
| GPIO2 | 3 | SDA1 | GP14 | EXT1.36 | I2C1.Sda |
| GPIO3 | 5 | SCL1 | GP15 | EXT1.38 | I2C1.Scl |
| GPIO4 | 7 | — | GP18 | EXT1.7 | PIO / SIO |
| GPIO5 | 29 | — | GP19 | EXT1.9 | PIO / SIO |
| GPIO6 | 31 | — | GP20 | EXT1.11 | PIO / SIO |
| GPIO7 | 26 | SPI0 CE1 | GP21 | EXT1.13 | PIO / SIO |
| GPIO8 | 24 | SPI0 CE0 | GP22 | EXT1.15 | PIO / SIO |
| GPIO9 | 21 | SPI0 MISO | GP23 | EXT1.17 | PIO / SIO |
| GPIO10 | 19 | SPI0 MOSI | GP26 | EXT1.27 | PIO / SIO |
| GPIO11 | 23 | SPI0 SCLK | GP27 | EXT1.29 | PIO / SIO |
| GPIO12 | 32 | PWM0 | GP28 | EXT1.31 | PIO / SIO |
| GPIO13 | 33 | PWM1 | GP29 | EXT1.33 | PIO / SIO |
| GPIO14 | 8 | TXD0 | GP13 | EXT1.34 | UART0.Rx |
| GPIO15 | 10 | RXD0 | GP12 | EXT1.32 | UART0.Tx |
| GPIO16 | 36 | SPI1 CE2 | GP30 | EXT1.35 | PIO / SIO |
| GPIO17 | 11 | SPI1 CE1 | GP31 | EXT1.37 | PIO / SIO |
| GPIO18 | 12 | PCM_CLK/PWM0 | GP32 | EXT2.3 | PIO / SIO |
| GPIO19 | 35 | PCM_FS/PWM1 | GP33 | EXT2.5 | PIO / SIO |
| GPIO20 | 38 | PCM_DIN | GP34 | EXT2.7 | PIO / SIO |
| GPIO21 | 40 | PCM_DOUT | GP35 | EXT2.9 | PIO / SIO |
| GPIO22 | 15 | — | GP36 | EXT2.11 | PIO / SIO |
| GPIO23 | 16 | — | GP37 | EXT2.13 | PIO / SIO |
| GPIO24 | 18 | — | GP38 | EXT2.15 | PIO / SIO |
| GPIO25 | 22 | — | GP39 | EXT2.17 | PIO / SIO |
| GPIO26 | 37 | — | GP40 | EXT2.23 | PIO / SIO |
| GPIO27 | 13 | — | GP41 | EXT2.25 | PIO / SIO |

Housekeeping takes the rest. The Pi's 3V3, 5V and ground pins tie to the
harness rails rather than to a fixture pin.

| Fixture GP | PICO2-XXL | Signal | Mux |
| --- | --- | --- | --- |
| GP0 | EXT1.4 | PWR_EN — load switch `ON` | SIO |
| GP1 | EXT1.6 | PWR_ALERT — INA226 alert | SIO |
| GP2 | EXT1.8 | RUN_DRV — open-drain `RUN` | SIO |
| GP3 | EXT1.10 | BLEED_EN — 3V3 bleeder gate | SIO |
| GP4 | EXT1.12 | INA226 SDA | I2C0.Sda |
| GP5 | EXT1.14 | INA226 SCL | I2C0.Scl |
| GP6 | EXT1.16 | VBUS1_EN | SIO |
| GP7 | EXT1.18 | VBUS2_EN | SIO |
| GP25 | EXT1.25 | VBUS3_EN — also lights the user LED | SIO |
| GP42 | EXT2.27 | AUDIO_L | ADC |
| GP43 | EXT2.29 | AUDIO_R | ADC |
| GP44 | EXT2.31 | PI_3V3_SENSE | ADC |
| GP45 | EXT2.33 | VBUS1_FAULT | ADC-capable |
| GP46 | EXT2.35 | VBUS2_FAULT | ADC-capable |
| GP47 | EXT2.37 | VBUS3_FAULT | ADC-capable |

That is 28 plus 15 against 43 usable, so it fits with nothing spare. Two
notes on the margins. `VBUS3_EN` deliberately uses GP25, the LED pin: a
2.2 kΩ load is harmless on an output, and the LED then indicates that a USB
port is powered, which is a feature rather than a compromise. And GP8–11
and GP24 come back — five pins — for any build that fits neither PSRAM nor
the SD card, which is the reserve if the housekeeping list grows.

**The module must be hardware revision C or later.** Rev C was the first to
place EXT1 and EXT2 exactly 900 mil apart centre to centre; on Rev A and B
they sit 0.34 mm closer, and the mounting holes moved 0.5 mm between B and
C. A socket footprint drawn to 900 mil will fight an earlier board. Rev B
is also where SD_DAT0 moved from GPIO12 to GPIO24, which is what makes
GP12 free for the console pair above — on a Rev A board this pin map does
not hold.

### The GPIO14/15 handoff

GPIO14/15 are both the console and a device under test, and that conflict
cannot be designed away — it can only be sequenced.

**The console cannot move to another header pin.** UART0's alt functions
are GPIO14/15 (ALT0), 32/33 (ALT3) and 36/37 (ALT2); the mini-UART's are
14/15 (ALT5), 32/33 (ALT5) and 40/41 (ALT5). None of GPIO32–41 are on the
40-pin header — 32/33 go to the Bluetooth module — so 14/15 is the only
pair available, and since both UARTs land there, the console cannot even
be parked on one while the other is tested.

This is why the console is tunnelled through the fixture rather than
wired to a separate adapter. A permanently attached USB-serial adapter
puts a **second driver** on the console net: its idle-high TX fights the
fixture's shadow pin whenever a test drives GPIO15, which would need a bus
switch on the line or a human unplugging a cable. With the fixture as the
only driver, the role change is a firmware state transition.

The sequence, explicit on both sides and never inferred:

1. The runner sends `console detach` on the control interface. The
   fixture stops the passthrough and reassigns its GP14/15 pins to
   whatever the case needs — UART peer, logic capture, or plain GPIO.
2. The Pi's test binary prints its banner and a line announcing it is
   taking GPIO14/15, tears down the console UART, and runs the case,
   accumulating results in RAM. For a UART case the results can be
   exchanged over 14/15 directly, since the fixture is the peer.
3. The binary restores the console on 14/15 at 115200 and prints the
   accumulated `#HIL` lines.
4. The runner sends `console attach` and reads them.

Steps 2–3 are a window with no console. A case that hangs inside it never
restores the console, which is why the per-case timeout, power cycle and
re-`HELLO` recovery loop is mandatory infrastructure rather than a
refinement — bare-metal cases hang rather than fail. The window is not
unwitnessed, though: the fixture is sitting on those exact pins with a
capture running, so a hang there leaves more evidence than it would with a
separate adapter, which would see nothing at all.

Two assertions come out of the fixture owning this line. It can measure
the console's real bit period and framing, so UART0 at 1.5 Mbaud is
verified on the link `rpi-loader` actually uses rather than a synthetic
case. And the mini-UART's dependence on a pinned `core_freq=250` becomes
measurable the same way, since a drifting core clock shows up directly as
a wrong bit period.

## Rest of the HAT

- **1 kΩ series resistors on every shadowed line**, fixture pins Hi-Z by
  default. Contention then cannot damage anything, and a test that forgets to
  release a pin fails loudly instead of smoking a pad.

  1 kΩ out of the 330 Ω – 1 kΩ range the shadowing experiment started from,
  because the upper end costs nothing here and buys the most margin.
  Contention is 3.3 V / 1 kΩ = 3.3 mA, against a 16 mA per-pin maximum on
  BCM283x and 12 mA on RP2040 — roughly a 4× margin on the tighter of the
  two, where 330 Ω would leave 10 mA and almost none. The cost is edge rate:
  into the ~50 pF of a breadboard hop plus pin capacitance, τ is 50 ns and a
  10–90 % rise about 110 ns, comfortably inside the 667 ns bit period the
  loader's 1.5 Mbaud transfers need. Somewhere past ~4.7 kΩ that stops being
  true, which is what bounds the range from above rather than any DC concern.

  Measured on the two-wire breadboard fixture with 1 kΩ in each signal line:
  the console is unaffected — the loader still negotiates to 1.5 Mbaud and
  the whole board suite passes exactly as on direct wiring — and the fixture
  drives a released board pin cleanly in both directions.

  **On an RP2350 fixture that is not enough for a line it only watches.** A
  series resistor in front of a pad with no pull is exactly the arrangement
  erratum E9 describes, and the marker line found it: with 1 kΩ in line and
  nothing else, the fixture recorded 55 of the 1640 edges a case emitted.
  Not 55 clean ones either — the 1 ms square wave came through on an exact
  timebase grid while everything at 50 µs and below vanished, and stray
  10–40 µs pairs appeared between the real edges. A signal that survives at
  1 kHz and disintegrates by 20 kHz is worse than one that fails outright,
  because a bench wired that way looks like it works.

  So every **observed** line needs a pull-down at the fixture's pad, on the
  fixture side of the series resistor — at the board's end it does nothing
  about a latch at the pad. 8.2 kΩ is the largest value the errata sheet
  accepts; with 1 kΩ in front of it a driven high still reaches 2.94 V
  against a 2.31 V V<sub>IH</sub>. Restoring it takes the capture from 55
  edges back to all 1640, and removing it loses them again.

  What the erratum actually needs is a *defined level*, not specifically a
  pull-down — a pad that is driven, or held by a pull in either direction,
  has no floating state to latch. That is why the two-wire breadboard's
  console lines never needed one: GPIO14/15 are driven from one end or the
  other at all times. On a 1:1 shadow that exemption disappears, because
  every line including 14/15 is observed high-impedance during a header
  sweep, so on the harness they carry both resistors like any other.

  **The four I2C lines are the real exception, and a pull-down breaks
  them.** GPIO0/1 (ID_SD/ID_SC) and GPIO2/3 (SDA1/SCL1) are open-drain
  buses that idle high through a pull-up, and a pull-down to ground fights
  it. Against the 3.9 kΩ the ID bus already carries, 1 kΩ plus 8.2 kΩ to
  ground puts the bus high at 3.3 × 9.2/13.1 = **2.32 V** against a 2.31 V
  V<sub>IH</sub> — on the threshold, which fails intermittently rather than
  cleanly, and takes the ID EEPROM and the `I2C_SLAVE` role with it. So
  those four lines keep the series resistor and **omit the pull-down**,
  letting the bus pull-up hold the pad instead.

  The series resistor then sizes the pull-up, because the fixture has to
  assert a low *through* its 1 kΩ and the divider has to land under a
  0.99 V V<sub>IL</sub>:

  | Bus pull-up | Fixture's asserted low | |
  | --- | --- | --- |
  | 1.8 kΩ | 1.18 V | fails |
  | 2.2 kΩ | 1.03 V | fails |
  | 3.9 kΩ | 0.67 V | works — the ID bus as built |
  | 4.7 kΩ | 0.58 V | works |

  So **I2C1 wants 3.9–4.7 kΩ, not the common 1.8 kΩ.** Fit 1.8 kΩ and the
  fixture can never assert a valid low, so the slave role simply does not
  work with nothing on the schematic to suggest why. The 1 kΩ still earns
  its place on these lines, because GPIO2/3 are also ordinary GPIO that a
  header sweep drives push-pull, where contention is real.

  That leaves **28 series resistors and 24 pull-downs**, plus bus pull-ups
  on the four I2C lines and on the open-drain `PWR_ALERT` and `VBUS_FAULT`
  inputs. Worth fitting as 4-element arrays rather than 52 discretes.

  What the pull-down costs is the top of the resolution range. With 1 kΩ and
  a 10 kΩ pull-down the bench still resolves every deliberate edge — a 100
  period square wave, 1 µs pulses at a 973 ns median, and a 1000-edge 20 kHz
  burst with none missing, agreeing with the board's own clock to 43 ppm —
  but of 400 back-to-back `set_high`/`set_low` runts it resolves 2, where
  direct wiring resolves 336. Those runts are not a controlled stimulus and
  no case can ask for one, so this is the right trade; it is also the
  measurement behind "a marker has to be held wide enough to see."

  The other thing it costs is the board's **internal pull-ups**. Those are
  around 50 kΩ on BCM283x, so against 1 kΩ plus 8.2 kΩ to ground a line
  with the board's pull-up enabled sits at 3.3 × 8.2/59.2 ≈ **0.46 V** —
  below V<sub>IL</sub> at both ends. A case that enables a pull-up and
  expects to read high cannot pass, and the fixture cannot see pull-up
  configuration digitally at all. Internal pull-*downs* are unaffected,
  since they pull the same way the external one does.

  Six lines have a way out that costs nothing. The shadow puts Pi GPIO10,
  11, 12, 13, 26 and 27 on GP26–29 and GP40–41, all of which are
  ADC-capable, so on those the fixture can *measure* the divider instead
  of reading a level: an enabled pull-up shows ≈0.46 V, about 570 counts at
  12 bits, against 0 V for a driven low or a float. Pull-up configuration
  is therefore testable on six of the twenty-eight without any extra
  hardware — the case just has to reach for the ADC rather than a digital
  read.

  What the resistor does **not** do is let the fixture override a pin the
  board is actively driving, and it is worth being unambiguous about that
  because the arithmetic invites the opposite conclusion. Driving the
  fixture's end of the line low while the Pi's UART held GPIO14 high read
  back **low at the fixture and high at the Pi**: each end owns its own side
  and the resistor takes the difference. On direct wiring the identical
  operation read back *high* — the Pi's driver simply won, and the two pads
  were shorted. So the resistor converts a short into a divider; it does not
  arbitrate. Shadowing a pin the board drives still requires the board to
  release it first, which for GPIO14/15 is what the console handoff is for
  and for every other pin is the case's own responsibility.
- **ID EEPROM** on ID_SD/ID_SC. Identifies the HAT revision to the runner,
  and incidentally puts a real device on BSC0's HAT routing.
- **Real devices**, because a PIO emulation of a peripheral only tests the
  driver against our own understanding of it: I2C EEPROM, temperature
  sensor and an SH1106 footprint; an SPI flash or MCP3008; IR receiver and
  IR LED; a PCM5102 I2S DAC (SCK grounded, XSMT high, FMT low — the Pi
  emits no MCLK, and a floating SCK is silence).
- **Analog audio path**: TRRS jack breakout → DC block and divider →
  fixture ADC, plus a header out to a USB audio dongle.
- **3-pin recovery console header** (GND/TXD/RXD) on the GPIO14/15 net,
  left unconnected in normal operation. Takes a hand-plugged USB-serial
  adapter for bootstrapping a new board or working on a wedged fixture.
- **Marker-pin header.** The convention: a test binary toggles a
  designated GPIO around events and the fixture timestamps the edges with
  PIO. That single primitive yields objective numbers for PWM
  frequency and duty, UART baud, SPI clock rate, generic-timer drift over
  minutes, IRQ latency, DMA completion latency, and page-flip interval —
  16.67 ms ± ε meaning vsync genuinely works. Two capture modes: raw
  sampling for short high-rate bursts, edge timestamping for long windows,
  so that "LRCLK ran 10 s with no gap" is an assertion about audio
  underrun.

## USB strategy

USB gets its own section because it is the one domain where the obvious
arrangement does not work.

**The constraint.** RP2040 and RP2350 have exactly one USB controller, it
is full-speed only, and it is either host or device but not both. The
fixture's controller is already spent on the CDC control link, so the
fixture cannot also present itself as a device to the Pi. Adding a second
MCU to do that job means a second control path for it — the Pi has taken
its only USB port — which is a UART tunnel and a routing layer in
the firmware for one stimulus. Not worth it.

**The decision: no fixture MCU on the USB bus at all.** The Pi's USB
devices are real ones, plugged into the Pi's own USB-A ports through
**in-line VBUS switches driven by fixture GPIO**. Cutting VBUS is the
attach/detach stimulus, the device set is fixed and known, and the whole
thing is sequenced over the control interface alongside every other fixture
command.

This covers more than an MCU gadget would, not less:

- A USB 2.0 **flash drive is a high-speed device**, so it exercises DWC2's
  direct high-speed path — which a full-speed-only MCU can never reach.
- A **full-speed keyboard** behind the Pi's hub exercises **split
  transactions**, the genuinely bug-prone part of a DWC2 host driver. On
  Pi 2/3 every external port sits behind the LAN9514, so both paths are
  reachable on the same board.
- A **small hub** covers topology and depth. It is a device under test on
  its own switched port, not bench infrastructure.
- Descriptor dumps for each device assert against a golden, and real
  devices carry real-world quirks that a synthesised gadget does not.

### Why VBUS switching rather than a switchable hub

The obvious part for this job is a hub with per-port power switching,
driven by `uhubctl`. It does not work here. `uhubctl` switches ports by
sending USB control transfers **to the hub, from the host the hub is
attached to** — and in this rig that host is the Pi, running bare metal
and being the thing under test. The orchestrator has no path to such a
hub at all, and having the DUT sequence its own stimulus through
hub-class support that is itself under test is circular.

Switching VBUS from the fixture inverts that. Cutting VBUS to a
bus-powered device removes its D+ pull-up, which is an electrical detach
as far as the host controller is concerned, and every device in the set is
bus-powered. Control stays with the orchestrator, it costs roughly a
dollar and one GPIO per port instead of $25–35, and it behaves identically
on every Pi model rather than depending on which hubs happen to implement
per-port power switching.

A USB power-distribution switch (TPS2051B / AP22653 class, SOT-23-5) is
the right part: enable input, current limit, and a fault flag the fixture
can read, so a device browning out the port becomes an assertion rather
than a mystery. Only VBUS is switched; D+/D− pass straight through.

**These switches cannot live on the HAT.** The Pi's USB is not on the
40-pin header, so they belong on a small satellite board — USB-A female
sockets in, short pigtails to the Pi's ports, and a ribbon to the HAT
carrying the enable lines, fault flags and ground. Keep the pass-through
traces short: the board sits in the middle of a 480 Mbps link, and a stub
with sloppy routing turns into an intermittent the rig will blame on the
HAL. For Phase 0 the same thing is a relay board or a load-switch breakout
interrupting the red wire of a USB extension cable.

**One physical consequence to note**: because USB is not on the header,
USB testing always involves cables, even though every other connection is
the HAT. On Pi 4 this is convenient — DWC2 is on the USB-C port, which the
HAT frees up by powering through pins 2/4, while the USB-A ports go
through the VL805 xHCI.

### What this does not cover

Stated plainly, because these are real gaps and not oversights.

- **Scripted keystroke injection.** A real keyboard cannot be told what to
  type, so there is no "typed exactly `abc`" assertion. HID coverage is
  descriptor correctness, interrupt-IN polling behaviour, idle NAK
  handling, and output reports (toggling a lock LED via `Set_Report`).
- **Arbitrary and malformed descriptors.** The strongest possible USB host
  test is a Linux `raw-gadget` peer synthesising non-standard and
  deliberately broken devices — the interface syzkaller uses to fuzz host
  stacks. That needs a device controller, which a mini PC does not have,
  so it needs an OTG-capable Linux board. Deferred: if the orchestrator is
  itself a Pi 4 or 5, its USB-C is a UDC and this costs nothing, so the
  cheap version of this capability is a configuration choice about the
  orchestrator rather than a purchase.
- **A wire-level witness.** Every USB assertion is currently the Pi
  reporting on itself, which is the failure mode the networking tests
  avoid by taking an independent pcap. A spare RP2040 running
  [usb-sniffer-lite](https://github.com/ataradov/usb-sniffer-lite) taps
  D+/D− and streams decoded packets out a VCP for a few dollars. It is
  low- and full-speed only, so it cannot see the flash drive's traffic.
  Worth adding once the rest works; not part of rev A.
- **The USB device side of the HAL.** There is no device-mode driver to
  test yet. When there is, the topology inverts and gets simpler: a plain
  cable from the Pi's USB-C to the orchestrator, with `lsusb -v`, `usbmon`
  and pyusb as the witness, and no extra hardware at all. Worth reserving
  that cable path now.

## Determinism: the rig brings its own world

The largest source of flaky HIL results is the environment, not the
device.

- **Ethernet**: an isolated subnet on its own switch, never the house LAN,
  with `dnsmasq` for DHCP and fixed UDP-echo, HTTP, NTP and
  TLS-with-pinned-cert endpoints. Give each board a **fixed lease keyed to
  its MAC**: the runner then knows the DUT's address before it boots and
  can assert it got the one it expected, which is a free identity check on
  top of the mailbox banner.
- **Wi-Fi**: the rig's own AP — `hostapd` on a dedicated dongle, or a
  cheap travel router — with a fixed SSID, BSSID and PSK on its own
  subnet. Then a scan asserts *"the known BSSID is present"* rather than
  "some networks were found", and a deliberate wrong-PSK case covers the
  failure path.
- **HDMI**: the USB capture stick presents a fixed EDID, making EDID
  parsing deterministic regardless of what monitor is attached. An EDID
  emulator dongle covers a second mode set.
- **Bluetooth**: a dedicated USB BT dongle claimed raw by Bumble, so the
  peer is a script rather than the host OS's stack.

## One harness, boards swapped through it

There is one harness, and the board under test is swapped into it. That is
a deliberate trade, and it is worth being honest about which way it cuts.

What it costs: a swap means unmating the HAT and moving Ethernet, HDMI,
USB and audio across — five connections and a human at the bench. So
**unattended running is bounded to whichever board is currently mated**. A
full sweep of the matrix is a sequence of sessions with a swap between
them, not one overnight run. The camera is worse than that and is dealt
with separately below.

What it buys, beyond not building four of everything: every witness in the
bill of materials is bought once, so the bench can have *all* of them
rather than the subset four boards could afford. Nothing is shared through
a multiplexer, so there is no HDMI auto-switch to characterize, no
per-board addressing in the runner, and no question of which fixture a
command reached. The whole class of "which board did that come from" bugs
does not exist.

Two things follow for the design, and both are already true of it:

- **The HAT must be model-agnostic**, because it is the one HAT and it
  meets every board in turn. That is what
  [Power control](#power-control-the-harness-supplies-the-pis-5v) is about:
  5V on pins 2/4 works on every model, and nothing needs soldering per
  swap.
- **The runner should not be told which board is mated — it should ask.**
  `Mailbox::board_revision` returns the same packed revision code
  `/proc/cpuinfo` reports, so the board identifies itself on every run.
  A fixed DHCP lease keyed to the MAC is the same fact arriving by a
  second route. Neither needs a config file describing the bench, and
  neither can go stale after a swap the way a config file can.

Connector wear would be the obvious consumable, and it is why the Pi
interface is an adapter and a ribbon rather than a socket on the harness —
see [Mechanical](#mechanical-an-adapter-a-ribbon-and-a-power-lead). The
repeated mating happens at a $3 cable and a locking power connector,
neither of which is soldered to anything that matters, so wearing one out
is a purchase rather than a rework. That leaves the **CSI ribbon** as the
part that will fail first, and it is addressed below.

### The boards it is swapped between

| Board | SoC / core | Arches | Ethernet | 3.5 mm | USB | HDMI | CSI |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Zero W | BCM2835 / ARM1176, 1 core | ARMv6 | none | none | 1× OTG, DWC2 direct | mini | 22-pin |
| Pi 3 B | BCM2837 / A53 ×4 | 32 + 64 | LAN9514 | yes | 4× behind LAN9514 | full | 15-pin |
| Pi 4 B | BCM2711 / A72 ×4 | 32 + 64 | dedicated GbE | yes | VL805 xHCI + DWC2 on USB-C | micro ×2 | 15-pin |
| Pi 5 | BCM2712 + RP1 / A76 ×4 | 64 only | via RP1 | none | via RP1 | micro ×2 | 22-pin ×2 |

What each one uniquely exercises:

- **Zero W** — ARMv6, a single core, and no ARM generic timer, so the
  multicore and generic-timer groups do not apply at all. Its USB is DWC2
  with **no hub in the path**, which is the only place the direct FS/HS
  transfer path is reachable; every port on a Pi 3 sits behind the
  LAN9514, so those boards can only ever exercise split transactions.
- **Pi 3 B** — the reference board: both execution states, the LAN9514,
  the analog jack, Wi-Fi and Bluetooth.
- **Pi 4 B** — the relocated BCM2711 peripheral map, dedicated gigabit
  Ethernet rather than a USB hub, and xHCI alongside DWC2.
- **Pi 5** — RP1 across PCIe, which is a different bus topology rather
  than a relocated map. 64-bit only, since the A76 does not support
  AArch32 at EL1, so it *removes* a matrix cell rather than adding one.

Pi 2 is omitted: BCM2836's peripheral map is effectively BCM2837's, so it
would only add the Cortex-A7 and a no-radio configuration. Zero 2 W is
omitted for the same reason — BCM2710A1 is the same die as BCM2837 — but
it is the cheapest board in the set to support, since it needs no HAL work
at all and shares the Zero's no-hub USB topology. With one harness the
marginal cost of adding it to the matrix is the board and a swap, not a
second rig, so it is worth having on the shelf.

Note that the Zero's unique USB coverage is gated behind the ARMv6 port,
which is the most expensive port in the set. If that coverage is wanted
before ARMv6 lands, a Zero 2 W provides it with existing drivers.

### What a swap actually touches

Most of the bench does not move. Worth knowing which parts do, because
that list is the swap procedure.

**Stays put:** the harness board and the PICO2-XXL mated to it, the 5V
inlet, the VBUS switch board, the USB device set, the Ethernet switch, the
Wi-Fi AP, the Bluetooth dongle, the HDMI capture stick and the audio
dongle. All of it belongs to the harness, not to a board.

**Stays on the board:** its header adapter. That is the point of it — the
Pi's own 40-pin header is mated once, when the adapter goes on, and never
again.

**Changes with the board:** the signal ribbon and the power lead unplug
from the adapter, and plug into the next board's. Then Ethernet, USB, the
3.5 mm audio lead where the board has a jack, and the HDMI cable, which is
a different connector per board so it is the cable that changes rather
than being re-plugged. Nothing needs re-configuring afterwards, since the
board announces its own revision.

**Does not travel at all:** the camera. The CSI ribbon will not survive
repeated insertion, so the camera stays a declared capability on whichever
board is left wired for it and skips everywhere else. That is the one
place where "swap the board" is the wrong answer, and it is the reason the
camera tests are tier-gated rather than part of a sweep.

One benefit of the single harness is that HDMI needs no multiplexing. With
a rack, a 5-input auto-switch would have to be characterized and trusted;
here the capture stick is wired straight to whichever board is mated, and
there is no switch in the path to have an opinion about EDID.

### HDMI: two things to get right

- **Four cable types** — mini for the Zero, full-size for Pi 3, micro for
  Pi 4 and Pi 5. Only one is in use at a time, but all four have to be
  bought deliberately, and the right one is part of the swap.
- **Allow settling, then verify before asserting.** A board that is
  powered but has not brought up HDMI presents no signal at all, so
  confirm the capture device reports the expected mode before making any
  claim about pixels. With no switch in the path the capture stick's own
  EDID is the only one in play, which is one fewer thing to characterize
  and one fewer thing to drift.

### USB devices

VBUS comes from the Pi's own port, so a device cannot be shared between
boards without a high-speed mux — which is exactly the signal integrity
risk avoided by keeping USB off the HAT. With one board mated at a time
that never comes up: three switched ports (keyboard, flash drive, small
hub) belong to the harness, and whichever board is in the socket is the
one they attach to.

**The Zero is the exception**: one micro-USB OTG port, so it gets one
device *or* a hub, not both. Putting a hub on it would destroy the
direct-path coverage that is the reason to have a Zero at all, so on that
board only a single switched device is connected.

### Two consequences worth planning for

**The orchestrator's USB port budget is comfortable.** One fixture
presents two CDC interfaces, plus the capture stick, audio dongle,
Bluetooth dongle and Wi-Fi AP — six devices rather than the nine a rack
would have needed. A powered hub is still worth having for the current
budget, but the port count is no longer a constraint on the design.

**Pi 5 needs its power budget checked.** The switch itself is no longer
the obstacle — 6A covers a Pi 5's 5A official supply — but the wiring to
the header is, since 5A over a metre of 18 AWG lands under the voltage the
board wants, and 5A through two header pins is close to their contact
rating. Its PMIC and power button also introduce soft-off states that a
plain load switch does not model.

## Phases

Staged so nothing is ever blocked on a PCB, and so PCB spend follows
driver support rather than leading it.

- **Phase 0** — breadboard, wired for one bus group at a time. Proves the
  runner design and the self-reporting protocol with no PCB at all. An
  RP2040 board already on hand does the first half of this; the PICO2-XXL
  does the rest of it, since 32 of its GPIO reach a breadboard and every
  test that is not a full-header sweep fits in 32.
- **Phase 1** — HAT rev A: PICO2-XXL socketed, switched 5V with the load
  switch and 3V3 sense, full-header pin shadowing through series
  resistors, ID EEPROM, the real-device complement, analog audio path,
  marker-pin header, recovery console header, optional `RUN` header, plus
  the USB VBUS switch satellite board and its ribbon. Hand-solderable
  throughout — 0805 passives, a pre-built module rather than a bare QFN.
- **Phase 2** — respin: the hardware backfeed interlock, whatever rev A
  gets wrong, and possibly absorbing the QFN-80 onto the HAT using
  Olimex's published design.
- **Phase 3** — bench build-out: the Ethernet switch, the Wi-Fi AP, a
  powered hub and the capture hardware, so the harness has every witness
  rather than a subset. Validate rev A by swapping between **Pi 3 and
  Pi 4 first**, since those are the only boards whose peripheral drivers
  exist today; Pi 5 and Zero W wait on the RP1 and ARMv6 ports
  respectively and cost only a swap once they land.

## Cross-checks worth keeping

**Assert from both ends wherever the orchestrator can see the same event.**
The device printing its own success is not evidence: a pcap catches bad
checksums, a missing ARP and wrong TCP window behaviour that are invisible
from the device's point of view, `rpi-loader`'s `sd-read` checks from
outside what the Pi's write path claimed to write, and the 3V3 rail sense
plus a re-`HELLO` proves a watchdog reset really happened rather than the
code merely reaching the line.
