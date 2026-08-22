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
| Olimex PICO2-XL (RP2350B) | €5 | fixture | the fixture MCU |
| HAT rev A PCB + passives | $25–40 | fixture | wiring, power switching, real devices |
| High-side load switch | ~$2 | fixture | Pi power control |
| INA226 or shunt | ~$2 | fixture | rail current sense |
| USB-serial adapter | ~$5 | orchestrator | smoke tier, and recovery — the bench-tier console is tunnelled through the fixture |
| USB VBUS switch board | ~$5 | fixture | USB attach/detach stimulus |
| USB flash drive, keyboard, small hub | ~$15 | — | the real USB devices under test |
| UVC HDMI capture stick | ~$15 | orchestrator | video pixel assertions |
| USB audio dongle | ~$10 | orchestrator | analog audio capture |
| USB Ethernet NIC | ~$10 | orchestrator | isolated wired network |
| Wi-Fi AP or dongle | ~$20 | orchestrator | isolated wireless network |
| Bluetooth dongle | ~$10 | orchestrator | scripted BLE peer |

Excluding the Pis, which the test matrix requires anyway. Around $120 for
a single-board rig with every witness present; a useful subset is far
cheaper, since tests skip on absent capabilities rather than failing.
Scaling to the four-board rack lands around $350–450 — see
[Multi-board rack](#multi-board-rack) — but that spend is staged behind
driver support rather than paid up front.

## Fixture MCU: Olimex PICO2-XL

An [Olimex PICO2-XL](https://www.olimex.com/Products/RaspberryPi/PICO/PICO2-XL/open-source-hardware)
(RP2350B, 48 GPIO, €5), socketed on the HAT. Development starts on
RP2040 boards, since those are already on hand and the firmware keeps its
board differences in one pin-map module, so the move is a feature flag
rather than a port.

### Why not a Pico or Pico 2

The package, not the chip, is the constraint. RP2350 ships in two:
RP2350**A** (QFN-60, 30 GPIO, 4 ADC) and RP2350**B** (QFN-80, 48 GPIO,
8 ADC). A **Pico 2 is the A part**, and like a Pico it breaks out only 26
of its 30 GPIO — GP23/24/25/29 go to SMPS mode, VBUS sense, the LED and
the VSYS divider. Raspberry Pi has never shipped a B-package Pico, so 48
GPIO means a third-party board or a bare QFN-80 on the HAT itself.

| | Pico (RP2040) | Pico 2 (RP2350A) | PICO2-XL (RP2350B) |
| --- | --- | --- | --- |
| SRAM (capture depth) | 264 KB | 520 KB | 520 KB |
| Soft peripherals | 8 PIO SMs | 12 PIO SMs | 12 PIO SMs |
| GPIO on the die | 30 | 30 | **48** |
| GPIO exposed | 26 | 26 | **48** |
| ADC channels free | 3 | 3 | 8 |
| Control link | native USB CDC | same | same, USB-C |

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
- **8 ADC channels** rather than 4. Analog audio L and R, the 3V3-rail
  sense and the current shunt each want their own; that set does not fit
  in the 3 a Pico leaves free.
- **Open-source hardware.** KiCad sources, schematic and Gerbers are
  published, so the footprint drops into the HAT layout, and if a later
  revision absorbs the fixture instead of socketing it, the reference
  design for the QFN-80 support circuitry is already in hand.
- **Power topology fits.** VSYS takes 1.8–5.5 V and the board regulates
  its own 3V3, so the HAT feeds it from the always-on 5V rail with no
  LDO of its own.
- **2 MB flash / 520 KB SRAM** is ample for the fixture firmware. The PICO2-XXL
  (€9 — 16 MB flash, 8 MB PSRAM, microSD) is the same 50 × 28 mm outline
  if capture depth ever justifies it.

### Consequences for the layout

It is **not castellated** — 2.54 mm through-hole pads, headers
unpopulated — so rev A sockets it rather than reflowing it. That is the
better choice for a first spin regardless: the fixture stays replaceable,
and the same board moves between the breadboard and the HAT.

At 50 × 28 × 8.3 mm it occupies roughly a third of a 65 × 56.5 mm HAT,
with header rows down both long edges. Its placement, and the stack height
above the Pi's own header, are the dominant mechanical constraints.

Erratum **RP2350-E9** — GPIO inputs latching part-way high when relying on
the internal pull-down — applies to both packages and needs reading before
layout, because a fixture whose job is high-impedance observation of lines
that go floating is close to its worst case. The errata sheet's external
pull-down works around it, and this design wants external resistors on
those lines anyway.

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
GPIO14/15 buys three things. It halves the cables per board in a rack, and
makes board identity self-describing over the control interface instead of
depending on udev rules to tell apart adapters that often share a USB
serial number. It lets the fixture timestamp console bytes against the
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

## Power control: the HAT supplies the Pi's 5V

The HAT switches 5V into header pins 2 and 4. This is the primary reset
mechanism, in preference to the `RUN` pad.

Why: the HAT plugs onto the header and that is the *entire* physical
connection per board — no solder step, no flying leads, and nothing
board-revision-specific, since 5V is pins 2/4 on every model whereas
`RUN` is a 2-pin pad on Pi 2/3 and `GLOBAL_EN` on Pi 4, neither of them on
the 40-pin header. Back-powering the 5V rail through the header is
permitted by the HAT specification and is what UPS and PoE HATs do. It
also resets the LAN9514, the Wi-Fi chip and attached USB devices, which a
`RUN` reset leaves in whatever state they wedged in.

Five things this has to get right.

1. **The HAT needs its own upstream supply.** This inverts the usual HAT
   relationship: the fixture must stay alive while the Pi is dead, so it
   cannot be powered from the Pi's 3V3 pins. A dedicated 5V inlet →
   always-on 5V rail, which feeds the fixture's VSYS directly and, through
   the load switch, header pins 2 **and** 4 in parallel with several GND
   pins. Budget 4A: a Pi 4 under load with peripherals draws upwards of
   1.5A, and the rig's own supply must not be a variable. That budget rules
   out a passive USB-C breakout with 5.1k CC resistors, which without PD
   negotiation is entitled to 500–900mA and can reach 3A only if the source
   happens to advertise it. See [Where the 5V comes
   from](#where-the-5v-comes-from).
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

   A switched bleeder is the cheap insurance: an N-FET and ~150–330 Ω from a
   3V3 header pin to ground on a fixture GPIO, enabled only while the switch
   is off. Ten or twenty milliamps swamps any plausible clamp injection and
   collapses the rail in tens of milliseconds. Two things it does not buy:
   3V3 is a *proxy*, since the SoC's core rails come off their own switchers
   and draining 3V3 does not prove those fell, so any shortened wait has to
   be validated against actual cold boots rather than against the sense
   reading; and it has to be interlocked off while the board runs, or it
   corrupts every current measurement taken through the shunt.

### Where the 5V comes from

A bench brick into a barrel jack is the obvious answer and it works. The
better one, when the orchestrator is a PC, is **the orchestrator's own ATX
supply** — a 4-pin Molex peripheral tail gives red +5V and two grounds on
18 AWG, and a 350W unit's 5V column covers a four-board rack (four HATs at
4A is 80W, inside the combined +3.3V/+5V cap those labels usually carry;
read the column rather than the wattage). Avoid SATA power for this — three
+5V pins, but contacts rated around 1.5A each — and avoid +5VSB, which is
always live but only good for 2–3A.

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

- **Fuse the inlet, per HAT.** A 4A brick self-limits into a fault; a 5V
  rail rated 15–20A does not. The load switch protects the Pi's branch, but
  the always-on rail feeding the fixture and the HAT's own devices is
  upstream of it. A 5A polyfuse or blade fuse.
- **The load switch's `EN` must default off.** At PC power-on the rail comes
  up hard across every HAT at once; if `EN` floats or pulls high, every Pi
  gets hard-switched with no soft-start before any fixture firmware is
  running, which is the uncontrolled cold start item 5 is about. Pull `EN`
  down so the rail arrives with the Pi off and the fixture turns it on
  deliberately.
- **Budget the cable drop.** 18 AWG is roughly 21 mΩ/m, so a metre out and
  back at 4A is about 170mV, landing a nominal 5.0V rail near 4.83V at the
  header against the 4.75V the Pi wants — before the switch's own drop. Short
  runs, and one short 5V bus for a rack rather than four long tails. It is
  worth measuring at the header under load rather than assuming.
- **The rail dies when the PC does.** +5V is only live in S0, so shutting
  down or rebooting the orchestrator power-cycles every board under test and
  every fixture. Almost certainly what you want, since a fixture is no use
  without its orchestrator — but "always-on" now means "always on while the
  PC is up", which is better known in advance than diagnosed as a fault.

Noise is not the concern it looks like: ATX specifies +5V at ±5% with 50mV
ripple, inside what a Pi tolerates, and the inlet wants local bulk anyway.
The ±5% is worth carrying downstream, though — it means the rail can sit
legitimately at 5.25V, so anything on it with a 6V absolute maximum, an
ideal diode passing VSYS included, has well under a volt of headroom and
wants a clamp at the inlet alongside the fuse.

**Part choice**: a high-side load switch IC (TPS22965 / TPS2557 / AP2281
class) rather than a discrete FET or a relay. One package gives a low
R<sub>DS(on)</sub> 3A+ pass element, **soft-start** (hard-switching into
the Pi's discharged bulk capacitance is an inrush spike that can brown out
the shared supply), a **programmable current limit** around 3–3.5A — which
also replaces the input polyfuse being bypassed by powering through the
header — and a **FAULT output** the fixture can read, so an overcurrent
during something like camera LDO bring-up becomes an assertion instead of
a mystery brownout. Relays work and the cycle count is a non-issue, but
they are bulkier, slower, need a coil driver and flyback diode, and offer
no soft-start or current limit.

### Committing it to the board rather than to a prototype

Neither of the switch's two settings is waiting on a bench measurement.
The current limit sits above the published PSU ratings — 2.5A for a Pi 3,
3A for a Pi 4 — and below what two 5V header pins will carry, which
brackets it at 3–3.5A. The soft-start window is wide at both ends: a
500mA-limited supply boots the board, so a slow ramp is safe, and any ramp
at all beats hard-switching into discharged bulk capacitance. So the board
carries the adjustment rather than a prototype deriving it — a swappable
current-limit resistor, a populated-or-not slew-rate capacitor, test points
either side of the shunt and on 3V3, `EN` and `FAULT` — and every pin gets
checked against the datasheet before it becomes a footprint. A wrong value
is a tweezer; a wrong pinout is a respin.

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
here — leaving a 20V, 4A-class SOT-23 around 30mΩ, or a SOIC-8 in the low
tens of milliohms for the full 3A. At 2A the budget is roughly 30mΩ to stay
inside the Pi's 5V tolerance.

Whichever switches it, the pass element's body diode conducts
load-to-supply, exactly as the non-reverse-blocking load switches do —
which is what makes "the user's own PSU is still plugged in" a runtime
check rather than a documented warning.

**Current sense** is nearly free once the whole Pi rail passes through one
point: an INA226 on I2C, or a shunt into a fixture ADC channel. It buys
assertions that are otherwise impossible — idle versus `wfe`-parked versus
four-cores-spinning current, i.e. *did the code actually park the cores*;
power-domain control taking effect rather than the mailbox call merely
returning success; and a regression guard on the boot path's power
behaviour.

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
**1:1 shadow of the whole header** fits with roughly 20 left for
housekeeping: load-switch enable and FAULT, the 3V3-rail sense, INA226
I2C, the `RUN` open-drain, analog audio in, marker pins, and an
enable/fault pair per USB VBUS switch.

This is what 26 pins cannot do. The union of every bus under test is only
17 pins — I2C1 (2/3), SPI0 (7–11), PWM (12/13/18/19), PCM (18–21), aux
SPI1 (16–21), UART0 and mini-UART (14/15) — so a Pico-class fixture covers
the bus tests perfectly well. What it cannot cover is a sweep of the GPIO
driver across the whole header, which would have to split into halves.
Shadowing everything at once also means adding a pin to a future test is a
firmware change rather than a respin.

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

## Multi-board rack

Switching the device under test by hand means unmating the HAT and
unplugging Ethernet, HDMI, USB, audio and the camera ribbon — six
connections, two of them fragile, and a human standing at the bench for
every cell of the matrix. That defeats unattended running for exactly the
runs that take longest. So: **one HAT per board, permanently mated, never
touched again.**

### The four boards

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
omitted as a *full* rig for the same reason — BCM2710A1 is the same die as
BCM2837 — but it is the cheapest board in the set to support, since it
needs no HAL work at all and shares the Zero's no-hub USB topology. It
earns a smoke rig (a load switch and a serial adapter, no HAT) whenever
widening the matrix is worth ten dollars.

Note that the Zero's unique USB coverage is gated behind the ARMv6 port,
which is the most expensive port in the set. If that coverage is wanted
before ARMv6 lands, a Zero 2 W provides it with existing drivers.

### Only one board is ever live

The HAT owns each board's 5V, so exactly one board is powered at a time.
That is what makes a rack affordable: shared infrastructure needs no
active multiplexing, only simultaneous connection.

**Per board, permanently mated (~$60):** HAT, PICO2-XL, VBUS switch board,
USB device set, an Ethernet cable to the shared switch, HDMI cable to the
shared switch. The analog audio path needs no sharing at all, since it
terminates at that board's own fixture ADC — and on Zero W and Pi 5 it does
not apply, as neither has a 3.5 mm jack.

**Shared, no switching needed:** the Ethernet switch, the Wi-Fi AP and the
Bluetooth dongle. Only the powered board talks.

**Shared through an auto-switch:** HDMI. A 5-input auto-switching HDMI
switch follows whichever input carries an active signal, so with one board
live the selection needs no control channel.

**Tier-gated rather than duplicated:** the camera. One 15-pin and one
22-pin CSI ribbon, and the ribbons will not survive repeated insertion, so
the camera is a declared capability on whichever board holds it and skips
elsewhere.

### HDMI: three things to get right

- **Four cable types** — mini for the Zero, full-size for Pi 3, micro for
  Pi 4 and Pi 5. Plugged once each, but they have to be bought
  deliberately.
- **Characterize the switch's EDID once and treat it as the golden
  value.** Auto-switches differ: some pass the sink's EDID through, some
  present their own. Either is acceptable *if it is stable* — a switch
  presenting its own EDID is arguably better, being independent of which
  capture stick is fitted — but it has to be recorded, because the display
  tests assert against it.
- **Allow settling, then verify before asserting.** Switch lock takes a
  moment, and a board that is powered but has not brought up HDMI presents
  no signal at all. Confirm the capture device reports the expected mode
  before making any claim about pixels. Power the switch from the
  **always-on** rail, never a switched one.

### USB devices are per board, unavoidably

VBUS comes from the Pi's own port, so a device dies with its board and
cannot be shared without a high-speed mux — which is exactly the signal
integrity risk avoided by keeping USB off the HAT. Three switched ports
each on Pi 3, Pi 4 and Pi 5 (keyboard, flash drive, small hub), around
nine VBUS switch modules in total.

**The Zero is the exception**: one micro-USB OTG port, so it gets one
device *or* a hub, not both. Putting a hub on it would destroy the
direct-path coverage that is the reason to have a Zero at all, so it gets
a single switched device.

### Two consequences worth planning for

**The orchestrator's USB port budget binds before money does.** Four
fixtures present eight CDC interfaces between them, plus the capture stick,
audio dongle, Bluetooth dongle and Wi-Fi AP — around nine devices. Budget
a powered hub, and prefer smoke rigs (one plain serial adapter each) for
boards that do not need a fixture.

**Pi 5 needs its power budget checked.** It draws more than the 4A this
design budgets, and its PMIC and power button introduce soft-off states
that a plain load switch does not model.

## Phases

Staged so nothing is ever blocked on a PCB, and so PCB spend follows
driver support rather than leading it.

- **Phase 0** — perfboard or breadboard with an RP2040 board already on
  hand, wired for one bus group at a time. Proves the runner design and
  the self-reporting protocol with no new hardware.
- **Phase 1** — HAT rev A: PICO2-XL socketed, switched 5V with the load
  switch and 3V3 sense, full-header pin shadowing through series
  resistors, ID EEPROM, the real-device complement, analog audio path,
  marker-pin header, recovery console header, optional `RUN` header, plus
  the USB VBUS switch satellite board and its ribbon. Hand-solderable
  throughout — 0805 passives, a pre-built module rather than a bare QFN.
- **Phase 2** — respin: the hardware backfeed interlock, whatever rev A
  gets wrong, and possibly absorbing the QFN-80 onto the HAT using
  Olimex's published design.
- **Phase 3** — rack build-out: the HDMI auto-switch, the Ethernet switch,
  a powered hub, and a HAT per board. Build the **Pi 3 and Pi 4 HATs
  first**, since those are the only boards whose peripheral drivers exist
  today; the Pi 5 and Zero W HATs wait on the RP1 and ARMv6 ports
  respectively. Rev A therefore gets validated on two boards before the
  design is committed to four.

## Cross-checks worth keeping

**Assert from both ends wherever the orchestrator can see the same event.**
The device printing its own success is not evidence: a pcap catches bad
checksums, a missing ARP and wrong TCP window behaviour that are invisible
from the device's point of view, `rpi-loader`'s `sd-read` checks from
outside what the Pi's write path claimed to write, and the 3V3 rail sense
plus a re-`HELLO` proves a watchdog reset really happened rather than the
code merely reaching the line.
