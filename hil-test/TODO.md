# TODO — `hil` branch

Open work on the hardware-in-the-loop bench. Pending items only: anything
finished comes out of this file rather than being marked done, since a list
of past work is what the commit history is for.

Design and rationale live in `hil-test/README.md` and
`hil-test/hardware/README.md`; this is only what is left.

## Before the harness board can be designed

Ordered by what a wrong answer would cost. The board commits to numbers and
structure that nothing has measured yet, and the expensive mistakes are the
structural ones — a wrong passive is rework, a wrong topology is a respin.

### The Pi interface, with probes in hand

Four measurements, none needing more than a drilled scrap of FR4, a bench
supply and calipers. The first two can invalidate the approach; the second
two only set values.

- [ ] **Is the Pi 4's underside clear beneath the header pads?** Unlike the
      3B it carries bottom-side components, and a probe field needs ~30 mm of
      clearance below the tails. If anything sits in that footprint the whole
      sit-on-top topology fails on Pi 4 specifically, which is the board the
      matrix most needs filled. Nothing on paper answers this; it wants a
      board and a straight edge.
- [ ] **How far do the pin tails protrude, on each model?** A cup tip needs a
      stub to capture. If a board's tails are trimmed near flush and domed
      over with solder there is nothing to seat on, and the fix is a different
      tip suffix — crown or flat — from the same shelf. Cheap to know before
      40 receptacles are committed, expensive after. Measure with calipers on
      every model the bench will hold, since assembly varies.
- [ ] **Contact resistance at working stroke, on a solder-coated tail.** The
      whole power path waits on this number. Folding 30 mΩ per contact into
      the drop budget already puts 3 A *under* the 4.75 V floor on 1 m of
      18 AWG, so the measured value decides whether 16 AWG and a 5 mΩ shunt
      are sufficient or whether the topology needs rethinking. Four-wire:
      force a known current through the joint, measure the drop separately,
      subtract the probe's own body resistance. Take it at working stroke
      rather than barely touching or fully bottomed, and again after a few
      cycles, since first touch on oxide reads differently.
- [ ] **Spring force at working compression**, one probe onto a kitchen
      scale. Multiply by 40: that is the clamp. Candidates ranged 80–200 gf,
      so the answer is somewhere between 3 and 8 kgf, and it decides whether
      the hold-down is a screwed plate with support along the header line or
      something lighter. Also worth settling **cup versus crown on a square
      tail** while the parts are out — the tails are square and the cup is
      round, so whether it skates under slight lateral offset is worth seeing
      rather than reasoning about.

### Structure the board commits to

- [ ] **The pin map does not exist yet.** The pin budget argues the *count*
      works; nothing assigns fixture GPIO to Pi GPIO. And the shadow cannot be
      an arbitrary permutation: for the fixture to act as an I2C slave on the
      Pi's GPIO2/3, the pins shadowing them must themselves be I2C-capable,
      and likewise SPI0 (Pi 7–11), PCM/I2S (Pi 18–21) and the ADC channels
      the audio and rail sense need, which on the RP2350B live only on
      GPIO40–47. That is a constraint-satisfaction problem across 28 lines
      plus housekeeping, avoiding the six pins already committed to PSRAM, SD
      and the LED — and getting it wrong is a respin, not a rework. It is the
      largest single input the schematic is missing.
- [ ] **Inlet overvoltage is unresolved.** The recommended supply is an ATX
      Molex tail, whose yellow wire is +12 V, and everything behind the inlet
      has a 6 V absolute maximum. A TVS cannot cover this: one that stays off
      at a legitimate 5.25 V clamps around 10 V, above what it is protecting.
      So the choice is a real overvoltage cutoff at the inlet, a connector
      that resists being wired wrong, or accepting the risk deliberately —
      but it should be a decision rather than an omission, and it is upstream
      of the always-on rail so it protects the fixture too.
- [ ] **Repeat the cold boot on every model.** One board proves the mechanism,
      not the fleet: Pi 3B+ and Pi 4 carry a PMIC whose behaviour on a slow 5V
      decay is unknown, and a board that does not come back is the one case
      that costs the "the Pi sits down and that is the entire connection"
      claim.

### Scope measurements, no firmware needed

These set component values that are currently invented. They are bench
measurements rather than bench *tests* — the fixture cannot make them, and
in the audio case physically cannot.

- [ ] **Analog audio at the jack**: amplitude *and* DC offset, playing a
      known tone. Decides whether the input network is bias-only or
      bias-plus-attenuation. Note the ADC is unipolar 0–3.3 V, so an AC
      signal needs re-biasing to mid-rail regardless of amplitude.
- [ ] **When 3V3 crosses 0.3 V**, rather than when it reads zero. Reaching
      zero takes about a second, but the recipe waits on the threshold and the
      tail below it is leakage-limited, so the crossing is the number the
      firmware needs and it may be a fraction of that.

### Only if the assembled switch misbehaves

Not blockers, and deliberately not taken in advance: they characterise a
hard-switched event the finished topology does not produce, since soft-start
is there to suppress it. Cheaper to take with the real switch in place and a
symptom to aim at than as a number nothing is waiting on.

- [ ] **Inrush on restore**, if the rail sags enough to reset the fixture or
      drop a USB port — which un-soft-started hand switching already did once.
      No differential probe needed: put the shunt in the *ground* return and
      probe it single-ended, with every other cable off the board, since each
      one is a parallel return that bypasses it. The bulk capacitance behind
      it comes from a constant-current ramp — `C = I / (dV/dt)`, measured with
      and without the board attached and subtracted, because the supply's own
      output capacitance charges alongside it.

## Fixture capabilities

Each is a `HELLO` capability bit that exists in the vocabulary and is not
claimed. Cases needing them skip with a reason until they are.

- [ ] `GPIO_SHADOW`. The technique is settled — `CONSOLE_DRIVE`/`CONSOLE_PINS`
      shadow GPIO14/15 in both directions, through the series resistor — but
      the bit means the whole header 1:1, which needs the harness board.
      Claiming it for two pins would have the runner stop skipping and start
      failing cases against wires that are not there.
- [ ] `POWER_SWITCH` — with it, `reset_board` stops needing a human and the
      timeout-and-recover loop becomes real. The host has no control command
      for it yet; `conftest.py` fails loudly if a fixture ever claims the bit
      without one.
- [ ] `I2C_SLAVE`, which has a case waiting on it (see "Test cases") and is
      therefore ahead of the rest of this list rather than one of it.
- [ ] `RAIL_SENSE`, `CURRENT_SENSE`, `USB_VBUS_SWITCH`, `SPI_SLAVE`,
      `LOGIC_CAPTURE`, `I2S_CAPTURE`, `AUDIO_ADC`, `RUN_RESET`.
- [ ] **A pull-down per observed line on the harness board.** RP2350 erratum
      E9 means 1 kΩ in series in front of a watched pad is not enough on its
      own, and the board plans 28 such lines. The breadboard uses 10 kΩ
      because that is what was to hand; the errata sheet's bound is 8.2 kΩ,
      so the board should carry 8.2 kΩ or lower rather than inheriting the
      value that happened to work on one board at one temperature.
- [ ] **No test covers the marker wire itself.** `marker_arm()` plus
      `captured` is a level probe, and "the first edge arrives at the
      announced grace and not before" is a continuity check — both were used
      by hand to find E9 and neither exists as a case, so the next broken
      wire costs the same afternoon.

## Test cases

`hil_smoke` and `hil_core` cover 13 assertions across two binaries. The
compatibility matrix has 632 cells still unknown; these are the ones
reachable with no fixture beyond the console.

- [ ] **I2C against a slave that acknowledges and then goes quiet.** Needs
      the fixture's `I2C_SLAVE` role, and is the strongest argument for
      claiming that bit: no real device misbehaves on demand, which is
      exactly why this failure reached a consumer instead of a test. It
      covers three shipped fixes at once — `Error::Timeout` and
      `Error::Incomplete` on the blocking path, and the async path's
      NAK handling, whose one piece of evidence today is a hand-run
      example (a NAK returning in one address phase rather than parking
      forever). Four slave behaviours are worth scripting: ACK the
      address then stop driving; answer a read short; hold SCL past
      `CLKT`; and NAK the address outright.
- [ ] **Stack headroom, on both execution states.** The ceiling that cost
      a consumer a silent hang was 32 KiB on AArch32 against 512 KiB on
      AArch64, and a case reporting `stack::size`/`headroom` would have
      shown the asymmetry immediately. Cheap now that both are a reserved
      1 MiB region: assert the region is the size the linker script says
      and that a deliberately deep frame does not reach `__stack_top`.
- [ ] Multicore bring-up (needs the `multicore` feature, so its own binary).
- [ ] FPU / NEON.
- [ ] `critical-section`.
- [ ] PMU / performance counters.
- [ ] Watchdog, and reset-cause reporting — distinct from `Reboot`, which is
      currently covered only as a side effect of how a case ends.
- [ ] Shutdown / power-off.
- [ ] SD: block 0 signature, multi-block, DMA-backed, 4-bit bus, the
      `embedded-sdmmc` adapter.
- [ ] Framebuffer checksum: draw a known pattern, read it back through the
      mailbox. Catches most display regressions with no capture hardware.
- [ ] `set_clock_rate_hz` — the read side is covered, the write side is not.
      It is the one `⚠️` in the matrix. Changing a clock mid-suite moves the
      timing every other case measures against, so it needs isolating.
- [ ] Relocating `_start`, OTP read, GPIO expander, ARM local timer.

## Runner

- [ ] **`bench.toml`** — the rig inventory. The runner discovers fixture
      capabilities over `HELLO`, and the board under test announces its own
      revision, but nothing describes what the *bench* owns: whether the HDMI
      capture, the audio dongle, the AP and the USB device set are actually
      attached today. Those are the witnesses cases skip on, and right now
      their presence is assumed rather than declared.
- [ ] **Timeout, power-cycle, retry loop.** Per-case timeout exists; the
      recover-and-continue behaviour around it does not, and neither does
      flagging a case that only passes on retry.
- [ ] **HTML report** with per-case artifacts — transcripts now, waveforms
      and frames later.
- [ ] **CI**: a second workflow, nightly plus on-demand by label. Hardware is
      serialised and slow, which is why `ci.yml` has no HIL job today. Note
      that with one harness a run only ever covers the board currently
      clamped in it, so the workflow has to record which model it was —
      from the board's own revision code, not from configuration.

## Compatibility matrix

- [ ] Fill Pi 4 / BCM2711 by running the existing binaries against one. The
      images already build for `bcm2711` in both execution states and have
      never been loaded on the hardware.
- [ ] Split the **Pi 1 / Zero** column if ARMv6 is ever targeted. It spans a
      Pi 1 B+ with Ethernet and an analog jack and a Zero W with neither but
      a radio, so board-level rows in it cannot hold one answer.
