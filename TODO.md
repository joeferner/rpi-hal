# TODO — `hil` branch

Open work on the hardware-in-the-loop bench. Pending items only: anything
finished comes out of this file rather than being marked done, since a list
of past work is what the commit history is for.

Design and rationale live in `hil-test/README.md` and
`hil-test/hardware/README.md`; this is only what is left.

## Before the HAT can be designed

Ordered by what a wrong answer would cost. The board commits to numbers and
structure that nothing has measured yet, and the expensive mistakes are the
structural ones — a wrong passive is rework, a wrong topology is a respin.

- [ ] **Power switching part choice.** Cutting 5V at the header and restoring
      it a second later cold-boots cleanly, so the topology holds and what is
      left is the part. Neither of its two settings is waiting on a bench
      number: the current limit sits above the published PSU ratings — 2.5 A
      for a Pi 3, 3 A for a Pi 4 — and below what two 5V header pins will
      take, which brackets it at 3–3.5 A; and the soft-start window is wide at
      both ends, since a 500 mA-limited supply boots the board and any ramp at
      all beats none. So make both adjustable on the assembled board — a
      swappable limit resistor, a populated-or-not slew cap, test points
      either side of the shunt and on 3V3 — and verify every pin against the
      datasheet before it becomes a footprint. A wrong value is a tweezer; a
      wrong pinout is a respin.
- [ ] **Repeat the cold boot on every model.** One board proves the mechanism,
      not the fleet: Pi 3B+ and Pi 4 carry a PMIC whose behaviour on a slow 5V
      decay is unknown, and a board that does not come back is the one case
      that costs the "HAT plugs on and that is the entire connection" claim.

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
      the bit means the whole header 1:1, which needs the HAT. Claiming it for
      two pins would have the runner stop skipping and start failing cases
      against wires that are not there.
- [ ] `POWER_SWITCH` — with it, `reset_board` stops needing a human and the
      timeout-and-recover loop becomes real. The host has no control command
      for it yet; `conftest.py` fails loudly if a fixture ever claims the bit
      without one.
- [ ] `I2C_SLAVE`, which has a case waiting on it (see "Test cases") and is
      therefore ahead of the rest of this list rather than one of it.
- [ ] `RAIL_SENSE`, `CURRENT_SENSE`, `USB_VBUS_SWITCH`, `SPI_SLAVE`,
      `LOGIC_CAPTURE`, `I2S_CAPTURE`, `AUDIO_ADC`, `RUN_RESET`.
- [ ] **A pull-down per observed line on the HAT.** RP2350 erratum E9 means
      1 kΩ in series in front of a watched pad is not enough on its own, and
      the HAT plans 28 such lines. The breadboard uses 10 kΩ because that is
      what was to hand; the errata sheet's bound is 8.2 kΩ, so the board
      should carry 8.2 kΩ or lower rather than inheriting the value that
      happened to work on one board at one temperature.
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
      capabilities over `HELLO` but has nothing describing what the *bench*
      owns, which is what a rack needs to address boards by name.
- [ ] **Timeout, power-cycle, retry loop.** Per-case timeout exists; the
      recover-and-continue behaviour around it does not, and neither does
      flagging a case that only passes on retry.
- [ ] **HTML report** with per-case artifacts — transcripts now, waveforms
      and frames later.
- [ ] **CI**: a second workflow, nightly plus on-demand by label. Hardware is
      serialised and slow, which is why `ci.yml` has no HIL job today.
- [ ] **Multi-board rack**: per-board fixtures, shared HDMI auto-switch and
      Ethernet switch, `--fixture-serial` addressing.

## Compatibility matrix

- [ ] Fill Pi 4 / BCM2711 by running the existing binaries against one. The
      images already build for `bcm2711` in both execution states and have
      never been loaded on the hardware.
- [ ] Split the **Pi 1 / Zero** column if ARMv6 is ever targeted. It spans a
      Pi 1 B+ with Ethernet and an analog jack and a Zero W with neither but
      a radio, so board-level rows in it cannot hold one answer.
