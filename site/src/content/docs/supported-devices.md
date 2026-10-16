---
title: Supported devices
description: Which Raspberry Pi boards rpi-hal runs on, and how far along each one is.
---

<!--
To become a compatibility matrix: one row per peripheral or capability,
one column per board.
-->

Every board rpi-hal runs on, and how complete its support is. The
[README](https://github.com/joeferner/rpi-hal/blob/main/hal/README.md)
has the status of each driver.

An application selects the chip with exactly one cargo feature. None is
a default, because there is no sensible default chip:

| Chip feature | Chip | Boards | Targets |
| --- | --- | --- | --- |
| `bcm2837` | BCM2836, BCM2837 | Pi 2, Pi 3 | `armv7a-none-eabi`, `aarch64-unknown-none-softfloat` |
| `bcm2711` | BCM2711 | Pi 4 | `armv7a-none-eabi`, `aarch64-unknown-none-softfloat` |
| `bcm2835` | BCM2835 | Pi 1, Pi Zero, Pi Zero W | `armv6-none-eabi` (nightly) |

## Pi 2 and Pi 3

**BCM2836 (Pi 2 Model B rev 1.1, Cortex-A7) and BCM2837 (Pi 2 rev 1.2
and Pi 3, Cortex-A53).** The main targets, where rpi-hal's drivers are
developed first.

The Pi 2 rev 1.1's Cortex-A7 cores are 32-bit only, so that board runs
`armv7a-none-eabi` builds alone. The Pi 3 runs either.

## Pi 4

**BCM2711, Cortex-A72. Preliminary.** The `bcm2711` feature selects the
relocated peripheral memory map and PAC. Working, in both 32-bit and
64-bit builds:

- boot, GPIO and the System Timer;
- the MMU identity map and mailbox coherency;
- the UART console;
- the SD card, through the BCM2711-specific `EMMC2` controller, and so
  the loader's card commands.

Most other drivers are untested here, and there is no interrupt
controller (GIC-400) support yet, so nothing interrupt-driven runs,
Embassy included. [Issue #29](https://github.com/joeferner/rpi-hal/issues/29)
tracks the bring-up.

## Pi 1 and Pi Zero

**BCM2835, ARM1176JZF-S. Preliminary.** The `bcm2835` feature selects the
chip's peripheral map and PAC. Building for `armv6-none-eabi` selects the
ARMv6 boot code, barriers and MMU programming. Working, on a Pi Zero W:

- boot and GPIO;
- the UART console;
- the System Timer;
- the MMU with the data and instruction caches on, including the
  exclusive monitor that atomics rely on.

Interrupts and the drivers above the SoC basics are untested.

Three features don't exist on this chip at all, rather than being
untested: `multicore` (it has one core), and `generic_timer` and `pmu`,
which rest on ARMv7-only CPU state or on the ARM-local peripheral block
the BCM2835 doesn't have.

`armv6-none-eabi` is a tier-3 target with no precompiled `core`, so this
board needs **nightly** Rust and `-Z build-std`. A project from
[Start a project](../start/) is set up for it already.

## Planned

- **Pi Zero 2 W** (RP3A0, the Pi 3's BCM2837 in a smaller package).
- **Pi 5** (BCM2712, with most of its I/O moved onto the separate RP1
  chip).

Neither runs rpi-hal yet.
