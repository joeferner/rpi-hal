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
| `bcm2712` | BCM2712 | Pi 5 | `aarch64-unknown-none-softfloat` (from 0.10.0) |

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

## Pi 5

**BCM2712, Cortex-A76. Preliminary**, from rpi-hal 0.10.0. 64-bit only,
since the Pi 5's firmware starts nothing else.

Most of the Pi 5's I/O moved off the SoC onto a separate chip, RP1,
reached over PCIe, so its peripherals are new hardware rather than the
older blocks at a new address. The `bcm2712` feature therefore builds
only what has been brought up on it so far, and leaves the rest of the
crate out. Working:

- boot, the MMU and the caches;
- the System Timer;
- GPIO on the 40-pin header, through RP1;
- the ACT LED, on the SoC's own always-on GPIO block;
- the UART console on GPIO14/15, RP1's UART0;
- the VideoCore mailbox;
- the SD card, reads and writes, through both FAT layers;
- I2C on the header's GPIO2/3 and the HAT ID bus's GPIO0/1, through RP1's
  own controllers.

These keep the API they have on the other boards, so the `blink`,
`uart_hello`, SD card and I2C examples build for the Pi 5 unchanged. There is
no interrupt controller support, second core, DMA or USB here yet. The
loader runs on it (from 0.6.0); Embassy and the project template don't
support the Pi 5 yet.

The Pi 5 boots from firmware in its on-board EEPROM, so its card holds no
firmware files, just `kernel8.img` and a `config.txt` with `os_check=0`,
`kernel_address=0x80000` and `pciex4_reset=0`. The last one keeps RP1
reachable; without it every header pin is dead.

## Planned

- **Pi Zero 2 W** (RP3A0, the Pi 3's BCM2837 in a smaller package).

It doesn't run rpi-hal yet.
