---
title: rpi-hal-embassy
description: Embassy support for bare-metal Raspberry Pi, on top of rpi-hal.
---

[Embassy](https://embassy.dev) support for bare-metal Raspberry Pi.
Embassy's executor is portable; what it needs per platform is a time
driver and an architecture-specific idle/wake path. This crate supplies
both:

- an `embassy-time` driver over the BCM System Timer, and
- a thread-mode executor that behaves the same on ARMv6, ARMv7-A and
  AArch64, with per-core executors on cores 1 to 3.

It also carries the `embassy-net` drivers: USB Ethernet (the LAN9514 and
LAN7800) and the on-board Wi-Fi radio.

| Chip feature | Boards |
| --- | --- |
| `bcm2837` (default) | Pi 2, Pi 3 |
| `bcm2835` | Pi 1, Pi Zero |

Not the Pi 4 or the Pi 5 yet: the time driver is interrupt-driven, and
rpi-hal has no interrupt controller for either.

Its version tracks rpi-hal's, so matching numbers go together.

- **API reference:** [docs.rs/rpi-hal-embassy](https://docs.rs/rpi-hal-embassy)
- **README:** [embassy/README.md](https://github.com/joeferner/rpi-hal/blob/main/embassy/README.md)
- **Changes:** [embassy/CHANGELOG.md](https://github.com/joeferner/rpi-hal/blob/main/embassy/CHANGELOG.md)
- **crates.io:** [rpi-hal-embassy](https://crates.io/crates/rpi-hal-embassy)
