---
title: rpi-loader
description: A resident UART agent for the Raspberry Pi, its host CLI, and the OTA bundle format.
---

A self-relocating UART command agent. Flash it once from an SD card;
after that it stays resident, and the host drives it over the serial
cable: upload a build to memory and jump to it, or read, write and list
files on the SD card, instead of rewriting the card for every build.

```sh
cargo install rpi-loader
rpi-loader --device /dev/ttyUSB0 boot --load-addr 0x8000 target/kernel7.img
```

| Image | Boards | Kernel file |
| --- | --- | --- |
| `bcm2837` | Pi 2, Pi 3 | `kernel7.img` (32-bit), `kernel8.img` (64-bit; not a Pi 2 v1.1, whose cores are 32-bit only) |
| `bcm2711` | Pi 4 | `kernel7.img` (32-bit), `kernel8.img` (64-bit) |
| `bcm2835` | Pi 1, Pi Zero | `kernel.img` |
| `bcm2712` | Pi 5 (preliminary, from 0.6.0) | `kernel8.img` (64-bit only) |

[Start a project](../../start/) puts the right image on the card under
the right name. Releases carry all six images and their checksums; the
firmware loads only the bare names, so an image downloaded by hand has to
be renamed on the way to the card.

Two crates live here:

- **`rpi-loader`:** the host CLI. Its version moves with the firmware
  images, as the two halves of one wire protocol.
- **`rpi-loader-ota`:** the over-the-air update bundle, packed on a host
  and validated and installed on the device. rpi-kickstart's `ota`
  feature uses it.

- **README:** [loader/README.md](https://github.com/joeferner/rpi-hal/blob/main/loader/README.md)
- **OTA API reference:** [docs.rs/rpi-loader-ota](https://docs.rs/rpi-loader-ota)
- **Changes:** [loader/CHANGELOG.md](https://github.com/joeferner/rpi-hal/blob/main/loader/CHANGELOG.md)
- **crates.io:** [rpi-loader](https://crates.io/crates/rpi-loader), [rpi-loader-ota](https://crates.io/crates/rpi-loader-ota)
