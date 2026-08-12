# Changelog

Notable changes to `rpi-loader`, in the format of
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This project
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

The firmware and the host CLI share one version and ship as one release:
they are two halves of a wire protocol, and a version that identifies
only one of them says nothing useful about compatibility.

## [Unreleased]

### Added

- The loader firmware: a self-relocating UART command agent for the
  Raspberry Pi 2, 3 and 4 that stays resident and services commands, so
  the SD card is rewritten only to re-flash the loader itself. Builds for
  AArch32 (`kernel7.img`) and AArch64 (`kernel8.img`), for BCM2837 and
  BCM2711.
- `mem-write`, `exec`, and `boot` for getting an image into memory and
  running it, with CRC-checked chunks and per-chunk retries.
- `sd-list`, `sd-read`, `sd-write`, `sd-delete`, and `sd-mkdir` for
  working with the SD card's FAT boot partition over the same link.
- A bidirectional `terminal`, so a kernel that prompts for input can be
  driven. Ctrl-] exits; every other key, Ctrl-C included, reaches the
  device.
- `list`, which reports the host's USB serial ports without opening one.
- The host CLI in Rust, published to crates.io as `rpi-loader`.
