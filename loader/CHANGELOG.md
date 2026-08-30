# Changelog

Notable changes to `rpi-loader`, in the format of
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This project
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

The firmware and the host CLI share one version and ship as one release:
they are two halves of a wire protocol, and a version that identifies
only one of them says nothing useful about compatibility.

## [Unreleased]

### Added

- **`eeprom-write` and `eeprom-read`**: program and dump a serial EEPROM
  on the HAT ID bus (BSC0 on GPIO0/1, `ID_SD`/`ID_SC`), which is where a
  board's identity lives — the HAT specification's image, and whatever a
  design puts beside it. `eeprom-write` takes the `.eep` file `eepmake`
  produces; `eeprom-read` with no `--length` reads the image length out
  of the HAT header first, rather than dumping the whole address space.
  Both take `--address` (default `0x50`) and `--offset`; the write also
  takes `--page-size` (default 32, safe for every part from the 24C32 up).

  The device reads every page back after programming it and fails the
  command on a mismatch, because a write-protected part acknowledges
  every byte and stores none — without the read-back, writing to one
  would report success and change nothing.

  New wire commands `EEPROM_READ` (9) and `EEPROM_WRITE` (10), and error
  codes 7 (I2C transfer failed), 8 (read-back mismatch), 9 (a request
  outside what the device will address) and 10 (the part never answered
  the read-back). A loader
  predating them answers an unknown command byte with `FAIL` and then
  reads the arguments that followed as further commands, answering each
  the same way: the CLI reports the command as failed, and the next
  invocation's handshake clears what is left. Both halves ship as one
  release, so that combination should only ever be a stale flash.

### Fixed

- Terminal mode no longer puts the invoking terminal into raw mode when
  stdin is not a terminal. `crossterm`'s `enable_raw_mode` does not consult
  stdin — on Unix it falls back to opening `/dev/tty` — so redirecting stdin
  did not prevent it, and a caller that stopped the loader with a signal
  never reached the restore, handing back a shell with no echo and `\n` no
  longer implying `\r`. Affected anything driving `boot` or `terminal` from
  a script rather than a keyboard.

## [0.1.0] - 2026-08-12

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

[0.1.0]: https://github.com/joeferner/rpi-loader/releases/tag/v0.1.0
