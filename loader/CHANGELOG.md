# Changelog

Notable changes to `rpi-loader`, in the format of
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This project
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

The firmware and the host CLI share one version and ship as one release:
they are two halves of a wire protocol, and a version that identifies
only one of them says nothing useful about compatibility.

The `rpi-loader-ota` library in `ota/` is not part of that pair and keeps
its own history in [`ota/CHANGELOG.md`](ota/CHANGELOG.md). Its consumers
are firmware projects in other repositories, and a renamed command-line
flag here is no reason to bump their dependency.

## [Unreleased]

### Added

- **`bundle`**, a subcommand that packs an over-the-air update bundle
  from a `bundle.toml` and, with `--upload <url>`, posts it to a running
  board. The first subcommand that opens no serial port: the container is
  `rpi-loader-ota`, shared with the firmware that installs one, and this
  is the half that builds them. A manifest rather than arguments because
  what goes into a bundle is a property of the project — a kernel, whole
  directories, firmware blobs and `config.txt`, each with a destination —
  and arguments carrying that would put a card's layout in a `Makefile`
  where nothing can check it.

  Uploading is plain HTTP with no TLS anywhere in the dependency tree;
  the endpoint is a board on a local network.

## [0.2.0] - 2026-08-30

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
  the read-back).

### Changed

- **The handshake's protocol version is now 2**, in both halves. It is a
  bump for an addition rather than a change, which the version byte
  cannot express — but the mismatch it names is the ordinary one for this
  project, not an exotic one: the loader image is flashed once and left on
  the card for months while the CLI is reinstalled from crates.io
  whenever, so a newer CLI meeting an older loader is the expected way for
  the two to drift.

  Without it, `eeprom-write` against a 0.1.0 loader comes back as
  `error code 0` — the unknown command byte answered with `FAIL` — which
  reads as a fault on the I2C bus rather than as a stale image on the SD
  card. With it, the handshake says `device protocol version 1, expected
  2` first. A mismatch stays a warning, so an older CLI keeps driving a
  newer loader for every command they share.

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

[0.2.0]: https://github.com/joeferner/rpi-loader/releases/tag/v0.2.0
[0.1.0]: https://github.com/joeferner/rpi-loader/releases/tag/v0.1.0
