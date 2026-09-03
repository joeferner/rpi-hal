# Changelog

Notable changes to `rpi-loader-ota`, in the format of
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This package
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Its own file, and its own versions. The `rpi-loader` CLI and the loader
firmware beside it ship as one release because they are two halves of a
wire protocol; this is a library, its consumers are firmware projects in
other repositories, and tying it to that version would bump their
dependency every time a command-line flag was renamed.

## [0.2.0] - 2026-09-02

### Added

- **`apply`**, behind the feature of the same name: the other half of an
  update. It takes a validated bundle and a `resident-fat` volume and
  writes every entry where its path says, in an order the crate imposes
  rather than one the bundle chooses — ordinary files, then Raspberry Pi
  firmware, then `config.txt`, then the kernel. The kernel is last
  because while a board has one boot image that write *is* the commit, so
  everything that could fail has to have failed already.

  Nested destinations are created as needed, since a bundle can carry a
  path and `write_file` resolves a parent rather than making one.

- **Entries the card already holds are read and not rewritten.** The same
  function answers both halves of the question — before a write it
  decides whether to write at all, and after one it *is* the
  verification — so a skipped entry is checked exactly as strictly as a
  written one. A bundle carrying the Raspberry Pi firmware carries about
  3 MB of it, and that changes roughly once a year; on hardware, applying
  an unchanged bundle now costs 1055 ms and **no card writes at all**,
  against 3724 ms to write the same thing in full.

- **`Progress`**, which is how timing stays with the caller. Every method
  defaults to doing nothing and `()` implements the whole trait, so a
  caller names only what it wants. The boundaries separate the write from
  the read-back deliberately: those are not the same operation and do not
  have the same fix, so one figure covering both would hide which of them
  an improvement had touched.

- **`Checksum`**, the streaming form of `checksum`, for checking a file
  already on a card against a fixed scratch buffer rather than a second
  copy of it in memory.

### Changed

- `tests/` is no longer published. The suite builds its FAT32 volumes
  with `mkfs.vfat` and judges them with `fsck.vfat`, so it fails rather
  than skips without `dosfstools` — a suite that cannot run from the
  tarball it ships in says nothing about the crate.

## [0.1.0] - 2026-09-01

First release.

### Added

- **The bundle container**, version 2 of the format: a list of
  (path, role, bytes) under an IEEE CRC-32, encoded on a host and
  validated on a device by the same code. An update can therefore replace
  anything on a boot partition — a settings file, a firmware blob, a
  certificate, `config.txt`, the Raspberry Pi firmware itself — rather
  than a kernel and one hardcoded directory of assets.

- **`Bundle::parse`, which allocates nothing.** It validates in place and
  borrows, so a device holds one buffer — the upload as it arrived — and
  nothing else. `encode`, behind the default `alloc` feature, is the
  other direction.

- **Roles.** An entry is a `File`, the `Kernel`, `Firmware` or `Config`,
  and the installer decides what that means. Naming the boot image by
  role rather than by position or filename is what lets the device choose
  its own destination, which matters to anything running two kernel
  slots.

- **Rules enforced identically in both directions**, so a packer cannot
  build what a device would reject: no absolute paths, `..` components,
  empty components or backslashes; no duplicate destinations; at most one
  kernel and one `config.txt`; and a `start*.elf` only alongside its
  matching `fixup*.dat`, since the two are released together and a
  mismatched pair does not boot.

- **`checksum`**, so a device checksums a file already on its card the
  same way the packer checksummed the entry, and can skip rewriting one
  whose bytes have not changed.

[0.2.0]: https://github.com/joeferner/rpi-loader/releases/tag/ota-v0.2.0
[0.1.0]: https://github.com/joeferner/rpi-loader/releases/tag/ota-v0.1.0
