# Changelog

Notable changes to `rpi-loader-ota`, in the format of
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This package
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Its own file, and its own versions. The `rpi-loader` CLI and the loader
firmware beside it ship as one release because they are two halves of a
wire protocol; this is a library, its consumers are firmware projects in
other repositories, and tying it to that version would bump their
dependency every time a command-line flag was renamed.

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

[0.1.0]: https://github.com/joeferner/rpi-loader/releases/tag/ota-v0.1.0
