# Changelog

Notable changes to `rpi-kickstart`, in the format of
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This crate
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **The console sink and `logln!`**, behind a `console` feature:
  `console::init` installs one sink for the whole image, and `logln!`
  reaches it from anywhere, stamping each line with the uptime. The sink
  is behind a `critical-section` mutex, so a log from an interrupt
  handler cannot interleave with one from a task and shred a line.

  Two seams that the copies this was taken from did not have. The sink is
  a `&'static mut (dyn Write + Send)` rather than a `rpi_hal::uart::Uart`:
  that keeps the module clear of the PAC — and so of chip selection — for
  something whose whole job is `write_str`, and it lets a board that has
  given the PL011 to a Bluetooth controller log to the mini UART, or a
  board with no cable attached log to a RAM ring. The clock is passed in
  as a `fn() -> u64` rather than chosen, because choosing would make the
  most basic module here impose the heaviest dependency in the crate; a
  board on Embassy passes `|| Instant::now().as_micros()`.

  Logging before `init` is discarded rather than a fault — modules that
  run before bring-up reaches the console need that to be true, a fault
  reporter most of all.

  `examples/console.rs` demonstrates all three.

- **The repository skeleton**: manifest, both bare-metal targets, the
  `make` check set, CI and the release workflow.

  `examples/hello.rs` is a whole image that boots on `rpi-hal`'s `rt`,
  brings up the UART and prints a rising heartbeat. Nothing in the
  library yet, so what it proves is everything around one — the pinned
  toolchain, `armv7a-none-eabi` and `aarch64-unknown-none-softfloat`,
  `rpi-hal`'s `rpi-link.x` on the linker search path, and both load
  addresses. A count that keeps rising is the difference between a board
  that booted and a board that booted and then faulted.
