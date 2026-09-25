# Changelog

Notable changes to `rpi-kickstart`, in the format of
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This crate
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **The global heap**, behind a `heap` feature: `heap::init` sizes the
  region with `rpi_hal::mem::heap_region` and hands it to an
  `embedded-alloc` TLSF allocator declared here as the
  `#[global_allocator]`. The bounds come from the firmware rather than
  being hardcoded, so one image is correct whatever `gpu_mem` the board
  is set to.

  `init` is safe, which it could not be without the guard it carries:
  `TlsfHeap::init` is `unsafe` because a second call hands the allocator
  a region it has already given parts of away, and that is silent
  corruption rather than anything reported. A second call returns
  `Error::AlreadyInitialized`.

  Like `entropy`, this is a program-wide decision — a binary has exactly
  one allocator — so it is off by default and a board wanting its own
  leaves it alone.

  It also implies `rpi-hal/rt`, the one place this crate asks for a boot
  sequence. Not a convenience: `rpi_hal::mem` is behind `rt` because the
  region's lower bound is `__bss_end`, a symbol only `rt`'s linker script
  defines.

- **`rpi-hal` raised to 0.7.0**, which is where `mem::heap_region` lives.

- **Hardware entropy and the `getrandom` backend**, behind an `entropy`
  feature: `entropy::fill` for callers who want bytes, and
  `register_custom_getrandom!` so the RustCrypto primitives under
  `rustls` reach the same generator. The `rpi_hal::rng` instance is a
  lazily-built static behind a `critical-section` mutex — constructing
  one arms a warmup discard of 262,144 samples, so a fresh instance per
  call would pay that for every byte of a handshake.

  Enabling it is a program-wide decision, like a `#[global_allocator]`:
  the registration is a symbol the whole binary resolves against. Hence
  off by default, so a board with its own backend simply leaves it alone.

- **Chip features** `bcm2835`, `bcm2837` and `bcm2711`, forwarding to
  `rpi-hal`'s. None is a default, and most boards never name one — an
  application's own `rpi-hal` line already selects a chip and cargo
  unifies it. They are spelled `rpi-hal?/…` so that naming a chip does
  not drag the HAL into a build that only wants `console`.

  Nothing in this repository builds with `--all-features` as a result:
  the chip features are not a set to turn all of, since more than one
  resolves by `rpi-hal`'s precedence rather than failing. The `make`
  recipes and `[package.metadata.docs.rs]` name an explicit set.

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
