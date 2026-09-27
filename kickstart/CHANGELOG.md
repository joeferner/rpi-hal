# Changelog

Notable changes to `rpi-kickstart`, in the format of
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This crate
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Reading a settings file**, behind a `config` feature. The file is TOML
  and the schema is the board's own `#[derive(Deserialize)]` struct, so
  there is no key table and no trait to implement. `config::parse` takes
  the bytes off the card and returns the settings plus every key the
  schema did not name — skipped rather than refused, so older firmware
  still boots a card written for newer. `config::value` holds the checks
  TOML cannot make (`label`, `host`, `word`, `url`, `address`,
  `duration`, `degrees`), and `.at(&spanned)` locates a failed one at the
  setting it was run on. Syntax errors, type errors, bad UTF-8 and failed
  checks all come back as one `Problem`, which prints as
  `BOARD.TOML:12:18: …`.

  Pure, and implies nothing — not `console`, not storage — which is what
  makes it the first module here with tests. They run on the host under
  `make test`, which needed the examples' bare-metal dev-dependencies
  moved under `cfg(target_os = "none")`; nothing they link builds for an
  OS target.

- **`net`, which gets the board onto the network whichever way it can.**
  A board states what it is willing to use and in what order —
  `connection = ["ethernet", "wifi"]` — and `net::discover` takes the
  first that is actually there. Nothing above it learns which it got.

  Ethernet covers both chips: a Pi 2B/3B's LAN9514 and a 3B+'s LAN7800.
  Each driver declines a device that is not its own, so offering every
  enumerated device to both is how the board identifies itself.

  **"Ethernet is not available" is two questions, and both are
  deadlines.** *No chip* — which on a 3B+ cannot be decided quickly,
  because its LAN7800 sits behind two cascaded hubs and attaches seconds
  after power-on, so "absent" and "not here yet" are indistinguishable
  until `chip_timeout_ms` expires. *No cable* — auto-negotiation takes a
  second or three even when one is in, hence `link_timeout_ms`. They are
  separate settings because they fail differently: a short chip timeout
  misses a 3B+ entirely, a short link timeout loses to a slow switch port.

  Answering the second question costs a spare chip reset. The PHY reports
  nothing until it has been powered and released, so `discover` starts the
  chip to ask — and `rpi-hal-embassy`'s adapter, which owns bring-up for
  the network stack, then resets it and starts it its own way. A fraction
  of a second against having no way to ask at all.

  **Wi-Fi is the second interface**, behind a `wifi` feature as Ethernet is
  behind `ethernet`. A board that lists both gets Ethernet when a cable is
  in and the radio when it is not, from one image — which is what makes a
  Pi 3B, a 3B+ and a Zero W run the same binary.

  The radio's bring-up is a closure the board supplies
  (`Hardware::wifi`) rather than something this module does, because every
  step of it is a board's own decision: where the firmware image, the nvram
  and the regulatory blob live, and how the credentials are spelled.
  Handing the EMMC controller from the SD driver to the SDIO one gives up
  the card slot for the rest of the boot, which is not a thing to do on a
  board's behalf. What `net` does own is *when* — the closure runs at most
  once, and only if the walk got that far, so a board with a cable in never
  pays the several hundred kilobytes of card reading a radio costs.

  This is also why `Interface` carries its own MAC. The Ethernet chips are
  programmed with the address the firmware mailbox reports; the radio has
  one of its own. A stack built with the wrong one associates and then
  answers no ARP.

  **It picks once, at startup.** A cable pulled an hour later does not
  move the board to Wi-Fi; that needs swapping which runner is attached to
  a live queue pair, which is why `rpi-hal-embassy` now lets a board own
  that pair, and is not built.

  Enabling `net` without `ethernet` or `wifi` is refused at compile time.
  It would build a priority walk with nothing to find, and report that
  only at run time, one entry at a time.

- **`mdns`**, an mDNS responder for one name, and the `mdns` example that
  puts it on a real link.

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
