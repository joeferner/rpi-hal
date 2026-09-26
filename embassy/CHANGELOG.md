# Changelog

Notable changes to `rpi-hal-embassy`, in the format of
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This crate
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **`ethernet`, one `embassy-net` adapter for both USB-Ethernet chips.**
  Generic over `rpi-hal`'s `usb::ethernet::EthernetAsync`, so the same
  adapter serves a Pi 2B/3B's LAN9514 and a 3B+'s LAN7800 — which share
  nothing below that trait. A 3B+ previously had no `embassy-net`
  interface at all.

  `lan9514` is now that module under its old name: type aliases and a
  one-line forward, so a board naming its chip keeps compiling.

  The queues stay sized by a fixed `MTU` of 1514 rather than each driver's
  own, because `ch::State` takes it as a const generic and `E::MTU` is not
  usable in that position on stable Rust. `new` asserts the two agree, so
  a future chip with a different frame size fails loudly instead of
  quietly truncating. That shared value is also what would let one
  interface replace another under a live stack later.

- **`ethernet::attach` and `wifi::attach`**, which put a chip behind a
  queue pair the *caller* built, and `channel` — `embassy-net-driver-channel`
  re-exported so a caller can build one.

  `new` is still there and is what a board with one fixed interface wants.
  `attach` is for a board with more than one that picks between them:
  Ethernet if a cable is in, Wi-Fi otherwise. There the queue pair has to
  outlive any particular choice, so it is created once where the choosing
  happens and whichever interface wins is attached to it.

  The queue pair is where the stack's world ends — `embassy_net::Stack`
  holds its sockets and buffers on the `Device` side, everything
  chip-specific is on the other, and both adapters hand over the identical
  `ch::Device<'_, 1514>`. A board that owns the pair can therefore keep one
  stack across a change of interface; one that lets each adapter build its
  own would have to tear the stack down and lose every socket with it.

### Changed

- **The adapter brings the chip up itself, so `new` takes an *unstarted*
  driver and an `EthernetConfig`.** Breaking: callers used to call `start`
  first and pass a `mac`.

  This exists to make a whole class of bug unreachable. A driver may
  configure the chip's answer to an empty bulk IN differently between its
  blocking and awaited bring-ups — parking a receive needs a NAK, polling
  one needs the opposite — so starting it with `Ethernet::start` and then
  running it here was neither a compile error nor a failure. It was a
  receive that never parks, and a runner spinning the executor for the
  life of the interface while every frame still arrived. Owning the
  bring-up means the two can no longer disagree.

  `EthernetConfig` carries what the caller used to set on the driver
  beforehand and can no longer reach: the MAC, and whether to pass all
  multicast. That second one is worth reading before leaving at its
  default — the chip drops multicast before the stack sees it, DHCP is
  broadcast so the interface looks entirely healthy, and mDNS is what
  silently never works.

  It is `EthernetConfig` and not `Config` because every consumer of this
  adapter also imports `embassy_net::Config`.

  A bring-up that fails parks with the link reported down rather than
  retrying: retrying needs a delay, and `embassy-time` is deliberately
  kept off this path. `ethernet::start_error` says why it failed.

## [0.7.0] - 2026-09-25

### Added

- **An `embassy-net` adapter over `rpi-hal`'s Wi-Fi driver**, behind a
  `wifi` feature: `wifi::new` returns the queue-pair `WifiDriver` the
  stack takes and a `WifiRunner` to spawn, the same shape the LAN9514
  adapter has and for the same reason — `embassy_net_driver::Driver`'s
  methods are synchronous, so the bus work has to live on the far side of
  a channel.

  Its runner polls where the LAN9514's awaits an interrupt. Every call
  into `rpi-hal`'s Wi-Fi driver is blocking, so there is nothing for it to
  await; it drains the chip, hands over one queued frame and sleeps a
  millisecond. Receiving comes first deliberately — the firmware advances
  its transmit credit window in the headers of received frames, so a
  runner favouring transmit would starve itself of the credit it needs.

  `wifi::rx_stats`/`tx_stats` report what each direction has managed,
  including the last error in full. They are counters rather than log
  lines because the failure this most wants watching — a malformed SDPCM
  header, which the driver resynchronizes from — arrives in floods, and
  printing each one costs more than the receive it reports on.

  The runner also watches the association and, given a
  `WifiRunner::reconnecting(Reconnect { .. })`, puts it back on its own: an
  access point that reboots overnight costs a board a minute of downtime
  rather than a reboot. It has to live here because the runner owns the
  chip from the moment the stack is built — every call into `rpi-hal`'s
  driver takes it by `&mut`, and there is one.

  What it watches is `Wifi::bssid`, the firmware's own answer, and only
  after the radio has received nothing for ten seconds: a probe is a
  control command, and `rpi-hal`'s control path drops data frames that
  arrive while it waits for the reply, so a link carrying anything is
  never disturbed by one. The rejoin is `start_join` and then polling
  rather than `join_wpa2`, which blocks for up to fifteen seconds — on an
  executor that is every other task on the core stopped.

  `wifi::associated` is the current state and `wifi::link_stats` counts
  what it has cost. The stack is told the link is down for as long as it
  is, so DHCP runs again on the far side: the network a board comes back
  on to need not be the one it left.

  `examples/embassy_net_wifi.rs` is the end-to-end path: firmware off the
  card, join, DHCP, and TCP echo on port 7, rejoining by itself.

  The `wifi` feature deliberately does not imply `embassy-net-driver`.
  The two adapters share only `embassy-net-driver-channel`: this one
  wants `embassy-time` for its ticker and has no use for `rpi-hal/async`,
  and that one is the other way round.

- **`irq::dispatch`, which services every interrupt source this crate and
  `rpi-hal` know about**, and an `irq-dispatch` feature that installs it
  as `__irq_handler` outright.

  `rpi-hal`'s default `__irq_handler` is a weak `bx lr`, so an
  application that never defines one links and boots. The first
  `embassy-time` deadline then fires, nothing acknowledges the System
  Timer's Compare 1 match, the controller goes on asserting, and the core
  re-enters the handler forever. Every task stops at that instant, with no
  error anywhere and a symptom indistinguishable from a hang in whatever
  ran last. Every example here carried the handler by hand, and so does
  every application built on this crate.

  `dispatch` is the composable half: the System Timer always, and under
  the new `async` feature the interrupt-driven drivers — USB, GPIO edges,
  I2C, UART, SD. Each is gated on its own pending bit, and each entry
  point is harmless when there is nothing to do, so it is safe to call
  from a handler that goes on to check sources of its own. That needs no
  feature.

  `irq-dispatch` is for an application with no sources of its own. Off by
  default, and not for cost — it is a handful of `if`s. It is that the
  symbol is a definition rather than a hook, so an application that has
  its own and enables this gets a duplicate-symbol link error. Leaving it
  off means a plain `cargo add` cannot produce that error out of nowhere;
  opting in is how an application says it has no handler.

- **An `async` feature**, which forwards `rpi-hal/async` and tells
  `dispatch` those drivers are in the build — a `cfg` can only test this
  crate's own features, not a dependency's. `embassy-net-driver` implies
  it as well as `rpi-hal/async`, and the two are different statements: the
  first is what the adapter needs to compile, the second is what wakes the
  USB transfers its runner parks on.

### Changed

- **Requires `rpi-hal` 0.7.0**, which is what makes the Wi-Fi adapter
  above resolvable. It was written against HAL changes that were on that
  crate's main branch and in no release — its coalesced-frame handling and
  `Wifi::bssid`, which the reconnect watch asks — so this repository
  carried a `[patch.crates-io]` pointing at a working copy and did not
  build from a clean checkout. 0.7.0 carries them, so the patch comes out
  and the version moves.

- **The Makefile names the examples it builds** rather than sweeping them
  with `--all-features --examples`, which now fails to link: ten examples
  over, each defines the `__irq_handler` that `irq-dispatch` also defines.
  `examples/embassy_blink.rs` drops its hand-written handler and is built
  with the feature, which is the only place it is exercised — and the
  reason it cannot be part of a sweep. This is the shape `rpi-hal`'s
  Makefile already uses, for the same reason.

## [0.6.0] - 2026-09-20

### Added

- **Pi 1 / Pi Zero support (BCM2835, ARMv6)**, behind a `bcm2835`
  feature forwarded to `rpi-hal`'s. The time driver needed nothing: it is
  the System Timer and the legacy interrupt controller, both of which
  that chip has at a different base. The executor needed one line — the
  pender's `dsb ish`, which has no ARMv6 spelling, there being no
  domain-qualified barriers, and becomes the full-system CP15 operation.

  `make build-armv6` / `examples-armv6` / `clippy-armv6` and
  `scripts/build-example-armv6.sh` cover the target, which is the only
  thing here that needs nightly: `armv6-none-eabi` is tier 3, so
  `-Z build-std` has to compile `core` for it.

### Changed

- **Requires `rpi-hal` 0.6.0**, for the `bcm2835` feature.

- **The chip is now a feature of this crate** (`bcm2837` by default,
  forwarding to `rpi-hal`'s) rather than a hardcoded entry on the
  `rpi-hal` dependency line. It has to be: `rpi-hal` prefers `bcm2837`
  over `bcm2835` when both are enabled, so a Pi Zero build that could not
  turn `bcm2837` off would silently compile the Pi 3's peripheral base
  into a Pi Zero binary. Existing users are unaffected — a plain build is
  still the Pi 2/3 one — unless they were passing `default-features =
  false`, which now also turns the chip off and needs `features =
  ["bcm2837"]` adding back.

## [0.5.0] - 2026-09-04

### Changed

- **Requires `rpi-hal` 0.5.0** — breaking, since the two crates share
  types and a consumer has to move with it.

  That release fixes a bug worth upgrading for on its own: the LAN9514's
  MAC was left in half duplex, so it discarded frames whenever the
  interface transmitted and received at the same time. Nothing in this
  crate could see it — the adapter handed every frame over successfully —
  and the symptom was a peer waiting out a retransmission timeout. On a
  real board it took the median response time for a page of eight files
  from 1.0 s to 13.7 ms.

  The receive loop is ported to that release's `receive_frames_async`,
  which returns an iterator over the frames a transfer carried rather than
  the first one, and drains it. No behavioural change today: with
  `HW_CFG.MEF` clear the chip still sends one frame per transfer, which
  `RxStats::batched` staying at zero confirms on hardware.

### Added

- `lan9514::rx_stats` and `lan9514::tx_stats`, reporting what the two
  frame loops managed since boot: frames attempted and delivered, sends
  the chip refused, receives that came back empty or errored, transfers
  that carried more than one frame, and the last `TransferError` in each
  direction.

  A frame either loop drops is otherwise invisible. The receive loop
  discards a transfer that yields nothing usable and asks again; the
  transmit loop releases the buffer whether or not the send succeeded.
  Both are deliberate — the queue would wedge on the first failure
  otherwise — but between them there was no way to tell a frame that went
  from a frame that did not, and the symptom of either is latency
  somewhere else. Three relaxed atomics per frame, against a USB transfer.

- Two network examples, both printing machine-readable result lines and
  carrying their measured numbers in their comments:

  - `embassy_net_burst` — measures *inbound* loss without transmitting.
    The host sends a burst of marked UDP datagrams, each carrying its
    sequence number and the burst total; the board reports how many
    arrived. Establishes that the receive path handles a sustained 4,000
    frames a second and a 256-frame back-to-back burst without losing
    one. A `STARVE` switch holds interrupts off in bursts, for asking what
    a receive loop that cannot be woken costs.
  - `embassy_picoserve_site` — serves eight files at realistic sizes over
    eight web tasks, with a real application's buffer sizes and timeouts,
    for measuring what a browser actually does to this stack. Its `write`
    timeout is deliberately not `picoserve`'s default: one second is where
    a response is *aborted* mid-body, and a concurrent burst of these
    files crosses it.

## [0.4.0] - 2026-09-02

### Changed

- **Requires `rpi-hal` 0.4.0.** Breaking, and not because anything here
  moved: the two crates share types, so a consumer cannot hold this crate
  at 0.3.0 and the HAL at 0.4.0 — cargo would resolve both and hand the
  runner a `Lan9514` from a different crate version than the one it was
  compiled against. Raising the requirement is what keeps the two in one
  graph, and the version number matches the HAL release it is built
  against, as every release here has since 0.3.0.

  Nothing in this crate's own API changes. The HAL release it moves to is
  breaking for one reason — `rpi_hal::sd::Error` gained a `NoCard`
  variant, which breaks an exhaustive match — and nothing here matches on
  it, so the upgrade is a version requirement and a lockfile.

## [0.3.0] - 2026-08-30

### Added

- A `multicore` feature, for running an executor on cores 1-3 as well as
  core 0, and an `embassy_multicore` example demonstrating one per core.
  The feature adds no code: it selects `rpi-hal`'s cross-core
  `critical-section` implementation, which the driver's shared timer
  queue needs once a second core can reach it.
- An `embassy-net-driver` feature and a `lan9514` module: the
  `embassy-net` adapter over `rpi-hal`'s LAN9514 USB-Ethernet driver,
  moved here from `rpi-hal` (where it was that crate's
  `embassy-net-driver` feature) and rebuilt on
  `embassy-net-driver-channel`.

  `lan9514::new` returns a `Lan9514Driver` for `embassy_net::new` and a
  `Lan9514Runner` the application spawns as a task. The split is what
  makes an async driver possible at all: `embassy_net_driver::Driver`'s
  `receive`/`transmit` are synchronous, so a `Driver` implemented
  directly over the chip has nowhere to await and must do its USB work
  with the blocking methods, holding the executor for the length of every
  transfer. As a queue pair plus a task, the USB work is free to await.

  Two consequences for an application. It must dispatch the USB interrupt
  (`Lic::enable_usb_irq`, and `usb::dwc2::on_irq` from its
  `__irq_handler`) — the runner's transfers complete on that and nothing
  else. And it no longer calls `lan9514::wake_rx` on a ticker, because
  there is nothing left to poll: the receive parks on the bulk endpoint
  and wakes on the interrupt, so the latency-versus-wake-ups interval an
  application used to have to choose is gone.

  The runner takes two USB host channels rather than one, one per
  direction, so a transmit never has to cancel a parked receive.

### Changed

- **This crate's version now tracks `rpi-hal`'s.** 0.2.0 is skipped
  deliberately and will never exist: the two crates are used together and
  share types, so matching numbers say at a glance which HAL a given
  release is for. It also removes the question this release exists to
  answer — 0.1.0 of this crate wanting 0.1.0 of a HAL that had moved twice
  since.
- **Requires `rpi-hal` 0.3.0**, up from 0.1.0, which matters to an
  application more than a dependency bump usually does: this crate's
  published 0.1.0 asks for `rpi-hal` 0.1.0, so taking a newer HAL
  alongside it put *two* versions of the HAL in one graph. That fails as
  `time_driver::init` refusing a `Timer` — "expected
  `rpi_hal::timer::Timer`, found a different `rpi_hal::timer::Timer`" —
  which says nothing about the real cause. Pinning both halves of this
  release to 0.3.0 is what removes it.

  0.3.0 is also a floor rather than a preference: the `lan9514` module
  below is built on `Lan9514::split` and the LAN9514's `_async` methods,
  which no earlier release has.

  Nothing in this crate's own API changes, but the
  two `embassy-net` examples move to 0.2.0's owned USB `Channel`: they
  allocate a channel for the stack with `Dwc2Host::alloc_channel` rather
  than lending the whole controller, and `Lan9514Driver` grew a second
  lifetime for it. A consequence worth noting for anyone copying the
  pattern — the controller has to be widened to `'static` *before*
  `alloc_channel`, since a `Channel` borrows the controller it came from and
  the driver keeps its channel for the life of the program.

## [0.1.0] - 2026-08-12

### Added

- An `embassy-time` driver over the BCM System Timer, with the tick rate
  pinned to the hardware's fixed 1MHz rather than left to the
  application to choose.
- A thread-mode executor that behaves identically on AArch32 and
  AArch64, supplying its own `__pender` and idle loop rather than taking
  an upstream `platform-*` backend — `platform-cortex-ar` covers only
  AArch32, and `platform-spin` never idles the CPU.
- Examples exercising the driver and executor against real peripherals:
  blink, button, `Instant`/`Duration`, async UART echo, an
  `embassy-net` TCP echo server, and a `picoserve` HTTP server.

[0.7.0]: https://github.com/joeferner/rpi-hal-embassy/releases/tag/v0.7.0
[0.6.0]: https://github.com/joeferner/rpi-hal-embassy/releases/tag/v0.6.0
[0.5.0]: https://github.com/joeferner/rpi-hal-embassy/releases/tag/v0.5.0
[0.4.0]: https://github.com/joeferner/rpi-hal-embassy/releases/tag/v0.4.0
[0.3.0]: https://github.com/joeferner/rpi-hal-embassy/releases/tag/v0.3.0
[0.1.0]: https://github.com/joeferner/rpi-hal-embassy/releases/tag/v0.1.0
