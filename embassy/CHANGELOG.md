# Changelog

Notable changes to `rpi-hal-embassy`, in the format of
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This crate
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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

[0.5.0]: https://github.com/joeferner/rpi-hal-embassy/releases/tag/v0.5.0
[0.4.0]: https://github.com/joeferner/rpi-hal-embassy/releases/tag/v0.4.0
[0.3.0]: https://github.com/joeferner/rpi-hal-embassy/releases/tag/v0.3.0
[0.1.0]: https://github.com/joeferner/rpi-hal-embassy/releases/tag/v0.1.0
