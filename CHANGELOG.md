# Changelog

Notable changes to `rpi-hal`, in the format of
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This crate
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Async LAN9514** (`async` feature): `send_frame_async`,
  `receive_frame_async`, `start_async`, `is_link_up_async` and the
  register accessors behind them, as twins of the blocking methods.
  `receive_frame_async` differs from its twin in more than spelling: it
  leaves the bulk IN parked on an empty receive FIFO rather than first
  asking `RX_FIFO_INF` whether a frame is waiting, so the receive becomes
  interrupt-driven instead of polled. The blocking method cannot do that
  — the DWC2 retries a NAK'd bulk transfer in hardware without halting
  the channel, so it would spin out its whole transfer timeout on every
  idle poll — which is why the pre-check stays there and only there.
- `usb::lan9514::Lan9514::split` (`async` feature), returning
  `Lan9514Rx`/`Lan9514Tx`: the two bulk endpoints borrowed apart, so a
  receive can stay parked on one host channel while transmits go out on
  another. Without it a transmit could only happen by cancelling a parked
  receive, which loses any frame the chip was part-way through handing
  over.
- `usb::control::vendor_in_async` / `vendor_out_async`, the vendor
  register access those methods are built on.
- `usb::lan9514::MTU` is now unconditional rather than gated on an
  adapter feature — it is a property of Ethernet and of this chip, and an
  out-of-crate adapter needs the same number.

### Removed

- **The `embassy-net-driver` feature**, with `usb::lan9514::Lan9514Driver`
  and `usb::lan9514::wake_rx`. The `embassy-net` adapter now lives in the
  `rpi-hal-embassy` crate, built on `Lan9514::split` and the async methods
  above, and is a `Driver` plus a runner task rather than a `Driver` that
  does its own USB work. `embassy_net_driver::Driver` is synchronous, so
  an adapter shaped that way could never have awaited anything.

  `wake_rx` goes with it, and that is the point of the exercise: an
  application no longer has to poll the driver on a ticker and guess an
  interval, because there is now a real event to wake on.

  Nothing here affects the `smoltcp` adapter or the blocking frame
  methods. `smoltcp`'s `phy::Device` is synchronous by construction, so
  those stay exactly as they were.

## [0.2.0] - 2026-08-19

### Added

- **VCHIQ** (`vchiq` feature, implies `mmu`): the VideoCore firmware's
  shared-memory message transport — slot ring, service open/close,
  messages, and page-list bulk DMA — as `vchiq::Vchiq`. Polled rather than
  interrupt-driven; see `vchiq::Vchiq::poll`.
- **MMAL** (`mmal` feature, implies `vchiq`): `mmal::Mmal`, a client for
  the firmware's multimedia framework — components, ports, parameters and
  buffer exchange, with buffers moving by `&'static mut [u8]` ownership
  transfer in both directions.
- **Hardware H.264 decode** (`mmal` feature): `video_decode::VideoDecoder`,
  driving the firmware's `ril.video_decode` component. Takes an H.264
  Annex B byte stream in arbitrary chunks and returns whole I420 frames,
  handling the mid-stream format change the decoder announces once it has
  parsed the stream's geometry. `examples/h264_decode.rs` plays a file off
  the SD card on the display, converting each frame to RGB on the ARM.
- `mmu::set_uncached`, which remaps a granule-aligned region of RAM as
  Normal Non-cacheable — what makes a shared-memory protocol with a second
  bus master possible at all, and why `vchiq` implies `mmu`. The `mmu`
  module is public for it.
- `mailbox::Mailbox::vchiq_init`, the property tag that hands the firmware
  the VCHIQ shared region.
- `vchiq::Stats` and `mmal::Stats`: counts of what has crossed each
  interface. A stalled shared-memory exchange reports nothing about
  itself, so comparing what was sent against what came back is what makes
  one diagnosable.
- crates.io version and docs.rs badges in `README.md`, alongside CI.
- **USB host channels as owned handles**: `usb::dwc2::Dwc2Host::alloc_channel`
  hands out a `usb::dwc2::Channel`, and every transfer primitive now lives
  on that rather than on the controller. A channel carries its own DMA
  staging buffer and borrows the controller immutably, so several can be
  outstanding at once and two endpoints can be driven independently —
  where before one `&mut Dwc2Host` and one shared buffer meant one
  transfer at a time, on a channel index every caller hardcoded to 0.
  Exhaustion is reported (`None`, and `EnumerationError::OutOfChannels`)
  rather than queued.
- **Interrupt-driven USB** (`async` feature): `usb::dwc2::asynch`, with an
  `_async` twin of each `Channel` transfer primitive plus
  `Dwc2Host::wait_for_port_change` and `Channel::wait_microframes`, all
  serviced by `usb::dwc2::on_irq` from the application's `__irq_handler`.
  No time crate is involved: the channel-halt interrupt reports
  completion and start-of-frame supplies the microframe scheduling a
  periodic split needs, so the whole path is bus-clocked. An async
  transfer has no timeout of its own — dropping the future imposes one
  and aborts the channel. `examples/usb_irq.rs` drives it with a
  hand-rolled `block_on`, no executor.
- `lic::Lic::enable_usb_irq`/`disable_usb_irq`/`is_usb_pending`, routing
  the DWC2 controller's line to the ARM core.
- `usb::lan9514::Lan9514::from_endpoint`, for a LAN9514 that an external
  host stack has already addressed and configured — the counterpart to
  `from_device` when this crate's `enumerate` isn't the thing walking the
  bus.
- `cpu::core_id`, the calling core's id from `MPIDR`. Deliberately not part
  of `multicore`, which is compiled only behind its own feature: code that
  runs on every core — an interrupt handler, a panic handler naming where
  it died — needs to ask this without opting into the machinery for
  *starting* cores, and `generic_timer` already needed it to address a
  per-core register on a single-core build.

### Changed

- `usb::enumerate` takes `&Dwc2Host` rather than `&mut`, and its callback
  receives a `&mut Channel` in place of the controller. A callback that
  needs a channel outliving enumeration captures the same `&Dwc2Host` and
  allocates its own.
- `usb::lan9514::Lan9514Phy::new` and `Lan9514Driver::new` take an owned
  `Channel` instead of `&mut Dwc2Host`, so the rest of the controller's
  channels stay free while a network stack runs.
- Every `usb::control`, `usb::hub` and `usb::hid` entry point takes
  `&mut Channel` in place of `&mut Dwc2Host`.
- `Dwc2Host::last_channel_interrupt` is now `Channel::last_interrupt`,
  reporting per channel rather than per controller.
- `Dwc2Host::init` leaves `GINTMSK.SOFM` masked. It was set
  unconditionally and harmless only because nothing routed USB to the ARM
  core; now that `Lic::enable_usb_irq` exists, an unmasked 8kHz level
  source with nothing to acknowledge it would be a hang rather than
  merely wasted cycles. The async path unmasks it only while a channel is
  waiting on a microframe.
- A channel start programs `HCINTMSK` to `CHH` alone instead of every
  condition, so "this channel raised `HAINT`" means "it halted". The
  `HCINT` bits themselves still latch, so nothing that reads them at the
  halt — including the split logic's `ACK` check — sees any difference.

### Fixed

- The README's CI badge no longer renders as a broken image on crates.io.
  GitHub refuses to serve its own `actions/workflows/…/badge.svg` to pages
  on another origin — browsers report `ERR_HTTP2_SERVER_REFUSED_STREAM` —
  so the badge worked on GitHub and nowhere else. It now comes from
  shields.io, which serves the same status to both. crates.io renders a
  README once, at publish time, so 0.1.0's page keeps the broken badge and
  the fix is visible only from this version on.

## [0.1.0] - 2026-08-12

First release, so there is nothing to diff against: the entry below is
what the crate covers rather than what changed in it. Later entries will
be actual changes against this baseline. `README.md` documents each item
in detail, and the [issue tracker](https://github.com/joeferner/rpi-hal/issues)
has what is deliberately not here yet.

### Added

- **Boot runtime** (`rt`, on by default): `_start`, `.bss` zeroing,
  exception vectors, and a `critical-section` implementation, for both
  `armv7a-none-eabi` (32-bit, `kernel7.img`) and
  `aarch64-unknown-none-softfloat` (64-bit, `kernel8.img`). A consumer
  that owns its own boot sequence turns the feature off.
- **Linker script**, published on the linker search path as `rpi-link.x`
  and already carrying the load address for the target being built, so a
  downstream binary names it with one `-T` line and keeps no copy of its
  own.
- **Identity-mapped MMU** (`mmu`, on by default), which is what makes
  `core::sync::atomic` work at all: `ldrex`/`strex` are architecturally
  UNPREDICTABLE against the Device memory every address defaults to with
  translation off. A consumer can supply its own table instead.
- **Secondary cores** (`multicore`), plus the cross-core spinlock that
  becomes necessary once one is running.
- **Chip selection**, exactly one of `bcm2837` (Pi 2, Pi 3) or `bcm2711`
  (Pi 4) — neither is a default, since there is no sensible default target
  chip. `bcm2711` is preliminary: boot, GPIO, the System Timer, UART and
  SD via EMMC2 are verified on hardware, most other drivers are untested,
  and there is no interrupt controller for it yet, so nothing
  interrupt-driven is available under it.
- **Peripheral drivers**, with `embedded-hal` 1.0 and `embedded-io` trait
  implementations alongside inherent methods where the traits fit:
  - GPIO (typestated pins, edge/level interrupts), UART0, the mini UART,
    SPI0, the AUX block's SPI1/SPI2, and I2C on BSC0/BSC1.
  - The System Timer, the per-core ARM generic timer, the watchdog, and
    reboot/shutdown through the PM block.
  - The VideoCore mailbox property interface, including a scanout
    framebuffer with page flipping, and the FT5406 touch panel.
  - SD/MMC over the Arasan EMMC controller (and BCM2711's separate EMMC2),
    DMA, PWM and PCM/I2S audio out, the hardware RNG, and the performance
    monitor unit.
  - USB: the DWC2 host controller, the LAN9514 Ethernet chip, and HID
    keyboard/mouse/gamepad devices over a shared report-descriptor parser.
  - Pi 3 on-board radio: Wi-Fi over SDIO (firmware and CLM blob load,
    scanning, WPA2, and a full TCP/IP path), and Bluetooth over HCI — BLE
    advertising, scanning, connections, L2CAP, an ATT/GATT server and
    client, SMP pairing with bonding, and HID over GATT.
  - Camera: the Unicam1 CSI-2 receiver with an OV5647 sensor driver.
- **3D graphics** (`v3d`, Pi 3 only): VideoCore IV's binning/render
  pipeline and control-list builders, enough for a depth-tested, textured
  draw per frame.
- **Interrupts**: CPU-level masking, the legacy interrupt controller, and
  the `__irq_handler` contract that makes dispatch the application's — see
  `README.md`, because a missing or incomplete handler is the one mistake
  here that produces no error message at all.
- **Async** (`async`): `embedded-hal-async`'s `digital::Wait` on input
  pins and `embedded-io-async`'s `Read`/`Write` on `Uart`, as plain
  `poll`/`Waker` code with no executor dependency — see the
  `rpi-hal-embassy` crate for an executor and time driver built on it.
- **Integration adapters**, each behind a feature of the same name:
  `embedded-sdmmc` (`BlockDevice` over SD), `smoltcp` (`phy::Device` over
  Ethernet and Wi-Fi), and `embassy-net-driver` (`Driver` over Ethernet).
- Around 75 examples, each of which runs on real hardware rather than in a
  simulator.

### Notes

- Requires stable Rust **1.88** or newer — `#[unsafe(naked)]` and
  `naked_asm!` in the FPU bring-up are the floor. Enabling `smoltcp`
  raises it to 1.91, which is smoltcp's own requirement, not this crate's.
  Nightly is not needed.
- Licensed under either MIT or Apache-2.0, at your option.

[0.2.0]: https://github.com/joeferner/rpi-hal/releases/tag/v0.2.0
[0.1.0]: https://github.com/joeferner/rpi-hal/releases/tag/v0.1.0
