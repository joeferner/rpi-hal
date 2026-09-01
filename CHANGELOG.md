# Changelog

Notable changes to `rpi-hal`, in the format of
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This crate
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed

- **`Sd::init` muxed the Ethernet PHY's pins away on a Pi 4.** It routed
  GPIO48-53 to alternate function 7 on every chip, but on BCM2711 the
  card slot is on EMMC2, which drives dedicated pads outside the 54-pin
  bank — `bcm2711.dtsi`'s `emmc2` node has no `pinctrl` property at all,
  which is why the Pi 4 SD path worked regardless. What GPIO48-53 carry
  on that board is the gigabit Ethernet PHY's RGMII interface
  (`RGMII_RXD0`..`RXD3`, `RGMII_TXCLK`, `RGMII_TXCTL`), so the mux was
  pure side effect: it severs the MAC from the PHY, and points four
  lines the PHY drives at a host controller that drives them back during
  a transfer. `route_gpio_to_emmc` is now compiled out under `bcm2711`;
  `Sd::init` keeps its `GPIO` argument on both chips so a call site
  doesn't have to change. Untested on hardware in the direction that
  matters — nothing in this crate drives BCM2711 Ethernet yet, so
  nothing here could have noticed.

  The comment that justified sharing the routing said GPIO48-53's ALT3
  assignment was "unchanged (confirmed by diffing `bcm2711-lpa` against
  `bcm2837-lpa`)". That was true and beside the point: a PAC diff
  describes the SoC's function numbering, not what a board wired to the
  pads.

- **PWM and PCM clock divisors were silently masked, not clamped.** The
  Clock Manager's `DIVI` field is 12 bits, but `Pwm::init` and `Pcm::init`
  take a `u16` and said nothing about the limit — so a larger value was
  neither rejected nor saturated. The PAC's field writer masked it, making a
  divisor of 12500 program as `12500 & 0xFFF` = 212 and run the clock 59
  times too fast, with every register reading back exactly as written. Both
  now clamp. `Pwm::audio_clock_divisor` and `Pcm::clock_divisor` had the same
  fault from the other end, clamping their results to `u16::MAX` — sixteen
  times what the field holds — and now clamp to the real maximum.

### Added

- **Interrupt-driven SD transfers**, behind the `async` feature:
  `Sd::read_block_async`/`read_blocks_async`/`write_block_async`/
  `write_blocks_async` and the DMA pair
  `read_blocks_dma_async`/`write_blocks_dma_async`, plus `sd::on_irq` and
  `Lic::enable_emmc_irq`/`disable_emmc_irq`/`is_emmc_pending` to route the
  controller's line. The blocking methods are unchanged and untouched by
  this; the async ones park on the controller's interrupt where those
  spin, which matters most for a write, whose closing `DATA_DONE` is the
  card programming an entire internal erase block — milliseconds per
  command that an executor previously lost in full.
  `examples/sd_async.rs` reports, for each transfer, the share of its
  duration during which the core had nothing to do.

  Dropping a transfer future — `embassy_time::with_timeout`, `select!`, a
  cancelled task — stops the card and resets the controller's data
  circuit before the drop returns, and so does an error return. Without
  that, an abandoned data phase would leave part of an aborted block in
  the host FIFO for the *next* transfer to return as though it were data.

  Two things it deliberately does not do: enable anything in `IRPT_EN`
  outside a wait (a level source nobody services is a hang on this
  controller, so each wait opens only the bits it parks on and closes
  them again), and impose its own timeout beyond the blocking path's
  one-second backstop — wrap the future in the executor's own. BCM2836/7
  only for now: routing the line needs `lic`, which BCM2711 has no
  equivalent of yet.

- **Non-blocking DMA transfers to and from a peripheral FIFO**:
  `Channel::start_from_peripheral` and `Channel::start_to_peripheral`,
  which start the transfers `copy_from_peripheral`/`copy_to_peripheral`
  block on and hand back a `Transfer` guard instead, so a caller can wait
  on something better than a polling loop. The read side defers its cache
  invalidate to the guard's drop, which is the first point at which the
  engine is known to have finished.

- **GPIO internal pull resistors.** `gpio::Pull`, `Pin::set_pull`, and
  `Pin::into_pull_up_input`/`into_pull_down_input`/`into_floating_input`
  configure a pin's internal pull-up/pull-down — previously unreachable
  from outside the crate, so a consumer wiring a button or an
  open-collector sensor had to add an external resistor or poke
  `GPPUD` themselves. `Pin::pull` reads the setting back, on `bcm2711`
  only: the legacy `GPPUD`/`GPPUDCLK` pair clocks a value into a pin
  without storing it anywhere readable. `examples/gpio_pull.rs` checks
  both resistors against an unconnected pin, and
  `examples/gpio_irq_button.rs` now uses the internal pull-down instead
  of asking for a 10k resistor.

  The two SoCs use unrelated registers here — the legacy
  `GPPUD`/`GPPUDCLK` clock-in sequence versus BCM2711's
  `GPIO_PUP_PDN_CNTRL_REG0..3`, with *different encodings* of the pull
  value — and four drivers (`uart`, `mini_uart`, `sd`, `sdio`) each
  carried their own copy of the sequence for their own pins. They now all
  route through the one implementation in `src/gpio.rs`, which is the
  only place that knows which scheme applies.

- **`resident-fat` feature**: `sd::SdBlockDevice`, an adapter implementing
  `resident-fat`'s `BlockDevice` trait over the SD driver, with
  `sd::SdBlockDeviceError` for its errors.
  `examples/sd_resident_fat_read.rs` mounts the boot partition and reads
  files.

  Alongside the `embedded-sdmmc` adapter rather than replacing it: the two
  traits differ in their unit of transfer, and which one suits depends on
  the filesystem above. `resident-fat` transfers a plain `&[u8]` spanning a
  whole run of consecutive blocks, which is already what the driver's
  multi-block path takes, so the adapter splits the caller's buffer with
  `as_chunks` and hands the pieces over — no staging buffer, no copy, and
  `max_transfer_blocks` is the controller's real 65535 rather than a
  buffer's size. Reaching `resident-fat` through its own `embedded-sdmmc`
  bridge and `sd::SdCard` still works, and remains the right route for a
  consumer already invested in that trait.

  Unlike every other feature here, this one carries an allocator
  requirement: `resident-fat` uses `alloc`, so a binary that enables it
  must register a `#[global_allocator]`. This crate still neither defines
  nor needs one.
- `Pwm::MAX_CLOCK_DIVISOR` and `Pcm::MAX_CLOCK_DIVISOR`, so a caller can
  check its own constant at compile time rather than discovering the limit as
  a peripheral running at an inexplicable rate.
- `Pwm::clock_hz` and `Pcm::clock_hz`, reporting the rate a divisor will
  actually produce. They apply the same clamp `init` does, so they describe
  the hardware rather than echoing the request back; logging one beside the
  intended rate is how an out-of-range divisor becomes visible.
- `Pwm::MIN_CLOCK_HZ` and `Pcm::MIN_CLOCK_HZ`, the floor the 12-bit divisor
  imposes — roughly 122 kHz, which is a real design constraint and not a
  rounding concern.
- `Pwm::divisor_for`, picking a divisor from a target clock rate. The
  counterpart to `audio_clock_divisor` for callers not on the audio path,
  where computing `500_000_000 / target` by hand is exactly where an
  out-of-range divisor comes from.

## [0.3.0] - 2026-08-30

### Added

- **HDMI audio** (`mmal` feature): `audio_render::AudioRenderer`, audio out
  through the firmware's `ril.audio_render` component —
  `Destination::Hdmi`, or `Destination::Local` for the 3.5 mm jack.
  Interleaved signed-16-bit samples in, paced by the renderer itself rather
  than by any timer on this side. `examples/hdmi_audio.rs` plays a stereo
  tone.
- `mmal::AudioFormat`, and `mmal::PortInfo::audio` alongside the existing
  `video`: the two are the same bytes read two ways, since the message
  carries one type-specific union, and `port_info_set` now writes whichever
  the port's `es_type` names.
- `mmal::parameter_set_string` and `mmal::ENCODING_PCM_SIGNED_LE`, the
  parameter shape and the encoding the audio renderer is configured with.

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
- **Async I2C** (`async` feature): `embedded_hal_async::i2c::I2c` on the
  same `I2c` type, parking on the controller's `DONE`/`TXW`/`RXR`
  interrupts rather than polling `S`, so the millisecond a six-byte read
  at 100kHz costs goes to the executor instead of a spin loop. With it,
  `i2c::on_irq` and `Lic::enable_i2c_irq`/`disable_i2c_irq`/
  `is_i2c_pending`. BSC0 and BSC1 share one interrupt line, so the
  handler checks both controllers, and leaves alone any that a blocking
  transfer is driving (it arms none of these conditions).

  Timeouts are the caller's here rather than the driver's: wrap the
  future in `embassy_time::with_timeout` or equivalent. Cancelling one
  that way is safe — the drop masks the interrupts, clears the FIFOs and
  cleans the status, so the next transfer starts from a known state. The
  stored `Timer` deadline still applies as a backstop, but only where the
  future is polled at all, which the module docs spell out.
- **`examples/soc_temperature.rs`**, printing die temperature, ARM clock
  and throttling status together once a second. No new API —
  `Mailbox::temperature_millicelsius` and `Mailbox::throttled` have been
  there all along, and a consumer asking for a way to read the CPU
  temperature is what showed they could not be found. The README's
  mailbox entry now names them too.
- **`i2c::divider_for` and `spi::divider_for`**: `(core_hz, target_hz)`
  to the raw `CDIV` those drivers' `init` takes. Every consumer was
  writing the same arithmetic and getting the same chance to be wrong,
  the reset default of 1500 being documented as 100kHz against a nominal
  150MHz core clock and actually being 166kHz on a board running
  250MHz. Rounding is upwards in both, so the bus never clocks faster
  than asked — what a device states is a maximum, and erring the other
  way fails intermittently rather than visibly.

  `core_hz` is still the caller's to fetch (`Mailbox::clock_rate_hz`
  with `ClockId::Core`) rather than something `init` queries: it can
  fail, it costs a round trip to the GPU, and an application bringing up
  several buses should ask once.
- **`i2c::I2c::<BSC0>::init_id`**: BSC0 on its GPIO0/1 (ALT0) routing —
  `ID_SD`/`ID_SC` on header pins 27/28, the HAT ID EEPROM bus — beside the
  existing `init`, which stays on GPIO44/45. One controller, two
  electrically separate buses, so the routing is a constructor rather than
  an argument, and only one of them can be live at a time. Previously the
  ID bus was unreachable from this crate, which put any board-identity or
  per-unit calibration part sitting on it out of reach too.
  `examples/i2c_hat_eeprom.rs` reads a HAT EEPROM's vendor info atom over
  it.
- **`stack`** (`rt` feature): `stack::headroom`, `used`, `pointer`,
  `bottom`, `top` and `size` — how much of the main stack is left, from
  inside the running program. `headroom`/`used` are `Option` because a
  secondary core runs on its own `multicore::Stack` and the AArch32
  exception modes on their own banked regions, where the question has no
  meaningful answer.

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

### Changed

- **The stack is a reserved region with a stated size**, rather than
  whatever happened to sit below the load address. The linker scripts
  reserve `__stack_size` (1 MiB), `__stack_slack` (2 MiB of margin below
  it), and on AArch32 `__irq_stack_size` (64 KiB) plus
  `__abt_stack_size`/`__und_stack_size`/`__fiq_stack_size` (32 KiB each);
  the boot code points each `sp` at its own region. Any of them can be
  changed without editing the script, via
  `-Wl,--defsym=__stack_size=0x400000` in the consumer's own flags. The
  region is `NOLOAD`, so none of it costs image bytes.

  Programs that supply their own linker script *and* use the `rt` feature
  must define `__stack_top` (and, on AArch32, `__irq_stack_top`,
  `__abt_stack_top`, `__und_stack_top`, `__fiq_stack_top`); the link
  fails loudly naming the missing symbol otherwise. A program using the
  crate's `rpi-link.x` needs no changes.
- **`__unhandled_exception` is now weak** on both architectures, so an
  application can define its own and report a fault instead of parking
  silently. The crate's default (a `wfe` loop) is unchanged when nothing
  overrides it.
- **`i2c::I2c` gained a lifetime and `init` a parameter**: both
  `I2c::<BSC1>::init` and `I2c::<BSC0>::init` now take a `&Timer`, which
  the driver stores as `I2c<'_, I>`. The timer bounds every transfer (see
  Fixed, below); it has to be stored rather than passed per call because
  transfers are reached through `embedded_hal::i2c::I2c::transaction`,
  whose signature this crate doesn't control.
- `i2c::Error` gained `Timeout` and `Incomplete { received, requested }`,
  so it is no longer exhaustively matchable on the two previous variants.
  Both map to `ErrorKind::Other` — `embedded-hal` 1.0 has no closer
  variant, since its `Overrun` means the receive buffer was overrun.
- **A clock-stretch timeout (`S.CLKT`) is now reported**, as
  `Error::Timeout`, by the blocking transfers as well as the new async
  ones — a slave that held SCL past the `CLKT` allowance produced a
  transfer the hardware cut short, and returning its bytes as if nothing
  had happened was wrong. `CLKT` is also cleared alongside `DONE`/`ERR`
  now: it latches, so one uncleared timeout would have been read as a
  fault by every transfer after it, on a bus that had recovered.

### Fixed

- **The IRQ stack no longer sits inside the main stack.** It was set to
  `_start - IRQ_STACK_SIZE`, 4 KiB into the region main mode was growing
  down through, so any main-mode frame deeper than 4 KiB occupied memory
  the first interrupt would push onto — the opposite of what the comment
  there claimed. The two are now adjacent reserved regions.
- **The stack no longer grows down through low memory**, where the
  firmware leaves the ATAGs and, on AArch64, the armstub8 spin table that
  `multicore` starts cores 1-3 through.
- **An I2C transfer can no longer hang the program.** Both transfer loops
  polled `S` with no exit but `ERR` or `DONE`, and a slave that
  acknowledges and then stops driving — one stretching the clock
  indefinitely, a half-soldered part, a line held low — sets neither. The
  loop was then infinite, and since this is a blocking driver it took
  whatever else the program had to do with it: an executor, a network
  stack, everything. Transfers are now bounded against the System Timer
  (a fixed allowance plus a per-byte one) and report `Error::Timeout`.
- **A short read no longer spins forever.** `read_one` waited for `DONE`
  *and* a full buffer, so a transfer that completed having delivered
  fewer bytes than `DLEN` asked for was waiting on a condition that had
  already become unreachable. That case is now `Error::Incomplete`, which
  carries both counts — how many bytes arrived is what says whether a
  device is mute, truncating, or was simply over-read.
- After either failure the controller is returned to a usable baseline
  (FIFOs and status cleared) so a subsequent transfer starts from a known
  state. Best-effort by necessity: the BSC has no documented abort and
  owns the pins while enabled, so nothing here can walk a slave off a bus
  it is still holding — that transfer times out too, which is survivable
  where a hang wasn't.

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

[0.3.0]: https://github.com/joeferner/rpi-hal/releases/tag/v0.3.0
[0.2.0]: https://github.com/joeferner/rpi-hal/releases/tag/v0.2.0
[0.1.0]: https://github.com/joeferner/rpi-hal/releases/tag/v0.1.0
