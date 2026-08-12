# Changelog

Notable changes to `rpi-hal`, in the format of
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). This crate
follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - ReleaseDate

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

[0.1.0]: https://github.com/joeferner/rpi-hal/releases/tag/v0.1.0
