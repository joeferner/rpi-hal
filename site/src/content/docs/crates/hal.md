---
title: rpi-hal
description: The hardware abstraction layer for bare-metal Raspberry Pi.
---

The hardware abstraction layer: boot code, the MMU and caches,
interrupts, multicore bring-up, and the SoC's peripherals: GPIO, UARTs,
SPI, I2C, PWM, PCM/I2S, DMA, the SD card, USB host, the mailbox, and
more. It looks and feels like other `embedded-hal` implementations
(`stm32xx-hal`, `rp2040-hal`): the same naming, the same
typestate/ownership patterns, and `embedded-hal` traits implemented
alongside inherent methods rather than instead of them.

| Chip feature | Boards | Target |
| --- | --- | --- |
| `bcm2837` | Pi 2, Pi 3 | `armv7a-none-eabi`, `aarch64-unknown-none-softfloat` |
| `bcm2711` | Pi 4 (preliminary: no interrupt controller yet) | the same two |
| `bcm2835` | Pi 1, Pi Zero (preliminary) | `armv6-none-eabi` (nightly) |
| `bcm2712` | Pi 5 (preliminary, from 0.10.0: GPIO, UART, System Timer, mailbox, SD card, I2C so far) | `aarch64-unknown-none-softfloat` |

None of the chip features is a default; an application names exactly one.

```toml
[dependencies]
rpi-hal = { version = "0.10", features = ["bcm2837"] }
```

- **API reference:** [docs.rs/rpi-hal](https://docs.rs/rpi-hal)
- **README** (status of every driver, the toolchain, the
  `__irq_handler` contract, virtual memory, the stack):
  [hal/README.md](https://github.com/joeferner/rpi-hal/blob/main/hal/README.md)
- **Running its examples:**
  [hal/docs/getting-started.md](https://github.com/joeferner/rpi-hal/blob/main/hal/docs/getting-started.md)
- **Changes:** [hal/CHANGELOG.md](https://github.com/joeferner/rpi-hal/blob/main/hal/CHANGELOG.md)
- **crates.io:** [rpi-hal](https://crates.io/crates/rpi-hal)
