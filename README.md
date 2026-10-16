# rpi-hal

Bare-metal Rust for the Raspberry Pi.

**Docs, blog, and a project starter:**
[joeferner.github.io/rpi-hal](https://joeferner.github.io/rpi-hal/), built
from [`site/`](site).

This repository holds a family of crates, each in its own directory with
its own version, changelog and crates.io release:

| Directory | Crate | What it is |
| --- | --- | --- |
| [`hal/`](hal) | [`rpi-hal`](https://crates.io/crates/rpi-hal) | The hardware abstraction layer: boot, MMU, interrupts and the SoC's peripherals, implementing `embedded-hal` |
| [`embassy/`](embassy) | [`rpi-hal-embassy`](https://crates.io/crates/rpi-hal-embassy) | Embassy support: the time driver, the executor, and `embassy-net` drivers |
| [`kickstart/`](kickstart) | [`rpi-kickstart`](https://crates.io/crates/rpi-kickstart) | The bring-up every application does on top of `rpi-hal`: console, heap, config, storage, networking, web, OTA |
| [`loader/`](loader) | [`rpi-loader`](https://crates.io/crates/rpi-loader), [`rpi-loader-ota`](https://crates.io/crates/rpi-loader-ota) | A resident UART loader: flash it to the card once, then upload firmware over serial. The host CLI, the firmware images, and the OTA bundle format |

New here? Start with [getting started](hal/docs/getting-started.md):
building an example and putting it on a card.

## Start a new project

```sh
cargo install cargo-generate
cargo generate --git https://github.com/joeferner/rpi-hal template
```

It asks which board (and, on a Pi 3 or Pi 4, 32- or 64-bit), and
generates a project on rpi-hal and rpi-kickstart that logs a heartbeat on
the UART console. `make sd-card` fetches what goes on the SD card, and
`make run` uploads and boots it with rpi-loader. The boards it offers,
and everything it knows about each, live in
[`template/boards.toml`](template/boards.toml).

Each directory is a separate cargo project with its own toolchain pin
and `.cargo/config.toml`, so run `cargo` and `make` from inside it (or
use the root `Makefile`, which does that for you). There's deliberately
no `Cargo.toml`, `.cargo/` or `rust-toolchain.toml` at the repository
root: cargo reads its config from the working directory upward, and one
here would apply to every crate below it.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option.
