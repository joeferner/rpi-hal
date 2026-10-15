# rpi-hal

Bare-metal Rust for the Raspberry Pi.

This repository holds a family of crates, each in its own directory with
its own version, changelog and crates.io release:

| Directory | Crate | What it is |
| --- | --- | --- |
| [`hal/`](hal) | [`rpi-hal`](https://crates.io/crates/rpi-hal) | The hardware abstraction layer: boot, MMU, interrupts and the SoC's peripherals, implementing `embedded-hal` |

Each directory is a separate cargo project with its own toolchain pin
and `.cargo/config.toml`, so run `cargo` and `make` from inside it (or
use the root `Makefile`, which does that for you). There's deliberately
no `Cargo.toml`, `.cargo/` or `rust-toolchain.toml` at the repository
root: cargo reads its config from the working directory upward, and one
here would apply to every crate below it.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option.
