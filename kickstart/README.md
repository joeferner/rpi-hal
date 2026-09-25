# rpi-kickstart

[![CI](https://img.shields.io/github/actions/workflow/status/joeferner/rpi-kickstart/ci.yml?branch=main&label=CI)](https://github.com/joeferner/rpi-kickstart/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/rpi-kickstart.svg)](https://crates.io/crates/rpi-kickstart)
[![docs.rs](https://img.shields.io/docsrs/rpi-kickstart)](https://docs.rs/rpi-kickstart)

The bring-up a bare-metal Raspberry Pi application does before it is an
application, on top of the `rpi-hal` crate.

`rpi-hal` stops at the peripheral: it hands out a `Uart`, a `Mailbox`, an
`Sd`. What every board then does with them turns out to be the same
program written again — a console the whole image can print to, a heap
sized from what the firmware reports, a card mounted and a settings file
read off it, a site served out of RAM, a clock that something has to set.
This crate is that program, written once.

## Status

Early. The repository, the build and the release path are in place, and
the modules are being moved in one at a time, each behind the feature
named for it.

| Feature | What it is |
| --- | --- |
| `console` | One sink for the whole image, and `logln!` |
| `entropy` | The SoC's hardware RNG, and the `getrandom` backend over it |

Still to come: clock, heap, storage, config, site, web, metrics, mdns,
ota.

## Chips

`bcm2835` (Pi 1, Zero), `bcm2837` (Pi 2, 3) and `bcm2711` (Pi 4) forward
to `rpi-hal`'s features of the same name. None is a default.

Most boards never name one here: an application's own `rpi-hal` line
already selects a chip and cargo unifies it across the graph. They exist
for a build that has no such line — this repository's own, and a consumer
that depends on `rpi-kickstart` without depending on `rpi-hal` directly.

They are not mutually exclusive, because cargo features cannot be. Two at
once resolves by `rpi-hal`'s precedence — `bcm2711` > `bcm2837` >
`bcm2835` — rather than failing, which is why nothing here builds with
`--all-features`; the `make` recipes name an explicit set instead.

Only `entropy` needs a chip so far. `console` deliberately reaches
nothing in `rpi-hal`, so a board taking only that needs no selection at
all.

`examples/hello.rs` takes no features at all, which is what makes it
useful: it proves the parts that have nothing to do with the crate's
contents — toolchain, targets, linker script, both load addresses — so if
it boots, none of those is what broke.

## Design

**Everything is opt-in.** Each capability is its own feature and its own
module, and none are on by default. A board states what it is and pays
for nothing else — a board with no network takes no TLS, and a board with
a DS3231 takes no SNTP.

**Everything is replaceable.** Where a piece has a plausible second
implementation — the time source above all — what this crate owns is the
*sink* the rest of the image reads through, and filling it is the
application's choice. Nothing here is a fork to escape from.

**The tasks stay with the application.** An
`#[embassy_executor::task]` cannot be generic, so this crate never
declares one: it exposes `async fn`s and the application wraps them in
tasks with its own concrete types. Routing, the `#[global_allocator]` and
the `#[panic_handler]` stay there too, for the same reason — they are
single, program-wide choices that a library taking them away cannot give
back.

## Relationship to the other crates

| Crate | What it is |
| --- | --- |
| `rpi-hal` | The HAL: peripherals, boot sequence, MMU, drivers |
| `rpi-hal-embassy` | Embassy's platform pieces: time driver, executor, network adapters |
| `rpi-kickstart` | This crate: the application layer above both |
| `rpi-loader-ota` | The OTA bundle format, shared with the loader |

## Building

```sh
cargo build                                         # AArch32 (Pi 2, Pi 3)
cargo build --target aarch64-unknown-none-softfloat # AArch64 (Pi 3)
```

AArch32 is the default target only because it is the one a Pi 2 can run;
`make build` covers both. The example images link against `rpi-hal`'s
published `rpi-link.x`, which is already at the load address for the
target being built — 0x8000 for a `kernel7.img`, 0x80000 for a
`kernel8.img` — so an image direct-boots where the firmware puts it.

`make pre-commit` runs the whole check set: formatting, clippy, the
library and example builds for both architectures, and a doc build with
warnings denied.

ARMv6 (Pi 1, Pi Zero) has no `make` recipes here, unlike `rpi-hal` and
`rpi-hal-embassy`. That target is tier 3 — no precompiled `core`, so
nightly and `-Z build-std` — and nothing in this crate is
architecture-specific, so adding it is a `make` change rather than a
source one whenever a board wants it.

## Examples

See [`examples/`](examples/). Each one's header comment says what it
demonstrates and what its output is evidence for.

`scripts/build-example.sh <name>` produces `target/kernel7.img`;
`scripts/build-example64.sh <name>` produces `target/kernel8.img`. Copy
either to an SD card's boot partition, or send it over UART with
`rpi-loader`:

```sh
scripts/build-example.sh hello
rpi-loader --device /dev/ttyUSB0 boot --load-addr 0x8000 target/kernel7.img
```

## License

MIT OR Apache-2.0
