# rpi-loader

[![CI](https://img.shields.io/github/actions/workflow/status/joeferner/rpi-loader/ci.yml?branch=main&label=CI)](https://github.com/joeferner/rpi-loader/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/rpi-loader.svg)](https://crates.io/crates/rpi-loader)

Host-side CLI for [rpi-loader](https://github.com/joeferner/rpi-loader), a
self-relocating UART command agent for the Raspberry Pi 2, 3 and 4. Flash
the loader onto an SD card once; after that this tool drives it over
serial — upload firmware to memory and jump to it, or read, write, list
and delete files on the card — instead of pulling the card for every
build.

The loader stays resident and services one command at a time, so this is
a subcommand CLI rather than a one-shot uploader. Each invocation
reconnects to the running loader with a fresh handshake; the Pi is
power-cycled only to re-flash the loader itself.

## Install

```sh
cargo install rpi-loader
```

The loader image that runs on the Pi is a separate build, published as a
release artifact. See the
[repository](https://github.com/joeferner/rpi-loader) for flashing it.

## Usage

Find the cable first — `list` shows USB serial ports and what each one
reports about itself, which is how you tell two of them apart:

```sh
$ rpi-loader list
/dev/ttyUSB0  FTDI TTL232R-3V3  0403:6001  serial FTF3KY4Y
```

Add `--all` to include the legacy `/dev/ttyS*` range, PCI and Bluetooth
serial. Listing opens nothing and needs no privileges, so it works even
where opening the port itself would be refused.

Everything else takes the device with `--device`, on either side of the
subcommand:

```sh
DEV=/dev/serial/by-id/usb-FTDI_TTL232R-3V3_*

# Upload a kernel and boot it, then act as a terminal
rpi-loader --device $DEV boot --load-addr 0x8000 path/to/kernel7.img

# Files on the SD card's FAT boot partition
rpi-loader --device $DEV sd-list /
rpi-loader --device $DEV sd-read /config.txt ./config.txt
rpi-loader --device $DEV sd-write ./app.bin /APP.BIN
rpi-loader --device $DEV sd-delete /APP.BIN
rpi-loader --device $DEV sd-mkdir /LOGS

# Lower-level memory control
rpi-loader --device $DEV mem-write 0x8000 path/to/kernel7.img
rpi-loader --device $DEV exec 0x8000 --terminal

# Watch and drive a running kernel (no handshake)
rpi-loader --device $DEV terminal
```

## Terminal mode

`terminal`, and the terminal that `boot` and `exec --terminal` drop into,
is bidirectional: device output is printed, and what you type is sent
straight out the serial port. Input is unbuffered and unechoed locally,
so what you see is what the device echoed back.

**Ctrl-]** exits. Ctrl-C deliberately does not — it is forwarded like any
other key, which is what lets you interrupt a program running on the
device. When stdin is a pipe rather than a keyboard there is no raw mode
to speak of, and Ctrl-C goes back to its usual job of ending the process.

`boot` requires `--load-addr`: `0x8000` for a 32-bit `kernel7.img`,
`0x80000` for a 64-bit `kernel8.img`. There is deliberately no default —
sending an image to the wrong address is a mistake worth making the
caller state their intent about.

## Baud

The handshake and terminal always run at 115200, matching the loader's
own UART bring-up and a freshly booted kernel's default. The bulk
transfers (`boot`, `mem-write`, `sd-read`, `sd-write`) negotiate up to
`--baud` (1500000 by default) and always drop back to 115200 before
returning, so the next invocation — and any kernel that gets booted —
finds the link at the rate it expects.

Pass `--baud 115200` to disable the speedup, or a lower value if a
marginal cable or long wiring corrupts chunks faster than the retry loop
can recover:

```sh
rpi-loader --device $DEV boot --load-addr 0x8000 --baud 921600 kernel7.img
```

## Serial port access without sudo

`/dev/ttyUSB*` is usually owned by `root` plus a system group (`uucp`,
`dialout`, `plugdev`, depending on distribution), so without setup this
needs `sudo`. Add yourself to that group — or install a udev rule; the
repository ships one — and log out and back in, since group membership
does not apply to already-running sessions.

## Protocol

The wire protocol is documented in `src/link.rs` on this side and in the
firmware's `main.rs` on the device side. Transfers are chunked and
CRC-32 checked in both directions, with per-chunk retries, so a marginal
cable degrades throughput rather than corrupting an image.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or
  <http://opensource.org/licenses/MIT>)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual licensed as above, without any additional terms or
conditions.
