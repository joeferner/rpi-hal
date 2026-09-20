# rpi-loader

[![CI](https://img.shields.io/github/actions/workflow/status/joeferner/rpi-loader/ci.yml?branch=main&label=CI)](https://github.com/joeferner/rpi-loader/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/rpi-loader.svg)](https://crates.io/crates/rpi-loader)

Self-relocating UART command agent for the Raspberry Pi. Flash this once
from an SD card; after that it stays resident and the host drives it over
serial — upload firmware to memory and jump to it, or read/write/list
files on the SD card — instead of rewriting the SD card for every build.

Supported boards:

- Pi 2 Model B rev 1.2 and Pi 3 (BCM2836/BCM2837) — the `bcm2837` builds,
  in either execution state: AArch32 (`kernel7.img`) and AArch64
  (`kernel8.img`).
- Pi 4 (BCM2711) — the `bcm2711` builds, which move the peripheral memory
  map and PAC through `rpi-hal`'s feature of the same name. Both execution
  states again.
- Pi 1 and Pi Zero (BCM2835) — the `bcm2835` build, which is also the only
  one that is a different *instruction set* rather than a different chip:
  ARMv6, its own boot stub (`firmware/src/boot6.s`), and a `kernel.img`
  with no digit. AArch32 only, the ARM1176 having no 64-bit mode, and the
  only build here that needs a nightly toolchain (see "Or build one").

A green CI badge means it compiles, and nothing more. Every check that
runs there is either a compile-time one or a protocol test against a fake
device on a pty; whether a transfer survives a real 1.5Mbaud link is
established on hardware, not in CI.

Depends on [`rpi-hal`](https://crates.io/crates/rpi-hal) for GPIO/UART
and SD/FAT access. The FAT filesystem layer is
[`embedded-sdmmc`](https://crates.io/crates/embedded-sdmmc), on top of
`rpi-hal`'s `SdCard` block-device adapter.

## Layout

- `firmware/` — the loader itself, a `no_std` package built for a bare
  metal ARM target. It is never published to crates.io: what it produces
  is a raw image to copy onto an SD card, not something `cargo install`
  can build.
- `cli/` — the host-side driver that talks to a running loader over
  serial. This is the package published to crates.io as `rpi-loader`,
  and the only part of the project `cargo install` can build.
- `ota/` — the over-the-air update bundle format, published separately as
  `rpi-loader-ota`. A `no_std` library rather than a tool: the CLI uses it
  to pack a bundle, and a board's own firmware links it to validate and
  install one. It carries its own version, because its consumers are
  firmware projects elsewhere and there is no reason a renamed CLI flag
  should bump their dependency.
- The repository root has no cargo configuration on purpose. Cargo
  discovers `.cargo/config.toml` by walking up from the working
  directory, so a root-level one naming a bare metal target would be
  inherited by host-side tooling beside it, which then fails to build.

## Commands

After a handshake, the loader services a command at a time and stays
resident for the next — so a single flashed loader backs any number of
host operations. Each `rpi-loader` subcommand names the serial device
with `--device`, then takes its own arguments:

- `mem-write <addr> <file>` — write `<file>` to memory at `<addr>` (hex
  or decimal, e.g. `0x8000`), checksummed, no jump.
- `exec <addr>` — jump to `<addr>`. Add `--terminal` to stay attached as
  a passthrough terminal afterward.
- `sd-list [path]` — list a directory on the SD card (default `/`;
  nested paths like `/boot/overlays` supported).
- `sd-read <remote> <local>` — copy `<remote>` (a path on the SD card)
  to the `<local>` file on the host.
- `sd-write <local> <remote>` — copy the `<local>` host file to
  `<remote>` on the SD card, creating or truncating it.
- `sd-delete <remote>` — delete `<remote>` from the SD card.
- `sd-mkdir <remote>` — create the directory `<remote>` on the SD card
  (a single level; the parent directories must already exist).
- `eeprom-read <local>` — copy an EEPROM on the HAT ID bus (GPIO0/1) to
  the `<local>` file. With no `--length`, reads the image length out of
  the HAT header first.
- `eeprom-write <local>` — program `<local>` into that EEPROM, verifying
  every page as it goes. See below.
- `boot <image>` — convenience: `mem-write` the image, `exec` its load
  address, then act as a terminal. Requires `--load-addr` (`0x8000` for
  a 32-bit `kernel7.img`, `0x80000` for a 64-bit `kernel8.img`).
- `terminal` — bidirectional passthrough terminal, with no handshake
  (for watching and driving an already-running kernel). What the device
  sends is printed; what you type is sent. **Ctrl-]** exits — every
  other key, Ctrl-C included, goes to the device, which is what lets you
  interrupt something running *there*.
- `list` — list the host's USB serial ports (`--all` for every port), to
  find what to pass to `--device`.
- `bundle [manifest]` — pack an over-the-air update bundle described by a
  `bundle.toml`, and with `--upload <url>` post it to a running board.
  See below.
- `completions [shell]` — write a shell completion script to stdout
  (`bash` unless another is named; `zsh`, `fish`, `elvish` and
  `powershell` also work). See below.

`list`, `bundle` and `completions` are the subcommands that neither open
a port nor need one.

The bulk commands (`mem-write`, `sd-read`, `sd-write`, `boot`,
`eeprom-read`, `eeprom-write`) also take
`--baud` to pick the transfer rate (see below). Booting an uploaded
image is just `mem-write` + `exec`; `boot` chains them plus the terminal
to reproduce the classic one-shot upload flow.

The wire protocol is documented on the device side in
`firmware/src/main.rs` and on the host side in `cli/src/link.rs`.

## Shell completion

`rpi-loader completions` writes a completion script to stdout, generated
from the same command tree clap builds for `--help` — so it describes
whatever subcommands and flags the binary you are running actually has,
rather than a copy that has to be kept in step by hand.

Where it belongs depends on the shell and the machine, so it is printed
rather than installed. For bash on a typical Linux box:

```sh
# system-wide
rpi-loader completions bash | sudo tee /usr/share/bash-completion/completions/rpi-loader

# or just for you
mkdir -p ~/.local/share/bash-completion/completions
rpi-loader completions bash > ~/.local/share/bash-completion/completions/rpi-loader
```

Open a new shell, and `rpi-loader sd-<TAB>` completes. `zsh`, `fish`,
`elvish` and `powershell` are the other accepted arguments.

The bash script goes one step further than clap can: `--device <TAB>`
offers the serial ports that are actually attached, by asking
`rpi-loader list` at the moment you press Tab. clap knows only that
`--device` takes a string, which would otherwise complete every filename
on the machine and leave you to type `/dev/tty` first. The ports are a
runtime fact, so nothing baked into a script could answer it — which is
also why this part is bash-only; the other shells get clap's script
unchanged.

A ready-made bash script also ships with each release
(`rpi-loader-<version>.bash`), for installing it without a built binary
to hand.

## Over-the-air bundles

`bundle` is the odd one out: it speaks no serial protocol at all. A board
running its own firmware — not this loader — can be sent a bundle over
the network and install it on itself, and the container that carries one
is `rpi-loader-ota`, the package in `ota/`. It is here because a wire
format needs exactly one implementation, and this is where the half that
builds one belongs.

A project describes its card in a `bundle.toml` beside its `Makefile`:

```toml
magic = "WATR"          # four ASCII bytes, matching the firmware's
name  = "water"         # -> target/water.bundle

[kernel]
source = "target/kernel7.img"
dest   = "KERNEL7.IMG"

[[files]]
source = "www"          # a directory: everything under it, recursively
dest   = "WWW"

[[files]]
source = "vendor/start.elf"
dest   = "START.ELF"
role   = "firmware"     # file (default), firmware, or config
```

```sh
# Pack it.
rpi-loader bundle                       # or: rpi-loader bundle path/to/bundle.toml

# Pack it and send it to a board, which answers with what the update
# cost its card.
rpi-loader bundle --upload http://10.0.0.5/api/v1/ota

# Pack it and write its contents onto a card in a reader, for an update
# a board cannot be sent over the network.
rpi-loader bundle --sdcard /media/you/boot
```

`--sdcard` exists for the update that cannot arrive the usual way: a build
that changes the bundle format the running firmware reads, or one that
broke networking, or a board that is not on the network yet. It unpacks
the bundle it just built rather than writing the source files again, so a
card written by hand and a board updated over HTTP carry provably the same
bytes.

It writes **only what the bundle carries**, which is deliberately less than
a card needs to boot: a manifest describes what an *update* replaces, so
the Raspberry Pi firmware, and any settings file a project leaves out on
purpose so updates do not reset it, are not written. The directory has to
exist already — a mount point does, and a typo does not.

Notes worth knowing before writing one:

- **`magic` lives in the manifest and has no command-line override.** It
  is the only thing stopping one board's update being installed on
  another, and a flag that could change it would be a way to build a
  bundle carrying the wrong one. A project shipping both a 32- and a
  64-bit build wants two magics, since nothing else in a bundle says
  which architecture its kernel is for.
- **Paths are relative to the manifest**, not to where the command runs.
- **A directory source is packed recursively and sorted**, so the same
  tree always produces the same bytes. Anything beginning with a dot is
  skipped at every level.
- **A `start*.elf` must be packed with its matching `fixup*.dat`.** They
  are released as a pair and a mismatched one does not boot, so the
  packer refuses rather than letting a board find out.
- **`--upload` is plain HTTP, with no TLS in the dependency tree.** The
  endpoint is a board on a local network.

## 32-bit and 64-bit

The loader builds for both execution states:

- **AArch32** (`kernel7.img`, loads at `0x8000`) — the default, and the
  only option on BCM2836 boards (Cortex-A7, 32-bit only).
- **AArch64** (`kernel8.img`, loads at `0x80000`) — for BCM2837
  (Cortex-A53) and BCM2711 (Cortex-A72) boards, selected by
  `arm_64bit=1` in `config.txt`.

The upload protocol, chunking, checksums, and UART driver are shared;
only the boot stub (`firmware/src/boot.s` vs `boot64.s`) and load address
differ. A loaded kernel runs in the *same* execution state as the
loader, so a `kernel8.img` loader hands off to AArch64 kernels and a
`kernel7.img` loader to AArch32 kernels — pick the build matching the
kernels you intend to upload.

## How it works

1. GPU firmware loads this at the kernel load address for the selected
   execution state — `0x8000` for AArch32 (`kernel7.img`) or `0x80000`
   for AArch64 (`kernel8.img`). The steps below describe the 32-bit
   case; the 64-bit path is identical with those addresses swapped and
   `boot64.s` in place of `boot.s`.
2. Its own `_start` (in `firmware/src/boot.s`, not the shared one from
   `rpi-hal` — see below) immediately copies the whole running image
   to `0x00200000` and jumps into that copy, since a kernel it may be
   asked to load also expects to run at `0x8000` and can't be safely
   written there while this loader is still executing there. The stack
   is placed above the relocated region by the linker script (the
   `__stack_top` symbol), so it clears the loader's own code and data
   no matter how large the image grows.
3. From the relocated copy: brings up UART0, then blocks waiting for
   the host's `HELLO` — so the Pi can be powered on before the host
   tool starts, or vice versa. It answers with an `ACK` + version byte.
4. Enters a command loop, servicing one command at a time and returning
   for the next. `mem-write` reads a header (`total_size`, `chunk_size`,
   `load_addr`, `overall_checksum`), sanity-checks `load_addr` against
   the ~2MB gap before `0x00200000`, receives the payload as
   CRC-checked chunks, and re-verifies the whole thing. The `sd-*`
   commands bring the card up on demand and read/write/list files on
   the FAT boot partition.
5. On `exec`, jumps to the requested address — running a freshly
   uploaded kernel's own `_start` exactly as if it had booted from an
   SD card. This is the only command that doesn't return to the loop.

Because the loader outlives any single host invocation, each `rpi-loader`
subcommand is its own process that reconnects with a fresh `HELLO`; the
command loop re-answers it, so the Pi is power-cycled only to re-flash
the loader itself, never between commands.

This is single-core work throughout: the GPU firmware only ever
releases core 0 to a loaded image, holding cores 1-3 in its own stub
until they're explicitly woken. A multi-core kernel loaded this way
wakes them itself, straight out of that firmware stub, exactly as it
would if the firmware had loaded it directly — so the loader has
nothing to do for the other cores.

## Why its own `_start`

`rpi-hal` provides a standard (non-relocating) `_start` by default,
gated behind its `rt` feature. This loader needs a fundamentally
different boot sequence, so it depends on `rpi-hal` with
`default-features = false` and supplies its own via
`firmware/src/boot.s`.

## Flashing the loader (one-time)

Get an image either way — download a released one, or build it — then
copy it to the SD card as described under "Onto the card" below.

### Download a released image

Each release carries five images, one per board and execution state.
Pick the one matching yours:

| Asset | Board | Execution state |
| --- | --- | --- |
| `rpi-loader-<version>-bcm2837-kernel7.img` | Pi 2 v1.2, Pi 3 | AArch32 |
| `rpi-loader-<version>-bcm2837-kernel8.img` | Pi 2 v1.2, Pi 3 | AArch64 |
| `rpi-loader-<version>-bcm2711-kernel7.img` | Pi 4 | AArch32 |
| `rpi-loader-<version>-bcm2711-kernel8.img` | Pi 4 | AArch64 |
| `rpi-loader-<version>-bcm2835-kernel.img` | Pi 1, Pi Zero | ARMv6 |

```sh
VERSION=0.1.0
BASE=https://github.com/joeferner/rpi-loader/releases/download/v$VERSION
curl -LO $BASE/rpi-loader-$VERSION-bcm2837-kernel8.img
curl -LO $BASE/SHA256SUMS
sha256sum -c --ignore-missing SHA256SUMS
```

The assets carry the version and chip in their names because a release
page cannot hold several files all called `kernel7.img` — but the Pi's
firmware loads *only* the bare names, and only the one its CPU selects,
so rename on the way to the card:

```sh
cp rpi-loader-$VERSION-bcm2837-kernel8.img /path/to/boot/kernel8.img
```

### Or build one

For a 32-bit (AArch32) loader:

```sh
make build-bcm2837     # -> firmware/target/kernel7.img
```

For a 64-bit (AArch64) loader:

```sh
make build64-bcm2837   # -> firmware/target/kernel8.img
```

Both have `-bcm2711` counterparts for Pi 4 boards. Run `make` from the
repository root; it drives cargo inside `firmware/`, which is where the
bare metal target and toolchain are pinned. Everything builds on stable.

For a Pi 1 or Pi Zero:

```sh
make build-bcm2835     # -> firmware/target/kernel.img
```

`kernel.img`, with no digit: `start.elf` picks the kernel filename from
the CPU it finds, so a board handed a `kernel7.img` looks for a file that
is not there and stops, with nothing on the console to say so. There is
no 64-bit counterpart — the ARM1176 has no 64-bit mode. This is also the
one recipe here that needs nightly, because `armv6-none-eabi` is a tier-3
target with no precompiled `core`:

```sh
rustup toolchain install nightly --component rust-src --component clippy
```

`clippy` is in there for `make clippy6`; `--profile minimal` installs
only what is named, so a minimal nightly without it fails that recipe
rather than the build.

### Onto the card

Copy the image to an SD card's boot partition (FAT32, marked bootable),
named `kernel7.img` or `kernel8.img`, alongside `bootcode.bin`,
`start.elf`, and `fixup.dat` from the
[`raspberrypi/firmware`](https://github.com/raspberrypi/firmware/tree/master/boot)
repository's `boot/` directory.

- For `kernel7.img`, no `config.txt` is needed — the firmware defaults
  to loading `kernel7.img` on multicore ARMv7 boards.
- For `kernel8.img`, add a `config.txt` containing `arm_64bit=1` so the
  firmware boots the board in AArch64 and loads `kernel8.img`.

## Serial port access without sudo

By default, `/dev/ttyUSB*`/`/dev/serial/by-id/...` is owned by `root`
plus a system group (`uucp`, `dialout`, etc. depending on distro) —
without setup, both `rpi-loader` and a plain terminal (`picocom`, etc.)
need `sudo` to open it. Install the udev rule here once instead:

```sh
sudo cp udev/60-ftdi-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger
```

If it still doesn't work, create the group and add yourself to it, then
**fully log out and back in** (group membership doesn't apply to
already-running sessions):

```sh
sudo groupadd --system plugdev
sudo usermod -aG plugdev $USER
```

Unplug and replug the cable after either step.

## Driving the loader after that

The host tool is the `rpi-loader` CLI in `cli/`:

```sh
cargo install rpi-loader          # or: cd cli && cargo build --release
```

The serial device is given with `--device`, accepted on either side of
the subcommand:

```sh
# Which cable is it?
rpi-loader list          # -> /dev/ttyUSB0  FTDI TTL232R-3V3  0403:6001  serial ...

DEV=/dev/serial/by-id/usb-FTDI_TTL232R-3V3_*

# Upload a kernel and boot it (the classic flow), then act as a terminal
rpi-loader --device $DEV boot --load-addr 0x8000 path/to/kernel7.img

# Files on the SD card's FAT boot partition
rpi-loader --device $DEV sd-list /
rpi-loader --device $DEV sd-read /config.txt ./config.txt
rpi-loader --device $DEV sd-write ./app.bin /APP.BIN
rpi-loader --device $DEV sd-delete /APP.BIN
rpi-loader --device $DEV sd-mkdir /LOGS

# The board's ID EEPROM, on the HAT bus (GPIO0/1)
rpi-loader --device $DEV eeprom-write ./myboard.eep
rpi-loader --device $DEV eeprom-read ./readback.eep

# Lower-level memory control
rpi-loader --device $DEV mem-write 0x8000 path/to/kernel7.img
rpi-loader --device $DEV exec 0x8000 --terminal

# Watch a running kernel (no handshake)
rpi-loader --device $DEV terminal
```

`boot` has no default load address: pass `--load-addr 0x8000` for a
32-bit `kernel7.img` or `--load-addr 0x80000` for a 64-bit
`kernel8.img`. The device validates the address against its own memory
map, so a mismatch fails loudly rather than landing an image somewhere
harmful.

The handshake and terminal always run at 115200; the bulk transfers
(`boot`, `mem-write`, `sd-read`, `sd-write`) negotiate up to a faster
baud (1500000 by default) so larger transfers move quickly, then always
drop back to 115200 before returning — so the loader is left listening
at the rate the next invocation (and a booted kernel) comes up at. Pass
`--baud 115200` to disable the speedup, or a lower value if a marginal
cable or long wiring corrupts chunks faster than the retry loop can
recover:

```sh
rpi-loader --device $DEV boot --load-addr 0x8000 --baud 921600 kernel7.img
```

For a 64-bit loader, pass the AArch64 load address so the image lands
where `kernel8.img`-style binaries expect to run:

```sh
rpi-loader --device $DEV boot --load-addr 0x80000 path/to/kernel8.img
```

The `sd-*` commands operate on the first MBR partition (the Pi's boot
FAT partition on a stock card) and support nested paths. Large transfers
are slow — see "Limitations" below.

### Proving out the 64-bit path

`firmware/scripts/test_payload64.s` is a tiny AArch64 payload that just
prints a line over UART0 and parks — enough to confirm the whole 64-bit
chain (firmware → AArch64 boot stub → relocation → receive → jump)
without a full 64-bit kernel:

```sh
make build64-bcm2837                  # flash firmware/target/kernel8.img (with arm_64bit=1)
firmware/scripts/build_test_payload.sh   # -> firmware/target/test_payload64.bin
rpi-loader --device <device> boot --load-addr 0x80000 \
    firmware/target/test_payload64.bin
```

Seeing `[rpi-loader: 64-bit payload running]` in the passthrough
terminal confirms the handoff.

### The ID EEPROM

`eeprom-write` and `eeprom-read` reach a serial EEPROM on BSC0's GPIO0/1
routing — `ID_SD`/`ID_SC`, pins 27 and 28 of the 40-pin header — which is
where a board keeps its own identity: the HAT specification's image
(vendor, product, UUID, GPIO map, an optional device tree overlay), and
whatever a design adds beside it, such as per-unit calibration. The
firmware reads that EEPROM early in boot and then leaves the bus alone,
so the loader has it to itself.

```sh
# Program an image produced by Raspberry Pi's `eepmake`, then read it back
rpi-loader --device $DEV eeprom-write ./myboard.eep
rpi-loader --device $DEV eeprom-read ./readback.eep
cmp ./myboard.eep ./readback.eep
```

Four things worth knowing:

- **The write is verified on the device.** Every page is read back after
  it is programmed, and a mismatch fails the command. This is not
  belt-and-braces: a write-protected part (`WP` tied high, which on a
  board that puts write protect on a solder jumper is the default state)
  acknowledges every byte and stores none, so without the read-back a
  write to a protected EEPROM would report a clean success.
- **`--page-size` must not exceed the part's own page.** The default, 32
  bytes, is the page of a 24C32 — the smallest part the HAT specification
  allows — and divides every larger part's page, so it is always safe. A
  24C256's page is 64 bytes and programs in half the time. Naming one too
  large corrupts data rather than failing: a page write that runs past the
  boundary wraps to the start of the same page instead of carrying.
- **Addressing is two bytes**, so 64 KiB is the ceiling, and parts below
  the 24C32 (which address with one byte) are not supported.
- **`--address` defaults to `0x50`**, what the HAT specification assigns
  the ID EEPROM. Another part on the same bus can be reached by naming
  its address.

Producing the `.eep` image is a separate job, and the Raspberry Pi
`hats` repository's `eepmake` is the tool for it: it turns a text
settings file into the binary this command programs.

## Limitations

- **`sd-read`/`sd-write` are slow**, and noticeably so on files of any
  size. Two independent reasons, neither of them a missing driver
  feature. `rpi-hal` does multi-block transfers (`CMD18`/`CMD25` with an
  auto-`CMD12` stop) and its `embedded-sdmmc` adapter uses them whenever
  it is handed more than one block — but `embedded-sdmmc`'s block cache
  holds exactly one block, so through the filesystem it never is, and
  every 512 bytes costs its own SD command. Separately, the transfer is
  lockstep: the device defers each chunk's `OK` until it has finished
  writing that chunk. That deferral is not an oversight — it is the flow
  control keeping the UART's 16-byte RX FIFO from overflowing — but it
  leaves the link idle for the whole of every SD write.
- **Single-core throughout.** The loader never touches cores 1-3; the
  GPU firmware holds them in its own stub, and a multicore kernel loaded
  this way wakes them itself exactly as it would if the firmware had
  loaded it directly.
- **A loaded kernel runs in the same execution state as the loader**, so
  a 32-bit loader cannot boot a 64-bit kernel or the reverse. Flash the
  build matching the kernels you intend to upload.

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
