# Getting started: run rpi-hal examples on Pi 1/2/3/4 and Pi Zero

See the [`examples/`](../examples) directory for what's available. Each
example is a single file with a module-level doc comment describing what
it exercises and how to tell it's working.

All examples use the same SD card setup — only the file you build and
copy onto it differs.

## What you need

- A Raspberry Pi 2 (BCM2836), Pi 3 (BCM2837 — the register map is
  identical to the Pi 2's), Pi 4 (BCM2711), or Pi 1 / Pi Zero
  (BCM2835). Which chip you have selects the
  `bcm2837`/`bcm2711`/`bcm2835` Cargo feature (see "Cargo setup"
  below) and which boot firmware files go on the SD card (see
  "Prepare the SD card" below). Not every example builds against
  `bcm2711` yet — check the example's own `required-features` in
  `Cargo.toml` against
  [issue #29](https://github.com/joeferner/rpi-hal/issues/29), which
  tracks what is and isn't verified on that chip. `bcm2835` is also
  partial, and differs in kind: it is ARMv6, so it needs a different
  target and a nightly toolchain, and `multicore`/`generic_timer`/`pmu`
  do not exist on it at all.
- A microSD card and a way to write it from your host machine.
- An LED, a ~330Ω resistor, and two jumper wires (or a breadboard) for
  examples that drive GPIO.
- A 3.3V-logic USB-to-TTL serial (FTDI-style) cable, and a serial
  terminal program on the host (`screen`, `minicom`, or `picocom`), for
  examples that use UART.
- A single jumper wire from GPIO10 (MOSI, physical pin 19) to GPIO9
  (MISO, physical pin 21) for `spi_loopback` — no external SPI device
  needed.
- [`cargo-binutils`](https://github.com/rust-embedded/cargo-binutils),
  used to turn the compiled ELF into a raw binary:
  `cargo install cargo-binutils`.
- _Recommended_ USB Power Switch - Provides an easy way to reboot your
  Raspberry Pi without unplugging and re-plugging USB cables each time
  you want to change code.

Check the example you're building for its exact hardware requirements.

## Faster iteration: rpi-loader

Rewriting the SD card for every build gets tedious fast.
[`rpi-loader`](https://github.com/joeferner/rpi-loader) is a
UART-based upload loader (in the style of
[raspbootin](https://github.com/mrvn/raspbootin)) that lets you push a
new build over the serial cable instead. It's a separate project from
`rpi-hal` — see its own repo for setup and usage. This guide covers the
plain-SD-card path below, which still works and needs no extra setup
beyond what's already on the card.

## Wiring the LED

Use an external LED rather than the onboard ACT LED — its GPIO pin
and on/off polarity have been reported inconsistently across board
revisions and sources, which makes it a bad first sanity check.

- GPIO4 → resistor → LED anode
- LED cathode → Ground

On the 40-pin header, GPIO4 is physical pin 7; any Ground pin works
(e.g. physical pin 6 or 9).

## Wiring UART (FTDI-style cable)

Uses UART0 (PL011) on GPIO14 (TXD0, physical pin 8) and GPIO15 (RXD0,
physical pin 10).

**Use a 3.3V-logic cable.** Many FTDI-style cables (e.g. the common
TTL-232R-3V3) also expose a 5V or VBUS pin — never connect that to the
Pi.

A standard FTDI TTL-232R-3V3 cable's wire colors:

| Cable wire | Signal            | Connect to                           |
| ---------- | ----------------- | ------------------------------------ |
| Black      | GND               | Any Ground pin (e.g. physical pin 6) |
| Orange     | TX (cable output) | GPIO15 / physical pin 10 (Pi RX)     |
| Yellow     | RX (cable input)  | GPIO14 / physical pin 8 (Pi TX)      |
| Red        | VCC (3.3V)        | **Leave disconnected**               |

Note the cross-over: the cable's TX goes to the Pi's RX pin, and vice
versa. If your cable uses different colors, check its datasheet for
which wire is TX vs RX rather than assuming this table's colors apply.

Find which device the cable enumerated as via its stable by-id path
(survives reboots/replugs, unlike `/dev/ttyUSBn` numbering):

```sh
ls -l /dev/serial/by-id/
```

Then open a serial terminal at 115200 8N1:

```sh
picocom -b 115200 /dev/serial/by-id/usb-FTDI_TTL232R-3V3_*
```

## Prepare the SD card

All the Pi's boot firmware needs is a single FAT32 partition marked
bootable. If you're starting from a blank card (not one already set up
by Raspberry Pi OS or the Imager tool), partition and format it first.

**Identify the device carefully before running anything below** — a
wrong device name will happily partition your main disk. `fdisk`/`mkfs`
give no confirmation prompt.

```sh
lsblk                 # note current drives
# insert the SD card
lsblk                 # find the new entry, e.g. /dev/sdX — a small
                       # removable device matching the card's capacity
```

Use the whole-disk device (`/dev/sdX`), not a partition (`/dev/sdX1`).
If anything auto-mounted, unmount it first:

```sh
sudo umount /dev/sdX*
```

Partition with `fdisk`:

```sh
sudo fdisk /dev/sdX
```

At the `fdisk` prompt:

- `o` — new empty DOS partition table
- `n` then `p` then `1`, accept the default first/last sector — one
  partition spanning the whole card
- `t` then `c` — set the partition type to W95 FAT32 (LBA)
- `a` — mark it bootable
- `w` — write and exit

Format and mount it:

```sh
sudo mkfs.vfat -F 32 -n RPIBOOT /dev/sdX1
mkdir -p /tmp/rpiboot
sudo mount /dev/sdX1 /tmp/rpiboot
```

Now populate it. The firmware files differ by board:

- **Pi 2/3:** download `bootcode.bin`, `start.elf`, and `fixup.dat`
  from the
  [`raspberrypi/firmware`](https://github.com/raspberrypi/firmware/tree/master/boot)
  repository's `boot/` directory, and copy all three into
  `/tmp/rpiboot`. No `config.txt` is required — the firmware defaults
  to loading `kernel7.img` on multicore ARMv7 boards, which matches
  the output of the build script below.
- **Pi 4:** download `start4.elf` and `fixup4.dat` from the same
  `boot/` directory instead — Pi 4's on-board boot EEPROM replaces
  `bootcode.bin`'s job, so that file isn't used (and is ignored even
  if present). Pi 4 defaults to loading `kernel7l.img` (32-bit, LPAE),
  not `kernel7.img`, so add a `config.txt` alongside the firmware
  files with:

  ```ini
  kernel=kernel7.img
  ```

  (64-bit builds don't need this — `kernel8.img` is already the
  default kernel filename on both Pi 3 and Pi 4.)

- **Pi 1 / Pi Zero:** the same three files as Pi 2/3 (`bootcode.bin`,
  `start.elf`, `fixup.dat`; they serve every BCM2835/6/7 board). Two
  things about this board are silent failures, so both are worth
  getting right before wondering whether the code works:

  **The image must be named `KERNEL.IMG`, with no digit.** `start.elf`
  picks the kernel filename from the CPU it finds: `kernel.img` on
  ARMv6, `kernel7.img` on ARMv7, `kernel7l.img` on the Pi 4's 32-bit
  mode, `kernel8.img` on AArch64. A card carrying only `kernel7.img`
  leaves this board looking for a file that isn't there, and it stops —
  no ARM code ever runs, so the console is silent rather than garbled,
  which looks exactly like a dead board. `build-example.sh <name>
  bcm2835` emits `target/kernel.img` for this reason.

  **`config.txt` should pin the UART clock:**

  ```ini
  init_uart_clock=48000000
  disable_splash=1
  ```

  `Uart::init` programs its divisors for a 48MHz PL011 reference clock
  and has no way to discover it was wrong, so a firmware that chose
  otherwise scales every bit period and hands back a console that looks
  like noise at a plausible baud rate. Don't add `enable_uart=1`: that
  asks the firmware for a console of its own on the same GPIO14/15 pins
  the cable is on. There is no `arm_64bit` line to set either way —
  this core has no 64-bit mode, and a stray `arm_64bit=1` carried over
  from a Pi 3 card sends the firmware looking for `kernel8.img`.

  The on-board Bluetooth on a Zero W claims the PL011 by default,
  leaving the mini UART on the header pins. Nothing needs to be done
  about it: `Uart::init` remuxes GPIO14/15 to ALT0 itself, taking the
  header back for the PL011 whatever the firmware decided.

After copying the kernel image too (next section), unmount before
removing the card:

```sh
sudo umount /tmp/rpiboot
```

## Build and flash

```sh
./scripts/build-example.sh <example-name>            # Pi 2/3, see examples/ for the available names
./scripts/build-example.sh <example-name> bcm2711    # Pi 4
./scripts/build-example.sh <example-name> bcm2835    # Pi 1, Pi Zero
```

This produces `target/kernel7.img` — or `target/kernel.img` for
`bcm2835`, which also builds for ARMv6 and so needs the nightly
toolchain and `rust-src` described in README.md's "Toolchain". Not
every example builds for that chip: the ones resting on hardware it
does not have (the ARM generic timer, a second core) are excluded, and
`make examples` lists the set that is known to build.

Copy the image to the SD card's boot partition alongside the firmware
files from "Prepare the SD card" above (`bootcode.bin`/`start.elf`/
`fixup.dat` for Pi 1/2/3/Zero, `start4.elf`/`fixup4.dat` plus
`config.txt` for Pi 4), eject safely, and boot the Pi.

## Building your own application against rpi-hal

The examples above live inside this crate. To write your *own*
application in a separate crate, depend on `rpi-hal` the way any external
consumer would. The setup mirrors what the examples get for free, plus
one line pointing the linker at the script this crate publishes.

### Cargo setup

A `no_std`, `no_main` binary crate on stable Rust 1.88 or newer. Pin the
target with a `rust-toolchain.toml`, which also installs it:

```toml
[toolchain]
channel = "stable"
# 32-bit (Pi 2/3/4, kernel7.img):  "armv7a-none-eabi"
# 64-bit (Pi 3/4, kernel8.img):    "aarch64-unknown-none-softfloat"
# ARMv6 (Pi 1/Zero, kernel.img):   "armv6-none-eabi" -- see the note
#   below; it goes in [build], never here.
targets = ["aarch64-unknown-none-softfloat"]
```

and a `.cargo/config.toml` that selects it, so plain `cargo build` targets
the board instead of the host:

```toml
[build]
target = "aarch64-unknown-none-softfloat"
```

That's the whole toolchain requirement — `rustup` ships a precompiled
`core` and `alloc` for both targets, so nothing here needs nightly or
`-Zbuild-std`, including a binary that declares a `#[global_allocator]`
and uses `alloc`. (This crate's own repository builds its examples with
nightly and `build-std = ["core", "alloc"]`, but that's a local choice
rather than a requirement to copy.)

The Pi 1 / Pi Zero is the exception, and it is the target's doing rather
than this crate's: `armv6-none-eabi` is tier 3, so there is no
precompiled `core` to install and `-Zbuild-std` has to compile one,
which needs nightly. See README.md's "Toolchain" for the three files.
Note that the target must *not* be listed under `targets` in
`rust-toolchain.toml` — rustup would try to download a standard library
that is not published, and fail.

Install [`cargo-binutils`](https://github.com/rust-embedded/cargo-binutils)
(`cargo install cargo-binutils`) for the `cargo objcopy` step below.

Depend on `rpi-hal` from crates.io. Its default features `rt` (the
`_start`/panic/IRQ boot sequence) and `mmu` are what a normal
application wants. None of the chip features is a default, though —
pick `bcm2837` (Pi 2/3), `bcm2711` (Pi 4) or `bcm2835` (Pi 1, Pi Zero)
explicitly, matching the board you're targeting; add other integration
features (`smoltcp`, `embedded-sdmmc`, `multicore`, …) as needed:

```toml
[dependencies]
rpi-hal = { version = "0.6", features = ["bcm2837"] }
```

If you are writing a library on top of this one rather than an
application, do **not** name a chip feature on that line. It makes the
choice unconditional for everyone downstream, and the chip features have
a precedence order (`bcm2711` > `bcm2837` > `bcm2835`), so a consumer
who adds a lower-precedence one gets the wrong memory map silently
rather than an error. Give your own crate a feature per chip and forward
it, defaulting to whichever board you expect:

```toml
[features]
default = ["bcm2837"]
bcm2837 = ["rpi-hal/bcm2837"]
bcm2835 = ["rpi-hal/bcm2835"]

[dependencies]
rpi-hal = { version = "0.6", default-features = false }
```

### Linker script (required)

A bare-metal binary needs a linker script that places the image at the
firmware's load address and brackets `.bss` with the
`__bss_start`/`__bss_end` symbols rpi-hal's boot code zeroes. Without one
the link fails with `undefined symbol: __bss_end`.

You don't have to write it. `rpi-hal` publishes its own as `rpi-link.x`,
already pointing at the load address for whichever target you're building,
and puts it on the linker's search path — so naming it is the whole step:

```toml
# .cargo/config.toml
[target.aarch64-unknown-none-softfloat]
rustflags = ["-C", "link-arg=-Trpi-link.x"]
```

That's one entry per target you build for, alongside the `[build] target`
above; the `-T` line itself is the same for both architectures, since the
32-bit/64-bit choice is already baked into what `rpi-link.x` contains
(`0x8000` for a `kernel7.img`, `0x80000` for a `kernel8.img`). No build
script is involved.

Prefer this to a copy of the script in your own crate. A copy keeps
whatever rpi-hal's boot code expected on the day you copied it, and the
expectations do change — the `ALIGN(4)` that `.bss` zeroing needs was added
after the first scripts were written, and an unaligned `__bss_start`
data-aborts during boot with no vector table installed to catch it.

If you do need your own — a self-relocating loader, or a non-default load
address — pass it by absolute path from a `build.rs`
(`cargo:rustc-link-arg=-T{abs path}`) instead of by bare name, so it can't
be shadowed, and don't also pass `-Trpi-link.x`.

One thing to know either way: `rustflags` in `.cargo/config.toml` is
ignored entirely if a `RUSTFLAGS` environment variable is set, and the
symptom is the same `undefined symbol: __bss_end` as having no script at
all.

The stack grows down from the load address, so it is as large as that
address: ~512 KiB at `0x80000` (64-bit), but only ~32 KiB at `0x8000`
(32-bit). Keep large, long-lived buffers in `static`/`.bss` rather than
on the stack — a big on-stack buffer can overflow the 32-bit stack and
hang the board mid-bring-up.

### Build the image

Same `objcopy` step the example scripts use, against your binary:

```sh
cargo build --release
cargo objcopy --release -- -O binary target/kernel8.img
```

### Run it

Either path from "Build and flash" / "Faster iteration" above applies:

- **SD card:** copy `target/kernel8.img` (with `arm_64bit=1` in
  `config.txt`), `target/kernel7.img`, or `target/kernel.img` on ARMv6,
  to the boot partition and boot. The name is not a convention here —
  the firmware picks which one it looks for from the CPU, and finding
  none it stops silently.
- **rpi-loader:** upload over UART, matching the link address —
  `--load-addr 0x80000` for a 64-bit image, `0x8000` for either 32-bit
  one.
