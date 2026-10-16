---
title: Getting started
description: From nothing to a heartbeat on the serial console of a Raspberry Pi, with no operating system.
---

By the end of this page a Raspberry Pi is running your own Rust program,
with no operating system, and printing to your terminal over a serial
cable. After the first time, every rebuild goes to the board over that
same cable: the SD card stays in.

## What you need

- A Raspberry Pi: a 1, Zero, 2, 3 or 4. See [Boards](../boards/) for
  exactly which.
- A 3.3 V USB-to-serial cable. Its RX goes to GPIO14 (pin 8), its TX to
  GPIO15 (pin 10), and ground to pin 6.
- An SD card with one FAT32 partition.
- Rust, installed with [rustup](https://rustup.rs). The project pins the
  toolchain and target it needs, and rustup installs them on the first
  build.
- Two cargo tools:

  ```sh
  cargo install cargo-binutils rpi-loader
  ```

  `cargo objcopy` (from cargo-binutils) turns a build into the raw image
  the Pi's firmware loads, and `rpi-loader` uploads it to the board.

## Get a project

Either download one from [Start a project](../start/), or generate it in
the terminal:

```sh
cargo install cargo-generate
cargo generate --git https://github.com/joeferner/rpi-hal template
```

Both give the same project. The zip also includes the files for the SD
card; with `cargo generate`, run `make sd-card` to fetch them.

## Set up the card

Copy everything in the project's `sd-card/` folder onto the card's FAT32
partition:

- the Raspberry Pi boot firmware for your board;
- `config.txt`, which tells the firmware how to start your board;
- the rpi-loader, named after the one file your board's firmware loads
  (`kernel.img`, `kernel7.img` or `kernel8.img`).

Put the card in the Pi, connect the serial cable, and power it on. The
loader starts and waits for the host. It stays on the card from now on.

## Build and run

```sh
make run DEVICE=/dev/ttyUSB0
```

This builds the project, uploads the image over the cable, and boots it.
The heartbeat then appears on the same cable:

```
my-pi-app: hello from the Raspberry Pi 3 (B, B+, A+)
up 1s
up 2s
```

That's the whole loop: edit `src/main.rs`, run `make run` again.

To run without the loader, copy the build (`target/kernel7.img`, or
whichever name your board loads) onto the card in place of the loader's.

## Where next

- [rpi-kickstart](../crates/kickstart/) has what a board usually needs
  next: a heap, a settings file on the card, networking, a web server, and
  OTA updates, each behind a feature.
- [rpi-hal](../crates/hal/) has the peripherals.
- rpi-hal's
  [own guide](https://github.com/joeferner/rpi-hal/blob/main/hal/docs/getting-started.md)
  covers running its examples, wiring, and preparing a blank card from
  scratch.
