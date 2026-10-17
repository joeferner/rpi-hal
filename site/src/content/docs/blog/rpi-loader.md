---
title: "rpi-loader: stop moving the SD card"
date: 2026-10-16
excerpt: Flash one small image to the card once, then upload every build over a serial cable.
tags:
  - rpi-loader
---

Every bare-metal Raspberry Pi project starts with the same loop. Build.
Power the board off. Pull the SD card, put it in the computer, copy
`kernel7.img` over, eject it, put it back, power on, and watch the
serial console to see whether it worked. Then do it all again for a
one-line change.

It is slow, it wears out the card and the slot, and it is miserable once
the board lives in an enclosure. A microcontroller has SWD or a USB
bootloader for exactly this; a Pi has nothing of the kind.
[rpi-loader](/rpi-hal/crates/loader/) is that missing piece.

## The idea

Put one small image on the card, once. From then on the Pi boots into a
loader that sits on the UART waiting to be told what to do, and the card
never has to move again.

It is a resident command agent rather than a one-shot bootloader: it
answers one command, then waits for the next, for as long as the board
is powered. Each run of the host tool is its own process and reconnects
with a fresh handshake, so the order you start things in doesn't matter.
Power the Pi first or start the tool first; either way they find each
other.

## What it looks like

Find the cable:

```sh
rpi-loader list
# /dev/ttyUSB0  FTDI TTL232R-3V3  0403:6001  ...
```

Upload a build to RAM, jump to it, and stay attached as a terminal, so
the program's output appears where you typed the command:

```sh
rpi-loader --device /dev/ttyUSB0 boot --load-addr 0x8000 target/kernel7.img
```

That is the everyday loop, and it is what a project from the
[project selector](/rpi-hal/start/) runs as `make run`.

To leave the terminal, press **Ctrl-]**, not Ctrl-C. Every other key,
Ctrl-C included, goes to the board, which is what lets you interrupt
something running *there* rather than the tool on your machine.

The loader can also reach the SD card in place, so editing `config.txt`
doesn't mean touching the card either:

```sh
rpi-loader --device /dev/ttyUSB0 sd-read /config.txt ./config.txt
# edit it, then put it back
rpi-loader --device /dev/ttyUSB0 sd-write ./config.txt /config.txt
```

There are a few more commands: listing, deleting and making directories
on the card, a plain terminal for watching a kernel that is already
running, and programming the ID EEPROM a HAT or a custom board carries.
The [README](https://github.com/joeferner/rpi-hal/blob/main/loader/README.md)
has the full list.

## Getting it

The host tool is on crates.io:

```sh
cargo install rpi-loader
```

The loader images are on the
[release page](https://github.com/joeferner/rpi-hal/releases/latest),
or already inside a project zip from the
[project selector](/rpi-hal/start/), with the boot firmware and a
`config.txt` beside them. Copy those to the card, connect a 3.3 V
USB-serial cable to GPIO14 and GPIO15 (pins 8 and 10, ground on pin 6),
and power on. The [getting started](/rpi-hal/getting-started/) guide
walks through it.

After that the card stays in the board.

### What goes on the card

The card needs one FAT32 partition and only a handful of files:

| File | What it is |
| --- | --- |
| `bootcode.bin` | The first stage of the Pi's boot firmware, which brings up SDRAM and loads `start.elf`. Not on a Pi 4, whose boot EEPROM does this job. |
| `start.elf` (`start4.elf` on a Pi 4) | The GPU firmware proper. It reads `config.txt`, sets up the hardware, loads the kernel image and releases the ARM cores. |
| `fixup.dat` (`fixup4.dat` on a Pi 4) | Tells `start.elf` how memory is split between the GPU and the ARM. Released as a pair with `start.elf`; mismatched, the board won't boot. |
| `LICENCE.broadcom` | The licence the firmware files are distributed under. |
| `config.txt` | The firmware's settings for your board: 32- or 64-bit, which kernel file to load, and anything else the board needs. |
| `kernel7.img`, `kernel8.img` or `kernel.img` | The loader, renamed to the one filename your board's firmware loads. Which name depends on the CPU, and, on a Pi 3 or Pi 4, on `arm_64bit` in `config.txt`. |

The first four come from Raspberry Pi's
[firmware repository](https://github.com/raspberrypi/firmware), and a
project zip has all six with the right names already.

## How it works

### Getting out of the way

The Pi's firmware loads every kernel at the same address: `0x8000` for a
32-bit image, `0x80000` for a 64-bit one. That includes the loader, which
is a problem, because the kernel it is about to receive expects that
address too, and nothing can overwrite the code it is running from.

So the loader's first act, before anything else, is to copy itself to
`0x00200000` and jump into the copy. It is linked to run from there, and
its stack sits above it, which leaves the whole region from the load
address up to the copy free for the incoming image.

### The wire

The handshake runs at 115200 baud, which every USB-serial cable manages.
For bulk transfers the two ends then agree on something faster (1.5
Mbaud by default) and drop back to 115200 afterwards, so the next
command, or the program just booted, finds the link where it expects.

Data moves in chunks, each with its own CRC and each acknowledged before
the next is sent. That is flow control, not caution: the Pi's UART holds
only 16 bytes in its receive FIFO, and a host that streamed freely would
overrun it. A bad chunk is sent again, and the whole image is checked
once more at the end.

### Refusing to do damage

The loader checks every upload's address against its own memory map, so
a wrong `--load-addr` fails with an error instead of overwriting the
loader in the middle of the transfer.

On the card, `sd-write` receives the whole file into RAM before writing
any of it, so an interrupted upload leaves the old file as it was. And
the loader keeps no state between commands: it mounts the card afresh
for each one, so resetting the board between two commands loses
nothing.

### One image per execution state

A loader hands off in the execution state it is running in, so a 32-bit
loader boots 32-bit kernels and a 64-bit loader boots 64-bit ones. With
three chips, two of which can run either way, that makes five images:

| Image | Boards |
| --- | --- |
| `bcm2837` `kernel7.img` | Pi 2, Pi 3, 32-bit |
| `bcm2837` `kernel8.img` | Pi 2 v1.2, Pi 3, 64-bit |
| `bcm2711` `kernel7.img` | Pi 4, 32-bit |
| `bcm2711` `kernel8.img` | Pi 4, 64-bit |
| `bcm2835` `kernel.img` | Pi 1, Pi Zero |

The project selector picks the right one for the board you choose, and
names it the way your board's firmware expects.

:::note[Update, 2026-10-17]
The loader now runs on the Pi 5 too, as a preliminary sixth image,
`bcm2712` `kernel8.img`, built from source until the next release (see
[supported devices](/rpi-hal/supported-devices/)). As with the Pi 4,
its card needs no `bootcode.bin`, and on the
Pi 5 it needs no `start.elf` or `fixup.dat` either: that firmware lives in
the board's EEPROM.
:::
