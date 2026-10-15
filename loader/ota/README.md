# rpi-loader-ota

[![CI](https://img.shields.io/github/actions/workflow/status/joeferner/rpi-loader/ci.yml?branch=main&label=CI)](https://github.com/joeferner/rpi-loader/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/rpi-loader-ota.svg)](https://crates.io/crates/rpi-loader-ota)

The over-the-air update bundle a bare-metal Raspberry Pi installs on
itself: one container, packed on a host and validated on the device.

A bundle is a list of **(path, role, bytes)** under one checksum, so a
board can reject a damaged transfer before it writes anything to its card.
Each entry says where it lands, so an update can replace anything on a
boot partition: a kernel, a website, a settings file, a certificate,
`config.txt`, the Raspberry Pi firmware itself — or any subset of those.
A bundle carrying **no** kernel is how a website is updated without
rewriting an image that has not changed.

This is `no_std`, and it is not Pi-specific beyond the shape of the
problem — anything that boots a raw image from a FAT partition and can
receive a few megabytes over some transport can use it.

## Why a crate

Because **the encoder and the decoder are the same code**. A wire format
described in two places is a format that has already drifted: the two
implementations this one was collected from agreed on every byte and
disagreed on what they enforced, each accepting bundles the other
rejected.

The host end is the [`rpi-loader`](https://crates.io/crates/rpi-loader)
CLI, which packs bundles and can upload one to a running board. The device
end is this crate, linked into the firmware.

## The container

Version 2 of the format, little-endian throughout:

```text
magic   : 4 bytes             per-application; see `Format`
version : u8      = 2
reserved: u8      = 0
count   : u16                 number of entries
entries : count x {
            role    : u8      file, kernel, firmware or config
            path_len: u8
            path    : [u8; path_len]
            size    : u32
            data    : [u8; size]
          }
crc32   : u32                 IEEE CRC-32 over every preceding byte
```

The **magic** is the application's own, and is what stops a bundle built
for one board installing on another — nothing else in the container says
which board it belongs to.

`Bundle::parse` **allocates nothing**. It validates in place and borrows,
so a device holds one buffer — the upload as it arrived — and nothing
else. The checksum is verified before any entry is walked, because a
corrupt transfer produces plausible-looking lengths and parsing those
first yields a misleading error.

The same rules run in both directions, so a packer cannot build what a
device would reject: no absolute paths, no `..` or empty components, no
backslashes, no duplicate destinations, at most one kernel and one
`config.txt`, and a `start*.elf` only alongside its matching
`fixup*.dat` — the two are released together and a mismatched pair does
not boot.

## Roles, and the order they install in

A role says what an entry *is*, and the installer decides what that means.
Naming the boot image by role rather than by filename or position is what
lets a device choose its own destination.

| Role | What it is |
| --- | --- |
| `File` | anything the application owns |
| `Kernel` | the boot image |
| `Firmware` | `bootcode.bin`, `start*.elf`, `fixup*.dat` |
| `Config` | `config.txt` |

`apply` writes them in **role order rather than packed order**, and the
crate imposes it rather than trusting the bundle: files, then firmware,
then `config.txt`, then the kernel. The kernel is last because with one
boot image that write *is* the commit — everything that could fail has to
have failed already. `config.txt` precedes it because losing that file
falls back to loading `kernel7.img`, so it cannot on its own stop a board
booting.

One entry has no safety net, and it is worth knowing which: **`bootcode.bin`**
is loaded by the ROM by name, so there is no slot to write it to and no
config line to point elsewhere. A failed write there means the card comes
out. It is ~50 KB, it changes almost never, and on the Pi 4 it is in SPI
EEPROM and not on the card at all.

## Packing one

The host end is the `rpi-loader` CLI, which reads a `bundle.toml`
describing what the card holds and emits the container above. See that
crate for the manifest format; it is not repeated here, because one
description of a format in two places is the thing this crate exists to
prevent.

## Features

| | Feature | Pulls in |
| --- | --- | --- |
| The container: encode, decode, checksum | `alloc` *(default)* | nothing |
| Installing one onto a FAT volume | `apply` | [`resident-fat`](https://crates.io/crates/resident-fat) |

The split keeps the host CLI from depending on a filesystem implementation
in order to write a file on a host, and keeps a device that only installs
bundles from compiling an encoder it never calls.

## What your application still supplies

**The transport and the reboot.** How a bundle arrives — an HTTP route, a
job handed to another core, a command over serial — is application shaped,
and a crate that took it would be choosing your web framework.

**The measurement.** What an update costs the card is usually the number
worth having, and collecting it here would mean depending on an async
runtime for a clock and on whatever is counting device commands. The
install reports its progress; the caller decides what to time.

## License

MIT OR Apache-2.0, at your option.
