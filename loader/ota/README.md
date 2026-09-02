# rpi-loader-ota

[![CI](https://img.shields.io/github/actions/workflow/status/joeferner/rpi-loader/ci.yml?branch=main&label=CI)](https://github.com/joeferner/rpi-loader/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/rpi-loader-ota.svg)](https://crates.io/crates/rpi-loader-ota)

The over-the-air update bundle a bare-metal Raspberry Pi installs on
itself: one container, packed on a host and validated on the device.

A bundle carries a kernel image and the files that ship beside it under a
single checksum, so a board can reject a damaged transfer before it writes
anything to its card. The kernel goes to the boot partition, the rest
alongside it.

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
