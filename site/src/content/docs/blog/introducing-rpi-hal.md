---
title: Introducing rpi-hal
date: 2026-10-16
excerpt: Bare-metal Rust on the Raspberry Pi, with no operating system underneath.
tags:
  - rpi-hal
---

`rpi-hal` is a hardware abstraction layer for running Rust directly on a
Raspberry Pi: no Linux, no operating system of any kind, just your
program, the hardware, and a crate that knows how to talk to it. It currently
covers the Pi 1 and Pi Zero, the Pi 2, the Pi 3 and, in a preliminary
way, the Pi 4, in both 32-bit and 64-bit builds where the chip has both.
The Pi Zero 2 W and the Pi 5 are on the list for later; the
[supported devices](/rpi-hal/supported-devices/) page tracks where each
board stands.

:::note[Update, 2026-10-17]
The Pi 5 has started. A preliminary `bcm2712` feature is on `main`:
boot, the System Timer, the header GPIO and the UART console, with the
same API as the other boards, so the `blink` program below builds for a
Pi 5 unchanged. It ships with the next release.
:::

If you have written firmware for an STM32 or an RP2040 in Rust, it is
meant to feel familiar. If you haven't, it is meant to be a reasonable
place to start.

## What it is

The Rust embedded ecosystem has settled on a shape for a HAL. A
peripheral access crate (PAC) describes the registers; the HAL wraps
them in types that own the hardware they drive; and the HAL implements
the [`embedded-hal`](https://docs.rs/embedded-hal) traits, so that a
driver written against `OutputPin` or `I2c` works on any chip that
provides them. `stm32xx-hal` and `rp2040-hal` look this way, and so does
`rpi-hal`, on purpose: the same naming, the same typestate pattern (a
pin's mode is part of its type, so writing to an input doesn't compile),
and the `embedded-hal` traits implemented *beside* the crate's own
methods rather than instead of them.

Here is a complete program that blinks an LED on GPIO4:

```rust
#![no_std]
#![no_main]

use embedded_hal::digital::OutputPin;
use rpi_hal::gpio::{Input, Pin};
use rpi_hal::pac;
use rpi_hal::timer::Timer;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    rpi_hal::halt();
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    let mut led = Pin::<4, Input>::new(peripherals.GPIO).into_output();
    let timer = Timer::new(peripherals.SYSTMR);

    loop {
        let _ = led.set_high();
        timer.delay_ms(500);
        let _ = led.set_low();
        timer.delay_ms(500);
    }
}
```

Apart from `kmain` where you might expect `main`, nothing in it is
particular to a Pi. That is the goal: a basic application written
against `embedded-hal` for another chip should port with only the
setup lines changed.

What sits underneath is more than an MCU HAL usually carries, because a
Pi is not a microcontroller. `rpi-hal` brings up the MMU and the caches,
installs the exception vectors and routes interrupts, and wakes the
other three cores if you ask it to. Then come the peripherals: GPIO, the
UARTs, SPI, I2C, PWM, PCM/I2S, DMA, the SD card, USB host, the
VideoCore mailbox and more. The
[README](https://github.com/joeferner/rpi-hal/blob/main/hal/README.md)
keeps the status of every one of them.

## Why bare metal on a Pi at all

A Pi is usually a small Linux computer, and for most things that is
exactly what you want. But it is also a very capable piece of hardware:
four cores, hundreds of megabytes to a few
gigabytes of RAM, HDMI, USB host, Ethernet, and on some boards Wi-Fi and
Bluetooth. Running without an operating system changes what that
hardware is like to use:

- **The machine is entirely yours.** No scheduler preempts you, no
  kernel decides when your interrupt is serviced, no background service
  takes a core. Timing is whatever your code makes it.
- **So is the memory.** No kernel reserves a slice of it, and there is
  no process address space to fit inside: your program can use all of
  the RAM the board has, apart from the share you give the GPU in
  `config.txt`.
- **The SD card barely matters.** It doesn't carry a Linux distribution,
  only the Pi's boot firmware and your image, a few megabytes in all, so
  the smallest card you can find is plenty. And nothing writes to it
  unless your program decides to: no logs, no swap, no package cache
  wearing it out in the background. A card that is written only when you
  say so lasts far longer than one under an operating system.
- **It starts when the power does.** There's no distribution to boot, no
  filesystem to check, nothing to shut down cleanly. Pull the plug
  whenever you like: unless your program is in the middle of writing
  to the card, there is nothing on it to corrupt.
- **You see everything.** There is no driver between you and a register,
  which is as much a way to learn how the hardware actually works as it
  is a way to control it.

The price is that there is no driver between you and a register. Linux
gives you a working USB stack, a network stack and a filesystem for
free; on bare metal someone has to write them. Doing that work once, in
crates other people can build on, is what this project is for.

One thing bare metal does *not* get rid of is the Pi's own boot
firmware. On every Pi the GPU starts first, reads `bootcode.bin`,
`start.elf` and `config.txt` from the SD card, and only then loads your
image and releases the ARM cores. `rpi-hal` takes over from that point.

## The family

`rpi-hal` is the foundation. Three more crates live beside it in the
same repository:

- **[rpi-hal-embassy](/rpi-hal/crates/embassy/)** brings
  [Embassy](https://embassy.dev) to the Pi: a time driver on the System
  Timer, an executor that sleeps with `wfe` and wakes on `sev`, and
  `embassy-net` drivers for the onboard Ethernet. Async Rust, with tasks
  instead of a main loop.
- **[rpi-kickstart](/rpi-hal/crates/kickstart/)** is the bring-up every
  application ends up writing anyway: a console with `logln!`, a heap, a
  config file on the SD card, storage, networking with DHCP and SNTP, a
  small web server, Prometheus metrics, and over-the-air updates. Each is
  a cargo feature, so you take only what you use.
- **[rpi-loader](/rpi-hal/crates/loader/)** makes development
  bearable. Without it, every change means pulling the SD card, copying a
  file and putting it back. With it, you copy one loader image to the
  card once, and from then on `rpi-loader` uploads your build over a
  USB-serial cable and starts it, in seconds. It can also read and write
  the card in place, so the card can stay in the board.

All four are on crates.io, are dual-licensed MIT or Apache-2.0, and
follow their own version numbers.

## How to start

The quickest way is the [project selector](/rpi-hal/start/): pick your
board, and it hands you a zip with a project ready to build. That
project has the right target and linker settings, a `config.txt` for
the board, the matching loader image, and a `main.rs` that logs a
heartbeat over the UART console. If you prefer the command line, the
same template is available through `cargo generate`:

```sh
cargo install cargo-generate
cargo generate --git https://github.com/joeferner/rpi-hal template
```

Either way, `make sd-card` fetches the firmware files and the loader
image for the card, and `make run` builds, uploads and boots your
program. The [getting started](/rpi-hal/getting-started/) guide walks
through the wiring and the first boot.

The code is at
[github.com/joeferner/rpi-hal](https://github.com/joeferner/rpi-hal).
Issues and pull requests are welcome, and so are reports of which boards
you've run it on.
