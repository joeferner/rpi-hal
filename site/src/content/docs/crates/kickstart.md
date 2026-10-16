---
title: rpi-kickstart
description: The bring-up a bare-metal Raspberry Pi application does before it is an application.
---

rpi-hal stops at the peripheral: it hands out a `Uart`, a `Mailbox`, an
`Sd`. What every board then does with them turns out to be the same
program written again: a console the whole image can print to, a heap
sized from what the firmware reports, a card mounted and a settings file
read off it, a site served out of RAM, a clock that something has to set.
rpi-kickstart is that program, written once.

Every capability is a feature, so a board takes only what it uses. The
[project template](../../start/) starts with `console` alone.

| Feature | What it is |
| --- | --- |
| `console` | One sink for the whole image, and `logln!` |
| `heap` | A TLSF `#[global_allocator]`, sized from what the firmware reports |
| `entropy` | The SoC's hardware RNG, and the `getrandom` backend over it |
| `config` | A TOML settings file, with `file:line:column` errors |
| `storage` | The card's FAT volume, mounted with `resident-fat` |
| `site` | Everything under `/WWW` on the card, served from RAM |
| `clock`, `tz`, `sntp` | The wall clock, local time, and setting it from NTP |
| `net`, `ethernet`, `wifi`, `mdns` | Getting onto the network, and a name on it |
| `web`, `auth` | A `picoserve` web server, and password login for it |
| `tls`, `https`, `notify` | TLS 1.3 client connections, HTTPS requests, push and email notifications |
| `metrics` | Prometheus metrics, including the build's identity |
| `ota` | Over-the-air updates, in rpi-loader-ota's bundle format |
| `splash` | A boot logo baked into the kernel |

The README has the full list and what each feature needs.

- **API reference:** [docs.rs/rpi-kickstart](https://docs.rs/rpi-kickstart)
- **README:** [kickstart/README.md](https://github.com/joeferner/rpi-hal/blob/main/kickstart/README.md)
- **Changes:** [kickstart/CHANGELOG.md](https://github.com/joeferner/rpi-hal/blob/main/kickstart/CHANGELOG.md)
- **crates.io:** [rpi-kickstart](https://crates.io/crates/rpi-kickstart)
