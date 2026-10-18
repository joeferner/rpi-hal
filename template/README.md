# {{project-name}}

A bare-metal Rust program for the {{board_name}}, on
[rpi-hal](https://crates.io/crates/rpi-hal) and
[rpi-kickstart](https://crates.io/crates/rpi-kickstart). It starts a
console on the UART and logs a heartbeat once a second.

## What you need

- A USB-to-serial cable (3.3 V): its RX on GPIO14 (pin 8), its TX on
  GPIO15 (pin 10), and ground on pin 6. The console runs at 115200 8N1.
- An SD card with one FAT32 partition.
- Rust via [rustup](https://rustup.rs). `rust-toolchain.toml` installs
  the right toolchain and target on first build.
- `cargo install cargo-binutils rpi-loader`: `cargo objcopy` turns the
  build into the raw image the firmware loads, and `rpi-loader` uploads
  it to the board.

## Set up the card, once

```sh
make sd-card
```

This fetches the Raspberry Pi boot firmware this board needs (none on a
Pi 5, whose firmware lives in its EEPROM) and the
[rpi-loader](https://github.com/joeferner/rpi-hal/tree/main/loader) image
into `sd-card/`, checking each against `sd-card.sha256`. Copy everything
in `sd-card/` onto the card's boot partition, including `config.txt`.

The loader goes on the card as `{{kernel}}`, the only filename the
firmware loads on this board. It stays resident, so after this you never
have to take the card out again.

## Build and run

```sh
make run DEVICE=/dev/ttyUSB0
```

This builds `target/{{kernel}}`, uploads it to the board over the serial
cable, and boots it. The heartbeat appears on the same cable:

```
{{project-name}}: hello from the {{board_name}}
up 1s
up 2s
```

To boot without the loader, copy `target/{{kernel}}` onto the card in
place of the loader's `{{kernel}}`.

## Next

`src/main.rs` is the whole program. rpi-kickstart has the rest of what a
board usually needs, each behind a feature in `Cargo.toml`: a heap, a
settings file and storage on the card, networking, a web server, OTA
updates. See its [documentation](https://docs.rs/rpi-kickstart), and
[rpi-hal's](https://docs.rs/rpi-hal) for the peripherals.
