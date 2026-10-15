#!/usr/bin/env bash
# Builds an ARMv6 example into a raw binary for a Pi 1 or Pi Zero. The
# BCM2835 counterpart to build-example.sh, which differs from it in three
# ways, all downstream of this being a different instruction set rather
# than a different chip:
#
#   - `--target armv6-none-eabi`, and with it `rpi-hal`'s ARMv6 boot code,
#     barriers and MMU programming. There is no 64-bit counterpart script:
#     the ARM1176 has no 64-bit mode.
#   - `+nightly -Z build-std=core`. That target is tier 3, so rustup ships
#     no precompiled `core` and one has to be built from source. Needs
#     `rustup toolchain install nightly --component rust-src` once; the
#     other two scripts stay on the stable toolchain this repository pins.
#   - The image is `kernel.img`, with no digit. `start.elf` picks the
#     kernel filename from the CPU it finds, so a board handed a
#     `kernel7.img` looks for a file that is not there and stops, with
#     nothing on the console to say so.
set -euo pipefail

if [ $# -ne 1 ]; then
    echo "usage: $0 <example-name>" >&2
    echo "  e.g. $0 embassy_button" >&2
    exit 1
fi

example="$1"
target="armv6-none-eabi"

cd "$(dirname "$0")/.."

# Some examples declare `required-features` in Cargo.toml -- ask cargo
# rather than hardcoding them here, same as the other two scripts, so this
# can't drift out of sync. Note that `embassy_multicore` is not buildable
# here whatever it declares: `multicore` is a `compile_error!` on a
# single-core chip, which is the intended answer rather than a gap.
features=$(cargo metadata --no-deps --format-version 1 |
    jq -r --arg name "$example" \
        '.packages[0].targets[] | select(.name == $name) | (.["required-features"] // []) | join(",")')

# `--no-default-features` because the chip is this crate's own default
# feature and `rpi-hal` prefers `bcm2837` when both are on -- leaving the
# default in place would compile the Pi 3's peripheral base into a Pi Zero
# image. See Cargo.toml.
#
# Same flags on both invocations: `objcopy` re-runs `build` internally and
# would silently relink without them otherwise (see build-example.sh).
build_args=(
    --example "$example" --release --target "$target"
    -Z build-std=core
    --no-default-features --features "bcm2835${features:+,$features}"
)

cargo +nightly build "${build_args[@]}"
cargo +nightly objcopy "${build_args[@]}" -- -O binary target/kernel.img

echo "Built target/kernel.img (linked at 0x8000)."
echo "Deploy either way:"
echo "  - SD card: copy target/kernel.img to the boot partition as"
echo "    KERNEL.IMG -- the name matters, see rpi-hal's getting-started."
echo "  - rpi-loader over UART, matching the 0x8000 link address:"
echo "    rpi-loader --device <device> boot --load-addr 0x8000 target/kernel.img"
