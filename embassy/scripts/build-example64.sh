#!/usr/bin/env bash
# Builds an AArch64 example into a raw binary for upload through rpi-loader
# (a 64-bit kernel8.img loader). The AArch64 counterpart to
# build-example.sh: rpi-hal's `rt` boot sequence supports AArch64, so
# examples build with the default features (rt + mmu) and boot on rt's own
# `_start` -- no hand-written boot stub needed.
set -euo pipefail

if [ $# -ne 1 ]; then
    echo "usage: $0 <example-name>" >&2
    echo "  e.g. $0 embassy_now" >&2
    exit 1
fi

example="$1"
target="aarch64-unknown-none-softfloat"

cd "$(dirname "$0")/.."

# Some examples (e.g. multicore_blink) declare `required-features` in
# Cargo.toml -- ask cargo rather than hardcoding them here, same as
# build-example.sh, so this can't drift out of sync.
features=$(cargo metadata --no-deps --format-version 1 |
    jq -r --arg name "$example" \
        '.packages[0].targets[] | select(.name == $name) | (.["required-features"] // []) | join(",")')

# Same flags on both invocations: `objcopy` re-runs `build` internally and
# would silently relink without them otherwise (see build-example.sh).
build_args=(--example "$example" --release --target "$target")
if [ -n "$features" ]; then
    build_args+=(--features "$features")
fi

cargo build "${build_args[@]}"
cargo objcopy "${build_args[@]}" -- -O binary target/kernel8.img

echo "Built target/kernel8.img (linked at 0x80000, the firmware's default"
echo "AArch64 load address)."
echo "Deploy either way:"
echo "  - SD card: copy target/kernel8.img to the boot partition (with"
echo "    arm_64bit=1 in config.txt) and it direct-boots, no loader."
echo "  - rpi-loader over UART, matching the 0x80000 link address:"
echo "    rpi-loader --device <device> boot --load-addr 0x80000 target/kernel8.img"
