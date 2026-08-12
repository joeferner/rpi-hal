#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ] || [ $# -gt 2 ]; then
    echo "usage: $0 <example-name> [bcm2837|bcm2711]" >&2
    echo "  e.g. $0 blink" >&2
    echo "       $0 uart_hello" >&2
    echo "       $0 blink bcm2711   # Pi 4" >&2
    exit 1
fi

example="$1"
# Neither of rpi-hal's chip features is a default (see its Cargo.toml) --
# `bcm2837` here since every example targets Pi 2/3 unless told otherwise.
# Not every example builds against `bcm2711` yet (see TODO.md's "Raspberry
# Pi 4" section): most that use interrupts need `Lic`, which doesn't exist
# under that feature.
chip="${2:-bcm2837}"

cd "$(dirname "$0")/.."

# Some examples (e.g. multicore_blink) declare `required-features` in
# Cargo.toml -- ask cargo itself rather than hardcoding a per-example
# feature list here, so this script can't drift out of sync with
# Cargo.toml. Both cargo invocations below must share the exact same
# flags: `objcopy` re-invokes `build` internally, and if it didn't get
# the same `--features`, it would silently relink without them instead
# of just reusing the artifact from the line above.
features=$(cargo metadata --no-deps --format-version 1 |
    jq -r --arg name "$example" \
        '.packages[0].targets[] | select(.name == $name) | (.["required-features"] // []) | join(",")')

build_args=(--example "$example" --release --features "$chip${features:+,$features}")

cargo build "${build_args[@]}"
cargo objcopy "${build_args[@]}" -- -O binary target/kernel7.img

echo "Built target/kernel7.img ($chip) — copy it to the SD card boot partition."
