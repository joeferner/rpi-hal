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
# Not every example works against `bcm2711`, and the two ways it fails look
# nothing alike. Anything using interrupts fails to *build*: it needs `Lic`,
# and the legacy interrupt controller doesn't exist on that chip (its
# GIC-400 isn't supported yet). Anything using USB builds and then finds an
# empty root port at run time: the hub and Ethernet a Pi 2/3 reaches over
# DWC2 are one soldered-on LAN9514, where a Pi 4 has a VL805 xHCI behind
# PCIe and a native GENET MAC instead. Each example's header says which
# board it expects.
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
