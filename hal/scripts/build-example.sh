#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ] || [ $# -gt 2 ]; then
    echo "usage: $0 <example-name> [bcm2837|bcm2711|bcm2835]" >&2
    echo "  e.g. $0 blink" >&2
    echo "       $0 uart_hello" >&2
    echo "       $0 blink bcm2711   # Pi 4" >&2
    echo "       $0 blink bcm2835   # Pi 1, Pi Zero" >&2
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

if [ "$chip" = bcm2835 ]; then
    # The one chip whose *target* differs: ARMv6, not ARMv7-A. Two
    # consequences beyond the `--target` itself.
    #
    # The feature set is spelled out rather than left to the defaults:
    # the chip has to be named anyway, and naming `rt` and `mmu` beside
    # it says what an ARMv6 build actually covers.
    #
    # And the image is `kernel.img`. `start.elf` picks the kernel
    # filename from the CPU it finds -- `kernel.img` on ARMv6,
    # `kernel7.img` on ARMv7 -- so a Pi Zero handed a `kernel7.img`
    # looks for a file that isn't there and stops, with no ARM code run
    # and nothing on the console to say so.
    target_args=(--target armv6-none-eabi)
    feature_args=(--no-default-features --features "bcm2835,rt,mmu${features:+,$features}")
    image=kernel.img
else
    target_args=()
    feature_args=(--features "$chip${features:+,$features}")
    image=kernel7.img
fi

build_args=(--example "$example" --release "${target_args[@]}" "${feature_args[@]}")

cargo build "${build_args[@]}"
cargo objcopy "${build_args[@]}" -- -O binary "target/$image"

echo "Built target/$image ($chip) — copy it to the SD card boot partition."
