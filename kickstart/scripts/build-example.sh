#!/usr/bin/env bash
set -euo pipefail

if [ $# -ne 1 ]; then
    echo "usage: $0 <example-name>" >&2
    echo "  e.g. $0 hello" >&2
    exit 1
fi

example="$1"

cd "$(dirname "$0")/.."

# An example that needs a feature declares `required-features` in
# Cargo.toml -- ask cargo itself rather than hardcoding a per-example
# feature list here, so this script can't drift out of sync with
# Cargo.toml. Nothing declares one yet, and this is what means no script
# change is needed when something does. Both cargo invocations below must
# share the exact same
# flags: `objcopy` re-invokes `build` internally, and if it didn't get
# the same `--features`, it would silently relink without them instead
# of just reusing the artifact from the line above.
features=$(cargo metadata --no-deps --format-version 1 |
    jq -r --arg name "$example" \
        '.packages[0].targets[] | select(.name == $name) | (.["required-features"] // []) | join(",")')

build_args=(--example "$example" --release)
if [ -n "$features" ]; then
    build_args+=(--features "$features")
fi

cargo build "${build_args[@]}"
cargo objcopy "${build_args[@]}" -- -O binary target/kernel7.img

echo "Built target/kernel7.img — deploy either way:"
echo "  - SD card: copy it to the boot partition and it direct-boots."
echo "  - rpi-loader over UART, matching the 0x8000 link address:"
echo "    rpi-loader --device <device> boot --load-addr 0x8000 target/kernel7.img"
