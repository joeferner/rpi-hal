#!/usr/bin/env bash
set -euo pipefail

if [ $# -ne 1 ]; then
    echo "usage: $0 <example-name>" >&2
    echo "  e.g. $0 blink" >&2
    echo "       $0 uart_hello" >&2
    exit 1
fi

example="$1"

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

build_args=(--example "$example" --release)
if [ -n "$features" ]; then
    build_args+=(--features "$features")
fi

cargo build "${build_args[@]}"
cargo objcopy "${build_args[@]}" -- -O binary target/kernel7.img

echo "Built target/kernel7.img — copy it to the SD card boot partition."
