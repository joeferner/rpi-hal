#!/usr/bin/env bash
# Waits until every crate <crate-dir> depends on *in this repository* is
# on crates.io at the version this commit has, so `cargo package` -- which
# drops each dependency's `path` and resolves it from crates.io -- can
# find it.
#
#   scripts/ci/wait-for-deps.sh embassy
#
# This is what lets a batch release push all its tags at once in any
# order: `embassy-v0.10.0`'s run sits here until `hal-v0.10.0`'s run has
# published rpi-hal 0.10.0. Every kind of dependency counts, dev included:
# packaging resolves the whole graph to write the lockfile.
#
# Waits up to $WAIT_MINUTES (default 20). A dependency that never appears
# usually means its own release wasn't tagged, or failed.
set -euo pipefail

root="$(git rev-parse --show-toplevel)"
crate_dir="$root/${1:?usage: wait-for-deps.sh <crate-dir>}"
deadline=$(( $(date +%s) + ${WAIT_MINUTES:-20} * 60 ))

# The crate's own toolchain and config come from its directory.
cd "$crate_dir"

deps="$(cargo metadata --no-deps --format-version 1 \
    | jq -r '[.packages[0].dependencies[] | select(.path != null) | [.name, .path]] | unique[] | @tsv')"

if [ -z "$deps" ]; then
    echo "no in-repo dependencies"
    exit 0
fi

while IFS=$'\t' read -r dep path; do
    version="$(sed -n 's/^version = "\(.*\)"/\1/p' "$path/Cargo.toml" | head -1)"
    until "$root/scripts/ci/on-crates-io.sh" "$dep" "$version"; do
        if [ "$(date +%s)" -ge "$deadline" ]; then
            echo "gave up waiting for $dep $version on crates.io; was its release tagged, and did it succeed?" >&2
            exit 1
        fi
        echo "waiting for $dep $version on crates.io..."
        sleep 20
    done
    echo "$dep $version is on crates.io"
done <<<"$deps"
