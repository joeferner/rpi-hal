#!/usr/bin/env bash
# What `cargo package` checks -- that the packaged .crate builds with
# nothing from this working copy -- except that the crates it depends on
# *in this repository* come from here rather than from crates.io.
#
# `cargo package` builds the tarball against crates.io, because publishing
# drops each dependency's `path`. So a change that touches rpi-hal and
# rpi-hal-embassy together fails rpi-hal-embassy's verification until that
# rpi-hal is published -- which can't happen before the change is merged.
# This packages without cargo's own verification, unpacks the result, and
# builds it with one `patch.crates-io` override per in-repo dependency.
# Everything else about the check is the same: only what the package
# contains is in the build. The real, unpatched `cargo package` still runs
# in the release job, once the dependencies are on crates.io.
#
# Usage: scripts/ci/package-verify.sh <crate-dir> [cargo args...]
#   e.g. scripts/ci/package-verify.sh hal --features bcm2837
# <crate-dir> is relative to the repository root. The cargo args go to
# both the package and the build, as they would to `cargo package`.
set -euo pipefail

root="$(git rev-parse --show-toplevel)"
crate_dir="$root/$1"
shift

# The crate's own rust-toolchain.toml and .cargo/config.toml are read from
# the working directory, so everything runs from there.
cd "$crate_dir"

# Kept apart from the normal target directory. The build below compiles
# the unpacked copy, and sharing target/ would leave fingerprints whose
# source paths point into that copy: every later `cargo build` would then
# report "Finished" without recompiling, and edits to src/ would have no
# effect until `cargo clean`.
export CARGO_TARGET_DIR="$crate_dir/target/verify"

meta="$(cargo metadata --no-deps --format-version 1)"
name="$(jq -r '.packages[0].name' <<<"$meta")"
version="$(jq -r '.packages[0].version' <<<"$meta")"

# Still refuses a dirty working tree, as `cargo package` does: what gets
# checked is the committed state.
cargo package --no-verify "$@"

unpacked="$CARGO_TARGET_DIR/unpacked"
rm -rf "$unpacked"
mkdir -p "$unpacked"
tar -xzf "$CARGO_TARGET_DIR/package/$name-$version.crate" -C "$unpacked"

# Every dependency the manifest gives a `path` -- normal, dev or build --
# once each. A patch the build doesn't end up using only draws a warning.
patches=()
while IFS=$'\t' read -r dep path; do
    echo "package-verify: $dep from $path"
    patches+=(--config "patch.crates-io.$dep.path=\"$path\"")
done < <(jq -r '[.packages[0].dependencies[] | select(.path != null) | [.name, .path]] | unique[] | @tsv' <<<"$meta")

cargo build --manifest-path "$unpacked/$name-$version/Cargo.toml" "$@" "${patches[@]}"
