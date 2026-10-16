#!/usr/bin/env bash
# Generates a project from template/ for every board and arch in
# template/boards.json and builds each one, the way someone running
# `cargo generate` would: against crates.io.
#
#   scripts/template/check.sh            # every variant
#   scripts/template/check.sh pi3/64     # just these
#
# With TEMPLATE_PATCH=1, each build instead uses this repository's
# rpi-hal, rpi-kickstart (and what they reach by path), so a change on
# main that breaks the template shows up before it is released.
#
# Projects go under $TEMPLATE_OUT (default: target/template-check at the
# repository root), one directory per variant, regenerated every run.
set -euo pipefail

root="$(git rev-parse --show-toplevel)"
out="${TEMPLATE_OUT:-$root/target/template-check}"

if [ "$#" -gt 0 ]; then
    variants=("$@")
else
    mapfile -t variants < <(jq -r '.boards[] | .id as $b | .arches[] | "\($b)/\(.)"' "$root/template/boards.json")
fi

# The in-repo crates a generated project reaches, directly or through
# another in-repo crate's path dependency. All of them have to be patched
# together, or the build holds two copies of one.
patches=()
if [ "${TEMPLATE_PATCH:-0}" = 1 ]; then
    for spec in rpi-hal:hal rpi-kickstart:kickstart rpi-hal-embassy:embassy rpi-loader-ota:loader/ota; do
        patches+=(--config "patch.crates-io.${spec%%:*}.path=\"$root/${spec##*:}\"")
    done
fi

failed=()
for variant in "${variants[@]}"; do
    board="${variant%%/*}"
    arch="${variant##*/}"
    name="demo-$board-$arch"
    dest="$out/$name"
    echo "=== $variant"

    rm -rf "$dest"
    mkdir -p "$out"
    defines=(--define "board=$board")
    # Only boards that offer a choice are asked for one.
    if [ "$(jq -r --arg b "$board" '.boards[] | select(.id == $b) | .arches | length' "$root/template/boards.json")" -gt 1 ]; then
        defines+=(--define "arch=$arch")
    fi
    cargo generate --path "$root/template" --name "$name" --destination "$out" \
        --vcs none --silent "${defines[@]}"

    # Every Liquid tag must have been rendered.
    if grep -rnE '\{\{|\{%' "$dest" --exclude-dir=target; then
        echo "$variant: unrendered template syntax above" >&2
        failed+=("$variant")
        continue
    fi

    if (cd "$dest" && cargo build --release "${patches[@]}" &&
        cargo objcopy --release "${patches[@]}" -- -O binary "target/$(jq -r --arg b "$board" --arg a "$arch" '.boards[] | select(.id == $b) | .variants[$a].kernel' "$root/template/boards.json")"); then
        echo "$variant: ok"
    else
        failed+=("$variant")
    fi
done

if [ "${#failed[@]}" -gt 0 ]; then
    echo "failed: ${failed[*]}" >&2
    exit 1
fi
