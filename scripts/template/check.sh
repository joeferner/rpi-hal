#!/usr/bin/env bash
# Generates a project from template/ for every board and arch in
# template/boards.json and builds each one, the way someone running
# `cargo generate` would: against crates.io.
#
#   scripts/template/check.sh            # every variant
#   scripts/template/check.sh pi3/64     # just these
#
# With TEMPLATE_DIFF=1, each project is also rendered by the site's
# project selector (site/scripts/scaffold-cli.mjs, the same code the
# browser runs) and the two are diffed: the zip and `cargo generate` must
# hand out the same project. Needs `npm ci` in site/ first.
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
#
# Written into the generated project's .cargo/config.toml rather than
# passed as `--config` flags: `cargo objcopy` rejects `--config` given more
# than once, and it re-runs the build, so it has to see the same patches
# as `cargo build` or it would silently relink against crates.io.
patches=""
if [ "${TEMPLATE_PATCH:-0}" = 1 ]; then
    patches="[patch.crates-io]"$'\n'
    for spec in rpi-hal:hal rpi-kickstart:kickstart rpi-hal-embassy:embassy rpi-loader-ota:loader/ota; do
        patches+="${spec%%:*} = { path = \"$root/${spec##*:}\" }"$'\n'
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

    if [ "${TEMPLATE_DIFF:-0}" = 1 ]; then
        selector_out="$out/selector"
        rm -rf "${selector_out:?}/$name"
        node_args=(--board "$board" --name "$name" --out "$selector_out")
        if [ "${#defines[@]}" -gt 2 ]; then
            node_args+=(--arch "$arch")
        fi
        node "$root/site/scripts/scaffold-cli.mjs" "${node_args[@]}"
        if ! diff -r "$dest" "$selector_out/$name"; then
            echo "$variant: the site's project differs from cargo-generate's (above)" >&2
            failed+=("$variant")
            continue
        fi
        echo "$variant: the site renders the same project"
    fi

    if [ -n "$patches" ]; then
        printf '\n%s' "$patches" >>"$dest/.cargo/config.toml"
    fi

    kernel="$(jq -r --arg b "$board" --arg a "$arch" '.boards[] | select(.id == $b) | .variants[$a].vars.kernel' "$root/template/boards.json")"
    if (cd "$dest" && cargo build --release && cargo objcopy --release -- -O binary "target/$kernel"); then
        echo "$variant: ok"
    else
        failed+=("$variant")
    fi
done

if [ "${#failed[@]}" -gt 0 ]; then
    echo "failed: ${failed[*]}" >&2
    exit 1
fi
