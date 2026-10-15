#!/usr/bin/env bash
# Exits 0 if <crate> <version> is on crates.io, 1 if not.
#
#   scripts/ci/on-crates-io.sh rpi-hal 0.9.1
#
# Asks the sparse index, the cheapest authority on the question. Used so a
# re-run of a release that already published skips `cargo publish` rather
# than dying on "crate version already uploaded", and by
# wait-for-deps.sh.
set -euo pipefail

crate="${1:?usage: on-crates-io.sh <crate> <version>}"
version="${2:?missing version}"

# The index path: names of one, two or three characters get their own
# directories; anything longer is the first two characters, then the next
# two, then the name. Always lowercase.
name="$(tr '[:upper:]' '[:lower:]' <<<"$crate")"
case "${#name}" in
    1) path="1/$name" ;;
    2) path="2/$name" ;;
    3) path="3/${name:0:1}/$name" ;;
    *) path="${name:0:2}/${name:2:2}/$name" ;;
esac

index="$(curl -sS --fail "https://index.crates.io/$path" 2>/dev/null || true)"
grep -q "\"vers\":\"$version\"" <<<"$index"
