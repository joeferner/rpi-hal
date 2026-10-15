#!/usr/bin/env bash
# What a release is of: a pushed tag, or a component name from a manual
# dispatch, resolved to the crate it publishes and where that crate lives.
# Prints `key=value` lines for `$GITHUB_OUTPUT`:
#
#   component  hal | embassy | kickstart | loader | loader-ota
#   dir        the package `cargo publish` runs in
#   crate      its crates.io name
#   changelog  the CHANGELOG.md its release notes come from
#   version    from the manifest
#   tag        <prefix><version>, the tag the release belongs to
#   args       extra arguments for `cargo package`/`cargo publish`
#
# Usage:
#   scripts/ci/release-meta.sh embassy-v0.10.0   # a tag: must match the manifest
#   scripts/ci/release-meta.sh embassy           # a dispatch: the manifest decides
#
# Manifests are read from the working tree, or from the commit named in
# $RELEASE_REV when it is set (scripts/release/push-tags.sh checks a tag
# that way before pushing it).
set -euo pipefail

root="$(git rev-parse --show-toplevel)"
cd "$root"

# component | tag prefix | dir | crate | changelog | args
#
# `args` exists for rpi-hal alone: no chip is a default feature, and the
# verification build inside `cargo package`/`cargo publish` uses default
# features, so without one `lib.rs`'s `compile_error!` fires and the
# publish aborts on a package that is actually fine.
#
# The prefixes can't shadow each other: `loader-ota-v0.3.0` does not start
# with `loader-v`.
table='hal|hal-v|hal|rpi-hal|hal/CHANGELOG.md|--features bcm2837
embassy|embassy-v|embassy|rpi-hal-embassy|embassy/CHANGELOG.md|
kickstart|kickstart-v|kickstart|rpi-kickstart|kickstart/CHANGELOG.md|
loader|loader-v|loader/cli|rpi-loader|loader/CHANGELOG.md|
loader-ota|loader-ota-v|loader/ota|rpi-loader-ota|loader/ota/CHANGELOG.md|'

manifest_version() {
    local manifest="$1"
    if [ -n "${RELEASE_REV:-}" ]; then
        git show "$RELEASE_REV:$manifest"
    else
        cat "$manifest"
    fi | sed -n 's/^version = "\(.*\)"/\1/p' | head -1
}

arg="${1:?usage: release-meta.sh <tag-or-component>}"

while IFS='|' read -r component prefix dir crate changelog args; do
    if [ "$arg" = "$component" ]; then
        tag_given=""
    elif [ "${arg#"$prefix"}" != "$arg" ]; then
        tag_given="$arg"
    else
        continue
    fi

    version="$(manifest_version "$dir/Cargo.toml")"
    test -n "$version" || { echo "$dir/Cargo.toml has no version" >&2; exit 1; }

    # The CLI and the firmware are two halves of one wire protocol and
    # ship as one version; the firmware is never published, so this is the
    # only place their agreement is checked.
    if [ "$component" = loader ]; then
        firmware="$(manifest_version loader/firmware/Cargo.toml)"
        if [ "$firmware" != "$version" ]; then
            echo "loader/cli is $version but loader/firmware is $firmware; the two ship as one release" >&2
            exit 1
        fi
    fi

    tag="$prefix$version"
    if [ -n "$tag_given" ] && [ "$tag_given" != "$tag" ]; then
        echo "tag $tag_given does not match $dir/Cargo.toml's version $version (expected $tag)" >&2
        exit 1
    fi

    printf 'component=%s\ndir=%s\ncrate=%s\nchangelog=%s\nversion=%s\ntag=%s\nargs=%s\n' \
        "$component" "$dir" "$crate" "$changelog" "$version" "$tag" "$args"
    exit 0
done <<<"$table"

echo "$arg is neither a release tag (hal-v*, embassy-v*, kickstart-v*, loader-v*, loader-ota-v*) nor a component" >&2
exit 1
