#!/usr/bin/env bash
# Pushes release tags, one `git push` each.
#
#   scripts/release/push-tags.sh hal-v0.10.0 embassy-v0.10.0 kickstart-v0.3.0
#
# One at a time because GitHub creates no push events at all when more
# than three tags arrive in a single push: `git push --tags` with four
# releases in it would push them and silently start no release workflow.
#
# Every tag is checked before any is pushed, so a typo doesn't leave half
# a batch released: it has to exist locally, sit on a commit that is on
# origin/main, and match the version in the manifest *at that commit*
# (release.yml checks the same thing, but only after the push).
#
# Order doesn't matter: each release waits for the crates it depends on to
# reach crates.io (scripts/ci/wait-for-deps.sh).
set -euo pipefail

if [ "$#" -eq 0 ]; then
    echo "usage: push-tags.sh <tag>..." >&2
    exit 2
fi

root="$(git rev-parse --show-toplevel)"
git fetch -q origin main

for tag in "$@"; do
    if ! git rev-parse -q --verify "refs/tags/$tag" >/dev/null; then
        echo "$tag: no such tag; create it with: git tag -a $tag -m \"...\" <commit>" >&2
        exit 1
    fi
    if ! git merge-base --is-ancestor "$tag^{commit}" origin/main; then
        echo "$tag: its commit is not on origin/main; tag the merged release commit" >&2
        exit 1
    fi
    RELEASE_REV="$tag" "$root/scripts/ci/release-meta.sh" "$tag" >/dev/null
    echo "$tag: ok"
done

for tag in "$@"; do
    git push origin "refs/tags/$tag"
done
