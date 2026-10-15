#!/usr/bin/env bash
# A version's section of a Keep a Changelog file.
#
#   scripts/ci/changelog.sh check   <CHANGELOG.md> <version>
#   scripts/ci/changelog.sh extract <CHANGELOG.md> <version>
#
# `check` fails unless the file has a dated `## [<version>] - YYYY-MM-DD`
# heading and no literal `ReleaseDate` placeholder (the `cargo-release`
# convention for a section not yet dated). An undated section ships a
# changelog that claims the version was never released; a missing one
# leaves nothing to write the release notes from.
#
# `extract` prints that section's body: everything up to the *next*
# `## [` heading. The obvious
# `sed -n '/## \[<version>\]/,/^\[<version>\]:/p'` does not work: its
# closing address matches the link reference at the bottom of the file
# rather than anything near the section, so the range runs past every
# older heading and the notes become the entire changelog. It fails
# silently, so the only symptom is an over-long release page.
set -euo pipefail

mode="${1:?usage: changelog.sh check|extract <CHANGELOG.md> <version>}"
file="${2:?missing CHANGELOG.md path}"
version="${3:?missing version}"

case "$mode" in
    check)
        if grep -q ReleaseDate "$file"; then
            echo "$file still has the ReleaseDate placeholder" >&2
            exit 1
        fi
        if ! grep -qE "^## \[$version\] - [0-9]{4}-[0-9]{2}-[0-9]{2}" "$file"; then
            echo "$file has no dated '## [$version] - YYYY-MM-DD' section" >&2
            exit 1
        fi
        ;;
    extract)
        awk -v v="## [$version]" '
            index($0, v) == 1 { inside = 1; next }
            inside && /^## \[/ { exit }
            inside { print }
        ' "$file"
        ;;
    *)
        echo "unknown mode $mode (check or extract)" >&2
        exit 2
        ;;
esac
