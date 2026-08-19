# Releasing

How to cut a release of `rpi-hal`. Maintainer-facing; nothing here is
needed to *use* the crate.

A publish to crates.io is permanent — a version can be yanked but never
replaced or deleted, and the version number can never be reused. Most of
what follows exists to make a mistake fail *before* that point.

## One-time setup

Only needed once per repository (or when a token expires).

- **crates.io API token.** Create one under Account Settings → API Tokens
  with the **publish-update** scope — plus **publish-new** for the very
  first release of the crate — then store it as a repository secret:

  ```sh
  gh secret set CARGO_REGISTRY_TOKEN
  ```

- **The `crates-io` environment.** `.github/workflows/release.yml` declares
  it. Create it under Settings → Environments and add yourself as a
  **required reviewer**: the tag push then parks the workflow at "waiting
  for approval" and gives one last look before the irreversible step.

- **Repository visibility.** `Cargo.toml`'s `repository` and
  `documentation` fields, the README badge, and the README's issue links
  all point at GitHub. While the repository is private, every one of those
  is a 404 for anyone reading the crates.io page.

## Per-release steps

### 1. Decide the version

Semantic versioning, with the usual pre-1.0 caveat that `0.x` bumps the
*minor* for breaking changes. Things that are breaking here and are easy to
miss:

- Adding or removing a field on a public struct or enum variant — several
  error types carry diagnostic fields, and trimming one is a breaking
  change (`sd::Error::CardError` was cut from eight fields to two before
  0.1.0 precisely because it was free to do then).
- Renaming or gating an existing public item behind a new feature.
- Raising `rust-version`. An MSRV bump is at least a minor release.

Adding a feature, a driver, or a trait implementation is a minor bump.

### 2. Bump the version and update the changelog

On a branch — the `main` ruleset requires a pull request, so nothing goes
in directly:

```sh
git checkout -b release-<version>
```

- `Cargo.toml`: set `version`.
- `cargo check --features bcm2837` — refreshes `Cargo.lock`, which is
  tracked and would otherwise be stale in the published tarball.
- `CHANGELOG.md`: give the changes a version heading —
  `## [<version>] - <YYYY-MM-DD>` — and add a link reference at the bottom
  pointing at `releases/tag/v<version>`. If an `## [Unreleased]` heading is
  sitting there, rename it; if there isn't one, write the version heading
  directly. Both are normal (see "The changelog needs no reopening" below).

The date matters: the release workflow **refuses to publish** while the
literal `ReleaseDate` placeholder is present, so a section left
undated fails the release rather than shipping a changelog that claims the
version was never released.

### 3. Open the PR and let CI run

```sh
gh pr create --fill
```

The ruleset requires the CI checks to pass. Merge with squash:

```sh
gh pr merge --squash --delete-branch
```

### 4. Verify locally, on a clean tree

```sh
git checkout main && git pull
make pre-commit      # fmt, clippy, both chips, examples, docs
make package         # what `cargo publish` will verify
```

`make package` refuses a dirty working tree, which is deliberate: what gets
published is the committed state, not what happens to be on disk.

### 5. Tag and push

```sh
git tag -a v<version> -m "rpi-hal <version>"
git push origin v<version>
```

The tag **must** start with `v` — that is the workflow's trigger pattern,
and a bare `0.2.0` silently does nothing at all. It must also match
`Cargo.toml`'s version, which the workflow checks and fails on.

### 6. Approve and watch

```sh
gh run watch
```

The release job re-runs `make package`, then publishes. If you set up the
required reviewer, approve it in the Actions UI when it parks.

### 7. Verify the publish

```sh
open https://crates.io/crates/rpi-hal
open https://docs.rs/crate/rpi-hal/<version>/builds
```

The docs.rs build is the one thing CI cannot prove, because docs.rs builds
with default features unless told otherwise, and this crate does not
compile without a chip feature. `[package.metadata.docs.rs]` names one; if
that page shows a failure, that section is where to look. A successful
build shows "Available on crate feature …" badges on the feature-gated
items, which confirms the `--cfg docsrs` path worked.

### 8. Create the GitHub release

Not decoration: `CHANGELOG.md`'s version links point at
`/releases/tag/v<version>`, which only resolves once a release object
exists.

```sh
gh release create v<version> \
  --title "rpi-hal <version>" \
  --notes-file <(awk -v v="## [<version>]" '
    index($0, v) == 1 { inside = 1; next }
    inside && /^## \[/ { exit }
    inside { print }
  ' CHANGELOG.md)
```

The range has to end at the *next* `## [` heading, which is why this is
`awk` and not the obvious `sed -n '/## \[<version>\]/,/^\[<version>\]:/p'`.
That closing address matches the link reference at the bottom of the file,
not anything near the section, so the range runs past every older heading
and the "notes" become the entire changelog. It fails silently — `gh`
accepts whatever it is handed — so the only symptom is an over-long
release page nobody rereads.

That's the release. Nothing further is required.

## The changelog needs no reopening

Keep a Changelog suggests holding an empty `## [Unreleased]` section open at
all times. Don't: with a protected `main`, creating it is a commit and a
pull request whose entire content is a heading with nothing under it.

Instead the section is created by **whichever change first needs it**, in
that change's own pull request — the PR that adds a driver adds the heading
above its own bullet. The heading then exists exactly when there is
something to put under it, and step 2 renames it. If a release happens to
contain only changes that warranted no entry, there is no heading to rename
and step 2 writes the version heading directly.

The same reasoning applies to post-release version bumps, which is why
there is no `0.2.0-dev` step here either: `Cargo.toml` carries the last
released version between releases, and step 2 is where it moves.

## What the automation enforces, and how it fails

| Guard | Where | Symptom if it trips |
| --- | --- | --- |
| Tag matches `Cargo.toml` version | `release.yml` | Release job fails before publishing |
| Changelog is dated | `release.yml` | Same |
| Packaged tarball actually builds | `make package`, in both CI and the release job | Same |
| A chip feature is selected | `lib.rs`'s `compile_error!` | `cargo package`/`publish` with no `--features` aborts, because the verification build uses *default* features and no chip is a default. `make package` passes `bcm2837` for exactly this reason |
| PRs required on `main` | Repository ruleset | Direct pushes rejected |

One coupling to know about: the ruleset's required status checks are
matched against the **job names** in `ci.yml`. Renaming a job there leaves
the ruleset waiting on a name that never reports, and every PR blocks until
the ruleset is updated too. It fails closed, which is the safe direction,
but it is a puzzling half hour if you have forgotten why.

## If something goes wrong

- **The publish failed partway.** Nothing was uploaded unless the
  `Publish` step itself succeeded. Fix the cause and re-run the workflow
  from the Actions UI (`workflow_dispatch`) — no need to move the tag.
  Note that a dispatch run skips the tag-match check, since there is no tag
  in its context.
- **A bad version reached crates.io.** It cannot be replaced. Yank it
  (`cargo yank --version <version>`), which leaves existing lockfiles
  working but stops new dependents from selecting it, then release a fix
  under a new version number.
- **The tag is wrong but nothing is published.** Delete it locally and on
  the remote (`git tag -d v<version>`,
  `git push --delete origin v<version>`) and start again from step 5. Once
  a version *is* published, leave its tag alone.
