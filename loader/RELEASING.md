# Releasing

How to cut a release of `rpi-loader`. Maintainer-facing; nothing here is
needed to *use* the loader or the CLI.

A release is two things at once: the `rpi-loader` CLI published to
crates.io, and the four loader images attached to a GitHub release. The
publish is permanent — a version can be yanked but never replaced or
deleted, and the version number can never be reused. Most of what follows
exists to make a mistake fail *before* that point.

## One-time setup

Only needed once per repository (or when a token expires).

- **crates.io API token.** Create one under Account Settings → API Tokens
  with the **publish-update** scope — plus **publish-new** for the very
  first release — then store it as a repository secret:

  ```sh
  gh secret set CARGO_REGISTRY_TOKEN
  ```

  Secrets do not carry over from another repository, so having published
  `rpi-hal` does not cover this one.

- **The `crates-io` environment.** `.github/workflows/release.yml`
  declares it. Create it under Settings → Environments and add yourself as
  a **required reviewer**: the tag push then parks the workflow at
  "waiting for approval" and gives one last look before the irreversible
  step.

- **Repository visibility.** `cli/Cargo.toml`'s `repository` field, the
  README badges, and the changelog's version links all point at GitHub.
  While the repository is private, every one of those is a 404 for anyone
  reading the crates.io page.

## Per-release steps

### 1. Decide the version

Semantic versioning, with the usual pre-1.0 caveat that `0.x` bumps the
*minor* for breaking changes. Both packages carry the same version and
move together — see the note at the top of `CHANGELOG.md` — and the
release workflow refuses to run if the two manifests disagree.

What counts as breaking here is wider than a Rust API, because most of
what this project exposes is not one:

- **The wire protocol.** Any change to a command byte, a header layout,
  the chunk framing, or an error code breaks a CLI talking to an older
  loader, or the reverse. There is a version byte in the handshake for
  exactly this reason; a mismatch is currently a warning, so treat the
  protocol as the compatibility surface it is.
- **The CLI's arguments.** Renaming a subcommand or flag, or making an
  optional one required, breaks whatever scripts people wrapped around it.
- **The load addresses** the images are linked at, since an image is
  uploaded to an address a caller passes in.
- Raising `rust-version` on the CLI. An MSRV bump is at least a minor
  release.

### 2. Bump the version and update the changelog

On a branch — the `main` ruleset requires a pull request, so nothing goes
in directly:

```sh
git checkout -b release-<version>
```

- `cli/Cargo.toml` and `firmware/Cargo.toml`: set `version` in both.
- `make build-cli build-bcm2837` — refreshes both `Cargo.lock` files,
  which are tracked and would otherwise be stale in the published tarball.
- `CHANGELOG.md`: give the changes a version heading —
  `## [<version>] - <YYYY-MM-DD>` — and add a link reference at the bottom
  pointing at `releases/tag/v<version>`. If an `## [Unreleased]` heading is
  sitting there, rename it; if there isn't one, write the version heading
  directly. Both are normal (see "The changelog needs no reopening"
  below).

The date is not decoration: the release workflow greps for
`## [<version>] - <date>` and **refuses to publish** without it. It is
also where the release notes come from, so an empty section produces an
empty release.

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
make pre-commit      # fmt, clippy, all four images, the CLI, its tests, docs
make package         # what `cargo publish` will verify
```

`make package` refuses a dirty working tree, which is deliberate: what
gets published is the committed state, not what happens to be on disk.

### 5. Tag and push

```sh
git tag -a v<version> -m "rpi-loader <version>"
git push origin v<version>
```

The tag **must** start with `v` — that is the workflow's trigger pattern,
and a bare `0.2.0` silently does nothing at all. It must also match the
version in both manifests, which the workflow checks and fails on.

### 6. Approve and watch

```sh
gh run watch
```

The release job re-verifies everything, builds the four images, creates
the GitHub release with them attached, and only then publishes to
crates.io. If you set up the required reviewer, approve it in the Actions
UI when it parks.

### 7. Verify the result

```sh
open https://crates.io/crates/rpi-loader
open https://github.com/joeferner/rpi-loader/releases/latest
```

The release carries four images and a `SHA256SUMS`:

| Asset | Board | Execution state |
| --- | --- | --- |
| `rpi-loader-<version>-bcm2837-kernel7.img` | Pi 2 v1.2, Pi 3 | AArch32 |
| `rpi-loader-<version>-bcm2837-kernel8.img` | Pi 2 v1.2, Pi 3 | AArch64 |
| `rpi-loader-<version>-bcm2711-kernel7.img` | Pi 4 | AArch32 |
| `rpi-loader-<version>-bcm2711-kernel8.img` | Pi 4 | AArch64 |

They are named per chip and version because a release page cannot hold
four files all called `kernel7.img`, but the firmware loads *only* the
bare names — so the release notes tell people to rename on the way to the
SD card. Worth re-reading those notes after the first release; that
instruction is the easiest thing here to get wrong.

There is no docs.rs page to check. The published package is a binary with
no library target, so its documentation would be empty; the README on the
crates.io page is what people read.

## What the automation enforces, and how it fails

| Guard | Where | Symptom if it trips |
| --- | --- | --- |
| Both manifests carry the same version | `release.yml` | Release job fails before publishing |
| Tag matches the manifests | `release.yml` | Same. Skipped on a `workflow_dispatch` run, which has no tag |
| Changelog has a dated section for the version | `release.yml` | Same |
| Packaged tarball actually builds | `make package`, in both CI and the release job | Same |
| Images were actually produced | `ci.yml` | CI fails on the pull request, long before a tag exists |
| The CLI still builds on its declared MSRV | `ci.yml` | Same |
| PRs required on `main` | Repository ruleset | Direct pushes rejected |

One coupling to know about: the ruleset's required status checks are
matched against the **job names** in `ci.yml`. Renaming a job there leaves
the ruleset waiting on a name that never reports, and every PR blocks
until the ruleset is updated too. It fails closed, which is the safe
direction, but it is a puzzling half hour if you have forgotten why.

The release job is written to be re-runnable. Uploading assets clobbers
whatever is already attached, and the publish step first asks the crates.io
index whether the version exists — otherwise a re-run would die on "crate
version already uploaded" and never reach whatever it was re-run for.

## If something goes wrong

- **The publish failed partway.** Nothing reached crates.io unless the
  `Publish` step itself succeeded. Fix the cause and re-run the workflow
  from the Actions UI (`workflow_dispatch`) — no need to move the tag.
- **The images are wrong but the publish succeeded.** Rebuild and
  re-upload; release assets, unlike a crates.io version, can be replaced.
  `gh release upload v<version> <files> --clobber`, or just re-run the
  workflow.
- **A bad version reached crates.io.** It cannot be replaced. Yank it
  (`cargo yank --version <version>`), which leaves existing lockfiles
  working but stops new dependents from selecting it, then release a fix
  under a new version number.
- **The tag is wrong but nothing is published.** Delete it locally and on
  the remote (`git tag -d v<version>`,
  `git push --delete origin v<version>`) and start again from step 5. Once
  a version *is* published, leave its tag alone.

## The changelog needs no reopening

Keep a Changelog suggests holding an empty `## [Unreleased]` section open
at all times. Don't: with a protected `main`, creating it is a commit and
a pull request whose entire content is a heading with nothing under it.

Instead the section is created by **whichever change first needs it**, in
that change's own pull request — the PR that adds a subcommand adds the
heading above its own bullet. The heading then exists exactly when there
is something to put under it, and step 2 renames it.

The same reasoning applies to post-release version bumps, which is why
there is no `0.2.0-dev` step here either: the manifests carry the last
released version between releases, and step 2 is where they move.
