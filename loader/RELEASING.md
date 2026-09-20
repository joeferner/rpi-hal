# Releasing

How to cut a release. Maintainer-facing; nothing here is needed to *use*
the loader, the CLI or the library.

**There are two kinds of release, and they are independent.**

| Tag | Ships | Where |
| --- | --- | --- |
| `v<version>` | the `rpi-loader` CLI, and the four loader images | crates.io + a GitHub release |
| `ota-v<version>` | the `rpi-loader-ota` library | crates.io |

The CLI and the firmware move together because they are two halves of a
wire protocol. The library does not, because its consumers are firmware
projects in other repositories and a renamed command-line flag is no
reason to bump their dependency. Each has its own changelog: `CHANGELOG.md`
for the pair, `ota/CHANGELOG.md` for the library.

Either publish is permanent — a version can be yanked but never replaced
or deleted, and the number can never be reused. Most of what follows
exists to make a mistake fail *before* that point.

## One-time setup

Only needed once per repository (or when a token expires).

- **crates.io API token.** Create one under Account Settings → API Tokens
  with the **publish-update** scope — plus **publish-new** for any release
  that claims a name not yet on crates.io — then store it as a repository
  secret:

  ```sh
  gh secret set CARGO_REGISTRY_TOKEN
  ```

  Secrets do not carry over from another repository, so having published
  `rpi-hal` does not cover this one.

  **Check the token's scope before releasing a package for the first
  time.** crates.io tokens can be limited to named crates as well as to
  actions, and a token created to publish updates of one crate cannot
  claim another. That failure lands at the last step of the workflow,
  after the GitHub release object already exists.

- **The `crates-io` environment.** `.github/workflows/release.yml`
  declares it. Create it under Settings → Environments and add yourself as
  a **required reviewer**: the tag push then parks the workflow at
  "waiting for approval" and gives one last look before the irreversible
  step.

- **Repository visibility.** `cli/Cargo.toml`'s `repository` field, the
  README badges, and the changelog's version links all point at GitHub.
  While the repository is private, every one of those is a 404 for anyone
  reading the crates.io page.

## Releasing the library

`rpi-loader-ota` is the short version of everything below, because it has
no firmware, no images and no wire protocol — just a Rust API and a bundle
format.

1. On a branch: set `version` in `ota/Cargo.toml`, run `make test-ota` so
   `ota/Cargo.lock` is refreshed, and give `ota/CHANGELOG.md` a dated
   `## [<version>] - <YYYY-MM-DD>` heading plus a link reference at the
   bottom. The workflow greps for that date and refuses to publish without
   it.
2. **Raise the CLI's requirement to match**, in the same change:
   `rpi-loader-ota`'s version in `cli/Cargo.toml`, then a `cargo check` in
   `cli/` for its lockfile. This is not optional and not a follow-up — a
   path dependency has to satisfy the version written beside it, so
   leaving the CLI asking for the old one fails *every* job with
   `failed to select a version`, not just the packaging one.
3. `make package-ota` on a clean tree.
4. Merge the PR, then tag and push:

   ```sh
   git checkout main && git pull
   git tag ota-v<version> && git push origin ota-v<version>
   ```
5. Approve the parked workflow.

**The CLI depends on this package, so it has to be published first.**
`cargo package` on the CLI resolves `rpi-loader-ota` from crates.io — the
path dependency is stripped when packaging, which is the point of writing
both a `version` and a `path` — and a version that is not there yet fails
the CLI's release before it starts.

**So `package verifies` is red from step 2 until the publish, and that is
the expected state rather than a fault.** Once step 2 has raised the CLI's
requirement, every other job is green and that one job asks crates.io for
a version this release has not published yet. Nothing in the pull request
can make it pass; publishing is what fixes it. Since the ruleset lists it
among the required checks, the merge in step 4 has to get past a check
that cannot go green first, and there are two ways:

- **Bypass it**, with `gh pr merge --squash --admin`. This works only if
  the `main` ruleset grants a bypass actor. A ruleset is not classic
  branch protection: with `bypass_actors` empty it refuses a repository
  admin as flatly as anyone else, and `--admin` comes back with
  `Required status check "package verifies" is failing`. Check with
  `gh api repos/:owner/:repo/rulesets/<id> --jq .bypass_actors` before
  planning around it.
- **Publish first, then merge with no bypass at all.** Dispatch the
  release job against the release branch —
  `gh workflow run release.yml --ref <branch> -f package=ota` — which is
  what the `package` input exists for, since a dispatch has no tag to
  read. crates.io then has the version, `package verifies` goes green on
  a re-run, and the pull request merges normally. Tagging `main`
  afterwards is safe: the publish step asks the sparse index first and
  skips a version that is already there, so the tag lands where the
  changelog's link expects it without publishing twice.

Either way the publish wants doing promptly rather than leaving `main`
sitting with a red required check.

**What counts as breaking:** the Rust API as usual, and the bundle format
itself. A change to the container's bytes is breaking in a way a semver
bump cannot really express, because a bundle is parsed by the firmware
already running and installs the firmware that replaces it — so a board
can never be sent a container its current build does not understand.
Changing it means reaching every deployed board some other way once. The
version byte in the header exists to make that a clean rejection rather
than a puzzle.

## Releasing the CLI and the firmware

### 1. Decide the version

Semantic versioning, with the usual pre-1.0 caveat that `0.x` bumps the
*minor* for breaking changes. The CLI and the firmware carry the same
version and move together — see the note at the top of `CHANGELOG.md` —
and the release workflow refuses to run if those two manifests disagree.
`ota/Cargo.toml` is not part of that check and is not expected to match.

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

The release carries five images, a bash completion and a `SHA256SUMS`:

| Asset | Board | Execution state |
| --- | --- | --- |
| `rpi-loader-<version>-bcm2837-kernel7.img` | Pi 2 v1.2, Pi 3 | AArch32 |
| `rpi-loader-<version>-bcm2837-kernel8.img` | Pi 2 v1.2, Pi 3 | AArch64 |
| `rpi-loader-<version>-bcm2711-kernel7.img` | Pi 4 | AArch32 |
| `rpi-loader-<version>-bcm2711-kernel8.img` | Pi 4 | AArch64 |
| `rpi-loader-<version>-bcm2835-kernel.img` | Pi 1, Pi Zero | AArch32 (ARMv6) |
| `rpi-loader-<version>.bash` | — | host-side shell completion |

The images are named per chip and version because a release page cannot
hold several files all called `kernel7.img`, but the firmware loads
*only* the bare names — so the release notes tell people to rename on the
way to the SD card. Worth re-reading those notes after the first release;
that instruction is the easiest thing here to get wrong.

BCM2835 has no `kernel8.img` counterpart: the ARM1176 has no 64-bit mode,
and its image is `kernel.img` with no digit because `start.elf` picks the
filename from the CPU it finds.

The completion is generated by the CLI the release job just built, so it
needs no separate check beyond its presence — but it is the one asset
whose absence would be silent, since nothing downstream reads it.

There is no docs.rs page to check. The published package is a binary with
no library target, so its documentation would be empty; the README on the
crates.io page is what people read.

## What the automation enforces, and how it fails

| Guard | Where | Symptom if it trips |
| --- | --- | --- |
| The CLI and firmware manifests carry the same version | `release.yml` | Release job fails before publishing |
| Tag matches the manifest it names | `release.yml` | Same. Skipped on a `workflow_dispatch` run, which has no tag |
| Changelog has a dated section for the version | `release.yml` | Same |
| Packaged tarball actually builds | `make package` / `make package-ota`, in both CI and the release jobs | Same |
| Images were actually produced | `ci.yml` | CI fails on the pull request, long before a tag exists |
| Every package still builds on its declared MSRV | `ci.yml` | Same |
| PRs required on `main` | Repository ruleset | Direct pushes rejected |

A `v*` tag runs only the CLI job and an `ota-v*` tag only the library job;
the prefixes cannot both match, since `refs/tags/ota-v0.1.0` does not start
with `refs/tags/v`. A `workflow_dispatch` run has no tag to read and asks
which package it means.

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
