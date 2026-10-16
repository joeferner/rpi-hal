# Releasing

How to cut a release of any crate in this repository. Maintainer-facing;
nothing here is needed to *use* the crates.

Every crate releases on its own tag, with its own version and changelog:

| Tag | Ships | Changelog | Where |
| --- | --- | --- | --- |
| `hal-v<version>` | `rpi-hal` | `hal/CHANGELOG.md` | crates.io + a GitHub release |
| `embassy-v<version>` | `rpi-hal-embassy` | `embassy/CHANGELOG.md` | crates.io + a GitHub release |
| `kickstart-v<version>` | `rpi-kickstart` | `kickstart/CHANGELOG.md` | crates.io + a GitHub release |
| `loader-ota-v<version>` | `rpi-loader-ota` | `loader/ota/CHANGELOG.md` | crates.io + a GitHub release |
| `loader-v<version>` | the `rpi-loader` CLI and the five loader images | `loader/CHANGELOG.md` | crates.io + a GitHub release with the images, marked **Latest** |

Tags from before these crates shared a repository have no prefix:
rpi-hal's are the bare `v0.1.0`…`v0.9.0` here, and the others' were
renamed on import (`embassy-v0.9.0` and so on) with their release pages
left on the archived repositories.

A publish to crates.io is permanent: a version can be yanked but never
replaced or deleted, and its number can never be reused. Most of what
follows exists to make a mistake fail *before* that point.

## One-time setup

Only needed once per repository (or when a token expires).

- **crates.io API token**, stored as the repository secret
  `CARGO_REGISTRY_TOKEN` (`gh secret set CARGO_REGISTRY_TOKEN`). It needs
  the **publish-update** scope for all five crates, plus **publish-new**
  for any release that claims a name not yet on crates.io. crates.io
  tokens can be limited to named crates, and one limited to a single
  crate fails at the very last step of another crate's release, after its
  GitHub release already exists.
- **The `crates-io` environment**, declared by
  `.github/workflows/release.yml`, with yourself as a **required
  reviewer**: every release run parks at "waiting for approval" and gives
  one last look before the irreversible step.
- **The `main` ruleset** requires a pull request and one status check,
  `ci-ok`. It also allows only squash and rebase merges and requires
  linear history.

## Per-release steps

The same for every crate; what differs is how to choose the version (see
[Choosing the version](#choosing-the-version)).

### 1. Decide the version

See [Choosing the version](#choosing-the-version) for each crate's rules.

### 2. Bump the version and update the changelog

On a branch — `main` takes pull requests only:

```sh
git checkout -b release-<what>
```

- **The crate's `Cargo.toml`:** set `version`. For the loader, set it in
  both `loader/cli/Cargo.toml` and `loader/firmware/Cargo.toml`; the
  release refuses to run if they disagree.
- **Every crate here that depends on it, if the new version no longer
  satisfies their requirement** (a `0.x` minor bump, say). A path
  dependency has to satisfy the `version` written beside it, so leaving
  one behind fails *every* build of the dependent with `failed to select a
  version`. That makes it hard to forget, but it means those dependents'
  requirements, and so usually their versions, move in the same pull
  request. See [Releasing several crates at once](#releasing-several-crates-at-once).
- **Refresh the lock files** the bump touches: a build in each affected
  directory (`make -C <dir> pre-commit` covers it). They are tracked and
  ship in the package.
- **The changelog:** give the changes a version heading,
  `## [<version>] - <YYYY-MM-DD>`, and add a link reference at the bottom
  pointing at `https://github.com/joeferner/rpi-hal/releases/tag/<tag>`.
  If an `## [Unreleased]` heading is sitting there, rename it; if there
  isn't one, write the version heading directly. Both are normal (see
  [The changelog needs no reopening](#the-changelog-needs-no-reopening)).

The date is not decoration: the release workflow refuses to publish
without a dated section, or with the literal `ReleaseDate` placeholder
left in. The section is also where the release notes come from, so an
empty one produces an empty release page.

### 3. Open the PR and let CI run

```sh
gh pr create --fill
gh pr merge --squash --delete-branch   # once ci-ok is green
```

CI checks every crate the change touches and everything that depends on
them, including package verification against this repository's crates,
so a release PR that bumps rpi-hal and rpi-hal-embassy together goes
green before either is published.

### 4. Verify locally, on a clean tree

```sh
git checkout main && git pull
make pre-commit              # every crate
make -C <dir> package        # what CI's package check runs
```

`make package` refuses a dirty working tree, which is deliberate: what
gets published is the committed state, not what happens to be on disk.

### 5. Tag and push

Annotated tags, on the merged commit, pushed with the script:

```sh
git tag -a embassy-v<version> -m "rpi-hal-embassy <version>"
scripts/release/push-tags.sh embassy-v<version>
```

The script checks each tag before pushing any: it must exist, sit on a
commit that is on `origin/main`, and match the version in that commit's
manifest. Then it pushes them **one per `git push`**. GitHub creates no
push events at all when more than three tags arrive in one push, so a
batch pushed with `git push --tags` would start no release whatsoever,
silently.

The tag's prefix is what selects the crate. A tag without one (`v0.10.0`,
`0.10.0`) triggers nothing.

### 6. Approve and watch

```sh
gh run watch
```

Approve each run in the Actions UI when it parks on the `crates-io`
environment. A run then:

1. checks the tag against the manifest (and, for the loader, that the CLI
   and firmware agree);
2. checks the changelog has a dated section;
3. waits until every crate it depends on in this repository is on
   crates.io at the version this commit has;
4. runs the full `cargo package`, with every dependency resolved from
   crates.io;
5. for the loader, builds the five images, the bash completion and
   `SHA256SUMS`;
6. creates the GitHub release from the changelog section;
7. publishes, unless that version is already on crates.io.

### 7. Verify the result

- **crates.io:** `https://crates.io/crates/<crate>`.
- **docs.rs** (not the loader, which is a binary with no library):
  `https://docs.rs/crate/<crate>/<version>/builds`. This is the one thing
  CI cannot prove. docs.rs builds for a host target with default features
  unless told otherwise, and nothing here builds that way: rpi-hal needs
  a chip feature, and everything else depends on rpi-hal, which is
  bare-metal ARM. Each crate's `[package.metadata.docs.rs]` says what to
  build instead; if the page shows a failure, that section is where to
  look. For rpi-hal, "Available on crate feature …" badges on the gated
  items confirm the `--cfg docsrs` path worked.
- **The release page:** the notes should be this version's section, not
  the whole changelog.
- **The loader**, in addition: the release carries five images, a bash
  completion and `SHA256SUMS`, and is marked Latest:

  | Asset | Board | Execution state |
  | --- | --- | --- |
  | `rpi-loader-<version>-bcm2837-kernel7.img` | Pi 2, Pi 3 | AArch32 |
  | `rpi-loader-<version>-bcm2837-kernel8.img` | Pi 2 v1.2, Pi 3 | AArch64 |
  | `rpi-loader-<version>-bcm2711-kernel7.img` | Pi 4 | AArch32 |
  | `rpi-loader-<version>-bcm2711-kernel8.img` | Pi 4 | AArch64 |
  | `rpi-loader-<version>-bcm2835-kernel.img` | Pi 1, Pi Zero | AArch32 (ARMv6) |
  | `rpi-loader-<version>.bash` | — | host-side shell completion |

  The images are named per chip and version because a release page
  cannot hold several files all called `kernel7.img`, but the firmware
  loads *only* the bare names, so the release notes tell people to rename
  them on the way to the SD card. That instruction is the easiest thing
  here to get wrong. BCM2835 has no `kernel8.img`: the ARM1176 has no
  64-bit mode, and its image is `kernel.img` with no digit because
  `start.elf` picks the filename from the CPU it finds. The completion is
  the one asset whose absence would be silent, since nothing downstream
  reads it.

## Choosing the version

Semantic versioning throughout, with the usual pre-1.0 caveat that `0.x`
bumps the *minor* for breaking changes. Raising `rust-version` is at
least a minor release for every crate. What else counts as breaking
depends on the crate.

### rpi-hal

- Adding or removing a field on a public struct or enum variant. Several
  error types carry diagnostic fields, and trimming one is a breaking
  change (`sd::Error::CardError` was cut from eight fields to two before
  0.1.0 precisely because it was free to do then).
- Renaming or gating an existing public item behind a new feature.

Adding a feature, a driver, or a trait implementation is a minor bump.

### rpi-hal-embassy

**Its version tracks rpi-hal's.** Since 0.3.0 the two move together, so
a release here takes the number of the rpi-hal release it is built
against (0.2.0 was skipped to get them into step). The two crates are
always used together and share types, so matching numbers answer "which
HAL is this for?" without anyone reading a manifest. It also means a
release can be a bigger number than semver alone would ask for, which is
fine: skipping versions is allowed, reusing them is not.

What counts as breaking is wider than its own API:

- **The rpi-hal version requirement.** Raising it forces every consumer
  to move too, since the crates share types (`Lic`, the PAC).
- **The `embassy-*` version requirements.** `embassy-time-driver` and
  `embassy-executor` are the contract this crate implements; moving to a
  new major of either changes which `embassy-time` an application can use
  alongside it. That is breaking even when nothing here changes.
- **The tick rate.** `tick-hz-1_000_000` is pinned to the System Timer's
  fixed 1MHz. It should never move, but if it did, every `Duration` in
  every consumer would silently mean something else.

Adding a driver capability or an example is a minor bump.

### rpi-kickstart

**Its version is its own.** rpi-hal-embassy tracks rpi-hal's number
because the two share types and are always used together; that reasoning
does not reach a layer above. Plain semantic versioning applies.

What counts as breaking is wider than its own API:

- **The rpi-hal and rpi-hal-embassy version requirements.** Raising
  either forces every consumer to move too, since types cross the
  boundary in both directions.
- **Removing or renaming a feature.** Every capability is a feature, so
  the feature list is as much of the API as the types are. Adding one is
  a minor bump; taking one away is breaking even if the module survives
  under another name.
- **Changing what a sink expects.** The pluggable pieces, the clock above
  all, are contracts an application fills in. Changing the shape of one
  breaks a board that filled it, silently at compile time if the types
  still line up.

Adding a module or an example is a minor bump.

### rpi-loader-ota

Its own version, separate from the CLI's: its consumers are firmware
projects, and a renamed command-line flag is no reason to bump their
dependency. The CLI and rpi-kickstart depend on it by path, so a bump
that leaves their requirements behind fails their builds (step 2).

**What counts as breaking:** the Rust API as usual, and the bundle format
itself. A change to the container's bytes is breaking in a way a semver
bump cannot really express, because a bundle is parsed by the firmware
already running and installs the firmware that replaces it, so a board
can never be sent a container its current build does not understand.
Changing it means reaching every deployed board some other way once. The
version byte in the header exists to make that a clean rejection rather
than a puzzle.

### rpi-loader (the CLI and the firmware)

The CLI and the firmware carry the same version and move together, as
two halves of one wire protocol (see the note at the top of
`loader/CHANGELOG.md`). The release refuses to run if the two manifests
disagree. `loader/ota` is not part of that check.

What counts as breaking is wider than a Rust API, because most of what
the loader exposes is not one:

- **The wire protocol.** Any change to a command byte, a header layout,
  the chunk framing, or an error code breaks a CLI talking to an older
  loader, or the reverse. There is a version byte in the handshake for
  exactly this reason; a mismatch is currently a warning, so treat the
  protocol as the compatibility surface it is.
- **The CLI's arguments.** Renaming a subcommand or flag, or making an
  optional one required, breaks whatever scripts people wrapped around
  it.
- **The load addresses** the images are linked at, since an image is
  uploaded to an address a caller passes in.

## Releasing several crates at once

The usual case after a change that spans crates: one pull request bumps
every crate involved and dates every changelog, then all the tags go out
together.

```sh
git tag -a hal-v0.10.0 -m "rpi-hal 0.10.0"
git tag -a embassy-v0.10.0 -m "rpi-hal-embassy 0.10.0"
git tag -a kickstart-v0.3.0 -m "rpi-kickstart 0.3.0"
scripts/release/push-tags.sh hal-v0.10.0 embassy-v0.10.0 kickstart-v0.3.0
```

Each tag starts its own run, and they run in parallel. **Order doesn't
matter:** a run waits (`scripts/ci/wait-for-deps.sh`, up to 20 minutes)
until every crate it depends on in this repository is on crates.io at
the version this commit has. `embassy-v0.10.0` sits there until
`hal-v0.10.0` has published rpi-hal 0.10.0, and `kickstart-v0.3.0` until
both have. Approve them all; they finish in dependency order.

A crate whose dependency's new version still satisfies its requirement (a
patch release of rpi-hal, say) needs no bump and no release of its own.

The dependency order, for reference: rpi-hal and rpi-loader-ota have no
in-repo dependencies; rpi-hal-embassy depends on rpi-hal; rpi-kickstart
on rpi-hal, rpi-hal-embassy and rpi-loader-ota; the loader CLI on
rpi-loader-ota; the loader firmware (unpublished) on rpi-hal.

## What the automation enforces, and how it fails

| Guard | Where | Symptom if it trips |
| --- | --- | --- |
| The tag names a known crate and matches its manifest | `push-tags.sh`, then `release.yml` (`scripts/ci/release-meta.sh`) | The tag isn't pushed; or the run fails before publishing. A dispatch run has no tag and skips the match |
| The loader's CLI and firmware manifests agree | Same | Same |
| A dated changelog section for the version | `release.yml` (`scripts/ci/changelog.sh`) | Run fails before publishing |
| In-repo dependencies are on crates.io | `release.yml` (`scripts/ci/wait-for-deps.sh`) | Run waits, then fails after 20 minutes. Usually a dependency's release wasn't tagged, or failed |
| The packaged tarball builds | CI against this repository's crates (`scripts/ci/package-verify.sh`); the release against crates.io (`cargo package`) | CI fails on the pull request; or the run fails before publishing |
| A chip feature is selected for rpi-hal | `lib.rs`'s `compile_error!` | `cargo package`/`publish` with no `--features` aborts, because their verification build uses *default* features and no chip is a default. `release-meta.sh` passes `bcm2837` for exactly this reason |
| Images were actually produced | CI (`_loader.yml`) | CI fails on the pull request |
| Every crate builds on its declared MSRV | CI | Same |
| PRs and `ci-ok` required on `main` | Repository ruleset | Direct pushes rejected; unmerged PRs blocked |

One coupling to know about: the ruleset's required check is matched by
**job name**, `ci-ok` in `ci.yml`. Renaming it leaves the ruleset waiting
on a name that never reports, and every PR blocks until the ruleset is
updated too. It fails closed, which is the safe direction, but it is a
puzzling half hour if you have forgotten why.

Release runs are written to be re-runnable. Release notes are rewritten
rather than appended, loader assets are uploaded with `--clobber`, and
the publish step first asks the crates.io index whether the version
exists. Without that, a re-run would die on "crate version already
uploaded" and never reach whatever it was re-run for.

## If something goes wrong

- **A run failed partway.** Nothing reached crates.io unless the
  `Publish` step itself succeeded. Fix the cause and re-run it from the
  Actions UI, or dispatch `release.yml` with the crate as `component`;
  there's no need to move the tag.
- **A dependency's run failed in a batch.** The runs that depend on it
  wait 20 minutes and then fail too. Fix and re-run the dependency, then
  re-run the others.
- **The loader images are wrong but the publish succeeded.** Rebuild and
  re-upload: release assets, unlike a crates.io version, can be replaced.
  Re-run the workflow, or `gh release upload loader-v<version> <files> --clobber`.
- **A bad version reached crates.io.** It cannot be replaced. Yank it
  (`cargo yank --version <version> <crate>`), which leaves existing
  lockfiles working but stops new dependents from selecting it, then
  release a fix under a new version number.
- **The tag is wrong but nothing is published.** Delete it locally and on
  the remote (`git tag -d <tag>`, `git push --delete origin <tag>`) and
  start again from step 5. Once a version *is* published, leave its tag
  alone.

## The changelog needs no reopening

Keep a Changelog suggests holding an empty `## [Unreleased]` section open
at all times. Don't: with a protected `main`, creating it is a commit and
a pull request whose entire content is a heading with nothing under it.

Instead the section is created by **whichever change first needs it**,
in that change's own pull request: the PR that adds a driver adds the
heading above its own bullet. The heading then exists exactly when there
is something to put under it, and step 2 renames it. If a release happens
to contain only changes that warranted no entry, there is no heading to
rename and step 2 writes the version heading directly.

The same reasoning applies to post-release version bumps, which is why
there is no `-dev` version step either: each manifest carries the last
released version between releases, and step 2 is where it moves.
