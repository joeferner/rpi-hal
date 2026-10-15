# Releasing

How to cut a release of `rpi-hal-embassy`. Maintainer-facing; nothing
here is needed to *use* the crate.

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

  Secrets do not carry over between repositories, so the ones set on
  `rpi-hal` or `rpi-loader` do not cover this crate.

- **The `crates-io` environment.** `.github/workflows/release.yml`
  declares it. Create it under Settings → Environments and add yourself as
  a **required reviewer**: the tag push then parks the workflow at
  "waiting for approval" and gives one last look before the irreversible
  step.

- **Repository visibility.** `Cargo.toml`'s `repository` and
  `documentation` fields, the README badges, and the changelog's version
  links all point at GitHub. While the repository is private, every one of
  those is a 404 for anyone reading the crates.io page.

## Per-release steps

### 1. Decide the version

**This crate's version tracks `rpi-hal`'s.** Since 0.3.0 the two move
together, so a release here takes the number of the HAL release it is
built against — 0.2.0 was skipped to get them into step. The two crates
are always used together and share types, so matching numbers answer
"which HAL is this for?" without anyone reading a manifest. It also means
a release can be a number bigger than semver alone would ask for, which
is fine: skipping versions is allowed, reusing them is not.

Otherwise semantic versioning, with the usual pre-1.0 caveat that `0.x`
bumps the *minor* for breaking changes. What counts as breaking here is
wider than this crate's own API:

- **The `rpi-hal` version requirement.** Raising it forces every consumer
  to move too, since the two crates share types (`Lic`, the PAC).

  Check `Cargo.toml` for a `[patch.crates-io]` override on it before
  releasing. One is there while a feature here depends on `rpi-hal` API
  that is written but not yet published; the requirement has to be raised
  to the release carrying it and the `[patch]` deleted, in the same
  change. `cargo package --features <the feature>` is what catches a
  release where that was forgotten, since `[patch]` is ignored there.
- **The `embassy-*` version requirements.** `embassy-time-driver` and
  `embassy-executor` are the contract this crate implements; moving to a
  new major of either changes which `embassy-time` an application can use
  alongside it. That is breaking even when nothing here changes.
- **The tick rate.** `tick-hz-1_000_000` is pinned to the System Timer's
  fixed 1MHz. It should never move, but if it did, every `Duration` in
  every consumer would silently mean something else.
- Raising `rust-version`. An MSRV bump is at least a minor release.

Adding a driver capability or an example is a minor bump.

### 2. Bump the version and update the changelog

On a branch — the `main` ruleset requires a pull request, so nothing goes
in directly:

```sh
git checkout -b release-<version>
```

- `Cargo.toml`: set `version`.
- `make build` — refreshes `Cargo.lock`, which is tracked and would
  otherwise be stale in the published tarball.
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
make pre-commit      # fmt, clippy, library, examples, docs
make package         # what `cargo publish` will verify
```

`make package` refuses a dirty working tree, which is deliberate: what
gets published is the committed state, not what happens to be on disk.

### 5. Tag and push

```sh
git tag -a v<version> -m "rpi-hal-embassy <version>"
git push origin v<version>
```

The tag **must** start with `v` — that is the workflow's trigger pattern,
and a bare `0.2.0` silently does nothing at all. It must also match
`Cargo.toml`'s version, which the workflow checks and fails on.

### 6. Approve and watch

```sh
gh run watch
```

The release job re-verifies everything, creates the GitHub release from
the changelog section, and only then publishes. If you set up the required
reviewer, approve it in the Actions UI when it parks.

### 7. Verify the publish

```sh
open https://crates.io/crates/rpi-hal-embassy
open https://docs.rs/crate/rpi-hal-embassy/<version>/builds
```

The docs.rs build is the one thing CI cannot prove. docs.rs builds for a
host target by default, and nothing in this dependency graph compiles for
one — `rpi-hal` is bare-metal ARM. `[package.metadata.docs.rs]` names
`armv7a-none-eabi` as the default target and AArch64 as a second; if that
page shows a failure, that section is where to look.

## What the automation enforces, and how it fails

| Guard | Where | Symptom if it trips |
| --- | --- | --- |
| Tag matches `Cargo.toml` version | `release.yml` | Release job fails before publishing |
| Changelog has a dated section for the version | `release.yml` | Same |
| Packaged tarball actually builds | `make package`, in both CI and the release job | Same |
| Library builds on the declared MSRV | `ci.yml` | CI fails on the pull request, long before a tag exists |
| Both architectures build, library and examples | `ci.yml` | Same |
| PRs required on `main` | Repository ruleset | Direct pushes rejected |

One coupling to know about: the ruleset's required status checks are
matched against the **job names** in `ci.yml`. Renaming a job there leaves
the ruleset waiting on a name that never reports, and every PR blocks
until the ruleset is updated too. It fails closed, which is the safe
direction, but it is a puzzling half hour if you have forgotten why.

The release job is written to be re-runnable: the release notes are
rewritten rather than appended, and the publish step first asks the
crates.io index whether the version exists — otherwise a re-run would die
on "crate version already uploaded" and never reach whatever it was
re-run for.

## If something goes wrong

- **The publish failed partway.** Nothing was uploaded unless the
  `Publish` step itself succeeded. Fix the cause and re-run the workflow
  from the Actions UI (`workflow_dispatch`) — no need to move the tag.
  Note that a dispatch run skips the tag-match check, since there is no
  tag in its context.
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
that change's own pull request — the PR that adds a capability adds the
heading above its own bullet. The heading then exists exactly when there
is something to put under it, and step 2 renames it.

The same reasoning applies to post-release version bumps, which is why
there is no `0.2.0-dev` step here either: `Cargo.toml` carries the last
released version between releases, and step 2 is where it moves.
