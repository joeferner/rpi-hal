# Build/lint orchestration for rpi-kickstart. The default (AArch32) target
# and the linker-script rustflags are pinned in .cargo/config.toml, so
# plain `cargo` invocations pick them up without repeating flags here.
#
# AArch64 is named explicitly below rather than made a second default,
# because there is no way to ask cargo for two. Every recipe that builds
# anything therefore runs twice: this crate's source is architecture-
# neutral, but its examples link, and linking is where a load address or a
# missing `rpi-link.x` shows up.
#
# ARMv6 (Pi 1, Pi Zero) is deliberately absent. `rpi-hal` and
# `rpi-hal-embassy` both carry recipes for it, at the cost of a nightly
# toolchain and `-Z build-std` -- that target is tier 3 and rustup
# publishes no `core` for it. Nothing here is architecture-specific, so
# adding those recipes is a `make` change and not a source one, and it can
# wait for a board that wants it.

# Both architectures in one invocation. `--target` may be repeated, which
# is what keeps each recipe below to two lines instead of four -- the
# second dimension being the feature set, and that one cannot be folded
# away.
ARCHES := --target armv7a-none-eabi --target aarch64-unknown-none-softfloat

.PHONY: build examples fmt fmt-check clippy doc package pre-commit clean

# Twice, because a feature-gated module is not compiled at all without its
# feature: the plain pass proves the crate is usable with nothing turned
# on, and `--all-features` is the only pass that sees the modules.
build:
	cargo build --release $(ARCHES)
	cargo build --release $(ARCHES) --all-features

# Same two passes, and here the second one is load-bearing for a different
# reason: cargo silently *skips* an example whose `required-features` are
# unmet rather than reporting it, so without `--all-features` an example
# can rot indefinitely while the build stays green.
#
# `--all-features` will stop being the right spelling the moment two
# features conflict -- `rpi-hal-embassy` reached that point with an
# example that defines `__irq_handler` -- at which point this becomes an
# enumerated list.
examples:
	cargo build --release $(ARCHES) --examples
	cargo build --release $(ARCHES) --examples --all-features

fmt:
	cargo fmt

fmt-check:
	cargo fmt -- --check

# `--examples` included so the lint runs over them too: they are the only
# code here that talks to hardware, which makes them the code most likely
# to earn a lint.
clippy:
	cargo clippy --release $(ARCHES) --examples -- -D warnings
	cargo clippy --release $(ARCHES) --examples --all-features -- -D warnings

# `-D warnings` is the whole point: a plain doc build almost never fails,
# so without it this catches nothing. What it does catch is broken
# intra-doc links -- including the non-obvious case where a module's own
# `//!` links resolve in the *crate root's* scope, because they get merged
# with the outer doc comment on the `pub mod` declaration in lib.rs.
#
# `--all-features` because a feature-gated module is otherwise not
# documented at all, and so not checked at all.
doc:
	RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features

# What `cargo publish` will verify: it builds the packaged tarball, which
# catches the "works in this working copy, broken on crates.io" class of
# problem. cargo refuses a dirty working tree here on its own, which is
# the behaviour we want -- what gets published is the committed state.
#
# The separate CARGO_TARGET_DIR is not tidiness. The verification build
# compiles the extracted tarball with the dev profile, and sharing the
# normal target directory lets it leave a fingerprint whose source paths
# point into that extracted copy -- after which every later `cargo build`
# reports "Finished" without recompiling, and edits to src/ have no effect
# until `cargo clean`.
package:
	CARGO_TARGET_DIR=target/verify cargo package

pre-commit: fmt clippy build examples doc

clean:
	cargo clean
