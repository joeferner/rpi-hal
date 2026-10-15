# Build/lint orchestration for rpi-hal-embassy. The default (AArch32)
# target and the linker-script rustflags are pinned in
# .cargo/config.toml, so plain `cargo` invocations pick them up without
# repeating flags here. Build the other architecture with
# `--target aarch64-unknown-none-softfloat`.

.PHONY: build build-armv6 build-aarch64 examples examples-armv6 \
	examples-aarch64 fmt fmt-check clippy clippy-armv6 doc package \
	pre-commit clean

# The ARMv6 (BCM2835) invocation, shared by the three recipes below so the
# flags cannot drift apart between building and linting. Four things it
# does that the default does not, all of them consequences of the Pi Zero
# being a different instruction set rather than a different chip:
#
#   `+nightly -Z build-std=core`: `armv6-none-eabi` is tier 3, so rustup
#   publishes no `core` for it and one has to be compiled. This is the
#   only part of this repository that is not on the pinned stable
#   toolchain; it needs `rustup toolchain install nightly --component
#   rust-src` once.
#
#   `--no-default-features --features bcm2835`: the chip is this crate's
#   own default feature (see Cargo.toml) and `rpi-hal` prefers `bcm2837`
#   when both are on, so leaving the default in place would compile the
#   Pi 3's peripheral base into a Pi Zero binary.
#
# `multicore` never appears here: the BCM2835 has one core, and `rpi-hal`
# rejects that combination with a `compile_error!` rather than quietly
# dropping the module -- so there is no `--all-features` pass for this
# target the way there is above.
ARMV6 := --release --target armv6-none-eabi -Z build-std=core \
	--no-default-features --features bcm2835

build:
	cargo build --release

build-armv6:
	cargo +nightly build $(ARMV6)

# The feature set changes which examples exist -- cargo silently skips a
# target whose `required-features` are unmet rather than reporting it --
# so a plain build is not full coverage. `EXAMPLE_FEATURES` is every
# feature *except* `irq-dispatch`, which cannot be in a sweep: it defines
# `__irq_handler`, and every example but one defines its own, so building
# them together is a duplicate symbol. That example is named on its own
# line below, which is also the only place the feature is exercised.
EXAMPLE_FEATURES := multicore,async,embassy-net-driver,wifi

examples:
	cargo build --release --examples
	cargo build --release --examples --features $(EXAMPLE_FEATURES)
	cargo build --release --features irq-dispatch --example embassy_blink

examples-armv6:
	cargo +nightly build $(ARMV6) --examples

# The other architecture. The examples are the only thing that links, and
# linking is where a missing `rpi-link.x` or a stray RUSTFLAGS shows up --
# so this is worth having even though the source is shared.
#
# These recipes exist rather than the equivalent `cargo` lines living in
# `ci.yml` so that `EXAMPLE_FEATURES` above is the only place that knows
# which features sweep the examples. When it was written out in both, the
# two drifted the moment `irq-dispatch` was added: the Makefile learned to
# stop using `--all-features` and the workflow did not, so `make
# pre-commit` passed and CI failed on a duplicate `__irq_handler`.
AARCH64 := --release --target aarch64-unknown-none-softfloat

build-aarch64:
	cargo build $(AARCH64)

examples-aarch64:
	cargo build $(AARCH64) --examples
	cargo build $(AARCH64) --examples --features $(EXAMPLE_FEATURES)
	cargo build $(AARCH64) --features irq-dispatch --example embassy_blink

fmt:
	cargo fmt

fmt-check:
	cargo fmt -- --check

clippy:
	cargo clippy --release --examples -- -D warnings
	cargo clippy --release --examples --features $(EXAMPLE_FEATURES) -- -D warnings
	# Separate for the same reason as in `examples` above.
	cargo clippy --release --features irq-dispatch --example embassy_blink -- -D warnings

clippy-armv6:
	cargo +nightly clippy $(ARMV6) --examples -- -D warnings

# `-D warnings` is the whole point: a plain doc build almost never fails, so
# without it this catches nothing. What it does catch is broken intra-doc
# links -- including the non-obvious case where a module's own `//!` links
# resolve in the *crate root's* scope, because they get merged with the
# outer doc comment on the `pub mod` declaration in lib.rs.
#
# `--all-features` because the feature-gated modules are otherwise not
# documented at all, and so not checked at all: a broken link in one of
# them passed a plain `cargo doc` for as long as the feature existed.
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

# AArch64 is in here rather than left to CI because it is the one
# architecture whose linking this cannot otherwise prove: the ARMv6 and
# ARMv7 builds above share a linker script and a feature set, and AArch64
# is where the two example sweeps differ. Leaving it out is what let a
# duplicate-symbol failure reach CI green-locally.
pre-commit: fmt clippy clippy-armv6 build build-armv6 build-aarch64 examples \
	examples-armv6 examples-aarch64 doc

clean:
	cargo clean
