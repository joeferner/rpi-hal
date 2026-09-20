# Build/lint orchestration for rpi-loader. The firmware is a package of
# its own in `firmware/`, so every recipe below runs cargo from there
# rather than from the repository root -- `--manifest-path` would not do,
# because cargo discovers `.cargo/config.toml` by walking up from the
# *working directory*, and that file is what pins the AArch32 target. The
# AArch64 build overrides the target explicitly via ARCH64.
#
# That same discovery rule is why the firmware sits in a subdirectory at
# all: a root-level config naming a bare metal target would be inherited
# by the host CLI beside it, which then fails to build (it compiles host
# code for `armv7a-none-eabi` and stops at the missing `#[panic_handler]`).
# The repository root deliberately has no cargo configuration.
#
# Both packages build on stable. The firmware pins a toolchain only to
# pick up the two bare-metal targets and the components these recipes
# invoke, not to reach an unstable feature.
#
# `build-bcm2711`/`build64-bcm2711` -> Pi 4 (BCM2711)
# `build-bcm2837`/`build64-bcm2837` -> Pi 2/3 (BCM2836/2837)
#   firmware/target/kernel7.img (AArch32, loads at 0x8000)
#   firmware/target/kernel8.img (AArch64, loads at 0x80000)
#
# Which one boots is a firmware choice: with arm_64bit=1 in config.txt
# the firmware loads kernel8.img, otherwise it defaults to kernel7.img.
#
# rpi-hal's `bcm2837` feature is always on (baked into the firmware's own
# Cargo.toml dependency line, since it has no chip-neutral default to fall
# back to) -- `--features bcm2711` below adds rpi-hal's `bcm2711` feature
# alongside it, which wins the PAC/memory-map selection (see rpi-hal's
# Cargo.toml); `bcm2837-lpa` still compiles in for that build, just
# unused. `cargo objcopy` needs the same `--features` as `cargo build` on
# every target below -- it re-invokes `build` internally, and would
# silently relink without them otherwise.

ARCH64 := aarch64-unknown-none-softfloat
# Named here because the OTA package has no `.cargo/config.toml` of its own
# to default it -- and deliberately so, since it is a library that also
# builds for the host and must not have a target imposed on it. The
# firmware's recipes below never mention the 32-bit target for the opposite
# reason: `firmware/.cargo/config.toml` already makes it the default there.
ARCH32 := armv7a-none-eabi
# The ARMv6 target for `build-bcm2835`, named for the same reason as
# ARCH64: `firmware/.cargo/config.toml` defaults to ARCH32, and this
# overrides it.
ARCH6 := armv6-none-eabi
FIRMWARE := firmware
CLI := cli
OTA := ota

.PHONY: build-bcm2711 build64-bcm2711 build-bcm2837 build64-bcm2837 \
	build-bcm2835 build-cli \
	fmt fmt-check clippy clippy6 clippy64 clippy-cli clippy-ota test-cli \
	test-ota doc package package-ota pre-commit clean

build-bcm2711:
	cd $(FIRMWARE) && cargo build --release --features bcm2711
	cd $(FIRMWARE) && cargo objcopy --release --features bcm2711 -- -O binary target/kernel7.img

build64-bcm2711:
	cd $(FIRMWARE) && cargo build --release --target $(ARCH64) --features bcm2711
	cd $(FIRMWARE) && cargo objcopy --release --target $(ARCH64) --features bcm2711 -- -O binary target/kernel8.img

build-bcm2837:
	cd $(FIRMWARE) && cargo build --release
	cd $(FIRMWARE) && cargo objcopy --release -- -O binary target/kernel7.img

# Pi 1 / Pi Zero (BCM2835). The odd one out in three ways, all of them
# consequences of the chip being ARMv6 rather than ARMv7-A:
#
#   - The target is named here, and it is `armv6-none-eabi`. There is no
#     64-bit counterpart recipe: the ARM1176 has no 64-bit mode.
#   - It is the one build in this repository that needs nightly. That
#     target is tier 3, so rustup publishes no `core` for it and
#     `-Z build-std` has to compile one. `+nightly` rather than a
#     `rust-toolchain.toml` change, so everything else stays on stable;
#     it needs `rustup toolchain install nightly --component rust-src`
#     once.
#   - The image is `kernel.img`, with no digit. `start.elf` picks the
#     kernel filename from the CPU it finds, so a Pi Zero handed a
#     `kernel7.img` looks for a file that is not there and stops, with
#     nothing on the console to say so.
#
# `--no-default-features` because the chip is this package's own default
# feature and rpi-hal prefers `bcm2837` when both are on -- see
# firmware/Cargo.toml.
build-bcm2835:
	cd $(FIRMWARE) && cargo +nightly build --release -Z build-std=core --target $(ARCH6) --no-default-features --features bcm2835
	cd $(FIRMWARE) && cargo +nightly objcopy --release -Z build-std=core --target $(ARCH6) --no-default-features --features bcm2835 -- -O binary target/kernel.img

build64-bcm2837:
	cd $(FIRMWARE) && cargo build --release --target $(ARCH64)
	cd $(FIRMWARE) && cargo objcopy --release --target $(ARCH64) -- -O binary target/kernel8.img

# The host CLI, built for whatever the host is -- no target override, no
# `objcopy`. This is the package that gets published to crates.io.
build-cli:
	cd $(CLI) && cargo build --release

fmt:
	cd $(FIRMWARE) && cargo fmt
	cd $(CLI) && cargo fmt
	cd $(OTA) && cargo fmt

fmt-check:
	cd $(FIRMWARE) && cargo fmt -- --check
	cd $(CLI) && cargo fmt -- --check
	cd $(OTA) && cargo fmt -- --check

clippy:
	cd $(FIRMWARE) && cargo clippy --release -- -D warnings

clippy64:
	cd $(FIRMWARE) && cargo clippy --release --target $(ARCH64) -- -D warnings

# See `build-bcm2835` for the nightly and the feature flags.
clippy6:
	cd $(FIRMWARE) && cargo +nightly clippy --release -Z build-std=core --target $(ARCH6) --no-default-features --features bcm2835 -- -D warnings

clippy-cli:
	cd $(CLI) && cargo clippy --release --all-targets -- -D warnings

# Twice, and the second pass is the one that earns its keep. The OTA
# package is `no_std`, but a host build links `std` transitively and never
# says so, which means an accidental `std::` would sail through every check
# above and fail for the first person to put it on a board. Building it for
# a target that has no `std` at all is the only thing that catches it.
#
# One bare metal target rather than both of the loader's: nothing in the
# package is architecture-specific, and the absence of `std` is a property
# of the target family, not of the instruction set. Two builds would
# re-check the same thing.
#
# `--all-features` on both, because a feature that only compiles for the
# host is not a feature this package can offer. `--all-targets` on the host
# pass only -- it pulls in the test harness, which needs `std` by
# definition and cannot build for a bare metal target.
clippy-ota:
	cd $(OTA) && cargo clippy --release --all-targets --all-features -- -D warnings
	cd $(OTA) && cargo clippy --release --all-features --target $(ARCH32) -- -D warnings

# The CLI's tests drive the real binary against a fake device on a pty, so
# they need no hardware -- but they also prove nothing about timing, which
# is the half of this protocol only a real board can exercise. There is no
# `test` target for the firmware: its tests would have to run on the
# device, and nothing here can do that.
test-cli:
	cd $(CLI) && cargo test --release

# On the host, with no hardware and no fake device on a pty: the container
# is pure code, so unlike everything else here a test of it proves the
# whole of what it claims. `--all-features` so the install half is covered
# too and not just the format.
test-ota:
	cd $(OTA) && cargo test --release --all-features

# `-D warnings` is the whole point: a plain doc build almost never fails, so
# without it this catches nothing. What it does catch is broken intra-doc
# links -- including the non-obvious case where a module's own `//!` links
# resolve in the *crate root's* scope, because they get merged with the
# outer doc comment on the `pub mod` declaration.
#
# One target only, unlike clippy above: rustdoc's link resolution doesn't
# depend on the architecture, so documenting both would re-check the same
# links for the sake of the little code that is arch-gated.
#
# `--all-features` on the OTA package, matching what docs.rs is told to
# build. Without it the feature-gated half is never documented here, and a
# broken link inside it is found by the docs.rs builder after the release
# that published it.
doc:
	cd $(FIRMWARE) && RUSTDOCFLAGS="-D warnings" cargo doc --no-deps
	cd $(CLI) && RUSTDOCFLAGS="-D warnings" cargo doc --no-deps
	cd $(OTA) && RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features

# What `cargo publish` will verify: it builds the packaged tarball, which
# catches the "works in this working copy, broken on crates.io" class of
# problem -- a file the build needs that packaging left out, or a path
# that only resolves here. Two of the three packages are publishable; the
# firmware sets `publish = false`. cargo refuses a dirty working tree here
# on its own, which is the behaviour we want: what gets published is the
# committed state, not what happens to be on disk.
#
# The separate CARGO_TARGET_DIR is not tidiness. The verification build
# compiles the *extracted tarball* (target/package/rpi-loader-<version>/)
# with the dev profile, and sharing the normal target directory lets it
# overwrite target/debug/rpi-loader and leave a fingerprint whose source
# paths point into that extracted copy. Those files never change again,
# so every later `cargo build`/`cargo run` reports "Finished" without
# recompiling and silently runs the packaged binary -- edits to src/ have
# no effect at all until `cargo clean -p rpi-loader`. Isolating the
# target directory keeps the verify build from touching the one the
# normal builds use.
package:
	cd $(CLI) && CARGO_TARGET_DIR=target/verify cargo package

# Separate from `package` rather than another line inside it, because the
# two are released independently -- the CLI and the firmware ship as one
# version, and the OTA package carries its own. A target that verified both
# would imply they move together.
package-ota:
	cd $(OTA) && CARGO_TARGET_DIR=target/verify cargo package

pre-commit: fmt clippy clippy6 clippy64 clippy-cli clippy-ota build-bcm2711 build64-bcm2711 build-bcm2837 build64-bcm2837 build-bcm2835 build-cli test-cli test-ota doc

clean:
	cd $(FIRMWARE) && cargo clean
	cd $(CLI) && cargo clean
	cd $(OTA) && cargo clean
