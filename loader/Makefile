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
FIRMWARE := firmware
CLI := cli

.PHONY: build-bcm2711 build64-bcm2711 build-bcm2837 build64-bcm2837 build-cli \
	fmt fmt-check clippy clippy64 clippy-cli test-cli doc package pre-commit clean

build-bcm2711:
	cd $(FIRMWARE) && cargo build --release --features bcm2711
	cd $(FIRMWARE) && cargo objcopy --release --features bcm2711 -- -O binary target/kernel7.img

build64-bcm2711:
	cd $(FIRMWARE) && cargo build --release --target $(ARCH64) --features bcm2711
	cd $(FIRMWARE) && cargo objcopy --release --target $(ARCH64) --features bcm2711 -- -O binary target/kernel8.img

build-bcm2837:
	cd $(FIRMWARE) && cargo build --release
	cd $(FIRMWARE) && cargo objcopy --release -- -O binary target/kernel7.img

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

fmt-check:
	cd $(FIRMWARE) && cargo fmt -- --check
	cd $(CLI) && cargo fmt -- --check

clippy:
	cd $(FIRMWARE) && cargo clippy --release -- -D warnings

clippy64:
	cd $(FIRMWARE) && cargo clippy --release --target $(ARCH64) -- -D warnings

clippy-cli:
	cd $(CLI) && cargo clippy --release --all-targets -- -D warnings

# The CLI's tests drive the real binary against a fake device on a pty, so
# they need no hardware -- but they also prove nothing about timing, which
# is the half of this protocol only a real board can exercise. There is no
# `test` target for the firmware: its tests would have to run on the
# device, and nothing here can do that.
test-cli:
	cd $(CLI) && cargo test --release

# `-D warnings` is the whole point: a plain doc build almost never fails, so
# without it this catches nothing. What it does catch is broken intra-doc
# links -- including the non-obvious case where a module's own `//!` links
# resolve in the *crate root's* scope, because they get merged with the
# outer doc comment on the `pub mod` declaration.
#
# One target only, unlike clippy above: rustdoc's link resolution doesn't
# depend on the architecture, so documenting both would re-check the same
# links for the sake of the little code that is arch-gated.
doc:
	cd $(FIRMWARE) && RUSTDOCFLAGS="-D warnings" cargo doc --no-deps
	cd $(CLI) && RUSTDOCFLAGS="-D warnings" cargo doc --no-deps

# What `cargo publish` will verify: it builds the packaged tarball, which
# catches the "works in this working copy, broken on crates.io" class of
# problem -- a file the build needs that packaging left out, or a path
# that only resolves here. Only the CLI is publishable; the firmware sets
# `publish = false`. cargo refuses a dirty working tree here on its own,
# which is the behaviour we want: what gets published is the committed
# state, not what happens to be on disk.
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

pre-commit: fmt clippy clippy64 clippy-cli build-bcm2711 build64-bcm2711 build-bcm2837 build64-bcm2837 build-cli test-cli doc

clean:
	cd $(FIRMWARE) && cargo clean
	cd $(CLI) && cargo clean
