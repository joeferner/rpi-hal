# Build/lint orchestration for rpi-hal-embassy. The default (AArch32)
# target and the linker-script rustflags are pinned in
# .cargo/config.toml, so plain `cargo` invocations pick them up without
# repeating flags here. Build the other architecture with
# `--target aarch64-unknown-none-softfloat`.

.PHONY: build examples fmt fmt-check clippy doc package pre-commit clean

build:
	cargo build --release

examples:
	cargo build --release --examples

fmt:
	cargo fmt

fmt-check:
	cargo fmt -- --check

clippy:
	cargo clippy --release --examples -- -D warnings

# `-D warnings` is the whole point: a plain doc build almost never fails, so
# without it this catches nothing. What it does catch is broken intra-doc
# links -- including the non-obvious case where a module's own `//!` links
# resolve in the *crate root's* scope, because they get merged with the
# outer doc comment on the `pub mod` declaration in lib.rs.
doc:
	RUSTDOCFLAGS="-D warnings" cargo doc --no-deps

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
