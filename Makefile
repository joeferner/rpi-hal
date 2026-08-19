# Build/lint orchestration for rpi-hal. Target and -Zbuild-std=core are
# already pinned in .cargo/config.toml, so plain `cargo` invocations
# pick them up without repeating flags here.

.PHONY: build-bcm2837 build-bcm2711 examples fmt fmt-check clippy doc package pre-commit clean

# `bcm2837`/`bcm2711` (see Cargo.toml) are chip selection: neither is a
# default feature, since there's no sensible default target chip, so
# every invocation below picks one explicitly.
build-bcm2837:
	cargo build --release --features bcm2837

# `bcm2711` is preliminary: no interrupt
# controller exists yet (`lic` is `cfg`'d out under this feature), which
# most examples depend on, so this only checks the library, not
# `--examples` -- a lightweight regression guard for the memory-map/PAC
# selection itself, not full example coverage.
build-bcm2711:
	cargo build --release --features bcm2711

examples:
	cargo build --release --examples --features bcm2837
	# The multicore examples have required-features = ["multicore"], so a
	# plain --examples build skips them -- build them explicitly so they're
	# covered.
	cargo build --release --features bcm2837,multicore --example multicore_blink --example multicore_uart --example multicore_id
	# Likewise the integration-adapter examples are gated on their own
	# features (embedded-sdmmc, smoltcp) and skipped by a plain build.
	cargo build --release --features bcm2837,embedded-sdmmc,smoltcp --example sd_fat_read --example usb_ethernet_smoltcp --example bt_probe --example ble_advertise --example ble_scan
	# Same again for the v3d examples, gated on `v3d` (BCM2837-only).
	cargo build --release --features bcm2837,v3d --example v3d_probe --example gpu_cube
	# And the video decoder, gated on `mmal` (which pulls in `vchiq`) plus
	# `embedded-sdmmc` for the stream it plays off the card.
	cargo build --release --features bcm2837,mmal,embedded-sdmmc --example h264_decode

fmt:
	cargo fmt

fmt-check:
	cargo fmt -- --check

clippy:
	cargo clippy --release --examples --features bcm2837 -- -D warnings
	# Same as `examples` above: without the feature, the multicore code
	# path (src/multicore.rs and the examples) isn't linted at all.
	cargo clippy --release --features bcm2837,multicore --example multicore_blink --example multicore_uart --example multicore_id -- -D warnings
	# Same again for the integration-adapter examples and their src-side
	# adapters, which a plain lint doesn't compile.
	cargo clippy --release --features bcm2837,embedded-sdmmc,smoltcp --example sd_fat_read --example usb_ethernet_smoltcp --example bt_probe --example ble_advertise --example ble_scan -- -D warnings
	# Library-only lint for BCM2711 -- see `build-bcm2711`'s comment on why
	# examples aren't included.
	cargo clippy --release --features bcm2711 -- -D warnings
	# Same again for the v3d examples, gated on `v3d` (BCM2837-only).
	cargo clippy --release --features bcm2837,v3d --example v3d_probe --example gpu_cube -- -D warnings
	# And for the video decoder and the VCHIQ/MMAL stack under it, gated on
	# `mmal` -- none of which a plain lint compiles either.
	cargo clippy --release --features bcm2837,mmal,embedded-sdmmc --example h264_decode -- -D warnings

# `-D warnings` is the whole point: a plain doc build almost never fails, so
# without it this catches nothing -- broken intra-doc links are the main
# reason to build docs here at all. Two builds, one per chip feature
# (`--all-features` would turn both on at once, which isn't a build anyone
# actually uses -- `pac` would resolve to whichever chip's `cfg` wins,
# silently documenting one chip's memory map while claiming to cover both),
# plus every other feature so the feature-gated modules (multicore,
# embedded-sdmmc, smoltcp, ...) are documented at all.
#
# One rustdoc quirk to know when a new module-header link fails here: a module
# documented *both* by an outer `///` on its `pub mod` line in lib.rs and by
# its own `//!` header has the two merged, and the merged text resolves
# intra-doc links in the declaration's scope -- the crate root -- where the
# module's own items aren't visible. So a bare `[`Sd::read_block`]` in sd.rs
# fails with "no item named `Sd` in scope" even though it reads as correct;
# qualifying the path (`[`read_block`](crate::sd::Sd::read_block)`) fixes it.
# Item-level docs are unaffected, which is why only module headers are hit.
#
# `--cfg docsrs` on the first build mirrors what docs.rs passes (see
# `[package.metadata.docs.rs]` in Cargo.toml), so the nightly-only `doc_cfg`
# path lib.rs gates behind that cfg is exercised here rather than first
# failing on the docs.rs builder after a release is already published.
doc:
	RUSTDOCFLAGS="-D warnings --cfg docsrs" cargo doc --no-deps --features bcm2837,multicore,async,embedded-sdmmc,smoltcp,embassy-net-driver,v3d,mmal
	RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --features bcm2711,multicore,async,embedded-sdmmc,smoltcp,embassy-net-driver

# A chip feature is not optional here, and the reason is easy to trip over:
# `cargo package` and `cargo publish` finish by building the packaged tarball,
# and that verification build uses *default* features -- which select no chip,
# so `lib.rs`'s `compile_error!` fires and the publish aborts on a package that
# is actually fine. Naming a chip gives the verify step a configuration that
# exists. `cargo publish` takes the same flag when the time comes.
#
# Kept out of `pre-commit`: it reaches the network to update the crates.io
# index, which a local commit check shouldn't need.
#
# One thing this does not check, so it belongs in whatever runs the release:
# `CHANGELOG.md`'s top heading carries a literal `ReleaseDate` placeholder
# (the `cargo-release` convention) that has to become the actual date before
# publishing -- automatically if that tool is doing the release, by hand
# otherwise. `grep ReleaseDate CHANGELOG.md` is the check.
package:
	cargo package --features bcm2837

pre-commit: fmt clippy build-bcm2837 build-bcm2711 examples doc

clean:
	cargo clean
