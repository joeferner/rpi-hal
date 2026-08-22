# Build/run orchestration for the hardware-in-the-loop bench.
#
# Three toolchains live under this directory and none of them share a
# workspace with rpi-hal, so every target below changes directory first
# rather than relying on the parent's cargo configuration.

.PHONY: firmware-rp2040 firmware-rp2350 flash-rp2040 flash-rp2350 tools cases case images test test-board test-board-manual test-board64 test-board32 test-handoff test-loopback test-manual test-all lint lint-py fmt fmt-check pre-commit clean

# Flashing needs a host tool that cargo invokes as the target runner. When
# it is absent cargo reports a bare "No such file or directory", naming
# neither the tool nor how to get it, so the flash targets check first.
define need_tool
@command -v $(1) >/dev/null || { \
	printf '%s not found on PATH.\n  %s\n' '$(1)' '$(2)'; \
	exit 1; \
}
endef

# Reports which flashing tools are present. Neither is a project
# dependency -- they are one-time host installs.
tools:
	@command -v elf2uf2-rs >/dev/null \
		&& echo "elf2uf2-rs: $$(command -v elf2uf2-rs)  [RP2040]" \
		|| echo "elf2uf2-rs: MISSING  -- run: cargo install elf2uf2-rs"
	@command -v picotool >/dev/null \
		&& echo "picotool:   $$(command -v picotool)  [RP2350]" \
		|| echo "picotool:   MISSING  -- distro package, or build from https://github.com/raspberrypi/picotool"

# The fixture firmware. `rp2040` is the default feature (a Pico), while the
# Olimex PICO2-XL needs the RP2350 target and feature set -- the pin map
# differs, not the logic.
firmware-rp2040:
	cd firmware && cargo build --release

firmware-rp2350:
	cd firmware && cargo build --release --target thumbv8m.main-none-eabihf \
		--no-default-features --features rp235x

# Flashes a board already in BOOTSEL. The runner in
# firmware/.cargo/config.toml picks the right tool per target: elf2uf2-rs
# for the RP2040, picotool for the RP2350.
flash-rp2040:
	$(call need_tool,elf2uf2-rs,run: cargo install elf2uf2-rs)
	cd firmware && cargo run --release

flash-rp2350:
	$(call need_tool,picotool,install your distro package or build from https://github.com/raspberrypi/picotool)
	cd firmware && cargo run --release --target thumbv8m.main-none-eabihf \
		--no-default-features --features rp235x

# The device-under-test binaries, built four ways. Chip selection is a
# compile-time feature in rpi-hal, and execution state is fixed by the
# firmware at reset, so one binary never spans the matrix -- each cell needs
# its own build.
CASE_TARGETS = armv7a-none-eabi aarch64-unknown-none-softfloat
CASE_CHIPS = bcm2837 bcm2711

cases:
	@set -e; for t in $(CASE_TARGETS); do for c in $(CASE_CHIPS); do \
		echo "== $$t $$c"; \
		cd cases && cargo build --release --target $$t --features $$c && cd ..; \
	done; done

# One cell of the matrix, for iterating without rebuilding the other three:
#   make case TARGET=armv7a-none-eabi CHIP=bcm2837
TARGET ?= armv7a-none-eabi
CHIP ?= bcm2837
case:
	cd cases && cargo build --release --target $(TARGET) --features $(CHIP)

# Raw images for the loader. `mem-write` copies file bytes straight to
# memory, so it needs a flat binary rather than the ELF cargo leaves behind
# -- an ELF would land its headers at the load address and execute garbage.
#
# The load address follows the execution state, matching what the firmware
# would have used: 0x8000 for AArch32, 0x80000 for AArch64.
CASE_BINS = hil_smoke hil_core hil_console hil_shadow hil_marker
IMAGE_DIR = cases/target/images

images: case
	@mkdir -p $(IMAGE_DIR)
	@set -e; for b in $(CASE_BINS); do \
		out=$(IMAGE_DIR)/$$b-$(TARGET)-$(CHIP).img; \
		cd cases && cargo objcopy --release --target $(TARGET) \
			--features $(CHIP) --bin $$b -- -O binary ../$$out && cd ..; \
		echo "built $$out"; \
	done

# The host suite. Cases needing hardware that isn't attached skip with a
# reason naming what was missing, so this is worth running with no bench at
# all -- the protocol codec is covered either way.
test:
	cd host && uv run --with pyusb --with pyserial --with pytest \
		python -m pytest -ra

# The fixture's own self-test, with its console TX jumpered to its RX (GP0 to
# GP1 on a Pico) and nothing else attached. Run this first whenever the
# loader cannot reach a board: it proves or clears the bridge firmware
# without the board or the wiring in the picture.
test-loopback:
	cd host && uv run --with pyusb --with pyserial --with pytest \
		python -m pytest -ra --console-loopback -k loopback

# The reset path, with a human power-cycling the board on request. Stands in
# for the HAT's load switch: -s so the prompt is visible, since pytest
# captures stdout by default. Never run this unattended -- it waits on a
# person.
test-manual:
	cd host && uv run --with pyusb --with pyserial --with pytest \
		python -m pytest -ra -s --manual-reset tests/test_recovery.py

# The full loop against a real board: build the image, load it through the
# fixture, run it, parse the verdict. Needs a wired-up Pi running rpi-loader,
# already in the execution state TARGET names.
test-board: images
	cd host && uv run --with pyusb --with pyserial --with pytest \
		python -m pytest -ra -s -m board --case-target $(TARGET) --case-chip $(CHIP)

# The same, but able to recover and to switch execution state, by asking a
# person to power-cycle the board. Needed whenever the board is not already
# in TARGET's state, or has been left halted by a previous case -- both of
# which require a reset, and there is no load switch until the HAT exists.
# Waits on a human, so never run it unattended.
test-board-manual: images
	cd host && uv run --with pyusb --with pyserial --with pytest \
		python -m pytest -ra -s -m board --manual-reset \
		--case-target $(TARGET) --case-chip $(CHIP)

# Shorthands for the two execution states, since getting TARGET wrong is not
# an error you see -- a mismatched image transfers, jumps, and decodes as
# garbage, leaving the board silent. `board_arch` catches it by reading
# config.txt first, but only if TARGET says what you meant.
test-board64:
	$(MAKE) test-board-manual TARGET=aarch64-unknown-none-softfloat CHIP=$(CHIP)

test-board32:
	$(MAKE) test-board-manual TARGET=armv7a-none-eabi CHIP=$(CHIP)

# The console handoff on its own. Split out from `test-board` because it is
# the one case that leaves the bench unusable when it goes wrong -- a board
# that hangs inside the blind window never restores its console -- so it is
# worth being able to run and re-run in isolation while bringing it up.
test-handoff: images
	cd host && uv run --with pyusb --with pyserial --with pytest \
		python -m pytest -ra -s --manual-reset \
		--case-target $(TARGET) --case-chip $(CHIP) \
		tests/test_console_handoff.py

# Everything that can be checked without hardware, for a pre-commit sweep.
test-all: firmware-rp2040 cases lint test

# Clippy over both Rust trees. The cases only need one cell of the matrix
# linted -- the lints are not target-specific, and doing all four would
# quadruple the time for no extra findings.
lint:
	cd firmware && cargo clippy --release -- -D warnings
	cd cases && cargo clippy --release --target $(TARGET) --features $(CHIP) -- -D warnings

# The host tree, which is Python and so gets none of the above. Run through
# `uvx` rather than as project dependencies: they are developer tools, and
# putting them in `pyproject.toml` would make a contributor with one board and
# a serial cable install a linter to run a test.
#
# The rule selection and target version are pinned in `host/pyproject.toml`,
# not left to whichever version uvx fetches -- otherwise a check that runs on
# every commit starts failing on untouched code the day the tool updates.
lint-py:
	cd host && uvx ruff check .
	cd host && uvx ty check .

fmt:
	cd firmware && cargo fmt
	cd cases && cargo fmt
	cd host && uvx ruff format .

fmt-check:
	cd firmware && cargo fmt --check
	cd cases && cargo fmt --check
	cd host && uvx ruff format --check .

# What to run before committing anything under hil-test. Called by rpi-hal's
# own `pre-commit`, so it never has to be remembered separately.
#
# Formats rather than checking formatting, matching rpi-hal's `pre-commit`:
# reformatting is not a decision anyone needs to be asked about, and a target
# that only complains means running it twice. Everything after it fails loudly
# instead, because those are real findings.
#
# Deliberately excludes the RP2350 firmware, which does not currently link,
# and anything needing hardware -- `test` skips the whole bench when none is
# attached, so it stays useful on a laptop.
pre-commit: fmt lint lint-py cases firmware-rp2040 test

clean:
	cd firmware && cargo clean
	cd cases && cargo clean
	rm -rf host/.venv host/.pytest_cache
	find host -name __pycache__ -type d -exec rm -rf {} +
