# Each crate is its own cargo project with its own toolchain pin and
# `.cargo/config.toml`, both of which are read from the working directory.
# So every target here runs the crate's own Makefile from inside its
# directory (`$(MAKE) -C`), never cargo from the repository root.

# `hil` is the hardware-in-the-loop bench, which is three toolchains of its
# own -- thumbv6m firmware, armv7a/aarch64 case binaries, and a Python host
# runner -- none of which share a cargo project with the crates. Its
# Python half is checked with `ruff` and `ty`, fetched on demand by `uvx`
# rather than installed as project dependencies: a contributor with one
# board and a serial cable should not have to install a linter to run a
# test.
COMPONENTS := hal embassy kickstart loader hil

.PHONY: pre-commit $(COMPONENTS)

pre-commit: $(COMPONENTS)

$(COMPONENTS):
	$(MAKE) -C $@ pre-commit
