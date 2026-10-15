# Each crate is its own cargo project with its own toolchain pin and
# `.cargo/config.toml`, both of which are read from the working directory.
# So every target here runs the crate's own Makefile from inside its
# directory (`$(MAKE) -C`), never cargo from the repository root.

COMPONENTS := hal

.PHONY: pre-commit $(COMPONENTS)

pre-commit: $(COMPONENTS)

$(COMPONENTS):
	$(MAKE) -C $@ pre-commit
