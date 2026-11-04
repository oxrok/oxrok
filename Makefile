# Release archives: one self-contained oxrok binary per platform, with
# libsigrokdecode's Python decoders run by the built-in RustPython
# interpreter and shipped next to it in decoders/.
#
#   Linux    musl, statically linked (needs musl-gcc: apt install musl-tools)
#   Windows  MSVC with the C runtime linked in
#   macOS    links only system libraries and frameworks
#
#   make dist                  build and package for the host
#   make dist TARGET=<triple>  for another target
#   make dist FEATURES=        without the Python decoders
#   make notices               only the third-party notices, for the target
#   make notices-all           the same, for every target the release builds
#   make dist-src              the vendored source tarball of HEAD
#   make dist-src-check        build that tarball with no registry or network
#   make clean-dist            remove dist/
#
# The notices need cargo-about, the version the release workflow pins:
#   cargo install --locked --features cli --version 0.9.3 cargo-about
# dist-src needs git, GNU tar and xz. The tarball carries the decoders, the
# vendored crates and the notices, so `make dist` works in it with no
# network and no cargo-about.

CARGO    ?= cargo
# Extra arguments for cargo build; --offline when building a source tarball.
CARGOFLAGS ?=
FEATURES ?= rustpython
HOST     := $(shell rustc -vV | sed -n 's/^host: //p')
VERSION  := $(shell sed -n 's/^version = "\(.*\)"/\1/p' Cargo.toml | head -n 1)
DIST     := dist
PYTHON   ?= $(shell python3 -c '' 2>/dev/null && echo python3 || echo python)

# The decoders oxrok is compared with: libsigrokdecode 0.5.3.
SRD_VERSION := 0.5.3
SRD_URL     := https://github.com/sigrokproject/libsigrokdecode/archive/refs/tags/libsigrokdecode-$(SRD_VERSION).tar.gz
# A source tarball carries them; a checkout downloads them.
SRD_BUNDLED := third_party/libsigrokdecode-$(SRD_VERSION)
SRD_DIR     := $(if $(wildcard $(SRD_BUNDLED)/COPYING),$(SRD_BUNDLED),build/libsigrokdecode-libsigrokdecode-$(SRD_VERSION))

# Linux builds use musl so the binary is static.
ifneq (,$(findstring linux-gnu,$(HOST)))
TARGET   ?= $(subst linux-gnu,linux-musl,$(HOST))
else
TARGET   ?= $(HOST)
endif

ifneq (,$(findstring windows,$(TARGET)))
EXE      := .exe
ARCHIVE  := zip
else
EXE      :=
ARCHIVE  := tar.gz
endif

ifneq (,$(or $(findstring -msvc,$(TARGET)),$(findstring -musl,$(TARGET))))
export RUSTFLAGS += -C target-feature=+crt-static
endif

# The C parts of RustPython's dependencies need a musl compiler, and
# musl-gcc links too, so pymath's -lm finds musl's libm instead of the
# host's glibc one.
ifneq (,$(findstring -musl,$(TARGET)))
TARGET_VAR := $(shell echo $(TARGET) | tr '[:lower:]-' '[:upper:]_')
export CC_$(subst -,_,$(TARGET)) ?= musl-gcc
export CARGO_TARGET_$(TARGET_VAR)_LINKER ?= musl-gcc
endif

export MACOSX_DEPLOYMENT_TARGET ?= 11.0

NAME     := oxrok-$(VERSION)-$(TARGET)
STAGE    := $(DIST)/$(NAME)
BIN      := target/$(TARGET)/release/oxrok$(EXE)
SHA256   := $(shell command -v sha256sum >/dev/null 2>&1 && echo sha256sum || echo "shasum -a 256")

# THIRD-PARTY-NOTICES.txt, generated rather than committed. In a vendored
# source tarball the one at its top is already there and covers every
# release target, so building from it needs no cargo-about.
NOTICES_NAME := THIRD-PARTY-NOTICES.txt
NOTICES      := build/THIRD-PARTY-NOTICES-$(TARGET).txt

# The matrix in .github/workflows/release.yml.
RELEASE_TARGETS := x86_64-unknown-linux-musl aarch64-unknown-linux-musl \
                   x86_64-apple-darwin aarch64-apple-darwin x86_64-pc-windows-msvc

# The source offer: HEAD's tree, what `git archive` leaves out of what the
# binary archives ship, and the crates of every platform.
SRC_NAME  := oxrok-$(VERSION)-vendored
SRC_STAGE := $(DIST)/$(SRC_NAME)
SRC_TAR   := $(DIST)/$(SRC_NAME).tar.xz
# A clean place to build it, and what to run the build under to take the
# network away (the release workflow's `unshare`).
SRC_CHECK ?= $(DIST)/src-check
SRC_CHECK_WRAP ?=

.PHONY: build decoders notices notices-all dist dist-src dist-src-check clean-dist

build:
	rustup target add $(TARGET)
	$(CARGO) build --release --locked $(CARGOFLAGS) --target $(TARGET) $(if $(FEATURES),--features $(FEATURES))

# Outside target/, which CI caches and prunes.
$(SRD_DIR)/COPYING:
	rm -rf $(SRD_DIR)
	mkdir -p build
	curl -fsSL $(SRD_URL) | tar -xz -C build
	test -f $@

decoders: $(SRD_DIR)/COPYING

notices:
	mkdir -p build
ifeq (,$(wildcard $(NOTICES_NAME)))
	$(PYTHON) tools/third-party/notices.py --target $(TARGET) --features "$(FEATURES)" --output $(NOTICES)
else
	cp $(NOTICES_NAME) $(NOTICES)
endif

notices-all:
	mkdir -p build
	$(PYTHON) tools/third-party/notices.py $(foreach t,$(RELEASE_TARGETS),--target $(t)) --features "$(FEATURES)" --output build/$(NOTICES_NAME)

# The notices first: when they fail, there is no point in the build.
dist: notices build $(if $(FEATURES),decoders)
	rm -rf $(STAGE)
	mkdir -p $(STAGE)
	cp $(BIN) README.md LICENSE NOTICE CHANGELOG.md $(STAGE)/
	cp $(NOTICES) $(STAGE)/$(NOTICES_NAME)
ifneq (,$(FEATURES))
	cp -R $(SRD_DIR)/decoders $(STAGE)/decoders
	cp $(SRD_DIR)/COPYING $(STAGE)/decoders/COPYING
endif
ifeq (zip,$(ARCHIVE))
	cd $(DIST) && 7z a -tzip $(NAME).zip $(NAME) >/dev/null
else
	cd $(DIST) && tar -czf $(NAME).tar.gz $(NAME)
endif
	cd $(DIST) && $(SHA256) $(NAME).$(ARCHIVE) > $(NAME).$(ARCHIVE).sha256
	rm -rf $(STAGE)
	@echo $(DIST)/$(NAME).$(ARCHIVE)

dist-src: notices-all decoders
	@test -z "$$(git status --porcelain)" || { echo "dist-src: the tree has changes; the tarball is HEAD, commit first" >&2; exit 1; }
	rm -rf $(SRC_STAGE)
	mkdir -p $(SRC_STAGE)/.cargo $(SRC_STAGE)/$(SRD_BUNDLED)
	git archive HEAD | tar -x -C $(SRC_STAGE)
	git rev-parse HEAD > $(SRC_STAGE)/GIT-COMMIT
	$(CARGO) vendor --locked --versioned-dirs --manifest-path $(SRC_STAGE)/Cargo.toml $(SRC_STAGE)/vendor > /dev/null
	printf '[source.crates-io]\nreplace-with = "vendored-sources"\n\n[source.vendored-sources]\ndirectory = "vendor"\n' > $(SRC_STAGE)/.cargo/config.toml
	cp build/$(NOTICES_NAME) $(SRC_STAGE)/$(NOTICES_NAME)
	cp -R $(SRD_DIR)/decoders $(SRC_STAGE)/$(SRD_BUNDLED)/decoders
	cp $(SRD_DIR)/COPYING $(SRC_STAGE)/$(SRD_BUNDLED)/COPYING
	XZ_OPT=-T0 tar --sort=name --owner=0 --group=0 --numeric-owner --mtime=@$$(git log -1 --format=%ct) -cJf $(SRC_TAR) -C $(DIST) $(SRC_NAME)
	rm -rf $(SRC_STAGE)
	cd $(DIST) && $(SHA256) $(SRC_NAME).tar.xz > $(SRC_NAME).tar.xz.sha256
	@echo $(SRC_TAR)

# `make dist` in the extracted tarball, which is `cargo build --offline
# --locked` with the release features and an empty CARGO_HOME, so nothing
# can come from the registry cache either.
dist-src-check:
	rm -rf $(SRC_CHECK)
	mkdir -p $(SRC_CHECK)/cargo-home
	tar -xJf $(SRC_TAR) -C $(SRC_CHECK)
	cd $(SRC_CHECK)/$(SRC_NAME) && $(SRC_CHECK_WRAP) env CARGO_HOME=$(abspath $(SRC_CHECK))/cargo-home CARGO_NET_OFFLINE=true \
		$(MAKE) dist DIST=dist TARGET=$(TARGET) FEATURES=$(FEATURES) CARGO=$(CARGO) CARGOFLAGS=--offline
	$(SRC_CHECK)/$(SRC_NAME)/target/$(TARGET)/release/oxrok$(EXE) --version
	test -f $(SRC_CHECK)/$(SRC_NAME)/dist/$(NAME).$(ARCHIVE)
	rm -rf $(SRC_CHECK)

clean-dist:
	rm -rf $(DIST) build
