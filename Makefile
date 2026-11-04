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
#   make clean-dist            remove dist/

CARGO    ?= cargo
FEATURES ?= rustpython
HOST     := $(shell rustc -vV | sed -n 's/^host: //p')
VERSION  := $(shell sed -n 's/^version = "\(.*\)"/\1/p' Cargo.toml | head -n 1)
DIST     := dist

# The decoders oxrok is compared with: libsigrokdecode 0.5.3.
SRD_VERSION := 0.5.3
SRD_URL     := https://github.com/sigrokproject/libsigrokdecode/archive/refs/tags/libsigrokdecode-$(SRD_VERSION).tar.gz
SRD_DIR     := build/libsigrokdecode-libsigrokdecode-$(SRD_VERSION)

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

.PHONY: build decoders dist clean-dist

build:
	rustup target add $(TARGET)
	$(CARGO) build --release --locked --target $(TARGET) $(if $(FEATURES),--features $(FEATURES))

# Outside target/, which CI caches and prunes.
$(SRD_DIR)/COPYING:
	rm -rf $(SRD_DIR)
	mkdir -p build
	curl -fsSL $(SRD_URL) | tar -xz -C build
	test -f $@

decoders: $(SRD_DIR)/COPYING

dist: build $(if $(FEATURES),decoders)
	rm -rf $(STAGE)
	mkdir -p $(STAGE)
	cp $(BIN) README.md LICENSE NOTICE CHANGELOG.md $(STAGE)/
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

clean-dist:
	rm -rf $(DIST) build
