# Changelog

All notable changes to oxrok are listed here. The format follows Keep a
Changelog and the project uses semantic versioning. Until 1.0 a minor
release may change the library API.

## [Unreleased]

### Changed

- `emu/` is on the `vusb` crate: the contract, the bus, the shells, USB/IP
  and the control socket are vusb's, and `emu/` keeps the models, the `.tx`
  log as an observer, `hidcable` and the checks on a firmware upload. The
  server takes commands through `vusb-ctl` on `--control PATH` rather than
  on standard input, and `tools/diffcli.sh` needs `vusb-ctl`
  (`cargo install vusb-cli --version 0.2.1 --locked`). The goldens have not
  changed.

### Added

- The `emu` crate (`oxrok-emu`): a USB/IP device server with a
  `DeviceModel` trait, attach and detach through Linux's `vhci-hcd`
  without the usbip tools, a transaction log in the fixture dialect
  and pedantic checks on what the host sends. The dcttech relay board
  is the first model; `tools/diffcli.sh --emu` runs sigrok-cli and
  oxrok against it and compares output and transactions
  (`docs/USB.md`).
- The `rustpython` feature: libsigrokdecode's Python decoders on a
  RustPython 0.6.0 interpreter built into the binary, with its
  standard library frozen in, so no Python installation is needed.
  Same output as the CPython host over 271 sigrok-test runs. With both
  features `OXROK_PYTHON=rustpython` selects it.
- `make dist` and the `Release binaries` workflow (run by hand, and on
  each published release, which gets the archives attached): static
  Linux binaries (musl, x86_64 and aarch64), macOS (x86_64, arm64) and
  Windows (x86_64), built with `rustpython`, with the libsigrokdecode
  0.5.3 decoders in `decoders/` next to the binary.
- Python decoders are also looked for in `decoders` next to the
  executable.
- Installation notes for the `python` feature in the README and the
  crate documentation.
- `tests/libsigrok_parity.rs`: the value tables of libsigrok's own unit
  tests (sample rate, period and voltage strings, SI prefixes, unit
  strings, the binary input, module listings, the demo sample rate,
  triggers, version) run against oxrok.

### Changed

- `oxrok::pydecode` has a module per host, `cpython` and `rustpython`;
  `PyItem` stays the opaque item type of the CPython host, `RpItem` is
  the RustPython one.
- Decoder threads have an 8 MiB stack.

### Fixed

- HID on Linux: the product string is the USB device's own, as hidapi
  reports it, not the kernel's joined "manufacturer product" name, so
  `dcttech-usbrelay` finds its board on Linux; `conn` is the hidraw
  node there, as under libsigrok, and `conn=/dev/hidrawN` is accepted.
- Periods take `ps` and `fs` like libsigrok, and a unit without digits
  (`-c timebase=ms`) is rejected instead of read as zero.
- A decoder channel mapped to a name that matches no channel no longer
  cancels the default mapping: as in sigrok-cli, the name is skipped
  with a warning and, when no name matched, every channel keeps its
  default.
- The native `spi` decoder takes word sizes above 64 bits; a 152 bit
  word came out as two 64 bit words with the rest lost. Integers wider
  than 64 bits pass between decoders and to Python intact instead of
  becoming floats.

## [0.1.0-alpha.1] - 2026-10-23

First publication, to claim the name on crates.io. Experimental; see
the README for what has been verified on hardware.

### Changed

- The LogicPort register map moved from `docs/LOGICPORT.md` into the
  driver's `protocol.rs`; the `hid_list`, `usb_interfaces` and
  `ftdi_probe` examples are gone, `OXROK_USB_RECORD` covers their use.
- One crate: the 21 workspace crates are modules of `oxrok` (`core`,
  the transports, `drivers::*`, `formats`, `decode`); the binary sits
  behind the default `cli` feature. The bench tool stays a separate
  unpublished crate in `hil/`.
- License: the whole workspace is GPL-3.0-or-later, as the sigrok code
  it ports; the MIT OR Apache-2.0 dual license is gone.
- The bits and hex outputs drop a trigger marker that falls in the last,
  partial line, as sigrok's do; serial devices and `rigol-ds` show no
  `conn` in `--show`, rational lists print one value per line and
  signed integer keys print their name alone, as sigrok-cli does.
- The reference for byte compatibility is now sigrok-cli 0.8.0-git with
  libsigrok 0.6.0-git: the CSV time column starts at 0 and the VCD body
  has a blank line after the definitions and two spaces after each
  timestamp. `-I` and `-O --show` print options as 0.8 does.

### Added

- `serial`: serial port transport with the sigrok `serialcomm`
  syntax and record and replay fixtures (`OXROK_SERIAL_RECORD`).
- `dmm`: multimeter packet parsers (FS9721, FS9922, DTM0660, Metex
  14 byte, VC-96, ASYC-II, M2110, UT71x, ES519xx, VC-870, RadioShack
  22-812, BM25x, BM52x, BM82x, BM85x, BM86x, QM1578, EEVblog 121GW,
  MS2115B, MS8250D, Meterman 38XR) and the `serial-dmm` driver with
  all 69 meter ids of libsigrok, one driver per meter as in sigrok.
- `scpi`: SCPI over USBTMC (`usb`), raw and Rigol TCP and
  serial ports, with the IEEE 488.2 helpers and a text fixture of
  commands and responses (`OXROK_SCPI_RECORD`). `usb` reports
  interface classes and endpoints.
- the `raspberrypi-pico` driver: the `raspberrypi-pico` driver for the
  sigrok-pico firmware, verified against sigrok-cli on a Pico.
- The 27 `uni-t-dmm` meter ids on the UT-D04 cable (over `hid/ch9325`),
  the UT372 parser, and the `uni-t-ut32x` thermometer driver.
- the `dcttech-usbrelay` driver and the `microchip-pickit2` driver,
  both over the HID transport; the PICkit2 thereby works on macOS too.
- the `rigol-ds` driver: the `rigol-ds` scope driver with all 61
  Rigol and Agilent models of libsigrok, verified on a DS1102E; frames as analog packets
  between `FrameBegin` and `FrameEnd`, which the analog output prints
  as `FRAME-BEGIN` and `FRAME-END` like sigrok-cli.
- `hid`: USB HID transport over `async-hid`, and serial over HID
  cables in `serial` (Brymen BU-86X, WCH CH9325, SiLabs CP2110,
  Victor), named `hid[/CHIP][/sn=SERIAL]` as in sigrok. Meters that
  come with such a cable scan without `conn=`. Verified against
  sigrok-cli with a Pico emulating the three cables.
- Trigger extras in the shared model and the `-t` grammar: occurrence
  counts (`D0=r*5`, consecutive `D0=r*5c`), durations (`D0=1>100`,
  `D0=1>50us`), value ranges on a channel group (`D0-D2=3..5`,
  `D0-D2=!7`) and two stage or/and combines (`D0=r|D1=r`, `D0=r&D1=r`).
  Devices advertise what they support with `trigger_features`; the
  software trigger takes counts and durations.
- LogicPort: all of the above in hardware, plus the options
  `clock_qualifier`, `sample_point` (FPGA images 0 to 5), `invert`,
  `frequency_channel` and the read only `frequency` counter. Bench
  sections for each; 60 checks.
- Workspace with `core`, `formats`, the `demo` driver and
  the `oxrok` command line tool; CI for Linux, macOS and Windows.
- Configuration model: the sigrok keys with their ids, names and value
  types, plus driver defined keys.
- Data feed packets, the `Driver` and `Device` traits with a per device
  acquisition thread, and a `Session` that fans packets out.
- Demo driver with the logic patterns `oxrok`, `random`, `incremental`,
  `walking-one`, `walking-zero`, `all-low`, `all-high`, `graycode` and the
  analog patterns `square`, `sine`, `triangle`, `sawtooth`, `random`,
  with amplitude, offset, averaging and frame limits.
- Outputs `bits`, `hex`, `ascii` and `analog`, byte compatible with
  sigrok-cli after the header line.
- Command line: `--scan`, `--show`, `-d`, `-c`, `-C`, `-g`, `-t` with
  several stages, `--samples`, `--time`, `--frames`, `--continuous`,
  `-O`, `-o`, `--get`, `--set`, `-L`, `-V`, `-l`.
- Session files (`.sr`): the `srzip` output writes them in one pass, and
  `-i file.sr` reads them, including analog channels. Files round trip
  with sigrok-cli in both directions.
- Outputs `vcd`, `csv` and `binary`, byte compatible with sigrok-cli.
- Inputs `vcd` (scalars, vectors, reals, scopes, `downsample`, `skip`,
  `compress`, `numchannels`, `samplerate_overwrite`) and `binary`;
  `-i FILE [-I module:options]` with format detection, `-C` on files,
  and `-i FILE --show` for a file's properties. `-I module --show` and
  `-O module --show` describe a module.
- `--strict`: warnings fail the run. File inputs stop at the first
  chunk that warned.
- `usb`: the `Transport` trait with a nusb backend (default) and
  a libusb backend (`libusb` feature, `OXROK_USB_BACKEND` to choose at
  run time), device enumeration with `bus.address` matching, and a
  text fixture format with `Recorder` and `Replay` transports.
- `ftdi`: FT232/FT245 vendor requests, baud rate encoding per
  AN_120, bit modes, pin and EEPROM reads, status byte stripping.
  Verified on a LogicPort with
  both backends on macOS; the recording is a replay test.
- the `logicport` driver: the Intronix LogicPort LA1034 with all its
  sample rates, state mode, compression, two trigger levels, frames and
  the threshold DAC. FPGA images load from the vendor's `.ccf` or from
  `.rbf` files found in the sigrok and oxrok firmware directories.
  Output matches the libsigrok driver byte for byte; two recorded
  sessions replay in CI.
- `OXROK_USB_RECORD=FILE` records any driver session; fixtures store
  large payloads as digests and tolerate repeated polls.
- `oxrok-hil`: the hardware bench, driven by the ESP32 signal generator.
- `docs/DRIVER_PORTING.md` and `docs/DEVICES.md`.
- `Transport::bulk_in_stream`: queued bulk reads on both backends,
  recorded and replayed like single reads.
- `ezusb`: FX2 firmware upload (raw or Intel HEX) and the wait
  for renumeration. `core::firmware` holds the shared lookup of
  firmware files.
- `core::SoftTrigger` and `PreTriggerBuffer`: host side triggers
  for streaming devices.
- Every Rust source names its sigrok counterpart and that file's
  copyright holders in a header comment; see `docs/LICENSING.md`.
- `docs/DRIVER_PORTING.md` grew with the fx2lafw and parity lessons.
- `tools/diffcli.sh`: runs the documented flows through sigrok-cli and
  oxrok and diffs them, with `--hw` for the LogicPort and fx2lafw cases.
- The demo device honours triggers through the software trigger.
- `fx2lafw`: `probe_names` scan option and `limit_frames`.
- `decode`: the protocol decoder runtime (decoder API with
  libsigrokdecode's `wait` semantics, instance threads, output
  routing) and `decoders` with native `uart`, `i2c` and `spi`
  decoders. The CLI
  gains `-P`, `-A`, `-B`, `-M`, `--protocol-decoder-samplenum` and
  `--protocol-decoder-ann-class`, `-P dec --show` and the decoder
  section of `-L`. Their output is identical to sigrok-cli's in every
  mode on synthetic captures.
- fx2lafw on macOS: the open after a firmware upload no longer fails
  with "interface not found"; the USB backend reconfigures and reopens
  the device when its interface is missing.
- Conformance fixtures from the bench: UART, I2C and SPI captures taken
  with the LogicPort from the ESP32, decoded identically to sigrok-cli.
- Stacked decoders (`-P i2c,eeprom24xx`), synchronous as in
  libsigrokdecode, and `pydecode` behind the `python` feature of
  the CLI: libsigrokdecode's Python decoders run unchanged on the same
  runtime through PyO3. All 112 upstream decoders load; the checked
  ones print what sigrok-cli prints.
- the `fx2lafw` driver: Saleae Logic clones, USBee, Cypress FX2 and
  sigrok FX2 LA boards, 8 or 16 channels, with the firmware upload and
  the software trigger. Verified on a Saleae Logic clone: firmware
  upload, 60 s at 24 MS/s without drops, `--show` identical to
  sigrok-cli. A written session and a recorded one replay as tests.
