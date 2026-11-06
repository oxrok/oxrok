# Changelog

All notable changes to oxrok are listed here. The format follows Keep a
Changelog and the project uses semantic versioning. Until 1.0 a minor
release may change the library API.

## [Unreleased]

## [0.1.0-alpha.2] - 2026-11-06

Still experimental, and still a pre-release: 25 of libsigrok's 87 drivers are
not ported, and most of what is here has never met its device.

**What has been checked how.** 63 driver modules, 55 of them new here. Four
have run on real hardware (`intronix-logicport`, `fx2lafw`, `rigol-ds`,
`raspberrypi-pico`), though `fx2lafw`'s upload and open order changed since
and were checked on the emulator only. Three have met a microcontroller
standing in for the device (`dcttech-usbrelay`, `microchip-pickit2`, the
`serial-dmm` family). **All the drivers listed below are emulator-verified and
have never run on hardware.** That means the same output and transactions as
libsigrok against a device model written from the C driver and public sources,
on the paths the cases reach, apart from the differences kept on purpose and
listed per device. It shows parity with libsigrok, not correct behaviour on a real
device: timing, real analog front ends and quirks the C relies on without
saying are not tested. The README has the table, with cases and C coverage per
driver.

### Upgrading

- A capture that fails exits 1 and says `capture failed: WHY` last on stderr,
  on every driver and port. sigrok-cli and earlier oxrok exit 0, so a script
  that relied on that sees a new failure (`docs/CLI.md`, "Exit status").
- Library: `Packet::Failed` is a new variant and `Packet` is not
  `#[non_exhaustive]`, so an exhaustive `match` stops compiling. A driver
  finishes an acquisition with `PacketSink::end`, and `Session::run` returns
  `Error::CaptureFailed` after the last packet. The rest of the library API
  moved too; none of it is stable before 1.0, and a dependency on
  `"0.1.0-alpha.1"` accepts this version on `cargo update`, so pin with `=`.
- Values oxrok refuses where sigrok-cli takes them: `0x10` and `3.3V` for a
  number, which `strtof` reads and oxrok does not, and a boolean in `-c` or
  `-d` that is not clearly on or off (`docs/CLI.md`).
- `-c` checks all its options before sending any, and sends `enabled=on` last.
  A range error is still found at send time, so a set point ahead of it has
  gone out.
- `conn=bt/...` is refused with a message: oxrok has no Bluetooth path, and
  RFCOMM goes through `/dev/rfcommN`.
- The UT32x thermometers are their own driver module, `uni_t_ut32x`, with the
  same id.
- `oxrok::pydecode` has a module per host, `cpython` and `rustpython`;
  `PyItem` stays the opaque item type of the CPython host, `RpItem` is the
  RustPython one.

### Drivers added

<!-- One line per driver id or family, in alphabetical order, as ports merge. -->

- `agilent-dmm`: Agilent and Keysight U12xx meters
- `appa-55ii`: APPA 55II thermometer
- `arachnid-labs-re-load-pro`: Re:load Pro load
- `asix-sigma`: ASIX SIGMA and SIGMA2
- `atorch`: Atorch meters and loads
- `atten-pps3203`: Atten PPS3203T-3S supply
- `bkprecision-1856d`: BK Precision 1856D frequency counter
- `cem-dt-885x`: CEM DT-885x sound level meters
- `chronovu-la`: ChronoVu LA8 and LA16
- `conrad-digi-35-cpu`: Conrad DIGI 35 CPU supply
- `devantech-eth008`: Devantech relay boards
- `dreamsourcelab-dslogic`: DSLogic family
- `fluke-dmm`: Fluke 8x, 18x, 199B and 28x meters
- `ftdi-la`: FTDI chips as logic analyzers
- `greatfet`: GreatFET One
- `gwinstek-gpd`: GW Instek GPD-2303S and 3303S supplies
- `gwinstek-psp`: GW Instek PSP supplies
- `hantek-4032l`: Hantek 4032L
- `hantek-6xxx`: Hantek 6022BE family
- `hantek-dso`: Hantek DSO-2090 family
- `icstation-usbrelay`: ICStation ICSE relay boards
- `ikalogic-scanalogic2`: IKALOGIC Scanalogic-2
- `ikalogic-scanaplus`: IKALOGIC ScanaPLUS
- `itech-it8500`: ITECH IT8500 and BK 8500 loads
- `juntek-jds6600`: Juntek JDS6600 signal generator
- `kecheng-kc-330b`: Kecheng KC-330B sound level meter
- `kern-ew-6200-2nm`: KERN EW and EG-N scales
- `kingst-la2016`: Kingst LA2016 family
- `korad-kaxxxxp`: Korad KA series supplies
- `lascar-el-usb`: Lascar EasyLog EL-USB loggers
- `lecroy-logicstudio`: LeCroy LogicStudio 16
- `link-mso19`: Link Instruments MSO-19
- `manson-hcs-3xxx`: Manson HCS-3xxx supplies
- `mastech-ms6514`: Mastech MS6514 thermometer
- `maynuo-m97`: Maynuo M97 and M98 loads
- `motech-lps-301`: Motech LPS-301 supply
- `norma-dmm`, `siemens-b102x`: Norma DM9x0 and Siemens B102x meters
- `ols`: Open Bench Logic Sniffer
- `p-ols`: Pipistrello OLS
- `pce-322a`: PCE-322A sound level meter
- `rdtech-dps`, `rdtech-rd`: RDTech DPS, DPH and RD supplies
- `rdtech-tc`: RDTech TC66C
- `rdtech-um`: RDTech UMxx meters
- `rohde-schwarz-sme-0x`: Rohde & Schwarz SME generators
- `saleae-logic16`: Saleae Logic16
- `saleae-logic-pro`: Saleae Logic Pro 16
- `deree-de5000`, `mastech-ms5308`, `peaktech-2170`, `uni-t-ut612`,
  `voltcraft-4080` and others (module `serial_lcr`): ES51919 and VC4080 LCR meters
- `sysclk-lwla`: Sysclk LWLA1034 and LWLA1016
- `sysclk-sla5032`: Sysclk SLA5032
- `testo`: Testo 435
- `uni-t-dmm`: the UT-D04 cable's raw USB route, 27 meters
- `uni-t-ut181a`: UNI-T UT181A meter
- `uni-t-ut32x`: UNI-T UT32x thermometers, on a serial line and on the CH9325
  cable, as a module of their own
- `zeroplus-logic-cube`: ZEROPLUS Logic Cube, 13 boards
- `zketech-ebd-usb`: ZKETECH EBD-USB load

### Known issues

- The emulator job in CI can lose a session: at this release `cem-dt-885x`'s
  model lost its emulator after an interrupt's close, and that model's later
  cases failed with it. It looks like the unlink race tracked in
  `docs/ROADMAP.md`, though its kernel line was not seen. It is in the test
  tooling, which is not in this crate. A few unit tests still fail under long
  stalls of the host (`docs/ROADMAP.md`), and `csv_matches_sigrok_cli` fails
  now and then.
- A demo capture with a trigger on an analog channel hangs.
- `-O csv` holds rows in memory when it falls behind, with all 13 of `demo`'s
  channels at 200 kHz, and makes up 0 for a channel absent from a packet.
- BLE units have no path (`conn=bt/` above).
- The libusb backend has only occasional emulator runs; the regular ones use
  nusb.
- `docs/CLI.md` lists the other differences from sigrok-cli, and
  `docs/ROADMAP.md` the open items.

### Added

- `-O null`, an output that discards.
- `conn=tcp-raw/HOST/PORT` for any serial driver.
- `oxrok::modbus` (Modbus RTU) and `oxrok::lcr`, the LCR meter parsers.
- The `emu` crate (`oxrok-emu`), not published: a USB/IP device server with a
  `DeviceModel` trait, attach and detach through Linux's `vhci-hcd`
  without the usbip tools, a transaction log in the fixture dialect
  and pedantic checks on what the host sends. The dcttech relay board
  is the first model; `tools/diffcli.sh --emu` runs sigrok-cli and
  oxrok against it and compares output and transactions
  (`docs/USB.md`). Its core is now the `vusb` crate, and
  `tools/diffcli.sh` needs `vusb-ctl`
  (`cargo install vusb-cli --version 0.2.1 --locked`).
- The `rustpython` feature: libsigrokdecode's Python decoders on a
  RustPython 0.6.0 interpreter built into the binary, with its
  standard library frozen in, so no Python installation is needed.
  Same output as the CPython host over 271 sigrok-test runs. With both
  features `OXROK_PYTHON=rustpython` selects it.
- `make dist` and the `Release binaries` workflow (run by hand, and on
  each published release, which gets the archives attached): static
  Linux binaries (musl, x86_64 and aarch64), macOS (x86_64, arm64) and
  Windows (x86_64), built with `rustpython`, with the libsigrokdecode
  0.5.3 decoders in `decoders/` next to the binary, and a vendored source
  tarball that builds offline; each has its third-party notices
  (`docs/LICENSING.md`).
- Python decoders are also looked for in `decoders` next to the
  executable.
- Installation notes for the `python` feature in the README and the
  crate documentation.
- `tests/libsigrok_parity.rs`: the value tables of libsigrok's own unit
  tests (sample rate, period and voltage strings, SI prefixes, unit
  strings, the binary input, module listings, the demo sample rate,
  triggers, version) run against oxrok.

### Changed

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
