# oxrok

An experimental sigrok compatible logic analyzer stack in Rust: a
library, drivers and a command line tool. No C libraries and no Python
installation needed, one `cargo build`: libsigrokdecode's Python
decoders run on a built-in RustPython interpreter (feature
`rustpython`) or on the system's CPython (feature `python`).

oxrok is not part of the sigrok project. It keeps sigrok's file formats,
command line flags and decoder ids so existing tools, scripts and habits
carry over.

## How it was written

oxrok is, so far, a translation of the sigrok C and Python code into
Rust done by a large language model (Claude), with a human directing
the work: choosing what to port, reviewing the structure and the
protocol handling, wiring up the bench hardware and running the
comparisons. Every driver and format is checked against sigrok-cli's
output for the same command, byte for byte after the version line,
either on real hardware or on a microcontroller that emulates the
device. Treat code that has only seen the emulator or a recording as
untested until the table below says otherwise, and read it with that
in mind.

## Status

Early. The crate builds on Linux, macOS and Windows with a sigrok-cli
style command line, the `bits`, `hex`, `ascii`, `analog`, `csv`, `vcd`,
`binary` and `srzip` outputs, `.sr`, VCD and binary inputs, USB, serial,
HID and SCPI transports with record and replay, eight driver families
and native `uart`, `i2c` and `spi` decoders that print what
libsigrokdecode prints. With the `rustpython` or `python` feature the
unchanged libsigrokdecode decoders run too. See `docs/ROADMAP.md`.

### Hardware validated so far

| driver | checked on | how |
|---|---|---|
| `intronix-logicport` | two LogicPort LA1034 units | 60 bench checks with an ESP32 signal generator, captures identical to sigrok-cli |
| `fx2lafw` | a Saleae Logic clone (`0925:3881`) | firmware upload, captures identical to sigrok-cli |
| `rigol-ds` | a Rigol DS1102E | `--show`, frames and settings identical to sigrok-cli; also on a Pico emulating the scope over USBTMC |
| `raspberrypi-pico` | a Pico running the sigrok-pico firmware | digital, analog and mixed captures identical to sigrok-cli |
| `serial-dmm` (UT61E, UT60A) | meter streams replayed by an ESP32 | readings identical to sigrok-cli; no real meter yet |
| HID cables (BU-86X, CH9325, Victor), UT-D04 meters, UT32x | a Pico emulating the cable, and the host emulator (`oxrok-emu`) | readings and USB transactions identical to sigrok-cli; CP2110 unverified |
| `dcttech-usbrelay` | the host emulator over USB/IP (`oxrok-emu`), and a Pico emulating the board | output and USB transactions identical to sigrok-cli; no real board yet |
| `microchip-pickit2` | the host emulator (`oxrok-emu`), and a Pico emulating the device | output and USB transactions identical to sigrok-cli; no real programmer yet |
| `uart`, `i2c`, `spi` decoders | captures from an ESP32 | annotations identical to libsigrokdecode |

The other 67 multimeters, the other Rigol and Agilent scope series and
the CP2110 cable are ports with replay tests only. `docs/DEVICES.md`
has the details per device and `hil/` the firmware used on the bench.

```sh
oxrok --scan
oxrok -d demo:logic_channels=4 -g Logic -c pattern=walking-one --samples 20 -O bits
oxrok -d demo --show
oxrok -d demo --samples 1k -o capture.sr      # a session file PulseView opens
oxrok -i capture.sr -O csv
oxrok -d intronix-logicport -c samplerate=100M -t D0=r --samples 2k -O vcd -o cap.vcd
oxrok -i cap.vcd -P uart:baudrate=115200:rx=D0 -A uart=rx-data
```

The output after the first header line matches sigrok-cli byte for byte
for the same command, which is how the test suite checks it.

## Installing

### Release binaries

Each GitHub release has an archive per platform: Linux x86_64 and
aarch64 (static, musl), macOS x86_64 and arm64, Windows x86_64. Each
holds one `oxrok` binary built with the `rustpython` feature and, in
`decoders/`, the decoders of libsigrokdecode 0.5.3, so nothing else
needs installing. Unpack it anywhere and run `oxrok` from there; keep
`decoders/` next to the binary.

The archives are built by `make dist` (see the `Makefile`) in the
`Release binaries` workflow, which also runs by hand from the Actions
tab.

### With cargo

The current release is a pre-release, so name its version:

```sh
cargo install oxrok --version 0.1.0-alpha.1 --locked
cargo install --path . --locked                    # from a checkout
```

`--locked` builds with the dependency versions the release was tested
with.

### With the Python decoders

Two features run libsigrokdecode's Python decoders unchanged, next to
the native `uart`, `i2c` and `spi`:

- `rustpython` builds the RustPython interpreter and its standard
  library into the binary. Nothing is needed at run time but the
  decoders. It needs Rust 1.96 or newer and `--locked`, which keeps a
  dependency pin RustPython 0.6.0 relies on. The output is the same
  as on CPython: 271 sigrok-test runs of the Python decoders compare
  equal. Those runs took 1.2 times as long as on CPython; long decodes
  take up to about three times as long.
- `python` uses the system's CPython through pyo3.

```sh
cargo install oxrok --version 0.1.0-alpha.1 --locked --features rustpython
```

With both features, CPython is used unless `OXROK_PYTHON=rustpython`.

The `python` feature needs two things:

- Python 3.9 or newer with its shared library, at build time and at
  run time, since the binary links against the interpreter found when
  it was built. Homebrew's and python.org's Pythons include the
  library; on Debian and Ubuntu install `python3-dev`, on Fedora
  `python3-devel`; on Windows use the python.org installer with Python
  added to `PATH`.
- The decoders: `brew install libsigrokdecode`, the libsigrokdecode
  package of your distribution, or the `decoders` directory of a
  libsigrokdecode checkout.

```sh
cargo install oxrok --version 0.1.0-alpha.1 --locked --features python
PYO3_PYTHON=/usr/bin/python3.12 cargo install oxrok --version 0.1.0-alpha.1 --locked --features python
```

The build uses `PYO3_PYTHON` when set, else the active virtualenv, else
`python` or `python3` on `PATH`. Reinstall oxrok after removing or
upgrading that Python.

Decoders are looked up in `OXROK_DECODERS_DIR`, `SIGROKDECODE_DIR`,
`decoders` next to the executable,
`~/.local/share/libsigrokdecode/decoders`, then
`/opt/homebrew/share`, `/usr/local/share` and `/usr/share` (each
followed by `libsigrokdecode/decoders`). To check the setup:

```sh
oxrok -L                       # lists the Python decoders too
oxrok -P can --show            # a decoder with no native version
oxrok -i capture.sr -P uart:baudrate=31250,midi -A midi
```

A native decoder takes precedence over a Python one with the same id.
`docs/DECODERS.md` describes the host.

## Layout

One crate, `oxrok`, with the library and the binary. Modules:

| module | contents |
|---|---|
| `core` | config keys and values, packets, the `Driver` and `Device` traits, sessions, triggers, parsing helpers |
| `formats` | text outputs, the `.sr` session format, VCD, CSV and binary readers and writers |
| `usb`, `serial`, `hid`, `scpi` | transports, each with record and replay fixtures |
| `ftdi`, `ezusb` | FTDI vendor requests and FIFO reads; Cypress FX2 firmware upload |
| `dmm` | multimeter packet parsers |
| `drivers::*` | demo, logicport, fx2lafw, serial_dmm, rigol_ds, raspberrypi_pico, dcttech_usbrelay, microchip_pickit2 |
| `decode`, `decoders` | the decoder API and the native `uart`, `i2c`, `spi` decoders |
| `pydecode` | runs libsigrokdecode's Python decoders (features `rustpython`, `python`) |

Cargo features: `cli` (the binary, default), `nusb` (default) and
`libusb` USB backends, `rustpython`, `python`. `hil/oxrok-hil` is the bench tool, not
published.

## Beyond sigrok: triggers

sigrok's trigger model stops at one condition per channel per stage.
oxrok extends the `-t` syntax where the hardware can follow, and the
parser rejects what a device cannot do:

```sh
-t 'D0=r*5'          # the fifth rising edge; *5c for the fifth in a row
-t 'D0=1>50us'       # D0 high for longer than 50 us (or >100 in samples)
-t 'D0-D2=3..5'      # the value on D2 D1 D0 inside 3..5; =!7 for outside
-t 'D0=r;D1=1'       # stages in sequence, as in sigrok-cli
-t 'D0=r|D1=r'       # either stage; & for both, in any order
```

The LogicPort runs all of these in its FPGA, with two stages, which is
what the hardware offers and its vendor software exposes. The software
trigger behind the demo and fx2lafw devices takes counts and durations.
These forms, the LogicPort's clock qualifier, setup and hold sampling
points, input inversion and frequency counter are not in sigrok; see
`docs/CLI.md` and `docs/DEVICES.md`.

## Compatibility with sigrok

- Configuration keys use the sigrok ids (`samplerate`, `limit_samples`,
  `voltage_threshold`, ...), so `-c` options and `.sr` metadata mean the
  same thing. Drivers can add their own keys; front ends show them like
  the rest.
- The command line accepts the sigrok-cli flags for the common flows and
  prints the same output. Differences are listed in `docs/CLI.md` as they
  appear.
- The C sigrok stack is never linked. It is used only as a test oracle
  during development.

## Building

Stable Rust, 1.95 or newer.

```sh
cargo build --workspace
cargo nextest run --workspace     # or cargo test
cargo clippy --workspace --all-targets
cargo fmt --all --check
```

## License

GPL-3.0-or-later, the license of the sigrok code it ports. See
`docs/LICENSING.md`.
