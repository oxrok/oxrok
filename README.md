<p align="center"><img src="https://raw.githubusercontent.com/mangelajo/oxrok/main/docs/logo/lockup.svg" alt="oxrok" width="250"></p>

An experimental sigrok compatible stack for logic analyzers, scopes and
meters, in Rust: a library, drivers and a command line tool. It builds
with `cargo` alone, links no C sigrok library, and takes sigrok-cli's
flags, file formats, driver ids and decoder ids. libsigrokdecode's Python
decoders run unchanged, on a built-in RustPython interpreter (feature
`rustpython`) or on the system's CPython (feature `python`).

oxrok is not part of the sigrok project.

## How it was written

oxrok is a translation of the sigrok C and Python code into Rust, done by
a large language model (Claude), with a human directing the work: choosing
what to port, reviewing the structure and the protocol handling, wiring up
the bench hardware and running the comparisons. Read the code with that in
mind, and read the table below for what has met which kind of evidence.

## Status

Early, and most of it has never touched the device it drives.

On 2026-11-05 `src/lib.rs` registers 39 driver modules, with 135 driver
ids (`oxrok -L`; most ids are multimeters). 38 of the modules are
counterparts of libsigrok's 87 `src/hardware` directories, and one, the
LogicPort, is not in libsigrok; the `serial-dmm` module also takes in
`uni-t-ut32x`. More are being ported: [docs/ROADMAP.md](docs/ROADMAP.md) has the current state.

Also there: the `bits`, `hex`, `ascii`, `analog`, `csv`, `vcd`, `binary`
and `srzip` outputs; `.sr`, VCD and binary inputs (no CSV input yet, see
[docs/CLI.md](docs/CLI.md)); USB, serial, HID and SCPI transports with
record and replay; native `uart`, `i2c` and `spi` decoders.

### What has been checked how

"Emulator" means the cases described in
[How drivers are tested](#how-drivers-are-tested): parity with libsigrok
against a device model, not contact with hardware. Cases are counted from
`tests/emu/MODEL/*.out`, gcov from `tests/emu/MODEL/gcov.txt`, both on
2026-11-05. gcov is the share of the C driver's lines the cases reach.

Run on real hardware:

| driver | hardware | emulator | notes |
|---|---|---|---|
| `intronix-logicport` | two LogicPort LA1034 | no model | 60 bench checks with an ESP32 signal generator; captures match the libsigrok fork that has this driver |
| `fx2lafw` | a Saleae Logic clone (`0925:3881`) | 78 cases, gcov 91.2% | upload and open order changed since the bench run, checked on the emulator only |
| `rigol-ds` | a Rigol DS1102E | no model | other series have replay tests only |
| `raspberrypi-pico` | a Pico with the sigrok-pico firmware | no model | digital, analog and mixed captures |

Run on a microcontroller standing in for the device, and on the emulator,
with no real device yet:

| driver | stand-in | emulator | notes |
|---|---|---|---|
| `dcttech-usbrelay` | Pico | 30 cases, gcov 82.2% | |
| `microchip-pickit2` | Pico | 48 cases, gcov 84.0% | |
| `serial-dmm` family | ESP32 for UT61E and UT60A streams; Pico for the BU-86X, CH9325 and Victor cables | 92 cases over 5 models (4 cables, CDC-ACM), gcov 60% to 77% | CP2110 and CDC-ACM on the emulator only; the other meters have replay tests only |

Emulator and replay only, never run on hardware:

| driver | emulator | gcov |
|---|---|---|
| `asix-sigma` | 110 cases | 89.4% |
| `atten-pps3203` | 133 | 90.5% |
| `chronovu-la` | 52 | 87.0% |
| `dreamsourcelab-dslogic` | 94 | 90.2% |
| `fluke-dmm` | 159 | 98.6% |
| `ftdi-la` | 46 | 86.4% |
| `greatfet` | 131 | 84.8% |
| `gwinstek-gpd` | 141 | 95.6% |
| `hantek-4032l` | 135 | 91.4% |
| `hantek-6xxx` | 102 | 91.0% |
| `hantek-dso` | 148 | 96.2% |
| `ikalogic-scanalogic2` | 143 | 88.6% |
| `ikalogic-scanaplus` | 53 | 93.8% |
| `kecheng-kc-330b` | 129 | 79.3% |
| `kingst-la2016` | 158 | 91.2% |
| `korad-kaxxxxp` | 130 | 88.6% |
| `lascar-el-usb` | 152 | 93.9% |
| `lecroy-logicstudio` | 94 | 95.9% |
| `manson-hcs-3xxx` | 171 | 91.9% |
| `ols` | 213 | 94.7% |
| `p-ols` (Pipistrello) | 107 | 97.0% |
| `rdtech-dps`, `rdtech-rd` | 226 | 89.2% |
| `rdtech-tc` | 73 | 93.8% |
| `rdtech-um` | 90 | 93.7% |
| `saleae-logic16` | 87 | 91.6% |
| `saleae-logic-pro` | 109 | 97.1% |
| `sysclk-lwla` | 141 | 91.1% |
| `sysclk-sla5032` | 165 | 91.5% |
| `testo` | 70 | 91.3% |
| `uni-t-dmm` raw USB | 78 | 97.4% |
| `zeroplus-logic-cube` | 134 | 87.6% |

`demo` is software, covered by the software cases of
`tools/diffcli.sh`. The native decoders were compared with libsigrokdecode on captures from an ESP32.

[docs/DEVICES.md](docs/DEVICES.md) has the details per device, and what is
known to differ. If you own one of these devices, a report is worth more
than another emulator case: the output of a capture next to sigrok-cli's,
and a recording (`OXROK_USB_RECORD=file oxrok -d DRIVER ...`) if they
differ.

CI runs on Linux, macOS and Windows.

## Trying it

From a checkout, with no device:

```sh
oxrok --scan
oxrok -d demo:logic_channels=4 -C D0,D1,D2,D3 -g Logic -c pattern=walking-one --samples 20 -O bits
oxrok -d demo --show
oxrok -d demo --samples 1k -o capture.sr      # a session file for PulseView
oxrok -i capture.sr -O csv
oxrok -i tests/fixtures/uart-hi.vcd -P uart:baudrate=115200:rx=top.D0 -A uart=rx-data
```

On a tested command the output after the first header line is the same as
sigrok-cli's, byte for byte. That is what the test suite compares, on its
cases; the demo's random analog channel (`A4`), for one, differs from run
to run in either tool. Known differences are in [docs/CLI.md](docs/CLI.md),
and command lines for real devices in [docs/DEVICES.md](docs/DEVICES.md).

## Installing

### From a checkout

No binaries have been released yet: the repository's Releases page is empty
on 2026-11-04. Build it with Rust 1.95 or newer:

```sh
cargo install --path . --locked
```

crates.io has one version, `0.1.0-alpha.1` of 2026-10-23, published to
claim the name. It has 8 of the 23 driver modules and none of the
emulator's work, so use the checkout.

On Linux a USB device is root only without a udev rule, and oxrok ships no
rules yet; [docs/USB.md](docs/USB.md#troubleshooting) has this and the
macOS and Windows notes. Drivers that upload firmware look for it where
libsigrok does (`SIGROK_FIRMWARE_DIR`, `sigrok-firmware` under the data
directories; `src/core/firmware.rs` has the order). oxrok ships none.

### Features

| feature | what | needs |
|---|---|---|
| `cli` (default) | the `oxrok` binary | |
| `nusb` (default) | pure Rust USB backend: Linux, macOS, Windows, Android | |
| `libusb` | libusb backend, also for the BSDs, Solaris, Haiku; built from source. `OXROK_USB_BACKEND=libusb` selects it, nusb stays the default | a C compiler |
| `rustpython` | Python decoders on RustPython, built into the binary | Rust 1.96 or newer, `--locked` |
| `python` | Python decoders on the system's CPython, through pyo3 | Python 3.9 or newer with its shared library, at build and run time |

```sh
cargo install --path . --locked --features rustpython
cargo install --path . --locked --features python
PYO3_PYTHON=/usr/bin/python3.12 cargo install --path . --locked --features python
```

For `python`: Homebrew's and python.org's Pythons include the library; on
Debian and Ubuntu install `python3-dev`, on Fedora `python3-devel`; on
Windows use the python.org installer with Python added to `PATH`. The build
uses `PYO3_PYTHON` when set, else the active virtualenv, else `python` or
`python3` on `PATH`. Reinstall oxrok after removing or upgrading that
Python.

Either feature needs the decoders themselves: `brew install
libsigrokdecode`, your distribution's package, or the `decoders` directory
of a libsigrokdecode checkout (0.5.3 is the version oxrok follows). They
are looked up in `OXROK_DECODERS_DIR`, `SIGROKDECODE_DIR`, `decoders` next
to the executable, `~/.local/share/libsigrokdecode/decoders`, then
`/opt/homebrew/share`, `/usr/local/share` and `/usr/share` (each followed
by `libsigrokdecode/decoders`). To check:

```sh
oxrok -L                       # lists the Python decoders too
oxrok -P can --show            # a decoder with no native version
```

A native decoder takes precedence over a Python one with the same id. With
both features, CPython is used unless `OXROK_PYTHON=rustpython`. Over 271
sigrok-test runs the output on RustPython is the same as on CPython, and
the set took 1.2 times as long; long decodes take up to about three times
as long. [docs/DECODERS.md](docs/DECODERS.md) describes the host.

### Release artefacts

None is published yet. This is what a release will hold, built by `make
dist` and `make dist-src` in the `Release binaries` workflow
(`.github/workflows/release.yml`), each file with a `.sha256`:

- One archive per target: Linux x86_64 and aarch64 (static, musl), macOS
  x86_64 and arm64, Windows x86_64. It has `oxrok` built with `rustpython`,
  libsigrokdecode 0.5.3's `decoders/` to keep next to it, `LICENSE`,
  `NOTICE`, `THIRD-PARTY-NOTICES.txt` for that target, `README.md` and
  `CHANGELOG.md`.
- One source tarball, `oxrok-<version>-vendored.tar.xz`: the tagged tree,
  every crate of `Cargo.lock` in `vendor/`, the decoders and the notices,
  so that `make dist` works in it with no network.

No firmware is in either. [docs/LICENSING.md](docs/LICENSING.md) has the
details.

## How drivers are tested

### The problem

libsigrok has dozens of drivers for devices nobody here owns. Porting them
from the C source alone would give code that compiles and has never talked
to anything. Recordings would need the devices too.

### The emulator

`oxrok-emu` (the `emu` crate) is a USB/IP server in Rust, on the
[`vusb`](https://crates.io/crates/vusb) crate. A device model is
plain Rust: descriptors, answers to control and bulk transfers, and checks
on what the host sends. The server attaches it through Linux's `vhci-hcd`
module, so the kernel enumerates a USB device like any other, and neither
tool knows the difference. sigrok-cli runs unmodified.

```text
  oxrok-emu                                Linux kernel
 +-------------------+    USB/IP      +------------------------+
 | device model      |   over TCP     | vhci-hcd               |
 |   pedantic checks |<-------------->|   (virtual host        |
 |   fault injection |   (loopback)   |    controller)         |
 | transaction log   |                | usbcore, usbhid, ...   |
 +-------------------+                +-----------+------------+
                                                  |
                                       /dev/bus/usb, /dev/hidraw
                                                  |
                                   +--------------+--------------+
                                   |                             |
                                 libusb                         nusb
                                   |                             |
                       libsigrok, sigrok-cli                   oxrok
                       (the oracle, the same model)         (the port)
```

Transport shells, vusb's, carry what a kind of device shares: HID, the FTDI
chips, EZ-USB firmware upload and renumeration, CDC-ACM. `emu/` holds the
models, the `.tx` log and what knows sigrok's fixtures. On
2026-11-04 there are 24 models (`oxrok-emu list`), all USB: serial and SCPI
instruments are still open.

### Differential testing

The oracle is the C libsigrok, built with `--coverage`, at a pinned commit
([docs/DEVICES.md](docs/DEVICES.md#oracle)). A scenario file,
`tools/emu/MODEL.sh`, lists the cases, and `tools/diffcli.sh --emu` runs
each one on a fresh model under both tools. They must give the same output
and cause the same USB transaction log, with no violation logged under
either. Identical bytes on the wire is a tighter claim than identical
output. It is not unconditional: node numbers and device addresses are made
generic, and a case can filter what differs for a stated reason, such as log
prefixes or, narrowly, the order of transactions that depends on the C's
callback timing.

This is part of `tools/emu/usbrelay.sh`:

```sh
EMU_MODEL=usbrelay
EMU_OPTIONS="-o relays=2"
EMU_DRIVER=dcttech-usbrelay

emu_cases() {
    R=dcttech-usbrelay
    emu_case "relay show" --keep-header -- -d $R --show
    emu_case "relay get R1" --record --keep-header -- -d $R -g R1 --get enabled
    emu_case "relay by path" --keep-header -- -d $R:conn=$EMU_HIDRAW -g R1 --get enabled
    # The SET_REPORT that switches a relay STALLed.
    usbrelay_options relays=2 stall-set=1
    emu_case "relay fault switch stalled" --keep-header --expect-fail --stderr -- -d $R -g R2 --set -c enabled=on
}
```

What follows the `--` is the command line both tools get.

- **Goldens.** Each case's output and transactions are committed as
  `tests/emu/MODEL/SLUG.out` and `.tx`. `relay-get-r1.tx`, in the same
  dialect as the replay fixtures:

  ```text
  ctl_in 21 01 0300 0000 8 -> 4142434445000000
  ctl_in 21 01 0300 0000 8 -> 4142434445000000
  ```

- **CI without sigrok.** `tools/diffcli.sh --no-oracle` runs oxrok alone
  against those goldens. A `.github/workflows/ci.yml` job does this; it is
  skipped, not failed, when the runner's kernel has no `vhci-hcd`.
- **gcov.** `--gcov` reports which lines of the C driver the cases reached,
  in `tests/emu/MODEL/gcov.txt`. Each unreached line is meant to get a case,
  a fault option, or a reason in the driver's "not emulated" list. The
  figure says when to stop writing cases. It says nothing about oxrok's own
  lines.
- **Beyond sigrok-cli.** `tools/libsigrok-run.c` drives libsigrok directly
  where sigrok-cli cannot: lecroy-logicstudio offers no limit, so sigrok-cli
  refuses to capture, and a stop from inside a transfer callback has no
  command line. It takes sigrok-cli's place through `emu_case --libsigrok`.

Counts on 2026-11-04: 2041 cases passed in a full run across all models,
oracle-only ones included, and about 1744 under `--no-oracle`, which skips
cases that have no oxrok driver or only the C runner. `cargo test
--workspace` runs 920 unit and integration tests.

### Models are pedantic and hostile

A model does not accept whatever arrives. `Device` flags transfers to
endpoints the configuration lacks, or before `SET_CONFIGURATION`; a model
flags what its protocol forbids: lengths, register values, sequence numbers,
order. A violation fails the case. A clean run under sigrok-cli checks the
model, and a clean run under oxrok checks the port. Each check gets a unit
test that makes it fire.

Models also misbehave on request, with options that are off by default:
STALLed requests, short, empty or garbage answers, a transfer that never
completes. The harness can unplug the device or interrupt the tool at a
chosen transfer. Error paths are where the C and the port differ most.

### Where the two differ on purpose

libsigrok has bugs, and they are not copied. When a case differs, the first
job is to find out which side is wrong. A claim that the C is wrong needs
its lines quoted.

- `--oracle-bug`: sigrok-cli fails (a crash, say). oxrok is held to its
  goldens and sigrok-cli's failure is kept as `SLUG.oracle`.
- `--oracle-differs`: sigrok-cli exits 0 with another result, kept the same
  way.

Either kind fails once the two agree, so a fix in libsigrok shows up. On
2026-11-04 there are 197 `.oracle` files. The C bugs and each port's
differences are documented per device in
[docs/DEVICES.md](docs/DEVICES.md).

### Adversarial review

Each model and port also goes to a reviewer, another agent, reading it
against the C. The lessons go into
[docs/DRIVER_PORTING.md](docs/DRIVER_PORTING.md). Three of them:

- **Pipistrello.** The model's trigger timing came from the C driver, and
  two `--oracle-differs` cases measured a driver "bug" against it, which is
  circular. The driver author's firmware source overturned three of the
  model's assumptions. Device sources now outrank a model's guesses, though
  that source was only read, not simulated, and may not be what boards run.
- **rusb.** A false comment said rusb drops partial data on a timeout, and
  a shared method was built on it. It caused a hang, which hantek-6xxx's
  review hit in 5 of 8 runs, and 27 false failures in a libusb run, until
  someone read rusb's source. A claim about a dependency now needs that.
- **lecroy-logicstudio.** A review found that 20 of the model's 50 checks
  never fired in any test, and two others were wrong: hence the unit test
  per check.

### Limits

- A model is built from the C driver and whatever public sources exist, so
  it can share the C's misunderstanding of the device. Matching it proves
  parity with libsigrok on the paths the cases reach, not correct behaviour
  on hardware. Quirks the C relies on without saying cannot be found this
  way: the LogicPort bench found three.
- Timing is not real. Models count the host's transfers rather than the
  clock, and USB/IP has no bus. Lost samples at speed, short packet framing
  on the wire and real analog front ends need hardware.
- Where the vendor's firmware cannot be redistributed, both tools upload a
  stand-in of the right name and size. It is opaque to the model.
- Hardware checks remain in `hil/` (firmware for an ESP32 and a Pico, the
  `oxrok-hil` bench tool, `tools/diffcli.sh --hw`), with the open list in
  [docs/ROADMAP.md](docs/ROADMAP.md#open-verification).
- The emulator is Linux only, so macOS and Windows USB paths get replays,
  not emulation.
- The regular runs use nusb. The libusb backend has only occasional
  emulator runs, and differs from the goldens at least by string reads at
  listing.
- The oracle carries one patch, restoring a line in testo's scan without
  which stock libsigrok finds no Testo. One driver, `hp-3478a`, is not built
  in it, for want of libgpib.

### Running it

Without any of this, on any OS:

```sh
cargo test --workspace
```

That includes replay fixtures, recorded from the emulator by `emu_case
--record` into `tests/fixtures/DRIVER/`, which `tests/emu_replay.rs` plays
back through the oxrok binary, with no root or device. By hand, from a
checkout:

```sh
OXROK_HID_REPLAY=tests/fixtures/dcttech_usbrelay/emu-relay-get-r1.txt \
    target/debug/oxrok -d dcttech-usbrelay -g R1 --get enabled
```

A replay fails on the first call that differs from the recording, but it
checks only the one or two cases recorded per driver.

The emulator needs Linux, the `vhci-hcd` module (in `kernel-modules-extra`
on Fedora, `linux-modules-extra-$(uname -r)` on Debian and Ubuntu) and
passwordless sudo, as the server attaches devices as root. The harness
drives the server through vusb's `vusb-ctl`:

```sh
sudo modprobe vhci-hcd
cargo install vusb-cli --version 0.1.1 --locked
cargo build --workspace

# oxrok against the committed goldens; no sigrok needed.
sh tools/diffcli.sh --no-oracle
sh tools/diffcli.sh --no-oracle --emu-model usbrelay

# Both tools. Build the oracle as docs/DEVICES.md says and put its prefix
# on PATH and LD_LIBRARY_PATH.
sh tools/diffcli.sh                                # software cases, no USB
sh tools/diffcli.sh --emu                          # those and every model
sh tools/diffcli.sh --emu-model usbrelay           # one model
sh tools/diffcli.sh --emu-model usbrelay --gcov    # and its C coverage

# Serve a model by hand; the lock keeps it out of a running session's way.
flock /tmp/oxrok-emu.lock sudo target/debug/oxrok-emu serve usbrelay --attach --open-devices -v --control /tmp/emu.sock
vusb-ctl -s /tmp/emu.sock status
```

To add a model, see
[docs/USB.md, "Writing a device model"](docs/USB.md#writing-a-device-model):
a file in `emu/src/models`, a scenario in `tools/emu`, goldens blessed with
`EMU_BLESS=1` and then read, gcov, and an entry in `docs/DEVICES.md`.
[docs/USB.md](docs/USB.md#emulation-over-usbip) describes the server and the
harness, and [docs/ROADMAP.md](docs/ROADMAP.md#emulation-plan) the plan and
what counts as emulator-verified.

## Layout

The `oxrok` crate has the library and the binary:

| module | contents |
|---|---|
| `core` | config keys and values, packets, the `Driver` and `Device` traits, sessions, triggers, firmware lookup |
| `formats` | text outputs, srzip and the `.sr` session format, VCD and binary input and output |
| `usb`, `serial`, `hid`, `scpi` | transports, each with record and replay |
| `ftdi`, `ezusb` | FTDI requests, FIFO reads and libftdi's behaviour; Cypress FX2 firmware upload |
| `dmm` | multimeter packet parsers |
| `drivers::*` | one module per driver, as in the tables above |
| `decode`, `decoders` | the decoder API and the native `uart`, `i2c`, `spi` decoders |
| `pydecode` | runs libsigrokdecode's Python decoders (features `rustpython`, `python`) |
| `cli` | the command line (feature `cli`) |

The workspace adds `emu` (`oxrok-emu`) and `hil/oxrok-hil` (the bench
tool), neither published. `tools/` has the harness and scenarios,
`tests/emu` the goldens, `tests/fixtures` the replay fixtures and `docs/`
the depth.

## Beyond sigrok: triggers

sigrok's trigger model stops at one condition per channel per stage. oxrok
extends the `-t` syntax where the hardware can follow, and the parser
rejects what a device cannot do:

```sh
-t 'D0=r*5'          # the fifth rising edge; *5c for the fifth in a row
-t 'D0=1>50us'       # D0 high for longer than 50 us (or >100 in samples)
-t 'D0-D2=3..5'      # the value on D2 D1 D0 inside 3..5; =!7 for outside
-t 'D0=r;D1=1'       # stages in sequence, as in sigrok-cli
-t 'D0=r|D1=r'       # either stage; & for both, in any order
```

The LogicPort runs all of these in its FPGA, with two stages, which is what
the hardware offers. The software trigger behind the demo and fx2lafw
devices takes counts and durations. These forms, and the LogicPort's clock
qualifier, setup and hold sampling points, input inversion and frequency
counter, are not in sigrok; see [docs/CLI.md](docs/CLI.md) and
[docs/DEVICES.md](docs/DEVICES.md).

## Compatibility with sigrok

- Configuration keys use the sigrok ids (`samplerate`, `limit_samples`,
  `voltage_threshold`, ...), so `-c` options and `.sr` metadata mean the
  same thing. Drivers can add their own keys.
- The command line takes the sigrok-cli flags for the common flows.
  [docs/CLI.md](docs/CLI.md) lists the differences and what is missing.
- The C sigrok stack is never linked. It is used only as a test oracle.

## Building and contributing

Stable Rust, 1.95 or newer.

```sh
cargo build --workspace
cargo nextest run --workspace     # or cargo test
cargo clippy --workspace --all-targets
cargo fmt --all --check
```

[CONTRIBUTING.md](CONTRIBUTING.md) has the rules, and
[docs/DRIVER_PORTING.md](docs/DRIVER_PORTING.md) the shape of a driver and
the pitfalls met so far.

## License

GPL-3.0-or-later, the license of the sigrok code it ports. The
zeroplus-logic-cube driver also ports BSD-style files, whose notice is in
[NOTICE](NOTICE). See [docs/LICENSING.md](docs/LICENSING.md).
