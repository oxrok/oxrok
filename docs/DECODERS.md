# Protocol decoders

`oxrok::decode` is the runtime, `oxrok::decoders` the native decoders.
The command line flags are sigrok-cli's: `-P uart:baudrate=9600:rx=D0`
runs a decoder, `-A uart=rx-data:rx-start` selects annotation classes
or rows, `-B uart=rx` writes a binary class to stdout, `-M` shows
metadata, `--protocol-decoder-samplenum` and
`--protocol-decoder-ann-class` add fields, and `-l 4` shows everything
with quotes and abbreviations as sigrok-cli does.

## The API

A decoder is a `Decoder` with a static `DecoderInfo`: id, names,
channels (required and optional), options with typed defaults and
allowed values, annotation classes and rows, binary classes. Its
`decode` loop asks a `Context` for samples with `wait`, which takes
groups of conditions (levels, edges, a sample count to skip) and
returns the pin levels at the first sample where a group holds, with
`matched` saying which. It emits with `put_ann`, `put_binary`,
`put_python` (items for stacked decoders) and `put_meta`.

This is libsigrokdecode's Python API with Rust types, so a Python
decoder maps onto it one to one and a native decoder reads like one.
`wait` follows the same rules: the first sample is examined in place,
edges compare with the previous sample (none before the first), `skip`
counts from the current sample.

## Running

A `Stack` runs each instance on its own thread fed by a bounded sample
stream; outputs arrive on a channel in emission order. The command line
creates the stack on the first logic packet, when the sample width is
known, feeds every logic packet and prints as output arrives. Instance
ids are `<decoder>-<n>`.

Unassigned decoder channels default to the logic channel of the same
index, as in libsigrokdecode; a channel name that does not exist is a
warning. Option values are parsed to the type of their default and
checked against the allowed values.

## Native decoders

30 of the 111 decoders of libsigrokdecode 0.5.3 are native now;
`docs/NATIVE_DECODERS.md` is the checklist, with the status, batch and
the documented exceptions for every decoder. Each native decoder
reproduces libsigrokdecode's annotation classes, rows, texts, sample
ranges, binary output and metadata, and has an in-process native-vs-
Python parity test: the two run on the same capture and configuration,
and the full output sequence must match. The sigrok-test corpus is an
opt-in, larger check (`OXROK_CORPUS=1`).

`uart`, `i2c` and `spi` also have byte-for-byte sigrok-cli goldens in
`tests/decode_cli.rs`, and `tools/diffcli.sh` repeats the comparison in
every output mode. Two details to know: `uart` keeps an upstream quirk
that selections depend on (stop bits are tagged with the parity-ok
class), and the i2c decoder's short name is `I2C`, not the upstream
`I2C` with a superscript two, so the sources stay ASCII.

Each native decoder's source names the `pd.py` it corresponds to and
that file's authors, like the rest of the code (`docs/LICENSING.md`),
and is written from the protocol and the decoder's documented output.

## Python decoders

With the `rustpython` or the `python` cargo feature (installation in
the README) oxrok runs the unchanged `pd.py` files of a
libsigrokdecode installation. The decoders are looked for in
`OXROK_DECODERS_DIR`, `SIGROKDECODE_DIR`, `decoders` next to the
executable, `~/.local/share/libsigrokdecode/decoders` and the system
directories. Native decoders take precedence over a Python decoder of
the same id.

Each Python decoder is a runtime `Decoder` whose `decode` runs on the
instance's thread. Both hosts provide the `sigrokdecode` module the
decoders import: the `Decoder` base class with `wait`, `put`,
`register` and `has_channel`, the output constants and
`SRD_CONF_SAMPLERATE`; `options`, `samplenum` and `matched` are set on
the instance as libsigrokdecode does. `wait` lets other threads run
while it blocks for samples. All 111 decoders of libsigrokdecode 0.5.3
load on both hosts; `timing`, chains such as `uart,midi`,
`i2c,eeprom24xx`, `i2c,rtc8564`, `spi,sdcard_spi` and `parallel` print
what sigrok-cli prints.

Stacking is synchronous: a decoder's Python output is handed to the
decoder above before the lower one continues, so annotations of a chain
interleave as in libsigrokdecode. An exception in the upper decoder on
one item is logged and the next item is delivered anyway, which is
also libsigrokdecode's behaviour. Items pass between two Python
decoders as the original objects; between a native and a Python decoder
they are converted (lists, numbers, strings, bytes, None).

### The two hosts

- `python` (`src/pydecode/cpython.rs`): the system's CPython through
  PyO3 0.30. `sigrokdecode` is a module written in Rust; `wait`
  detaches from the interpreter while it blocks.
- `rustpython` (`src/pydecode/rustpython.rs`): RustPython 0.6.0 with
  its standard library frozen into the binary, so nothing but the
  decoders is needed at run time. `sigrokdecode` is
  `src/pydecode/sigrokdecode.py`, over a native `_oxrok` module that
  reaches the running decoder's context. RustPython has no global lock;
  each thread gets its own virtual machine on the shared interpreter.

With both features CPython is used unless `OXROK_PYTHON=rustpython`.

What it takes to run the decoders on RustPython the way CPython does:

- `sum()` of floats is CPython's since 3.12, with Neumaier's
  compensation. RustPython adds plainly, which changed the last digit
  of `timing` averages and `graycode` periods, so the shim installs a
  copy of CPython's algorithm as `builtins.sum`.
- Compiler warnings about the invalid escapes in older decoders are
  silenced, and no bytecode is written next to the decoders.
- RustPython 0.6.0 builds only with pins: its ruff parser fork changed
  its API within 0.16.x, so the five `rustpython-ruff_*` crates are
  pinned to 0.16.5, and pymath accepts any `malachite-bigint` 0.x while
  RustPython needs 0.12 (the lockfile pins it; install with
  `--locked`). The parser needs Rust 1.96.
- Interpreter start and deep decoders need more than the default 2 MiB
  thread stack in a debug build; decoder threads get 8 MiB.

Over 271 sigrok-test runs of the Python decoders (two captures per
decoder, chains included, annotations and binary outputs) both hosts
print the same bytes; the whole set took 1.2 times as long on
RustPython. Long decodes in one process take up to about three times
as long.

The default build has no Python dependency.

## Fixtures

`tests/fixtures` holds the captures the conformance tests
run on, each with sigrok-cli's output next to it:

- `uart-hi.vcd`, `i2c-write-read.vcd`, `spi-mode0.vcd`: synthetic,
  generated by the scripts described in the test files, covering the
  frame formats precisely.
- `uart-esp32.sr`, `i2c-esp32.sr`, `spi-esp32.sr`: taken with the
  LogicPort from the ESP32 generator on the bench: its `PONG` reply at
  921600 baud, an I2C write and read to address 0x50 played by its
  sequencer on mapped pins, and an SPI transfer from its `SPI` command.

Recording them needs one generator connection across the capture:
opening the serial port resets the ESP32, so `oxrok-hil --cmd ...
--delay 4000 --cmd ...` runs the setup, waits while the analyzer arms,
and then plays the waveform.

## Not yet

`--protocol-decoder-jsontrace`, and a Python host that needs no
interpreter on the machine.
