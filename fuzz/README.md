# Fuzz targets

Random and mutated input for the code that reads what oxrok does not
control: files, logic samples, and what an instrument sends. The test
suite compares oxrok with sigrok on well-formed input; these look for the
panic on the rest. A panic in a decoder or a driver thread ends a capture,
and a `.sr` or `.vcd` file may come from anyone.

This is a [cargo-fuzz](https://github.com/rust-fuzz/cargo-fuzz) crate
with a workspace of its own. It is not a member of the repository's
workspace, is not in the published crate, and needs a nightly toolchain,
which the rest of the repository does not.

## Targets

| target | input | what it reaches |
| --- | --- | --- |
| `input_vcd` | a file | the `vcd` input module, fed in pieces |
| `input_binary` | a file | the `binary` input module, fed in pieces |
| `sr_open` | a file | `formats::sr::SessionReader`: the zip, the metadata, the capture files |
| `decoder_any` | 2 bytes decoder, 1 byte rate and channels, then samples | any of `decoders::ALL` on a `decode::Stack`, default options; a stacked decoder runs over the decoders below it |
| `serial_frames` | 1 byte protocol, then the line | framing and parsing of every `serial-dmm` protocol and of the LCR chips |
| `scpi_block` | 1 byte call and read size, then the answer | `Scpi::get_block`, `get_floatv`, `get_string`, `*IDN?` parsing over a canned transport |

What passes: the code returns, with a value or an error. A decoder that
ends with an error on samples that make no sense is fine, as a Python
decoder that raises is for libsigrokdecode. What fails: a panic (index,
slice, `unwrap`, arithmetic overflow: debug assertions are on), a hang
(over the `-timeout`) or memory without bound. A hang or an out of memory
report on a file that honestly asks for that much, such as a VCD with one
far timestamp, is to be weighed, not fixed by reflex.

The drivers themselves are not here: their fault cases (short and garbage
answers, stalls, unplugs) are in the emulator tier, against sigrok-cli.

## Running

Once:

    rustup toolchain install nightly
    cargo install --locked cargo-fuzz

From the repository root, where `rust-toolchain.toml` says stable, so
name nightly:

    cp Cargo.lock fuzz/Cargo.lock      # the dependency versions that ship
    sh fuzz/seed.sh                    # corpora from tests/fixtures
    cargo +nightly fuzz run input_vcd -- -max_total_time=300 -timeout=30
    cargo +nightly fuzz build          # all of them, to see that they compile
    cargo +nightly fuzz list

`fuzz/Cargo.lock`, `fuzz/corpus/`, `fuzz/artifacts/` and `fuzz/target/`
are not committed. A corpus grows as a target runs and is worth keeping
on the machine between runs.

A failing input is written to `fuzz/artifacts/TARGET/`. To see it again,
and to shrink it:

    cargo +nightly fuzz run input_vcd fuzz/artifacts/input_vcd/crash-...
    cargo +nightly fuzz tmin input_vcd fuzz/artifacts/input_vcd/crash-...

Then fix the code, and put the shrunk input in a `#[test]` beside it: the
test suite, not a corpus, is what keeps a fixed panic fixed.

Without nightly the targets still build and replay a file, with no
coverage feedback, which is enough to check a fix or that a target
compiles:

    cargo +stable build --manifest-path fuzz/Cargo.toml
    fuzz/target/debug/input_vcd some-file.vcd

## In CI

`.github/workflows/nightly.yml` runs each target for five minutes every
night, from the seeds, and by hand from the Actions tab for longer. It is
not a required check and opens no issue: a target that fails turns its
job red, uploads the input as the artifact `fuzz-TARGET` and says in the
job summary how to run it.

## Adding a target

A file in `fuzz_targets/`, a `[[bin]]` in `Cargo.toml`, a line in the
matrix of `nightly.yml` and a row in the table above. Seeds, when the
input is a file format, go in `seed.sh`. Keep a target to one parser or
one family, with a leading byte to choose among them: libFuzzer learns
one input shape at a time.
