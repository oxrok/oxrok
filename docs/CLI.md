# The oxrok command line

`oxrok` accepts the sigrok-cli flags for the common flows and prints the
same output, byte for byte after the first header line (which names the
tool). The reference is sigrok-cli 0.8.0-git with libsigrok 0.6.0-git
(the current upstream code); `tools/diffcli.sh` runs the documented
flows through both tools and diffs them. Where 0.5.2 differed (the CSV
time column started one period late, the VCD body had no blank line
and single spaces), oxrok follows 0.6.

## Differences from sigrok-cli

Deliberate, and kept small:

- The header line of the logic outputs says `oxrok <version>` instead of
  `libsigrok <version>`. Same for `$version` and `$scope` in VCD files,
  `sigrok version` in session metadata and the CSV comment header.
  libsigrok's VCD reader prefixes the scope name to the channels of a
  file oxrok wrote (`oxrok.D0`); oxrok's reader treats both scope names
  as its own.
- `META samplerate: N` lines are printed only when the `analog` output
  is selected with `-O analog`. sigrok-cli prints them whenever a file
  input or a driver reports a sample rate, even into binary output.
- Analog packets go to the `analog` output only when the selected output
  cannot show them (`bits`, `hex`, `ascii`, `binary`). sigrok-cli also
  prints stray `A0: ...` lines next to `csv` and `vcd` output.
- The `binary` input parses `samplerate=200k` as 200 kHz. sigrok-cli
  reads it as 200 Hz, since it ignores the suffix for this option.
- A size with a fraction keeps its suffix: `samplerate=12.5M` is
  12.5 MHz. libsigrok's `sr_parse_sizestring()` skips the suffix after
  the fraction, so sigrok-cli sets 12 Hz (and a bare `12.5` makes it
  read past the string); oxrok refuses a bare fraction.
- The VCD input reads undefined bits (`x`, `z`, `u`, `-`) as 0 with one
  warning. sigrok-cli stops with an error. `--strict` turns every
  warning into a failed run, at any `-l` level, which gives the
  sigrok-cli behaviour for scripts that want it.
- Triggers: `-t` takes several stages separated by `;`, e.g.
  `-t 'D0=r;D1=1'`. One stage is written as in sigrok-cli. On top of
  that, when the device supports it (the parser rejects what it does
  not):
  - `D0=r*5` fires on the fifth rising edge, `D0=r*5c` on the fifth in
    a row: an edge while the stage's other conditions do not hold starts
    the count over.
  - `D0=1>100` fires once D0 has been high for more than 100 samples,
    `D0=1>50us` with a time, converted with the sample rate set by `-c`
    (in state mode, clock cycles).
  - `D0-D2=3..5` fires while the value on D2 D1 D0 (D0 least
    significant) is within 3..5, `D0-D2=!7` while it is not 7. Decimal
    or `0x` hex.
  - `D0=r|D1=r` fires on either stage, `D0=r&D1=r` once both have fired
    in any order. Exactly two stages.

  Counts, durations and ranges belong to the stage their token is in.
  The demo and fx2lafw devices (software trigger) take counts and
  durations; the LogicPort takes all of them with two stages, see
  `docs/DEVICES.md`. sigrok-cli rejects these forms. `>`, `;`, `|` and
  `&` need quoting in a shell.
- `rigol-ds` converts `horiz_triggerpos` to seconds with the number
  of horizontal divisions; libsigrok multiplies by the number of
  timebases in its table.
- `microchip-pickit2` talks HID reports instead of raw USB, so it also
  works on macOS, where libsigrok cannot claim the device.
- The demo device honours triggers (sigrok's ignores them): samples
  before the trigger are dropped, except the capture ratio's share of
  the sample limit, and the trigger is marked in the output.
- `--show` marks the current value in a float range list such as the
  LogicPort's `voltage_threshold`. sigrok-cli computes that list by
  accumulating the step, so its values never equal the current one and
  it marks none.
- `-I module --show` and `-O module --show` print the same option
  names with oxrok's own wording.
- `fx2lafw` sends frame packets only when `limit_frames` is not 1.
  libsigrok wraps every capture in a frame and re-arms once more before
  stopping, so its `bits` output ends with `FRAME-END` and
  `FRAME-BEGIN` lines. oxrok's fx2lafw sends the header before the
  first frame begins, so csv output from it starts with the frame
  separator (an empty line by default); libsigrok's begins that frame
  before the header, which sigrok-cli does not pass to its output.
- On the USBee AX with A0 enabled, triggers match the logic byte of
  each 16 bit sample. libsigrok's soft trigger takes one byte per
  sample (the logic channels, `soft-trigger.c:53`) on a stream of two
  (`protocol.c:439`): it matches the ADC's bytes as logic samples,
  starts the data at about twice the trigger offset, and sends the
  pre-trigger samples as logic packets of interleaved bytes, with no
  analog values.
- A trigger on an analog channel (`-t A0=r` on the USBee AX) is
  refused. sigrok-cli accepts it, and its soft trigger reads A0 as
  logic bit 8 (`soft-trigger.c:149`), on one-byte samples the next
  byte of the stream.
- The csv output writes logic and analog columns side by side
  correctly. libsigrok's csv indexes the analog samples by the column
  number over all channels (`output/csv.c:491`), so with logic columns
  first an analog column shows another sample's value and reads past
  its buffer.
- The demo device's default logic pattern is `oxrok`, a bitmap of that
  word. The other pattern names are the same. Disabled channels read low
  individually; sigrok's demo masks by the number of enabled channels.
- A `.sr` file written by oxrok is created when the first data arrives
  and finished at the end of the capture, in one pass. sigrok-cli reopens
  the archive for every chunk.

- `ftdi-la` reports `conn`: the device line reads `ftdi-la:conn=1.5`
  and `--get conn` answers the bus.address. libsigrok's driver never
  sets the conn it advertises, so sigrok-cli prints no `conn=` and
  `--get conn` fails with "invalid argument".
- `ftdi-la` keeps the samplerate in 64 bits: `samplerate=4295967296`
  fails with libftdi's "Unsupported baudrate", where libsigrok's
  `uint32_t` turns it into 1 MHz.
- `ftdi-la` gives the FTDI chip's port A back to the kernel's serial
  driver when it closes, so `/dev/ttyUSBn` comes back after a capture;
  sigrok-cli (through libftdi) leaves it detached.
- `saleae-logic16` refuses a sample rate whose data stays under 8000
  bytes a second (`-C 0 -c samplerate=1k`) with "Unable to sample at
  1000Hz.". libsigrok computes its transfer plan before it checks the
  rate, divides by zero (`api.c:544`), and sigrok-cli dies of SIGFPE.

## Not available yet

- `--protocol-decoder-jsontrace`. Python decoders need the `python`
  build feature; `docs/DECODERS.md`.
- `-T` (transform modules), `--list-supported-wiki`, the `gnuplot` and
  `scale` options of the `csv` output.
- The `csv` input and the other long tail input modules (ChronoVu LA8,
  STF, Trace32, ISF, WAV, LogicPort .lpf).
- The other libsigrok hardware drivers: the USB logic analyzers come
  next, the rest after them (`docs/ROADMAP.md`).
