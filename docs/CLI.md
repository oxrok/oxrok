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
- `chronovu-la` honours `--samples` and `--time`: the device still sends
  its whole memory, and oxrok outputs only what was asked. libsigrok
  stores both limits, never reads them, and outputs all 8 Mi samples (4
  Mi on the LA16).
- `chronovu-la` on the LA16 marks the trigger at the sample that met it,
  also for edge-only triggers. libsigrok compares single bytes against a
  16 bit mask, so `-t 0=1` gives a 1 byte packet that crashes `hex` and
  `bits` and shifts `csv`, and an edge-only trigger gives no marker. One
  limit: an edge channel already at its new level on sample 0 counts as
  the edge, so if the device kept samples before its trigger the marker
  can land on sample 0, ahead of the real edge.
- `chronovu-la` ends a capture whose device is unplugged, logging each
  failure once. libsigrok recurses on the failed write until the stack
  overflows.
- `chronovu-la` waits for a trigger for 10 s plus the capture time at
  the rate set, in reads at least 16 ms apart. libsigrok waits 10 s of
  wall time at any rate, which on a real LA8 is shorter than a full
  memory takes below about 833 kHz.
- `chronovu-la` resets the sequencer and closes the chip on a stop, so
  the next capture needs the device opened again. libsigrok only removes
  its source and sends the end (`api.c:502-508`), leaving the device
  mid-capture.
- `chronovu-la` opens the device the scan found. libftdi opens the first
  with those ids and that product string, so of two boards libsigrok
  opens the first for both.
- asix-sigma: a capture that wrapped the memory has the whole last row
  more than sigrok-cli's: 448 samples for a dense signal at up to 50
  MHz. At 100 and 200 MHz an event is 2 and 4 samples, and a sparse
  signal repeats samples across gaps, so the number differs.
- asix-sigma: a trigger in the first 28 events of the memory gets a
  marker; sigrok-cli prints none.
- asix-sigma: `external_clock_source=16` with `external_clock=1`
  captures as sigrok-cli does, with a warning on stderr.
- asix-sigma: a forced stop that never completes ends the capture with
  "Could not poll for post-trigger state." after 1000 mode reads, or 100
  once a stop was asked for; sigrok-cli keeps polling.
- `ikalogic-scanaplus` ends a capture on whichever of `--time` and
  `--samples` is reached first. libsigrok tests the sample limit first
  on each decoded read (protocol.c:358-367), so `--time 1 --samples
  300000` gives 300000 samples there and 100000 here.
- `ikalogic-scanaplus` decodes the stream correctly after a short or odd
  read. libsigrok's output is shifted or split then
  (protocol.c:345-352).
- `ikalogic-scanaplus` gives port A back to the kernel's serial driver
  when it closes, at the scan's close too, like `ftdi-la`.
- p-ols: captures that sigrok-cli crashes on or never ends (a limit not
  a multiple of 4, 4 stages, `captureratio=100`) complete. All 32
  channels at 200 MHz and a board without a memory size are refused with
  a message.
- p-ols: on a slow board `samplerate` reads the rate it starts at, not
  0.
- All drivers: a scan option the driver does not list is refused
  ("Unknown scan option 'conn'.", exit 1); sigrok-cli prints nothing and
  exits 0, as libsigrok finds no device. `zeroplus-logic-cube` takes
  `conn=` without listing it.
- `-c a=1:b=2` sets the options in the order given. sigrok-cli sets them
  in its hash table's order, which shows on a device where setting an
  option talks to the hardware (the Kingst PWM outputs).
- kingst-la2016 captures stop at `--samples`, and a rate whose divider
  does not fit 16 bits is refused.
- `dreamsourcelab-dslogic`: `--get clock_edge` answers the edge set;
  sigrok-cli always prints `rising`.
- `dreamsourcelab-dslogic`: on the first DSLogic, the threshold after
  open reads `0.7-1.4`, the bitstream loaded; sigrok-cli reports
  `1.4-3.6`.
- `dreamsourcelab-dslogic`: `voltage_threshold` outside 0 to 5 V is
  refused; sigrok-cli writes an out of range DAC value and exits 0.
- `dreamsourcelab-dslogic`: a trigger position above about 10 % of a 256
  Mibit board's memory is honoured up to 90 %; sigrok-cli caps it at
  about 10 %.
- `dreamsourcelab-dslogic`: a STALLed or short trigger packet logs
  "Failed to read the trigger position" and ends the capture; sigrok-cli
  hangs.
- `dreamsourcelab-dslogic`: a trigger of more than one stage, `0=rf`
  included, is refused ("Only 1 trigger stages are supported.");
  sigrok-cli writes every stage to stage 0 and exits 0.
- `dreamsourcelab-dslogic`: a trigger on the capture's last sample works
  with `-O hex`; sigrok-cli dies of SIGSEGV.
- `dreamsourcelab-dslogic`: a transfer cut inside an atom gives 0 for
  the bits it lacks; sigrok-cli reads what the buffer held before.
- hantek-4032l: `-c limit_samples=N` with N of 2^32 or more is refused,
  where sigrok-cli takes the lower 32 bits.
- hantek-4032l: a failed configure or get command ends the capture with
  a logged error, where sigrok-cli runs until killed.
- hantek-4032l: a device gone during the data logs one line, where
  sigrok-cli logs one per transfer in flight.
- hantek-4032l: two trigger stages are refused with "Only 1 trigger
  stages are supported." where sigrok-cli says "Only one trigger stage
  supported for now."
- hantek-4032l: a Ctrl-C ends the capture.
- `sysclk-lwla`: a `conn=` that matches nothing or is invalid lists no
  device, where sigrok-cli lists every LWLA.
- `sysclk-lwla`: a read that fails during a capture still passes on the
  samples decoded so far, where sigrok-cli drops those not yet in a full
  packet.
- `sysclk-lwla`: a rate the divider cannot reach logs a warning, which
  fails under `--strict`.
- `sysclk-lwla`: Ctrl-C ends the capture with the device's whole stop
  sequence and passes on the samples decoded so far.
- zeroplus-logic-cube: `conn=` is accepted, not listed. `--show` lists
  no scan option, as sigrok-cli. The device line names `conn=` only when
  it was given or more than one board is found.
- zeroplus-logic-cube: a samplerate not listed fails with "Unsupported
  samplerate", where sigrok-cli crashes or takes it.
- zeroplus-logic-cube: a failed or short read ends the capture rather
  than repeating samples.
- zeroplus-logic-cube: warnings for a second edge, and for a channel
  asked high and low.
- zeroplus-logic-cube: Ctrl-C stops a capture, which sigrok-cli cannot.
- A driver that ends each capture itself and has no limit keys
  (lecroy-logicstudio) starts a capture with -O, -o or -t alone;
  sigrok-cli prints its help there and cannot capture with this driver
  at all. --samples, --time, --frames and --continuous are refused
  alike.
- `lecroy-logicstudio` without a trigger refuses the capture, leaving
  the output file alone; libsigrok would wait for ever.
- hantek-dso: `--get num_hdiv` and `--get num_vdiv` print 10 and 8;
  sigrok-cli fails.
- hantek-dso: a frame with a trigger offset of 0 is written normally;
  sigrok-cli aborts on a double free.
- hantek-dso: the paths on which sigrok-cli never ends (an empty data
  transfer, an unplug, a DSO-5200's 9 bit frames) end. After an empty
  transfer oxrok reads on: on the model that gives the whole frame; on a
  device where an empty packet means no more, the frame ends with an
  error.
- hantek-dso: `-c` pairs are set as given; the order changes only the
  interim commands, and a capture resends all settings.
- `saleae-logic-pro`: a `conn=` that names no board selects none, where
  libsigrok then selects every Logic Pro (`api.c:219-236`), and the
  firmware goes only to boards the conn names (`api.c:194-212`). A short
  transfer gives only its own samples, where libsigrok converts all 16
  KiB of the buffer, stale bytes included (`protocol.c:906`). The
  messages "Failed to do I2C wake read" and "Failed FPGA register
  read-back" print the byte they check (`protocol.c:341`, `540`).
- `saleae-logic-pro`: the first failed transfer ends the capture, with a
  warning; libsigrok drops that transfer and goes on until its idle
  timeout. Running out of transfers (1 s) and a failed authentication
  also warn where libsigrok is silent at the default log level, so
  `--strict` fails those runs. A firmware file whose last part isn't
  flagged final is a failed upload here, where libsigrok logs "Missing
  final part." and calls it a success (`api.c:111-116`); the board is
  listed either way.
- `saleae-logic-pro --continuous` stops on Ctrl-C, not on any key: oxrok
  doesn't read stdin. sigrok-cli stops at once when stdin is at EOF.
- hantek-6xxx: `--samples N` gives N samples; sigrok-cli passes on the
  whole transfer: 256 for 8, 1,048,576 for 1,000,000.
- hantek-6xxx: a rate, range or coupling the scope refuses fails the
  start; sigrok-cli logs it and goes on, with samples labelled at the
  setting it asked for.
- hantek-6xxx: a rate off the list reads back as the 100 kHz it runs at;
  sigrok-cli says the rate asked for, and its csv time column follows
  it.
- hantek-6xxx: with CH1 disabled, CH2 keeps its name; sigrok-cli calls
  it CH1.
- hantek-6xxx: `coupling=DC` on a DC-only board reads back DC;
  sigrok-cli says AC.
- hantek-6xxx: `--get num_vdiv` answers; sigrok-cli says not applicable.
- hantek-6xxx: `vdiv=1.28V`, `57.6mV` and `17.6mV` work on the ISDS205B;
  sigrok-cli cannot parse them.
- hantek-6xxx: a capture whose transfers come back empty or fail ends,
  where sigrok-cli waits for ever.
- hantek-6xxx: Ctrl-C ends the capture, where sigrok-cli dies.

## Not available yet

- `--protocol-decoder-jsontrace`. Python decoders need the `python`
  build feature; `docs/DECODERS.md`.
- `-T` (transform modules), `--list-supported-wiki`, the `gnuplot` and
  `scale` options of the `csv` output.
- The `csv` input and the other long tail input modules (ChronoVu LA8,
  STF, Trace32, ISF, WAV, LogicPort .lpf).
- The other libsigrok hardware drivers: the USB logic analyzers come
  next, the rest after them (`docs/ROADMAP.md`).
