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
- `-c` checks every option of every `-c` before sending any: its scope,
  its key, its value, that the key can be set in that scope, and what
  the core refuses for every driver (`limit_samples=0`). It then sends
  `enabled=off` first, the others in the order given and `enabled=on`
  last, stopping at the first error, so an output goes on only if every
  setpoint was accepted. In a capture, `enabled=on` waits until the rest
  of the command line was accepted, just before the start. sigrok-cli
  sets them in its hash table's order (session.c:700-701), and each -c
  in turn, so it may have switched an output by the time it refuses a
  later option; this shows on a device where setting an option talks to
  the hardware (Kingst PWM, relays, power supplies). Separate `-c` keep
  sigrok-cli's order except for `enabled`.
- The last `enabled` of a scope counts. A device-wide `enabled=off`
  drops an `enabled=on` given before it on a channel group, where
  sigrok-cli pulses that output. `-c enabled=on -c
  channel_group=R1:enabled=off` (all but R1) keeps its order, so R1 has
  power between the two.
- A failed start leaves the output as it is, as in sigrok-cli and as a
  finished capture does: the state before is not known and the device
  may be gone. oxrok says so on stderr ("The capture failed with the
  output on: it was left on.").
- `channel_group=` is the scope only as the first element of a `-c` or
  `--get`, as in sigrok-cli; later on it is an unknown option. An empty
  first element (`-c :enabled=on`) is refused; sigrok-cli dies on an
  assertion. An empty group name (`channel_group=`, `-g ""`) is refused
  too, where sigrok-cli takes it as device-wide: a shell variable that
  came out empty should not switch every output. This is a decision of
  2026-11-05, for now; `EMPTY_GROUP_IS_DEVICE_WIDE` in `src/cli/parse.rs`
  says how to undo it. `global`, in any case, is device-wide in both.
  Group names match without case on every driver.
- A boolean in `-c` or `-d` must be `true`, `yes`, `on`, `1`, `false`,
  `no`, `off` or `0`, in any case; anything else is refused.
  `sr_parse_boolstring()` takes an empty string as true, matches
  prefixes (`1x`) and takes the rest as false.
- A boolean output option (`-O csv:header=`) takes the same words in any
  case; sigrok-cli takes only `true`, `yes`, `false` and `no` in lower
  case and refuses the rest ("Unable to convert", so also `header=TRUE`,
  `on`, `off`, `1`, `0`). `-O csv:header=` with an empty value is the
  default; sigrok-cli refuses it.
- A time limit whose samples overflow saturates; sigrok-cli's
  multiplication wraps (session.c:61) and gives a limit unrelated to the
  time.
- "Failed to open device" carries the reason ("Failed to open device:
  device is busy."), in `--show` as in a capture; sigrok-cli stops at
  "device.".
- A refused value says why ("Failed to set device option 'samplerate':
  invalid argument: it cannot be 0."); sigrok-cli stops at "invalid
  argument.". The line before it is the same text under oxrok's log
  prefix.
- `-d driver:key` without a value and `--show`: oxrok stops at the error
  before any output; sigrok-cli has printed the driver's functions and
  scan options first, then gives the same error and exit status.
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
- uni-t-dmm: The ids of `uni-t-dmm` also accept no `conn=` and
  `conn=hid...` (through the HID layer, `serialcomm=` too), where
  sigrok-cli finds no device. `conn=VID.PID` of another chip's cable
  goes there too, and off Linux so does a CH9325 cable the HID layer
  sees. A `conn=` that is no USB adds a line naming the `-ser` driver. A
  report count above 7 is cut to 7. The csv and analog outputs match on
  a capture that ends in a USB error, although oxrok's driver sends the
  end packet the C does not.
- testo: A conn= that names nothing finds nothing, where sigrok-cli
  selects every device.
- testo: oxrok completes what sigrok-cli does not: two measurements of
  one unit, another order of measurements (sigrok-cli skips some),
  answers of 65 bytes or more, and captures after a bad prefix or an
  overrun (it asks again).
- testo: oxrok ends what sigrok-cli waits on for ever: a device that
  does not answer, after Testo's suggested 7 s and at least 256 reads;
  failing reads after 256; damaged answers after 16 in a row.
- testo: Answers of 65 bytes or more fail in sigrok-cli only when one
  transfer holds two packets, so it depends on the chip's latency timer.
- testo: The analog and csv outputs print NaN as printf does, `nan` and
  `-nan`.
- `sysclk-sla5032`: `--samples` is honoured: sigrok-cli prints what the
  device's rounded depths hold (1024 samples for a limit of 512).
- `sysclk-sla5032`: `conn=` matching nothing, or a bad spec, lists
  nothing; sigrok-cli lists the device.
- `sysclk-sla5032`: a chunk that ends in the 2 s timeout with data is
  written out; sigrok-cli drops it and prints nothing. Records claiming
  over 2^31 samples give the limit's samples; sigrok-cli prints none.
- `sysclk-sla5032`: warnings "The FPGA's key step failed" and "The
  capture ended early" (a failure, no samples, or a chunk with no end
  marker); exit status stays 0 as with sigrok-cli.
- ikalogic-scanalogic2: `-t` with two stages is refused where sigrok-cli
  captures without a trigger. A serial over 2^31 prints as it is, not
  negative. A sample packet lacking bytes the capture uses ends it with
  an error, where sigrok-cli repeats old samples. `samplerate=2.5M`
  works, where sigrok-cli reads 2. `conn=` is refused by name where
  sigrok-cli's `--scan` finds nothing, silently.
- kecheng-kc-330b: `sample_interval` is in ms (125, 500, 1000, 2000,
  5000, 10000, 60000). sigrok-cli's `--get` prints (1, 8), and its set
  and `--show` die.
- kecheng-kc-330b: `--show` and a plain `--get` send nothing, while
  sigrok-cli's close configures.
- kecheng-kc-330b: A `--set` that really changes a setting sends a
  configure outside memory mode, which clears the meter's log by the C's
  comment. sigrok-cli's `--set` sends nothing. A set to the current
  value sends nothing in either.
- kecheng-kc-330b: A log read clears nothing unless another setting was
  asked for along with it.
- kecheng-kc-330b: oxrok takes `conn=`.
- kecheng-kc-330b: A live capture retries bad answers, 5 in a row at
  most; sigrok-cli hangs.
- kecheng-kc-330b: oxrok's USB timeouts are 100 to 250 ms against the
  C's 5 to 40.
- lascar-el-usb: `--samples N` cuts the output at N samples per channel,
  and the whole memory is still read. sigrok-cli ignores the limit.
- lascar-el-usb: half degrees and half percents are kept (23.5 where
  sigrok-cli says 23).
- lascar-el-usb: a device that stops sending or halts its endpoint ends
  the capture after 30 reads with an error line, exit 0. sigrok-cli
  never ends.
- greatfet: a limit inside a byte of packed sample memory gives exactly
  the limit, where sigrok-cli gives up to 7 more.
- greatfet: a `samplerate` of 2^32 or more is refused, where sigrok-cli
  sends it truncated and captures.
- greatfet: oxrok warns on an answered rate below about 49.8 kHz or over
  204 MHz; sigrok-cli says nothing.
- `--continuous` with stdin at EOF: sigrok-cli stops at once; oxrok runs
  until interrupted.
- korad-kaxxxxp: `--get voltage`, `voltage_target`, `current` or
  `current_limit` with no reply fails, where sigrok-cli prints 0.0 with
  rc=0. In a capture such a reading is no sample and doesn't count;
  sigrok-cli sends 0 and counts it. A capture ends after 5 such readings
  in a row.
- korad-kaxxxxp: oxrok refuses `voltage_target=nan` and
  `current_limit=nan`; sigrok-cli sends `VSET1:  nan` to the supply.
- korad-kaxxxxp: `-0` goes out as `VSET1:00.00` and `ISET1:0.000`;
  sigrok-cli sends `VSET1:-0.00` and `ISET1:-0.000`.
- korad-kaxxxxp: sigrok-cli sends `VSET1:00.00` for `voltage_target=abc`
  and `VSET1:12.00` for `12V`; oxrok refuses both.
- korad-kaxxxxp: a device that leaves ends a korad capture, with exit 0;
  libsigrok repeats the last values until the limit.
- korad-kaxxxxp: each query first drops what is in the input queue,
  which C does not do; it puts nothing on the wire.
- rdtech-tc honours `-C` on every output, where libsigrok sends all six
  channels (protocol.c:266-274).
- rdtech-tc: with only a frame limit, a capture ends after 5 s with a
  reply owed, so a later reply loses frames sigrok-cli would get.
  libsigrok waits for ever. Under `--time` or `--continuous` it waits as
  libsigrok does.
- rdtech-tc: 5 failed poll writes in a row end a capture, where
  libsigrok retries for ever.
- rdtech-tc: a device name that is not UTF-8 shows U+FFFD, where
  sigrok-cli passes the bytes.
- rdtech-tc: `bt/` ports are not available.
- rdtech-um: `-C` selects the channels sent; sigrok-cli's driver sends
  all six, and its analog output prints them.
- rdtech-um: a reply that stays incomplete is dropped after 10 s, and
  bytes with no id at once, and polling goes on, where sigrok-cli stops
  polling for good.
- rdtech-um: bytes ahead of an id are dropped at any read; sigrok-cli
  looks only at the read that brings a reply's second byte, so it can
  end the capture on a failed check.
- rdtech-um: an unplug ends the capture, where sigrok-cli logs a failed
  poll without end.
- rdtech-um: a silent meter ends a capture that only `--frames` could
  end, after 30 s; with `--time` or `--continuous` it is polled on, as
  in sigrok-cli.
- rdtech-um: a meter that stopped reading is no error, as in sigrok-cli:
  a poll waits 1 ms for room.
- All formats: SI-prefixed analog values match sigrok's float rounding
  exactly (1500 V at 4 digits prints 1.5000001 kV).
- rdtech-dps, rdtech-rd: values are read and set in the scale of
  `api.c`'s table on all models, where sigrok-cli gives inf or -nan and
  writes 0 on all but the RD6012P; that table is unproven on some models
  (docs/DEVICES.md). `voltage_target` and `current_limit` round, and a
  value over the model's list or the register, or a negative one, is
  refused; sigrok-cli truncates and wraps. `modbusaddr` over 255 is
  refused. A DPS's panel is unlocked on every way out that reaches the
  driver, where sigrok-cli unlocks only when it exits 0. A capture whose
  only limit is `--samples` ends after 5 s with no good read, exit 0;
  under `--time` and `--continuous` it waits and recovers, and
  sigrok-cli hangs in the first case and recovers in the others. An
  RD6012P scaled write first reads register 20, one more read than
  sigrok-cli.
- All drivers: `-c` sends `range` after `enabled=off` and before the
  other settings.
- `atten-pps3203`: `--continuous` with stdin at EOF differs. sigrok-cli
  stops before its first poll, so it sends no set frame and leaves the
  supply untouched; oxrok does not read stdin, sends the poll and zeroes
  the supply. Run with stdin held open, sigrok-cli polls as well.
- `atten-pps3203`: `--get` and `--show` read the scan's answer;
  sigrok-cli says 0 and off until a capture's first answer.
- `atten-pps3203`: setpoints go to the nearest 0.01 V or 1 mA;
  sigrok-cli truncates, so 3.3 V is 3.29. NaN is refused; sigrok-cli
  sends it as 0.
- `atten-pps3203`: a capture ends after 2 s of silence, and a stop after
  300 ms; sigrok-cli never ends, even when stopped. An extra byte does
  not shift later frames, and an answer no supply could give is no
  reading.
- `atten-pps3203`: oxrok warns on stderr of what a set frame changes,
  and when a setting sent nothing though the device reports otherwise;
  sigrok-cli says nothing. The wire is the same.
- manson-hcs-3xxx: a silent device is no device, and a failed setting,
  after 2 s; sigrok-cli waits for ever. A timed-out `enabled=on` says
  the output may be on. A capture with only a sample limit ends after 5
  s with no reading, where sigrok-cli asks for ever; under `--time` or
  `--continuous` both keep asking and go on when it answers. A reply
  that is not eight digits and a mode is no sample; sigrok-cli takes
  what strtod makes of it, 0 V for garbage. A fragment of a reply is
  dropped after 500 ms; sigrok-cli joins it to the next.
  `voltage_target=nan` is refused and `current_limit=-0` sends
  `CURR000`; sigrok-cli sends `VOLTnan` and `CURR-00`. A `GMAX` of `inf`
  is a limit of 0; sigrok-cli passes `voltage_target=inf` on as
  `VOLTinf`.
- With libserialport 0.1.2 or later, sigrok-cli fails `--show`, `--get`,
  `--set` and captures on `gwinstek-gpd` with "Failed to open device",
  as the driver's scan leaves the port locked; on 0.1.1 it runs them.
  oxrok runs them on any version.
- `gwinstek-gpd` accepts `current_limit=3.2` on the GPD-3303S, the
  maximum it lists; libsigrok refuses it, as `strtof` gives a value
  above the table's double.
- `gwinstek-gpd` honours `-C`; libsigrok sends every channel.
- `gwinstek-gpd` ends a capture on an invalid reading. With only a
  sample limit it also ends one after 2 s with no reply. libsigrok hangs
  under `--samples`, or parses old bytes. Under `--time` or
  `--continuous` both wait for a silent device.
- `gwinstek-gpd`'s `-O csv` warns: four rows a round, with made-up zeros
  (known limit, shared fix pending). libsigrok's csv reads past its
  buffer here.
- `-c` holds back `enabled=on` until the rest was accepted, and sends
  nothing on a parse error. A range error is found at send time, so a
  set point ahead of it has gone out.
- `0x10` and `3.3V` as a value are refused by the shared CLI, where
  sigrok-cli's `strtof` takes them.
- maynuo-m97: a capture asks for its readings, where sigrok-cli at the
  pinned commit sends no request and has no samples: its `--samples`
  never ends, and its `--time` ends with nothing. A capture with only a
  sample limit ends after 5 s with no good read; one under `--time` or
  `--continuous` waits and goes on when the load comes back. Five
  requests in a row that cannot go out end any capture. A
  `voltage_target`, `current_limit` or threshold that is NaN or infinite
  is refused; any other value goes out, as in C, also one outside the
  list. `modbusaddr` over 255 is refused. `PC1` is cleared on every way
  out that reaches a close or a drop, where sigrok-cli's errors leave it
  set.
- `motech-lps-301`: a device that does not answer is no device (a scan)
  or a failed setting after 250 ms, where sigrok-cli waits for as long
  as it stays silent. A capture with only a sample limit ends after 2 s
  with no sample.
- `motech-lps-301`: a reading that is no number sends no sample for that
  round; sigrok-cli sends 0. A reading later than 250 ms is dropped with
  its round, where sigrok-cli waits and takes it.
- `motech-lps-301`: `voltage_target=nan` and `current_limit=nan` are
  refused; `-0`, and what single precision makes -0 of, goes out as 0.
- `motech-lps-301`: `enabled` is a channel group option here (`-g CG1`),
  and `-c` sends `enabled=off` first and `enabled=on` last, as on all
  drivers.
- juntek-jds6600: a silent device ends `--get`, `--set` or the scan
  after 1.2 s; sigrok-cli waits for ever. A frequency reply of fewer
  than two words is no device, where sigrok-cli crashes. A NaN and an
  infinite phase are refused and nothing is sent. -0 goes out as 0, as a
  precaution. An enable the device refused is not remembered.
  `serialcomm=` is refused as on all drivers (the C core drops it and
  finds nothing). The scan's first reply is waited for 100 ms, not 10
  ms; it accepts more devices and refuses none, and that a busy host
  needs it is an inference. A `-nan` reply prints `nan` where sigrok-cli
  prints `-nan`. A serial number that is not UTF-8 goes through lossy
  conversion, where sigrok-cli keeps the raw bytes.
- juntek-jds6600, library clients: every setting is narrowed to single
  precision to match sigrok-cli's `strtof` (session.c:570), though C's
  driver keeps a double (api.c:296, protocol.c:731-734). 12345678.91 Hz
  goes out as `1234567900,0` here and `1234567891,0` through libsigrok,
  and a phase of 1e300 is refused here.
- `zketech-ebd-usb`: `--set` of `current_limit` or `uvc_threshold` sends
  nothing, as in sigrok-cli, and oxrok warns at the close. A capture
  warns when it ends with the load possibly on, including the second
  toggle of a one-frame capture. A NaN is refused, where sigrok-cli
  stores it. `-C` is honoured, where C sends all four channels. An
  unplug ends a capture or scan at once, where sigrok-cli takes some 200
  s. A read error sends what a stop sends and ends. `-C V.D-` is refused
  by both tools, in different words, as the dash reads as a range.
- agilent-dmm: `-O csv` with a display disabled writes every row, where
  libsigrok's csv loses rows because its channel count includes disabled
  channels (`output/csv.c:191`, `713`).
- agilent-dmm: an identity that is empty as a string is no device;
  sigrok-cli segfaults (`api.c:145-146`). A line that fills the buffer
  is dropped and the next read, where C stops reading
  (`protocol.c:188`). A port gone ends the capture; C retries for ever.
- agilent-dmm: a capture that only a sample limit can end stops after 10
  s with nothing new from the meter, with an error. sigrok-cli goes on
  asking.
- agilent-dmm: `-O analog:digits=spec` shows the same digits as `all`,
  as oxrok's analog data has one digit count; libsigrok has a separate
  spec figure for this driver. Read from the code, not run.
- agilent-dmm: a refused log start says `not supported: ...` where
  sigrok-cli logs the same text and `Failed to start session.`
- agilent-dmm: lines with VT, FF, NEL, LS, PS, a CR hidden by a NUL, or
  non-ASCII digits match differently from GLib's regular expressions. No
  device is known to send them.
- `gwinstek-psp` refuses NaN and sends 0 for -0; libsigrok sends both.
- `gwinstek-psp` honours `-C`; libsigrok sends both readings.
- `gwinstek-psp` has no sample from a status that failed, and ends a
  capture when the port fails. With only a sample limit it gives up
  after 5 s of hearing nothing or only part of a status; a whole status
  that does not read, as while the panel sets a limit, does not end it,
  and no samples come until one reads. libsigrok repeats the last
  reading and counts it. Under `--time` both wait.
- `gwinstek-psp` gives up after 3 s on a device that keeps talking;
  libsigrok's scan never ends.
- `gwinstek-psp`: a `--get` after a set answers what was set until the
  next `L`; with the output on, a target set stays as set, as no `L`
  tells it. libsigrok answers the last `L`.
- `icstation-usbrelay`: **a board can be identified once per power
  cycle.** A run after any `--show`, `--get`, `--set` or capture, failed
  ones too, sends a request the board takes as a relay byte, then exits
  1. oxrok warns of it on stderr; sigrok-cli doesn't. See
  `docs/DEVICES.md` for what it switches.
- `icstation-usbrelay`: `--get conn` answers, and `--show` lists its
  value. libsigrok advertises the key and then returns "not applicable"
  (api.c:138-140, 153-154). The scan line stays as sigrok-cli's, with no
  conn: `show.c:314` adds it only when that get succeeds, and here the
  choice is to keep the line, since the user named the only board.
- `icstation-usbrelay`: a library client's second open of one device
  object sends no `51`, where `dev_open` does (api.c:225-226).
- `icstation-usbrelay`: `-c channel_group=R1:enabled=on -c enabled=off`
  sends one byte (`0f`) where sigrok-cli sends two (`0e 0f`), from the
  shared CLI's plan, so R1 doesn't pulse.
- `itech-it8500`: an error leaves the load in local mode; sigrok-cli
  exits with it in remote mode and its panel locked.
- `itech-it8500`: a status of success is no reading (an inference from
  the manuals); sigrok-cli reads 0.128 V from it, and lists a device for
  it in a scan.
- `itech-it8500`: an extra byte shifts nothing, and a scan with a
  27-byte first answer finds the load. In sigrok-cli such a scan finds
  nothing, and in a capture every later answer fails with no sample
  while it polls without waiting.
- `itech-it8500`: a capture of a silent load ends at its time limit, or
  after 5 s with only a sample limit; sigrok-cli waits for ever. Under
  `--continuous` both wait.
- `itech-it8500`: a setpoint that fits no int (`nan`, `inf`, `1e10`) is
  refused and nothing is sent; sigrok-cli sends 0x80000000. A negative
  one goes out in both.
- `itech-it8500`: after a timed-out command oxrok waits for a quiet line
  before the next one; sigrok-cli does not. The wire is the same.
- `itech-it8500`: a `--get` after a change of state logs nothing;
  sigrok-cli logs "session was NULL".
- `itech-it8500`: the barcode at `-l 4` is the 22 bytes; sigrok-cli
  reads on into the heap.
- `itech-it8500`: a library's `value as f32` sends 3299 for 3.3 where
  libsigrok sends 3300, as with Manson. sigrok-cli sends 3299 too.
- arachnid-labs-re-load-pro: a setting with no answer fails outside a
  capture, where sigrok-cli reports success. The firmware answers each
  one every time, so silence means a lost line or a link down, and
  `enabled=off` may have left the current flowing. In a capture a
  setting only writes, as in sigrok-cli.
- arachnid-labs-re-load-pro: an `err` line fails a `-c` or `--get`,
  where sigrok-cli reports success or reads 0, and a value with no
  answer, or an answer cut short with no line end, fails `--get`.
- arachnid-labs-re-load-pro: `--get otp_active` and `--get uvc_active`
  read what has arrived, so they report an `overtemp` or `undervolt`
  already sent; sigrok-cli looks for those lines only in a capture.
- arachnid-labs-re-load-pro: a request reads through other lines for its
  1 s wait, so a reading left by monitoring is not taken for the answer.
- arachnid-labs-re-load-pro: `current_limit=nan` and `uvc_threshold=nan`
  are refused and nothing is sent; sigrok-cli sends `set -2147483648`.
- arachnid-labs-re-load-pro: `--time` ends at the limit with nothing
  arriving, where sigrok-cli waits for the next line, so a capture may
  hold one sample fewer. A `--samples` capture ends after 5 s with no
  reading, and a capture ends on a port gone; sigrok-cli waits for ever
  on both.
- arachnid-labs-re-load-pro: a scan drops the input first, so a late
  monitoring line does not lose the device.
- arachnid-labs-re-load-pro: `-O analog:digits=spec` prints 3 digits,
  not the C's 4: `AnalogData` holds one digit count. No case pins this.
- rohde-schwarz-sme-0x: a read that fails, or a value out of range,
  fails with a message; sigrok-cli prints a value nobody set, or
  nothing, and exits 0. A NaN is refused. An empty reply and a reply
  that is not finite or in hexadecimal are errors, and so is a `POW?`
  reply beyond 1000. After a query with no good answer on a serial port,
  the next waits up to 2 s for the line to fall silent.
  `output_frequency=3000000000` is accepted on an SME06.
- rohde-schwarz-sme-0x: settings stay double precision: 123456789 goes
  out as 123456789.0 and 2999999999 as 2999999999.0, where sigrok-cli's
  `strtof` sends 123456792.0 and 3000000000.0. 3000000001 is refused on
  an SME03, where `strtof` rounds it into range.
- rohde-schwarz-sme-0x: with no `conn=` only USBTMC devices are probed;
  sigrok-cli also opens USB serial ports from `scpi_serial.c`'s table of
  ids, and sets their line, which oxrok does not with `conn=` either.
- `appa-55ii`: a capture with a time limit on a silent device ends at
  the limit, one whose port goes away ends with an error, and a Live
  capture under `--samples` alone ends with an error after 10 s with no
  byte; sigrok-cli waits for ever on all three. Warnings, on stderr
  only, say when a log's records were dropped, when it ended short and
  when it had no metadata. `--samples` in `data_source=Memory` can give
  more than asked, as in sigrok-cli: a read's records all go out.
- `mastech-ms6514`: a scan finds the device, where stock sigrok-cli
  never does (`api.c:85`, `serial.c:833`). It listens for 1 s.
  Comparisons are against C with that argument patched.
- `mastech-ms6514`: `-C` picks by the channel a reading names, where C
  goes by display, so its `-C T1` with T2 on main gives T2's reading.
- `mastech-ms6514`: a time limit ends at its time, also on a silent
  unit, where C waits for bytes. A hung-up port ends the capture, where
  C hangs.
- `mastech-ms6514`: a second capture on an open port drops what is
  queued.
- `mastech-ms6514`: a warning with `-O csv` (a known limit: rows by
  count can mix frames), and one when a dump ends before any stored
  value.
- `mastech-ms6514`: `-C T1-T2` is refused by both tools, as a range, in
  their own words.
- `pce-322a`: a get after a `-c` on `spl_weight_freq`, `spl_weight_time`
  or `spl_meas_range` answers what was set; a second `-c` counts from
  it, so fewer toggles may go out. This is a choice, not a C bug. A
  first live reading that contradicts a setting warns.
- `pce-322a`: `--samples` cuts a memory capture's output, with the same
  requests on the wire. A status ending in 0x10 is read, with a warning.
  A silent meter ends a `--samples` or memory capture after 5 s, and a
  port that is gone ends a capture at once.
- `devantech-eth008` works on a tty, where sigrok-cli opens read only
  (`api.c:105`) and finds nothing. oxrok sets 9600/8n1 there and takes
  no `serialcomm`, so a user can't change the line; sigrok-cli leaves it
  as it finds it.
- `devantech-eth008` ends a request after 5 s on TCP and 1 s on a tty.
  Over TCP sigrok-cli's `recv` never gives up, so a slow board works
  there at any latency and a silent one hangs it.
- `devantech-eth008`: `-g AIn --show` prints the value; sigrok-cli exits
  on a GLib critical. `--get voltage` is the same on both.
- `devantech-eth008` drops a byte too many before the next request;
  sigrok-cli reads every later answer one byte along.
- `devantech-eth008` refuses `enabled=perhaps`, and `serialcomm=` by
  name with exit 1; sigrok-cli turns all relays off, and logs the option
  and finds nothing with exit 0.
- `conn=tcp-raw/HOST/PORT` works for any serial driver, numeric port
  only. A read keeps to its timeout, so `korad-kaxxxxp`'s scan finishes
  where sigrok-cli was still running at 40 s; a peer that closes or
  resets reports "device disconnected"; a write has a 5 s bound on room;
  a drain succeeds, where C's `SR_ERR_NA` fails openbench-logic-sniffer
  and raspberrypi-pico; a connect has 5 s per address, and name lookup
  has no bound.
- `uni-t-ut32x`: `-C` limits what is output, where sigrok-cli prints
  every channel the meter sends. A sample limit still counts all
  records, so with the meter on another channel there is no output, no
  message and exit 0. With 1 of 3 records on the channel it says `Device
  only sent 1 samples`, though the device sent 3.
- `uni-t-ut32x`: `--time` and `--samples` end on a silent device, where
  sigrok-cli waits for ever. `conn=hid/ch9325` and `conn=hid` open the
  cable, where sigrok-cli cannot. `conn=1a86.e008` is an oxrok
  extension, meaning the chip and taking the first cable the search
  finds.
- uni-t-ut181a: `range=auto` sends a range of 0, where sigrok-cli sends
  a mode of 0, which the meter is not known to take.
  `measured_quantity=` works, where sigrok-cli crashes. A range or mode
  set that gets `ER` or no answer fails, and `--get range` after a set
  says what the meter took, auto after a mode.
- uni-t-ut181a: a chunk after a recording's first comes from that
  recording, not recording 1. A download ends on `ER`, an empty chunk or
  5 s of silence, where sigrok-cli waits or asks for ever; it exits 0
  with the message on stderr.
- uni-t-ut181a: a time limit ends a silent meter, and a live `--time`
  does not wait for one more reading. A first reading may take 2 s,
  where sigrok-cli finds no device after 100 ms. A live capture with
  only a sample or frame limit ends after 10 s with nothing read. An
  unplug ends a capture.
- uni-t-ut181a: a recording's name that is not UTF-8 shows U+FFFD, where
  sigrok-cli exits. A recording's sample interval comes as a META line
  after the header, which sigrok-cli drops. A start's failure says
  "timeout" or "the meter refused the request", where sigrok-cli says it
  could not start.
- uni-t-ut181a: `-O csv` warns: channels with no data print 0,
  indistinguishable from a reading (docs/ROADMAP.md), and with `-C
  P1,P2` the frame separator falls between rows, where sigrok-cli has it
  ahead of the header and reads past the packet for channels with no
  data.
- All serial drivers: `conn=bt/...` is refused, naming `/dev/rfcommN`,
  where sigrok-cli opens the link itself.
- atorch: `-C` selects the channels sent; sigrok-cli's driver sends all
  of them, and the analog output prints them.
- atorch: all the messages of a read are taken at once. sigrok-cli's
  driver looks only after a read that brought bytes, so a second report
  in a read waits for the next byte. `--frames 2` with two reports in
  one read and nothing after hangs there, and `--time 1500` with two
  reports per read gives 2 frames against 3 here.
- atorch: each capture starts empty. sigrok-cli can give a report left
  from the scan as the first frame. A second capture on an open device
  (library only) drops what the port gathered.
- atorch: a time limit with no report ends 3 s past the limit with a
  warning, and `--frames` alone ends after 30 s of silence with an
  error; sigrok-cli waits with no end (runs passed 120 s). While reports
  come, the first past a time limit is the last, as in sigrok-cli.
- atorch: an unplug ends the capture; sigrok-cli never ends. A real read
  error ends it too, where C dies of a SIGSEGV (EIO injected on a pty).
  The limit is the end: C's callback goes on after its stop, with frames
  and a repeated End on a pty, which sigrok-cli hides and a library
  client would see.
- atorch: "Unrecognized device type" names the report's type; sigrok-cli
  names the fourth byte of its buffer, another byte with noise ahead.
- `-P` maps a channel name to the logic channel's bit, whatever the
  device's list order. An analog channel's name is refused ("Channel
  'DSO' is analog: decoder 'uart' needs a logic channel for 'rx'."),
  where sigrok-cli reads the logic channel of the same index.
- `link-mso19`: a setting during a capture is refused as busy, where C
  takes it. `-t DSO=1` fails with exit 1, where sigrok-cli logs the
  refusal and exits 0 having captured nothing; this holds for a level on
  any analog channel. `-c triggerlevel=x` and a NaN position are
  refused, where sigrok-cli takes them. `-c probe_factor=` above 65535
  and `samplerate=4294967396` are refused. Mixed csv has the right
  channel in each column.
- `link-mso19`: floats stay double: `offset=0.6` sends DAC word 333
  where sigrok-cli's single precision sends 332, 60 mV off at a 10x
  probe, and `--get offset` after 3.3 reads 3.2999999999999998 where
  sigrok-cli reads 3.2999999523162842. It costs a step at 52 of 399
  tenths from 0.1 to 39.9 V.
- norma-dmm, siemens-b102x: hex letters and flag pairs read as their
  values, so other models, units and flags than sigrok-cli on such
  lines. A line with a 21st hex digit and a line of 255 bytes with no LF
  are errors. Part of a line is dropped when its request expires. After
  an expiry requests go less often than in sigrok-cli: 2, 4, then 8 s
  while nothing is heard, and a late reply asks nothing until the others
  are answered. A silent `--samples` warns once after 30 s, and replies
  ended by CR alone warn once.
- Under `flow=1`, `rts=` is not applied by any driver, as in libsigrok's
  use of libserialport.
- fluke-dmm: a 199B's readings carry AC, DC and diode (`1.5 V DC`),
  where sigrok-cli prints `1.5 V`: `fluke-190.c:171` sends no flags.
- fluke-dmm: a line of 255 bytes or more with no CR is dropped and the
  capture goes on, where sigrok-cli stops reading for good.
- fluke-dmm: a port that is gone ends a capture, with exit 0, where
  sigrok-cli retries each second for ever.
- fluke-dmm: a 28x line of two fields gives no sample, where sigrok-cli
  reads past its array and crashes depending on the heap
  (`fluke-28x.c:196`).
- fluke-dmm: a write that finds no room logs a warning, where sigrok-cli
  says nothing.

## Not available yet

- `--protocol-decoder-jsontrace`. Python decoders need the `python`
  build feature; `docs/DECODERS.md`.
- `-T` (transform modules), `--list-supported-wiki`, the `gnuplot` and
  `scale` options of the `csv` output.
- The `csv` input and the other long tail input modules (ChronoVu LA8,
  STF, Trace32, ISF, WAV, LogicPort .lpf).
- The other libsigrok hardware drivers: the rest of the serial
  instruments come next, the rest after them (`docs/ROADMAP.md`).
