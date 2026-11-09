# The MCP server

`oxrok mcp` serves the [Model Context Protocol](https://modelcontextprotocol.io)
over stdio, so an LLM agent (Claude Code, Claude Desktop, Cursor, Zed,
any client that spawns a process) can ask what oxrok supports and run
the command line, and, as the server grows, scan, capture, measure and
decode through the same stack the CLI uses. [PLAN.mcp.md](../PLAN.mcp.md)
has the design and the phases; this page is what exists.

What is served today:

- the reference tools: `version`, `list_drivers`, `driver_info`,
  `list_decoders`, `decoder_info`, `list_formats`, `firmware_status`;
- `cli`, which runs the oxrok command line with sigrok-cli arguments;
- the device tools: `scan`, `open`, `close`, `describe_device`,
  `get_option`, `set_option`, `enable_channels`;
- the capture tools: `capture` (synchronous, with a limit and a
  timeout), `capture_start`/`capture_status`/`capture_stop` (a run the
  agent follows, open-ended without a limit), `list_captures`,
  `drop_capture`, `capture_summary`, `load_capture`, `export_capture`;
- the data tools over a capture in the store: `capture_logic`
  (transitions, runs or samples, paged), `capture_analog` (buckets),
  `measure`, `find`, and `capture_view`, a window of the capture as a
  PNG (logic lanes, analog traces, the time axis, the trigger and an
  analyzer's frames) for an agent that sees, or as the window's edges
  merged in sample order for one that reads;
- the analyzer tools: a decoder stack over a capture whose output is
  kept, so the agent decodes once and then pages, searches and counts:
  `add_analyzer`, `update_analyzer`, `remove_analyzer`,
  `list_analyzers`, `analyzer_status`, `analyzer_frames`,
  `analyzer_summary`, `analyzer_find`, `analyzer_binary`,
  `export_binary`, `analyzer_meta`, and the one-shot `capture_decode`.

Every acquisition runs on a thread of its own and feeds its capture in
the store packet by packet. `capture` waits for it; `capture_start`
returns at once, and the capture grows while the agent does other
things: `capture_status {wait}` follows it with progress
notifications, `capture_summary`, `measure`, `find` and `capture_logic`
answer on what has come so far, `add_analyzer` decodes it as it comes,
and `capture_stop` ends it and frees the device. Without a limit a
started capture runs until it is stopped, or until it alone outgrows
the memory budget (`--max-memory`), which stops it and says so in
`stopped`; finished captures are evicted first, oldest first, and a
running one never is. The budget covers the analyzers' frames too: an
analyzer that fills what is left ends in an error that says so.

The workflow the instructions give the model: `scan` → `open` →
`describe_device` → `set_option` / `enable_channels` → `capture` →
`measure`, `find`, `capture_logic` on the `capture_id` → `export_capture`.
A capture lives in the server's memory, under a budget; the agent reads
statistics and transitions from it, never a megabyte of bits. When the
question is about timing between channels, or the shape of an analog
trace, `capture_view` draws a window of the capture as a picture
(`image/png` content, which a multimodal model looks at) with the
decoded frames of an analyzer under the signals; the agent narrows the
window around a `find` result or a frame, since a whole long capture in
one picture shows only density. The `text` form gives the edges in the
window merged in sample order, exact where a picture is approximate,
with the lanes drawn in characters beside them.

Decoding: `add_analyzer {capture_id, stack: "i2c:scl=D0:sda=D1,eeprom24xx"}`
(or `capture_decode` for a one-shot answer) → `analyzer_status {wait}` →
`analyzer_summary` (counts per row and class: "were there NACKs?") →
`analyzer_frames` (paged, filtered by row, class, range, text or value)
→ `analyzer_find` (the next frame after a sample). A frame carries the
decoder's text and, where the decoder has binary output, the raw
`value` and its `bits`, so an agent compares numbers rather than
parsing `0x50` back out of a string. The radix (`native`, `hex`, `dec`,
`bin`, `ascii`) is chosen per request where the decoder gives raw
values; a decoder that renders the radix itself through its `format`
option takes it from `update_analyzer`, which decodes again. The frame
store and the radix rules are oxrok-ui's, moved into
`oxrok::decode`, so both front ends agree on the text of a frame.

## Configuring a client

Claude Code:

```sh
claude mcp add oxrok -- oxrok mcp
```

Claude Desktop, Cursor and the others take the same JSON, in their
`mcpServers` configuration file:

```json
{ "mcpServers": { "oxrok": { "command": "oxrok", "args": ["mcp"] } } }
```

Use the full path to `oxrok` when the client does not inherit your
`PATH`. Flags go after `mcp`: `"args": ["mcp", "--no-cli"]`.

The MCP Inspector checks a server by hand:

```sh
npx @modelcontextprotocol/inspector oxrok mcp
```

## Flags

```
oxrok mcp [--read-only] [--no-hardware] [--no-cli] [--tools a,b,c]
          [--drivers demo,fx2lafw] [--allow-path DIR]... [--max-memory 512M]
          [--timeout 60s] [--list-tools [--json|--markdown]]
          [--instructions FILE] [-l LEVEL]
```

- `--read-only` keeps only the tools whose `readOnlyHint` is true: the
  reference tools, `scan`, `describe_device`, `get_option`, and the
  tools that read a capture or an analyzer (`capture_status`,
  `capture_summary`, `capture_logic`, `capture_analog`, `measure`,
  `find`, `list_captures`, `list_analyzers`, `analyzer_status`,
  `analyzer_frames`, `analyzer_summary`, `analyzer_find`,
  `analyzer_binary`, `analyzer_meta`). It drops `cli`, `open`,
  `set_option`, `capture`, `capture_start`, `capture_stop`, `close`,
  `enable_channels`, `load_capture`, `export_capture`, `drop_capture`,
  `add_analyzer`, `update_analyzer`, `remove_analyzer`,
  `export_binary` and `capture_decode`.
- `--no-hardware` drops the tools tagged `hardware`: `open`,
  `set_option`, `capture` and `capture_start`. Scanning stays: it finds
  devices without opening them. `cli` is not tagged, since it can do anything the
  command line can; drop it with `--no-cli` or `--read-only`.
- `--no-cli` drops the `cli` pass-through.
- `--tools` serves exactly the tools named, comma separated, and fails
  on a name the server does not have.
- `--drivers` lets only these drivers be scanned and opened; the
  reference tools still describe every driver.
- `--allow-path DIR`, repeatable, is where `load_capture` reads and
  `export_capture` writes; the working directory when not given.
- `--max-memory` is what the captures may take before the oldest are
  dropped (`512M` by default); a result says which were evicted.
- `--timeout` is how long `cli` waits for a command and how long a
  `capture` may run (seconds, or with a unit: `30s`, `2m`); a call can
  ask for less. A capture stopped at the timeout keeps what came and
  says `timed_out`.
- `--list-tools` prints the catalog the other flags leave and exits:
  one line per tool, `--json` for the `tools/list` result, `--markdown`
  for the reference below.
- `--instructions FILE` replaces the text sent to the model at
  `initialize` (the vocabulary and the workflow) with the file's.
- `-l` is the log level, as the command line's; logs go to stderr,
  stdout belongs to the protocol.

The filters compose: `--no-cli --tools version,cli` fails, because `cli`
is gone before `--tools` looks for it. A tool dropped by a flag is
"unknown" to the client, not "disabled".

## What the agent can do

Nothing beyond what the flags leave. The reference tools read the
build's own tables and the firmware directories; they touch no device.
`scan` finds devices without opening them (it does probe serial ports
when no driver is given). `open` claims a device and may upload its
firmware; `set_option` changes a setting, which on a supply or a load
is a live output; `capture` and `capture_start` run an acquisition.
`cli` runs the `oxrok`
executable as a subprocess with the arguments the model gives, which is
everything the command line can do; it refuses `-d` on a driver with a
device open on the server, so a device never has two owners. Its output
is cut at a limit (1 MiB per stream by default, `max_bytes` per call)
and the command is killed at the timeout. Files are read and written
only under `--allow-path`. Serve `--no-cli` or `--read-only` to an agent
that should only answer questions, `--no-hardware` to one that may read
captures but not touch the bench.

Open devices are closed when the client disconnects (EOF on stdin), on
Ctrl-C, and when `close` is called; a capture holds its device until it
ends or `capture_stop` ends it, and a call on that handle meanwhile gets
"busy". Running captures are stopped when the client disconnects.

Every tool carries MCP's annotations (`readOnlyHint`,
`destructiveHint`, `idempotentHint`, `openWorldHint`), so a client can
ask before the ones that act. `cli`, `open`, `set_option`, `capture`
and `capture_start` are destructive; `export_capture` and
`export_binary` write files; the rest change at most the server's own
state.

## Protocol

JSON-RPC 2.0, one message per line, over stdin and stdout; protocol
revisions 2025-06-18, 2025-03-26 and 2024-11-05 (a client asking for
another gets the newest). `initialize`, `ping`, `tools/list`,
`tools/call`, `logging/setLevel`, `notifications/cancelled` (a running
tool sees it and may return early), `notifications/progress` when a
call carries a `progressToken`, and `notifications/tools/list_changed`
when the catalog is swapped. A tool's result is `structuredContent`
plus a text rendering of it; a tool that ran and failed (an unknown
driver, a command that exited 1) answers with `isError`, so the model
reads the reason; arguments that do not match the schema are a JSON-RPC
`-32602`.

`ping` and cancellation are answered while a tool runs: the connection
has a reader thread and a handler thread.

## As a library

`oxrok::mcp`, behind the `mcp` feature (on with `cli`): `Tool`,
`Catalog`, `Host` (the registries, the limits, the open devices by
handle, the `CaptureStore` and the analyzers), `Capture` (the data and
the statistics over it) and `Server`. `oxrok::decode::{Analyzer,
FrameStore, Radix}` are the decoder-stack runner (over a complete
capture or a `Source::Live` one that is still growing), the frame store
and the radix rules, usable without the MCP. `oxrok::mcp::tools::catalog(&host)` is the
library's catalog; `Catalog::select`, `only`, `without`, `read_only`,
`merge` and `prefixed` narrow or extend it; `Server::serve(reader,
writer)` runs the protocol over any pair of streams. A tool is a
`serde`/`schemars` argument struct and a function
(`oxrok::mcp::function`), so the schema a client sees and the parser
that reads a call cannot drift.

## Tools

Generated by `oxrok mcp --list-tools --markdown`; `tests/mcp.rs` fails
when this section differs from it.

<!-- tools:begin -->
### `version`

Version. Tags: catalog. Read-only.

The oxrok version, the features built in (USB backends, the Python decoder runtime), where Python decoders are looked for, and how many drivers and decoders this build has. Call it first.

No arguments.

Example: `{}`

### `list_drivers`

List drivers. Tags: catalog. Read-only.

The device drivers: id (what `-d` takes), name, classes, scan option keys, and whether the driver needs firmware files. Filter by `class` or `query`; `driver_info` has the details of one.

Arguments:

- `class` (string): Keep the drivers of one class: `logic-analyzer`, `oscilloscope`, `multimeter`, `power-supply`, `electronic-load`, `lcr-meter`, `signal-generator`, `demo-device`, ... as `list_drivers` reports them.
- `query` (string): Keep the drivers whose id or name contains this, case-insensitive.

Example: `{"class":"logic-analyzer"}`

### `driver_info`

Driver info. Tags: catalog. Read-only.

One driver: classes, scan options with their value kinds (what `-d driver:key=value` accepts), the firmware files it needs with where each was found, and whether its captures end on their own.

Arguments:

- `driver` (string, required): The driver id, as `-d` takes it: `fx2lafw`, `demo`.

Example: `{"driver":"demo"}`

### `list_decoders`

List decoders. Tags: catalog. Read-only.

The protocol decoders: id (what `-P` takes), name, description, tags, what they take as input (`logic` or another decoder's output) and whether the implementation is native or Python. Filter by `tag` or `query`; `decoder_info` has the channels and options of one.

Arguments:

- `query` (string): Keep the decoders whose id, name or description contains this, case-insensitive.
- `tag` (string): Keep the decoders with this tag, case-insensitive: `Embedded/industrial`, `Memory`, `Util`, ...

Example: `{"query":"i2c"}`

### `decoder_info`

Decoder info. Tags: catalog. Read-only.

One decoder: required and optional channels (the roles `-P dec:role=D0` assigns), options with defaults and allowed values, annotation classes and rows (what `-A` selects), binary classes (`-B`), inputs, outputs, tags and the documentation.

Arguments:

- `decoder` (string, required): The decoder id, as `-P` takes it: `i2c`, `uart`, `eeprom24xx`.

Example: `{"decoder":"i2c"}`

### `list_formats`

List formats. Tags: catalog. Read-only.

The input modules (`-I`, for reading files) and output modules (`-O`, for writing captures: `bits`, `csv`, `vcd`, `srzip`, ...) with their options and file extensions.

No arguments.

Example: `{}`

### `firmware_status`

Firmware status. Tags: catalog. Read-only.

The firmware and bitstream files the drivers need, where each was found or that it was not, where to get the missing ones, and the directories searched in order.

No arguments.

Example: `{}`

### `cli`

Run the command line. Tags: cli. Acts on hardware or runs commands.

Runs the oxrok command line, which takes sigrok-cli's arguments, and returns its stdout, stderr and exit code. Useful flags: `-d DRIVER` (`--scan` to find devices, `--show` for a device's options), `--samples N` or `--time MS`, `-c samplerate=1m`, `-C D0,D1`, `-t D0=r`, `-O bits|csv|vcd|srzip`, `-o FILE`, `-i FILE` with `-I FORMAT`, `-P i2c:scl=D0:sda=D1` with `-A i2c=address` for a decoder, `-L` for everything supported. Keep outputs small: a sample limit and a decoder beat raw samples. The `demo` driver needs no hardware. Output beyond the limit is cut and flagged.

Arguments:

- `args` (array of string, required): The arguments as sigrok-cli takes them, one per element: `["-d", "demo", "--samples", "100", "-O", "bits"]`.
- `max_bytes` (integer): Keep at most this many bytes of stdout and of stderr each. The server's limit otherwise.
- `timeout` (number): Give up after this many seconds and kill the command. The server's limit otherwise.

Example: `{"args":["-d","demo:logic_channels=2","--samples","8","-O","bits"]}`

### `scan`

Scan for devices. Tags: device. Read-only.

Finds devices: with a driver spec (`demo`, `fx2lafw`, `demo:logic_channels=4`), or every driver when none is given. Returns each device's `device_id` for `open`, its vendor, model, version, serial, connection, classes and channel names, and the handle when it is open already. Finding a device does not open it.

Arguments:

- `driver` (string): A driver spec as `-d` takes it, with scan options: `demo:logic_channels=4`, `fx2lafw`, `rigol-ds:conn=tcp-raw/10.0.0.5/5555`. Every driver when left out, which probes serial ports and takes a while.

Example: `{"driver":"demo:logic_channels=4:analog_channels=1"}`

### `open`

Open a device. Tags: device, hardware. Acts on hardware or runs commands.

Opens a device found by `scan` (by `device_id`) or the one device a `driver` spec finds now, and returns its `handle` with everything `describe_device` reports. Open devices are closed when the server exits. Opening may upload firmware and takes the device from other programs.

Arguments:

- `device_id` (string): A `device_id` from the last `scan`.
- `driver` (string): Or a driver spec to scan now, which must find one device.

Example: `{"driver":"demo:logic_channels=4"}`

### `close`

Close a device. Tags: device. Changes the server's state.

Closes an open device and frees its handle. Captures made from it stay in the store.

Arguments:

- `handle` (integer, required): The handle `open` returned.

Example: `{"handle":1}`

### `describe_device`

Describe a device. Tags: device. Read-only.

An open device: identity, classes, channels (index, name, kind, enabled), channel groups, and every option with its access (get, set, list), current value and allowed values or range, in the device's scope and in each group's. What `--show` prints, as data.

Arguments:

- `group` (string): Only the options of this channel group; every scope otherwise.
- `handle` (integer, required): The handle `open` returned.

Example: `{"handle":1}`

### `get_option`

Get an option. Tags: device. Read-only.

Reads one option of an open device: its value as a number, string or boolean, and as sigrok's text.

Arguments:

- `group` (string): The channel group, for options a group has; `global` or left out for the device's own.
- `handle` (integer, required): The handle `open` returned.
- `key` (string, required): The option key: `samplerate`, `voltage_target`, `pattern`, ...

Example: `{"handle":1,"key":"samplerate"}`

### `set_option`

Set an option. Tags: device, hardware. Acts on hardware or runs commands.

Sets one option of an open device, with the checks `-c` makes: the key must exist in the scope and be settable, the value must parse and be allowed. Returns the value read back. On a supply or a load this changes a live output: `enabled=on` switches it on.

Arguments:

- `group` (string): The channel group, for options a group has.
- `handle` (integer, required): The handle `open` returned.
- `key` (string, required): The option key: `samplerate`, `voltage_target`, `enabled`, ...
- `value` (string): The value in sigrok's text syntax: `1m` or `200k` for a rate, `1.5` for a voltage, `on`/`off` for a switch, a name from the option's list. A boolean key with no value is `on`.

Example: `{"handle":1,"key":"samplerate","value":"1m"}`

### `enable_channels`

Enable channels. Tags: device. Changes the server's state.

Enables the channels named and disables the rest, renaming with `name=newname`; what `-C` does. Returns the channel list.

Arguments:

- `channels` (array of string, required): The channels to enable, as `-C` takes them: names, numeric ranges (`0-3`), and `name=newname` to rename. Every other channel is disabled.
- `handle` (integer, required): The handle `open` returned.

Example: `{"channels":["D0","D1=sda"],"handle":1}`

### `capture`

Capture. Tags: capture, hardware. Acts on hardware or runs commands.

Runs one acquisition on an open device and keeps it in the store: applies `config`, sets the limit (`samples`, `time` or `frames`), arms `triggers`, runs until the limit or the timeout, and returns the `capture_id` with the summary (`capture_summary`). The data stays on the server: read it with `capture_logic`, `measure`, `find` and `capture_analog`, or write it with `export_capture`. For a long or an open-ended run use `capture_start`.

Arguments:

- `config` (array of string): Options applied first, as `-c` takes them: `samplerate=1m`, `channel_group=Logic:pattern=walking-one`.
- `frames` (string): How many frames (oscilloscope sweeps).
- `handle` (integer, required): The handle `open` returned.
- `samples` (string): How many samples, with a size suffix: `1k`, `2m`. `capture` needs one of `samples`, `time` or `frames`; `capture_start` without one runs until `capture_stop`.
- `time` (string): How long, in ms or with a unit: `10ms`, `2s`.
- `timeout` (number): Give up after this many seconds, keeping what came. `capture` takes the server's limit otherwise; `capture_start` has none.
- `triggers` (string): A trigger as `-t` takes it: `D0=r` (rising), `D1=f`, `D2=1`, `D0=r,D1=1` for one stage, `;` between stages.
- `wait_trigger` (boolean): Keep only the samples from the trigger on.

Example: `{"config":["samplerate=1m"],"handle":1,"samples":"1k"}`

### `capture_start`

Start a capture. Tags: capture, hardware. Acts on hardware or runs commands.

Starts an acquisition like `capture` and returns at once with the `capture_id`; without a limit it runs until `capture_stop`. The capture grows in the store meanwhile: `capture_status` follows it (`wait` for it to end), and the data tools and `add_analyzer` work on what has come so far. A run that outgrows the memory budget is stopped. The device is busy until the capture ends.

Arguments:

- `config` (array of string): Options applied first, as `-c` takes them: `samplerate=1m`, `channel_group=Logic:pattern=walking-one`.
- `frames` (string): How many frames (oscilloscope sweeps).
- `handle` (integer, required): The handle `open` returned.
- `samples` (string): How many samples, with a size suffix: `1k`, `2m`. `capture` needs one of `samples`, `time` or `frames`; `capture_start` without one runs until `capture_stop`.
- `time` (string): How long, in ms or with a unit: `10ms`, `2s`.
- `timeout` (number): Give up after this many seconds, keeping what came. `capture` takes the server's limit otherwise; `capture_start` has none.
- `triggers` (string): A trigger as `-t` takes it: `D0=r` (rising), `D1=f`, `D2=1`, `D0=r,D1=1` for one stage, `;` between stages.
- `wait_trigger` (boolean): Keep only the samples from the trigger on.

Example: `{"config":["samplerate=1m"],"handle":1}`

### `capture_status`

Capture status. Tags: capture. Read-only.

Whether a capture is still running, its samples, bytes and duration so far, the trigger, and why it ended when it did (`stopped`: `capture_stop`, `timeout`, `memory budget`; `failed` for a device error). With `wait`, waits up to that many seconds for it to end, sending progress meanwhile.

Arguments:

- `capture_id` (integer, required): The id `capture_start` returned.
- `wait` (number): Wait up to this many seconds for the capture to end first, reporting progress meanwhile.

Example: `{"capture_id":1,"wait":5}`

### `capture_stop`

Stop a capture. Tags: capture. Changes the server's state.

Ends a running capture, waits for the device to stop, and returns the summary of what came; the device is free again. A capture that has ended already just gives its summary.

Arguments:

- `capture_id` (integer, required): The id `capture` or `load_capture` returned.

Example: `{"capture_id":1}`

### `list_captures`

List captures. Tags: capture. Read-only.

The captures in the store: id, source, channels, sample count, rate, size and whether it is still running, oldest first, with the memory used by captures and by analyzer frames, and the budget.

No arguments.

Example: `{}`

### `drop_capture`

Drop a capture. Tags: capture. Changes the server's state.

Removes a capture from the store and frees its memory, with the analyzers over it; a running one is stopped first.

Arguments:

- `capture_id` (integer, required): The id `capture` or `load_capture` returned.

Example: `{"capture_id":1}`

### `capture_summary`

Capture summary. Tags: capture. Read-only.

A capture at a glance: rate, sample count, duration, trigger position, and per logic channel its transitions, duty cycle, first and last level and shortest pulse; per analog channel min, max, mean and RMS with the unit. Read this before asking for data. On a running capture, what has come so far.

Arguments:

- `capture_id` (integer, required): The id `capture` or `load_capture` returned.

Example: `{"capture_id":1}`

### `load_capture`

Load a file. Tags: capture. Changes the server's state.

Reads a `.sr` session file, a VCD or raw binary samples into the store, under the server's allowed directories, and returns the `capture_id` with the summary.

Arguments:

- `channels` (array of string): The channels to keep, as `-C` takes them.
- `format` (string): The input module and its options as `-I` takes them (`vcd:downsample=2`, `binary:numchannels=8:samplerate=1m`); detected from the file when left out.
- `path` (string, required): The file: a `.sr` session, a VCD, or raw binary samples.

Example: `{"path":"capture.sr"}`

### `export_capture`

Export a capture. Tags: capture. Acts on hardware or runs commands.

Writes a capture to a file with an output module (`srzip` for a `.sr` PulseView can open, `csv`, `vcd`, `bits`, ...) under the server's allowed directories. Returns the path and size, never the bytes. A running capture is written as it stands.

Arguments:

- `capture_id` (integer, required): The id `capture` or `load_capture` returned.
- `format` (string): The output module and its options as `-O` takes them: `srzip`, `csv`, `vcd`, `bits:width=64`, ... From the extension when left out (`.sr`, `.csv`, `.vcd`), else `srzip`.
- `path` (string, required): The file to write.

Example: `{"capture_id":1,"path":"capture.sr"}`

### `capture_logic`

Logic data. Tags: capture. Read-only.

Logic channels of a capture over a range, as transitions by default: the level at `from` and where it changes, with sample numbers and seconds. `rle` gives runs, `samples` the raw levels (capped). Paged with `offset` and `limit`; `truncated` says when there is more. Prefer `measure` and `find` to reading samples.

Arguments:

- `capture_id` (integer, required): The id `capture` or `load_capture` returned.
- `channels` (array of string): The channel names; every enabled logic channel when left out.
- `form` (string): `transitions` (default): where each channel changes, as `{sample, t, level}`, with the level at `from`; `rle`: runs as `{sample, level, length}`; `samples`: the levels as 0 and 1, which costs a token per sample.
- `from` (integer): The first sample, 0 when left out.
- `from_time` (number): The start in seconds instead of `from`.
- `limit` (integer): At most this many items per channel: 200 by default, 5000 at most (10000 for `samples`).
- `offset` (integer): Skip this many items per channel.
- `to` (integer): One past the last sample, the end when left out.
- `to_time` (number): The end in seconds instead of `to`.

Example: `{"capture_id":1,"channels":["D0"],"limit":50}`

### `capture_analog`

Analog data. Tags: capture. Read-only.

One analog channel of a capture over a range, reduced to buckets with min, max and mean each, so a long trace is a short table. With the unit and what was measured.

Arguments:

- `buckets` (integer): How many buckets to reduce the range to, each with min, max and mean: 100 by default, 2000 at most.
- `capture_id` (integer, required): The id `capture` or `load_capture` returned.
- `channel` (string, required): The analog channel's name.
- `from` (integer): The first sample, 0 when left out.
- `from_time` (number): The start in seconds instead of `from`.
- `to` (integer): One past the last sample, the end when left out.
- `to_time` (number): The end in seconds instead of `to`.

Example: `{"buckets":20,"capture_id":1,"channel":"A0"}`

### `measure`

Measure a channel. Tags: capture. Read-only.

Measurements over a range of one channel. Logic: transitions, rising and falling edges, frequency and period (from rising edges), duty cycle, high and low pulse widths (min, max, mean). Analog: min, max, peak to peak, mean, RMS, and the frequency by crossings of the mean. Seconds where the rate is known.

Arguments:

- `capture_id` (integer, required): The id `capture` or `load_capture` returned.
- `channel` (string, required): The channel's name.
- `from` (integer): The first sample, 0 when left out.
- `from_time` (number): The start in seconds instead of `from`.
- `to` (integer): One past the last sample, the end when left out.
- `to_time` (number): The end in seconds instead of `to`.

Example: `{"capture_id":1,"channel":"D0"}`

### `find`

Find an edge or pulse. Tags: capture. Read-only.

The first edge (any, rising or falling) or the first pulse at least `min_width` long on a logic channel at or after a sample or time: how to walk a trace without reading it. Returns the sample, the time and the level after it, or `found: false`.

Arguments:

- `after` (integer): Look from this sample on; 0 when left out.
- `after_time` (number): Look from this time on, in seconds, instead of `after`.
- `capture_id` (integer, required): The id `capture` or `load_capture` returned.
- `channel` (string, required): The logic channel's name.
- `min_width` (number): With `pulse`: the shortest run wanted, in seconds.
- `min_width_samples` (integer): With `pulse`: the shortest run wanted, in samples.
- `what` (string): `edge` (default), `rising`, `falling`, or `pulse` for the first run (high or low) at least `min_width` long.

Example: `{"capture_id":1,"channel":"D0","what":"rising"}`

### `capture_view`

View a window. Tags: capture. Read-only.

A window of a capture to look at: by default a PNG with the logic channels as lanes, the analog channels as traces with their scale, a time axis, the trigger, and, with `analyzer_id`, the decoded frames as labelled boxes under the signals, so the timing between channels and against the decode is visible at a glance. `form: text` gives the window's edges merged in sample order instead, which is exact where a picture is approximate. Narrow the window (`from`/`to` or `from_time`/`to_time`) around a `find` result or a frame; a whole long capture in one picture shows only density.

Arguments:

- `analyzer_id` (integer): Draw this analyzer's frames under the signals, one lane per row.
- `capture_id` (integer, required): The id `capture` or `load_capture` returned.
- `channels` (array of string): The channels to draw, top to bottom; every enabled one when left out.
- `form` (string): `image` (default): a PNG of the window, logic lanes, analog traces with their scale, a time axis, the trigger and the frames of `analyzer_id`; `text`: the edges in the window merged in sample order, with the lanes drawn in characters.
- `from` (integer): The first sample, 0 when left out.
- `from_time` (number): The start in seconds instead of `from`.
- `max_events` (integer): With `form: text`: at most this many edges (200 by default, 2000 at most).
- `rows` (array of string): With `analyzer_id`: only these rows (`addr-data`); the rows that are not bit rows otherwise.
- `to` (integer): One past the last sample, the end when left out.
- `to_time` (number): The end in seconds instead of `to`.
- `width` (integer): The picture's width in pixels (1024 by default, 200 to 4096), or the text lanes' columns (100 by default, 400 at most).

Example: `{"analyzer_id":1,"capture_id":1,"from_time":0.0,"to_time":0.0002}`

### `add_analyzer`

Add an analyzer. Tags: decode. Changes the server's state.

Runs a decoder stack over a capture and keeps the output: give the `capture_id` and the stack in `-P` syntax (`i2c:scl=D0:sda=D1`, `uart:rx=D0:baudrate=115200`, `i2c:scl=D0:sda=D1,eeprom24xx`) or as `decoders`. Returns the `analyzer_id`, the instances with their rows and how each takes a radix, and the status; the decode runs on its own thread, so follow with `analyzer_status {wait}` and then `analyzer_summary` and `analyzer_frames`.

Arguments:

- `capture_id` (integer, required): The capture to decode.
- `decoders` (array of object): The stack as data, bottom first. Or `stack`.
- `name` (string): A name for the listing.
- `radix` (string): The analyzer's radix: `native`, `hex`, `dec`, `bin`, `ascii`. The decoder's default otherwise (`ascii` for `uart`, else `native`).
- `stack` (string): The stack in `-P` syntax, bottom first: `i2c:scl=D0:sda=D1,eeprom24xx`, `uart:rx=D0:baudrate=115200`. Or `decoders`.

Example: `{"capture_id":1,"stack":"uart:rx=D0:baudrate=115200"}`

### `update_analyzer`

Update an analyzer. Tags: decode. Changes the server's state.

Changes an analyzer's stack, options or radix. Decoders cannot seek, so the stack is rebuilt and decodes from sample 0; the `generation` goes up, and pages taken from the old one are stale.

Arguments:

- `analyzer_id` (integer, required): The analyzer.
- `decoders` (array of object): A new stack as data; the old one otherwise.
- `radix` (string): A new radix.
- `stack` (string): A new stack in `-P` syntax; the old one otherwise.

Example: `{"analyzer_id":1,"radix":"hex"}`

### `remove_analyzer`

Remove an analyzer. Tags: decode. Changes the server's state.

Stops an analyzer, waiting for its thread, and frees its frames.

Arguments:

- `analyzer_id` (integer, required): The analyzer.

Example: `{"analyzer_id":1}`

### `list_analyzers`

List analyzers. Tags: decode. Read-only.

Every analyzer: its capture, stack, radix, status, progress, generation and frame count.

Arguments:

- `capture_id` (integer): Only the analyzers of this capture.

Example: `{}`

### `analyzer_status`

Analyzer status. Tags: decode. Read-only.

Status (`running`, `complete`, `error`), progress, generation and frame count of an analyzer; with `wait`, waits up to that many seconds for the decode to end first.

Arguments:

- `analyzer_id` (integer, required): The analyzer.
- `wait` (number): Wait up to this many seconds for the decode to end.

Example: `{"analyzer_id":1,"wait":10}`

### `analyzer_frames`

Analyzer frames. Tags: decode. Read-only.

The decoded frames, in sample order, filtered by instance, rows, classes, range, text, value or errors, and paged with `offset` and `limit`. Each frame has `ss`/`es` (samples), `t0`/`t1` (seconds), `instance`, `row`, `class`, `text`, and `value`/`bits` when the decoder gave a raw value: compare numbers on `value`, not on the text. `radix` renders the values per request.

Arguments:

- `all_texts` (boolean): Include every text variant the decoder gave, longest first.
- `analyzer_id` (integer, required): The analyzer.
- `classes` (array of string): Only these annotation classes, by id (the names `-A` takes).
- `errors_only` (boolean): Only frames the error heuristic flagged.
- `from` (integer): From this sample.
- `from_time` (number): From this time, in seconds.
- `generation` (integer): The generation the offsets were taken from; refused when the analyzer was rebuilt since.
- `instance` (string): One instance (`i2c-1`); every instance otherwise.
- `limit` (integer): At most this many frames: 100 by default, 2000 at most.
- `offset` (integer): Skip this many matching frames.
- `radix` (string): Render values in this radix; the analyzer's otherwise.
- `rows` (array of string): Only these rows, by id (`rx-data`, `address-write`, ...).
- `text` (string): Only frames whose rendered text contains this, case-insensitive.
- `to` (integer): To this sample, exclusive.
- `to_time` (number): To this time, in seconds.
- `value` (integer): Only frames with this raw value (`80` for I²C address 0x50).

Example: `{"analyzer_id":1,"limit":50,"rows":["rx-data"]}`

### `analyzer_summary`

Analyzer summary. Tags: decode. Read-only.

Per instance, row and class: frame counts, error counts, and the first and last frame. The answer to "were there NACKs?" or "how many bytes?" in one call.

Arguments:

- `analyzer_id` (integer, required): The analyzer.

Example: `{"analyzer_id":1}`

### `analyzer_find`

Find a frame. Tags: decode. Read-only.

The frame of a row at a sample or time, or the next after / the previous before one, optionally the next whose text contains `text` or whose value is `value`: walking frames as `find` walks edges.

Arguments:

- `analyzer_id` (integer, required): The analyzer.
- `at` (integer): The frame containing this sample.
- `at_time` (number): `at`, in seconds.
- `instance` (string): The instance; the one that has `row` otherwise.
- `next_after` (integer): The first frame starting after this sample.
- `next_after_time` (number): `next_after`, in seconds.
- `prev_before` (integer): The last frame ending before this sample.
- `prev_before_time` (number): `prev_before`, in seconds.
- `radix` (string): Render in this radix.
- `row` (string, required): The row to look in, by id.
- `text` (string): With `next_after`/`prev_before`: keep looking until the text contains this.
- `value` (integer): With `next_after`/`prev_before`: keep looking until the raw value is this.

Example: `{"analyzer_id":1,"next_after":0,"row":"address-write"}`

### `analyzer_binary`

Binary output. Tags: decode. Read-only.

One instance's binary output class (what `-B` selects) over a range of samples, as hex, ascii, dec, bin or base64, capped at `max_bytes`; `export_binary` writes it all to a file.

Arguments:

- `analyzer_id` (integer, required): The analyzer.
- `class` (string): The binary class, by id (what `-B dec=class` names); the first otherwise.
- `from` (integer): Items starting from this sample.
- `instance` (string): The instance; the first with binary output otherwise.
- `max_bytes` (integer): At most this many bytes in the answer: 4096 by default, 1 MiB at most. `export_binary` writes them all to a file.
- `radix` (string): `hex` (default), `ascii`, `dec`, `bin` or `base64`.
- `to` (integer): Items starting before this sample.

Example: `{"analyzer_id":1,"class":"rx","radix":"ascii"}`

### `export_binary`

Export binary output. Tags: decode. Acts on hardware or runs commands.

Writes one instance's binary output class over a range of samples to a file under the allowed directories, whole: the bytes a UART carried, an EEPROM's contents. Returns the path and size.

Arguments:

- `analyzer_id` (integer, required): The analyzer.
- `class` (string): The binary class, by id (what `-B dec=class` names); the first otherwise.
- `from` (integer): Items starting from this sample.
- `instance` (string): The instance; the first with binary output otherwise.
- `path` (string, required): The file to write, under the allowed directories.
- `to` (integer): Items starting before this sample.

Example: `{"analyzer_id":1,"class":"rx","path":"rx.bin"}`

### `analyzer_meta`

Meta output. Tags: decode. Read-only.

An instance's meta output (what `-M` selects): named values such as a measured bitrate, with their sample ranges, paged.

Arguments:

- `analyzer_id` (integer, required): The analyzer.
- `instance` (string): The instance; every instance otherwise.
- `limit` (integer): At most this many items: 100 by default.
- `offset` (integer): Skip this many items.

Example: `{"analyzer_id":1}`

### `capture_decode`

Decode a capture. Tags: decode. Changes the server's state.

The one-shot form: adds an analyzer, waits for it, and returns the summary with one page of frames; the analyzer is dropped unless `keep` is true, in which case `analyzer_id` is returned for more questions.

Arguments:

- `capture_id` (integer, required): The capture to decode.
- `decoders` (array of object): The stack as data. Or `stack`.
- `keep` (boolean): Keep the analyzer for more questions; dropped otherwise.
- `limit` (integer): At most this many frames in the page: 100 by default.
- `radix` (string): The radix.
- `rows` (array of string): Only these rows in the page of frames.
- `stack` (string): The stack in `-P` syntax: `i2c:scl=D0:sda=D1`. Or `decoders`.
- `timeout` (number): Wait at most this many seconds for the decode.

Example: `{"capture_id":1,"limit":50,"stack":"i2c:scl=D0:sda=D1"}`
<!-- tools:end -->
