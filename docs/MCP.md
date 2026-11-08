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
  timeout), `list_captures`, `drop_capture`, `capture_summary`,
  `load_capture`, `export_capture`;
- the data tools over a capture in the store: `capture_logic`
  (transitions, runs or samples, paged), `capture_analog` (buckets),
  `measure` and `find`.

Decoders over a capture (analyzers) and the asynchronous capture come
next.

The workflow the instructions give the model: `scan` → `open` →
`describe_device` → `set_option` / `enable_channels` → `capture` →
`measure`, `find`, `capture_logic` on the `capture_id` → `export_capture`.
A capture lives in the server's memory, under a budget; the agent reads
statistics and transitions from it, never a megabyte of bits.

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
  reference tools, `scan`, `describe_device`, `get_option` and the
  tools that read a capture. It drops `cli`, `open`, `set_option`,
  `capture`, `close`, `enable_channels`, `load_capture`,
  `export_capture` and `drop_capture`.
- `--no-hardware` drops the tools tagged `hardware`: `open`,
  `set_option` and `capture`. Scanning stays: it finds devices without
  opening them. `cli` is not tagged, since it can do anything the
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
is a live output; `capture` runs an acquisition. `cli` runs the `oxrok`
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
ends, and a call on that handle meanwhile gets "busy".

Every tool carries MCP's annotations (`readOnlyHint`,
`destructiveHint`, `idempotentHint`, `openWorldHint`), so a client can
ask before the ones that act. `cli`, `open`, `set_option` and `capture`
are destructive; `export_capture` writes a file; the rest change at most
the server's own state.

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
handle and the `CaptureStore`), `Capture` (the data and the statistics
over it) and `Server`. `oxrok::mcp::tools::catalog(&host)` is the
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

Runs one acquisition on an open device and keeps it in the store: applies `config`, sets the limit (`samples`, `time` or `frames`), arms `triggers`, runs until the limit or the timeout, and returns the `capture_id` with the summary (`capture_summary`). The data stays on the server: read it with `capture_logic`, `measure`, `find` and `capture_analog`, or write it with `export_capture`.

Arguments:

- `config` (array of string): Options applied first, as `-c` takes them: `samplerate=1m`, `channel_group=Logic:pattern=walking-one`.
- `frames` (string): How many frames (oscilloscope sweeps).
- `handle` (integer, required): The handle `open` returned.
- `samples` (string): How many samples, with a size suffix: `1k`, `2m`. One of `samples`, `time` or `frames` is needed.
- `time` (string): How long, in ms or with a unit: `10ms`, `2s`.
- `timeout` (number): Give up after this many seconds, keeping what came. The server's limit otherwise.
- `triggers` (string): A trigger as `-t` takes it: `D0=r` (rising), `D1=f`, `D2=1`, `D0=r,D1=1` for one stage, `;` between stages.
- `wait_trigger` (boolean): Keep only the samples from the trigger on.

Example: `{"config":["samplerate=1m"],"handle":1,"samples":"1k"}`

### `list_captures`

List captures. Tags: capture. Read-only.

The captures in the store: id, source, channels, sample count, rate and size, oldest first, with the memory used and the budget.

No arguments.

Example: `{}`

### `drop_capture`

Drop a capture. Tags: capture. Changes the server's state.

Removes a capture from the store and frees its memory.

Arguments:

- `capture_id` (integer, required): The id `capture` or `load_capture` returned.

Example: `{"capture_id":1}`

### `capture_summary`

Capture summary. Tags: capture. Read-only.

A capture at a glance: rate, sample count, duration, trigger position, and per logic channel its transitions, duty cycle, first and last level and shortest pulse; per analog channel min, max, mean and RMS with the unit. Read this before asking for data.

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

Writes a capture to a file with an output module (`srzip` for a `.sr` PulseView can open, `csv`, `vcd`, `bits`, ...) under the server's allowed directories. Returns the path and size, never the bytes.

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
<!-- tools:end -->
