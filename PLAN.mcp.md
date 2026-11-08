# oxrok MCP server: plan

An MCP (Model Context Protocol) server for oxrok, so an LLM agent can
find devices, capture, measure and decode through the same stack the CLI
and the UI use. It ships two ways:

1. **From the command line:** `oxrok mcp`, a stdio server with the
   library's tools, for Claude Code, Claude Desktop, Cursor, Zed and any
   other client that spawns a process.
2. **From the UI:** oxrok-ui serves an *extended* MCP: a chosen subset of
   the library's tools plus tools that act on the UI itself (sessions,
   view, markers, analyzers, screenshots), exposed through a local socket
   and a stdio shim.

This is a plan to react to. Nothing is built yet. Decisions marked
**(open)** are collected at the end.

## What it is for

An agent sitting next to a bench should be able to:

- answer "what can oxrok drive, and what does my `fx2lafw` support?"
  without reading the docs;
- scan, open a device, set a sample rate, capture 10 ms on four channels,
  and get back something it can reason about (edge counts, frequencies,
  a decoded I²C transaction), not a megabyte of bits;
- run a decoder over a capture or a `.sr` file and page through the
  annotations;
- in the UI, do the same against the session the user is looking at, and
  show the user what it found (zoom there, drop a marker, add an
  analyzer).

And a person should be able to say exactly which of those an agent may
do: read-only, no hardware, a named list of tools.

## Principles

- **One tool model, many servers.** A tool is a name, a JSON schema and a
  function. The CLI server and the UI server are two *catalogs* built
  from the same tools. Nothing in a tool knows which server it is in.
- **Tools depend on narrow traits, not on the server's state.** A tool
  that reads a capture takes a `&dyn CaptureSource`; the CLI server's
  in-memory store and the UI's session store both implement it. That is
  what makes "select some of the library's tools for the UI" possible
  without forks.
- **Bounded results.** Everything that returns data has a limit and an
  offset, defaults small, and prefers transitions and statistics over
  raw samples. An LLM context is the scarce resource.
- **sigrok-cli semantics, not a new vocabulary.** Driver ids, keys
  (`samplerate`), value syntax (`1m`, `D0=r`), decoder ids and output
  module names are the ones sigrok users and the models already know.
- **Same rules as the rest of oxrok.** `#![forbid(unsafe_code)]`, few
  dependencies, `cargo deny` clean, replay fixtures so `cargo test` needs
  no hardware. No tokio unless an HTTP transport earns it.
- **Hardware is opt-in, and the dangerous bits are marked.** Setting a
  supply's `voltage_target` is not the same as listing decoders. Every
  tool carries MCP's annotations (`readOnlyHint`, `destructiveHint`) and
  the server has switches to drop whole classes of tools.

## Where it lives

A module `src/mcp/` in the `oxrok` crate behind a feature `mcp`, the way
`cli` and `pydecode` are gated today. The `oxrok` binary gets `mcp` from
its `cli` feature; oxrok-ui enables `oxrok/mcp` and builds its own server
from the module. One crate to version, one place for the tool model.

```
src/mcp/
  mod.rs          the public surface: Server, Catalog, Tool, Host
  jsonrpc.rs      JSON-RPC 2.0 framing over any Read + Write (stdio, sockets)
  protocol.rs     initialize, ping, tools/list, tools/call, notifications
  tool.rs         ToolDesc, Tool trait, ToolResult, ToolError, annotations
  catalog.rs      Catalog: a list of tools with select / without / merge
  host.rs         Host: registry, open devices, the capture store, limits
  source.rs       CaptureSource and DeviceHost, the traits tools depend on
  tools/
    catalog.rs    version, list_drivers, driver_info, list_decoders, ...
    cli.rs        the sigrok-cli pass-through
    device.rs     scan, open, close, describe_device, get_option, set_option
    capture.rs    capture, capture_start/status/stop, list/drop/load/export
    data.rs       capture_logic, capture_analog, measure
    decode.rs     capture_decode
src/cli/mcp.rs    the `oxrok mcp` subcommand: flags → Host → stdio Server
```

A separate `oxrok-mcp` crate was considered and rejected for now: the
tools reach into `core`, `formats`, `decode` and `pydecode`, and oxrok-ui
pins `oxrok` by exact version anyway. If the module grows an HTTP
transport with its own dependency tree, that transport can become a crate
then.

## The tool model

```rust
pub struct ToolDesc {
    pub name: &'static str,            // "scan", "capture_logic"
    pub title: &'static str,
    pub description: &'static str,     // written for the model, with an example
    pub input_schema: serde_json::Value,
    pub output_schema: Option<serde_json::Value>,
    pub annotations: Annotations,      // read_only, destructive, idempotent, open_world
    pub tags: &'static [Tag],          // Catalog, Device, Hardware, Capture, Decode, Cli
}

pub trait Tool: Send + Sync {
    fn desc(&self) -> &ToolDesc;
    fn call(&self, args: serde_json::Value, cx: &mut Call) -> Result<ToolResult, ToolError>;
}
```

`Call` carries what a handler may need from the protocol: a progress
reporter (`cx.progress(done, total, message)`), a cancellation check
(`cx.cancelled()`), and a logger that goes to the client as
`notifications/message`. `ToolResult` is `structured: Value` plus the
`content` blocks MCP wants (text by default, image for screenshots); the
server fills `content` from `structured` when a tool gives only the
latter.

Arguments are `serde::Deserialize` structs with `schemars::JsonSchema`
derived, so the schema a client sees and the parser that reads the call
cannot drift. `schemars` 1.x is MIT, pulls nothing heavy, and gives the
`description` of each field from its doc comment.

### Catalog

```rust
pub struct Catalog { tools: Vec<Arc<dyn Tool>> }

impl Catalog {
    pub fn select(self, keep: impl Fn(&ToolDesc) -> bool) -> Self;
    pub fn without(self, names: &[&str]) -> Self;
    pub fn only(self, names: &[&str]) -> Result<Self, UnknownTool>;
    pub fn merge(self, other: Catalog) -> Result<Self, DuplicateTool>;
    pub fn prefixed(self, prefix: &str) -> Self;   // "oxrok_" when the UI's own names collide
    pub fn read_only(self) -> Self;                // keeps readOnlyHint tools
    pub fn names(&self) -> Vec<&str>;
}
```

The server holds its catalog behind `Arc<RwLock<Catalog>>`. Swapping it
sends `notifications/tools/list_changed`, so the UI can turn tools on and
off while a client is connected. `tools/list` is answered from the
current catalog; `tools/call` on a name not in it is "unknown tool", not
"disabled", so a dropped tool is indistinguishable from one that never
existed.

### Server and transport

```rust
pub struct Server {
    info: ServerInfo,              // name "oxrok", version, instructions for the model
    catalog: Arc<RwLock<Catalog>>,
}

impl Server {
    pub fn serve<R: Read, W: Write>(&self, r: R, w: W) -> Result<()>;   // one client, until EOF
    pub fn serve_stdio(&self) -> Result<()>;
}
```

The protocol layer is written by hand: JSON-RPC 2.0, one message per
line, `initialize` with version negotiation, `notifications/initialized`,
`ping`, `tools/list` (with `cursor`, though the catalogs are small),
`tools/call`, `notifications/cancelled`, `notifications/progress`,
`logging/setLevel`. That is a few hundred lines on `serde_json`, sync,
and it runs the same over stdin/stdout, a Unix socket, a Windows named
pipe or a test pipe. Requests are handled one at a time per connection,
which is what stdio clients send anyway; a long tool call still answers
`ping` because the reader thread and the handler thread are separate.

Why not `rmcp`, the official SDK: it is good and it is Apache-2.0, but it
requires tokio and brings the async model into a crate that has none;
for a stdio server with tools only, the protocol surface is small and
stable. The decision is marked **(open)** below with the trade; the tool
model above is the same either way, and an `rmcp` adapter
(`impl ServerHandler for Catalog`) is a day's work if the HTTP transport
is wanted later.

Logging: a stdio server owns stdout, so every log line goes to stderr
(oxrok's logger already does) and, when the client asked for it, to
`notifications/message`.

## The core tools

Named as the model will see them. Tags in brackets. Everything returns
structured JSON; text content is a short rendering of the same.

### Catalog [Catalog, read-only, no hardware]

| tool | what it answers |
|---|---|
| `version` | oxrok version, features built in, decoder runtime and directory |
| `list_drivers` | id, long name, classes, scan options, whether it needs firmware; `class` filter |
| `driver_info` | one driver: scan options with kinds, firmware files and whether they were found, hardware-verification status from `docs/DEVICES.md` when the build carries it |
| `list_decoders` | id, name, description, tags; `tag`/`query` filters |
| `decoder_info` | channels (required/optional), options with defaults, annotation rows and classes, binary classes, what it stacks on: what `-P x --show` prints |
| `list_formats` | input and output modules with their options |
| `firmware_status` | the `--list-firmware` JSON, with the search paths |

### Pass-through [Cli]

| tool | what it answers |
|---|---|
| `cli` | runs the `oxrok` binary with sigrok-cli arguments and returns stdout, stderr and the exit code, capped at `max_bytes` |

This is the first tool worth having and the last one to remove. Models
already know sigrok-cli's flags; `cli {"args": ["-d","demo","--samples","100","-O","bits"]}`
gives the agent the whole CLI on day one, and stays the escape hatch for
anything the structured tools do not cover. It runs `current_exe()` as a
subprocess with a timeout and a kill, so a driver that hangs does not
hang the server. It is tagged `Cli` so a catalog can drop it, and it
refuses `-d` for a device the host has open (two owners of one USB
device). Default on for `oxrok mcp`, default off in the UI.

### Devices [Device; `set_option` and `open` are Hardware]

| tool | what it does |
|---|---|
| `scan` | `driver` and `conn` optional; returns devices with a stable `device_id` (`driver:serial` or `driver:conn`, the UI's `DeviceId` rule), vendor, model, version, channels |
| `open` | opens a scanned device; returns its `handle` and `describe_device` |
| `close` | |
| `describe_device` | channels and groups, options with access, current value and the allowed list or range; what `--show` prints |
| `get_option` / `set_option` | `key`, `value` (sigrok text syntax: `1m`, `1.5`, `on`), `group`; values parsed and checked with the CLI's rules |
| `enable_channels` | the `-C` syntax or a list |

Open devices live in the `Host` by handle. A capture takes the device
into a `Session` on a worker thread and puts it back after; a call on a
handle whose device is capturing gets `busy`. Stdin EOF, `Ctrl-C` and
`close` all go through `Device::close`, which the roadmap notes the CLI
does not yet do on every exit; the MCP host does from the start.

### Captures [Capture; `capture*` on a handle is Hardware]

| tool | what it does |
|---|---|
| `capture` | `handle`, one of `samples`/`time`/`frames`, `triggers` (`-t` syntax), `wait_trigger`, optional `config` applied first; runs to the end or `timeout` and returns a `capture_id` and the summary below |
| `capture_start` / `capture_status` / `capture_stop` | the same, asynchronous: for long or continuous runs; `status` reports samples so far, and progress notifications go out while it runs |
| `list_captures` / `drop_capture` | |
| `load_capture` | a `.sr`, VCD or binary file, with `-I` options, into the store |
| `export_capture` | writes a file with any output module (`sr`, `csv`, `vcd`, `bits`, ...) and its options; returns the path and size, never the bytes |
| `capture_summary` | samplerate, sample count, duration, trigger position, per logic channel: transitions, duty, first and last level, min pulse; per analog channel: min, max, mean, RMS, unit |

A capture in the store is the device info, the channel list, the
samplerate, the logic data as `unit_size` + bytes, the analog blocks, the
trigger and frame markers. The store has a byte budget (`--max-memory`,
default a few hundred MB); the oldest capture goes when a new one does
not fit, and the tool result says so.

### Data [Capture, read-only]

| tool | what it answers |
|---|---|
| `capture_logic` | `channels`, `from`/`to` (seconds or samples), `form`: `transitions` (default, `[{t, level}]`), `rle`, or `samples` (capped hard); `offset`/`limit` |
| `capture_analog` | `channel`, range, `buckets`: min/max/mean per bucket, so a 1 M sample trace is 200 rows |
| `measure` | on one logic channel over a range: frequency, period, duty, high/low time stats, edge count; on an analog one: Vpp, mean, RMS, frequency by zero crossing. The UI's `engine::measure` has these; the oxrok side gets its own small, shared implementation so both agree |
| `find` | the first/next edge, pulse wider than, or level change after `t` on a channel: how an agent walks a trace without reading it |

### Decode [Decode, read-only]

| tool | what it answers |
|---|---|
| `capture_decode` | `capture_id`, `stack` in `-P` syntax (`i2c:scl=D0:sda=D1,eeprom24xx`), `annotations` (`-A` syntax), `rows`, range, `offset`/`limit`; returns `[{t0, t1, decoder, row, class, text}]` plus the count, and binary output on request |

Runs on the native decoders or the Python ones exactly as `-P` does, over
the capture's logic data, with the same instance and option parsing.

### Server flags

```
oxrok mcp [--read-only] [--no-hardware] [--no-cli] [--tools a,b,c]
          [--drivers demo,fx2lafw] [--max-memory 512M] [--timeout 30s]
          [--list-tools [--json|--markdown]] [--instructions FILE]
```

`--tools` is `Catalog::only`; `--read-only` drops every tool without
`readOnlyHint`; `--no-hardware` drops the `Hardware` tag (scanning still
works: it finds devices without opening them); `--drivers` limits scans.
`--list-tools` prints the catalog and exits, and is what the docs and a
doc check are generated from.

The `instructions` sent at `initialize` are a short text for the model:
the workflow (`scan` → `open` → `set_option samplerate` → `capture` →
`measure`/`capture_decode`), the id syntaxes, and "prefer `transitions`
and `measure` to `samples`".

## The CLI side

A `mcp` subcommand on the existing clap `Args`, with
`subcommand_negates_reqs` and `args_conflicts_with_subcommands`, so
every sigrok-cli flag keeps its meaning and `oxrok mcp --help` is
discoverable. Client configuration is then:

```json
{ "mcpServers": { "oxrok": { "command": "oxrok", "args": ["mcp"] } } }
```

and `claude mcp add oxrok -- oxrok mcp`.

The reusable, non-printing parts of `src/cli/` move into the library so
the MCP tools and the CLI run the same code: `parse::driver_spec`,
`plan_configs`, `select_channels`, `parse_triggers`, the generic
`key=value` option parsing, decoder stack parsing (`decode::parse_spec`
is already in the lib), the `Decoding` setup, and the `Pipeline`'s
limit and output-module selection. They become `oxrok::setup` (or
`core::setup`), need no clap, and `src/cli/` calls them. The printing
stays in the binary. This is a refactor on its own, done before the
device tools, and it is what the UI's `OxrokBackend` would also use
instead of its own copies.

## The UI side

### Transport

oxrok-ui listens on a local socket with the same line-delimited JSON-RPC
as stdio: a Unix domain socket at `$XDG_RUNTIME_DIR/oxrok-ui/mcp.sock`
(macOS: `~/Library/Application Support/oxrok-ui/mcp.sock`), mode 0600,
and a named pipe `\\.\pipe\oxrok-ui-mcp` on Windows. A client reaches it
through a stdio shim:

```json
{ "mcpServers": { "oxrok-ui": { "command": "oxrok-ui", "args": ["--mcp-stdio"] } } }
```

`--mcp-stdio` connects to the socket and copies bytes both ways; if the
UI is not running it starts it (or says so, **(open)**). Every MCP client
supports stdio, the framing code is the one already written, and
nothing listens on a TCP port. The socket is the UI's server; the shim
has no state.

Streamable HTTP, for a client on another machine or in a browser, is a
later step and the point where `rmcp` would pay for itself.

### Threading

The socket thread owns the `Server` and runs tool handlers. Handlers
that read capture data go straight to the session's shared store (the
`SharedCapture` `Arc`, the transition store), no UI thread involved.
Handlers that change UI state send a `UiRequest` through a channel,
call `request_repaint`, and wait on a reply channel with a timeout; the
app services the queue at the top of `update()`, next to the worker
events. That is the pattern `Worker` already uses in the other
direction.

### The UI catalog

```rust
let core = oxrok::mcp::tools::catalog(host.clone());
let catalog = core
    .select(|t| t.tags.contains(&Tag::Catalog) || t.tags.contains(&Tag::Decode))   // no device tools: the worker owns devices
    .merge(ui_tools(app_handle))?;
```

The UI implements `CaptureSource` over its sessions, so `capture_logic`,
`capture_analog`, `measure`, `find` and `capture_decode` from the library
work on the session the user is looking at, with `capture_id` being the
session id. It does not include `scan`/`open`/`capture`: its worker owns
the devices, and the UI tools below do the equivalent through it, where
the user can see it happen.

UI tools (`ui_` prefix, so a client with both servers can tell them
apart):

| tool | what it does |
|---|---|
| `ui_list_sessions` | sessions with device, state, sample count, rate, visible range |
| `ui_new_session` | from a device id (the start screen's list) or a file |
| `ui_session_settings` / `ui_set_session_settings` | mode (timed/continuous), rate, options, enabled rows, trigger |
| `ui_capture` | start the session's capture; waits for it to end unless `wait: false` |
| `ui_stop` | |
| `ui_get_view` / `ui_set_view` | the visible window; zoom to a range or around `t` |
| `ui_markers` / `ui_add_marker` / `ui_remove_marker` | timing markers and pairs |
| `ui_measurements` | the range measurements the user has placed, with their metrics |
| `ui_analyzers` / `ui_add_analyzer` / `ui_remove_analyzer` | the Analyzers panel |
| `ui_analyzer_frames` | the data table of one analyzer, paginated, with the same filter as the UI |
| `ui_screenshot` | the capture view as PNG, returned as image content; optional range to show first |
| `ui_save` / `ui_open_file` | `.oxrok` files; export through the library's `export_capture` |
| `ui_run_command` / `ui_list_commands` | the command registry (`app::commands`), so every palette action is reachable without a tool each |

### Picking tools in the UI

A *Settings → MCP* panel: enable the server, show the socket path and
the client configuration to copy, and a checklist of every tool in the
full catalog (library and UI ones) grouped by tag, with *Read-only* and
*No hardware* presets. The choice persists in the app settings and is
applied by swapping the server's catalog, which notifies connected
clients. A status line shows connected clients and the last call.

## Safety

- Annotations on every tool; `set_option`, `open`, `capture*` on hardware
  are `destructiveHint: true`, `readOnlyHint: false`. Supplies and loads
  are where this matters.
- `--read-only`, `--no-hardware`, `--tools` and the UI checklist all go
  through `Catalog::select`, so there is one mechanism and it is testable.
- `export_capture` and `load_capture` take paths; the server has
  `--allow-paths DIR` (default: the working directory, and the UI's
  documents directory) and refuses others.
- `cli` is a subprocess with a timeout; its output is capped; it cannot
  be combined with an open handle on the same device.
- The socket is per-user and 0600; the named pipe gets the default ACL
  for the current user. No TCP listener in this plan.

## Testing

- `tests/mcp.rs`: spawns `oxrok mcp` with pipes and a tiny client
  (`initialize`, `tools/list`, `tools/call`) on the `demo` driver. Golden
  JSON for `list_drivers`, `decoder_info i2c`, a 100-sample demo capture's
  summary, `capture_logic` transitions, a `capture_decode` of demo's
  pattern. Deterministic, no hardware.
- Replay: `OXROK_USB_REPLAY` and friends work under the MCP host as they
  do under the CLI, so a driver's fixture can also pin `scan`/`open`/
  `capture` through the tools.
- Every tool's `input_schema` is validated against the args struct (a
  round trip of the example in its description), and `--list-tools` is
  diffed against `docs/MCP.md` in CI, the way the drivers table is kept.
- Catalog tests: `select`, `only` with an unknown name, `merge` with a
  duplicate, `read_only`, `list_changed` after a swap.
- Protocol tests: a client that offers an unknown protocol version, a
  call with bad arguments, cancellation mid-capture, `ping` during a
  long call.
- UI: a kittest test that starts the socket server on the mock backend,
  connects, calls `ui_list_sessions` and `capture_logic`, and
  `ui_screenshot`.
- Manual: the MCP Inspector against `oxrok mcp` and the shim.

## Documentation

- `docs/MCP.md`: configuring Claude Code, Claude Desktop, Cursor; the
  flags; the tool reference generated by `oxrok mcp --list-tools
  --markdown`; the UI's socket and shim; what the agent can and cannot
  do to hardware.
- A paragraph in `README.md` and in the UI's.
- `docs/ROADMAP.md` gets an "MCP" section listing the phases not yet
  done, updated as each lands (per `AGENTS.md`).

## Phases

Each phase is shippable on its own.

### Phase 0: decisions

The open ones below. Half a day.

### Phase 1: a stdio server with the catalog and the CLI pass-through

`src/mcp/` with the protocol, the tool model, the catalog, and the
`Catalog`-tagged tools plus `cli`. The `mcp` subcommand and its flags.
`tests/mcp.rs` with the pipe client. `docs/MCP.md`. Useful immediately:
an agent gets the whole CLI and the reference data.

### Phase 2: move the CLI's setup code into the library

`oxrok::setup` as described, with `src/cli/` calling it and the CLI's
golden tests proving nothing changed. No MCP work in this phase; it is
what makes the next one honest.

### Phase 3: devices and captures

`Host` with open devices and the capture store; `scan`, `open`, `close`,
`describe_device`, `get_option`, `set_option`, `enable_channels`,
`capture` (synchronous with timeout), `load_capture`, `export_capture`,
`capture_summary`, `capture_logic`, `capture_analog`, `measure`, `find`.
Close on every exit. Replay-backed tests.

### Phase 4: decode and the asynchronous capture

`capture_decode` over native and Python decoders. `capture_start`,
`capture_status`, `capture_stop`, progress notifications, cancellation.
The memory budget and eviction. `--read-only`, `--no-hardware`,
`--tools`, `--allow-paths`.

### Phase 5: the UI server

The socket listener and `--mcp-stdio` shim in oxrok-ui. `CaptureSource`
over the session store. The `ui_` tools, in the order of the table (the
read-only ones first, then capture, then view and markers, then
screenshots and commands). The Settings panel with the checklist and
persisted selection. Kittest coverage.

### Phase 6: later

- Streamable HTTP through `rmcp`, as a feature, when a remote client is
  wanted. The `Catalog` becomes an `rmcp::ServerHandler`.
- MCP resources: `oxrok://captures/{id}` and `oxrok://devices/{handle}`
  as subscribable JSON, so a client sees a capture finish without
  polling.
- Prompts: "decode this bus" and "characterise this clock" workflows.
- The MCP Tasks extension for long captures, once clients use it.

## Open decisions

1. **Protocol layer: hand-rolled or `rmcp`?** (open) Recommended:
   hand-rolled for stdio and the socket; tokio-free, a few hundred lines,
   under the project's own rules. `rmcp` when HTTP is wanted. The tool
   model is independent of the answer.
2. **`oxrok mcp` subcommand or an `oxrok-mcp` binary?** (open)
   Recommended: the subcommand; one binary in the release archives and
   in `cargo install`. The binary is a two-line wrapper if a client ever
   needs it.
3. **Schemas: `schemars` derive or hand-written `json!`?** (open)
   Recommended: `schemars`, so the schema and the parser are one thing.
   One small MIT dependency.
4. **The UI shim: `oxrok-ui --mcp-stdio` or `oxrok mcp --ui`?** (open)
   Recommended: in oxrok-ui, so the shim and the server ship together
   and the CLI does not learn the UI's socket path. Whether the shim
   starts the UI when it is not running, or fails with the path, is part
   of this.
5. **Which library tools does the UI include by default?** (open)
   Recommended: `Catalog`, `Decode` and the read-only `Capture` data
   tools over the session store; no device tools; `cli` off.
6. **`measure` in oxrok or in the UI?** (open) Recommended: a small
   implementation in `oxrok::mcp::tools::data` (or `core::measure`) that
   the UI's `engine::measure` can later adopt, so the two servers give
   the same numbers.
7. **Memory budget default and whether captures persist to a temp
   directory when evicted.** (open) Recommended: 512 MB, no persistence;
   `export_capture` is the way to keep one.
8. **Should `capture` without a limit be allowed (continuous until
   `capture_stop`)?** (open) Recommended: only through `capture_start`,
   never through the synchronous `capture`.
