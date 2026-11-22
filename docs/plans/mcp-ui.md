# The MCP server in oxrok-ui: plan

What is left of the MCP plan. The command line's server, `oxrok mcp`,
is built and [MCP.md](../MCP.md) documents it, with the reasons behind
its design. This page is the part not built: an extended server in
oxrok-ui, and the later steps on both sides. It is a plan to react to;
the decisions marked **(open)** are collected at the end. When a step
lands, its text leaves this page, and the page goes when nothing is
left ([CONTRIBUTING.md](../../CONTRIBUTING.md), "Plans").

oxrok-ui serves an *extended* MCP: a chosen subset of the library's
tools plus tools that act on the UI itself (sessions, view, markers,
analyzers, screenshots), through a local socket and a stdio shim. An
agent can then work on the session the user is looking at and show the
user what it found (zoom there, drop a marker, add an analyzer), and
the user picks which tools it gets.

Most of the work is in oxrok-ui; what this repository owes it is named
where it comes up.

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
has no state. The socket is per user and 0600; the named pipe gets the
default ACL for the current user.

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
let core = oxrok::mcp::tools::catalog(&host);
let catalog = core
    .select(|t| t.tags.contains(&Tag::Catalog) || t.tags.contains(&Tag::Decode))   // no device tools: the worker owns devices
    .merge(ui_tools(app_handle))?;
```

The data tools (`capture_logic`, `capture_analog`, `measure`, `find`,
`capture_view`, `capture_decode`) should work on the session the user is
looking at, with `capture_id` being the session id. The library does not
allow that yet: its tools read the `Host`'s own `CaptureStore`
(`Host::captures`), and the narrow trait this plan called
`CaptureSource`, which the store and the UI's sessions would both
implement, was never written. That trait, or a `CaptureStore` the UI can
back with its sessions, is the first piece of work here and is in this
repository.

The library's analyzer tools are not in the UI catalog as they are: in
the UI an analyzer is something the user sees, in the panel and on the
trace, so the agent's analyzers should be the user's. The `ui_` analyzer
tools below take the same arguments and return the same frame shape
(`text`, `value`, `bits`, radix per request), over the session's own
analyzers. Until the UI's decoder thread moves onto `decode::Analyzer`,
they map onto its `Analyzer`, `TaskSpec` and `FrameStore`; after, they
are thin wrappers. Two differences remain: a UI analyzer is one decoder,
not a stack (`TaskSpec` has one `decoder`), so `ui_add_analyzer` refuses
a stack until the UI gains them; and colour, lanes and visible rows get
the UI's defaults, which the user can change. The UI catalog does not
include `scan`/`open`/`capture`: its worker owns the devices, and the UI tools below do the equivalent through it, where
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
| `ui_analyzers` / `ui_add_analyzer` / `ui_update_analyzer` / `ui_remove_analyzer` | the Analyzers panel: the library's `list`/`add`/`update`/`remove_analyzer` arguments, one decoder each; `update` can set the radix the panel shows |
| `ui_analyzer_status` / `ui_analyzer_frames` / `ui_analyzer_summary` / `ui_analyzer_find` | as the library's, over the session's analyzers; `frames` also takes the data table's search text |
| `ui_show_frame` | zooms the view to a frame and optionally drops a marker on it: how the agent shows the user what it found |
| `ui_screenshot` | the capture view as PNG, returned as image content; optional range to show first |
| `ui_save` / `ui_open_file` | `.oxrok` files; export through the library's `export_capture` |
| `ui_run_command` / `ui_list_commands` | the command registry (`app::commands`), so every palette action is reachable without a tool each |

### Picking tools in the UI

An MCP panel in the settings: enable the server, show the socket path and
the client configuration to copy, and a checklist of every tool in the
full catalog (library and UI ones) grouped by tag, with *Read-only* and
*No hardware* presets. The choice persists in the app settings and is
applied by swapping the server's catalog, which notifies connected
clients. A status line shows connected clients and the last call.

### Testing

A kittest test that starts the socket server on the mock backend,
connects, and calls `ui_list_sessions`, `capture_logic` and
`ui_screenshot`. By hand: the MCP Inspector against the shim.

## Steps

### The UI server

The socket listener and the `--mcp-stdio` shim in oxrok-ui. The data
tools over the session store (the trait above). oxrok-ui's radix code
replaced by `oxrok::decode::radix`, which was moved from it. The `ui_`
tools, in the order of the table (the read-only ones first, then
capture, then analyzers, then view and markers, then screenshots and
commands). The Settings panel with the checklist and the persisted
selection. Kittest coverage.

### Later

- The UI's decoder thread and `FrameStore` moved onto
  `oxrok::decode::Analyzer`, and stacked analyzers in the UI.
- An analyzer trigger.
  `capture_start {stop_when: {analyzer, text | value | class}}` stops a
  capture when a matching frame appears, as the UI's Analyzer Trigger card
  does. It works when decoding keeps up with the device. A buffered device
  (the LogicPort) delivers its data at the end of a capture, so there it
  is "capture again until a frame matches", the UI's `Bench::capture`
  pattern, with a round limit.
- Streamable HTTP through `rmcp`, as a feature, when a client on another
  machine or in a browser is wanted. The `Catalog` becomes an
  `rmcp::ServerHandler`.
- MCP resources: `oxrok://captures/{id}` and `oxrok://devices/{handle}`
  as subscribable JSON, so a client sees a capture finish without
  polling.
- Prompts: "decode this bus" and "characterise this clock" workflows.
- The MCP Tasks extension for long captures, once clients use it.

## Open decisions

1. **The UI shim: `oxrok-ui --mcp-stdio` or `oxrok mcp --ui`?** (open)
   Recommended: in oxrok-ui, so the shim and the server ship together
   and the CLI does not learn the UI's socket path. Whether the shim
   starts the UI when it is not running, or fails with the path, is part
   of this.
2. **Which library tools does the UI include by default?** (open)
   Recommended: the reference tools, the decoder ones and the read-only
   data tools over the session store; no device tools; `cli` off.
3. **Should the UI expose the library's analyzer tools as well as its
   own?** (open) Recommended: no; the agent's analyzers are the user's
   analyzers in the panel. A library analyzer on a UI session would be
   invisible to the user.
4. **Radix on a decoder that has both a `format` option and binary
   output (`uart`).** The library prefers the raw values, so the radix
   is free per request; the UI passes `format` to the decoder until it
   adopts the shared code. The texts can differ slightly (the decoder's
   own ASCII escaping against `ascii_char`): whether the UI's panel
   changes with the move is (open).
