// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! An MCP (Model Context Protocol) server for oxrok, so an LLM agent can
//! find out what oxrok supports, run the command line, and later scan,
//! capture, measure and decode through the same stack the CLI uses.
//!
//! The pieces, from the inside out:
//!
//! - [`Tool`]: a name, a JSON schema and a function. Nothing in a tool
//!   knows which server offers it.
//! - [`Catalog`]: the tools one server offers; filtered with
//!   [`Catalog::select`], [`Catalog::only`], [`Catalog::read_only`] and
//!   friends, so a person can say exactly what an agent may do.
//! - [`Host`]: what the tools work on: the driver and decoder registries
//!   and the limits.
//! - [`Server`]: the protocol, JSON-RPC 2.0 one message per line over any
//!   `Read` and `Write`: stdin and stdout, a socket, a test pipe.
//!
//! [`tools::catalog`] builds the library's catalog over a [`Host`];
//! `oxrok mcp` serves it over stdio. The protocol layer is written by
//! hand and synchronous: a few hundred lines on `serde_json`, no tokio.
//! `PLAN.mcp.md` in the repository has the design and the phases.

#![forbid(unsafe_code)]

pub mod capture;
pub mod catalog;
pub mod host;
pub mod jsonrpc;
pub mod protocol;
pub mod tool;
pub mod tools;
pub mod view;

pub use capture::{Capture, CaptureStore};
pub use catalog::{Catalog, CatalogError};
pub use host::{Host, Limits};
pub use protocol::{PROTOCOL_VERSIONS, Server, ServerInfo};
pub use tool::{
    Annotations, Call, Content, LogLevel, Tag, Tool, ToolDesc, ToolError, ToolResult, ToolSpec,
    function,
};

/// The instructions sent to the model at `initialize` unless `oxrok mcp
/// --instructions` gives a file: the vocabulary, the workflow and what
/// to prefer.
pub const DEFAULT_INSTRUCTIONS: &str = "\
oxrok is a sigrok compatible stack: logic analyzers, oscilloscopes, multimeters, \
power supplies and protocol decoders. Its vocabulary is sigrok's: driver ids \
(`fx2lafw`, `demo`), option keys (`samplerate`), value syntax (`1m`, `D0=r`), \
decoder ids (`i2c`, `uart`) and output modules (`bits`, `csv`, `vcd`).

Start with `version` and `list_drivers`; `driver_info`, `decoder_info` and \
`list_formats` answer the details before a run, and `firmware_status` says \
which firmware files were found.

The workflow on a device: `scan` (a driver spec such as `demo:logic_channels=4` \
or `fx2lafw`) -> `open` by `device_id` -> `describe_device` for the options -> \
`set_option` (`samplerate` `1m`) and `enable_channels` -> `capture` with a \
limit (`samples` `1k`, `time` `10ms`) and optional `triggers` (`D0=r`) -> read \
the summary it returns, then `measure` (frequency, duty, pulse widths), `find` \
(the next edge or pulse) and `capture_logic` (transitions, paged) on the \
`capture_id`; `capture_analog` reduces an analog channel to buckets. Prefer \
`measure`, `find` and transitions to `form: samples`. For a long or an open-ended \
run, `capture_start` (no limit means until `capture_stop`) returns at once; \
`capture_status {wait}` follows it, the data tools and `add_analyzer` work on it \
while it runs, and `capture_stop` ends it. `export_capture` writes a `.sr` for \
PulseView; `load_capture` reads one back. The `demo` driver needs no hardware \
and makes patterns (`channel_group=Logic:pattern=walking-one`).

`cli` runs the oxrok command line with sigrok-cli arguments, one per element: \
`{\"args\": [\"-d\", \"demo\", \"--samples\", \"100\", \"-O\", \"bits\"]}`; use \
it for what the tools do not cover (`-L`, an output module's text). Decoding \
goes through `add_analyzer` or `capture_decode`, whose frames are paged and \
searched, not through `cli -P`. Keep `cli` outputs small. It cannot run on a \
device this server has open.

Hardware and the files on disk are shared with the person at the bench: say \
what a call will do before it changes a setting on a supply or a load, and \
`close` what you opened when done.";
