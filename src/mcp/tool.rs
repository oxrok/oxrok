// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The tool model: a name, a JSON schema and a function, independent of
//! the server that offers it.

use super::jsonrpc::{self, Sink};
use schemars::JsonSchema;
use serde::Serialize;
use serde::de::DeserializeOwned;
use serde_json::{Value, json};
use std::fmt;
use std::marker::PhantomData;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};

/// What a tool does to the world, as MCP's tool annotations tell it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)]
pub struct Annotations {
    /// Changes nothing.
    pub read_only: bool,
    /// May change something that cannot be undone: a setting on a
    /// supply, a file.
    pub destructive: bool,
    /// Calling it twice with the same arguments is the same as once.
    pub idempotent: bool,
    /// Reaches beyond the server: hardware, the network.
    pub open_world: bool,
}

impl Annotations {
    /// Reads and reports.
    pub const READ_ONLY: Self =
        Self { read_only: true, destructive: false, idempotent: true, open_world: false };
    /// Changes the server's own state (a capture store, an open handle),
    /// not the world.
    pub const MUTATING: Self =
        Self { read_only: false, destructive: false, idempotent: false, open_world: false };
    /// Drives hardware or runs commands.
    pub const HARDWARE: Self =
        Self { read_only: false, destructive: true, idempotent: false, open_world: true };

    /// The `annotations` object of a `tools/list` entry.
    pub fn to_json(self) -> Value {
        json!({
            "readOnlyHint": self.read_only,
            "destructiveHint": self.destructive,
            "idempotentHint": self.idempotent,
            "openWorldHint": self.open_world,
        })
    }
}

/// A class of tools a catalog keeps or drops as one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Tag {
    /// What oxrok supports: drivers, decoders, formats, firmware.
    Catalog,
    /// The command line pass-through.
    Cli,
    /// Devices: scanning, opening, options.
    Device,
    /// Opens or drives hardware.
    Hardware,
    /// Captures and their data.
    Capture,
    /// Protocol decoding.
    Decode,
}

impl Tag {
    /// Every tag.
    pub const ALL: [Tag; 6] =
        [Tag::Catalog, Tag::Cli, Tag::Device, Tag::Hardware, Tag::Capture, Tag::Decode];

    /// The name on the command line and in the docs.
    pub fn name(self) -> &'static str {
        match self {
            Tag::Catalog => "catalog",
            Tag::Cli => "cli",
            Tag::Device => "device",
            Tag::Hardware => "hardware",
            Tag::Capture => "capture",
            Tag::Decode => "decode",
        }
    }

    /// The tag called `name`.
    pub fn from_name(name: &str) -> Option<Tag> {
        Tag::ALL.into_iter().find(|t| t.name() == name)
    }
}

/// Everything a client is told about a tool.
#[derive(Clone, Debug)]
pub struct ToolDesc {
    /// The name the model calls: `scan`, `capture_logic`.
    pub name: String,
    /// A few words for people.
    pub title: String,
    /// Written for the model: what it does, what it returns, what to
    /// prefer.
    pub description: String,
    /// Arguments that work, shown in the docs and checked by the tests.
    pub example: Value,
    /// The JSON schema of the arguments.
    pub input_schema: Value,
    /// The JSON schema of `structuredContent`, when the tool declares one.
    pub output_schema: Option<Value>,
    /// What it does to the world.
    pub annotations: Annotations,
    /// The classes it belongs to.
    pub tags: Vec<Tag>,
}

impl ToolDesc {
    /// Whether the tool carries `tag`.
    pub fn has_tag(&self, tag: Tag) -> bool {
        self.tags.contains(&tag)
    }

    /// The `tools/list` entry.
    pub fn to_json(&self) -> Value {
        let mut v = json!({
            "name": self.name,
            "title": self.title,
            "description": self.description,
            "inputSchema": self.input_schema,
            "annotations": self.annotations.to_json(),
        });
        if let Some(out) = &self.output_schema {
            v["outputSchema"] = out.clone();
        }
        v
    }
}

/// One block of a result, as the client shows it.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum Content {
    /// Text.
    Text {
        /// The text.
        text: String,
    },
    /// An image.
    Image {
        /// Base64.
        data: String,
        /// `image/png` and the like.
        #[serde(rename = "mimeType")]
        mime_type: String,
    },
}

/// What a tool returns: a structured value, content blocks, or both.
/// The server renders the structured value as text when a tool gives no
/// content of its own.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ToolResult {
    /// The machine readable answer, `structuredContent`.
    pub structured: Option<Value>,
    /// What a client without structured content shows.
    pub content: Vec<Content>,
}

impl ToolResult {
    /// A structured answer, rendered as text by the server.
    pub fn structured(value: Value) -> Self {
        Self { structured: Some(value), content: Vec::new() }
    }

    /// A text answer.
    pub fn text(text: impl Into<String>) -> Self {
        Self { structured: None, content: vec![Content::Text { text: text.into() }] }
    }

    /// Adds a text block.
    #[must_use]
    pub fn with_text(mut self, text: impl Into<String>) -> Self {
        self.content.push(Content::Text { text: text.into() });
        self
    }

    /// The `tools/call` result.
    pub fn to_json(&self) -> Value {
        let content: Vec<Value> = if self.content.is_empty() {
            let text = self
                .structured
                .as_ref()
                .map(|v| serde_json::to_string_pretty(v).unwrap_or_default())
                .unwrap_or_default();
            vec![json!({"type": "text", "text": text})]
        } else {
            self.content.iter().map(|c| serde_json::to_value(c).unwrap_or(Value::Null)).collect()
        };
        let mut v = json!({"content": content});
        if let Some(s) = &self.structured {
            v["structuredContent"] = s.clone();
        }
        v
    }
}

/// A tool that did not do its job. Reported to the model as a result
/// with `isError`, so it reads the reason and tries again; arguments
/// that do not parse are a protocol error instead.
#[derive(Clone, Debug, PartialEq)]
pub struct ToolError {
    /// One or two sentences.
    pub message: String,
    /// Anything more.
    pub data: Option<Value>,
    /// The arguments were wrong, not the run.
    pub invalid_args: bool,
}

impl ToolError {
    /// The tool failed.
    pub fn new(message: impl Into<String>) -> Self {
        Self { message: message.into(), data: None, invalid_args: false }
    }

    /// The arguments were wrong.
    pub fn invalid_args(message: impl Into<String>) -> Self {
        Self { message: message.into(), data: None, invalid_args: true }
    }

    /// Adds structured data.
    #[must_use]
    pub fn with_data(mut self, data: Value) -> Self {
        self.data = Some(data);
        self
    }

    /// The `tools/call` result with `isError`.
    pub fn to_json(&self) -> Value {
        let mut v = json!({
            "content": [{"type": "text", "text": self.message}],
            "isError": true,
        });
        if let Some(d) = &self.data {
            v["structuredContent"] = d.clone();
        }
        v
    }
}

impl fmt::Display for ToolError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ToolError {}

impl From<crate::core::Error> for ToolError {
    fn from(e: crate::core::Error) -> Self {
        Self::new(e.to_string())
    }
}

impl From<std::io::Error> for ToolError {
    fn from(e: std::io::Error) -> Self {
        Self::new(e.to_string())
    }
}

impl From<serde_json::Error> for ToolError {
    fn from(e: serde_json::Error) -> Self {
        Self::new(e.to_string())
    }
}

impl From<String> for ToolError {
    fn from(s: String) -> Self {
        Self::new(s)
    }
}

impl From<&str> for ToolError {
    fn from(s: &str) -> Self {
        Self::new(s)
    }
}

/// The levels of `notifications/message`, RFC 5424's.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum LogLevel {
    /// Debug.
    Debug,
    /// Info.
    Info,
    /// Notice.
    Notice,
    /// Warning.
    Warning,
    /// Error.
    Error,
    /// Critical.
    Critical,
    /// Alert.
    Alert,
    /// Emergency.
    Emergency,
}

impl LogLevel {
    const ALL: [LogLevel; 8] = [
        LogLevel::Debug,
        LogLevel::Info,
        LogLevel::Notice,
        LogLevel::Warning,
        LogLevel::Error,
        LogLevel::Critical,
        LogLevel::Alert,
        LogLevel::Emergency,
    ];

    /// The name on the wire.
    pub fn name(self) -> &'static str {
        match self {
            LogLevel::Debug => "debug",
            LogLevel::Info => "info",
            LogLevel::Notice => "notice",
            LogLevel::Warning => "warning",
            LogLevel::Error => "error",
            LogLevel::Critical => "critical",
            LogLevel::Alert => "alert",
            LogLevel::Emergency => "emergency",
        }
    }

    /// The level called `name`.
    pub fn from_name(name: &str) -> Option<LogLevel> {
        LogLevel::ALL.into_iter().find(|l| l.name() == name)
    }

    pub(crate) fn rank(self) -> u8 {
        self as u8
    }

    pub(crate) fn from_rank(rank: u8) -> LogLevel {
        LogLevel::ALL.get(rank as usize).copied().unwrap_or(LogLevel::Emergency)
    }
}

/// What a running tool may ask of the protocol: report progress, check
/// for cancellation, log to the client.
pub struct Call {
    sink: Option<Sink>,
    progress_token: Option<Value>,
    cancelled: Arc<AtomicBool>,
    log_level: Arc<AtomicU8>,
}

impl Call {
    /// A call with no client behind it: progress and logs go nowhere and
    /// it is never cancelled. For tests, and for using the tools as a
    /// library.
    pub fn detached() -> Self {
        Self {
            sink: None,
            progress_token: None,
            cancelled: Arc::new(AtomicBool::new(false)),
            log_level: Arc::new(AtomicU8::new(LogLevel::Info.rank())),
        }
    }

    pub(crate) fn new(
        sink: Sink,
        progress_token: Option<Value>,
        cancelled: Arc<AtomicBool>,
        log_level: Arc<AtomicU8>,
    ) -> Self {
        Self { sink: Some(sink), progress_token, cancelled, log_level }
    }

    /// Reports progress, when the client asked for it with a token.
    pub fn progress(&self, done: f64, total: Option<f64>, message: Option<&str>) {
        let (Some(sink), Some(token)) = (&self.sink, &self.progress_token) else {
            return;
        };
        let mut params = json!({"progressToken": token, "progress": done});
        if let Some(t) = total {
            params["total"] = json!(t);
        }
        if let Some(m) = message {
            params["message"] = json!(m);
        }
        let _ = sink.send(&jsonrpc::notification("notifications/progress", params));
    }

    /// Whether the client cancelled this call. A long tool checks it
    /// between steps and returns early.
    pub fn cancelled(&self) -> bool {
        self.cancelled.load(Ordering::Relaxed)
    }

    /// Sends a log message to the client, if the level the client set
    /// lets it through.
    pub fn log(&self, level: LogLevel, logger: &str, data: Value) {
        let Some(sink) = &self.sink else {
            return;
        };
        if level < LogLevel::from_rank(self.log_level.load(Ordering::Relaxed)) {
            return;
        }
        let mut params = json!({"level": level.name(), "logger": logger});
        params["data"] = data;
        let _ = sink.send(&jsonrpc::notification("notifications/message", params));
    }
}

/// A tool: a description and a function.
pub trait Tool: Send + Sync {
    /// What the client is told.
    fn desc(&self) -> &ToolDesc;

    /// Runs it.
    fn call(&self, args: Value, cx: &mut Call) -> Result<ToolResult, ToolError>;

    /// Parses `args` as [`Tool::call`] would, without running anything:
    /// the check behind the schema.
    fn check_args(&self, args: &Value) -> Result<(), String>;
}

/// The static facts of a tool, before its function.
pub struct ToolSpec {
    /// The name the model calls.
    pub name: &'static str,
    /// A few words for people.
    pub title: &'static str,
    /// Written for the model.
    pub description: &'static str,
    /// Arguments that work.
    pub example: Value,
    /// What it does to the world.
    pub annotations: Annotations,
    /// Its classes.
    pub tags: &'static [Tag],
}

struct FnTool<A, F> {
    desc: ToolDesc,
    f: F,
    _args: PhantomData<fn(A)>,
}

impl<A, F> Tool for FnTool<A, F>
where
    A: DeserializeOwned + JsonSchema + Send + Sync,
    F: Fn(A, &mut Call) -> Result<ToolResult, ToolError> + Send + Sync,
{
    fn desc(&self) -> &ToolDesc {
        &self.desc
    }

    fn call(&self, args: Value, cx: &mut Call) -> Result<ToolResult, ToolError> {
        let args: A = serde_json::from_value(args)
            .map_err(|e| ToolError::invalid_args(format!("Invalid arguments: {e}")))?;
        (self.f)(args, cx)
    }

    fn check_args(&self, args: &Value) -> Result<(), String> {
        serde_json::from_value::<A>(args.clone()).map(|_| ()).map_err(|e| e.to_string())
    }
}

/// The JSON schema of `A` as MCP wants it: the object schema itself,
/// without the `$schema` and `title` keys the generator adds.
pub fn schema_of<A: JsonSchema>() -> Value {
    let schema = schemars::SchemaGenerator::default().into_root_schema_for::<A>();
    let mut v = serde_json::to_value(schema).unwrap_or_else(|_| json!({"type": "object"}));
    if let Some(o) = v.as_object_mut() {
        o.remove("$schema");
        o.remove("title");
    }
    v
}

/// A tool from a function over its arguments. The schema comes from the
/// `JsonSchema` derive on `A`, so the schema the client sees and the
/// parser that reads the call cannot drift.
pub fn function<A, F>(spec: ToolSpec, f: F) -> Arc<dyn Tool>
where
    A: DeserializeOwned + JsonSchema + Send + Sync + 'static,
    F: Fn(A, &mut Call) -> Result<ToolResult, ToolError> + Send + Sync + 'static,
{
    let desc = ToolDesc {
        name: spec.name.to_owned(),
        title: spec.title.to_owned(),
        description: spec.description.to_owned(),
        example: spec.example,
        input_schema: schema_of::<A>(),
        output_schema: None,
        annotations: spec.annotations,
        tags: spec.tags.to_vec(),
    };
    Arc::new(FnTool { desc, f, _args: PhantomData })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;

    #[derive(Deserialize, JsonSchema)]
    #[serde(deny_unknown_fields)]
    struct AddArgs {
        /// Left.
        a: i64,
        /// Right, 0 when left out.
        #[serde(default)]
        b: Option<i64>,
    }

    fn add() -> Arc<dyn Tool> {
        function::<AddArgs, _>(
            ToolSpec {
                name: "add",
                title: "Add",
                description: "Adds two numbers.",
                example: json!({"a": 1, "b": 2}),
                annotations: Annotations::READ_ONLY,
                tags: &[Tag::Catalog],
            },
            |args, _cx| Ok(ToolResult::structured(json!({"sum": args.a + args.b.unwrap_or(0)}))),
        )
    }

    #[test]
    fn the_schema_follows_the_struct() {
        let t = add();
        let s = &t.desc().input_schema;
        assert_eq!(s["type"], "object");
        assert_eq!(s["properties"]["a"]["type"], "integer");
        assert_eq!(s["properties"]["a"]["description"], "Left.");
        assert_eq!(s["required"], json!(["a"]));
        assert_eq!(s["additionalProperties"], false);
        assert!(s.get("$schema").is_none());
        assert!(t.check_args(&t.desc().example).is_ok());
        assert!(t.check_args(&json!({"a": "x"})).is_err());
    }

    #[test]
    fn calls_and_reports_bad_arguments() {
        let t = add();
        let mut cx = Call::detached();
        let r = t.call(json!({"a": 2, "b": 3}), &mut cx).unwrap();
        assert_eq!(r.structured, Some(json!({"sum": 5})));
        assert_eq!(r.to_json()["content"][0]["text"], "{\n  \"sum\": 5\n}");
        let e = t.call(json!({"a": 2, "c": 3}), &mut cx).unwrap_err();
        assert!(e.invalid_args, "{e}");
        assert!(e.message.contains("unknown field"), "{e}");
    }

    #[test]
    fn errors_render_with_is_error() {
        let e = ToolError::new("No such device.").with_data(json!({"code": 7}));
        assert_eq!(
            e.to_json(),
            json!({"content":[{"type":"text","text":"No such device."}],"isError":true,"structuredContent":{"code":7}})
        );
        assert_eq!(LogLevel::from_name("warning"), Some(LogLevel::Warning));
        assert_eq!(LogLevel::from_rank(LogLevel::Alert.rank()), LogLevel::Alert);
        assert_eq!(Tag::from_name("hardware"), Some(Tag::Hardware));
    }
}
