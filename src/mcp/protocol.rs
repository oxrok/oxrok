// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The server: MCP's `initialize`, `ping`, `tools/list`, `tools/call`,
//! `logging/setLevel` and the notifications, over one connection at a
//! time.
//!
//! Two threads per connection: a reader that parses lines, answers
//! `ping` itself and turns `notifications/cancelled` into a flag the
//! running tool sees, and the handler that runs everything else in
//! order. A long tool call therefore still answers `ping`, and a client
//! that cancels is heard.

use super::catalog::Catalog;
use super::jsonrpc::{self, Id, Message, RpcError, Sink};
use super::tool::{Call, LogLevel};
use serde::Deserialize;
use serde_json::{Value, json};
use std::collections::HashMap;
use std::io::{self, BufRead, BufReader, Read, Write};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::atomic::{AtomicBool, AtomicU8};
use std::sync::mpsc::{self, Sender};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError, RwLock};
use std::thread;

/// The protocol revisions served, newest first. A client asking for one
/// of them gets it; any other request gets the newest.
pub const PROTOCOL_VERSIONS: &[&str] = &["2025-06-18", "2025-03-26", "2024-11-05"];

/// What the client learns at `initialize`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ServerInfo {
    /// The server's name.
    pub name: String,
    /// Its version.
    pub version: String,
    /// A text for the model: the vocabulary and the workflow.
    pub instructions: Option<String>,
}

impl ServerInfo {
    /// `oxrok`, this version, the default instructions.
    pub fn oxrok() -> Self {
        Self {
            name: "oxrok".to_owned(),
            version: crate::core::VERSION.to_owned(),
            instructions: Some(super::DEFAULT_INSTRUCTIONS.to_owned()),
        }
    }
}

/// The requests being handled, and the cancellations that arrived
/// before their request was registered (the reader is ahead of the
/// handler).
#[derive(Default)]
struct InflightState {
    running: HashMap<Id, Arc<AtomicBool>>,
    early: std::collections::HashSet<Id>,
}

type Inflight = Arc<Mutex<InflightState>>;

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// An MCP server over a catalog.
pub struct Server {
    info: ServerInfo,
    catalog: Arc<RwLock<Catalog>>,
    clients: Mutex<Vec<Sink>>,
    log_level: Arc<AtomicU8>,
}

impl Server {
    /// A server offering `catalog`.
    pub fn new(info: ServerInfo, catalog: Catalog) -> Self {
        Self {
            info,
            catalog: Arc::new(RwLock::new(catalog)),
            clients: Mutex::new(Vec::new()),
            log_level: Arc::new(AtomicU8::new(LogLevel::Info.rank())),
        }
    }

    /// The catalog served, shared.
    pub fn catalog(&self) -> Arc<RwLock<Catalog>> {
        self.catalog.clone()
    }

    /// Replaces the catalog and tells connected clients the tools
    /// changed. A tool dropped this way is "unknown" from then on, not
    /// "disabled".
    pub fn set_catalog(&self, catalog: Catalog) {
        *self.catalog.write().unwrap_or_else(PoisonError::into_inner) = catalog;
        let note = jsonrpc::notification("notifications/tools/list_changed", json!({}));
        for client in lock(&self.clients).iter() {
            let _ = client.send(&note);
        }
    }

    /// Serves one client until its input ends.
    pub fn serve<R, W>(&self, reader: R, writer: W) -> io::Result<()>
    where
        R: Read + Send + 'static,
        W: Write + Send + 'static,
    {
        let sink = Sink::new(writer);
        lock(&self.clients).push(sink.clone());
        let inflight: Inflight = Arc::default();
        let (tx, rx) = mpsc::channel();
        let reader_thread = {
            let sink = sink.clone();
            let inflight = inflight.clone();
            thread::Builder::new()
                .name("mcp-reader".to_owned())
                .spawn(move || read_loop(reader, &sink, &inflight, &tx))?
        };
        for message in rx {
            self.handle(message, &sink, &inflight);
        }
        let result = reader_thread.join().unwrap_or(Ok(()));
        lock(&self.clients).retain(|c| !c.same(&sink));
        result
    }

    /// Serves stdin and stdout until stdin ends. Logs must go to stderr
    /// while this runs; stdout belongs to the protocol.
    pub fn serve_stdio(&self) -> io::Result<()> {
        self.serve(io::stdin(), io::stdout())
    }

    fn handle(&self, message: Message, sink: &Sink, inflight: &Inflight) {
        match message {
            Message::Request { id, method, params } => {
                let result = match method.as_str() {
                    "initialize" => Ok(self.initialize(&params)),
                    "ping" => Ok(json!({})),
                    "tools/list" => {
                        Ok(self.catalog.read().unwrap_or_else(PoisonError::into_inner).to_json())
                    }
                    "tools/call" => self.tools_call(&id, params, sink, inflight),
                    "logging/setLevel" => self.set_log_level(&params),
                    _ => Err(RpcError::method_not_found(&method)),
                };
                let reply = match result {
                    Ok(v) => jsonrpc::response(&id, v),
                    Err(e) => jsonrpc::error_response(Some(&id), &e),
                };
                // A client that closed its end is seen by the reader as
                // EOF; nothing to do about a failed write here.
                let _ = sink.send(&reply);
            }
            Message::Notification { method, .. } => {
                if method != "notifications/initialized" {
                    log::debug!("Ignoring notification {method}.");
                }
            }
            Message::Response { .. } => {}
        }
    }

    fn initialize(&self, params: &Value) -> Value {
        let requested = params.get("protocolVersion").and_then(Value::as_str);
        let version =
            requested.filter(|v| PROTOCOL_VERSIONS.contains(v)).unwrap_or(PROTOCOL_VERSIONS[0]);
        let mut v = json!({
            "protocolVersion": version,
            "capabilities": {"tools": {"listChanged": true}, "logging": {}},
            "serverInfo": {"name": self.info.name, "version": self.info.version},
        });
        if let Some(text) = &self.info.instructions {
            v["instructions"] = json!(text);
        }
        v
    }

    fn tools_call(
        &self,
        id: &Id,
        params: Value,
        sink: &Sink,
        inflight: &Inflight,
    ) -> Result<Value, RpcError> {
        #[derive(Deserialize)]
        struct CallParams {
            name: String,
            #[serde(default)]
            arguments: Option<Value>,
            #[serde(default, rename = "_meta")]
            meta: Option<Value>,
        }
        let p: CallParams = serde_json::from_value(params)
            .map_err(|e| RpcError::invalid_params(format!("Invalid params: {e}")))?;
        let tool = self
            .catalog
            .read()
            .unwrap_or_else(PoisonError::into_inner)
            .get(&p.name)
            .cloned()
            .ok_or_else(|| RpcError::invalid_params(format!("Unknown tool '{}'.", p.name)))?;
        let args = p.arguments.unwrap_or_else(|| json!({}));
        let token = p.meta.and_then(|m| m.get("progressToken").cloned());
        let cancelled = {
            let mut state = lock(inflight);
            let flag = Arc::new(AtomicBool::new(state.early.remove(id)));
            state.running.insert(id.clone(), flag.clone());
            flag
        };
        let mut cx = Call::new(sink.clone(), token, cancelled, self.log_level.clone());
        let outcome = catch_unwind(AssertUnwindSafe(|| tool.call(args, &mut cx)));
        lock(inflight).running.remove(id);
        match outcome {
            Ok(Ok(result)) => Ok(result.to_json()),
            Ok(Err(e)) if e.invalid_args => Err(RpcError::invalid_params(e.message)),
            Ok(Err(e)) => Ok(e.to_json()),
            Err(_) => Err(RpcError::internal(format!("Tool '{}' panicked.", p.name))),
        }
    }

    fn set_log_level(&self, params: &Value) -> Result<Value, RpcError> {
        let name = params.get("level").and_then(Value::as_str).unwrap_or_default();
        let level = LogLevel::from_name(name)
            .ok_or_else(|| RpcError::invalid_params(format!("Unknown log level '{name}'.")))?;
        self.log_level.store(level.rank(), std::sync::atomic::Ordering::Relaxed);
        Ok(json!({}))
    }
}

/// Reads lines until EOF. `ping` and `notifications/cancelled` are dealt
/// with here, so they are heard while a tool runs; everything else goes
/// to the handler in order.
fn read_loop<R: Read>(
    reader: R,
    sink: &Sink,
    inflight: &Inflight,
    tx: &Sender<Message>,
) -> io::Result<()> {
    for line in BufReader::new(reader).lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        match jsonrpc::parse(&line) {
            Err(e) => sink.send(&jsonrpc::error_response(None, &e))?,
            Ok(Message::Request { id, method, .. }) if method == "ping" => {
                sink.send(&jsonrpc::response(&id, json!({})))?;
            }
            Ok(Message::Notification { method, params }) if method == "notifications/cancelled" => {
                let id: Option<Id> =
                    params.get("requestId").cloned().and_then(|v| serde_json::from_value(v).ok());
                if let Some(id) = id {
                    let mut state = lock(inflight);
                    match state.running.get(&id) {
                        Some(flag) => flag.store(true, std::sync::atomic::Ordering::Relaxed),
                        None => {
                            state.early.insert(id);
                        }
                    }
                }
            }
            Ok(message) => {
                if tx.send(message).is_err() {
                    break;
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mcp::tool::{Annotations, Tag, ToolResult, ToolSpec, function};
    use schemars::JsonSchema;
    use serde::Deserialize;
    use std::time::{Duration, Instant};

    #[derive(Clone, Default)]
    struct Out(Arc<Mutex<Vec<u8>>>);

    impl Write for Out {
        fn write(&mut self, b: &[u8]) -> io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(b);
            Ok(b.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    #[derive(Deserialize, JsonSchema)]
    #[serde(deny_unknown_fields)]
    struct EchoArgs {
        /// Comes back.
        text: String,
    }

    #[derive(Deserialize, JsonSchema)]
    struct NoArgs {}

    fn catalog() -> Catalog {
        let echo = function::<EchoArgs, _>(
            ToolSpec {
                name: "echo",
                title: "Echo",
                description: "Returns its text.",
                example: json!({"text": "hi"}),
                annotations: Annotations::READ_ONLY,
                tags: &[Tag::Catalog],
            },
            |a, cx| {
                cx.progress(1.0, Some(1.0), Some("done"));
                Ok(ToolResult::structured(json!({"text": a.text})))
            },
        );
        let fails = function::<NoArgs, _>(
            ToolSpec {
                name: "fails",
                title: "Fails",
                description: "Always fails.",
                example: json!({}),
                annotations: Annotations::READ_ONLY,
                tags: &[],
            },
            |_, _| Err("It failed.".into()),
        );
        let waits = function::<NoArgs, _>(
            ToolSpec {
                name: "waits",
                title: "Waits",
                description: "Waits for cancellation.",
                example: json!({}),
                annotations: Annotations::READ_ONLY,
                tags: &[],
            },
            |_, cx| {
                let start = Instant::now();
                while !cx.cancelled() && start.elapsed() < Duration::from_secs(10) {
                    thread::sleep(Duration::from_millis(5));
                }
                Ok(ToolResult::structured(json!({"cancelled": cx.cancelled()})))
            },
        );
        Catalog::from_tools(vec![echo, fails, waits]).unwrap()
    }

    /// Runs `lines` through a server and returns the replies, parsed.
    fn run(lines: &[&str]) -> Vec<Value> {
        let server = Server::new(ServerInfo::oxrok(), catalog());
        let input = lines.join("\n") + "\n";
        let out = Out::default();
        server.serve(io::Cursor::new(input.into_bytes()), out.clone()).unwrap();
        let bytes = out.0.lock().unwrap().clone();
        String::from_utf8(bytes)
            .unwrap()
            .lines()
            .map(|l| serde_json::from_str(l).unwrap())
            .collect()
    }

    #[test]
    fn initialize_negotiates_the_version() {
        let out = run(&[
            r#"{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-03-26","capabilities":{},"clientInfo":{"name":"t","version":"0"}}}"#,
            r#"{"jsonrpc":"2.0","method":"notifications/initialized"}"#,
            r#"{"jsonrpc":"2.0","id":2,"method":"initialize","params":{"protocolVersion":"1999-01-01"}}"#,
        ]);
        assert_eq!(out[0]["result"]["protocolVersion"], "2025-03-26");
        assert_eq!(out[0]["result"]["serverInfo"]["name"], "oxrok");
        assert_eq!(out[0]["result"]["capabilities"]["tools"]["listChanged"], true);
        assert!(out[0]["result"]["instructions"].as_str().unwrap().contains("sigrok"));
        assert_eq!(out[1]["result"]["protocolVersion"], PROTOCOL_VERSIONS[0]);
        assert_eq!(out.len(), 2);
    }

    #[test]
    fn lists_and_calls_tools() {
        let out = run(&[
            r#"{"jsonrpc":"2.0","id":1,"method":"tools/list"}"#,
            r#"{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"echo","arguments":{"text":"hi"},"_meta":{"progressToken":"p1"}}}"#,
            r#"{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"fails"}}"#,
            r#"{"jsonrpc":"2.0","id":4,"method":"tools/call","params":{"name":"echo","arguments":{"nope":1}}}"#,
            r#"{"jsonrpc":"2.0","id":5,"method":"tools/call","params":{"name":"missing"}}"#,
        ]);
        let tools = out[0]["result"]["tools"].as_array().unwrap();
        assert_eq!(tools.len(), 3);
        assert_eq!(tools[0]["name"], "echo");
        assert_eq!(tools[0]["inputSchema"]["required"], json!(["text"]));
        assert_eq!(tools[0]["annotations"]["readOnlyHint"], true);
        // The progress notification goes out before the answer.
        assert_eq!(out[1]["method"], "notifications/progress");
        assert_eq!(
            out[1]["params"],
            json!({"progressToken":"p1","progress":1.0,"total":1.0,"message":"done"})
        );
        assert_eq!(out[2]["id"], 2);
        assert_eq!(out[2]["result"]["structuredContent"], json!({"text":"hi"}));
        assert_eq!(out[2]["result"]["content"][0]["type"], "text");
        assert_eq!(out[3]["result"]["isError"], true);
        assert_eq!(out[3]["result"]["content"][0]["text"], "It failed.");
        assert_eq!(out[4]["error"]["code"], jsonrpc::INVALID_PARAMS);
        assert!(out[4]["error"]["message"].as_str().unwrap().contains("unknown field"));
        assert_eq!(out[5]["error"]["code"], jsonrpc::INVALID_PARAMS);
        assert_eq!(out[5]["error"]["message"], "Unknown tool 'missing'.");
    }

    #[test]
    fn answers_ping_errors_and_unknown_methods() {
        let out = run(&[
            "not json",
            r#"{"jsonrpc":"2.0","id":1,"method":"ping"}"#,
            r#"{"jsonrpc":"2.0","id":2,"method":"resources/list"}"#,
            r#"{"jsonrpc":"2.0","id":3,"method":"logging/setLevel","params":{"level":"debug"}}"#,
            r#"{"jsonrpc":"2.0","id":4,"method":"logging/setLevel","params":{"level":"loud"}}"#,
            r#"{"jsonrpc":"2.0","id":9,"result":{}}"#,
        ]);
        assert_eq!(out[0]["error"]["code"], jsonrpc::PARSE_ERROR);
        assert_eq!(out[0]["id"], Value::Null);
        assert_eq!(out[1], json!({"jsonrpc":"2.0","id":1,"result":{}}));
        assert_eq!(out[2]["error"]["code"], jsonrpc::METHOD_NOT_FOUND);
        assert_eq!(out[3]["result"], json!({}));
        assert_eq!(out[4]["error"]["code"], jsonrpc::INVALID_PARAMS);
        assert_eq!(out.len(), 5);
    }

    #[test]
    fn a_cancelled_call_is_told_so() {
        // The reader sees the cancellation while the handler is inside
        // the tool, which then returns early.
        let start = Instant::now();
        let out = run(&[
            r#"{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"waits"}}"#,
            r#"{"jsonrpc":"2.0","id":2,"method":"ping"}"#,
            r#"{"jsonrpc":"2.0","method":"notifications/cancelled","params":{"requestId":1}}"#,
        ]);
        assert!(start.elapsed() < Duration::from_secs(5));
        let call = out.iter().find(|m| m["id"] == 1).unwrap();
        assert_eq!(call["result"]["structuredContent"], json!({"cancelled": true}));
        assert!(out.iter().any(|m| m["id"] == 2));
    }

    #[test]
    fn swapping_the_catalog_notifies() {
        let server = Server::new(ServerInfo::oxrok(), catalog());
        let out = Out::default();
        // A connection that stays open while the catalog is swapped.
        let (mut w, r) = os_pipe();
        let handle = {
            let out = out.clone();
            let server = Arc::new(server);
            let s = server.clone();
            (thread::spawn(move || s.serve(r, out).unwrap()), server)
        };
        let (thread, server) = handle;
        while lock(&server.clients).is_empty() {
            thread::sleep(Duration::from_millis(1));
        }
        server.set_catalog(catalog().only(&["echo"]).unwrap());
        writeln!(w, r#"{{"jsonrpc":"2.0","id":1,"method":"tools/list"}}"#).unwrap();
        drop(w);
        thread.join().unwrap();
        let text = String::from_utf8(out.0.lock().unwrap().clone()).unwrap();
        let msgs: Vec<Value> = text.lines().map(|l| serde_json::from_str(l).unwrap()).collect();
        assert_eq!(msgs[0]["method"], "notifications/tools/list_changed");
        assert_eq!(msgs[1]["result"]["tools"].as_array().unwrap().len(), 1);
        assert!(lock(&server.clients).is_empty());
    }

    /// A pipe: what is written to the first end is read from the second.
    fn os_pipe() -> (impl Write, impl Read + Send + 'static) {
        struct Rx(mpsc::Receiver<Vec<u8>>, Vec<u8>);
        impl Read for Rx {
            fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
                if self.1.is_empty() {
                    match self.0.recv() {
                        Ok(chunk) => self.1 = chunk,
                        Err(_) => return Ok(0),
                    }
                }
                let n = buf.len().min(self.1.len());
                buf[..n].copy_from_slice(&self.1[..n]);
                self.1.drain(..n);
                Ok(n)
            }
        }
        struct Tx(mpsc::Sender<Vec<u8>>);
        impl Write for Tx {
            fn write(&mut self, b: &[u8]) -> io::Result<usize> {
                let _ = self.0.send(b.to_vec());
                Ok(b.len())
            }
            fn flush(&mut self) -> io::Result<()> {
                Ok(())
            }
        }
        let (tx, rx) = mpsc::channel();
        (Tx(tx), Rx(rx, Vec::new()))
    }
}
