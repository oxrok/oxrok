// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! JSON-RPC 2.0, one message per line, over any `Read` and `Write`.
//!
//! MCP's stdio transport is exactly this framing; a Unix socket or a
//! test pipe runs the same code. Batches (a JSON array of messages) are
//! refused: MCP dropped them in its 2025-06-18 revision and no stdio
//! client sends them.

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::fmt;
use std::io::{self, Write};
use std::sync::{Arc, Mutex, PoisonError};

/// A request id: the client's choice of a number or a string.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Id {
    /// A number.
    Num(i64),
    /// A string.
    Str(String),
}

/// The line was not JSON.
pub const PARSE_ERROR: i64 = -32700;
/// The JSON was not a request.
pub const INVALID_REQUEST: i64 = -32600;
/// No such method.
pub const METHOD_NOT_FOUND: i64 = -32601;
/// Bad parameters: an unknown tool or arguments that do not parse.
pub const INVALID_PARAMS: i64 = -32602;
/// The server failed.
pub const INTERNAL_ERROR: i64 = -32603;

/// A JSON-RPC error object.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RpcError {
    /// One of the codes above.
    pub code: i64,
    /// One sentence.
    pub message: String,
    /// Anything more.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub data: Option<Value>,
}

impl RpcError {
    /// An error with `code`.
    pub fn new(code: i64, message: impl Into<String>) -> Self {
        Self { code, message: message.into(), data: None }
    }

    /// `INVALID_PARAMS`.
    pub fn invalid_params(message: impl Into<String>) -> Self {
        Self::new(INVALID_PARAMS, message)
    }

    /// `METHOD_NOT_FOUND`, naming the method.
    pub fn method_not_found(method: &str) -> Self {
        Self::new(METHOD_NOT_FOUND, format!("Method not found: {method}"))
    }

    /// `INTERNAL_ERROR`.
    pub fn internal(message: impl Into<String>) -> Self {
        Self::new(INTERNAL_ERROR, message)
    }
}

impl fmt::Display for RpcError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} ({})", self.message, self.code)
    }
}

impl std::error::Error for RpcError {}

/// A message from the client.
#[derive(Clone, Debug, PartialEq)]
pub enum Message {
    /// A call that wants an answer.
    Request {
        /// Goes back in the answer.
        id: Id,
        /// The method.
        method: String,
        /// Its parameters, `Null` when none were given.
        params: Value,
    },
    /// A call that wants no answer.
    Notification {
        /// The method.
        method: String,
        /// Its parameters, `Null` when none were given.
        params: Value,
    },
    /// An answer to a request the server made. The server makes none
    /// that need one, so these are accepted and dropped.
    Response {
        /// The id it answers.
        id: Option<Id>,
    },
}

#[derive(Deserialize)]
struct Raw {
    jsonrpc: Option<String>,
    #[serde(default)]
    id: Option<Id>,
    method: Option<String>,
    #[serde(default)]
    params: Option<Value>,
    #[serde(default)]
    result: Option<Value>,
    #[serde(default)]
    error: Option<Value>,
}

/// Parses one line.
pub fn parse(line: &str) -> Result<Message, RpcError> {
    if line.trim_start().starts_with('[') {
        return Err(RpcError::new(INVALID_REQUEST, "Invalid request: batches are not supported."));
    }
    let raw: Raw = serde_json::from_str(line)
        .map_err(|e| RpcError::new(PARSE_ERROR, format!("Parse error: {e}")))?;
    if raw.jsonrpc.as_deref() != Some("2.0") {
        return Err(RpcError::new(INVALID_REQUEST, "Invalid request: jsonrpc must be \"2.0\"."));
    }
    let params = raw.params.unwrap_or(Value::Null);
    match (raw.method, raw.id) {
        (Some(method), Some(id)) => Ok(Message::Request { id, method, params }),
        (Some(method), None) => Ok(Message::Notification { method, params }),
        (None, id) if raw.result.is_some() || raw.error.is_some() => Ok(Message::Response { id }),
        (None, _) => Err(RpcError::new(INVALID_REQUEST, "Invalid request: no method.")),
    }
}

/// A successful answer to `id`.
pub fn response(id: &Id, result: Value) -> Value {
    let mut v = json!({"jsonrpc": "2.0", "id": id});
    v["result"] = result;
    v
}

/// A failed answer to `id`, or to no request at all when the line could
/// not be parsed (`id` is `null` then, as the specification says).
pub fn error_response(id: Option<&Id>, error: &RpcError) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "error": error})
}

/// A notification from the server.
pub fn notification(method: &str, params: Value) -> Value {
    let mut v = json!({"jsonrpc": "2.0", "method": method});
    v["params"] = params;
    v
}

/// Where the messages to one client go. Cloned into every thread that
/// writes (the handler, the reader answering `ping`, a tool reporting
/// progress); one message at a time, one line each.
#[derive(Clone)]
pub struct Sink(Arc<Mutex<Box<dyn Write + Send>>>);

impl Sink {
    /// A sink writing to `w`.
    pub fn new(w: impl Write + Send + 'static) -> Self {
        Self(Arc::new(Mutex::new(Box::new(w))))
    }

    /// Writes `message` as one line and flushes.
    pub fn send(&self, message: &Value) -> io::Result<()> {
        // Serialised before the lock is taken and in one write, so two
        // threads never interleave and a compact rendering has no line
        // break of its own.
        let mut line = serde_json::to_vec(message)?;
        line.push(b'\n');
        let mut w = self.0.lock().unwrap_or_else(PoisonError::into_inner);
        w.write_all(&line)?;
        w.flush()
    }

    /// Whether both write to the same client.
    pub fn same(&self, other: &Sink) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}

impl fmt::Debug for Sink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("Sink")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_requests_and_notifications() {
        let m = parse(r#"{"jsonrpc":"2.0","id":1,"method":"ping"}"#).unwrap();
        assert_eq!(
            m,
            Message::Request { id: Id::Num(1), method: "ping".into(), params: Value::Null }
        );
        let m = parse(r#"{"jsonrpc":"2.0","id":"a","method":"x","params":{"k":1}}"#).unwrap();
        assert_eq!(
            m,
            Message::Request {
                id: Id::Str("a".into()),
                method: "x".into(),
                params: json!({"k":1})
            }
        );
        let m = parse(r#"{"jsonrpc":"2.0","method":"notifications/initialized"}"#).unwrap();
        assert_eq!(
            m,
            Message::Notification {
                method: "notifications/initialized".into(),
                params: Value::Null
            }
        );
        let m = parse(r#"{"jsonrpc":"2.0","id":3,"result":{}}"#).unwrap();
        assert_eq!(m, Message::Response { id: Some(Id::Num(3)) });
    }

    #[test]
    fn refuses_what_is_not_a_request() {
        assert_eq!(parse("nope").unwrap_err().code, PARSE_ERROR);
        assert_eq!(parse(r#"{"id":1,"method":"ping"}"#).unwrap_err().code, INVALID_REQUEST);
        assert_eq!(parse(r#"{"jsonrpc":"2.0","id":1}"#).unwrap_err().code, INVALID_REQUEST);
        assert_eq!(parse(r"[]").unwrap_err().code, INVALID_REQUEST);
    }

    #[test]
    fn builds_the_three_message_shapes() {
        assert_eq!(
            response(&Id::Num(1), json!({"ok": true})),
            json!({"jsonrpc":"2.0","id":1,"result":{"ok":true}})
        );
        let err = RpcError::method_not_found("foo");
        assert_eq!(
            error_response(None, &err),
            json!({"jsonrpc":"2.0","id":null,"error":{"code":-32601,"message":"Method not found: foo"}})
        );
        assert_eq!(
            notification("notifications/progress", json!({"progress": 1})),
            json!({"jsonrpc":"2.0","method":"notifications/progress","params":{"progress":1}})
        );
    }

    #[test]
    fn a_sink_writes_one_line_per_message() {
        #[derive(Clone)]
        struct Buf(Arc<Mutex<Vec<u8>>>);
        impl Write for Buf {
            fn write(&mut self, b: &[u8]) -> io::Result<usize> {
                self.0.lock().unwrap().extend_from_slice(b);
                Ok(b.len())
            }
            fn flush(&mut self) -> io::Result<()> {
                Ok(())
            }
        }
        let buf = Buf(Arc::default());
        let sink = Sink::new(buf.clone());
        sink.send(&json!({"a": "line\nbreak"})).unwrap();
        sink.send(&json!({"b": 2})).unwrap();
        let out = String::from_utf8(buf.0.lock().unwrap().clone()).unwrap();
        assert_eq!(out, "{\"a\":\"line\\nbreak\"}\n{\"b\":2}\n");
        assert!(sink.same(&sink.clone()));
    }
}
