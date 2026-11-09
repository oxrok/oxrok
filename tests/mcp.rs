// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! `oxrok mcp` through its stdio transport: a tiny client speaks JSON-RPC
//! to the binary over pipes. Deterministic, no hardware: the reference
//! tools and the `demo` driver.

use serde_json::{Value, json};
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, Command, Stdio};
use std::sync::mpsc::{self, Receiver};
use std::time::Duration;

/// One `oxrok mcp` process and the lines it sends back.
struct Client {
    child: Child,
    stdin: std::process::ChildStdin,
    lines: Receiver<String>,
    next_id: i64,
    /// Notifications that arrived while waiting for an answer.
    notifications: Vec<Value>,
}

impl Client {
    fn start(flags: &[&str]) -> Client {
        let mut child = Command::new(env!("CARGO_BIN_EXE_oxrok"))
            .arg("mcp")
            .args(flags)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .expect("spawn oxrok mcp");
        let stdin = child.stdin.take().unwrap();
        let stdout = child.stdout.take().unwrap();
        let (tx, lines) = mpsc::channel();
        std::thread::spawn(move || {
            for line in BufReader::new(stdout).lines() {
                let Ok(line) = line else { break };
                if tx.send(line).is_err() {
                    break;
                }
            }
        });
        Client { child, stdin, lines, next_id: 0, notifications: Vec::new() }
    }

    fn send(&mut self, message: &Value) {
        let mut line = serde_json::to_string(message).unwrap();
        line.push('\n');
        self.stdin.write_all(line.as_bytes()).unwrap();
        self.stdin.flush().unwrap();
    }

    fn read(&mut self) -> Value {
        let line = self.lines.recv_timeout(Duration::from_secs(60)).expect("a reply in time");
        serde_json::from_str(&line).unwrap_or_else(|e| panic!("not JSON: {e}: {line}"))
    }

    /// Sends a request and returns its reply, keeping notifications aside.
    fn request(&mut self, method: &str, params: Value) -> Value {
        self.next_id += 1;
        let id = self.next_id;
        let mut m = json!({"jsonrpc": "2.0", "id": id, "method": method});
        m["params"] = params;
        self.send(&m);
        loop {
            let m = self.read();
            if m.get("id") == Some(&json!(id)) {
                return m;
            }
            assert!(m.get("method").is_some(), "unexpected message {m}");
            self.notifications.push(m);
        }
    }

    /// The `result` of a request that must succeed.
    fn ok(&mut self, method: &str, params: Value) -> Value {
        let mut m = self.request(method, params);
        assert!(m.get("error").is_none(), "{method} failed: {m}");
        m["result"].take()
    }

    fn initialize(&mut self) -> Value {
        let r = self.ok(
            "initialize",
            json!({"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "test", "version": "0"}}),
        );
        self.send(&json!({"jsonrpc": "2.0", "method": "notifications/initialized"}));
        r
    }

    /// Calls a tool and returns its `structuredContent`.
    fn call(&mut self, name: &str, args: Value) -> Value {
        let mut p = json!({"name": name});
        p["arguments"] = args;
        let mut r = self.ok("tools/call", p);
        assert_ne!(r.get("isError"), Some(&json!(true)), "{name} errored: {r}");
        r["structuredContent"].take()
    }

    /// Closes stdin and waits: the server must end on EOF, successfully.
    fn finish(mut self) {
        drop(self.stdin);
        let status = self.child.wait().unwrap();
        assert!(status.success(), "{status}");
    }
}

fn list_tools(flags: &[&str]) -> (String, bool) {
    let out = Command::new(env!("CARGO_BIN_EXE_oxrok"))
        .args(["mcp", "--list-tools"])
        .args(flags)
        .output()
        .unwrap();
    let text = if out.status.success() { &out.stdout } else { &out.stderr };
    (String::from_utf8_lossy(text).into_owned(), out.status.success())
}

const ALL_TOOLS: [&str; 36] = [
    "version",
    "list_drivers",
    "driver_info",
    "list_decoders",
    "decoder_info",
    "list_formats",
    "firmware_status",
    "cli",
    "scan",
    "open",
    "close",
    "describe_device",
    "get_option",
    "set_option",
    "enable_channels",
    "capture",
    "list_captures",
    "drop_capture",
    "capture_summary",
    "load_capture",
    "export_capture",
    "capture_logic",
    "capture_analog",
    "measure",
    "find",
    "add_analyzer",
    "update_analyzer",
    "remove_analyzer",
    "list_analyzers",
    "analyzer_status",
    "analyzer_frames",
    "analyzer_summary",
    "analyzer_find",
    "analyzer_binary",
    "analyzer_meta",
    "capture_decode",
];

/// The tools that change nothing.
const READ_ONLY: [&str; 23] = [
    "version",
    "list_drivers",
    "driver_info",
    "list_decoders",
    "decoder_info",
    "list_formats",
    "firmware_status",
    "scan",
    "describe_device",
    "get_option",
    "list_captures",
    "capture_summary",
    "capture_logic",
    "capture_analog",
    "measure",
    "find",
    "list_analyzers",
    "analyzer_status",
    "analyzer_frames",
    "analyzer_summary",
    "analyzer_find",
    "analyzer_binary",
    "analyzer_meta",
];

/// The tools that open or drive hardware.
const HARDWARE: [&str; 3] = ["open", "set_option", "capture"];

fn without<'a>(all: &[&'a str], drop: &[&str]) -> Vec<&'a str> {
    all.iter().copied().filter(|t| !drop.contains(t)).collect()
}

#[test]
fn initializes_lists_and_ends_on_eof() {
    let mut c = Client::start(&[]);
    let init = c.initialize();
    assert_eq!(init["protocolVersion"], "2025-06-18");
    assert_eq!(init["serverInfo"]["name"], "oxrok");
    assert_eq!(init["serverInfo"]["version"], env!("CARGO_PKG_VERSION"));
    assert!(init["instructions"].as_str().unwrap().contains("`cli`"));
    assert_eq!(init["capabilities"]["tools"]["listChanged"], true);
    assert_eq!(c.ok("ping", json!({})), json!({}));
    let tools = c.ok("tools/list", json!({}));
    let names: Vec<&str> =
        tools["tools"].as_array().unwrap().iter().filter_map(|t| t["name"].as_str()).collect();
    assert_eq!(names, ALL_TOOLS);
    for t in tools["tools"].as_array().unwrap() {
        assert_eq!(t["inputSchema"]["type"], "object", "{}", t["name"]);
        assert!(t["annotations"]["readOnlyHint"].is_boolean(), "{}", t["name"]);
        assert_ne!(t["description"].as_str().unwrap().len(), 0);
    }
    let cli = tools["tools"].as_array().unwrap().iter().find(|t| t["name"] == "cli").unwrap();
    assert_eq!(cli["annotations"]["readOnlyHint"], false);
    assert_eq!(cli["annotations"]["destructiveHint"], true);
    c.finish();
}

#[test]
fn reference_tools_match_their_goldens() {
    let mut c = Client::start(&[]);
    c.initialize();
    let version = c.call("version", json!({}));
    assert_eq!(version["version"], env!("CARGO_PKG_VERSION"));
    assert!(version["drivers"].as_u64().unwrap() > 50);
    assert!(version["features"].as_array().unwrap().contains(&json!("cli")));

    let drivers = c.call("list_drivers", json!({}));
    assert_eq!(
        drivers["count"].as_u64().unwrap(),
        drivers["drivers"].as_array().unwrap().len() as u64
    );
    let demo = drivers["drivers"].as_array().unwrap().iter().find(|d| d["id"] == "demo").unwrap();
    assert!(demo["classes"].as_array().unwrap().contains(&json!("demo-device")));
    assert_eq!(demo["needs_firmware"], false);
    let fx2 = drivers["drivers"].as_array().unwrap().iter().find(|d| d["id"] == "fx2lafw").unwrap();
    assert_eq!(fx2["needs_firmware"], true);
    assert!(fx2["classes"].as_array().unwrap().contains(&json!("logic-analyzer")));

    golden("driver_info_demo", &c.call("driver_info", json!({"driver": "demo"})));
    let mut i2c = c.call("decoder_info", json!({"decoder": "i2c"}));
    // The documentation text is the decoder's own; the structure is what is pinned.
    assert!(i2c["doc"].as_str().is_some_and(|d| !d.is_empty()));
    i2c.as_object_mut().unwrap().remove("doc");
    golden("decoder_info_i2c", &i2c);

    let formats = c.call("list_formats", json!({}));
    let outputs: Vec<&str> =
        formats["output"].as_array().unwrap().iter().filter_map(|m| m["id"].as_str()).collect();
    assert_eq!(
        outputs,
        ["analog", "ascii", "binary", "bits", "csv", "hex", "null", "srzip", "vcd"]
    );
    let inputs: Vec<&str> =
        formats["input"].as_array().unwrap().iter().filter_map(|m| m["id"].as_str()).collect();
    assert_eq!(inputs, ["binary", "vcd"]);

    let fw = c.call("firmware_status", json!({}));
    assert!(fw["drivers"].as_array().unwrap().iter().any(|d| d["driver"] == "fx2lafw"));
    assert_ne!(fw["search_dirs"].as_array().unwrap().len(), 0);
    c.finish();
}

/// `value` equals `tests/fixtures/mcp/<name>.json`. With
/// `OXROK_UPDATE_GOLDENS=1` the file is written instead.
fn golden(name: &str, value: &Value) {
    let path = format!("{}/tests/fixtures/mcp/{name}.json", env!("CARGO_MANIFEST_DIR"));
    let text = serde_json::to_string_pretty(value).unwrap() + "\n";
    if std::env::var_os("OXROK_UPDATE_GOLDENS").is_some() {
        std::fs::create_dir_all(std::path::Path::new(&path).parent().unwrap()).unwrap();
        std::fs::write(&path, &text).unwrap();
        return;
    }
    let want = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{path}: {e}; run with OXROK_UPDATE_GOLDENS=1 to write it"));
    assert_eq!(text, want, "{path} differs; OXROK_UPDATE_GOLDENS=1 rewrites it");
}

#[test]
fn the_cli_tool_runs_the_binary() {
    let mut c = Client::start(&[]);
    c.initialize();
    let r = c.call("cli", json!({"args": ["-d", "demo", "--scan"]}));
    assert_eq!(r["exit_code"], 0);
    assert_eq!(
        r["stdout"],
        "The following devices were found:\ndemo - Demo device with 13 channels: D0 D1 D2 D3 D4 D5 D6 D7 A0 A1 A2 A3 A4\n"
    );
    assert_eq!(r["timed_out"], false);
    assert_eq!(r["stdout_truncated"], false);

    // A command that fails is a result, not a tool error: the agent reads stderr.
    let r = c.call("cli", json!({"args": ["-d", "nope", "--scan"]}));
    assert_eq!(r["exit_code"], 1);
    assert!(r["stderr"].as_str().unwrap().contains("nope"), "{r}");

    // The limit cuts and says so.
    let r = c.call(
        "cli",
        json!({"args": ["-d", "demo:logic_channels=8:analog_channels=0", "--samples", "1000", "-O", "bits"], "max_bytes": 100}),
    );
    assert_eq!(r["stdout_truncated"], true);
    assert_eq!(r["stdout"].as_str().unwrap().len(), 100);

    // Not another server.
    let e = c.request("tools/call", json!({"name": "cli", "arguments": {"args": ["mcp"]}}));
    assert_eq!(e["error"]["code"], -32602);
    c.finish();
}

#[test]
#[allow(clippy::many_single_char_names)]
fn the_demo_device_over_the_pipe() {
    let dir = std::env::temp_dir().join(format!("oxrok-mcp-pipe-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let mut c = Client::start(&["--allow-path", dir.to_str().unwrap(), "--drivers", "demo"]);
    c.initialize();
    let s = c.call("scan", json!({"driver": "demo:logic_channels=4:analog_channels=1"}));
    assert_eq!(s["count"], 1);
    assert_eq!(s["devices"][0]["device_id"], "demo");
    let o = c.call("open", json!({"device_id": "demo"}));
    let handle = o["handle"].as_u64().unwrap();
    assert_eq!(o["driver"], "demo");
    let r = c.call("set_option", json!({"handle": handle, "key": "samplerate", "value": "1m"}));
    assert_eq!(r["text"], "1 MHz");
    c.call(
        "set_option",
        json!({"handle": handle, "key": "pattern", "value": "walking-one", "group": "Logic"}),
    );
    // A driver this server has open is not for the cli tool.
    let e =
        c.ok("tools/call", json!({"name": "cli", "arguments": {"args": ["-d", "demo", "--scan"]}}));
    assert_eq!(e["isError"], true);
    assert!(e["content"][0]["text"].as_str().unwrap().contains("open on this server"));
    // Another driver is not allowed at all.
    let e = c.ok("tools/call", json!({"name": "scan", "arguments": {"driver": "fx2lafw"}}));
    assert_eq!(e["isError"], true);

    let cap = c.call("capture", json!({"handle": handle, "samples": "1k", "triggers": "D0=r"}));
    let id = cap["capture_id"].as_u64().unwrap();
    assert_eq!(cap["samplerate"], 1_000_000);
    assert_eq!(cap["samples"], 1000);
    assert_eq!(cap["logic"].as_array().unwrap().len(), 4);
    assert_eq!(cap["logic"][0]["duty"], 0.2);
    assert_eq!(cap["timed_out"], false);
    let m = c.call("measure", json!({"capture_id": id, "channel": "D0"}));
    assert_eq!(m["frequency"], 200_000.0);
    assert_eq!(m["duty"], 0.2);
    let f = c.call("find", json!({"capture_id": id, "channel": "D1", "what": "rising"}));
    assert_eq!(f["found"], true);
    let l = c.call("capture_logic", json!({"capture_id": id, "channels": ["D0"], "limit": 3}));
    assert_eq!(l["channels"][0]["transitions"].as_array().unwrap().len(), 3);
    assert_eq!(l["channels"][0]["truncated"], true);
    let a = c.call("capture_analog", json!({"capture_id": id, "channel": "A0", "buckets": 5}));
    assert_eq!(a["buckets"].as_array().unwrap().len(), 5);
    assert_eq!(a["unit"], "V");

    let path = dir.join("demo.sr");
    let e = c.call("export_capture", json!({"capture_id": id, "path": path}));
    assert!(e["bytes"].as_u64().unwrap() > 0);
    let back = c.call("load_capture", json!({"path": path}));
    assert_eq!(back["samples"], 1000);
    assert_eq!(back["logic"][0]["duty"], 0.2);
    let e = c.ok("tools/call", json!({"name": "export_capture", "arguments": {"capture_id": id, "path": "/tmp/elsewhere.sr"}}));
    assert_eq!(e["isError"], true);
    let list = c.call("list_captures", json!({}));
    assert_eq!(list["count"], 2);
    c.call("drop_capture", json!({"capture_id": id}));
    assert_eq!(c.call("close", json!({"handle": handle}))["closed"], handle);
    c.finish();
    std::fs::remove_dir_all(&dir).unwrap();
}

/// The frames an analyzer keeps are the annotations `-P` prints on the
/// same input: same instances, classes, sample ranges and texts. The CLI
/// prints in emission order and the store in start order, so the lines
/// are compared sorted.
#[test]
fn analyzer_frames_match_the_cli_decoder_output() {
    let fixtures = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures");
    let vcd = format!("{fixtures}/i2c-write-read.vcd");
    let out = Command::new(env!("CARGO_BIN_EXE_oxrok"))
        .args(["-l", "4", "-i", &vcd, "-P", "i2c"])
        .output()
        .unwrap();
    assert!(out.status.success());
    let mut cli: Vec<String> =
        String::from_utf8(out.stdout).unwrap().lines().map(str::to_owned).collect();
    cli.sort();
    assert!(cli.len() > 20, "{}", cli.len());

    let mut c = Client::start(&["--allow-path", fixtures]);
    c.initialize();
    let cap = c.call("load_capture", json!({"path": vcd}));
    let id = cap["capture_id"].as_u64().unwrap();
    let dec = c.call(
        "capture_decode",
        json!({"capture_id": id, "stack": "i2c", "keep": true, "limit": 1}),
    );
    assert_eq!(dec["status"], "complete");
    let analyzer = dec["analyzer_id"].as_u64().unwrap();
    let mut ours = Vec::new();
    let mut offset = 0;
    loop {
        let page = c.call(
            "analyzer_frames",
            json!({"analyzer_id": analyzer, "all_texts": true, "limit": 2000, "offset": offset}),
        );
        let frames = page["frames"].as_array().unwrap();
        for f in frames {
            let texts: Vec<String> = f["texts"]
                .as_array()
                .unwrap()
                .iter()
                .map(|t| format!("\"{}\"", t.as_str().unwrap()))
                .collect();
            ours.push(format!(
                "{}-{} {}: {}: {}",
                f["ss"],
                f["es"],
                f["instance"].as_str().unwrap(),
                f["class"].as_str().unwrap(),
                texts.join(" ")
            ));
        }
        offset += frames.len();
        if page["truncated"] != true {
            break;
        }
    }
    ours.sort();
    assert_eq!(ours, cli);
    // The summary counts what the frames show, and values come with the bytes.
    let sum = c.call("analyzer_summary", json!({"analyzer_id": analyzer}));
    assert_eq!(sum["instances"][0]["frames"].as_u64().unwrap() as usize, cli.len());
    let data = c.call(
        "analyzer_frames",
        json!({"analyzer_id": analyzer, "classes": ["data-write"], "radix": "hex", "limit": 5}),
    );
    assert!(data["total"].as_u64().unwrap() > 0, "{data}");
    assert!(data["frames"][0]["value"].is_number(), "{}", data["frames"][0]);
    assert!(data["frames"][0]["text"].as_str().unwrap().contains("0x"), "{}", data["frames"][0]);
    c.finish();
}

#[test]
fn bad_calls_are_protocol_errors() {
    let mut c = Client::start(&[]);
    c.initialize();
    let e = c.request("tools/call", json!({"name": "nope"}));
    assert_eq!(e["error"]["code"], -32602);
    assert_eq!(e["error"]["message"], "Unknown tool 'nope'.");
    let e = c.request("tools/call", json!({"name": "driver_info", "arguments": {"drv": "demo"}}));
    assert_eq!(e["error"]["code"], -32602);
    let e = c.request("resources/list", json!({}));
    assert_eq!(e["error"]["code"], -32601);
    // A tool that ran and found nothing is an error result, not a protocol error.
    let r = c.ok("tools/call", json!({"name": "driver_info", "arguments": {"driver": "nope"}}));
    assert_eq!(r["isError"], true);
    assert_eq!(r["content"][0]["text"], "Unknown driver 'nope'.");
    c.finish();
}

#[test]
fn flags_narrow_the_catalog() {
    let names = |flags: &[&str]| -> Vec<String> {
        let (text, ok) = list_tools(flags);
        assert!(ok, "{text}");
        text.lines().map(|l| l.split_whitespace().next().unwrap().to_owned()).collect()
    };
    assert_eq!(names(&[]), ALL_TOOLS);
    assert_eq!(names(&["--no-cli"]), without(&ALL_TOOLS, &["cli"]));
    assert_eq!(
        names(&["--read-only"]),
        READ_ONLY.iter().map(|s| (*s).to_owned()).collect::<Vec<_>>()
    );
    assert_eq!(names(&["--no-hardware"]), without(&ALL_TOOLS, &HARDWARE));
    assert_eq!(
        names(&["--no-hardware", "--no-cli"]),
        without(&ALL_TOOLS, &["cli", "open", "set_option", "capture"])
    );
    assert_eq!(names(&["--tools", "cli,version"]), ["version", "cli"]);
    let (text, ok) = list_tools(&["--tools", "version,nope"]);
    assert!(!ok);
    assert_eq!(text, "invalid argument: Unknown tool 'nope'.\n");

    // --json is the tools/list result the server would give.
    let (text, ok) = list_tools(&["--json", "--tools", "version"]);
    assert!(ok);
    let listed: Value = serde_json::from_str(&text).unwrap();
    let mut c = Client::start(&["--tools", "version"]);
    c.initialize();
    assert_eq!(c.ok("tools/list", json!({})), listed);
    let e = c.request("tools/call", json!({"name": "cli", "arguments": {"args": []}}));
    assert_eq!(e["error"]["message"], "Unknown tool 'cli'.");
    c.finish();
}

#[test]
fn the_docs_carry_the_tool_reference() {
    let (markdown, ok) = list_tools(&["--markdown"]);
    assert!(ok);
    let path = format!("{}/docs/MCP.md", env!("CARGO_MANIFEST_DIR"));
    let docs = std::fs::read_to_string(&path).unwrap();
    let begin = "<!-- tools:begin -->\n";
    let end = "<!-- tools:end -->";
    let start = docs.find(begin).expect("tools:begin marker") + begin.len();
    let stop = docs[start..].find(end).expect("tools:end marker") + start;
    assert_eq!(
        docs[start..stop].trim_end().to_owned() + "\n",
        markdown.trim_end().to_owned() + "\n",
        "docs/MCP.md is stale: paste the output of `oxrok mcp --list-tools --markdown` between its markers"
    );
}
