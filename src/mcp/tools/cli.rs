// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! `cli`: the sigrok-cli pass-through.
//!
//! The first tool worth having and the last one to remove: models know
//! sigrok-cli's flags, so this gives an agent the whole command line on
//! day one, and stays the escape hatch for what the structured tools do
//! not cover. It runs the executable as a subprocess with a timeout and
//! a kill, so a driver that hangs does not hang the server.

use crate::mcp::Host;
use crate::mcp::tool::{
    Annotations, Tag, Tool, ToolError, ToolResult, ToolSpec, function, seconds,
};
use schemars::JsonSchema;
use serde::Deserialize;
use serde_json::json;
use std::io::Read;
use std::path::Path;
use std::process::{Command, Stdio};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

/// `cli`.
#[derive(Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CliArgs {
    /// The arguments as sigrok-cli takes them, one per element:
    /// `["-d", "demo", "--samples", "100", "-O", "bits"]`.
    pub args: Vec<String>,
    /// Give up after this many seconds and kill the command. The
    /// server's limit otherwise.
    #[serde(default)]
    pub timeout: Option<f64>,
    /// Keep at most this many bytes of stdout and of stderr each. The
    /// server's limit otherwise.
    #[serde(default)]
    pub max_bytes: Option<usize>,
}

/// What a run gave.
#[derive(Debug, PartialEq)]
pub struct CliOutput {
    /// The exit code, `None` when killed by a signal or the timeout.
    pub exit_code: Option<i32>,
    /// Standard output, UTF-8 with replacement characters.
    pub stdout: String,
    /// Standard error, the same.
    pub stderr: String,
    /// Whether stdout was cut at the limit.
    pub stdout_truncated: bool,
    /// Whether stderr was cut at the limit.
    pub stderr_truncated: bool,
    /// Whether the command was killed at the timeout.
    pub timed_out: bool,
    /// How long it ran.
    pub seconds: f64,
}

/// The tool over `host`.
pub fn tool(host: &Arc<Host>) -> Arc<dyn Tool> {
    let h = host.clone();
    function::<CliArgs, _>(
        ToolSpec {
            name: "cli",
            title: "Run the command line",
            description: "Runs the oxrok command line, which takes sigrok-cli's arguments, and \
                          returns its stdout, stderr and exit code. Useful flags: `-d DRIVER` \
                          (`--scan` to find devices, `--show` for a device's options), \
                          `--samples N` or `--time MS`, `-c samplerate=1m`, `-C D0,D1`, \
                          `-t D0=r`, `-O bits|csv|vcd|srzip`, `-o FILE`, `-i FILE` with \
                          `-I FORMAT`, `-P i2c:scl=D0:sda=D1` with `-A i2c=address` for a \
                          decoder, `-L` for everything supported. Keep outputs small: a sample \
                          limit and a decoder beat raw samples. The `demo` driver needs no \
                          hardware. Output beyond the limit is cut and flagged.",
            example: json!({"args": ["-d", "demo:logic_channels=2", "--samples", "8", "-O", "bits"]}),
            annotations: Annotations::HARDWARE,
            tags: &[Tag::Cli],
        },
        move |args, _| {
            let program = h.cli_program().ok_or_else(|| {
                ToolError::new("The server does not know which executable to run.")
            })?;
            if args.args.first().is_some_and(|a| a == "mcp") {
                return Err(ToolError::invalid_args("The cli tool does not start another server."));
            }
            if let Some(driver) = driver_arg(&args.args)
                && h.open_drivers().iter().any(|d| d == driver)
            {
                return Err(ToolError::new(format!(
                    "A {driver} device is open on this server; use the device tools on its \
                     handle, or `close` it first. Two owners of one device do not work."
                )));
            }
            let limits = h.limits();
            // A call may ask for less than the server allows, never more.
            let timeout = match args.timeout {
                Some(t) => seconds(t, "timeout")?.min(limits.cli_timeout),
                None => limits.cli_timeout,
            };
            let max_bytes =
                args.max_bytes.map_or(limits.cli_max_bytes, |m| m.min(limits.cli_max_bytes));
            let out = run(program, &args.args, timeout, max_bytes)?;
            Ok(ToolResult::structured(json!({
                "exit_code": out.exit_code,
                "stdout": out.stdout,
                "stderr": out.stderr,
                "stdout_truncated": out.stdout_truncated,
                "stderr_truncated": out.stderr_truncated,
                "timed_out": out.timed_out,
                "seconds": out.seconds,
            })))
        },
    )
}

/// The driver id of a `-d`/`--driver` argument, when there is one.
fn driver_arg(args: &[String]) -> Option<&str> {
    let mut it = args.iter();
    while let Some(a) = it.next() {
        let spec = match a.as_str() {
            "-d" | "--driver" => it.next()?.as_str(),
            s if s.starts_with("--driver=") => &s["--driver=".len()..],
            s if s.starts_with("-d") && s.len() > 2 => &s[2..],
            _ => continue,
        };
        return Some(spec.split(':').next().unwrap_or(spec));
    }
    None
}

/// Runs `program` with `args`, stdin closed, for at most `timeout`, keeping
/// `max_bytes` of each output stream.
pub fn run(
    program: &Path,
    args: &[String],
    timeout: Duration,
    max_bytes: usize,
) -> Result<CliOutput, ToolError> {
    let start = Instant::now();
    let mut child = Command::new(program)
        .args(args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| ToolError::new(format!("Cannot run {}: {e}", program.display())))?;
    let stdout = child.stdout.take().map(|r| thread::spawn(move || read_capped(r, max_bytes)));
    let stderr = child.stderr.take().map(|r| thread::spawn(move || read_capped(r, max_bytes)));
    let mut timed_out = false;
    let status = loop {
        if let Some(status) = child.try_wait()? {
            break Some(status);
        }
        if start.elapsed() >= timeout {
            let _ = child.kill();
            let _ = child.wait();
            timed_out = true;
            break None;
        }
        thread::sleep(Duration::from_millis(10));
    };
    let (stdout, stdout_truncated) = join(stdout);
    let (stderr, stderr_truncated) = join(stderr);
    Ok(CliOutput {
        exit_code: status.and_then(|s| s.code()),
        stdout,
        stderr,
        stdout_truncated,
        stderr_truncated,
        timed_out,
        seconds: start.elapsed().as_secs_f64(),
    })
}

fn join(handle: Option<thread::JoinHandle<(Vec<u8>, bool)>>) -> (String, bool) {
    let (bytes, truncated) = handle.and_then(|h| h.join().ok()).unwrap_or_default();
    (String::from_utf8_lossy(&bytes).into_owned(), truncated)
}

/// Reads everything, keeps the first `max` bytes, and says whether more
/// came. The rest is read and dropped so the child never blocks on a
/// full pipe.
fn read_capped(mut r: impl Read, max: usize) -> (Vec<u8>, bool) {
    let mut kept = Vec::new();
    let mut truncated = false;
    let mut buf = [0u8; 8192];
    loop {
        match r.read(&mut buf) {
            Ok(0) | Err(_) => break,
            Ok(n) => {
                let room = max.saturating_sub(kept.len());
                if n > room {
                    truncated = true;
                }
                kept.extend_from_slice(&buf[..n.min(room)]);
            }
        }
    }
    (kept, truncated)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_the_driver_argument() {
        let s = |v: &[&str]| v.iter().map(|s| (*s).to_owned()).collect::<Vec<_>>();
        assert_eq!(driver_arg(&s(&["-d", "demo:logic_channels=2", "--scan"])), Some("demo"));
        assert_eq!(driver_arg(&s(&["--driver", "fx2lafw"])), Some("fx2lafw"));
        assert_eq!(driver_arg(&s(&["--driver=fx2lafw:conn=1.4"])), Some("fx2lafw"));
        assert_eq!(driver_arg(&s(&["-ddemo"])), Some("demo"));
        assert_eq!(driver_arg(&s(&["-L"])), None);
        assert_eq!(driver_arg(&s(&["-d"])), None);
    }

    #[test]
    fn caps_what_it_keeps() {
        let (kept, truncated) = read_capped(&b"abcdef"[..], 4);
        assert_eq!((kept.as_slice(), truncated), (&b"abcd"[..], true));
        let (kept, truncated) = read_capped(&b"abc"[..], 4);
        assert_eq!((kept.as_slice(), truncated), (&b"abc"[..], false));
    }

    #[cfg(unix)]
    #[test]
    fn runs_times_out_and_truncates() {
        let sh = Path::new("/bin/sh");
        let args = |s: &str| vec!["-c".to_owned(), s.to_owned()];
        let out =
            run(sh, &args("echo out; echo err >&2; exit 3"), Duration::from_secs(5), 1000).unwrap();
        assert_eq!(out.exit_code, Some(3));
        assert_eq!(out.stdout, "out\n");
        assert_eq!(out.stderr, "err\n");
        assert!(!out.timed_out && !out.stdout_truncated);
        // The command would run 30 s; the timeout kills it, however slow
        // the machine is about spawning and reaping.
        let out = run(sh, &args("sleep 30"), Duration::from_millis(100), 1000).unwrap();
        assert!(out.timed_out);
        assert_eq!(out.exit_code, None);
        assert!(out.seconds < 20.0, "{}", out.seconds);
        let out = run(sh, &args("printf 123456789"), Duration::from_secs(5), 4).unwrap();
        assert_eq!(out.stdout, "1234");
        assert!(out.stdout_truncated);
        assert!(run(Path::new("/nonexistent/x"), &[], Duration::from_secs(1), 10).is_err());
    }
}
