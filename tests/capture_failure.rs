// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! How a capture's failure gets out: every driver ends through
//! `PacketSink::end`, and the tool's exit status says what happened.

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

/// The `.rs` files under `dir`, however deep.
fn sources(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(dir).unwrap() {
        let path = entry.unwrap().path();
        if path.is_dir() {
            sources(&path, out);
        } else if path.extension().is_some_and(|e| e == "rs") {
            out.push(path);
        }
    }
}

/// A driver that sends its own end has nowhere to say that it failed. Tests
/// may: they play the part of a device.
#[test]
fn no_driver_sends_its_end_by_hand() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src/drivers");
    let mut files = Vec::new();
    sources(&root, &mut files);
    assert!(files.len() > 40, "{} files under {}", files.len(), root.display());
    let mut found = Vec::new();
    for file in files {
        let text = std::fs::read_to_string(&file).unwrap();
        let code = text.split("#[cfg(test)]").next().unwrap();
        for (i, line) in code.lines().enumerate() {
            let line = line.trim_start();
            if !line.starts_with("//") && line.contains("Packet::End") {
                found.push(format!("{}:{}: {line}", file.display(), i + 1));
            }
        }
    }
    assert!(found.is_empty(), "use PacketSink::end:\n{}", found.join("\n"));
}

/// An output that cannot be written ends the capture, which then failed.
#[cfg(target_os = "linux")]
#[test]
fn an_output_that_cannot_be_written_fails_the_capture() {
    let full = std::fs::OpenOptions::new().write(true).open("/dev/full").unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_oxrok"))
        .args(["-d", "demo", "--samples", "8", "-O", "hex"])
        .stdin(Stdio::null())
        .stdout(full)
        .stderr(Stdio::piped())
        .output()
        .unwrap();
    let err = String::from_utf8_lossy(&out.stderr);
    assert_eq!(out.status.code(), Some(1), "{err}");
    assert!(err.lines().any(|l| l.starts_with("capture failed: output: ")), "{err}");
}

/// A capture that met its limit exits 0 and says nothing of the kind.
#[test]
fn a_capture_that_meets_its_limit_succeeds() {
    let out = Command::new(env!("CARGO_BIN_EXE_oxrok"))
        .args(["-d", "demo", "--samples", "8", "-O", "hex"])
        .stdin(Stdio::null())
        .output()
        .unwrap();
    let err = String::from_utf8_lossy(&out.stderr);
    assert_eq!(out.status.code(), Some(0), "{err}");
    assert!(!err.contains("capture failed"), "{err}");
}
