// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! How a capture's failure gets out: every driver ends through
//! `PacketSink::end`, and the tool's exit status says what happened.

#[cfg(unix)]
use std::io::Read as _;
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
    // A test module in a file of its own.
    files.retain(|file| !file.ends_with("tests.rs"));
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

/// `oxrok ... | head`: a reader that goes stops the capture, and nothing failed.
#[cfg(unix)]
#[test]
fn a_reader_that_goes_is_no_failure() {
    let mut child = Command::new(env!("CARGO_BIN_EXE_oxrok"))
        .args(["-d", "demo", "--continuous", "-O", "hex"])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdout = child.stdout.take().unwrap();
    stdout.read_exact(&mut [0; 16]).unwrap();
    drop(stdout);
    // Nothing else ends this capture: a test that hangs here is the bug.
    let out = child.wait_with_output().unwrap();
    let err = String::from_utf8_lossy(&out.stderr);
    assert_eq!(out.status.code(), Some(0), "{err}");
    assert!(!err.contains("capture failed"), "{err}");
}

/// The replay of a capture that fails after its data, with an output of its
/// own in place of the recorded one: it says why, and the output file is whole.
fn failed_capture_into(format: &str, file: &Path) {
    let fixture = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/saleae_logic_pro/emu-recorded-capture.txt");
    let out = Command::new(env!("CARGO_BIN_EXE_oxrok"))
        .args(["-d", "saleae-logic-pro", "-C", "0,1,2,3", "--continuous", "-O", format, "-o"])
        .arg(file)
        .env("OXROK_USB_REPLAY", &fixture)
        .stdin(Stdio::null())
        .output()
        .unwrap();
    let err = String::from_utf8_lossy(&out.stderr);
    assert_eq!(out.status.code(), Some(1), "{err}");
    assert_eq!(err.lines().last(), Some("capture failed: device error: no transfer for 1 s"));
}

#[test]
fn a_failed_capture_finishes_its_output_file() {
    let dir = std::env::temp_dir().join(format!("oxrok-capture-failure-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    // What the session file holds is all the capture sent.
    let session = dir.join("failed.sr");
    failed_capture_into("srzip", &session);
    let hex = Command::new(env!("CARGO_BIN_EXE_oxrok"))
        .arg("-i")
        .arg(&session)
        .args(["-O", "hex", "-C", "0"])
        .output()
        .unwrap();
    assert!(hex.status.success(), "{}", String::from_utf8_lossy(&hex.stderr));
    assert!(hex.stdout.len() > 1000, "{} bytes", hex.stdout.len());
    // A VCD ends with the time of the sample after the last.
    let vcd = dir.join("failed.vcd");
    failed_capture_into("vcd", &vcd);
    let text = std::fs::read_to_string(&vcd).unwrap();
    assert!(text.lines().last().is_some_and(|l| l.starts_with('#')), "{}", text.len());
    std::fs::remove_dir_all(&dir).unwrap();
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
