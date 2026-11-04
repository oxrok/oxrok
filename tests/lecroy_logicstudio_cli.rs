// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! `lecroy-logicstudio` through the command line, on a hand written replay:
//! what a refused capture leaves behind.

use std::path::Path;
use std::process::Command;

/// A board with its firmware: the claim, and nothing after it.
const FIXTURE: &str = "# oxrok usb fixture 1\ndevice vid=05ff pid=a002\nclaim 0 -> ok\n";

/// With no trigger the capture is refused before the output is opened, so
/// an existing file keeps its contents and no new file appears.
#[test]
fn a_refused_capture_leaves_the_output_file_alone() {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("lecroy-refused");
    std::fs::create_dir_all(&dir).unwrap();
    let fixture = dir.join("fixture.txt");
    std::fs::write(&fixture, FIXTURE).unwrap();
    let existing = dir.join("existing.sr");
    let fresh = dir.join("fresh.sr");
    let _ = std::fs::remove_file(&fresh);
    std::fs::write(&existing, b"a capture worth keeping").unwrap();
    for out in [&existing, &fresh] {
        let run = Command::new(env!("CARGO_BIN_EXE_oxrok"))
            .args(["-d", "lecroy-logicstudio", "-o"])
            .arg(out)
            .env("OXROK_USB_REPLAY", &fixture)
            .output()
            .unwrap();
        assert!(!run.status.success());
        let err = String::from_utf8_lossy(&run.stderr);
        assert!(err.contains("captures on its trigger alone"), "{err}");
    }
    assert_eq!(std::fs::read(&existing).unwrap(), b"a capture worth keeping");
    assert!(!fresh.exists());
}
