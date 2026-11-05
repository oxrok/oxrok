// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later

//! `scpi::open_keeping_line` records as `scpi::open` does. `OXROK_SCPI_RECORD` is
//! the process's, so the part that needs it runs in a child of this test. It has a
//! test binary of its own because the fork matters: a child inherits the serial
//! port's `flock` until its exec, and another test's open of its pty then fails.

#![cfg(target_os = "linux")]

use oxrok::scpi::{Resource, open_keeping_line};
use rustix::pty::{OpenptFlags, grantpt, openpt, ptsname, unlockpt};

const VAR: &str = "OXROK_SCPI_RECORD";

#[test]
fn a_kept_line_records_too() {
    if std::env::var(VAR).is_ok_and(|f| !f.is_empty()) {
        let master = openpt(OpenptFlags::RDWR | OpenptFlags::NOCTTY).unwrap();
        grantpt(&master).unwrap();
        unlockpt(&master).unwrap();
        let slave = ptsname(&master, Vec::new()).unwrap();
        let slave = slave.to_str().unwrap().to_owned();
        let mut s = open_keeping_line(&Resource::Serial(slave), None).unwrap();
        s.send("X").unwrap();
        return;
    }
    let file = std::env::temp_dir().join(format!("oxrok-kept-{}.txt", std::process::id()));
    let status = std::process::Command::new(std::env::current_exe().unwrap())
        .args(["--exact", "a_kept_line_records_too"])
        .env(VAR, &file)
        .status()
        .unwrap();
    assert!(status.success());
    let text = std::fs::read_to_string(&file).unwrap();
    std::fs::remove_file(&file).unwrap();
    assert!(text.contains("send \"X\\n\""), "{text}");
}
