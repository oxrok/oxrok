// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! What `cargo publish` would send to crates.io, from `cargo package
//! --list`: nothing outside the places named here, and the files outside
//! `src/` that a test of the crate reads.

use std::path::Path;
use std::process::Command;

/// Where a packaged file may be. A new top level file or directory is
/// published only once it is here and in `include` in `Cargo.toml`.
const ALLOWED: [&str; 14] = [
    // Written by cargo.
    ".cargo_vcs_info.json",
    "Cargo.lock",
    "Cargo.toml",
    "Cargo.toml.orig",
    "CHANGELOG.md",
    "LICENSE",
    "NOTICE",
    "README.md",
    "contrib/",
    "examples/",
    "src/",
    "tests/fixtures/",
    "tests/support/",
    "tools/emu/",
];

/// What the tests of a crate checkout read outside `src/` and
/// `tests/fixtures/`: an `include` that loses one fails them only there.
const NEEDED: [&str; 3] =
    ["contrib/60-oxrok.rules", "tests/support/mod.rs", "tools/emu/fx2lafw.firmware"];

#[test]
fn the_crate_is_what_include_names() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    // Only in a checkout of the repository: an unpacked crate or source
    // tarball is already the list, and has no git to ask.
    if !root.join(".git").exists() {
        return;
    }
    // --allow-dirty for the changes of a working tree, which --list only
    // names. Nothing is built or fetched.
    let out = Command::new(env!("CARGO"))
        .args(["package", "--list", "--allow-dirty", "--offline", "--manifest-path"])
        .arg(root.join("Cargo.toml"))
        .output()
        .expect("run cargo package --list");
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
    let listed = String::from_utf8(out.stdout).unwrap().replace('\\', "/");
    let files: Vec<&str> = listed.lines().collect();
    let stray: Vec<&&str> = files
        .iter()
        .filter(|f| {
            // The tests themselves, which are files of tests/ and no deeper.
            let test = Path::new(f).parent() == Some(Path::new("tests"))
                && Path::new(f).extension().is_some_and(|e| e == "rs");
            let allowed = ALLOWED.iter().any(|a| match a.strip_suffix('/') {
                Some(_) => f.starts_with(a),
                None => *f == a,
            });
            !(allowed || test)
        })
        .collect();
    assert!(
        stray.is_empty(),
        "published to crates.io but not in tests/package.rs: {stray:?}; see `include` in Cargo.toml"
    );
    for needed in NEEDED {
        assert!(
            files.contains(&needed),
            "{needed} is not in the crate: see `include` in Cargo.toml"
        );
    }
    // The agent and plan files, and a local configuration, by name.
    for private in ["AGENTS.md", "PLAN.mcp.md", "opencode.json", ".mcp.json"] {
        assert!(!files.contains(&private), "{private} would be published");
    }
}
