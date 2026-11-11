//! The flow a front end runs when a driver misses firmware, end to end
//! through the library: the miss is noted, grouped into what a dialog
//! shows, the user's files are imported with progress, and the driver's
//! lookup then succeeds. A test binary of its own: it sets the
//! environment.

use oxrok::core::firmware::{self, fetch};
use std::cell::RefCell;

#[derive(Default)]
struct Log(RefCell<Vec<String>>);

impl fetch::Progress for Log {
    fn message(&self, msg: &str) {
        self.0.borrow_mut().push(msg.to_owned());
    }
}

#[test]
fn a_missed_file_is_offered_imported_and_then_found() {
    // A scratch tree of this process's own, removed at the end.
    let root = std::env::temp_dir().join(format!("oxrok-firmware-flow-{}", std::process::id()));
    let (install, home, given) = (root.join("install"), root.join("home"), root.join("given"));
    for d in [&install, &home, &given] {
        std::fs::create_dir_all(d).unwrap();
    }
    // SAFETY: this test binary runs this one test; no other thread reads
    // the environment.
    unsafe {
        std::env::set_var("OXROK_FIRMWARE_INSTALL_DIR", &install);
        std::env::set_var("HOME", &home);
        std::env::set_var("XDG_DATA_DIRS", &home);
        for var in
            ["OXROK_FIRMWARE_DIR", "SIGROK_FIRMWARE_DIR", "SIGROK_FIRMWARE_PATH", "XDG_DATA_HOME"]
        {
            std::env::remove_var(var);
        }
    }
    let file = "fx2lafw-usb-c-grok.fw";

    // A driver looks the file up and misses it.
    assert!(firmware::read(file, "a hint").is_err());
    let missing = firmware::take_missing_grouped();
    assert_eq!(missing.len(), 1);
    let pkg = missing[0].package.expect("a known source");
    assert_eq!(missing[0].files, vec![file]);
    assert!(!missing[0].downloadable(), "built from source: the user provides it");
    assert!(pkg.manual.is_some(), "a dialog has instructions to show");
    assert!(!firmware::package_status(pkg).complete());

    // The user picks a folder holding the files; the front end imports it.
    for f in pkg.files {
        std::fs::write(given.join(f), b"firmware").unwrap();
    }
    let log = Log::default();
    let dest = firmware::install_dir().unwrap();
    assert_eq!(dest, install);
    let installed = fetch::import(pkg, std::slice::from_ref(&given), &dest, &log).unwrap();
    assert_eq!(installed.files.len(), pkg.files.len());
    assert!(log.0.borrow().iter().any(|m| m.starts_with("Installed ")));

    // Now the driver finds it, and nothing is missing.
    assert!(firmware::package_status(pkg).complete());
    assert_eq!(firmware::read(file, "a hint").unwrap(), b"firmware");
    assert_eq!(firmware::take_missing(), Vec::<String>::new());
    std::fs::remove_dir_all(&root).unwrap();
}
