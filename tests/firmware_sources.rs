//! Every firmware file a driver loads has a source in the registry, so a
//! missing file can always be fetched, or explained.

use oxrok::core::firmware::fetch;
use oxrok::core::{FirmwareKind, Registry};

#[test]
fn every_uploaded_file_has_a_source() {
    let mut registry = Registry::new();
    oxrok::register_all(&mut registry);
    let mut unknown = Vec::new();
    for driver in registry.drivers() {
        for dep in driver.firmware().into_iter().filter(|d| d.kind == FirmwareKind::Upload) {
            for file in &dep.files {
                if fetch::package_for(file).is_none() {
                    unknown.push(format!("{}: {file}", driver.id()));
                }
            }
        }
    }
    assert!(unknown.is_empty(), "files without a source in oxrok-firmware-fetch: {unknown:#?}");
}

#[test]
fn every_package_says_how_to_get_it() {
    for p in fetch::packages() {
        assert!(
            p.downloadable() || p.manual.is_some(),
            "{} has neither downloads nor instructions",
            p.id
        );
        assert!(!p.files.is_empty(), "{} installs nothing", p.id);
    }
}

#[test]
fn package_ids_and_files_are_unique() {
    let mut ids = std::collections::HashSet::new();
    let mut files = std::collections::HashSet::new();
    for p in fetch::packages() {
        assert!(ids.insert(p.id), "id {} twice", p.id);
        for f in p.files {
            assert!(files.insert(*f), "{f} in two packages");
        }
    }
}
