// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! `--udev-rules` against the committed `contrib/60-oxrok.rules`, and
//! every driver that opens USB or HID devices claims them.

use oxrok::core::{Registry, UsbClaim};
use std::collections::HashMap;
use std::process::Command;

#[test]
fn committed_rules_are_what_the_drivers_say() {
    let out =
        Command::new(env!("CARGO_BIN_EXE_oxrok")).arg("--udev-rules").output().expect("run oxrok");
    assert!(out.status.success(), "{}", String::from_utf8_lossy(&out.stderr));
    let printed = String::from_utf8(out.stdout).unwrap();
    let committed =
        std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/contrib/60-oxrok.rules"))
            .expect("contrib/60-oxrok.rules");
    assert!(
        printed == committed,
        "contrib/60-oxrok.rules is not what the drivers claim: run `make udev-rules`"
    );
    assert!(printed.contains("ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"dc48\""));
    assert!(printed.contains("ENV{ID_USB_INTERFACES}==\"*:fe0301:*\""));
}

/// Drivers that `docs/drivers.toml` lists as USB but that need no rule:
/// they open a serial port behind the cable, which the tty's group
/// covers, or only what a `conn=` names on a GPIB adapter.
const NO_RULE: [&str; 16] = [
    "agilent-dmm",
    "arachnid-labs-re-load-pro",
    "asix-omega-rtm-cli",
    "hpib-pps",
    "icstation-usbrelay",
    "itech-it8500",
    "juntek-jds6600",
    "link-mso19",
    "mic-98581",
    "mic-98583",
    "ols",
    "rdtech-tc",
    "rdtech-um",
    "rohde-schwarz-sme-0x",
    "uni-t-ut181a",
    "zketech-ebd-usb",
];

/// `connection` per driver id, from `docs/drivers.toml`; empty when the
/// file is not there (a crate checkout has no `docs/`). An `ids` array
/// may run over several lines.
fn connections() -> HashMap<String, String> {
    let Ok(text) =
        std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/docs/drivers.toml"))
    else {
        return HashMap::new();
    };
    let mut out = HashMap::new();
    let (mut ids, mut connection) = (Vec::new(), None);
    let mut in_ids = false;
    let flush =
        |ids: &mut Vec<String>, connection: &mut Option<String>, out: &mut HashMap<_, _>| {
            if let Some(c) = connection.take() {
                for id in ids.drain(..) {
                    let before = out.insert(id.clone(), c.clone());
                    assert!(before.is_none(), "{id} is in two entries of docs/drivers.toml");
                }
            }
            ids.clear();
        };
    for line in text.lines() {
        let line = line.trim();
        let ids_part = if in_ids {
            Some(line)
        } else if let Some(rest) = line.strip_prefix("ids = [") {
            in_ids = true;
            Some(rest)
        } else {
            None
        };
        if let Some(part) = ids_part {
            let (part, closed) = match part.split_once(']') {
                Some((before, _)) => (before, true),
                None => (part.split('#').next().unwrap_or(""), false),
            };
            ids.extend(
                part.split(',')
                    .map(|s| s.trim().trim_matches('"').to_owned())
                    .filter(|s| !s.is_empty()),
            );
            in_ids = !closed;
        } else if line.starts_with("[drivers.") {
            flush(&mut ids, &mut connection, &mut out);
        } else if let Some(rest) = line.strip_prefix("connection = ") {
            connection = Some(rest.trim().trim_matches('"').to_owned());
        }
    }
    flush(&mut ids, &mut connection, &mut out);
    out
}

#[test]
fn every_usb_or_hid_driver_claims_its_devices() {
    let connections = connections();
    if connections.is_empty() {
        return;
    }
    let mut registry = Registry::new();
    oxrok::register_all(&mut registry);
    let mut missing = Vec::new();
    let unlisted: Vec<&str> = registry
        .drivers()
        .map(oxrok::core::Driver::id)
        .filter(|id| !connections.contains_key(*id))
        .collect();
    assert!(unlisted.is_empty(), "drivers with no connection in docs/drivers.toml: {unlisted:?}");
    for d in registry.drivers() {
        let connection = &connections[d.id()];
        if matches!(connection.as_str(), "usb" | "hid")
            && d.usb_claims().is_empty()
            && !NO_RULE.contains(&d.id())
        {
            missing.push(d.id());
        }
        if NO_RULE.contains(&d.id()) {
            assert!(
                d.usb_claims().iter().all(|c| matches!(c, UsbClaim::Class { .. })),
                "{} needs no rule but claims ids",
                d.id()
            );
        }
    }
    assert!(missing.is_empty(), "USB drivers with no usb_claims(): {missing:?}");
}
