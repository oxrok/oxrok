//! The udev rules for the USB devices the drivers open themselves, from
//! [`Driver::usb_claims`]: what `oxrok --udev-rules` prints and
//! `contrib/60-oxrok.rules` holds.
//!
//! The rules grant nothing: they tag the devices with `ID_OXROK=1`, as
//! libsigrok's `60-libsigrok.rules` tags with `ID_SIGROK`, and one of
//! `contrib/61-oxrok-uaccess.rules` (the seated user, through logind)
//! and `contrib/61-oxrok-plugdev.rules` (group `plugdev`) grants the
//! access. A device by its ids is one `ATTRS{idVendor}`,
//! `ATTRS{idProduct}` line, which udev matches on the device and on its
//! `hidraw` node alike, as `ATTRS` walks up the parents; a class claim
//! is one `ENV{ID_USB_INTERFACES}` line, from the `usb_id` builtin that
//! `50-udev-default.rules` runs on every USB device.

use crate::core::device::{Registry, UsbClaim};
use std::collections::BTreeMap;
use std::fmt::Write as _;

const HEADER: &str = "\
# udev rules for the USB devices oxrok's drivers open themselves.
#
# Generated from the drivers by `oxrok --udev-rules`; `make udev-rules`
# writes contrib/60-oxrok.rules. Do not edit by hand: change the
# driver's `usb_claims` and regenerate.
#
# These rules grant nothing: they tag the devices with ID_OXROK=1. One of
# 61-oxrok-uaccess.rules (the seated user, through logind) and
# 61-oxrok-plugdev.rules (group plugdev) grants the access; install
# this file and one of the two in /etc/udev/rules.d/.
#
# Not here: serial ports behind a USB cable, which the tty's group
# (dialout or uucp) covers.
#
# This file is part of oxrok, GPL-3.0-or-later.

ACTION!=\"add|change\", GOTO=\"oxrok_rules_end\"
SUBSYSTEM!=\"usb|usbmisc|usb_device|hidraw\", GOTO=\"oxrok_rules_end\"
";

/// The names an id is claimed under, in driver order, each with the
/// drivers that use it.
type Names<'a> = Vec<(String, Vec<&'a str>)>;

/// The `60-oxrok.rules` file for the drivers of `registry`: one line per
/// vendor and product id, sorted, each id once with every driver that
/// claims it in the comment above; class claims first.
#[must_use]
pub fn rules(registry: &Registry) -> String {
    let mut ids: BTreeMap<(u16, u16), Names> = BTreeMap::new();
    let mut classes: BTreeMap<(u8, u8, u8), (&str, Vec<&str>)> = BTreeMap::new();
    for d in registry.drivers() {
        for claim in d.usb_claims() {
            match claim {
                UsbClaim::Id { id, vendor, model } => {
                    let name = format!("{vendor} {model}");
                    let names = ids.entry((id.vendor, id.product)).or_default();
                    if let Some((_, drivers)) = names.iter_mut().find(|(n, _)| *n == name) {
                        if !drivers.contains(&d.id()) {
                            drivers.push(d.id());
                        }
                    } else {
                        names.push((name, vec![d.id()]));
                    }
                }
                UsbClaim::Class { class, subclass, protocol, name } => {
                    let drivers = &mut classes
                        .entry((class, subclass, protocol))
                        .or_insert((name, Vec::new()))
                        .1;
                    if !drivers.contains(&d.id()) {
                        drivers.push(d.id());
                    }
                }
            }
        }
    }
    let mut out = String::from(HEADER);
    for ((class, subclass, protocol), (name, drivers)) in &classes {
        let _ = writeln!(out, "\n# {name}, by interface class: {}", drivers.join(", "));
        let _ = writeln!(
            out,
            "ENV{{ID_USB_INTERFACES}}==\"*:{class:02x}{subclass:02x}{protocol:02x}:*\", ENV{{ID_OXROK}}=\"1\""
        );
    }
    for ((vendor, product), names) in &ids {
        let comment: Vec<String> =
            names.iter().map(|(name, drivers)| format!("{name} ({})", some_of(drivers))).collect();
        let _ = writeln!(out, "\n# {}", comment.join("; "));
        let _ = writeln!(
            out,
            "ATTRS{{idVendor}}==\"{vendor:04x}\", ATTRS{{idProduct}}==\"{product:04x}\", ENV{{ID_OXROK}}=\"1\""
        );
    }
    out.push_str("\nLABEL=\"oxrok_rules_end\"\n");
    out
}

/// Up to three driver ids, then how many more: the meter families
/// register a driver per model.
fn some_of(drivers: &[&str]) -> String {
    match drivers {
        [a, b, c, rest @ ..] if !rest.is_empty() => {
            format!("{a}, {b}, {c} and {} more", rest.len())
        }
        _ => drivers.join(", "),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{Device, DeviceClass, Driver, Key, Result, Value};

    struct Fake(&'static str, Vec<UsbClaim>);

    impl Driver for Fake {
        fn id(&self) -> &'static str {
            self.0
        }
        fn long_name(&self) -> &'static str {
            "a fake"
        }
        fn classes(&self) -> &'static [DeviceClass] {
            &[]
        }
        fn scan(&self, _: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
            Ok(Vec::new())
        }
        fn usb_claims(&self) -> Vec<UsbClaim> {
            self.1.clone()
        }
    }

    #[test]
    fn ids_are_sorted_and_shared_ids_are_one_line() {
        let mut r = Registry::new();
        r.register(Box::new(Fake(
            "b-driver",
            vec![
                UsbClaim::ids(0x0403, 0x6001, "FTDI", "FT232R"),
                UsbClaim::ids(0x0403, 0x6001, "FTDI", "FT232R"),
            ],
        )));
        r.register(Box::new(Fake(
            "a-driver",
            vec![
                UsbClaim::ids(0x0403, 0x6001, "ChronoVu", "LA8 (old ids)"),
                UsbClaim::ids(0x0100, 0xffff, "X", "Y"),
                UsbClaim::Class { class: 0xfe, subclass: 3, protocol: 1, name: "USBTMC" },
            ],
        )));
        r.register(Box::new(Fake("none", Vec::new())));
        for id in ["m1", "m2", "m3", "m4", "m5"] {
            r.register(Box::new(Fake(id, vec![UsbClaim::ids(0x0820, 0x0001, "Brymen", "BU-86X")])));
        }
        let text = rules(&r);
        let lines: Vec<&str> = text.lines().filter(|l| !l.is_empty()).collect();
        assert_eq!(lines[0], "# udev rules for the USB devices oxrok's drivers open themselves.");
        let body: Vec<&str> =
            lines.iter().copied().skip_while(|l| !l.starts_with("ACTION")).collect();
        assert_eq!(
            body,
            [
                "ACTION!=\"add|change\", GOTO=\"oxrok_rules_end\"",
                "SUBSYSTEM!=\"usb|usbmisc|usb_device|hidraw\", GOTO=\"oxrok_rules_end\"",
                "# USBTMC, by interface class: a-driver",
                "ENV{ID_USB_INTERFACES}==\"*:fe0301:*\", ENV{ID_OXROK}=\"1\"",
                "# X Y (a-driver)",
                "ATTRS{idVendor}==\"0100\", ATTRS{idProduct}==\"ffff\", ENV{ID_OXROK}=\"1\"",
                "# ChronoVu LA8 (old ids) (a-driver); FTDI FT232R (b-driver)",
                "ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6001\", ENV{ID_OXROK}=\"1\"",
                "# Brymen BU-86X (m1, m2, m3 and 2 more)",
                "ATTRS{idVendor}==\"0820\", ATTRS{idProduct}==\"0001\", ENV{ID_OXROK}=\"1\"",
                "LABEL=\"oxrok_rules_end\"",
            ]
        );
    }

    #[test]
    fn no_claims_is_a_file_with_no_ids() {
        let text = rules(&Registry::new());
        assert!(text.ends_with("GOTO=\"oxrok_rules_end\"\n\nLABEL=\"oxrok_rules_end\"\n"));
        assert!(!text.contains("ATTRS"));
    }
}
