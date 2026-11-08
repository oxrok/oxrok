// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! What stock C printed against what oxrok prints, from the goldens of the one
//! capture with samples that C ends: `spo cut`, a block cut where C's reads run out.
//!
//! The two differ by design, since C drops a waveform's first two samples, so the case
//! can only say that they differ. This says how: C's lines are oxrok's from the third,
//! each the same voltage in the same digits, which is the only comparison of a
//! sample's value this driver has with C.

use std::path::Path;

fn golden(name: &str) -> Vec<String> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/emu/siglent-sds").join(name);
    std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e}", path.display()))
        .lines()
        .filter(|l| l.starts_with("CH1:"))
        .map(str::to_owned)
        .collect()
}

#[test]
fn cs_samples_are_oxroks_from_the_third() {
    let c = golden("spo-cut.oracle");
    let oxrok = golden("spo-cut.out");
    assert_eq!(c.len(), 37, "the block holds 400 bytes, C's first read takes 363");
    assert_eq!(oxrok.len(), 39, "the 400 less a head of 15 and a descriptor of 346");
    assert_eq!(c.as_slice(), &oxrok[2..]);
    let distinct: std::collections::BTreeSet<&String> = c.iter().collect();
    assert!(distinct.len() > 10, "values enough to mean something: {}", distinct.len());
}
