// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `ook_oregon`, `ook_vis`, `usb_packet`, `usb_request`, `cfp`
//! and `eeprom93xx` against the unchanged libsigrokdecode Python
//! decoders and the sigrok-test corpus.
//!
//! These tests need the `python` or `rustpython` feature and a
//! libsigrokdecode decoders directory. Without either they are compiled
//! away, so a plain `cargo test` still passes.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{
        Capture, CorpusCase, assert_corpus_parity_def, corpus_config, corpus_dumps_dir,
        corpus_test_dir, load, python, run_chain,
    };
    use oxrok::decode::{DecoderDef, Output};
    use oxrok::decoders::{
        cfp, eeprom93xx, mdio, microwire, ook, ook_oregon, ook_vis, usb_packet, usb_request,
        usb_signalling,
    };

    const OOK_OREGON: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/ook_oregon/oregon_bthr968.sr"
    );
    const OOK_VIS: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/ook_vis/oregon_wgr800.sr");
    const USB_PACKET: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/usb_packet/usb_failed_setup_fullspeed.sr"
    );
    const USB_REQUEST: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/usb_request/usb_failed_setup_fullspeed.sr"
    );
    const CFP: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/cfp/clause45_pluggable_transceiver.sr"
    );
    const EEPROM93XX: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/eeprom93xx/st_m93c66.sr");

    fn available(id: &str) -> bool {
        let found = oxrok::pydecode::defs().iter().any(|d| d.info.id == id);
        if !found {
            eprintln!("no Python decoder '{id}' found, skipping");
        }
        found
    }

    fn case(name: &str, channels: &[(&str, usize)], options: &[(&str, &str)]) -> CorpusCase {
        CorpusCase {
            name: name.to_owned(),
            channels: channels.iter().map(|&(id, n)| (id.to_owned(), n)).collect(),
            options: options.iter().map(|&(k, v)| (k.to_owned(), v.to_owned())).collect(),
            input: None,
            stack: Vec::new(),
        }
    }

    /// Outputs as they can be compared across hosts: an `End` error keeps
    /// only whether it happened.
    fn comparable(output: &Output) -> Output {
        match output {
            Output::End { instance, error } => Output::End {
                instance: instance.clone(),
                error: error.as_ref().map(|_| "<error>".to_owned()),
            },
            _ => output.clone(),
        }
    }

    /// Runs a hand-built decoder chain through both implementations.
    fn assert_chain(cases: &[(&'static DecoderDef, &str, CorpusCase)], capture: &Capture) {
        let native_parts: Vec<(&'static DecoderDef, _)> =
            cases.iter().map(|(def, _, c)| (*def, corpus_config(def.info, c))).collect();
        let python_parts: Vec<(&'static DecoderDef, _)> = cases
            .iter()
            .map(|(_, id, c)| (python(id), corpus_config(python(id).info, c)))
            .collect();
        let native = run_chain(&native_parts, capture);
        let reference = run_chain(&python_parts, capture);
        assert_eq!(
            native.len(),
            reference.len(),
            "output count differs\nnative: {native:#?}\npython: {reference:#?}"
        );
        for (i, (n, p)) in native.iter().zip(&reference).enumerate() {
            assert_eq!(
                comparable(n),
                comparable(p),
                "output {i} differs\nnative: {n:#?}\npython: {p:#?}"
            );
        }
        assert!(!native.is_empty(), "the chain emitted nothing");
    }

    #[test]
    fn decoder_info_matches_python() {
        for (id, def) in [
            ("ook_oregon", &ook_oregon::DEF),
            ("ook_vis", &ook_vis::DEF),
            ("usb_packet", &usb_packet::DEF),
            ("usb_request", &usb_request::DEF),
            ("cfp", &cfp::DEF),
            ("eeprom93xx", &eeprom93xx::DEF),
        ] {
            if !available(id) {
                continue;
            }
            assert_eq!(def.info, python(id).info, "{id}: decoder info differs");
        }
    }

    #[test]
    fn ook_oregon_fixture_parity() {
        if !available("ook_oregon") {
            return;
        }
        let capture = load(OOK_OREGON);
        assert_chain(
            &[
                (&ook::DEF, "ook", case("oregon_bthr968_ook", &[("data", 0)], &[])),
                (&ook_oregon::DEF, "ook_oregon", case("oregon_bthr968", &[], &[])),
            ],
            &capture,
        );
    }

    #[test]
    fn ook_oregon_unknown_option_parity() {
        if !available("ook_oregon") {
            return;
        }
        let capture = load(OOK_OREGON);
        assert_chain(
            &[
                (&ook::DEF, "ook", case("oregon_bthr968_ook_unknown", &[("data", 0)], &[])),
                (
                    &ook_oregon::DEF,
                    "ook_oregon",
                    case("oregon_bthr968_unknown", &[], &[("unknown", "Temp_Hum")]),
                ),
            ],
            &capture,
        );
    }

    #[test]
    fn ook_vis_fixture_parity() {
        if !available("ook_vis") {
            return;
        }
        let capture = load(OOK_VIS);
        assert_chain(
            &[
                (&ook::DEF, "ook", case("oregon_wgr800_ook", &[("data", 0)], &[])),
                (&ook_vis::DEF, "ook_vis", case("oregon_wgr800", &[], &[])),
            ],
            &capture,
        );
    }

    #[test]
    fn ook_vis_reference_parity() {
        if !available("ook_vis") {
            return;
        }
        let capture = load(OOK_VIS);
        assert_chain(
            &[
                (&ook::DEF, "ook", case("oregon_wgr800_ook_ref", &[("data", 0)], &[])),
                (
                    &ook_vis::DEF,
                    "ook_vis",
                    case(
                        "oregon_wgr800_ref",
                        &[],
                        &[("refsample", "show numbers"), ("displayas", "Nibble - Hex rev")],
                    ),
                ),
            ],
            &capture,
        );
    }

    #[test]
    fn usb_packet_fixture_parity() {
        if !available("usb_packet") {
            return;
        }
        let capture = load(USB_PACKET);
        assert_chain(
            &[
                (
                    &usb_signalling::DEF,
                    "usb_signalling",
                    case(
                        "usb_failed_setup_signalling",
                        &[("dp", 1), ("dm", 0)],
                        &[("signalling", "full-speed")],
                    ),
                ),
                (
                    &usb_packet::DEF,
                    "usb_packet",
                    case("usb_failed_setup", &[], &[("signalling", "full-speed")]),
                ),
            ],
            &capture,
        );
    }

    #[test]
    fn usb_request_fixture_parity() {
        if !available("usb_request") {
            return;
        }
        let capture = load(USB_REQUEST);
        assert_chain(
            &[
                (
                    &usb_signalling::DEF,
                    "usb_signalling",
                    case(
                        "usb_failed_setup_signalling_req",
                        &[("dp", 1), ("dm", 0)],
                        &[("signalling", "full-speed")],
                    ),
                ),
                (
                    &usb_packet::DEF,
                    "usb_packet",
                    case("usb_failed_setup_packet", &[], &[("signalling", "full-speed")]),
                ),
                (&usb_request::DEF, "usb_request", case("usb_failed_setup_request", &[], &[])),
            ],
            &capture,
        );
    }

    #[test]
    fn usb_request_first_ack_parity() {
        if !available("usb_request") {
            return;
        }
        let capture = load(USB_REQUEST);
        assert_chain(
            &[
                (
                    &usb_signalling::DEF,
                    "usb_signalling",
                    case(
                        "usb_failed_setup_signalling_first_ack",
                        &[("dp", 1), ("dm", 0)],
                        &[("signalling", "full-speed")],
                    ),
                ),
                (
                    &usb_packet::DEF,
                    "usb_packet",
                    case("usb_failed_setup_packet_first_ack", &[], &[("signalling", "full-speed")]),
                ),
                (
                    &usb_request::DEF,
                    "usb_request",
                    case(
                        "usb_failed_setup_request_first_ack",
                        &[],
                        &[("in_request_start", "first-ack")],
                    ),
                ),
            ],
            &capture,
        );
    }

    #[test]
    fn cfp_fixture_parity() {
        if !available("cfp") {
            return;
        }
        let capture = load(CFP);
        assert_chain(
            &[
                (&mdio::DEF, "mdio", case("clause45_mdio", &[("mdc", 0), ("mdio", 1)], &[])),
                (&cfp::DEF, "cfp", case("clause45_pluggable_transceiver", &[], &[])),
            ],
            &capture,
        );
    }

    #[test]
    fn eeprom93xx_fixture_parity() {
        if !available("eeprom93xx") {
            return;
        }
        let capture = load(EEPROM93XX);
        assert_chain(
            &[
                (
                    &microwire::DEF,
                    "microwire",
                    case("st_m93c66_microwire", &[("cs", 1), ("sk", 3), ("si", 4), ("so", 5)], &[]),
                ),
                (&eeprom93xx::DEF, "eeprom93xx", case("st_m93c66", &[], &[])),
            ],
            &capture,
        );
    }

    #[test]
    fn eeprom93xx_ascii_parity() {
        if !available("eeprom93xx") {
            return;
        }
        let capture = load(EEPROM93XX);
        assert_chain(
            &[
                (
                    &microwire::DEF,
                    "microwire",
                    case(
                        "st_m93c66_microwire_ascii",
                        &[("cs", 1), ("sk", 3), ("si", 4), ("so", 5)],
                        &[],
                    ),
                ),
                (
                    &eeprom93xx::DEF,
                    "eeprom93xx",
                    case("st_m93c66_ascii", &[], &[("format", "ascii")]),
                ),
            ],
            &capture,
        );
    }

    /// True when the sigrok-test corpus should run.
    fn corpus_ready() -> bool {
        if std::env::var_os("OXROK_CORPUS").is_none() {
            eprintln!("set OXROK_CORPUS=1 to run the sigrok-test corpus");
            return false;
        }
        if corpus_test_dir().is_none() || corpus_dumps_dir().is_none() {
            eprintln!("no sigrok-test/sigrok-dumps checkout, skipping corpus");
            return false;
        }
        true
    }

    fn corpus(id: &str, def: &'static DecoderDef) {
        if !available(id) || !corpus_ready() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(def, id);
        eprintln!("{id}: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "{id}: no corpus case ran");
    }

    #[test]
    fn ook_oregon_corpus_parity() {
        corpus("ook_oregon", &ook_oregon::DEF);
    }

    #[test]
    fn ook_vis_corpus_parity() {
        corpus("ook_vis", &ook_vis::DEF);
    }

    #[test]
    fn usb_packet_corpus_parity() {
        corpus("usb_packet", &usb_packet::DEF);
    }

    #[test]
    fn usb_request_corpus_parity() {
        corpus("usb_request", &usb_request::DEF);
    }

    #[test]
    fn cfp_corpus_parity() {
        corpus("cfp", &cfp::DEF);
    }

    #[test]
    fn eeprom93xx_corpus_parity() {
        corpus("eeprom93xx", &eeprom93xx::DEF);
    }
}
