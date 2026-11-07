// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `onewire_link` and `ir_nec` against the unchanged
//! libsigrokdecode Python decoders and the sigrok-test corpus.
//!
//! These tests need the `python` or `rustpython` feature and a
//! libsigrokdecode decoders directory. Without either they are compiled
//! away, so a plain `cargo test` still passes.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{
        Capture, CorpusCase, assert_corpus_parity_def, assert_parity_def, corpus_config, load,
        python, run,
    };
    use oxrok::decode::DecoderDef;
    use oxrok::decoders::{ir_nec, onewire_link};

    const ONEWIRE: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/onewire_link/owdir.sr");
    const IR_NEC: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/ir_nec/tv_matsui_0001_button_av_hold.sr"
    );

    fn available(id: &str) -> bool {
        let found = oxrok::pydecode::defs().iter().any(|d| d.info.id == id);
        if !found {
            eprintln!("no Python decoder '{id}' found, skipping");
        }
        found
    }

    /// Runs a hand-built corpus case through both implementations.
    fn assert_case(
        def: &'static DecoderDef,
        python_id: &str,
        case: &CorpusCase,
        capture: &Capture,
    ) {
        let cfg = corpus_config(def.info, case);
        let native = run(def, &cfg, capture);
        let reference = run(python(python_id), &cfg, capture);
        assert_eq!(
            native.len(),
            reference.len(),
            "{}: output count differs\nnative: {native:#?}\npython: {reference:#?}",
            case.name
        );
        for (i, (n, p)) in native.iter().zip(&reference).enumerate() {
            assert_eq!(n, p, "{}: output {i} differs\nnative: {n:#?}\npython: {p:#?}", case.name);
        }
    }

    #[test]
    fn onewire_link_fixture_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("onewire_link") {
            return;
        }
        let capture = load(ONEWIRE);
        let case = CorpusCase {
            name: "owdir".to_owned(),
            // The channel and index from sigrok-test's test.conf.
            channels: vec![("owr".to_owned(), 0)],
            options: Vec::new(),
            input: None,
            stack: Vec::new(),
        };
        assert_case(&onewire_link::DEF, "onewire_link", &case, &capture);
    }

    #[test]
    fn onewire_link_overdrive_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("onewire_link") {
            return;
        }
        let capture = load(ONEWIRE);
        let name = capture.channels[0].clone();
        assert_parity_def(
            &onewire_link::DEF,
            "onewire_link",
            &[("owr", name.as_str()), ("overdrive", "yes")],
            &capture,
        );
    }

    #[test]
    fn ir_nec_fixture_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("ir_nec") {
            return;
        }
        let capture = load(IR_NEC);
        let case = CorpusCase {
            name: "tv_matsui_0001_button_av_hold".to_owned(),
            // The channel and index from sigrok-test's test.conf.
            channels: vec![("ir".to_owned(), 6)],
            options: Vec::new(),
            input: None,
            stack: Vec::new(),
        };
        assert_case(&ir_nec::DEF, "ir_nec", &case, &capture);
    }

    #[test]
    fn ir_nec_carrier_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("ir_nec") {
            return;
        }
        let capture = load(IR_NEC);
        let name = capture.channels[6].clone();
        assert_parity_def(
            &ir_nec::DEF,
            "ir_nec",
            &[("ir", name.as_str()), ("cd_freq", "38000")],
            &capture,
        );
    }

    #[test]
    fn onewire_link_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("onewire_link") {
            return;
        }
        if std::env::var_os("OXROK_CORPUS").is_none() {
            eprintln!("set OXROK_CORPUS=1 to run the sigrok-test corpus");
            return;
        }
        if crate::support::corpus_test_dir().is_none()
            || crate::support::corpus_dumps_dir().is_none()
        {
            eprintln!("no sigrok-test/sigrok-dumps checkout, skipping corpus");
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&onewire_link::DEF, "onewire_link");
        eprintln!("onewire_link: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "onewire_link: no corpus case ran");
    }

    #[test]
    fn ir_nec_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("ir_nec") {
            return;
        }
        if std::env::var_os("OXROK_CORPUS").is_none() {
            eprintln!("set OXROK_CORPUS=1 to run the sigrok-test corpus");
            return;
        }
        if crate::support::corpus_test_dir().is_none()
            || crate::support::corpus_dumps_dir().is_none()
        {
            eprintln!("no sigrok-test/sigrok-dumps checkout, skipping corpus");
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&ir_nec::DEF, "ir_nec");
        eprintln!("ir_nec: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "ir_nec: no corpus case ran");
    }
}
