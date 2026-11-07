// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `gpib` and `ieee488` against the unchanged libsigrokdecode
//! Python decoders and the sigrok-test corpus.
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
    use oxrok::decoders::{gpib, ieee488};

    const GPIB: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/gpib/hp33120a-idn.sr");
    const IEEE488: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/ieee488/hp33120a-idn.sr");

    /// The channel ids and numeric indexes from sigrok-test's
    /// `decoder/test/ieee488/test.conf`; `gpib` has the same 16 channels
    /// in the same order.
    const CHANNELS: [&str; 16] = [
        "dio1", "dio2", "dio3", "dio4", "dio5", "dio6", "dio7", "dio8", "eoi", "dav", "nrfd",
        "ndac", "ifc", "srq", "atn", "ren",
    ];

    /// The same channels by their logic names, for `assert_parity_def`.
    const PAIRS: [(&str, &str); 16] = [
        ("dio1", "DIO1"),
        ("dio2", "DIO2"),
        ("dio3", "DIO3"),
        ("dio4", "DIO4"),
        ("dio5", "DIO5"),
        ("dio6", "DIO6"),
        ("dio7", "DIO7"),
        ("dio8", "DIO8"),
        ("eoi", "EOI"),
        ("dav", "DAV"),
        ("nrfd", "NRFD"),
        ("ndac", "NDAC"),
        ("ifc", "IFC"),
        ("srq", "SRQ"),
        ("atn", "ATN"),
        ("ren", "REN"),
    ];

    fn available(id: &str) -> bool {
        let found = oxrok::pydecode::defs().iter().any(|d| d.info.id == id);
        if !found {
            eprintln!("no Python decoder '{id}' found, skipping");
        }
        found
    }

    /// A hand-built corpus case carrying the numeric channel indexes
    /// from `test.conf`.
    fn case(name: &str) -> CorpusCase {
        CorpusCase {
            name: name.to_owned(),
            channels: CHANNELS
                .iter()
                .enumerate()
                .map(|(index, id)| ((*id).to_owned(), index))
                .collect(),
            options: Vec::new(),
            input: None,
            stack: Vec::new(),
        }
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
    fn gpib_fixture_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("gpib") {
            return;
        }
        let capture = load(GPIB);
        assert_case(&gpib::DEF, "gpib", &case("hp33120a-idn"), &capture);
    }

    #[test]
    fn gpib_fixture_parity_by_channel_names() {
        if !crate::support::python_available() {
            return;
        }
        if !available("gpib") {
            return;
        }
        let capture = load(GPIB);
        assert_parity_def(&gpib::DEF, "gpib", &PAIRS, &capture);
    }

    #[test]
    fn ieee488_fixture_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("ieee488") {
            return;
        }
        let capture = load(IEEE488);
        assert_case(&ieee488::DEF, "ieee488", &case("hp33120a-idn"), &capture);
    }

    #[test]
    fn ieee488_fixture_parity_by_channel_names() {
        if !crate::support::python_available() {
            return;
        }
        if !available("ieee488") {
            return;
        }
        let capture = load(IEEE488);
        assert_parity_def(&ieee488::DEF, "ieee488", &PAIRS, &capture);
    }

    #[test]
    fn gpib_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("gpib") {
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
        let (ran, skipped) = assert_corpus_parity_def(&gpib::DEF, "gpib");
        eprintln!("gpib: {ran} corpus cases ran, {skipped} skipped");
        // sigrok-test has no decoder/test/gpib directory at the moment; the
        // GPIB captures are only exercised by the ieee488 corpus cases.
        // Once the directory exists, require at least one case to run.
        let has_cases = crate::support::corpus_test_dir()
            .is_some_and(|dir| dir.join("decoder/test/gpib").is_dir());
        if has_cases {
            assert!(ran > 0, "gpib: no corpus case ran");
        }
    }

    #[test]
    fn ieee488_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("ieee488") {
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
        let (ran, skipped) = assert_corpus_parity_def(&ieee488::DEF, "ieee488");
        eprintln!("ieee488: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "ieee488: no corpus case ran");
    }
}
