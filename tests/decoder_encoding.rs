// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `graycode`, `morse`, `miller` and `rc_encode` against the
//! unchanged libsigrokdecode Python decoders and the sigrok-test corpus.
//!
//! These tests need the `python` or `rustpython` feature. Without either
//! they are compiled away, so a plain `cargo test` still passes.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{
        Capture, CorpusCase, assert_corpus_parity_def, assert_parity_def, corpus_cases,
        corpus_config, corpus_dumps_dir, corpus_test_dir, load, python, run,
    };
    use oxrok::decode::DecoderDef;
    use oxrok::decoders::{graycode, miller, morse, rc_encode};

    const GRAYCODE: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/graycode/rotary-ramp.sr");
    const MORSE: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/morse/hi.vcd");
    const MILLER: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/miller/mfrc522-1gs.sr");
    const RC_ENCODE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/rc_encode/maplin_l95ar_single_a.sr"
    );

    fn available(id: &str) -> bool {
        let found = oxrok::pydecode::defs().iter().any(|d| d.info.id == id);
        if !found {
            eprintln!("no Python decoder '{id}' found, skipping");
        }
        found
    }

    /// Runs a hand-built corpus case through both implementations, with
    /// the numeric channel indexes sigrok-test's test.conf uses.
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

    fn case(name: &str, channels: &[(&str, usize)], options: &[(&str, &str)]) -> CorpusCase {
        CorpusCase {
            name: name.to_owned(),
            channels: channels.iter().map(|&(id, n)| (id.to_owned(), n)).collect(),
            options: options.iter().map(|&(k, v)| (k.to_owned(), v.to_owned())).collect(),
            input: None,
            stack: Vec::new(),
        }
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

    #[test]
    fn info_matches_python() {
        for def in [
            &oxrok::decoders::graycode::DEF,
            &oxrok::decoders::morse::DEF,
            &oxrok::decoders::miller::DEF,
            &oxrok::decoders::rc_encode::DEF,
        ] {
            assert_eq!(def.info, python(def.info.id).info);
        }
    }

    #[test]
    fn graycode_rotary_ramp_parity() {
        if !available("graycode") {
            return;
        }
        let capture = load(GRAYCODE);
        // The channel and indexes from sigrok-test's rotary_ramp test.
        let case = case("rotary_ramp", &[("d0", 0), ("d1", 1)], &[]);
        assert_case(&graycode::DEF, "graycode", &case, &capture);
    }

    #[test]
    fn graycode_edges_option_parity() {
        if !available("graycode") {
            return;
        }
        let capture = load(GRAYCODE);
        let d0 = capture.channels[0].as_str();
        let d1 = capture.channels[1].as_str();
        // The corpus does not set `edges`; this exercises turns and rpm.
        assert_parity_def(
            &graycode::DEF,
            "graycode",
            &[("d0", d0), ("d1", d1), ("edges", "4")],
            &capture,
        );
    }

    #[test]
    fn morse_hi_parity() {
        if !available("morse") {
            return;
        }
        let capture = load(MORSE);
        let case = case("hi", &[("data", 0)], &[]);
        assert_case(&morse::DEF, "morse", &case, &capture);
    }

    #[test]
    fn miller_mfrc522_parity() {
        if !available("miller") {
            return;
        }
        let capture = load(MILLER);
        // The channel and index from sigrok-test's mfrc522-1gs test.
        let case = case("mfrc522-1gs", &[("data", 0)], &[]);
        assert_case(&miller::DEF, "miller", &case, &capture);
    }

    #[test]
    fn rc_encode_maplin_l95ar_parity() {
        if !available("rc_encode") {
            return;
        }
        let capture = load(RC_ENCODE);
        // The channel and index from sigrok-test's maplin_l95ar tests.
        let case = case("maplin_l95ar_single_a", &[("data", 0)], &[]);
        assert_case(&rc_encode::DEF, "rc_encode", &case, &capture);
    }

    #[test]
    fn rc_encode_maplin_l95ar_remote_parity() {
        if !available("rc_encode") {
            return;
        }
        let capture = load(RC_ENCODE);
        let case =
            case("maplin_l95ar_single_a_remote", &[("data", 0)], &[("remote", "maplin_l95ar")]);
        assert_case(&rc_encode::DEF, "rc_encode", &case, &capture);
    }

    #[test]
    fn graycode_corpus_parity() {
        if !available("graycode") || !corpus_ready() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&graycode::DEF, "graycode");
        eprintln!("graycode: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "graycode: no corpus case ran");
    }

    #[test]
    fn morse_corpus_parity() {
        if !available("morse") || !corpus_ready() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&morse::DEF, "morse");
        eprintln!("morse: {ran} corpus cases ran, {skipped} skipped");
        // sigrok-test has no morse test.conf yet; assert when it gains one.
        if !corpus_cases("morse").is_empty() {
            assert!(ran > 0, "morse: no corpus case ran");
        }
    }

    #[test]
    fn miller_corpus_parity() {
        if !available("miller") || !corpus_ready() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&miller::DEF, "miller");
        eprintln!("miller: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "miller: no corpus case ran");
    }

    #[test]
    fn rc_encode_corpus_parity() {
        if !available("rc_encode") || !corpus_ready() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&rc_encode::DEF, "rc_encode");
        eprintln!("rc_encode: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "rc_encode: no corpus case ran");
    }
}
