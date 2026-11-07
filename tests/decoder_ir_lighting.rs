// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `ir_rc5`, `ir_rc6`, `cec`, `dali` and `dmx512` against the
//! unchanged libsigrokdecode Python decoders and the sigrok-test corpus.
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
    use oxrok::decoders::{cec, dali, dmx512, ir_rc5, ir_rc6};

    const IR_RC5: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/ir_rc5/vcr_philips_0082_button_standby_hold.sr"
    );
    const IR_RC6: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/ir_rc6/kathrein_rc674_numbers.sr"
    );
    const CEC: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/cec/tv_sony_amp_yamaha_switch_on_seq.sr"
    );
    const DALI: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/dali/rako_rsrdali_query_ballast.sr"
    );
    const DMX512: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/dmx512/500khz_all_255.sr");

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

    fn case(name: &str, channel: &str, index: usize) -> CorpusCase {
        CorpusCase {
            name: name.to_owned(),
            channels: vec![(channel.to_owned(), index)],
            options: Vec::new(),
            input: None,
            stack: Vec::new(),
        }
    }

    #[test]
    fn decoder_info_matches_python() {
        for (id, def) in [
            ("ir_rc5", &ir_rc5::DEF),
            ("ir_rc6", &ir_rc6::DEF),
            ("cec", &cec::DEF),
            ("dali", &dali::DEF),
            ("dmx512", &dmx512::DEF),
        ] {
            if !available(id) {
                continue;
            }
            assert_eq!(def.info, python(id).info, "{id}: decoder info differs");
        }
    }

    #[test]
    fn ir_rc5_fixture_parity() {
        if !available("ir_rc5") {
            return;
        }
        let capture = load(IR_RC5);
        // The channel and index from sigrok-test's test.conf.
        let case = case("vcr_philips_0082_button_standby_hold", "ir", 1);
        assert_case(&ir_rc5::DEF, "ir_rc5", &case, &capture);
    }

    #[test]
    fn ir_rc5_extended_parity() {
        if !available("ir_rc5") {
            return;
        }
        let capture = load(IR_RC5);
        let name = capture.channels[1].clone();
        assert_parity_def(
            &ir_rc5::DEF,
            "ir_rc5",
            &[("ir", name.as_str()), ("protocol", "extended")],
            &capture,
        );
    }

    #[test]
    fn ir_rc6_fixture_parity() {
        if !available("ir_rc6") {
            return;
        }
        let capture = load(IR_RC6);
        let case = case("kathrein_rc674_numbers", "ir", 0);
        assert_case(&ir_rc6::DEF, "ir_rc6", &case, &capture);
    }

    #[test]
    fn cec_fixture_parity() {
        if !available("cec") {
            return;
        }
        let capture = load(CEC);
        let case = case("tv_sony_amp_yamaha_switch_on_seq", "cec", 3);
        assert_case(&cec::DEF, "cec", &case, &capture);
    }

    #[test]
    fn dali_fixture_parity() {
        if !available("dali") {
            return;
        }
        let capture = load(DALI);
        let case = case("rako_rsrdali_query_ballast", "dali", 0);
        assert_case(&dali::DEF, "dali", &case, &capture);
    }

    #[test]
    fn dmx512_fixture_parity() {
        if !available("dmx512") {
            return;
        }
        let capture = load(DMX512);
        // The corpus stacks this decoder on a UART, but the capture is the
        // raw DMX line, which the decoder also reads directly.
        let case = case("500khz_all_255", "dmx", 0);
        assert_case(&dmx512::DEF, "dmx512", &case, &capture);
    }

    fn corpus(id: &str, def: &'static DecoderDef) {
        if !available(id) {
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
        let (ran, skipped) = assert_corpus_parity_def(def, id);
        eprintln!("{id}: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "{id}: no corpus case ran");
    }

    #[test]
    fn ir_rc5_corpus_parity() {
        corpus("ir_rc5", &ir_rc5::DEF);
    }

    #[test]
    fn ir_rc6_corpus_parity() {
        corpus("ir_rc6", &ir_rc6::DEF);
    }

    #[test]
    fn cec_corpus_parity() {
        corpus("cec", &cec::DEF);
    }

    #[test]
    fn dali_corpus_parity() {
        corpus("dali", &dali::DEF);
    }

    #[test]
    fn dmx512_corpus_parity() {
        corpus("dmx512", &dmx512::DEF);
    }
}
