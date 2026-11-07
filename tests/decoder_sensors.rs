// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `t55xx`, `am230x`, `dsi`, `qi` and `flexray` against the
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
        Capture, CorpusCase, assert_corpus_parity_def, assert_parity_def, corpus_config,
        corpus_dumps_dir, corpus_test_dir, load, python, run,
    };
    use oxrok::decode::DecoderDef;
    use oxrok::decoders::{am230x, dsi, flexray, qi, t55xx};

    const T55XX: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/t55xx/em4305_write.sr");
    const AM230X: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/am230x/am2302_1mhz.sr");
    const DSI: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/dsi/rako_wda-600_dsi.sr");
    const QI: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/qi/gpmq8005a-nexus5.sr");
    const FLEXRAY: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/flexray/flexray_2s16_0d_one_cycle.sr"
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

    fn case(name: &str, channels: &[(&str, usize)], options: &[(&str, &str)]) -> CorpusCase {
        CorpusCase {
            name: name.to_owned(),
            channels: channels.iter().map(|&(id, n)| (id.to_owned(), n)).collect(),
            options: options.iter().map(|&(k, v)| (k.to_owned(), v.to_owned())).collect(),
            input: None,
            stack: Vec::new(),
        }
    }

    #[test]
    fn decoder_info_matches_python() {
        if !crate::support::python_available() {
            return;
        }
        for (id, def) in [
            ("t55xx", &t55xx::DEF),
            ("am230x", &am230x::DEF),
            ("dsi", &dsi::DEF),
            ("qi", &qi::DEF),
            ("flexray", &flexray::DEF),
        ] {
            if !available(id) {
                continue;
            }
            assert_eq!(def.info, python(id).info, "{id}: decoder info differs");
        }
    }

    #[test]
    fn t55xx_fixture_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("t55xx") {
            return;
        }
        // em4305_write: channel data=0, all options at their defaults.
        let capture = load(T55XX);
        assert_parity_def(&t55xx::DEF, "t55xx", &[], &capture);
    }

    #[test]
    fn t55xx_case_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("t55xx") {
            return;
        }
        let capture = load(T55XX);
        let case = case("em4305_write", &[("data", 0)], &[]);
        assert_case(&t55xx::DEF, "t55xx", &case, &capture);
    }

    #[test]
    fn am230x_fixture_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("am230x") {
            return;
        }
        // am2302_1mhz: channel sda=0, device at its am230x default.
        let capture = load(AM230X);
        assert_parity_def(&am230x::DEF, "am230x", &[], &capture);
    }

    #[test]
    fn am230x_case_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("am230x") {
            return;
        }
        let capture = load(AM230X);
        let case = case("am2302_1mhz", &[("sda", 0)], &[]);
        assert_case(&am230x::DEF, "am230x", &case, &capture);
    }

    #[test]
    fn dsi_fixture_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("dsi") {
            return;
        }
        // rako_wda-600_dsi: channel dsi=0, active-high polarity.
        let capture = load(DSI);
        assert_parity_def(&dsi::DEF, "dsi", &[], &capture);
    }

    #[test]
    fn dsi_case_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("dsi") {
            return;
        }
        let capture = load(DSI);
        let case = case("rako_wda-600_dsi", &[("dsi", 0)], &[]);
        assert_case(&dsi::DEF, "dsi", &case, &capture);
    }

    #[test]
    fn qi_fixture_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("qi") {
            return;
        }
        // gpmq8005a-nexus5: channel qi=0.
        let capture = load(QI);
        assert_parity_def(&qi::DEF, "qi", &[], &capture);
    }

    #[test]
    fn qi_case_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("qi") {
            return;
        }
        let capture = load(QI);
        let case = case("gpmq8005a-nexus5", &[("qi", 0)], &[]);
        assert_case(&qi::DEF, "qi", &case, &capture);
    }

    #[test]
    fn flexray_fixture_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("flexray") {
            return;
        }
        // flexray_2s16_0d_one_cycle: channel 0, channel type A,
        // 10Mbit/s, all at their defaults.
        let capture = load(FLEXRAY);
        assert_parity_def(&flexray::DEF, "flexray", &[], &capture);
    }

    #[test]
    fn flexray_case_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("flexray") {
            return;
        }
        let capture = load(FLEXRAY);
        let case = case(
            "flexray_2s16_0d_one_cycle",
            &[("channel", 0)],
            &[("channel_type", "A"), ("bitrate", "10000000")],
        );
        assert_case(&flexray::DEF, "flexray", &case, &capture);
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
    fn t55xx_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        corpus("t55xx", &t55xx::DEF);
    }

    #[test]
    fn am230x_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        corpus("am230x", &am230x::DEF);
    }

    #[test]
    fn dsi_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        corpus("dsi", &dsi::DEF);
    }

    #[test]
    fn qi_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        corpus("qi", &qi::DEF);
    }

    #[test]
    fn flexray_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        corpus("flexray", &flexray::DEF);
    }
}
