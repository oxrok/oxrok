// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `stepper_motor`, `wiegand`, `em4100` and `em4305` against the
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
        Capture, CorpusCase, assert_corpus_parity_def, corpus_config, corpus_dumps_dir,
        corpus_test_dir, load, python, run,
    };
    use oxrok::decode::DecoderDef;
    use oxrok::decoders::{em4100, em4305, stepper_motor, wiegand};

    const STEPPER_MOTOR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/stepper_motor/smoothieware_snippet.sr"
    );
    const WIEGAND: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/wiegand/wiegand-8a640910.sr");
    const EM4100: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/em4100/em4100_0200f5ed8d.sr");
    const EM4305: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/em4305/em4305_write.sr");

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
            ("stepper_motor", &stepper_motor::DEF),
            ("wiegand", &wiegand::DEF),
            ("em4100", &em4100::DEF),
            ("em4305", &em4305::DEF),
        ] {
            if !available(id) {
                continue;
            }
            assert_eq!(def.info, python(id).info, "{id}: decoder info differs");
        }
    }

    #[test]
    fn stepper_motor_fixture_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("stepper_motor") {
            return;
        }
        let capture = load(STEPPER_MOTOR);
        // The channels and indexes from sigrok-test's test.conf; `mm`
        // exercises steps_per_mm and the two-decimal format.
        let case = case("smoothieware_snippet_mm", &[("step", 5), ("dir", 6)], &[("unit", "mm")]);
        assert_case(&stepper_motor::DEF, "stepper_motor", &case, &capture);
    }

    #[test]
    fn stepper_motor_steps_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("stepper_motor") {
            return;
        }
        let capture = load(STEPPER_MOTOR);
        let case =
            case("smoothieware_snippet_steps", &[("step", 5), ("dir", 6)], &[("unit", "steps")]);
        assert_case(&stepper_motor::DEF, "stepper_motor", &case, &capture);
    }

    #[test]
    fn stepper_motor_options_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("stepper_motor") {
            return;
        }
        let capture = load(STEPPER_MOTOR);
        let case = case(
            "smoothieware_snippet_custom_scale",
            &[("step", 5), ("dir", 6)],
            &[("unit", "mm"), ("steps_per_mm", "200")],
        );
        assert_case(&stepper_motor::DEF, "stepper_motor", &case, &capture);
    }

    #[test]
    fn wiegand_fixture_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("wiegand") {
            return;
        }
        let capture = load(WIEGAND);
        let case = case("wiegand_8a640910", &[("d0", 0), ("d1", 1)], &[]);
        assert_case(&wiegand::DEF, "wiegand", &case, &capture);
    }

    #[test]
    fn wiegand_active_high_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("wiegand") {
            return;
        }
        let capture = load(WIEGAND);
        // Inverted interpretation of the same capture; exercises the
        // invalid/state handling.
        let case =
            case("wiegand_8a640910_active_high", &[("d0", 0), ("d1", 1)], &[("active", "high")]);
        assert_case(&wiegand::DEF, "wiegand", &case, &capture);
    }

    #[test]
    fn em4100_fixture_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("em4100") {
            return;
        }
        let capture = load(EM4100);
        let case = case("em4100_0200f5ed8d", &[("data", 1)], &[]);
        assert_case(&em4100::DEF, "em4100", &case, &capture);
    }

    #[test]
    fn em4305_fixture_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("em4305") {
            return;
        }
        let capture = load(EM4305);
        let case = case("em4305_write", &[("data", 0)], &[]);
        assert_case(&em4305::DEF, "em4305", &case, &capture);
    }

    #[test]
    fn em4100_options_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("em4100") {
            return;
        }
        let capture = load(EM4100);
        for options in [
            &[("polarity", "active-low")][..],
            &[("datarate", "32")][..],
            &[("coilfreq", "100000")][..],
        ] {
            eprintln!("em4100 options {options:?}");
            let case = case("em4100_0200f5ed8d_options", &[("data", 1)], options);
            assert_case(&em4100::DEF, "em4100", &case, &capture);
        }
    }

    #[test]
    fn em4305_options_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("em4305") {
            return;
        }
        let capture = load(EM4305);
        for options in
            [&[("em4100_decode", "off")][..], &[("w_gap", "10"), ("w_one_max", "30")][..]]
        {
            eprintln!("em4305 options {options:?}");
            let case = case("em4305_write_options", &[("data", 0)], options);
            assert_case(&em4305::DEF, "em4305", &case, &capture);
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

    fn corpus(id: &str, def: &'static DecoderDef) {
        if !available(id) || !corpus_ready() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(def, id);
        eprintln!("{id}: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "{id}: no corpus case ran");
    }

    #[test]
    fn stepper_motor_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        corpus("stepper_motor", &stepper_motor::DEF);
    }

    #[test]
    fn wiegand_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        corpus("wiegand", &wiegand::DEF);
    }

    #[test]
    fn em4100_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        corpus("em4100", &em4100::DEF);
    }

    #[test]
    fn em4305_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        corpus("em4305", &em4305::DEF);
    }
}
