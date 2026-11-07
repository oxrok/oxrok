// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `ds1307`, `lm75`, `rtc8564`, `pca9571`, `tca6408a` and
//! `atsha204a` against the unchanged libsigrokdecode Python decoders of
//! the same ids, all stacked on `i2c`.
//!
//! The regression cases read committed captures with the channel numbers
//! of sigrok-test's `test.conf`; the corpus cases run the whole
//! sigrok-test set when `OXROK_CORPUS=1`. Without the `python` or
//! `rustpython` feature everything is compiled away, so a plain
//! `cargo test` still passes.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{
        Capture, CorpusCase, assert_corpus_parity_def, assert_parity_def, corpus_config, load,
        python, run_chain,
    };
    use oxrok::decode::{Config, DecoderDef, Output};
    use oxrok::decoders::{atsha204a, ds1307, i2c, i2cfilter, lm75, pca9571, rtc8564, tca6408a};

    const DS1307_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/ds1307/rtc_ds1307_200khz.sr");
    const LM75_SR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/lm75/rding_temper_i2c_eeprom_and_sensor.sr"
    );
    const RTC8564_SR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/rtc8564/rtc_epson_8564je_snippet.sr"
    );
    const PCA9571_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/pca9571/pca9571_simple.sr");
    const TCA6408A_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/tca6408a/tca6408a.sr");
    const ATSHA204A_SR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/atsha204a/atsha204a_snippet.sr"
    );

    fn corpus_enabled() -> bool {
        if std::env::var_os("OXROK_CORPUS").is_some() {
            return true;
        }
        eprintln!("set OXROK_CORPUS=1 to run the sigrok-test corpus");
        false
    }

    fn case(name: &str, channels: &[(&str, usize)], options: &[(&str, &str)]) -> CorpusCase {
        CorpusCase {
            name: name.to_owned(),
            channels: channels.iter().map(|(id, n)| ((*id).to_owned(), *n)).collect(),
            options: options.iter().map(|(id, v)| ((*id).to_owned(), (*v).to_owned())).collect(),
            input: None,
            stack: Vec::new(),
        }
    }

    /// Outputs as they can be compared across hosts: an `End` error keeps
    /// only whether it happened, since the Python host reports a
    /// traceback and not the native error text.
    fn comparable(output: &Output) -> Output {
        match output {
            Output::End { instance, error } => Output::End {
                instance: instance.clone(),
                error: error.as_ref().map(|_| "<error>".to_owned()),
            },
            _ => output.clone(),
        }
    }

    /// Runs the native chain `parts` (bottom first) and the Python chain
    /// with the same configs, and asserts the outputs match.
    fn assert_chain(label: &str, parts: &[(&'static DecoderDef, Config)], capture: &Capture) {
        let reference_parts: Vec<(&'static DecoderDef, Config)> =
            parts.iter().map(|(def, cfg)| (python(def.info.id), cfg.clone())).collect();
        let native = run_chain(parts, capture);
        let reference = run_chain(&reference_parts, capture);
        assert_eq!(
            native.len(),
            reference.len(),
            "{label}: output count differs\nnative: {native:#?}\nreference: {reference:#?}"
        );
        for (i, (n, p)) in native.iter().zip(&reference).enumerate() {
            assert!(
                comparable(n) == comparable(p),
                "{label}: output {i} differs\nnative: {n:#?}\nreference: {p:#?}"
            );
        }
        assert!(!native.is_empty(), "{label}: the chain emitted nothing");
    }

    fn corpus(id: &str, def: &'static DecoderDef) {
        if !corpus_enabled() {
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
    fn info_matches_python() {
        if !crate::support::python_available() {
            return;
        }
        for id in ["ds1307", "lm75", "rtc8564", "pca9571", "tca6408a", "atsha204a"] {
            let def = oxrok::decoders::ALL
                .iter()
                .copied()
                .find(|d| d.info.id == id)
                .unwrap_or_else(|| panic!("no native decoder '{id}'"));
            assert_eq!(def.info, python(id).info, "{id}: decoder info differs");
        }
    }

    /// `assert_parity_def` needs a logic-input decoder; check the lower
    /// `i2c` layer on a fixture so a chain difference can be localised.
    #[test]
    fn i2c_baseline_parity() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(DS1307_SR);
        assert_parity_def(&i2c::DEF, "i2c", &[("scl", "SCL"), ("sda", "SDA")], &capture);
    }

    #[test]
    fn ds1307_regression() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(DS1307_SR);
        let i2c_case = case("rtc_ds1307_200khz", &[("scl", 0), ("sda", 1)], &[]);
        let target_case = case("ds1307", &[], &[]);
        let parts = vec![
            (&i2c::DEF, corpus_config(i2c::DEF.info, &i2c_case)),
            (&ds1307::DEF, corpus_config(ds1307::DEF.info, &target_case)),
        ];
        assert_chain("i2c->ds1307", &parts, &capture);
    }

    #[test]
    fn lm75_regression() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(LM75_SR);
        // sigrok-test's rding_temper_fm75_3: sda=0, scl=1, filter 0x4f.
        let i2c_case = case("rding_temper", &[("sda", 0), ("scl", 1)], &[]);
        let filter_case = case("rding_temper_79", &[], &[("address", "79")]);
        let target_case = case("lm75", &[], &[]);
        let parts = vec![
            (&i2c::DEF, corpus_config(i2c::DEF.info, &i2c_case)),
            (&i2cfilter::DEF, corpus_config(i2cfilter::DEF.info, &filter_case)),
            (&lm75::DEF, corpus_config(lm75::DEF.info, &target_case)),
        ];
        assert_chain("i2c->i2cfilter->lm75", &parts, &capture);
    }

    #[test]
    fn rtc8564_regression() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(RTC8564_SR);
        let i2c_case = case("rtc_epson_8564je_snippet", &[("scl", 0), ("sda", 1)], &[]);
        let target_case = case("rtc8564", &[], &[]);
        let parts = vec![
            (&i2c::DEF, corpus_config(i2c::DEF.info, &i2c_case)),
            (&rtc8564::DEF, corpus_config(rtc8564::DEF.info, &target_case)),
        ];
        assert_chain("i2c->rtc8564", &parts, &capture);
    }

    #[test]
    fn pca9571_regression() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(PCA9571_SR);
        let i2c_case = case("pca9571_simple", &[("sda", 0), ("scl", 1)], &[]);
        let target_case = case("pca9571", &[], &[]);
        let parts = vec![
            (&i2c::DEF, corpus_config(i2c::DEF.info, &i2c_case)),
            (&pca9571::DEF, corpus_config(pca9571::DEF.info, &target_case)),
        ];
        assert_chain("i2c->pca9571", &parts, &capture);
    }

    #[test]
    fn tca6408a_regression() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(TCA6408A_SR);
        let i2c_case = case("tca6408a_all_slaves", &[("scl", 0), ("sda", 1)], &[]);
        let target_case = case("tca6408a", &[], &[]);
        let parts = vec![
            (&i2c::DEF, corpus_config(i2c::DEF.info, &i2c_case)),
            (&tca6408a::DEF, corpus_config(tca6408a::DEF.info, &target_case)),
        ];
        assert_chain("i2c->tca6408a", &parts, &capture);
    }

    #[test]
    fn atsha204a_regression() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(ATSHA204A_SR);
        let i2c_case = case("atsha204a_snippet", &[("sda", 0), ("scl", 1)], &[]);
        let target_case = case("atsha204a", &[], &[]);
        let parts = vec![
            (&i2c::DEF, corpus_config(i2c::DEF.info, &i2c_case)),
            (&atsha204a::DEF, corpus_config(atsha204a::DEF.info, &target_case)),
        ];
        assert_chain("i2c->atsha204a", &parts, &capture);
    }

    #[test]
    fn ds1307_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("ds1307", &ds1307::DEF);
    }

    #[test]
    fn lm75_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("lm75", &lm75::DEF);
    }

    #[test]
    fn rtc8564_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("rtc8564", &rtc8564::DEF);
    }

    #[test]
    fn pca9571_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("pca9571", &pca9571::DEF);
    }

    #[test]
    fn tca6408a_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("tca6408a", &tca6408a::DEF);
    }

    #[test]
    fn atsha204a_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("atsha204a", &atsha204a::DEF);
    }
}
