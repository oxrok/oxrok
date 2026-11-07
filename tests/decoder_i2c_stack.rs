// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `i2cfilter`, `i2cdemux` and `eeprom24xx` against the unchanged
//! libsigrokdecode Python decoders of the same ids.
//!
//! The stacked decoders are exercised on top of the native `i2c`, and the
//! Python chain is built from the same configuration, so a difference in
//! either the lower decoder's items or the target's outputs shows up.
//! Everything needs the `python` or `rustpython` feature; a plain
//! `cargo test` compiles it away.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{
        Capture, CorpusCase, assert_corpus_parity_def, corpus_config, load, python, run_chain,
    };
    use oxrok::decode::{Config, DecoderDef, Output};
    use oxrok::decoders::{eeprom24xx, i2c, i2cdemux, i2cfilter};

    /// One decoder of a chain, bottom first.
    type Parts = Vec<(&'static DecoderDef, Config)>;

    const TREKSTOR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/i2cfilter/trekstor_ebr30_a_i2c_0x15.sr"
    );
    const DEMUX: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/i2cdemux/trekstor_ebr30_a_i2c_0x15.sr"
    );
    const EEPROM: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/eeprom24xx/lcsoft-mini-board-fx2-init.sr"
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

    /// The native and Python chain for a target case.
    fn chains(
        target: &'static DecoderDef,
        i2c_case: &CorpusCase,
        target_case: &CorpusCase,
    ) -> (Parts, Parts) {
        let i2c_cfg = corpus_config(i2c::DEF.info, i2c_case);
        let target_cfg = corpus_config(target.info, target_case);
        let native = vec![(&i2c::DEF, i2c_cfg.clone()), (target, target_cfg.clone())];
        let reference = vec![(python("i2c"), i2c_cfg), (python(target.info.id), target_cfg)];
        (native, reference)
    }

    fn assert_chain(label: &str, native: &Parts, reference: &Parts, capture: &Capture) {
        let native = run_chain(native, capture);
        let reference = run_chain(reference, capture);
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

    fn comparable(output: &Output) -> Output {
        match output {
            Output::End { instance, error } => Output::End {
                instance: instance.clone(),
                error: error.as_ref().map(|_| "<error>".to_owned()),
            },
            _ => output.clone(),
        }
    }

    #[test]
    fn info_matches_python() {
        for def in [&i2cfilter::DEF, &i2cdemux::DEF, &eeprom24xx::DEF] {
            assert_eq!(def.info, python(def.info.id).info);
        }
    }

    #[test]
    fn i2cfilter_regression() {
        let capture = load(TREKSTOR);
        let i2c_case = case("trekstor", &[("scl", 0), ("sda", 1)], &[]);
        let filter_case = case("trekstor_52", &[], &[("address", "52")]);
        let (native, reference) = chains(&i2cfilter::DEF, &i2c_case, &filter_case);
        assert_chain("i2cfilter", &native, &reference, &capture);
    }

    #[test]
    fn i2cfilter_direction_regression() {
        let capture = load(TREKSTOR);
        let i2c_case = case("trekstor", &[("scl", 0), ("sda", 1)], &[]);
        let filter_case =
            case("trekstor_21_write", &[], &[("address", "21"), ("direction", "write")]);
        let (native, reference) = chains(&i2cfilter::DEF, &i2c_case, &filter_case);
        assert_chain("i2cfilter write", &native, &reference, &capture);
    }

    #[test]
    fn i2cdemux_regression() {
        let capture = load(DEMUX);
        let i2c_case = case("trekstor", &[("scl", 0), ("sda", 1)], &[]);
        let demux_case = case("trekstor_demux", &[], &[]);
        let (native, reference) = chains(&i2cdemux::DEF, &i2c_case, &demux_case);
        assert_chain("i2cdemux", &native, &reference, &capture);
    }

    #[test]
    fn eeprom24xx_regression() {
        let capture = load(EEPROM);
        let i2c_case = case("lcsoft", &[("scl", 1), ("sda", 0)], &[]);
        let eeprom_case = case("lcsoft", &[], &[]);
        let (native, reference) = chains(&eeprom24xx::DEF, &i2c_case, &eeprom_case);
        assert_chain("eeprom24xx", &native, &reference, &capture);
    }

    #[test]
    fn eeprom24xx_after_filter_regression() {
        let capture = load(TREKSTOR);
        let i2c_case = case("trekstor", &[("scl", 0), ("sda", 1)], &[]);
        let filter_case = case("trekstor_52", &[], &[("address", "52")]);
        let eeprom_case = case("trekstor_52", &[], &[]);
        let i2c_cfg = corpus_config(i2c::DEF.info, &i2c_case);
        let filter_cfg = corpus_config(i2cfilter::DEF.info, &filter_case);
        let eeprom_cfg = corpus_config(eeprom24xx::DEF.info, &eeprom_case);
        let native = vec![
            (&i2c::DEF, i2c_cfg.clone()),
            (&i2cfilter::DEF, filter_cfg.clone()),
            (&eeprom24xx::DEF, eeprom_cfg.clone()),
        ];
        let reference = vec![
            (python("i2c"), i2c_cfg),
            (python("i2cfilter"), filter_cfg),
            (python("eeprom24xx"), eeprom_cfg),
        ];
        assert_chain("i2c->i2cfilter->eeprom24xx", &native, &reference, &capture);
    }

    #[test]
    fn i2cfilter_corpus() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&i2cfilter::DEF, "i2cfilter");
        eprintln!("i2cfilter corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no i2cfilter corpus case ran");
    }

    #[test]
    fn i2cdemux_corpus() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&i2cdemux::DEF, "i2cdemux");
        eprintln!("i2cdemux corpus: {ran} ran, {skipped} skipped");
        // sigrok-test has no i2cdemux/test.conf yet, so there is nothing
        // to run. If cases appear, at least one must run rather than be
        // skipped for a missing input or a non-native lower decoder.
        assert!(
            ran > 0 || ran + skipped == 0,
            "i2cdemux: corpus cases exist but none ran ({ran} ran, {skipped} skipped)"
        );
    }

    #[test]
    fn eeprom24xx_corpus() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&eeprom24xx::DEF, "eeprom24xx");
        eprintln!("eeprom24xx corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no eeprom24xx corpus case ran");
    }
}
