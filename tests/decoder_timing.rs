// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Parity of the native `counter`, `guess_bitrate`, `jitter`, `pwm` and
//! `timing` decoders with the unchanged libsigrokdecode Python decoders.
//!
//! These tests need the `python` or `rustpython` feature; without either
//! they are compiled away, so a plain `cargo test` still passes.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{CorpusCase, assert_corpus_parity_def, corpus_config, load, python, run};
    use oxrok::decoders::{counter, guess_bitrate, jitter, pwm, timing};

    const COUNTER: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/counter/9bit_spi.sr");
    const GUESS_BITRATE: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/guess_bitrate/es51978.sr");
    const JITTER: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/jitter/toim4243_10byte_send_snippet.sr"
    );
    const PWM: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/pwm/pwmtest_snippet.sr");
    const TIMING: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/timing/am2301_1mhz.sr");

    fn corpus_enabled() -> bool {
        if std::env::var_os("OXROK_CORPUS").is_none() {
            eprintln!("set OXROK_CORPUS=1 to run the sigrok-test corpus");
            return false;
        }
        true
    }

    /// Asserts that the native and Python decoders agree on a corpus
    /// case built by hand from the numeric indices in test.conf.
    fn check_case(def: &'static oxrok::decode::DecoderDef, capture: &str, case: &CorpusCase) {
        let config = corpus_config(def.info, case);
        let capture = load(capture);
        let native = run(def, &config, &capture);
        let reference = run(python(def.info.id), &config, &capture);
        assert!(!native.is_empty(), "{}: no output", def.info.id);
        assert_eq!(native, reference, "{}: numeric channel mapping", def.info.id);
    }

    /// `counter`: the `9bit_spi` capture uses `channel data=2`.
    #[test]
    fn counter_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&counter::DEF, "counter");
        eprintln!("counter: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "counter: no corpus case ran");
    }

    /// `guess_bitrate`: `channel data=0` on the ES51978 capture.
    #[test]
    fn guess_bitrate_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&guess_bitrate::DEF, "guess_bitrate");
        eprintln!("guess_bitrate: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "guess_bitrate: no corpus case ran");
    }

    /// `jitter`: `channel clk=3 channel sig=2` with both polarities
    /// falling on the TOIM4243 capture.
    #[test]
    fn jitter_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&jitter::DEF, "jitter");
        eprintln!("jitter: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "jitter: no corpus case ran");
    }

    /// `pwm`: `channel data=4` on the ALSA test capture.
    #[test]
    fn pwm_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&pwm::DEF, "pwm");
        eprintln!("pwm: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "pwm: no corpus case ran");
    }

    /// `timing`: `channel data=5` on the AM2301 capture.
    #[test]
    fn timing_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&timing::DEF, "timing");
        eprintln!("timing: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "timing: no corpus case ran");
    }

    #[test]
    fn counter_regression() {
        if !crate::support::python_available() {
            return;
        }
        check_case(
            &counter::DEF,
            COUNTER,
            &CorpusCase {
                name: "9bit_spi_rising_2bit_words".into(),
                channels: vec![("data".into(), 2)],
                options: vec![
                    ("data_edge".into(), "rising".into()),
                    ("divider".into(), "2".into()),
                ],
                input: Some("spi/wordwidths/9bit_spi.sr".into()),
                stack: Vec::new(),
            },
        );
    }

    #[test]
    fn guess_bitrate_regression() {
        if !crate::support::python_available() {
            return;
        }
        check_case(
            &guess_bitrate::DEF,
            GUESS_BITRATE,
            &CorpusCase {
                name: "es51978".into(),
                channels: vec![("data".into(), 0)],
                options: Vec::new(),
                input: Some("uart/cyrustek_es51978/es51978.sr".into()),
                stack: Vec::new(),
            },
        );
    }

    #[test]
    fn jitter_regression() {
        if !crate::support::python_available() {
            return;
        }
        check_case(
            &jitter::DEF,
            JITTER,
            &CorpusCase {
                name: "toim4243".into(),
                channels: vec![("clk".into(), 3), ("sig".into(), 2)],
                options: vec![
                    ("clk_polarity".into(), "falling".into()),
                    ("sig_polarity".into(), "falling".into()),
                ],
                input: Some("uart/toim4243/toim4243_10byte_send_snippet.sr".into()),
                stack: Vec::new(),
            },
        );
    }

    #[test]
    fn pwm_regression() {
        if !crate::support::python_available() {
            return;
        }
        check_case(
            &pwm::DEF,
            PWM,
            &CorpusCase {
                name: "alsa_test_snippet".into(),
                channels: vec![("data".into(), 4)],
                options: Vec::new(),
                input: Some("pwm/alsa_test/pwmtest_snippet.sr".into()),
                stack: Vec::new(),
            },
        );
    }

    #[test]
    fn timing_regression() {
        if !crate::support::python_available() {
            return;
        }
        check_case(
            &timing::DEF,
            TIMING,
            &CorpusCase {
                name: "am2301_1mhz_avg100".into(),
                channels: vec![("data".into(), 5)],
                options: vec![("avg_period".into(), "100".into())],
                input: Some("am230x/am2301/am2301_1mhz.sr".into()),
                stack: Vec::new(),
            },
        );
        // The same capture with no averaging and the delta column.
        check_case(
            &timing::DEF,
            TIMING,
            &CorpusCase {
                name: "am2301_1mhz_noavg_rising_delta".into(),
                channels: vec![("data".into(), 5)],
                options: vec![
                    ("avg_period".into(), "0".into()),
                    ("edge".into(), "rising".into()),
                    ("delta".into(), "yes".into()),
                ],
                input: Some("am230x/am2301/am2301_1mhz.sr".into()),
                stack: Vec::new(),
            },
        );
    }
}
