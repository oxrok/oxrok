// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Parity of the native `dcf77`, `spdif`, `i2s` and `tdm_audio` decoders
//! with the unchanged libsigrokdecode Python decoders of the same ids.
//!
//! The tests need the `python` or `rustpython` feature; without either
//! they are compiled away, so a plain `cargo test` still passes.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{CorpusCase, assert_corpus_parity_def, corpus_config, load, python, run};
    use oxrok::decoders::{dcf77, i2s, spdif, tdm_audio};

    const DCF77: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/dcf77/dcf77_20s.sr");
    const SPDIF: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/spdif/2ch-16bit-48khz.sr");
    const I2S: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/i2s/2ch-32bit-8khz-no-samplerate.sr"
    );
    const TDM_AUDIO: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/tdm_audio/tdm-4ch-16bit-cap1.sr"
    );

    fn corpus_enabled() -> bool {
        if std::env::var_os("OXROK_CORPUS").is_none() {
            eprintln!("set OXROK_CORPUS=1 to run the sigrok-test corpus");
            return false;
        }
        true
    }

    /// Asserts that the native and Python decoders agree on a case built
    /// by hand from the numeric channel indices in test.conf.
    fn check_case(def: &'static oxrok::decode::DecoderDef, capture: &str, case: &CorpusCase) {
        let config = corpus_config(def.info, case);
        let capture = load(capture);
        let native = run(def, &config, &capture);
        let reference = run(python(def.info.id), &config, &capture);
        assert!(!native.is_empty(), "{}: no output", def.info.id);
        assert_eq!(native, reference, "{}: numeric channel mapping", def.info.id);
    }

    #[test]
    fn info_matches_python() {
        if !crate::support::python_available() {
            return;
        }
        for def in [&dcf77::DEF, &spdif::DEF, &i2s::DEF, &tdm_audio::DEF] {
            assert_eq!(def.info, python(def.info.id).info);
        }
    }

    #[test]
    fn dcf77_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&dcf77::DEF, "dcf77");
        eprintln!("dcf77: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "dcf77: no corpus case ran");
    }

    #[test]
    fn spdif_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&spdif::DEF, "spdif");
        eprintln!("spdif: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "spdif: no corpus case ran");
    }

    #[test]
    fn i2s_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&i2s::DEF, "i2s");
        eprintln!("i2s: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "i2s: no corpus case ran");
    }

    #[test]
    fn tdm_audio_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&tdm_audio::DEF, "tdm_audio");
        eprintln!("tdm_audio: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "tdm_audio: no corpus case ran");
    }

    /// `dcf77`: `channel data=1` on the Pollin DCF1 capture.
    #[test]
    fn dcf77_regression() {
        if !crate::support::python_available() {
            return;
        }
        check_case(
            &dcf77::DEF,
            DCF77,
            &CorpusCase {
                name: "dcf77_20s".into(),
                channels: vec![("data".into(), 1)],
                options: Vec::new(),
                input: Some("dcf77/pollin_dcf1_module/dcf77_20s.sr".into()),
                stack: Vec::new(),
            },
        );
    }

    /// `spdif`: `channel data=0` on the 2ch-16bit-48khz capture.
    #[test]
    fn spdif_regression() {
        if !crate::support::python_available() {
            return;
        }
        check_case(
            &spdif::DEF,
            SPDIF,
            &CorpusCase {
                name: "2ch-16bit-48khz".into(),
                channels: vec![("data".into(), 0)],
                options: Vec::new(),
                input: Some("spdif/2ch-16bit-48khz/2ch-16bit-48khz.sr".into()),
                stack: Vec::new(),
            },
        );
    }

    /// `i2s`: `channel sck=0 channel ws=1 channel sd=2`, on the capture
    /// without a sample rate: the decoder does not need one.
    #[test]
    fn i2s_regression() {
        if !crate::support::python_available() {
            return;
        }
        check_case(
            &i2s::DEF,
            I2S,
            &CorpusCase {
                name: "2ch-32bit-8khz-no-samplerate".into(),
                channels: vec![("sck".into(), 0), ("ws".into(), 1), ("sd".into(), 2)],
                options: Vec::new(),
                input: Some("i2s/2ch-32bit-8khz/2ch-32bit-8khz-no-samplerate.sr".into()),
                stack: Vec::new(),
            },
        );
    }

    /// `tdm_audio`: `channel clock=3 channel frame=2 channel data=1`.
    #[test]
    fn tdm_audio_regression() {
        if !crate::support::python_available() {
            return;
        }
        check_case(
            &tdm_audio::DEF,
            TDM_AUDIO,
            &CorpusCase {
                name: "tdm-4ch-16bit-cap1".into(),
                channels: vec![("clock".into(), 3), ("frame".into(), 2), ("data".into(), 1)],
                options: Vec::new(),
                input: Some("tdm_audio/tdm-4ch-16bit-cap1.sr".into()),
                stack: Vec::new(),
            },
        );
    }
}
