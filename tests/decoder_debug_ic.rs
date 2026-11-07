// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Parity of the native `avr_pdi`, `swim`, `ac97` and `aud` decoders with
//! the unchanged libsigrokdecode Python decoders.
//!
//! These tests need the `python` or `rustpython` feature; without either
//! they are compiled away, so a plain `cargo test` still passes.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{
        Capture, CorpusCase, assert_corpus_parity_def, assert_parity_def, corpus_config, load,
        python, run,
    };
    use oxrok::decoders::{ac97, aud, avr_pdi, swim};

    const AVR_PDI: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/avr_pdi/atxmega128a1-pdi-identify.sr"
    );
    const SWIM: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/swim/sample1.sr");
    const AC97: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/ac97/asus_a8ne_fm_s_ac97_powerup_snippet2.sr"
    );
    const AUD: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/aud/audgen-01.sr");

    fn corpus_enabled() -> bool {
        if std::env::var_os("OXROK_CORPUS").is_none() {
            eprintln!("set OXROK_CORPUS=1 to run the sigrok-test corpus");
            return false;
        }
        true
    }

    /// The avr_pdi corpus uses `channel reset=0 channel data=1`.
    #[test]
    fn avr_pdi_corpus_parity() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&avr_pdi::DEF, "avr_pdi");
        eprintln!("avr_pdi: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "avr_pdi: no corpus case ran");
    }

    /// The swim corpus uses `channel swim=1`.
    #[test]
    fn swim_corpus_parity() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&swim::DEF, "swim");
        eprintln!("swim: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "swim: no corpus case ran");
    }

    /// The ac97 corpus uses `channel clk=0 channel out=1 channel in=2
    /// channel sync=3`.
    #[test]
    fn ac97_corpus_parity() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&ac97::DEF, "ac97");
        eprintln!("ac97: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "ac97: no corpus case ran");
    }

    /// The aud corpus uses `channel audck=0 channel naudsync=1
    /// channel audata3=2 channel audata2=3 channel audata1=4
    /// channel audata0=5`.
    #[test]
    fn aud_corpus_parity() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&aud::DEF, "aud");
        eprintln!("aud: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "aud: no corpus case ran");
    }

    #[test]
    fn avr_pdi_regression() {
        let capture = load(AVR_PDI);
        // The numeric channel indexes from test.conf, as the corpus runner
        // builds them: reset and data on logic channels 0 and 1.
        let case = CorpusCase {
            name: "atxmega128a1_identify".into(),
            channels: vec![("reset".into(), 0), ("data".into(), 1)],
            options: Vec::new(),
            input: Some("avr_pdi/atxmega128a1/atxmega128a1-pdi-identify.sr".into()),
            stack: Vec::new(),
        };
        let config = corpus_config(avr_pdi::DEF.info, &case);
        let native = run(&avr_pdi::DEF, &config, &capture);
        let reference = run(python("avr_pdi"), &config, &capture);
        assert!(!native.is_empty(), "avr_pdi: no output");
        assert_eq!(native, reference, "avr_pdi: numeric channel mapping");
    }

    #[test]
    fn avr_pdi_break_parity() {
        // A run of eleven low DATA bit times is a PDI BREAK. The upstream
        // decoder's `if bitval == '0':` never matches an int, so it calls
        // put() with a None end sample and ends with an error; assert the
        // native port ends the same way.
        let mut data = vec![0b11; 4];
        for _ in 0..20 {
            data.push(0b00);
            data.push(0b00);
            data.push(0b10);
            data.push(0b10);
        }
        let capture = Capture {
            channels: vec!["RESET".into(), "DATA".into()],
            unit_size: 1,
            samplerate: 10_000_000,
            data,
        };
        assert_parity_def(
            &avr_pdi::DEF,
            "avr_pdi",
            &[("reset", "RESET"), ("data", "DATA")],
            &capture,
        );
    }

    #[test]
    fn swim_regression() {
        let capture = load(SWIM);
        // tests/sample1.sr: SWIM is logic channel 1.
        let case = CorpusCase {
            name: "tests_1".into(),
            channels: vec![("swim".into(), 1)],
            options: Vec::new(),
            input: Some("swim/tests/sample1.sr".into()),
            stack: Vec::new(),
        };
        let config = corpus_config(swim::DEF.info, &case);
        let native = run(&swim::DEF, &config, &capture);
        let reference = run(python("swim"), &config, &capture);
        assert!(!native.is_empty(), "swim: no output");
        assert_eq!(native, reference, "swim: numeric channel mapping");

        // The debug option adds the byte counter annotation.
        let case = CorpusCase {
            name: "tests_1_debug".into(),
            channels: vec![("swim".into(), 1)],
            options: vec![("debug".into(), "yes".into())],
            input: Some("swim/tests/sample1.sr".into()),
            stack: Vec::new(),
        };
        let config = corpus_config(swim::DEF.info, &case);
        let native = run(&swim::DEF, &config, &capture);
        let reference = run(python("swim"), &config, &capture);
        assert!(!native.is_empty(), "swim: no output with debug");
        assert_eq!(native, reference, "swim: debug option");
    }

    #[test]
    fn ac97_regression() {
        let capture = load(AC97);
        // alc655_snippet2: BIT-CLK, SDATA-OUT, SDATA-IN and SYNC are logic
        // channels 0, 1, 2 and 3.
        let case = CorpusCase {
            name: "alc655_snippet2".into(),
            channels: vec![
                ("clk".into(), 0),
                ("out".into(), 1),
                ("in".into(), 2),
                ("sync".into(), 3),
            ],
            options: Vec::new(),
            input: Some("ac97/realtek_alc655/asus_a8ne_fm_s_ac97_powerup_snippet2.sr".into()),
            stack: Vec::new(),
        };
        let config = corpus_config(ac97::DEF.info, &case);
        let native = run(&ac97::DEF, &config, &capture);
        let reference = run(python("ac97"), &config, &capture);
        assert!(!native.is_empty(), "ac97: no output");
        assert_eq!(native, reference, "ac97: numeric channel mapping");
    }

    #[test]
    fn aud_regression() {
        let capture = load(AUD);
        // audgen-01: AUDCK, nAUDSYNC and AUDATA3..0 are logic channels 0..5.
        let case = CorpusCase {
            name: "audgen-01".into(),
            channels: vec![
                ("audck".into(), 0),
                ("naudsync".into(), 1),
                ("audata3".into(), 2),
                ("audata2".into(), 3),
                ("audata1".into(), 4),
                ("audata0".into(), 5),
            ],
            options: Vec::new(),
            input: Some("aud/audgen-01.sr".into()),
            stack: Vec::new(),
        };
        let config = corpus_config(aud::DEF.info, &case);
        let native = run(&aud::DEF, &config, &capture);
        let reference = run(python("aud"), &config, &capture);
        assert!(!native.is_empty(), "aud: no output");
        assert_eq!(native, reference, "aud: numeric channel mapping");
    }
}
