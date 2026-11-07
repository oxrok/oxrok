// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Parity of the native `parallel` and `ps2` decoders with the unchanged
//! libsigrokdecode Python decoders.
//!
//! These tests need the `python` or `rustpython` feature; without either
//! they are compiled away, so a plain `cargo test` still passes.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{
        CorpusCase, assert_corpus_parity_def, assert_parity_def, corpus_config, load, python, run,
    };
    use oxrok::decoders::{parallel, ps2};

    const PARALLEL: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/parallel/incremental_8ch_short.sr"
    );
    const PS2: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/ps2/ps2_keyboard_asdfgh_no_inhibit.sr"
    );

    fn corpus_enabled() -> bool {
        if std::env::var_os("OXROK_CORPUS").is_none() {
            eprintln!("set OXROK_CORPUS=1 to run the sigrok-test corpus");
            return false;
        }
        true
    }

    /// The parallel corpus uses `channel clk=0 channel d0=0 ... channel
    /// d7=7` for the clocked incremental capture.
    #[test]
    fn parallel_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&parallel::DEF, "parallel");
        eprintln!("parallel: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "parallel: no corpus case ran");
    }

    /// The ps2 corpus uses `channel clk=3 channel data=2` and `channel
    /// clk=0 channel data=1`.
    #[test]
    fn ps2_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&ps2::DEF, "ps2");
        eprintln!("ps2: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "ps2: no corpus case ran");
    }

    #[test]
    fn parallel_regression() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(PARALLEL);
        // The numeric channel indexes from test.conf, as the corpus
        // runner builds them: clk and d0..d7 all on logic channels 0..7.
        let case = CorpusCase {
            name: "incremental_8ch_short_clock".into(),
            channels: vec![
                ("clk".into(), 0),
                ("d0".into(), 0),
                ("d1".into(), 1),
                ("d2".into(), 2),
                ("d3".into(), 3),
                ("d4".into(), 4),
                ("d5".into(), 5),
                ("d6".into(), 6),
                ("d7".into(), 7),
            ],
            options: Vec::new(),
            input: Some("misc/demo/incremental_8ch_short.sr".into()),
            stack: Vec::new(),
        };
        let config = corpus_config(parallel::DEF.info, &case);
        let native = run(&parallel::DEF, &config, &capture);
        let reference = run(python("parallel"), &config, &capture);
        assert!(!native.is_empty(), "parallel: no output");
        assert_eq!(native, reference, "parallel: numeric channel mapping");
    }

    #[test]
    fn ps2_regression() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(PS2);
        // ps2_keyboard_asdfgh_no_inhibit: `channel clk=0 channel data=1`.
        let case = CorpusCase {
            name: "ps2_keyboard_asdfgh_no_inhibit".into(),
            channels: vec![("clk".into(), 0), ("data".into(), 1)],
            options: Vec::new(),
            input: Some("ps2/keyboard/ps2_keyboard_asdfgh_no_inhibit.sr".into()),
            stack: Vec::new(),
        };
        let config = corpus_config(ps2::DEF.info, &case);
        let native = run(&ps2::DEF, &config, &capture);
        let reference = run(python("ps2"), &config, &capture);
        assert!(!native.is_empty(), "ps2: no output");
        assert_eq!(native, reference, "ps2: numeric channel mapping");
    }

    /// The same ps2 mapping by channel name, through `parse_spec()`.
    #[test]
    fn ps2_regression_by_name() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(PS2);
        assert_parity_def(&ps2::DEF, "ps2", &[("clk", "Clock"), ("data", "Data")], &capture);
    }
}
