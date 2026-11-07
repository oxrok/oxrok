// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Parity of the native `jtag` and `swd` decoders with the unchanged
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
    use oxrok::decoders::{jtag, swd};

    const JTAG: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/jtag/olimex_stm32-h103_idle.sr"
    );
    const SWD: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/swd/init_write_0xabbabeeb.sr"
    );

    fn corpus_enabled() -> bool {
        if std::env::var_os("OXROK_CORPUS").is_none() {
            eprintln!("set OXROK_CORPUS=1 to run the sigrok-test corpus");
            return false;
        }
        true
    }

    /// The jtag corpus uses `channel tdi=1 channel tdo=4 channel tck=3
    /// channel tms=2`.
    #[test]
    fn jtag_corpus_parity() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&jtag::DEF, "jtag");
        eprintln!("jtag: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "jtag: no corpus case ran");
    }

    /// The swd corpus uses `channel swclk=0 channel swdio=1`.
    #[test]
    fn swd_corpus_parity() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&swd::DEF, "swd");
        eprintln!("swd: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "swd: no corpus case ran");
    }

    #[test]
    fn jtag_regression() {
        let capture = load(JTAG);
        // The same mapping by channel name, through parse_spec().
        assert_parity_def(
            &jtag::DEF,
            "jtag",
            &[("tdi", "TDI"), ("tdo", "TDO"), ("tck", "TCK"), ("tms", "TMS")],
            &capture,
        );

        // The numeric channel indexes from test.conf, as the corpus
        // runner builds them.
        let case = CorpusCase {
            name: "olimex_stm32-h103_idle".into(),
            channels: vec![
                ("tdi".into(), 1),
                ("tdo".into(), 4),
                ("tck".into(), 3),
                ("tms".into(), 2),
            ],
            options: Vec::new(),
            input: Some("jtag/olimex_stm32-h103/olimex_stm32-h103_idle.sr".into()),
            stack: Vec::new(),
        };
        let config = corpus_config(&jtag::DEF.info, &case);
        let native = run(&jtag::DEF, &config, &capture);
        let reference = run(python("jtag"), &config, &capture);
        assert!(!native.is_empty(), "jtag: no output");
        assert_eq!(native, reference, "jtag: numeric channel mapping");
    }

    #[test]
    fn swd_regression() {
        let capture = load(SWD);
        // swclk/swdio are sample bits 0 and 1, as the corpus maps them.
        assert_parity_def(&swd::DEF, "swd", &[], &capture);

        let case = CorpusCase {
            name: "init_write_0xabbabeeb".into(),
            channels: vec![("swclk".into(), 0), ("swdio".into(), 1)],
            options: Vec::new(),
            input: Some("swd/stlink_openocd/init_write_0xabbabeeb.sr".into()),
            stack: Vec::new(),
        };
        let config = corpus_config(&swd::DEF.info, &case);
        let native = run(&swd::DEF, &config, &capture);
        let reference = run(python("swd"), &config, &capture);
        assert!(!native.is_empty(), "swd: no output");
        assert_eq!(native, reference, "swd: numeric channel mapping");
    }
}
