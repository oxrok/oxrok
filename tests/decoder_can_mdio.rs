// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `can` and `mdio` against the unchanged libsigrokdecode Python
//! decoders of the same ids.
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
        CorpusCase, assert_corpus_parity_def, assert_parity_def, corpus_config, load, python, run,
    };

    const CAN_SR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/can/mcp2515dm-bm-125kbits_msg_222_5bytes.sr"
    );
    const MDIO_SR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/mdio/lan8720a_read_write_read.sr"
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

    #[test]
    fn can_regression() {
        let case = case(
            "mcp2515dm-bm-125kbits_msg_222_5bytes",
            &[("can_rx", 2)],
            &[("nominal_bitrate", "125000")],
        );
        let cfg = corpus_config(oxrok::decoders::can::DEF.info, &case);
        let capture = load(CAN_SR);
        let native = run(&oxrok::decoders::can::DEF, &cfg, &capture);
        let reference = run(python("can"), &cfg, &capture);
        assert_eq!(native, reference, "can: native and Python outputs differ");
    }

    #[test]
    fn can_sample_point_option() {
        let capture = load(CAN_SR);
        let chan = capture.channels[2].clone();
        assert_parity_def(
            &oxrok::decoders::can::DEF,
            "can",
            &[("can_rx", chan.as_str()), ("nominal_bitrate", "125000"), ("sample_point", "80.0")],
            &capture,
        );
    }

    #[test]
    fn mdio_regression() {
        let case = case("lan8720a_read_write_read", &[("mdc", 0), ("mdio", 1)], &[]);
        let cfg = corpus_config(oxrok::decoders::mdio::DEF.info, &case);
        let capture = load(MDIO_SR);
        let native = run(&oxrok::decoders::mdio::DEF, &cfg, &capture);
        let reference = run(python("mdio"), &cfg, &capture);
        assert_eq!(native, reference, "mdio: native and Python outputs differ");
    }

    #[test]
    fn mdio_debug_bits_option() {
        let capture = load(MDIO_SR);
        let mdc = capture.channels[0].clone();
        let mdio = capture.channels[1].clone();
        assert_parity_def(
            &oxrok::decoders::mdio::DEF,
            "mdio",
            &[("mdc", mdc.as_str()), ("mdio", mdio.as_str()), ("show_debug_bits", "yes")],
            &capture,
        );
    }

    #[test]
    fn can_corpus() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&oxrok::decoders::can::DEF, "can");
        eprintln!("can corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no can corpus case ran");
    }

    #[test]
    fn mdio_corpus() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&oxrok::decoders::mdio::DEF, "mdio");
        eprintln!("mdio corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no mdio corpus case ran");
    }
}
