// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! In-process parity between the native USB decoders and the unchanged
//! libsigrokdecode Python decoders.
//!
//! These tests need the `python` or `rustpython` feature and a
//! libsigrokdecode decoders directory. Without either they are compiled
//! away, so a plain `cargo test` still passes.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{
        Capture, CorpusCase, assert_corpus_parity_def, corpus_config, load, python, run,
    };
    use oxrok::decode::DecoderDef;

    const SIGNALLING_FIXTURE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/usb_signalling/usb_failed_setup_fullspeed.sr"
    );
    const SIGNALLING_LOW_SPEED_FIXTURE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/usb_signalling/logitech_rx250_leftclick.sr"
    );
    const POWER_DELIVERY_FIXTURE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/usb_power_delivery/thinkpad_yoga_370-aukey_45w.sr"
    );

    /// A test.conf style case with its numeric channel assignments.
    fn corpus_case(name: &str, channels: &[(&str, usize)], options: &[(&str, &str)]) -> CorpusCase {
        CorpusCase {
            name: name.to_owned(),
            channels: channels.iter().map(|&(id, index)| (id.to_owned(), index)).collect(),
            options: options.iter().map(|&(id, value)| (id.to_owned(), value.to_owned())).collect(),
            input: None,
            stack: Vec::new(),
        }
    }

    fn assert_case(def: &'static DecoderDef, case: &CorpusCase, capture: &Capture) {
        let config = corpus_config(def.info, case);
        let native = run(def, &config, capture);
        let reference = run(python(def.info.id), &config, capture);
        let first = native.iter().zip(&reference).position(|(n, r)| n != r);
        let Some(first) = first else {
            assert_eq!(native.len(), reference.len(), "{}: output count differs", case.name);
            assert!(!native.is_empty(), "{}: the decoder emitted nothing", case.name);
            return;
        };
        let lo = first.saturating_sub(3);
        let native_end = (first + 3).min(native.len());
        let reference_end = (first + 3).min(reference.len());
        panic!(
            "{}: first difference at output {first} (native {} outputs, reference {})\n\
             native[{lo}..]: {:#?}\nreference[{lo}..]: {:#?}",
            case.name,
            native.len(),
            reference.len(),
            &native[lo..native_end],
            &reference[lo..reference_end],
        );
    }

    fn corpus(name: &str, def: &'static DecoderDef) {
        if std::env::var_os("OXROK_CORPUS").is_none() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(def, name);
        eprintln!("{name}: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "{name}: no corpus case ran ({skipped} skipped)");
    }

    #[test]
    fn info_matches_python() {
        if !crate::support::python_available() {
            return;
        }
        for def in
            [&oxrok::decoders::usb_signalling::DEF, &oxrok::decoders::usb_power_delivery::DEF]
        {
            assert_eq!(def.info, python(def.info.id).info);
        }
    }

    #[test]
    fn usb_signalling_fixture() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(SIGNALLING_FIXTURE);
        let case = corpus_case(
            "usb_failed_setup_fullspeed",
            &[("dp", 1), ("dm", 0)],
            &[("signalling", "full-speed")],
        );
        assert_case(&oxrok::decoders::usb_signalling::DEF, &case, &capture);
    }

    #[test]
    fn usb_signalling_low_speed_fixture() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(SIGNALLING_LOW_SPEED_FIXTURE);
        let case = corpus_case(
            "logitech_rx250_leftclick",
            &[("dp", 0), ("dm", 1)],
            &[("signalling", "low-speed")],
        );
        assert_case(&oxrok::decoders::usb_signalling::DEF, &case, &capture);
    }

    #[test]
    fn usb_power_delivery_fixture() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(POWER_DELIVERY_FIXTURE);
        let case = corpus_case(
            "thinkpad_yoga_370-aukey_45w",
            &[("cc1", 0), ("cc2", 1)],
            &[("fulltext", "yes")],
        );
        assert_case(&oxrok::decoders::usb_power_delivery::DEF, &case, &capture);
    }

    #[test]
    fn usb_signalling_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("usb_signalling", &oxrok::decoders::usb_signalling::DEF);
    }

    #[test]
    fn usb_power_delivery_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("usb_power_delivery", &oxrok::decoders::usb_power_delivery::DEF);
    }
}
