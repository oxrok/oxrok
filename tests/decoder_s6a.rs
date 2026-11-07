// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `jtag_ejtag`, `jtag_stm32`, `onewire_network`, `ds2408`,
//! `ds243x` and `ds28ea00` against the unchanged libsigrokdecode Python
//! decoders and the sigrok-test corpus.
//!
//! These tests need the `python` or `rustpython` feature; without either
//! they are compiled away, so a plain `cargo test` still passes.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{
        Capture, CorpusCase, assert_corpus_parity_def, assert_parity_def, corpus_config, load,
        python, run_chain,
    };
    use oxrok::decode::{Config, DecoderDef, Output};
    use oxrok::decoders::{
        ds28ea00, ds243x, ds2408, jtag, jtag_ejtag, jtag_stm32, onewire_link, onewire_network,
    };

    const JTAG_EJTAG: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/jtag_ejtag/ejtag_sigrok.sr");
    const JTAG_STM32: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/jtag_stm32/olimex_stm32-h103_irscan_drscan.sr"
    );
    const ONEWIRE_NETWORK: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/onewire_network/owdir.sr");
    // The DS2408 dumps are analog captures imported without a header (see
    // onewire/ds2408/README in sigrok-dumps); the fixture is the same
    // capture thresholded to a logic signal at the same sample rate.
    const DS2408: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/ds2408/read-pio-registers.sr"
    );
    const DS2408_WRITE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/ds2408/channel-access-write.sr"
    );
    const DS2408_WRITE_FAIL: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/ds2408/channel-access-write-fail.sr"
    );
    const DS243X: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/ds243x/ds2432.sr");
    const DS28EA00: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/ds28ea00/ds28ea00.sr");

    fn corpus_enabled() -> bool {
        if std::env::var_os("OXROK_CORPUS").is_none() {
            eprintln!("set OXROK_CORPUS=1 to run the sigrok-test corpus");
            return false;
        }
        true
    }

    /// The `Config` for a decoder with numeric channel assignments, as
    /// sigrok-test's `channel id=number` pairs build them.
    fn cfg(def: &DecoderDef, channels: &[(&str, usize)]) -> Config {
        let case = CorpusCase {
            name: def.info.id.to_owned(),
            channels: channels.iter().map(|(id, n)| ((*id).to_owned(), *n)).collect(),
            options: Vec::new(),
            input: None,
            stack: Vec::new(),
        };
        corpus_config(def.info, &case)
    }

    /// Outputs as they can be compared across hosts: an `End` error keeps
    /// only whether it happened, as the Python host reports an exception
    /// class and traceback rather than the native error text.
    fn comparable(output: &Output) -> Output {
        match output {
            Output::End { instance, error } => Output::End {
                instance: instance.clone(),
                error: error.as_ref().map(|_| "<error>".to_owned()),
            },
            _ => output.clone(),
        }
    }

    /// Runs a native chain and the same chain of Python decoders and
    /// asserts the outputs match. Returns the number of outputs.
    fn assert_chain(
        label: &str,
        capture: &Capture,
        parts: &[(&'static DecoderDef, Config)],
    ) -> usize {
        let mut python_parts = Vec::with_capacity(parts.len());
        for (def, config) in parts {
            let id = def.info.id;
            if !oxrok::pydecode::defs().iter().any(|d| d.info.id == id) {
                eprintln!("no Python decoder '{id}' found, skipping {label}");
                return 0;
            }
            python_parts.push((python(id), config.clone()));
        }
        let native = run_chain(parts, capture);
        let reference = run_chain(&python_parts, capture);
        assert_eq!(
            native.len(),
            reference.len(),
            "{label}: output count differs\nnative: {native:#?}\npython: {reference:#?}"
        );
        for (i, (n, p)) in native.iter().zip(&reference).enumerate() {
            assert_eq!(
                comparable(n),
                comparable(p),
                "{label}: output {i} differs\nnative: {n:#?}\npython: {p:#?}"
            );
        }
        native.len()
    }

    /// The jtag chain over the EJTAG capture.
    fn jtag_ejtag_chain() -> Vec<(&'static DecoderDef, Config)> {
        vec![
            (&jtag::DEF, cfg(&jtag::DEF, &[("tdi", 3), ("tdo", 2), ("tck", 0), ("tms", 1)])),
            (&jtag_ejtag::DEF, cfg(&jtag_ejtag::DEF, &[])),
        ]
    }

    /// The jtag chain over the STM32 capture.
    fn jtag_stm32_chain() -> Vec<(&'static DecoderDef, Config)> {
        vec![
            (&jtag::DEF, cfg(&jtag::DEF, &[("tdi", 1), ("tdo", 4), ("tck", 3), ("tms", 2)])),
            (&jtag_stm32::DEF, cfg(&jtag_stm32::DEF, &[])),
        ]
    }

    /// The onewire_network chain over a capture whose 1-Wire line is
    /// channel 0.
    fn onewire_network_chain() -> Vec<(&'static DecoderDef, Config)> {
        vec![
            (&onewire_link::DEF, cfg(&onewire_link::DEF, &[("owr", 0)])),
            (&onewire_network::DEF, cfg(&onewire_network::DEF, &[])),
        ]
    }

    /// Adds a transport-layer decoder above the onewire_network chain.
    fn onewire_device_chain(def: &'static DecoderDef) -> Vec<(&'static DecoderDef, Config)> {
        let mut parts = onewire_network_chain();
        parts.push((def, cfg(def, &[])));
        parts
    }

    #[test]
    fn jtag_ejtag_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&jtag_ejtag::DEF, "jtag_ejtag");
        eprintln!("jtag_ejtag: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "jtag_ejtag: no corpus case ran");
    }

    #[test]
    fn jtag_stm32_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&jtag_stm32::DEF, "jtag_stm32");
        eprintln!("jtag_stm32: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "jtag_stm32: no corpus case ran");
    }

    #[test]
    fn onewire_network_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&onewire_network::DEF, "onewire_network");
        eprintln!("onewire_network: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "onewire_network: no corpus case ran");
    }

    #[test]
    fn ds2408_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        // sigrok-test has no ds2408 test.conf; the case list is empty.
        let (ran, skipped) = assert_corpus_parity_def(&ds2408::DEF, "ds2408");
        eprintln!("ds2408: {ran} corpus cases ran, {skipped} skipped");
    }

    #[test]
    fn ds243x_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&ds243x::DEF, "ds243x");
        eprintln!("ds243x: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "ds243x: no corpus case ran");
    }

    #[test]
    fn ds28ea00_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&ds28ea00::DEF, "ds28ea00");
        eprintln!("ds28ea00: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "ds28ea00: no corpus case ran");
    }

    #[test]
    fn jtag_ejtag_regression() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(JTAG_EJTAG);
        // The fixture's channel names and the test.conf mapping.
        assert_parity_def(
            &jtag::DEF,
            "jtag",
            &[("tdi", "TDI"), ("tdo", "TDO"), ("tck", "TCK"), ("tms", "TMS")],
            &capture,
        );
        let parts = jtag_ejtag_chain();
        let outputs = assert_chain("jtag_ejtag", &capture, &parts);
        assert!(outputs > 0, "jtag_ejtag: no output");
    }

    #[test]
    fn jtag_stm32_regression() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(JTAG_STM32);
        assert_parity_def(
            &jtag::DEF,
            "jtag",
            &[("tdi", "TDI"), ("tdo", "TDO"), ("tck", "TCK"), ("tms", "TMS")],
            &capture,
        );
        let parts = jtag_stm32_chain();
        let outputs = assert_chain("jtag_stm32", &capture, &parts);
        assert!(outputs > 0, "jtag_stm32: no output");
    }

    #[test]
    fn onewire_network_regression() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(ONEWIRE_NETWORK);
        assert_parity_def(&onewire_link::DEF, "onewire_link", &[("owr", "0")], &capture);
        let parts = onewire_network_chain();
        let outputs = assert_chain("onewire_network", &capture, &parts);
        assert!(outputs > 0, "onewire_network: no output");
    }

    #[test]
    fn ds2408_regression() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(DS2408);
        let parts = onewire_device_chain(&ds2408::DEF);
        let outputs = assert_chain("ds2408", &capture, &parts);
        assert!(outputs > 0, "ds2408: no output");
    }

    /// The channel access write success and failure paths.
    #[test]
    fn ds2408_channel_access_write_regression() {
        if !crate::support::python_available() {
            return;
        }
        for (label, path) in [("write", DS2408_WRITE), ("write-fail", DS2408_WRITE_FAIL)] {
            let capture = load(path);
            let parts = onewire_device_chain(&ds2408::DEF);
            let outputs = assert_chain(&format!("ds2408-{label}"), &capture, &parts);
            assert!(outputs > 0, "ds2408-{label}: no output");
        }
    }

    #[test]
    fn ds243x_regression() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(DS243X);
        // The ds2432 dump names its first channel OWR.
        assert_parity_def(&onewire_link::DEF, "onewire_link", &[("owr", "OWR")], &capture);
        let parts = onewire_device_chain(&ds243x::DEF);
        let outputs = assert_chain("ds243x", &capture, &parts);
        assert!(outputs > 0, "ds243x: no output");
    }

    #[test]
    fn ds28ea00_regression() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(DS28EA00);
        assert_parity_def(&onewire_link::DEF, "onewire_link", &[("owr", "0")], &capture);
        let parts = onewire_device_chain(&ds28ea00::DEF);
        let outputs = assert_chain("ds28ea00", &capture, &parts);
        assert!(outputs > 0, "ds28ea00: no output");
    }
}
