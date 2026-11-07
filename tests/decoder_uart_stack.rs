// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `lin`, `modbus` and `midi` against the unchanged
//! libsigrokdecode Python decoders of the same ids, both stacked on
//! `uart`.
//!
//! The regression cases read committed captures with the UART and target
//! decoder configurations of sigrok-test's `test.conf`; the corpus cases
//! run the whole sigrok-test set when `OXROK_CORPUS=1`. Without the
//! `python` or `rustpython` feature everything is compiled away, so a
//! plain `cargo test` still passes.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{
        Capture, CorpusCase, assert_corpus_parity_def, corpus_config, load, python, run_chain,
    };
    use oxrok::decode::DecoderDef;

    const LIN_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/lin/burst.sr");
    const MODBUS_SR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/modbus/brainchild-io-16do.sr"
    );
    const MIDI_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/midi/midi_key1.sr");

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

    /// Runs `uart` then `target` natively and through the Python chain,
    /// and compares every output in order.
    fn assert_chain(
        target: &'static DecoderDef,
        uart_case: &CorpusCase,
        target_case: &CorpusCase,
        capture: &Capture,
    ) {
        let uart_cfg = corpus_config(oxrok::decoders::uart::DEF.info, uart_case);
        let target_cfg = corpus_config(target.info, target_case);
        let native_parts = [(&oxrok::decoders::uart::DEF, uart_cfg.clone()), (target, target_cfg)];
        let python_parts = [
            (python("uart"), uart_cfg),
            (python(target.info.id), corpus_config(target.info, target_case)),
        ];
        let native = run_chain(&native_parts, capture);
        let reference = run_chain(&python_parts, capture);
        let first = native.iter().zip(&reference).position(|(n, r)| n != r);
        let Some(first) = first else {
            assert_eq!(native.len(), reference.len(), "{}: output count differs", target_case.name);
            assert!(!native.is_empty(), "{}: the chain emitted nothing", target_case.name);
            return;
        };
        let lo = first.saturating_sub(3);
        let native_end = (first + 3).min(native.len());
        let reference_end = (first + 3).min(reference.len());
        panic!(
            "{}: first difference at output {first} (native {} outputs, reference {})\n\
             native[{lo}..]: {:#?}\nreference[{lo}..]: {:#?}",
            target_case.name,
            native.len(),
            reference.len(),
            &native[lo..native_end],
            &reference[lo..reference_end],
        );
    }

    #[test]
    fn info_matches_python() {
        for def in
            [&oxrok::decoders::lin::DEF, &oxrok::decoders::modbus::DEF, &oxrok::decoders::midi::DEF]
        {
            assert_eq!(def.info, python(def.info.id).info);
        }
    }

    #[test]
    fn lin_regression() {
        let uart =
            case("lin_burst_uart", &[("rx", 0)], &[("baudrate", "19200"), ("format", "ascii")]);
        let lin = case("lin_burst", &[], &[("version", "2")]);
        let capture = load(LIN_SR);
        assert_chain(&oxrok::decoders::lin::DEF, &uart, &lin, &capture);
    }

    #[test]
    fn modbus_regression() {
        let uart = case(
            "brainchild_uart",
            &[("rx", 0), ("tx", 1)],
            &[
                ("baudrate", "19200"),
                ("invert_rx", "yes"),
                ("invert_tx", "yes"),
                ("parity", "even"),
            ],
        );
        let modbus = case("brainchild_io_16do", &[], &[("scchannel", "RX"), ("cschannel", "TX")]);
        let capture = load(MODBUS_SR);
        assert_chain(&oxrok::decoders::modbus::DEF, &uart, &modbus, &capture);
    }

    #[test]
    fn midi_regression() {
        let uart = case("midi_key1_uart", &[("rx", 0)], &[("baudrate", "31250")]);
        let midi = case("midi_key1", &[], &[]);
        let capture = load(MIDI_SR);
        assert_chain(&oxrok::decoders::midi::DEF, &uart, &midi, &capture);
    }

    #[test]
    fn lin_corpus() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&oxrok::decoders::lin::DEF, "lin");
        eprintln!("lin corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no lin corpus case ran");
    }

    #[test]
    fn modbus_corpus() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&oxrok::decoders::modbus::DEF, "modbus");
        eprintln!("modbus corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no modbus corpus case ran");
    }

    #[test]
    fn midi_corpus() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&oxrok::decoders::midi::DEF, "midi");
        eprintln!("midi corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no midi corpus case ran");
    }
}
