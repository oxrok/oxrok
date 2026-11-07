// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `amulet_ascii`, `arm_etmv3`, `arm_itm`, `arm_tpiu` and
//! `pan1321` against the unchanged libsigrokdecode Python decoders of the
//! same ids, all stacked on `uart`.
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

    const AMULET_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/amulet_ascii/bootup.sr");
    const ARM_ETMV3_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/arm_etmv3/trace_example.sr");
    const ARM_ITM_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/arm_itm/trace_example.sr");
    const ARM_TPIU_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/arm_tpiu/trace_example.sr");
    const PAN1321_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/pan1321/pan1321_JRES.sr");

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

    /// Runs a native chain and the same chain through Python, bottom
    /// first, and compares every output in order.
    fn assert_chain(defs: &[(&'static DecoderDef, CorpusCase)], capture: &Capture) {
        let native_parts: Vec<(&'static DecoderDef, _)> =
            defs.iter().map(|(def, case)| (*def, corpus_config(def.info, case))).collect();
        let python_parts: Vec<(&'static DecoderDef, _)> = defs
            .iter()
            .map(|(def, case)| (python(def.info.id), corpus_config(def.info, case)))
            .collect();
        let native = run_chain(&native_parts, capture);
        let reference = run_chain(&python_parts, capture);
        let first = native.iter().zip(&reference).position(|(n, r)| n != r);
        let Some(first) = first else {
            assert_eq!(native.len(), reference.len(), "output count differs");
            assert!(!native.is_empty(), "the chain emitted nothing");
            return;
        };
        let lo = first.saturating_sub(3);
        let native_end = (first + 3).min(native.len());
        let reference_end = (first + 3).min(reference.len());
        panic!(
            "first difference at output {first} (native {} outputs, reference {})\n\
             native[{lo}..]: {:#?}\nreference[{lo}..]: {:#?}",
            native.len(),
            reference.len(),
            &native[lo..native_end],
            &reference[lo..reference_end],
        );
    }

    #[test]
    fn info_matches_python() {
        if !crate::support::python_available() {
            return;
        }
        for def in [
            &oxrok::decoders::amulet_ascii::DEF,
            &oxrok::decoders::arm_etmv3::DEF,
            &oxrok::decoders::arm_itm::DEF,
            &oxrok::decoders::arm_tpiu::DEF,
            &oxrok::decoders::pan1321::DEF,
        ] {
            assert_eq!(def.info, python(def.info.id).info);
        }
    }

    #[test]
    fn amulet_ascii_regression() {
        if !crate::support::python_available() {
            return;
        }
        let uart = case("amulet_bootup_uart", &[("tx", 0), ("rx", 1)], &[]);
        let amulet = case("amulet_bootup", &[], &[]);
        let capture = load(AMULET_SR);
        assert_chain(
            &[(&oxrok::decoders::uart::DEF, uart), (&oxrok::decoders::amulet_ascii::DEF, amulet)],
            &capture,
        );
    }

    #[test]
    fn arm_etmv3_regression() {
        if !crate::support::python_available() {
            return;
        }
        let uart = case("stm32f105_uart", &[("rx", 4)], &[("baudrate", "8000000")]);
        let tpiu = case("stm32f105_tpiu", &[], &[("stream", "2")]);
        let etm = case("stm32f105_etmv3", &[], &[]);
        let capture = load(ARM_ETMV3_SR);
        assert_chain(
            &[
                (&oxrok::decoders::uart::DEF, uart),
                (&oxrok::decoders::arm_tpiu::DEF, tpiu),
                (&oxrok::decoders::arm_etmv3::DEF, etm),
            ],
            &capture,
        );
    }

    #[test]
    fn arm_itm_regression() {
        if !crate::support::python_available() {
            return;
        }
        let uart = case("stm32f105_uart", &[("rx", 4)], &[("baudrate", "8000000")]);
        let tpiu = case("stm32f105_tpiu", &[], &[("stream", "1")]);
        let itm = case("stm32f105_itm", &[], &[]);
        let capture = load(ARM_ITM_SR);
        assert_chain(
            &[
                (&oxrok::decoders::uart::DEF, uart),
                (&oxrok::decoders::arm_tpiu::DEF, tpiu),
                (&oxrok::decoders::arm_itm::DEF, itm),
            ],
            &capture,
        );
    }

    #[test]
    fn arm_tpiu_regression() {
        if !crate::support::python_available() {
            return;
        }
        let uart = case("stm32f105_uart", &[("rx", 4)], &[("baudrate", "8000000")]);
        let tpiu = case("stm32f105_tpiu", &[], &[("stream", "2")]);
        let capture = load(ARM_TPIU_SR);
        assert_chain(
            &[(&oxrok::decoders::uart::DEF, uart), (&oxrok::decoders::arm_tpiu::DEF, tpiu)],
            &capture,
        );
    }

    #[test]
    fn pan1321_regression() {
        if !crate::support::python_available() {
            return;
        }
        let uart = case("pan1321_jres_uart", &[("tx", 0), ("rx", 3)], &[("baudrate", "115200")]);
        let pan = case("pan1321_jres", &[], &[]);
        let capture = load(PAN1321_SR);
        assert_chain(
            &[(&oxrok::decoders::uart::DEF, uart), (&oxrok::decoders::pan1321::DEF, pan)],
            &capture,
        );
    }

    #[test]
    fn amulet_ascii_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) =
            assert_corpus_parity_def(&oxrok::decoders::amulet_ascii::DEF, "amulet_ascii");
        eprintln!("amulet_ascii corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no amulet_ascii corpus case ran");
    }

    #[test]
    fn arm_etmv3_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) =
            assert_corpus_parity_def(&oxrok::decoders::arm_etmv3::DEF, "arm_etmv3");
        eprintln!("arm_etmv3 corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no arm_etmv3 corpus case ran");
    }

    #[test]
    fn arm_itm_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&oxrok::decoders::arm_itm::DEF, "arm_itm");
        eprintln!("arm_itm corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no arm_itm corpus case ran");
    }

    #[test]
    fn arm_tpiu_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&oxrok::decoders::arm_tpiu::DEF, "arm_tpiu");
        eprintln!("arm_tpiu corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no arm_tpiu corpus case ran");
    }

    #[test]
    fn pan1321_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&oxrok::decoders::pan1321::DEF, "pan1321");
        eprintln!("pan1321 corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no pan1321 corpus case ran");
    }
}
