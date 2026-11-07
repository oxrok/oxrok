// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `iec`, `maple_bus`, `ook`, `signature`, `tlc5620`, `microwire`
//! and `mcs48` against the unchanged libsigrokdecode Python decoders of
//! the same ids.
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
    use oxrok::decoders::{iec, maple_bus, mcs48, microwire, ook, signature, tlc5620};

    fn fixture(path: &str) -> String {
        format!("{}/tests/fixtures/decoders/{path}", env!("CARGO_MANIFEST_DIR"))
    }

    fn corpus_enabled() -> bool {
        if std::env::var_os("OXROK_CORPUS").is_some() {
            return true;
        }
        eprintln!("set OXROK_CORPUS=1 to run the sigrok-test corpus");
        false
    }

    fn case(name: &str, channels: &[(&str, usize)]) -> CorpusCase {
        CorpusCase {
            name: name.to_owned(),
            channels: channels.iter().map(|(id, n)| ((*id).to_owned(), *n)).collect(),
            options: Vec::new(),
            input: None,
            stack: Vec::new(),
        }
    }

    /// The CBM 1571 IEC capture has SRQ=0, ATN=2, CLK=3, DATA=4 and is
    /// not in sigrok-test's corpus.
    fn iec_case() -> CorpusCase {
        case("cbm_1571_read_status", &[("srq", 0), ("atn", 2), ("clk", 3), ("data", 4)])
    }

    /// `channel sdcka=1 channel sdckb=5` in sigrok-test.
    fn maple_bus_case() -> CorpusCase {
        case("bus_enumeration", &[("sdcka", 1), ("sdckb", 5)])
    }

    /// `channel data=0`, the default ook options.
    fn ook_case() -> CorpusCase {
        case("oregon_bthr968", &[("data", 0)])
    }

    /// `channel start=0 channel stop=1 channel clk=7 channel data=5`.
    fn signature_case() -> CorpusCase {
        case("signature-0003", &[("start", 0), ("stop", 1), ("clk", 7), ("data", 5)])
    }

    /// `channel data=0 channel clk=1 channel load=2 channel ldac=3`.
    fn tlc5620_case() -> CorpusCase {
        case("ti_tlc5620_one_change", &[("data", 0), ("clk", 1), ("load", 2), ("ldac", 3)])
    }

    /// `channel cs=1 channel sk=3 channel si=4 channel so=5`.
    fn microwire_case() -> CorpusCase {
        case("st_m93c66", &[("cs", 1), ("sk", 3), ("si", 4), ("so", 5)])
    }

    /// The i8039 sample: a8=0 a9=1 a10=2 a11=3 a12=4 ale=5 psen=7
    /// d0..d7=8..15.
    fn mcs48_case() -> CorpusCase {
        case(
            "i8039_sample",
            &[
                ("a8", 0),
                ("a9", 1),
                ("a10", 2),
                ("a11", 3),
                ("a12", 4),
                ("ale", 5),
                ("psen", 7),
                ("d0", 8),
                ("d1", 9),
                ("d2", 10),
                ("d3", 11),
                ("d4", 12),
                ("d5", 13),
                ("d6", 14),
                ("d7", 15),
            ],
        )
    }

    fn regression(
        name: &str,
        def: &'static oxrok::decode::DecoderDef,
        case: &CorpusCase,
        capture_path: &str,
    ) {
        let cfg = corpus_config(def.info, case);
        let capture = load(&fixture(capture_path));
        let native = run(def, &cfg, &capture);
        let reference = run(python(name), &cfg, &capture);
        assert_eq!(native, reference, "{name}: native and Python outputs differ");
    }

    #[test]
    fn iec_regression() {
        if !crate::support::python_available() {
            return;
        }
        regression("iec", &iec::DEF, &iec_case(), "iec/cbm_1571_read_status.sr");
    }

    #[test]
    fn iec_named_channels() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(&fixture("iec/cbm_1571_read_status.sr"));
        assert_parity_def(
            &iec::DEF,
            "iec",
            &[("data", "DATA"), ("clk", "CLK"), ("atn", "ATN"), ("srq", "SRQ")],
            &capture,
        );
    }

    /// The same Oregon capture through the other decode modes.
    fn ook_option_case(name: &str, options: &[(&str, &str)]) -> CorpusCase {
        CorpusCase {
            name: name.to_owned(),
            channels: vec![("data".to_owned(), 0)],
            options: options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect(),
            input: None,
            stack: Vec::new(),
        }
    }

    #[test]
    fn ook_nrz() {
        if !crate::support::python_available() {
            return;
        }
        regression(
            "ook",
            &ook::DEF,
            &ook_option_case("oregon_nrz", &[("decodeas", "NRZ")]),
            "ook/oregon_bthr968.sr",
        );
    }

    #[test]
    fn ook_diff_manchester() {
        if !crate::support::python_available() {
            return;
        }
        regression(
            "ook",
            &ook::DEF,
            &ook_option_case("oregon_diffman", &[("decodeas", "Diff Manchester")]),
            "ook/oregon_bthr968.sr",
        );
        regression(
            "ook",
            &ook::DEF,
            &ook_option_case(
                "oregon_diffman_var0",
                &[("decodeas", "Diff Manchester"), ("diffmanvar", "0")],
            ),
            "ook/oregon_bthr968.sr",
        );
    }

    #[test]
    fn ook_explicit_preambles() {
        if !crate::support::python_available() {
            return;
        }
        for preamble in ["1010", "1111"] {
            regression(
                "ook",
                &ook::DEF,
                &ook_option_case("oregon_preamble", &[("preamble", preamble), ("invert", "no")]),
                "ook/oregon_bthr968.sr",
            );
        }
    }

    #[test]
    fn tlc5620_without_ldac() {
        if !crate::support::python_available() {
            return;
        }
        // LOAD is assigned but LDAC is not, so `self.ldac` is not 0 and
        // writes take the register path.
        regression(
            "tlc5620",
            &tlc5620::DEF,
            &case("ti_tlc5620_no_ldac", &[("data", 0), ("clk", 1), ("load", 2)]),
            "tlc5620/ti_tlc5620_one_change.sr",
        );
    }

    #[test]
    fn tlc5620_without_load() {
        if !crate::support::python_available() {
            return;
        }
        // Only DATA and CLK assigned: no LOAD edge ever happens.
        regression(
            "tlc5620",
            &tlc5620::DEF,
            &case("ti_tlc5620_no_load", &[("data", 0), ("clk", 1)]),
            "tlc5620/ti_tlc5620_one_change.sr",
        );
    }

    #[test]
    fn mcs48_without_a12() {
        if !crate::support::python_available() {
            return;
        }
        let mut without_bank = mcs48_case();
        without_bank.channels.retain(|(id, _)| id != "a12");
        regression("mcs48", &mcs48::DEF, &without_bank, "mcs48/i8039_sample.sr");
    }

    #[test]
    fn maple_bus_regression() {
        if !crate::support::python_available() {
            return;
        }
        regression("maple_bus", &maple_bus::DEF, &maple_bus_case(), "maple_bus/bus_enumeration.sr");
    }

    #[test]
    fn ook_regression() {
        if !crate::support::python_available() {
            return;
        }
        regression("ook", &ook::DEF, &ook_case(), "ook/oregon_bthr968.sr");
    }

    #[test]
    fn signature_regression() {
        if !crate::support::python_available() {
            return;
        }
        regression("signature", &signature::DEF, &signature_case(), "signature/0003.sr");
    }

    #[test]
    fn tlc5620_regression() {
        if !crate::support::python_available() {
            return;
        }
        regression("tlc5620", &tlc5620::DEF, &tlc5620_case(), "tlc5620/ti_tlc5620_one_change.sr");
    }

    #[test]
    fn microwire_regression() {
        if !crate::support::python_available() {
            return;
        }
        regression("microwire", &microwire::DEF, &microwire_case(), "microwire/st_m93c66.sr");
    }

    #[test]
    fn mcs48_regression() {
        if !crate::support::python_available() {
            return;
        }
        regression("mcs48", &mcs48::DEF, &mcs48_case(), "mcs48/i8039_sample.sr");
    }

    #[test]
    fn iec_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&iec::DEF, "iec");
        eprintln!("iec corpus: {ran} ran, {skipped} skipped");
        // sigrok-test has no iec cases; nothing to require.
    }

    #[test]
    fn maple_bus_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&maple_bus::DEF, "maple_bus");
        eprintln!("maple_bus corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no maple_bus corpus case ran");
    }

    #[test]
    fn ook_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&ook::DEF, "ook");
        eprintln!("ook corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no ook corpus case ran");
    }

    #[test]
    fn signature_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&signature::DEF, "signature");
        eprintln!("signature corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no signature corpus case ran");
    }

    #[test]
    fn tlc5620_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&tlc5620::DEF, "tlc5620");
        eprintln!("tlc5620 corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no tlc5620 corpus case ran");
    }

    #[test]
    fn microwire_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&microwire::DEF, "microwire");
        eprintln!("microwire corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no microwire corpus case ran");
    }

    #[test]
    fn mcs48_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&mcs48::DEF, "mcs48");
        eprintln!("mcs48 corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no mcs48 corpus case ran");
    }
}
