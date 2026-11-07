// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `ade77xx`, `adf435x`, `adns5020`, `cc1101`, `enc28j60` and
//! `sdcard_spi` against the unchanged libsigrokdecode Python decoders of
//! the same ids, all stacked on `spi`.
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
        Capture, CorpusCase, assert_corpus_parity_def, assert_parity_def, corpus_config, load,
        python, run_chain,
    };
    use oxrok::decode::{Config, DecoderDef, Output};
    use oxrok::decoders::{ade77xx, adf435x, adns5020, cc1101, enc28j60, sdcard_spi, spi};

    /// One decoder of a chain, bottom first.
    type Parts = Vec<(&'static DecoderDef, Config)>;

    const ADE77XX: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/ade77xx/ade7758-phase-b-zx-irq-context.sr"
    );
    const ADF435X: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/adf435x/set-4000mhz.sr");
    const ADNS5020: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/adns5020/adns5020-cy7c63813_init.sr"
    );
    const CC1101: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/cc1101/cc1101-read-write.sr");
    // A 1M sample excerpt from the start of sigrok-dumps'
    // enc28j60-init-and-ping.sr, which is a 1 GB session file when
    // unpacked and too slow for the committed regression test.
    const ENC28J60: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/enc28j60/enc28j60-init-snippet.sr"
    );
    const SDCARD_SPI: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/sdcard_spi/cmd17.sr");

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

    /// The native and Python chain for a target case.
    fn chains(
        target: &'static DecoderDef,
        spi_case: &CorpusCase,
        target_case: &CorpusCase,
    ) -> (Parts, Parts) {
        let spi_cfg = corpus_config(spi::DEF.info, spi_case);
        let target_cfg = corpus_config(target.info, target_case);
        let native = vec![(&spi::DEF, spi_cfg.clone()), (target, target_cfg.clone())];
        let reference = vec![(python("spi"), spi_cfg), (python(target.info.id), target_cfg)];
        (native, reference)
    }

    fn comparable(output: &Output) -> Output {
        match output {
            Output::End { instance, error } => Output::End {
                instance: instance.clone(),
                error: error.as_ref().map(|_| "<error>".to_owned()),
            },
            _ => output.clone(),
        }
    }

    fn assert_chain(label: &str, native: &Parts, reference: &Parts, capture: &Capture) {
        let native = run_chain(native, capture);
        let reference = run_chain(reference, capture);
        assert_eq!(
            native.len(),
            reference.len(),
            "{label}: output count differs\nnative: {native:#?}\nreference: {reference:#?}"
        );
        for (i, (n, p)) in native.iter().zip(&reference).enumerate() {
            assert!(
                comparable(n) == comparable(p),
                "{label}: output {i} differs\nnative: {n:#?}\nreference: {p:#?}"
            );
        }
        // The target must have produced something beyond the lower SPI
        // layer's own output.
        assert!(native.len() > 8, "{label}: the chain emitted too little");
    }

    #[test]
    fn decoder_info_matches_python() {
        for def in [
            &ade77xx::DEF,
            &adf435x::DEF,
            &adns5020::DEF,
            &cc1101::DEF,
            &enc28j60::DEF,
            &sdcard_spi::DEF,
        ] {
            assert_eq!(def.info, python(def.info.id).info, "{}: decoder info differs", def.info.id);
        }
    }

    /// The lower `spi` layer on one of the fixtures, through the plain
    /// single-decoder helper.
    #[test]
    fn spi_layer_parity() {
        let capture = load(CC1101);
        assert_parity_def(&spi::DEF, "spi", &[], &capture);
    }

    #[test]
    fn ade77xx_regression() {
        // The ADE7758 dump has no CS#: clk=0, miso=1, mosi=2.
        let capture = load(ADE77XX);
        let spi_case = case("ade7758", &[("clk", 0), ("miso", 1), ("mosi", 2)], &[]);
        let target_case = case("ade77xx", &[], &[]);
        let (native, reference) = chains(&ade77xx::DEF, &spi_case, &target_case);
        assert_chain("ade77xx", &native, &reference, &capture);
    }

    #[test]
    fn adf435x_regression() {
        // sigrok-test's set-4000mhz: cs=0, mosi=1, clk=2.
        let capture = load(ADF435X);
        let spi_case = case("set-4000mhz", &[("cs", 0), ("mosi", 1), ("clk", 2)], &[]);
        let target_case = case("adf435x", &[], &[]);
        let (native, reference) = chains(&adf435x::DEF, &spi_case, &target_case);
        assert_chain("adf435x", &native, &reference, &capture);
    }

    #[test]
    fn adns5020_regression() {
        // sigrok-test's init: clk=4, mosi=3, cs=5.
        let capture = load(ADNS5020);
        let spi_case = case("init", &[("clk", 4), ("mosi", 3), ("cs", 5)], &[]);
        let target_case = case("adns5020", &[], &[]);
        let (native, reference) = chains(&adns5020::DEF, &spi_case, &target_case);
        assert_chain("adns5020", &native, &reference, &capture);
    }

    #[test]
    fn cc1101_regression() {
        // sigrok-test's cc1101-read-write: mosi=0, miso=2, clk=1, cs=5.
        let capture = load(CC1101);
        let spi_case =
            case("cc1101-read-write", &[("mosi", 0), ("miso", 2), ("clk", 1), ("cs", 5)], &[]);
        let target_case = case("cc1101", &[], &[]);
        let (native, reference) = chains(&cc1101::DEF, &spi_case, &target_case);
        assert_chain("cc1101", &native, &reference, &capture);
    }

    #[test]
    fn enc28j60_regression() {
        // sigrok-test's enc28j60-init-and-ping: cs=0, miso=1, clk=2, mosi=3.
        let capture = load(ENC28J60);
        let spi_case =
            case("enc28j60-init-and-ping", &[("cs", 0), ("miso", 1), ("clk", 2), ("mosi", 3)], &[]);
        let target_case = case("enc28j60", &[], &[]);
        let (native, reference) = chains(&enc28j60::DEF, &spi_case, &target_case);
        assert_chain("enc28j60", &native, &reference, &capture);
    }

    #[test]
    fn sdcard_spi_regression() {
        // sigrok-test's cmd17: cs=0, mosi=1, clk=2, miso=3.
        let capture = load(SDCARD_SPI);
        let spi_case = case("cmd17", &[("cs", 0), ("mosi", 1), ("clk", 2), ("miso", 3)], &[]);
        let target_case = case("sdcard_spi", &[], &[]);
        let (native, reference) = chains(&sdcard_spi::DEF, &spi_case, &target_case);
        assert_chain("sdcard_spi", &native, &reference, &capture);
    }

    #[test]
    fn ade77xx_corpus() {
        if !corpus_enabled() {
            return;
        }
        // There is no ade77xx test.conf in sigrok-test yet.
        let (ran, skipped) = assert_corpus_parity_def(&ade77xx::DEF, "ade77xx");
        eprintln!("ade77xx corpus: {ran} ran, {skipped} skipped");
    }

    #[test]
    fn adf435x_corpus() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&adf435x::DEF, "adf435x");
        eprintln!("adf435x corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no adf435x corpus case ran");
    }

    #[test]
    fn adns5020_corpus() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&adns5020::DEF, "adns5020");
        eprintln!("adns5020 corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no adns5020 corpus case ran");
    }

    #[test]
    fn cc1101_corpus() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&cc1101::DEF, "cc1101");
        eprintln!("cc1101 corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no cc1101 corpus case ran");
    }

    #[test]
    fn enc28j60_corpus() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&enc28j60::DEF, "enc28j60");
        eprintln!("enc28j60 corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no enc28j60 corpus case ran");
    }

    #[test]
    fn sdcard_spi_corpus() {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&sdcard_spi::DEF, "sdcard_spi");
        eprintln!("sdcard_spi corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no sdcard_spi corpus case ran");
    }
}
