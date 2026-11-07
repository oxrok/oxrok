// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `avr_isp` and `spiflash` against the unchanged
//! libsigrokdecode Python decoders of the same ids, both stacked on `spi`.
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
        CorpusCase, assert_corpus_parity_def, corpus_config, load, python, run_chain,
    };
    use oxrok::decoders::{avr_isp, spi, spiflash};

    const AVR_ISP_SR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/avr_isp/isp_atmega88_scan.sr"
    );
    const SPIFLASH_SR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/spiflash/mx25l1605d_probe.sr"
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

    /// Runs the native chain and the Python chain of the same configs
    /// over `capture` and returns `(native, python)` outputs.
    fn chain_pair(
        lower: &'static oxrok::decode::DecoderDef,
        upper: &'static oxrok::decode::DecoderDef,
        lower_cfg: &oxrok::decode::Config,
        upper_cfg: &oxrok::decode::Config,
        capture: &crate::support::Capture,
    ) -> (Vec<oxrok::decode::Output>, Vec<oxrok::decode::Output>) {
        let native = run_chain(&[(lower, lower_cfg.clone()), (upper, upper_cfg.clone())], capture);
        let python = run_chain(
            &[
                (python(lower.info.id), lower_cfg.clone()),
                (python(upper.info.id), upper_cfg.clone()),
            ],
            capture,
        );
        (native, python)
    }

    #[test]
    fn avr_isp_regression() {
        if !crate::support::python_available() {
            return;
        }
        // sigrok-test's atmega88_scan: cs=1, clk=2, mosi=0, miso=3.
        let spi_case =
            case("atmega88_scan", &[("cs", 1), ("clk", 2), ("mosi", 0), ("miso", 3)], &[]);
        let avr_case = case("avr_isp", &[], &[]);
        let spi_cfg = corpus_config(spi::DEF.info, &spi_case);
        let avr_cfg = corpus_config(avr_isp::DEF.info, &avr_case);
        let capture = load(AVR_ISP_SR);
        let (native, reference) =
            chain_pair(&spi::DEF, &avr_isp::DEF, &spi_cfg, &avr_cfg, &capture);
        assert!(!native.is_empty(), "avr_isp: no output");
        assert_eq!(native, reference, "avr_isp: native and Python outputs differ");
    }

    #[test]
    fn spiflash_regression() {
        if !crate::support::python_available() {
            return;
        }
        // sigrok-test's probe_hex: cs=0, miso=1, clk=2, mosi=3.
        let spi_case = case("probe_hex", &[("cs", 0), ("miso", 1), ("clk", 2), ("mosi", 3)], &[]);
        let flash_case =
            case("spiflash", &[], &[("chip", "macronix_mx25l1605d"), ("format", "hex")]);
        let spi_cfg = corpus_config(spi::DEF.info, &spi_case);
        let flash_cfg = corpus_config(spiflash::DEF.info, &flash_case);
        let capture = load(SPIFLASH_SR);
        let (native, reference) =
            chain_pair(&spi::DEF, &spiflash::DEF, &spi_cfg, &flash_cfg, &capture);
        assert!(!native.is_empty(), "spiflash: no output");
        assert_eq!(native, reference, "spiflash: native and Python outputs differ");
    }

    #[test]
    fn avr_isp_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&avr_isp::DEF, "avr_isp");
        eprintln!("avr_isp corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no avr_isp corpus case ran");
    }

    #[test]
    fn spiflash_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&spiflash::DEF, "spiflash");
        eprintln!("spiflash corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no spiflash corpus case ran");
    }
}
