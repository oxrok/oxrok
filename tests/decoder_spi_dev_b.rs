// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `max7219`, `mrf24j40`, `nes_gamepad`, `nrf24l01`, `rfm12`,
//! `rgb_led_spi`, `ssi32` and `x2444m` against the unchanged
//! libsigrokdecode Python decoders of the same ids, both stacked on
//! `spi`.
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
        Capture, CorpusCase, assert_corpus_parity_def, corpus_config, load, python, run_chain,
    };
    use oxrok::decode::{Config, DecoderDef, Output};
    use oxrok::decoders::{
        max7219, mrf24j40, nes_gamepad, nrf24l01, rfm12, rgb_led_spi, spi, ssi32, x2444m,
    };

    const MAX7219_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/max7219/max7219.sr");
    const MRF24J40_SR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/mrf24j40/mrf24j40-reset-wakeup.sr"
    );
    const NES_GAMEPAD_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/nes_gamepad/a.sr");
    const NRF24L01_SR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/nrf24l01/nrf24l01-test-misc.sr"
    );
    const RFM12_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/rfm12/rfm12.sr");
    const RGB_LED_SPI_SR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/rgb_led_spi/ws2801_2ch_5mhz.sr"
    );
    const SSI32_SR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/ssi32/ssi32_bananapi_loop_test.sr"
    );
    const X2444M_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/x2444m/x2444m.sr");

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
        lower: &'static DecoderDef,
        upper: &'static DecoderDef,
        lower_cfg: &Config,
        upper_cfg: &Config,
        capture: &Capture,
    ) -> (Vec<Output>, Vec<Output>) {
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

    /// Compares a native `spi` + decoder chain against the Python one.
    fn assert_fixture(
        id: &str,
        upper: &'static DecoderDef,
        spi_case: &CorpusCase,
        upper_case: &CorpusCase,
        path: &str,
    ) {
        let spi_cfg = corpus_config(spi::DEF.info, spi_case);
        let upper_cfg = corpus_config(upper.info, upper_case);
        let capture = load(path);
        let (native, reference) = chain_pair(&spi::DEF, upper, &spi_cfg, &upper_cfg, &capture);
        assert!(!native.is_empty(), "{id}: no output");
        assert_eq!(
            native.len(),
            reference.len(),
            "{id}: output count differs ({} vs {})",
            native.len(),
            reference.len()
        );
        for (i, (n, p)) in native.iter().zip(&reference).enumerate() {
            assert_eq!(n, p, "{id}: output {i} differs\nnative: {n:#?}\npython: {p:#?}");
        }
    }

    fn corpus(id: &str, def: &'static DecoderDef, cases: usize) {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(def, id);
        eprintln!("{id} corpus: {ran} ran, {skipped} skipped");
        if cases > 0 {
            assert!(ran > 0, "no {id} corpus case ran");
        }
    }

    /// The MAX7219 capture has no sigrok-test case; the README gives the
    /// probes: 0 MISO, 1 CS#, 2 MOSI, 3 CLK.
    #[test]
    fn max7219_regression() {
        if !crate::support::python_available() {
            return;
        }
        let spi_case = case("max7219", &[("cs", 1), ("clk", 3), ("mosi", 2)], &[]);
        let max_case = case("max7219", &[], &[]);
        assert_fixture("max7219", &max7219::DEF, &spi_case, &max_case, MAX7219_SR);
    }

    #[test]
    fn mrf24j40_regression() {
        if !crate::support::python_available() {
            return;
        }
        // mrf24j40-reset-wakeup: cs=5, clk=3, mosi=2, miso=4.
        let spi_case =
            case("mrf24j40-reset-wakeup", &[("cs", 5), ("clk", 3), ("mosi", 2), ("miso", 4)], &[]);
        let dev_case = case("mrf24j40", &[], &[]);
        assert_fixture("mrf24j40", &mrf24j40::DEF, &spi_case, &dev_case, MRF24J40_SR);
    }

    #[test]
    fn nes_gamepad_regression() {
        if !crate::support::python_available() {
            return;
        }
        // nes_gamepad_generated_a: cs=0, clk=2, miso=1, cpol=1.
        let spi_case = case(
            "nes_gamepad_generated_a",
            &[("cs", 0), ("clk", 2), ("miso", 1)],
            &[("cpol", "1")],
        );
        let pad_case = case("nes_gamepad", &[], &[]);
        assert_fixture("nes_gamepad", &nes_gamepad::DEF, &spi_case, &pad_case, NES_GAMEPAD_SR);
    }

    #[test]
    fn nrf24l01_regression() {
        if !crate::support::python_available() {
            return;
        }
        // nrf24l01-test-misc: cs=0, clk=1, mosi=2, miso=3.
        let spi_case =
            case("nrf24l01-test-misc", &[("cs", 0), ("clk", 1), ("mosi", 2), ("miso", 3)], &[]);
        let chip_case = case("nrf24l01", &[], &[]);
        assert_fixture("nrf24l01", &nrf24l01::DEF, &spi_case, &chip_case, NRF24L01_SR);
    }

    #[test]
    fn rfm12_regression() {
        if !crate::support::python_available() {
            return;
        }
        // rfm12: clk=0, miso=2, mosi=1, cs=3.
        let spi_case = case("rfm12", &[("clk", 0), ("miso", 2), ("mosi", 1), ("cs", 3)], &[]);
        let rf_case = case("rfm12", &[], &[]);
        assert_fixture("rfm12", &rfm12::DEF, &spi_case, &rf_case, RFM12_SR);
    }

    #[test]
    fn rgb_led_spi_regression() {
        if !crate::support::python_available() {
            return;
        }
        // ws2801_2ch_5mhz: clk=1, mosi=0.
        let spi_case = case("ws2801_2ch_5mhz", &[("clk", 1), ("mosi", 0)], &[]);
        let led_case = case("rgb_led_spi", &[], &[]);
        assert_fixture("rgb_led_spi", &rgb_led_spi::DEF, &spi_case, &led_case, RGB_LED_SPI_SR);
    }

    #[test]
    fn ssi32_regression() {
        if !crate::support::python_available() {
            return;
        }
        // ssi32_bananapi_loop_test: clk=2, miso=1, mosi=0, cs=3.
        let spi_case = case(
            "ssi32_bananapi_loop_test",
            &[("clk", 2), ("miso", 1), ("mosi", 0), ("cs", 3)],
            &[],
        );
        let dev_case = case("ssi32", &[], &[]);
        assert_fixture("ssi32", &ssi32::DEF, &spi_case, &dev_case, SSI32_SR);
    }

    #[test]
    fn x2444m_regression() {
        if !crate::support::python_available() {
            return;
        }
        // x2444m: clk=0, mosi=1, miso=2, cs=3, cs_polarity=active-high.
        let spi_case = case(
            "x2444m",
            &[("clk", 0), ("mosi", 1), ("miso", 2), ("cs", 3)],
            &[("cs_polarity", "active-high")],
        );
        let ram_case = case("x2444m", &[], &[]);
        assert_fixture("x2444m", &x2444m::DEF, &spi_case, &ram_case, X2444M_SR);
    }

    #[test]
    fn decoder_info_matches_python() {
        if !crate::support::python_available() {
            return;
        }
        for (id, def) in [
            ("max7219", &max7219::DEF),
            ("mrf24j40", &mrf24j40::DEF),
            ("nes_gamepad", &nes_gamepad::DEF),
            ("nrf24l01", &nrf24l01::DEF),
            ("rfm12", &rfm12::DEF),
            ("rgb_led_spi", &rgb_led_spi::DEF),
            ("ssi32", &ssi32::DEF),
            ("x2444m", &x2444m::DEF),
        ] {
            assert_eq!(def.info, python(id).info, "{id}: decoder info differs");
        }
    }

    #[test]
    fn max7219_corpus() {
        if !crate::support::python_available() {
            return;
        }
        // sigrok-test has no max7219 test.conf.
        corpus("max7219", &max7219::DEF, 0);
    }

    #[test]
    fn mrf24j40_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("mrf24j40", &mrf24j40::DEF, 4);
    }

    #[test]
    fn nes_gamepad_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("nes_gamepad", &nes_gamepad::DEF, 12);
    }

    #[test]
    fn nrf24l01_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("nrf24l01", &nrf24l01::DEF, 14);
    }

    #[test]
    fn rfm12_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("rfm12", &rfm12::DEF, 1);
    }

    #[test]
    fn rgb_led_spi_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("rgb_led_spi", &rgb_led_spi::DEF, 1);
    }

    #[test]
    fn ssi32_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("ssi32", &ssi32::DEF, 2);
    }

    #[test]
    fn x2444m_corpus() {
        if !crate::support::python_available() {
            return;
        }
        corpus("x2444m", &x2444m::DEF, 1);
    }
}
