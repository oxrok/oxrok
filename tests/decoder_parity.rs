// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! In-process parity between a native decoder and the unchanged
//! libsigrokdecode Python decoder of the same id.
//!
//! These tests need the `python` or `rustpython` feature and a
//! libsigrokdecode decoders directory. Without either they are compiled
//! away, so a plain `cargo test` still passes.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{assert_parity, load, load_vcd, python};

    const UART: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/uart-hi.vcd");
    const I2C: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/i2c-write-read.vcd");
    const SPI: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/spi-mode0.vcd");
    const UART_ESP32: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/uart-esp32.sr");
    const I2C_ESP32: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/i2c-esp32.sr");
    const SPI_ESP32: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/spi-esp32.sr");

    fn available(id: &str) -> bool {
        let found = oxrok::pydecode::defs().iter().any(|d| d.info.id == id);
        if !found {
            eprintln!("no Python decoder '{id}' found, skipping");
        }
        found
    }

    #[test]
    fn uart_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("uart") {
            return;
        }
        let capture = load_vcd(UART);
        assert_parity("uart", &[("rx", "top.D0")], &capture);
    }

    #[test]
    fn uart_esp32_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("uart") {
            return;
        }
        let capture = load(UART_ESP32);
        assert_parity("uart", &[("rx", "D21"), ("baudrate", "921600")], &capture);
        assert_parity(
            "uart",
            &[("rx", "D21"), ("baudrate", "921600"), ("format", "ascii")],
            &capture,
        );
    }

    #[test]
    fn uart_option_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("uart") {
            return;
        }
        let capture = load_vcd(UART);
        for pairs in [
            &[("rx", "top.D0"), ("format", "ascii")][..],
            &[("rx", "top.D0"), ("bit_order", "msb-first")][..],
            &[("rx", "top.D0"), ("data_bits", "7")][..],
            &[("rx", "top.D0"), ("parity", "even")][..],
            &[("rx", "top.D0"), ("stop_bits", "1.5")][..],
            &[("rx", "top.D0"), ("invert_rx", "yes")][..],
        ] {
            eprintln!("uart options {pairs:?}");
            assert_parity("uart", pairs, &capture);
        }
    }

    #[test]
    fn i2c_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("i2c") {
            return;
        }
        let capture = load_vcd(I2C);
        assert_parity("i2c", &[], &capture);
    }

    #[test]
    fn i2c_esp32_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("i2c") {
            return;
        }
        let capture = load(I2C_ESP32);
        assert_parity("i2c", &[("scl", "D11"), ("sda", "D12")], &capture);
    }

    #[test]
    fn i2c_option_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("i2c") {
            return;
        }
        let capture = load_vcd(I2C);
        for pairs in [&[("address_format", "unshifted")][..], &[("address_format", "shifted")][..]]
        {
            assert_parity("i2c", pairs, &capture);
        }
    }

    #[test]
    fn spi_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("spi") {
            return;
        }
        let capture = load_vcd(SPI);
        assert_parity("spi", &[], &capture);
    }

    #[test]
    fn spi_esp32_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("spi") {
            return;
        }
        let capture = load(SPI_ESP32);
        assert_parity(
            "spi",
            &[("clk", "D28"), ("miso", "D26"), ("mosi", "D5"), ("cs", "D25")],
            &capture,
        );
    }

    #[test]
    fn spi_option_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("spi") {
            return;
        }
        let capture = load_vcd(SPI);
        for pairs in [
            &[("cpol", "1"), ("cpha", "1")][..],
            &[("bitorder", "lsb-first")][..],
            &[("wordsize", "4")][..],
            &[("cs_polarity", "active-high")][..],
        ] {
            assert_parity("spi", pairs, &capture);
        }
    }

    /// The harness must be able to run the Python decoder even when a
    /// native decoder has the same id.
    #[test]
    fn python_decoder_is_forced_by_id() {
        if !crate::support::python_available() {
            return;
        }
        if !available("uart") {
            return;
        }
        let capture = load_vcd(UART);
        let def = python("uart");
        let cfg = crate::support::config(def.info, &[("rx", "top.D0")], &capture.channels);
        let outputs = crate::support::run(def, &cfg, &capture);
        assert!(
            outputs.iter().any(|o| matches!(o, oxrok::decode::Output::Annotation { class: 0, .. }))
        );
    }

    /// The pilot decoders against every single-decoder case in the
    /// sigrok-test corpus. The checkouts are optional; without them the
    /// test says so and returns.
    #[test]
    fn pilot_corpus_parity() {
        if !crate::support::python_available() {
            return;
        }
        if !available("uart") {
            return;
        }
        if std::env::var_os("OXROK_CORPUS").is_none() {
            eprintln!("set OXROK_CORPUS=1 to run the sigrok-test corpus");
            return;
        }
        if crate::support::corpus_test_dir().is_none()
            || crate::support::corpus_dumps_dir().is_none()
        {
            eprintln!("no sigrok-test/sigrok-dumps checkout, skipping corpus");
            return;
        }
        for id in ["uart", "i2c", "spi"] {
            if std::env::var("OXROK_CORPUS_DECODER").is_ok_and(|only| only != id) {
                continue;
            }
            let (ran, skipped) = crate::support::assert_corpus_parity(id);
            eprintln!("{id}: {ran} corpus cases ran, {skipped} skipped");
            assert!(ran > 0, "{id}: no corpus case ran");
        }
    }
}
