// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `sdcard_sd`, `st7735`, `rgb_led_ws281x`, `sda2506` and
//! `seven_segment` against the unchanged libsigrokdecode Python
//! decoders and the sigrok-test corpus.
//!
//! These tests need the `python` or `rustpython` feature and a
//! libsigrokdecode decoders directory. Without either they are compiled
//! away, so a plain `cargo test` still passes.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{
        Capture, CorpusCase, assert_corpus_parity_def, assert_parity_def, corpus_config, load,
        python, run,
    };
    use oxrok::decode::DecoderDef;
    use oxrok::decoders::{rgb_led_ws281x, sda2506, sdcard_sd, seven_segment, st7735};

    const SDCARD_SD: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/sdcard_sd/working_snippet.sr"
    );
    const ST7735: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/st7735/st7735_snippet.sr");
    const RGB_LED_WS281X: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/rgb_led_ws281x/ws2812b_neopixel24_4mhz_snippet.sr"
    );
    const SDA2506: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/sda2506/blaupunkt-enter-wrong-code.sr"
    );
    const SEVEN_SEGMENT: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/seven_segment/test_7_segment_0-9.sr"
    );

    fn available(id: &str) -> bool {
        let found = oxrok::pydecode::defs().iter().any(|d| d.info.id == id);
        if !found {
            eprintln!("no Python decoder '{id}' found, skipping");
        }
        found
    }

    /// Runs a hand-built corpus case through both implementations.
    fn assert_case(
        def: &'static DecoderDef,
        python_id: &str,
        case: &CorpusCase,
        capture: &Capture,
    ) {
        let cfg = corpus_config(def.info, case);
        let native = run(def, &cfg, capture);
        let reference = run(python(python_id), &cfg, capture);
        assert_eq!(
            native.len(),
            reference.len(),
            "{}: output count differs\nnative: {native:#?}\npython: {reference:#?}",
            case.name
        );
        for (i, (n, p)) in native.iter().zip(&reference).enumerate() {
            assert_eq!(n, p, "{}: output {i} differs\nnative: {n:#?}\npython: {p:#?}", case.name);
        }
    }

    fn case(name: &str, channels: &[(&str, usize)]) -> CorpusCase {
        CorpusCase {
            name: name.to_owned(),
            channels: channels.iter().map(|(id, i)| ((*id).to_owned(), *i)).collect(),
            options: Vec::new(),
            input: None,
            stack: Vec::new(),
        }
    }

    #[test]
    fn decoder_info_matches_python() {
        for (id, def) in [
            ("sdcard_sd", &sdcard_sd::DEF),
            ("st7735", &st7735::DEF),
            ("rgb_led_ws281x", &rgb_led_ws281x::DEF),
            ("sda2506", &sda2506::DEF),
            ("seven_segment", &seven_segment::DEF),
        ] {
            if !available(id) {
                continue;
            }
            assert_eq!(def.info, python(id).info, "{id}: decoder info differs");
        }
    }

    #[test]
    fn sdcard_sd_fixture_parity() {
        if !available("sdcard_sd") {
            return;
        }
        let capture = load(SDCARD_SD);
        // The channel and index from sigrok-test's test.conf.
        let case = case("imx6_quad_working_snippet", &[("cmd", 4), ("clk", 0)]);
        assert_case(&sdcard_sd::DEF, "sdcard_sd", &case, &capture);
    }

    #[test]
    fn st7735_fixture_parity() {
        if !available("st7735") {
            return;
        }
        let capture = load(ST7735);
        let case = case("st7735_snippet", &[("cs", 0), ("dc", 2), ("mosi", 3), ("clk", 4)]);
        assert_case(&st7735::DEF, "st7735", &case, &capture);
    }

    #[test]
    fn rgb_led_ws281x_fixture_parity() {
        if !available("rgb_led_ws281x") {
            return;
        }
        let capture = load(RGB_LED_WS281X);
        let case = case("ws2812b_neopixel24_4mhz_snippet", &[("din", 0)]);
        assert_case(&rgb_led_ws281x::DEF, "rgb_led_ws281x", &case, &capture);
    }

    #[test]
    fn sda2506_fixture_parity() {
        if !available("sda2506") {
            return;
        }
        let capture = load(SDA2506);
        let case = case("blaupunkt-enter-wrong-code", &[("clk", 5), ("d", 2), ("ce", 0)]);
        assert_case(&sda2506::DEF, "sda2506", &case, &capture);
    }

    #[test]
    fn seven_segment_fixture_parity() {
        if !available("seven_segment") {
            return;
        }
        let capture = load(SEVEN_SEGMENT);
        let case = case(
            "test_7_segment_0-9",
            &[("a", 0), ("b", 1), ("c", 2), ("d", 3), ("e", 4), ("f", 5), ("g", 6), ("dp", 7)],
        );
        assert_case(&seven_segment::DEF, "seven_segment", &case, &capture);
    }

    #[test]
    fn rgb_led_ws281x_name_parity() {
        if !available("rgb_led_ws281x") {
            return;
        }
        let capture = load(RGB_LED_WS281X);
        let din = capture.channels[0].clone();
        assert_parity_def(
            &rgb_led_ws281x::DEF,
            "rgb_led_ws281x",
            &[("din", din.as_str())],
            &capture,
        );
    }

    #[test]
    fn seven_segment_common_anode_parity() {
        if !available("seven_segment") {
            return;
        }
        let capture = load(SEVEN_SEGMENT);
        let mut case = case(
            "test_7_segment_common_anode",
            &[("a", 0), ("b", 1), ("c", 2), ("d", 3), ("e", 4), ("f", 5), ("g", 6), ("dp", 7)],
        );
        case.options.push(("polarity".to_owned(), "common-anode".to_owned()));
        assert_case(&seven_segment::DEF, "seven_segment", &case, &capture);
    }

    fn corpus(id: &str, def: &'static DecoderDef) {
        if !available(id) {
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
        let (ran, skipped) = assert_corpus_parity_def(def, id);
        eprintln!("{id}: {ran} corpus cases ran, {skipped} skipped");
        assert!(ran > 0, "{id}: no corpus case ran");
    }

    #[test]
    fn sdcard_sd_corpus_parity() {
        corpus("sdcard_sd", &sdcard_sd::DEF);
    }

    #[test]
    fn st7735_corpus_parity() {
        corpus("st7735", &st7735::DEF);
    }

    #[test]
    fn rgb_led_ws281x_corpus_parity() {
        corpus("rgb_led_ws281x", &rgb_led_ws281x::DEF);
    }

    #[test]
    fn sda2506_corpus_parity() {
        corpus("sda2506", &sda2506::DEF);
    }

    #[test]
    fn seven_segment_corpus_parity() {
        corpus("seven_segment", &seven_segment::DEF);
    }
}
