// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `lpc` and `z80` against the unchanged libsigrokdecode Python
//! decoders of the same ids.
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
    use oxrok::decoders::{lpc, z80};

    const LPC_SR: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/lpc/lpc_vultureprog_snippet.sr"
    );
    const Z80_SYNC_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/z80/kc85-cpuclk.sr");
    const Z80_ASYNC_SR: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/z80/kc85-20mhz.sr");

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

    /// `channel lad0=0 channel lad1=1 channel lad2=2 channel lad3=3
    /// channel lclk=4 channel lframe=5` in sigrok-test.
    fn lpc_case() -> CorpusCase {
        case(
            "lpc_vultureprog_snippet",
            &[("lframe", 5), ("lclk", 4), ("lad0", 0), ("lad1", 1), ("lad2", 2), ("lad3", 3)],
        )
    }

    /// The z80 corpus maps the KC85 capture's 34 probes.
    fn z80_case(name: &str) -> CorpusCase {
        case(
            name,
            &[
                ("m1", 1),
                ("a0", 6),
                ("a1", 7),
                ("a2", 8),
                ("a3", 9),
                ("a4", 10),
                ("a5", 11),
                ("a6", 12),
                ("a7", 13),
                ("a8", 14),
                ("a9", 15),
                ("a10", 16),
                ("a11", 17),
                ("a12", 18),
                ("a13", 19),
                ("a14", 20),
                ("a15", 21),
                ("iorq", 22),
                ("mreq", 23),
                ("rd", 24),
                ("wr", 25),
                ("d0", 26),
                ("d1", 27),
                ("d2", 28),
                ("d3", 29),
                ("d4", 30),
                ("d5", 31),
                ("d6", 32),
                ("d7", 33),
            ],
        )
    }

    #[test]
    fn lpc_regression() {
        if !crate::support::python_available() {
            return;
        }
        let cfg = corpus_config(lpc::DEF.info, &lpc_case());
        let capture = load(LPC_SR);
        let native = run(&lpc::DEF, &cfg, &capture);
        let reference = run(python("lpc"), &cfg, &capture);
        assert_eq!(native, reference, "lpc: native and Python outputs differ");
    }

    #[test]
    fn lpc_named_channels() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(LPC_SR);
        assert_parity_def(
            &lpc::DEF,
            "lpc",
            &[
                ("lframe", "LFRAME#"),
                ("lclk", "LCLK"),
                ("lad0", "LAD0"),
                ("lad1", "LAD1"),
                ("lad2", "LAD2"),
                ("lad3", "LAD3"),
            ],
            &capture,
        );
    }

    #[test]
    fn z80_sync_regression() {
        if !crate::support::python_available() {
            return;
        }
        let cfg = corpus_config(z80::DEF.info, &z80_case("kc85_sync_disasm"));
        let capture = load(Z80_SYNC_SR);
        let native = run(&z80::DEF, &cfg, &capture);
        let reference = run(python("z80"), &cfg, &capture);
        assert_eq!(native, reference, "z80: native and Python outputs differ");
    }

    #[test]
    fn z80_async_regression() {
        if !crate::support::python_available() {
            return;
        }
        let cfg = corpus_config(z80::DEF.info, &z80_case("kc85_async_disasm"));
        let capture = load(Z80_ASYNC_SR);
        let native = run(&z80::DEF, &cfg, &capture);
        let reference = run(python("z80"), &cfg, &capture);
        assert_eq!(native, reference, "z80: native and Python outputs differ");
    }

    #[test]
    fn z80_named_channels() {
        if !crate::support::python_available() {
            return;
        }
        let capture = load(Z80_SYNC_SR);
        assert_parity_def(
            &z80::DEF,
            "z80",
            &[
                ("d0", "D0"),
                ("d1", "D1"),
                ("d2", "D2"),
                ("d3", "D3"),
                ("d4", "D4"),
                ("d5", "D5"),
                ("d6", "D6"),
                ("d7", "D7"),
                ("m1", "/M1"),
                ("rd", "/RD"),
                ("wr", "/WR"),
                ("mreq", "/MREQ"),
                ("iorq", "/IORQ"),
                ("a0", "A0"),
                ("a1", "A1"),
                ("a2", "A2"),
                ("a3", "A3"),
                ("a4", "A4"),
                ("a5", "A5"),
                ("a6", "A6"),
                ("a7", "A7"),
                ("a8", "A8"),
                ("a9", "A9"),
                ("a10", "A10"),
                ("a11", "A11"),
                ("a12", "A12"),
                ("a13", "A13"),
                ("a14", "A14"),
                ("a15", "A15"),
            ],
            &capture,
        );
    }

    #[test]
    fn lpc_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&lpc::DEF, "lpc");
        eprintln!("lpc corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no lpc corpus case ran");
    }

    #[test]
    fn z80_corpus() {
        if !crate::support::python_available() {
            return;
        }
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(&z80::DEF, "z80");
        eprintln!("z80 corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no z80 corpus case ran");
    }
}
