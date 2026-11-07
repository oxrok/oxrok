// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Native `edid`, `hdcp`, `mlx90614`, `mxc6225xu`, `nunchuk` and `xfp`
//! against the unchanged libsigrokdecode Python decoders of the same ids.
//!
//! Every one of them is stacked on the native `i2c` (and `mxc6225xu`
//! also on the native `i2cfilter`), and the Python chain is built from
//! the same configuration, so a difference in either the lower decoder's
//! items or the target's outputs shows up. Everything needs the `python`
//! or `rustpython` feature; a plain `cargo test` compiles it away.

#[cfg(any(feature = "python", feature = "rustpython"))]
mod support;

#[cfg(any(feature = "python", feature = "rustpython"))]
mod tests {
    use crate::support::{
        Capture, CorpusCase, assert_corpus_parity_def, assert_parity_def, corpus_config, load,
        python, run_chain,
    };
    use oxrok::decode::{Config, DecoderDef, Output};
    use oxrok::decoders::{edid, hdcp, i2c, i2cfilter, mlx90614, mxc6225xu, nunchuk, xfp};

    /// One decoder of a chain, bottom first.
    type Parts = Vec<(&'static DecoderDef, Config)>;

    const EDID: &str =
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/edid/samsung_le46b620r3p.sr");
    const HDCP: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/hdcp/hdcp.sr");
    const MLX90614: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/mlx90614/melexis_mlx90614_5s_24deg.sr"
    );
    const MXC6225XU: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/mxc6225xu/trekstor_ebr30_a_i2c_0x15.sr"
    );
    const NUNCHUK: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/decoders/nunchuk/wii_nunchuk_data_button_c.sr"
    );
    const XFP: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/decoders/xfp/xfp.sr");

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

    fn i2c_case(name: &str, scl: usize, sda: usize) -> CorpusCase {
        case(name, &[("scl", scl), ("sda", sda)], &[])
    }

    /// Native and Python chains from bottom to top.
    fn chains(
        target: &'static DecoderDef,
        i2c_case: &CorpusCase,
        filter_case: Option<&CorpusCase>,
        target_case: &CorpusCase,
    ) -> (Parts, Parts) {
        let i2c_cfg = corpus_config(i2c::DEF.info, i2c_case);
        let filter_cfg = filter_case.map(|c| corpus_config(i2cfilter::DEF.info, c));
        let target_cfg = corpus_config(target.info, target_case);
        let mut native = vec![(&i2c::DEF, i2c_cfg.clone())];
        let mut reference = vec![(python("i2c"), i2c_cfg)];
        if let Some(cfg) = filter_cfg {
            native.push((&i2cfilter::DEF, cfg.clone()));
            reference.push((python("i2cfilter"), cfg));
        }
        native.push((target, target_cfg.clone()));
        reference.push((python(target.info.id), target_cfg));
        (native, reference)
    }

    fn assert_chain(label: &str, native: &Parts, reference: &Parts, capture: &Capture) {
        let native_out = run_chain(native, capture);
        let reference_out = run_chain(reference, capture);
        assert_eq!(
            native_out.len(),
            reference_out.len(),
            "{label}: output count differs\nnative: {native_out:#?}\nreference: {reference_out:#?}"
        );
        for (i, (n, p)) in native_out.iter().zip(&reference_out).enumerate() {
            assert!(
                comparable(n) == comparable(p),
                "{label}: output {i} differs\nnative: {n:#?}\nreference: {p:#?}"
            );
        }
        assert!(!native_out.is_empty(), "{label}: the chain emitted nothing");
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

    #[test]
    fn info_matches_python() {
        for def in
            [&edid::DEF, &hdcp::DEF, &mlx90614::DEF, &mxc6225xu::DEF, &nunchuk::DEF, &xfp::DEF]
        {
            assert_eq!(def.info, python(def.info.id).info);
        }
    }

    /// A stacked decoder has no logic input of its own; native and Python
    /// must both end the run with an error and no annotations.
    #[test]
    fn stacked_decoders_reject_logic_input() {
        let capture = load(NUNCHUK);
        for def in
            [&edid::DEF, &hdcp::DEF, &mlx90614::DEF, &mxc6225xu::DEF, &nunchuk::DEF, &xfp::DEF]
        {
            assert_parity_def(def, def.info.id, &[], &capture);
        }
    }

    #[test]
    fn edid_regression() {
        let capture = load(EDID);
        let (native, reference) = chains(
            &edid::DEF,
            &i2c_case("samsung_le46b620r3p", 0, 1),
            None,
            &case("edid", &[], &[]),
        );
        assert_chain("i2c->edid", &native, &reference, &capture);
    }

    #[test]
    fn hdcp_regression() {
        let capture = load(HDCP);
        let (native, reference) =
            chains(&hdcp::DEF, &i2c_case("unknown_device", 0, 1), None, &case("hdcp", &[], &[]));
        assert_chain("i2c->hdcp", &native, &reference, &capture);
    }

    #[test]
    fn mlx90614_regression() {
        let capture = load(MLX90614);
        let (native, reference) =
            chains(&mlx90614::DEF, &i2c_case("5s_24deg", 5, 7), None, &case("mlx90614", &[], &[]));
        assert_chain("i2c->mlx90614", &native, &reference, &capture);
    }

    #[test]
    fn mxc6225xu_regression() {
        let capture = load(MXC6225XU);
        let (native, reference) = chains(
            &mxc6225xu::DEF,
            &i2c_case("trekstor_ebr30_a", 0, 1),
            Some(&case("trekstor_52", &[], &[("address", "21")])),
            &case("mxc6225xu", &[], &[]),
        );
        assert_chain("i2c->i2cfilter->mxc6225xu", &native, &reference, &capture);
    }

    #[test]
    fn nunchuk_regression() {
        let capture = load(NUNCHUK);
        let (native, reference) = chains(
            &nunchuk::DEF,
            &i2c_case("wii_nunchuk_data_button_c", 0, 1),
            None,
            &case("nunchuk", &[], &[]),
        );
        assert_chain("i2c->nunchuk", &native, &reference, &capture);
    }

    #[test]
    fn xfp_regression() {
        let capture = load(XFP);
        let (native, reference) =
            chains(&xfp::DEF, &i2c_case("xfp", 0, 1), None, &case("xfp", &[], &[]));
        assert_chain("i2c->xfp", &native, &reference, &capture);
    }

    fn corpus(name: &str, def: &'static DecoderDef) {
        if !corpus_enabled() {
            return;
        }
        let (ran, skipped) = assert_corpus_parity_def(def, name);
        eprintln!("{name} corpus: {ran} ran, {skipped} skipped");
        assert!(ran > 0, "no {name} corpus case ran");
    }

    #[test]
    fn edid_corpus() {
        corpus("edid", &edid::DEF);
    }

    #[test]
    fn hdcp_corpus() {
        corpus("hdcp", &hdcp::DEF);
    }

    #[test]
    fn mlx90614_corpus() {
        corpus("mlx90614", &mlx90614::DEF);
    }

    #[test]
    fn mxc6225xu_corpus() {
        corpus("mxc6225xu", &mxc6225xu::DEF);
    }

    #[test]
    fn nunchuk_corpus() {
        corpus("nunchuk", &nunchuk::DEF);
    }

    #[test]
    fn xfp_corpus() {
        corpus("xfp", &xfp::DEF);
    }
}
