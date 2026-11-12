// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Throughput of the host software triggers, in million samples per
//! second: how fast a stream they keep up with.
//!
//! ```text
//! cargo run --release --example trigger_bench [-- OPTIONS]
//!   --samples N   samples per run, default 64 Mi
//!   --data KIND   quiet, counter or random (default: all three)
//!   --unit N      bytes per sample, 1, 2 or 4 (default: 1 and 2)
//!   --min MS/s    exit with an error when a run is slower than this
//! ```
//!
//! The data arrives in fx2lafw's transfers at 24 MHz. The top channel
//! is held low and every trigger needs it, so none fires and each run
//! goes over all the samples; a column marked `!N` fired N times anyway,
//! which is a bug. The columns are libsigrok's matcher
//! ([`SoftTriggerLogic`]), the general one ([`SoftTrigger`]), and the
//! general one with fx2lafw's pre-trigger buffer of 1 Mi samples.

use oxrok::core::trigger::{
    Trigger, TriggerCombine, TriggerCount, TriggerDuration, TriggerMatch as M, TriggerRange,
};
use oxrok::core::{PreTriggerBuffer, SoftTrigger, SoftTriggerLogic};
use oxrok::drivers::fx2lafw::transfer_plan;
use std::hint::black_box;
use std::process::ExitCode;
use std::time::Instant;

/// Samples of `unit` bytes, the top channel always low.
fn samples(kind: &str, unit: usize, n: usize) -> Vec<u8> {
    let top = 1u64 << (unit * 8 - 1);
    let mut x = 0x9E37_79B9_7F4A_7C15u64;
    let mut out = Vec::with_capacity(n * unit);
    for i in 0..n as u64 {
        let v = match kind {
            // A short burst every 100 000 samples.
            "quiet" => u64::from(i % 100_000 < 50),
            "counter" => i,
            _ => {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                x
            }
        };
        out.extend_from_slice(&(v & !top).to_le_bytes()[..unit]);
    }
    out
}

fn conds(stages: &[&[(usize, M)]]) -> Trigger {
    let mut t = Trigger::new();
    for (i, s) in stages.iter().enumerate() {
        for &(ch, m) in *s {
            t.add(i, ch, m);
        }
    }
    t
}

/// The triggers, each on the top channel `top`.
fn triggers(top: usize) -> Vec<(&'static str, Trigger)> {
    let mut v = vec![
        ("edge", conds(&[&[(top, M::Rising)]])),
        ("edge + 3 levels", conds(&[&[(0, M::Rising), (1, M::One), (2, M::One), (top, M::One)]])),
        ("2 stages", conds(&[&[(0, M::Rising)], &[(top, M::One)]])),
    ];
    let mut t = conds(&[&[(top, M::One)]]);
    t.set_range(0, TriggerRange { channels: vec![0, 1, 2, 3], low: 3, high: 5, inside: true });
    v.push(("range D0-D3 + level", t));
    let mut t = Trigger::new();
    t.set_range(
        0,
        TriggerRange { channels: (0..=top).collect(), low: 1 << top, high: u64::MAX, inside: true },
    );
    v.push(("range, all channels", t));
    // Every value is inside, so it never matches, and a block filter
    // can't tell: the matcher's cost when nothing can be skipped.
    let mut t = Trigger::new();
    t.set_range(0, TriggerRange { channels: vec![0, 1, 2, 3], low: 0, high: 15, inside: false });
    v.push(("range outside all", t));
    let mut t = conds(&[&[(top, M::Rising)]]);
    t.set_count(0, TriggerCount { count: 10, consecutive: false });
    v.push(("count 10", t));
    let mut t = conds(&[&[(top, M::One)]]);
    t.set_duration(0, TriggerDuration::Longer(100));
    v.push(("duration 100", t));
    let mut t = conds(&[&[(0, M::Rising), (top, M::One)], &[(top, M::Falling)]]);
    t.combine = TriggerCombine::Or;
    v.push(("or", t));
    // A pulse width on a channel that toggles, timed whenever it does.
    let mut t = conds(&[&[(0, M::Falling), (top, M::One)]]);
    t.set_duration(0, TriggerDuration::Between(2, 10));
    v.push(("pulse D0 2..10 + level", t));
    // And on one that never does.
    let mut t = conds(&[&[(top, M::Falling)]]);
    t.set_duration(0, TriggerDuration::Shorter(100));
    v.push(("pulse, idle channel", t));
    v
}

/// Whether libsigrok's matcher runs `t`: levels and edges only.
fn plain(t: &Trigger) -> bool {
    t.combine == TriggerCombine::Sequence
        && t.stages.iter().all(|s| s.count.is_none() && s.duration.is_none() && s.range.is_none())
}

/// A run's rate in MS/s and how many times it fired.
#[derive(Clone, Copy)]
struct Run {
    rate: f64,
    fired: usize,
}

fn run(data: &[u8], unit: usize, chunk: usize, mut feed: impl FnMut(&[u8]) -> bool) -> Run {
    let n = data.len() / unit;
    let mut fired = 0;
    let t0 = Instant::now();
    for c in data.chunks(chunk) {
        fired += usize::from(feed(black_box(c)));
    }
    Run { rate: n as f64 / t0.elapsed().as_secs_f64() / 1e6, fired }
}

struct Options {
    n: usize,
    kinds: Vec<&'static str>,
    units: Vec<usize>,
    min: f64,
}

/// The command line, or `None` after saying what is wrong with it.
fn options() -> Option<Options> {
    let mut n = 64 << 20;
    let mut kinds = vec!["quiet", "counter", "random"];
    let mut units = vec![1, 2];
    let mut min = 0.0;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let Some(v) = args.next() else {
            eprintln!("{a}: a value is missing");
            return None;
        };
        let ok = match a.as_str() {
            "--samples" => v.parse().map(|x| n = x).is_ok(),
            "--data" => ["quiet", "counter", "random"]
                .into_iter()
                .find(|&k| k == v)
                .map(|k| kinds = vec![k])
                .is_some(),
            "--unit" => matches!(v.as_str(), "1" | "2" | "4")
                .then(|| units = vec![v.parse().unwrap()])
                .is_some(),
            "--min" => v.parse().map(|x| min = x).is_ok(),
            _ => {
                eprintln!("unknown option {a}");
                return None;
            }
        };
        if !ok {
            eprintln!("{a}: bad value {v:?}");
            return None;
        }
    }
    Some(Options { n, kinds, units, min })
}

fn main() -> ExitCode {
    let Some(Options { n, kinds, units, min }) = options() else {
        return ExitCode::FAILURE;
    };
    let chunk = transfer_plan(24_000_000).0;
    let mut slow = Vec::new();
    for &unit in &units {
        let top = unit * 8 - 1;
        for &kind in &kinds {
            let data = samples(kind, unit, n);
            println!("\n{unit} byte samples, {kind}, MS/s");
            println!("{:<22} {:>10} {:>10} {:>10}", "trigger", "libsigrok", "general", "+pre");
            for (name, t) in triggers(top) {
                let stages: Vec<Vec<(usize, M)>> = t
                    .stages
                    .iter()
                    .map(|s| s.conditions.iter().map(|c| (c.channel, c.matches)).collect())
                    .collect();
                let logic = plain(&t).then(|| {
                    let mut stl = SoftTriggerLogic::new(stages, unit, 1000);
                    run(&data, unit, chunk, |c| {
                        let hit = stl.check(c).is_some();
                        if hit {
                            // check() does not rearm itself; fx2lafw
                            // resets the stage the same way.
                            stl.cur_stage = 0;
                        }
                        hit
                    })
                });
                let mut st = SoftTrigger::new(&t, unit).expect("stages");
                let soft = run(&data, unit, chunk, |c| {
                    let hit = st.feed(c).is_some();
                    if hit {
                        st = SoftTrigger::new(&t, unit).expect("stages");
                    }
                    hit
                });
                let mut st = SoftTrigger::new(&t, unit).expect("stages");
                let mut pre = PreTriggerBuffer::new(unit, 1 << 20);
                let with_pre = run(&data, unit, chunk, |c| {
                    let hit = st.feed(c).is_some();
                    pre.push(c);
                    if hit {
                        st = SoftTrigger::new(&t, unit).expect("stages");
                    }
                    hit
                });
                let (soft, with_pre) = (Some(soft), Some(with_pre));
                let cell = |r: &Option<Run>| match r {
                    None => "-".to_string(),
                    Some(r) if r.fired > 0 => format!("{:.1} !{}", r.rate, r.fired),
                    Some(r) => format!("{:.1}", r.rate),
                };
                println!(
                    "{name:<22} {:>10} {:>10} {:>10}",
                    cell(&logic),
                    cell(&soft),
                    cell(&with_pre)
                );
                for (col, r) in [("libsigrok", logic), ("general", soft), ("+pre", with_pre)] {
                    if let Some(r) = r
                        && (r.rate < min || r.fired > 0)
                    {
                        slow.push(format!("{unit} byte {kind} {name} {col}"));
                    }
                }
            }
        }
    }
    if slow.is_empty() {
        return ExitCode::SUCCESS;
    }
    eprintln!("\nslower than {min} MS/s, or fired:");
    for s in slow {
        eprintln!("  {s}");
    }
    ExitCode::FAILURE
}
