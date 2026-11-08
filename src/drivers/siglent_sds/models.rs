// Counterpart in sigrok:
//   libsigrok/src/hardware/siglent-sds/api.c,
//   libsigrok/src/hardware/siglent-sds/protocol.h,
//   Copyright (C) 2018 mhooijboer <marchelh@gmail.com>,
//   with commits by Uwe Hermann, marchelh, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The tables of `api.c`: series, models, timebases and the rest.

/// How a series talks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Protocol {
    /// SDS1000X, SDS1000X+, SDS2000X.
    Spo,
    /// SDS1000CML, CNL and DL.
    NonSpo,
    /// SDS1000X-E.
    Eseries,
}

/// A product series.
#[derive(Debug)]
pub struct Series {
    /// Series name.
    pub name: &'static str,
    /// How it talks.
    pub protocol: Protocol,
    /// Slowest timebase.
    pub max_timebase: (u64, u64),
    /// Smallest vertical scale.
    pub min_vdiv: (u64, u64),
    /// Horizontal divisions.
    pub num_hdiv: i32,
}

/// A model.
#[derive(Debug)]
pub struct Model {
    /// Its series.
    pub series: &'static Series,
    /// The name in `*IDN?`.
    pub name: &'static str,
    /// Fastest timebase.
    pub min_timebase: (u64, u64),
    /// Analog channels.
    pub analog_channels: usize,
    /// It has the 16 inputs of a logic analyzer.
    pub has_digital: bool,
}

const fn series(
    name: &'static str,
    protocol: Protocol,
    min_vdiv: (u64, u64),
    num_hdiv: i32,
) -> Series {
    Series { name, protocol, max_timebase: (50, 1), min_vdiv, num_hdiv }
}

static CML: Series = series("SDS1000CML", Protocol::NonSpo, (2, 1000), 18);
static CNL: Series = series("SDS1000CNL", Protocol::NonSpo, (2, 1000), 18);
static DL: Series = series("SDS1000DL", Protocol::NonSpo, (2, 1000), 18);
static X: Series = series("SDS1000X", Protocol::Spo, VDIVS[0], 14);
static XP: Series = series("SDS1000X+", Protocol::Spo, VDIVS[0], 14);
static XE: Series = series("SDS1000XE", Protocol::Eseries, VDIVS[0], 14);
static X2000: Series = series("SDS2000X", Protocol::Spo, VDIVS[0], 14);

const fn model(
    series: &'static Series,
    name: &'static str,
    ns: u64,
    analog_channels: usize,
    has_digital: bool,
) -> Model {
    Model { series, name, min_timebase: (ns, 1_000_000_000), analog_channels, has_digital }
}

/// C's `supported_models`, in its order.
pub static MODELS: [Model; 26] = [
    model(&CML, "SDS1152CML", 20, 2, false),
    model(&CML, "SDS1102CML", 10, 2, false),
    model(&CML, "SDS1072CML", 5, 2, false),
    model(&CNL, "SDS1202CNL", 20, 2, false),
    model(&CNL, "SDS1102CNL", 10, 2, false),
    model(&CNL, "SDS1072CNL", 5, 2, false),
    model(&DL, "SDS1202DL", 20, 2, false),
    model(&DL, "SDS1102DL", 10, 2, false),
    model(&DL, "SDS1022DL", 5, 2, false),
    model(&DL, "SDS1052DL", 5, 2, false),
    model(&DL, "SDS1052DL+", 5, 2, false),
    model(&X, "SDS1102X", 2, 2, false),
    model(&XP, "SDS1102X+", 2, 2, false),
    model(&X, "SDS1202X", 2, 2, false),
    model(&XP, "SDS1202X+", 2, 2, false),
    model(&XE, "SDS1202X-E", 1, 2, false),
    model(&XE, "SDS1104X-E", 1, 4, true),
    model(&XE, "SDS1204X-E", 1, 4, true),
    model(&X2000, "SDS2072X", 2, 2, false),
    model(&X2000, "SDS2074X", 2, 4, false),
    model(&X2000, "SDS2102X", 2, 2, false),
    model(&X2000, "SDS2104X", 2, 4, false),
    model(&X2000, "SDS2202X", 2, 2, false),
    model(&X2000, "SDS2204X", 2, 4, false),
    model(&X2000, "SDS2302X", 2, 2, false),
    model(&X2000, "SDS2304X", 2, 4, false),
];

/// All timebases, as seconds.
#[rustfmt::skip]
pub const TIMEBASES: &[(u64, u64)] = &[
    (1, 1_000_000_000), (2, 1_000_000_000), (5, 1_000_000_000),
    (10, 1_000_000_000), (20, 1_000_000_000), (50, 1_000_000_000),
    (100, 1_000_000_000), (200, 1_000_000_000), (500, 1_000_000_000),
    (1, 1_000_000), (2, 1_000_000), (5, 1_000_000),
    (10, 1_000_000), (20, 1_000_000), (50, 1_000_000),
    (100, 1_000_000), (200, 1_000_000), (500, 1_000_000),
    (1, 1000), (2, 1000), (5, 1000),
    (10, 1000), (20, 1000), (50, 1000),
    (100, 1000), (200, 1000), (500, 1000),
    (1, 1), (2, 1), (5, 1),
    (10, 1), (20, 1), (50, 1),
    (100, 1),
];

/// All vertical scales, as volts. C's first is `{ 500, 100000 }` under the comment
/// "microvolts", and goes out as `500UV`, with 5 mV kept as what it means: a typo
/// for 500 µV, the X-E's smallest, so here it says so.
#[rustfmt::skip]
pub const VDIVS: &[(u64, u64)] = &[
    (500, 1_000_000),
    (1, 1000), (2, 1000), (5, 1000),
    (10, 1000), (20, 1000), (50, 1000),
    (100, 1000), (200, 1000), (500, 1000),
    (1, 1), (2, 1), (5, 1),
    (10, 1), (20, 1), (50, 1),
    (100, 1),
];

/// C's `trigger_sources`. One without digital inputs lists the first five.
pub const TRIGGER_SOURCES: &[&str] = &[
    "CH1", "CH2", "Ext", "Ext /5", "AC Line", "D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8",
    "D9", "D10", "D11", "D12", "D13", "D14", "D15",
];
/// `r` and `f`.
pub const TRIGGER_SLOPES: &[&str] = &["r", "f"];
/// The first three letters are what goes out.
pub const COUPLINGS: &[&str] =
    &["A1M AC 1 Meg", "A50 AC 50 Ohm", "D1M DC 1 Meg", "D50 DC 50 Ohm", "GND"];
/// C's `probe_factor`.
pub const PROBE_FACTORS: &[u64] = &[1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000];
/// C's `averages`.
pub const AVERAGES: &[u64] = &[4, 16, 32, 64, 128, 256, 512, 1024];
/// C's `data_sources`; a non-SPO model lists the first.
pub const DATA_SOURCES: &[&str] = &["Display", "History"];

impl Model {
    /// The model `*IDN?` names, whoever made it: C reads no vendor.
    pub fn find(name: &str) -> Option<&'static Model> {
        MODELS.iter().find(|m| m.name == name)
    }

    /// The timebases this model has: C's loop over the table, from its fastest to
    /// its series' slowest.
    pub fn timebases(&self) -> &'static [(u64, u64)] {
        let from = TIMEBASES.iter().position(|t| *t == self.min_timebase).unwrap_or(0);
        let to = TIMEBASES
            .iter()
            .position(|t| *t == self.series.max_timebase)
            .unwrap_or(TIMEBASES.len() - 1);
        &TIMEBASES[from..=to]
    }

    /// The scales this model has.
    pub fn vdivs(&self) -> &'static [(u64, u64)] {
        let from = VDIVS.iter().position(|v| *v == self.series.min_vdiv).unwrap_or(0);
        &VDIVS[from..]
    }

    /// The trigger sources it lists.
    pub fn trigger_sources(&self) -> &'static [&'static str] {
        if self.has_digital { TRIGGER_SOURCES } else { &TRIGGER_SOURCES[..5] }
    }

    /// The data sources it lists.
    pub fn data_sources(&self) -> &'static [&'static str] {
        if self.series.protocol == Protocol::NonSpo { &DATA_SOURCES[..1] } else { DATA_SOURCES }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_models_lists_go_from_its_own_end_to_its_series() {
        let e = Model::find("SDS1104X-E").unwrap();
        assert_eq!(e.timebases().len(), 33, "1 ns to 50 s");
        assert_eq!(e.timebases()[0], (1, 1_000_000_000));
        assert_eq!(*e.timebases().last().unwrap(), (50, 1));
        assert_eq!(e.vdivs().len(), 17);
        assert_eq!(e.trigger_sources().len(), 21);
        let dl = Model::find("SDS1202DL").unwrap();
        assert_eq!(dl.timebases().len(), 29, "20 ns to 50 s");
        assert_eq!(dl.vdivs().len(), 15, "from 2 mV");
        assert_eq!(dl.vdivs()[0], (2, 1000));
        assert_eq!(dl.trigger_sources().len(), 5);
        assert_eq!(dl.data_sources(), ["Display"]);
        assert_eq!(Model::find("SDS1202X").unwrap().data_sources(), ["Display", "History"]);
    }

    #[test]
    fn only_a_name_of_the_table_is_a_model() {
        assert!(Model::find("SDS1052DL+").is_some());
        assert!(Model::find("SDS1052").is_none());
        assert!(Model::find("sds1052dl").is_none());
    }

    #[test]
    fn each_series_has_its_protocol_and_divisions() {
        for m in &MODELS {
            let (protocol, hdiv) = match m.series.name {
                "SDS1000CML" | "SDS1000CNL" | "SDS1000DL" => (Protocol::NonSpo, 18),
                "SDS1000XE" => (Protocol::Eseries, 14),
                _ => (Protocol::Spo, 14),
            };
            assert_eq!(m.series.protocol, protocol, "{}", m.name);
            assert_eq!(m.series.num_hdiv, hdiv, "{}", m.name);
        }
        let digital: Vec<&str> = MODELS.iter().filter(|m| m.has_digital).map(|m| m.name).collect();
        assert_eq!(digital, ["SDS1104X-E", "SDS1204X-E"]);
    }
}
