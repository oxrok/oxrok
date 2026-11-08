// Counterpart in sigrok:
//   libsigrok/src/hardware/hameg-hmo/protocol.c,
//   Copyright (C) 2013 poljar (Damir Jelić) <poljarinho@gmail.com>,
//   Copyright (C) 2018 Guido Trentalancia <guido@trentalancia.com>,
//   with commits by Uwe Hermann, Gerhard Sittig, Stefan Brüns, Soeren Apel,
//   Bert Vermeulen, Martin Ling, Mathias Katzer, Daniel Elstner,
//   Johannes Römer, Valentin Ochs, xandinator
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The scopes `hameg-hmo` knows, and the commands of its two dialects.

/// Which commands a scope takes for its logic inputs. The rest is common.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dialect {
    /// `hameg_scpi_dialect`: `:PODn:...`.
    Pod,
    /// `rohde_schwarz_log_not_pod_scpi_dialect`: `:LOGn:...` and `:DIGn:...`. C's, and
    /// unchecked: the RTB2000 manual shows a channel's state as `DIGital<m>:DISPlay`, where
    /// this sends `:LOGn:STAT` for it (see the `hameg-hmo` model's notes).
    Log,
}

impl Dialect {
    /// `SCPI_CMD_GET_DIG_DATA`.
    pub fn dig_data(self, pod: usize) -> String {
        match self {
            Dialect::Pod => format!(":FORM UINT,8;:POD{pod}:DATA?"),
            Dialect::Log => format!(":FORM UINT,8;:LOG{pod}:DATA?"),
        }
    }

    /// `SCPI_CMD_GET_DIG_POD_STATE`.
    pub fn get_pod_state(self, pod: usize) -> String {
        match self {
            Dialect::Pod => format!(":POD{pod}:STAT?"),
            Dialect::Log => format!(":LOG{pod}:STAT?"),
        }
    }

    /// `SCPI_CMD_SET_DIG_POD_STATE`.
    pub fn set_pod_state(self, pod: usize, on: bool) -> String {
        match self {
            Dialect::Pod => format!(":POD{pod}:STAT {}", u8::from(on)),
            Dialect::Log => format!(":LOG{pod}:STAT {}", u8::from(on)),
        }
    }

    /// `SCPI_CMD_GET_DIG_POD_THRESHOLD`.
    pub fn get_threshold(self, number: usize) -> String {
        match self {
            Dialect::Pod => format!(":POD{number}:THR?"),
            Dialect::Log => format!(":DIG{number}:TECH?"),
        }
    }

    /// `SCPI_CMD_SET_DIG_POD_THRESHOLD`.
    pub fn set_threshold(self, number: usize, name: &str) -> String {
        match self {
            Dialect::Pod => format!(":POD{number}:THR {name}"),
            Dialect::Log => format!(":DIG{number}:TECH {name}"),
        }
    }

    /// `SCPI_CMD_GET_DIG_POD_USER_THRESHOLD`; `user` is the 1 or 2 of `USER1` and
    /// `USER2`, which the second dialect's command doesn't name.
    pub fn get_user_threshold(self, number: usize, user: usize) -> String {
        match self {
            Dialect::Pod => format!(":POD{number}:THR:UDL{user}?"),
            Dialect::Log => format!(":DIG{number}:THR?"),
        }
    }

    /// `SCPI_CMD_SET_DIG_POD_USER_THRESHOLD`.
    pub fn set_user_threshold(self, number: usize, user: usize, value: &str) -> String {
        match self {
            Dialect::Pod => format!(":POD{number}:THR:UDL{user} {value}"),
            Dialect::Log => format!(":DIG{number}:THR {value}"),
        }
    }
}

/// A series.
#[derive(Debug)]
pub struct Model {
    /// The names `*IDN?` gives.
    pub names: &'static [&'static str],
    /// Analog channels.
    pub analog: usize,
    /// Logic channels, 8 to a pod.
    pub digital: usize,
    /// The commands for its logic inputs.
    pub dialect: Dialect,
    /// `coupling` values.
    pub coupling: &'static [&'static str],
    /// `logic_threshold` values.
    pub thresholds: &'static [&'static str],
    /// A threshold's command has the pod's number from 1, not its first channel's.
    pub threshold_for_pod: bool,
    /// `triggersource` values.
    pub trigger_sources: &'static [&'static str],
    /// `timebase` values.
    pub timebases: &'static [(u64, u64)],
    /// `vdiv` values.
    pub vdivs: &'static [(u64, u64)],
}

impl Model {
    /// Logic pods.
    pub fn pods(&self) -> usize {
        self.digital / DIGITAL_CHANNELS_PER_POD
    }

    /// What a threshold's command calls the pod of this index.
    pub fn threshold_number(&self, pod: usize) -> usize {
        if self.threshold_for_pod { pod + 1 } else { pod * DIGITAL_CHANNELS_PER_POD }
    }
}

/// Logic channels per pod.
pub const DIGITAL_CHANNELS_PER_POD: usize = 8;
/// `MAX_ANALOG_CHANNEL_COUNT + MAX_DIGITAL_CHANNEL_COUNT`: the longest trigger pattern
/// the state holds.
pub const PATTERN_ROOM: usize = 20;
/// `num_ydivs`, the same for all.
pub const NUM_YDIVS: i32 = 8;
/// `trigger_slopes`.
pub const TRIGGER_SLOPES: &[&str] = &["POS", "NEG", "EITH"];
/// Threshold names, by what they do.
pub const USER1: &str = "USER1";
/// See [`USER1`].
pub const USER2: &str = "USER2";
/// See [`USER1`].
pub const MAN: &str = "MAN";

const COUPLING: &[&str] = &["AC", "ACL", "DC", "DCL", "GND"];
const COUPLING_RTB200X: &[&str] = &["ACL", "DCL", "GND"];
const COUPLING_RTM300X: &[&str] = &["ACL", "DC", "DCL", "GND"];
const COUPLING_RTH100X: &[&str] = &["ACL", "DCL"];

const THRESHOLD: &[&str] = &["TTL", "ECL", "CMOS", USER1, USER2];
const THRESHOLD_RTB200X_RTM300X: &[&str] = &["TTL", "ECL", "CMOS", MAN];

const AN2_DIG8: &[&str] = &[
    "CH1", "CH2", "LINE", "EXT", "PATT", "BUS1", "BUS2", "D0", "D1", "D2", "D3", "D4", "D5", "D6",
    "D7",
];
const AN2_DIG16: &[&str] = &[
    "CH1", "CH2", "LINE", "EXT", "PATT", "BUS1", "BUS2", "D0", "D1", "D2", "D3", "D4", "D5", "D6",
    "D7", "D8", "D9", "D10", "D11", "D12", "D13", "D14", "D15",
];
const AN2_DIG16_SBUS: &[&str] = &[
    "CH1", "CH2", "LINE", "EXT", "PATT", "SBUS1", "SBUS2", "D0", "D1", "D2", "D3", "D4", "D5",
    "D6", "D7", "D8", "D9", "D10", "D11", "D12", "D13", "D14", "D15",
];
const AN2_DIG8_ISOL: &[&str] = &["CH1", "CH2", "D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7"];
const AN4_DIG8_ISOL: &[&str] =
    &["CH1", "CH2", "CH3", "CH4", "D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7"];
const AN4_DIG8: &[&str] = &[
    "CH1", "CH2", "CH3", "CH4", "LINE", "EXT", "PATT", "BUS1", "BUS2", "D0", "D1", "D2", "D3",
    "D4", "D5", "D6", "D7",
];
const AN4_DIG16: &[&str] = &[
    "CH1", "CH2", "CH3", "CH4", "LINE", "EXT", "PATT", "BUS1", "BUS2", "D0", "D1", "D2", "D3",
    "D4", "D5", "D6", "D7", "D8", "D9", "D10", "D11", "D12", "D13", "D14", "D15",
];
const AN4_DIG16_SBUS: &[&str] = &[
    "CH1", "CH2", "CH3", "CH4", "LINE", "EXT", "PATT", "SBUS1", "SBUS2", "D0", "D1", "D2", "D3",
    "D4", "D5", "D6", "D7", "D8", "D9", "D10", "D11", "D12", "D13", "D14", "D15",
];

const NS: u64 = 1_000_000_000;
const US: u64 = 1_000_000;
const MS: u64 = 1_000;

/// `timebases`.
const TIMEBASES: &[(u64, u64)] = &[
    (1, NS),
    (2, NS),
    (5, NS),
    (10, NS),
    (20, NS),
    (50, NS),
    (100, NS),
    (200, NS),
    (500, NS),
    (1, US),
    (2, US),
    (5, US),
    (10, US),
    (20, US),
    (50, US),
    (100, US),
    (200, US),
    (500, US),
    (1, MS),
    (2, MS),
    (5, MS),
    (10, MS),
    (20, MS),
    (50, MS),
    (100, MS),
    (200, MS),
    (500, MS),
    (1, 1),
    (2, 1),
    (5, 1),
    (10, 1),
    (20, 1),
    (50, 1),
];
/// `timebases_hmo_compact`: no 1 ns.
const TIMEBASES_COMPACT: &[(u64, u64)] = TIMEBASES.split_at(1).1;

const VDIVS_UPTO_50V: &[(u64, u64)] = &[
    (1, MS),
    (2, MS),
    (5, MS),
    (10, MS),
    (20, MS),
    (50, MS),
    (100, MS),
    (200, MS),
    (500, MS),
    (1, 1),
    (2, 1),
    (5, 1),
    (10, 1),
    (20, 1),
    (50, 1),
];
const VDIVS_UPTO_10V: &[(u64, u64)] = VDIVS_UPTO_50V.split_at(13).0;

/// `scope_models`.
#[rustfmt::skip]
pub const MODELS: &[Model] = &[
    Model { names: &["HMO722", "HMO1022", "HMO1522", "HMO2022"], analog: 2, digital: 8, dialect: Dialect::Pod, coupling: COUPLING, thresholds: THRESHOLD, threshold_for_pod: true, trigger_sources: AN2_DIG8, timebases: TIMEBASES_COMPACT, vdivs: VDIVS_UPTO_10V },
    Model { names: &["RTC1002", "HMO1002", "HMO1202"], analog: 2, digital: 8, dialect: Dialect::Pod, coupling: COUPLING, thresholds: THRESHOLD, threshold_for_pod: true, trigger_sources: AN2_DIG8, timebases: TIMEBASES, vdivs: VDIVS_UPTO_10V },
    Model { names: &["HMO3032", "HMO3042", "HMO3052", "HMO3522"], analog: 2, digital: 16, dialect: Dialect::Pod, coupling: COUPLING, thresholds: THRESHOLD, threshold_for_pod: true, trigger_sources: AN2_DIG16, timebases: TIMEBASES, vdivs: VDIVS_UPTO_10V },
    Model { names: &["HMO724", "HMO1024", "HMO1524", "HMO2024"], analog: 4, digital: 8, dialect: Dialect::Pod, coupling: COUPLING, thresholds: THRESHOLD, threshold_for_pod: true, trigger_sources: AN4_DIG8, timebases: TIMEBASES_COMPACT, vdivs: VDIVS_UPTO_10V },
    Model { names: &["HMO2524", "HMO3034", "HMO3044", "HMO3054", "HMO3524"], analog: 4, digital: 16, dialect: Dialect::Pod, coupling: COUPLING, thresholds: THRESHOLD, threshold_for_pod: true, trigger_sources: AN4_DIG16, timebases: TIMEBASES, vdivs: VDIVS_UPTO_10V },
    Model { names: &["RTB2002"], analog: 2, digital: 16, dialect: Dialect::Log, coupling: COUPLING_RTB200X, thresholds: THRESHOLD_RTB200X_RTM300X, threshold_for_pod: false, trigger_sources: AN2_DIG16_SBUS, timebases: TIMEBASES, vdivs: VDIVS_UPTO_50V },
    Model { names: &["RTB2004"], analog: 4, digital: 16, dialect: Dialect::Log, coupling: COUPLING_RTB200X, thresholds: THRESHOLD_RTB200X_RTM300X, threshold_for_pod: false, trigger_sources: AN4_DIG16_SBUS, timebases: TIMEBASES, vdivs: VDIVS_UPTO_50V },
    Model { names: &["RTM3002"], analog: 2, digital: 16, dialect: Dialect::Log, coupling: COUPLING_RTM300X, thresholds: THRESHOLD_RTB200X_RTM300X, threshold_for_pod: false, trigger_sources: AN2_DIG16_SBUS, timebases: TIMEBASES, vdivs: VDIVS_UPTO_10V },
    Model { names: &["RTM3004"], analog: 4, digital: 16, dialect: Dialect::Log, coupling: COUPLING_RTM300X, thresholds: THRESHOLD_RTB200X_RTM300X, threshold_for_pod: false, trigger_sources: AN4_DIG16_SBUS, timebases: TIMEBASES, vdivs: VDIVS_UPTO_10V },
    Model { names: &["RTA4004"], analog: 4, digital: 16, dialect: Dialect::Log, coupling: COUPLING_RTM300X, thresholds: THRESHOLD_RTB200X_RTM300X, threshold_for_pod: false, trigger_sources: AN4_DIG16_SBUS, timebases: TIMEBASES, vdivs: VDIVS_UPTO_10V },
    Model { names: &["RTH1002"], analog: 2, digital: 8, dialect: Dialect::Log, coupling: COUPLING_RTH100X, thresholds: THRESHOLD, threshold_for_pod: true, trigger_sources: AN2_DIG8_ISOL, timebases: TIMEBASES, vdivs: VDIVS_UPTO_50V },
    Model { names: &["RTH1004"], analog: 4, digital: 8, dialect: Dialect::Log, coupling: COUPLING_RTH100X, thresholds: THRESHOLD, threshold_for_pod: true, trigger_sources: AN4_DIG8_ISOL, timebases: TIMEBASES, vdivs: VDIVS_UPTO_50V },
];

/// The series of a name, exact as `*IDN?` gives it.
pub fn find(name: &str) -> Option<&'static Model> {
    MODELS.iter().find(|m| m.names.contains(&name))
}

/// `manufacturers`.
pub const MANUFACTURERS: &[&str] = &["HAMEG", "Rohde&Schwarz"];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tables_have_c_counts() {
        assert_eq!(TIMEBASES.len(), 33);
        assert_eq!(TIMEBASES_COMPACT.len(), 32);
        assert_eq!(TIMEBASES_COMPACT[0], (2, NS));
        assert_eq!(VDIVS_UPTO_50V.len(), 15);
        assert_eq!(VDIVS_UPTO_10V.len(), 13);
        assert_eq!(VDIVS_UPTO_10V.last(), Some(&(10, 1)));
        assert_eq!(MODELS.len(), 12);
    }

    #[test]
    fn a_threshold_has_the_number_its_series_uses() {
        let hmo = find("HMO3054").unwrap();
        assert_eq!((hmo.threshold_number(0), hmo.threshold_number(1)), (1, 2));
        let rtb = find("RTB2004").unwrap();
        assert_eq!((rtb.threshold_number(0), rtb.threshold_number(1)), (0, 8));
        assert_eq!(find("RTH1002").unwrap().threshold_number(0), 1);
    }

    #[test]
    fn names_are_exact() {
        assert!(find("HMO1024").is_some());
        assert!(find("hmo1024").is_none());
        assert!(find("HMO10245").is_none());
        assert_eq!(find("RTH1004").unwrap().trigger_sources.len(), 12);
    }

    #[test]
    fn the_dialects_spell_their_commands() {
        assert_eq!(Dialect::Pod.dig_data(2), ":FORM UINT,8;:POD2:DATA?");
        assert_eq!(Dialect::Log.dig_data(2), ":FORM UINT,8;:LOG2:DATA?");
        assert_eq!(Dialect::Pod.set_pod_state(1, true), ":POD1:STAT 1");
        assert_eq!(Dialect::Log.get_pod_state(2), ":LOG2:STAT?");
        assert_eq!(Dialect::Pod.get_threshold(2), ":POD2:THR?");
        assert_eq!(Dialect::Log.set_threshold(8, "MAN"), ":DIG8:TECH MAN");
        assert_eq!(Dialect::Pod.get_user_threshold(1, 2), ":POD1:THR:UDL2?");
        assert_eq!(Dialect::Log.get_user_threshold(8, 1), ":DIG8:THR?");
        assert_eq!(Dialect::Pod.set_user_threshold(1, 2, "1"), ":POD1:THR:UDL2 1");
        assert_eq!(Dialect::Log.set_user_threshold(1, 2, "1"), ":DIG1:THR 1");
    }
}
