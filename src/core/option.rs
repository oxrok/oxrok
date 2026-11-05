// Counterpart in sigrok:
//   libsigrok/src/hwdriver.c,
//   libsigrok/src/std.c (the option list helpers).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Device option descriptors: which keys a device has and how they can
//! be used.

use crate::core::key::Key;
use crate::core::trigger::TriggerMatch;

/// `std_gvar_min_max_step_thresholds()`: point ranges from `min` to `max`.
/// The steps accumulate as in the C code, so that a value near 1.5 is
/// `1.4999999999999958`, a device's current 1.5 equals no entry and
/// `--show` marks none current, as sigrok-cli does. Only values next to
/// zero are made exactly 0.
pub fn min_max_step_thresholds(min: f64, max: f64, step: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    let mut d = min;
    while d <= max + step / 2.0 {
        let v = if d > -step / 2.0 && d < step / 2.0 { 0.0 } else { d };
        out.push((v, v));
        d += step;
    }
    out
}

/// What a device allows for a key.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Access {
    /// The current value can be read.
    pub get: bool,
    /// The value can be changed.
    pub set: bool,
    /// The allowed values can be listed.
    pub list: bool,
}

impl Access {
    /// Readable and writable.
    pub const GET_SET: Access = Access { get: true, set: true, list: false };
    /// Readable, writable and with a list of allowed values.
    pub const GET_SET_LIST: Access = Access { get: true, set: true, list: true };
    /// Read only.
    pub const GET: Access = Access { get: true, set: false, list: false };
    /// Only a list of allowed values, e.g. the trigger matches.
    pub const LIST: Access = Access { get: false, set: false, list: true };
    /// A capability flag with no value, e.g. continuous sampling.
    pub const NONE: Access = Access { get: false, set: false, list: false };
}

/// One option a device offers.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OptionDesc {
    /// The key.
    pub key: Key,
    /// How it can be used.
    pub access: Access,
}

impl OptionDesc {
    /// A new descriptor.
    pub const fn new(key: Key, access: Access) -> Self {
        OptionDesc { key, access }
    }
}

/// The allowed values of a key, as `list()` reports them.
#[derive(Clone, Debug, PartialEq)]
pub enum ValueList {
    /// A list of unsigned integers, e.g. supported sample rates.
    U64(Vec<u64>),
    /// A continuous range of unsigned integers with a step, e.g. a
    /// sample rate range.
    RangeStepU64 {
        /// Lowest value.
        min: u64,
        /// Highest value.
        max: u64,
        /// Step between values.
        step: u64,
    },
    /// Minimum and maximum of an unsigned integer, e.g. the sample limit.
    MinMaxU64(u64, u64),
    /// A list of floats, e.g. voltage thresholds.
    F64(Vec<f64>),
    /// A continuous range of floats with a step. Values are
    /// `min + i * step`, computed exactly, never accumulated.
    RangeStepF64 {
        /// Lowest value.
        min: f64,
        /// Highest value.
        max: f64,
        /// Step between values.
        step: f64,
    },
    /// A list of strings, e.g. pattern names.
    Str(Vec<String>),
    /// A list of rationals, e.g. timebases or vertical scales.
    Rational(Vec<(u64, u64)>),
    /// The trigger match types a device supports.
    TriggerMatches(Vec<TriggerMatch>),
    /// A list of float pairs, e.g. the voltage thresholds of a device
    /// with fixed logic families (`std_gvar_thresholds()`).
    RangeF64(Vec<(f64, f64)>),
}

impl ValueList {
    /// The float values of a `RangeStepF64`, each computed as
    /// `min + i * step`, or the values of an `F64` list.
    pub fn f64_values(&self) -> Option<Vec<f64>> {
        match self {
            ValueList::F64(v) => Some(v.clone()),
            ValueList::RangeStepF64 { min, max, step } => {
                if *step <= 0.0 || max < min {
                    return Some(vec![*min]);
                }
                let n = ((max - min) / step).round() as u64;
                Some((0..=n).map(|i| min + i as f64 * step).collect())
            }
            _ => None,
        }
    }
}

/// Resolves a key name against the device's options of `group`, so a
/// driver's custom keys work like the well known ones.
pub fn resolve_key(
    device: &dyn crate::core::device::Device,
    name: &str,
    group: Option<&str>,
) -> crate::core::error::Result<Key> {
    if let Ok(options) = device.options(group)
        && let Some(o) = options.iter().find(|o| o.key.id() == name)
    {
        return Ok(o.key.clone());
    }
    Key::from_id(name)
        .ok_or_else(|| crate::core::error::Error::Arg(format!("Unknown device option '{name}'.")))
}

/// The log target of what hwdriver.c logs: `oxrok::core::hwdriver`
/// (tools/diffcli.sh prints it as libsigrok's `sr: hwdriver: `).
const HWDRIVER: &str = "oxrok::core::hwdriver";

/// The value checks of hwdriver.c's `check_key()`, which need no device, for
/// a front end that validates a command line before it sends any of it.
pub fn check_value(key: &Key, value: &crate::core::value::Value) -> crate::core::error::Result<()> {
    use crate::core::error::Error;
    match key {
        // Setting any of these to 0 is not useful.
        Key::LimitMsec | Key::LimitSamples | Key::Samplerate if value.as_u64() == Some(0) => {
            log::error!(target: HWDRIVER, "Cannot set '{}' to 0.", key.id());
            Err(Error::arg("it cannot be 0"))
        }
        Key::CaptureRatio if value.as_u64().is_some_and(|r| r > 100) => {
            log::error!(target: HWDRIVER, "Capture ratio must be 0..100.");
            Err(Error::arg("it must be 0..100"))
        }
        _ => Ok(()),
    }
}

/// What hwdriver.c's `check_key()` asks of a set, for every driver, with
/// the messages libsigrok has.
fn check_set(
    device: &dyn crate::core::device::Device,
    key: &Key,
    value: &crate::core::value::Value,
    group: Option<&str>,
) -> crate::core::error::Result<()> {
    use crate::core::error::Error;
    const LOG: &str = HWDRIVER;
    check_value(key, value)?;
    let suffix = if group.is_some() {
        " for this device instance and channel group"
    } else {
        " for this device instance"
    };
    let Ok(options) = device.options(group) else {
        log::error!(target: LOG, "No options available{suffix}.");
        return Err(Error::Arg(format!("no options available{suffix}")));
    };
    let Some(option) = options.iter().find(|o| &o.key == key) else {
        log::error!(target: LOG, "Option '{}' not available{suffix}.", key.id());
        return Err(Error::Arg(format!("not available{suffix}")));
    };
    if !option.access.set {
        log::error!(target: LOG, "Option '{}' not available to set{suffix}.", key.id());
        return Err(Error::Arg(format!("not available to set{suffix}")));
    }
    Ok(())
}

/// `sr_config_set()`: sets an option on a device, after the checks the core
/// makes for every driver. A front end sets options through this;
/// [`Device::set`](crate::core::device::Device::set) is the driver's part.
pub fn config_set(
    device: &mut dyn crate::core::device::Device,
    key: &Key,
    value: &crate::core::value::Value,
    group: Option<&str>,
) -> crate::core::error::Result<()> {
    check_set(&*device, key, value, group)?;
    device.set(key, value, group)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[allow(clippy::float_cmp)]
    fn range_steps_are_exact() {
        let l = ValueList::RangeStepF64 { min: -6.0, max: 6.0, step: 0.05 };
        let v = l.f64_values().unwrap();
        assert_eq!(v.len(), 241);
        // The value for 1.5 V is reachable by equality, unlike with an
        // accumulated sum.
        assert!(v.contains(&1.5));
        assert_eq!(v[0], -6.0);
        assert_eq!(v[240], 6.0);
    }
}
