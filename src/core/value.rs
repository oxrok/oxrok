// Counterpart in sigrok:
//   sigrok-cli/parsers.c,
//   libsigrok/src/strutil.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! Configuration values and their kinds.

use crate::core::analog::{Mq, MqFlags};
use crate::core::error::{Error, Result};
use crate::core::util;
use std::fmt;

/// The type of a configuration value.
///
/// Each [`Key`](crate::core::Key) has one kind. The kinds mirror the sigrok
/// value types, so a key keeps the meaning it has in sigrok-cli and in
/// `.sr` metadata.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ValueKind {
    /// On or off.
    Bool,
    /// An unsigned integer, e.g. a sample rate in Hz.
    U64,
    /// A signed integer, e.g. a channel count.
    I32,
    /// A floating point number, e.g. a voltage.
    F64,
    /// A string, often one of a list of choices.
    Str,
    /// A time as a rational number of seconds (p/q).
    RationalPeriod,
    /// A voltage as a rational number of volts (p/q).
    RationalVolt,
    /// Two unsigned integers, low and high.
    RangeU64,
    /// Two floating point numbers, low and high.
    RangeF64,
    /// A key and a value, both strings.
    KeyValue,
    /// A measured quantity with its flags.
    Mq,
}

/// A configuration value.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    /// On or off.
    Bool(bool),
    /// An unsigned integer.
    U64(u64),
    /// A signed integer.
    I32(i32),
    /// A floating point number.
    F64(f64),
    /// A string.
    Str(String),
    /// A rational number p/q.
    Rational(u64, u64),
    /// Two unsigned integers, low and high.
    RangeU64(u64, u64),
    /// Two floating point numbers, low and high.
    RangeF64(f64, f64),
    /// A key and a value.
    KeyValue(String, String),
    /// A measured quantity with its flags.
    Mq(Mq, MqFlags),
}

impl Value {
    /// The kind of this value.
    pub fn kind(&self) -> ValueKind {
        match self {
            Value::Bool(_) => ValueKind::Bool,
            Value::U64(_) => ValueKind::U64,
            Value::I32(_) => ValueKind::I32,
            Value::F64(_) => ValueKind::F64,
            Value::Str(_) => ValueKind::Str,
            // A rational is a period unless the key says otherwise.
            Value::Rational(..) => ValueKind::RationalPeriod,
            Value::RangeU64(..) => ValueKind::RangeU64,
            Value::RangeF64(..) => ValueKind::RangeF64,
            Value::KeyValue(..) => ValueKind::KeyValue,
            Value::Mq(..) => ValueKind::Mq,
        }
    }

    /// The value as an unsigned integer, if it is one.
    pub fn as_u64(&self) -> Option<u64> {
        match self {
            Value::U64(v) => Some(*v),
            Value::I32(v) if *v >= 0 => Some(*v as u64),
            _ => None,
        }
    }

    /// The value as a signed integer, if it is one.
    pub fn as_i32(&self) -> Option<i32> {
        match self {
            Value::I32(v) => Some(*v),
            Value::U64(v) => i32::try_from(*v).ok(),
            _ => None,
        }
    }

    /// The value as a float, if it is numeric.
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::F64(v) => Some(*v),
            Value::U64(v) => Some(*v as f64),
            Value::I32(v) => Some(f64::from(*v)),
            Value::Rational(p, q) if *q != 0 => Some(*p as f64 / *q as f64),
            _ => None,
        }
    }

    /// The value as a bool, if it is one.
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Value::Bool(v) => Some(*v),
            _ => None,
        }
    }

    /// The value as a string, if it is one.
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::Str(v) => Some(v),
            _ => None,
        }
    }

    /// Parses `text` as a value of `kind`, with sigrok-cli's rules:
    /// integers take size suffixes (`1m` is one million, `200k` two
    /// hundred thousand), ranges are `low-high`, key values are
    /// `key=value`, measured quantities are `mq/flag/flag`. A boolean is
    /// [`util::parse_bool`]'s, where sigrok-cli has no refusal.
    pub fn parse(kind: ValueKind, text: &str) -> Result<Value> {
        let bad = || Error::Arg(format!("invalid value '{text}'"));
        Ok(match kind {
            ValueKind::Bool => Value::Bool(util::parse_bool(text).ok_or_else(bad)?),
            ValueKind::U64 => Value::U64(util::parse_sizestring(text)?),
            ValueKind::I32 => {
                Value::I32(i32::try_from(util::parse_sizestring(text)?).map_err(|_| bad())?)
            }
            ValueKind::F64 => Value::F64(text.trim().parse().map_err(|_| bad())?),
            ValueKind::Str => Value::Str(text.to_owned()),
            ValueKind::RationalPeriod => {
                let (p, q) = util::parse_period(text).ok_or_else(bad)?;
                Value::Rational(p, q)
            }
            ValueKind::RationalVolt => {
                let (p, q) = util::parse_voltage(text).ok_or_else(bad)?;
                Value::Rational(p, q)
            }
            ValueKind::RangeU64 => {
                let (lo, hi) = text.split_once('-').ok_or_else(bad)?;
                Value::RangeU64(
                    lo.trim().parse().map_err(|_| bad())?,
                    hi.trim().parse().map_err(|_| bad())?,
                )
            }
            ValueKind::RangeF64 => {
                // A single number means a point range, which sigrok-cli
                // cannot express but every driver understands.
                if let Ok(v) = text.trim().parse::<f64>() {
                    return Ok(Value::RangeF64(v, v));
                }
                let (lo, hi) = split_signed_range(text).ok_or_else(bad)?;
                Value::RangeF64(lo, hi)
            }
            ValueKind::KeyValue => {
                let (k, v) = text.split_once('=').ok_or_else(bad)?;
                Value::KeyValue(k.to_owned(), v.to_owned())
            }
            ValueKind::Mq => {
                let mut parts = text.split('/');
                let mq = parts.next().and_then(Mq::from_id).ok_or_else(bad)?;
                let mut flags = MqFlags::empty();
                for f in parts {
                    flags |= MqFlags::from_id(f).ok_or_else(bad)?;
                }
                Value::Mq(mq, flags)
            }
        })
    }
}

/// Splits `a-b` where either number may carry a sign, e.g. `-1.5--0.5`.
fn split_signed_range(text: &str) -> Option<(f64, f64)> {
    let t = text.trim();
    // Try every '-' that is not the leading sign as the separator.
    for (i, _) in t.char_indices().filter(|&(i, c)| c == '-' && i > 0) {
        let (lo, hi) = (&t[..i], &t[i + 1..]);
        if let (Ok(lo), Ok(hi)) = (lo.trim().parse::<f64>(), hi.trim().parse::<f64>()) {
            return Some((lo, hi));
        }
    }
    None
}

impl fmt::Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Value::Bool(v) => f.write_str(if *v { "true" } else { "false" }),
            Value::U64(v) => write!(f, "{v}"),
            Value::I32(v) => write!(f, "{v}"),
            Value::F64(v) => write!(f, "{v}"),
            Value::Str(v) => f.write_str(v),
            Value::Rational(p, q) => write!(f, "{p}/{q}"),
            Value::RangeU64(a, b) => write!(f, "{a}-{b}"),
            Value::RangeF64(a, b) => write!(f, "{a}-{b}"),
            Value::KeyValue(k, v) => write!(f, "{k}={v}"),
            Value::Mq(mq, flags) => {
                f.write_str(mq.id())?;
                for flag in flags.iter() {
                    write!(f, "/{}", flag.id())?;
                }
                Ok(())
            }
        }
    }
}

impl From<bool> for Value {
    fn from(v: bool) -> Self {
        Value::Bool(v)
    }
}

impl From<u64> for Value {
    fn from(v: u64) -> Self {
        Value::U64(v)
    }
}

impl From<i32> for Value {
    fn from(v: i32) -> Self {
        Value::I32(v)
    }
}

impl From<f64> for Value {
    fn from(v: f64) -> Self {
        Value::F64(v)
    }
}

impl From<&str> for Value {
    fn from(v: &str) -> Self {
        Value::Str(v.to_owned())
    }
}

impl From<String> for Value {
    fn from(v: String) -> Self {
        Value::Str(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_sizes_and_ranges() {
        assert_eq!(Value::parse(ValueKind::U64, "200k").unwrap(), Value::U64(200_000));
        assert_eq!(Value::parse(ValueKind::I32, "4").unwrap(), Value::I32(4));
        assert_eq!(Value::parse(ValueKind::Bool, "yes").unwrap(), Value::Bool(true));
        assert_eq!(Value::parse(ValueKind::Bool, "OFF").unwrap(), Value::Bool(false));
        assert!(Value::parse(ValueKind::Bool, "").is_err());
        assert!(Value::parse(ValueKind::Bool, "1x").is_err());
        assert_eq!(Value::parse(ValueKind::RangeU64, "10-20").unwrap(), Value::RangeU64(10, 20));
        assert_eq!(
            Value::parse(ValueKind::RangeF64, "1.5-1.5").unwrap(),
            Value::RangeF64(1.5, 1.5)
        );
        assert_eq!(Value::parse(ValueKind::RangeF64, "1.6").unwrap(), Value::RangeF64(1.6, 1.6));
        assert_eq!(
            Value::parse(ValueKind::RangeF64, "-1.5--0.5").unwrap(),
            Value::RangeF64(-1.5, -0.5)
        );
        assert!(Value::parse(ValueKind::U64, "lots").is_err());
    }

    #[test]
    fn parses_measured_quantities() {
        let v = Value::parse(ValueKind::Mq, "voltage/dc").unwrap();
        assert_eq!(v, Value::Mq(Mq::Voltage, MqFlags::DC));
        assert_eq!(v.to_string(), "voltage/dc");
    }
}
