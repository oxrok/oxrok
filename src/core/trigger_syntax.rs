// Counterpart in sigrok:
//   sigrok-cli/parsers.c (parse_triggerstring),
//   Copyright (C) 2011 Bert Vermeulen <bert@biot.com>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, extended with counts, durations, value ranges and combine modes.

//! The `-t` trigger grammar.
//!
//! sigrok-cli's forms parse unchanged: `D0=r,D1=1` is one stage with two
//! conditions, `;` separates stages, and several letters on one channel
//! (`D0=rf`) put each letter in the next stage. On top of that:
//!
//! ```text
//! D0=r*5        the fifth rising edge
//! D0=r*5c       the fifth in a row (consecutive)
//! D0=1>100      D0 high for more than 100 samples
//! D0=1>50us     the same with a time, converted with the sample rate
//! D0=f<50us     a high pulse on D0 shorter than 50 us, at its falling edge
//! D0=r<100      a low pulse shorter than 100 samples, at its rising edge
//! D0=f>10us<50us  a high pulse longer than 10 us and shorter than 50 us
//! D0=f>10us     a high pulse longer than 10 us, at its end
//! D0=f<50us,D1=1  a short high pulse, with D1 high when it ends
//! D0-D2=3..5    the value on D2 D1 D0 (D0 least significant) within 3..5
//! D0-D2=!3..5   outside 3..5; a single number means that value
//! D0=r|D1=r     either stage; D0=r&D1=r both stages in any order
//! ```
//!
//! Counts, durations and ranges belong to the stage their token is in.
//! Both ends of a width are strict, and a time is rounded so that no
//! width that qualifies is left out: down after `>`, up after `<`.

use crate::core::channel::ChannelKind;
use crate::core::device::Device;
use crate::core::error::{Error, Result};
use crate::core::key::Key;
use crate::core::option::ValueList;
use crate::core::trigger::{
    Trigger, TriggerCombine, TriggerCount, TriggerDuration, TriggerMatch, TriggerRange,
};
use crate::core::util::parse_period;

/// Parses `spec` for `device`, checking channels, matches and features.
pub fn parse(device: &dyn Device, spec: &str) -> Result<Trigger> {
    let Ok(ValueList::TriggerMatches(supported)) = device.list(&Key::TriggerMatch, None) else {
        return Err(Error::arg("Device doesn't support any triggers."));
    };
    let samplerate = device.get(&Key::Samplerate, None).ok().and_then(|v| v.as_u64()).unwrap_or(0);
    let channel = |name: &str| -> Result<usize> {
        device
            .channels()
            .iter()
            .find(|c| c.enabled && c.name == name)
            .map(|c| c.index)
            .ok_or_else(|| Error::Arg(format!("Invalid channel '{name}'.")))
    };

    let (sep, combine) = separator(spec)?;
    let mut trigger = Trigger::new();
    trigger.combine = combine;
    let mut base_stage = 0;
    for stage_text in spec.split(sep) {
        let mut deepest = base_stage;
        for token in stage_text.split(',') {
            let bad = || Error::Arg(format!("Invalid trigger '{token}'."));
            let (lhs, rhs) = token.split_once('=').ok_or_else(bad)?;
            if let Some((first, last)) = lhs.split_once('-') {
                let range = parse_range(first, last, rhs, &channel).ok_or_else(bad)?;
                trigger.set_range(base_stage, range);
                continue;
            }
            let ch = channel(lhs)?;
            let analog = device
                .channels()
                .iter()
                .any(|c| c.enabled && c.name == lhs && c.kind == ChannelKind::Analog);
            let modifier_at = rhs.find(['*', '>', '<']);
            let codes = &rhs[..modifier_at.unwrap_or(rhs.len())];
            if codes.is_empty() {
                return Err(bad());
            }
            for (i, code) in codes.chars().enumerate() {
                let m = TriggerMatch::from_code(code)
                    .ok_or_else(|| Error::Arg(format!("Invalid trigger match '{code}'.")))?;
                if !supported.contains(&m) {
                    return Err(Error::Arg(format!(
                        "Trigger match '{code}' not supported by device."
                    )));
                }
                // As `sr_trigger_match_add`: an index alone cannot tell an analog
                // channel from a logic one that shares it (link-mso19).
                if analog && matches!(m, TriggerMatch::Zero | TriggerMatch::One) {
                    return Err(Error::arg("Invalid trigger match for an analog channel."));
                }
                trigger.add(base_stage + i, ch, m);
                deepest = deepest.max(base_stage + i);
            }
            if let Some(at) = modifier_at {
                if codes.chars().count() > 1 {
                    return Err(bad());
                }
                apply_modifier(&mut trigger, base_stage, &rhs[at..], samplerate).ok_or_else(bad)?;
            }
        }
        base_stage = deepest + 1;
    }
    trigger.check(&device.trigger_features())?;
    Ok(trigger)
}

/// The stage separator in use and the combine it means.
fn separator(spec: &str) -> Result<(char, TriggerCombine)> {
    let kinds: Vec<char> = [';', '|', '&'].into_iter().filter(|c| spec.contains(*c)).collect();
    match kinds.as_slice() {
        [] | [';'] => Ok((';', TriggerCombine::Sequence)),
        ['|'] => Ok(('|', TriggerCombine::Or)),
        ['&'] => Ok(('&', TriggerCombine::And)),
        _ => Err(Error::arg("Mixed stage separators in trigger.")),
    }
}

fn number(text: &str) -> Option<u64> {
    if let Some(hex) = text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).ok()
    } else {
        text.parse().ok()
    }
}

/// `first-last=[!]lo[..hi]`: a value range on the channels from `first`
/// (least significant) to `last` by index.
fn parse_range(
    first: &str,
    last: &str,
    rhs: &str,
    channel: &dyn Fn(&str) -> Result<usize>,
) -> Option<TriggerRange> {
    let (a, b) = (channel(first).ok()?, channel(last).ok()?);
    if a > b {
        return None;
    }
    let (inside, values) =
        if let Some(rest) = rhs.strip_prefix('!') { (false, rest) } else { (true, rhs) };
    let (low, high) = if let Some((lo, hi)) = values.split_once("..") {
        (number(lo)?, number(hi)?)
    } else {
        let v = number(values)?;
        (v, v)
    };
    Some(TriggerRange { channels: (a..=b).collect(), low, high, inside })
}

/// A number of samples, or a time converted with the sample rate and
/// rounded down or up.
fn samples(text: &str, samplerate: u64, round_up: bool) -> Option<u64> {
    if let Ok(n) = text.parse::<u64>() {
        return Some(n);
    }
    let (p, q) = parse_period(text)?;
    if samplerate == 0 {
        return None;
    }
    let (n, q) = (u128::from(p) * u128::from(samplerate), u128::from(q.max(1)));
    let n = if round_up { n.div_ceil(q) } else { n / q };
    u64::try_from(n).ok()
}

/// `*N`, `*Nc`, `>A`, `<B` or `>A<B` on `stage`, where `A` and `B` are
/// samples or times. Returns `None` when the text is malformed.
fn apply_modifier(trigger: &mut Trigger, stage: usize, text: &str, samplerate: u64) -> Option<()> {
    if let Some(rest) = text.strip_prefix('*') {
        let (digits, consecutive) =
            if let Some(d) = rest.strip_suffix('c') { (d, true) } else { (rest, false) };
        let count: u32 = digits.parse().ok()?;
        trigger.set_count(stage, TriggerCount { count, consecutive });
        return Some(());
    }
    let duration = if let Some(below) = text.strip_prefix('<') {
        TriggerDuration::Shorter(samples(below, samplerate, true)?)
    } else {
        let rest = text.strip_prefix('>')?;
        match rest.split_once('<') {
            None => TriggerDuration::Longer(samples(rest, samplerate, false)?),
            // Both ends are strict. Bounds with nothing between them
            // stay as they are, for `check` to name.
            Some((above, below)) => {
                let above = samples(above, samplerate, false)?;
                let below = samples(below, samplerate, true)?;
                TriggerDuration::Between(above.checked_add(1)?, below.checked_sub(1)?)
            }
        }
    };
    trigger.set_duration(stage, duration);
    Some(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::channel::Channel;
    use crate::core::device::{DeviceClass, DeviceInfo};
    use crate::core::option::OptionDesc;
    use crate::core::trigger::TriggerFeatures;
    use crate::core::value::Value;
    use crate::core::{PacketSink, Result};

    /// Four logic channels, all matches, every feature, a 1 MHz rate.
    struct Dev {
        info: DeviceInfo,
        channels: Vec<Channel>,
        features: TriggerFeatures,
    }

    impl Device for Dev {
        fn info(&self) -> &DeviceInfo {
            &self.info
        }
        fn classes(&self) -> &[DeviceClass] {
            &[]
        }
        fn channels(&self) -> &[Channel] {
            &self.channels
        }
        fn enable_channel(&mut self, _: usize, _: bool) -> Result<()> {
            Ok(())
        }
        fn rename_channel(&mut self, _: usize, _: &str) -> Result<()> {
            Ok(())
        }
        fn options(&self, _: Option<&str>) -> Result<Vec<OptionDesc>> {
            Ok(Vec::new())
        }
        fn get(&self, key: &Key, _: Option<&str>) -> Result<Value> {
            match key {
                Key::Samplerate => Ok(Value::U64(1_000_000)),
                _ => Err(Error::UnknownKey(key.id().to_owned())),
            }
        }
        fn set(&mut self, _: &Key, _: &Value, _: Option<&str>) -> Result<()> {
            Ok(())
        }
        fn list(&self, key: &Key, _: Option<&str>) -> Result<ValueList> {
            match key {
                Key::TriggerMatch => Ok(ValueList::TriggerMatches(vec![
                    TriggerMatch::Zero,
                    TriggerMatch::One,
                    TriggerMatch::Rising,
                    TriggerMatch::Falling,
                    TriggerMatch::Edge,
                ])),
                _ => Err(Error::UnknownKey(key.id().to_owned())),
            }
        }
        fn trigger_features(&self) -> TriggerFeatures {
            self.features
        }
        fn start(&mut self, _: PacketSink) -> Result<()> {
            Ok(())
        }
        fn stop(&mut self) -> Result<()> {
            Ok(())
        }
    }

    fn dev(features: TriggerFeatures) -> Dev {
        Dev {
            info: DeviceInfo::default(),
            channels: (0..4).map(Channel::logic).collect(),
            features,
        }
    }

    fn all() -> TriggerFeatures {
        TriggerFeatures {
            count: true,
            consecutive: true,
            duration: true,
            pulse: true,
            range: true,
            or: true,
            and: true,
            max_stages: 2,
            holdoff: true,
        }
    }

    #[test]
    fn a_level_on_an_analog_channel_is_refused() {
        let mut d = dev(TriggerFeatures::default());
        d.channels.push(Channel::new(1, ChannelKind::Analog, "A0"));
        let refused = "Invalid trigger match for an analog channel";
        assert!(parse(&d, "A0=1").unwrap_err().to_string().contains(refused));
        assert!(parse(&d, "A0=0").unwrap_err().to_string().contains(refused));
        assert!(parse(&d, "A0=r").is_ok());
        assert!(parse(&d, "D1=1").is_ok());
    }

    #[test]
    fn sigrok_forms_are_unchanged() {
        let d = dev(TriggerFeatures::default());
        let t = parse(&d, "D0=r,D1=1;D2=f").unwrap();
        assert_eq!(t.stages.len(), 2);
        assert_eq!(t.stages[0].conditions.len(), 2);
        assert_eq!(t.stages[1].conditions[0].matches, TriggerMatch::Falling);
        let t = parse(&d, "D0=rf").unwrap();
        assert_eq!(t.stages.len(), 2);
        assert_eq!(t.combine, TriggerCombine::Sequence);
        assert!(parse(&d, "D9=r").unwrap_err().to_string().contains("Invalid channel"));
        assert!(parse(&d, "D0=x").unwrap_err().to_string().contains("Invalid trigger match"));
        assert!(parse(&d, "D0").unwrap_err().to_string().contains("Invalid trigger 'D0'"));
    }

    #[test]
    fn counts_durations_ranges_and_combines() {
        let d = dev(all());
        let t = parse(&d, "D0=r*5c,D1=1").unwrap();
        assert_eq!(t.stages[0].count, Some(TriggerCount { count: 5, consecutive: true }));
        let t = parse(&d, "D0=1>100").unwrap();
        assert_eq!(t.stages[0].duration, Some(TriggerDuration::Longer(100)));
        let t = parse(&d, "D0=1>50us").unwrap();
        assert_eq!(t.stages[0].duration, Some(TriggerDuration::Longer(50)));
        let t = parse(&d, "D0-D2=3..5").unwrap();
        let r = t.stages[0].range.as_ref().unwrap();
        assert_eq!((r.channels.as_slice(), r.low, r.high, r.inside), (&[0, 1, 2][..], 3, 5, true));
        let t = parse(&d, "D0-D2=!0x7").unwrap();
        let r = t.stages[0].range.as_ref().unwrap();
        assert_eq!((r.low, r.high, r.inside), (7, 7, false));
        let t = parse(&d, "D0=r|D1=r").unwrap();
        assert_eq!((t.combine, t.stages.len()), (TriggerCombine::Or, 2));
        let t = parse(&d, "D0=r&D1=r").unwrap();
        assert_eq!(t.combine, TriggerCombine::And);
        // A range with a count on the same stage, and a value too wide.
        let t = parse(&d, "D0-D1=2,D2=r*3").unwrap();
        assert!(t.stages[0].range.is_some() && t.stages[0].count.is_some());
        assert!(parse(&d, "D0-D1=5").unwrap_err().to_string().contains("does not fit 2"));
    }

    #[test]
    fn errors_name_the_problem() {
        let d = dev(all());
        for (spec, text) in [
            ("D0=r*", "Invalid trigger 'D0=r*'"),
            ("D0=r*0", "Invalid trigger count '0'"),
            ("D0=rf*3", "Invalid trigger 'D0=rf*3'"),
            ("D0=r;D1=r|D2=r", "Mixed stage separators"),
            ("D0=r|D1=r|D2=r", "Only 2 trigger stages"),
            ("D2-D0=1", "Invalid trigger 'D2-D0=1'"),
            ("D0=e>10", "one rising or falling edge"),
            ("D0=1<10", "needs the edge that ends the pulse"),
            ("D0=f<1", "shorter than 1 samples"),
            ("D0=f>10<11", "within 11..10 samples"),
            ("D0=f>10<0", "Invalid trigger 'D0=f>10<0'"),
            ("D0=f<", "Invalid trigger 'D0=f<'"),
            ("D0=f<10>5", "Invalid trigger 'D0=f<10>5'"),
            ("D0=f>5<10<20", "Invalid trigger 'D0=f>5<10<20'"),
            ("D0=f<10*2", "Invalid trigger 'D0=f<10*2'"),
            ("D0=rf<10", "Invalid trigger 'D0=rf<10'"),
        ] {
            let e = parse(&d, spec).unwrap_err().to_string();
            assert!(e.contains(text), "{spec}: {e}");
        }
        let plain = dev(TriggerFeatures::default());
        assert!(parse(&plain, "D0=r*2").unwrap_err().to_string().contains("counts not supported"));
        assert!(parse(&plain, "D0-D1=1").unwrap_err().to_string().contains("value ranges"));
        // A device with durations and no pulse widths, as the LogicPort.
        let levels = dev(TriggerFeatures { pulse: false, ..all() });
        assert!(parse(&levels, "D0=1>10").is_ok());
        for spec in ["D0=r>10", "D0=f<10", "D0=f>2<10"] {
            let e = parse(&levels, spec).unwrap_err().to_string();
            assert!(e.contains("Pulse width triggers not supported"), "{spec}: {e}");
        }
    }

    #[test]
    fn pulse_widths() {
        use TriggerDuration::{Between, Longer, Shorter};
        let d = dev(all());
        let pulse = |spec: &str| {
            let t = parse(&d, spec).unwrap_or_else(|e| panic!("{spec}: {e}"));
            assert_eq!(t.stages.len(), 1, "{spec}");
            let edge = t.stages[0].conditions.iter().find(|c| c.matches.is_edge()).unwrap();
            (edge.matches, t.stages[0].duration.unwrap())
        };
        assert_eq!(pulse("D0=f<100"), (TriggerMatch::Falling, Shorter(100)));
        assert_eq!(pulse("D0=r<100"), (TriggerMatch::Rising, Shorter(100)));
        assert_eq!(pulse("D0=f>10"), (TriggerMatch::Falling, Longer(10)));
        // Both ends are strict.
        assert_eq!(pulse("D0=f>10<50"), (TriggerMatch::Falling, Between(11, 49)));
        assert_eq!(pulse("D0=f>10<12"), (TriggerMatch::Falling, Between(11, 11)));
        // Times at 1 MHz: whole samples as they are.
        assert_eq!(pulse("D0=f<50us"), (TriggerMatch::Falling, Shorter(50)));
        assert_eq!(pulse("D0=f>10us<50us"), (TriggerMatch::Falling, Between(11, 49)));
        assert_eq!(pulse("D0=f>10us<1ms"), (TriggerMatch::Falling, Between(11, 999)));
        // A fraction of a sample rounds down after `>` and up after `<`:
        // longer than 2.5 samples is 3 and up, shorter than 2.5 is 2 and
        // down.
        assert_eq!(pulse("D0=f>2500ns"), (TriggerMatch::Falling, Longer(2)));
        assert_eq!(pulse("D0=f<2500ns"), (TriggerMatch::Falling, Shorter(3)));
        assert_eq!(pulse("D0=f>2500ns<7500ns"), (TriggerMatch::Falling, Between(3, 7)));
        // Longer than half a sample is any pulse: none is left out. A
        // level held for longer than no sample is still refused.
        assert_eq!(pulse("D0=f>500ns"), (TriggerMatch::Falling, Longer(0)));
        assert_eq!(pulse("D0=f>500ns<2500ns"), (TriggerMatch::Falling, Between(1, 2)));
        assert_eq!(pulse("D0=f>0"), (TriggerMatch::Falling, Longer(0)));
        let e = parse(&d, "D0=1>500ns").unwrap_err().to_string();
        assert!(e.contains("Invalid trigger duration '0'"), "{e}");
        // Levels and a range in the same stage, a sequence, or.
        let t = parse(&d, "D0=f<50us,D1=1,D2-D3=2").unwrap();
        assert_eq!(t.stages.len(), 1);
        assert_eq!(t.stages[0].conditions.len(), 2);
        assert!(t.stages[0].range.is_some());
        let t = parse(&d, "D1=r;D0=f<50").unwrap();
        assert_eq!(t.stages[1].duration, Some(Shorter(50)));
        let t = parse(&d, "D0=f<50|D1=r>20").unwrap();
        assert_eq!(t.combine, TriggerCombine::Or);
        assert_eq!(t.stages[1].duration, Some(Longer(20)));
    }
}
