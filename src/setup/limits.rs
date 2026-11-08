// Counterpart in sigrok:
//   sigrok-cli/session.c, main.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! How long a capture runs: `--time`, `--samples`, `--frames`,
//! `--continuous`.

use crate::core::util::{parse_sizestring, parse_timestring};
use crate::core::{Device, Error, Key, Result, Value, ValueList, config_set};

/// The limits a command line asks for, as written.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CaptureLimits {
    /// `--time`: ms, or with a unit (`2s`, `500ms`).
    pub time: Option<String>,
    /// `--samples`: a count with a size suffix (`1k`, `2m`).
    pub samples: Option<String>,
    /// `--frames`: a count.
    pub frames: Option<String>,
    /// `--continuous`.
    pub continuous: bool,
}

/// sigrok-cli's `maybe_config_set()` (main.c:87-97), which its `--samples` and
/// `--frames` use: a key the device cannot set fails without `check_key()`'s
/// message.
fn maybe_config_set(device: &mut dyn Device, key: &Key, value: u64) -> Result<()> {
    if device.option(key, None).is_some_and(|o| o.access.set) {
        config_set(device, key, &Value::U64(value), None)
    } else {
        Err(Error::not_supported(key.id()))
    }
}

/// The samples of a time limit. sigrok-cli's multiplication wraps
/// (session.c:61) and gives a limit unrelated to the time; here it saturates.
fn time_samples(rate: u64, msec: u64) -> u64 {
    rate.saturating_mul(msec) / 1000
}

fn set_limit_time(device: &mut dyn Device, text: &str) -> Result<u64> {
    let msec = parse_timestring(text)
        .filter(|t| *t > 0)
        .ok_or_else(|| Error::Arg(format!("Invalid time '{text}'")))?;
    if device.option(&Key::LimitMsec, None).is_some_and(|o| o.access.set) {
        config_set(device, &Key::LimitMsec, &Value::U64(msec), None)
            .map_err(|_| Error::arg("Failed to configure time limit."))?;
        return Ok(0);
    }
    if device.option(&Key::Samplerate, None).is_some_and(|o| o.access.get) {
        let rate = device.get(&Key::Samplerate, None)?.as_u64().unwrap_or(0);
        let samples = time_samples(rate, msec);
        if samples == 0 {
            return Err(Error::arg("Not enough time at this samplerate."));
        }
        config_set(device, &Key::LimitSamples, &Value::U64(samples), None)
            .map_err(|_| Error::arg("Failed to configure time-based sample limit."))?;
        return Ok(samples);
    }
    Err(Error::arg("This device does not support time limits."))
}

/// Configures the limits on the device, in sigrok-cli's order and with
/// its words, and returns the sample limit that resulted (0 when the
/// limit is not in samples), for the short-count warning after the run.
pub fn apply_limits(device: &mut dyn Device, limits: &CaptureLimits) -> Result<u64> {
    if limits.continuous && device.option(&Key::Continuous, None).is_none() {
        return Err(Error::arg("This device does not support continuous sampling."));
    }
    let mut limit_samples = 0u64;
    if let Some(t) = &limits.time {
        limit_samples = set_limit_time(device, t)?;
    }
    if let Some(s) = &limits.samples {
        limit_samples =
            parse_sizestring(s).map_err(|_| Error::Arg(format!("Invalid sample limit '{s}'.")))?;
        if let Ok(ValueList::MinMaxU64(min, max)) = device.list(&Key::LimitSamples, None) {
            if limit_samples < min {
                return Err(Error::Arg(format!(
                    "The device stores at least {min} samples with the current settings."
                )));
            }
            if limit_samples > max {
                return Err(Error::Arg(format!(
                    "The device can store only {max} samples with the current settings."
                )));
            }
        }
        maybe_config_set(device, &Key::LimitSamples, limit_samples)
            .map_err(|_| Error::arg("Failed to configure sample limit."))?;
    }
    if let Some(f) = &limits.frames {
        let frames =
            parse_sizestring(f).map_err(|_| Error::Arg(format!("Invalid frame limit '{f}'.")))?;
        maybe_config_set(device, &Key::LimitFrames, frames)
            .map_err(|_| Error::arg("Failed to configure frame limit."))?;
    }
    Ok(limit_samples)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{Access, Channel, DeviceClass, DeviceInfo, OptionDesc, PacketSink};

    #[test]
    fn a_time_limit_does_not_overflow() {
        assert_eq!(time_samples(1000, 1500), 1500);
        assert_eq!(time_samples(200_000, 1), 200);
        assert_eq!(time_samples(999, 1), 0);
        assert_eq!(time_samples(u64::MAX, 2), u64::MAX / 1000);
        assert_eq!(time_samples(2, u64::MAX), u64::MAX / 1000);
    }

    /// A device that reads `limit_samples` and has no way to set it.
    struct ReadsLimit {
        info: DeviceInfo,
        set_calls: usize,
    }

    impl Device for ReadsLimit {
        fn info(&self) -> &DeviceInfo {
            &self.info
        }
        fn classes(&self) -> &[DeviceClass] {
            &[]
        }
        fn channels(&self) -> &[Channel] {
            &[]
        }
        fn enable_channel(&mut self, _: usize, _: bool) -> Result<()> {
            Ok(())
        }
        fn rename_channel(&mut self, _: usize, _: &str) -> Result<()> {
            Ok(())
        }
        fn options(&self, _: Option<&str>) -> Result<Vec<OptionDesc>> {
            Ok(vec![OptionDesc::new(Key::LimitSamples, Access::GET)])
        }
        fn get(&self, key: &Key, _: Option<&str>) -> Result<Value> {
            Err(Error::UnknownKey(key.id().to_owned()))
        }
        fn set(&mut self, _: &Key, _: &Value, _: Option<&str>) -> Result<()> {
            self.set_calls += 1;
            Ok(())
        }
        fn list(&self, key: &Key, _: Option<&str>) -> Result<ValueList> {
            Err(Error::UnknownKey(key.id().to_owned()))
        }
        fn start(&mut self, _: PacketSink) -> Result<()> {
            Ok(())
        }
        fn stop(&mut self) -> Result<()> {
            Ok(())
        }
    }

    /// sigrok-cli's --samples and --frames fail silently here: not through
    /// `check_key()`, whose message `config_set` would log.
    #[test]
    fn samples_ask_for_set_access_first() {
        let mut dev = ReadsLimit { info: DeviceInfo::default(), set_calls: 0 };
        let e = maybe_config_set(&mut dev, &Key::LimitSamples, 5).unwrap_err();
        assert!(matches!(&e, Error::NotSupported(_)), "{e}");
        let e = maybe_config_set(&mut dev, &Key::LimitFrames, 5).unwrap_err();
        assert!(matches!(&e, Error::NotSupported(_)), "{e}");
        assert_eq!(dev.set_calls, 0);
        let limits = CaptureLimits { samples: Some("5".into()), ..CaptureLimits::default() };
        let e = apply_limits(&mut dev, &limits).unwrap_err();
        assert!(matches!(&e, Error::Arg(m) if m == "Failed to configure sample limit."), "{e}");
        let limits = CaptureLimits { continuous: true, ..CaptureLimits::default() };
        let e = apply_limits(&mut dev, &limits).unwrap_err();
        assert!(
            matches!(&e, Error::Arg(m) if m == "This device does not support continuous sampling."),
            "{e}"
        );
    }

    #[test]
    fn the_demo_device_takes_samples_and_time() {
        let mut r = crate::core::Registry::new();
        r.register(Box::new(crate::drivers::demo::DemoDriver));
        let Ok(mut dev) = super::super::pick_device(&r, Some("demo")) else { panic!("demo") };
        let limits = CaptureLimits { samples: Some("1k".into()), ..CaptureLimits::default() };
        assert_eq!(apply_limits(dev.as_mut(), &limits).unwrap(), 1000);
        let limits = CaptureLimits { time: Some("10".into()), ..CaptureLimits::default() };
        // The demo takes `limit_msec` itself, so the limit is not in samples.
        assert_eq!(apply_limits(dev.as_mut(), &limits).unwrap(), 0);
        assert_eq!(dev.get(&Key::LimitMsec, None).unwrap().as_u64(), Some(10));
        let limits = CaptureLimits { samples: Some("lots".into()), ..CaptureLimits::default() };
        let e = apply_limits(dev.as_mut(), &limits).unwrap_err();
        assert!(matches!(&e, Error::Arg(m) if m == "Invalid sample limit 'lots'."), "{e}");
    }
}
