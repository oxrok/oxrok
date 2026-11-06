// Counterpart in sigrok:
//   libsigrok/src/hardware/juntek-jds6600/api.c,
//   Copyright (C) 2023 Gerhard Sittig <gerhard.sittig@gmx.net>
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Juntek JDS6600 DDS signal generator, also sold as the Joy-IT JDS6600,
//! two channels on a CH340 at 115200/8n1.
//!
//! `conn=` names the port. A scan reads the type, the serial number and all
//! the channel state, and sets nothing. An open and a close send nothing.
//! `--get` asks the device each time, and each `set` sends one request and
//! waits for `:ok`.
//!
//! The device acquires nothing: it has no limit keys, so a command line asking
//! for a capture is refused. The channels start disabled, so a session refuses
//! to start one (`Session::run`, as `sr_session_start` in C). Only a client that
//! calls [`Device::start`] itself gets a header and an end at once.
//!
//! Where this leaves `api.c`:
//!
//! - A NaN is refused, as it would go out as text the firmware was never seen
//!   to take, and so is an infinite phase, whose remainder is one. An infinite
//!   value elsewhere is cut to the limit, as there.
//! - An enable the device did not take is not kept, where `api.c:246` and
//!   `277` keep it for the next write to send.
//! - Every setting is narrowed to single precision, which is sigrok-cli's
//!   `strtof` (`session.c:570`) and not the driver's: `api.c:296` and
//!   `protocol.c:731-734` keep a double. It gives a full comparison with
//!   sigrok-cli, and costs a library client digits: 12345678.91 Hz goes out as
//!   `1234567900,0` here and `1234567891,0` in C, and a phase of 1e300 is
//!   refused here as it narrows to infinity. Narrowing in the CLI's parsing
//!   instead would be a shared change.
//!
//! See [`protocol`].

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelGroup, ChannelKind, Device, DeviceClass, DeviceInfo, Driver, Error,
    Key, OptionDesc, Packet, PacketSink, Result, Value, ValueList,
};
use crate::serial::{Params, Port};
use protocol::{CHANNELS, Link};
use std::sync::Mutex;

const SCAN_OPTIONS: [Key; 1] = [Key::Conn];
const CLASSES: [DeviceClass; 1] = [DeviceClass::SignalGenerator];
const SERIALCOMM: &str = "115200/8n1";
/// `SR_CONF_SET` alone.
const SET_ONLY: Access = Access { get: false, set: true, list: false };
static DEV_OPTIONS: [OptionDesc; 3] = [
    OptionDesc::new(Key::Conn, Access::GET),
    OptionDesc::new(Key::Enabled, SET_ONLY),
    OptionDesc::new(Key::Phase, Access::GET_SET),
];
static GROUP_OPTIONS: [OptionDesc; 6] = [
    OptionDesc::new(Key::Enabled, Access::GET_SET),
    OptionDesc::new(Key::PatternMode, Access::GET_SET_LIST),
    OptionDesc::new(Key::OutputFrequency, Access::GET_SET_LIST),
    OptionDesc::new(Key::Amplitude, Access::GET_SET),
    OptionDesc::new(Key::Offset, Access::GET_SET),
    OptionDesc::new(Key::DutyCycle, Access::GET_SET_LIST),
];
/// `config_list`'s frequency range starts and steps at this.
const FREQUENCY_STEP: f64 = 0.01;
const DUTY_STEP: f64 = 0.001;

/// The `juntek-jds6600` driver.
pub struct JdsDriver;

impl Driver for JdsDriver {
    fn id(&self) -> &'static str {
        "juntek-jds6600"
    }

    fn long_name(&self) -> &'static str {
        "JUNTEK JDS6600"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let Some(conn) = conn else {
            return Ok(Vec::new());
        };
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        // Every failure here is no device, as `scan` returns NULL for each.
        let port = match crate::serial::open(&conn, params) {
            Ok(port) => port,
            Err(e) => {
                log::error!("{e}");
                return Ok(Vec::new());
            }
        };
        let mut link = Link::new(Some(port));
        if let Err(e) = link.probe() {
            log::debug!("Scan failed: {e}.");
            return Ok(Vec::new());
        }
        // The scan's `serial_close`; `dev_open` opens it again.
        link.port = None;
        Ok(vec![Box::new(JdsDevice::new(link, conn, params))])
    }
}

/// One generator.
pub struct JdsDevice {
    info: DeviceInfo,
    channels: Vec<Channel>,
    groups: Vec<ChannelGroup>,
    names: Vec<String>,
    conn: String,
    params: Params,
    /// A port to use instead of opening `conn`, for tests.
    pending: Option<Box<dyn Port>>,
    link: Mutex<Link>,
}

impl JdsDevice {
    fn new(link: Link, conn: String, params: Params) -> Self {
        let names: Vec<String> = (1..=CHANNELS).map(|n| format!("CH{n}")).collect();
        JdsDevice {
            info: DeviceInfo {
                driver: "juntek-jds6600",
                vendor: "Juntek".to_owned(),
                model: "JDS6600".to_owned(),
                version: String::new(),
                serial: link.serial.clone(),
                conn: Some(conn.clone()),
            },
            // Disabled, as `api.c:109-110` makes them: they measure nothing.
            channels: names
                .iter()
                .enumerate()
                .map(|(i, name)| {
                    let mut channel = Channel::new(i, ChannelKind::Analog, name.clone());
                    channel.enabled = false;
                    channel
                })
                .collect(),
            groups: names
                .iter()
                .enumerate()
                .map(|(i, name)| ChannelGroup::new(name.clone(), vec![i]))
                .collect(),
            names: protocol::wave_names(),
            conn,
            params,
            pending: None,
            link: Mutex::new(link),
        }
    }

    /// A device that a scan of `port` found; [`Device::open`] takes the port.
    /// For tests and for clients with a port of their own.
    ///
    /// # Errors
    ///
    /// What the scan's requests give.
    pub fn from_port(mut port: Box<dyn Port>) -> Result<Self> {
        let params = Params::parse(SERIALCOMM).expect("the default parses");
        let conn = port.name().to_owned();
        port.purge()?;
        let mut link = Link::new(Some(port));
        link.probe()?;
        let port = link.port.take();
        let mut device = Self::new(link, conn, params);
        device.pending = port;
        Ok(device)
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }

    /// Which channel `group` names, `None` being all.
    fn channel(&self, group: Option<&str>) -> Result<Option<usize>> {
        let Some(name) = group else { return Ok(None) };
        self.groups
            .iter()
            .position(|g| g.name.eq_ignore_ascii_case(name))
            .map(Some)
            .ok_or_else(|| Error::ChannelGroup(name.to_owned()))
    }

    fn set_group(&self, key: &Key, value: &Value, channel: usize) -> Result<()> {
        let mut link = self.link();
        match key {
            Key::Enabled => {
                let mut enabled = link.enabled;
                enabled[channel] = on(value)?;
                link.set_enabled(enabled)
            }
            Key::PatternMode => {
                let index = value
                    .as_str()
                    .and_then(|name| self.names.iter().position(|n| n == name))
                    .ok_or_else(|| Error::arg("no such pattern"))?;
                link.set_waveform(channel, index)
            }
            Key::OutputFrequency => link.set_frequency(channel, number(key, value)?),
            Key::Amplitude => link.set_amplitude(channel, number(key, value)?),
            Key::Offset => link.set_offset(channel, number(key, value)?),
            Key::DutyCycle => link.set_duty(channel, number(key, value)?),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }
}

fn on(value: &Value) -> Result<bool> {
    value.as_bool().ok_or_else(|| Error::arg("enabled takes on or off"))
}

/// A setting's number, in the single precision sigrok-cli reads it in, so that
/// the same text gives the same request (0.0005 V is 1 mV in it, 0 in a
/// double). See the module's documentation on what that costs and on a NaN.
fn number(key: &Key, value: &Value) -> Result<f64> {
    let refused = || Error::Arg(format!("{} takes a number", key.id()));
    let v = f64::from(value.as_f64().ok_or_else(refused)? as f32);
    if v.is_nan() { Err(refused()) } else { Ok(v) }
}

impl Device for JdsDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &self.channels
    }

    fn channel_groups(&self) -> &[ChannelGroup] {
        &self.groups
    }

    fn enable_channel(&mut self, pos: usize, enabled: bool) -> Result<()> {
        let ch = self.channels.get_mut(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?;
        ch.enabled = enabled;
        Ok(())
    }

    fn rename_channel(&mut self, pos: usize, name: &str) -> Result<()> {
        let ch = self.channels.get_mut(pos).ok_or(Error::Arg(format!("No channel {pos}.")))?;
        name.clone_into(&mut ch.name);
        Ok(())
    }

    fn options(&self, group: Option<&str>) -> Result<Vec<OptionDesc>> {
        Ok(match self.channel(group)? {
            None => DEV_OPTIONS.to_vec(),
            Some(_) => GROUP_OPTIONS.to_vec(),
        })
    }

    fn get(&self, key: &Key, group: Option<&str>) -> Result<Value> {
        let channel = self.channel(group)?;
        let mut link = self.link();
        let unknown = || Error::UnknownKey(key.id().to_owned());
        let Some(ch) = channel else {
            return match key {
                Key::Conn => Ok(Value::Str(self.conn.clone())),
                Key::Phase => Ok(Value::F64(link.get_phase()?)),
                _ => Err(unknown()),
            };
        };
        Ok(match key {
            Key::Enabled => {
                link.get_enabled()?;
                Value::Bool(link.enabled[ch])
            }
            Key::PatternMode => Value::Str(self.names[link.get_waveform(ch)?].clone()),
            Key::OutputFrequency => Value::F64(link.get_frequency(ch)?),
            Key::Amplitude => Value::F64(link.get_amplitude(ch)?),
            Key::Offset => Value::F64(link.get_offset(ch)?),
            Key::DutyCycle => Value::F64(link.get_duty(ch)?),
            _ => return Err(unknown()),
        })
    }

    fn set(&mut self, key: &Key, value: &Value, group: Option<&str>) -> Result<()> {
        if let Some(channel) = self.channel(group)? {
            return self.set_group(key, value, channel);
        }
        let mut link = self.link();
        match key {
            Key::Enabled => link.set_enabled([on(value)?; CHANNELS]),
            Key::Phase => {
                let degrees = number(key, value)?;
                if degrees.is_infinite() {
                    return Err(Error::Arg("phase takes a number".to_owned()));
                }
                link.set_phase(degrees)
            }
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn list(&self, key: &Key, group: Option<&str>) -> Result<ValueList> {
        let unknown = || Error::UnknownKey(key.id().to_owned());
        self.channel(group)?.ok_or_else(unknown)?;
        match key {
            Key::PatternMode => Ok(ValueList::Str(self.names.clone())),
            Key::OutputFrequency => Ok(ValueList::RangeStepF64 {
                min: FREQUENCY_STEP,
                max: self.link().top_frequency as f64,
                step: FREQUENCY_STEP,
            }),
            Key::DutyCycle => Ok(ValueList::RangeStepF64 { min: 0.0, max: 1.0, step: DUTY_STEP }),
            _ => Err(unknown()),
        }
    }

    /// `std_serial_dev_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        let mut link = self.link.lock().expect("link lock");
        if link.port.is_some() {
            return Ok(());
        }
        let port = match self.pending.take() {
            Some(port) => port,
            None => crate::serial::open(&self.conn, self.params)?,
        };
        link.port = Some(port);
        Ok(())
    }

    /// `std_serial_dev_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.link().port = None;
        Ok(())
    }

    /// `std_dummy_dev_acquisition_start`: nothing to acquire, so the session
    /// ends at once.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        sink.send(Packet::header());
        sink.end(Ok(()));
        Ok(())
    }

    /// `std_dummy_dev_acquisition_stop`.
    fn stop(&mut self) -> Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::protocol::tests::{Fake, SCAN};
    use super::*;

    /// A device through a scan, open, with `replies` to what follows. The
    /// scan's replies have the first channel on and the second off.
    fn device(replies: &[&str]) -> (JdsDevice, Fake) {
        let all: Vec<&str> = SCAN.iter().copied().chain(replies.iter().copied()).collect();
        let fake = Fake::with(&all);
        let mut device = JdsDevice::from_port(Box::new(fake.clone())).unwrap();
        device.open().unwrap();
        (device, fake)
    }

    /// What was sent after the scan.
    fn sent(fake: &Fake) -> Vec<String> {
        fake.writes()[SCAN.len()..].to_vec()
    }

    #[test]
    fn it_says_what_it_is() {
        let (d, _) = device(&[]);
        assert_eq!(d.info().vendor, "Juntek");
        assert_eq!(d.info().model, "JDS6600");
        assert_eq!(d.info().serial, "1234");
        let groups: Vec<&str> = d.channel_groups().iter().map(|g| g.name.as_str()).collect();
        assert_eq!(groups, ["CH1", "CH2"]);
        let channels: Vec<&str> = d.channels().iter().map(|c| c.name.as_str()).collect();
        assert_eq!(channels, ["CH1", "CH2"]);
        assert!(d.channels().iter().all(|c| !c.enabled));
        assert_eq!(d.classes(), [DeviceClass::SignalGenerator]);
        assert_eq!(JdsDriver.scan_options(), [Key::Conn]);
    }

    #[test]
    fn no_conn_is_no_device() {
        assert!(JdsDriver.scan(&[]).unwrap().is_empty());
        assert!(JdsDriver.scan(&[(Key::Serialcomm, Value::from("9600/8n1"))]).is_err());
    }

    #[test]
    fn opening_and_closing_send_nothing() {
        let (mut d, fake) = device(&[]);
        d.close().unwrap();
        assert_eq!(sent(&fake), Vec::<String>::new());
        assert!(d.get(&Key::Phase, None).is_err(), "closed");
    }

    #[test]
    fn a_set_sends_its_one_request_and_waits_for_ok() {
        for (group, key, value, want) in [
            (Some("CH1"), Key::PatternMode, Value::from("arb-60"), ":w21=160.\r\n"),
            (Some("CH2"), Key::PatternMode, Value::from("sine"), ":w22=0.\r\n"),
            (Some("CH1"), Key::OutputFrequency, Value::F64(1000.0), ":w23=100000,0.\r\n"),
            (Some("ch2"), Key::OutputFrequency, Value::F64(7e7), ":w24=6000000000,0.\r\n"),
            (Some("CH1"), Key::Amplitude, Value::F64(5.0), ":w25=5000.\r\n"),
            (Some("CH2"), Key::Amplitude, Value::F64(0.0005), ":w26=1.\r\n"),
            (Some("CH1"), Key::Offset, Value::F64(1.0), ":w27=1100.\r\n"),
            (Some("CH2"), Key::Offset, Value::F64(-1.0), ":w28=900.\r\n"),
            (Some("CH1"), Key::DutyCycle, Value::F64(0.5), ":w29=500.\r\n"),
            (Some("CH2"), Key::DutyCycle, Value::F64(0.25), ":w30=250.\r\n"),
            (None, Key::Phase, Value::F64(90.0), ":w31=900.\r\n"),
            (None, Key::Enabled, Value::Bool(false), ":w20=0,0.\r\n"),
            (None, Key::Enabled, Value::Bool(true), ":w20=1,1.\r\n"),
            (Some("CH1"), Key::Enabled, Value::Bool(false), ":w20=0,0.\r\n"),
            (Some("CH2"), Key::Enabled, Value::Bool(true), ":w20=1,1.\r\n"),
        ] {
            let (mut d, fake) = device(&[":ok\r\n"]);
            d.set(&key, &value, group).unwrap();
            assert_eq!(sent(&fake), [want], "{key} {group:?}");
            let (mut d, _) = device(&[":err\r\n"]);
            assert!(matches!(d.set(&key, &value, group), Err(Error::Device(_))), "{key} {group:?}");
        }
    }

    #[test]
    fn what_is_refused_sends_nothing() {
        for (group, key, value) in [
            (Some("CH1"), Key::OutputFrequency, Value::F64(f64::NAN)),
            (Some("CH1"), Key::Amplitude, Value::F64(f64::NAN)),
            (Some("CH1"), Key::Offset, Value::F64(f64::NAN)),
            (Some("CH1"), Key::DutyCycle, Value::F64(f64::NAN)),
            (None, Key::Phase, Value::F64(f64::NAN)),
            (None, Key::Phase, Value::F64(f64::INFINITY)),
            (None, Key::Phase, Value::F64(f64::NEG_INFINITY)),
            (Some("CH1"), Key::PatternMode, Value::from("arb-61")),
            (Some("CH1"), Key::Amplitude, Value::Bool(true)),
            (Some("CH1"), Key::Enabled, Value::F64(1.0)),
            (None, Key::Enabled, Value::F64(1.0)),
            (None, Key::Amplitude, Value::F64(1.0)),
            (Some("CH1"), Key::Phase, Value::F64(1.0)),
            (Some("CH3"), Key::Amplitude, Value::F64(1.0)),
        ] {
            let (mut d, fake) = device(&[":ok\r\n"]);
            assert!(d.set(&key, &value, group).is_err(), "{key} {group:?}");
            assert_eq!(sent(&fake), Vec::<String>::new(), "{key} {group:?}");
        }
    }

    #[test]
    fn a_library_clients_number_is_narrowed_too() {
        let (mut d, fake) = device(&[":ok\r\n"]);
        d.set(&Key::OutputFrequency, &Value::F64(12_345_678.91), Some("CH1")).unwrap();
        assert_eq!(sent(&fake), [":w23=1234567900,0.\r\n"]);
        let (mut d, fake) = device(&[":ok\r\n"]);
        assert!(d.set(&Key::Phase, &Value::F64(1e300), None).is_err());
        assert_eq!(sent(&fake), Vec::<String>::new());
    }

    #[test]
    fn infinities_are_cut_to_the_limits() {
        let (mut d, fake) = device(&[":ok\r\n", ":ok\r\n"]);
        d.set(&Key::Amplitude, &Value::F64(f64::INFINITY), Some("CH1")).unwrap();
        d.set(&Key::OutputFrequency, &Value::F64(f64::NEG_INFINITY), Some("CH1")).unwrap();
        assert_eq!(sent(&fake), [":w25=20000.\r\n", ":w23=1,0.\r\n"]);
    }

    #[test]
    fn one_channels_enable_sends_the_other_as_last_seen() {
        let (mut d, fake) = device(&[":r20=0,1.\r\n", ":ok\r\n"]);
        assert_eq!(d.get(&Key::Enabled, Some("CH2")).unwrap(), Value::Bool(true));
        d.set(&Key::Enabled, &Value::Bool(true), Some("CH1")).unwrap();
        assert_eq!(sent(&fake), [":r20=0.\r\n", ":w20=1,1.\r\n"]);
    }

    #[test]
    fn a_get_asks_the_device() {
        for (group, key, reply, want, asked) in [
            (Some("CH1"), Key::PatternMode, ":r21=101.\r\n", Value::from("arb-01"), ":r21=0.\r\n"),
            (Some("CH2"), Key::PatternMode, ":r22=x.\r\n", Value::from("sine"), ":r22=0.\r\n"),
            (
                Some("CH2"),
                Key::OutputFrequency,
                ":r24=100,3.\r\n",
                Value::F64(1000.0),
                ":r24=0.\r\n",
            ),
            (Some("CH1"), Key::Amplitude, ":r25=500.\r\n", Value::F64(0.5), ":r25=0.\r\n"),
            (Some("CH2"), Key::Offset, ":r28=1100.\r\n", Value::F64(1.0), ":r28=0.\r\n"),
            (Some("CH1"), Key::DutyCycle, ":r29=250.\r\n", Value::F64(0.25), ":r29=0.\r\n"),
            (None, Key::Phase, ":r31=900.\r\n", Value::F64(90.0), ":r31=0.\r\n"),
        ] {
            let (d, fake) = device(&[reply]);
            assert_eq!(d.get(&key, group).unwrap(), want, "{key}");
            assert_eq!(sent(&fake), [asked], "{key}");
        }
    }

    #[test]
    fn a_get_that_goes_wrong_is_an_error() {
        for (group, key) in [
            (Some("CH1"), Key::Enabled),
            (Some("CH1"), Key::PatternMode),
            (Some("CH1"), Key::OutputFrequency),
            (Some("CH1"), Key::Amplitude),
            (Some("CH1"), Key::Offset),
            (Some("CH1"), Key::DutyCycle),
            (None, Key::Phase),
        ] {
            // Device, as the scenario's filter maps that, and no other, to C's words.
            let (d, _) = device(&[":x\r\n"]);
            assert!(matches!(d.get(&key, group), Err(Error::Device(_))), "{key}");
        }
        let (d, _) = device(&[]);
        assert!(d.get(&Key::Amplitude, None).is_err());
        assert!(d.get(&Key::Phase, Some("CH1")).is_err());
        assert!(d.get(&Key::Conn, Some("CH1")).is_err());
        assert_eq!(d.get(&Key::Conn, None).unwrap(), Value::from("fake"));
    }

    #[test]
    fn the_lists_are_as_there() {
        let (d, _) = device(&[]);
        let Ok(ValueList::Str(names)) = d.list(&Key::PatternMode, Some("CH1")) else { panic!() };
        assert_eq!(names.len(), 77);
        let frequency = d.list(&Key::OutputFrequency, Some("CH2")).unwrap();
        let ValueList::RangeStepF64 { min, max, step } = frequency else { panic!() };
        assert_eq!((min, max, step), (0.01, 60e6, 0.01));
        let duty = d.list(&Key::DutyCycle, Some("CH1")).unwrap();
        let ValueList::RangeStepF64 { min, max, step } = duty else { panic!() };
        assert_eq!((min, max, step), (0.0, 1.0, 0.001));
        assert!(d.list(&Key::Amplitude, Some("CH1")).is_err());
        assert!(d.list(&Key::PatternMode, None).is_err());
    }

    #[test]
    fn the_options_are_as_there_in_order() {
        let (d, _) = device(&[]);
        let keys =
            |group| -> Vec<Key> { d.options(group).unwrap().into_iter().map(|o| o.key).collect() };
        assert_eq!(keys(None), [Key::Conn, Key::Enabled, Key::Phase]);
        assert_eq!(
            keys(Some("CH1")),
            [
                Key::Enabled,
                Key::PatternMode,
                Key::OutputFrequency,
                Key::Amplitude,
                Key::Offset,
                Key::DutyCycle
            ]
        );
    }

    #[test]
    fn a_capture_is_a_header_and_an_end() {
        let (mut d, fake) = device(&[]);
        let (tx, rx) = std::sync::mpsc::sync_channel(4);
        d.start(PacketSink::new(tx, 0)).unwrap();
        assert!(matches!(rx.recv().unwrap().1, Packet::Header { .. }));
        assert!(matches!(rx.recv().unwrap().1, Packet::End));
        d.stop().unwrap();
        assert_eq!(sent(&fake), Vec::<String>::new());
    }
}
