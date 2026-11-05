// Counterpart in sigrok:
//   libsigrok/src/hardware/rohde-schwarz-sme-0x/api.c,
//   Copyright (C) 2016 Vlad Ivanov <vlad.ivanov@lab-systems.ru>,
//   with commits by Uwe Hermann, Gerhard Sittig, Soeren Apel
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The Rohde&Schwarz SME series of signal generators: SME02, SME03, SME03A,
//! SME03E and SME06, with SCPI on the serial port, or on anything else [`scpi`]
//! has, as `conn=` says. C has GPIB too, and oxrok has no GPIB.
//!
//! A scan sends a bare LF for remote control, asks `*IDN?` and closes. An open
//! and a close send nothing. `--get` asks the device each time, and a setting
//! is a line with no answer, so a value the device does not take goes unnoticed.
//! With no `serialcomm=`, the port's line stays what it is, as in C.
//!
//! The device acquires nothing and has no channels: it has no limit keys, so a
//! command line asking for a capture is refused, and a session refuses to start
//! one. Only a client that calls [`Device::start`] itself gets a header and an
//! end at once. Where C's dummy start sends neither, and its stop takes the SCPI
//! instance for a serial port (`api.c:236-237`, `std.c:366`), no session gets
//! that far.
//!
//! Where this leaves `api.c` and `protocol.c`:
//!
//! - A failed read is an error, where `config_get` drops the result and returns
//!   a double it never set (`api.c:170`, `176-177`, `180-181`).
//! - A value out of range, and a failed write, are errors, where `config_set`
//!   drops the result and says OK (`api.c:203`, `207`).
//! - A NaN is refused, where it passes `protocol.c:71`'s and `:85`'s comparisons
//!   and goes out as `FREQ nanHz`.
//! - An empty reply is no reading, and what is not finite is none, where
//!   `sr_atod_ascii` gives 0 and `inf`.
//! - A reply that cannot be the query's answer is an error. See [`protocol`].
//! - The SME06's top is 6 GHz: [`protocol::MODELS`].
//! - With no `conn=`, only USBTMC devices are probed. C also opens serial ports
//!   by a table of USB ids (`scpi_serial.c:43-52`), with a line each, none of
//!   them this instrument's, which has no USB.
//! - A setting stays a double, as in the driver. sigrok-cli's `strtof`
//!   (`session.c:570`) narrows it first, so 123456789 Hz goes out from sigrok-cli
//!   as 123456792.0 and from here as 123456789.0.

#![forbid(unsafe_code)]

pub mod protocol;

use crate::core::{
    Access, Channel, ChannelGroup, Device, DeviceClass, DeviceInfo, Driver, Error, Key, OptionDesc,
    Packet, PacketSink, Result, Value, ValueList,
};
use crate::scpi::{self, Resource, Scpi};
use protocol::Link;
use std::sync::Mutex;

const SCAN_OPTIONS: [Key; 2] = [Key::Conn, Key::Serialcomm];
const CLASSES: [DeviceClass; 1] = [DeviceClass::SignalGenerator];
static DEV_OPTIONS: [OptionDesc; 2] = [
    OptionDesc::new(Key::OutputFrequency, Access::GET_SET_LIST),
    OptionDesc::new(Key::Amplitude, Access::GET_SET_LIST),
];

/// The `rohde-schwarz-sme-0x` driver.
pub struct SmeDriver;

/// `probe_device`: a generator of the table, or `None`.
fn probe(mut scpi: Scpi, serialcomm: Option<&str>) -> Option<SmeDevice> {
    // Its result is dropped in C too: the query that follows says.
    if let Err(e) = scpi.send("\n") {
        log::debug!("Remote control: {e}.");
    }
    let id = match scpi.hw_id() {
        Ok(id) => id,
        Err(e) => {
            log::debug!("*IDN?: {e}.");
            return None;
        }
    };
    let Some(model) = protocol::find_model(&id) else {
        log::debug!("Device {} {} is not supported by this driver.", id.manufacturer, id.model);
        return None;
    };
    let resource = scpi.resource().to_owned();
    Some(SmeDevice::new(&id, *model, resource, serialcomm.map(str::to_owned)))
}

impl Driver for SmeDriver {
    fn id(&self) -> &'static str {
        "rohde-schwarz-sme-0x"
    }

    fn long_name(&self) -> &'static str {
        "Rohde&Schwarz SME-0x"
    }

    fn scan_options(&self) -> &'static [Key] {
        &SCAN_OPTIONS
    }

    fn classes(&self) -> &'static [DeviceClass] {
        &CLASSES
    }

    fn scan(&self, options: &[(Key, Value)]) -> Result<Vec<Box<dyn Device>>> {
        let mut conn = None;
        let mut serialcomm = None;
        for (key, value) in options {
            match key {
                Key::Conn => conn = value.as_str().map(str::to_owned),
                Key::Serialcomm => serialcomm = value.as_str().map(str::to_owned),
                other => return Err(Error::UnknownKey(other.id().to_owned())),
            }
        }
        let comm = serialcomm.as_deref();
        // What fails here is no device, as `sr_scpi_scan_resource` has it.
        let resource = match conn.as_deref().map(Resource::parse).transpose() {
            Ok(resource) => resource,
            Err(e) => {
                log::info!("{e}");
                return Ok(Vec::new());
            }
        };
        let found = match resource {
            None | Some(Resource::Usbtmc(None)) => {
                match scpi::scan(conn.as_deref(), comm, |s| Ok(probe(s, comm))) {
                    Ok(found) => found,
                    Err(e) => {
                        log::info!("{e}");
                        Vec::new()
                    }
                }
            }
            Some(resource) => match scpi::open_keeping_line(&resource, comm) {
                Ok(s) => probe(s, comm).into_iter().collect(),
                Err(e) => {
                    log::info!("Couldn't open SCPI device: {e}");
                    Vec::new()
                }
            },
        };
        Ok(found.into_iter().map(|d| Box::new(d) as Box<dyn Device>).collect())
    }
}

/// One generator.
pub struct SmeDevice {
    info: DeviceInfo,
    resource: String,
    serialcomm: Option<String>,
    /// A transport to use instead of opening `resource`, for tests.
    pending: Option<Scpi>,
    link: Mutex<Link>,
}

impl SmeDevice {
    fn new(
        id: &scpi::HwInfo,
        model: protocol::Model,
        resource: String,
        serialcomm: Option<String>,
    ) -> Self {
        SmeDevice {
            info: DeviceInfo {
                driver: "rohde-schwarz-sme-0x",
                vendor: id.manufacturer.clone(),
                model: id.model.clone(),
                version: id.firmware_version.clone(),
                serial: id.serial_number.clone(),
                conn: None,
            },
            resource,
            serialcomm,
            pending: None,
            link: Mutex::new(Link::new(model)),
        }
    }

    /// A device that a scan of `scpi` found; [`Device::open`] takes `scpi`. For
    /// tests and for clients with a transport of their own.
    ///
    /// # Errors
    ///
    /// When it is not one of the table's, or does not answer.
    pub fn from_scpi(mut scpi: Scpi) -> Result<Self> {
        scpi.send("\n")?;
        let id = scpi.hw_id()?;
        let model = protocol::find_model(&id).ok_or_else(|| {
            Error::device(format!("Not supported: {} {}.", id.manufacturer, id.model))
        })?;
        let mut device = Self::new(&id, *model, scpi.resource().to_owned(), None);
        device.pending = Some(scpi);
        Ok(device)
    }

    fn link(&self) -> std::sync::MutexGuard<'_, Link> {
        self.link.lock().expect("link lock")
    }
}

fn number(key: &Key, value: &Value) -> Result<f64> {
    value.as_f64().ok_or_else(|| Error::Arg(format!("{} takes a number", key.id())))
}

impl Device for SmeDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn classes(&self) -> &[DeviceClass] {
        &CLASSES
    }

    fn channels(&self) -> &[Channel] {
        &[]
    }

    fn channel_groups(&self) -> &[ChannelGroup] {
        &[]
    }

    fn enable_channel(&mut self, pos: usize, _enabled: bool) -> Result<()> {
        Err(Error::Arg(format!("No channel {pos}.")))
    }

    fn rename_channel(&mut self, pos: usize, _name: &str) -> Result<()> {
        Err(Error::Arg(format!("No channel {pos}.")))
    }

    fn options(&self, _group: Option<&str>) -> Result<Vec<OptionDesc>> {
        Ok(DEV_OPTIONS.to_vec())
    }

    fn get(&self, key: &Key, _group: Option<&str>) -> Result<Value> {
        let mut link = self.link();
        match key {
            Key::OutputFrequency => link.get_frequency().map(Value::F64),
            Key::Amplitude => link.get_level().map(Value::F64),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    fn set(&mut self, key: &Key, value: &Value, _group: Option<&str>) -> Result<()> {
        let mut link = self.link();
        match key {
            Key::OutputFrequency => link.set_frequency(number(key, value)?),
            Key::Amplitude => link.set_level(number(key, value)?),
            other => Err(Error::UnknownKey(other.id().to_owned())),
        }
    }

    /// `STD_CONFIG_LIST`, which knows only the option lists, though `devopts`
    /// promises these keys a list.
    fn list(&self, key: &Key, _group: Option<&str>) -> Result<ValueList> {
        Err(Error::UnknownKey(key.id().to_owned()))
    }

    /// `sr_scpi_open`: the port and nothing on the wire.
    fn open(&mut self) -> Result<()> {
        let mut link = self.link.lock().expect("link lock");
        if link.scpi.is_some() {
            return Ok(());
        }
        let scpi = match self.pending.take() {
            Some(scpi) => scpi,
            None => scpi::open_keeping_line(
                &Resource::parse(&self.resource)?,
                self.serialcomm.as_deref(),
            )?,
        };
        link.attach(scpi);
        Ok(())
    }

    /// `sr_scpi_close`: the port and nothing on the wire.
    fn close(&mut self) -> Result<()> {
        self.link().scpi = None;
        Ok(())
    }

    /// `std_dummy_dev_acquisition_start`: nothing to acquire, so what a client
    /// waits for comes at once. On a thread of its own, as a queue of one would
    /// hold up the caller at the second packet. It owns nothing and needs no join.
    fn start(&mut self, sink: PacketSink) -> Result<()> {
        std::thread::spawn(move || {
            sink.send(Packet::header());
            sink.send(Packet::End);
        });
        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::protocol::tests::Fake;
    use super::*;

    const IDN: &[u8] = b"Rohde&Schwarz,SME02,12345,1.2\n";

    fn device(replies: &[Option<&[u8]>]) -> (SmeDevice, Fake) {
        let mut all: Vec<Option<&[u8]>> = vec![Some(IDN)];
        all.extend_from_slice(replies);
        let fake = Fake::with(&all);
        let mut device = SmeDevice::from_scpi(fake.scpi()).unwrap();
        device.open().unwrap();
        (device, fake)
    }

    #[test]
    fn the_scan_sends_its_two_lines() {
        let (device, fake) = device(&[]);
        assert_eq!(fake.written(), "\n*IDN?\n");
        let info = device.info();
        assert_eq!(info.vendor, "Rohde&Schwarz");
        assert_eq!(info.model, "SME02");
        assert_eq!(info.serial, "12345");
        assert_eq!(info.version, "1.2");
        assert_eq!(info.conn, None);
    }

    #[test]
    fn another_device_is_none() {
        for idn in [&b"ACME,SME02,1,1\n"[..], b"Rohde&Schwarz,SMT02,1,1\n", b"a,b\n"] {
            let fake = Fake::with(&[Some(idn)]);
            assert!(probe(fake.scpi(), None).is_none());
            assert_eq!(fake.written(), "\n*IDN?\n");
        }
        let fake = Fake::with(&[Some(IDN)]);
        let found = probe(fake.scpi(), Some("9600/8n1")).unwrap();
        assert_eq!(found.serialcomm.as_deref(), Some("9600/8n1"));
        assert_eq!(found.resource, "fake");
    }

    /// The remote line's failure says nothing: the query after it does.
    #[test]
    fn a_failed_remote_line_is_no_failed_probe() {
        let fake = Fake::with(&[Some(IDN)]);
        fake.line().failing_writes = 1;
        assert!(probe(fake.scpi(), None).is_some());
        assert_eq!(fake.written(), "*IDN?\n");
    }

    #[test]
    fn gets_and_sets() {
        let (mut device, fake) = device(&[Some(b"100000000\n"), Some(b"-30\n")]);
        fake.line().written.clear();
        assert_eq!(device.get(&Key::OutputFrequency, None).unwrap(), Value::F64(1e8));
        assert_eq!(device.get(&Key::Amplitude, None).unwrap(), Value::F64(-30.0));
        device.set(&Key::OutputFrequency, &Value::F64(1e6), None).unwrap();
        device.set(&Key::Amplitude, &Value::F64(-10.5), None).unwrap();
        assert_eq!(fake.written(), "FREQ?\nPOW?\nFREQ 1000000.0Hz\nPOW -10.5dBm\n");
    }

    #[test]
    fn what_a_device_refuses() {
        let (mut device, fake) = device(&[]);
        fake.line().written.clear();
        // The SME02's top.
        assert!(device.set(&Key::OutputFrequency, &Value::F64(1_500_000_000.5), None).is_err());
        assert!(device.set(&Key::OutputFrequency, &Value::F64(f64::NAN), None).is_err());
        assert!(device.set(&Key::OutputFrequency, &Value::Str("x".to_owned()), None).is_err());
        assert!(device.set(&Key::Conn, &Value::F64(1.0), None).is_err());
        assert!(device.get(&Key::Conn, None).is_err());
        assert!(device.list(&Key::OutputFrequency, None).is_err());
        assert_eq!(fake.written(), "");
    }

    #[test]
    fn a_failed_get_is_an_error_not_a_value() {
        let (device, _fake) = device(&[Some(b"abc\n")]);
        assert!(device.get(&Key::OutputFrequency, None).is_err());
    }

    #[test]
    fn the_options_and_a_list() {
        let (device, _fake) = device(&[]);
        let keys: Vec<Key> = device.options(None).unwrap().iter().map(|o| o.key.clone()).collect();
        assert_eq!(keys, [Key::OutputFrequency, Key::Amplitude]);
        assert_eq!(SmeDriver.scan_options(), [Key::Conn, Key::Serialcomm]);
        assert_eq!(device.channels().len(), 0);
    }

    #[test]
    fn close_lets_go_and_get_fails() {
        let (mut device, _fake) = device(&[]);
        device.close().unwrap();
        assert!(device.get(&Key::Amplitude, None).is_err());
        assert!(device.link().scpi.is_none());
    }

    #[test]
    fn scan_of_what_cannot_open_is_nothing() {
        let conn = |c: &str| vec![(Key::Conn, Value::Str(c.to_owned()))];
        for c in ["tcp-raw/host", "", "vxi/host", "/dev/oxrok-no-such-node"] {
            assert!(SmeDriver.scan(&conn(c)).unwrap().is_empty(), "{c}");
        }
        let mut options = conn("/dev/oxrok-no-such-node");
        options.push((Key::Serialcomm, Value::Str("fast".to_owned())));
        assert!(SmeDriver.scan(&options).unwrap().is_empty());
        assert!(SmeDriver.scan(&[(Key::Samplerate, Value::U64(1))]).is_err());
    }

    #[test]
    fn a_client_that_starts_gets_a_header_and_an_end() {
        let (mut device, _fake) = device(&[]);
        // A queue of one: the start must not wait for it to be read. On a thread,
        // so that one that does fails here.
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let (started, returned) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            device.start(PacketSink::new(tx, 0)).unwrap();
            device.stop().unwrap();
            started.send(()).unwrap();
        });
        returned.recv_timeout(std::time::Duration::from_secs(30)).unwrap();
        assert!(matches!(rx.recv().unwrap().1, Packet::Header { .. }));
        assert!(matches!(rx.recv().unwrap().1, Packet::End));
    }
}
