// Counterpart in sigrok:
//   libsigrok/src/hardware/baylibre-acme/api.c,
//   libsigrok/src/hardware/baylibre-acme/protocol.c,
//   Copyright (C) 2015 Bartosz Golaszewski <bgolaszewski@baylibre.com>,
//   with commits by Uwe Hermann, Lars-Peter Clausen, Daniel Elstner,
//   Daniel Lezcano, Martin Ling, Soeren Apel, Aurelien Jacobs, Gerhard Sittig
//   (libsigrok libsigrok-unreleased-1848-g0bc24877).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicU32;
use std::sync::mpsc::{Receiver, sync_channel};

use super::protocol::{
    Eeprom, ReadError, c_strtol, hwmon_number, parse_value, rev_b_addr_to_num, scale, under,
};
use super::*;
use crate::core::{Mq, Unit};

/// How long a test waits for what must happen: a stop or a limit that does not
/// bite must fail, not hang the run.
const BOUND: Duration = Duration::from_secs(30);

/// A tree of its own under the temporary directory, gone when dropped.
struct Tree(PathBuf);

impl Tree {
    fn new() -> Tree {
        static N: AtomicU32 = AtomicU32::new(0);
        let p = std::env::temp_dir().join(format!(
            "oxrok-baylibre-acme-{}-{}",
            std::process::id(),
            N.fetch_add(1, Ordering::SeqCst)
        ));
        let _ = fs::remove_dir_all(&p);
        fs::create_dir_all(p.join("sys/class")).unwrap();
        Tree(p)
    }

    fn put(&self, path: &str, data: impl AsRef<[u8]>) {
        let p = under(&self.0, path);
        fs::create_dir_all(p.parent().unwrap()).unwrap();
        fs::write(p, data).unwrap();
    }

    fn get(&self, path: &str) -> String {
        fs::read_to_string(under(&self.0, path)).unwrap_or_default()
    }

    fn dir(&self, path: &str) {
        fs::create_dir_all(under(&self.0, path)).unwrap();
    }

    /// A chip at `addr` and, when it answered, its hwmon device.
    fn chip(&self, addr: u8, name: &str, hwmon: Option<u32>) {
        let dev = format!("/sys/class/i2c-adapter/i2c-1/1-00{addr:02x}");
        self.put(&format!("{dev}/name"), format!("{name}\n"));
        if let Some(n) = hwmon {
            self.dir(&format!("{dev}/hwmon/hwmon{n}"));
            let h = format!("/sys/class/hwmon/hwmon{n}");
            self.put(&format!("{h}/name"), format!("{name}\n"));
            self.put(&format!("{h}/update_interval"), "35\n");
            if name == "ina226" {
                self.put(&format!("{h}/power1_input"), "1925000\n");
                self.put(&format!("{h}/curr1_input"), "388\n");
                self.put(&format!("{h}/in1_input"), "4975\n");
                self.put(&format!("{h}/shunt_resistor"), "10000\n");
            } else {
                self.put(&format!("{h}/temp1_input"), "25000\n");
                self.put(&format!("{h}/temp2_input"), "40063\n");
            }
        }
    }

    fn gpio(&self, gpio: u32, direction: &str, value: u8) {
        self.put(&format!("/sys/class/gpio/gpio{gpio}/direction"), format!("{direction}\n"));
        self.put(&format!("/sys/class/gpio/gpio{gpio}/value"), format!("{value}\n"));
    }

    /// The default cape of the emulator's model: a revision A power probe with a
    /// switch on 1, a revision B one with a 20 mΩ shunt on 2, a tmp435 on 5.
    fn cape() -> Tree {
        let t = Tree::new();
        t.put("/sys/class/gpio/export", "");
        t.chip(0x40, "ina226", Some(0));
        t.chip(0x41, "ina226", Some(1));
        t.chip(0x4c, "tmp435", Some(2));
        t.chip(0x44, "ina226", None);
        t.gpio(487, "in", 1);
        t.gpio(486, "out", 1);
        t.gpio(491, "out", 1);
        t.gpio(479, "in", 0);
        t.put("/sys/class/i2c-adapter/i2c-1/1-0051/eeprom", eeprom(1, b'B', 20_000, 1));
        std::os::unix::fs::symlink(
            "../../i2c-adapter/i2c-1",
            under(&t.0, "/sys/class/i2c-dev/i2c-1/device"),
        )
        .or_else(|_| {
            fs::create_dir_all(under(&t.0, "/sys/class/i2c-dev/i2c-1"))?;
            std::os::unix::fs::symlink(
                "../../i2c-adapter/i2c-1",
                under(&t.0, "/sys/class/i2c-dev/i2c-1/device"),
            )
        })
        .unwrap();
        t
    }

    fn scan(&self) -> Vec<Box<dyn Device>> {
        BaylibreAcmeDriver::with_root(&self.0).scan(&[]).unwrap()
    }
}

impl Drop for Tree {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

fn eeprom(kind: u32, rev: u8, shunt: u32, pws: u8) -> Vec<u8> {
    let mut e = vec![0xffu8; 256];
    e[0..4].copy_from_slice(&kind.to_be_bytes());
    e[4..8].copy_from_slice(&u32::from(rev).to_be_bytes());
    e[8..12].copy_from_slice(&shunt.to_be_bytes());
    e[12] = pws;
    e
}

fn names(d: &dyn Device) -> Vec<String> {
    d.channels().iter().map(|c| c.name.clone()).collect()
}

/// A capture's packets until its End, with a bound.
fn capture(d: &mut dyn Device) -> Vec<Packet> {
    let (tx, rx) = sync_channel(4096);
    d.start(PacketSink::new(tx, 0)).unwrap();
    collect(&rx)
}

fn collect(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let mut out = Vec::new();
    loop {
        let (_, p) = rx.recv_timeout(BOUND).expect("the capture ends");
        let end = matches!(p, Packet::End);
        out.push(p);
        if end {
            return out;
        }
    }
}

fn analog(packets: &[Packet]) -> Vec<&AnalogData> {
    packets
        .iter()
        .filter_map(|p| match p {
            Packet::Analog(a) => Some(a),
            _ => None,
        })
        .collect()
}

fn failed(packets: &[Packet]) -> Option<&str> {
    packets.iter().find_map(|p| match p {
        Packet::Failed(why) => Some(why.as_str()),
        _ => None,
    })
}

#[test]
fn values_are_the_kernels_numbers_and_nothing_else() {
    assert_eq!(parse_value(b"4975\n"), Some(4975));
    assert_eq!(parse_value(b"-12\n"), Some(-12));
    assert_eq!(parse_value(b"0"), Some(0));
    for junk in [
        &b""[..],
        b"\n",
        b"junk\n",
        b"5000mV\n",
        b" 5\n",
        b"+5\n",
        b"-\n",
        b"5\n\n",
        b"99999999999999999999\n",
    ] {
        assert_eq!(parse_value(junk), None, "{junk:?}");
    }
    // What C's strtol makes of them.
    assert_eq!(c_strtol(b"junk\n"), 0);
    assert_eq!(c_strtol(b"5000mV\n"), 5000);
    assert_eq!(c_strtol(b"  -7x"), -7);
    assert_eq!(c_strtol(b"99999999999999999999"), i64::MAX);
}

/// The units, pinned: the files' µW, mA, mV and m°C become W, A, V and °C, as
/// C's `type_digits` scales them, in single precision.
#[test]
fn units_scale_as_c_does() {
    assert_eq!(ChannelType::EnrgPwr.digits(), 6);
    assert_eq!(ChannelType::EnrgCurr.digits(), 3);
    assert_eq!(ChannelType::EnrgVol.digits(), 3);
    assert_eq!(ChannelType::TempIn.digits(), 3);
    assert_eq!(ChannelType::TempOut.digits(), 3);
    assert_eq!((ChannelType::EnrgPwr.mq(), ChannelType::EnrgPwr.unit()), (Mq::Power, Unit::Watt));
    assert_eq!(
        (ChannelType::EnrgCurr.mq(), ChannelType::EnrgCurr.unit()),
        (Mq::Current, Unit::Ampere)
    );
    assert_eq!((ChannelType::EnrgVol.mq(), ChannelType::EnrgVol.unit()), (Mq::Voltage, Unit::Volt));
    assert_eq!(
        (ChannelType::TempOut.mq(), ChannelType::TempOut.unit()),
        (Mq::Temperature, Unit::Celsius)
    );
    assert_eq!(scale(1_925_000, 6), 1_925_000f32 * 10f32.powf(-6.0));
    assert!((scale(1_925_000, 6) - 1.925).abs() < 1e-6, "µW to W");
    assert!((scale(388, 3) - 0.388).abs() < 1e-6, "mA to A");
    assert!((scale(4975, 3) - 4.975).abs() < 1e-6, "mV to V");
    assert!((scale(40_063, 3) - 40.063).abs() < 1e-4, "m°C to °C");
    assert!((scale(-12, 3) + 0.012).abs() < 1e-6);
}

#[test]
fn hwmon_names_and_revision_b_numbers_read_as_c_does() {
    assert_eq!(hwmon_number("hwmon3"), Some(3));
    assert_eq!(hwmon_number("hwmon 12x"), Some(12));
    assert_eq!(hwmon_number("hwmon-1"), Some(-1));
    assert_eq!(hwmon_number("hwmonX"), None);
    assert_eq!(hwmon_number("device"), None);
    assert_eq!(hwmon_number("hwmon99999999999"), None);
    let nums: Vec<usize> =
        [0x40, 0x41, 0x44, 0x45, 0x42, 0x43, 0x46, 0x47].map(rev_b_addr_to_num).to_vec();
    assert_eq!(nums, [1, 2, 5, 6, 3, 4, 7, 8]);
    assert_eq!(rev_b_addr_to_num(0x4c), 13, "a temperature probe of revision B: past the tables");
}

#[test]
fn eeproms_are_checked_as_c_checks_them() {
    let e = eeprom(1, b'B', 20_000, 1);
    assert_eq!(
        Eeprom::parse(&e[..protocol::EEPROM_SIZE]),
        Some(Eeprom { kind: 1, rev: u32::from(b'B'), shunt: 20_000, pwr_sw: 1 })
    );
    assert!(Eeprom::parse(&e[..protocol::EEPROM_SIZE - 1]).is_none(), "short");
    assert!(Eeprom::parse(&eeprom(1, b'A', 0, 0)[..protocol::EEPROM_SIZE]).is_none(), "revision A");
    assert!(Eeprom::parse(&eeprom(4, b'B', 0, 0)[..protocol::EEPROM_SIZE]).is_none(), "type 4");
    for kind in [1, 2, 3] {
        assert!(Eeprom::parse(&eeprom(kind, b'B', 0, 0)[..protocol::EEPROM_SIZE]).is_some());
    }
}

#[test]
fn a_scan_finds_the_probes_and_sets_a_revision_b_shunt() {
    let t = Tree::cape();
    let devices = t.scan();
    assert_eq!(devices.len(), 1);
    let d = &devices[0];
    assert_eq!(
        names(d.as_ref()),
        [
            "P1_ENRG_PWR",
            "P1_ENRG_CURR",
            "P1_ENRG_VOL",
            "P2_ENRG_PWR",
            "P2_ENRG_CURR",
            "P2_ENRG_VOL",
            "P5_TEMP_IN",
            "P5_TEMP_OUT"
        ]
    );
    let groups: Vec<&str> = d.channel_groups().iter().map(|g| g.name.as_str()).collect();
    assert_eq!(groups, ["Probe_1", "Probe_2", "Probe_5"]);
    assert_eq!(t.get("/sys/class/hwmon/hwmon1/shunt_resistor"), "20000\n");
    assert_eq!(t.get("/sys/class/hwmon/hwmon0/shunt_resistor"), "10000\n");
    let keys = |g| d.options(Some(g)).unwrap().iter().map(|o| o.key.clone()).collect::<Vec<_>>();
    assert_eq!(keys("Probe_1"), [Key::ProbeFactor, Key::PowerOff]);
    assert_eq!(keys("Probe_2"), [Key::ProbeFactor, Key::PowerOff]);
    assert_eq!(keys("Probe_5"), Vec::<Key>::new());
    assert_eq!(d.get(&Key::ProbeFactor, Some("Probe_2")).unwrap(), Value::U64(20));
    assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(10));
    assert!(matches!(
        d.list(&Key::Samplerate, None).unwrap(),
        ValueList::RangeStepU64 { min: 1, max: 500, step: 1 }
    ));
}

#[test]
fn no_probe_is_no_device_and_no_sysfs_is_none_either() {
    let t = Tree::new();
    t.chip(0x40, "ina226", None);
    t.chip(0x4c, "tmp435", None);
    assert!(t.scan().is_empty());
    let none = BaylibreAcmeDriver::with_root(t.0.join("nowhere"));
    assert!(none.scan(&[]).unwrap().is_empty());
}

#[test]
fn a_hwmon_entry_that_names_nothing_loses_its_probe_and_no_crash() {
    let t = Tree::cape();
    fs::remove_dir_all(under(&t.0, "/sys/class/i2c-adapter/i2c-1/1-0040/hwmon/hwmon0")).unwrap();
    // Empty: C passes NULL to sscanf.
    let d = &t.scan()[0];
    assert_eq!(d.channel_groups().len(), 2);
    t.dir("/sys/class/i2c-adapter/i2c-1/1-0040/hwmon/hwmonX");
    assert_eq!(t.scan()[0].channel_groups().len(), 2);
}

#[test]
fn shunts_are_mohm_written_in_uohm_and_junk_is_no_value() {
    let t = Tree::cape();
    let mut d = t.scan().remove(0);
    d.set(&Key::ProbeFactor, &Value::U64(50), Some("Probe_1")).unwrap();
    assert_eq!(t.get("/sys/class/hwmon/hwmon0/shunt_resistor"), "50000\n");
    assert_eq!(d.get(&Key::ProbeFactor, Some("Probe_1")).unwrap(), Value::U64(50));
    // C's uint64_t product wraps.
    d.set(&Key::ProbeFactor, &Value::U64(u64::MAX), Some("Probe_1")).unwrap();
    assert_eq!(
        t.get("/sys/class/hwmon/hwmon0/shunt_resistor"),
        format!("{}\n", u64::MAX.wrapping_mul(1000))
    );
    t.put("/sys/class/hwmon/hwmon0/shunt_resistor", "999\n");
    assert_eq!(d.get(&Key::ProbeFactor, Some("Probe_1")).unwrap(), Value::U64(0));
    t.put("/sys/class/hwmon/hwmon0/shunt_resistor", "abc\n");
    assert!(d.get(&Key::ProbeFactor, Some("Probe_1")).is_err());
    let e = d.get(&Key::ProbeFactor, Some("Probe_5")).unwrap_err();
    assert!(e.to_string().contains("shunt"), "{e}");
    assert!(d.get(&Key::ProbeFactor, None).is_err());
    fs::remove_file(under(&t.0, "/sys/class/hwmon/hwmon0/shunt_resistor")).unwrap();
    assert_eq!(
        d.get(&Key::ProbeFactor, Some("Probe_1")).unwrap_err().to_string(),
        "not applicable"
    );
    assert_eq!(
        d.set(&Key::ProbeFactor, &Value::U64(5), Some("Probe_1")).unwrap_err().to_string(),
        "not applicable"
    );
}

#[test]
fn a_shunt_write_that_fails_is_an_error() {
    let t = Tree::cape();
    let mut d = t.scan().remove(0);
    let p = under(&t.0, "/sys/class/hwmon/hwmon0/shunt_resistor");
    fs::remove_file(&p).unwrap();
    std::os::unix::fs::symlink("/dev/full", &p).unwrap();
    let e = d.set(&Key::ProbeFactor, &Value::U64(50), Some("Probe_1")).unwrap_err();
    assert_eq!(e.to_string(), "input/output error");
}

#[test]
fn samplerates_go_to_update_interval_and_0_and_501_are_refused() {
    let t = Tree::cape();
    let mut d = t.scan().remove(0);
    d.set(&Key::Samplerate, &Value::U64(50), None).unwrap();
    assert_eq!(t.get("/sys/class/hwmon/hwmon0/update_interval"), "20\n");
    assert_eq!(t.get("/sys/class/hwmon/hwmon2/update_interval"), "20\n");
    assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(50));
    assert_eq!(
        d.set(&Key::Samplerate, &Value::U64(501), None).unwrap_err().to_string(),
        "invalid samplerate"
    );
    assert!(d.set(&Key::Samplerate, &Value::U64(0), None).is_err());
    assert_eq!(d.get(&Key::Samplerate, None).unwrap(), Value::U64(50));
    // A probe without the attribute is passed over.
    fs::remove_file(under(&t.0, "/sys/class/hwmon/hwmon2/update_interval")).unwrap();
    d.set(&Key::Samplerate, &Value::U64(3), None).unwrap();
    assert_eq!(t.get("/sys/class/hwmon/hwmon0/update_interval"), "333\n");
}

#[test]
fn power_switches_read_and_write_their_value() {
    let t = Tree::cape();
    let mut d = t.scan().remove(0);
    assert_eq!(d.get(&Key::PowerOff, Some("Probe_1")).unwrap(), Value::Bool(false));
    d.set(&Key::PowerOff, &Value::Bool(true), Some("Probe_1")).unwrap();
    assert_eq!(t.get("/sys/class/gpio/gpio486/value"), "0\n");
    assert_eq!(d.get(&Key::PowerOff, Some("Probe_1")).unwrap(), Value::Bool(true));
    d.set(&Key::PowerOff, &Value::Bool(false), Some("Probe_2")).unwrap();
    assert_eq!(t.get("/sys/class/gpio/gpio491/value"), "1\n");
    assert!(d.get(&Key::PowerOff, Some("Probe_5")).is_err(), "no switch");
    assert!(d.get(&Key::PowerOff, None).is_err());
    assert_eq!(t.get("/sys/class/gpio/export"), "", "nothing exported");
    t.put("/sys/class/gpio/gpio486/value", "junk\n");
    assert!(d.get(&Key::PowerOff, Some("Probe_1")).is_err(), "C takes -1 for on");
}

/// A kernel's export, on a thread: once `export` holds a number, its `gpioN`.
fn fake_export(root: PathBuf, gpio: u32) -> std::thread::JoinHandle<bool> {
    std::thread::spawn(move || {
        let export = under(&root, "/sys/class/gpio/export");
        let began = Instant::now();
        while began.elapsed() < BOUND {
            if fs::read_to_string(&export).is_ok_and(|s| s == format!("{gpio}\n")) {
                let d = under(&root, &format!("/sys/class/gpio/gpio{gpio}"));
                fs::create_dir_all(&d).unwrap();
                fs::write(d.join("direction"), "in\n").unwrap();
                fs::write(d.join("value"), "1\n").unwrap();
                return true;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        false
    })
}

#[test]
fn a_gpio_not_exported_is_exported_first_and_a_switch_set_with_its_direction() {
    let t = Tree::cape();
    let mut d = t.scan().remove(0);
    fs::remove_dir_all(under(&t.0, "/sys/class/gpio/gpio486")).unwrap();
    let kernel = fake_export(t.0.clone(), 486);
    d.set(&Key::PowerOff, &Value::Bool(true), Some("Probe_1")).unwrap();
    assert!(kernel.join().unwrap());
    assert_eq!(t.get("/sys/class/gpio/gpio486/direction"), "low\n", "off, with no glitch on");
    // A sense line read is exported and left an input.
    fs::remove_dir_all(under(&t.0, "/sys/class/gpio/gpio487")).unwrap();
    let kernel = fake_export(t.0.clone(), 487);
    let d = t.scan().remove(0);
    assert!(kernel.join().unwrap());
    assert_eq!(t.get("/sys/class/gpio/gpio487/direction"), "in\n");
    assert_eq!(d.options(Some("Probe_1")).unwrap().len(), 2);
}

#[test]
fn a_sense_line_that_cannot_be_read_is_no_switch() {
    let t = Tree::cape();
    t.put("/sys/class/gpio/gpio487/value", "junk\n");
    let d = &t.scan()[0];
    assert_eq!(
        d.options(Some("Probe_1")).unwrap(),
        [OptionDesc::new(Key::ProbeFactor, Access::GET_SET)]
    );
}

#[test]
fn a_capture_reads_each_enabled_channel_once_a_tick() {
    let t = Tree::cape();
    let mut d = t.scan().remove(0);
    d.set(&Key::Samplerate, &Value::U64(500), None).unwrap();
    d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
    d.enable_channel(1, false).unwrap();
    let packets = capture(d.as_mut());
    assert!(matches!(packets[0], Packet::Header { .. }), "{:?}", packets[0]);
    let frames = packets.iter().filter(|p| matches!(p, Packet::FrameBegin)).count();
    assert_eq!(frames, 3);
    let a = analog(&packets);
    assert_eq!(a.len(), 3 * 7);
    let first: Vec<(usize, f32, i32)> =
        a[..7].iter().map(|a| (a.channels[0], a.samples[0], a.digits)).collect();
    assert_eq!(first[0], (0, scale(1_925_000, 6), 6));
    assert_eq!(first[1], (2, scale(4975, 3), 3), "channel 1 disabled");
    assert_eq!(first[6], (7, scale(40_063, 3), 3));
    assert!(failed(&packets).is_none());
}

#[test]
fn junk_or_nothing_in_a_value_file_drops_its_channel_and_none_left_fails() {
    for (what, data) in [("junk", &b"junk\n"[..]), ("empty", b""), ("number and junk", b"5000mV\n")]
    {
        let t = Tree::cape();
        let mut d = t.scan().remove(0);
        for ch in 0..8 {
            d.enable_channel(ch, ch == 2).unwrap();
        }
        d.set(&Key::Samplerate, &Value::U64(500), None).unwrap();
        d.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
        t.put("/sys/class/hwmon/hwmon0/in1_input", data);
        let packets = capture(d.as_mut());
        assert!(analog(&packets).is_empty(), "{what}");
        assert_eq!(
            failed(&packets),
            Some("device error: No channel can be read any more."),
            "{what}"
        );
    }
}

#[test]
fn a_read_error_drops_its_channel_and_the_rest_go_on() {
    let t = Tree::cape();
    let mut d = t.scan().remove(0);
    for ch in 0..8 {
        d.enable_channel(ch, ch == 1 || ch == 2).unwrap();
    }
    d.set(&Key::Samplerate, &Value::U64(500), None).unwrap();
    d.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
    let p = under(&t.0, "/sys/class/hwmon/hwmon0/in1_input");
    fs::remove_file(&p).unwrap();
    fs::create_dir(&p).unwrap();
    let packets = capture(d.as_mut());
    let a = analog(&packets);
    assert_eq!(
        a.iter().map(|a| a.channels[0]).collect::<Vec<_>>(),
        [1, 1],
        "no -1 for the failed read"
    );
    assert!(failed(&packets).is_none());
}

#[test]
fn a_value_file_missing_is_no_capture() {
    let t = Tree::cape();
    let mut d = t.scan().remove(0);
    // Disabled or not: C opens them all.
    d.enable_channel(6, false).unwrap();
    fs::remove_file(under(&t.0, "/sys/class/hwmon/hwmon2/temp1_input")).unwrap();
    let (tx, _rx) = sync_channel(16);
    let e = d.start(PacketSink::new(tx, 0)).unwrap_err();
    assert!(e.to_string().contains("P5_TEMP_IN"), "{e}");
}

#[test]
fn a_value_file_read_says_why_it_gave_nothing() {
    let t = Tree::cape();
    let sysfs = Sysfs::new(&t.0);
    let mut f = sysfs.open_channel(0, ChannelType::EnrgVol).unwrap();
    assert_eq!(f.read().unwrap(), 4975);
    t.put("/sys/class/hwmon/hwmon0/in1_input", "-3\n");
    assert_eq!(f.read().unwrap(), -3, "read from the start again");
    t.put("/sys/class/hwmon/hwmon0/in1_input", "");
    assert!(matches!(f.read(), Err(ReadError::Empty)));
    t.put("/sys/class/hwmon/hwmon0/in1_input", "x\n");
    let e = f.read().unwrap_err();
    assert_eq!(e.to_string(), "not a number: \"x\\n\"");
}

#[test]
fn the_timer_ticks_a_period_after_the_start_and_counts_what_was_missed() {
    let start = Instant::now();
    let mut timer = protocol::Timer::new(start, 10);
    assert_eq!(timer.next(), start + Duration::from_millis(100));
    assert_eq!(timer.take(start + Duration::from_millis(99)), 0);
    assert_eq!(timer.take(start + Duration::from_millis(100)), 1);
    assert_eq!(timer.next(), start + Duration::from_millis(200));
    assert_eq!(timer.take(start + Duration::from_millis(450)), 3, "two missed");
    assert_eq!(timer.take(start + Duration::from_millis(450)), 0);
    assert_eq!(protocol::Timer::new(start, 1).next(), start + Duration::from_secs(1), "1 Hz works");
}

#[test]
fn limits_are_met_on_samples_or_time() {
    let l = Limits { samples: 3, msec: 0 };
    assert!(!l.met(2, Duration::from_secs(99)));
    assert!(l.met(3, Duration::ZERO));
    let l = Limits { samples: 0, msec: 250 };
    assert!(!l.met(99, Duration::from_millis(249)));
    assert!(l.met(0, Duration::from_millis(250)));
    assert!(!Limits { samples: 0, msec: 0 }.met(u64::MAX, Duration::MAX));
}

/// `--time 50` at 10 Hz: the first tick is at 100 ms, and C sends that sample
/// before it looks at the limit. So must this, rather than nothing.
#[test]
fn a_time_limit_shorter_than_a_period_still_gets_its_sample() {
    let t = Tree::cape();
    let mut d = t.scan().remove(0);
    d.set(&Key::LimitMsec, &Value::U64(50), None).unwrap();
    let began = Instant::now();
    let packets = capture(d.as_mut());
    assert_eq!(packets.iter().filter(|p| matches!(p, Packet::FrameBegin)).count(), 1);
    assert!(began.elapsed() >= Duration::from_millis(100));
}

#[test]
fn a_continuous_capture_stops_and_a_close_joins_it() {
    let t = Tree::cape();
    let mut d = t.scan().remove(0);
    d.set(&Key::Samplerate, &Value::U64(500), None).unwrap();
    let (tx, rx) = sync_channel(1);
    d.start(PacketSink::new(tx, 0)).unwrap();
    // A frame or two, then nobody reads: the worker blocks on a full queue.
    let _ = rx.recv_timeout(BOUND).unwrap();
    std::thread::sleep(Duration::from_millis(30));
    // On a thread with a bound: a stop ignored must fail the test, not hang it.
    let (done_tx, done_rx) = sync_channel(1);
    std::thread::spawn(move || {
        d.stop().unwrap();
        let began = Instant::now();
        d.close().unwrap();
        let _ = done_tx.send((d, began.elapsed()));
    });
    let (mut d, took) = done_rx.recv_timeout(BOUND).expect("the close joins");
    drop(rx);
    assert!(took < Duration::from_secs(5), "the close joined at once: {took:?}");
    // A second capture on the same device starts afresh.
    d.set(&Key::LimitSamples, &Value::U64(1), None).unwrap();
    let packets = capture(d.as_mut());
    assert_eq!(packets.iter().filter(|p| matches!(p, Packet::FrameBegin)).count(), 1);
}

#[test]
fn falling_behind_sends_the_last_values_again_and_the_limit_holds() {
    // A sink that a test holds up: three ticks are due at once.
    let t = Tree::cape();
    let mut d = t.scan().remove(0);
    for ch in 0..8 {
        d.enable_channel(ch, ch == 2).unwrap();
    }
    d.set(&Key::Samplerate, &Value::U64(100), None).unwrap();
    d.set(&Key::LimitSamples, &Value::U64(4), None).unwrap();
    let (tx, rx) = sync_channel(1);
    d.start(PacketSink::new(tx, 0)).unwrap();
    // The header waits in the queue; the first frame's FrameBegin blocks.
    std::thread::sleep(Duration::from_millis(80));
    let packets = collect(&rx);
    let frames = packets.iter().filter(|p| matches!(p, Packet::FrameBegin)).count();
    assert_eq!(frames, 4, "never past the limit");
    let values: Vec<f32> = analog(&packets).iter().map(|a| a.samples[0]).collect();
    assert_eq!(values.len(), 4);
    assert!(values.iter().all(|v| v.to_bits() == scale(4975, 3).to_bits()));
}

#[test]
fn the_driver_is_cs() {
    let d = BaylibreAcmeDriver::default();
    assert_eq!(d.id(), "baylibre-acme");
    assert_eq!(d.long_name(), "BayLibre ACME (Another Cute Measurement Equipment)");
    assert_eq!(d.classes(), [DeviceClass::Thermometer, DeviceClass::PowerMeter]);
    assert!(d.scan(&[(Key::Conn, Value::Str("x".into()))]).is_err());
    let _ = Path::new("/");
}
