//! Replays of the sessions the emulator records: every
//! `tests/fixtures/DRIVER/emu-*.txt` was written by an `emu_case
//! --record` of `tools/diffcli.sh --emu` (docs/USB.md, "Record and
//! replay"), with the command line in its `# args:` comment and oxrok's
//! output next to it as `emu-*.out`. Each is run again here through the
//! library, the device built on a replay of the fixture by the table in
//! [`build`], and must give that output and consume the recording. So
//! the drivers' logic runs in `cargo test` on every platform, without
//! the emulator or root.

use oxrok::core::session::Flow;
use oxrok::core::util::parse_sizestring;
use oxrok::core::{Device, Driver, Key, Session, Value, resolve_key, trigger_syntax};
use oxrok::drivers::dcttech_usbrelay::{self, UsbRelayDevice, UsbRelayDriver};
use oxrok::drivers::fx2lafw::{Fx2lafwDevice, Fx2lafwDriver};
use oxrok::drivers::microchip_pickit2::{Pickit2Device, Pickit2Driver, UsbLink};
use oxrok::drivers::serial_dmm::{DmmDevice, SerialDmmDriver, models};
use oxrok::formats::output::{self, Options, OutputContext};
use oxrok::hid::HidDevice;
use oxrok::serial::hid::{Chip, HidPort};
use oxrok::serial::{Params, Port};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// The drivers with an emulator model: each must have a recording.
const RECORDED: [&str; 6] = [
    "fx2lafw",
    "microchip-pickit2",
    "dcttech-usbrelay",
    "brymen-bm86x",
    "uni-t-ut61e-ser",
    "victor-dmm",
];

/// A command line of the kind the recorded cases use.
#[derive(Default)]
struct Command {
    driver: String,
    scan: Vec<(String, String)>,
    configs: Vec<String>,
    channels: Option<String>,
    trigger: Option<String>,
    samples: Option<String>,
    output: Option<String>,
    group: Option<String>,
    get: Option<String>,
    set: bool,
}

impl Command {
    fn parse(args: &str) -> Command {
        let mut c = Command::default();
        let mut words = args.split_whitespace();
        while let Some(w) = words.next() {
            let mut value =
                || words.next().unwrap_or_else(|| panic!("{w} needs a value")).to_owned();
            match w {
                "-d" => {
                    let spec = value();
                    let mut parts = spec.split(':');
                    parts.next().unwrap().clone_into(&mut c.driver);
                    for p in parts {
                        let (k, v) = p.split_once('=').expect("scan option key=value");
                        c.scan.push((k.to_owned(), v.to_owned()));
                    }
                }
                "-c" => c.configs.push(value()),
                "-C" => c.channels = Some(value()),
                "-t" => c.trigger = Some(value()),
                "--samples" => c.samples = Some(value()),
                "-O" => c.output = Some(value()),
                "-g" => c.group = Some(value()),
                "--get" => c.get = Some(value()),
                "--set" => c.set = true,
                other => panic!("the replay test does not know the argument {other}"),
            }
        }
        c
    }

    fn scan_option(&self, key: &str) -> Option<&str> {
        self.scan.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
    }
}

/// A HID replay shared with the test, which checks at the end that it
/// was played through.
#[derive(Clone)]
struct SharedHid(Arc<Mutex<oxrok::hid::record::Replay>>);

impl HidDevice for SharedHid {
    fn read_report(&mut self, buf: &mut [u8], timeout: Duration) -> oxrok::core::Result<usize> {
        self.0.lock().unwrap().read_report(buf, timeout)
    }
    fn write_report(&mut self, data: &[u8]) -> oxrok::core::Result<()> {
        self.0.lock().unwrap().write_report(data)
    }
    fn get_feature(&mut self, buf: &mut [u8]) -> oxrok::core::Result<usize> {
        self.0.lock().unwrap().get_feature(buf)
    }
    fn set_feature(&mut self, data: &[u8]) -> oxrok::core::Result<()> {
        self.0.lock().unwrap().set_feature(data)
    }
}

type Finish = Box<dyn Fn() -> oxrok::core::Result<()>>;

/// The device of `cmd` on a replay of `fixture`, built as the driver's
/// scan builds it on the real transport, with the driver's long name
/// and a check that the replay was played to its end.
fn build(cmd: &Command, fixture: &str) -> (Box<dyn Device>, String, Finish) {
    match cmd.driver.as_str() {
        "fx2lafw" => {
            let replay = oxrok::usb::record::Replay::from_text(fixture).unwrap();
            let check = replay.clone();
            let mut dev = Fx2lafwDevice::from_transport(Box::new(replay)).unwrap();
            if let Some(spec) = cmd.scan_option("probe_names") {
                dev.name_channels(spec);
            }
            let finish: Finish = Box::new(move || check.finish());
            (Box::new(dev), Fx2lafwDriver.long_name().to_owned(), finish)
        }
        "microchip-pickit2" => {
            let replay = oxrok::usb::record::Replay::from_text(fixture).unwrap();
            let check = replay.clone();
            let link = UsbLink::claim(Box::new(replay)).unwrap();
            let mut dev = Pickit2Device::with_link(Some(Box::new(link)), "replay".to_owned());
            if let Some(spec) = cmd.scan_option("probe_names") {
                dev.name_channels(spec);
            }
            let finish: Finish = Box::new(move || check.finish());
            (Box::new(dev), Pickit2Driver.long_name().to_owned(), finish)
        }
        "dcttech-usbrelay" => {
            let replay = oxrok::hid::record::Replay::from_text(fixture).unwrap();
            let line = replay.device().clone();
            let shared = SharedHid(Arc::new(Mutex::new(replay)));
            let mut hid = shared.clone();
            let (serial, state) = dcttech_usbrelay::read_state(&mut hid).unwrap();
            let product = line.product.unwrap_or_default();
            let count = dcttech_usbrelay::relay_count(&product).expect("a relay count");
            let dev = UsbRelayDevice::new(
                Box::new(hid),
                line.manufacturer.unwrap_or_default(),
                product,
                serial,
                "replay".to_owned(),
                count,
                state,
            );
            let finish: Finish = Box::new(move || shared.0.lock().unwrap().finish());
            (Box::new(dev), UsbRelayDriver.long_name().to_owned(), finish)
        }
        id => {
            // A meter behind a serial over HID cable: the cable's chip
            // from the recorded USB id, its line set as the open does.
            let model = models::find(id).unwrap_or_else(|| panic!("no replay recipe for {id}"));
            let replay = oxrok::hid::record::Replay::from_text(fixture).unwrap();
            let line = replay.device().clone();
            let chip = Chip::for_usb_id(line.vendor_id, line.product_id).expect("a HID cable");
            let shared = SharedHid(Arc::new(Mutex::new(replay)));
            let mut port = HidPort::new(chip, Box::new(shared.clone()), "replay".to_owned());
            let serialcomm = cmd.scan_option("serialcomm").unwrap_or(model.serialcomm);
            port.set_params(Params::parse(serialcomm).unwrap()).unwrap();
            let dev = DmmDevice::from_port(model, Box::new(port));
            let finish: Finish = Box::new(move || shared.0.lock().unwrap().finish());
            (Box::new(dev), SerialDmmDriver(model).long_name().to_owned(), finish)
        }
    }
}

/// `-c` settings, in the group of `-g`.
fn apply_configs(dev: &mut dyn Device, cmd: &Command) {
    for config in &cmd.configs {
        for item in config.split(':') {
            let (k, v) = item.split_once('=').expect("key=value");
            let group = cmd.group.as_deref();
            let key = resolve_key(dev, k, group).unwrap();
            let value = Value::parse(key.kind(), v).unwrap();
            dev.set(&key, &value, group).unwrap();
        }
    }
}

/// Runs `cmd` on `dev` and returns what oxrok prints, as the harness
/// keeps it (its `strip_header` unless the case kept the header).
fn run(mut dev: Box<dyn Device>, title: &str, cmd: &Command, strip: bool) -> String {
    dev.open().unwrap();
    let mut text = String::new();
    if cmd.set {
        apply_configs(dev.as_mut(), cmd);
        dev.close().unwrap();
        return text;
    }
    if let Some(name) = &cmd.get {
        apply_configs(dev.as_mut(), cmd);
        let group = cmd.group.as_deref();
        let key = resolve_key(dev.as_ref(), name, group).unwrap();
        let value = dev.get(&key, group).unwrap();
        dev.close().unwrap();
        // As GLib prints these.
        return match value {
            Value::Bool(b) => format!("{b}\n"),
            Value::U64(n) => format!("{n}\n"),
            other => panic!("the replay test does not print {other:?}"),
        };
    }
    apply_configs(dev.as_mut(), cmd);
    if let Some(spec) = &cmd.channels {
        let names: Vec<&str> = spec.split(',').collect();
        for (pos, ch) in dev.channels().to_vec().iter().enumerate() {
            dev.enable_channel(pos, names.contains(&ch.name.as_str())).unwrap();
        }
    }
    let trigger = cmd.trigger.as_ref().map(|t| trigger_syntax::parse(dev.as_ref(), t).unwrap());
    if let Some(s) = &cmd.samples {
        dev.set(&Key::LimitSamples, &Value::U64(parse_sizestring(s).unwrap()), None).unwrap();
    }
    // The output as the CLI's pipeline builds it: the module of -O (bits
    // by default), analog packets it leaves go to the analog module.
    let spec = cmd.output.clone().unwrap_or_else(|| "bits:width=64".to_owned());
    let mut parts = spec.split(':');
    let module = output::find(parts.next().unwrap()).unwrap();
    let options: Options = parts
        .map(|p| {
            let (k, v) = p.split_once('=').unwrap_or((p, ""));
            (k.to_owned(), v.to_owned())
        })
        .collect();
    let channels = dev.channels().to_vec();
    let ctx = OutputContext {
        channels: &channels,
        samplerate: dev.get(&Key::Samplerate, None).ok().and_then(|v| v.as_u64()),
        title: title.to_owned(),
        filename: None,
    };
    let mut main = module.instantiate(&ctx, &options).unwrap();
    let mut analog = (!module.analog).then(|| {
        let opts: Options = [("meta".to_owned(), "false".to_owned())].into();
        output::find("analog").unwrap().instantiate(&ctx, &opts).unwrap()
    });
    let mut session = Session::new();
    session.add_device(dev);
    session.set_trigger(trigger);
    let mut out = Vec::new();
    session
        .run(|_, packet| {
            let mut buf = Vec::new();
            main.receive(packet, &mut buf).unwrap();
            if buf.is_empty()
                && let Some(a) = &mut analog
            {
                a.receive(packet, &mut buf).unwrap();
            }
            out.extend(buf);
            Flow::Continue
        })
        .unwrap();
    for mut d in session.into_devices() {
        d.close().unwrap();
    }
    text.push_str(&String::from_utf8(out).unwrap());
    if !strip {
        return text;
    }
    let mut lines: Vec<&str> =
        text.lines().filter(|l| !l.starts_with("META ") && !l.starts_with("FRAME-")).collect();
    if lines.first().is_some_and(|l| l.starts_with("oxrok ")) {
        lines.remove(0);
    }
    lines.iter().flat_map(|l| [l, "\n"]).collect()
}

fn fixtures() -> Vec<PathBuf> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests").join("fixtures");
    let mut out = Vec::new();
    for dir in std::fs::read_dir(&root).unwrap() {
        let dir = dir.unwrap().path();
        if !dir.is_dir() {
            continue;
        }
        for f in std::fs::read_dir(&dir).unwrap() {
            let f = f.unwrap().path();
            let name = f.file_name().unwrap().to_string_lossy().into_owned();
            if name.starts_with("emu-") && f.extension().is_some_and(|e| e == "txt") {
                out.push(f);
            }
        }
    }
    out.sort();
    out
}

/// One fixture: its output, or why it failed.
fn replay(path: &Path) -> Result<(), String> {
    let fixture = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    // Git may check text out with CRLF on Windows.
    let fixture = fixture.replace("\r\n", "\n");
    let comment = |key: &str| {
        fixture.lines().find_map(|l| l.strip_prefix(key)).map(str::trim).map(str::to_owned)
    };
    let args = comment("# args:").ok_or("no '# args:' line")?;
    let strip = comment("# header:").as_deref() != Some("kept");
    let expected = std::fs::read_to_string(path.with_extension("out"))
        .map_err(|e| format!("{}: {e}", path.with_extension("out").display()))?
        .replace("\r\n", "\n");
    let cmd = Command::parse(&args);
    let (dev, title, finish) = build(&cmd, &fixture);
    let got = run(dev, &title, &cmd, strip);
    if got != expected {
        return Err(format!("output differs:\n--- recorded\n{expected}--- replayed\n{got}"));
    }
    finish().map_err(|e| format!("not played to the end: {e}"))
}

#[test]
fn emulator_recordings_replay() {
    let paths = fixtures();
    let mut failures = Vec::new();
    let mut drivers = Vec::new();
    for path in &paths {
        let text = std::fs::read_to_string(path).unwrap();
        if let Some(args) = text.lines().find_map(|l| l.strip_prefix("# args:")) {
            drivers.push(Command::parse(args).driver);
        }
        // Each in a thread of its own: a panic (an unwrap on what the
        // replay refused) is one fixture's failure, and a driver that
        // waits for ever on a replay that went astray (the PICkit2
        // waits for its trigger as long as it is not stopped) fails
        // after a while rather than holding the test.
        let (tx, rx) = std::sync::mpsc::channel();
        let p = path.clone();
        std::thread::spawn(move || {
            let _ = tx.send(replay(&p));
        });
        match rx.recv_timeout(Duration::from_secs(60)) {
            Ok(Ok(())) => {}
            Ok(Err(e)) => failures.push(format!("{}: {e}", path.display())),
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                failures.push(format!("{}: panicked (see above)", path.display()));
            }
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                failures.push(format!("{}: still running after 60 s", path.display()));
            }
        }
    }
    for d in RECORDED {
        if !drivers.iter().any(|x| x == d) {
            failures.push(format!("no emulator recording of {d}"));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n\n"));
}
