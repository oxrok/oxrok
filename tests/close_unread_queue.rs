// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Every driver bounds what it sends once it was asked to stop: a stop or
//! a close joins the acquisition thread, and must not wait for ever on a
//! queue nobody reads. The demo device is closed that way here, as any
//! client of the library can do it; the other drivers need a transport,
//! and are held to it by their sources and their own unit tests.

use oxrok::core::{Device, Key, Packet, PacketSink, Registry, Value};
use std::path::Path;
use std::sync::mpsc::{channel, sync_channel};
use std::time::Duration;

/// The drivers that take a `PacketSink` and neither tell it of their stop
/// flag (`stopped_by`) nor bound a send themselves (`send_while`,
/// `end_while`), by the name of their module. A driver leaves this list
/// when it does one of the two, with a test of its own that closes it on
/// an unread queue; none joins it.
const NOT_BOUNDED: [&str; 39] = [
    "asix_sigma",
    "chronovu_la",
    "conrad_digi_35_cpu",
    "dcttech_usbrelay",
    "devantech_eth008",
    "dreamsourcelab_dslogic",
    "fluke_dmm",
    "ftdi_la",
    "fx2lafw",
    "greatfet",
    "gwinstek_gpd",
    "hantek_4032l",
    "hantek_6xxx",
    "hantek_dso",
    "hp_59306a",
    "icstation_usbrelay",
    "ikalogic_scanalogic2",
    "ikalogic_scanaplus",
    "juntek_jds6600",
    "kecheng_kc_330b",
    "kingst_la2016",
    "korad_kaxxxxp",
    "lascar_el_usb",
    "lecroy_logicstudio",
    "manson_hcs_3xxx",
    "microchip_pickit2",
    "ols",
    "pipistrello_ols",
    "raspberrypi_pico",
    "rdtech_tc",
    "rdtech_um",
    "rohde_schwarz_sme_0x",
    "saleae_logic16",
    "saleae_logic_pro",
    "sysclk_lwla",
    "sysclk_sla5032",
    "testo",
    "uni_t_dmm",
    "zeroplus_logic_cube",
];

/// The sources of the driver in `dir`, in one text.
fn sources(dir: &Path) -> String {
    let mut text = String::new();
    for entry in std::fs::read_dir(dir).expect("a driver's directory") {
        let path = entry.expect("an entry").path();
        if path.is_dir() {
            text.push_str(&sources(&path));
        } else if path.extension().is_some_and(|e| e == "rs") {
            text.push_str(&std::fs::read_to_string(&path).expect("a source"));
        }
    }
    text
}

#[test]
fn every_driver_bounds_its_sends_or_is_known_not_to() {
    let drivers = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/src/drivers"));
    let mut seen = 0;
    let mut new = Vec::new();
    let mut done = Vec::new();
    for entry in std::fs::read_dir(drivers).expect("src/drivers") {
        let path = entry.expect("an entry").path();
        if !path.is_dir() {
            continue;
        }
        let name = path.file_name().unwrap().to_string_lossy().into_owned();
        let text = sources(&path);
        if !text.contains("PacketSink") {
            continue;
        }
        seen += 1;
        let bounded = ["stopped_by", "send_while", "end_while"].iter().any(|f| text.contains(f));
        let listed = NOT_BOUNDED.contains(&name.as_str());
        match (bounded, listed) {
            (false, false) => new.push(name),
            (true, true) => done.push(name),
            _ => {}
        }
    }
    // A tree with no drivers in it would pass whatever they did.
    assert!(seen > 80, "{seen} drivers found under {}", drivers.display());
    new.sort();
    done.sort();
    assert!(
        new.is_empty(),
        "{new:?} send to a queue with no bound after a stop: tell the sink of the stop flag \
         (`PacketSink::stopped_by`) before the acquisition thread takes it, and close the device \
         on an unread queue in a test"
    );
    assert!(done.is_empty(), "{done:?} bound their sends now: take them out of NOT_BOUNDED");
    for name in NOT_BOUNDED {
        assert!(drivers.join(name).is_dir(), "NOT_BOUNDED names {name}, which is no driver");
    }
}

/// Runs `body` with a limit, so that a close that waits fails the test
/// rather than holding the suite.
fn within_limit<T: Send + 'static>(body: impl FnOnce() -> T + Send + 'static) -> T {
    let (tx, rx) = channel();
    std::thread::spawn(move || {
        let _ = tx.send(body());
    });
    rx.recv_timeout(Duration::from_secs(30)).expect("the stop waited on a queue nobody reads")
}

fn demo() -> Box<dyn Device> {
    let mut registry = Registry::new();
    oxrok::register_all(&mut registry);
    let mut dev = registry.find("demo").expect("the demo driver").scan(&[]).unwrap().remove(0);
    dev.open().unwrap();
    dev
}

#[test]
fn the_demo_device_stops_and_closes_with_a_queue_nobody_reads() {
    let (left, mut dev) = within_limit(|| {
        let mut dev = demo();
        // No limit: it sends until it is stopped.
        dev.set(&Key::Samplerate, &Value::U64(1_000_000), None).unwrap();
        let (tx, rx) = sync_channel(4);
        dev.start(PacketSink::new(tx, 0)).unwrap();
        std::thread::sleep(Duration::from_millis(100));
        dev.stop().unwrap();
        dev.close().unwrap();
        (rx.try_iter().map(|(_, packet)| packet).collect::<Vec<_>>(), dev)
    });
    // What the queue held is the start of the capture.
    assert_eq!(left.len(), 4);
    assert!(matches!(left[0], Packet::Header { .. }), "{:?}", left[0]);
    assert!(left[1..].iter().all(|p| matches!(p, Packet::Logic(_) | Packet::Analog(_))));

    // And a reader that reads gets all of the next one, to its End.
    dev.open().unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
    let (tx, rx) = sync_channel(4);
    dev.start(PacketSink::new(tx, 0)).unwrap();
    let packets: Vec<Packet> = rx.iter().map(|(_, packet)| packet).collect();
    dev.stop().unwrap();
    assert!(matches!(packets.last(), Some(Packet::End)), "{:?}", packets.last());
    let samples: usize = packets
        .iter()
        .filter_map(|p| match p {
            Packet::Logic(logic) => Some(logic.data.len() / logic.unit_size),
            _ => None,
        })
        .sum();
    assert_eq!(samples, 1000);
}
