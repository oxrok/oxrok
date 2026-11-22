//! Tests of the driver against a board made of a few lines: what it
//! sends, in which order, and what it makes of the samples.

use super::*;
use crate::core::trigger::{TriggerCondition, TriggerStage};
use crate::usb::ControlRequest;
use std::collections::VecDeque;
use std::sync::mpsc::Receiver;
use std::time::Duration;

/// What the board is and does, shared with the test.
#[derive(Default)]
struct BoardState {
    /// Every transfer, as a line.
    log: Vec<String>,
    /// Answers waiting on endpoint 1.
    answers: VecDeque<Vec<u8>>,
    /// The key count; `None` for firmware that reports none.
    key: Option<u8>,
    /// The status `a0` is answered with.
    params_status: u8,
    /// What the sample stream gives, in order; then silence.
    transfers: VecDeque<Result<Vec<u8>>>,
}

struct Board {
    info: UsbDeviceInfo,
    serial: &'static str,
    bcd_usb: u16,
    state: Arc<Mutex<BoardState>>,
}

fn board(serial: &'static str, bcd_usb: u16) -> (Box<dyn Transport>, Arc<Mutex<BoardState>>) {
    let state = Arc::new(Mutex::new(BoardState { key: Some(7), ..BoardState::default() }));
    let info = UsbDeviceInfo {
        vendor_id: ID.vendor,
        product_id: ID.product,
        bus: 3,
        address: 9,
        ports: vec![1, 4],
        ..UsbDeviceInfo::default()
    };
    (Box::new(Board { info, serial, bcd_usb, state: Arc::clone(&state) }), state)
}

/// The transfers since the last call.
fn taken(state: &Arc<Mutex<BoardState>>) -> Vec<String> {
    std::mem::take(&mut state.lock().unwrap().log)
}

impl Transport for Board {
    fn info(&self) -> &UsbDeviceInfo {
        &self.info
    }
    fn set_configuration(&mut self, _configuration: u8) -> Result<()> {
        Ok(())
    }
    fn string_indexes(&mut self) -> Result<[u8; 3]> {
        Ok([1, 2, 3])
    }
    fn usb_version(&mut self) -> Result<u16> {
        Ok(self.bcd_usb)
    }
    fn claim_interface(&mut self, interface: u8) -> Result<()> {
        self.state.lock().unwrap().log.push(format!("claim {interface}"));
        Ok(())
    }
    fn release_interface(&mut self, _interface: u8) -> Result<()> {
        Ok(())
    }
    fn reset(&mut self) -> Result<()> {
        Ok(())
    }
    fn max_packet_size(&mut self, _endpoint: u8) -> Result<usize> {
        Ok(512)
    }
    fn interfaces(&mut self) -> Result<Vec<crate::usb::InterfaceDesc>> {
        Ok(Vec::new())
    }
    fn control_out(&mut self, _r: ControlRequest, _d: &[u8], _t: Duration) -> Result<()> {
        Err(Error::device("stalled"))
    }
    /// The string descriptors: the language table, then string 3.
    fn control_in(&mut self, r: ControlRequest, _len: usize, _t: Duration) -> Result<Vec<u8>> {
        self.state.lock().unwrap().log.push(format!("string {:#06x}", r.value));
        match r.value {
            0x0300 => Ok(vec![4, 3, 0x09, 0x04]),
            0x0303 => {
                let mut d = vec![2 + 2 * self.serial.len() as u8, 3];
                d.extend(self.serial.bytes().flat_map(|b| [b, 0]));
                Ok(d)
            }
            _ => Err(Error::device("stalled")),
        }
    }
    fn bulk_out(&mut self, endpoint: u8, data: &[u8], _t: Duration) -> Result<usize> {
        assert_eq!(endpoint, protocol::EP_COMMAND);
        assert_eq!(data.len(), protocol::COMMAND_SIZE, "a command is 64 bytes");
        let mut s = self.state.lock().unwrap();
        let (command, n) = (data[0], usize::from(data[1]));
        assert!(data[2 + n..].iter().all(|b| *b == 0), "a command is zero padded");
        let args: Vec<String> = data[2..2 + n].iter().map(|b| format!("{b:02x}")).collect();
        s.log.push(format!("{command:02x} {}", args.join(" ")).trim_end().to_owned());
        let answer = |status: u8, payload: &[u8]| {
            let mut a = vec![command | 0x10, 1 + payload.len() as u8, status];
            a.extend_from_slice(payload);
            a
        };
        let a = match command {
            protocol::CMD_SET_LOGIC_PARAMS => Some(answer(s.params_status, &[])),
            protocol::CMD_GET_LOGIC_PARAMS => Some(match s.key {
                Some(k) => answer(0, &[0x10, 0, k]),
                None => answer(0, &[0x10, 0]),
            }),
            protocol::CMD_GET_VERSION => Some(answer(0, &[2, 1])),
            protocol::CMD_SET_LOGIC_LEVEL
            | protocol::CMD_SET_LED_BRIGHT
            | protocol::CMD_SET_ADC_PARAMS
            | protocol::CMD_SET_ADC_CHANNEL => Some(answer(0, &[])),
            // Starts and stops have no answer.
            _ => None,
        };
        s.answers.extend(a);
        Ok(data.len())
    }
    fn bulk_in(&mut self, endpoint: u8, len: usize, _t: Duration) -> Result<Vec<u8>> {
        assert_eq!((endpoint, len), (protocol::EP_ANSWER, protocol::ANSWER_SIZE));
        let mut s = self.state.lock().unwrap();
        if let Some(a) = s.answers.pop_front() {
            s.log.push(format!("answer {:02x}", a[0]));
            Ok(a)
        } else {
            s.log.push("answer none".into());
            Err(Error::Timeout)
        }
    }
    fn interrupt_out(&mut self, _e: u8, data: &[u8], _t: Duration) -> Result<usize> {
        Ok(data.len())
    }
    fn interrupt_in(&mut self, _e: u8, _l: usize, _t: Duration) -> Result<Vec<u8>> {
        Ok(Vec::new())
    }
    fn bulk_in_stream(
        &mut self,
        endpoint: u8,
        size: usize,
        depth: usize,
    ) -> Result<Box<dyn BulkStream>> {
        self.state.lock().unwrap().log.push(format!("stream {endpoint:02x} {depth}x{size}"));
        Ok(Box::new(Samples(Arc::clone(&self.state))))
    }
}

struct Samples(Arc<Mutex<BoardState>>);

impl BulkStream for Samples {
    fn next(&mut self, _timeout: Duration) -> Result<Vec<u8>> {
        self.0.lock().unwrap().transfers.pop_front().unwrap_or(Err(Error::Timeout))
    }
}

/// An open V2 on a USB 3 link.
fn open_v2() -> (Ch32h417Device, Arc<Mutex<BoardState>>) {
    let (t, state) = board("b5e339e33906", 0x0300);
    let mut dev = Ch32h417Device::from_transport(t).unwrap();
    dev.open().unwrap();
    taken(&state);
    (dev, state)
}

/// Runs a capture to its end and gives its packets, the header left out.
fn capture(dev: &mut Ch32h417Device) -> Vec<Packet> {
    let (tx, rx) = std::sync::mpsc::sync_channel(1024);
    dev.start(PacketSink::new(tx, 0)).unwrap();
    let packets = until_end(&rx);
    dev.stop().unwrap();
    packets
}

fn until_end(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let mut out = Vec::new();
    loop {
        let (_, p) = rx.recv_timeout(Duration::from_secs(10)).expect("the capture ends");
        match p {
            Packet::Header { .. } => {}
            Packet::End => {
                out.push(Packet::End);
                return out;
            }
            other => out.push(other),
        }
    }
}

/// Samples `from..to` of a 16 bit counter, as the wire carries them.
fn counter16(from: u16, to: u16) -> Vec<u8> {
    (from..to).flat_map(u16::to_le_bytes).collect()
}

fn rising(channel: usize) -> Trigger {
    Trigger {
        stages: vec![TriggerStage {
            conditions: vec![TriggerCondition {
                channel,
                matches: TriggerMatch::Rising,
                value: 0.0,
            }],
            ..TriggerStage::default()
        }],
        ..Trigger::default()
    }
}

#[test]
fn the_scan_reads_the_serial_claims_and_drains() {
    let (t, state) = board("B5E339E33906", 0x0300);
    let dev = Ch32h417Device::from_transport(t).unwrap();
    assert_eq!(taken(&state), ["string 0x0300", "string 0x0303", "claim 0", "answer none"]);
    let info = dev.info();
    assert_eq!(
        (info.vendor.as_str(), info.model.as_str(), info.serial.as_str()),
        ("CH32H417", "WeAct Studio Logic Analyzer V2", "b5e339e33906")
    );
    assert_eq!(info.conn.as_deref(), Some("usb/3-1.4"));
    assert_eq!(dev.channels().len(), 16);
    assert_eq!(dev.channel_groups().len(), 1);
}

#[test]
fn a_serial_of_no_known_board_is_no_device() {
    let (t, state) = board("7f0000", 0x0300);
    assert!(Ch32h417Device::from_transport(t).is_err());
    // Nothing is claimed or read on a board that is not taken.
    assert_eq!(taken(&state), ["string 0x0300", "string 0x0303"]);
}

#[test]
fn the_original_board_has_two_analog_channels() {
    let (t, _state) = board("0123", 0x0300);
    let dev = Ch32h417Device::from_transport(t).unwrap();
    assert_eq!(dev.info().model, "CH32H417 Logic Analyzer");
    let names: Vec<&str> = dev.channels()[16..].iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, ["A0", "A1"]);
    let groups: Vec<&str> = dev.channel_groups().iter().map(|g| g.name.as_str()).collect();
    assert_eq!(groups, ["Logic", "A0", "A1"]);
    assert_eq!(
        dev.list(&Key::PatternMode, None).unwrap(),
        ValueList::Str(vec!["Logic Analyzer".into(), "ADC".into()])
    );
}

#[test]
fn conn_takes_the_scans_id_and_sigroks_forms() {
    let (t, _state) = board("b5", 0x0300);
    let info = t.info().clone();
    assert!(conn_matches("usb/3-1.4", &info));
    assert!(conn_matches("3.9", &info));
    assert!(conn_matches("1a86.5537", &info));
    assert!(!conn_matches("usb/3-1.5", &info));
    assert!(!conn_matches("3.8", &info));
    assert!(!conn_matches("nonsense", &info));
    // No ports known: the bus and the address.
    let bare = UsbDeviceInfo { ports: Vec::new(), ..info };
    assert_eq!(connection_id(&bare), "3.9");
}

#[test]
fn the_open_takes_the_links_rates_and_the_key_count() {
    let (t, state) = board("b5e3", 0x0300);
    let mut dev = Ch32h417Device::from_transport(t).unwrap();
    taken(&state);
    dev.open().unwrap();
    assert_eq!(taken(&state), ["a4", "answer b4"]);
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(200_000_000));
    assert_eq!(dev.get(&USB_VERSION, None).unwrap(), Value::U64(3));
    assert_eq!(
        dev.list(&Key::Samplerate, None).unwrap(),
        ValueList::U64(protocol::RATES_USB3.to_vec())
    );
    // The count at the open is no press.
    assert_eq!(dev.get(&KEY_PRESSED, None).unwrap(), Value::Bool(false));
    state.lock().unwrap().key = Some(8);
    assert_eq!(dev.get(&KEY_PRESSED, None).unwrap(), Value::Bool(true));
    assert_eq!(dev.get(&KEY_PRESSED, None).unwrap(), Value::Bool(false));
}

#[test]
fn a_usb2_link_has_its_own_rates_by_the_sample_width() {
    let (t, _state) = board("b5e3", 0x0210);
    let mut dev = Ch32h417Device::from_transport(t).unwrap();
    dev.open().unwrap();
    assert_eq!(dev.get(&USB_VERSION, None).unwrap(), Value::U64(2));
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(20_000_000));
    assert!(dev.set(&Key::Samplerate, &Value::U64(40_000_000), None).is_err());
    dev.set(&Key::Samplerate, &Value::U64(8_000_000), None).unwrap();
    // The same count again keeps the rate; eight channels pick the other
    // table and its fastest rate.
    dev.set(&Key::NumLogicChannels, &Value::I32(16), None).unwrap();
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(8_000_000));
    dev.set(&Key::NumLogicChannels, &Value::I32(8), None).unwrap();
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(40_000_000));
    assert_eq!(
        dev.list(&Key::Samplerate, None).unwrap(),
        ValueList::U64(protocol::RATES_USB2_8.to_vec())
    );
    assert!(dev.set(&Key::NumLogicChannels, &Value::I32(0), None).is_err());
    assert!(dev.set(&Key::NumLogicChannels, &Value::I32(17), None).is_err());
}

#[test]
fn firmware_without_the_key_count_has_no_key_options() {
    let (t, state) = board("b5e3", 0x0300);
    state.lock().unwrap().key = None;
    let mut dev = Ch32h417Device::from_transport(t).unwrap();
    dev.open().unwrap();
    taken(&state);
    assert!(matches!(dev.get(&KEY_START_STOP, None), Err(Error::NotSupported(_))));
    assert!(matches!(
        dev.set(&KEY_START_STOP, &Value::Bool(true), None),
        Err(Error::NotSupported(_))
    ));
    // No press is asked for either.
    assert_eq!(dev.get(&KEY_PRESSED, None).unwrap(), Value::Bool(false));
    assert_eq!(taken(&state), Vec::<String>::new());
}

#[test]
fn settings_that_go_to_the_device_at_once() {
    let (mut dev, state) = open_v2();
    assert_eq!(dev.get(&HARDWARE_VERSION, None).unwrap(), Value::U64(0x0201));
    assert_eq!(taken(&state), ["ac", "answer bc"]);
    dev.set(&LED_BRIGHTNESS, &Value::U64(55), None).unwrap();
    assert_eq!(taken(&state), ["af 37", "answer bf"]);
    assert_eq!(dev.get(&LED_BRIGHTNESS, None).unwrap(), Value::U64(55));
    assert!(dev.set(&LED_BRIGHTNESS, &Value::U64(101), None).is_err());
    // 0.6 V on the pins is 1.0 V at the input: ten tenths, bit 15 set.
    assert_eq!(dev.get(&Key::VoltageThreshold, None).unwrap(), Value::RangeF64(0.0, 0.0));
    dev.set(&Key::VoltageThreshold, &Value::RangeF64(0.6, 0.6), None).unwrap();
    assert_eq!(taken(&state), ["a1 80 0a", "answer b1"]);
    assert_eq!(dev.get(&Key::VoltageThreshold, None).unwrap(), Value::RangeF64(0.6, 0.6));
    assert!(dev.set(&Key::VoltageThreshold, &Value::RangeF64(3.3, 3.3), None).is_err());
    assert_eq!(taken(&state), Vec::<String>::new());
    // A V2 has no ADC to switch to.
    assert!(dev.set(&Key::PatternMode, &Value::Str("ADC".into()), None).is_err());
    assert!(dev.set(&Key::CaptureRatio, &Value::U64(101), None).is_err());
}

#[test]
fn the_options_are_the_c_drivers() {
    let (dev, _state) = open_v2();
    let ids: Vec<&str> = dev.options(None).unwrap().iter().map(|o| o.key.id()).collect();
    assert_eq!(
        ids,
        [
            "continuous",
            "limit_frames",
            "limit_samples",
            "conn",
            "samplerate",
            "triggermatch",
            "captureratio",
            "logic_channels",
            "pattern",
            "voltage_threshold",
            "hardware_version",
            "usb_version",
            "link_stop_reason",
            "key_start_stop",
            "key_pressed",
            "led_brightness",
        ]
    );
    // Any group gets the ADC's options, as config_list() gives them.
    let ids: Vec<&str> = dev.options(Some("Logic")).unwrap().iter().map(|o| o.key.id()).collect();
    assert_eq!(ids, ["samplerate", "limit_samples", "adc_precision", "adc_channel"]);
    assert!(matches!(dev.options(Some("A0")), Err(Error::ChannelGroup(_))));
}

#[test]
fn a_capture_sends_the_c_drivers_sequence() {
    let (mut dev, state) = open_v2();
    dev.set(&Key::Samplerate, &Value::U64(4_000_000), None).unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(6), None).unwrap();
    state.lock().unwrap().transfers.push_back(Ok(counter16(0, 100)));
    let packets = capture(&mut dev);
    assert_eq!(
        packets,
        [Packet::FrameBegin, Packet::logic(2, counter16(0, 6)), Packet::FrameEnd, Packet::End]
    );
    // Drain, stop, parameters (two bytes, index 24, no key stop) and
    // their answer, the transfers, the start; and the stop at the end.
    assert_eq!(
        taken(&state),
        ["answer none", "a3", "a0 10 18 00", "answer b0", "stream 82 32x262144", "a2", "a3"]
    );
    assert_eq!(dev.get(&LINK_STOP_REASON, None).unwrap(), Value::U64(0));
}

#[test]
fn eight_channels_are_a_byte_a_sample_and_the_key_stop_goes_down() {
    let (mut dev, state) = open_v2();
    dev.set(&Key::NumLogicChannels, &Value::I32(8), None).unwrap();
    dev.set(&KEY_START_STOP, &Value::Bool(true), None).unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(3), None).unwrap();
    state.lock().unwrap().transfers.push_back(Ok(vec![1, 2, 3, 4, 5]));
    let packets = capture(&mut dev);
    assert_eq!(packets[1], Packet::logic(1, vec![1, 2, 3]));
    assert!(taken(&state).contains(&"a0 08 00 01".to_owned()));
}

#[test]
fn a_refused_start_stops_the_device_and_fails() {
    let (mut dev, state) = open_v2();
    state.lock().unwrap().params_status = 1;
    let (tx, _rx) = std::sync::mpsc::sync_channel(8);
    let e = dev.start(PacketSink::new(tx, 0)).unwrap_err();
    assert!(e.to_string().contains("status 0x01"), "{e}");
    assert_eq!(taken(&state), ["answer none", "a3", "a0 10 00 00", "answer b0", "a3"]);
    // The link is back: the device still answers.
    assert_eq!(dev.get(&HARDWARE_VERSION, None).unwrap(), Value::U64(0x0201));
}

#[test]
fn silence_fails_the_capture_unless_the_key_ended_it() {
    let (mut dev, state) = open_v2();
    dev.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
    state.lock().unwrap().transfers.push_back(Ok(counter16(0, 10)));
    let packets = capture(&mut dev);
    assert_eq!(packets.len(), 5, "{packets:?}");
    assert_eq!(packets[0], Packet::FrameBegin);
    assert_eq!(packets[1], Packet::logic(2, counter16(0, 10)));
    // The frame cut short is ended, and the capture failed.
    assert_eq!(packets[2], Packet::FrameEnd);
    assert!(matches!(&packets[3], Packet::Failed(m) if m.contains("no samples for 500 ms")));
    assert_eq!(dev.get(&LINK_STOP_REASON, None).unwrap(), Value::U64(1));
    // The key count is asked before the stop.
    let log = taken(&state);
    assert_eq!(log[log.len() - 3..], ["a4", "answer b4", "a3"]);

    // The same silence after a press of the key is the capture's end.
    {
        let mut s = state.lock().unwrap();
        s.transfers.push_back(Ok(counter16(0, 10)));
        s.key = Some(9);
    }
    let packets = capture(&mut dev);
    assert_eq!(packets.len(), 4, "{packets:?}");
    assert_eq!(packets[3], Packet::End);
    assert_eq!(dev.get(&LINK_STOP_REASON, None).unwrap(), Value::U64(0));
}

#[test]
fn a_failed_sample_transfer_fails_the_capture() {
    let (mut dev, state) = open_v2();
    dev.set(&Key::LimitSamples, &Value::U64(1000), None).unwrap();
    state.lock().unwrap().transfers.push_back(Err(Error::Disconnected));
    let packets = capture(&mut dev);
    assert!(matches!(&packets[0], Packet::Failed(m) if m.contains("disconnected")), "{packets:?}");
    assert_eq!(dev.get(&LINK_STOP_REASON, None).unwrap(), Value::U64(1));
}

#[test]
fn a_rate_of_another_table_is_refused_before_anything_is_sent() {
    let (t, state) = board("b5e3", 0x0200);
    let mut dev = Ch32h417Device::from_transport(t).unwrap();
    dev.open().unwrap();
    taken(&state);
    // Forced past the set: the table changed under the rate.
    dev.settings.samplerate = 200_000_000;
    assert!(dev.check_capture(None).is_err());
    let (tx, _rx) = std::sync::mpsc::sync_channel(8);
    assert!(dev.start(PacketSink::new(tx, 0)).is_err());
    // Only the stop a failed start is followed by.
    assert_eq!(taken(&state), ["a3"]);
}

#[test]
fn the_adc_mode_sends_volts_of_the_selected_input() {
    let (t, state) = board("0123", 0x0300);
    let mut dev = Ch32h417Device::from_transport(t).unwrap();
    dev.open().unwrap();
    dev.set(&Key::PatternMode, &Value::Str("ADC".into()), None).unwrap();
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(20_000_000));
    dev.set(&Key::Samplerate, &Value::U64(13_330_000), None).unwrap();
    dev.set(&ADC_CHANNEL, &Value::U64(1), Some("A1")).unwrap();
    dev.set(&Key::LimitSamples, &Value::U64(2), None).unwrap();
    assert!(dev.set(&ADC_PRECISION, &Value::U64(9), None).is_err());
    assert!(dev.set(&ADC_CHANNEL, &Value::U64(2), None).is_err());
    taken(&state);
    state.lock().unwrap().transfers.push_back(Ok(vec![0, 128, 255]));
    let packets = capture(&mut dev);
    let Packet::Analog(a) = &packets[1] else { panic!("{packets:?}") };
    assert_eq!(a.channels, [17]);
    assert_eq!(a.samples, protocol::adc_volts(&[0, 128], false));
    // Stop, the parameters (8 bits, index 2 + 3), the input, the start.
    assert_eq!(
        taken(&state),
        [
            "answer none",
            "a9",
            "a6 08 05",
            "answer b6",
            "a7 01",
            "answer b7",
            "stream 83 32x262144",
            "a8",
            "a9"
        ]
    );
    // A trigger there is refused before anything is sent.
    dev.set_trigger(Some(rising(0))).unwrap();
    assert!(matches!(dev.check_capture(dev.trigger.as_ref()), Err(Error::NotSupported(_))));
}

/// The packets `Frames` makes of `transfers`.
fn frames(mut f: Frames, transfers: &[Vec<u8>]) -> (Vec<Packet>, bool) {
    let (tx, rx) = std::sync::mpsc::sync_channel(1024);
    let sink = PacketSink::new(tx, 0);
    let mut done = false;
    for t in transfers {
        assert!(!done, "a transfer after the last frame");
        done = f.feed(t, &sink);
    }
    f.finish(&sink);
    drop(sink);
    (rx.iter().map(|(_, p)| p).collect(), done)
}

fn stl(pre: usize) -> SoftTriggerLogic {
    SoftTriggerLogic::new(vec![vec![(0, TriggerMatch::Rising)]], 1, pre).arm_when_full()
}

#[test]
fn frames_follow_one_another_without_a_trigger() {
    let data: Vec<u8> = (0..10).collect();
    let (packets, done) = frames(Frames::new(1, 4, 2, None, None), std::slice::from_ref(&data));
    assert!(done);
    assert_eq!(
        packets,
        [
            Packet::FrameBegin,
            Packet::logic(1, vec![0, 1, 2, 3]),
            Packet::FrameEnd,
            Packet::FrameBegin,
            Packet::logic(1, vec![4, 5, 6, 7]),
            Packet::FrameEnd,
        ]
    );
    // A frame that ends with a transfer: the next begins with the next.
    let (packets, done) =
        frames(Frames::new(1, 4, 2, None, None), &[vec![0, 1, 2, 3], vec![4, 5, 6, 7, 8]]);
    assert!(done);
    assert_eq!(packets.len(), 6);
    assert_eq!(packets[4], Packet::logic(1, vec![4, 5, 6, 7]));
    // No frame limit: frames until a stop, the last one ended by it.
    let (packets, done) = frames(Frames::new(1, 4, 0, None, None), &[data]);
    assert!(!done);
    assert_eq!(packets.len(), 9);
    assert_eq!(packets[7], Packet::logic(1, vec![8, 9]));
    assert_eq!(packets[8], Packet::FrameEnd);
}

#[test]
fn no_sample_limit_is_one_frame_without_an_end() {
    let (packets, done) = frames(Frames::new(2, 0, 1, None, None), &[vec![1, 2, 3, 4, 5]]);
    assert!(!done);
    // Half a sample at the end of a transfer is dropped.
    assert_eq!(packets, [Packet::FrameBegin, Packet::logic(2, vec![1, 2, 3, 4]), Packet::FrameEnd]);
}

#[test]
fn a_trigger_starts_each_frame_with_its_pre_trigger_samples() {
    // D0: 0 0 1 1 0 1 1 1 0 0 1 0; rises at 2, 5 and 10.
    let data = vec![0u8, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0];
    // Four samples a frame, half of them before the trigger.
    let (packets, done) =
        frames(Frames::new(1, 4, 2, Some(stl(2)), None), std::slice::from_ref(&data));
    assert!(done);
    assert_eq!(
        packets,
        [
            Packet::FrameBegin,
            Packet::logic(1, vec![0, 0]),
            Packet::Trigger,
            Packet::logic(1, vec![1, 1]),
            Packet::FrameEnd,
            // The next frame holds two samples again before it matches: the
            // rise at 5 falls into them, the one at 10 fires.
            Packet::FrameBegin,
            Packet::logic(1, vec![0, 0]),
            Packet::Trigger,
            Packet::logic(1, vec![1, 0]),
            Packet::FrameEnd,
        ]
    );
    // No pre-trigger samples: the first edge, as libsigrok's.
    let (packets, _) =
        frames(Frames::new(1, 4, 1, Some(stl(0)), None), std::slice::from_ref(&data));
    assert_eq!(
        packets,
        [Packet::FrameBegin, Packet::Trigger, Packet::logic(1, vec![1, 1, 0, 1]), Packet::FrameEnd,]
    );
    // Nothing is sent while the trigger waits, across transfers.
    let (packets, done) = frames(
        Frames::new(1, 4, 1, Some(stl(0)), None),
        &[vec![0, 0], vec![0, 1, 1], vec![0, 0, 0]],
    );
    assert!(done);
    assert_eq!(
        packets,
        [
            Packet::FrameBegin,
            Packet::Trigger,
            Packet::logic(1, vec![1, 1]),
            Packet::logic(1, vec![0, 0]),
            Packet::FrameEnd,
        ]
    );
}

#[test]
fn a_frame_that_ends_with_a_transfer_waits_for_a_new_trigger() {
    // The first frame's last sample is the transfer's last one.
    let (packets, done) =
        frames(Frames::new(1, 2, 2, Some(stl(0)), None), &[vec![0, 1, 1], vec![1, 0, 1, 0]]);
    assert!(done);
    assert_eq!(
        packets,
        [
            Packet::FrameBegin,
            Packet::Trigger,
            Packet::logic(1, vec![1, 1]),
            Packet::FrameEnd,
            // Not at once: the level held is no edge, and the first sample
            // after a reset has none before it.
            Packet::FrameBegin,
            Packet::Trigger,
            Packet::logic(1, vec![1, 0]),
            Packet::FrameEnd,
        ]
    );
}
