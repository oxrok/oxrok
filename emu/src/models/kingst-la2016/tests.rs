//! The model driven as `kingst-la2016` drives it, and one violation of
//! each kind.

use super::*;
use crate::device::Device;

fn vendor(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> Setup {
    Setup { request_type, request, value, index, length }
}

fn rd(request: u8, value: u16, length: u16) -> Setup {
    vendor(0xc0, request, value, 0, length)
}

fn wr(request: u8, value: u16, length: u16) -> Setup {
    vendor(0x40, request, value, 0, length)
}

fn set_config(dev: &mut Device, v: &mut Violations) {
    let s = Setup { request_type: 0, request: 9, value: 1, index: 0, length: 0 };
    assert_eq!(dev.control(s, &[], v), Reply::ack());
}

fn data(r: Reply) -> Vec<u8> {
    let Reply::Data(d) = r else { panic!("{r:?}") };
    d
}

fn spi_write(dev: &mut Device, reg: u8, bytes: &[u8], v: &mut Violations) -> Reply {
    let index = if reg == REG_TRIGGER { 16 } else { 0 };
    dev.control(vendor(0x40, CMD_FPGA_SPI, u16::from(reg), index, bytes.len() as u16), bytes, v)
}

fn spi_read(dev: &mut Device, reg: u8, len: u16, v: &mut Violations) -> Vec<u8> {
    data(dev.control(rd(CMD_FPGA_SPI, u16::from(reg), len), &[], v))
}

fn device(options: &[(&str, &str)]) -> Device {
    let o: Vec<(String, String)> =
        options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
    Device::new(build(&o).unwrap())
}

/// The scan and open of the driver on a device whose firmware runs:
/// identification, the bitstream, the init, the PWM writes of
/// `dev_open()`.
fn identify_and_init(dev: &mut Device, bitstream: &[u8], v: &mut Violations) {
    set_config(dev, v);
    assert_eq!(data(dev.control(rd(CMD_EEPROM, 0x20, 4), &[], v)), [0x20, 0x04, 0xdf, 0xfb]);
    assert_eq!(
        data(dev.control(rd(CMD_EEPROM, 0x08, 8), &[], v)),
        [0x08, 0xf7, 0x00, 0x00, 0x08, 0xf7, 0x10, 0xef],
        "the example of protocol.c"
    );
    // A blank FPGA: the check fails, the upload follows.
    assert_eq!(data(dev.control(rd(CMD_FPGA_INIT, 0, 1), &[], v)), [1]);
    let size = (bitstream.len() as u32).to_le_bytes();
    assert_eq!(dev.control(wr(CMD_FPGA_INIT, 0, 4), &size, v), Reply::ack());
    let mut padded = bitstream.to_vec();
    padded.resize(bitstream.len().div_ceil(4096) * 4096, 0);
    for chunk in bitstream.chunks(4096) {
        assert_eq!(dev.transfer_out(2, chunk, v), Reply::ack());
    }
    let tail = padded.len() - bitstream.len();
    assert_eq!(dev.transfer_out(2, &vec![0; tail], v), Reply::ack());
    assert_eq!(data(dev.control(rd(CMD_FPGA_INIT, 0, 1), &[], v)), [0]);
    assert_eq!(dev.control(wr(CMD_FPGA_ENABLE, 1, 0), &[], v), Reply::ack());
    assert_eq!(spi_read(dev, REG_RUN, 2, v), [0xe1, 0x85]);
    assert_eq!(dev.control(wr(CMD_BULK_RESET, 0, 0), &[], v), Reply::ack());
    // dev_open(): both PWM outputs off.
    spi_write(dev, REG_PWM_EN, &[0], v);
    spi_write(dev, REG_PWM_EN, &[0], v);
}

/// `set_sample_config()`'s 16 bytes.
fn sampling(limit: u64, pre: u64, mem: u64, divider: u16) -> Vec<u8> {
    let mut b = Vec::new();
    b.extend_from_slice(&limit.to_le_bytes()[..5]);
    b.extend_from_slice(&pre.to_le_bytes()[..5]);
    b.extend_from_slice(&(mem >> 8).to_le_bytes()[..3]);
    b.extend_from_slice(&divider.to_le_bytes());
    b.push(0);
    b
}

fn trigger(channels: u32, enabled: u32, level: u32, hf: u32) -> Vec<u8> {
    [channels, enabled, level, hf].iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// `la2016_setup_acquisition()` at 1.4 V.
fn setup(dev: &mut Device, mode: u8, trig: &[u8], samp: &[u8], v: &mut Violations) {
    let (r56, r79) = threshold_duties(1.4);
    let mut t = r56.to_le_bytes().to_vec();
    t.extend_from_slice(&r79.to_le_bytes());
    spi_write(dev, REG_THRESHOLD, &t, v);
    spi_write(dev, REG_CAPT_MODE, &[mode], v);
    spi_write(dev, REG_TRIGGER, trig, v);
    spi_write(dev, REG_SAMPLING, samp, v);
}

#[test]
fn the_standin_sizes_are_those_of_the_firmware_list() {
    let list = include_str!("../../../../tools/emu/kingst-la2016.firmware");
    let size = |name: &str| -> usize {
        let line = list
            .lines()
            .find(|l| l.split_whitespace().next() == Some(name))
            .unwrap_or_else(|| panic!("{name} is not in tools/emu/kingst-la2016.firmware"));
        line.split_whitespace().nth(1).unwrap().parse().unwrap()
    };
    assert_eq!(size(MCU_FIRMWARE), MCU_FIRMWARE_SIZE);
    for p in PROFILES {
        assert_eq!(size(&p.bitstream()), p.bitstream_size, "{}", p.key);
    }
}

#[test]
fn the_threshold_duties_are_those_of_the_driver() {
    // 1.4 V: 302 * 1.4 + 121 = 543.8; 4.0 V: 302 * 4 - 363 = 845.
    assert_eq!(threshold_duties(1.4), (543, 0x00f2));
    assert_eq!(threshold_duties(4.0), (845, 0));
    assert_eq!(threshold_duties(0.4), (241, 0x00f2));
}

#[test]
fn uploads_identifies_captures_and_downloads_as_the_driver_does() {
    let model = Kingst::new(PROFILES[0], false, false, Faults::default());
    let bitstream = model.bitstream();
    let mut dev = Device::new(Box::new(model));
    let mut v = Violations::default();
    assert_eq!(dev.descriptors().device[15], 0, "no iProduct before the firmware");
    set_config(&mut dev, &mut v);
    let image = Kingst::mcu_image();
    dev.control(wr(0xa0, 0xe600, 1), &[1], &mut v);
    for (i, c) in image.chunks(4096).enumerate() {
        dev.control(wr(0xa0, (i * 4096) as u16, c.len() as u16), c, &mut v);
    }
    dev.control(wr(0xa0, 0xe600, 1), &[0], &mut v);
    assert!(v.is_empty(), "{v:?}");
    assert!(dev.take_renumeration());
    dev.reset();
    assert_eq!(dev.descriptors().device[15], 2, "iProduct 2 once the firmware runs");
    identify_and_init(&mut dev, &bitstream, &mut v);
    assert!(v.is_empty(), "{v:?}");

    // 3000 samples at 20 kHz (200 MHz / 10000, 1000 samples a poll),
    // rising D0, ratio 90 %.
    let mem = 1024 * 1024 * 1024 / 8 * 90 / 100;
    setup(&mut dev, 0, &trigger(0xffff, 1, 0, 0), &sampling(3000, 2700, mem, 10000), &mut v);
    spi_write(&mut dev, REG_RUN, &[3], &mut v);
    assert!(v.is_empty(), "{v:?}");
    let states: Vec<u16> = (0..4)
        .map(|_| {
            let b = spi_read(&mut dev, REG_RUN, 2, &mut v);
            u16::from_le_bytes([b[0], b[1]])
        })
        .collect();
    // 2700 samples before the trigger point; D0 rises next at 3004.
    assert_eq!(states, [RUN_PRE, RUN_PRE, RUN_WAIT, RUN_DONE]);
    let info = spi_read(&mut dev, REG_SAMPLING, 16, &mut v);
    let w = |i: usize| u32::from_le_bytes([info[i], info[i + 1], info[i + 2], info[i + 3]]);
    let (packets, before, write_pos) = (w(0), w(4), w(8));
    assert_eq!(packets % 5, 0);
    assert!(before > 0 && before < packets);
    let n = packets / 5 * 16;
    dev.control(wr(CMD_BULK_RESET, 0, 0), &[], &mut v);
    let mut window = (write_pos - n).to_le_bytes().to_vec();
    window.extend_from_slice(&n.to_le_bytes());
    spi_write(&mut dev, REG_BULK, &window, &mut v);
    for _ in 0..8 {
        dev.in_submitted(6, 512 * 1024);
    }
    assert_eq!(dev.transfer_in(6, 512 * 1024, &mut v), Reply::Nak, "not before bulk start");
    dev.control(wr(CMD_BULK_START, 0, 0), &[], &mut v);
    let d = data(dev.transfer_in(6, 512 * 1024, &mut v));
    assert_eq!(d.len(), n as usize);
    // The window starts 2700 samples before 3004: at 304, value 76,
    // which lasts until 307.
    assert_eq!(&d[..3], &[76, 0, 4], "value 76, four samples");
    assert_eq!(dev.transfer_in(6, 512 * 1024, &mut v), Reply::Nak, "nothing left");
    spi_write(&mut dev, REG_RUN, &[0], &mut v);
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn streams_the_enabled_channels() {
    let mut dev = device(&[("mcu", "1"), ("fpga", "1")]);
    let mut v = Violations::default();
    set_config(&mut dev, &mut v);
    setup(&mut dev, 1, &trigger(0b11, 0, 0, 0), &sampling(64, 0, 0x100, 200), &mut v);
    dev.control(wr(CMD_BULK_RESET, 0, 0), &[], &mut v);
    dev.in_submitted(6, 512 * 1024);
    spi_write(&mut dev, REG_RUN, &[3], &mut v);
    dev.control(wr(CMD_BULK_START, 0, 0), &[], &mut v);
    let d = data(dev.transfer_in(6, 512 * 1024, &mut v));
    assert_eq!(d.len(), STREAM_BYTES);
    assert_eq!(&d[..4], &[0xf0, 0xf0, 0x00, 0xff], "D0 then D1 over samples 0..16");
    assert_eq!(dev.transfer_in(6, 512 * 1024, &mut v), Reply::Nak, "one per submission");
    dev.in_submitted(6, 512 * 1024);
    assert_eq!(data(dev.transfer_in(6, 512 * 1024, &mut v)).len(), STREAM_BYTES);
    spi_write(&mut dev, REG_RUN, &[0], &mut v);
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn a_loaded_fpga_passes_the_driver_check() {
    let mut dev = device(&[("mcu", "1"), ("fpga", "1")]);
    let mut v = Violations::default();
    set_config(&mut dev, &mut v);
    assert_eq!(data(dev.control(rd(CMD_FPGA_INIT, 0, 1), &[], &mut v)), [0]);
    assert_eq!(spi_read(&mut dev, REG_RUN, 2, &mut v), [0xe1, 0x85]);
    assert_eq!(spi_read(&mut dev, REG_PWM_EN, 1, &mut v), [0]);
    // The dump at spew level reads the whole space.
    assert_eq!(spi_read(&mut dev, REG_RUN, 0x80, &mut v).len(), 0x80);
    assert!(v.is_empty(), "{v:?}");
    let mut dev = device(&[("mcu", "1"), ("fpga", "1"), ("pwm", "2")]);
    set_config(&mut dev, &mut v);
    assert_eq!(spi_read(&mut dev, REG_PWM_EN, 1, &mut v), [2], "PWM2 left running");
}

#[test]
fn flags_what_the_driver_never_sends() {
    let mut v = Violations::default();
    // The bare chip answers only its loader.
    let mut dev = device(&[]);
    set_config(&mut dev, &mut v);
    assert_eq!(dev.control(rd(CMD_EEPROM, 0x08, 8), &[], &mut v), Reply::Stall);
    assert_eq!(v.drain().len(), 1, "a request to the bare FX2");

    let mut dev = device(&[("mcu", "1")]);
    set_config(&mut dev, &mut v);
    dev.control(rd(CMD_EEPROM, 0x00, 8), &[], &mut v);
    assert_eq!(v.drain().len(), 1, "EEPROM read the driver does not make");
    assert_eq!(dev.control(wr(CMD_EEPROM, 0x08, 1), &[0], &mut v), Reply::Stall);
    assert_eq!(v.drain().len(), 1, "EEPROM write");
    assert_eq!(dev.control(rd(0x60, 0, 8), &[], &mut v), Reply::Stall);
    assert_eq!(v.drain().len(), 1, "the authentication request");
    dev.control(wr(0xa0, 0xe600, 1), &[1], &mut v);
    assert_eq!(v.drain().len(), 1, "firmware load to the running firmware");
    spi_write(&mut dev, REG_PWM_EN, &[0], &mut v);
    assert_eq!(v.drain().len(), 1, "register write before the FPGA is enabled");
    dev.control(wr(CMD_FPGA_ENABLE, 1, 0), &[], &mut v);
    assert_eq!(v.drain().len(), 1, "enable of a blank FPGA");
    assert_eq!(dev.transfer_out(2, &[0; 64], &mut v), Reply::Stall);
    assert_eq!(v.drain().len(), 1, "bitstream without FPGA init");
    // A wrong size, a wrong byte, no padding.
    let model = Kingst::new(PROFILES[0], true, false, Faults::default());
    let mut bitstream = model.bitstream();
    let mut dev = Device::new(Box::new(model));
    set_config(&mut dev, &mut v);
    dev.control(wr(CMD_FPGA_INIT, 0, 4), &1000u32.to_le_bytes(), &mut v);
    assert_eq!(v.drain().len(), 1, "announced size");
    bitstream[5000] ^= 1;
    for c in bitstream.chunks(4096) {
        dev.transfer_out(2, c, &mut v);
    }
    assert_eq!(data(dev.control(rd(CMD_FPGA_INIT, 0, 1), &[], &mut v)), [1]);
    assert_eq!(v.drain().len(), 3, "no padding (size, chunks), and a byte that differs");
    dev.transfer_out(2, &[0; 64], &mut v);
    assert_eq!(v.drain().len(), 1, "the failed upload left the FPGA blank");
    dev.control(wr(CMD_FPGA_INIT, 0, 4), &(bitstream.len() as u32).to_le_bytes(), &mut v);
    dev.transfer_out(2, &[0; 8192], &mut v);
    assert_eq!(v.drain().len(), 1, "a chunk above 4096");
}

#[test]
fn flags_bad_configurations() {
    let mut v = Violations::default();
    let mut dev = device(&[("mcu", "1"), ("fpga", "1")]);
    set_config(&mut dev, &mut v);
    spi_write(&mut dev, REG_THRESHOLD, &[0x10, 0x02, 0xf2, 0x00], &mut v);
    assert_eq!(v.drain().len(), 1, "a threshold of no listed voltage");
    spi_write(&mut dev, REG_CAPT_MODE, &[2], &mut v);
    assert_eq!(v.drain().len(), 1, "capture mode 2");
    spi_write(&mut dev, REG_CAPT_MODE, &[0], &mut v);
    spi_write(&mut dev, REG_SAMPLING, &sampling(100, 0, 0x100, 200), &mut v);
    assert_eq!(v.drain().len(), 1, "sample config before the trigger config");
    spi_write(&mut dev, REG_TRIGGER, &trigger(0, 0, 0, 0), &mut v);
    assert_eq!(v.drain().len(), 1, "no channel");
    spi_write(&mut dev, REG_TRIGGER, &trigger(0x1ffff, 3, 0, 0), &mut v);
    assert_eq!(v.drain().len(), 2, "a channel beyond 16, two edges");
    spi_write(&mut dev, REG_TRIGGER, &trigger(0xffff, 1, 2, 0), &mut v);
    assert_eq!(v.drain().len(), 1, "a level outside the trigger channels");
    let t = trigger(0xffff, 1, 0, 0);
    let mem = 1024 * 1024 * 1024 / 8 * 50 / 100;
    spi_write(&mut dev, REG_TRIGGER, &t, &mut v);
    spi_write(&mut dev, REG_SAMPLING, &sampling(100, 50, mem, 0), &mut v);
    assert_eq!(v.drain().len(), 1, "a divider of 0");
    spi_write(&mut dev, REG_SAMPLING, &sampling(100, 50, mem, 66), &mut v);
    assert!(v.is_empty(), "3 MHz is no rate of the list, but the driver sends it: {v:?}");
    spi_write(&mut dev, REG_SAMPLING, &sampling(100, 20, mem, 200), &mut v);
    assert_eq!(v.drain().len(), 1, "samples and memory of different ratios");
    spi_write(&mut dev, REG_TRIGGER, &trigger(0xffff, 0, 0, 0), &mut v);
    spi_write(&mut dev, REG_SAMPLING, &sampling(100, 50, 0x100, 200), &mut v);
    assert_eq!(v.drain().len(), 1, "pre-trigger samples without a trigger");
    spi_write(&mut dev, REG_SAMPLING, &sampling(0, 0, 0x100, 200), &mut v);
    assert_eq!(v.drain().len(), 1, "no samples");
    spi_write(&mut dev, REG_RUN, &[3], &mut v);
    assert!(v.is_empty(), "a full config: {v:?}");
    spi_write(&mut dev, REG_TRIGGER, &trigger(0xffff, 0, 0, 0), &mut v);
    assert_eq!(v.drain().len(), 1, "config written while capturing");
    spi_write(&mut dev, REG_RUN, &[3], &mut v);
    assert_eq!(v.drain().len(), 1, "run while running");
    spi_write(&mut dev, REG_RUN, &[0], &mut v);
    spi_write(&mut dev, REG_RUN, &[3], &mut v);
    assert_eq!(v.drain().len(), 1, "run without a fresh config");
    spi_write(&mut dev, REG_RUN, &[7], &mut v);
    assert_eq!(v.drain().len(), 1, "run mode 7");
    // The download window must be the capture info's.
    spi_write(&mut dev, REG_CAPT_MODE, &[0], &mut v);
    spi_write(&mut dev, REG_TRIGGER, &trigger(0xffff, 0, 0, 0), &mut v);
    spi_write(&mut dev, REG_SAMPLING, &sampling(100, 0, 0x100, 200), &mut v);
    spi_write(&mut dev, REG_RUN, &[3], &mut v);
    spi_read(&mut dev, REG_RUN, 2, &mut v);
    spi_read(&mut dev, REG_RUN, 2, &mut v);
    let mut window = CAPTURE_BASE.to_le_bytes().to_vec();
    window.extend_from_slice(&16u32.to_le_bytes());
    spi_write(&mut dev, REG_BULK, &window, &mut v);
    assert_eq!(v.drain().len(), 1, "a window other than the capture's");
    dev.control(wr(CMD_BULK_RESET, 0, 0), &[], &mut v);
    dev.control(wr(CMD_BULK_START, 0, 0), &[], &mut v);
    assert_eq!(v.drain().len(), 1, "bulk start without a window");
    // PWM.
    spi_write(&mut dev, REG_PWM_EN, &[4], &mut v);
    assert_eq!(v.drain().len(), 1, "a third PWM output");
    spi_write(&mut dev, REG_PWM_EN, &[1], &mut v);
    let pwm = |period: u32, duty: u32| [period.to_le_bytes(), duty.to_le_bytes()].concat();
    spi_write(&mut dev, REG_PWM1, &pwm(200, 100), &mut v);
    assert_eq!(v.drain().len(), 1, "PWM1 changed while enabled");
    spi_write(&mut dev, REG_PWM2, &pwm(5, 6), &mut v);
    assert_eq!(v.drain().len(), 2, "above 20 MHz, duty above the period");
    spi_write(&mut dev, 0x40, &[0], &mut v);
    assert_eq!(v.drain().len(), 1, "a register the driver never writes");
    spi_read(&mut dev, 0x04, 2, &mut v);
    assert_eq!(v.drain().len(), 1, "a register read the driver never makes");
    dev.control(vendor(0x40, CMD_FPGA_SPI, u16::from(REG_TRIGGER), 0, 16), &t, &mut v);
    assert_eq!(v.drain().len(), 1, "the trigger config without its wIndex 16");
    dev.transfer_in(6, 500, &mut v);
    assert!(v.is_empty(), "no transfer checked before a bulk start");
    // A memory-less device streams.
    let mut dev = device(&[("mcu", "1"), ("fpga", "1"), ("model", "la1010a1")]);
    set_config(&mut dev, &mut v);
    spi_write(&mut dev, REG_CAPT_MODE, &[0], &mut v);
    assert_eq!(v.drain().len(), 1, "capture to memory without memory");
    spi_write(&mut dev, REG_CAPT_MODE, &[1], &mut v);
    spi_write(&mut dev, REG_TRIGGER, &trigger(0xffff, 1, 0, 0), &mut v);
    assert_eq!(v.drain().len(), 1, "a hardware trigger in streaming mode");
}

#[test]
fn faults_stall_requests_and_change_answers() {
    let mut v = Violations::default();
    let mut dev = device(&[("mcu", "1"), ("stall", "eeprom@2,read:0x00")]);
    set_config(&mut dev, &mut v);
    assert_ne!(dev.control(rd(CMD_EEPROM, 0x20, 4), &[], &mut v), Reply::Stall);
    assert_eq!(dev.control(rd(CMD_EEPROM, 0x08, 8), &[], &mut v), Reply::Stall, "the second");
    assert_eq!(dev.control(rd(CMD_FPGA_SPI, 0, 2), &[], &mut v), Reply::Stall);
    let mut dev = device(&[
        ("mcu", "1"),
        ("eeprom-id", "0102030405060708"),
        ("date", "21130000"),
        ("fpga", "1"),
        ("run-state", "0x1234"),
    ]);
    set_config(&mut dev, &mut v);
    assert_eq!(data(dev.control(rd(CMD_EEPROM, 0x08, 8), &[], &mut v)), [1, 2, 3, 4, 5, 6, 7, 8]);
    assert_eq!(data(dev.control(rd(CMD_EEPROM, 0x20, 4), &[], &mut v)), [0x21, 0x13, 0, 0]);
    assert_eq!(spi_read(&mut dev, REG_RUN, 2, &mut v), [0x34, 0x12]);
    // A short first chunk and another answer after the upload.
    let model = build(&[
        ("mcu".into(), "1".into()),
        ("short-bitstream".into(), "1".into()),
        ("fpga-init".into(), "5".into()),
    ])
    .unwrap();
    let mut dev = Device::new(model);
    set_config(&mut dev, &mut v);
    let size = (PROFILES[0].bitstream_size as u32).to_le_bytes();
    dev.control(wr(CMD_FPGA_INIT, 0, 4), &size, &mut v);
    assert_eq!(dev.transfer_out(2, &[0; 4096], &mut v), Reply::Data(vec![0; 2048]));
    assert!(v.is_empty(), "a fault is the device's, not the host's: {v:?}");
    let mut dev = device(&[("mcu", "1"), ("fpga-init", "5")]);
    set_config(&mut dev, &mut v);
    let bitstream = Kingst::new(PROFILES[0], true, false, Faults::default()).bitstream();
    dev.control(wr(CMD_FPGA_INIT, 0, 4), &size, &mut v);
    for c in bitstream.chunks(4096) {
        dev.transfer_out(2, c, &mut v);
    }
    let tail = bitstream.len().div_ceil(4096) * 4096 - bitstream.len();
    dev.transfer_out(2, &vec![0; tail], &mut v);
    assert_eq!(data(dev.control(rd(CMD_FPGA_INIT, 0, 1), &[], &mut v)), [5]);
    assert!(v.is_empty(), "{v:?}");
    for bad in [
        ("stall", "reset"),
        ("stall", "read:zz"),
        ("stall", "eeprom@x"),
        ("pwm", "4"),
        ("eeprom-id", "01"),
        ("run-state", "x"),
        ("model", "la9999"),
        ("fpga", "1"),
    ] {
        assert!(build(&[(bad.0.into(), bad.1.into())]).is_err(), "{bad:?}");
    }
}

#[test]
fn faults_empty_then_stalled_transfers_paced_by_submissions() {
    let mut v = Violations::default();
    let mut dev = device(&[("mcu", "1"), ("fpga", "1"), ("empty", "1"), ("stall-in", "1")]);
    set_config(&mut dev, &mut v);
    setup(&mut dev, 1, &trigger(1, 0, 0, 0), &sampling(64, 0, 0x100, 200), &mut v);
    dev.control(wr(CMD_BULK_RESET, 0, 0), &[], &mut v);
    spi_write(&mut dev, REG_RUN, &[3], &mut v);
    dev.control(wr(CMD_BULK_START, 0, 0), &[], &mut v);
    let mut got = Vec::new();
    for _ in 0..3 {
        got.push(dev.transfer_in(6, 512, &mut v));
        assert_eq!(dev.transfer_in(6, 512, &mut v), Reply::Nak, "one per submission");
        dev.in_submitted(6, 512);
    }
    assert_eq!(got[..2], [Reply::Data(vec![]), Reply::Stall]);
    assert_eq!(data(got[2].clone()).len(), 512);
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn a_wide_device_lays_out_six_packets_and_two_sequence_bytes() {
    let mut v = Violations::default();
    let mut dev = device(&[("mcu", "1"), ("fpga", "1"), ("model", "la5032a0")]);
    set_config(&mut dev, &mut v);
    // 1 MHz from 800 MHz: divider 800.
    setup(&mut dev, 0, &trigger(u32::MAX, 0, 0, 0), &sampling(10, 0, 0x100, 800), &mut v);
    spi_write(&mut dev, REG_RUN, &[3], &mut v);
    spi_read(&mut dev, REG_RUN, 2, &mut v);
    assert_eq!(spi_read(&mut dev, REG_RUN, 2, &mut v), RUN_DONE.to_le_bytes());
    let info = spi_read(&mut dev, REG_SAMPLING, 16, &mut v);
    assert_eq!(info[0], 6, "three runs padded to six packets");
    let write_pos = u32::from_le_bytes([info[8], info[9], info[10], info[11]]);
    assert_eq!(write_pos, CAPTURE_BASE + 32);
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn faults_of_the_loader_the_states_and_the_download() {
    let mut v = Violations::default();
    // The second loader request STALLs; a dead firmware comes back bare.
    let mut dev = device(&[("stall", "loader@2"), ("fw-dead", "1")]);
    set_config(&mut dev, &mut v);
    assert_eq!(dev.control(wr(0xa0, 0xe600, 1), &[1], &mut v), Reply::ack());
    assert_eq!(dev.control(wr(0xa0, 0, 16), &[0; 16], &mut v), Reply::Stall);
    let image = Kingst::mcu_image();
    for (i, c) in image.chunks(4096).enumerate() {
        dev.control(wr(0xa0, (i * 4096) as u16, c.len() as u16), c, &mut v);
    }
    dev.control(wr(0xa0, 0xe600, 1), &[0], &mut v);
    assert!(dev.take_renumeration());
    dev.reset();
    assert_eq!(dev.descriptors().device[15], 0, "still the bare FX2");
    // Three samples more; four bytes more.
    let mut dev =
        device(&[("mcu", "1"), ("fpga", "1"), ("more-samples", "3"), ("extra-bytes", "4")]);
    set_config(&mut dev, &mut v);
    setup(&mut dev, 0, &trigger(0xffff, 0, 0, 0), &sampling(2, 0, 0x100, 200), &mut v);
    spi_write(&mut dev, REG_RUN, &[3], &mut v);
    assert_eq!(spi_read(&mut dev, REG_RUN, 2, &mut v)[0], 0xed, "done in one poll");
    let info = spi_read(&mut dev, REG_SAMPLING, 16, &mut v);
    let write_pos = u32::from_le_bytes([info[8], info[9], info[10], info[11]]);
    let mut window = (write_pos - 16).to_le_bytes().to_vec();
    window.extend_from_slice(&16u32.to_le_bytes());
    spi_write(&mut dev, REG_BULK, &window, &mut v);
    dev.control(wr(CMD_BULK_START, 0, 0), &[], &mut v);
    let d = data(dev.transfer_in(6, 512 * 1024, &mut v));
    assert_eq!(d.len(), 20);
    assert_eq!(&d[..3], &[0, 0, 4], "five samples, not two");
    assert!(v.is_empty(), "{v:?}");
    for bad in [("fw-dead", "2"), ("stall", "loader@0x")] {
        assert!(build(&[(bad.0.into(), bad.1.into())]).is_err(), "{bad:?}");
    }
}

#[test]
fn a_capture_samples_on_by_the_polls_and_a_halt_keeps_what_it_has() {
    let mut v = Violations::default();
    let info = |dev: &mut Device, v: &mut Violations| {
        let b = spi_read(dev, REG_SAMPLING, 16, v);
        let w = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        (w(0), w(4))
    };
    // 1 M samples at 20 kHz, halted after two polls: 2000 samples.
    let mut dev = device(&[("mcu", "1"), ("fpga", "1")]);
    set_config(&mut dev, &mut v);
    setup(&mut dev, 0, &trigger(0xffff, 0, 0, 0), &sampling(1_000_000, 0, 0x100, 10000), &mut v);
    spi_write(&mut dev, REG_RUN, &[3], &mut v);
    assert_eq!(spi_read(&mut dev, REG_RUN, 2, &mut v), RUN_POST.to_le_bytes(), "no pre-trigger");
    spi_read(&mut dev, REG_RUN, 2, &mut v);
    spi_write(&mut dev, REG_RUN, &[0], &mut v);
    assert_eq!(spi_read(&mut dev, REG_RUN, 2, &mut v), RUN_DONE.to_le_bytes());
    let want = capture::capture(0, 2000, None, Layout { wide: false });
    assert_eq!(info(&mut dev, &mut v), (want.packets, 0));
    // A trigger on CH15 does not fire for 128000 samples: halted while
    // waiting, the memory holds the last 100 samples, all before it.
    setup(
        &mut dev,
        0,
        &trigger(0xffff, 0x8000, 0x8000, 0x8000),
        &sampling(100, 50, 1024 * 1024 * 1024 / 8 / 2, 10000),
        &mut v,
    );
    spi_write(&mut dev, REG_RUN, &[3], &mut v);
    assert_eq!(spi_read(&mut dev, REG_RUN, 2, &mut v), RUN_WAIT.to_le_bytes());
    spi_write(&mut dev, REG_RUN, &[0], &mut v);
    spi_read(&mut dev, REG_RUN, 2, &mut v);
    let want = capture::capture(900, 100, Some(100), Layout { wide: false });
    assert_eq!(info(&mut dev, &mut v), (want.packets, want.before_trigger));
    assert_eq!(want.before_trigger, 1, "one run of 0x5a, before the padding");
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn flags_bitstream_chunks_and_early_register_reads() {
    let mut v = Violations::default();
    let model = Kingst::new(PROFILES[0], true, false, Faults::default());
    let bitstream = model.bitstream();
    let mut dev = Device::new(Box::new(model));
    set_config(&mut dev, &mut v);
    spi_read(&mut dev, REG_RUN, 0x80, &mut v);
    assert!(v.is_empty(), "the dump of the spew level is the driver's habit");
    spi_read(&mut dev, REG_RUN, 2, &mut v);
    assert_eq!(v.drain().len(), 1, "a register read before the FPGA is enabled");
    // The right bytes, in chunks of 2048.
    let mut padded = bitstream.clone();
    padded.resize(bitstream.len().div_ceil(4096) * 4096, 0);
    dev.control(wr(CMD_FPGA_INIT, 0, 4), &(bitstream.len() as u32).to_le_bytes(), &mut v);
    for c in padded.chunks(2048) {
        dev.transfer_out(2, c, &mut v);
    }
    assert_eq!(data(dev.control(rd(CMD_FPGA_INIT, 0, 1), &[], &mut v)), [1]);
    assert_eq!(v.drain().len(), 1, "chunks of 2048");
}

/// A stream started, one transfer submitted.
fn stream_started(options: &[(&str, &str)], v: &mut Violations) -> Device {
    let mut dev = device(options);
    set_config(&mut dev, v);
    setup(&mut dev, 1, &trigger(1, 0, 0, 0), &sampling(64, 0, 0x100, 200), v);
    dev.control(wr(CMD_BULK_RESET, 0, 0), &[], v);
    spi_write(&mut dev, REG_RUN, &[3], v);
    dev.in_submitted(6, 512);
    dev.control(wr(CMD_BULK_START, 0, 0), &[], v);
    dev
}

#[test]
fn a_slow_transfer_holds_its_data_until_cancelled() {
    let mut v = Violations::default();
    let mut dev = stream_started(&[("mcu", "1"), ("fpga", "1"), ("slow-in", "1")], &mut v);
    assert_eq!(dev.transfer_in(6, 512, &mut v), Reply::Nak);
    assert_eq!(dev.in_unlinked(6, 512).map(|d| d.len()), Some(512));
    assert_eq!(dev.in_unlinked(6, 512), None, "one per submission");
    dev.in_submitted(6, 512);
    assert_eq!(dev.in_unlinked(6, 512).map(|d| d.len()), Some(512));
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn a_transfer_without_the_option_holds_nothing_when_cancelled() {
    let mut v = Violations::default();
    let mut dev = stream_started(&[("mcu", "1"), ("fpga", "1")], &mut v);
    assert_eq!(dev.in_unlinked(6, 512), None);
    assert_eq!(data(dev.transfer_in(6, 512, &mut v)).len(), 512);
}

#[test]
fn the_in_limit_holds_what_follows() {
    let mut v = Violations::default();
    let mut dev = stream_started(&[("mcu", "1"), ("fpga", "1"), ("in-limit", "1")], &mut v);
    assert_eq!(data(dev.transfer_in(6, 512, &mut v)).len(), 512);
    dev.in_submitted(6, 512);
    assert_eq!(dev.transfer_in(6, 512, &mut v), Reply::Nak);
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn the_first_poll_of_a_capture_is_the_event_point() {
    let mut v = Violations::default();
    let mut dev = device(&[("mcu", "1"), ("fpga", "1")]);
    set_config(&mut dev, &mut v);
    dev.arm(crate::model::Arm::parse("interrupt model").unwrap());
    let reply = Reply::ack();
    spi_read(&mut dev, REG_RUN, 2, &mut v);
    assert_eq!(dev.fired(0, true, &reply), None, "a run state outside a capture");
    setup(&mut dev, 0, &trigger(1, 0, 0, 0), &sampling(10_000_000, 0, 0x100, 10_000), &mut v);
    spi_write(&mut dev, REG_RUN, &[3], &mut v);
    assert_eq!(dev.fired(0, false, &reply), None);
    spi_read(&mut dev, REG_RUN, 2, &mut v);
    assert!(dev.fired(0, true, &reply).is_some());
    dev.arm(crate::model::Arm::parse("interrupt model").unwrap());
    spi_read(&mut dev, REG_RUN, 2, &mut v);
    assert_eq!(dev.fired(0, true, &reply), None, "once");
}
