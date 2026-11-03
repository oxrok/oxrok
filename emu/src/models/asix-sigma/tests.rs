//! The model driven as `asix-sigma` drives it, and one violation of
//! each kind.

use super::*;
use crate::shells::ftdi::libftdi_divisor;

fn out(request: u8, value: u16, index: u16) -> Setup {
    Setup { request_type: 0x40, request, value, index, length: 0 }
}

fn sigma(options: &[(&str, &str)]) -> Box<dyn DeviceModel> {
    let o: Vec<(String, String)> =
        options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
    build(&o).unwrap()
}

fn ctl(dev: &mut dyn DeviceModel, s: Setup, v: &mut Violations) {
    assert_eq!(dev.control(s, &[], v), Reply::ack(), "{s:?}");
}

/// libftdi's open, then `upload_firmware` up to the bitstream: bit-bang
/// with mask 0xdf at 750 kBd (times 4), the suicide sequence, the PROG
/// pulse, the purge, and the first read of the pins.
fn prepare(dev: &mut dyn DeviceModel, v: &mut Violations) -> Vec<u8> {
    let (value, bits, _) = libftdi_divisor(Chip::Bm, 9600);
    ctl(dev, out(0, 0, 1), v);
    ctl(dev, out(3, value, bits), v);
    ctl(dev, out(0x0b, 0x01df, 1), v);
    let (value, bits, _) = libftdi_divisor(Chip::Bm, 3_000_000);
    ctl(dev, out(3, value, bits), v);
    for _ in 0..4 {
        dev.out(2, &[0x84, 0x84, 0x88, 0x84, 0x88, 0x84, 0x88, 0x84], v);
    }
    dev.out(2, &[0x01, 0x03, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01], v);
    ctl(dev, out(0, 1, 1), v);
    ctl(dev, out(0, 2, 1), v);
    let Reply::Data(d) = dev.in_transfer(1, 4096, v) else { panic!() };
    d
}

/// `sigma_fw_2_bitbang`: two pin states per bit.
fn bitbang(bits: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    for &byte in bits {
        for i in (0..8).rev() {
            let d = if byte >> i & 1 != 0 { 0x40 } else { 0 };
            out.extend([d | 0x01, d]);
        }
    }
    out
}

/// The rest of the upload and `sigma_fpga_init_la`.
fn finish(dev: &mut dyn DeviceModel, name: &str, size: usize, v: &mut Violations) -> Vec<u8> {
    for chunk in bitbang(&netlist_bits(name, size)).chunks(4096) {
        dev.out(2, chunk, v);
    }
    ctl(dev, out(0x0b, 0, 1), v);
    ctl(dev, out(0, 1, 1), v);
    ctl(dev, out(0, 2, 1), v);
    assert_eq!(dev.in_transfer(1, 4096, v), Reply::Data(vec![1, 0x60]));
    dev.out(
        2,
        &[0x00, 0x10, 0x40, 0x0f, 0x10, 0x25, 0x35, 0x40, 0x0f, 0x10, 0x2a, 0x3a, 0x40, 0x03, 0x10,
            0x20, 0x38],
        v,
    );
    let Reply::Data(d) = dev.in_transfer(1, 4096, v) else { panic!() };
    d
}

#[test]
fn uploads_and_answers_as_the_driver_expects() {
    for &(name, size, netlist) in NETLISTS {
        let mut dev = Sigma::new(
            {
                let mut c = FtdiConfig::new(Chip::Bm);
                c.serial = Some("A6010A3C".into());
                c
            },
            BoardFaults::default(),
            2,
        );
        let mut v = Violations::default();
        let d = prepare(&mut dev, &mut v);
        // Status bytes, then the pins: CCLK as last written, INIT high.
        assert_eq!(&d[..4], &[0x01, 0x60, 0x21, 0x21]);
        assert_eq!(d.len(), 4096);
        assert_eq!(finish(&mut dev, name, size, &mut v), [0x01, 0x60, 0xa6, 0x55, 0xaa]);
        assert_eq!(dev.board().fpga().map(Fpga::netlist), Some(netlist));
        assert!(v.is_empty(), "{v:?}");
    }
}

#[test]
fn init_stays_low_while_clearing() {
    let mut dev = sigma(&[("init-busy", "2")]);
    let mut v = Violations::default();
    let d = prepare(dev.as_mut(), &mut v);
    assert_eq!(&d[2..4], &[0x01, 0x01]);
    let Reply::Data(d) = dev.in_transfer(1, 4096, &mut v) else { panic!() };
    assert_eq!(d[2], 0x01);
    let Reply::Data(d) = dev.in_transfer(1, 4096, &mut v) else { panic!() };
    assert_eq!(d[2], 0x21);
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn faults_of_the_fpga() {
    let mut v = Violations::default();
    let mut dev = sigma(&[("config", "fail")]);
    prepare(dev.as_mut(), &mut v);
    assert_eq!(finish(dev.as_mut(), "asix-sigma-50.fw", 2000, &mut v), [0x01, 0x60]);
    let mut dev = sigma(&[("id", "0xa5"), ("scratch", "stuck")]);
    prepare(dev.as_mut(), &mut v);
    assert_eq!(
        finish(dev.as_mut(), "asix-sigma-50.fw", 2000, &mut v),
        [0x01, 0x60, 0xa5, 0xff, 0xff]
    );
    let mut dev = sigma(&[("scratch", "bit0")]);
    prepare(dev.as_mut(), &mut v);
    assert_eq!(
        finish(dev.as_mut(), "asix-sigma-50.fw", 2000, &mut v),
        [0x01, 0x60, 0xa6, 0x55, 0xab]
    );
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn flags_a_wrong_upload() {
    let mut v = Violations::default();
    let mut dev = sigma(&[]);
    prepare(dev.as_mut(), &mut v);
    // A bitstream one byte short.
    for chunk in bitbang(&netlist_bits("asix-sigma-50.fw", 1999)).chunks(4096) {
        dev.out(2, chunk, &mut v);
    }
    ctl(dev.as_mut(), out(0x0b, 0, 1), &mut v);
    assert_eq!(v.drain().len(), 1, "a bitstream of no netlist");
    // DIN changing with CCLK, and a pin state the driver never writes.
    ctl(dev.as_mut(), out(0x0b, 0x01df, 1), &mut v);
    dev.out(2, &[0x41, 0x00], &mut v);
    assert_eq!(v.drain().len(), 1, "DIN at the edge");
    dev.out(2, &[0x20], &mut v);
    assert_eq!(v.drain().len(), 1, "a pin state");
    ctl(dev.as_mut(), out(0x0b, 0x01ff, 1), &mut v);
    assert_eq!(v.drain().len(), 1, "INIT driven");
    dev.control(out(0x0b, 0x0400, 1), &[], &mut v);
    // The shell flags synchronous bit-bang on a BM itself.
    assert!(!v.drain().is_empty(), "another bit mode");
}

#[test]
fn suicide_drops_the_netlist() {
    let mut v = Violations::default();
    let mut c = FtdiConfig::new(Chip::Bm);
    c.serial = Some("A6010A3C".into());
    let mut dev = Sigma::new(c, BoardFaults::default(), 2);
    prepare(&mut dev, &mut v);
    finish(&mut dev, "asix-sigma-100.fw", 2100, &mut v);
    assert!(dev.board().fpga().is_some());
    ctl(&mut dev, out(0x0b, 0x01df, 1), &mut v);
    dev.out(2, &[0x84, 0x84, 0x88, 0x84, 0x88, 0x84, 0x88, 0x84], &mut v);
    assert!(dev.board().fpga().is_none());
    assert!(v.is_empty(), "{v:?}");
}

#[test]
fn variants_and_options() {
    for (name, product, ..) in VARIANTS {
        let m = sigma(&[("variant", name)]);
        assert_eq!(m.descriptors().product_id(), *product);
        assert_eq!(m.descriptors().vendor_id(), VENDOR);
    }
    let m = sigma(&[]);
    assert_eq!(m.descriptors().bcd_device(), 0x0400);
    assert_eq!(m.descriptors().strings, ["ASIX", "SIGMA", "A6010A3C"]);
    let m = sigma(&[("serial", "")]);
    assert_eq!(m.descriptors().strings, ["ASIX", "SIGMA"]);
    let o = |k: &str, v: &str| vec![(k.to_owned(), v.to_owned())];
    assert!(build(&o("variant", "omega2")).is_err());
    assert!(build(&o("step", "0")).is_err());
    assert!(build(&o("scratch", "loose")).is_err());
    assert!(build(&o("speed", "1")).is_err());
}

#[test]
fn standin_sizes_match_the_firmware_list() {
    let list = include_str!("../../../../tools/emu/asix-sigma.firmware");
    let listed: Vec<(String, usize)> = list
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        .map(|l| {
            let mut w = l.split_whitespace();
            (w.next().unwrap().to_owned(), w.next().unwrap().parse().unwrap())
        })
        .collect();
    let ours: Vec<(String, usize)> =
        NETLISTS.iter().map(|&(n, s, _)| (n.to_owned(), s)).collect();
    assert_eq!(listed, ours);
}
