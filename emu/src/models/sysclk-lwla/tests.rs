//! The model driven as `sysclk-lwla` drives it, and one violation of
//! each kind.

use super::*;
use crate::device::Device;

const SET_CONFIG: Setup = Setup { request_type: 0, request: 9, value: 1, index: 0, length: 0 };

fn opts(o: &[(&str, &str)]) -> Vec<(String, String)> {
    o.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
}

/// The host side: the driver's command words and reads.
struct Host {
    dev: Device,
    v: Violations,
}

impl Host {
    fn new(o: &[(&str, &str)]) -> Host {
        let mut dev = Device::new(build(&opts(o)).unwrap());
        let mut v = Violations::default();
        dev.control(SET_CONFIG, &[], &mut v);
        Host { dev, v }
    }

    fn violations(&mut self) -> Vec<String> {
        self.v.drain()
    }

    fn clean(&mut self) {
        let v = self.violations();
        assert!(v.is_empty(), "violations: {v:?}");
    }

    fn send(&mut self, words: &[u16]) -> Reply {
        let data: Vec<u8> = words.iter().flat_map(|w| w.to_le_bytes()).collect();
        self.dev.transfer_out(EP_COMMAND, &data, &mut self.v)
    }

    fn receive(&mut self, max: usize) -> Reply {
        self.dev.transfer_in(EP_REPLY, max, &mut self.v)
    }

    fn data(&mut self, max: usize) -> Vec<u8> {
        match self.receive(max) {
            Reply::Data(d) => d,
            other => panic!("expected data, got {other:?}"),
        }
    }

    fn bitstream(&mut self, name: &str) -> Reply {
        let (_, size) = BITSTREAMS.iter().find(|(n, _)| *n == name).unwrap();
        let body = standin(name, *size);
        let mut data = u32::try_from(body.len() + 4).unwrap().to_be_bytes().to_vec();
        data.extend_from_slice(&body);
        self.dev.transfer_out(EP_CONFIG, &data, &mut self.v)
    }

    fn write_reg(&mut self, reg: u16, value: u32) {
        assert_eq!(self.send(&[2, reg, (value >> 16) as u16, value as u16]), Reply::ack());
    }

    fn read_reg(&mut self, reg: u16) -> u32 {
        assert_eq!(self.send(&[1, reg]), Reply::ack());
        let d = self.data(512);
        assert_eq!(d.len(), 4);
        lwla_u32(&d)
    }

    fn read_long(&mut self, addr: u32) -> u64 {
        self.write_reg(R34_LONG_ADDR, addr);
        self.read_reg(R34_LONG_STROBE);
        let high = self.read_reg(R34_LONG_HIGH);
        let low = self.read_reg(R34_LONG_LOW);
        u64::from(high) << 32 | u64::from(low)
    }

    fn write_long(&mut self, addr: u32, value: u64) {
        self.write_reg(R34_LONG_ADDR, addr);
        self.write_reg(R34_LONG_LOW, value as u32);
        self.write_reg(R34_LONG_HIGH, (value >> 32) as u32);
        self.write_reg(R34_LONG_STROBE, 0);
    }

    fn read_lregs(&mut self, first: u16, count: u16) -> Vec<u64> {
        assert_eq!(self.send(&[8, first, count]), Reply::ack());
        let d = self.data(1024);
        d.chunks(8).map(|c| u64::from(lwla_u32(&c[4..])) << 32 | u64::from(lwla_u32(&c[..4]))).collect()
    }

    /// `dev_open()` of the LWLA1034: bitstream, two test reads, the
    /// quirk check.
    fn open34(&mut self) {
        assert_eq!(self.bitstream("sysclk-lwla1034-int.rbf"), Reply::ack());
        assert_eq!(self.read_long(L_TEST_ID), 0, "the first read is to be ignored");
        assert_eq!(self.read_long(L_TEST_ID), TEST_ID_1034);
        assert_eq!(self.read_lregs(0, 10).len(), 10);
        self.clean();
    }

    /// `setup_acquisition()` and the start of the LWLA1034.
    fn start34(&mut self, mask: u64, div: u64, boost: u32, trigger: [u64; 3]) {
        for (reg, value) in [
            (R34_MEM_CTRL, 2),
            (R34_MEM_CTRL, 1),
            (R34_LONG_ADDR, 10),
            (R34_LONG_LOW, 0x74),
            (R34_LONG_HIGH, 0),
            (R34_LONG_STROBE, 0),
            (R34_CLK_BOOST, boost),
        ] {
            self.write_reg(reg, value);
        }
        let mut w = vec![7u16, 0, 10];
        for value in [mask, div, trigger[0], trigger[1], trigger[2], u64::from(DEPTH - 16), 0, 0, 0, 0]
        {
            w.extend_from_slice(&[(value >> 16) as u16, value as u16, (value >> 48) as u16, (value >> 32) as u16]);
        }
        assert_eq!(self.send(&w), Reply::ack());
        self.write_long(10, 1);
        self.clean();
    }

    /// The stop and the readout of the LWLA1034: the memory words.
    fn stop_and_read34(&mut self, stop: bool) -> Vec<u64> {
        if stop {
            self.write_long(10, 0);
            self.write_reg(R34_CLK_BOOST, 0);
        }
        let fill = self.read_reg(R34_MEM_FILL);
        self.write_reg(R34_CLK_BOOST, 1);
        self.write_reg(R34_MEM_CTRL, 2);
        self.write_reg(R34_MEM_START, 4);
        let mut words = Vec::new();
        let mut at = 4u32;
        while at < fill {
            let count = (fill - at).div_ceil(8).min(28) * 8;
            assert_eq!(self.send(&[6, (at >> 16) as u16, at as u16, 0, count as u16]), Reply::ack());
            let d = self.data(1024);
            assert_eq!(d.len(), count as usize / 8 * 36);
            for slice in d.chunks(36) {
                let high = lwla_u32(&slice[32..]);
                for i in 0..8 {
                    let low = u64::from(lwla_u32(&slice[4 * i..]));
                    words.push(low | u64::from(high >> (28 - 4 * i) & 0xf) << 32);
                }
            }
            at += count;
        }
        words.truncate((fill - 4) as usize);
        self.write_reg(R34_CLK_BOOST, 0);
        self.clean();
        words
    }

    /// `dev_open()` of the LWLA1016.
    fn open16(&mut self, design: &str) {
        assert_eq!(self.bitstream(design), Reply::ack());
        assert_eq!(self.read_reg(R16_TEST_ID), 0);
        assert_eq!(self.read_reg(R16_TEST_ID), TEST_ID_1016);
        self.write_reg(R16_MEM_CTRL, 1);
        self.write_reg(R16_MEM_CTRL, 0);
        for at in [0u16, 24] {
            assert_eq!(self.send(&[3, 0, at, 0, 24]), Reply::ack());
            assert_eq!(self.data(512).len(), 96);
        }
        self.clean();
    }

    fn start16(&mut self, mask: u32, div: u32, trigger: [u32; 3]) {
        self.write_reg(R16_CHAN_MASK, mask);
        self.write_reg(R16_DIV_COUNT, div);
        for (reg, value) in [
            (R16_CAP_CTRL, 0),
            (R16_DURATION, 0),
            (R16_MEM_CTRL, 1),
            (R16_MEM_CTRL, 0),
            (R16_MEM_CTRL, 2),
            (R16_CAP_CTRL, 3),
            (R16_CAP_CTRL, 0x10),
            (R16_CAP_CTRL, 0),
            (R16_CAP_COUNT, DEPTH - 5),
        ] {
            self.write_reg(reg, value);
        }
        self.write_reg(R16_TEST_ID, trigger[2] << 16 | trigger[1]);
        self.write_reg(R16_CAP_CTRL, CAP16_TRG_EN | trigger[0] << 16);
        self.clean();
    }

    /// One status poll of the LWLA1016: status, fill, duration.
    fn poll16(&mut self) -> (u32, u32, u32) {
        (self.read_reg(R16_CAP_CTRL), self.read_reg(R16_MEM_WR_PTR), self.read_reg(R16_DURATION))
    }

    fn stop_and_read16(&mut self) -> Vec<u32> {
        self.write_reg(R16_CAP_CTRL, 0);
        self.write_reg(R16_DIV_COUNT, 0);
        let stop = self.read_reg(R16_CAP_COUNT) + 1;
        self.write_reg(R16_MEM_CTRL, 0);
        let mut words = Vec::new();
        let mut at = 2u32;
        while at < stop {
            let count = (stop - at).min(250);
            assert_eq!(self.send(&[3, 0, at as u16, 0, count as u16]), Reply::ack());
            let d = self.data(1024);
            assert_eq!(d.len(), count as usize * 4);
            words.extend(d.chunks(4).map(lwla_u32));
            at += count;
        }
        self.write_reg(R16_MEM_CTRL, 1);
        self.write_reg(R16_MEM_CTRL, 0);
        self.clean();
        words
    }
}

/// `LWLA_TO_UINT32` on the first four bytes.
fn lwla_u32(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], b[3]]).rotate_left(16)
}

/// `read_response()` of `lwla1034.c`: the samples of the words.
fn decode36(words: &[u64]) -> Vec<u64> {
    let mut out = Vec::new();
    let mut it = words.iter();
    while let Some(&w) = it.next() {
        let mut run = (w >> 34 & 1) + 1;
        if w & 1 << 35 != 0 {
            run += it.next().unwrap() << 1;
        }
        for _ in 0..run {
            out.push(w & ((1 << 34) - 1));
        }
    }
    out
}

/// The samples the words decode to, without expanding them.
fn run_total36(words: &[u64]) -> u64 {
    let mut total = 0;
    let mut it = words.iter();
    while let Some(&w) = it.next() {
        total += (w >> 34 & 1) + 1;
        if w & 1 << 35 != 0 {
            total += it.next().unwrap() << 1;
        }
    }
    total
}

fn signal(channels: u32) -> Signal {
    Signal { period: 1, ext_trg: 1000, ext_clock: 1_000_000, tick: 100, channels }
}

#[test]
fn builds_both_boards_and_refuses_bad_options() {
    let lwla = build(&[]).unwrap();
    assert_eq!(lwla.descriptors().product_id(), PID_1034);
    let lwla = build(&opts(&[("model", "1016")])).unwrap();
    assert_eq!(lwla.descriptors().product_id(), PID_1016);
    assert_eq!(lwla.descriptors().vendor_id(), VID);
    for (k, v) in [
        ("model", "1032"),
        ("period", "0"),
        ("tick", "x"),
        ("stall", "read-everything"),
        ("stall_reply", "write-reg"),
        ("short_reply", "bitstream"),
        ("stall", "read-reg:0"),
        ("halt_in", "2"),
        ("stale", "5000"),
        ("colour", "red"),
    ] {
        assert!(build(&opts(&[(k, v)])).is_err(), "{k}={v} taken");
    }
}

#[test]
fn standin_sizes_match_the_firmware_list() {
    let list = include_str!("../../../../tools/emu/sysclk-lwla.firmware");
    let listed: Vec<(String, usize)> = list
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        .map(|l| {
            let mut w = l.split_whitespace();
            (w.next().unwrap().to_owned(), w.next().unwrap().parse().unwrap())
        })
        .collect();
    let ours: Vec<(String, usize)> = BITSTREAMS.iter().map(|&(n, s)| (n.to_owned(), s)).collect();
    assert_eq!(listed, ours);
}

#[test]
fn encodes_runs_the_driver_decodes() {
    let sig = signal(34);
    // Every channel: runs of one sample, one word each.
    let (words, end, full) = encode(&sig, Format::Rle36, Board::Lwla1034.all(), 0, 100, 1000);
    assert_eq!((words.len(), end, full), (100, 100, false));
    assert_eq!(decode36(&words), (0..100).collect::<Vec<u64>>());
    // Channel 2 alone: runs of two, the run bit; channel 3: runs of 4,
    // a length word.
    for (mask, len, per_run) in [(0b10u64, 2u64, 1usize), (0b100, 4, 2), (0b1000, 8, 2)] {
        let (words, _, _) = encode(&sig, Format::Rle36, mask, 0, 64, 1000);
        assert_eq!(words.len(), 64 / len as usize * per_run, "mask {mask:#b}");
        let want: Vec<u64> = (0..64).map(|s| s & mask).collect();
        assert_eq!(decode36(&words), want);
    }
    // A run longer than one pair holds is split.
    let (words, _, _) = encode(&sig, Format::Rle36, 0, 0, RUN36_MAX + 5, 1000);
    // (Counted, not expanded: 2^37 samples do not fit in memory.)
    assert_eq!(words.len(), 4);
    assert_eq!(run_total36(&words), RUN36_MAX + 5);
    // The memory runs full: the words stop at the limit, the end sample
    // is the first one not stored.
    let (words, end, full) = encode(&sig, Format::Rle36, 0b100, 0, 1000, 9);
    assert_eq!((words.len(), end, full), (8, 16, true));
    // The LWLA1016: two samples a word, low half first; or runs of up
    // to 65536.
    let sig = signal(16);
    let (words, end, _) = encode(&sig, Format::Plain16, 0xffff, 10, 15, 100);
    assert_eq!((words, end), (vec![11 << 16 | 10, 13 << 16 | 12], 14));
    let (words, _, _) = encode(&sig, Format::Rle16, 0x8000, 0, 70_000, 100);
    assert_eq!(words, vec![0x7fff << 16, 0x7fff << 16 | 0x8000, (4464 - 1) << 16]);
    let (words, _, _) = encode(&sig, Format::Rle16, 0, 0, 70_000, 100);
    assert_eq!(words, vec![0xffff << 16, (4464 - 1) << 16]);
}

#[test]
fn triggers_on_levels_edges_and_the_trg_input() {
    let sig = signal(34);
    let level = Trigger { mask: 0b100, values: 0b100, ..Trigger::default() };
    assert_eq!(level.search(&sig, 0, 100), Some(4));
    let rising = Trigger { mask: 0b1000, values: 0b1000, edges: 0b1000, ..Trigger::default() };
    assert_eq!(rising.search(&sig, 0, 100), Some(8));
    assert_eq!(rising.search(&sig, 9, 100), Some(24));
    let falling = Trigger { mask: 0b10, values: 0, edges: 0b10, ..Trigger::default() };
    assert_eq!(falling.search(&sig, 0, 100), Some(4));
    // Two rising edges at once never come.
    let never = Trigger { mask: 0b11, values: 0b11, edges: 0b11, ..Trigger::default() };
    assert_eq!(never.search(&sig, 0, 100_000), None);
    let trg = Trigger { trg_rise: true, ..Trigger::default() };
    assert_eq!(trg.search(&sig, 0, 5000), Some(1000));
    let trg = Trigger { trg_fall: true, ..Trigger::default() };
    assert_eq!(trg.search(&sig, 0, 5000), Some(2000));
    assert_eq!(Trigger::default().search(&sig, 0, 10), Some(0));
    assert_eq!(Trigger::default().search(&sig, 10, 10), None);
}

#[test]
fn lwla1034_opens_captures_and_reads_out() {
    let mut h = Host::new(&[]);
    h.open34();
    // 1 MHz, channels 1 to 4, no trigger.
    h.start34(0xf, 99, 0, [0; 3]);
    let st = h.read_lregs(5, 5);
    assert_eq!(st[4], ST34_CAPTURING | ST34_TRIGGERED | ST34_MEM_AVAIL);
    assert_eq!(st[2], 100, "100 ms per poll");
    // runs of one sample: a word each, from address 4.
    assert_eq!(st[0], 4 + 100_000);
    let words = h.stop_and_read34(true);
    let samples = decode36(&words);
    assert_eq!(samples.len(), 100_000);
    assert!(samples.iter().enumerate().all(|(s, &v)| v == s as u64 & 0xf));
}

#[test]
fn lwla1034_boost_runs_the_clock_fast_and_fills_the_memory() {
    let mut h = Host::new(&[]);
    h.open34();
    // 125 MHz: 100 ms fill the memory; the counter reads 0.8 ms units.
    h.start34(Board::Lwla1034.all(), 0, 1, [0; 3]);
    let st = h.read_lregs(5, 5);
    assert_eq!(st[4], ST34_TRIGGERED, "stopped, memory full");
    assert_eq!(st[0], u64::from(DEPTH - 16));
    let words = (DEPTH - 20) as u64;
    assert_eq!(st[2], words * 1250 / 125_000_000);
    let read = h.stop_and_read34(false);
    assert_eq!(read.len() as u64, words);
}

#[test]
fn lwla1034_triggers_late_and_on_the_trg_input() {
    let mut h = Host::new(&[("tick", "1")]);
    h.open34();
    // Channel 5 rising: sample 16 at 10 kHz is 1.6 ms in.
    h.start34(0xff, 9999, 0, [1 << 4, 1 << 4, 1 << 4]);
    assert_eq!(h.read_lregs(5, 5)[4], ST34_CAPTURING | ST34_MEM_AVAIL);
    assert_eq!(h.read_lregs(5, 5)[4], ST34_CAPTURING | ST34_TRIGGERED | ST34_MEM_AVAIL);
    let samples = decode36(&h.stop_and_read34(true));
    assert_eq!(samples, (16..20).collect::<Vec<u64>>());
    // The TRG input's rising edge, on the external clock.
    let mut h = Host::new(&[("ext_trg", "50")]);
    assert_eq!(h.bitstream("sysclk-lwla1034-extpos.rbf"), Reply::ack());
    h.start34(1, 0, 1, [0, 0, 1 << 35]);
    h.read_lregs(5, 5);
    let samples = decode36(&h.stop_and_read34(true));
    assert_eq!(samples[0], 50 & 1);
    assert_eq!(samples.len(), 100_000 - 50);
}

#[test]
fn lwla1016_captures_plain_and_run_length_coded() {
    let mut h = Host::new(&[("model", "1016")]);
    h.open16("sysclk-lwla1016-100.rbf");
    h.start16(0xffff, 99, [0; 3]);
    let (status, fill, ms) = h.poll16();
    assert_eq!(status, CAP16_TRG_EN | CAP16_SAMPLE_EN | CAP16_NOT_END);
    assert_eq!((fill, ms), (2 + 50_000, 100));
    let words = h.stop_and_read16();
    assert_eq!(words.len(), 50_000);
    assert_eq!(words[3], 7 << 16 | 6);
    let mut h = Host::new(&[("model", "1016")]);
    h.open16("sysclk-lwla1016-100-ts.rbf");
    h.start16(0b100, 99, [0; 3]);
    h.poll16();
    let words = h.stop_and_read16();
    assert_eq!(&words[..3], &[3 << 16, 3 << 16 | 4, 3 << 16]);
}

#[test]
fn short_transfers_split_replies_into_64_bytes() {
    let mut h = Host::new(&[("short_transfers", "1")]);
    assert_eq!(h.bitstream("sysclk-lwla1034-int.rbf"), Reply::ack());
    h.read_long(L_TEST_ID);
    assert_eq!(h.send(&[8, 0, 10]), Reply::ack());
    assert_eq!(h.data(512).len(), 64);
    assert_eq!(h.data(512).len(), 16);
    assert_eq!(h.receive(512), Reply::Nak);
    h.clean();
}

#[test]
fn faults_answer_as_a_failing_board() {
    let mut h = Host::new(&[("stall", "read-reg:2,bitstream:1"), ("short_reply", "read-lregs")]);
    assert_eq!(h.bitstream("sysclk-lwla1034-int.rbf"), Reply::Stall);
    assert_eq!(h.bitstream("sysclk-lwla1034-int.rbf"), Reply::ack());
    assert_eq!(h.send(&[1, R34_LONG_STROBE]), Reply::ack());
    h.data(512);
    assert_eq!(h.send(&[1, R34_LONG_HIGH]), Reply::Stall);
    assert_eq!(h.send(&[1, R34_LONG_HIGH]), Reply::ack());
    h.data(512);
    assert_eq!(h.send(&[8, 0, 10]), Reply::ack());
    assert_eq!(h.data(512).len(), 78);
    h.clean();
    let mut h = Host::new(&[("stall_reply", "read-reg"), ("test_id", "0x1")]);
    assert_eq!(h.bitstream("sysclk-lwla1034-int.rbf"), Reply::ack());
    assert_eq!(h.send(&[1, R34_LONG_STROBE]), Reply::ack());
    assert_eq!(h.receive(512), Reply::Stall);
    let mut h = Host::new(&[("test_id", "0x1")]);
    assert_eq!(h.bitstream("sysclk-lwla1034-int.rbf"), Reply::ack());
    h.read_long(L_TEST_ID);
    assert_eq!(h.read_long(L_TEST_ID), 1);
    let mut h = Host::new(&[("halt_in", "1")]);
    assert_eq!(h.receive(512), Reply::Stall);
    let mut h = Host::new(&[("stale", "600")]);
    assert_eq!(h.data(512).len(), 512);
    assert_eq!(h.data(512).len(), 88);
    assert_eq!(h.receive(512), Reply::Nak);
    h.clean();
}

/// Each check, with the violation it gives.
#[test]
fn flags_what_the_driver_never_sends() {
    let expect = |h: &mut Host, what: &str| {
        let v = h.violations();
        assert!(v.iter().any(|m| m.contains(what)), "wanted '{what}', got {v:?}");
    };
    let mut h = Host::new(&[]);
    h.send(&[1, R34_MEM_FILL]);
    expect(&mut h, "FPGA is blank");
    h.receive(512);
    // A wrong bitstream, then a bad length header.
    h.dev.transfer_out(EP_CONFIG, &[0, 0, 0, 8, 1, 2, 3, 4], &mut h.v);
    expect(&mut h, "none of the");
    let body = standin("sysclk-lwla1034-int.rbf", 10_000);
    let mut data = 5u32.to_be_bytes().to_vec();
    data.extend_from_slice(&body);
    h.dev.transfer_out(EP_CONFIG, &data, &mut h.v);
    expect(&mut h, "header says 5 bytes");
    h.dev.transfer_out(EP_CONFIG, &[0, 0], &mut h.v);
    expect(&mut h, "no length header");
    h.bitstream("sysclk-lwla1016-100.rbf");
    expect(&mut h, "none of the");
    h.open34();
    h.dev.transfer_out(EP_COMMAND, &[1, 0, 2], &mut h.v);
    expect(&mut h, "16-bit words");
    h.send(&[9, 0]);
    expect(&mut h, "unknown command 9");
    h.send(&[1, 0x1078, 0]);
    expect(&mut h, "of 3 words; it has 2");
    h.send(&[3, 0, 0, 0, 8]);
    expect(&mut h, "not the Lwla1034's");
    h.send(&[1, R34_MEM_FILL]);
    h.send(&[1, R34_MEM_FILL]);
    expect(&mut h, "while a reply is unread");
    h.receive(4);
    h.send(&[8, 0, 10]);
    h.receive(64);
    expect(&mut h, "reply of 80 bytes read with 64");
    h.read_reg(0x1000);
    expect(&mut h, "does not read");
    h.write_reg(0x1000, 0);
    expect(&mut h, "does not write");
    h.write_reg(R34_MEM_FILL, 0);
    expect(&mut h, "read only");
    h.write_reg(R34_MEM_CTRL, 3);
    expect(&mut h, "memory control 0x3");
    h.write_reg(R34_CLK_BOOST, 2);
    expect(&mut h, "clock boost 2");
    h.write_reg(R34_MEM_START, DEPTH);
    expect(&mut h, "beyond the memory");
    h.write_reg(R34_LONG_ADDR, 9);
    h.read_reg(R34_LONG_LOW);
    expect(&mut h, "without a strobe read");
    h.write_long(3, 0);
    expect(&mut h, "written alone");
    h.write_long(10, 0x55);
    expect(&mut h, "capture control 0x55");
    h.read_lregs(8, 5);
    expect(&mut h, "long register read of 8 to 13");
    // Long registers: masks, divider, threshold, status.
    h.write_reg(R34_CLK_BOOST, 1);
    let mut w = vec![7u16, 0, 10];
    for value in [1u64 << 40, 5, 0b10, 0, 1, u64::from(DEPTH) + 1, 0, 0, 0, 7] {
        w.extend_from_slice(&[(value >> 16) as u16, value as u16, (value >> 48) as u16, (value >> 32) as u16]);
    }
    h.send(&w);
    let v = h.violations();
    for what in [
        "long register 0 written",
        "long register 1 written",
        "long register 5 written",
        "long register 9 written",
        "outside the enable mask",
    ] {
        assert!(v.iter().any(|m| m.contains(what)), "wanted '{what}', got {v:?}");
    }
    h.send(&[7, 0, 1, 0]);
    expect(&mut h, "takes 7");
    // Armed without the configuration, cleared capture control, and in
    // read mode.
    let mut h = Host::new(&[]);
    h.open34();
    h.write_reg(R34_MEM_CTRL, 2);
    h.write_long(10, 1);
    let v = h.violations();
    for what in ["before the long registers", "without clearing", "in read mode"] {
        assert!(v.iter().any(|m| m.contains(what)), "wanted '{what}', got {v:?}");
    }
    h.read_reg(R34_MEM_FILL);
    expect(&mut h, "fill level read while capturing");
    h.send(&[6, 0, 4, 0, 8]);
    h.receive(1024);
    let v = h.violations();
    for what in ["while capturing", "before its start address"] {
        assert!(v.iter().any(|m| m.contains(what)), "wanted '{what}', got {v:?}");
    }
    h.write_long(10, 0);
    h.write_reg(R34_MEM_CTRL, 1);
    h.write_reg(R34_MEM_START, 4);
    h.send(&[6, 0, 4, 0, 12]);
    h.receive(1024);
    let v = h.violations();
    for what in ["write mode", "not whole slices"] {
        assert!(v.iter().any(|m| m.contains(what)), "wanted '{what}', got {v:?}");
    }
    h.write_reg(R34_MEM_CTRL, 2);
    h.write_reg(R34_MEM_START, 4);
    h.send(&[6, 0, 12, 0, 8]);
    h.receive(1024);
    expect(&mut h, "the read pointer is at 4");
    h.send(&[6, 0, 20, 0, 240]);
    h.receive(2048);
    expect(&mut h, "over the FX2's 1024");
    h.send(&[6, 0x4, 0, 0, 8]);
    h.receive(1024);
    expect(&mut h, "beyond the memory");
    // Shut down: the FPGA is off.
    h.bitstream("sysclk-lwla1034-off.rbf");
    h.send(&[1, R34_MEM_FILL]);
    expect(&mut h, "shut-down bitstream");
}

#[test]
fn flags_what_the_lwla1016_driver_never_sends() {
    let expect = |h: &mut Host, what: &str| {
        let v = h.violations();
        assert!(v.iter().any(|m| m.contains(what)), "wanted '{what}', got {v:?}");
    };
    let mut h = Host::new(&[("model", "1016")]);
    h.open16("sysclk-lwla1016-100-ts.rbf");
    h.send(&[8, 0, 10]);
    expect(&mut h, "not the Lwla1016's");
    h.write_reg(R16_CHAN_MASK, 0x1_0000);
    expect(&mut h, "beyond 16 channels");
    h.write_reg(R16_DURATION, 3);
    expect(&mut h, "clears it with 0");
    h.write_reg(R16_MEM_CTRL, 4);
    expect(&mut h, "memory control 0x4");
    h.write_reg(R16_CAP_COUNT, DEPTH + 1);
    expect(&mut h, "capture count");
    h.write_reg(R16_DIV_COUNT, 1_000_000);
    expect(&mut h, "below 100 Hz");
    h.write_reg(R16_MEM_WR_PTR, 0);
    expect(&mut h, "read only");
    h.write_reg(R16_CAP_CTRL, 0x20);
    expect(&mut h, "capture control 0x20");
    h.read_reg(0x10c0);
    expect(&mut h, "does not read");
    // Armed without a threshold, then with triggers outside the mask in
    // read mode.
    let mut h = Host::new(&[("model", "1016")]);
    h.open16("sysclk-lwla1016-100.rbf");
    h.write_reg(R16_CAP_CTRL, CAP16_TRG_EN);
    expect(&mut h, "before the capture count");
    h.write_reg(R16_CAP_COUNT, 100);
    h.write_reg(R16_TEST_ID, 1 << 16 | 2);
    h.write_reg(R16_CAP_CTRL, CAP16_TRG_EN | 1 << 16);
    let v = h.violations();
    for what in ["outside the enable mask", "in read mode"] {
        assert!(v.iter().any(|m| m.contains(what)), "wanted '{what}', got {v:?}");
    }
    h.read_reg(R16_CAP_COUNT);
    expect(&mut h, "count read while capturing");
    h.write_reg(R16_CAP_CTRL, 0);
    h.write_reg(R16_MEM_CTRL, 0);
    h.send(&[3, 0, 5, 0, 8]);
    h.receive(1024);
    expect(&mut h, "the read pointer is at 2");
}

#[test]
fn flags_the_remaining_checks() {
    let expect = |h: &mut Host, what: &str| {
        let v = h.violations();
        assert!(v.iter().any(|m| m.contains(what)), "wanted '{what}', got {v:?}");
    };
    let mut h = Host::new(&[]);
    let mut data = u32::try_from(BITSTREAM_MAX + 5).unwrap().to_be_bytes().to_vec();
    data.resize(BITSTREAM_MAX + 5, 0);
    h.dev.transfer_out(EP_CONFIG, &data, &mut h.v);
    expect(&mut h, "over 256 KiB");
    h.open34();
    h.send(&[7, 0]);
    expect(&mut h, "long register write of 2 words");
    h.send(&[7, 9, 2, 0, 0, 0, 0, 0, 0, 0, 0]);
    expect(&mut h, "long register write of 9 to 11");
    h.write_reg(R34_LONG_ADDR, 50);
    h.read_reg(R34_LONG_STROBE);
    expect(&mut h, "read of long register 50");
    h.start34(1, 99, 0, [0; 3]);
    h.write_long(10, 0x74);
    expect(&mut h, "cleared while capturing");
    h.start34(1, 99, 0, [0; 3]);
    h.write_long(10, 1);
    expect(&mut h, "armed twice");
    h.write_long(10, 0);
    h.write_reg(R34_MEM_CTRL, 2);
    h.write_reg(R34_MEM_START, 4);
    h.send(&[6, 0, 4, 0, 0]);
    h.receive(1024);
    expect(&mut h, "memory read of 0 words");

    let mut h = Host::new(&[("model", "1016")]);
    h.open16("sysclk-lwla1016-100.rbf");
    h.start16(1, 99, [0; 3]);
    h.write_reg(R16_CAP_COUNT, 100);
    h.write_reg(R16_CAP_CTRL, CAP16_TRG_EN);
    expect(&mut h, "armed twice");
    h.write_reg(R16_CAP_CTRL, 3);
    expect(&mut h, "capture control 0x3 while capturing");
    h.send(&[3, 3, 0xfff0, 0, 32]);
    h.receive(1024);
    expect(&mut h, "beyond the memory");

    let mut model = Lwla::new(Board::Lwla1034, signal(34));
    let mut v = Violations::default();
    assert_eq!(model.out(1, &[0, 0], &mut v), Reply::Stall);
    assert_eq!(model.in_transfer(2, 512, &mut v), Reply::Stall);
    let setup = Setup { request_type: 0x40, request: 0xa0, value: 0, index: 0, length: 0 };
    assert_eq!(model.control(setup, &[], &mut v), Reply::Stall);
    assert_eq!(v.drain().len(), 3);
}

/// What the host sends is flagged, never allocated: the model holds no
/// more than the board's memory and its FX2's two buffers.
#[test]
fn host_counts_are_bounded() {
    let mut h = Host::new(&[]);
    h.open34();
    h.write_reg(R34_MEM_CTRL, 2);
    h.write_reg(R34_MEM_START, 4);
    assert_eq!(h.send(&[6, 0, 4, 0xffff, 0xfff8]), Reply::ack());
    assert_eq!(h.data(4096).len(), 28 * 36);
    assert!(!h.violations().is_empty());
    assert_eq!(h.send(&[8, 0, 0xffff]), Reply::ack());
    assert_eq!(h.data(1024).len(), 11 * 8);
    assert!(!h.violations().is_empty());
    // A threshold beyond the memory does not grow it.
    let mut w = vec![7u16, 0, 10];
    for value in [Board::Lwla1034.all(), 0, 0, 0, 0, u64::MAX, 0, 0, 0, 0] {
        w.extend_from_slice(&[(value >> 16) as u16, value as u16, (value >> 48) as u16, (value >> 32) as u16]);
    }
    h.send(&w);
    h.write_reg(R34_MEM_CTRL, 1);
    h.write_long(10, 0x74);
    h.write_long(10, 1);
    h.violations();
    let st = h.read_lregs(5, 5);
    assert_eq!(st[0], u64::from(DEPTH));

    let mut h = Host::new(&[("model", "1016")]);
    h.open16("sysclk-lwla1016-100.rbf");
    assert_eq!(h.send(&[3, 0, 0, 0xffff, 0xffff]), Reply::ack());
    assert_eq!(h.data(4096).len(), REPLY_MAX);
    h.write_reg(R16_CHAN_MASK, 0xffff);
    h.write_reg(R16_MEM_CTRL, 2);
    h.write_reg(R16_CAP_COUNT, u32::MAX);
    h.write_reg(R16_CAP_CTRL, CAP16_TRG_EN);
    h.violations();
    let (_, fill, _) = h.poll16();
    assert_eq!(fill, 2 + DEPTH);
}
