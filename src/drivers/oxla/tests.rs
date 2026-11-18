// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! The driver against a board on a fake port.
//!
//! The board is written from the protocol document, not from the gateware:
//! it takes the commands, keeps the registers and, at ARM, makes the
//! capture the registers ask for of its counter pattern (or of idle
//! inputs, all low), trigger terms and all. It holds the driver to the
//! document as this file reads it; the recordings of a real board in
//! `tests/fixtures/oxla` are what holds it to the device.

use super::protocol::{
    FLAG_RLE, FLAG_TEST, OP_ARM, OP_ID, OP_RESET, OP_RMEM, OP_RREG, OP_STOP, OP_WREG, REG_BUILD,
    REG_CHANNELS, REG_CHMASK, REG_CLOCK, REG_CLOCK_HZ, REG_CLOCKS, REG_COUNT, REG_DESC, REG_DIV,
    REG_FLAGS, REG_LIMITS, REG_POST_SAMPLES, REG_POST_WORDS, REG_PRE_WORDS, REG_STATUS, REG_STOP,
    REG_TCOUNT, REG_TERM_A, REG_TERM_B, REG_TMODE, REG_TRIG, RMEM_CHUNK, STATUS_DONE,
    STATUS_OVERFLOW, STATUS_READY, STATUS_RUNNING, STATUS_TRIGGERED,
};
use super::*;
use crate::core::{TriggerCombine, TriggerCount};
use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, sync_channel};

/// The clocks of a board that has three, by the value of `CLOCK`.
const CLOCKS: [u32; 3] = [81_000_000, 108_000_000, 135_000_000];
/// Samples the board takes at ARM before it says the capture still runs.
const BUDGET: u64 = 300_000;

/// What the driver did to the board.
#[derive(Default)]
struct Seen {
    /// Each command, opcode and arguments.
    commands: Vec<Vec<u8>>,
    /// What the board would not have taken.
    faults: Vec<String>,
    /// From now on nothing answers.
    muted: bool,
}

/// How a board misbehaves, or is slow.
#[derive(Clone, Copy, Default)]
struct Manner {
    /// Looks at the status that say running before one says done.
    polls: u32,
    /// The capture ends with the overflow bit.
    overflow: bool,
    /// A memory read loses its last byte from this many reads on.
    lossy_from: Option<usize>,
    /// Nothing answers.
    silent: bool,
    /// The ID is of something else.
    stranger: bool,
    /// Looks at the status after a change of clock that find it not ready.
    unready: u32,
}

/// The state of a capture between samples.
#[derive(Default)]
struct Run {
    words: u64,
    /// Raw: the bits not yet in a word.
    acc: u64,
    have: u32,
    /// Raw: samples that started in the word in progress.
    starts: u32,
    /// RLE: the run in progress, the sample and how often it was seen.
    open: Option<(u32, u32)>,
    /// The word of the trigger sample, which of its word's samples it is,
    /// and the words written when it came.
    trigger: Option<(u64, u32)>,
    after: u64,
    events: u32,
    /// A sequence saw its first term.
    armed: bool,
    prev: Option<u32>,
    last_bits: u32,
    done: bool,
}

struct Board {
    id: Id,
    manner: Manner,
    seen: Arc<Mutex<Seen>>,
    regs: [u32; 0x20],
    mem: Vec<u32>,
    run: Option<Run>,
    polls_left: u32,
    unready_left: u32,
    rmems: usize,
    command: Vec<u8>,
    out: VecDeque<u8>,
}

fn args_of(op: u8) -> usize {
    match op {
        OP_WREG => 5,
        OP_RREG => 1,
        OP_RMEM => 6,
        _ => 0,
    }
}

impl Board {
    fn new(id: Id, manner: Manner) -> (Board, Arc<Mutex<Seen>>) {
        let seen = Arc::new(Mutex::new(Seen::default()));
        let mut board = Board {
            id,
            manner,
            seen: Arc::clone(&seen),
            regs: [0; 0x20],
            mem: vec![0; id.mem_words as usize],
            run: None,
            polls_left: 0,
            unready_left: 0,
            rmems: 0,
            command: Vec::new(),
            out: VecDeque::new(),
        };
        board.reset();
        (board, seen)
    }

    fn reset(&mut self) {
        self.regs = [0; 0x20];
        self.regs[usize::from(REG_POST_SAMPLES)] = 0xffff_ffff;
        self.regs[usize::from(REG_CHMASK)] = 0xffff_ffff;
        self.run = None;
    }

    fn reg(&self, addr: u8) -> u32 {
        self.regs[usize::from(addr)]
    }

    fn fault(&self, what: String) {
        self.seen.lock().unwrap().faults.push(what);
    }

    fn id_bytes(&self) -> Vec<u8> {
        let mut d = if self.manner.stranger { b"1SLO".to_vec() } else { b"OXLA".to_vec() };
        d.extend([self.id.version, self.id.caps, self.id.channels, self.id.samples_per_clock]);
        d.extend(self.id.clock_hz.to_le_bytes());
        d.extend(self.id.mem_words.to_le_bytes());
        d
    }

    fn write_word(&mut self, run: &mut Run, word: u32) {
        let at = (run.words % u64::from(self.id.mem_words)) as usize;
        self.mem[at] = word;
        run.words += 1;
        if let Some((word, _)) = run.trigger
            && run.words > word
            && run.words - 1 - word == u64::from(self.reg(REG_POST_WORDS))
        {
            run.last_bits = 32;
            run.done = true;
        }
    }

    /// Whether a term matches `now` after `prev`.
    fn term(&self, base: u8, now: u32, prev: Option<u32>) -> bool {
        let r = |i: u8| self.reg(base + i);
        let (mask, value, rise, fall, edge) = (r(0), r(1), r(2), r(3), r(4));
        let levels = now & mask == value & mask;
        match prev {
            Some(prev) => {
                levels
                    && !prev & now & rise == rise
                    && prev & !now & fall == fall
                    && (prev ^ now) & edge == edge
            }
            // The first sample matches no edge.
            None => levels && rise | fall | edge == 0,
        }
    }

    /// Whether `now` is the trigger sample.
    fn triggers(&self, run: &mut Run, now: u32) -> bool {
        let a = self.term(REG_TERM_A, now, run.prev);
        let b = self.term(REG_TERM_B, now, run.prev);
        let (first, second) = if self.reg(REG_TMODE) == 5 { (a, b) } else { (b, a) };
        let event = match self.reg(REG_TMODE) {
            0 => true,
            1 => a,
            2 => b,
            3 => a || b,
            4 => a && b,
            _ => {
                let event = run.armed && second;
                run.armed = (run.armed || first) && !event;
                event
            }
        };
        if event {
            run.events += 1;
        }
        event && run.events > self.reg(REG_TCOUNT) & 0xffff
    }

    /// Ends what is in progress: the word, or the run.
    fn flush(&mut self, run: &mut Run) {
        if let Some((value, seen)) = run.open.take() {
            self.write_word(run, value | (seen - 1) << self.id.channels);
        } else if run.have > 0 {
            run.last_bits = run.have;
            let word = run.acc as u32;
            run.acc = 0;
            run.have = 0;
            self.write_word(run, word);
            return;
        }
        run.last_bits = 32;
    }

    /// Takes samples until the capture ends, `budget` of them at the most.
    fn sample(&mut self, run: &mut Run, from: u64, budget: u64) {
        let w = u32::from(self.id.channels);
        let flags = self.reg(REG_FLAGS);
        // The counter does not go through the channel mask; the pins do,
        // and they are all low here.
        let mask = self.id.channel_mask();
        let longest = 1 << (32 - w);
        for i in from..from + budget {
            if run.done {
                return;
            }
            let now = if flags & FLAG_TEST != 0 { i as u32 & mask } else { 0 };
            let looked = run.trigger.is_none() && run.words >= u64::from(self.reg(REG_PRE_WORDS));
            let fires = looked && self.triggers(run, now);
            run.prev = Some(now);
            if flags & FLAG_RLE != 0 {
                match &mut run.open {
                    Some((value, seen)) if *value == now && *seen < longest && !fires => *seen += 1,
                    _ => {
                        self.flush(run);
                        run.open = Some((now, 1));
                    }
                }
                if fires {
                    run.trigger = Some((run.words, 0));
                }
            } else {
                if fires {
                    run.trigger = Some((run.words, run.starts));
                }
                run.acc |= u64::from(now) << run.have;
                run.have += w;
                run.starts += 1;
                if run.have >= 32 {
                    let word = run.acc as u32;
                    run.acc >>= 32;
                    run.have -= 32;
                    run.starts = 0;
                    self.write_word(run, word);
                }
            }
            if run.trigger.is_some() && !fires {
                run.after += 1;
            }
            if run.trigger.is_some() && run.after == u64::from(self.reg(REG_POST_SAMPLES)) {
                if !run.done {
                    self.flush(run);
                }
                run.done = true;
            }
        }
    }

    fn status(&mut self) -> u32 {
        if self.unready_left > 0 {
            self.unready_left -= 1;
            return 0;
        }
        let Some(run) = &self.run else {
            return STATUS_READY;
        };
        if !run.done {
            return STATUS_READY | STATUS_RUNNING;
        }
        if self.polls_left > 0 {
            self.polls_left -= 1;
            return STATUS_READY | STATUS_RUNNING;
        }
        let mut status = STATUS_READY | STATUS_DONE | run.last_bits << 8;
        if run.trigger.is_some() {
            status |= STATUS_TRIGGERED;
        }
        if self.manner.overflow {
            status |= STATUS_OVERFLOW;
        }
        status
    }

    fn register(&mut self, addr: u8) -> u32 {
        let mem = u64::from(self.id.mem_words);
        if addr == REG_STATUS {
            return self.status();
        }
        // A board whose ID says so describes itself, as release 0.2.0 does.
        if self.id.has_description() {
            let adc = self.id.is_adc();
            let w = u32::from(self.id.channels);
            match addr {
                REG_DESC => return 1,
                REG_BUILD => return 0x00_02_00,
                REG_CLOCKS => return 0x0103,
                REG_CHANNELS => return if adc { w << 16 | 1 << 8 } else { w },
                REG_LIMITS => return 24 << 8 | u32::from(adc),
                a if (REG_CLOCK_HZ..REG_CLOCK_HZ + 3).contains(&a) => {
                    return CLOCKS[usize::from(a - REG_CLOCK_HZ)];
                }
                _ => {}
            }
        }
        let Some(run) = self.run.as_ref().filter(|r| r.done) else {
            return 0;
        };
        match addr {
            REG_TRIG => run.trigger.map_or(0, |(word, sub)| (word % mem) as u32 | sub << 24),
            REG_STOP => (run.words % mem) as u32,
            REG_COUNT => run.words.min(mem) as u32,
            _ => 0,
        }
    }

    fn execute(&mut self) {
        let c = std::mem::take(&mut self.command);
        let muted = {
            let mut seen = self.seen.lock().unwrap();
            seen.commands.push(c.clone());
            seen.muted
        };
        if self.manner.silent || muted {
            return;
        }
        let le = |b: &[u8]| b.iter().rev().fold(0u32, |v, b| v << 8 | u32::from(*b));
        match c[0] {
            OP_RESET => self.reset(),
            OP_ID => {
                let id = self.id_bytes();
                self.out.extend(id);
            }
            // The clock, on a board that has more than one: everything
            // else starts over.
            OP_WREG if c[1] == REG_CLOCK => {
                if self.id.has_clocks() {
                    self.id.clock_hz = CLOCKS[le(&c[2..6]) as usize % 3];
                    self.reset();
                    self.unready_left = self.manner.unready;
                } else {
                    self.fault("the clock written on a board with one".to_owned());
                }
            }
            OP_WREG if usize::from(c[1]) < self.regs.len() => {
                if self.run.as_ref().is_some_and(|r| !r.done) {
                    self.fault(format!("register {:#x} written during a capture", c[1]));
                }
                self.regs[usize::from(c[1])] = le(&c[2..6]);
            }
            OP_RREG => {
                let value = self.register(c[1]);
                self.out.extend(value.to_le_bytes());
            }
            OP_ARM => {
                if self.unready_left > 0 {
                    self.fault("armed before it was ready".to_owned());
                }
                let mut run = Run::default();
                self.sample(&mut run, 0, BUDGET);
                self.polls_left = self.manner.polls;
                self.run = Some(run);
            }
            OP_STOP => {
                if let Some(mut run) = self.run.take() {
                    if !run.done {
                        self.flush(&mut run);
                        run.done = true;
                    }
                    self.polls_left = 0;
                    self.run = Some(run);
                }
            }
            OP_RMEM => {
                let (addr, count) = (le(&c[1..4]), le(&c[4..7]));
                if count > RMEM_CHUNK {
                    self.fault(format!("{count} words asked for in one go"));
                }
                if self.run.as_ref().is_none_or(|r| !r.done) {
                    self.fault("memory read with no capture done".to_owned());
                }
                let mem = self.id.mem_words;
                let mut bytes: Vec<u8> = (0..count)
                    .flat_map(|i| self.mem[((addr + i) % mem) as usize].to_le_bytes())
                    .collect();
                self.rmems += 1;
                if self.manner.lossy_from.is_some_and(|n| self.rmems > n) {
                    bytes.pop();
                }
                self.out.extend(bytes);
            }
            _ => {}
        }
    }
}

impl Port for Board {
    fn name(&self) -> &'static str {
        "/dev/ttyFAKE"
    }
    fn set_params(&mut self, _: Params) -> Result<()> {
        Ok(())
    }
    fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
        Ok(())
    }
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let n = buf.len().min(self.out.len());
        if n == 0 {
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
        }
        for b in &mut buf[..n] {
            *b = self.out.pop_front().expect("counted");
        }
        Ok(n)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        for b in data {
            self.command.push(*b);
            if self.command.len() == 1 + args_of(self.command[0]) {
                self.execute();
            }
        }
        Ok(())
    }
    fn purge(&mut self) -> Result<()> {
        self.out.clear();
        Ok(())
    }
}

fn id(channels: u8, per_clock: u8, caps: u8, clock_hz: u32, mem_words: u32) -> Id {
    Id { version: 1, caps, channels, samples_per_clock: per_clock, clock_hz, mem_words }
}

/// `la16-rle` at 108 MHz.
fn la16() -> Id {
    id(16, 1, 1, 108_000_000, 1 << 21)
}

/// `la14-ddr` at 135 MHz.
fn la14() -> Id {
    id(14, 2, 2, 135_000_000, 2_097_151)
}

/// `la8-ddr` at 108 MHz.
fn la8() -> Id {
    id(8, 2, 2, 108_000_000, 1 << 21)
}

/// `la16-rle` with its three clocks, at 108 MHz as after power on.
fn la16_clocks() -> Id {
    id(16, 1, 9, 108_000_000, 1 << 21)
}

const FAST: Pace = Pace { answer: Duration::from_millis(100), poll: Duration::from_millis(1) };

/// A device with the one clock of its ID, as a scan would have found it.
fn found(id: Id) -> Found {
    Found { id, clocks: Clocks::new(&id, Vec::new()), description: None }
}

/// An open device on a board of `id`, scanned, set to its counter pattern.
fn device(id: Id, manner: Manner) -> (OxlaDevice, Arc<Mutex<Seen>>) {
    let (mut board, seen) = Board::new(id, manner);
    let found = probe_within(&mut board, FAST).unwrap().expect("a board");
    let mut dev = OxlaDevice::new(found, "fake".to_owned(), Box::new(board));
    dev.pace = FAST;
    dev.open().unwrap();
    dev.set(&Key::PatternMode, &Value::Str("Internal".into()), None).unwrap();
    (dev, seen)
}

fn set(dev: &mut OxlaDevice, key: &Key, value: u64) {
    dev.set(key, &Value::U64(value), None).unwrap();
}

fn trig(stages: &[&[(usize, TriggerMatch)]]) -> Trigger {
    let mut t = Trigger::new();
    for (i, stage) in stages.iter().enumerate() {
        for (ch, m) in *stage {
            t.add(i, *ch, *m);
        }
    }
    t
}

/// What a counter never does: its bit 1 rises as bit 0 falls.
fn never() -> Trigger {
    trig(&[&[(0, TriggerMatch::Rising), (1, TriggerMatch::Rising)]])
}

fn sink() -> (PacketSink, Receiver<(usize, Packet)>) {
    let (tx, rx) = sync_channel(1024);
    (PacketSink::new(tx, 0), rx)
}

/// The packets up to `End`, with a bound of their own.
fn until_end(rx: &Receiver<(usize, Packet)>) -> Vec<Packet> {
    let mut got = Vec::new();
    loop {
        let (_, p) = rx.recv_timeout(Duration::from_secs(20)).expect("an End in time");
        let end = matches!(p, Packet::End);
        got.push(p);
        if end {
            return got;
        }
    }
}

fn capture(dev: &mut OxlaDevice) -> Vec<Packet> {
    let (s, rx) = sink();
    dev.start(s).unwrap();
    let packets = until_end(&rx);
    dev.stop().unwrap();
    packets
}

/// The samples of the logic packets, and how many of them are ahead of the
/// trigger packet.
fn samples(packets: &[Packet]) -> (Vec<u16>, Option<usize>) {
    let mut all = Vec::new();
    let mut marker = None;
    for p in packets {
        match p {
            Packet::Logic(l) => all.extend(
                l.data
                    .chunks(l.unit_size)
                    .map(|s| s.iter().rev().fold(0u16, |v, b| v << 8 | u16::from(*b))),
            ),
            Packet::Trigger => {
                assert!(marker.is_none(), "one trigger packet");
                marker = Some(all.len());
            }
            _ => {}
        }
    }
    (all, marker)
}

fn failure(packets: &[Packet]) -> Option<&str> {
    packets.iter().find_map(|p| if let Packet::Failed(m) = p { Some(m.as_str()) } else { None })
}

fn counter(bits: u8, from: u64, n: u64) -> Vec<u16> {
    (from..from + n).map(|v| (v & ((1 << bits) - 1)) as u16).collect()
}

fn commands(seen: &Arc<Mutex<Seen>>) -> Vec<Vec<u8>> {
    seen.lock().unwrap().commands.clone()
}

fn no_faults(seen: &Arc<Mutex<Seen>>) {
    assert_eq!(seen.lock().unwrap().faults, Vec::<String>::new());
}

#[test]
fn a_probe_gets_in_step_and_reads_the_id() {
    let (mut board, seen) = Board::new(la14(), Manner::default());
    // Half a command and its answer under way, as a session that died
    // leaves them.
    board.write(&[OP_WREG, REG_DIV]).unwrap();
    board.out.extend([1, 2, 3]);
    assert_eq!(probe(&mut board).unwrap(), Some(found(la14())));
    let c = commands(&seen);
    // The eight NOPs: four are the rest of the write, of zero.
    assert_eq!(c[0], [OP_WREG, REG_DIV, 0, 0, 0, 0]);
    assert_eq!(c[c.len() - 2..], [vec![OP_RESET], vec![OP_ID]]);
    assert_eq!(c.len(), 7);
}

#[test]
fn what_does_not_answer_as_one_is_no_device() {
    let (mut board, _) = Board::new(la16(), Manner { silent: true, ..Manner::default() });
    assert_eq!(probe_within(&mut board, FAST).unwrap(), None);
    let (mut board, _) = Board::new(la16(), Manner { stranger: true, ..Manner::default() });
    assert_eq!(probe_within(&mut board, FAST).unwrap(), None);
}

#[test]
fn a_capture_with_no_trigger_is_the_first_samples() {
    let (mut dev, seen) = device(la16(), Manner { polls: 3, ..Manner::default() });
    set(&mut dev, &Key::LimitSamples, 10_000);
    let packets = capture(&mut dev);
    assert!(matches!(packets[0], Packet::Header { .. }));
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets), (counter(16, 0, 10_000), None));
    no_faults(&seen);

    let c = commands(&seen);
    // In step, the ID, the registers, armed; then the status until done.
    let arm = c.iter().position(|c| c == &[OP_ARM]).unwrap();
    assert_eq!(c[arm - 20..arm - 18], [[OP_RESET], [OP_ID]]);
    assert!(c[arm - 18..arm].iter().all(|c| c[0] == OP_WREG));
    assert_eq!(c[arm - 1], [OP_WREG, REG_POST_SAMPLES, 0x0f, 0x27, 0, 0]);
    assert_eq!(c[arm + 1..arm + 5], vec![vec![OP_RREG, REG_STATUS]; 4]);
    assert_eq!(
        c[arm + 5..arm + 8],
        [[OP_RREG, REG_TRIG], [OP_RREG, REG_STOP], [OP_RREG, REG_COUNT]]
    );
    // 5000 words, 2048 a request.
    assert_eq!(
        c[arm + 8..],
        [
            [OP_RMEM, 0, 0, 0, 0x00, 0x08, 0],
            [OP_RMEM, 0x00, 0x08, 0, 0x00, 0x08, 0],
            [OP_RMEM, 0x00, 0x10, 0, 0x88, 0x03, 0],
        ]
    );
}

#[test]
fn eight_channels_are_a_byte_a_sample() {
    let (mut dev, seen) = device(la8(), Manner::default());
    set(&mut dev, &Key::LimitSamples, 1001);
    let packets = capture(&mut dev);
    let Packet::Logic(l) = &packets[1] else { panic!("logic after the header") };
    assert_eq!((l.unit_size, l.data.len()), (1, 1001));
    assert_eq!(samples(&packets), (counter(8, 0, 1001), None));
    // The highest rate is two samples per clock.
    assert!(commands(&seen).contains(&vec![OP_WREG, REG_FLAGS, 6, 0, 0, 0]));
    no_faults(&seen);
}

#[test]
fn the_trigger_is_marked_with_its_share_ahead_of_it() {
    for id in [la16(), la14(), la8()] {
        let (mut dev, seen) = device(id, Manner::default());
        set(&mut dev, &Key::LimitSamples, 1000);
        set(&mut dev, &Key::CaptureRatio, 30);
        dev.set_trigger(Some(trig(&[&[(7, TriggerMatch::Rising)]]))).unwrap();
        let packets = capture(&mut dev);
        assert_eq!(failure(&packets), None);
        // The counter's bit 7 rises at 128 first: fewer ahead than wanted.
        let (all, marker) = samples(&packets);
        assert_eq!(marker, Some(128), "{}", id.variant());
        assert_eq!(all, counter(id.channels, 0, 128 + 700), "{}", id.variant());
        no_faults(&seen);
    }
}

#[test]
fn a_late_trigger_has_all_of_its_share_ahead() {
    // Bit 12 rises at 4096, and is high with bit 0 at 4097. Bit 7 of eight
    // falls at 256, and bit 4 changes next at 272.
    for (id, stages, at) in [
        (la16(), &[&[(12, TriggerMatch::Rising)][..]][..], 4096),
        (la14(), &[&[(12, TriggerMatch::One), (0, TriggerMatch::One)][..]][..], 4097),
        (la8(), &[&[(7, TriggerMatch::Falling)][..], &[(4, TriggerMatch::Edge)][..]][..], 272),
    ] {
        let (mut dev, seen) = device(id, Manner::default());
        set(&mut dev, &Key::LimitSamples, 201);
        set(&mut dev, &Key::CaptureRatio, 50);
        dev.set_trigger(Some(trig(stages))).unwrap();
        let packets = capture(&mut dev);
        let (all, marker) = samples(&packets);
        assert_eq!(marker, Some(100), "{}", id.variant());
        assert_eq!(all, counter(id.channels, at - 100, 201), "{}", id.variant());
        no_faults(&seen);
    }
}

#[test]
fn or_and_a_count() {
    let (mut dev, _) = device(la16(), Manner::default());
    set(&mut dev, &Key::LimitSamples, 10);
    // Bit 9 rises at 512, or bit 5 falls at 64.
    let mut t = trig(&[&[(9, TriggerMatch::Rising)], &[(5, TriggerMatch::Falling)]]);
    t.combine = TriggerCombine::Or;
    dev.set_trigger(Some(t)).unwrap();
    assert_eq!(samples(&capture(&mut dev)).0, counter(16, 64, 10));
    // The thousandth change of bit 2 is at 4000.
    let mut t = trig(&[&[(2, TriggerMatch::Edge)]]);
    t.set_count(0, TriggerCount { count: 1000, consecutive: false });
    dev.set_trigger(Some(t)).unwrap();
    assert_eq!(samples(&capture(&mut dev)), (counter(16, 4000, 10), Some(0)));
}

#[test]
fn runs_expand_and_the_cut_is_in_samples() {
    let (mut dev, seen) = device(la16(), Manner::default());
    // Only D4 and up. The device's counter still counts on D0 to D3, a
    // word a sample, and they are low in what is sent.
    for pos in 0..4 {
        dev.enable_channel(pos, false).unwrap();
    }
    dev.set(&Key::Rle, &Value::Bool(true), None).unwrap();
    set(&mut dev, &Key::Samplerate, 54_000_000);
    set(&mut dev, &Key::LimitSamples, 1000);
    set(&mut dev, &Key::CaptureRatio, 25);
    // D8 rises at 256, the start of a run.
    dev.set_trigger(Some(trig(&[&[(8, TriggerMatch::Rising)]]))).unwrap();
    let packets = capture(&mut dev);
    assert_eq!(failure(&packets), None);
    let (all, marker) = samples(&packets);
    assert_eq!(marker, Some(250));
    let expected: Vec<u16> = (6..1006).map(|i| (i & !0xf) as u16).collect();
    assert_eq!(all, expected);
    no_faults(&seen);
    let c = commands(&seen);
    assert!(c.contains(&vec![OP_WREG, REG_FLAGS, 5, 0, 0, 0]));
    assert!(c.contains(&vec![OP_WREG, REG_DIV, 1, 0, 0, 0]));
    assert!(c.contains(&vec![OP_WREG, REG_CHMASK, 0xf0, 0xff, 0, 0]));
    // 250 words ahead of the trigger's at the most, and the 750 after it.
    assert!(c.contains(&vec![OP_RMEM, 6, 0, 0, 0xe8, 0x03, 0]));
}

#[test]
fn long_runs_come_in_packets() {
    let (mut dev, _) = device(la16(), Manner::default());
    dev.set(&Key::PatternMode, &Value::Str("None".into()), None).unwrap();
    dev.set(&Key::Rle, &Value::Bool(true), None).unwrap();
    set(&mut dev, &Key::LimitSamples, 200_000);
    // Idle inputs: three runs of 65536 and a bit.
    let packets = capture(&mut dev);
    let sizes: Vec<usize> = packets
        .iter()
        .filter_map(|p| if let Packet::Logic(l) = p { Some(l.data.len()) } else { None })
        .collect();
    assert_eq!(sizes.iter().sum::<usize>(), 400_000);
    assert!(sizes.iter().all(|s| *s <= PACKET_BYTES));
    assert!(samples(&packets).0.iter().all(|s| *s == 0));
}

#[test]
fn a_ring_written_around_is_read_around() {
    // 70 words hold 160 samples of 14 bits; the trigger comes at 4096.
    let small = id(14, 2, 2, 135_000_000, 70);
    let (mut dev, seen) = device(small, Manner::default());
    set(&mut dev, &Key::LimitSamples, 150);
    set(&mut dev, &Key::CaptureRatio, 60);
    dev.set_trigger(Some(trig(&[&[(12, TriggerMatch::Rising)]]))).unwrap();
    let packets = capture(&mut dev);
    assert_eq!(failure(&packets), None);
    assert_eq!(samples(&packets), (counter(14, 4096 - 90, 150), Some(90)));
    no_faults(&seen);
    assert!(set_fails(&mut dev, 161));
}

/// Whether a capture of `limit` samples is refused.
fn set_fails(dev: &mut OxlaDevice, limit: u64) -> bool {
    set(dev, &Key::LimitSamples, limit);
    dev.check_capture(None).is_err()
}

#[test]
fn a_stop_sends_the_device_s_stop_and_what_it_had() {
    let (mut dev, seen) = device(la16(), Manner::default());
    set(&mut dev, &Key::LimitSamples, 1000);
    dev.set_trigger(Some(never())).unwrap();
    let (s, rx) = sink();
    dev.start(s).unwrap();
    assert!(matches!(rx.recv_timeout(Duration::from_secs(5)), Ok((_, Packet::Header { .. }))));
    assert!(rx.recv_timeout(Duration::from_millis(50)).is_err(), "still waiting");
    dev.stop().unwrap();
    let packets = until_end(&rx);
    assert_eq!(failure(&packets), None);
    // The last thousand of the board's 300,000, and no marker.
    assert_eq!(samples(&packets), (counter(16, BUDGET - 1000, 1000), None));
    let c = commands(&seen);
    let stop = c.iter().position(|c| c == &[OP_STOP]).unwrap();
    assert_eq!(c[stop + 1], [OP_RREG, REG_STATUS]);
    assert_eq!(c.iter().filter(|c| c == &&[OP_STOP]).count(), 1);
    no_faults(&seen);
}

#[test]
fn an_overflow_fails_the_capture() {
    let (mut dev, seen) = device(la16(), Manner { overflow: true, ..Manner::default() });
    dev.set(&Key::Rle, &Value::Bool(true), None).unwrap();
    let packets = capture(&mut dev);
    let why = failure(&packets).expect("a failed capture");
    assert!(why.contains("overflowed") && why.contains("half the clock rate"), "{why}");
    assert!(!packets.iter().any(|p| matches!(p, Packet::Logic(_))));
    assert!(matches!(packets.last(), Some(Packet::End)));
    assert!(!commands(&seen).iter().any(|c| c[0] == OP_RMEM));
    // The next capture is none the worse.
    let (mut dev, _) = device(la16(), Manner::default());
    assert_eq!(failure(&capture(&mut dev)), None);
}

#[test]
fn a_byte_lost_in_a_chunk_fails_the_capture() {
    let (mut dev, _) = device(la16(), Manner { lossy_from: Some(1), ..Manner::default() });
    let packets = capture(&mut dev);
    let why = failure(&packets).expect("a failed capture");
    assert!(why.contains("8191 of the 8192 bytes") && why.contains("2048 of 5000 words"), "{why}");
    assert!(!packets.iter().any(|p| matches!(p, Packet::Logic(_))));
}

#[test]
fn a_device_that_goes_silent_fails_the_capture() {
    // It answers the scan and the start, and then no more.
    let (mut dev, seen) = device(la16(), Manner::default());
    dev.set_trigger(Some(never())).unwrap();
    let (s, rx) = sink();
    dev.start(s).unwrap();
    assert!(matches!(rx.recv_timeout(Duration::from_secs(5)), Ok((_, Packet::Header { .. }))));
    seen.lock().unwrap().muted = true;
    let packets = until_end(&rx);
    dev.stop().unwrap();
    assert!(failure(&packets).is_some_and(|m| m.contains("register 0x80")), "{packets:?}");
    // And one that never answers does not start.
    let (board, _) = Board::new(la16(), Manner { silent: true, ..Manner::default() });
    let mut dev = OxlaDevice::new(found(la16()), "fake".to_owned(), Box::new(board));
    dev.pace = FAST;
    dev.open().unwrap();
    let (s, _rx) = sink();
    assert!(matches!(dev.start(s), Err(Error::Device(m)) if m.contains("not an oxla ID")));
}

#[test]
fn captures_follow_one_another_and_one_at_a_time() {
    let (mut dev, seen) = device(la16(), Manner::default());
    set(&mut dev, &Key::LimitSamples, 100);
    for _ in 0..2 {
        let (s, rx) = sink();
        // At once after End, with no stop.
        dev.start(s).unwrap();
        assert_eq!(samples(&until_end(&rx)).0.len(), 100);
    }
    dev.stop().unwrap();
    assert_eq!(commands(&seen).iter().filter(|c| c == &&[OP_ARM]).count(), 2);

    dev.set_trigger(Some(never())).unwrap();
    let (s, _rx) = sink();
    dev.start(s).unwrap();
    let (s, _rx2) = sink();
    assert!(matches!(dev.start(s), Err(Error::Busy)));
    dev.stop().unwrap();
    dev.close().unwrap();
    let (s, _rx3) = sink();
    assert!(dev.start(s).is_err(), "closed");
}

#[test]
fn a_capture_the_device_cannot_make_never_starts() {
    let (mut dev, seen) = device(la8(), Manner::default());
    let before = commands(&seen).len();
    set(&mut dev, &Key::LimitSamples, 8_388_609);
    let (s, _rx) = sink();
    assert!(matches!(dev.start(s), Err(Error::Arg(_))));
    assert!(dev.check_capture(None).is_err());
    assert_eq!(commands(&seen).len(), before, "nothing went out");
    set(&mut dev, &Key::LimitSamples, 8_388_608);
    assert!(dev.check_capture(None).is_ok());
    // Three stages, and, a count on two stages.
    let rising = |ch| [(ch, TriggerMatch::Rising)];
    let three = trig(&[&rising(0), &rising(1), &rising(2)]);
    assert!(dev.set_trigger(Some(three)).is_err());
    let mut and = trig(&[&rising(0), &rising(1)]);
    and.combine = TriggerCombine::And;
    assert!(dev.set_trigger(Some(and)).is_err());
    let mut counted = trig(&[&rising(0), &rising(1)]);
    counted.set_count(0, TriggerCount { count: 2, consecutive: false });
    assert!(dev.set_trigger(Some(counted)).is_err());
    assert!(dev.set_trigger(Some(trig(&[&rising(8)]))).is_err(), "eight channels");
    assert!(dev.set_trigger(Some(trig(&[&rising(0), &rising(1)]))).is_ok());
    assert!(dev.set_trigger(None).is_ok());
}

#[test]
fn the_options_are_what_the_id_allows() {
    let (mut dev, _) = device(la14(), Manner::default());
    let keys: Vec<Key> = dev.options(None).unwrap().into_iter().map(|o| o.key).collect();
    assert_eq!(
        keys,
        [
            Key::Conn,
            Key::LimitSamples,
            Key::Samplerate,
            Key::TriggerMatch,
            Key::CaptureRatio,
            Key::PatternMode
        ]
    );
    assert!(dev.set(&Key::Rle, &Value::Bool(true), None).is_err());
    assert!(dev.get(&Key::Rle, None).is_err());
    assert_eq!(dev.channels().len(), 14);
    assert_eq!(dev.channels()[13].name, "D13");
    assert_eq!(dev.info().version, "la14-ddr");
    assert_eq!(dev.info().conn.as_deref(), Some("fake"));
    // The highest rate to begin with.
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(270_000_000));
    set(&mut dev, &Key::Samplerate, 1_000_000);
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(1_000_000));
    // 135 MHz / 13 and / 14 are 10.38 and 9.64 MHz.
    set(&mut dev, &Key::Samplerate, 10_000_000);
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(9_642_857));
    assert!(dev.set(&Key::Samplerate, &Value::U64(270_000_001), None).is_err());
    assert!(dev.set(&Key::Samplerate, &Value::U64(7), None).is_err());
    // 135 MHz / 2^24 is 8 Hz.
    let rates = ValueList::RangeStepU64 { min: 8, max: 270_000_000, step: 1 };
    assert_eq!(dev.list(&Key::Samplerate, None).unwrap(), rates);
    assert!(matches!(dev.list(&Key::LimitSamples, None), Ok(ValueList::MinMaxU64(1, 4_793_488))));
    assert!(dev.set(&Key::LimitSamples, &Value::U64(0), None).is_err());
    assert!(dev.set(&Key::CaptureRatio, &Value::U64(101), None).is_err());
    assert!(dev.set(&Key::PatternMode, &Value::Str("Sawtooth".into()), None).is_err());
    assert_eq!(dev.get(&Key::PatternMode, None).unwrap(), Value::Str("Internal".into()));
    assert!(dev.options(Some("Logic")).is_err());

    let (mut dev, _) = device(la16(), Manner::default());
    assert!(dev.options(None).unwrap().iter().any(|o| o.key == Key::Rle));
    assert_eq!(dev.get(&Key::Rle, None).unwrap(), Value::Bool(false));
    dev.set(&Key::Rle, &Value::Bool(true), None).unwrap();
    let most = u64::from(u32::MAX);
    assert_eq!(dev.list(&Key::LimitSamples, None).unwrap(), ValueList::MinMaxU64(1, most));
}

#[test]
fn rle_at_two_samples_per_clock_is_refused() {
    // A bitstream with both, which no build has yet.
    let (mut dev, _) = device(id(8, 2, 3, 108_000_000, 1 << 21), Manner::default());
    dev.set(&Key::Rle, &Value::Bool(true), None).unwrap();
    assert!(dev.check_capture(None).is_err());
    set(&mut dev, &Key::Samplerate, 108_000_000);
    assert!(dev.check_capture(None).is_ok());
}

#[test]
fn a_scan_asks_each_clock_setting_what_it_is() {
    let (mut board, seen) = Board::new(la16_clocks(), Manner { unready: 2, ..Manner::default() });
    let found = probe_within(&mut board, FAST).unwrap().unwrap();
    assert_eq!(found.id, la16_clocks());
    let hz: Vec<u32> = found.clocks.settings().iter().map(|c| c.hz).collect();
    assert_eq!(hz, CLOCKS);
    let selects: Vec<Option<u32>> = found.clocks.settings().iter().map(|c| c.select).collect();
    assert_eq!(selects, [Some(0), Some(1), Some(2)]);
    // The ID, then for each setting: the write, the status until it is
    // ready, the ID.
    let c = commands(&seen);
    let from = c.iter().position(|c| c == &[OP_ID]).unwrap();
    let mut expected = vec![vec![OP_ID]];
    for select in 0..3 {
        expected.push(vec![OP_WREG, REG_CLOCK, select, 0, 0, 0]);
        expected.extend(vec![vec![OP_RREG, REG_STATUS]; 3]);
        expected.push(vec![OP_ID]);
    }
    assert_eq!(c[from..], expected);
    no_faults(&seen);
}

#[test]
fn a_capture_is_at_the_clock_of_its_rate() {
    let (mut dev, seen) = device(la16_clocks(), Manner::default());
    set(&mut dev, &Key::LimitSamples, 100);
    // The highest to begin with, where the scan left the device.
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(135_000_000));
    let rates = ValueList::RangeStepU64 { min: 5, max: 135_000_000, step: 1 };
    assert_eq!(dev.list(&Key::Samplerate, None).unwrap(), rates);
    let clock_writes = |seen: &Arc<Mutex<Seen>>| -> Vec<u8> {
        commands(seen)
            .iter()
            .filter(|c| c.starts_with(&[OP_WREG, REG_CLOCK]))
            .map(|c| c[2])
            .collect()
    };
    assert_eq!(clock_writes(&seen), [0, 1, 2], "the scan's");
    // Every clock has 27 MHz: the one it is at.
    set(&mut dev, &Key::Samplerate, 27_000_000);
    assert_eq!(samples(&capture(&mut dev)).0, counter(16, 0, 100));
    assert_eq!(clock_writes(&seen), [0, 1, 2]);
    assert!(commands(&seen).contains(&vec![OP_WREG, REG_DIV, 4, 0, 0, 0]));
    // 81 MHz is another clock, set ahead of the registers.
    set(&mut dev, &Key::Samplerate, 81_000_000);
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(81_000_000));
    assert_eq!(samples(&capture(&mut dev)).0, counter(16, 0, 100));
    assert_eq!(clock_writes(&seen), [0, 1, 2, 0]);
    let c = commands(&seen);
    let clock = c.iter().rposition(|c| c.starts_with(&[OP_WREG, REG_CLOCK])).unwrap();
    assert_eq!(c[clock - 1], [OP_ID]);
    assert_eq!(
        c[clock + 1..clock + 4],
        [vec![OP_RREG, REG_STATUS], vec![OP_ID], vec![OP_WREG, REG_DIV, 0, 0, 0, 0]]
    );
    // Once there, it stays: 10 MHz is 81 MHz / 8.
    set(&mut dev, &Key::Samplerate, 10_000_000);
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(10_125_000));
    assert_eq!(samples(&capture(&mut dev)).0, counter(16, 0, 100));
    assert_eq!(clock_writes(&seen), [0, 1, 2, 0]);
    no_faults(&seen);
}

#[test]
fn a_clock_that_does_not_come_back_is_no_capture() {
    // Ready in time for the scan's 200 looks, and not for one more.
    let (mut dev, seen) = device(la16_clocks(), Manner::default());
    set(&mut dev, &Key::Samplerate, 81_000_000);
    let (mut board, _) = Board::new(la16_clocks(), Manner { unready: 1000, ..Manner::default() });
    board.id.clock_hz = CLOCKS[2];
    *dev.lock_port() = Some(Box::new(board));
    let (s, _rx) = sink();
    assert!(matches!(dev.start(s), Err(Error::Device(m)) if m.contains("clock setting 0")));
    drop(seen);
    // A bitstream loaded since the scan.
    let (other, _) = Board::new(la8(), Manner::default());
    *dev.lock_port() = Some(Box::new(other));
    let (s, _rx) = sink();
    assert!(matches!(dev.start(s), Err(Error::Device(m)) if m.contains("now a la8-ddr")));
}

/// A board behind a bridge that still holds what the design before it
/// sent: that comes in place of the first answer.
struct Stale {
    board: Board,
    junk: Option<Vec<u8>>,
}

impl Port for Stale {
    fn name(&self) -> &'static str {
        "stale"
    }
    fn set_params(&mut self, _: Params) -> Result<()> {
        Ok(())
    }
    fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
        Ok(())
    }
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        match self.junk.take() {
            Some(junk) => {
                self.board.out.clear();
                buf[..junk.len()].copy_from_slice(&junk);
                Ok(junk.len())
            }
            None => self.board.read(buf, timeout),
        }
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        self.board.write(data)
    }
    fn purge(&mut self) -> Result<()> {
        self.board.purge()
    }
}

/// A device that is to be loaded with `variant`, whose load gives each of
/// `ports` in turn and then fails; and the loads there were.
fn to_load(
    variant: &str,
    ports: Vec<Box<dyn Port>>,
) -> (OxlaDevice, Arc<std::sync::atomic::AtomicUsize>) {
    let loads = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let count = Arc::clone(&loads);
    let mut ports = VecDeque::from(ports);
    let load: Loader = Box::new(move || {
        count.fetch_add(1, Ordering::SeqCst);
        ports.pop_front().ok_or_else(|| Error::Firmware("no bitstream".to_owned()))
    });
    let id = match variant {
        "la16-rle" => la16_clocks(),
        "la8-ddr" => la8(),
        other => panic!("{other}"),
    };
    let mut dev = OxlaDevice::to_load(id, &CLOCKS, "fake".to_owned(), load);
    dev.pace = FAST;
    (dev, loads)
}

#[test]
fn a_board_is_loaded_at_the_open_and_is_then_what_it_says() {
    // The board has another memory than the build's: what it says counts.
    let mut id = la16_clocks();
    id.mem_words = 1 << 20;
    let (board, seen) = Board::new(id, Manner::default());
    let (mut dev, loads) = to_load("la16-rle", vec![Box::new(board)]);
    // Listed as the variant, with its channels and clocks, and not loaded.
    assert!(dev.needs_load());
    assert_eq!((dev.info().version.as_str(), dev.channels().len()), ("la16-rle", 16));
    assert_eq!(dev.get(&Key::Samplerate, None).unwrap(), Value::U64(135_000_000));
    assert_eq!(loads.load(Ordering::SeqCst), 0);

    dev.open().unwrap();
    assert_eq!(loads.load(Ordering::SeqCst), 1);
    assert!(!dev.needs_load());
    assert_eq!(dev.id().mem_words, 1 << 20);
    // Open already: no second load.
    dev.open().unwrap();
    assert_eq!(loads.load(Ordering::SeqCst), 1);

    dev.set(&Key::PatternMode, &Value::Str("Internal".into()), None).unwrap();
    set(&mut dev, &Key::LimitSamples, 100);
    let (got, _) = samples(&capture(&mut dev));
    assert_eq!(got, counter(16, 0, 100));
    no_faults(&seen);
}

#[test]
fn what_the_bridge_still_held_is_not_the_answer_after_a_load() {
    let (board, _seen) = Board::new(la8(), Manner::default());
    let stale = Stale { board, junk: Some(vec![0x1c, 0x00, 0xe0, 0x1c, 0xfc]) };
    let (mut dev, loads) = to_load("la8-ddr", vec![Box::new(stale)]);
    dev.open().unwrap();
    assert_eq!((loads.load(Ordering::SeqCst), dev.needs_load()), (1, false));

    // Nor is a whole ID of the variant that ran before, which is not
    // asked for its clocks: the board that would answer is the new one.
    for id in [la16(), la16_clocks()] {
        let (before, _) = Board::new(id, Manner::default());
        let (board, seen) = Board::new(la8(), Manner::default());
        let stale = Stale { board, junk: Some(before.id_bytes()) };
        let (mut dev, loads) = to_load("la8-ddr", vec![Box::new(stale)]);
        dev.open().unwrap();
        assert_eq!((loads.load(Ordering::SeqCst), dev.needs_load()), (1, false));
        assert_eq!(dev.id().variant(), "la8-ddr");
        assert!(commands(&seen).iter().all(|c| c[0] != OP_WREG));
    }
}

#[test]
fn a_load_that_fails_or_gives_another_board_is_no_open() {
    // No bitstream to load: the error is the loader's, and the next open
    // tries again.
    let (mut dev, loads) = to_load("la16-rle", Vec::new());
    assert!(matches!(dev.open(), Err(Error::Firmware(_))));
    assert!(matches!(dev.open(), Err(Error::Firmware(_))));
    assert_eq!((loads.load(Ordering::SeqCst), dev.needs_load()), (2, true));

    // What answers after the load is another variant, or nothing.
    let (other, _) = Board::new(la8(), Manner::default());
    let (silent, _) = Board::new(la16(), Manner { silent: true, ..Manner::default() });
    let (mut dev, _loads) = to_load("la16-rle", vec![Box::new(other), Box::new(silent)]);
    for _ in 0..2 {
        let e = dev.open().unwrap_err().to_string();
        assert!(e.contains("does not answer as a la16-rle"), "{e}");
        assert!(dev.needs_load());
    }
}

/// `la8-adc` of release 0.2.0: the description bit, and an ADC.
fn la8_adc_described() -> Id {
    id(8, 1, 4 | 8 | 16, 108_000_000, 1 << 21)
}

#[test]
fn a_board_that_describes_itself_is_asked_and_not_retuned() {
    let (mut board, seen) = Board::new(la8_adc_described(), Manner::default());
    let found = probe_within(&mut board, FAST).unwrap().expect("a board");
    let d = found.description.as_ref().expect("a description");
    assert_eq!((d.build(), d.clocks_hz.as_slice()), ("0.2.0".to_owned(), &CLOCKS[..]));
    assert_eq!((d.logic_channels, d.analog_channels, d.analog_bits, d.div_min), (0, 1, 8, 1));
    // The clocks are those it lists, the lowest divider the one it says.
    let hz: Vec<u32> = found.clocks.settings().iter().map(|c| c.hz).collect();
    assert_eq!(hz, CLOCKS);
    assert_eq!(found.clocks.fastest().hz(), 67_500_000);
    // No write to CLOCK: the board is at the clock it had.
    assert!(commands(&seen).iter().all(|c| c[0] != OP_WREG));

    let dev = OxlaDevice::new(found, "fake".to_owned(), Box::new(board));
    assert_eq!(dev.info().version, "la8-adc 0.2.0");
    assert!(dev.description().is_some());
}

/// A board whose ID has the description bit, and whose registers read 0.
struct Bit(Board);
impl Port for Bit {
    fn name(&self) -> &'static str {
        "bit"
    }
    fn set_params(&mut self, _: Params) -> Result<()> {
        Ok(())
    }
    fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
        Ok(())
    }
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let n = self.0.read(buf, timeout)?;
        // The capability byte of an ID.
        if n == ID_LEN && &buf[..4] == b"OXLA" {
            buf[5] |= 16;
        }
        Ok(n)
    }
    fn write(&mut self, data: &[u8]) -> Result<()> {
        self.0.write(data)
    }
    fn purge(&mut self) -> Result<()> {
        self.0.purge()
    }
}

#[test]
fn a_description_bit_with_no_registers_is_the_document() {
    // The bit, and registers that read 0: the clocks are asked one by one.
    let (board, seen) = Board::new(la16_clocks(), Manner::default());
    let (mut plain, _) = Board::new(la16_clocks(), Manner::default());
    let want = probe_within(&mut plain, FAST).unwrap().unwrap();
    let found = probe_within(&mut Bit(board), FAST).unwrap().expect("a board");
    assert_eq!(found.description, None);
    assert_eq!(found.clocks, want.clocks);
    assert!(commands(&seen).iter().any(|c| c[0] == OP_WREG && c[1] == REG_CLOCK));
}

#[test]
fn an_answer_cut_short_after_a_load_is_asked_again_and_not_loaded_again() {
    // The ID comes whole and the first register after it does not.
    struct Short {
        board: Board,
        cut: bool,
    }
    impl Port for Short {
        fn name(&self) -> &'static str {
            "short"
        }
        fn set_params(&mut self, _: Params) -> Result<()> {
            Ok(())
        }
        fn set_handshake(&mut self, _: Option<bool>, _: Option<bool>) -> Result<()> {
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
            if self.cut && buf.len() == 4 && self.board.out.len() == 4 {
                self.cut = false;
                self.board.out.truncate(2);
            }
            self.board.read(buf, timeout)
        }
        fn write(&mut self, data: &[u8]) -> Result<()> {
            self.board.write(data)
        }
        fn purge(&mut self) -> Result<()> {
            self.board.purge()
        }
    }
    let (board, _seen) = Board::new(la16_clocks(), Manner::default());
    let (mut dev, loads) = to_load("la16-rle", vec![Box::new(Short { board, cut: true })]);
    dev.open().unwrap();
    assert_eq!((loads.load(Ordering::SeqCst), dev.needs_load()), (1, false));
}

#[test]
fn a_close_does_not_wait_on_a_queue_nobody_reads() {
    let (mut dev, _seen) = device(la16(), Manner::default());
    set(&mut dev, &Key::LimitSamples, 100_000);
    // Room for the header alone, and nobody reads.
    let (tx, rx) = sync_channel(1);
    dev.start(PacketSink::new(tx, 0)).unwrap();
    std::thread::sleep(Duration::from_millis(300));
    let before = Instant::now();
    dev.close().unwrap();
    assert!(before.elapsed() < Duration::from_secs(5));
    drop(rx);
}
