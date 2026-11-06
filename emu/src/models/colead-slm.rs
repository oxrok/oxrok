// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A Colead SL-5868P sound level meter on the CDC-ACM shell. It says `0x10`
//! when it has a measurement, and sends ten bytes when told `0x20`.
//!
//! The device, and how sure this is:
//!
//! - The sigrok wiki's page "Colead SL-5868P" gives 2400 8n1 over RS-232, two
//!   measurements a second, the `0x10` and `0x20`, and the record: `08 04`, the
//!   configuration (high nibble 1 normal, 2 max hold; low nibble which of
//!   fourteen settings, `0xe` and `0xf` unused), five digits of which `0x0a` is
//!   an ignored one and the last the decimal, a status (0 invalid, 1 valid) and
//!   the sum of the first nine bytes. It also says the `Read` key sends the
//!   stored records behind two bogus ones (digits all `0x0a`, byte 2 `0x09` then
//!   `0x08`), and that a return to live sends two more, the second with `0x07`.
//!   It does not say what a bogus record's status is, nor whether stored records
//!   are announced or asked for. So those are `packet=` and `announce=` lists to
//!   a case's taste, not a mode of this model.
//! - libsigrok's driver was written with a unit, and agrees with all of that.
//! - No firmware source, recording or manual was found. The chips the wiki names
//!   (W78E052D, HT1621B) say nothing of the protocol.
//!
//! Emulation artifacts:
//!
//! - The cable is RS-232, here CDC-ACM with a test id: only the byte stream and
//!   the line settings are the device's. On a misset line nothing passes, where
//!   a real one would give garbage.
//! - A record goes out whole and at once, where 2400 Bd takes 42 ms; the port's
//!   tests hold a byte-paced fake for that. `chunk=` cuts them, and `late=` holds
//!   one back.
//! - The rate of `0x10` is a count of read offers, [`Stream`], from each open, by
//!   default about the wiki's two a second. A real meter's clock runs while the
//!   port is closed too.
//! - The wiki does not say what a meter does with an announcement nobody asks
//!   for. This one goes on announcing, and any `0x20` gets the next record,
//!   announced or not.
//! - `burst=` is a silence no meter is known to have: it makes a capture's count
//!   of records independent of how long a tool listens.
//! - `-` in a list (no bytes) is a line losing bytes, not a known habit.
//!
//! Options, besides the shell's ([`Faults`]):
//!
//! - `packet=HEX|-[,...]`, what each `0x20` gets, in turn and then again, of any
//!   length up to 256 bytes;
//! - `announce=HEX|-[,...]`, what each announcement sends in place of `0x10`, in
//!   turn and then again;
//! - `period=N`, offers of a read per announcement (4 to 100000, default 25);
//! - `burst=N`, announcements per open, then silence and a mark a period later;
//!   0 never announces;
//! - `late=N:T`, the Nth `0x20` answered after T offers, with the announcements
//!   going on meanwhile;
//! - `uart=LINE`, the line settings, instead of 2400/8n1.
//!
//! The one pedantic check: a tool writes `0x20` and nothing else. The lines are
//! not looked at, as the wiki names none and a meter on RS-232 has no use for
//! them that it says.

use crate::model::DeviceModel;
use crate::shells::cdc::{Cdc, Faults, Identity, Line, LineEvent, Port, Serial};
use crate::shells::hidcable::{self as cable, Packets, Stream};
use crate::{Error, Result};

/// The model name.
pub const NAME: &str = "colead-slm";
/// What `oxrok-emu list` says about it.
pub const ABOUT: &str = "A Colead SL-5868P sound level meter on CDC-ACM; packet=HEX|-[,...], \
                         announce=HEX|-[,...], period=N, burst=N, late=N:T, uart=LINE, and the \
                         shell's stall=, chunk=, in-limit=, in-stall=, out-hold=";

/// "Measurement ready".
const READY: u8 = 0x10;
/// "Send measurement".
const SEND: u8 = 0x20;
/// 75.3 dB, Lp, A weighted, fast, with its status and sum.
const RECORD: [u8; 10] = [0x08, 0x04, 0x10, 0x0a, 0x0a, 0x07, 0x05, 0x03, 0x01, 0x40];
/// The most bytes in one entry of a list.
const LONGEST: usize = 256;

/// A meter from the `-o` options.
///
/// # Errors
///
/// On an unknown option or a value out of range.
pub fn build(options: &[(String, String)]) -> Result<Box<dyn DeviceModel>> {
    Ok(Box::new(function(options)?))
}

/// A list of byte strings, where `-` is none.
fn parse_list(value: &str) -> Result<Vec<Vec<u8>>> {
    let mut list = Vec::new();
    for entry in value.split(',') {
        if entry == "-" {
            list.push(Vec::new());
        } else {
            list.append(&mut cable::parse_packets(entry, 1, LONGEST)?);
        }
    }
    Ok(list)
}

fn function(options: &[(String, String)]) -> Result<Cdc<Meter>> {
    let mut faults = Faults::default();
    let mut uart = Line::parse("2400/8n1")?;
    let mut config = Config {
        packets: vec![RECORD.to_vec()],
        announces: vec![vec![READY]],
        period: 25,
        burst: None,
        late: None,
    };
    for (k, v) in options {
        if faults.option(k, v)? {
            continue;
        }
        match k.as_str() {
            "packet" => config.packets = parse_list(v)?,
            "announce" => config.announces = parse_list(v)?,
            "period" => config.period = cable::parse_number(k, v, 4..=100_000)?,
            "burst" => config.burst = cable::parse_burst(v)?,
            "late" => {
                let (nth, offers) = v
                    .split_once(':')
                    .ok_or_else(|| Error::Invalid(format!("late={v}: as in 2:40")))?;
                config.late = Some((
                    cable::parse_number(k, nth, 1..=100_000)?,
                    cable::parse_number(k, offers, 1..=100_000)?,
                ));
            }
            "uart" => uart = Line::parse(v)?,
            other => return Err(Error::Invalid(format!("{NAME} has no option {other}"))),
        }
    }
    let id = Identity::new(0x1209, 0x0001, "oxrok-emu", "Colead SL-5868P");
    Ok(Cdc::new(NAME, &id, faults, Meter::new(config)).behind_uart(uart))
}

#[derive(Clone, Debug)]
struct Config {
    packets: Vec<Vec<u8>>,
    announces: Vec<Vec<u8>>,
    period: u32,
    burst: Option<u32>,
    /// Which `0x20`, from 1, and for how many offers of a read.
    late: Option<(u32, u32)>,
}

struct Meter {
    packets: Packets,
    announces: Packets,
    stream: Stream,
    /// Announcements left of a burst, until the mark.
    left: Option<u32>,
    /// Offers since the burst's last announcement.
    quiet: u32,
    asked: u32,
    /// A record held back: the offers it still waits, and its bytes.
    held: Option<(u32, Vec<u8>)>,
    config: Config,
}

impl Meter {
    fn new(config: Config) -> Meter {
        Meter {
            packets: Packets::new(config.packets.clone()),
            announces: Packets::new(config.announces.clone()),
            // Offers come only with the tty open, and an open restarts it. Running
            // from the first, it also runs when a tool's line request stalled.
            stream: Stream::new(config.period, true).with_burst(config.burst),
            left: config.burst,
            quiet: 0,
            asked: 0,
            held: None,
            config,
        }
    }

    fn start(&mut self) {
        self.left = self.config.burst;
        self.stream = Stream::new(self.config.period, true).with_burst(self.config.burst);
        self.quiet = 0;
    }
}

impl Serial for Meter {
    fn received(&mut self, data: &[u8], port: &mut Port<'_>) {
        for &byte in data {
            if byte != SEND {
                port.violation(format!("{byte:#04x} written; a record is asked for with 0x20"));
                continue;
            }
            self.asked = self.asked.saturating_add(1);
            let record = self.packets.take();
            match self.config.late {
                Some((nth, offers)) if nth == self.asked => self.held = Some((offers, record)),
                _ => port.send(&record),
            }
        }
    }

    fn changed(&mut self, event: LineEvent, _port: &mut Port<'_>) {
        if event.opened() {
            self.start();
        }
    }

    fn tick(&mut self, port: &mut Port<'_>) {
        if let Some((offers, record)) = &mut self.held {
            *offers -= 1;
            if *offers == 0 {
                port.send(record);
                self.held = None;
            }
        }
        if self.stream.poll() {
            port.send(&self.announces.take());
            if let Some(left) = &mut self.left {
                *left = left.saturating_sub(1);
            }
        } else if self.left == Some(0) {
            // A period after the last, so that the tool has read it before an
            // event lands: bytes just delivered are lost to an unplug's hangup,
            // or not, as the pace has it.
            self.quiet += 1;
            if self.quiet >= self.config.period {
                port.mark();
                self.left = None;
            }
        }
    }

    fn reset(&mut self) {
        *self = Meter::new(self.config.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shells::cdc::host::Host;

    fn meter(options: &[(&str, &str)]) -> Host<Meter> {
        let o: Vec<(String, String)> =
            options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect();
        Host::open(function(&o).unwrap(), Line::parse("2400/8n1").unwrap(), true, true)
    }

    /// Offers until `n` have been made, and what they brought.
    fn offers(h: &mut Host<Meter>, n: u32) -> Vec<u8> {
        let mut out = Vec::new();
        for _ in 0..n {
            if let crate::model::Reply::Data(d) = h.offer() {
                out.extend(d);
            }
        }
        out
    }

    #[test]
    fn it_announces_and_sends_a_record_when_told() {
        let mut h = meter(&[("period", "4")]);
        assert_eq!(offers(&mut h, 3), b"");
        assert_eq!(offers(&mut h, 1), [READY]);
        h.write(&[SEND]);
        assert_eq!(h.read(), RECORD);
        assert_eq!(h.v.drain().len(), 0);
    }

    #[test]
    fn the_default_record_adds_up_and_reads_as_the_wiki_says() {
        let sum: u32 = RECORD[..9].iter().map(|&b| u32::from(b)).sum();
        assert_eq!(sum & 0xff, u32::from(RECORD[9]));
    }

    #[test]
    fn what_a_tool_may_not_write_is_flagged_and_gets_no_answer() {
        for bad in [0x10u8, 0x21, 0x00, 0xff] {
            let mut h = meter(&[("period", "100000")]);
            h.write(&[bad]);
            assert_eq!(h.v.drain().len(), 1, "{bad:#x}");
            assert_eq!(h.read(), b"");
        }
    }

    #[test]
    fn lists_play_in_turn_and_dash_is_nothing() {
        let mut h = meter(&[("period", "100000"), ("packet", "aa,-,bbcc")]);
        for want in [&[0xaa][..], &[], &[0xbb, 0xcc], &[0xaa]] {
            h.write(&[SEND]);
            assert_eq!(h.read(), want);
        }
        let mut h = meter(&[("period", "4"), ("announce", "ff,-,1010")]);
        for want in [&[0xff][..], &[], &[0x10, 0x10], &[0xff]] {
            assert_eq!(offers(&mut h, 4), want);
        }
    }

    #[test]
    fn a_burst_runs_out_and_marks_a_period_later() {
        let mut h = meter(&[("period", "4"), ("burst", "2")]);
        assert_eq!(offers(&mut h, 8), [READY, READY]);
        assert!(!h.cdc.mark());
        assert_eq!(offers(&mut h, 4), b"");
        assert!(h.cdc.mark());
        assert_eq!(offers(&mut h, 40), b"");
        // An open starts it over.
        h.set_lines(true, true);
        assert_eq!(offers(&mut h, 8), [READY, READY]);
    }

    #[test]
    fn a_burst_of_none_never_announces() {
        let mut h = meter(&[("period", "4"), ("burst", "0")]);
        assert_eq!(offers(&mut h, 40), b"");
    }

    #[test]
    fn a_late_answer_waits_for_its_offers_as_announcements_go_on() {
        let mut h = meter(&[("period", "4"), ("late", "1:6"), ("packet", "aa")]);
        h.write(&[SEND]);
        assert_eq!(offers(&mut h, 5), [READY]);
        assert_eq!(offers(&mut h, 1), [0xaa]);
        h.write(&[SEND]);
        assert_eq!(h.read(), [0xaa]);
    }

    #[test]
    fn behind_its_uart_a_wrong_rate_hears_nothing() {
        let o = vec![("period".to_owned(), "4".to_owned())];
        let mut h = Host::open(function(&o).unwrap(), Line::parse("9600/8n1").unwrap(), true, true);
        h.write(&[SEND]);
        assert_eq!(offers(&mut h, 8), b"");
        h.set_line(Line::parse("2400/8n1").unwrap());
        h.write(&[SEND]);
        assert_eq!(h.read(), RECORD);
    }

    #[test]
    fn a_reset_goes_back_to_power_on() {
        let mut h = meter(&[("period", "100000"), ("packet", "aa,bb"), ("late", "2:1")]);
        h.write(&[SEND]);
        assert_eq!(h.read(), [0xaa]);
        h.write(&[SEND]);
        assert_eq!(h.read(), [0xbb]);
        h.cdc.model.reset();
        h.write(&[SEND]);
        assert_eq!(h.read(), [0xaa]);
    }

    #[test]
    fn options_parse() {
        for bad in [
            ("period", "3"),
            ("burst", "x"),
            ("late", "2"),
            ("late", "0:1"),
            ("late", "1:0"),
            ("packet", "zz"),
            ("announce", ""),
            ("uart", "2400"),
            ("nothing", "1"),
        ] {
            let o = vec![(bad.0.to_owned(), bad.1.to_owned())];
            assert!(build(&o).is_err(), "{bad:?}");
        }
        for ok in [
            ("period", "4"),
            ("burst", "0"),
            ("late", "1:1"),
            ("packet", "-"),
            ("announce", "10,-"),
            ("uart", "9600/8n1"),
            ("chunk", "1"),
        ] {
            let o = vec![(ok.0.to_owned(), ok.1.to_owned())];
            assert!(build(&o).is_ok(), "{ok:?}");
        }
    }
}
