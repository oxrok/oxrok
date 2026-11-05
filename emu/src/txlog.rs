// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The transaction log, as an [`Observer`] of the vusb bus: one line per
//! completed transfer, in the dialect of oxrok's USB fixtures
//! (`docs/USB.md`), `# section NAME` markers between the phases of a
//! run, and `! violation: ...` lines for what the host did outside the
//! protocol. Transfers the device holds with a NAK are not logged until
//! they complete, so a log does not depend on how often the host polls;
//! unlinks and other events are `#` comments, written only in verbose
//! mode. A model's [`Rules`] can ask for data longer than some size, in
//! either direction, to be written as `@LEN:SHA256`, the fixture
//! dialect's digest form, so a firmware upload or a stream of samples
//! does not make lines of megabytes, and for bytes the host leaves
//! undefined to show as `xx`. `tests/emu` holds this format.

use std::fmt::Write as _;
use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::ops::Range;
use std::path::Path;

use sha2::{Digest as _, Sha256};
use vusb::observer::{Notice, Observer, Outcome, Transfer};

use crate::model::{BULK, CONTROL, INTERRUPT, Reply, Setup};

/// How a model's transfers are logged. A model module that wants more
/// than the default exports `pub fn log_rules(options: &[(String,
/// String)]) -> Rules`, which `build.rs` finds.
#[derive(Clone, Copy, Debug, Default)]
pub struct Rules {
    /// Data longer than this is logged as `@LEN:SHA256` rather than in
    /// hex ([`HASH_ABOVE`] is the usual value); `None` logs every byte.
    pub digest_above: Option<usize>,
    /// The bytes of an OUT transfer of the data to endpoint number `ep`
    /// (not 0) that the host leaves undefined, such as a C driver's
    /// uninitialised struct field, which would make the log differ from
    /// one run to the next. The log shows them as `xx`
    /// ([`data_undefined`]); the model documents each.
    pub undefined: Option<fn(u8, &[u8]) -> Vec<Range<usize>>>,
}

impl Rules {
    /// Data longer than `above` as its digest, every byte defined.
    pub fn digest_above(above: usize) -> Rules {
        Rules { digest_above: Some(above), undefined: None }
    }
}

/// The log.
#[derive(Debug)]
pub struct TxLog {
    out: Option<BufWriter<File>>,
    verbose: bool,
    rules: Rules,
}

impl TxLog {
    /// A log to `path`, or to nowhere; events go in only when `verbose`.
    pub fn new(path: Option<&Path>, verbose: bool) -> io::Result<TxLog> {
        let out = match path {
            Some(p) => Some(BufWriter::new(File::create(p)?)),
            None => None,
        };
        Ok(TxLog { out, verbose, rules: Rules::default() })
    }

    /// The rules of the model behind the device from now on.
    pub fn set_rules(&mut self, rules: Rules) {
        self.rules = rules;
    }

    fn line(&mut self, s: &str) {
        if let Some(o) = &mut self.out {
            // Errors writing the log are not worth stopping the device.
            let _ = writeln!(o, "{s}");
            let _ = o.flush();
        }
    }

    /// An event, in verbose mode only.
    fn event(&mut self, message: &str) {
        if self.verbose {
            eprintln!("emu: {message}");
            self.line(&format!("# {message}"));
        }
    }

    /// The line of a transfer that was offered to the device.
    fn transfer_text(&self, t: &Transfer) -> String {
        let above = self.rules.digest_above;
        if let Some(setup) = t.setup {
            return control_line(setup, &t.data, &t.reply, above);
        }
        // An endpoint the configuration lacks has no kind; such a transfer
        // is a violation already.
        let kind = t.kind.unwrap_or(0xff);
        if t.is_in {
            // An unlinked transfer shows what reached the URB.
            let cut = match &t.reply {
                Reply::Data(d) if t.outcome == Outcome::Unlinked => {
                    Some(Reply::Data(d[..d.len().min(t.requested)].to_vec()))
                }
                _ => None,
            };
            let reply = cut.as_ref().unwrap_or(&t.reply);
            transfer_line(kind, t.endpoint, t.offered, &t.data, reply, above)
        } else {
            let undefined =
                self.rules.undefined.map_or_else(Vec::new, |f| f(t.endpoint & 0x0f, &t.data));
            transfer_line_undefined(
                kind,
                t.endpoint,
                t.offered,
                &t.data,
                &undefined,
                &t.reply,
                above,
            )
        }
    }
}

impl Observer for TxLog {
    fn transfer(&mut self, t: &Transfer) {
        let line = self.transfer_text(t);
        match t.outcome {
            Outcome::Held => self.event(&format!("seq {}: {line} (held)", t.seqnum)),
            Outcome::Babble => {
                self.line(&format!("{line} (babble: the URB holds {})", t.requested));
            }
            _ => self.line(&line),
        }
    }

    fn violation(&mut self, message: &str) {
        self.line(&format!("! violation: {message}"));
    }

    fn notice(&mut self, notice: Notice<'_>) {
        match notice {
            Notice::Comment(c) => self.event(c),
            Notice::Section(name) => self.line(&format!("# section {name}")),
            // Lines the comparisons keep, in the section of the run that
            // caused it: a tool that ends right after the transfer would
            // otherwise see them or not depending on timing.
            Notice::Renumeration { vendor, product } => {
                self.line("disconnect");
                self.line(&format!("connect {vendor:04x}:{product:04x}"));
            }
            // In the running section, right after the transfer that fired it.
            Notice::Fired(event) => self.line(event.word()),
            _ => {}
        }
    }
}

/// Bytes as hex pairs, `-` when empty.
pub fn hex(bytes: &[u8]) -> String {
    if bytes.is_empty() {
        return "-".to_owned();
    }
    bytes.iter().fold(String::with_capacity(2 * bytes.len()), |mut s, b| {
        let _ = write!(s, "{b:02x}");
        s
    })
}

/// The size above which oxrok's fixtures record data sent to a device
/// as its length and SHA-256 (`oxrok::usb::record::HASH_ABOVE`), and
/// the value models that log digests use.
pub const HASH_ABOVE: usize = 1024;

/// Bytes as [`hex`], or as `@LEN:SHA256` when longer than `above`.
pub fn data(bytes: &[u8], above: Option<usize>) -> String {
    if above.is_some_and(|n| bytes.len() > n) {
        format!("@{}:{}", bytes.len(), hex(&Sha256::digest(bytes)))
    } else {
        hex(bytes)
    }
}

/// Bytes as [`data`], those in `undefined` (bytes the host left
/// undefined, [`Rules::undefined`]) as `xx`; in
/// a digest they count as 0 and their ranges follow it, `@LEN:SHA256/xx=A-B,...`.
pub fn data_undefined(bytes: &[u8], undefined: &[Range<usize>], above: Option<usize>) -> String {
    let inside = |i: usize| undefined.iter().any(|r| r.contains(&i));
    if undefined.is_empty() {
        data(bytes, above)
    } else if above.is_some_and(|n| bytes.len() > n) {
        let zeroed: Vec<u8> =
            bytes.iter().enumerate().map(|(i, b)| if inside(i) { 0 } else { *b }).collect();
        let ranges: Vec<String> =
            undefined.iter().map(|r| format!("{}-{}", r.start, r.end.saturating_sub(1))).collect();
        format!("{}/xx={}", data(&zeroed, above), ranges.join(","))
    } else {
        bytes.iter().enumerate().fold(String::with_capacity(2 * bytes.len()), |mut s, (i, b)| {
            if inside(i) {
                s.push_str("xx");
            } else {
                let _ = write!(s, "{b:02x}");
            }
            s
        })
    }
}

fn result(reply: &Reply, is_in: bool, above: Option<usize>) -> String {
    match reply {
        Reply::Data(d) if is_in => data(d, above),
        Reply::Data(_) => "ok".to_owned(),
        Reply::Nak => "nak".to_owned(),
        Reply::Stall => "err stall".to_owned(),
    }
}

/// The line of a control transfer: the fixture format, the direction
/// bit of `bmRequestType` left to the `ctl_in`/`ctl_out` word; data
/// longer than `above` as its digest.
pub fn control_line(setup: Setup, bytes: &[u8], reply: &Reply, above: Option<usize>) -> String {
    let kind = setup.request_type & 0x7f;
    if setup.is_in() {
        format!(
            "ctl_in {kind:02x} {:02x} {:04x} {:04x} {} -> {}",
            setup.request,
            setup.value,
            setup.index,
            setup.length,
            result(reply, true, above)
        )
    } else {
        format!(
            "ctl_out {kind:02x} {:02x} {:04x} {:04x} {} -> {}",
            setup.request,
            setup.value,
            setup.index,
            data(bytes, above),
            result(reply, false, above)
        )
    }
}

/// The line of a bulk, interrupt or isochronous transfer on endpoint
/// `address`; `max` is the length asked for an IN, `data` the payload
/// of an OUT.
pub fn transfer_line(
    transfer_type: u8,
    address: u8,
    max: usize,
    bytes: &[u8],
    reply: &Reply,
    above: Option<usize>,
) -> String {
    transfer_line_undefined(transfer_type, address, max, bytes, &[], reply, above)
}

/// [`transfer_line`] with the OUT payload's `undefined` bytes shown as
/// [`data_undefined`] shows them.
pub fn transfer_line_undefined(
    transfer_type: u8,
    address: u8,
    max: usize,
    bytes: &[u8],
    undefined: &[Range<usize>],
    reply: &Reply,
    above: Option<usize>,
) -> String {
    let word = match transfer_type {
        BULK => "bulk",
        INTERRUPT => "int",
        CONTROL => "ctl",
        _ => "iso",
    };
    if address & 0x80 != 0 {
        format!("{word}_in {address:02x} {max} -> {}", result(reply, true, above))
    } else {
        let taken = match reply {
            Reply::Data(d) if d.is_empty() => bytes.len().to_string(),
            Reply::Data(d) => d.len().to_string(),
            other => result(other, false, above),
        };
        format!("{word}_out {address:02x} {} -> {taken}", data_undefined(bytes, undefined, above))
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use vusb::bus::Bus;
    use vusb::device::Device;
    use vusb::examples::relay::UsbRelay;
    use vusb::observer::SharedObserver;

    use super::*;

    /// What a log says of a board configured and read, with `rules`.
    fn relay_log(rules: Rules) -> String {
        let path = std::env::temp_dir().join(format!(
            "oxrok-emu-txlog-{}-{}",
            std::process::id(),
            rules.digest_above.unwrap_or(0)
        ));
        let mut log = TxLog::new(Some(&path), false).unwrap();
        log.set_rules(rules);
        let log: SharedObserver = Arc::new(Mutex::new(log));
        let mut bus = Bus::new(Device::new(Box::new(UsbRelay::new(2)))).with_observer(log);
        let setup = |request_type, request, value, length| Setup {
            request_type,
            request,
            value,
            index: 0,
            length,
        };
        bus.notice(Notice::Section("run"));
        bus.control(setup(0x00, 9, 1, 0), &[]);
        bus.control(setup(0xa1, 1, 0x0300, 8), &[]);
        bus.notice(Notice::Renumeration { vendor: 0x1234, product: 0xabcd });
        let text = std::fs::read_to_string(&path).unwrap();
        std::fs::remove_file(&path).unwrap();
        text
    }

    #[test]
    fn a_session_reads_as_the_goldens_do() {
        assert_eq!(
            relay_log(Rules::default()),
            "# section run\n\
             ctl_out 00 09 0001 0000 - -> ok\n\
             ctl_in 21 01 0300 0000 8 -> 4142434445000000\n\
             disconnect\n\
             connect 1234:abcd\n"
        );
    }

    #[test]
    fn rules_apply_to_control_data() {
        let text = relay_log(Rules::digest_above(2));
        assert!(text.contains("ctl_in 21 01 0300 0000 8 -> @8:"), "{text}");
    }

    #[test]
    fn undefined_bytes_show_as_xx() {
        let r = Reply::ack();
        let mid = 1..3;
        let head = 0..2;
        assert_eq!(
            transfer_line_undefined(
                BULK,
                0x02,
                0,
                &[1, 2, 3, 4],
                std::slice::from_ref(&mid),
                &r,
                None
            ),
            "bulk_out 02 01xxxx04 -> 4"
        );
        let long = vec![7u8; HASH_ABOVE + 2];
        let mut zeroed = long.clone();
        zeroed[..2].fill(0);
        assert_eq!(
            data_undefined(&long, std::slice::from_ref(&head), Some(HASH_ABOVE)),
            format!("{}/xx=0-1", data(&zeroed, Some(HASH_ABOVE)))
        );
        assert_eq!(data_undefined(&[5], &[], None), "05");
    }

    #[test]
    fn lines_follow_the_fixture_dialect() {
        let s = Setup { request_type: 0xa1, request: 1, value: 0x0300, index: 0, length: 8 };
        assert_eq!(
            control_line(s, &[], &Reply::Data(vec![0x41, 0x42]), None),
            "ctl_in 21 01 0300 0000 8 -> 4142"
        );
        let s = Setup { request_type: 0x21, request: 9, value: 0x0300, index: 0, length: 2 };
        assert_eq!(
            control_line(s, &[0xff, 1], &Reply::ack(), None),
            "ctl_out 21 09 0300 0000 ff01 -> ok"
        );
        assert_eq!(
            control_line(s, &[0xff, 1], &Reply::Stall, None),
            "ctl_out 21 09 0300 0000 ff01 -> err stall"
        );
        assert_eq!(
            transfer_line(INTERRUPT, 0x81, 64, &[], &Reply::Data(vec![1]), None),
            "int_in 81 64 -> 01"
        );
        assert_eq!(
            transfer_line(BULK, 0x02, 0, &[1, 2], &Reply::ack(), None),
            "bulk_out 02 0102 -> 2"
        );
        // Long data as its digest, the fixtures' form, when asked for.
        let long = vec![0u8; HASH_ABOVE + 1];
        assert_eq!(data(&long, None).len(), 2 * (HASH_ABOVE + 1));
        assert_eq!(data(&long[..HASH_ABOVE], Some(HASH_ABOVE)), "00".repeat(HASH_ABOVE));
        assert_eq!(
            transfer_line(BULK, 0x82, 2048, &[], &Reply::Data(long), Some(HASH_ABOVE)),
            format!("bulk_in 82 2048 -> @1025:{}", hex(&Sha256::digest([0u8; 1025])))
        );
    }
}
