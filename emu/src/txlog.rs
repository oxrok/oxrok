//! The transaction log: one line per completed transfer, in the
//! dialect of oxrok's USB fixtures (`docs/USB.md`), `# section NAME`
//! markers between the phases of a run, and `! violation: ...` lines
//! for what the host did outside the protocol. Transfers the device
//! holds with a NAK are not logged until they complete, so a log does
//! not depend on how often the host polls; unlinks and other events
//! are `#` comments, written only in verbose mode. For a model that
//! asks for it ([`crate::model::DeviceModel::log_digest_above`]), data
//! longer than that, in either direction, is written as `@LEN:SHA256`,
//! the fixture dialect's digest form, so a firmware upload or a stream
//! of samples does not make lines of megabytes.

use std::fmt::Write as _;
use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;

use sha2::{Digest as _, Sha256};

use crate::model::{BULK, CONTROL, INTERRUPT, Reply, Setup};

/// The log.
pub struct TxLog {
    out: Option<BufWriter<File>>,
    verbose: bool,
    violations: usize,
    section: String,
}

impl TxLog {
    /// A log to `path`, or to nowhere; events go in only when
    /// `verbose`. Violations are also printed to stderr.
    pub fn new(path: Option<&Path>, verbose: bool) -> io::Result<TxLog> {
        let out = match path {
            Some(p) => Some(BufWriter::new(File::create(p)?)),
            None => None,
        };
        Ok(TxLog { out, verbose, violations: 0, section: String::new() })
    }

    fn line(&mut self, s: &str) {
        if let Some(o) = &mut self.out {
            // Errors writing the log are not worth stopping the device.
            let _ = writeln!(o, "{s}");
            let _ = o.flush();
        }
    }

    /// Starts a section.
    pub fn section(&mut self, name: &str) {
        name.clone_into(&mut self.section);
        self.line(&format!("# section {name}"));
    }

    /// The current section's name.
    pub fn current_section(&self) -> &str {
        &self.section
    }

    /// One completed transfer.
    pub fn transfer(&mut self, line: &str) {
        self.line(line);
    }

    /// A violation, counted and also printed to stderr.
    pub fn violation(&mut self, message: &str) {
        self.violations += 1;
        eprintln!("violation [{}]: {message}", self.section);
        self.line(&format!("! violation: {message}"));
    }

    /// An event, in verbose mode only.
    pub fn event(&mut self, message: &str) {
        if self.verbose {
            eprintln!("emu: {message}");
            self.line(&format!("# {message}"));
        }
    }

    /// How many violations were logged.
    pub fn violations(&self) -> usize {
        self.violations
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
        format!("{word}_out {address:02x} {} -> {taken}", data(bytes, above))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
