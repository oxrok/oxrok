// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! The transaction log of `tests/emu`: vusb's [`Log`], which writes the
//! format (vusb's `docs/log.md`), under the [`Rules`] of the model behind
//! the device. What is oxrok's own is that a verbose log also tells
//! standard error what it writes as comments, and that the rules follow
//! the model ([`factory`]).

use std::io::Write;
use std::sync::{Arc, Mutex};

pub use vusb::observer::fixture::{DIGEST_ABOVE as HASH_ABOVE, Rules, hex};
use vusb::observer::fixture::{Log, line};
use vusb::observer::{Notice, Observer, Outcome, Transfer};
use vusb::session::ModelFactory;

use crate::models;

/// The log, with its comments on standard error too when verbose.
#[derive(Debug)]
pub struct TxLog<W> {
    log: Log<W>,
    verbose: bool,
}

impl<W: Write + Send> TxLog<W> {
    /// A log to `out`, under the default rules until [`TxLog::set_rules`];
    /// comments and held transfers go in only when `verbose`.
    pub fn new(out: W, verbose: bool) -> TxLog<W> {
        TxLog { log: Log::new(out, Rules::default()).verbose(verbose), verbose }
    }

    /// The rules of the model behind the device from now on.
    pub fn set_rules(&mut self, rules: Rules) {
        self.log.set_rules(rules);
    }
}

impl<W: Write + Send> Observer for TxLog<W> {
    fn transfer(&mut self, t: &Transfer) {
        if self.verbose && t.outcome == Outcome::Held {
            eprintln!("emu: seq {}: {} (held)", t.seqnum, line(t, &self.log.rules()));
        }
        self.log.transfer(t);
    }

    fn violation(&mut self, message: &str) {
        self.log.violation(message);
    }

    fn notice(&mut self, notice: Notice<'_>) {
        if let (true, Notice::Comment(c)) = (self.verbose, &notice) {
            eprintln!("emu: {c}");
        }
        self.log.notice(notice);
    }
}

/// What builds the model `name` for a session, for the start and for each
/// reset, and gives `log` the rules of what it built.
///
/// `options` builds a model too, to refuse what it cannot take, so the
/// rules can be a `reset` ahead of the model; nothing compares what goes
/// between the two.
pub fn factory<W: Write + Send + 'static>(name: &str, log: Arc<Mutex<TxLog<W>>>) -> ModelFactory {
    let name = name.to_owned();
    Box::new(move |options: &[(String, String)]| {
        let model = models::build(&name, options)?;
        log.lock().expect("log lock").set_rules(models::log_rules(&name, options));
        Ok(model)
    })
}

#[cfg(test)]
mod tests {
    use vusb::bus::Bus;
    use vusb::device::Device;
    use vusb::model::Setup;
    use vusb::observer::SharedObserver;

    use super::*;

    fn pairs(options: &[(&str, &str)]) -> Vec<(String, String)> {
        options.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect()
    }

    /// Each time the factory builds the model, as a `reset` does, the log
    /// goes on under the rules of what it built, in the same file.
    #[test]
    fn the_rules_follow_the_model_the_factory_builds() {
        let log = Arc::new(Mutex::new(TxLog::new(Vec::new(), false)));
        let build = factory("ikalogic-scanalogic2", Arc::clone(&log));
        // A board's one feature report, 128 bytes.
        let read = |options: &[(&str, &str)]| {
            let model = build(&pairs(options)).unwrap();
            let shared: SharedObserver = log.clone();
            let mut bus = Bus::new(Device::new(model)).with_observer(shared);
            bus.notice(Notice::Section("read"));
            let configure = Setup { request_type: 0x00, request: 9, value: 1, index: 0, length: 0 };
            bus.control(configure, &[]);
            let report =
                Setup { request_type: 0xa1, request: 1, value: 0x0300, index: 0, length: 128 };
            bus.control(report, &[]);
        };
        read(&[]);
        read(&[("digest", "1")]);
        read(&[]);

        let log = log.lock().unwrap();
        let text = String::from_utf8_lossy(log.log.get_ref());
        let reports: Vec<&str> = text.lines().filter(|l| l.starts_with("ctl_in")).collect();
        assert_eq!(reports.len(), 3, "{text}");
        assert!(!reports[0].contains('@'), "{text}");
        assert!(reports[1].starts_with("ctl_in 21 01 0300 0000 128 -> @128:"), "{text}");
        assert_eq!(reports[2], reports[0], "{text}");
    }

    /// `build.rs` finds a model's `log_rules`, and a model without any
    /// has the default.
    #[test]
    fn a_model_brings_its_rules() {
        let rules = |name| models::log_rules(name, &[]);
        assert_eq!(rules("fx2lafw").digest_above, Some(HASH_ABOVE));
        assert!(rules("dreamsourcelab-dslogic").undefined.is_some());
        assert_eq!(rules("usbrelay").digest_above, None);
        assert!(rules("usbrelay").undefined.is_none());
    }

    #[test]
    fn comments_go_in_when_verbose() {
        let text = |verbose| {
            let mut log = TxLog::new(Vec::new(), verbose);
            log.notice(Notice::Section("run"));
            log.notice(Notice::Comment("usbip client connected"));
            String::from_utf8(log.log.into_inner()).unwrap()
        };
        assert_eq!(text(false), "# section run\n");
        assert_eq!(text(true), "# section run\n# usbip client connected\n");
    }
}
