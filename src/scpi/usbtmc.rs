// Counterpart in sigrok:
//   libsigrok/src/scpi/scpi_usbtmc_libusb.c,
//   Copyright (C) 2014 Aurelien Jacobs <aurel@gnuage.org>,
//   with commits by Martin Ling, Bert Vermeulen, Uwe Hermann, Gerhard Sittig,
//   Soeren Apel, Daniel Elstner, Wolfram Sang, Timo Kokkonen
//   (libsigrok libsigrok-unreleased-1855-g20baa77f).
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it.

//! USBTMC with the USB488 subclass: SCPI messages in bulk transfers
//! with a twelve byte header.

use crate::core::{Error, Result};
use crate::scpi::ScpiTransport;
use crate::usb::{ControlRequest, Transport, UsbDeviceInfo};
use std::time::Duration;

const CLASS_APPLICATION: u8 = 0xfe;
const SUBCLASS_USBTMC: u8 = 0x03;
const PROTOCOL_USB488: u8 = 0x01;
const TRANSFER_TYPE_BULK: u8 = 2;

const MAX_TRANSFER: usize = 2048;
const TIMEOUT: Duration = Duration::from_secs(1);
const HEADER: usize = 12;

const DEV_DEP_MSG_OUT: u8 = 1;
const REQUEST_DEV_DEP_MSG_IN: u8 = 2;
const DEV_DEP_MSG_IN: u8 = 2;
const EOM: u8 = 0x01;

const INITIATE_CLEAR: u8 = 5;
const CHECK_CLEAR_STATUS: u8 = 6;
const GET_CAPABILITIES: u8 = 7;
const REN_CONTROL: u8 = 160;
const GO_TO_LOCAL: u8 = 161;
const LOCAL_LOCKOUT: u8 = 162;
const STATUS_SUCCESS: u8 = 0x01;
const STATUS_PENDING: u8 = 0x02;
/// How many times a clear's status is asked while it says pending, 10 ms apart.
const CLEAR_CHECKS: u32 = 100;
const USB488_CAP_RL1: u8 = 0x02;

/// Devices that misbehave when put into remote mode.
const NO_REMOTE: [(u16, u16); 7] = [
    (0x1ab1, 0x0588), // Rigol DS1000
    (0x1ab1, 0x04b0), // Rigol DS2000
    (0x1ab1, 0x04b1), // Rigol DS4000
    (0x1ab1, 0x0515), // Rigol MSO5000
    (0x0957, 0x0588), // Agilent DSO1000 (a Rigol DS1000)
    (0x0b21, 0xffff), // all Yokogawa
    (0xf4ec, 0xffff), // all Siglent SDS
];
/// Devices that want a USB reset after the interface is claimed.
const RESET_ON_OPEN: [(u16, u16); 1] = [(0xf4ec, 0xffff)];

fn listed(table: &[(u16, u16)], vid: u16, pid: u16) -> bool {
    table.iter().any(|(v, p)| *v == vid && (*p == 0xffff || *p == pid))
}

/// The USBTMC devices on the machine.
pub fn list() -> Result<Vec<UsbDeviceInfo>> {
    Ok(crate::usb::list_devices()?
        .into_iter()
        .filter(|d| {
            d.interfaces.iter().any(|i| {
                i.class == CLASS_APPLICATION
                    && i.subclass == SUBCLASS_USBTMC
                    && i.protocol == PROTOCOL_USB488
            })
        })
        .collect())
}

/// An open USBTMC device.
pub struct Usbtmc {
    usb: Option<Box<dyn Transport>>,
    /// The interface is ours: an open that has not been closed.
    claimed: bool,
    /// Where it opens again; a replay's transport has no such place.
    info: Option<UsbDeviceInfo>,
    resource: String,
    interface: u8,
    bulk_in: u8,
    bulk_out: u8,
    usb488_caps: u8,
    tag: u8,
    remote: bool,
    buffer: Vec<u8>,
    response_length: usize,
    response_read: usize,
    remaining: usize,
    attributes: u8,
    /// The tag of a `REQUEST_DEV_DEP_MSG_IN` whose answer hasn't begun to come.
    outstanding: Option<u8>,
    /// A read after a timeout waits for that request's answer instead of making
    /// another: [`ScpiTransport::wait_out_requests`].
    wait_out: bool,
}

fn class_in(request: u8, value: u16, index: u16) -> ControlRequest {
    ControlRequest { request_type: 0x21, request, value, index }
}

impl Usbtmc {
    /// Opens `info`, claims its USBTMC interface and reads its
    /// capabilities.
    pub fn open(info: &UsbDeviceInfo) -> Result<Usbtmc> {
        let usb = crate::usb::open(info)?;
        let mut dev = Self::from_transport(usb, format!("usbtmc/{}", info.conn()))?;
        dev.info = Some(info.clone());
        Ok(dev)
    }

    /// Sets up USBTMC on an open transport, e.g. a replay.
    pub fn from_transport(usb: Box<dyn Transport>, resource: String) -> Result<Usbtmc> {
        let mut dev = Usbtmc {
            usb: Some(usb),
            claimed: false,
            info: None,
            resource,
            interface: 0,
            bulk_in: 0,
            bulk_out: 0,
            usb488_caps: 0,
            tag: 0,
            remote: false,
            buffer: Vec::new(),
            response_length: 0,
            response_read: 0,
            remaining: 0,
            attributes: 0,
            outstanding: None,
            wait_out: false,
        };
        dev.start()?;
        Ok(dev)
    }

    fn usb(&mut self) -> Result<&mut Box<dyn Transport>> {
        self.usb.as_mut().ok_or_else(|| Error::device("USBTMC: the device is closed"))
    }

    /// What an open does on the wire: the interface, the capabilities, remote.
    fn start(&mut self) -> Result<()> {
        let usb = self.usb()?;
        let interfaces = usb.interfaces()?;
        let iface = interfaces
            .iter()
            .find(|i| {
                i.class.class == CLASS_APPLICATION
                    && i.class.subclass == SUBCLASS_USBTMC
                    && i.class.protocol == PROTOCOL_USB488
            })
            .ok_or_else(|| Error::device("Failed to find USBTMC interface."))?;
        let bulk_in = iface
            .endpoints
            .iter()
            .find(|e| e.transfer_type == TRANSFER_TYPE_BULK && e.address & 0x80 != 0)
            .map(|e| e.address)
            .ok_or_else(|| Error::device("USBTMC: no bulk IN endpoint"))?;
        let bulk_out = iface
            .endpoints
            .iter()
            .find(|e| e.transfer_type == TRANSFER_TYPE_BULK && e.address & 0x80 == 0)
            .map(|e| e.address)
            .ok_or_else(|| Error::device("USBTMC: no bulk OUT endpoint"))?;
        let interface = iface.class.number;
        usb.claim_interface(interface)?;
        let (vid, pid) = (usb.info().vendor_id, usb.info().product_id);
        if listed(&RESET_ON_OPEN, vid, pid) {
            usb.reset()?;
            // libusb_reset_device claims again what was claimed; nusb's reset doesn't,
            // and the backends' `reset` both forget it.
            usb.claim_interface(interface)?;
        }
        let mut usb488_caps = 0;
        if let Ok(caps) =
            usb.control_in(class_in(GET_CAPABILITIES, 0, u16::from(interface)), 24, TIMEOUT)
            && caps.len() == 24
        {
            usb488_caps = caps[15];
            log::debug!(
                "USBTMC capabilities: int {:#04x} dev {:#04x} usb488 {:#04x}",
                caps[4],
                caps[5],
                caps[15]
            );
        }
        self.claimed = true;
        self.interface = interface;
        self.bulk_in = bulk_in;
        self.bulk_out = bulk_out;
        self.usb488_caps = usb488_caps;
        self.remote = usb488_caps & USB488_CAP_RL1 != 0 && !listed(&NO_REMOTE, vid, pid);
        // A failure here leaves the instrument as it is, and open goes on
        // (scpi_usbtmc_remote's result is dropped at its one call).
        if self.remote
            && self.control(REN_CONTROL, 1).is_ok()
            && let Err(e) = self.control(LOCAL_LOCKOUT, 0)
        {
            log::debug!("Failed to enter local lockout state: {e}.");
        }
        Ok(())
    }

    fn control(&mut self, request: u8, value: u16) -> Result<()> {
        let setup = class_in(request, value, u16::from(self.interface));
        let status = self.usb()?.control_in(setup, 1, TIMEOUT)?;
        if status.first() != Some(&STATUS_SUCCESS) {
            return Err(Error::device(format!("USBTMC request {request} failed: {status:?}")));
        }
        Ok(())
    }

    fn next_tag(&mut self) -> u8 {
        self.tag = self.tag.wrapping_add(1);
        if self.tag == 0 {
            self.tag = 1;
        }
        self.tag
    }

    fn bulk_out(&mut self, msg_id: u8, data: &[u8], size: u32, attributes: u8) -> Result<()> {
        if data.len() + HEADER + 3 > MAX_TRANSFER {
            return Err(Error::device("USBTMC bulk out transfer is too big."));
        }
        let tag = self.next_tag();
        let mut msg = Vec::with_capacity(HEADER + data.len() + 3);
        msg.extend([msg_id, tag, !tag, 0]);
        msg.extend(size.to_le_bytes());
        msg.extend([attributes, 0, 0, 0]);
        msg.extend_from_slice(data);
        while !msg.len().is_multiple_of(4) {
            msg.push(0);
        }
        let endpoint = self.bulk_out;
        let sent = self.usb()?.bulk_out(endpoint, &msg, TIMEOUT)?;
        if sent < msg.len() {
            return Err(Error::device(format!("USBTMC short bulk out: {sent}/{}", msg.len())));
        }
        Ok(())
    }

    fn bulk_in_start(&mut self, timeout: Duration) -> Result<()> {
        let mut data = Vec::new();
        for attempt in 0..2 {
            let endpoint = self.bulk_in;
            data = self.usb()?.bulk_in(endpoint, MAX_TRANSFER, timeout)?;
            if data.is_empty() && attempt == 0 {
                // A Rigol DS1054Z sends an empty transfer at times and
                // the real message right after.
                log::warn!("USBTMC bulk in start was empty; retrying");
                continue;
            }
            break;
        }
        if data.len() < HEADER {
            return Err(Error::device(format!(
                "USBTMC bulk in returned too little data: {} bytes",
                data.len()
            )));
        }
        // An answer has begun, right or wrong: the request is spent.
        self.outstanding = None;
        if data[0] != DEV_DEP_MSG_IN || data[1] != self.tag || data[2] != !self.tag {
            return Err(Error::device("USBTMC invalid bulk in header."));
        }
        let message_size =
            u32::from_le_bytes([data[4], data[5], data[6], data[7]]) as usize + HEADER;
        self.attributes = data[8];
        self.response_length = data.len().min(message_size);
        self.response_read = HEADER;
        self.remaining = message_size - self.response_length;
        self.buffer = data;
        Ok(())
    }

    fn bulk_in_continue(&mut self, timeout: Duration) -> Result<()> {
        let endpoint = self.bulk_in;
        let data = self.usb()?.bulk_in(endpoint, MAX_TRANSFER, timeout)?;
        self.response_length = data.len().min(self.remaining);
        self.response_read = 0;
        self.remaining -= self.response_length;
        self.buffer = data;
        Ok(())
    }

    fn begin(&mut self, timeout: Duration) -> Result<()> {
        self.remaining = 0;
        // A request already made, waited out: its tag is the last one sent.
        if !(self.wait_out && self.outstanding.is_some()) {
            // INT32_MAX, as libsigrok asks: "all you have".
            self.bulk_out(REQUEST_DEV_DEP_MSG_IN, &[], 0x7fff_ffff, 0)?;
            self.outstanding = Some(self.tag);
        }
        self.bulk_in_start(timeout)
    }

    /// Whether a message has begun and not ended, so that a read goes on with it.
    fn in_a_message(&self) -> bool {
        self.response_read < self.response_length
            || self.remaining > 0
            || (self.response_length > 0 && self.attributes & EOM == 0)
    }
}

impl Drop for Usbtmc {
    fn drop(&mut self) {
        let _ = ScpiTransport::close(self);
    }
}

impl ScpiTransport for Usbtmc {
    fn resource(&self) -> &str {
        &self.resource
    }

    fn send(&mut self, data: &[u8]) -> Result<()> {
        // What a request still out may bring is the caller's to clear: the next
        // question has its own request.
        self.outstanding = None;
        self.bulk_out(DEV_DEP_MSG_OUT, data, data.len() as u32, EOM)
    }

    /// libsigrok makes a request at each read. Waiting out, a request already made
    /// is read for again, and a message begun is read on, so a timeout costs no
    /// answer: a second request would get an answer with the old tag, which is no
    /// valid header for the new one.
    fn read_begin(&mut self) -> Result<()> {
        if self.wait_out {
            if self.outstanding.is_some() {
                return self.begin(TIMEOUT);
            }
            if self.in_a_message() {
                return Ok(());
            }
        }
        self.begin(TIMEOUT)
    }

    fn wait_out_requests(&mut self, on: bool) {
        self.wait_out = on;
    }

    /// `INITIATE_CLEAR`, then `CHECK_CLEAR_STATUS` until it isn't pending: the
    /// instrument drops what it holds, answers to old questions among them. The
    /// class asks for a clear of the bulk OUT endpoint's halt after, which this
    /// transport has no call for, and a device that needs it fails the next write.
    fn clear(&mut self) -> Result<bool> {
        self.outstanding = None;
        self.response_length = 0;
        self.response_read = 0;
        self.remaining = 0;
        let interface = u16::from(self.interface);
        let status = self.usb()?.control_in(class_in(INITIATE_CLEAR, 0, interface), 1, TIMEOUT)?;
        if status.first() != Some(&STATUS_SUCCESS) {
            return Err(Error::device(format!("USBTMC INITIATE_CLEAR failed: {status:?}")));
        }
        for _ in 0..CLEAR_CHECKS {
            let check =
                self.usb()?.control_in(class_in(CHECK_CLEAR_STATUS, 0, interface), 2, TIMEOUT)?;
            match check.first() {
                Some(&STATUS_PENDING) => std::thread::sleep(Duration::from_millis(10)),
                Some(&STATUS_SUCCESS) => return Ok(true),
                _ => return Err(Error::device(format!("USBTMC clear status: {check:?}"))),
            }
        }
        Err(Error::device("USBTMC clear still pending"))
    }

    fn read_data(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        if self.response_read >= self.response_length {
            if self.remaining > 0 {
                self.bulk_in_continue(timeout)?;
            } else if self.attributes & EOM != 0 {
                return Err(Error::device("USBTMC: read past the end of the message"));
            } else {
                self.begin(timeout)?;
            }
        }
        let n = buf.len().min(self.response_length - self.response_read);
        buf[..n].copy_from_slice(&self.buffer[self.response_read..self.response_read + n]);
        self.response_read += n;
        Ok(n)
    }

    fn read_complete(&self) -> bool {
        self.response_read >= self.response_length
            && self.remaining == 0
            && self.attributes & EOM != 0
    }

    fn tags_answers(&self) -> bool {
        true
    }

    /// `bTag` goes on from where it was, as an instance's does in libsigrok.
    fn close(&mut self) -> Result<()> {
        if !std::mem::take(&mut self.claimed) {
            return Ok(());
        }
        if self.remote {
            let _ = self.control(GO_TO_LOCAL, 0);
        }
        let interface = self.interface;
        let released = self.usb()?.release_interface(interface);
        // A device that came from a replay stays, with nothing to open again.
        if self.info.is_some() {
            self.usb = None;
        }
        released
    }

    /// Nothing when it is open.
    fn reopen(&mut self) -> Result<()> {
        if self.claimed {
            return Ok(());
        }
        if self.usb.is_none()
            && let Some(info) = &self.info
        {
            self.usb = Some(crate::usb::open(info)?);
        }
        self.start()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::{BulkStream, EndpointDesc, InterfaceClass, InterfaceDesc};
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};

    /// What the fake device saw, and the bulk INs it has to give, in turn: `None`
    /// for one that times out.
    #[derive(Default)]
    struct Seen {
        ops: Vec<String>,
        ins: VecDeque<Option<Vec<u8>>>,
        clear_pending: u32,
    }

    struct Fake {
        info: UsbDeviceInfo,
        seen: Arc<Mutex<Seen>>,
    }

    impl Fake {
        fn seen(&self) -> std::sync::MutexGuard<'_, Seen> {
            self.seen.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
        }
    }

    impl Transport for Fake {
        fn info(&self) -> &UsbDeviceInfo {
            &self.info
        }
        fn set_configuration(&mut self, _configuration: u8) -> Result<()> {
            Ok(())
        }
        fn claim_interface(&mut self, interface: u8) -> Result<()> {
            self.seen().ops.push(format!("claim {interface}"));
            Ok(())
        }
        fn release_interface(&mut self, _interface: u8) -> Result<()> {
            Ok(())
        }
        fn reset(&mut self) -> Result<()> {
            self.seen().ops.push("reset".to_owned());
            Ok(())
        }
        fn max_packet_size(&mut self, _endpoint: u8) -> Result<usize> {
            Ok(512)
        }
        fn interfaces(&mut self) -> Result<Vec<InterfaceDesc>> {
            let bulk = |address| EndpointDesc { address, transfer_type: 2, max_packet_size: 512 };
            Ok(vec![InterfaceDesc {
                class: InterfaceClass { number: 0, class: 0xfe, subclass: 3, protocol: 1 },
                endpoints: vec![bulk(0x82), bulk(0x01)],
            }])
        }
        fn control_out(&mut self, _r: ControlRequest, _d: &[u8], _t: Duration) -> Result<()> {
            Ok(())
        }
        fn control_in(&mut self, r: ControlRequest, _l: usize, _t: Duration) -> Result<Vec<u8>> {
            let mut seen = self.seen();
            seen.ops.push(format!("ctl {}", r.request));
            Ok(match r.request {
                GET_CAPABILITIES => vec![0; 24],
                CHECK_CLEAR_STATUS if seen.clear_pending > 0 => {
                    seen.clear_pending -= 1;
                    vec![STATUS_PENDING, 0]
                }
                _ => vec![STATUS_SUCCESS, 0],
            })
        }
        fn bulk_out(&mut self, _e: u8, data: &[u8], _t: Duration) -> Result<usize> {
            let what = if data[0] == REQUEST_DEV_DEP_MSG_IN { "request" } else { "message" };
            self.seen().ops.push(format!("{what} {}", data[1]));
            Ok(data.len())
        }
        fn bulk_in(&mut self, _e: u8, _l: usize, _t: Duration) -> Result<Vec<u8>> {
            match self.seen().ins.pop_front() {
                Some(Some(data)) => Ok(data),
                _ => Err(Error::Timeout),
            }
        }
        fn interrupt_out(&mut self, _e: u8, data: &[u8], _t: Duration) -> Result<usize> {
            Ok(data.len())
        }
        fn interrupt_in(&mut self, _e: u8, _l: usize, _t: Duration) -> Result<Vec<u8>> {
            Err(Error::Timeout)
        }
        fn bulk_in_stream(&mut self, _e: u8, _s: usize, _d: usize) -> Result<Box<dyn BulkStream>> {
            Err(Error::not_supported("stream"))
        }
    }

    /// The first transfer of an answer to the request with `tag`, `whole` long.
    fn answer_of(tag: u8, whole: usize, text: &[u8]) -> Vec<u8> {
        let mut data = vec![DEV_DEP_MSG_IN, tag, !tag, 0];
        data.extend((whole as u32).to_le_bytes());
        data.extend([EOM, 0, 0, 0]);
        data.extend_from_slice(text);
        data
    }

    fn answer(tag: u8, text: &[u8]) -> Vec<u8> {
        answer_of(tag, text.len(), text)
    }

    fn open(vendor_id: u16) -> (Usbtmc, Arc<Mutex<Seen>>) {
        let seen = Arc::new(Mutex::new(Seen::default()));
        let info = UsbDeviceInfo { vendor_id, product_id: 0x1621, ..UsbDeviceInfo::default() };
        let tmc = Usbtmc::from_transport(
            Box::new(Fake { info, seen: Arc::clone(&seen) }),
            "usbtmc/fake".to_owned(),
        )
        .unwrap();
        seen.lock().unwrap().ops.clear();
        (tmc, seen)
    }

    fn ops(seen: &Arc<Mutex<Seen>>) -> Vec<String> {
        std::mem::take(&mut seen.lock().unwrap().ops)
    }

    /// nusb's reset drops the claim, which libusb's gives back.
    #[test]
    fn a_reset_at_the_open_claims_again() {
        let seen = Arc::new(Mutex::new(Seen::default()));
        let info = UsbDeviceInfo { vendor_id: 0xf4ec, ..UsbDeviceInfo::default() };
        let tmc =
            Usbtmc::from_transport(Box::new(Fake { info, seen: Arc::clone(&seen) }), String::new());
        drop(tmc);
        assert_eq!(ops(&seen)[..4], ["claim 0", "reset", "claim 0", "ctl 7"]);
        let info = UsbDeviceInfo { vendor_id: 0x1ab1, ..UsbDeviceInfo::default() };
        let tmc =
            Usbtmc::from_transport(Box::new(Fake { info, seen: Arc::clone(&seen) }), String::new());
        drop(tmc);
        assert_eq!(ops(&seen)[..2], ["claim 0", "ctl 7"], "no reset, one claim");
    }

    /// libsigrok's way, kept by default: each read makes a request.
    #[test]
    fn by_default_a_read_after_a_timeout_asks_again() {
        let (mut tmc, seen) = open(0x1ab1);
        tmc.send(b"Q?\n").unwrap();
        assert!(matches!(tmc.read_begin(), Err(Error::Timeout)));
        seen.lock().unwrap().ins.push_back(Some(answer(3, b"1\n")));
        tmc.read_begin().unwrap();
        assert_eq!(ops(&seen), ["message 1", "request 2", "request 3"]);
    }

    /// Waiting out: one request, read for until its answer comes, with its tag.
    #[test]
    fn waiting_out_a_request_reads_its_answer() {
        let (mut tmc, seen) = open(0x1ab1);
        tmc.wait_out_requests(true);
        tmc.send(b"Q?\n").unwrap();
        assert!(matches!(tmc.read_begin(), Err(Error::Timeout)));
        assert!(matches!(tmc.read_begin(), Err(Error::Timeout)));
        seen.lock().unwrap().ins.push_back(Some(answer(2, b"1\n")));
        tmc.read_begin().unwrap();
        let mut buf = [0u8; 16];
        assert_eq!(tmc.read_data(&mut buf, TIMEOUT).unwrap(), 2);
        assert!(tmc.read_complete());
        assert_eq!(ops(&seen), ["message 1", "request 2"]);
        // A new question makes its own request.
        tmc.send(b"Q?\n").unwrap();
        seen.lock().unwrap().ins.push_back(Some(answer(4, b"2\n")));
        tmc.read_begin().unwrap();
        assert_eq!(ops(&seen), ["message 3", "request 4"]);
    }

    /// A message cut by a timeout is read on, not asked for again.
    #[test]
    fn waiting_out_a_message_in_pieces_reads_on() {
        let (mut tmc, seen) = open(0x1ab1);
        tmc.wait_out_requests(true);
        tmc.send(b"Q?\n").unwrap();
        seen.lock().unwrap().ins.push_back(Some(answer_of(2, 5, b"12")));
        tmc.read_begin().unwrap();
        let mut buf = [0u8; 16];
        assert_eq!(tmc.read_data(&mut buf, TIMEOUT).unwrap(), 2);
        assert!(!tmc.read_complete());
        assert!(matches!(tmc.read_data(&mut buf, TIMEOUT), Err(Error::Timeout)));
        tmc.read_begin().unwrap();
        seen.lock().unwrap().ins.push_back(Some(b".5\n".to_vec()));
        assert_eq!(tmc.read_data(&mut buf, TIMEOUT).unwrap(), 3);
        assert_eq!(&buf[..3], b".5\n");
        assert!(tmc.read_complete());
        assert_eq!(ops(&seen), ["message 1", "request 2"]);
    }

    #[test]
    fn a_clear_asks_until_it_is_done_and_forgets_the_request() {
        let (mut tmc, seen) = open(0x1ab1);
        tmc.wait_out_requests(true);
        tmc.send(b"Q?\n").unwrap();
        assert!(matches!(tmc.read_begin(), Err(Error::Timeout)));
        seen.lock().unwrap().clear_pending = 2;
        assert!(tmc.clear().unwrap());
        assert_eq!(ops(&seen), ["message 1", "request 2", "ctl 5", "ctl 6", "ctl 6", "ctl 6"]);
        assert!(matches!(tmc.read_begin(), Err(Error::Timeout)));
        assert_eq!(ops(&seen), ["request 3"], "a new request after a clear");
    }
}
