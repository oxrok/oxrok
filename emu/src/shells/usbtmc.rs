// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>
// SPDX-License-Identifier: GPL-3.0-or-later OR Apache-2.0

//! A USBTMC function with the USB488 subclass, as the kernel's `usbtmc` and
//! libusb see it: one interface of class `fe/03/01`, a bulk OUT and a bulk IN
//! endpoint, the class's control requests, and the twelve byte header of the
//! bulk messages. A model never sees any of it: it implements [`Messages`],
//! which [`crate::shells::scpi::Engine`] does for an instrument.
//!
//! - A `DEV_DEP_MSG_OUT` hands its payload to [`Messages::message`].
//! - A `REQUEST_DEV_DEP_MSG_IN` opens a window for one `DEV_DEP_MSG_IN`. Until
//!   [`Messages::answer`] has something, a bulk IN gets a NAK, and each offer
//!   is one [`Messages::tick`]: the host's pace, about one per 20 ms.
//! - A new request replaces one nobody answered, as tools time out and go on
//!   without the abort the class asks for.
//!
//! Endpoints: [`EP_OUT`] 1 and [`EP_IN`] 2, so `--unplug in=2:N` counts bulk
//! INs. There is no interrupt endpoint: USB488 wants one only for `SR1`, and no
//! tool reads it. The kernel's driver asks for the capabilities whenever it
//! binds; `EMU_TMC=1` in a scenario keeps it away for the session
//! (`docs/USB.md`, "Writing a SCPI model").
//!
//! The options, all off by default ([`Faults`]): `stall=caps|ren|llo|local`,
//! `in-stall=N`, `empty-at=N`, `bad-tag-at=N`, `message=N`.

use crate::descriptor::{self, DeviceDescriptor};
use crate::model::{
    BULK, CLASS, Descriptors, DeviceModel, RECIPIENT_ENDPOINT, RECIPIENT_INTERFACE, Reply, Setup,
    Speed, Violations,
};
use crate::options::parse_number;
use crate::{Error, Result};

/// The bulk OUT endpoint, number 1.
pub const EP_OUT: u8 = 1;
/// The bulk IN endpoint, number 2.
pub const EP_IN: u8 = 2;

/// `MsgID` of a message from the host.
const DEV_DEP_MSG_OUT: u8 = 1;
/// `MsgID` of the host's request for a message, and of that message.
const DEV_DEP_MSG_IN: u8 = 2;
/// The header.
const HEADER: usize = 12;
/// `bmTransferAttributes`: the end of a message.
const EOM: u8 = 0x01;
/// The longest message taken; what is longer is flagged.
const LONGEST: usize = 1 << 20;

const INITIATE_ABORT_BULK_OUT: u8 = 1;
const CHECK_ABORT_BULK_OUT_STATUS: u8 = 2;
const INITIATE_ABORT_BULK_IN: u8 = 3;
const CHECK_ABORT_BULK_IN_STATUS: u8 = 4;
const INITIATE_CLEAR: u8 = 5;
const CHECK_CLEAR_STATUS: u8 = 6;
const GET_CAPABILITIES: u8 = 7;
const INDICATOR_PULSE: u8 = 64;
const READ_STATUS_BYTE: u8 = 128;
const REN_CONTROL: u8 = 160;
const GO_TO_LOCAL: u8 = 161;
const LOCAL_LOCKOUT: u8 = 162;

const STATUS_SUCCESS: u8 = 0x01;
/// USB488's device capabilities: it takes remote and local.
const CAP_RL1: u8 = 0x02;
/// USB488's device capabilities: it speaks SCPI.
const CAP_SCPI: u8 = 0x08;

/// What goes on behind the bulk messages.
pub trait Messages: Send {
    /// The payload of a `DEV_DEP_MSG_OUT`, without its padding.
    fn message(&mut self, data: &[u8], v: &mut Violations);
    /// What the device has to say now, whole; `None` NAKs.
    fn answer(&mut self) -> Option<Vec<u8>>;
    /// One offer of a bulk IN with nothing to answer.
    fn tick(&mut self);
    /// Whether the model reached its event point (`DeviceModel::mark`).
    fn mark(&mut self) -> bool {
        false
    }
    /// `INITIATE_CLEAR`: what waits on either side goes.
    fn clear(&mut self) {}
}

/// Who the device says it is.
#[derive(Clone, Debug)]
pub struct Identity {
    /// `idVendor`.
    pub vendor: u16,
    /// `idProduct`.
    pub product: u16,
    /// `iManufacturer`.
    pub manufacturer: String,
    /// `iProduct`.
    pub product_name: String,
    /// `iSerialNumber`.
    pub serial: String,
    /// The bus speed, which sets the packet sizes: 64 at full, 512 at high.
    pub speed: Speed,
    /// `RL1`: it says it takes `REN_CONTROL`, `GO_TO_LOCAL`, `LOCAL_LOCKOUT`.
    pub remote: bool,
}

impl Identity {
    /// A high speed device with no remote and local.
    pub fn new(vendor: u16, product: u16, manufacturer: &str, product_name: &str) -> Identity {
        Identity {
            vendor,
            product,
            manufacturer: manufacturer.to_owned(),
            product_name: product_name.to_owned(),
            serial: "EMU0001".to_owned(),
            speed: Speed::High,
            remote: false,
        }
    }
}

/// The shell's fault options.
#[derive(Clone, Debug, Default)]
#[expect(clippy::struct_excessive_bools, reason = "one switch per request a fault can stall")]
pub struct Faults {
    stall_caps: bool,
    stall_ren: bool,
    stall_llo: bool,
    stall_local: bool,
    in_stall: Option<u32>,
    empty_at: Option<u32>,
    bad_tag_at: Option<u32>,
    message: Option<usize>,
}

impl Faults {
    /// Takes `-o KEY=VALUE` if it is the shell's: `stall=caps|ren|llo|local`
    /// STALLs that request; `in-stall=N` halts the bulk IN after N answers,
    /// until a bus reset; `empty-at=N` sends an empty transfer before the Nth
    /// answer, as a DS1054Z can; `bad-tag-at=N` puts a wrong `bTag` on the Nth;
    /// `message=N` cuts an answer into messages of N bytes, with `EOM` on the
    /// last, so that the host requests each.
    ///
    /// # Errors
    ///
    /// On a value it cannot use.
    pub fn option(&mut self, key: &str, value: &str) -> Result<bool> {
        let number = |range: std::ops::RangeInclusive<u32>| parse_number(key, value, range);
        match key {
            "stall" => match value {
                "caps" => self.stall_caps = true,
                "ren" => self.stall_ren = true,
                "llo" => self.stall_llo = true,
                "local" => self.stall_local = true,
                _ => return Err(Error::Invalid(format!("stall={value}: caps, ren, llo or local"))),
            },
            "in-stall" => self.in_stall = Some(number(0..=1 << 24)?),
            "empty-at" => self.empty_at = Some(number(1..=1 << 24)?),
            "bad-tag-at" => self.bad_tag_at = Some(number(1..=1 << 24)?),
            "message" => self.message = Some(number(1..=1 << 24)? as usize),
            _ => return Ok(false),
        }
        Ok(true)
    }
}

/// The function: a [`DeviceModel`] around `M`.
#[expect(clippy::struct_excessive_bools, reason = "independent states of the function")]
pub struct Usbtmc<M> {
    name: &'static str,
    descriptors: Descriptors,
    remote: bool,
    faults: Faults,
    messages: M,
    /// `bTag` and `TransferSize` of the request not yet answered.
    request: Option<(u8, u32)>,
    /// What is left of the answer being sent, for further messages.
    answer: Vec<u8>,
    /// What is left of the message on the wire, for further transfers.
    sending: Vec<u8>,
    answers: u32,
    /// `empty-at` has sent its empty transfer.
    emptied: bool,
    stalled: bool,
    ren: bool,
}

impl<M: Messages> Usbtmc<M> {
    /// The function of `id` around `messages`.
    pub fn new(name: &'static str, id: &Identity, faults: Faults, messages: M) -> Usbtmc<M> {
        Usbtmc {
            name,
            descriptors: descriptors(id),
            remote: id.remote,
            faults,
            messages,
            request: None,
            answer: Vec::new(),
            sending: Vec::new(),
            answers: 0,
            emptied: false,
            stalled: false,
            ren: false,
        }
    }

    fn capabilities(&self) -> Vec<u8> {
        let mut caps = vec![0; 24];
        caps[0] = STATUS_SUCCESS;
        caps[2..4].copy_from_slice(&0x0100u16.to_le_bytes());
        caps[12..14].copy_from_slice(&0x0100u16.to_le_bytes());
        caps[14] = if self.remote { 0x02 } else { 0 };
        caps[15] = CAP_SCPI | if self.remote { CAP_RL1 } else { 0 };
        caps
    }

    fn bulk_out(&mut self, data: &[u8], v: &mut Violations) -> Reply {
        if data.len() < HEADER {
            v.report(format!("a bulk OUT of {} bytes has no USBTMC header", data.len()));
            return Reply::Stall;
        }
        let (id, tag, inverse) = (data[0], data[1], data[2]);
        if tag == 0 || inverse != !tag {
            v.report(format!("bTag {tag:#04x} with bTagInverse {inverse:#04x}"));
        }
        if data[3] != 0 || data[9..12] != [0, 0, 0] {
            v.report(format!("reserved bytes of a USBTMC header: {:02x?}", &data[..HEADER]));
        }
        let size = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        match id {
            DEV_DEP_MSG_OUT => self.message_out(tag, size, data, v),
            DEV_DEP_MSG_IN => {
                if data.len() != HEADER {
                    v.report(format!("REQUEST_DEV_DEP_MSG_IN of {} bytes", data.len()));
                }
                if size == 0 {
                    v.report("REQUEST_DEV_DEP_MSG_IN for 0 bytes");
                }
                if data[8] != 0 {
                    v.report(format!(
                        "REQUEST_DEV_DEP_MSG_IN with a TermChar this device says it lacks: \
                         attributes {:#04x}",
                        data[8]
                    ));
                }
                self.request = Some((tag, size));
                Reply::ack()
            }
            other => {
                v.report(format!("MsgID {other} is not one this device has"));
                Reply::Stall
            }
        }
    }

    fn message_out(&mut self, tag: u8, size: u32, data: &[u8], v: &mut Violations) -> Reply {
        let size = size as usize;
        let body = &data[HEADER..];
        if size == 0 {
            v.report("DEV_DEP_MSG_OUT of 0 bytes");
        }
        if size > LONGEST || body.len() < size {
            v.report(format!(
                "DEV_DEP_MSG_OUT {tag} says {size} bytes and carries {}: a message in one \
                 transfer is all a tool sends",
                body.len()
            ));
            return Reply::Stall;
        }
        if data[8] != EOM {
            v.report(format!("DEV_DEP_MSG_OUT without EOM: attributes {:#04x}", data[8]));
        }
        if !data.len().is_multiple_of(4)
            || body[size..].iter().any(|b| *b != 0)
            || body.len() - size > 3
        {
            v.report(format!("DEV_DEP_MSG_OUT of {size} bytes in {} is padded badly", data.len()));
        }
        // What was on its way is no longer wanted.
        self.sending.clear();
        self.answer.clear();
        self.messages.message(&body[..size], v);
        Reply::ack()
    }

    fn bulk_in(&mut self, max: usize) -> Reply {
        if self.stalled {
            return Reply::Stall;
        }
        if !self.sending.is_empty() {
            return self.next_transfer(max);
        }
        let Some((tag, size)) = self.request else {
            self.messages.tick();
            return Reply::Nak;
        };
        if self.answer.is_empty() {
            let Some(answer) = self.messages.answer() else {
                self.messages.tick();
                return Reply::Nak;
            };
            self.answer = answer;
            self.answers += 1;
            self.emptied = false;
        }
        if self.faults.in_stall.is_some_and(|n| self.answers > n) {
            self.stalled = true;
            return Reply::Stall;
        }
        if self.faults.empty_at == Some(self.answers) && !self.emptied {
            self.emptied = true;
            return Reply::Data(Vec::new());
        }
        self.request = None;
        let cap = self.faults.message.unwrap_or(usize::MAX).min(size as usize);
        let n = self.answer.len().min(cap);
        let eom = n == self.answer.len();
        let tag =
            if self.faults.bad_tag_at == Some(self.answers) { tag.wrapping_add(1) } else { tag };
        let mut out = Vec::with_capacity(HEADER + n + 3);
        out.extend_from_slice(&[DEV_DEP_MSG_IN, tag, !tag, 0]);
        out.extend_from_slice(&(n as u32).to_le_bytes());
        out.extend_from_slice(&[u8::from(eom), 0, 0, 0]);
        out.extend_from_slice(&self.answer[..n]);
        self.answer.drain(..n);
        while !out.len().is_multiple_of(4) {
            out.push(0);
        }
        self.sending = out;
        self.next_transfer(max)
    }

    fn next_transfer(&mut self, max: usize) -> Reply {
        let n = self.sending.len().min(max);
        Reply::Data(self.sending.drain(..n).collect())
    }

    /// `USBTMC_STATUS_SUCCESS`, and `tail`, as the class's requests answer.
    fn status(setup: Setup, tail: &[u8], v: &mut Violations) -> Reply {
        if !setup.is_in() {
            v.report(format!("request {} is an IN", setup.request));
            return Reply::Stall;
        }
        let mut answer = vec![STATUS_SUCCESS];
        answer.extend_from_slice(tail);
        if usize::from(setup.length) != answer.len() {
            v.report(format!(
                "request {} asks for {} bytes and has {}",
                setup.request,
                setup.length,
                answer.len()
            ));
        }
        Reply::Data(answer)
    }

    fn drop_transfer(&mut self) {
        self.sending.clear();
        self.answer.clear();
        self.request = None;
    }
}

fn descriptors(id: &Identity) -> Descriptors {
    let packet: u16 = if id.speed == Speed::High { 512 } else { 64 };
    let mut body = Vec::new();
    body.extend_from_slice(&descriptor::interface(0, 0, 2, [0xfe, 0x03, 0x01], 0));
    body.extend_from_slice(&descriptor::endpoint(EP_OUT, BULK, packet, 0));
    body.extend_from_slice(&descriptor::endpoint(0x80 | EP_IN, BULK, packet, 0));
    let device = DeviceDescriptor {
        usb: 0x0200,
        class: [0, 0, 0],
        max_packet0: 64,
        vendor: id.vendor,
        product: id.product,
        device: 0x0100,
        strings: [1, 2, 3],
        configurations: 1,
    };
    Descriptors {
        speed: id.speed,
        device: device.bytes(),
        configs: vec![descriptor::configuration(1, 0, 0x80, 100, &body)],
        strings: vec![id.manufacturer.clone(), id.product_name.clone(), id.serial.clone()],
    }
}

impl<M: Messages> DeviceModel for Usbtmc<M> {
    fn name(&self) -> &'static str {
        self.name
    }

    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn control(&mut self, setup: Setup, _data: &[u8], v: &mut Violations) -> Reply {
        let on_endpoint = matches!(
            setup.request,
            INITIATE_ABORT_BULK_OUT
                | CHECK_ABORT_BULK_OUT_STATUS
                | INITIATE_ABORT_BULK_IN
                | CHECK_ABORT_BULK_IN_STATUS
        );
        let wanted = if on_endpoint { RECIPIENT_ENDPOINT } else { RECIPIENT_INTERFACE };
        if setup.kind() != CLASS
            || setup.recipient() != wanted
            || (!on_endpoint && setup.index != 0)
        {
            v.report(format!(
                "request {:#04x} type {:#04x} index {} is not one the function has",
                setup.request, setup.request_type, setup.index
            ));
            return Reply::Stall;
        }
        match setup.request {
            GET_CAPABILITIES if self.faults.stall_caps => Reply::Stall,
            GET_CAPABILITIES => {
                if !setup.is_in() || setup.value != 0 || setup.length != 24 {
                    v.report(format!(
                        "GET_CAPABILITIES type {:#04x} wValue {} wLength {}",
                        setup.request_type, setup.value, setup.length
                    ));
                }
                Reply::Data(self.capabilities())
            }
            // The initiating requests answer the status and a tag, the checks the
            // status, three bytes and NBYTES.
            INITIATE_ABORT_BULK_OUT | INITIATE_ABORT_BULK_IN => {
                self.drop_transfer();
                Self::status(setup, &[(setup.value & 0xff) as u8], v)
            }
            CHECK_ABORT_BULK_OUT_STATUS | CHECK_ABORT_BULK_IN_STATUS => {
                Self::status(setup, &[0; 7], v)
            }
            CHECK_CLEAR_STATUS => Self::status(setup, &[0], v),
            INITIATE_CLEAR => {
                self.drop_transfer();
                self.messages.clear();
                Self::status(setup, &[], v)
            }
            READ_STATUS_BYTE => Self::status(setup, &[(setup.value & 0x7f) as u8, 0], v),
            REN_CONTROL | GO_TO_LOCAL | LOCAL_LOCKOUT if !self.remote => {
                v.report(format!(
                    "request {} on a device whose capabilities say no RL1",
                    setup.request
                ));
                Reply::Stall
            }
            REN_CONTROL if self.faults.stall_ren => Reply::Stall,
            LOCAL_LOCKOUT if self.faults.stall_llo => Reply::Stall,
            GO_TO_LOCAL if self.faults.stall_local => Reply::Stall,
            REN_CONTROL => {
                self.ren = setup.value & 1 != 0;
                Self::status(setup, &[], v)
            }
            LOCAL_LOCKOUT => {
                if !self.ren {
                    v.report("LOCAL_LOCKOUT before REN_CONTROL");
                }
                Self::status(setup, &[], v)
            }
            GO_TO_LOCAL | INDICATOR_PULSE => Self::status(setup, &[], v),
            other => {
                v.report(format!("request {other} is not one the function has"));
                Reply::Stall
            }
        }
    }

    fn out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        if ep != EP_OUT {
            v.report(format!("OUT to endpoint {ep}"));
            return Reply::Stall;
        }
        self.bulk_out(data, v)
    }

    fn in_transfer(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        if ep != EP_IN {
            v.report(format!("IN from endpoint {ep}"));
            return Reply::Stall;
        }
        self.bulk_in(max)
    }

    fn reset(&mut self) {
        self.drop_transfer();
        self.stalled = false;
        self.ren = false;
    }

    fn mark(&mut self) -> bool {
        self.messages.mark()
    }
}

/// A tool on the bulk endpoints, as seen from the function, for unit tests.
#[cfg(test)]
pub mod host {
    use super::*;

    /// The function with a tool at it, its tags counted as the tools count.
    pub struct Host<M> {
        /// The function.
        pub tmc: Usbtmc<M>,
        /// What it reported.
        pub v: Violations,
        /// The last `bTag`.
        pub tag: u8,
    }

    impl<M: Messages> Host<M> {
        /// The function, with no tag used.
        pub fn new(tmc: Usbtmc<M>) -> Host<M> {
            Host { tmc, v: Violations::default(), tag: 0 }
        }

        fn header(&mut self, id: u8, size: u32, attributes: u8) -> Vec<u8> {
            self.tag = self.tag.wrapping_add(1);
            if self.tag == 0 {
                self.tag = 1;
            }
            let mut m = vec![id, self.tag, !self.tag, 0];
            m.extend_from_slice(&size.to_le_bytes());
            m.extend_from_slice(&[attributes, 0, 0, 0]);
            m
        }

        /// A `DEV_DEP_MSG_OUT`.
        pub fn send(&mut self, text: &[u8]) -> Reply {
            let mut m = self.header(DEV_DEP_MSG_OUT, text.len() as u32, EOM);
            m.extend_from_slice(text);
            while !m.len().is_multiple_of(4) {
                m.push(0);
            }
            self.tmc.out(EP_OUT, &m, &mut self.v)
        }

        /// A `REQUEST_DEV_DEP_MSG_IN`.
        pub fn request(&mut self) -> Reply {
            let m = self.header(DEV_DEP_MSG_IN, 0x7fff_ffff, 0);
            self.tmc.out(EP_OUT, &m, &mut self.v)
        }

        /// One bulk IN of up to `max`.
        pub fn offer(&mut self, max: usize) -> Reply {
            self.tmc.in_transfer(EP_IN, max, &mut self.v)
        }

        /// A request and all of its message, header first; `None` on a NAK.
        pub fn read(&mut self) -> Option<Vec<u8>> {
            self.request();
            let Reply::Data(mut out) = self.offer(2048) else { return None };
            let size = u32::from_le_bytes([out[4], out[5], out[6], out[7]]) as usize;
            while out.len() < HEADER + size {
                let Reply::Data(more) = self.offer(2048) else { return None };
                out.extend(more);
            }
            Some(out)
        }

        /// The payload of [`Host::read`].
        pub fn answer(&mut self) -> Option<Vec<u8>> {
            let m = self.read()?;
            let size = u32::from_le_bytes([m[4], m[5], m[6], m[7]]) as usize;
            Some(m[HEADER..HEADER + size].to_vec())
        }
    }

    /// A class request to the interface.
    pub fn class_in(request: u8, value: u16, length: u16) -> Setup {
        Setup { request_type: 0xa1, request, value, index: 0, length }
    }
}

#[cfg(test)]
mod tests {
    use super::host::{Host, class_in};
    use super::*;

    /// Says back what it was told; `silent` holds its tongue.
    #[derive(Default)]
    struct Echo {
        said: Vec<Vec<u8>>,
        ticks: u32,
        silent: bool,
    }

    impl Messages for Echo {
        fn message(&mut self, data: &[u8], _v: &mut Violations) {
            self.said.push(data.to_vec());
        }
        fn answer(&mut self) -> Option<Vec<u8>> {
            if self.silent { None } else { self.said.pop() }
        }
        fn tick(&mut self) {
            self.ticks += 1;
        }
    }

    fn host(options: &[(&str, &str)]) -> Host<Echo> {
        let mut faults = Faults::default();
        for (k, v) in options {
            assert!(faults.option(k, v).unwrap(), "{k}");
        }
        let mut id = Identity::new(0x1234, 0x5678, "M", "P");
        id.remote = true;
        Host::new(Usbtmc::new("t", &id, faults, Echo::default()))
    }

    #[test]
    fn it_is_what_the_tools_look_for() {
        let h = host(&[]);
        let d = h.tmc.descriptors();
        let c = d.config_by_value(1).unwrap();
        assert_eq!(Descriptors::interfaces(c), [[0, 0, 0xfe, 0x03, 0x01]]);
        let e = Descriptors::endpoints(c);
        assert_eq!(
            e.iter().map(|e| (e.address, e.max_packet)).collect::<Vec<_>>(),
            [(0x01, 512), (0x82, 512)]
        );
    }

    #[test]
    fn a_message_comes_back_with_the_tag_of_its_request() {
        let mut h = host(&[]);
        assert_eq!(h.send(b"abc\n"), Reply::ack());
        let m = h.read().unwrap();
        assert_eq!(&m[..12], &[2, 2, !2, 0, 4, 0, 0, 0, 1, 0, 0, 0]);
        assert_eq!(&m[12..], b"abc\n");
        assert!(h.v.is_empty());
    }

    #[test]
    fn a_long_answer_goes_in_transfers_of_what_the_host_asks() {
        let mut h = host(&[]);
        h.tmc.messages.said.push(vec![7u8; 5000]);
        h.request();
        let Reply::Data(first) = h.offer(2048) else { panic!() };
        assert_eq!(first.len(), 2048);
        assert_eq!(&first[4..8], &5000u32.to_le_bytes());
        let Reply::Data(second) = h.offer(2048) else { panic!() };
        let Reply::Data(third) = h.offer(2048) else { panic!() };
        assert_eq!(second.len(), 2048);
        assert_eq!(first.len() + second.len() + third.len(), 5012);
        assert_eq!(h.offer(2048), Reply::Nak, "and no more");
    }

    #[test]
    fn a_new_message_drops_the_answer_on_its_way() {
        let mut h = host(&[]);
        h.tmc.messages.said.push(vec![7u8; 5000]);
        h.request();
        assert!(matches!(h.offer(2048), Reply::Data(d) if d.len() == 2048));
        h.send(b"new\n");
        assert_eq!(h.answer().unwrap(), b"new\n", "not the rest of the old one");
    }

    #[test]
    fn an_answer_keeps_to_what_the_request_asked_for() {
        let mut h = host(&[]);
        h.tmc.messages.said.push(b"abcdef".to_vec());
        let request = [DEV_DEP_MSG_IN, 1, !1, 0, 4, 0, 0, 0, 0, 0, 0, 0];
        assert_eq!(h.tmc.out(EP_OUT, &request, &mut h.v), Reply::ack());
        let Reply::Data(m) = h.offer(2048) else { panic!() };
        assert_eq!(&m[4..9], &[4, 0, 0, 0, 0], "four bytes, no EOM");
        assert_eq!(&m[12..], b"abcd");
    }

    #[test]
    fn an_answer_is_padded_to_four() {
        let mut h = host(&[]);
        h.send(b"abc\n");
        h.send(b"abcde\n");
        h.request();
        let Reply::Data(m) = h.offer(2048) else { panic!() };
        assert_eq!(m.len(), HEADER + 8, "{m:?}");
        assert_eq!(&m[4..8], &6u32.to_le_bytes());
    }

    #[test]
    fn no_request_or_no_answer_naks_and_counts_the_offer() {
        let mut h = host(&[]);
        assert_eq!(h.offer(64), Reply::Nak);
        h.request();
        assert_eq!(h.offer(64), Reply::Nak);
        assert_eq!(h.tmc.messages.ticks, 2);
        assert!(h.v.is_empty(), "a read that times out breaks no rule");
    }

    #[test]
    fn a_new_request_replaces_one_nobody_answered() {
        let mut h = host(&[]);
        h.tmc.messages.silent = true;
        h.request();
        assert_eq!(h.offer(64), Reply::Nak);
        h.tmc.messages.silent = false;
        h.send(b"x\n");
        let m = h.read().unwrap();
        assert_eq!(m[1], 3, "the tag of the last request");
        assert!(h.v.is_empty());
    }

    #[test]
    fn what_breaks_the_framing_is_flagged() {
        for (bytes, says) in [
            (vec![1u8, 2, 3], "no USBTMC header"),
            (vec![1, 0, 0xff, 0, 0, 0, 0, 0, 1, 0, 0, 0], "bTag 0x00"),
            (vec![1, 1, 0x00, 0, 0, 0, 0, 0, 1, 0, 0, 0], "bTagInverse"),
            (vec![9, 1, 0xfe, 0, 0, 0, 0, 0, 1, 0, 0, 0], "MsgID 9"),
            (vec![1, 1, 0xfe, 0, 4, 0, 0, 0, 1, 0, 0, 0, b'a'], "carries 1"),
            (vec![1, 1, 0xfe, 0, 1, 0, 0, 0, 0, 0, 0, 0, b'a', 0, 0, 0], "without EOM"),
            (vec![1, 1, 0xfe, 0, 1, 0, 0, 0, 1, 0, 0, 0, b'a'], "padded badly"),
            (vec![1, 1, 0xfe, 0, 1, 0, 0, 0, 1, 0, 0, 0, b'a', 9, 0, 0], "padded badly"),
            (vec![2, 1, 0xfe, 0, 0, 0, 0, 0, 0, 0, 0, 0], "for 0 bytes"),
            (vec![2, 1, 0xfe, 0, 9, 0, 0, 0, 2, 0, 0, 0], "TermChar"),
            (vec![1, 1, 0xfe, 1, 0, 0, 0, 0, 1, 0, 0, 0], "reserved bytes"),
            (vec![1, 1, 0xfe, 0, 0, 0, 0, 0, 1, 0, 0, 0], "of 0 bytes"),
        ] {
            let mut h = host(&[]);
            h.tmc.out(EP_OUT, &bytes, &mut h.v);
            let got = h.v.drain().join("; ");
            assert!(got.contains(says), "{bytes:?}: {got}");
        }
    }

    #[test]
    fn capabilities_say_what_the_tools_read() {
        let mut h = host(&[]);
        let Reply::Data(c) = h.tmc.control(class_in(GET_CAPABILITIES, 0, 24), &[], &mut h.v) else {
            panic!()
        };
        assert_eq!(c.len(), 24);
        assert_eq!((c[0], c[15] & CAP_RL1), (STATUS_SUCCESS, CAP_RL1));
        assert!(h.v.is_empty());
        h.tmc.control(class_in(GET_CAPABILITIES, 0, 2), &[], &mut h.v);
        h.tmc.control(class_in(GET_CAPABILITIES, 1, 24), &[], &mut h.v);
        assert_eq!(h.v.drain().len(), 2);
    }

    #[test]
    fn remote_control_is_checked() {
        let mut h = host(&[]);
        h.tmc.control(class_in(LOCAL_LOCKOUT, 0, 1), &[], &mut h.v);
        assert_eq!(h.v.drain().len(), 1, "a lockout before REN");
        h.tmc.control(class_in(REN_CONTROL, 1, 1), &[], &mut h.v);
        h.tmc.control(class_in(LOCAL_LOCKOUT, 0, 1), &[], &mut h.v);
        h.tmc.control(class_in(GO_TO_LOCAL, 0, 1), &[], &mut h.v);
        assert!(h.v.is_empty());
        let id = Identity::new(1, 2, "M", "P");
        let mut plain = Host::new(Usbtmc::new("t", &id, Faults::default(), Echo::default()));
        let r = plain.tmc.control(class_in(REN_CONTROL, 1, 1), &[], &mut plain.v);
        assert_eq!(r, Reply::Stall);
        assert_eq!(plain.v.drain().len(), 1, "no RL1 says the device");
    }

    #[test]
    fn the_class_requests_answer_as_the_class_says() {
        let mut h = host(&[]);
        h.send(b"a\n");
        h.request();
        let clear = h.tmc.control(class_in(INITIATE_CLEAR, 0, 1), &[], &mut h.v);
        assert_eq!(clear, Reply::Data(vec![STATUS_SUCCESS]));
        let check = h.tmc.control(class_in(CHECK_CLEAR_STATUS, 0, 2), &[], &mut h.v);
        assert_eq!(check, Reply::Data(vec![STATUS_SUCCESS, 0]));
        assert_eq!(h.offer(64), Reply::Nak, "the request went with the clear");
        let pulse = h.tmc.control(class_in(INDICATOR_PULSE, 0, 1), &[], &mut h.v);
        assert_eq!(pulse, Reply::Data(vec![STATUS_SUCCESS]));
        let stb = h.tmc.control(class_in(READ_STATUS_BYTE, 2, 3), &[], &mut h.v);
        assert_eq!(stb, Reply::Data(vec![STATUS_SUCCESS, 2, 0]));
        let abort = Setup {
            request_type: 0xa2,
            request: INITIATE_ABORT_BULK_IN,
            value: 7,
            index: 0x82,
            length: 2,
        };
        let answer = h.tmc.control(abort, &[], &mut h.v);
        assert_eq!(answer, Reply::Data(vec![STATUS_SUCCESS, 7]), "as the kernel asks");
        let check = Setup { request: CHECK_ABORT_BULK_OUT_STATUS, length: 8, ..abort };
        let answer = h.tmc.control(Setup { index: 1, ..check }, &[], &mut h.v);
        assert!(matches!(answer, Reply::Data(d) if d.len() == 8), "as the kernel asks");
        assert!(h.v.is_empty());
        let on_the_wrong_one = Setup { index: 3, ..class_in(GET_CAPABILITIES, 0, 24) };
        assert_eq!(h.tmc.control(on_the_wrong_one, &[], &mut h.v), Reply::Stall);
        assert_eq!(h.v.drain().len(), 1);
    }

    #[test]
    fn the_faults_do_what_they_say() {
        let mut h = host(&[("empty-at", "1")]);
        h.send(b"a\n");
        h.request();
        assert_eq!(h.offer(2048), Reply::Data(Vec::new()), "the empty one first");
        let Reply::Data(m) = h.offer(2048) else { panic!() };
        assert_eq!(&m[12..14], b"a\n", "then the message");

        let mut h = host(&[("bad-tag-at", "1")]);
        h.send(b"a\n");
        let m = h.read().unwrap();
        assert_eq!(m[1], 3, "the request's tag is 2");

        let mut h = host(&[("message", "2")]);
        h.send(b"abc\n");
        let m = h.read().unwrap();
        assert_eq!((m[4], m[8], &m[12..14]), (2, 0, &b"ab"[..]), "no EOM");
        let m = h.read().unwrap();
        assert_eq!((m[4], m[8], &m[12..14]), (2, 1, &b"c\n"[..]), "EOM on the last");

        let mut h = host(&[("in-stall", "0")]);
        h.send(b"a\n");
        h.request();
        assert_eq!(h.offer(64), Reply::Stall);
        assert_eq!(h.offer(64), Reply::Stall, "until a reset");
        h.tmc.reset();
        assert_eq!(h.offer(64), Reply::Nak, "a reset drops the request");

        let mut h = host(&[("stall", "caps")]);
        let r = h.tmc.control(class_in(GET_CAPABILITIES, 0, 24), &[], &mut h.v);
        assert_eq!(r, Reply::Stall);
        assert!(h.v.is_empty());
    }
}
