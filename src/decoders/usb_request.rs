// Counterpart in sigrok:
//   libsigrokdecode/decoders/usb_request/pd.py,
//   Copyright (C) 2015 Stefan Brüns <stefan.bruens@rwth-aachen.de>.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like
//   it, written from the USB transaction/request rules and the decoder's
//   documented classes and texts.
//
// Subtle points of the Python original that this port keeps:
//   - Requests are tracked per (address, endpoint). A SETUP token that
//     arrives for an existing control request ends that request; the
//     BULK/INTERRUPT branches treat a new transaction as a request start
//     and complete it on the first ACKed transaction with data.
//   - `self.handshake` is not reset between transactions, so a token
//     seen before this transaction's handshake still sees the previous
//     handshake. The timeout path replaces it with 'timeout'.
//   - The transaction timeout is the token's span plus half of it, and
//     the previous request is ended at that computed sample.
//   - The PCAP output is Linux usbmon (LINKTYPE 220): a global header
//     once, then a record header and a packet for SUBMIT and COMPLETE.
//     The record header's captured/original lengths are always
//     `64 + len(data)`, even if the 8 byte setup field was not full.
//   - `in_request_start=first-ack` makes BULK IN annotations start at
//     the first data transaction; a BULK IN with no data start then
//     fails the way Python's `max(None, 0)` does.
//   - Timestamps are `int(sample * secs_per_sample)` and the truncated
//     microsecond part of that float.

//! USB (low-speed/full-speed) transactions and requests.

use crate::core::{Error, Result};
use crate::decode::{
    AnnClass, AnnRow, BinClass, Config, Context, Decoder, DecoderDef, DecoderInfo, Item,
    OptionDesc, OptionValue,
};
use std::collections::HashMap;

// Annotation class indices, as the libsigrokdecode decoder numbers them.
const ANN_SETUP_READ: usize = 0;
const ANN_SETUP_WRITE: usize = 1;
const ANN_BULK_READ: usize = 2;
const ANN_BULK_WRITE: usize = 3;
const ANN_ERRORS: usize = 4;
const BIN_PCAP: usize = 0;

static OPTIONS: [OptionDesc; 1] = [OptionDesc {
    id: "in_request_start",
    desc: "Start IN requests on",
    default: OptionValue::s("submit"),
    values: &[OptionValue::s("submit"), OptionValue::s("first-ack")],
}];

/// The decoder description.
pub static INFO: DecoderInfo = DecoderInfo {
    id: "usb_request",
    name: "USB request",
    longname: "Universal Serial Bus (LS/FS) transaction/request",
    desc: "USB (low-speed/full-speed) transaction/request protocol.",
    license: "gplv2+",
    inputs: &["usb_packet"],
    outputs: &["usb_request"],
    tags: &["PC"],
    channels: &[],
    optional_channels: &[],
    options: &OPTIONS,
    annotations: &[
        AnnClass { id: "request-setup-read", desc: "Setup: Device-to-host" },
        AnnClass { id: "request-setup-write", desc: "Setup: Host-to-device" },
        AnnClass { id: "request-bulk-read", desc: "Bulk: Device-to-host" },
        AnnClass { id: "request-bulk-write", desc: "Bulk: Host-to-device" },
        AnnClass { id: "errors", desc: "Unexpected packets" },
    ],
    annotation_rows: &[
        AnnRow { id: "request-setup", desc: "USB SETUP", classes: &[0, 1] },
        AnnRow { id: "request-in", desc: "USB BULK IN", classes: &[2] },
        AnnRow { id: "request-out", desc: "USB BULK OUT", classes: &[3] },
        AnnRow { id: "errors", desc: "Errors", classes: &[4] },
    ],
    binary: &[BinClass { id: "pcap", desc: "PCAP format" }],
    // The package's __init__.py docstring, which the Python host reads.
    doc: "This decoder stacks on top of the 'usb_packet' PD and decodes the USB\n\
(low-speed and full-speed) transactions.\n\
\n\
Transactions and requests are tracked per device address and endpoint.\n\
\n\
Tracking of CONTROL requests is quite accurate, as these always start with\n\
a SETUP token and are completed by an IN or OUT transaction, the status\n\
packet. All transactions during the DATA stage are combined.\n\
\n\
For BULK and INTERRUPT requests, each transaction starts with an IN or OUT\n\
request, and is considered completed after the first transaction containing\n\
data has been ACKed. Normally a request is only completed after a short or\n\
zero length packet, but this would require knowledge about the max packet\n\
size of an endpoint.\n\
\n\
All INTERRUPT requests are treated as BULK requests, as on the link layer\n\
both are identical.\n\
\n\
The PCAP binary output contains 'SUBMIT' and 'COMPLETE' records. For\n\
CONTROL request, the SUBMIT contains the SETUP request, the data is\n\
either contained in the SUBMIT (Host-to-Device) or the COMPLETE\n\
(Device-to-Host) record.\n\
\n\
Details:\n\
https://en.wikipedia.org/wiki/USB\n\
http://www.usb.org/developers/docs/",
};

/// The decoder definition for the registry.
pub static DEF: DecoderDef = DecoderDef { info: &INFO, new: &boxed };

fn boxed() -> Box<dyn Decoder> {
    Box::new(UsbRequest::default())
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum TxState {
    Idle,
    TokenReceived,
    DataReceived,
}

impl TxState {
    fn name(self) -> &'static str {
        match self {
            TxState::Idle => "IDLE",
            TxState::TokenReceived => "TOKEN RECEIVED",
            TxState::DataReceived => "DATA RECEIVED",
        }
    }
}

/// One tracked request, the fields of the Python dictionary.
#[derive(Clone)]
struct Request {
    setup_data: Vec<i64>,
    data: Vec<i64>,
    rtype: Option<String>,
    ss: u64,
    es: Option<u64>,
    ss_data: Option<u64>,
    id: i64,
    addr: i64,
    ep: i64,
    w_length: Option<i64>,
    handshake: Option<String>,
}

/// The usb_request decoder.
pub struct UsbRequest {
    samplerate: u64,
    secs_per_sample: f64,
    requests: HashMap<(i64, i64), Request>,
    request_id: i64,
    transaction_state: TxState,
    ss_transaction: Option<u64>,
    es_transaction: Option<u64>,
    transaction_ep: Option<i64>,
    transaction_addr: Option<i64>,
    transaction_data: Vec<i64>,
    transaction_type: Option<String>,
    handshake: Option<String>,
    in_request_start: String,
    wrote_pcap_header: bool,
}

impl Default for UsbRequest {
    fn default() -> Self {
        UsbRequest {
            samplerate: 0,
            secs_per_sample: 0.0,
            requests: HashMap::new(),
            request_id: 0,
            transaction_state: TxState::Idle,
            ss_transaction: None,
            es_transaction: None,
            transaction_ep: None,
            transaction_addr: None,
            transaction_data: Vec::new(),
            transaction_type: None,
            handshake: None,
            in_request_start: "submit".to_owned(),
            wrote_pcap_header: false,
        }
    }
}

fn as_text(item: &Item) -> Result<&str> {
    match item {
        Item::Str(s) => Ok(s.as_str()),
        _ => Err(Error::device("usb_request: expected a string")),
    }
}

fn as_int(item: &Item) -> Result<i64> {
    match item {
        Item::Int(v) => Ok(*v),
        _ => Err(Error::device("usb_request: expected an integer")),
    }
}

fn as_int_list(item: &Item) -> Result<Vec<i64>> {
    let Item::List(items) = item else {
        return Err(Error::device("usb_request: expected a list of integers"));
    };
    items.iter().map(as_int).collect()
}

/// pd.py `pcap_global_header()`.
fn pcap_global_header() -> Vec<u8> {
    vec![
        0xa1, 0xb2, 0xc3, 0xd4, // Magic, microsecond resolution.
        0x00, 0x02, // Major version 2.
        0x00, 0x04, // Minor version 4.
        0x00, 0x00, 0x00, 0x00, // Correction vs. UTC.
        0x00, 0x00, 0x00, 0x00, // Timestamp accuracy.
        0xff, 0xff, 0xff, 0xff, // Max packet len.
        0x00, 0x00, 0x00, 0xdc, // LINKTYPE_USB_LINUX_MMAPPED.
    ]
}

/// pd.py `pcap_usb_pkt()`: the Linux usbmon record and its data.
fn pcap_record(req: &Request, ts: (u64, u64), is_submit: bool) -> (Vec<u8>, Vec<u8>) {
    let mut h = vec![0u8; 64];
    // The class attribute holds '-' for the setup tag and '<' for the
    // data tag until set_setup()/set_data() clears them.
    h[14] = b'-';
    h[15] = b'<';
    // set_urbid()
    h[4..8].copy_from_slice(&(req.id as u32).to_be_bytes());
    // set_urbtype()
    h[8] = if is_submit { b'S' } else { b'C' };
    // The transfer type starts at bulk (3).
    h[9] = 3;
    // set_timestamp()
    h[20..24].copy_from_slice(&(ts.0 as u32).to_be_bytes());
    h[24..28].copy_from_slice(&(ts.1 as u32).to_be_bytes());
    // set_addr_ep()
    h[11] = req.addr as u8;
    h[10] = req.ep as u8;

    if matches!(req.rtype.as_deref(), Some("SETUP IN" | "SETUP OUT")) {
        // set_transfertype(2), set_setup()
        h[9] = 2;
        h[14] = 0;
        let setup: Vec<u8> = req.setup_data.iter().map(|&b| b as u8).collect();
        // Python assigns to the bytearray slice, which resizes it.
        h.splice(40..48, setup);
    }
    if req.rtype.as_deref() == Some("BULK IN") {
        h[10] = (0x80 | req.ep) as u8;
    }

    // set_data()
    h[15] = 0;
    h[36..40].copy_from_slice(&(req.data.len() as u32).to_be_bytes());

    // record_header()
    let len = (64 + req.data.len()) as u32;
    let mut record = Vec::with_capacity(16);
    record.extend_from_slice(&(ts.0 as u32).to_be_bytes());
    record.extend_from_slice(&(ts.1 as u32).to_be_bytes());
    record.extend_from_slice(&len.to_be_bytes());
    record.extend_from_slice(&len.to_be_bytes());

    let mut packet = h;
    packet.extend(req.data.iter().map(|&b| b as u8));
    (record, packet)
}

impl UsbRequest {
    fn putr(&self, ctx: &mut dyn Context, ss: u64, es: u64, class: usize, text: String) {
        ctx.put_ann(ss, es, class, vec![text]);
    }

    fn putb(&self, ctx: &mut dyn Context, ss: u64, data: Vec<u8>) {
        ctx.put_binary(ss, ss, BIN_PCAP, data);
    }

    /// pd.py `write_pcap_header()`.
    fn write_pcap_header(&mut self, ctx: &mut dyn Context) {
        if !self.wrote_pcap_header {
            self.putb(ctx, 0, pcap_global_header());
            self.wrote_pcap_header = true;
        }
    }

    /// pd.py `ts_from_samplenum()`.
    fn ts_from_samplenum(&self, sample: u64) -> (u64, u64) {
        let ts = sample as f64 * self.secs_per_sample;
        let secs = ts.trunc() as u64;
        let usec = ((ts % 1.0) * 1e6).trunc() as u64;
        (secs, usec)
    }

    /// pd.py `request_summary()`.
    fn request_summary(req: &Request) -> String {
        let mut s = String::from("[");
        if matches!(req.rtype.as_deref(), Some("SETUP IN" | "SETUP OUT")) {
            for b in &req.setup_data {
                s.push_str(&format!(" {b:02X}"));
            }
            s.push_str(" ][");
        }
        for b in &req.data {
            s.push_str(&format!(" {b:02X}"));
        }
        s.push_str(&format!(" ] : {}", req.handshake.as_deref().unwrap_or("")));
        s
    }

    /// pd.py `handle_request()`.
    fn handle_request(
        &mut self,
        ctx: &mut dyn Context,
        request_start: bool,
        request_end: bool,
    ) -> Result<()> {
        if !request_start && !request_end {
            return Ok(());
        }
        self.write_pcap_header(ctx);
        let ep = self
            .transaction_ep
            .ok_or_else(|| Error::device("usb_request: no transaction endpoint"))?;
        let addr = self
            .transaction_addr
            .ok_or_else(|| Error::device("usb_request: no transaction address"))?;
        let key = (addr, ep);
        let request = self
            .requests
            .get(&key)
            .cloned()
            .ok_or_else(|| Error::device("usb_request: no request"))?;

        let mut ss_data = request.ss_data;
        if self.in_request_start == "submit" {
            ss_data = Some(request.ss);
        }

        if request_start {
            let ts = self.ts_from_samplenum(request.ss);
            let (record, packet) = pcap_record(&request, ts, true);
            self.putb(ctx, request.ss, record);
            self.putb(ctx, request.ss, packet);
        }

        if request_end {
            let es = request.es.ok_or_else(|| Error::device("usb_request: request has no end"))?;
            let summary = Self::request_summary(&request);
            match request.rtype.as_deref() {
                Some("SETUP IN") => {
                    self.putr(ctx, request.ss, es, ANN_SETUP_READ, format!("SETUP in: {summary}"));
                }
                Some("SETUP OUT") => {
                    self.putr(
                        ctx,
                        request.ss,
                        es,
                        ANN_SETUP_WRITE,
                        format!("SETUP out: {summary}"),
                    );
                }
                Some("BULK IN") => {
                    let ss_data = ss_data.ok_or_else(|| {
                        Error::device("usb_request: BULK IN without a data start")
                    })?;
                    self.putr(ctx, ss_data, es, ANN_BULK_READ, format!("BULK in: {summary}"));
                }
                Some("BULK OUT") => {
                    self.putr(ctx, request.ss, es, ANN_BULK_WRITE, format!("BULK out: {summary}"));
                }
                _ => {}
            }

            let ts = self.ts_from_samplenum(es);
            let (record, packet) = pcap_record(&request, ts, false);
            self.putb(ctx, request.ss, record);
            self.putb(ctx, request.ss, packet);
            self.requests.remove(&key);
        }
        Ok(())
    }

    /// pd.py `handle_transfer()`.
    fn handle_transfer(&mut self, ctx: &mut dyn Context) -> Result<()> {
        let request_end = matches!(self.handshake.as_deref(), Some("ACK" | "STALL" | "timeout"));
        let ep = self
            .transaction_ep
            .ok_or_else(|| Error::device("usb_request: no transaction endpoint"))?;
        let addr = self
            .transaction_addr
            .ok_or_else(|| Error::device("usb_request: no transaction address"))?;
        let transaction_type = self.transaction_type.clone().unwrap_or_default();

        // Handle protocol STALLs: the condition lasts until the next
        // SETUP transfer (8.5.3.4).
        if transaction_type == "SETUP" && self.requests.contains_key(&(addr, ep)) {
            let rtype = self.requests[&(addr, ep)].rtype.clone();
            if matches!(rtype.as_deref(), Some("SETUP IN" | "SETUP OUT")) {
                if let Some(r) = self.requests.get_mut(&(addr, ep)) {
                    r.es = self.ss_transaction;
                }
                self.handle_request(ctx, false, true)?;
            }
        }

        let request_started = if let std::collections::hash_map::Entry::Vacant(entry) =
            self.requests.entry((addr, ep))
        {
            entry.insert(Request {
                setup_data: Vec::new(),
                data: Vec::new(),
                rtype: None,
                ss: self.ss_transaction.unwrap_or(0),
                es: None,
                ss_data: None,
                id: self.request_id,
                addr,
                ep,
                w_length: None,
                handshake: None,
            });
            self.request_id += 1;
            true
        } else {
            false
        };

        if request_end && let Some(r) = self.requests.get_mut(&(addr, ep)) {
            r.es = self.es_transaction;
            r.handshake = self.handshake.clone();
        }

        let rtype = self.requests[&(addr, ep)].rtype.clone();

        if (rtype.is_none() || rtype.as_deref() == Some("BULK IN")) && transaction_type == "IN" {
            let transaction_data = self.transaction_data.clone();
            let ss_transaction = self.ss_transaction;
            if let Some(r) = self.requests.get_mut(&(addr, ep)) {
                r.rtype = Some("BULK IN".to_owned());
                if r.data.is_empty() && !transaction_data.is_empty() {
                    r.ss_data = ss_transaction;
                }
                r.data.extend(transaction_data);
            }
            self.handle_request(ctx, request_started, request_end)?;
        } else if (rtype.is_none() || rtype.as_deref() == Some("BULK OUT"))
            && transaction_type == "OUT"
        {
            let transaction_data = self.transaction_data.clone();
            let ack = self.handshake.as_deref() == Some("ACK");
            if let Some(r) = self.requests.get_mut(&(addr, ep)) {
                r.rtype = Some("BULK OUT".to_owned());
                if ack {
                    r.data.extend(transaction_data);
                }
            }
            self.handle_request(ctx, request_started, request_end)?;
        } else if rtype.is_none() && transaction_type == "SETUP" {
            let transaction_data = self.transaction_data.clone();
            if transaction_data.len() < 8 {
                return Err(Error::device("usb_request: short SETUP data"));
            }
            let w_length =
                i64::from((transaction_data[6] as u16) | ((transaction_data[7] as u16) << 8));
            if let Some(r) = self.requests.get_mut(&(addr, ep)) {
                r.setup_data = transaction_data.clone();
                r.w_length = Some(w_length);
            }
            if transaction_data[0] & 0x80 != 0 {
                if let Some(r) = self.requests.get_mut(&(addr, ep)) {
                    r.rtype = Some("SETUP IN".to_owned());
                }
                self.handle_request(ctx, true, false)?;
            } else {
                if let Some(r) = self.requests.get_mut(&(addr, ep)) {
                    r.rtype = Some("SETUP OUT".to_owned());
                }
                self.handle_request(ctx, w_length == 0, false)?;
            }
        } else if rtype.as_deref() == Some("SETUP IN") && transaction_type == "IN" {
            let transaction_data = self.transaction_data.clone();
            if let Some(r) = self.requests.get_mut(&(addr, ep)) {
                r.data.extend(transaction_data);
            }
        } else if rtype.as_deref() == Some("SETUP OUT") && transaction_type == "OUT" {
            let transaction_data = self.transaction_data.clone();
            let ack = self.handshake.as_deref() == Some("ACK");
            let mut done = false;
            if let Some(r) = self.requests.get_mut(&(addr, ep)) {
                if ack {
                    r.data.extend(transaction_data);
                }
                if r.w_length == Some(r.data.len() as i64) {
                    done = true;
                }
            }
            if done {
                self.handle_request(ctx, true, false)?;
            }
        } else if rtype.as_deref() == Some("SETUP IN") && transaction_type == "OUT" {
            self.handle_request(ctx, false, request_end)?;
        } else if rtype.as_deref() == Some("SETUP OUT") && transaction_type == "IN" {
            self.handle_request(ctx, false, request_end)?;
        } else {
            return Ok(());
        }
        Ok(())
    }

    fn error(&self, ctx: &mut dyn Context, ss: u64, es: u64, text: String) {
        ctx.put_ann(ss, es, ANN_ERRORS, vec![text]);
    }

    fn handle_packet(
        &mut self,
        ctx: &mut dyn Context,
        ss: u64,
        es: u64,
        pdata: &Item,
    ) -> Result<()> {
        let Item::List(parts) = pdata else {
            return Err(Error::device("usb_request: PACKET payload is not a list"));
        };
        if parts.len() != 3 {
            return Err(Error::device("usb_request: PACKET payload is not three elements"));
        }
        let pcategory = as_text(&parts[0])?;
        let pname = as_text(&parts[1])?;
        let pinfo = &parts[2];

        if pcategory == "TOKEN" {
            if pname == "SOF" {
                return Ok(());
            }
            if self.transaction_state == TxState::TokenReceived {
                let mut transaction_timeout = self
                    .es_transaction
                    .ok_or_else(|| Error::device("usb_request: no transaction end"))?;
                let ss_transaction = self
                    .ss_transaction
                    .ok_or_else(|| Error::device("usb_request: no transaction start"))?;
                // Token length is 35 bits, the timeout 16..18 bit times.
                transaction_timeout += (transaction_timeout - ss_transaction) / 2;
                if ss > transaction_timeout {
                    self.es_transaction = Some(transaction_timeout);
                    self.handshake = Some("timeout".to_owned());
                    self.handle_transfer(ctx)?;
                    self.transaction_state = TxState::Idle;
                }
            }

            if self.transaction_state != TxState::Idle {
                self.error(
                    ctx,
                    ss,
                    es,
                    format!(
                        "ERR: received {pname} token in state {}",
                        self.transaction_state.name()
                    ),
                );
                return Ok(());
            }

            let Item::List(fields) = pinfo else {
                return Err(Error::device("usb_request: TOKEN fields are not a list"));
            };
            if fields.len() != 5 {
                return Err(Error::device("usb_request: TOKEN is not five fields"));
            }
            let addr = as_int(&fields[2])?;
            let ep = as_int(&fields[3])?;
            self.transaction_data.clear();
            self.ss_transaction = Some(ss);
            self.es_transaction = Some(es);
            self.transaction_state = TxState::TokenReceived;
            self.transaction_ep = Some(if ep > 0 && pname == "IN" { ep + 0x80 } else { ep });
            self.transaction_addr = Some(addr);
            self.transaction_type = Some(pname.to_owned());
        } else if pcategory == "DATA" {
            if self.transaction_state != TxState::TokenReceived {
                self.error(
                    ctx,
                    ss,
                    es,
                    format!(
                        "ERR: received {pname} token in state {}",
                        self.transaction_state.name()
                    ),
                );
                return Ok(());
            }
            let Item::List(fields) = pinfo else {
                return Err(Error::device("usb_request: DATA fields are not a list"));
            };
            if fields.len() < 3 {
                return Err(Error::device("usb_request: DATA has no data bytes"));
            }
            self.transaction_data = as_int_list(&fields[2])?;
            self.transaction_state = TxState::DataReceived;
        } else if pcategory == "HANDSHAKE" {
            if !matches!(self.transaction_state, TxState::TokenReceived | TxState::DataReceived) {
                self.error(
                    ctx,
                    ss,
                    es,
                    format!(
                        "ERR: received {pname} token in state {}",
                        self.transaction_state.name()
                    ),
                );
                return Ok(());
            }
            self.handshake = Some(pname.to_owned());
            self.transaction_state = TxState::Idle;
            self.es_transaction = Some(es);
            self.handle_transfer(ctx)?;
        } else if pname == "PRE" {
            return Ok(());
        } else {
            self.error(
                ctx,
                ss,
                es,
                format!(
                    "ERR: received unhandled {pname} token in state {}",
                    self.transaction_state.name()
                ),
            );
            return Ok(());
        }
        Ok(())
    }
}

impl Decoder for UsbRequest {
    fn info(&self) -> &'static DecoderInfo {
        &INFO
    }

    fn start(&mut self, config: &Config) -> Result<()> {
        self.in_request_start = config.str("in_request_start", "submit").to_owned();
        Ok(())
    }

    fn metadata(&mut self, samplerate: u64) {
        self.samplerate = samplerate;
        if samplerate != 0 {
            self.secs_per_sample = 1.0 / samplerate as f64;
        }
    }

    fn decode(&mut self, _ctx: &mut dyn Context) -> Result<()> {
        Err(Error::not_supported("usb_request takes a usb_packet decoder as input"))
    }

    fn decode_item(&mut self, ss: u64, es: u64, item: &Item, ctx: &mut dyn Context) -> Result<()> {
        if self.samplerate == 0 {
            return Err(Error::device("Cannot decode without samplerate."));
        }
        let Item::List(parts) = item else {
            return Err(Error::device("usb_request: expected a two element packet"));
        };
        if parts.len() != 2 {
            return Err(Error::device("usb_request: expected a two element packet"));
        }
        let ptype = as_text(&parts[0])?;
        if ptype != "PACKET" {
            return Ok(());
        }
        self.handle_packet(ctx, ss, es, &parts[1])
    }
}
