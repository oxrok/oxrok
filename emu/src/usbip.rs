//! The USB/IP server: protocol version 1.1.1 as the Linux `vhci-hcd`
//! client speaks it. One TCP connection carries one device:
//! `OP_REQ_DEVLIST` lists it, `OP_REQ_IMPORT` claims it, then the
//! client submits URBs with `CMD_SUBMIT`, which the server answers
//! with `RET_SUBMIT`, and cancels them with `CMD_UNLINK`, answered
//! with `RET_UNLINK`. All header fields are big endian; a setup packet
//! is carried as its eight bytes.
//!
//! A transfer the model answers with NAK stays pending here, as a URB
//! does in a host controller, and is offered to the model again after
//! every other transfer and every 20 ms, until it completes or the
//! client unlinks it. Transfers on one endpoint complete in the order
//! they were submitted: one submitted behind a held one waits behind
//! it, as a host controller's queue for the endpoint would.
//!
//! When the model asks to renumerate
//! ([`crate::model::DeviceModel::take_renumeration`]), the server tells
//! whoever drives the run through the channel given to
//! [`Server::on_renumeration`]; that side detaches the device and
//! imports it again, and the new import resets the device, which takes
//! over the model's new descriptors.

use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use crate::device::Device;
use crate::model::{Descriptors, Event, Reply, Setup, Violations};
use crate::txlog::{TxLog, control_line, transfer_line_undefined};
use crate::{Error, Result};

/// The protocol version.
pub const VERSION: u16 = 0x0111;

const OP_REQ_DEVLIST: u16 = 0x8005;
const OP_REP_DEVLIST: u16 = 0x0005;
const OP_REQ_IMPORT: u16 = 0x8003;
const OP_REP_IMPORT: u16 = 0x0003;

const CMD_SUBMIT: u32 = 1;
const CMD_UNLINK: u32 = 2;
const RET_SUBMIT: u32 = 3;
const RET_UNLINK: u32 = 4;

/// Direction of a URB.
pub const DIR_OUT: u32 = 0;
/// Device to host.
pub const DIR_IN: u32 = 1;

const EPIPE: i32 = -32;
/// A device sent more than the URB holds: babble, as the Linux HCDs
/// report it.
const EOVERFLOW: i32 = -75;
const EINVAL: i32 = -22;
const ECONNRESET: i32 = -104;

const HEADER: usize = 48;
const DEVICE_RECORD: usize = 312;

/// The poll period for held transfers.
const TICK: Duration = Duration::from_millis(20);

fn be32(b: &[u8]) -> u32 {
    u32::from_be_bytes([b[0], b[1], b[2], b[3]])
}

fn cstr(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).into_owned()
}

fn fixed(s: &str, len: usize) -> Vec<u8> {
    let mut b = s.as_bytes()[..s.len().min(len - 1)].to_vec();
    b.resize(len, 0);
    b
}

/// The `usbip_usb_device` record of a device: 312 bytes.
pub fn device_record(
    desc: &Descriptors,
    path: &str,
    busid: &str,
    busnum: u32,
    devnum: u32,
) -> Vec<u8> {
    let config = desc.configs.first().map(Vec::as_slice).unwrap_or_default();
    let class = desc.device_class();
    let mut b = fixed(path, 256);
    b.extend(fixed(busid, 32));
    b.extend_from_slice(&busnum.to_be_bytes());
    b.extend_from_slice(&devnum.to_be_bytes());
    b.extend_from_slice(&(desc.speed as u32).to_be_bytes());
    b.extend_from_slice(&desc.vendor_id().to_be_bytes());
    b.extend_from_slice(&desc.product_id().to_be_bytes());
    b.extend_from_slice(&desc.bcd_device().to_be_bytes());
    b.extend_from_slice(&class);
    b.push(config.get(5).copied().unwrap_or(1));
    b.push(desc.configs.len() as u8);
    b.push(config.get(4).copied().unwrap_or(0));
    debug_assert_eq!(b.len(), DEVICE_RECORD);
    b
}

fn op_header(code: u16, status: u32) -> Vec<u8> {
    let mut b = VERSION.to_be_bytes().to_vec();
    b.extend_from_slice(&code.to_be_bytes());
    b.extend_from_slice(&status.to_be_bytes());
    b
}

/// A submitted URB, held or in flight.
#[derive(Clone, Debug)]
struct Urb {
    seqnum: u32,
    devid: u32,
    direction: u32,
    ep: u8,
    length: usize,
    packets: i32,
    setup: [u8; 8],
    data: Vec<u8>,
}

enum Pdu {
    Submit(Urb),
    Unlink { seqnum: u32, target: u32 },
}

/// Reads PDUs off the socket until it closes or a PDU is malformed.
fn read_pdus(mut stream: TcpStream, tx: &mpsc::Sender<Pdu>) -> io::Result<()> {
    loop {
        let mut h = [0u8; HEADER];
        stream.read_exact(&mut h)?;
        let command = be32(&h[0..4]);
        let seqnum = be32(&h[4..8]);
        let pdu = match command {
            CMD_SUBMIT => {
                let direction = be32(&h[12..16]);
                let length = be32(&h[24..28]) as i32;
                let packets = be32(&h[32..36]) as i32;
                let length = usize::try_from(length).unwrap_or(0);
                let mut data = Vec::new();
                if direction == DIR_OUT && length > 0 {
                    data.resize(length, 0);
                    stream.read_exact(&mut data)?;
                }
                if packets > 0 {
                    // Isochronous descriptors follow; read and drop them.
                    let mut iso = vec![0u8; 16 * packets as usize];
                    stream.read_exact(&mut iso)?;
                }
                let mut setup = [0u8; 8];
                setup.copy_from_slice(&h[40..48]);
                Pdu::Submit(Urb {
                    seqnum,
                    devid: be32(&h[8..12]),
                    direction,
                    ep: be32(&h[16..20]) as u8,
                    length,
                    packets,
                    setup,
                    data,
                })
            }
            CMD_UNLINK => Pdu::Unlink { seqnum, target: be32(&h[20..24]) },
            other => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("unknown USB/IP command {other}"),
                ));
            }
        };
        if tx.send(pdu).is_err() {
            return Ok(());
        }
    }
}

fn write_ret_submit(
    out: &mut TcpStream,
    seqnum: u32,
    status: i32,
    actual: usize,
    data: &[u8],
) -> io::Result<()> {
    let mut b = Vec::with_capacity(HEADER + data.len());
    b.extend_from_slice(&RET_SUBMIT.to_be_bytes());
    b.extend_from_slice(&seqnum.to_be_bytes());
    b.extend_from_slice(&[0u8; 12]);
    b.extend_from_slice(&status.to_be_bytes());
    b.extend_from_slice(&(actual as i32).to_be_bytes());
    b.extend_from_slice(&[0u8; 20]);
    b.extend_from_slice(data);
    out.write_all(&b)
}

fn write_ret_unlink(out: &mut TcpStream, seqnum: u32, status: i32) -> io::Result<()> {
    let mut b = Vec::with_capacity(HEADER);
    b.extend_from_slice(&RET_UNLINK.to_be_bytes());
    b.extend_from_slice(&seqnum.to_be_bytes());
    b.extend_from_slice(&[0u8; 12]);
    b.extend_from_slice(&status.to_be_bytes());
    b.extend_from_slice(&[0u8; 24]);
    out.write_all(&b)
}

/// The server: one device, one connection at a time. The device is
/// shared with whoever drives the run, so its model can be replaced
/// between runs (see [`Device::replace_model`]).
pub struct Server {
    device: Arc<Mutex<Device>>,
    log: Arc<Mutex<TxLog>>,
    busid: String,
    busnum: u32,
    devnum: u32,
    renumeration: Option<mpsc::Sender<String>>,
    events: Option<mpsc::Sender<Event>>,
}

impl Server {
    /// A server for `device`, exported under `busid`; `busnum` and
    /// `devnum` are what the client is told, and form the `devid` of
    /// every URB (`busnum << 16 | devnum`).
    pub fn new(
        device: Arc<Mutex<Device>>,
        log: Arc<Mutex<TxLog>>,
        busid: &str,
        busnum: u32,
        devnum: u32,
    ) -> Server {
        Server {
            device,
            log,
            busid: busid.to_owned(),
            busnum,
            devnum,
            renumeration: None,
            events: None,
        }
    }

    /// Where a renumeration request of the model is sent: one message
    /// per request, after the transfer that caused it was answered,
    /// naming the log section to go on in once the device is back.
    #[must_use]
    pub fn on_renumeration(mut self, tx: mpsc::Sender<String>) -> Server {
        self.renumeration = Some(tx);
        self
    }

    /// Where an armed event that fired is sent
    /// ([`crate::device::Device::arm`]), after its log line. Once an
    /// unplug fired, the server answers nothing more on the connection:
    /// the transfers the host submits until the device is detached are
    /// dropped (comments in a verbose log), as a device that is gone
    /// answers nothing.
    #[must_use]
    pub fn on_event(mut self, tx: mpsc::Sender<Event>) -> Server {
        self.events = Some(tx);
        self
    }

    fn device(&self) -> std::sync::MutexGuard<'_, Device> {
        self.device.lock().expect("device lock")
    }

    /// The `devid` the client must use.
    pub fn devid(&self) -> u32 {
        (self.busnum << 16) | self.devnum
    }

    /// The device's `usbip_usb_device` record.
    pub fn record(&self) -> Vec<u8> {
        let device = self.device();
        let path = format!("/oxrok-emu/{}", device.name());
        device_record(device.descriptors(), &path, &self.busid, self.busnum, self.devnum)
    }

    /// Accepts connections until `stop` is set.
    pub fn run(mut self, listener: &TcpListener, stop: &AtomicBool) -> Result<()> {
        listener.set_nonblocking(true)?;
        while !stop.load(Ordering::Relaxed) {
            match listener.accept() {
                Ok((stream, peer)) => {
                    stream.set_nonblocking(false)?;
                    stream.set_nodelay(true)?;
                    self.event(&format!("connection from {peer}"));
                    match self.connection(stream, stop) {
                        Ok(()) => self.event("connection closed"),
                        Err(e) => self.event(&format!("connection ended: {e}")),
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => thread::sleep(TICK),
                Err(e) => return Err(e.into()),
            }
        }
        Ok(())
    }

    fn event(&self, message: &str) {
        self.log.lock().expect("log lock").event(message);
    }

    fn connection(&mut self, mut stream: TcpStream, stop: &AtomicBool) -> io::Result<()> {
        let mut head = [0u8; 8];
        stream.read_exact(&mut head)?;
        let code = u16::from_be_bytes([head[2], head[3]]);
        match code {
            OP_REQ_DEVLIST => {
                let mut b = op_header(OP_REP_DEVLIST, 0);
                b.extend_from_slice(&1u32.to_be_bytes());
                b.extend(self.record());
                let device = self.device();
                if let Some(config) = device.descriptors().configs.first() {
                    for i in Descriptors::interfaces(config) {
                        if i[1] == 0 {
                            b.extend_from_slice(&[i[2], i[3], i[4], 0]);
                        }
                    }
                }
                stream.write_all(&b)
            }
            OP_REQ_IMPORT => {
                let mut busid = [0u8; 32];
                stream.read_exact(&mut busid)?;
                let want = cstr(&busid);
                if want != self.busid {
                    self.event(&format!("import of {want} refused, the device is {}", self.busid));
                    return stream.write_all(&op_header(OP_REP_IMPORT, 1));
                }
                let mut b = op_header(OP_REP_IMPORT, 0);
                b.extend(self.record());
                stream.write_all(&b)?;
                self.event(&format!("{want} imported"));
                self.device().reset();
                self.urbs(stream, stop)
            }
            other => {
                self.event(&format!("unknown operation {other:#06x}"));
                stream.write_all(&op_header(other & 0x7fff, 1))
            }
        }
    }

    fn urbs(&mut self, stream: TcpStream, stop: &AtomicBool) -> io::Result<()> {
        let reader = stream.try_clone()?;
        let (tx, rx) = mpsc::channel();
        let log = Arc::clone(&self.log);
        thread::spawn(move || {
            if let Err(e) = read_pdus(reader, &tx) {
                log.lock().expect("log lock").event(&format!("reader stopped: {e}"));
            }
        });
        let mut out = stream;
        let mut held: Vec<Urb> = Vec::new();
        let mut gone = false;
        loop {
            if stop.load(Ordering::Relaxed) {
                return Ok(());
            }
            match rx.recv_timeout(TICK) {
                Ok(Pdu::Submit(urb)) if gone => {
                    self.event(&format!(
                        "seq {}: ep {:#04x} {} dropped, the device is unplugged",
                        urb.seqnum,
                        urb.ep,
                        dir_word(urb.direction)
                    ));
                }
                Ok(Pdu::Submit(urb)) => {
                    if urb.ep != 0 && urb.direction == DIR_IN {
                        // The URB's own length, as the host submitted it
                        // (not rounded to the packet size as for the
                        // model's in_transfer(): see in_length()).
                        self.device().in_submitted(urb.ep, urb.length);
                    }
                    let queued = urb.ep != 0
                        && held.iter().any(|u| u.ep == urb.ep && u.direction == urb.direction);
                    if queued {
                        // Behind a held transfer of the same endpoint.
                        self.event(&format!(
                            "seq {}: ep {:#04x} {} queued behind a held transfer",
                            urb.seqnum,
                            urb.ep,
                            dir_word(urb.direction)
                        ));
                        held.push(urb);
                    } else {
                        let (reply, v) = self.perform(&urb);
                        let event = self.fired(&urb, &reply);
                        self.finish(&mut out, urb, reply, v, &mut held, true)?;
                        self.check_renumeration();
                        gone |= self.event_fired(event, &mut held);
                    }
                }
                Ok(Pdu::Unlink { seqnum, target }) => {
                    let found = held.iter().position(|u| u.seqnum == target);
                    let mut status = 0;
                    if let Some(i) = found {
                        let u = held.remove(i);
                        self.event(&format!(
                            "unlink seq {target}: ep {:#04x} {} held",
                            u.ep,
                            dir_word(u.direction)
                        ));
                        status = ECONNRESET;
                        // Packets it holds already: its RET_SUBMIT with
                        // them goes first, so the unlink finds it completed.
                        if self.complete_unlinked(&mut out, &u)? {
                            status = 0;
                        }
                    } else {
                        self.event(&format!("unlink seq {target}: already completed"));
                    }
                    write_ret_unlink(&mut out, seqnum, status)?;
                }
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return Ok(()),
            }
            let pending = std::mem::take(&mut held);
            // Endpoints whose oldest transfer is still held this round:
            // the transfers behind it wait.
            let mut blocked: Vec<(u8, u32)> = Vec::new();
            for urb in pending {
                let key = (urb.ep, urb.direction);
                if gone {
                    break;
                }
                if urb.ep != 0 && blocked.contains(&key) {
                    held.push(urb);
                    continue;
                }
                let (reply, v) = self.perform(&urb);
                if reply == Reply::Nak {
                    blocked.push(key);
                }
                let event = self.fired(&urb, &reply);
                self.finish(&mut out, urb, reply, v, &mut held, false)?;
                self.check_renumeration();
                gone |= self.event_fired(event, &mut held);
            }
        }
    }

    /// Whether the transfer just offered fires the armed event.
    fn fired(&self, urb: &Urb, reply: &Reply) -> Option<Event> {
        self.device().fired(urb.ep, urb.direction == DIR_IN, reply)
    }

    /// Logs an event that fired (`unplug`, `interrupt`, a line the
    /// comparisons keep, in the running section right after the
    /// transfer that fired it) and passes it on. Returns whether the
    /// device is now gone; the held transfers are dropped then.
    fn event_fired(&mut self, event: Option<Event>, held: &mut Vec<Urb>) -> bool {
        let Some(event) = event else {
            return false;
        };
        self.log.lock().expect("log lock").transfer(event.word());
        if let Some(tx) = &self.events {
            let _ = tx.send(event);
        } else {
            self.event("an armed event fired; nobody to pass it to");
        }
        if event == Event::Unplug {
            for u in held.drain(..) {
                self.event(&format!("seq {}: dropped, the device is unplugged", u.seqnum));
            }
            return true;
        }
        false
    }

    /// Passes a renumeration request of the model on. The log gets
    /// `disconnect` and `connect VID:PID` at once, in the section of
    /// the run that caused it (a tool that ends right after the
    /// transfer would otherwise see them or not depending on timing);
    /// what the host sends until the detach follows in that section.
    /// The channel gets the section to go back to after the kernel's
    /// enumeration of the new device.
    fn check_renumeration(&mut self) {
        if !self.device().take_renumeration() {
            return;
        }
        let Some(tx) = &self.renumeration else {
            self.event("the model asks to renumerate; nobody to do it, the device stays as it is");
            return;
        };
        let (vendor, product) = {
            let d = self.device();
            let m = d.model_descriptors();
            (m.vendor_id(), m.product_id())
        };
        let resume = {
            let mut log = self.log.lock().expect("log lock");
            log.event("the model asks to renumerate");
            log.transfer("disconnect");
            log.transfer(&format!("connect {vendor:04x}:{product:04x}"));
            log.current_section().to_owned()
        };
        let _ = tx.send(resume);
    }

    /// The length of an IN URB as the device sees it. A bulk IN asks
    /// for whole packets: the host controller takes up to
    /// `wMaxPacketSize` bytes per IN token whatever the URB's length, so
    /// a device cannot tell a URB of 1 byte from one of 64 on a 64 byte
    /// endpoint. libusb passes any length (libsigrok's Logic16 driver
    /// reads 1 byte answers), nusb only multiples of the packet size;
    /// the length is rounded up to the packet size for the model and the
    /// log, so the two read the same. Interrupt and control INs keep
    /// their length. The URB's own length still decides the outcome on
    /// the host: an answer longer than it is a babble (`finish()`).
    fn in_length(&self, urb: &Urb) -> usize {
        let address = 0x80 | urb.ep;
        match self.device().endpoint(address) {
            Some(e) if e.transfer_type() == crate::model::BULK && e.max_packet > 0 => {
                let mps = usize::from(e.max_packet);
                urb.length.div_ceil(mps).max(1) * mps
            }
            _ => urb.length,
        }
    }

    /// Offers a URB to the device.
    fn perform(&mut self, urb: &Urb) -> (Reply, Violations) {
        let mut v = Violations::default();
        if urb.devid != self.devid() {
            v.report(format!(
                "URB for devid {:#010x}, the device is {:#010x}",
                urb.devid,
                self.devid()
            ));
        }
        if urb.packets > 0 {
            v.report(format!("isochronous URB with {} packets; not supported", urb.packets));
            return (Reply::Stall, v);
        }
        let length = if urb.direction == DIR_IN { self.in_length(urb) } else { urb.length };
        let mut device = self.device();
        let reply = if urb.ep == 0 {
            device.control(Setup::from_bytes(&urb.setup), &urb.data, &mut v)
        } else if urb.direction == DIR_IN {
            device.transfer_in(urb.ep, length, &mut v)
        } else {
            device.transfer_out(urb.ep, &urb.data, &mut v)
        };
        (reply, v)
    }

    /// A held IN transfer just unlinked: when the model says it holds
    /// data ([`crate::model::DeviceModel::in_unlinked`]), logs it and
    /// completes the URB with it. Whether it did.
    fn complete_unlinked(&mut self, out: &mut TcpStream, urb: &Urb) -> io::Result<bool> {
        if urb.direction != DIR_IN || urb.ep == 0 {
            return Ok(false);
        }
        let length = self.in_length(urb);
        let Some(d) = self.device().in_unlinked(urb.ep, length) else { return Ok(false) };
        let address = 0x80 | urb.ep;
        let kind =
            self.device().endpoint(address).map_or(0xff, crate::model::Endpoint::transfer_type);
        let above = self.device().log_digest_above();
        let d = &d[..d.len().min(urb.length)];
        let line = transfer_line_undefined(
            kind,
            address,
            length,
            &urb.data,
            &[],
            &Reply::Data(d.to_vec()),
            above,
        );
        self.log.lock().expect("log lock").transfer(&line);
        write_ret_submit(out, urb.seqnum, ECONNRESET, d.len(), d)?;
        Ok(true)
    }

    /// Logs the outcome and answers the client, or holds the URB;
    /// `fresh` says the URB was just submitted rather than retried.
    fn finish(
        &mut self,
        out: &mut TcpStream,
        urb: Urb,
        reply: Reply,
        mut v: Violations,
        held: &mut Vec<Urb>,
        fresh: bool,
    ) -> io::Result<()> {
        let mut log = self.log.lock().expect("log lock");
        let line = if urb.ep == 0 {
            let above = self.device().log_digest_above();
            control_line(Setup::from_bytes(&urb.setup), &urb.data, &reply, above)
        } else {
            let address = if urb.direction == DIR_IN { 0x80 | urb.ep } else { urb.ep };
            let kind =
                self.device().endpoint(address).map_or(0xff, crate::model::Endpoint::transfer_type);
            let above = self.device().log_digest_above();
            let undefined = if urb.direction == DIR_IN {
                Vec::new()
            } else {
                self.device().log_undefined(urb.ep, &urb.data)
            };
            let length = if urb.direction == DIR_IN { self.in_length(&urb) } else { urb.length };
            transfer_line_undefined(kind, address, length, &urb.data, &undefined, &reply, above)
        };
        for message in v.drain() {
            log.violation(&message);
        }
        match reply {
            Reply::Nak => {
                if fresh {
                    log.event(&format!("seq {}: {line} (held)", urb.seqnum));
                }
                held.push(urb);
                Ok(())
            }
            Reply::Stall => {
                log.transfer(&line);
                write_ret_submit(out, urb.seqnum, EPIPE, 0, &[])
            }
            Reply::Data(d) => {
                if urb.direction == DIR_IN && urb.ep != 0 && d.len() > urb.length {
                    // More than the URB holds: the HCD ends it with
                    // -EOVERFLOW and the bytes that fit, as for a
                    // babbling device.
                    log.transfer(&format!("{line} (babble: the URB holds {})", urb.length));
                    let d = &d[..urb.length];
                    write_ret_submit(out, urb.seqnum, EOVERFLOW, d.len(), d)
                } else if urb.direction == DIR_IN {
                    log.transfer(&line);
                    // A control IN longer than wLength is cut, as the
                    // model trait says.
                    let d = &d[..d.len().min(urb.length)];
                    write_ret_submit(out, urb.seqnum, 0, d.len(), d)
                } else {
                    log.transfer(&line);
                    let taken =
                        if d.is_empty() { urb.data.len() } else { d.len().min(urb.data.len()) };
                    write_ret_submit(out, urb.seqnum, 0, taken, &[])
                }
            }
        }
    }
}

fn dir_word(direction: u32) -> &'static str {
    if direction == DIR_IN { "IN" } else { "OUT" }
}

/// Invalid-argument status, for callers that build their own replies.
pub const STATUS_EINVAL: i32 = EINVAL;

/// A client's import of `busid` from the server at `addr`: the socket
/// is ready to be handed to `vhci-hcd`, and the device record comes
/// with it.
pub fn import(addr: SocketAddr, busid: &str) -> Result<(TcpStream, Vec<u8>)> {
    let mut stream = TcpStream::connect(addr)?;
    stream.set_nodelay(true)?;
    let mut b = op_header(OP_REQ_IMPORT, 0);
    b.extend(fixed(busid, 32));
    stream.write_all(&b)?;
    let mut head = [0u8; 8];
    stream.read_exact(&mut head)?;
    let code = u16::from_be_bytes([head[2], head[3]]);
    let status = be32(&head[4..8]);
    if code != OP_REP_IMPORT || status != 0 {
        return Err(Error(format!("import of {busid} refused: code {code:#06x}, status {status}")));
    }
    let mut record = vec![0u8; DEVICE_RECORD];
    stream.read_exact(&mut record)?;
    Ok((stream, record))
}

/// A client's `CMD_SUBMIT`, for tests and tools: returns the frame.
pub fn cmd_submit(
    seqnum: u32,
    devid: u32,
    direction: u32,
    ep: u8,
    length: usize,
    setup: Option<Setup>,
    data: &[u8],
) -> Vec<u8> {
    let mut b = Vec::with_capacity(HEADER + data.len());
    b.extend_from_slice(&CMD_SUBMIT.to_be_bytes());
    b.extend_from_slice(&seqnum.to_be_bytes());
    b.extend_from_slice(&devid.to_be_bytes());
    b.extend_from_slice(&direction.to_be_bytes());
    b.extend_from_slice(&u32::from(ep).to_be_bytes());
    b.extend_from_slice(&0u32.to_be_bytes());
    b.extend_from_slice(&(length as i32).to_be_bytes());
    b.extend_from_slice(&0u32.to_be_bytes());
    b.extend_from_slice(&0u32.to_be_bytes());
    b.extend_from_slice(&0u32.to_be_bytes());
    b.extend_from_slice(&setup.map_or([0u8; 8], Setup::to_bytes));
    if direction == DIR_OUT {
        b.extend_from_slice(data);
    }
    b
}

/// A client's `CMD_UNLINK` of `target`.
pub fn cmd_unlink(seqnum: u32, target: u32) -> Vec<u8> {
    let mut b = Vec::with_capacity(HEADER);
    b.extend_from_slice(&CMD_UNLINK.to_be_bytes());
    b.extend_from_slice(&seqnum.to_be_bytes());
    b.extend_from_slice(&[0u8; 12]);
    b.extend_from_slice(&target.to_be_bytes());
    b.extend_from_slice(&[0u8; 24]);
    b
}

/// A reply read by a client: command, seqnum, status, data.
#[derive(Debug, PartialEq, Eq)]
pub struct Ret {
    /// `RET_SUBMIT` (3) or `RET_UNLINK` (4).
    pub command: u32,
    /// The sequence number answered.
    pub seqnum: u32,
    /// The status, 0 or a negative errno.
    pub status: i32,
    /// The data of an IN transfer.
    pub data: Vec<u8>,
}

/// Reads one reply, for tests and tools. `data_follows` says whether
/// the URB answered was an IN transfer, whose data trails a
/// `RET_SUBMIT`; a client knows its own URBs.
pub fn read_ret(stream: &mut TcpStream, data_follows: bool) -> io::Result<Ret> {
    let mut h = [0u8; HEADER];
    stream.read_exact(&mut h)?;
    let command = be32(&h[0..4]);
    let seqnum = be32(&h[4..8]);
    let status = be32(&h[20..24]) as i32;
    let mut data = Vec::new();
    if command == RET_SUBMIT && data_follows {
        let actual = usize::try_from(be32(&h[24..28]) as i32).unwrap_or(0);
        data.resize(actual, 0);
        stream.read_exact(&mut data)?;
    }
    Ok(Ret { command, seqnum, status, data })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::usbrelay::UsbRelay;

    fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> Setup {
        Setup { request_type, request, value, index, length }
    }

    #[test]
    fn serves_a_relay_board_to_a_client() {
        let log = Arc::new(Mutex::new(TxLog::new(None, false).unwrap()));
        let device = Arc::new(Mutex::new(Device::new(Box::new(UsbRelay::new(2)))));
        let server = Server::new(Arc::clone(&device), Arc::clone(&log), "1-1", 1, 2);
        let devid = server.devid();
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = thread::spawn(move || server.run(&listener, &stop2));

        assert!(import(addr, "9-9").is_err());
        let (mut s, record) = import(addr, "1-1").unwrap();
        assert_eq!(record.len(), DEVICE_RECORD);
        assert_eq!(&record[300..304], &[0x16, 0xc0, 0x05, 0xdf]);

        // Device descriptor.
        s.write_all(&cmd_submit(1, devid, DIR_IN, 0, 18, Some(setup(0x80, 6, 0x0100, 0, 18)), &[]))
            .unwrap();
        let r = read_ret(&mut s, true).unwrap();
        assert_eq!((r.command, r.seqnum, r.status, r.data.len()), (RET_SUBMIT, 1, 0, 18));
        // Configure, then a feature report through the HID interface.
        s.write_all(&cmd_submit(2, devid, DIR_OUT, 0, 0, Some(setup(0x00, 9, 1, 0, 0)), &[]))
            .unwrap();
        assert_eq!(read_ret(&mut s, false).unwrap().status, 0);
        s.write_all(&cmd_submit(3, devid, DIR_IN, 0, 8, Some(setup(0xa1, 1, 0x0300, 0, 8)), &[]))
            .unwrap();
        assert_eq!(read_ret(&mut s, true).unwrap().data, b"ABCDE\0\0\0");
        s.write_all(&cmd_submit(
            4,
            devid,
            DIR_OUT,
            0,
            8,
            Some(setup(0x21, 9, 0x0300, 0, 8)),
            &[0xff, 2, 0, 0, 0, 0, 0, 0],
        ))
        .unwrap();
        assert_eq!(read_ret(&mut s, false).unwrap().status, 0);
        s.write_all(&cmd_submit(5, devid, DIR_IN, 0, 8, Some(setup(0xa1, 1, 0x0300, 0, 8)), &[]))
            .unwrap();
        assert_eq!(read_ret(&mut s, true).unwrap().data, b"ABCDE\0\0\x02");
        // A fresh model behind the configured device: relays off again.
        device.lock().unwrap().replace_model(Box::new(UsbRelay::new(2)));
        s.write_all(&cmd_submit(50, devid, DIR_IN, 0, 8, Some(setup(0xa1, 1, 0x0300, 0, 8)), &[]))
            .unwrap();
        assert_eq!(read_ret(&mut s, true).unwrap().data, b"ABCDE\0\0\0");
        // An interrupt IN is held, then unlinked.
        s.write_all(&cmd_submit(6, devid, DIR_IN, 1, 64, None, &[])).unwrap();
        s.write_all(&cmd_submit(7, devid, DIR_IN, 0, 1, Some(setup(0x80, 8, 0, 0, 1)), &[]))
            .unwrap();
        let r = read_ret(&mut s, true).unwrap();
        assert_eq!((r.seqnum, r.data), (7, vec![1]));
        s.write_all(&cmd_unlink(8, 6)).unwrap();
        let r = read_ret(&mut s, false).unwrap();
        assert_eq!((r.command, r.seqnum, r.status), (RET_UNLINK, 8, ECONNRESET));
        s.write_all(&cmd_unlink(9, 6)).unwrap();
        assert_eq!(read_ret(&mut s, false).unwrap().status, 0);
        // A device qualifier request stalls.
        s.write_all(&cmd_submit(
            10,
            devid,
            DIR_IN,
            0,
            10,
            Some(setup(0x80, 6, 0x0600, 0, 10)),
            &[],
        ))
        .unwrap();
        assert_eq!(read_ret(&mut s, true).unwrap().status, EPIPE);
        drop(s);
        stop.store(true, Ordering::Relaxed);
        handle.join().unwrap().unwrap();
        assert_eq!(log.lock().unwrap().violations(), 0);
    }

    /// Answers each IN with `answer` bytes and remembers the lengths it
    /// was given: a bulk IN on 0x82 (64 byte packets), an interrupt IN
    /// on 0x81 (8 byte packets).
    struct Babbler {
        descriptors: crate::model::Descriptors,
        answer: usize,
        asked: Asked,
    }

    /// Which call (`submitted` or `transfer`), the endpoint, the length.
    type Asked = Arc<Mutex<Vec<(&'static str, u8, usize)>>>;

    impl crate::model::DeviceModel for Babbler {
        fn name(&self) -> &'static str {
            "babbler"
        }
        fn descriptors(&self) -> &crate::model::Descriptors {
            &self.descriptors
        }
        fn control(&mut self, _: Setup, _: &[u8], _: &mut Violations) -> Reply {
            Reply::ack()
        }
        fn out(&mut self, _: u8, _: &[u8], _: &mut Violations) -> Reply {
            Reply::ack()
        }
        fn in_transfer(&mut self, ep: u8, max: usize, _: &mut Violations) -> Reply {
            self.asked.lock().unwrap().push(("transfer", ep, max));
            Reply::Data(vec![0xa5; self.answer])
        }
        fn in_submitted(&mut self, ep: u8, max: usize) {
            self.asked.lock().unwrap().push(("submitted", ep, max));
        }
        fn reset(&mut self) {}
    }

    #[test]
    fn an_answer_longer_than_the_urb_is_a_babble() {
        use crate::descriptor::{self, DeviceDescriptor};
        let mut body = Vec::new();
        body.extend_from_slice(&descriptor::interface(0, 0, 2, [0xff, 0, 0], 0));
        body.extend_from_slice(&descriptor::endpoint(0x81, 3, 8, 1));
        body.extend_from_slice(&descriptor::endpoint(0x82, 2, 64, 0));
        let device = DeviceDescriptor {
            usb: 0x0200,
            class: [0, 0, 0],
            max_packet0: 64,
            vendor: 0x1234,
            product: 0x5678,
            device: 0x0100,
            strings: [0, 0, 0],
            configurations: 1,
        };
        let asked = Arc::new(Mutex::new(Vec::new()));
        let model = Babbler {
            descriptors: crate::model::Descriptors {
                speed: crate::model::Speed::Full,
                device: device.bytes(),
                configs: vec![descriptor::configuration(1, 0, 0x80, 50, &body)],
                strings: Vec::new(),
            },
            answer: 6,
            asked: Arc::clone(&asked),
        };
        let log = Arc::new(Mutex::new(TxLog::new(None, false).unwrap()));
        let device = Arc::new(Mutex::new(Device::new(Box::new(model))));
        let server = Server::new(Arc::clone(&device), Arc::clone(&log), "1-1", 1, 2);
        let devid = server.devid();
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = Arc::clone(&stop);
        let handle = thread::spawn(move || server.run(&listener, &stop2));
        let (mut s, _) = import(addr, "1-1").unwrap();
        s.write_all(&cmd_submit(1, devid, DIR_OUT, 0, 0, Some(setup(0x00, 9, 1, 0, 0)), &[]))
            .unwrap();
        assert_eq!(read_ret(&mut s, false).unwrap().status, 0);
        // Bulk: the model is asked for a whole packet, the URB of 4
        // bytes babbles with the 4 that fit.
        s.write_all(&cmd_submit(2, devid, DIR_IN, 2, 4, None, &[])).unwrap();
        let r = read_ret(&mut s, true).unwrap();
        assert_eq!((r.status, r.data), (EOVERFLOW, vec![0xa5; 4]));
        // A URB that holds the answer completes.
        s.write_all(&cmd_submit(3, devid, DIR_IN, 2, 6, None, &[])).unwrap();
        let r = read_ret(&mut s, true).unwrap();
        assert_eq!((r.status, r.data), (0, vec![0xa5; 6]));
        // Interrupt: the length is not rounded, and the same holds.
        s.write_all(&cmd_submit(4, devid, DIR_IN, 1, 2, None, &[])).unwrap();
        let r = read_ret(&mut s, true).unwrap();
        assert_eq!((r.status, r.data), (EOVERFLOW, vec![0xa5; 2]));
        drop(s);
        stop.store(true, Ordering::Relaxed);
        handle.join().unwrap().unwrap();
        assert_eq!(
            *asked.lock().unwrap(),
            [
                ("submitted", 2, 4),
                ("transfer", 2, 64),
                ("submitted", 2, 6),
                ("transfer", 2, 64),
                ("submitted", 1, 2),
                ("transfer", 1, 2),
            ]
        );
    }
}
