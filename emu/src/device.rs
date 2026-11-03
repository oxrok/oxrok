//! A USB device around a model: the standard requests addressed to the
//! device, the configuration state, endpoint halts, and the checks on
//! every transfer the host submits.

use crate::descriptor::{self, truncate};
use crate::model::{
    Arm, CONTROL, Descriptors, DeviceModel, Endpoint, Event, RECIPIENT_DEVICE, RECIPIENT_ENDPOINT,
    RECIPIENT_INTERFACE, Reply, STANDARD, Setup, Violations,
};

const GET_STATUS: u8 = 0;
const CLEAR_FEATURE: u8 = 1;
const SET_FEATURE: u8 = 3;
const SET_ADDRESS: u8 = 5;
const GET_DESCRIPTOR: u8 = 6;
const SET_DESCRIPTOR: u8 = 7;
const GET_CONFIGURATION: u8 = 8;
const SET_CONFIGURATION: u8 = 9;
const GET_INTERFACE: u8 = 10;
const SET_INTERFACE: u8 = 11;

const FEATURE_ENDPOINT_HALT: u16 = 0;
const FEATURE_REMOTE_WAKEUP: u16 = 1;

/// A model on the bus.
pub struct Device {
    model: Box<dyn DeviceModel>,
    descriptors: Descriptors,
    config: u8,
    halted: Vec<u8>,
    armed: Option<Arm>,
}

impl Device {
    /// Wraps a model; the device starts unconfigured.
    pub fn new(model: Box<dyn DeviceModel>) -> Device {
        let descriptors = model.descriptors().clone();
        Device { model, descriptors, config: 0, halted: Vec::new(), armed: None }
    }

    /// The model's name.
    pub fn name(&self) -> &'static str {
        self.model.name()
    }

    /// Puts a fresh model behind the same bus state (configuration,
    /// halts), so a second run starts from the device's initial state
    /// while the host still sees the device it enumerated. When the
    /// fresh model's descriptors differ from those the host enumerated
    /// (a device that renumerated during the run), the caller must
    /// re-attach it, see [`Device::needs_reenumeration`]; the
    /// descriptors are taken over at the next [`Device::reset`].
    pub fn replace_model(&mut self, model: Box<dyn DeviceModel>) {
        self.model = model;
        self.armed = None;
    }

    /// Arms an event for the run that follows; one at a time, and a
    /// fresh model ([`Device::replace_model`]) disarms it.
    pub fn arm(&mut self, arm: Arm) {
        self.armed = Some(arm);
    }

    /// After every offer of a transfer to the model: counts it for the
    /// armed event, asks the model for its point, and returns the event
    /// when it fires (once; it is disarmed then).
    pub fn fired(&mut self, ep: u8, is_in: bool, reply: &Reply) -> Option<Event> {
        let point = self.model.take_event_point();
        let arm = self.armed.as_mut()?;
        if arm.fires(ep & 0x0f, is_in, reply, point) {
            let event = arm.event;
            self.armed = None;
            return Some(event);
        }
        None
    }

    /// Whether the model's descriptors differ from those the host
    /// enumerated, so the device has to be attached again.
    pub fn needs_reenumeration(&self) -> bool {
        self.model.descriptors() != &self.descriptors
    }

    /// The model's descriptors, which after a renumeration are not yet
    /// those the host enumerated ([`Device::descriptors`]).
    pub fn model_descriptors(&self) -> &Descriptors {
        self.model.descriptors()
    }

    /// See [`DeviceModel::take_renumeration`].
    pub fn take_renumeration(&mut self) -> bool {
        self.model.take_renumeration()
    }

    /// See [`DeviceModel::log_digest_above`].
    pub fn log_digest_above(&self) -> Option<usize> {
        self.model.log_digest_above()
    }

    /// See [`DeviceModel::log_out`].
    pub fn log_out(&self, ep: u8, data: &[u8]) -> Option<Vec<u8>> {
        self.model.log_out(ep & 0x0f, data)
    }

    /// See [`DeviceModel::in_submitted`].
    pub fn in_submitted(&mut self, ep: u8, max: usize) {
        self.model.in_submitted(ep & 0x0f, max);
    }

    /// The descriptors.
    pub fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    /// The active `bConfigurationValue`, 0 when unconfigured.
    pub fn configuration(&self) -> u8 {
        self.config
    }

    /// An endpoint of the active configuration, by address.
    pub fn endpoint(&self, address: u8) -> Option<Endpoint> {
        if address.trailing_zeros() >= 4 {
            return Some(Endpoint {
                address,
                attributes: CONTROL,
                max_packet: u16::from(self.descriptors.device[7]),
                interval: 0,
            });
        }
        let config = self.descriptors.config_by_value(self.config)?;
        Descriptors::endpoints(config).into_iter().find(|e| e.address == address)
    }

    /// A bus reset or a new attach: unconfigured, halts cleared, the
    /// model reset, and its descriptors taken over (they change when a
    /// device renumerates).
    pub fn reset(&mut self) {
        self.config = 0;
        self.halted.clear();
        self.model.reset();
        self.descriptors = self.model.descriptors().clone();
    }

    /// A control transfer on endpoint 0.
    pub fn control(&mut self, setup: Setup, data: &[u8], v: &mut Violations) -> Reply {
        if !setup.is_in() && data.len() != usize::from(setup.length) {
            v.report(format!(
                "control OUT carries {} bytes, wLength says {}",
                data.len(),
                setup.length
            ));
        }
        if setup.kind() == STANDARD
            && let Some(reply) = self.standard(setup, v)
        {
            return reply;
        }
        let reply = self.model.control(setup, data, v);
        match reply {
            Reply::Data(d) if setup.is_in() => Reply::Data(truncate(&d, setup.length)),
            other => other,
        }
    }

    /// The standard requests the device answers itself.
    fn standard(&mut self, setup: Setup, v: &mut Violations) -> Option<Reply> {
        let reply = match (setup.recipient(), setup.request, setup.is_in()) {
            (RECIPIENT_DEVICE | RECIPIENT_INTERFACE, GET_STATUS, true) => Reply::Data(vec![0, 0]),
            (RECIPIENT_DEVICE, CLEAR_FEATURE | SET_FEATURE, false) => {
                if setup.value == FEATURE_REMOTE_WAKEUP {
                    Reply::ack()
                } else {
                    v.report(format!("device feature {} is not one the device has", setup.value));
                    Reply::Stall
                }
            }
            (RECIPIENT_DEVICE, SET_ADDRESS, false) => Reply::ack(),
            (RECIPIENT_DEVICE, GET_DESCRIPTOR, true) => self.descriptor(setup, v),
            (RECIPIENT_DEVICE, SET_DESCRIPTOR, false) => {
                v.report("SET_DESCRIPTOR");
                Reply::Stall
            }
            (RECIPIENT_DEVICE, GET_CONFIGURATION, true) => Reply::Data(vec![self.config]),
            (RECIPIENT_DEVICE, SET_CONFIGURATION, false) => {
                let value = setup.value as u8;
                if value == 0 || self.descriptors.config_by_value(value).is_some() {
                    self.config = value;
                    self.halted.clear();
                    Reply::ack()
                } else {
                    v.report(format!("SET_CONFIGURATION {value}: no such configuration"));
                    Reply::Stall
                }
            }
            (RECIPIENT_INTERFACE, GET_INTERFACE, true) => Reply::Data(vec![0]),
            (RECIPIENT_INTERFACE, SET_INTERFACE, false) => {
                let number = (setup.index & 0xff) as u8;
                let alternate = setup.value as u8;
                let known = self.descriptors.config_by_value(self.config).is_some_and(|c| {
                    Descriptors::interfaces(c).iter().any(|i| i[0] == number && i[1] == alternate)
                });
                if known {
                    Reply::ack()
                } else {
                    v.report(format!(
                        "SET_INTERFACE {number} alternate {alternate}: not in the configuration"
                    ));
                    Reply::Stall
                }
            }
            (RECIPIENT_ENDPOINT, GET_STATUS, true) => {
                let address = (setup.index & 0xff) as u8;
                Reply::Data(vec![u8::from(self.halted.contains(&address)), 0])
            }
            (RECIPIENT_ENDPOINT, CLEAR_FEATURE | SET_FEATURE, false) => {
                let address = (setup.index & 0xff) as u8;
                if setup.value != FEATURE_ENDPOINT_HALT {
                    v.report(format!(
                        "endpoint feature {} is not one an endpoint has",
                        setup.value
                    ));
                    return Some(Reply::Stall);
                }
                if self.endpoint(address).is_none() {
                    v.report(format!(
                        "halt request for endpoint {address:#04x}, which the configuration lacks"
                    ));
                    return Some(Reply::Stall);
                }
                self.halted.retain(|&a| a != address);
                if setup.request == SET_FEATURE {
                    self.halted.push(address);
                }
                Reply::ack()
            }
            _ => return None,
        };
        Some(reply)
    }

    fn descriptor(&self, setup: Setup, v: &mut Violations) -> Reply {
        let kind = (setup.value >> 8) as u8;
        let index = usize::from(setup.value & 0xff);
        let bytes = match kind {
            descriptor::DEVICE => Some(self.descriptors.device.clone()),
            descriptor::CONFIGURATION => {
                let c = self.descriptors.configs.get(index).cloned();
                if c.is_none() {
                    v.report(format!(
                        "GET_DESCRIPTOR configuration {index}: the device has {}",
                        self.descriptors.configs.len()
                    ));
                }
                c
            }
            descriptor::STRING => {
                if index == 0 {
                    Some(descriptor::language_table())
                } else {
                    let s = self.descriptors.strings.get(index - 1).map(|s| descriptor::string(s));
                    if s.is_none() {
                        v.report(format!(
                            "GET_DESCRIPTOR string {index}: not one the descriptors name"
                        ));
                    }
                    s
                }
            }
            // A full or low speed device has neither; a host may ask.
            descriptor::DEVICE_QUALIFIER | descriptor::BOS => None,
            other => {
                v.report(format!("GET_DESCRIPTOR of type {other:#04x} to the device"));
                None
            }
        };
        match bytes {
            Some(b) => Reply::Data(truncate(&b, setup.length)),
            None => Reply::Stall,
        }
    }

    fn check_endpoint(&self, address: u8, v: &mut Violations) -> Option<Endpoint> {
        if self.config == 0 {
            v.report(format!("transfer on endpoint {address:#04x} before SET_CONFIGURATION"));
        }
        let Some(ep) = self.endpoint(address) else {
            v.report(format!("transfer on endpoint {address:#04x}, which the configuration lacks"));
            return None;
        };
        Some(ep)
    }

    /// An IN transfer from endpoint number `ep`, up to `max` bytes.
    pub fn transfer_in(&mut self, ep: u8, max: usize, v: &mut Violations) -> Reply {
        let address = 0x80 | (ep & 0x0f);
        if self.check_endpoint(address, v).is_none() {
            return Reply::Stall;
        }
        if self.halted.contains(&address) {
            return Reply::Stall;
        }
        self.model.in_transfer(ep & 0x0f, max, v)
    }

    /// An OUT transfer to endpoint number `ep`.
    pub fn transfer_out(&mut self, ep: u8, data: &[u8], v: &mut Violations) -> Reply {
        let address = ep & 0x0f;
        if self.check_endpoint(address, v).is_none() {
            return Reply::Stall;
        }
        if self.halted.contains(&address) {
            return Reply::Stall;
        }
        self.model.out(address, data, v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::usbrelay::UsbRelay;

    fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> Setup {
        Setup { request_type, request, value, index, length }
    }

    #[test]
    fn enumerates_like_a_host() {
        let mut dev = Device::new(Box::new(UsbRelay::new(2)));
        let mut v = Violations::default();
        let Reply::Data(d) = dev.control(setup(0x80, GET_DESCRIPTOR, 0x0100, 0, 64), &[], &mut v)
        else {
            panic!()
        };
        assert_eq!(d.len(), 18);
        assert_eq!(&d[8..12], &[0xc0, 0x16, 0xdf, 0x05]);
        let Reply::Data(d) = dev.control(setup(0x80, GET_DESCRIPTOR, 0x0200, 0, 9), &[], &mut v)
        else {
            panic!()
        };
        let total = u16::from_le_bytes([d[2], d[3]]);
        let Reply::Data(d) =
            dev.control(setup(0x80, GET_DESCRIPTOR, 0x0200, 0, total), &[], &mut v)
        else {
            panic!()
        };
        assert_eq!(d.len(), usize::from(total));
        assert_eq!(
            dev.control(setup(0x80, GET_DESCRIPTOR, 0x0300, 0, 255), &[], &mut v),
            Reply::Data(vec![4, 3, 9, 4])
        );
        let Reply::Data(d) =
            dev.control(setup(0x80, GET_DESCRIPTOR, 0x0302, 0x0409, 255), &[], &mut v)
        else {
            panic!()
        };
        assert_eq!(
            String::from_utf16_lossy(
                &d[2..].chunks(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect::<Vec<_>>()
            ),
            "USBRelay2"
        );
        assert_eq!(
            dev.control(setup(0x80, GET_DESCRIPTOR, 0x0600, 0, 10), &[], &mut v),
            Reply::Stall
        );
        assert!(v.is_empty(), "{v:?}");
        // Before configuration the interrupt endpoint is not there.
        assert_eq!(dev.transfer_in(1, 64, &mut v), Reply::Stall);
        assert!(!v.is_empty());
        v.drain();
        assert_eq!(dev.control(setup(0x00, SET_CONFIGURATION, 1, 0, 0), &[], &mut v), Reply::ack());
        assert_eq!(dev.configuration(), 1);
        assert_eq!(dev.transfer_in(1, 64, &mut v), Reply::Nak);
        assert_eq!(
            dev.control(setup(0x82, GET_STATUS, 0, 0x81, 2), &[], &mut v),
            Reply::Data(vec![0, 0])
        );
        assert_eq!(dev.control(setup(0x02, SET_FEATURE, 0, 0x81, 0), &[], &mut v), Reply::ack());
        assert_eq!(dev.transfer_in(1, 64, &mut v), Reply::Stall);
        assert_eq!(dev.control(setup(0x02, CLEAR_FEATURE, 0, 0x81, 0), &[], &mut v), Reply::ack());
        assert_eq!(dev.transfer_in(1, 64, &mut v), Reply::Nak);
        assert!(v.is_empty(), "{v:?}");
        assert_eq!(dev.control(setup(0x00, SET_CONFIGURATION, 7, 0, 0), &[], &mut v), Reply::Stall);
        assert_eq!(v.drain().len(), 1);
    }

    #[test]
    fn armed_events_fire_once_at_their_point() {
        use crate::model::{Arm, Event, When};
        assert_eq!(
            Arm::parse("unplug in=1:3").unwrap(),
            Arm::new(Event::Unplug, When::In { ep: 1, n: 3 })
        );
        assert_eq!(Arm::parse("interrupt model").unwrap(), Arm::new(Event::Interrupt, When::Model));
        assert_eq!(
            Arm::parse("unplug out=2:1").unwrap(),
            Arm::new(Event::Unplug, When::Out { ep: 2, n: 1 })
        );
        for bad in ["", "unplug", "explode model", "unplug in=1:0", "unplug in=16:1", "unplug at=1"]
        {
            assert!(Arm::parse(bad).is_err(), "{bad}");
        }
        let mut dev = Device::new(Box::new(UsbRelay::new(2)));
        let data = Reply::Data(vec![1]);
        dev.arm(Arm::parse("unplug in=1:2").unwrap());
        assert_eq!(dev.fired(0x81, true, &Reply::Nak), None, "a NAK does not count");
        assert_eq!(dev.fired(0x01, false, &Reply::ack()), None, "nor does an OUT");
        assert_eq!(dev.fired(0x82, true, &data), None, "nor another endpoint");
        assert_eq!(dev.fired(0x81, true, &data), None);
        assert_eq!(dev.fired(0x81, true, &Reply::Stall), Some(Event::Unplug));
        assert_eq!(dev.fired(0x81, true, &data), None, "once");
        // A fresh model disarms.
        dev.arm(Arm::parse("interrupt out=0:1").unwrap());
        dev.replace_model(Box::new(UsbRelay::new(2)));
        assert_eq!(dev.fired(0, false, &Reply::ack()), None);
    }
}
