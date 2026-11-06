// Counterpart in sigrok:
//   libsigrok/src/session.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! A session runs devices and hands their packets to a consumer.

use crate::core::device::{Device, PacketSink};
use crate::core::error::{Error, Result};
use crate::core::packet::Packet;
use crate::core::trigger::Trigger;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, sync_channel};
use std::time::Duration;

/// How many packets may wait between the devices and the consumer.
const QUEUE_DEPTH: usize = 64;

/// What the consumer wants after a packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Flow {
    /// Keep going.
    Continue,
    /// Stop every device; the session ends once they sent their last
    /// packet.
    Stop,
}

/// One or more devices acquiring together.
pub struct Session {
    devices: Vec<Box<dyn Device>>,
    trigger: Option<Trigger>,
    stop_flag: Arc<AtomicBool>,
}

impl Default for Session {
    fn default() -> Self {
        Session::new()
    }
}

impl Session {
    /// An empty session.
    pub fn new() -> Self {
        Session { devices: Vec::new(), trigger: None, stop_flag: Arc::new(AtomicBool::new(false)) }
    }

    /// Adds a device. It must be open.
    pub fn add_device(&mut self, device: Box<dyn Device>) -> usize {
        self.devices.push(device);
        self.devices.len() - 1
    }

    /// The devices, in the order they were added.
    pub fn devices(&self) -> &[Box<dyn Device>] {
        &self.devices
    }

    /// Mutable access to the devices.
    pub fn devices_mut(&mut self) -> &mut [Box<dyn Device>] {
        &mut self.devices
    }

    /// Takes the devices out of the session, e.g. to close them.
    pub fn into_devices(self) -> Vec<Box<dyn Device>> {
        self.devices
    }

    /// The trigger for the next run, applied to every device.
    pub fn set_trigger(&mut self, trigger: Option<Trigger>) {
        self.trigger = trigger;
    }

    /// A flag that stops a running session when set, e.g. from a signal
    /// handler.
    pub fn stop_flag(&self) -> Arc<AtomicBool> {
        Arc::clone(&self.stop_flag)
    }

    /// Runs one acquisition on every device and calls `consumer` with
    /// each packet and the index of the device it came from. Returns when
    /// every device sent [`Packet::End`], or after the consumer asked to
    /// stop and the devices ended.
    ///
    /// A device that fails reaches the consumer as [`Packet::Failed`] where
    /// it happened, and its [`Packet::End`] follows; the other devices go on
    /// until they end. `run` then returns [`Error::CaptureFailed`] with the
    /// first failure's text, after every packet was delivered: what the
    /// consumer got before the failure is good, however little that was. A
    /// failure that arrives once a stop was asked for still reaches the
    /// consumer, but the run is a stopped one and returns `Ok`. A device
    /// whose thread ends without its `End` (a panic) counts as failed unless
    /// stopping, and the consumer gets an `End` in its name. A device that
    /// refuses to start is the `Err` of its `start`, and no failure of this
    /// kind.
    pub fn run(&mut self, mut consumer: impl FnMut(usize, &Packet) -> Flow) -> Result<()> {
        if self.devices.is_empty() {
            return Err(Error::arg("a session needs at least one device"));
        }
        self.stop_flag.store(false, Ordering::SeqCst);
        for dev in &mut self.devices {
            if !dev.channels().iter().any(|c| c.enabled) {
                return Err(Error::arg(format!(
                    "{} device has no enabled channels",
                    dev.info().driver
                )));
            }
            dev.set_trigger(self.trigger.clone())?;
        }
        let (tx, rx) = sync_channel(QUEUE_DEPTH);
        let mut started = 0;
        let mut start_err = None;
        for (i, dev) in self.devices.iter_mut().enumerate() {
            match dev.start(PacketSink::new(tx.clone(), i)) {
                Ok(()) => started += 1,
                Err(e) => {
                    start_err = Some(e);
                    break;
                }
            }
        }
        drop(tx);
        if let Some(e) = start_err {
            // Drain what the started devices send, so their threads end.
            let _ = stop_draining(&mut self.devices[..started], &rx, |_, _| {});
            while rx.recv().is_ok() {}
            return Err(e);
        }
        let mut ended = vec![false; started];
        let mut stopping = false;
        let mut failure = None;
        while ended.contains(&false) {
            match rx.recv_timeout(Duration::from_millis(100)) {
                Ok((dev, packet)) => {
                    if let Packet::Failed(why) = &packet
                        && !self.stop_flag.load(Ordering::SeqCst)
                    {
                        failure.get_or_insert_with(|| why.clone());
                    }
                    if consumer(dev, &packet) == Flow::Stop {
                        self.stop_flag.store(true, Ordering::SeqCst);
                    }
                    if matches!(packet, Packet::End) {
                        ended[dev] = true;
                    }
                }
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => break,
            }
            if !stopping && self.stop_flag.load(Ordering::SeqCst) {
                stopping = true;
                stop_draining(&mut self.devices, &rx, |dev, packet| {
                    // Already stopping: what the consumer answers no
                    // longer matters.
                    let _ = consumer(dev, &packet);
                    if matches!(packet, Packet::End) {
                        ended[dev] = true;
                    }
                })?;
            }
        }
        // Every sender is gone and some End never came: the thread died.
        for (dev, _) in ended.iter().enumerate().filter(|(_, ended)| !**ended) {
            if !self.stop_flag.load(Ordering::SeqCst) {
                failure.get_or_insert_with(|| {
                    format!(
                        "the {} driver's acquisition ended without saying how",
                        self.devices[dev].info().driver
                    )
                });
            }
            let _ = consumer(dev, &Packet::End);
        }
        failure.map_or(Ok(()), |why| Err(Error::CaptureFailed(why)))
    }
}

/// Stops `devices` while `each` takes what they still send.
///
/// A device's stop joins its acquisition thread, which may be blocked
/// on a full queue: stopping from the loop that empties the queue would
/// wait for ever. So the stops run on a thread of their own until they
/// return, and the caller's thread goes on emptying the queue.
fn stop_draining(
    devices: &mut [Box<dyn Device>],
    rx: &Receiver<(usize, Packet)>,
    mut each: impl FnMut(usize, Packet),
) -> Result<()> {
    std::thread::scope(|scope| {
        let stopper = scope.spawn(move || -> Result<()> {
            for dev in devices.iter_mut() {
                dev.stop()?;
            }
            Ok(())
        });
        while !stopper.is_finished() {
            match rx.recv_timeout(Duration::from_millis(10)) {
                Ok((dev, packet)) => each(dev, packet),
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => break,
            }
        }
        stopper.join().unwrap_or_else(|panic| std::panic::resume_unwind(panic))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::channel::{Channel, ChannelKind};
    use crate::core::device::{DeviceClass, DeviceInfo};
    use crate::core::key::Key;
    use crate::core::option::{OptionDesc, ValueList};
    use crate::core::value::Value;
    use std::sync::mpsc::channel;
    use std::thread::JoinHandle;

    /// More packets than the queue holds.
    const BURST: usize = QUEUE_DEPTH * 4;

    /// A driver whose thread sends its whole burst before it looks at
    /// the stop flag, and whose stop joins that thread, as a capture
    /// sent from a memory image does.
    struct Bursting {
        info: DeviceInfo,
        channels: Vec<Channel>,
        fail_start: bool,
        stop: Arc<AtomicBool>,
        worker: Option<JoinHandle<()>>,
    }

    impl Bursting {
        fn new(fail_start: bool) -> Box<Self> {
            Box::new(Bursting {
                info: DeviceInfo::default(),
                channels: vec![Channel::new(0, ChannelKind::Logic, "D0")],
                fail_start,
                stop: Arc::new(AtomicBool::new(false)),
                worker: None,
            })
        }
    }

    impl Device for Bursting {
        fn info(&self) -> &DeviceInfo {
            &self.info
        }
        fn classes(&self) -> &[DeviceClass] {
            &[]
        }
        fn channels(&self) -> &[Channel] {
            &self.channels
        }
        fn enable_channel(&mut self, _: usize, _: bool) -> Result<()> {
            Ok(())
        }
        fn rename_channel(&mut self, _: usize, _: &str) -> Result<()> {
            Ok(())
        }
        fn options(&self, _: Option<&str>) -> Result<Vec<OptionDesc>> {
            Ok(Vec::new())
        }
        fn get(&self, key: &Key, _: Option<&str>) -> Result<Value> {
            Err(Error::UnknownKey(key.id().to_owned()))
        }
        fn set(&mut self, key: &Key, _: &Value, _: Option<&str>) -> Result<()> {
            Err(Error::UnknownKey(key.id().to_owned()))
        }
        fn list(&self, key: &Key, _: Option<&str>) -> Result<ValueList> {
            Err(Error::UnknownKey(key.id().to_owned()))
        }
        fn start(&mut self, sink: PacketSink) -> Result<()> {
            if self.fail_start {
                return Err(Error::device("refused"));
            }
            self.worker = Some(std::thread::spawn(move || {
                for _ in 0..BURST {
                    if !sink.send(Packet::logic(1, vec![0])) {
                        return;
                    }
                }
                sink.send(Packet::End);
            }));
            Ok(())
        }
        fn stop(&mut self) -> Result<()> {
            self.stop.store(true, Ordering::SeqCst);
            if let Some(w) = self.worker.take() {
                w.join().expect("worker");
            }
            Ok(())
        }
    }

    /// Runs `body` with a limit, so that a deadlock fails the test
    /// rather than holding the suite.
    fn within_limit<T: Send + 'static>(body: impl FnOnce() -> T + Send + 'static) -> T {
        let (tx, rx) = channel();
        std::thread::spawn(move || {
            let _ = tx.send(body());
        });
        rx.recv_timeout(Duration::from_secs(20)).expect("the session deadlocked")
    }

    #[test]
    fn a_stop_with_a_full_queue_ends_and_delivers_everything() {
        let packets = within_limit(|| {
            let mut session = Session::new();
            session.add_device(Bursting::new(false));
            let mut packets = 0;
            session
                .run(|_, _| {
                    packets += 1;
                    // Stop at once: the thread is blocked on the queue.
                    std::thread::sleep(Duration::from_millis(1));
                    Flow::Stop
                })
                .unwrap();
            packets
        });
        assert_eq!(packets, BURST + 1, "every packet and the end");
    }

    #[test]
    fn a_stop_flag_with_a_full_queue_ends() {
        within_limit(|| {
            let mut session = Session::new();
            session.add_device(Bursting::new(false));
            let flag = session.stop_flag();
            session
                .run(|_, _| {
                    flag.store(true, Ordering::SeqCst);
                    Flow::Continue
                })
                .unwrap();
        });
    }

    /// A device whose thread sends one logic packet, then ends as told.
    struct Scripted {
        info: DeviceInfo,
        channels: Vec<Channel>,
        outcome: Option<Result<()>>,
        worker: Option<JoinHandle<()>>,
    }

    impl Scripted {
        /// `outcome`: how it ends; `None` is a thread that dies with no end.
        fn new(outcome: Option<Result<()>>) -> Box<Self> {
            Box::new(Scripted {
                info: DeviceInfo { driver: "scripted", ..DeviceInfo::default() },
                channels: vec![Channel::new(0, ChannelKind::Logic, "D0")],
                outcome,
                worker: None,
            })
        }
    }

    impl Device for Scripted {
        fn info(&self) -> &DeviceInfo {
            &self.info
        }
        fn classes(&self) -> &[DeviceClass] {
            &[]
        }
        fn channels(&self) -> &[Channel] {
            &self.channels
        }
        fn enable_channel(&mut self, _: usize, _: bool) -> Result<()> {
            Ok(())
        }
        fn rename_channel(&mut self, _: usize, _: &str) -> Result<()> {
            Ok(())
        }
        fn options(&self, _: Option<&str>) -> Result<Vec<OptionDesc>> {
            Ok(Vec::new())
        }
        fn get(&self, key: &Key, _: Option<&str>) -> Result<Value> {
            Err(Error::UnknownKey(key.id().to_owned()))
        }
        fn set(&mut self, key: &Key, _: &Value, _: Option<&str>) -> Result<()> {
            Err(Error::UnknownKey(key.id().to_owned()))
        }
        fn list(&self, key: &Key, _: Option<&str>) -> Result<ValueList> {
            Err(Error::UnknownKey(key.id().to_owned()))
        }
        fn start(&mut self, sink: PacketSink) -> Result<()> {
            let outcome = self.outcome.take();
            self.worker = Some(std::thread::spawn(move || {
                sink.send(Packet::logic(1, vec![7]));
                if let Some(outcome) = outcome {
                    sink.end(outcome);
                }
            }));
            Ok(())
        }
        fn stop(&mut self) -> Result<()> {
            if let Some(w) = self.worker.take() {
                w.join().expect("worker");
            }
            Ok(())
        }
    }

    /// The packets of one run, as (device, packet), and its result as text.
    fn run_all(
        devices: Vec<Box<dyn Device>>,
    ) -> (Vec<(usize, Packet)>, std::result::Result<(), String>) {
        let mut session = Session::new();
        for d in devices {
            session.add_device(d);
        }
        let mut packets = Vec::new();
        let result = session.run(|dev, p| {
            packets.push((dev, p.clone()));
            Flow::Continue
        });
        (packets, result.map_err(|e| e.to_string()))
    }

    #[test]
    fn a_failure_comes_back_after_the_data_and_the_end() {
        let (packets, result) =
            within_limit(|| run_all(vec![Scripted::new(Some(Err(Error::Disconnected)))]));
        let packets: Vec<Packet> = packets.into_iter().map(|(_, p)| p).collect();
        assert_eq!(
            packets,
            [
                Packet::logic(1, vec![7]),
                Packet::Failed("device disconnected".to_owned()),
                Packet::End
            ]
        );
        assert_eq!(result.unwrap_err(), "capture failed: device disconnected");
    }

    #[test]
    fn a_capture_that_ends_well_returns_ok() {
        let (packets, result) = within_limit(|| run_all(vec![Scripted::new(Some(Ok(())))]));
        assert_eq!(result, Ok(()));
        assert!(!packets.iter().any(|(_, p)| matches!(p, Packet::Failed(_))));
    }

    #[test]
    fn one_failed_device_waits_for_the_others_and_gives_the_first_reason() {
        let (packets, result) = within_limit(|| {
            run_all(vec![Scripted::new(Some(Err(Error::Timeout))), Scripted::new(Some(Ok(())))])
        });
        assert_eq!(result.unwrap_err(), "capture failed: timeout");
        for dev in 0..2 {
            assert!(packets.contains(&(dev, Packet::End)), "device {dev}: {packets:?}");
        }
    }

    #[test]
    fn a_thread_that_dies_is_a_failure_and_still_gets_an_end() {
        let (packets, result) = within_limit(|| run_all(vec![Scripted::new(None)]));
        assert!(result.unwrap_err().contains("the scripted driver's"));
        assert_eq!(packets.last(), Some(&(0, Packet::End)));
    }

    #[test]
    fn a_failure_after_a_stop_is_a_stopped_run() {
        let (seen, result) = within_limit(|| {
            let mut session = Session::new();
            session.add_device(Scripted::new(Some(Err(Error::Timeout))));
            let mut seen = false;
            // The first packet asks for the stop; the failure comes after.
            let result = session.run(|_, p| {
                seen |= matches!(p, Packet::Failed(_));
                Flow::Stop
            });
            (seen, result.map_err(|e| e.to_string()))
        });
        assert_eq!(result, Ok(()));
        assert!(seen, "the consumer still sees the failure");
    }

    #[test]
    fn end_while_gives_up_with_a_full_queue_and_no_reader() {
        let (tx, rx) = sync_channel(1);
        let sink = PacketSink::new(tx, 0);
        assert!(sink.send(Packet::FrameBegin));
        assert!(!sink.end_while(Err(Error::Timeout), || false));
        drop(rx);
        assert!(!sink.end(Err(Error::Timeout)));
    }

    #[test]
    fn a_failed_start_stops_a_device_with_a_full_queue() {
        let e = within_limit(|| {
            let mut session = Session::new();
            session.add_device(Bursting::new(false));
            session.add_device(Bursting::new(true));
            session.run(|_, _| Flow::Continue).unwrap_err().to_string()
        });
        assert!(e.contains("refused"), "{e}");
    }
}
