//! Core types of oxrok, a sigrok compatible logic analyzer stack.
//!
//! This crate holds everything a driver, a file format or a front end
//! needs to talk to each other:
//!
//! - [`Key`] and [`Value`]: the configuration model. The well known
//!   sigrok keys are variants with their sigrok ids, and a driver can add
//!   its own with [`Key::Custom`].
//! - [`Packet`]: the data feed. Drivers produce packets, outputs and
//!   decoders consume them.
//! - [`Driver`] and [`Device`]: the hardware contract. A device owns its
//!   acquisition thread and sends packets through a [`PacketSink`].
//! - [`Session`]: runs one or more devices and fans their packets out.
//! - [`util`]: parsing and formatting helpers shared by the CLI and the
//!   formats (sample rate strings, size suffixes, time strings).
//!
//! Nothing here touches USB or files. Those live in their own crates.

#![forbid(unsafe_code)]

pub mod analog;
pub mod channel;
pub mod device;
pub mod error;
pub mod firmware;
pub mod key;
pub mod option;
pub mod packet;
pub mod session;
pub mod sessions;
pub mod softtrig;
pub mod trigger;
pub mod trigger_syntax;
pub mod util;
pub mod value;

pub use analog::{Mq, MqFlags, Unit};
pub use channel::{Channel, ChannelGroup, ChannelKind};
pub use device::{Device, DeviceClass, DeviceInfo, Driver, PacketSink, Registry};
pub use error::{Error, Result};
pub use key::Key;
pub use option::{Access, OptionDesc, ValueList, check_value, config_set, resolve_key};
pub use packet::{AnalogData, LogicData, Packet};
pub use session::Session;
pub use softtrig::{PreTriggerBuffer, SoftTrigger, SoftTriggerLogic};
pub use trigger::{
    Trigger, TriggerCombine, TriggerCount, TriggerDuration, TriggerFeatures, TriggerMatch,
    TriggerRange, TriggerStage,
};
pub use value::{Value, ValueKind};

/// The version of the oxrok workspace this crate was built from.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
