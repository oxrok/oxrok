// Counterpart in sigrok:
//   libsigrok/src/error.c.
// This file is a port of the sigrok counterpart, GPL-3.0-or-later like it.

//! The error type shared by every oxrok crate.

/// What can go wrong when talking to a device, a file or a session.
///
/// The categories follow the ones libsigrok users know, so a front end
/// can keep its existing handling: an argument error is the caller's
/// fault, a device error is the hardware's, and a missing firmware file
/// names the file so the front end can tell the user what to install.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// An I/O error from the operating system.
    #[error("{0}")]
    Io(#[from] std::io::Error),
    /// An invalid argument from the caller.
    #[error("invalid argument: {0}")]
    Arg(String),
    /// The device or driver cannot do what was asked.
    #[error("not supported: {0}")]
    NotSupported(String),
    /// The device is already in use.
    #[error("device is busy")]
    Busy,
    /// The device is gone (unplugged), libusb's `LIBUSB_ERROR_NO_DEVICE`.
    #[error("device disconnected")]
    Disconnected,
    /// The device did not answer in time.
    #[error("timeout")]
    Timeout,
    /// A firmware or bitstream file is missing.
    #[error("firmware file not found: {0}")]
    Firmware(String),
    /// The device misbehaved.
    #[error("device error: {0}")]
    Device(String),
    /// The named channel group does not exist.
    #[error("no such channel group: {0}")]
    ChannelGroup(String),
    /// A key the device does not have.
    #[error("unknown option: {0}")]
    UnknownKey(String),
    /// A capture that started failed on the way: what
    /// [`Session::run`](crate::core::Session::run) returns after every packet,
    /// [`Packet::End`](crate::core::Packet::End) included, reached the
    /// consumer. The text is what [`Packet::Failed`](crate::core::Packet::Failed)
    /// held.
    #[error("capture failed: {0}")]
    CaptureFailed(String),
    /// The end of the input, e.g. of a decoder's sample stream.
    #[error("end of data")]
    Eof,
}

impl Error {
    /// An argument error from anything that formats.
    pub fn arg(msg: impl Into<String>) -> Self {
        Error::Arg(msg.into())
    }

    /// A not-supported error from anything that formats.
    pub fn not_supported(msg: impl Into<String>) -> Self {
        Error::NotSupported(msg.into())
    }

    /// A device error from anything that formats.
    pub fn device(msg: impl Into<String>) -> Self {
        Error::Device(msg.into())
    }
}

/// The result type used across oxrok.
pub type Result<T> = std::result::Result<T, Error>;
