// SPDX-License-Identifier: MIT OR Apache-2.0

//! Errors that can happen when working with [`Uart16550`].

use crate::backend::RegisterAddress;
use crate::spec::NonIntegerDivisorError;
use core::error::Error;
use core::fmt::Display;

#[cfg(doc)]
use crate::Uart16550;

/// The specified address is invalid because it is either null or doesn't offer
/// [`offsets::MAX`] subsequent addresses.
///
/// [`offsets::MAX`]: crate::spec::registers::offsets::MAX
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct InvalidAddressError<A: RegisterAddress>(pub(crate) A);

impl<A: RegisterAddress> Display for InvalidAddressError<A> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "invalid register address: {:x?}", self.0)
    }
}

impl<A: RegisterAddress> Error for InvalidAddressError<A> {}

/// The loopback test failed error.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct LoopbackFailedError;

impl Display for LoopbackFailedError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "the loopback test failed")
    }
}

impl Error for LoopbackFailedError {}

/// Errors that can happen when a [`Uart16550`] initialized in
/// [`Uart16550::init`].
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum InitError {
    /// The device could not be detected.
    DeviceNotPresent,
    /// The loopback self-test after initialization failed.
    LoopbackTestFailed,
    /// The configured baud rate can not be set as it results in an invalid
    /// divisor.
    InvalidBaudRate(NonIntegerDivisorError),
}

impl Display for InitError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::DeviceNotPresent => {
                write!(f, "the device could not be detected")
            }
            Self::LoopbackTestFailed => {
                write!(f, "the loopback self-test after initialization failed")
            }
            Self::InvalidBaudRate(e) => {
                write!(f, "invalid baud rate: {e}")
            }
        }
    }
}

impl Error for InitError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::InvalidBaudRate(err) => Some(err),
            _ => None,
        }
    }
}

/// There is currently no data to read.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct ByteReceiveError;

impl Display for ByteReceiveError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "there is no data to read")
    }
}

impl Error for ByteReceiveError {}

/// Errors that happen when trying to send a byte
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum ByteSendError {
    /// There is currently no capacity to send another byte.
    ///
    /// For example, the FIFO might be full.
    NoCapacity,
    /// The remote is not (yet) ready to receive more data.
    ///
    /// This can for example mean that it is still processing input data.
    RemoteNotClearToSend,
}

/// Errors indicating the device is not ready to send data.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum RemoteReadyToReceiveError {
    /// There is no remote connected.
    NoRemoteConnected,
    /// A remote endpoint is present but has not asserted readiness to receive
    /// data.
    ///
    /// This reflects only the remote's configured receive readiness and does
    /// not imply anything about its actual buffering capacity.
    RemoteNotConfigured,
    /// The remote is not (yet) ready to receive more data.
    ///
    /// This can for example mean that it is still processing input data.
    RemoteNotClearToSend,
}

impl Display for RemoteReadyToReceiveError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NoRemoteConnected => {
                write!(f, "there is no remote connection")
            }
            Self::RemoteNotConfigured => write!(
                f,
                "remote is connected but did not signal it is ready to receive data"
            ),
            Self::RemoteNotClearToSend => {
                write!(f, "remote is not (yet) ready to receive more data")
            }
        }
    }
}

impl Error for RemoteReadyToReceiveError {}
