// SPDX-License-Identifier: MIT OR Apache-2.0

//! Configuration for [`Uart16550`].
//!
//! [`Uart16550`]: crate::Uart16550

/// The speed of data transmission, measured in symbols per second (or bits, in
/// the case of simple UARTs).
///
/// This type is a convenient and non-ABI compatible abstraction. Use
/// [`calc_divisor`] to get the divisor for [`DLL`] and [`DLM`].
///
/// [`DLL`]: registers::DLL
/// [`DLM`]: registers::DLM
#[allow(missing_docs)]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash)]
pub enum BaudRate {
    // List of typical baud rates.
    #[default]
    Baud115200,
    Baud57600,
    Baud38400,
    Baud9600,
    Baud4800,
    Baud2400,
    Baud1200,
    Baud600,
    Baud300,
    Baud150,
    Baud110,
    Custom(u32),
}

impl BaudRate {
    /// Returns the value as corresponding integer.
    #[must_use]
    pub const fn to_integer(self) -> u32 {
        match self {
            BaudRate::Baud115200 => 115200,
            BaudRate::Baud57600 => 57600,
            BaudRate::Baud38400 => 38400,
            BaudRate::Baud9600 => 9600,
            BaudRate::Baud4800 => 4800,
            BaudRate::Baud2400 => 2400,
            BaudRate::Baud1200 => 1200,
            BaudRate::Baud600 => 600,
            BaudRate::Baud300 => 300,
            BaudRate::Baud150 => 150,
            BaudRate::Baud110 => 110,
            BaudRate::Custom(val) => val,
        }
    }

    /// Try to create the type from an integer representation of the baud rate.
    pub const fn from_integer(value: u32) -> Self {
        match value {
            115200 => Self::Baud115200,
            57600 => Self::Baud57600,
            38400 => Self::Baud38400,
            9600 => Self::Baud9600,
            4800 => Self::Baud4800,
            2400 => Self::Baud2400,
            1200 => Self::Baud1200,
            600 => Self::Baud600,
            300 => Self::Baud300,
            150 => Self::Baud150,
            110 => Self::Baud110,
            baud_rate => Self::Custom(baud_rate)
        }
    }
}

impl PartialOrd for BaudRate {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for BaudRate {
    fn cmp(&self, other: &Self) -> Ordering {
        self.to_integer().cmp(&other.to_integer())
    }
}

/// Configuration for [`Uart16550`].
///
/// Please note that sender and receiver **must agree** on the transmission
/// settings, otherwise you receive garbage.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Config {
    // Device Config
    /// Which interrupts to enable.
    pub interrupts: IER,

    // Transmission Config
    /// The baud rate to use.
    pub baud_rate: BaudRate,
    /// The length of each transmitted word.
    pub data_bits: WordLength,
    /// Whether extra stop bits should be used.
    ///
    /// See [`LCR::MORE_STOP_BITS`] for more info.
    pub extra_stop_bits: bool,
    /// Whether parity bits should be used.
    pub parity: Parity,

    // Other config
    /// Whether the driver should expect the remote to be a proper modem
    /// (terminal) to perform more checks.
    pub remote_modem_checks: bool,

}

impl Default for Config {
    fn default() -> Self {
        // Default is 8-N-1 connection and n data bits.
        Self {
            interrupts: IER::DATA_READY,

            baud_rate: Baud115200,
            data_bits: WordLength::EightBits,
            extra_stop_bits: false,
            parity: Parity::Disabled,

            remote_modem_checks: true,
        }
    }
}