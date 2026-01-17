// SPDX-License-Identifier: MIT OR Apache-2.0

//! # Constants, Register Offsets, and Register Bits.
//!
//! Models the raw low-level details as of the [datasheet], and avoids too
//! opinionated abstractions.
//!
//! [datasheet]: https://caro.su/msx/ocm_de1/16550.pdf

pub use crate::spec::errors::*;

/// Most typical 16550 clock frequency of 1.8432 Mhz.
pub const CLK_FREQUENCY_HZ: u32 = 1_843_200;

/// The maximum size of the internal read and write FIFO.
///
/// Each channel (tx: transmission, rx: reception) has its own queue.
pub const FIFO_SIZE: usize = 16;

mod errors {
    use core::error::Error;
    use core::fmt::{self, Display, Formatter};

    /// Error that is returned when [`calc_baud_rate`] could not calculate an even
    /// baud rate, i.e., a baud rate that is representable as integer.
    ///
    /// [`calc_baud_rate`]: crate::spec::calc_baud_rate
    #[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Hash)]
    pub struct NonIntegerBaudRateError {
        /// The frequency of the UART 16550.
        pub frequency: u32,
        /// The divisor.
        pub divisor: u32,
        /// The optional prescaler division factor.
        pub prescaler_division_factor: Option<u32>,
    }

    impl Display for NonIntegerBaudRateError {
        fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
            write!(
                f,
                "Input values do not result in even (integer representable) baud rate! frequency={}, divisor={}, prescaler_division_factor={:?}",
                self.frequency, self.divisor, self.prescaler_division_factor
            )
        }
    }

    impl Error for NonIntegerBaudRateError {}

    /// Error that is returned when [`calc_divisor`] could not calculate an even
    /// baud rate, i.e., a baud rate that is representable as integer.
    ///
    /// [`calc_divisor`]: crate::spec::calc_divisor
    #[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Hash)]
    pub struct NonIntegerDivisorError {
        /// The frequency of the UART 16550.
        pub frequency: u32,
        /// The divisor.
        pub baud_rate: u32,
        /// The optional prescaler division factor.
        pub prescaler_division_factor: Option<u32>,
    }

    impl Display for NonIntegerDivisorError {
        fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
            write!(
                f,
                "input values do not result in even (integer representable) baud rate! frequency={}, baud_rate={}, prescaler_division_factor={:?}",
                self.frequency, self.baud_rate, self.prescaler_division_factor,
            )
        }
    }

    impl Error for NonIntegerDivisorError {}
}

/// Calculates the baud rate from the frequency.
///
/// # Arguments
/// - `frequency`: The frequency of the microcontroller, typically
///   [`CLK_FREQUENCY_HZ`].
/// - `divisor`: The divisor to use.
/// - `prescaler_division_factor`: An optional additional division factor in
///   some more modern UART 16550 variants.
pub fn calc_baud_rate(
    frequency: u32,
    divisor: u32,
    prescaler_division_factor: Option<u32>,
) -> Result<u32, NonIntegerBaudRateError> {
    let psd = prescaler_division_factor.map_or(0, |psd| psd);
    let a = frequency;
    let b = 16 * (psd + 1) * divisor;

    if a % b == 0 {
        Ok(a / b)
    } else {
        Err(NonIntegerBaudRateError {
            frequency,
            prescaler_division_factor,
            divisor,
        })
    }
}

/// Similar to [`calc_baud_rate`] but with known baud rate to calculate the
/// frequency.
#[must_use]
pub const fn calc_frequency(
    baud_rate: u32,
    divisor: u32,
    prescaler_division_factor: Option<u32>,
) -> u32 {
    let psd = if let Some(psd) = prescaler_division_factor {
        psd
    } else {
        0
    };
    baud_rate * (16 * (psd + 1) * divisor)
}

/// Similar to [`calc_baud_rate`] but with known frequency to calculate the
/// divisor.
pub fn calc_divisor(
    frequency: u32,
    baud_rate: u32,
    prescaler_division_factor: Option<u32>,
) -> Result<u16, NonIntegerDivisorError> {
    calc_baud_rate(frequency, baud_rate, prescaler_division_factor)
        .map_err(|e| NonIntegerDivisorError {
            frequency: e.frequency,
            prescaler_division_factor: e.prescaler_division_factor,
            baud_rate,
        })
        // Unlikely but better be safe with an explicit panic.
        .map(|val| u16::try_from(val).unwrap())
}

/// Exposes low-level information about the on-chip register layout and provides
/// types that model individual registers.
///
/// The getters and setters in this module operate exclusively on raw bit
/// representations within the local computing context. They are limited to
/// extracting or updating the corresponding fields and do not perform direct
/// hardware access.
pub mod registers {
    use bitflags::bitflags;

    /// Provides the register offset from the base register.
    pub mod offsets {
        /// The maximum register offset, i.e., the amount of registers.
        ///
        /// This maximum index is therefore this value decremented by one.
        pub const MAX: usize = 8;

        /// For reads the Receiver Holding Register (RHR) and for writes the
        /// Transmitter Holding Register (THR), effectively acting as
        /// **data** register.
        pub const DATA: usize = 0;

        /// Interrupt Enable Register (IER).
        pub const IER: usize = 1;

        /// Interrupt Status Register (ISR).
        ///
        /// This register is used on **reads** from offset `2`.
        pub const ISR: usize = 2;

        /// FIFO Control Register (FSR).
        ///
        /// This register is used on **writes** to offset `2`.
        pub const FCR: usize = 2;

        /// Line Control Register (LCR).
        pub const LCR: usize = 3;

        /// Modem Control Register (MCR).
        pub const MCR: usize = 4;

        /// Line Status Register (LSR).
        pub const LSR: usize = 5;

        /// Modem Status Register (MSR).
        pub const MSR: usize = 6;

        /// Scratch Pad Register (SPR).
        pub const SPR: usize = 7;

        /* Registers accessible only when DLAB = 1 */

        /// Divisor Latch, Least significant byte (DLL).
        ///
        /// This is the low byte of the 16 bit divisor.
        pub const DLL: usize = 0;

        /// Divisor Latch, Most significant byte (DLL).
        ///
        /// This is the high byte of the 16 bit divisor.
        pub const DLM: usize = 1;

        /// Prescaler Division.
        ///
        /// This is a non-standard register (i.e., it is not present in the
        /// industry standard 16550 UART).
        pub const PSD: usize = 6;
    }

    /// Typing of the data register (RHR / THR).
    pub type DATA = u8;

    bitflags! {
        /// Typing of the Interrupt Enable Register (IER).
        ///
        /// This register individually enables each of the possible interrupt
        /// sources. A logic "1" in any of these bits enables the corresponding
        /// interrupt, while a logic "0" disables it.
        ///
        /// This is a **read/write** register.
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
        pub struct IER: u8 {
            /// Enables the data ready interrupt.
            ///
            /// This means data can be read (again).
            const DATA_READY = 1 << 0;
            /// Enables the THR Empty interrupt.
            ///
            /// This means data can be written (again).
            const THR_EMPTY = 1 << 1;
            /// Enables the Receiver Line Status interrupt.
            ///
            /// This means an error occurred: parity, framing, overrun.
            const RECEIVER_LINE_STATUS = 1 << 2;
            /// Enables the Modem Status interrupt.
            ///
            /// This tells you if the remote is ready for receive.
            const MODEM_STATUS = 1 << 3;
            /// Reserved.
            const _RESERVED0 = 1 << 4;
            /// Reserved.
            const _RESERVED1 = 1 << 5;
            /// Enables the non-standard interrupt issued when a DMA reception
            /// transfer is finished.
            const DMA_RX_END = 1 << 6;
            /// Enables the non-standard interrupt issued when a DMA
            /// transmission transfer is finished.
            const DMA_TX_END = 1 << 7;
        }
    }

    bitflags! {
        /// Typing of the Interrupt Status Register (ISR).
        ///
        /// **Read-only** register at offset [`offsets::ISR`] for identifying
        /// the interrupt with the highest priority that is currently pending.
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
        pub struct ISR: u8 {
            /// Indicates whether an interrupt is pending (0) or not
            /// (1). An interrupt is pending if this bit is cleared ('0').
            const INTERRUPT_STATUS = 1 << 0;
            /// Interrupt Identification Code (IIC, bit 0).
            const IIC_0 = 1 << 1;
            /// Interrupt Identification Code (IIC, bit 1).
            const IIC_1 = 1 << 2;
            /// Interrupt Identification Code (IIC, bit 2).
            const IIC_2 = 1 << 3;
            /// Reflects the state of the dmarx_end input pin which
            /// signals the end of a complete DMA transfer for received data.
            ///
            /// This is a non-standard flag that is enabled only if DMA End
            /// signaling has been enabled with bit 4 of FCR register. Otherwise
            /// it will always be read as '0'.
            const DMA_RX_END = 1 << 4;
            /// Reflects the state of the dmatx_end input pin which
            /// signals the end of a complete DMA transfer for transmitted data.
            /// This is a non-standard flag that is enabled only if DMA End
            /// signaling has been enabled with bit 4 of FCR register. Otherwise
            /// it will always be read as '0'.
            const DMA_TX_END = 1 << 5;
            /// Set if FIFOs are implemented and enabled (by setting FCR bit 0).
            ///
            /// Cleared in non-FIFO (16450) mode.
            const FIFOS_ENABLED0 = 1 << 6;
            /// Set if FIFOs are implemented and enabled (by setting FCR bit 0).
            ///
            /// Cleared in non-FIFO (16450) mode.
            const FIFOS_ENABLED1 = 1 << 7;
        }
    }

    impl ISR {
        /// Returns the matching [`InterruptType`], if there is an interrupt.
        ///
        /// The priority of the interrupt is available via
        /// [`InterruptType::priority`].
        #[must_use]
        pub fn interrupt_type(self) -> Option<InterruptType> {
            InterruptType::from_bits(self.bits())
        }
    }

    /// The possible interrupt types reported by the [`ISR`].
    ///
    ///
    /// This type is a convenient and non-ABI compatible abstraction.
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub enum InterruptType {
        /// There is an overrun error, parity error, framing error or break
        /// interrupt indication corresponding to the received data on top of
        /// the receiver's FIFO.
        ///
        /// Note that the FIFO error flag in LSR does not
        /// influence this interrupt, which is related only to the data on top
        /// of the Rx FIFO. This is directly related to the presence of a 1 in
        /// any of the LSR bits 1 to 4.
        ///
        /// **Interrupt reset method:**  Read the Line Status Register (LSR).
        ReceiverLineStatus,
        /// In non-FIFO mode, there is received data available in the RHR
        /// register.
        ///
        /// In FIFO-mode, the number of characters in the reception FIFO is
        /// equal or greater than the trigger level programmed in FCR. Note that
        /// this is not directly related to LSR bit 0, which always indicates
        /// that there is at least one word ready.
        ///
        /// **Interrupt reset method:** Read the Receiver Holding Register (RHR).
        ReceivedDataReady,
        /// There is at least one character in the receiver's FIFO and during a
        /// time corresponding to four characters at the selected baud rate no
        /// new character has been received and no reading has been executed on
        /// the receiver's FIFO.
        ///
        /// **Interrupt reset method:** Read the Receiver Holding Register (RHR).
        ReceptionTimeout,
        /// In non-FIFO mode, the 1-byte THR is empty. In FIFO mode, the
        /// complete 16-byte transmitter's FIFO is empty, so 1 to 16 characters
        /// can be written to THR.
        ///
        /// That is to say, THR Empty bit in LSR is one.
        ///
        /// **Interrupt reset method:** Write the data register. Alternatively,
        /// reading the Interrupt Status Register (ISR) will also clear the
        /// interrupt if this is the interrupt type being currently indicated
        /// (this will not clear the flag in the LSR).
        TransmitterHoldingRegisterEmpty,
        /// A change has been detected in the Clear To Send (CTS), Data Set
        /// Ready (DSR) or Carrier Detect (CD) input lines or a trailing edge
        /// in the Ring Indicator (RI) input line.
        ///
        /// That is to say, at least one of MSR bits 0 to 3 is one.
        ///
        /// **Interrupt reset method:** Read the Modem Status Register (MSR) .
        ModemStatus,
        /// A '1' has been detected in the dmarx_end input pin. This is supposed
        /// to imply the end of a complete DMA transfer for received data,
        /// executed by a DMA controller that provides this signal.
        ///
        /// **Interrupt reset method:** Read the Interrupt Status Register (ISR)
        /// (return of dmarx_end to zero does not reset the interrupt).
        DmaReceptionEndOfTransfer,
        /// A '1' has been detected in the dmatx_end input pin. This is supposed
        /// to imply the end of a complete DMA transfer for received data,
        /// executed by a DMA controller that provides this signal.
        ///
        /// **Interrupt reset method:** Read the Interrupt Status Register (ISR)
        /// (return of dmatx_end to zero does not reset the interrupt).
        DmaTransmissionEndOfTransfer,
    }

    impl InterruptType {
        /// Returns the priority level.
        ///
        /// Priority 1 is highest and 6 is lowest.
        ///
        /// The last two priority levels are not found in standard 16550 UART
        /// and may appear only if the DMA End signaling is enabled
        /// (bit 4 of FCR).
        #[must_use]
        pub const fn priority(self) -> u8 {
            match self {
                Self::ReceiverLineStatus => 1,
                Self::ReceivedDataReady => 2,
                Self::ReceptionTimeout => 2,
                Self::TransmitterHoldingRegisterEmpty => 3,
                Self::ModemStatus => 4,
                Self::DmaReceptionEndOfTransfer => 5,
                Self::DmaTransmissionEndOfTransfer => 6,
            }
        }

        /// Returns a [`InterruptType`] that corresponds to the bits in
        /// [`ISR`]
        #[must_use]
        pub fn from_bits(isr_bits: u8) -> Option<Self> {
            let bits = isr_bits & 0xf;

            let has_interrupt = (bits & 1) == 0;
            if !has_interrupt {
                return None;
            }
            let bits = bits >> 1;

            // Taken from the table on page 11/18 in <https://caro.su/msx/ocm_de1/16550.pdf>
            let typ = match bits {
                0b011 => Self::ReceiverLineStatus,
                0b010 => Self::ReceivedDataReady,
                0b110 => Self::ReceptionTimeout,
                0b001 => Self::TransmitterHoldingRegisterEmpty,
                0b000 => Self::ModemStatus,
                0b111 => Self::DmaReceptionEndOfTransfer,
                0b101 => Self::DmaTransmissionEndOfTransfer,
                _ => panic!("unexpected bit sequence: {bits:x}"),
            };

            Some(typ)
        }
    }

    bitflags! {
        /// Typing of the FIFO Control Register (FCR).
        ///
        /// **Write-only** register at offset [`offsets::FCR`] used to enable or
        /// disable FIFOs, clear receive/transmit FIFOs, and set the receiver
        /// trigger level.
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
        pub struct FCR: u8 {
            /// When set ('1') this bits enables both the transmitter and
            /// receiver FIFOs.
            ///
            /// In any writing to FCR, this bit must be set in order to affect
            /// the rest of the bits, except for bit 4. Changing this bit
            /// automatically resets both FIFOs.
            const FIFO_ENABLE = 1 << 0;
            /// Writing a one to this bit resets the receiver's FIFO (the
            /// pointers are reset and all the words are cleared).
            ///
            /// The Receiver Shift Register is not cleared, so any reception
            /// active will continue. The bit will automatically return to zero.
            const RX_FIFO_RESET = 1 << 1;
            /// Writing a one to this bit resets the transmitter's FIFO (the
            /// pointers are reset).
            ///
            /// The Transmitter Shift Register is not cleared, so any
            /// transmission active will continue. The bit will automatically
            /// return to zero.
            const TX_FIFO_RESET = 1 << 2;
            /// Selects the DMA mode. The DMA mode affects the way in
            /// which the DMA signaling outputs pins (txrdy, rxrdy and their
            /// inverted versions) behave.
            ///
            /// See the DMA signals explanation in the [datasheet] for details.
            ///
            /// [datasheet]: https://caro.su/msx/ocm_de1/16550.pdf
            ///
            /// Mode 0 is intended to transfer one character at a time. Mode 1
            /// is intended to transfer a set of characters at a time.
            ///
            /// # Recommendation
            /// This is typically not set in a kernel.
            const DMA_MODE = 1 << 3;
            /// Enables the DMA End signaling.
            ///
            /// This non-standard feature is useful when the UART is connected
            /// to a DMA controller which provides signals to indicate when a
            /// complete DMA transfer has been completed, either for reception
            /// or transmission (dmaend_rx and dmaend_tx input pins).
            const ENABLE_DMA_END = 1 << 4;
            /// Reserved.
            const _RESERVED0 = 1 << 5;
            /// First bit of [`FifoTriggerLevel`].
            const RX_FIFO_TRIGGER_LEVEL0 = 1 << 6;
            /// Second bit of [`FifoTriggerLevel`].
            const RX_FIFO_TRIGGER_LEVEL1 = 1 << 7;
        }
    }

    impl FCR {
        /// Returns the trigger level of the FIFO.
        #[must_use]
        pub const fn fifo_trigger_level(self) -> FifoTriggerLevel {
            let bits = (self.bits() >> 6) & 0b11;
            FifoTriggerLevel::from_raw_bits(bits)
        }

        /// Sets the trigger level of the FIFO.
        #[must_use]
        pub fn set_fifo_trigger_level(self, value: FifoTriggerLevel) -> Self {
            self | Self::from_bits_retain(value.to_raw_bits())
        }
    }

    /// The trigger level for the receiver's FIFO defined in [`FCR`].
    ///
    /// In FIFO mode an interrupt will be generated (if enabled) when the number
    /// of words in the receiver's FIFO is equal or greater than this trigger
    /// level.
    ///
    /// Besides, for FIFO mode operation a time out mechanism is implemented.
    /// Independently of the trigger level of the FIFO, an interrupt will be
    /// generated if there is at least one word in the FIFO and for a time
    /// equivalent to the transmission of four characters.
    ///
    /// This type is a convenient and non-ABI compatible abstraction. ABI
    /// compatibility is given via [`FifoTriggerLevel::from_raw_bits`] and
    /// [`FifoTriggerLevel::to_raw_bits`].
    #[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub enum FifoTriggerLevel {
        /// Interrupt is created after every character.
        One,
        /// Interrupt is created after every four characters.
        Four,
        /// Interrupt is created after every eight characters.
        Eight,
        /// Interrupt is created after every fourteen characters.
        ///
        /// # Recommendation
        /// This is the recommended default for best system performance.
        #[default]
        Fourteen,
    }

    impl FifoTriggerLevel {
        /// Translates the raw encoding into the corresponding value.
        ///
        /// This function operates on the value as-is and does not perform any
        /// shifting bits.
        #[must_use]
        pub const fn from_raw_bits(bits: u8) -> Self {
            let bits = bits & 0b11;
            match bits {
                0b00 => Self::One,
                0b01 => Self::Four,
                0b10 => Self::Eight,
                0b11 => Self::Fourteen,
                _ => unreachable!(),
            }
        }

        /// Translates the value into the corresponding raw encoding.
        ///
        /// This function operates on the value as-is and does not perform any
        /// shifting bits.
        #[must_use]
        pub const fn to_raw_bits(self) -> u8 {
            match self {
                Self::One => 0b00,
                Self::Four => 0b01,
                Self::Eight => 0b10,
                Self::Fourteen => 0b11,
            }
        }
    }

    bitflags! {
        /// Typing of the Line Control Register (LCR).
        ///
        /// Configures the serial frame format including word length, stop bits,
        /// parity, and controls access to the divisor latches via DLAB.
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
        pub struct LCR: u8 {
            /// First bit of [`WordLength`].
            const WORD_LENGTH0 = 1 << 0;
            /// Second bit of [`WordLength`].
            const WORD_LENGTH1 = 1 << 1;
            /// If cleared, only one stop bit will be transmitted. If set, two
            /// stop bits (1.5 with 5-bit data) will be transmitted before the
            /// start bit of the next character.
            const MORE_STOP_BITS = 1 << 2;
            /// Second bit of [`Parity`].
            const PARITY0 = 1 << 3;
            /// Second bit of [`Parity`].
            const PARITY1 = 1 << 4;
            /// Second bit of [`Parity`].
            const PARITY2 = 1 << 5;
            /// When this bit is set a break condition is forced in the
            /// transmission line. The serial output pin (txd) is forced to the
            /// spacing state (zero).
            ///
            /// When this bit is cleared, the break state is removed.
            ///
            /// # Recommendation
            /// Typically, this is not set in console/TTY use-cases.
            const SET_BREAK = 1 << 6;
            /// This is Divisor Latch Access Bit (DLAB).
            ///
            /// This bit **must** be set in order to access the [`DLL`],
            /// [`DLM`], and [`PSD`].
            const DLAB = 1 << 7;
        }
    }

    impl LCR {
        /// Returns the [`WordLength`].
        #[must_use]
        pub const fn word_length(self) -> WordLength {
            let bits = self.bits() & 0b11;
            WordLength::from_raw_bits(bits)
        }

        /// Sets the [`WordLength`].
        #[must_use]
        pub fn set_word_length(self, value: WordLength) -> Self {
            self | Self::from_bits_retain(value.to_raw_bits())
        }

        /// Returns the [`Parity`].
        #[must_use]
        pub const fn parity(self) -> Parity {
            let bits = (self.bits() >> 3) & 0b111;
            Parity::from_raw_bits(bits)
        }

        /// Sets the [`Parity`].
        #[must_use]
        pub fn set_parity(self, value: Parity) -> Self {
            self | Self::from_bits_retain(value.to_raw_bits())
        }
    }

    /// The length of words for the transmission and reception in [`LCR`].
    ///
    /// This type is a convenient and non-ABI compatible abstraction. ABI
    /// compatibility is given via [`WordLength::from_raw_bits`] and
    /// [`WordLength::to_raw_bits`].
    #[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub enum WordLength {
        /// Interrupt is created after every character.
        FiveBits,
        /// Interrupt is created after every four characters.
        SixBits,
        /// Interrupt is created after every eight characters.
        SevenBits,
        /// Interrupt is created after every fourteen characters.
        ///
        /// # Recommendation
        /// This is the recommended default.
        #[default]
        EightBits,
    }

    impl WordLength {
        /// Translates the raw encoding into the corresponding value.
        ///
        /// This function operates on the value as-is and does not perform any
        /// shifting bits.
        #[must_use]
        pub const fn from_raw_bits(bits: u8) -> Self {
            let bits = bits & 0b11;
            match bits {
                0b00 => Self::FiveBits,
                0b01 => Self::SixBits,
                0b10 => Self::SevenBits,
                0b11 => Self::EightBits,
                _ => unreachable!(),
            }
        }

        /// Translates the value into the corresponding raw encoding.
        ///
        /// This function operates on the value as-is and does not perform any
        /// shifting bits.
        #[must_use]
        pub const fn to_raw_bits(self) -> u8 {
            match self {
                Self::FiveBits => 0b00,
                Self::SixBits => 0b01,
                Self::SevenBits => 0b10,
                Self::EightBits => 0b11,
            }
        }
    }

    /// The length of words for the transmission as well as reception.
    ///
    /// This type is a convenient and non-ABI compatible abstraction. ABI
    /// compatibility is given via [`Parity::from_raw_bits`] and
    /// [`Parity::to_raw_bits`].
    #[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub enum Parity {
        /// No parity bit is transmitted nor expected.
        #[default]
        Disabled,
        /// The number of bits including the parity bit must be odd.
        Odd,
        /// The number of bits including the parity bit must be even.
        Even,
        /// The parity bit is sent as/checked to be `1`.
        Forced1,
        /// The parity bit is sent as/checked to be `0`.
        Forced0,
    }

    impl Parity {
        /// Translates the raw encoding into the corresponding value.
        ///
        /// This function operates on the value as-is and does not perform any
        /// shifting bits.
        #[must_use]
        pub const fn from_raw_bits(bits: u8) -> Self {
            let bits = bits & 0b111;
            let disabled = (bits & 1) == 0;
            if disabled {
                return Self::Disabled;
            }
            let bits = bits >> 1;
            match bits {
                0b00 => Self::Odd,
                0b01 => Self::Even,
                0b10 => Self::Forced1,
                0b11 => Self::Forced0,
                // We only have two bits left to check
                _ => unreachable!(),
            }
        }

        /// Translates the value into the corresponding raw encoding.
        ///
        /// This function operates on the value as-is and does not perform any
        /// shifting bits.
        #[must_use]
        pub const fn to_raw_bits(self) -> u8 {
            match self {
                Self::Disabled => 0b000,
                Self::Odd => 0b001,
                Self::Even => 0b011,
                Self::Forced1 => 0b101,
                Self::Forced0 => 0b111,
            }
        }
    }

    bitflags! {
        /// Typing of the Modem Control Register (MCR).
        ///
        /// Controls modem interface output signal.
        ///
        /// This is a **read/write** register.
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
        pub struct MCR: u8 {
            /// Controls the "data terminal ready" active low output (dtr_n).
            ///
            /// Signals the remote that the local UART device is powered on.
            ///
            /// A 1 in this bit makes dtr_n output a 0. When the bit is cleared,
            /// dtr_n outputs a 1.
            const DTR = 1 << 0;
            /// Controls the "request to send" active low output (rts_n) in the
            /// same way as bit 0 controls dtr_n.
            ///
            /// Signals the remote that the local UART is ready to receive data.
            const RTS = 1 << 1;
            /// Controls the general purpose, active low, output out1_n in the
            /// same way as bit 0 controls dtr_n.
            const OUT_1 = 1 << 2;
            /// Controls the general purpose, active low, output out2_n in
            /// the same way as bit 0 controls dtr_n.
            ///
            /// Besides, in typical x86 systems this acts as a global interrupt
            /// enable bit as it is connected to the systems interrupt
            /// controller. In this case, the complementary interrupt lines
            /// irq and irq_n will become active (1 and 0 respectively) only if
            /// this bit is 1 (and an interrupt condition is taken place).
            const OUT_2_INT_ENABLE = 1 << 3;
            /// Activate the loop back mode. Loop back mode is intended to test
            /// the UART communication.
            ///
            /// The serial output is connected internally to the serial input,
            /// so every character sent is looped back and received.
            const LOOP_BACK = 1 << 4;
            /// Reserved.
            const _RESERVED0 = 1 << 5;
            /// Reserved.
            const _RESERVED1 = 1 << 6;
            /// Reserved.
            const _RESERVED2 = 1 << 7;
        }
    }

    bitflags! {
        /// Typing of the Line Status Register (MCR).
        ///
        /// Reports the current status of the transmitter and receiver,
        ///  including data readiness, errors, and transmitter emptiness.
        ///
        /// This is a **read-only** register.
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
        pub struct LSR: u8 {
            /// It is set if one of more characters have been received and are
            /// waiting in the receiver's FIFO for the user to read them.
            ///
            /// It is zero if there is no available data in the receiver's FIFO.
            const DATA_READY = 1 << 0;
            /// Overrun Error flag. When it is set, a character has been
            /// completely assembled in the Receiver Shift Register without
            /// having free space to put it in the receiver's FIFO or holding
            /// register.
            ///
            /// When an overrun condition appears, the result is different
            /// depending on whether the 16-byte FIFO is active or not.
            const OVERRUN_ERROR = 1 << 1;
            ///  Parity Error flag. When it is set, it indicates that the parity
            /// of the received character is wrong according to the current
            /// setting in LCR.
            ///
            /// This bit is cleared as soon as the LSR is read.
            const PARITY_ERROR = 1 << 2;
            /// Framing Error flag. It indicates that the received character did
            /// not have a valid stop bit (i.e., a 0 was detected in the (first)
            /// stop bit position instead of a 1).
            ///
            /// This bit is cleared as soon as the LSR is read
            const FRAMING_ERROR = 1 << 3;
            /// Break Interrupt indicator. It is set to 1 if the receiver's line
            /// input rxd was held at zero for a complete character time.
            ///
            /// It is to say, the positions corresponding to the start bit, the
            /// data, the parity bit (if any) and the (first) stop bit were all
            /// detected as zeroes. Note that a Frame Error flag always
            /// accompanies this flag.
            ///
            /// This bit is cleared as soon as the LSR is read.
            const BREAK_INTERRUPT = 1 << 4;
            /// Transmit Holding Register Empty flag aka "ready to send".
            /// In non-FIFO mode, this bit is set whenever the 1-byte THR is
            /// empty.
            ///
            /// If the THR holds data to be transmitted, THR is immediately set
            /// when this data is passed to the TSR (Transmitter Shift Register).
            /// In FIFO mode, this bit is set when the transmitter's FIFO is
            /// completely empty, being 0 if there is at least one byte in the
            /// FIFO waiting to be passed to the TSR for transmission.
            ///
            /// This bit is cleared when the microprocessor writes new data in
            /// the THR (the data register).
            const THR_EMPTY = 1 << 5;
            /// Transmitter Empty flag. It is 1 when both the THR (or
            /// transmitter's FIFO) and the TSR are empty.
            ///
            /// Reading this bit as 1 means that no transmission is currently
            /// taking place in the txd output pin, the transmission line is
            /// idle.
            ///
            /// As soon as new data is written in the THR, this bit will be
            /// cleared.
            const TRANSMITTER_EMPTY = 1 << 6;
            /// This the FIFO data error bit. If the FIFO is not implemented or
            /// disabled (16450 mode), this bit is always zero.
            ///
            /// If the FIFO is active, this bit will be set as soon as any data
            /// character in the receiver's FIFO has parity or framing error or
            /// the break indication active.
            ///
            /// The bit is cleared when the microprocessor reads the LSR and the
            /// rest of the data in the receiver's FIFO do not have any of these
            /// three associated flags on.
            const FIFO_DATA_ERROR = 1 << 7;
        }
    }

    bitflags! {
        /// Typing of the Modem Status Register (MSR).
        ///
        /// Reflects the current state and change status of modem input.
        ///
        /// This is a **read-only** register.
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
        pub struct MSR: u8 {
            /// delta-CTS flag. If set, it means that the cts_n input has
            /// changed since the last time the microprocessor read this
            /// register.
            const DELTA_CTS = 1 << 0;
            /// delta-DSR flag. If set, it means that the dsr_n input has
            /// changed since the last time the microprocessor read this
            /// register.
            const DELTA_DSR = 1 << 1;
            /// Set when a trailing edge is detected in the ri_n input pin, it
            /// is to say, when ri_n changes from 0 to 1.
            const TRAILING_EDGE_RI = 1 << 2;
            /// delta-CD flag. If set, it means that the cd_n input has changed
            /// since the last time the microprocessor read this register.
            const DELTA_CD = 1 << 3;
            /// Clear To Send (CTS) is the complement of the cts_n input.
            ///
            /// This information comes from the remote side and tells if
            /// the remote can receive more data.
            const CTS = 1 << 4;
            /// Data Set Ready (DSR) is the complement of the dsr_n input.
            ///
            /// This information comes from the remote side and tells if
            /// the remote is powered on (hardware is present).
            const DSR = 1 << 5;
            /// Ring Indicator (RI) is the complement of the ri_n input.
            const RI = 1 << 6;
            /// Carrier Detect (CD) is the complement of the cd_n input.
            ///
            /// This information comes from the remote side and tells if
            /// the remote is actively messaged it is there.
            const CD = 1 << 7;
        }
    }

    /// Typing of the Scratch Pad Register (SPR).
    ///
    /// General-purpose read/write register with no defined hardware function,
    /// intended for software use or probing UART presence.
    ///
    /// This is a **read/write** register.
    pub type SPR = u8;

    /// Typing of the divisor latch register (low byte).
    ///
    /// Used to control the effective baud rate (see [`super::calc_baud_rate`]).
    ///
    /// This is a **read/write** register.
    pub type DLL = u8;

    /// Typing of the divisor latch register (high byte).
    ///
    /// Used to control the effective baud rate (see [`super::calc_baud_rate`]).
    ///
    /// This is a **read/write** register.
    pub type DLM = u8;

    /// All legal divisors for [`DLL`] and [`DLM`] that can create a valid and
    /// even baud rate using [`super::calc_baud_rate`].
    #[allow(missing_docs)]
    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
    #[repr(u16)]
    pub enum Divisor {
        #[default] // => baud rate 115200
        Divisor1,
        Divisor2,
        Divisor3,
        Divisor4,
        Divisor5,
        Divisor6,
        Divisor8,
        Divisor9,
        Divisor10,
        Divisor12,
        Divisor15,
        Divisor16,
        Divisor18,
        Divisor20,
        Divisor24,
        Divisor25,
        Divisor30,
        Divisor32,
        Divisor36,
        Divisor40,
        Divisor45,
        Divisor48,
        Divisor50,
        Divisor60,
        Divisor64,
        Divisor72,
        Divisor75,
        Divisor80,
        Divisor90,
        Divisor96,
        Divisor100,
        Divisor120,
        Divisor128,
        Divisor144,
        Divisor150,
        Divisor160,
        Divisor180,
        Divisor192,
        Divisor200,
        Divisor225,
        Divisor240,
        Divisor256,
        Divisor288,
        Divisor300,
        Divisor320,
        Divisor360,
        Divisor384,
        Divisor400,
        Divisor450,
        Divisor480,
        Divisor512,
        Divisor576,
        Divisor600,
        Divisor640,
        Divisor720,
        Divisor768,
        Divisor800,
        Divisor900,
        Divisor960,
        Divisor1152,
        Divisor1200,
        Divisor1280,
        Divisor1440,
        Divisor1536,
        Divisor1600,
        Divisor1800,
        Divisor1920,
        Divisor2304,
        Divisor2400,
        Divisor2560,
        Divisor2880,
        Divisor3200,
        Divisor3600,
        Divisor3840,
        Divisor4608,
        Divisor4800,
        Divisor5760,
        Divisor6400,
        Divisor7200,
        Divisor7680,
        Divisor9600,
        Divisor11520,
        Divisor12800,
        Divisor14400,
        Divisor19200,
        Divisor23040,
        Divisor28800,
        Divisor38400,
        Divisor57600,
    }

    bitflags! {
        /// Typing of the Prescaler Division (PSD) register.
        ///
        /// This is a non-standard register (i.e., it is not present in the
        /// industry standard 16550 UART). Its purpose is to provide a second
        /// division factor that could be useful in systems which are driven by
        /// a clock multiple of one of the typical frequencies used with this
        /// UART.
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
        pub struct PSD: u8 {
            /// Prescaler's Division Factor (bit 0).
            const PDF0 = 1 << 0;
            /// Prescaler's Division Factor (bit 1).
            const PDF1 = 1 << 1;
            /// Prescaler's Division Factor (bit 2).
            const PDF2 = 1 << 2;
            /// Prescaler's Division Factor (bit 3).
            const PDF3 = 1 << 3;
            /// Reserved.
            const _RESERVED0 = 1 << 4;
            /// Reserved.
            const _RESERVED1 = 1 << 5;
            /// Reserved.
            const _RESERVED2 = 1 << 6;
            /// Reserved.
            const _RESERVED3 = 1 << 7;
        }
    }

    impl PSD {
        /// Returns the Prescaler's Division Factor (PDF).
        #[must_use]
        pub const fn pdf(self) -> u8 {
            self.bits() & 0xf
        }

        /// Sets the Prescaler's Division Factor (PDF).
        #[must_use]
        pub const fn set_pdf(self, pdf: u8) -> Self {
            Self::from_bits_retain(pdf)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calc_baud_rate() {
        assert_eq!(calc_baud_rate(CLK_FREQUENCY_HZ, 1, None), Ok(115200));
        assert_eq!(calc_baud_rate(CLK_FREQUENCY_HZ, 2, None), Ok(57600));
        assert_eq!(calc_baud_rate(CLK_FREQUENCY_HZ, 3, None), Ok(38400));
        assert_eq!(calc_baud_rate(CLK_FREQUENCY_HZ, 6, None), Ok(19200));
        assert_eq!(calc_baud_rate(CLK_FREQUENCY_HZ, 9, None), Ok(12800));
        assert_eq!(calc_baud_rate(CLK_FREQUENCY_HZ, 12, None), Ok(9600));
        assert_eq!(calc_baud_rate(CLK_FREQUENCY_HZ, 15, None), Ok(7680));
        assert_eq!(calc_baud_rate(CLK_FREQUENCY_HZ, 16, None), Ok(7200));
        assert_eq!(
            calc_baud_rate(CLK_FREQUENCY_HZ, 73, None),
            Err(NonIntegerBaudRateError {
                frequency: CLK_FREQUENCY_HZ,
                divisor: 73,
                prescaler_division_factor: None,
            })
        );
    }

    #[test]
    fn test_calc_frequency() {
        assert_eq!(calc_frequency(115200, 1, None), CLK_FREQUENCY_HZ);
        assert_eq!(calc_frequency(57600, 2, None), CLK_FREQUENCY_HZ);
        assert_eq!(calc_frequency(38400, 3, None), CLK_FREQUENCY_HZ);
        assert_eq!(calc_frequency(19200, 6, None), CLK_FREQUENCY_HZ);
        assert_eq!(calc_frequency(12800, 9, None), CLK_FREQUENCY_HZ);
        assert_eq!(calc_frequency(9600, 12, None), CLK_FREQUENCY_HZ);
        assert_eq!(calc_frequency(7680, 15, None), CLK_FREQUENCY_HZ);
        assert_eq!(calc_frequency(7200, 16, None), CLK_FREQUENCY_HZ);
    }

    #[test]
    fn test_calc_divisor() {
        assert_eq!(calc_divisor(CLK_FREQUENCY_HZ, 115200, None), Ok(1));
        assert_eq!(calc_divisor(CLK_FREQUENCY_HZ, 57600, None,), Ok(2));
        assert_eq!(calc_divisor(CLK_FREQUENCY_HZ, 38400, None,), Ok(3));
        assert_eq!(calc_divisor(CLK_FREQUENCY_HZ, 19200, None,), Ok(6));
        assert_eq!(calc_divisor(CLK_FREQUENCY_HZ, 12800, None,), Ok(9));
        assert_eq!(calc_divisor(CLK_FREQUENCY_HZ, 9600, None,), Ok(12));
        assert_eq!(calc_divisor(CLK_FREQUENCY_HZ, 7680, None,), Ok(15));
        assert_eq!(calc_divisor(CLK_FREQUENCY_HZ, 7200, None,), Ok(16));
        assert_eq!(
            calc_divisor(CLK_FREQUENCY_HZ, 7211, None),
            Err(NonIntegerDivisorError {
                frequency: CLK_FREQUENCY_HZ,
                baud_rate: 7211,
                prescaler_division_factor: None,
            })
        );
    }
}
