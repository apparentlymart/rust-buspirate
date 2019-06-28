//! Module `spi` contains the API for SPI mode.
//!
//! This mode cannot be entered directly. Instead, create a `BusPirate` object
//! (from the root module of this crate) and call `to_bitbang` on it to enter
//! "binary bit-bang" mode, and then call `to_spi` to enter SPI mode:
//!
//! ```rust
//! let bp = BusPirate::new();
//! let bb = bp.to_bitbang()?;
//! let bspi = bb.to_spi()?;
//! ```
//!
//! The result of `to_spi` is an instance of `SPI`.

use crate::low;
use crate::BusPirate;
use crate::Error;
use embedded_hal::serial;

/// `SPI` represents a Bus Pirate device in SPI mode.
pub struct SPI<TX: serial::Write<u8>, RX: serial::Read<u8>> {
    pub(crate) ch: low::Channel<TX, RX>,
}

impl<TX, RX, TXErr, RXErr> SPI<TX, RX>
where
    TX: serial::Write<u8, Error = TXErr>,
    RX: serial::Read<u8, Error = RXErr>,
{
    /// `close` resets the Bus Pirate back into normal terminal mode, exiting
    /// SPI mode.
    pub fn close(self) -> Result<BusPirate<TX, RX>, Error<TXErr, RXErr>> {
        crate::close_handshake(self.ch)
    }

    /// `to_bitbang` switches back to "binary bit-bang" mode.
    pub fn to_bitbang(self) -> Result<crate::bitbang::BitBang<TX, RX>, Error<TXErr, RXErr>> {
        crate::binary_reset_handshake(self.ch)
    }

    /// `set_speed` changes the SPI clock rate for subsequent transactions.
    pub fn set_speed(&mut self, speed: Speed) -> Result<(), Error<TXErr, RXErr>> {
        use Speed::*;
        let bits = match speed {
            Speed30KHz => 0b000,
            Speed125KHz => 0b001,
            Speed250KHz => 0b010,
            Speed1MHz => 0b011,
            Speed2MHz => 0b100,
            Speed2_6MHz => 0b101,
            Speed4MHz => 0b110,
            Speed8MHz => 0b111,
        } as u8;
        self.ch.simple_command(0b01000000 | bits)
    }

    /// `set_config` changes some SPI-mode-specific configuration settings.
    pub fn set_config(&mut self, config: Config) -> Result<(), Error<TXErr, RXErr>> {
        self.ch.simple_command(config.command_byte())
    }

    /// `configure_peripherals` changes some settings related to general
    /// peripherals that can be used alongside SPI mode.
    pub fn configure_peripherals(
        &mut self,
        config: crate::peripherals::Config,
    ) -> Result<(), Error<TXErr, RXErr>> {
        self.ch.simple_command(config.command_byte())
    }

    /// `chip_select` sets the state of the chip select signal.
    ///
    /// Chip select is an inverted signal, so passing `true` will cause the
    /// electrical signal to move low, and `false` will cause the electrical
    /// signal to move high.
    pub fn chip_select(&mut self, active: bool) -> Result<(), Error<TXErr, RXErr>> {
        self.ch
            .simple_command(if active { 0b00000010 } else { 0b00000011 })
    }

    /// `transfer_byte` performs a single-byte SPI transfer.
    ///
    /// An SPI transfer receeives one bit in for every bit transmitted, so the
    /// result is the byte receieved from the connected device.
    ///
    /// To transmit a single byte without receiving any result, simply disregard
    /// the resulting byte. It is not possible to receive data without
    /// transmitting, but a common convention for recieving data only is to
    /// transmit zero.
    pub fn transfer_byte(&mut self, v: u8) -> Result<u8, Error<TXErr, RXErr>> {
        self.ch.write(0b00010000)?;
        self.ch.write(v)?;
        self.ch.flush()?;
        match self.ch.read()? {
            0x01 => self.ch.read(),
            _ => Err(Error::<TXErr, RXErr>::Protocol),
        }
    }

    /// `transfer_bytes` performs a multi-byte SPI transfer.
    ///
    /// An SPI transfer receeives one bit in for every bit transmitted, so the
    /// result gives the bytes receieved from the connected device. The
    /// backing array of the source slice is overwritten in-place, so the
    /// resulting slice refers to the same bytes as the argument.
    ///
    /// A maximum of 16 bytes can be transmitted per call. If a longer slice
    /// is given, the `Request` error is returned.
    pub fn transfer_bytes<'w>(&mut self, v: &'w mut [u8]) -> Result<&'w [u8], Error<TXErr, RXErr>> {
        if v.len() == 0 {
            return Ok(v); // Nothing to do, then.
        }
        if v.len() > 16 {
            return Err(Error::Request); // Too many bytes to send
        }

        let len = v.len() as u8;
        let cmd = (0b00010000 as u8) | (len - 1);
        self.ch.write(cmd)?;
        for i in 0..v.len() {
            self.ch.write(v[i])?;
        }
        self.ch.flush()?;

        match self.ch.read()? {
            0x01 => (),
            _ => return Err(Error::<TXErr, RXErr>::Protocol),
        }

        for i in 0..v.len() {
            v[i] = self.ch.read()?;
        }

        Ok(v)
    }

    /// `write_then_read` transmits zero or more bytes and then receives zero
    /// or more bytes, optionally setting the chip select signal active
    /// throughout.
    ///
    /// A common pattern with SPI devices is to transmit some sort of request
    /// (a register address, for example) and then transmit sufficient zero
    /// bits to receieve the device's response to that request. `write_then_read`
    /// implements that common pattern in an efficient way that allows the
    /// Bus Pirate to be in full control of the timing, and thus not be stalled
    /// by delays caused by waiting for further requests from its host.
    ///
    /// A maximum of 4096 bytes can be transmitted and recieved by this function.
    /// If either slice is greater than 4096 characters then the `Request`
    /// error is returned.
    pub fn write_then_read<'w>(
        &mut self,
        write_from: &[u8],
        read_into: &mut [u8],
        cs: bool,
    ) -> Result<(), Error<TXErr, RXErr>> {
        if write_from.len() > 4096 {
            return Err(Error::Request); // Too many bytes to send
        }
        if read_into.len() > 4096 {
            return Err(Error::Request); // Too many bytes to read
        }

        let wr_len = write_from.len() as u16;
        let rd_len = read_into.len() as u16;
        self.ch.write(if cs { 0b00000100 } else { 0b00000101 })?;
        self.ch.write((wr_len >> 8) as u8)?; // MSB of length to write
        self.ch.write(wr_len as u8)?; // LSB of length to write
        self.ch.write((rd_len >> 8) as u8)?; // MSB of length to read
        self.ch.write(rd_len as u8)?; // LSB of length to read

        for c in write_from {
            self.ch.write(*c)?;
        }

        match self.ch.read()? {
            0x01 => (),
            _ => return Err(Error::<TXErr, RXErr>::Protocol),
        }

        for i in 0..read_into.len() {
            read_into[i] = self.ch.read()?;
        }

        Ok(())
    }
}

/// `Speed` describes a clock speed to be used for Bus Pirate SPI data transfers.
pub enum Speed {
    Speed30KHz,
    Speed125KHz,
    Speed250KHz,
    Speed1MHz,
    Speed2MHz,
    Speed2_6MHz,
    Speed4MHz,
    Speed8MHz,
}

/// `PinOutput` describes an output mode to be used for Bus Pirate SPI data
/// transfers.
pub enum PinOutput {
    // `PinOutputHiZ` requests that the Bus Pirate set its outputs to a high
    // impedance state when signalling "active".
    PinOutputHiZ,
    // `PinOutputHiZ` requests that the Bus Pirate drive its outputs to 3.3V
    // when signalling "active".
    PinOutput3_3V,
}

/// `ClockPhase` describes a single phase of an SPI transmission clock cycle.
pub enum ClockPhase {
    ClockPhaseHigh,
    ClockPhaseLow,
}

/// `ClockEdge` describes a single transition edge of an SPI transmission clock cycle.
pub enum ClockEdge {
    ClockEdgeFalling,
    ClockEdgeRising,
}

/// `SampleTime` describes a point within an SPI transmission where data bits
/// are to be sampled.
pub enum SampleTime {
    SampleTimeMiddle,
    SampleTimeEnd,
}

/// `Config` describes SPI-specific Bus Pirate settings.
pub struct Config {
    pub pin_output: PinOutput,
    pub clock_idle_phase: ClockPhase,
    pub clock_edge: ClockEdge,
    pub sample_time: SampleTime,
}

/// `DEFAULT_CONFIG` is the initial state of SPI configuration when a Bus Pirate
/// is first started, according to the Bus Pirate documentation.
pub const DEFAULT_CONFIG: Config = Config {
    pin_output: PinOutput::PinOutputHiZ,
    clock_idle_phase: ClockPhase::ClockPhaseLow,
    clock_edge: ClockEdge::ClockEdgeFalling,
    sample_time: SampleTime::SampleTimeMiddle,
};

impl Config {
    pub(crate) fn command_byte(&self) -> u8 {
        let mut cmd = 0b10000000 as u8;
        cmd = cmd
            | (match self.pin_output {
                PinOutput::PinOutputHiZ => 0,
                PinOutput::PinOutput3_3V => 1,
            } << 3);
        cmd = cmd
            | (match self.clock_idle_phase {
                ClockPhase::ClockPhaseLow => 0,
                ClockPhase::ClockPhaseHigh => 1,
            } << 2);
        cmd = cmd
            | (match self.clock_edge {
                ClockEdge::ClockEdgeRising => 0,
                ClockEdge::ClockEdgeFalling => 1,
            } << 1);
        cmd = cmd
            | (match self.sample_time {
                SampleTime::SampleTimeMiddle => 0,
                SampleTime::SampleTimeEnd => 1,
            } << 0);
        cmd
    }
}

/// `Comms` is a trait implemented by `SPI` representing the main communications
/// actions it supports, while excluding the configuration actions.
pub trait Comms {
    type Error;

    /// `transfer` performs a single SPI transfer, transmitting each of the bits
    /// in the given slice and then overwriting them with the bits receieved
    /// from the target device during each clock cycle.
    ///
    /// `transfer` has no limit on the number of bytes it can transmit, but
    /// may need to send multiple separate transmission commands to the Bus
    /// Pirate to represent the full message. The clock rate for larger
    /// transmissions will therefore be irregular. For devices with sensitive
    /// timing requirements, consider `transaction` instead.
    fn transfer<'w>(&mut self, v: &'w mut [u8]) -> Result<&'w [u8], Self::Error>;

    /// `transaction` sends up to 4096 bytes of data and then receieves up to
    /// 4096 bytes ot data, optionally activating the chip select signal for
    /// the duration of the operation.
    ///
    /// If either given slice is longer than 4096 elements in length,
    /// `transaction` returns the `Request` error.
    fn transaction<'w>(
        &mut self,
        write_from: &'w [u8],
        read_into: &'w mut [u8],
        cs: bool,
    ) -> Result<(), Self::Error>;
}

impl<TX, RX, TXErr, RXErr> Comms for SPI<TX, RX>
where
    TX: embedded_hal::serial::Write<u8, Error = TXErr>,
    RX: embedded_hal::serial::Read<u8, Error = RXErr>,
{
    type Error = crate::Error<TXErr, RXErr>;

    fn transfer<'w>(&mut self, v: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        let mut remain = &mut v[..];
        while remain.len() > 0 {
            let len = if remain.len() > 16 { 16 } else { remain.len() };
            let (next, after) = remain.split_at_mut(len);
            self.transfer_bytes(next)?; // This overwrites elements of v in-place.
            remain = after;
        }
        Ok(v)
    }

    fn transaction<'w>(
        &mut self,
        write_from: &'w [u8],
        read_into: &'w mut [u8],
        cs: bool,
    ) -> Result<(), Self::Error> {
        self.write_then_read(write_from, read_into, cs)
    }
}
