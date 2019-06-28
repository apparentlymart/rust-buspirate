use crate::low;
use crate::BusPirate;
use crate::Error;
use embedded_hal::serial;

pub struct SPI<TX: serial::Write<u8>, RX: serial::Read<u8>> {
    pub(crate) ch: low::Channel<TX, RX>,
}

impl<TX, RX, TXErr, RXErr> SPI<TX, RX>
where
    TX: serial::Write<u8, Error = TXErr>,
    RX: serial::Read<u8, Error = RXErr>,
{
    pub fn close(self) -> Result<BusPirate<TX, RX>, Error<TXErr, RXErr>> {
        crate::close_handshake(self.ch)
    }

    pub fn to_hiz(self) -> Result<crate::hiz::HiZ<TX, RX>, Error<TXErr, RXErr>> {
        crate::binary_reset_handshake(self.ch)
    }

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

    pub fn set_config(&mut self, config: Config) -> Result<(), Error<TXErr, RXErr>> {
        self.ch.simple_command(config.command_byte())
    }

    pub fn configure_peripherals(
        &mut self,
        config: crate::peripherals::Config,
    ) -> Result<(), Error<TXErr, RXErr>> {
        self.ch.simple_command(config.command_byte())
    }

    pub fn chip_select(&mut self, active: bool) -> Result<(), Error<TXErr, RXErr>> {
        // Chip select is active low, so active = true means CS is low.
        self.ch
            .simple_command(if active { 0b00000010 } else { 0b00000011 })
    }

    pub fn transfer_byte(&mut self, v: u8) -> Result<u8, Error<TXErr, RXErr>> {
        self.ch.write(0b00010000)?;
        self.ch.write(v)?;
        self.ch.flush()?;
        match self.ch.read()? {
            0x01 => self.ch.read(),
            _ => Err(Error::<TXErr, RXErr>::Protocol),
        }
    }

    pub fn transfer_bytes<'w>(&mut self, v: &'w mut [u8]) -> Result<&'w [u8], Error<TXErr, RXErr>> {
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

pub enum PinOutput {
    PinOutputHiZ,
    PinOutput3_3V,
}

pub enum ClockPhase {
    ClockPhaseHigh,
    ClockPhaseLow,
}

pub enum ClockEdge {
    ClockEdgeFalling,
    ClockEdgeRising,
}

pub enum SampleTime {
    SampleTimeMiddle,
    SampleTimeEnd,
}

pub struct Config {
    pub pin_output: PinOutput,
    pub clock_idle_phase: ClockPhase,
    pub clock_edge: ClockEdge,
    pub sample_time: SampleTime,
}

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

pub trait Comms {
    type Error;

    fn transfer<'w>(&mut self, v: &'w mut [u8]) -> Result<&'w [u8], Self::Error>;
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
