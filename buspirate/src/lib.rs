#![no_std]

pub mod peripherals;
pub mod spi;

use core::marker::PhantomData;
use embedded_hal::serial;

const PROTO_VERSION_MSG: [u8; 5] = ['B' as u8, 'B' as u8, 'I' as u8, 'O' as u8, '1' as u8];
const PROTO_SPI_VERSION_MSG: [u8; 4] = ['S' as u8, 'P' as u8, 'I' as u8, '1' as u8];

#[derive(Debug, Clone)]
pub struct BusPirate<TX: serial::Write<u8>, RX: serial::Read<u8>> {
    tx: TX,
    rx: RX,
}

impl<TX, RX, TXErr, RXErr> BusPirate<TX, RX>
where
    TX: serial::Write<u8, Error = TXErr>,
    RX: serial::Read<u8, Error = RXErr>,
{
    pub fn new(tx: TX, rx: RX) -> Self {
        Self { tx: tx, rx: rx }
    }

    pub fn init(mut self) -> Result<Open<HiZ, TX, RX>, Error<TXErr, RXErr>> {
        // The Bus Pirate could be in any mode when we find it, so
        // we follow the advice given in the protocol documentation:
        // - Send newline 10 times to escape from any menu/prompts in progress
        // - Send '#' to reset
        // - Send nul (0x00) 20 times to enter binary protocol mode

        for _ in 0..10 {
            nb::block!(self.tx.write(0x10)).map_err(Error::tx)?;
        }
        nb::block!(self.tx.write('#' as u8)).map_err(Error::tx)?;
        nb::block!(self.tx.write(0x10)).map_err(Error::tx)?;
        nb::block!(self.tx.flush()).map_err(Error::tx)?;

        // Before we go any further, we'll read out anything that's in the
        // receive buffer. If the Bus Pirate is behaving as expected then
        // its initialization messages and "HiZ>" prompt will be there.
        eat_rx_buffer(&mut self.tx, &mut self.rx)?;

        binary_reset_handshake(self.tx, self.rx)
    }

    pub fn release(self) -> (TX, RX) {
        (self.tx, self.rx)
    }
}

pub struct HiZ; // HiZ Mode
pub struct SPI; // SPI Mode

pub trait Mode {}
impl Mode for HiZ {}
impl Mode for SPI {}

#[derive(Debug, Clone)]
pub struct Open<MODE: Mode, TX: serial::Write<u8>, RX: serial::Read<u8>> {
    tx: TX,
    rx: RX,
    mode: PhantomData<MODE>,
}

impl<MODE, TX, RX, TXErr, RXErr> Open<MODE, TX, RX>
where
    MODE: Mode,
    TX: serial::Write<u8, Error = TXErr>,
    RX: serial::Read<u8, Error = RXErr>,
{
    pub fn close(mut self) -> Result<BusPirate<TX, RX>, Error<TXErr, RXErr>> {
        // We'll reset the Bus Pirate back into user terminal mode before
        // we release the serial port objects.
        self.send_close()?;

        Ok(BusPirate {
            tx: self.tx,
            rx: self.rx,
        })
    }

    fn send_close(&mut self) -> Result<(), Error<TXErr, RXErr>> {
        nb::block!(self.tx.write(0b00001111)).map_err(Error::tx)?;
        Ok(())
    }

    fn simple_cmd_blocking(&mut self, cmd: u8) -> Result<(), Error<TXErr, RXErr>> {
        nb::block!(self.tx.write(cmd)).map_err(Error::tx)?;
        nb::block!(self.tx.flush()).map_err(Error::tx)?;

        match nb::block!(self.rx.read()).map_err(Error::rx)? {
            0x01 => Ok(()),
            _ => Err(Error::Protocol),
        }
    }
}

impl<TX, RX, TXErr, RXErr> Open<HiZ, TX, RX>
where
    TX: serial::Write<u8, Error = TXErr>,
    RX: serial::Read<u8, Error = RXErr>,
{
    pub fn to_spi(self) -> Result<Open<SPI, TX, RX>, Error<TXErr, RXErr>> {
        binary_mode_handshake::<SPI, TX, RX>(self.tx, self.rx, 0b00000001, &PROTO_SPI_VERSION_MSG)
    }
}

impl<TX, RX, TXErr, RXErr> Open<SPI, TX, RX>
where
    TX: serial::Write<u8, Error = TXErr>,
    RX: serial::Read<u8, Error = RXErr>,
{
    pub fn exit(self) -> Result<Open<HiZ, TX, RX>, Error<TXErr, RXErr>> {
        binary_reset_handshake(self.tx, self.rx)
    }

    pub fn set_speed(&mut self, speed: spi::Speed) -> Result<(), Error<TXErr, RXErr>> {
        use spi::Speed::*;
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
        self.simple_cmd_blocking(0b01000000 | bits)
    }

    pub fn set_config(&mut self, config: spi::Config) -> Result<(), Error<TXErr, RXErr>> {
        self.simple_cmd_blocking(config.command_byte())
    }

    pub fn configure_peripherals(
        &mut self,
        config: peripherals::Config,
    ) -> Result<(), Error<TXErr, RXErr>> {
        self.simple_cmd_blocking(config.command_byte())
    }

    pub fn chip_select(&mut self, active: bool) -> Result<(), Error<TXErr, RXErr>> {
        // Chip select is active low, so active = true means CS is low.
        self.simple_cmd_blocking(if active { 0b00000010 } else { 0b00000011 })
    }

    pub fn transfer_byte(&mut self, v: u8) -> Result<u8, Error<TXErr, RXErr>> {
        nb::block!(self.tx.write(0b00010000)).map_err(Error::tx)?;
        nb::block!(self.tx.write(v)).map_err(Error::tx)?;
        nb::block!(self.tx.flush()).map_err(Error::tx)?;
        match nb::block!(self.rx.read()).map_err(Error::rx)? {
            0x01 => nb::block!(self.rx.read()).map_err(Error::rx),
            _ => Err(Error::<TXErr, RXErr>::Protocol),
        }
    }

    pub fn transfer_bytes<'w>(&mut self, v: &'w mut [u8]) -> Result<&'w [u8], Error<TXErr, RXErr>> {
        if v.len() > 16 {
            return Err(Error::Request); // Too many bytes to send
        }

        let len = v.len() as u8;
        let cmd = (0b00010000 as u8) | (len - 1);
        nb::block!(self.tx.write(cmd)).map_err(Error::tx)?;
        for i in 0..v.len() {
            nb::block!(self.tx.write(v[i])).map_err(Error::tx)?;
        }
        nb::block!(self.tx.flush()).map_err(Error::tx)?;

        match nb::block!(self.rx.read()).map_err(Error::rx)? {
            0x01 => (),
            _ => return Err(Error::<TXErr, RXErr>::Protocol),
        }

        for i in 0..v.len() {
            v[i] = nb::block!(self.rx.read()).map_err(Error::rx)?;
        }

        Ok(v)
    }

    pub fn transfer_bytes_buffered<'w>(
        &mut self,
        v: &'w mut [u8],
        want: usize,
        cs: bool,
    ) -> Result<&'w [u8], Error<TXErr, RXErr>> {
        if v.len() > 4096 {
            return Err(Error::Request); // Too many bytes to send
        }
        if want > v.len() {
            return Err(Error::Request); // Can't read more than we're writing
        }

        let len = v.len() as u16;
        nb::block!(self.tx.write(if cs { 0b00000100 } else { 0b00000101 })).map_err(Error::tx)?;
        nb::block!(self.tx.write((len >> 8) as u8)).map_err(Error::tx)?; // MSB of length to write
        nb::block!(self.tx.write(len as u8)).map_err(Error::tx)?; // LSB of length to write
        nb::block!(self.tx.write((want >> 8) as u8)).map_err(Error::tx)?; // MSB of length to read
        nb::block!(self.tx.write(want as u8)).map_err(Error::tx)?; // LSB of length to read
        for i in 0..v.len() {
            nb::block!(self.tx.write(v[i])).map_err(Error::tx)?;
        }

        match nb::block!(self.rx.read()).map_err(Error::rx)? {
            0x01 => (),
            _ => return Err(Error::<TXErr, RXErr>::Protocol),
        }

        for i in 0..want {
            v[i] = nb::block!(self.rx.read()).map_err(Error::rx)?;
        }

        Ok(&v[0..want])
    }
}

#[derive(Debug)]
pub enum Error<TXErr, RXErr> {
    Protocol,
    Request,
    Write(TXErr),
    Read(RXErr),
}

impl<TXErr, RXErr> Error<TXErr, RXErr> {
    fn tx(got: TXErr) -> Self {
        Error::Write(got)
    }

    fn rx(got: RXErr) -> Self {
        Error::Read(got)
    }
}

fn binary_mode_handshake<MODE: Mode, TX: serial::Write<u8>, RX: serial::Read<u8>>(
    mut tx: TX,
    mut rx: RX,
    send: u8,
    expect: &'static [u8; 4],
) -> Result<Open<MODE, TX, RX>, Error<TX::Error, RX::Error>> {
    let mut ok = false;
    'tries: for _ in 0..10 {
        nb::block!(tx.flush()).map_err(Error::tx)?;
        nb::block!(tx.write(send)).map_err(Error::tx)?;

        let mut correct = 0;
        loop {
            match rx.read() {
                Ok(c) => {
                    if c != expect[correct] {
                        correct = 0;
                    }
                    if c == expect[correct] {
                        correct += 1;
                        if correct == expect.len() {
                            ok = true;
                            break 'tries;
                        }
                    }
                }
                Err(e) => match e {
                    nb::Error::WouldBlock => continue 'tries,
                    nb::Error::Other(e) => return Err(Error::rx(e)),
                },
            }
        }
    }

    if !ok {
        return Err(Error::Protocol);
    }

    eat_rx_buffer(&mut tx, &mut rx)?;

    Ok(Open {
        tx: tx,
        rx: rx,
        mode: PhantomData,
    })
}

fn binary_reset_handshake<TX: serial::Write<u8>, RX: serial::Read<u8>>(
    mut tx: TX,
    mut rx: RX,
) -> Result<Open<HiZ, TX, RX>, Error<TX::Error, RX::Error>> {
    let mut ok = false;
    'tries: for _ in 0..20 {
        nb::block!(tx.flush()).map_err(Error::tx)?;
        nb::block!(tx.write(0x00)).map_err(Error::tx)?;

        let mut correct = 0;
        loop {
            match rx.read() {
                Ok(c) => {
                    if c != PROTO_VERSION_MSG[correct] {
                        correct = 0;
                    }
                    if c == PROTO_VERSION_MSG[correct] {
                        correct += 1;
                        if correct == PROTO_VERSION_MSG.len() {
                            ok = true;
                            break 'tries;
                        }
                    }
                }
                Err(e) => match e {
                    nb::Error::WouldBlock => continue 'tries,
                    nb::Error::Other(e) => return Err(Error::rx(e)),
                },
            }
        }
    }

    if !ok {
        return Err(Error::Protocol);
    }

    eat_rx_buffer(&mut tx, &mut rx)?;

    Ok(Open {
        tx: tx,
        rx: rx,
        mode: PhantomData,
    })
}

fn eat_rx_buffer<TX: serial::Write<u8>, RX: serial::Read<u8>>(
    _tx: &mut TX,
    rx: &mut RX,
) -> Result<(), Error<TX::Error, RX::Error>> {
    loop {
        match rx.read() {
            Ok(_) => (), // Ignore
            Err(err) => match err {
                nb::Error::WouldBlock => return Ok(()), // Stop if there's nothing else to read
                nb::Error::Other(err) => return Err(Error::rx(err)), // Propagate
            },
        }
    }
}
