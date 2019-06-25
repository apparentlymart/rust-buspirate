#![no_std]

use core::marker::PhantomData;
use embedded_hal::serial;

const PROTO_VERSION_MSG: [u8; 5] = ['B' as u8, 'B' as u8, 'I' as u8, 'O' as u8, '1' as u8];

#[derive(Debug, Clone)]
pub struct BusPirate<TX, RX> {
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
        for _ in 0..20 {
            nb::block!(self.tx.write(0x00)).map_err(Error::tx)?;
        }

        // Now we're expecting the Bus Pirate to return the string "BBIO1"
        // to signal that it speaks the version 1 bitbang protocol.
        for i in 0..5 {
            let c = nb::block!(self.rx.read()).map_err(Error::rx)?;
            if c != PROTO_VERSION_MSG[i] {
                return Err(Error::Protocol);
            }
        }

        // If all of the above worked out then the Bus Pirate is now in
        // binary HiZ mode.
        Ok(Open {
            tx: self.tx,
            rx: self.rx,
            mode: PhantomData,
        })
    }
}

pub struct HiZ; // HiZ Mode
pub struct SPI; // SPI Mode

pub trait Mode {}
impl Mode for HiZ {}
impl Mode for SPI {}

#[derive(Debug, Clone)]
pub struct Open<MODE: Mode, TX, RX> {
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
    pub fn close(mut self) -> Result<Closed<TX, RX>, Error<TXErr, RXErr>> {
        // We'll reset the Bus Pirate back into user terminal mode before
        // we release the serial port objects.
        nb::block!(self.tx.write(0b00001111)).map_err(Error::tx)?;

        Ok(Closed {
            tx: self.tx,
            rx: self.rx,
        })
    }
}

impl<TX, RX> Open<HiZ, TX, RX>
where
    TX: serial::Write<u8>,
    RX: serial::Read<u8>,
{
}

impl<TX, RX> Open<SPI, TX, RX>
where
    TX: serial::Write<u8>,
    RX: serial::Read<u8>,
{
}

pub struct Closed<TX, RX> {
    tx: TX,
    rx: RX,
}

impl<TX, RX> Closed<TX, RX>
where
    TX: serial::Write<u8>,
    RX: serial::Read<u8>,
{
    pub fn release(self) -> (TX, RX) {
        (self.tx, self.rx)
    }
}

#[derive(Debug)]
pub enum Error<TXErr, RXErr> {
    Protocol,
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

    fn tx_result<R>(got: nb::Result<R, TXErr>) -> nb::Result<R, Self> {
        match got {
            Ok(v) => Ok(v),
            Err(err) => match err {
                nb::Error::WouldBlock => return Err(nb::Error::WouldBlock),
                nb::Error::Other(err) => return Err(nb::Error::Other(Error::Write(err))),
            },
        }
    }

    fn rx_result<R>(got: nb::Result<R, RXErr>) -> nb::Result<R, Self> {
        match got {
            Ok(v) => Ok(v),
            Err(err) => match err {
                nb::Error::WouldBlock => return Err(nb::Error::WouldBlock),
                nb::Error::Other(err) => return Err(nb::Error::Other(Error::Read(err))),
            },
        }
    }
}
