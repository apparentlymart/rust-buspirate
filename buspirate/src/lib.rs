#![no_std]

use core::marker::PhantomData;
use embedded_hal::serial;

const PROTO_VERSION_MSG: [u8; 5] = ['B' as u8, 'B' as u8, 'I' as u8, 'O' as u8, '1' as u8];
const PROTO_SPI_VERSION_MSG: [u8; 4] = ['S' as u8, 'P' as u8, 'I' as u8, '1' as u8];

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
        nb::block!(self.tx.write(0x10)).map_err(Error::tx)?;
        nb::block!(self.tx.flush()).map_err(Error::tx)?;

        // Before we go any further, we'll read out anything that's in the
        // receive buffer. If the Bus Pirate is behaving as expected then
        // its initialization messages and "HiZ>" prompt will be there.
        eat_rx_buffer(&mut self.tx, &mut self.rx)?;

        binary_reset_handshake(self.tx, self.rx)
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

impl<TX, RX, TXErr, RXErr> Open<HiZ, TX, RX>
where
    TX: serial::Write<u8, Error = TXErr>,
    RX: serial::Read<u8, Error = RXErr>,
{
    pub fn to_spi(self) -> Result<Open<SPI, TX, RX>, Error<TXErr, RXErr>> {
        binary_mode_handshake::<SPI, TX, RX>(self.tx, self.rx, 00000001, &PROTO_SPI_VERSION_MSG)
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
    tx: &mut TX,
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
