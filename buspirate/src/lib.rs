#![no_std]

mod low;
pub mod peripherals;
pub mod spi;

use embedded_hal::serial;

const PROTO_VERSION_MSG: [u8; 5] = ['B' as u8, 'B' as u8, 'I' as u8, 'O' as u8, '1' as u8];
const PROTO_SPI_VERSION_MSG: [u8; 4] = ['S' as u8, 'P' as u8, 'I' as u8, '1' as u8];

#[derive(Debug, Clone)]
pub struct BusPirate<TX: serial::Write<u8>, RX: serial::Read<u8>> {
    ch: low::Channel<TX, RX>,
}

impl<TX, RX, TXErr, RXErr> BusPirate<TX, RX>
where
    TX: serial::Write<u8, Error = TXErr>,
    RX: serial::Read<u8, Error = RXErr>,
{
    pub fn new(tx: TX, rx: RX) -> Self {
        Self {
            ch: low::Channel::new(tx, rx),
        }
    }

    pub fn init(mut self) -> Result<HiZ<TX, RX>, Error<TXErr, RXErr>> {
        // The Bus Pirate could be in any mode when we find it, so
        // we follow the advice given in the protocol documentation:
        // - Send newline 10 times to escape from any menu/prompts in progress
        // - Send '#' to reset
        // - Send nul (0x00) 20 times to enter binary protocol mode

        for _ in 0..10 {
            self.ch.write(0x10)?;
        }
        self.ch.write('#' as u8)?;
        self.ch.write(0x10)?;
        self.ch.flush()?;

        // Before we go any further, we'll read out anything that's in the
        // receive buffer. If the Bus Pirate is behaving as expected then
        // its initialization messages and "HiZ>" prompt will be there.
        self.ch.eat_rx_buffer()?;

        binary_reset_handshake(self.ch)
    }

    pub fn release(self) -> (TX, RX) {
        (self.ch.tx, self.ch.rx)
    }
}

pub struct HiZ<TX: serial::Write<u8>, RX: serial::Read<u8>> {
    ch: low::Channel<TX, RX>,
}

impl<TX, RX, TXErr, RXErr> HiZ<TX, RX>
where
    TX: serial::Write<u8, Error = TXErr>,
    RX: serial::Read<u8, Error = RXErr>,
{
    pub fn close(self) -> Result<BusPirate<TX, RX>, Error<TXErr, RXErr>> {
        close_handshake(self.ch)
    }

    pub fn to_spi(self) -> Result<spi::SPI<TX, RX>, Error<TXErr, RXErr>> {
        let ch = binary_mode_handshake(self.ch, 0b00000001, &PROTO_SPI_VERSION_MSG)?;
        Ok(spi::SPI { ch: ch })
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

fn binary_mode_handshake<TX: serial::Write<u8>, RX: serial::Read<u8>>(
    mut ch: low::Channel<TX, RX>,
    send: u8,
    expect: &'static [u8; 4],
) -> Result<low::Channel<TX, RX>, Error<TX::Error, RX::Error>> {
    let mut ok = false;
    'tries: for _ in 0..10 {
        ch.flush()?;
        ch.write(send)?;

        let mut correct = 0;
        loop {
            match ch.rx.read() {
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

    ch.eat_rx_buffer()?;

    Ok(ch)
}

fn binary_reset_handshake<TX: serial::Write<u8>, RX: serial::Read<u8>>(
    mut ch: low::Channel<TX, RX>,
) -> Result<HiZ<TX, RX>, Error<TX::Error, RX::Error>> {
    let mut ok = false;
    'tries: for _ in 0..20 {
        ch.flush()?;
        ch.write(0x00)?;

        let mut correct = 0;
        loop {
            match ch.rx.read() {
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

    ch.eat_rx_buffer()?;

    Ok(HiZ { ch: ch })
}

fn close_handshake<TX: serial::Write<u8>, RX: serial::Read<u8>>(
    mut ch: low::Channel<TX, RX>,
) -> Result<BusPirate<TX, RX>, Error<TX::Error, RX::Error>> {
    ch.write(0b00001111)?;
    Ok(BusPirate { ch: ch })
}
