use crate::Error;
use embedded_hal::serial;

#[derive(Debug, Clone)]
pub(crate) struct Channel<TX: serial::Write<u8>, RX: serial::Read<u8>> {
    pub tx: TX,
    pub rx: RX,
}

impl<TX, RX, TXErr, RXErr> Channel<TX, RX>
where
    TX: serial::Write<u8, Error = TXErr>,
    RX: serial::Read<u8, Error = RXErr>,
{
    pub fn new(tx: TX, rx: RX) -> Self {
        Self { tx: tx, rx: rx }
    }

    pub fn read(&mut self) -> Result<u8, Error<TXErr, RXErr>> {
        nb::block!(self.rx.read()).map_err(Error::rx)
    }

    pub fn write(&mut self, c: u8) -> Result<(), Error<TXErr, RXErr>> {
        nb::block!(self.tx.write(c)).map_err(Error::tx)
    }

    pub fn flush(&mut self) -> Result<(), Error<TXErr, RXErr>> {
        nb::block!(self.tx.flush()).map_err(Error::tx)
    }

    pub fn simple_command(&mut self, cmd: u8) -> Result<(), Error<TXErr, RXErr>> {
        nb::block!(self.tx.write(cmd)).map_err(Error::tx)?;
        nb::block!(self.tx.flush()).map_err(Error::tx)?;

        match nb::block!(self.rx.read()).map_err(Error::rx)? {
            0x01 => Ok(()),
            _ => Err(Error::Protocol),
        }
    }

    pub fn eat_rx_buffer(&mut self) -> Result<(), Error<TXErr, RXErr>> {
        loop {
            match self.rx.read() {
                Ok(_) => (), // Ignore
                Err(err) => match err {
                    nb::Error::WouldBlock => return Ok(()), // Stop if there's nothing else to read
                    nb::Error::Other(err) => return Err(Error::rx(err)), // Propagate
                },
            }
        }
    }
}
