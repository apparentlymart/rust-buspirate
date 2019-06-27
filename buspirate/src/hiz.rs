use crate::low;
use crate::BusPirate;
use crate::Error;
use embedded_hal::serial;

pub struct HiZ<TX: serial::Write<u8>, RX: serial::Read<u8>> {
    pub(crate) ch: low::Channel<TX, RX>,
}

impl<TX, RX, TXErr, RXErr> HiZ<TX, RX>
where
    TX: serial::Write<u8, Error = TXErr>,
    RX: serial::Read<u8, Error = RXErr>,
{
    pub fn close(self) -> Result<BusPirate<TX, RX>, Error<TXErr, RXErr>> {
        crate::close_handshake(self.ch)
    }

    pub fn to_spi(self) -> Result<crate::spi::SPI<TX, RX>, Error<TXErr, RXErr>> {
        let ch = crate::binary_mode_handshake(self.ch, 0b00000001, &crate::PROTO_SPI_VERSION_MSG)?;
        Ok(crate::spi::SPI { ch: ch })
    }
}
