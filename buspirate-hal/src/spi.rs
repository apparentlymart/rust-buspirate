use embedded_hal::blocking::spi;

pub struct BusPirateSPI<BP: buspirate::spi::TransferBuffered> {
    bp: BP,
}

impl<BP, Error> spi::Transfer<u8> for BusPirateSPI<BP>
where
    BP: buspirate::spi::TransferBuffered<Error = Error>,
{
    type Error = BP::Error;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.bp.transfer(words, words.len(), false)
    }
}
