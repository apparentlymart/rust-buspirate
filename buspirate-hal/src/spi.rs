use embedded_hal::blocking::spi;

pub struct BusPirateSPI<BP: buspirate::spi::Comms> {
    bp: BP,
}

impl<BP, Error> spi::Transfer<u8> for BusPirateSPI<BP>
where
    BP: buspirate::spi::Comms<Error = Error>,
{
    type Error = BP::Error;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.bp.transfer(words)
    }
}

impl<BP, Error> spi::Write<u8> for BusPirateSPI<BP>
where
    BP: buspirate::spi::Comms<Error = Error>,
{
    type Error = BP::Error;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut empty: [u8; 0] = [];
        self.bp.transaction(words, &mut empty[..], false)
    }
}
