//! Module `spi` provides implementations of some of the SPI-related traits
//! defined by `embedded-hal`.

use embedded_hal::blocking::spi;

/// `BusPirateSPI` implements the `Transfer` and `Write` traits from the
/// `embedded_hal::blocking::spi` module.
///
/// This type wraps a Bus Pirate in SPI mode (as implemented in the `buspirate`
/// crate) and implements the two blocking HAL traits in terms of it. The
/// Bus Pirate must be placed in SPI mode and configured appropriately before
/// calling `BusPirateSPI::new`.
pub struct BusPirateSPI<BP: buspirate::spi::Comms> {
    bp: BP,
}

impl<BP, Error> BusPirateSPI<BP>
where
    BP: buspirate::spi::Comms<Error = Error>,
{
    /// `new` wraps a given Bus Pirate SPI mode object to implement the blocking
    /// SPI traits.
    ///
    /// Pass an SPI-mode Bus Pirate object, from the `buspirate` crate:
    ///
    /// ```rust
    /// let bp_spi = BusPirate::new(tx, rx).to_bitbang()?.to_spi()?;
    /// let hal_spi = BusPirateSPI::new(bp_spi);
    /// ```
    ///
    /// You can then pass the resulting `BusPirateSPI` object to a HAL driver
    /// that expects to recieve a blocking SPI implementation.
    pub fn new(bp_spi: BP) -> BusPirateSPI<BP> {
        BusPirateSPI { bp: bp_spi }
    }
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
