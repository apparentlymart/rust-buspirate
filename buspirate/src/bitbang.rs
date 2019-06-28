//! Module `bigbang` contains the API for "binary bit-bang" mode.
//!
//! This mode cannot be entered directly. Instead, create a `BusPirate` object
//! (from the root module of this crate) and call `to_bitbang` on it:
//!
//! ```rust
//! let bp = BusPirate::new();
//! let bb = bp.to_bitbang()?;
//! ```
//!
//! The result of `to_bitbang` is an instance of `BitBang`.

use crate::low;
use crate::BusPirate;
use crate::Error;
use embedded_hal::serial;

/// `BitBang` represents a Bus Pirate device in "binary bit-bang" mode.
///
/// This mode serves both as an interface to directly control some of the
/// Bus Pirate's pins and as an intermediate step to reach the higher-level
/// protocol modes.
pub struct BitBang<TX: serial::Write<u8>, RX: serial::Read<u8>> {
    pub(crate) ch: low::Channel<TX, RX>,
}

impl<TX, RX, TXErr, RXErr> BitBang<TX, RX>
where
    TX: serial::Write<u8, Error = TXErr>,
    RX: serial::Read<u8, Error = RXErr>,
{
    /// `close` resets the Bus Pirate back into normal terminal mode, exiting
    /// binary bitbang mode.
    pub fn close(self) -> Result<BusPirate<TX, RX>, Error<TXErr, RXErr>> {
        crate::close_handshake(self.ch)
    }

    /// `to_spi` switches to SPI mode.
    ///
    /// This consumes the `BigBang` object. To retrieve it, call `to_bitbang`
    /// on the resulting SPI object to switch back to the bit-bang mode.
    pub fn to_spi(self) -> Result<crate::spi::SPI<TX, RX>, Error<TXErr, RXErr>> {
        let ch = crate::binary_mode_handshake(self.ch, 0b00000001, &crate::PROTO_SPI_VERSION_MSG)?;
        Ok(crate::spi::SPI { ch: ch })
    }
}
