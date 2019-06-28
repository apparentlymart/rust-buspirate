//! Bus Pirate client library
//!
//! This library implements the [Bus Pirate](http://dangerousprototypes.com/docs/Bus_Pirate)
//! binary protocol, allowing Rust programs to interact with a Bus Pirate and
//! in turn to interact with SPI, I2C, UART, etc devices.
//! The implemented protocol is that of the Bus Pirate v3.6.
//!
//! The library API uses types to ensure safe switching between different Bus
//! Pirate modes and to provide functions relating only to the current mode.
//! At initialization, the bus pirate is assumed to be in its normal terminal
//! mode, and so the first step implemented by this library is to switch into
//! binary mode. After that, the caller may transition into other binary modes
//! as desired.
//!
//! The entry point is `BusPirate::new`, which takes (and consumes) a serial
//! writer and a serial reader as defined by
//! [`embedded_hal::serial`](https://docs.rs/embedded-hal/0.2.3/embedded_hal/serial/).
//! If you are running on a general computing platform then you can use
//! [`serial_embedded_hal`](https://docs.rs/serial-embedded-hal/0.1.2/serial_embedded_hal/struct.Serial.html)
//! to connect with a serial port provided by your operating system:
//!
//! ```rust
//! let port = Serial::new(
//!     "/dev/ttyUSB0",
//!     &PortSettings {
//!         baud_rate: serial_embedded_hal::BaudRate::Baud115200,
//!         char_size: serial_embedded_hal::CharSize::Bits8,
//!         parity: serial_embedded_hal::Parity::ParityNone,
//!         stop_bits: serial_embedded_hal::StopBits::Stop1,
//!         flow_control: serial_embedded_hal::FlowControl::FlowNone,
//!     },
//! )?;
//! let (tx, rx) = port.split();
//! let bp = BusPirate::new(tx, rx);
//! ```
//!
//! A `BusPirate` object represents a Bus Pirate in normal terminal mode, not
//! yet configured to speak a binary protocol. Method `init` can then transition
//! into "binary bit-bang" mode, yielding a `bitbang::BitBang` object:
//!
//! ```rust
//! let bb = bp.init()?;
//! ```
//!
//! As well as offering direct control over the Bus Pirate's pins, bit-bang
//! mode is also a gateway into the other more specialized protocol modes. For
//! example, SPI mode:
//!
//! ```rust
//! let spi = bp.to_spi()?;
//! ```

#![no_std]

pub mod bitbang;
mod low;
pub mod peripherals;
pub mod spi;

use embedded_hal::serial;

const PROTO_VERSION_MSG: [u8; 5] = ['B' as u8, 'B' as u8, 'I' as u8, 'O' as u8, '1' as u8];
const PROTO_SPI_VERSION_MSG: [u8; 4] = ['S' as u8, 'P' as u8, 'I' as u8, '1' as u8];

/// `BusPirate` represents a Bus Pirate device in its normal terminal mode, not
/// yet initialized into any binary mode.
///
/// The primary method on `BusPirate` is `to_bitbang`, which transitions the
/// device into "binary bit-bang" mode. That mode then also allows transitions
/// into the other binary modes.
///
/// ```rust
/// let bb = bp.to_bitbang()?;
/// ```
#[derive(Debug, Clone)]
pub struct BusPirate<TX: serial::Write<u8>, RX: serial::Read<u8>> {
    ch: low::Channel<TX, RX>,
}

impl<TX, RX, TXErr, RXErr> BusPirate<TX, RX>
where
    TX: serial::Write<u8, Error = TXErr>,
    RX: serial::Read<u8, Error = RXErr>,
{
    /// `BusPirate::new` associates some serial channels with a new `BusPirate`
    /// object.
    ///
    /// The transmit and receive objects are consumed. If the caller needs to
    /// access them again, it must call `release` to discard the `BusPirate`
    /// object and recover the original objects.
    pub fn new(tx: TX, rx: RX) -> Self {
        Self {
            ch: low::Channel::new(tx, rx),
        }
    }

    /// `to_bitbang` directs the Bus Pirate to move into "binary bit-bang" mode.
    ///
    /// The Bus Pirate requires several steps to properly switch from terminal
    /// mode into binary bitbang mode, so this method can potentially be slow
    /// due to sending and receiving several characters.
    ///
    /// `to_bitbang` consumes the `BusPirate` object and returns a `BitBang`
    /// object in its place. To recover the `BusPirate` object, call `close`
    /// on the `BitBang` object to reset the Bus Pirate back into terminal mode.
    pub fn to_bitbang(mut self) -> Result<bitbang::BitBang<TX, RX>, Error<TXErr, RXErr>> {
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

    /// `release` returns the serial transmit and receive objects wrapped by
    /// the `BusPirate` object.
    ///
    /// This consumes the `BusPirate` object.
    pub fn release(self) -> (TX, RX) {
        (self.ch.tx, self.ch.rx)
    }
}

/// `Error` represents communication errors.
#[derive(Debug)]
pub enum Error<TXErr, RXErr> {
    /// `Protocol` indicates that the library receieved an invalid or unexpected
    /// response from the Bus Pirate in response to a request.
    Protocol,

    /// `Request` indicates that the caller provided invalid arguments that
    /// could not be checked at compile time.
    Request,

    /// `Write` indicates that the underlying serial write object returned an
    /// error.
    ///
    /// The data is the error returned by the underlying serial implementation.
    Write(TXErr),

    /// `Read` indicates that the underlying serial read object returned an
    /// error.
    ///
    /// The data is the error returned by the underlying serial implementation.
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
) -> Result<bitbang::BitBang<TX, RX>, Error<TX::Error, RX::Error>> {
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

    Ok(bitbang::BitBang { ch: ch })
}

fn close_handshake<TX: serial::Write<u8>, RX: serial::Read<u8>>(
    mut ch: low::Channel<TX, RX>,
) -> Result<BusPirate<TX, RX>, Error<TX::Error, RX::Error>> {
    ch.write(0b00001111)?;
    Ok(BusPirate { ch: ch })
}
