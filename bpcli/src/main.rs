use buspirate::BusPirate;
use serial_embedded_hal::{PortSettings, Serial};

fn main() {
    // FIXME: For the moment this is just a hacky testbed for the main
    // "buspirate" crate, not a real CLI tool that is useful to anyone.
    let port = Serial::new(
        "/dev/ttyUSB0",
        &PortSettings {
            baud_rate: serial_embedded_hal::BaudRate::Baud115200,
            char_size: serial_embedded_hal::CharSize::Bits8,
            parity: serial_embedded_hal::Parity::ParityNone,
            stop_bits: serial_embedded_hal::StopBits::Stop1,
            flow_control: serial_embedded_hal::FlowControl::FlowNone,
        },
    )
    .unwrap();
    let (tx, rx) = port.split();

    let bp = BusPirate::new(tx, rx).init().unwrap();

    let bp_spi = bp.to_spi().unwrap();

    bp_spi.exit().unwrap().close().unwrap();

    println!("Hello, world!");
}
