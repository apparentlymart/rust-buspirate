use buspirate::spi;
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
    println!("Opened tty");

    let bp = BusPirate::new(tx, rx).to_bitbang().unwrap();
    println!("Alloced and inited BusPirate");

    let mut bp_spi = bp.to_spi().unwrap();
    println!("Switched to SPI mode");

    bp_spi.set_speed(spi::Speed::Speed30KHz).unwrap();
    println!("Set SPI Speed");

    bp_spi.set_config(spi::DEFAULT_CONFIG).unwrap();
    println!("Set SPI Config");

    bp_spi.chip_select(true).unwrap();
    println!("Activated chip select");
    //bp_spi.transfer_byte(0b10101010).unwrap();
    bp_spi
        .transfer_bytes(&mut [0b10101010, 0b01010101])
        .unwrap();
    println!("Transferred bytes");
    bp_spi.chip_select(false).unwrap();
    println!("Deactivated chip select");

    bp_spi.to_hiz().unwrap().close().unwrap();
    println!("Closed (and thus reset) Bus Pirate");
}
