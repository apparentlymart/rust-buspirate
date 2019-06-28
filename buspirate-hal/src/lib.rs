//! `embedded-hal` implementations via a Bus Pirate device
//!
//! [`embedded-hal`](https://crates.io/crates/embedded-hal) is a hardware
//! abstraction layer for embedded systems. It is most commonly used in
//! applications destined to run on microcontrollers or other small computing
//! devices embedded inside products.
//!
//! This library contains implementations of some of the `embedded-hal` traits
//! in terms of the [Bus Pirate](http://dangerousprototypes.com/docs/Bus_Pirate),
//! thus allowing a HAL device driver to interact with its corresponding
//! hardware from a general-purpose computer using the Bus Pirate as an
//! intermediary.
//!
//! While this is unlikely to be useful in any actual embedded system, it could
//! be useful when developing HAL device drivers in order to achieve a faster
//! write/test cycle by running the driver code directly on your development
//! workstation.
//!
//! All of the trait implementations in this library require an
//! already-configured Bus Pirate mode object from the `buspirate` crate.

#![no_std]

extern crate embedded_hal;

pub mod spi;
