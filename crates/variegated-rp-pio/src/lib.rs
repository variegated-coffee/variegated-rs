//! Arithmetic the RP2040/RP2350 PIO needs and embassy does not do for you.
//!
//! Two things, both pure, both host-tested, and both the kind of code that is correct on
//! every board but one:
//!
//! - [`window`] -- the RP2350 `GPIOBASE` window, and pin numbers counted relative to it.
//! - [`clock`] -- turning a target bit rate into a PIO clock divider, for a program of a
//!   given cost in cycles per bit.
//!
//! # Why this is a crate rather than a module
//!
//! Because two transports need it and neither should depend on the other.
//! `variegated-pio-spi-sd-card` drives a card in SPI mode; `variegated-pio-mmc-bus` drives
//! one over a native 4-bit bus. They share no protocol, no programs and no types -- only
//! this. A leaf crate is the only arrangement in which the dependency arrows point
//! somewhere sensible, and duplicating [`window`] between them is not an option: it is
//! exactly the arithmetic that has already produced silent, hard-to-find faults, and two
//! copies would diverge.
//!
//! # No `embassy-rp` dependency, deliberately
//!
//! Nothing here touches a peripheral. That is what lets both transports keep their
//! window and clock arithmetic under host test while the code that uses it cannot link on a
//! host at all.

#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]

pub mod clock;
pub mod window;
