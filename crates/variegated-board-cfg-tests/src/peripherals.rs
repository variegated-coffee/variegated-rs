#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

/// `#[allow]` rather than a deletion: this trait is genuinely used, and load-bearing.
/// `Resources1`'s `p2: impl peripherals::Pin` makes the macro emit
/// `impl Resources1 where Resources1P2: peripherals::Pin {}`, which is what enforces the
/// bound -- point `p2` at `PIN_1` in board-cfg.toml and the crate stops compiling with
/// `the trait bound PIN_1: peripherals::Pin is not satisfied`. rustc's dead-code pass
/// just does not count a where-clause on an otherwise empty inherent impl as a use.
#[allow(dead_code)]
pub trait Pin {

}

pub(crate) struct PIN_0;
pub(crate) struct PIN_1;
pub(crate) struct PIN_2;

impl Pin for PIN_2 {

}

pub(crate) struct UART0;
pub(crate) struct UART1;
pub(crate) struct Peripherals {
    pub(crate) PIN_0: PIN_0,
    pub(crate) PIN_1: PIN_1,
    pub(crate) PIN_2: PIN_2,
    pub(crate) UART0: UART0,
    pub(crate) UART1: UART1,
}

impl Peripherals {
    pub(crate) const fn new() -> Self {
        Self {
            PIN_0,
            PIN_1,
            PIN_2,
            UART0,
            UART1,
        }
    }
}