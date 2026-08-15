#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

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