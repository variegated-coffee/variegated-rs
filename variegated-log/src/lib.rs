#![no_std]

use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
#[macro_export]
macro_rules! log_error {
    ($($arg:tt)*) => {
        {
            log::error!($($arg)*);
            defmt::error!($($arg)*);
        }
    };
}

#[macro_export]
macro_rules! log_warn {
    ($($arg:tt)*) => {
        {
            log::warn!($($arg)*);
            defmt::warn!($($arg)*);
        }
    };
}

#[macro_export]
macro_rules! log_info {
    ($($arg:tt)*) => {
        {
            log::info!($($arg)*);
            defmt::info!($($arg)*);
        }
    };
}

#[macro_export]
macro_rules! log_debug {
    ($($arg:tt)*) => {
        {
            log::debug!($($arg)*);
            defmt::debug!($($arg)*);
        }
    };
}

#[macro_export]
macro_rules! log_trace {
    ($($arg:tt)*) => {
        {
            log::trace!($($arg)*);
            defmt::trace!($($arg)*);
        }
    };
}

#[embassy_executor::task]
pub async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}
