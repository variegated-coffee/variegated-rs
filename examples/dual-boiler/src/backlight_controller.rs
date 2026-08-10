//! TFT backlight controller for dual boiler espresso machine
//!
//! The backlight is on, always, driven as a plain GPIO output.
//!
//! It has been through two simplifications. It used to run at ~1 kHz PWM to give a dim
//! (10%) standby level; switching an LED backlight's current at a kilohertz puts a square
//! wave on the supply next to a shared SPI bus, and on this board that turned out to be
//! the source of the SD card's CRC errors and timeouts -- removing it removed every one of
//! them. That left an on/off pin still following machine mode, and now it does not do that
//! either: a dark panel on an idle machine reads as a broken machine, and the panel draws
//! little enough that dimming it was never worth the ambiguity.
//!
//! What remains is a pin held high. The task exists only to own the [`Output`] for the
//! life of the program -- dropping it would release the pin -- so it parks rather than
//! looping. It takes no status subscriber, which leaves that slot free for something that
//! reads it.

use embassy_rp::gpio::{Level, Output};
use defmt::info;
use embassy_rp::Peri;

#[variegated_board_cfg::board_cfg("backlight_peripherals")]
pub(crate) struct BacklightPeripherals {
    pub(crate) pin: Peri<'static, ()>,
}

/// Turn the TFT backlight on and hold it there.
///
/// # Arguments
///
/// * `backlight_p` - Peripheral resources for the backlight pin
#[cfg(feature = "tft-display")]
#[embassy_executor::task]
pub async fn backlight_task(backlight_p: BacklightPeripherals) {
    let _backlight = Output::new(backlight_p.pin, Level::High);
    info!("TFT backlight on");

    // Never returns: the task's only job is to keep `_backlight` alive.
    core::future::pending::<()>().await
}
