//! TFT backlight controller for dual boiler espresso machine
//!
//! The backlight is on, always, driven as a plain GPIO output.
//!
//! **Do not dim it with PWM.** Switching an LED backlight's current at a kilohertz puts a
//! square wave on the supply next to a shared SPI bus, and on this board that is the
//! source of the SD card's CRC errors and timeouts -- the ~1 kHz standby dimming this
//! once did accounted for every one of them.
//!
//! It does not follow machine mode either, even as a plain on/off: a dark panel on an idle
//! machine reads as a broken machine, and the panel draws little enough that dimming is
//! not worth the ambiguity.
//!
//! What remains is a pin held high. The task exists only to own the [`Output`] for the
//! life of the program -- dropping it would release the pin -- so it parks rather than
//! looping. It takes no status subscriber, which leaves that slot free for something that
//! reads it.

use embassy_rp::gpio::{Level, Output};
use defmt::info;
use embassy_rp::Peri;
use embassy_time::Timer;

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
pub async fn backlight_task(
    backlight_p: BacklightPeripherals,
    checkin: variegated_checkin::CheckinHandle,
) {
    let _backlight = Output::new(backlight_p.pin, Level::High);
    info!("TFT backlight on");

    // Never returns: the task's only job is to keep `_backlight` alive.
    //
    // A heartbeat rather than `pending()`, so the row has a period like everything else. The
    // old shape was polled once and never again, which meant the host was told never to age
    // it -- and a row that can never be late is a row that cannot report core 1 having
    // stopped scheduling. Waking every few seconds to store two words is free next to the
    // 100 Hz renderer sharing this core.
    //
    // If this loop is ever left, the `Output` guard above goes with it and the backlight
    // physically turns off. That is the one explanation for a dark panel that is not the
    // renderer, and the row now reports it as a stale age rather than not at all.
    loop {
        checkin.good();
        Timer::after(variegated_checkin::HEARTBEAT).await;
    }
}
