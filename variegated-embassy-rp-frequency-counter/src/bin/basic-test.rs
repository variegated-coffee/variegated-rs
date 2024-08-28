#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, gpio, Peripheral, spi, usb};
use embassy_rp::gpio::{AnyPin, Input, Pin, Pull};
use embassy_rp::peripherals::{PIN_18, PIN_28, PIO0, USB};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::Timer;
use gpio::{Level, Output};
use {defmt_rtt as _, panic_probe as _};
use variegated_embassy_rp_frequency_counter::PioFrequencyCounter;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low);

    loop {
        // Timekeeping is globally available, no need to mess with hardware timers.
        led.set_high();
        Timer::after_millis(1).await;
        led.set_low();
        Timer::after_millis(1).await;
    }
}
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let pio = Pio::new(p.PIO0, Irqs);
    let mut counter = PioFrequencyCounter::new_either_edge(p.PIN_13, pio);

    //let p13 = Input::new(p.PIN_13, Pull::None);

    for _ in 1..5 {
        info!("Pre-Loop!");
    }

    spawner.spawn(blink(p.PIN_18.degrade())).unwrap();

    loop {
        //info!("Loop!");
        let freq = counter.read_frequency();
        info!("Frequency: {:?}", freq);
        Timer::after_millis(2000).await;
    }
}