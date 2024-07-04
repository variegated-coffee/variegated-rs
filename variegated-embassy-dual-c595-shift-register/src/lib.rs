#![no_std]

use embassy_rp::gpio::{AnyPin, Output};
use embassy_time::{Delay, Duration, Instant, Timer};

pub struct DualC595ShiftRegister<'a> {
    serial_pin: Output<'a, AnyPin>,
    shift_register_clock_pin: Output<'a, AnyPin>,
    storage_register_clock_pin: Output<'a, AnyPin>,
}

impl<'a> DualC595ShiftRegister<'a> {
    pub fn new(
        serial_pin: Output<'a, AnyPin>,
        shift_register_clock_pin: Output<'a, AnyPin>,
        storage_register_clock_pin: Output<'a, AnyPin>,
    ) -> Self {
        DualC595ShiftRegister {
            serial_pin,
            shift_register_clock_pin,
            storage_register_clock_pin,
        }
    }

    pub async fn write(&mut self, data: u16) {
        for i in 0..16 {
            match data & (1 << i) != 0 {
                true => self.serial_pin.set_high(),
                false => self.serial_pin.set_low(),
            }
            busy_wait_us(1);
            self.shift_register_clock_pin.set_high();
            busy_wait_us(1);
            self.shift_register_clock_pin.set_low();
        }

        self.serial_pin.set_low();

        self.storage_register_clock_pin.set_high();
        busy_wait_us(5);
        self.storage_register_clock_pin.set_low();
    }

    pub(crate) async fn clear(&mut self) {
        self.write(0).await;
    }
}

fn busy_wait_us(us: u64) {
    let duration = Duration::from_micros(us);
    let start = Instant::now();
    while Instant::now() - start < duration {
        core::hint::spin_loop();
    }
}