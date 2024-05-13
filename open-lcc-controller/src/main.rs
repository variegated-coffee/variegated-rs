#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use defmt::unwrap;
use embassy_executor::{Executor, Spawner};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PWM_CH4, USB};
use embassy_rp::usb::{Driver, self};
use static_cell::StaticCell;
use open_lcc_board_r2a;
use {defmt_rtt as _, panic_probe as _};

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn main_loop() {
    let (mut ctx, mut sensor_state, mut actuator_state) = open_lcc_machine_definition_lelit_bianca::create();

    loop {
        for mut cluster in &mut ctx.sensors {
            sensor_state = cluster.update_sensor_state(sensor_state);
        }

        for mut controller in &mut ctx.controllers {
            actuator_state = controller.update_actuator_state_from_sensor_state(&sensor_state, actuator_state);
        }

        for mut cluster in &mut ctx.actuators {
            cluster.update_from_actuator_state(&actuator_state);
        }
    }
}

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
#[cortex_m_rt::entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let p = embassy_rp::init(Default::default());
    let board_features = open_lcc_board_r2a::create_board_features(p);

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
//        unwrap!(spawner.spawn(logger_task()));
        unwrap!(spawner.spawn(main_loop())) 
    });
}