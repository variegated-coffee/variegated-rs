#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use defmt::unwrap;
use embassy_executor::{Executor, Spawner};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{I2C0, I2C1, PWM_CH4, SPI0, SPI1, UART0, UART1, USB};
use embassy_rp::usb::{Driver, self};
use static_cell::StaticCell;
use variegated_board_open_lcc_r2a;
use {defmt_rtt as _, panic_probe as _};

use embedded_alloc::Heap;
use variegated_board_features::OpenLCCBoardFeatures;

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
async fn main_loop(board_features: OpenLCCBoardFeatures<'static, PWM_CH4, UART0, UART1, I2C0, I2C1, SPI1, SPI0>) {
    let (mut ctx, mut sensor_state, mut actuator_state, board) = variegated_machine_definition_lelit_bianca::create(board_features);

    loop {
        for mut cluster in &mut ctx.sensors {
            sensor_state = cluster.update_sensor_state(sensor_state).await;
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
    let board_features = variegated_board_open_lcc_r2a::create_board_features(p);

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
//        unwrap!(spawner.spawn(logger_task()));
        unwrap!(spawner.spawn(main_loop(board_features))) 
    });
}