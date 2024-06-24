#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use defmt::unwrap;
use embassy_executor::{Executor, Spawner};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{I2C0, I2C1, PWM_CH4, SPI0, SPI1, UART0, UART1, USB};
use embassy_rp::usb::{Driver, self};
use embassy_sync::blocking_mutex::raw::{ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embedded_alloc::Heap;
use variegated_board_features::OpenLCCBoardFeatures;
use variegated_controller_lib::{ActualBoardFeatures, ActualBoardFeaturesMutex};

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
async fn main_loop(board_features: ActualBoardFeatures) {
//    let (mut ctx, mut sensor_state, mut actuator_state, board) = variegated_machine_definition_lelit_bianca::create(board_features);
    let (mut ctx, mut sensor_state, mut actuator_state, board) = variegated_machine_definition_gear_pump_silvia::create(board_features);

    let board_features_mutex: ActualBoardFeaturesMutex = Mutex::new(board);

    loop {
        for mut cluster in &mut ctx.sensors {
            let updated_state = cluster.update_sensor_state(sensor_state, &board_features_mutex).await;
            if let Ok(updated_state) = updated_state {
                sensor_state = updated_state;
            }
        }

        for mut controller in &mut ctx.controllers {
            actuator_state = controller.update_actuator_state_from_sensor_state(&sensor_state, actuator_state);
        }

        for mut cluster in &mut ctx.actuators {
            cluster.update_from_actuator_state(&actuator_state).await;
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

    cfg_if::cfg_if! {
        if #[cfg(feature = "open-lcc-board-r2a")] {
            let board_features = variegated_board_open_lcc_r2a::create_board_features(p);
        } else if #[cfg(feature = "apec-r0d")] {
            let board_features = variegated_board_apec::create_board_features(p);
        } else {
            compile_error!("No board feature set selected");
        } 
    }

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
//        unwrap!(spawner.spawn(logger_task()));
        unwrap!(spawner.spawn(main_loop(board_features))) 
    });
}