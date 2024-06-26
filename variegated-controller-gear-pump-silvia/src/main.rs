#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use defmt::unwrap;
use embassy_executor::{Executor, Spawner};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{I2C0, I2C1, PWM_CH4, SPI0, SPI1, UART0, UART1, USB};
use embassy_rp::usb::{Driver, self};
use embassy_sync::blocking_mutex::raw::{ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embedded_alloc::Heap;
use variegated_board_features::OpenLCCBoardFeatures;
use variegated_controller_lib::{ActualBoardFeatures, ActualBoardFeaturesMutex, ActualMutexType, ActuatorCluster, Controller, SensorCluster};
use variegated_embassy_ads124s08::registers::{IDACMagnitude, IDACMux, Mux, PGAGain, ReferenceInput};
use variegated_log::{log_info, logger_task};
use crate::controller::SilviaController;
use crate::sensor_cluster::silvia_adc_sensor_cluster::SilviaLinearVoltageToCelsiusConversionParameters;
use crate::state::{SilviaSystemActuatorState, SilviaSystemConfiguration, SilviaSystemGeneralState, SilviaSystemSensorState};

mod actuator_cluster;
mod sensor_cluster;
mod state;
mod controller;

#[global_allocator]
static HEAP: Heap = Heap::empty();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn main_loop(mut board_features: ActualBoardFeatures) {
    log_info!("Starting main loop");

    let board_features_mutex: ActualBoardFeaturesMutex = Mutex::new(board_features);
//    let configuration_mutex: ActualMutexType<SilviaSystemConfiguration> = Mutex::new(SilviaSystemConfiguration::default());

    let (mut gpio_sensor_cluster, mut adc_sensor_cluster) =
        sensor_cluster::create_sensor_clusters(&board_features_mutex).await;

    let mut controller = SilviaController::default();

    let (mut shift_reg_actuator_cluster) = actuator_cluster::create_actuator_clusters(&board_features_mutex).await;

    let mut sensor_state = SilviaSystemSensorState::default();
    let mut actuator_state = SilviaSystemActuatorState::default();
    let mut general_state = SilviaSystemGeneralState::default();

    loop {
        log_info!("Loop!");

        let res = gpio_sensor_cluster.update_sensor_state(&mut sensor_state, &board_features_mutex).await;
        let res = adc_sensor_cluster.update_sensor_state(&mut sensor_state, &board_features_mutex).await;

        let res = controller.update_actuator_state_from_sensor_state(&sensor_state, &mut actuator_state).await;

        let res = shift_reg_actuator_cluster.update_from_actuator_state(&actuator_state).await;

        Timer::after_millis(1000).await;
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
    let mut board_features = variegated_board_apec::create_board_features(p);

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        let driver = Driver::new(board_features.usb.take().expect("USB must be unused"), Irqs);
        unwrap!(spawner.spawn(logger_task(driver)));
        unwrap!(spawner.spawn(main_loop(board_features)))
    });
}