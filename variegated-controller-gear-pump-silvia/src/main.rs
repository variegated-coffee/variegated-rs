#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use alloc::format;
use defmt::unwrap;
use embassy_executor::{Executor, Spawner};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{I2C0, I2C1, SPI0, SPI1, UART0, UART1, USB};
use embassy_rp::uart::{Async, UartTx};
use embassy_rp::usb::{Driver, self};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Instant, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embedded_alloc::Heap;
use postcard::{to_allocvec_cobs, to_vec};
use serde::{Deserialize, Serialize};
use variegated_board_features::OpenLCCBoardFeatures;
use variegated_controller_lib::{ActualBoardFeatures, ActualBoardFeaturesMutex, ActualMutexType, ActuatorCluster, Controller, SensorCluster};
use variegated_embassy_ads124s08::registers::{IDACMagnitude, IDACMux, Mux, PGAGain, ReferenceInput};
use variegated_log::{log_info, logger_task};
use crate::controller::{SilviaCommands, SilviaController};
use crate::demo_proto::PertinentData;
use crate::sensor_cluster::silvia_adc_sensor_cluster::SilviaLinearVoltageToCelsiusConversionParameters;
use crate::state::{SilviaSystemActuatorState, SilviaSystemConfiguration, SilviaSystemGeneralState, SilviaSystemSensorState};

mod actuator_cluster;
mod sensor_cluster;
mod state;
mod controller;
mod demo_proto;

#[global_allocator]
static HEAP: Heap = Heap::empty();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn main_loop(mut board_features: ActualBoardFeatures) {
    log_info!("Starting main loop");

    let mut control_uart_tx = board_features.esp32_uart_tx.take().expect("UART TX must be present");
    let control_uart_rx = board_features.esp32_uart_rx.take().expect("UART RX must be present");

    log_info!("Foo");

    let board_features_mutex: ActualBoardFeaturesMutex = Mutex::new(board_features);
//    let configuration_mutex: ActualMutexType<SilviaSystemConfiguration> = Mutex::new(SilviaSystemConfiguration::default());
    
    let channel = Channel::<NoopRawMutex, SilviaCommands, 10>::new();
    
    let channel_ref = &channel;
    log_info!("Bar");

    let (mut gpio_sensor_cluster, mut adc_sensor_cluster, mut pump_freq, mut nau7802) =
        sensor_cluster::create_sensor_clusters(&board_features_mutex).await;

    log_info!("Baz");


    let mut controller = SilviaController::new(&channel);

    let (mut shift_reg_actuator_cluster, mut pwm_output) = actuator_cluster::create_actuator_clusters(&board_features_mutex).await;

    let mut sensor_state = SilviaSystemSensorState::default();
    let mut actuator_state = SilviaSystemActuatorState::default();
    let mut general_state = SilviaSystemGeneralState::default();

    Timer::after_millis(2000).await;
    
    adc_sensor_cluster.init_adc(&board_features_mutex).await;
    
    loop {
        let time = Instant::now();
        //log_info!("Loop!");
        
        controller.handle_commands().await;

        let res = gpio_sensor_cluster.update_sensor_state(&mut sensor_state, &board_features_mutex).await;
        let res = adc_sensor_cluster.update_sensor_state(&mut sensor_state, &board_features_mutex).await;
        let res = pump_freq.update_sensor_state(&mut sensor_state, &board_features_mutex).await;
        //let res = nau7802.update_sensor_state(&mut sensor_state, &board_features_mutex).await;
        
        let res = controller.update_states_from_sensor_state(&sensor_state, &mut actuator_state, &mut general_state).await;

        let res = shift_reg_actuator_cluster.update_from_actuator_state(&actuator_state).await;
        let res = pwm_output.update_from_actuator_state(&actuator_state).await;

        let data = PertinentData {
            boiler_temp: sensor_state.boiler_temp_c,
            boiler_pressure: sensor_state.boiler_pressure_bar,
            boiler_duty_cycle: actuator_state.brew_boiler_heating_element.current_duty_cycle_percent as f32,
            pump_duty_cycle: actuator_state.pump_duty_cycle,
            is_brewing: general_state.is_brewing,
        };
        
/*        let data_vec = to_allocvec_cobs(&data).unwrap();
        control_uart_tx.write(&data_vec).await;*/

        structured_log(&mut control_uart_tx, sensor_state, actuator_state, general_state).await;
        
        Timer::after_millis(50).await;
    }
}

async fn structured_log(uart: &mut UartTx<'_, UART1, Async>,sensor_state: SilviaSystemSensorState, actuator_state: SilviaSystemActuatorState, general_state: SilviaSystemGeneralState) {
    log_info!("Writing stuff to UART");
    let s = format!("/*{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?},{:?}*/\n",
        sensor_state.boiler_temp_c, 
        sensor_state.boiler_pressure_bar, 
        actuator_state.brew_boiler_heating_element.current_duty_cycle_percent, 
        actuator_state.pump_duty_cycle, 
        general_state.extraction_phase,
        if let Some(brew_time_s) = general_state.brew_time_s {
            brew_time_s
        } else {
            0.0
        },
        general_state.current_pressure_set_point,
        general_state.brew_boiler_pid.p,
        general_state.brew_boiler_pid.i,
        general_state.brew_boiler_pid.d,
        general_state.brew_pressure_pid.p,
        general_state.brew_pressure_pid.i,
        general_state.brew_pressure_pid.d,
        general_state.scale_tared_g,
        sensor_state.pump_rpm,
    );
    
    uart.write(s.as_bytes()).await.expect("Writing stuff");
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