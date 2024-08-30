use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio, PioPin};
use variegated_controller_lib::{ActualBoardFeaturesMutex, SensorCluster, SensorClusterError};
use variegated_embassy_rp_frequency_counter::PioFrequencyCounter;
use variegated_log::log_info;
use crate::state::SilviaSystemSensorState;

pub struct SilviaPioFrequencyCounter<'a> {
    pump_rpm_counter: PioFrequencyCounter<'a, PIO0, 0>,
    //flow_meter_rpm_counter: PioFrequencyCounter<'a, PIO0, 1>,
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

impl<'a> SilviaPioFrequencyCounter<'a> {
    pub fn new<Pin1T: PioPin, Pin2T: PioPin>(p: PIO0, pump_rpm_pin: Pin1T, flow_meter_rpm_pin: Pin2T) -> SilviaPioFrequencyCounter<'a> {
        let mut pio = Pio::new(p, Irqs);

        log_info!("Creating SilviaPioFrequencyCounter 1");
        let pump_rpm_counter = PioFrequencyCounter::new(pump_rpm_pin, &mut pio.common, pio.sm0);
        log_info!("Creating SilviaPioFrequencyCounter 2");
        //let flow_meter_rpm_counter = PioFrequencyCounter::new(flow_meter_rpm_pin, &mut pio.common, pio.sm1);
        log_info!("Creating SilviaPioFrequencyCounter 3");


        SilviaPioFrequencyCounter {
            pump_rpm_counter,
            //flow_meter_rpm_counter,
        }
    }
}

impl<'a> SensorCluster<SilviaSystemSensorState> for SilviaPioFrequencyCounter<'a> {
    async fn update_sensor_state(&mut self, state: &mut SilviaSystemSensorState, _board_features: &ActualBoardFeaturesMutex) -> Result<(), SensorClusterError> {
        let freq = self.pump_rpm_counter.read_frequency();

        state.pump_rpm = if freq < 10000.0 { freq } else { 0.0 };
        //state.flow_meter_rpm = self.flow_meter_rpm_counter.read_frequency();

        Ok(())
    }
}
