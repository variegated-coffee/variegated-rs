use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio, PioPin};
use variegated_controller_lib::{ActualBoardFeaturesMutex, SensorCluster, SensorClusterError};
use variegated_embassy_rp_frequency_counter::PioFrequencyCounter;
use crate::state::SilviaSystemSensorState;

pub struct SilviaPioFrequencyCounter<'a> {
    pump_rpm_counter: PioFrequencyCounter<'a, PIO0>,
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

impl<'a> SilviaPioFrequencyCounter<'a> {
    pub fn new<PinT: PioPin>(p: PIO0, pin: PinT) -> SilviaPioFrequencyCounter<'a> {
        let pio = Pio::new(p, Irqs);

        SilviaPioFrequencyCounter {
            pump_rpm_counter: PioFrequencyCounter::new(pin, pio),
        }
    }
}

impl<'a> SensorCluster<SilviaSystemSensorState> for SilviaPioFrequencyCounter<'a> {
    async fn update_sensor_state(&mut self, state: &mut SilviaSystemSensorState, _board_features: &ActualBoardFeaturesMutex) -> Result<(), SensorClusterError> {
        state.pump_rpm = self.pump_rpm_counter.read_frequency();

        Ok(())
    }
}
