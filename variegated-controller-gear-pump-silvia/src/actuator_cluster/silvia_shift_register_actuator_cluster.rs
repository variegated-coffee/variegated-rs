use variegated_board_apec::ApecR0dShiftRegisterPositions;
use variegated_controller_lib::{ActuatorCluster, ActuatorClusterError};
use crate::state::SilviaSystemActuatorState;

pub struct SilviaShiftRegisterOutputCluster<'a> {
    shift_register: variegated_embassy_dual_c595_shift_register::DualC595ShiftRegister<'a>,
}

impl<'a> SilviaShiftRegisterOutputCluster<'a> {
    pub fn new(shift_register: variegated_embassy_dual_c595_shift_register::DualC595ShiftRegister<'a>) -> SilviaShiftRegisterOutputCluster<'a> {
        SilviaShiftRegisterOutputCluster {
            shift_register
        }
    }
}

impl<'a> ActuatorCluster<SilviaSystemActuatorState> for SilviaShiftRegisterOutputCluster<'a> {
    async fn update_from_actuator_state(&mut self, system_actuator_state: &SilviaSystemActuatorState) -> Result<(), ActuatorClusterError>{
        let mut flags = ApecR0dShiftRegisterPositions::empty();
        
        if (system_actuator_state.brew_solenoid) {
            flags |= ApecR0dShiftRegisterPositions::CN2_FA7;
        }
        
        if (system_actuator_state.brew_boiler_heating_element.get_output()) {
            flags |= ApecR0dShiftRegisterPositions::CN10;
        }
        
        self.shift_register.write(flags.bits()).await;
        
        Ok(())
    }
}
