extern crate alloc;

use alloc::boxed::Box;
use async_trait::async_trait;
use variegated_controller_lib::{ActuatorCluster, ActuatorClusterError};
use crate::SilviaSystemActuatorState;

pub struct SilviaShiftRegisterOutputCluster<'a> {
    shift_register: variegated_embassy_dual_c595_shift_register::DualC595ShiftRegister<'a>
}

impl<'a> SilviaShiftRegisterOutputCluster<'a> {
    pub fn new(shift_register: variegated_embassy_dual_c595_shift_register::DualC595ShiftRegister<'a>) -> SilviaShiftRegisterOutputCluster<'a> {
        SilviaShiftRegisterOutputCluster {
            shift_register
        }
    }
}

#[async_trait]
impl<'a> ActuatorCluster<SilviaSystemActuatorState> for SilviaShiftRegisterOutputCluster<'a> {
    async fn update_from_actuator_state(&mut self, system_actuator_state: &SilviaSystemActuatorState) -> Result<(), ActuatorClusterError>{
        // @fixme Set the correct shift register positions
        if system_actuator_state.brew_solenoid {
            self.shift_register.write(1 << 0).await;
        } else {
            self.shift_register.write(0).await;
        }
        
        Ok(())
    }
}
