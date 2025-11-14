use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use alloc::collections::btree_map::BTreeMap;
use alloc::format;
use core::{fmt, iter};
use core::ops::{DerefMut, Range};
use defmt::{info, Format};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_storage_async::nor_flash::NorFlash;
use heapless::FnvIndexMap;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{fetch_all_items, remove_item, store_item, Key, SerializationError, Value};
use variegated_controller_types::{BoilerControlMode, BoilerControlTargetValuesUpdate, BoilerIndex, ControlCurve, FlowRateType, GroupBrewControlMode, GroupBrewControlTargetValuesUpdate, GroupIndex, InputVolumeType, MachineCommand, MAX_GROUPS, PidLimits, PidParameters, PidTerm, PressureType, RoutineIndex, Status, TemperatureType, WaterTapIndex, WeightType, UserActionIndex, DutyCycleType, ValveOpenType};

// Re-export types that are commonly used by consumers of this module
pub use variegated_controller_types::{
    Routine, RoutineParameter, ParameterUnit, RoutineType, RoutineStep,
    RoutineCommand, RoutineExit, RoutineExitCondition, StateCondition,
    ParameterValue, RoutineStepExitType, RoutineParameters, DerivedFormula
};

pub fn create_water_dispersal_routine(group: GroupIndex) -> Routine {
    let parameters = vec![
        RoutineParameter { 
            index: 0, 
            name: "Tgt Flow Rate".try_into().unwrap(),
            default: 2.0, 
            unit: Some(ParameterUnit::MillilitersPerSecond)
        },
        RoutineParameter { 
            index: 1, 
            name: "Water amt".try_into().unwrap(),
            default: 30.0, 
            unit: Some(ParameterUnit::Grams)
        },
    ];
    
    Routine {
        routine_type: RoutineType::UserDefined,
        name: "Water dispersal".try_into().unwrap(),
        parameters,
        derived_parameters: vec![], // No derived parameters for this routine
        steps: vec![
            // Step 0: Tare group scale
            RoutineStep {
                entry_command: vec![RoutineCommand::TareGroupScale(group)],
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::StateConditionMet(StateCondition::OutputWeightBelow(group, ParameterValue::Static(0.1))),
                    RoutineStepExitType::NextStep
                )],
                description: Some("Taring".try_into().unwrap()),
            },
            // Step 1: Set target to flow rate
            RoutineStep {
                entry_command: vec![RoutineCommand::SetGroupFixedDutyCycleWithTransition(group, ParameterValue::Static(50.0), ParameterValue::Static(5.0))],
//                entry_command: vec![RoutineCommand::SetGroupOutputFlowRateWithTransition(group, ParameterValue::Parameter(0), ParameterValue::Static(8.0))],
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::Always,
                    RoutineStepExitType::NextStep,
                )],
                description: Some("Setting target flow rate".try_into().unwrap()),
            },
            // Step 2: Start brewing, wait for the group to reach output weight above the specified amount
            RoutineStep {
                entry_command: vec![RoutineCommand::StartBrewing(group)],
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::StateConditionMet(StateCondition::OutputWeightAbove(group, ParameterValue::Parameter(1))),
                    RoutineStepExitType::NextStep
                )],
                description: Some("Dispensing water".try_into().unwrap()),
            },
            // Step 3: Stop brewing, then finish the routine
            RoutineStep {
                entry_command: vec![RoutineCommand::StopBrewing(group)],
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::Always,
                    RoutineStepExitType::Finished
                )],
                description: Some("Stopping water flow".try_into().unwrap()),
            },
        ],
        finally: vec![],
    }
}

pub struct RoutineExecutionContext<StateT, ConfigurationT> {
    pub(crate) routine_index: RoutineIndex,
    pub(crate) routine: Routine,
    pub(crate) currently_executing: bool,
    pub(crate) finished_executing: bool,
    pub(crate) current_step: Option<usize>,
    pub(crate) execution_start_time: Option<Instant>,
    pub(crate) step_start_time: Option<Instant>,
    pub(crate) parameters: RoutineParameters, // resolved parameters

    pub(crate) saved_state: StateT,
    pub(crate) saved_configuration: ConfigurationT,
}

impl<StateT, ConfigurationT> RoutineExecutionContext<StateT, ConfigurationT> {
    pub fn new(routine_index: RoutineIndex, routine: Routine, saved_state: StateT, saved_configuration: ConfigurationT, runtime_params: Option<RoutineParameters>) -> Self {
        // Merge runtime_params with defaults from routine.parameters
        let mut parameters = RoutineParameters::new();
        for param in &routine.parameters {
            let _ = parameters.insert(param.index, param.default);
        }
        if let Some(runtime) = runtime_params {
            for (idx, value) in runtime {
                let _ = parameters.insert(idx, value);
            }
        }
        
        Self {
            routine_index,
            routine,
            currently_executing: false,
            finished_executing: false,
            current_step: None,
            execution_start_time: None,
            step_start_time: None,
            parameters,
            saved_state,
            saved_configuration,
        }
    }

    fn resolve_value(&self, pv: &ParameterValue) -> f32 {
        match pv {
            ParameterValue::Static(v) => *v,
            ParameterValue::Parameter(idx) => {
                self.parameters.get(idx).copied().unwrap_or(0.0)
            }
            ParameterValue::DerivedParameter(idx) => {
                self.resolve_derived_parameter(*idx)
            }
        }
    }
    
    fn resolve_derived_parameter(&self, idx: u8) -> f32 {
        if let Some(derived) = self.routine.derived_parameters.iter()
            .find(|p| p.index == idx) {
            
            match &derived.formula {
                DerivedFormula::Linear { base_param, multiplier, offset } => {
                    let base_value = self.parameters.get(base_param).copied().unwrap_or(0.0);
                    base_value * multiplier + offset
                }
                DerivedFormula::Sum { params } => {
                    params.iter()
                        .map(|p| self.parameters.get(p).copied().unwrap_or(0.0))
                        .sum()
                }
                DerivedFormula::Difference { param_a, param_b } => {
                    let a = self.parameters.get(param_a).copied().unwrap_or(0.0);
                    let b = self.parameters.get(param_b).copied().unwrap_or(0.0);
                    a - b
                }
                DerivedFormula::Product { params } => {
                    params.iter()
                        .map(|p| self.parameters.get(p).copied().unwrap_or(0.0))
                        .product()
                }
            }
        } else {
            0.0
        }
    }
    
    fn create_linear_transition_curve(
        &self, 
        current_value: f32, 
        target_value: f32, 
        transition_time: f32
    ) -> ControlCurve {
        // Linear transition: value(t) = current + (target - current) * (t / transition_time)
        // Rearranged to curve form: value(t) = 0*t² + b*t + c
        // At t=0: value(0) = c = current_value
        // At t=transition_time: value(transition_time) = b*transition_time + c = target_value
        // Therefore: b = (target_value - current_value) / transition_time
        
        let b = (target_value - current_value) / transition_time;
        let c = current_value;
        
        // Limits should constrain the curve to the exact range between start and end
        // This prevents overshooting and ensures safety
        let min_val = current_value.min(target_value);
        let max_val = current_value.max(target_value);
        
        ControlCurve {
            a: 0.0,  // No quadratic term for linear transition
            b,       // Linear coefficient for smooth transition
            c,       // Starting value (current value)
            min: min_val, // Lower bound of transition range
            max: max_val, // Upper bound of transition range
        }
    }
    
    fn resolve_duration(&self, pv: &ParameterValue) -> Duration {
        let seconds = self.resolve_value(pv);
        Duration::from_millis((seconds * 1000.0) as u64)
    }
    
    fn resolve_command(&self, cmd: &RoutineCommand, status: &Status) -> MachineCommand {
        match cmd {
            RoutineCommand::StartBrewing(idx) => MachineCommand::StartBrewing(*idx),
            RoutineCommand::StopBrewing(idx) => MachineCommand::StopBrewing(*idx),
            RoutineCommand::TareGroupScale(idx) => MachineCommand::TareGroupScale(*idx),
            RoutineCommand::StartPumpingToWaterTap(idx) => MachineCommand::StartPumpingToWaterTap(*idx),
            RoutineCommand::StopPumpingToWaterTap(idx) => MachineCommand::StopPumpingToWaterTap(*idx),
            RoutineCommand::StartSteaming(idx) => MachineCommand::StartSteaming(*idx),
            RoutineCommand::StopSteaming(idx) => MachineCommand::StopSteaming(*idx),

            RoutineCommand::SetSteamValveOpenness(idx, pv) => {
                MachineCommand::SetSteamValveOpenness(*idx, self.resolve_value(pv) as ValveOpenType)
            }
            RoutineCommand::SetBoilerTemperature(idx, pv) => {
                MachineCommand::SetBoilerControlTarget(*idx,
                    BoilerControlMode::Temperature,
                    Some(BoilerControlTargetValuesUpdate {
                        temperature: Some(self.resolve_value(pv)),
                        pressure: None
                    }))
            }
            RoutineCommand::SetBoilerPressure(idx, pv) => {
                MachineCommand::SetBoilerControlTarget(*idx,
                    BoilerControlMode::Pressure,
                    Some(BoilerControlTargetValuesUpdate {
                        temperature: None,
                        pressure: Some(self.resolve_value(pv))
                    }))
            }
            RoutineCommand::SetGroupFlowRate(idx, pv) => {
                MachineCommand::SetGroupBrewControlTarget(*idx,
                    GroupBrewControlMode::GroupFlowRate,
                    Some(GroupBrewControlTargetValuesUpdate {
                        flow_rate: Some(self.resolve_value(pv)),
                        flow_rate_curve: None,
                        pressure: None,
                        pressure_curve: None,
                        output_flow_rate: None,
                        output_flow_rate_curve: None,
                        duty_cycle: None,
                        duty_cycle_curve: None
                    }))
            }
            RoutineCommand::SetGroupPressure(idx, pv) => {
                MachineCommand::SetGroupBrewControlTarget(*idx,
                    GroupBrewControlMode::Pressure,
                    Some(GroupBrewControlTargetValuesUpdate {
                        flow_rate: None,
                        flow_rate_curve: None,
                        pressure: Some(self.resolve_value(pv)),
                        pressure_curve: None,
                        output_flow_rate: None,
                        output_flow_rate_curve: None,
                        duty_cycle: None,
                        duty_cycle_curve: None
                    }))
            }
            RoutineCommand::SetGroupOutputFlowRate(idx, pv) => {
                MachineCommand::SetGroupBrewControlTarget(*idx,
                    GroupBrewControlMode::OutputFlowRate,
                    Some(GroupBrewControlTargetValuesUpdate {
                        flow_rate: None,
                        flow_rate_curve: None,
                        pressure: None,
                        pressure_curve: None,
                        output_flow_rate: Some(self.resolve_value(pv)),
                        output_flow_rate_curve: None,
                        duty_cycle: None,
                        duty_cycle_curve: None
                    }))
            }
            RoutineCommand::SetGroupFixedDutyCycle(idx, pv) => {
                MachineCommand::SetGroupBrewControlTarget(*idx,
                    GroupBrewControlMode::FixedDutyCycle,
                    Some(GroupBrewControlTargetValuesUpdate {
                        flow_rate: None,
                        flow_rate_curve: None,
                        pressure: None,
                        pressure_curve: None,
                        output_flow_rate: None,
                        output_flow_rate_curve: None,
                        duty_cycle: Some(self.resolve_value(pv) as u8),
                        duty_cycle_curve: None
                    }))
            }
            RoutineCommand::SetGroupFullOn(idx) => {
                MachineCommand::SetGroupBrewControlTarget(*idx, GroupBrewControlMode::FullOn, None)
            }
            RoutineCommand::SetGroupOff(idx) => {
                MachineCommand::SetGroupBrewControlTarget(*idx, GroupBrewControlMode::Off, None)
            }
            RoutineCommand::SetBoilerOff(idx) => {
                MachineCommand::SetBoilerControlTarget(*idx, BoilerControlMode::Off, None)
            }
            
            // Transition-enabled commands
            RoutineCommand::SetGroupFlowRateWithTransition(idx, target_pv, transition_pv) => {
                let target_value = self.resolve_value(target_pv);
                let transition_time = self.resolve_value(transition_pv);
                
                if transition_time <= 0.0 {
                    // No transition - use direct command
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlMode::GroupFlowRate,
                        Some(GroupBrewControlTargetValuesUpdate {
                            flow_rate: Some(target_value),
                            output_flow_rate: None,
                            pressure: None,
                            duty_cycle: None,
                            flow_rate_curve: None,
                            pressure_curve: None,
                            output_flow_rate_curve: None,
                            duty_cycle_curve: None
                        }))
                } else {
                    // Create linear transition curve from current value
                    let current_value = status.get_group_status(*idx)
                        .and_then(|gs| gs.input_flow_rate)
                        .unwrap_or(0.0);
                    let curve = self.create_linear_transition_curve(
                        current_value, target_value, transition_time
                    );
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlMode::GroupFlowRateCurve,
                        Some(GroupBrewControlTargetValuesUpdate {
                            flow_rate: None,
                            flow_rate_curve: Some(curve),
                            pressure: None,
                            pressure_curve: None,
                            output_flow_rate: None,
                            output_flow_rate_curve: None,
                            duty_cycle: None,
                            duty_cycle_curve: None
                        }))
                }
            }
            
            RoutineCommand::SetGroupPressureWithTransition(idx, target_pv, transition_pv) => {
                let target_value = self.resolve_value(target_pv);
                let transition_time = self.resolve_value(transition_pv);
                
                if transition_time <= 0.0 {
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlMode::Pressure,
                        Some(GroupBrewControlTargetValuesUpdate {
                            flow_rate: None,
                            output_flow_rate: None,
                            pressure: Some(target_value),
                            duty_cycle: None,
                            flow_rate_curve: None,
                            pressure_curve: None,
                            output_flow_rate_curve: None,
                            duty_cycle_curve: None
                        }))
                } else {
                    let current_value = status.get_group_status(*idx)
                        .and_then(|gs| gs.pressure)
                        .unwrap_or(0.0);
                    let curve = self.create_linear_transition_curve(
                        current_value, target_value, transition_time
                    );
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlMode::PressureCurve,
                        Some(GroupBrewControlTargetValuesUpdate {
                            flow_rate: None,
                            flow_rate_curve: None,
                            pressure: None,
                            pressure_curve: Some(curve),
                            output_flow_rate: None,
                            output_flow_rate_curve: None,
                            duty_cycle: None,
                            duty_cycle_curve: None
                        }))
                }
            }
            
            RoutineCommand::SetGroupOutputFlowRateWithTransition(idx, target_pv, transition_pv) => {
                let target_value = self.resolve_value(target_pv);
                let transition_time = self.resolve_value(transition_pv);
                
                if transition_time <= 0.0 {
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlMode::OutputFlowRate,
                        Some(GroupBrewControlTargetValuesUpdate {
                            flow_rate: None,
                            output_flow_rate: Some(target_value),
                            pressure: None,
                            duty_cycle: None,
                            flow_rate_curve: None,
                            pressure_curve: None,
                            output_flow_rate_curve: None,
                            duty_cycle_curve: None
                        }))
                } else {
                    let current_value = status.get_group_status(*idx)
                        .and_then(|gs| gs.output_flow_rate)
                        .unwrap_or(0.0);
                    let curve = self.create_linear_transition_curve(
                        current_value, target_value, transition_time
                    );
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlMode::OutputFlowRateCurve,
                        Some(GroupBrewControlTargetValuesUpdate {
                            flow_rate: None,
                            flow_rate_curve: None,
                            pressure: None,
                            pressure_curve: None,
                            output_flow_rate: None,
                            output_flow_rate_curve: Some(curve),
                            duty_cycle: None,
                            duty_cycle_curve: None
                        }))
                }
            }
            
            RoutineCommand::SetGroupFixedDutyCycleWithTransition(idx, target_pv, transition_pv) => {
                let target_value = self.resolve_value(target_pv);
                let transition_time = self.resolve_value(transition_pv);
                
                if transition_time <= 0.0 {
                    // No transition - use direct command
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlMode::FixedDutyCycle,
                        Some(GroupBrewControlTargetValuesUpdate {
                            flow_rate: None,
                            output_flow_rate: None,
                            pressure: None,
                            duty_cycle: Some(target_value as u8),
                            flow_rate_curve: None,
                            pressure_curve: None,
                            output_flow_rate_curve: None,
                            duty_cycle_curve: None
                        }))
                } else {
                    // Create linear transition curve from current duty cycle
                    let current_value = status.get_group_status(*idx)
                        .map(|gs| gs.pump_output.duty_cycle() as f32)
                        .unwrap_or(0.0);
                    let curve = self.create_linear_transition_curve(
                        current_value, target_value, transition_time
                    );
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlMode::FixedDutyCycleCurve,
                        Some(GroupBrewControlTargetValuesUpdate {
                            flow_rate: None,
                            flow_rate_curve: None,
                            pressure: None,
                            pressure_curve: None,
                            output_flow_rate: None,
                            output_flow_rate_curve: None,
                            duty_cycle: None,
                            duty_cycle_curve: Some(curve)
                        }))
                }
            }

            // Bumpless transfer commands
            RoutineCommand::InferGroupPressureIntegral(idx, pv) => {
                MachineCommand::InferGroupPressureIntegral(*idx, self.resolve_value(pv))
            }
            RoutineCommand::InferGroupFlowRateIntegral(idx, pv) => {
                MachineCommand::InferGroupFlowRateIntegral(*idx, self.resolve_value(pv))
            }
            RoutineCommand::InferGroupOutputFlowRateIntegral(idx, pv) => {
                MachineCommand::InferGroupOutputFlowRateIntegral(*idx, self.resolve_value(pv))
            }
        }
    }

    /// Get resolved finally commands to execute when routine exits
    pub fn finally(&self, status: &Status) -> Vec<MachineCommand> {
        self.routine.finally
            .iter()
            .map(|cmd| self.resolve_command(cmd, status))
            .collect()
    }

    pub fn step(&mut self, status: &Status, user_action: Option<UserActionIndex>) -> Vec<MachineCommand> {
        if self.finished_executing {
            return vec![]; // Routine has finished executing
        }

        if self.current_step.is_none() {
            self.currently_executing = true;
            self.execution_start_time = Some(Instant::now());
            return self.transition_to(0, status);
        }

        // Check exit conditions
        let exits = self.routine.steps[self.current_step.unwrap()].exits.clone();
        for exit in &exits {
            match exit.condition {
                RoutineExitCondition::Always => {
                    return self.handle_exit(exit, status);
                }
                RoutineExitCondition::Never => continue,
                RoutineExitCondition::After(pv) => {
                    let duration = self.resolve_duration(&pv);
                    if self.step_start_time.expect("Step start time is None - Shouldn't happen").elapsed() >= duration {
                        return self.handle_exit(exit, status);
                    }
                }
                RoutineExitCondition::AfterDurationRelativeToStart(pv) => {
                    let duration = self.resolve_duration(&pv);
                    if self.execution_start_time.expect("Execution start time is None - Shouldn't happen").elapsed() >= duration {
                        return self.handle_exit(exit, status);
                    }
                }
                RoutineExitCondition::StateConditionMet(condition) => {
                    if self.state_condition_met(condition, status) {
                        info!("State condition met: {:?}", condition);
                        return self.handle_exit(exit, status);
                    }
                }
                RoutineExitCondition::UserAction(action_index) => {
                    if let Some(user_action_index) = user_action {
                        if user_action_index == action_index {
                            info!("User action condition met: {:?}", action_index);
                            return self.handle_exit(exit, status);
                        }
                    }
                }
            }

        }

        vec![]
    }

    fn handle_exit(&mut self, exit: &RoutineExit, status: &Status) -> Vec<MachineCommand> {
        match exit.then {
            RoutineStepExitType::NextStep => {
                self.transition_to(self.current_step.unwrap() + 1, status)
            }
            RoutineStepExitType::JumpToStep(step) => {
                self.transition_to(step, status)
            }
            RoutineStepExitType::Finished => {
                self.currently_executing = false;
                self.finished_executing = true;
                self.current_step = None;
                self.execution_start_time = None;
                self.step_start_time = None;

                vec![]
            }
        }
    }

    pub fn transition_to(&mut self, step: usize, status: &Status) -> Vec<MachineCommand> {
        info!("Transitioning to step {}", step);
        self.current_step = Some(step);
        self.step_start_time = Some(Instant::now());
        self.routine.steps[step].entry_command.iter()
            .map(|cmd| self.resolve_command(cmd, status))
            .collect()
    }

    fn state_condition_met(&self, state_condition: StateCondition, status: &Status) -> bool {
        match state_condition {
            StateCondition::Brewing(idx) => status.get_group_status(idx).map_or(false, |s| s.is_brewing),
            StateCondition::NotBrewing(idx) => status.get_group_status(idx).map_or(false, |s| !s.is_brewing),
            StateCondition::BoilerTemperatureAbove(idx, pv) => {
                let threshold = self.resolve_value(&pv);
                status.get_boiler_status(idx).map_or(false, |s| s.temperature.unwrap_or(0.0) > threshold)
            }
            StateCondition::BoilerTemperatureBelow(idx, pv) => {
                let threshold = self.resolve_value(&pv);
                status.get_boiler_status(idx).map_or(false, |s| s.temperature.unwrap_or(0.0) < threshold)
            }
            StateCondition::BoilerPressureAbove(idx, pv) => {
                let threshold = self.resolve_value(&pv);
                status.get_boiler_status(idx).map_or(false, |s| s.pressure.unwrap_or(0.0) > threshold)
            }
            StateCondition::BoilerPressureBelow(idx, pv) => {
                let threshold = self.resolve_value(&pv);
                status.get_boiler_status(idx).map_or(false, |s| s.pressure.unwrap_or(0.0) < threshold)
            }
            StateCondition::GroupInputFlowRateAbove(idx, pv) => {
                let threshold = self.resolve_value(&pv);
                status.get_group_status(idx).map_or(false, |s| s.input_flow_rate.unwrap_or(0.0) > threshold)
            }
            StateCondition::GroupInputFlowRateBelow(idx, pv) => {
                let threshold = self.resolve_value(&pv);
                status.get_group_status(idx).map_or(false, |s| s.input_flow_rate.unwrap_or(0.0) < threshold)
            }
            StateCondition::GroupPressureAbove(idx, pv) => {
                let threshold = self.resolve_value(&pv);
                status.get_group_status(idx).map_or(false, |s| s.pressure.unwrap_or(0.0) > threshold)
            }
            StateCondition::GroupPressureBelow(idx, pv) => {
                let threshold = self.resolve_value(&pv);
                status.get_group_status(idx).map_or(false, |s| s.pressure.unwrap_or(0.0) < threshold)
            }
            StateCondition::WaterTapFlowRateAbove(_idx, _pv) => {
                // WaterTapStatus doesn't currently track flow rate
                false
            }
            StateCondition::WaterTapFlowRateBelow(_idx, _pv) => {
                // WaterTapStatus doesn't currently track flow rate
                false
            }
            StateCondition::OutputWeightAbove(idx, pv) => {
                let threshold = self.resolve_value(&pv);
                status.get_group_status(idx).map_or(false, |s| s.output_weight.unwrap_or(0.0) > threshold)
            }
            StateCondition::OutputWeightBelow(idx, pv) => {
                let threshold = self.resolve_value(&pv);
                status.get_group_status(idx).map_or(false, |s| s.output_weight.unwrap_or(0.0) < threshold)
            }
            StateCondition::InputVolumeAboveRelativeToStart(idx, pv) => {
                let threshold = self.resolve_value(&pv);
                status.get_group_status(idx).map_or(false, |s| {
                    s.brew_input_volume.map_or(false, |volume| volume > threshold as f64)
                })
            }
        }
    }
}


pub trait RoutineRepository {
    /// Get a routine by its index. Returns None if the routine doesn't exist.
    async fn get_routine(&mut self, index: RoutineIndex) -> Option<&Routine>;

    /// Add a new routine. Always assigns a Custom variant index, using the first available slot.
    async fn add_routine(&mut self, routine: Routine);

    /// Add an internal routine at a specific Internal index. Internal routines are never persisted to flash.
    /// Returns an error if the index is not an Internal variant.
    async fn add_internal_routine(&mut self, index: RoutineIndex, routine: Routine) -> Result<(), &'static str>;

    /// Remove a routine by its index. Returns the removed routine, or None if it doesn't exist.
    async fn remove_routine(&mut self, index: RoutineIndex) -> Option<Routine>;

    /// Update or create a routine at the specified index (works for any variant: Internal, Function, or Custom).
    async fn update_routine(&mut self, index: RoutineIndex, routine: Routine) -> Result<(), &'static str>;

    /// Iterate over all routines in the repository.
    async fn iterate_routines(&mut self) -> impl Iterator<Item = &Routine>;

    /// Iterate over all routines with their indices.
    async fn iterate_routines_with_indices(&mut self) -> impl Iterator<Item = (RoutineIndex, &Routine)>;

    /// Get the total count of routines.
    async fn get_routine_count(&mut self) -> usize;

    /// Optimize storage by erasing and rewriting all routines (flash storage only).
    async fn optimize_storage(&mut self) -> Result<(), &'static str>;
}

pub struct SequentialStorageRoutineRepository<'a, M: RawMutex, T: NorFlash> {
    flash: &'a Mutex<M, T>,
    range: Range<u32>,
    deserialization_buffer: [u8; 2048],
    cache: BTreeMap<u16, Routine>,
    cache_initialized: bool
}

impl <'a, M: RawMutex, T: NorFlash> SequentialStorageRoutineRepository<'a, M, T> {
    pub fn new(flash: &'a Mutex<M, T>, range: Range<u32>) -> Self {
        Self {
            flash,
            range,
            deserialization_buffer: [0u8; 2048],
            cache: BTreeMap::new(),
            cache_initialized: false
        }
    }

    async fn load_from_flash(&mut self) -> Result<(), &'static str> {
        if self.cache_initialized {
            //info!("Cache already initialized, skipping load");
            return Ok(());
        }

        let mut cache = NoCache::new();

        let mut guard = self.flash.lock().await;
        let flash_ref = guard.deref_mut();

        info!("Loading routines from flash...");
        // Create the iterator of map items
        let mut iterator = fetch_all_items::<u16, _, _>(
            flash_ref,
            self.range.clone(),
            &mut cache,
            &mut self.deserialization_buffer
        )
        .await
        .unwrap();

        while let Some((key, value)) = iterator
            .next::<Option<Routine>>(&mut self.deserialization_buffer)
            .await
            .unwrap()
        {
            // Skip Internal routines - they are never persisted to flash
            if let Some(idx) = RoutineIndex::from_storage_index(key) {
                if matches!(idx, RoutineIndex::Internal(_)) {
                    info!("Skipping internal routine at storage index {} during flash load", key);
                    continue;
                }
            }

            info!("Loaded routine at index {}: {:?}", key, value);
            if let Some(routine) = value {
                self.cache.insert(key, routine);
            } else {
                self.cache.remove(&key);
            }
        }

        self.cache_initialized = true;

        Ok(())
    }

    async fn store_in_flash(&mut self, index: u16, routine: &Option<Routine>) -> Result<(), &'static str> {
        let mut guard = self.flash.lock().await;
        let flash_ref = guard.deref_mut();

        let mut cache = NoCache::new();

        let key = index;

        store_item(
            flash_ref,
            self.range.clone(),
            &mut cache,
            &mut self.deserialization_buffer,
            &key,
            routine
        ).await
            .map_err(|_| "Failed to store item in flash")?;

        // @todo Handle full storage by erasing the range and rewriting all items

        info!("Stored routine at index {} in flash", index);

        Ok(())
    }
}

impl <'a, M: RawMutex, T: NorFlash> RoutineRepository for SequentialStorageRoutineRepository<'a, M, T> {
    async fn get_routine(&mut self, index: RoutineIndex) -> Option<&Routine> {
        info!("Getting routine at index {:?}", index);
        self.load_from_flash().await.ok()?;
        let storage_index = index.to_storage_index();
        self.cache.get(&storage_index)
    }

    async fn add_routine(&mut self, routine: Routine) {
        //info!("Adding new routine");
        self.load_from_flash().await.ok().unwrap();

        // Find first available Custom index
        let mut inner_index = 0usize;
        loop {
            let test_index = RoutineIndex::Custom(inner_index);
            let storage_index = test_index.to_storage_index();
            if !self.cache.contains_key(&storage_index) {
                break;
            }
            inner_index += 1;
        }

        let routine_index = RoutineIndex::Custom(inner_index);
        let storage_index = routine_index.to_storage_index();
        let opt = Some(routine);
        self.store_in_flash(storage_index, &opt).await.expect("Failed to store routine in flash");
        self.cache.insert(storage_index, opt.unwrap());
    }

    async fn add_internal_routine(&mut self, index: RoutineIndex, routine: Routine) -> Result<(), &'static str> {
        // Validate that the index is Internal variant
        if !matches!(index, RoutineIndex::Internal(_)) {
            return Err("add_internal_routine requires an Internal variant index");
        }

        let storage_index = index.to_storage_index();

        // Add to cache only, never write to flash
        self.cache.insert(storage_index, routine);

        info!("Added internal routine at index {:?} (not persisted to flash)", index);
        Ok(())
    }

    async fn remove_routine(&mut self, index: RoutineIndex) -> Option<Routine> {
        // Prevent removal of Internal routines (they are read-only)
        if matches!(index, RoutineIndex::Internal(_)) {
            info!("Cannot remove internal routine at index {:?}", index);
            return None;
        }

        let storage_index = index.to_storage_index();
        let routine = self.cache.remove(&storage_index);
        if routine.is_some() {
            let opt: Option<Routine> = None;
            let _ = self.store_in_flash(storage_index, &opt).await;
        }

        routine
    }

    async fn update_routine(&mut self, index: RoutineIndex, routine: Routine) -> Result<(), &'static str> {
        //info!("Updating routine at index {:?}", index);

        // Prevent updating Internal routines (they are read-only)
        if matches!(index, RoutineIndex::Internal(_)) {
            return Err("Cannot update internal routine - they are read-only");
        }

        self.load_from_flash().await?;

        let storage_index = index.to_storage_index();
        // For update, we allow creating new routines (not just updating existing ones)
        let opt = Some(routine);
        self.store_in_flash(storage_index, &opt).await?;
        self.cache.insert(storage_index, opt.unwrap());
        Ok(())
    }

    async fn iterate_routines(&mut self) -> impl Iterator<Item = &Routine> {
        //info!("Iterating over all routines");
        let res = self.load_from_flash().await;
        if res.is_err() {
            info!("Error loading routines from flash: {:?}", res.err());
        }

        self.cache.values()
    }

    async fn iterate_routines_with_indices(&mut self) -> impl Iterator<Item = (RoutineIndex, &Routine)> {
        //info!("Iterating over all routines with indices");
        let res = self.load_from_flash().await;
        if res.is_err() {
            info!("Error loading routines from flash: {:?}", res.err());
        }

        self.cache.iter().filter_map(|(storage_index, routine)| {
            RoutineIndex::from_storage_index(*storage_index).map(|idx| (idx, routine))
        })
    }

    async fn get_routine_count(&mut self) -> usize {
        //info!("Getting routine count");
        if let Err(e) = self.load_from_flash().await {
            info!("Error loading routines from flash: {:?}", e);
            return 0;
        }

        self.cache.len()
    }

    async fn optimize_storage(&mut self) -> Result<(), &'static str> {
        info!("Optimizing routine storage");

        // Load all routines into cache if not already loaded
        self.load_from_flash().await?;

        // Collect routines to re-store, excluding Internal routines (to avoid borrowing issues)
        let routines_to_store: Vec<(u16, Routine)> = self.cache.iter()
            .filter(|(storage_index, _)| {
                // Filter out Internal routines - they should never be written to flash
                if let Some(idx) = RoutineIndex::from_storage_index(**storage_index) {
                    !matches!(idx, RoutineIndex::Internal(_))
                } else {
                    true // Keep routines that can't be decoded (shouldn't happen)
                }
            })
            .map(|(index, routine)| (*index, routine.clone()))
            .collect();

        // Erase the entire flash range
        {
            let mut flash = self.flash.lock().await;
            info!("Erasing routine storage range");
            flash.erase(self.range.start, self.range.end).await
                .map_err(|_| "Failed to erase flash range")?;
        }

        // Yield to allow other tasks (like watchdog feeding) to run after long erase operation
        Timer::after_millis(1).await;

        // Re-store all routines from the collected Vec (Internal routines already filtered out)
        info!("Rewriting {} routines (excluding internal routines)", routines_to_store.len());
        for (storage_index, routine) in routines_to_store {
            let opt = Some(routine);
            self.store_in_flash(storage_index, &opt).await?;
            // Yield after each routine to prevent watchdog timeout
            Timer::after_millis(1).await;
        }

        info!("Routine storage optimization complete");
        Ok(())
    }
}

pub struct InMemoryRoutineRepository {
    routines: BTreeMap<u16, Routine>,
}

impl InMemoryRoutineRepository {
    pub fn new() -> Self {
        Self {
            routines: BTreeMap::new(),
        }
    }
}

impl RoutineRepository for InMemoryRoutineRepository {
    async fn get_routine(&mut self, index: RoutineIndex) -> Option<&Routine> {
        let storage_index = index.to_storage_index();
        self.routines.get(&storage_index)
    }

    async fn add_routine(&mut self, routine: Routine) {
        // Find first available Custom index
        let mut inner_index = 0usize;
        loop {
            let test_index = RoutineIndex::Custom(inner_index);
            let storage_index = test_index.to_storage_index();
            if !self.routines.contains_key(&storage_index) {
                break;
            }
            inner_index += 1;
        }

        let routine_index = RoutineIndex::Custom(inner_index);
        let storage_index = routine_index.to_storage_index();
        self.routines.insert(storage_index, routine);
    }

    async fn add_internal_routine(&mut self, index: RoutineIndex, routine: Routine) -> Result<(), &'static str> {
        // Validate that the index is Internal variant
        if !matches!(index, RoutineIndex::Internal(_)) {
            return Err("add_internal_routine requires an Internal variant index");
        }

        let storage_index = index.to_storage_index();
        self.routines.insert(storage_index, routine);
        Ok(())
    }

    async fn remove_routine(&mut self, index: RoutineIndex) -> Option<Routine> {
        // Prevent removal of Internal routines (they are read-only)
        if matches!(index, RoutineIndex::Internal(_)) {
            return None;
        }

        let storage_index = index.to_storage_index();
        self.routines.remove(&storage_index)
    }

    async fn update_routine(&mut self, index: RoutineIndex, routine: Routine) -> Result<(), &'static str> {
        // Prevent updating Internal routines (they are read-only)
        if matches!(index, RoutineIndex::Internal(_)) {
            return Err("Cannot update internal routine - they are read-only");
        }

        let storage_index = index.to_storage_index();
        // For update, we allow creating new routines (not just updating existing ones)
        self.routines.insert(storage_index, routine);
        Ok(())
    }

    async fn iterate_routines(&mut self) -> impl Iterator<Item = &Routine> {
        self.routines.values()
    }

    async fn iterate_routines_with_indices(&mut self) -> impl Iterator<Item = (RoutineIndex, &Routine)> {
        self.routines.iter().filter_map(|(storage_index, routine)| {
            RoutineIndex::from_storage_index(*storage_index).map(|idx| (idx, routine))
        })
    }

    async fn get_routine_count(&mut self) -> usize {
        self.routines.len()
    }

    async fn optimize_storage(&mut self) -> Result<(), &'static str> {
        // InMemory storage doesn't need optimization
        Ok(())
    }
}


pub fn create_shot_routine(group: GroupIndex) -> Routine {
    Routine {
        routine_type: RoutineType::UserDefined,
        name: "Smart shot".try_into().unwrap(),
        parameters: vec![
            RoutineParameter {
                index: 0,
                name: "Preinf. Time".try_into().unwrap(),
                default: 5.0,
                unit: Some(ParameterUnit::Seconds)
            },
            RoutineParameter {
                index: 1,
                name: "Brew Weight".try_into().unwrap(),
                default: 50.0,
                unit: Some(ParameterUnit::Grams)
            },
            RoutineParameter {
                index: 2,
                name: "Tgt Press".try_into().unwrap(),
                default: 8.0,
                unit: Some(ParameterUnit::Bar)
            },
            RoutineParameter {
                index: 3,
                name: "Resc Trigger".try_into().unwrap(),
                default: 2.5,
                unit: Some(ParameterUnit::MillilitersPerSecond)
            },
            RoutineParameter {
                index: 4,
                name: "Rescue Flow".try_into().unwrap(),
                default: 1.5,
                unit: Some(ParameterUnit::MillilitersPerSecond)
            },
        ],
        derived_parameters: vec![], // No derived parameters for this routine
        steps: vec![
            // Step 0
            RoutineStep {
                entry_command: vec![RoutineCommand::SetGroupFullOn(group)],
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::Always,
                    RoutineStepExitType::NextStep
                )],
                description: None,
            },
            // Step 1/2: Start filling at FullOn for 1 second (to avoid swings), then until pressure is above 2.0 bar (where the grouphead is filled)
            RoutineStep {
                entry_command: vec![RoutineCommand::StartBrewing(group)],
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::After(ParameterValue::Static(1.0)),
                    RoutineStepExitType::NextStep
                )],
                description: Some("Fast fill".try_into().unwrap()),
            },
            RoutineStep {
                entry_command: vec![],
                exits: vec![RoutineExit::with_description(
                    RoutineExitCondition::StateConditionMet(StateCondition::BoilerPressureAbove(group, ParameterValue::Static(2.0))),
                    RoutineStepExitType::NextStep,
                    "Group filled?".try_into().unwrap()
                )],
                description: Some("Fast fill".try_into().unwrap()),
            },
            // Step 3: Set pump to Off, then wait for preinfusion time
            RoutineStep {
                entry_command: vec![RoutineCommand::SetGroupOff(group)],
                exits: vec![RoutineExit::with_description(
                    RoutineExitCondition::After(ParameterValue::Parameter(0)),
                    RoutineStepExitType::NextStep,
                    "Pre-infusing".try_into().unwrap()
                )],
                description: Some("Pre-infusion".try_into().unwrap()),
            },
            // Step 4: Set pressure target to pressure, keep going for 4 seconds (to allow the pressure and flow to stabilize)
            RoutineStep {
                entry_command: vec![RoutineCommand::SetGroupPressure(group, ParameterValue::Parameter(2))],
                exits: vec![
                    RoutineExit::with_description(
                        RoutineExitCondition::After(ParameterValue::Static(2.0)),
                        RoutineStepExitType::NextStep,
                        "Stabilizing".try_into().unwrap()
                    )
                ],
                description: Some("Ramping to pressure".try_into().unwrap()),
            },
            // Step 5/6: Keep going at pressure for a total of total_brew_time seconds. If the flow rate is above rescue_trigger, switch to control by flow rate at rescue_flow_rate.
            RoutineStep {
                entry_command: vec![],
                exits: vec![
                    RoutineExit::with_description(
                        RoutineExitCondition::StateConditionMet(StateCondition::OutputWeightAbove(group, ParameterValue::Parameter(1))),
                        RoutineStepExitType::JumpToStep(7),
                        "Brew to weight".try_into().unwrap()
                    ),
                    RoutineExit::with_description(
                        RoutineExitCondition::StateConditionMet(StateCondition::GroupInputFlowRateAbove(group, ParameterValue::Parameter(3))),
                        RoutineStepExitType::NextStep,
                        "Shot rescue".try_into().unwrap()
                    )
                ],
                description: Some("Brewing".try_into().unwrap()),
            },
            RoutineStep {
                entry_command: vec![RoutineCommand::SetGroupFlowRate(group, ParameterValue::Parameter(4))],
                exits: vec![
                    RoutineExit::with_description(
                        RoutineExitCondition::StateConditionMet(StateCondition::OutputWeightAbove(group, ParameterValue::Parameter(1))),
                        RoutineStepExitType::NextStep,
                        "Brew to weight".try_into().unwrap()
                    ),
                ],
                description: Some("Shot rescue".try_into().unwrap()),
            },
            // Step 7: Stop brewing, then finish the routine
            RoutineStep {
                entry_command: vec![RoutineCommand::StopBrewing(group)],
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::Never,
                    RoutineStepExitType::Finished,
                )],
                description: Some("Finishing extraction".try_into().unwrap()),
            },
        ],
        finally: vec![],
    }
}

pub fn create_heatup_routine(boiler_index: BoilerIndex) -> Routine {
    let parameters = vec![
    ];

    Routine {
        routine_type: RoutineType::HeatUp,
        name: "Heat-up".try_into().unwrap(),
        parameters,
        derived_parameters: vec![], // No derived parameters for this routine
        steps: vec![
            RoutineStep {
                entry_command: vec![RoutineCommand::SetBoilerTemperature(boiler_index, ParameterValue::Parameter(0))],
                exits: vec![ RoutineExit::new(
                    RoutineExitCondition::StateConditionMet(StateCondition::BoilerTemperatureAbove(boiler_index, ParameterValue::Static(120.0))),
                    RoutineStepExitType::NextStep,
                )],
                description: Some("Heating to overshoot".try_into().unwrap()),
            },
            RoutineStep {
                entry_command: vec![],
                exits: vec![ RoutineExit::with_description(
                    RoutineExitCondition::After(ParameterValue::Static(300.0)),
                    RoutineStepExitType::NextStep,
                    "Waiting".try_into().unwrap()
                )],
                description: Some("Stabilizing temperature".try_into().unwrap()),
            },
            RoutineStep {
                entry_command: vec![RoutineCommand::SetBoilerTemperature(boiler_index, ParameterValue::Static(95.0))],
                exits: vec![ RoutineExit::with_description(
                    RoutineExitCondition::StateConditionMet(StateCondition::BoilerTemperatureBelow(boiler_index, ParameterValue::Static(96.0))),
                    RoutineStepExitType::Finished,
                    "Waiting".try_into().unwrap()
                )],
                description: Some("Adjusting to target".try_into().unwrap()),
            },
        ],
        finally: vec![],
    }
}

pub fn create_volumetric_shot_routine(group: GroupIndex, milliliters: f32, bloom_after: Option<Duration>, bloom_time: Option<Duration>, name: Option<String>) -> Routine {
    let mut steps = vec![
        RoutineStep {
            entry_command: vec![RoutineCommand::SetGroupFullOn(group)],
            exits: vec![RoutineExit::new(
                RoutineExitCondition::Always,
                RoutineStepExitType::NextStep
            )],
            description: None,
        },
        RoutineStep {
            entry_command: vec![RoutineCommand::StartBrewing(group)],
            exits: vec![RoutineExit::new(
                RoutineExitCondition::Always,
                RoutineStepExitType::NextStep
            )],
            description: Some("Filling".try_into().unwrap()),
        },
    ];

    if let (Some(bloom_after), Some(bloom_time)) = (bloom_after, bloom_time) {
        steps.push(RoutineStep {
            entry_command: vec![],
            exits: vec![RoutineExit::new(
                RoutineExitCondition::After(ParameterValue::Static(bloom_after.as_millis() as f32 / 1000.0)),
                RoutineStepExitType::NextStep
            )],
            description: Some("Filling".try_into().unwrap()),
        });
        steps.push(RoutineStep {
            entry_command: vec![RoutineCommand::SetGroupOff(group)],
            exits: vec![RoutineExit::with_description(
                RoutineExitCondition::After(ParameterValue::Static(bloom_after.as_millis() as f32 / 1000.0)),
                RoutineStepExitType::NextStep,
                "Blooming".try_into().unwrap()
            )],
            description: Some("Blooming".try_into().unwrap()),
        });
        steps.push(RoutineStep {
            entry_command: vec![RoutineCommand::SetGroupFullOn(group)],
            exits: vec![RoutineExit::new(
                RoutineExitCondition::Always,
                RoutineStepExitType::NextStep
            )],
            description: Some("Brewing".try_into().unwrap()),
        });
    }

    steps.push(RoutineStep {
        entry_command: vec![],
        exits: vec![RoutineExit::with_description(
            RoutineExitCondition::StateConditionMet(StateCondition::InputVolumeAboveRelativeToStart(group, ParameterValue::Static(milliliters))),
            RoutineStepExitType::NextStep,
            "Reached target volume".try_into().unwrap()
        )],
        description: Some("Brewing".try_into().unwrap()),
    });

    steps.push(RoutineStep {
        entry_command: vec![RoutineCommand::StopBrewing(group)],
        exits: vec![RoutineExit::with_description(
            RoutineExitCondition::Always,
            RoutineStepExitType::Finished,
            "Reached target volume".try_into().unwrap()
        )],
        description: Some("Brewing".try_into().unwrap()),
    });

    let routine_name = name.unwrap_or_else(|| "Volumetric shot".into());

    Routine {
        routine_type: RoutineType::UserDefined,
        name: routine_name,
        parameters: vec![],
        derived_parameters: vec![], // No derived parameters for this routine
        steps,
        finally: vec![],
    }
}

pub fn create_backflush_routine(group: GroupIndex, pump_duty_cycle: DutyCycleType) -> Routine {
    const PUMP_ON_TIME: f32 = 4.0; // seconds
    const PUMP_OFF_TIME: f32 = 5.0; // seconds
    const NUM_CYCLES: usize = 5;

    let mut steps = vec![
        // Step 0: Initialize - set group to full power
        RoutineStep {
            entry_command: vec![RoutineCommand::SetGroupFixedDutyCycle(group, ParameterValue::Static(pump_duty_cycle.into()))],
            exits: vec![RoutineExit::new(
                RoutineExitCondition::Always,
                RoutineStepExitType::NextStep
            )],
            description: Some("Initializing".try_into().unwrap()),
        },
    ];

    // Generate 10 cycles of pump on/off
    for cycle in 0..NUM_CYCLES {
        let is_last_cycle = cycle == NUM_CYCLES - 1;

        // Pump ON step
        steps.push(RoutineStep {
            entry_command: vec![RoutineCommand::StartBrewing(group)],
            exits: vec![RoutineExit::new(
                RoutineExitCondition::After(ParameterValue::Static(PUMP_ON_TIME)),
                RoutineStepExitType::NextStep
            )],
            description: Some(format!("Backflush cycle {} - pump on", cycle + 1).try_into().unwrap()),
        });

        // Pump OFF step
        let exit_type = if is_last_cycle {
            RoutineStepExitType::Finished
        } else {
            RoutineStepExitType::NextStep
        };

        steps.push(RoutineStep {
            entry_command: vec![RoutineCommand::StopBrewing(group)],
            exits: vec![RoutineExit::new(
                RoutineExitCondition::After(ParameterValue::Static(PUMP_OFF_TIME)),
                exit_type
            )],
            description: Some(format!("Backflush cycle {} - pump off", cycle + 1).try_into().unwrap()),
        });
    }

    Routine {
        routine_type: RoutineType::Cleaning,
        name: "Backflush".try_into().unwrap(),
        parameters: vec![],
        derived_parameters: vec![],
        steps,
        finally: vec![RoutineCommand::StopBrewing(group)],
    }
}