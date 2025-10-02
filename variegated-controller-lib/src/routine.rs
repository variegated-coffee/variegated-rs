use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use alloc::format;
use core::fmt;
use defmt::{info, Format};
use embassy_time::{Duration, Instant};
use heapless::FnvIndexMap;
use variegated_controller_types::{BoilerControlMode, BoilerControlTargetValuesUpdate, BoilerIndex, ControlCurve, FlowRateType, GroupBrewControlMode, GroupBrewControlTargetValuesUpdate, GroupIndex, InputVolumeType, MachineCommand, MAX_GROUPS, PidLimits, PidParameters, PidTerm, PressureType, RoutineIndex, Status, TemperatureType, WaterTapIndex, WeightType, Routine, RoutineParameter, ParameterUnit, RoutineType, RoutineStep, RoutineCommand, RoutineExit, RoutineExitCondition, StateCondition, ParameterValue, RoutineStepExitType, RoutineParameters, DerivedFormula, UserActionIndex};

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
                entry_command: Some(RoutineCommand::TareGroupScale(group)),
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::StateConditionMet(StateCondition::OutputWeightBelow(group, ParameterValue::Static(0.1))),
                    RoutineStepExitType::NextStep
                )],
                description: Some("Taring".try_into().unwrap()),
            },
            // Step 1: Set target to flow rate
            RoutineStep {
                entry_command: Some(RoutineCommand::SetGroupFixedDutyCycleWithTransition(group, ParameterValue::Static(50.0), ParameterValue::Static(5.0))),
//                entry_command: Some(RoutineCommand::SetGroupOutputFlowRateWithTransition(group, ParameterValue::Parameter(0), ParameterValue::Static(8.0))),
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::Always,
                    RoutineStepExitType::NextStep,
                )],
                description: Some("Setting target flow rate".try_into().unwrap()),
            },
            // Step 2: Start brewing, wait for the group to reach output weight above the specified amount
            RoutineStep {
                entry_command: Some(RoutineCommand::StartBrewing(group)),
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::StateConditionMet(StateCondition::OutputWeightAbove(group, ParameterValue::Parameter(1))),
                    RoutineStepExitType::NextStep
                )],
                description: Some("Dispensing water".try_into().unwrap()),
            },
            // Step 3: Stop brewing, then finish the routine
            RoutineStep {
                entry_command: Some(RoutineCommand::StopBrewing(group)),
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
        }
    }

    /// Get resolved finally commands to execute when routine exits
    pub fn finally(&self, status: &Status) -> Vec<MachineCommand> {
        self.routine.finally
            .iter()
            .map(|cmd| self.resolve_command(cmd, status))
            .collect()
    }

    pub fn step(&mut self, status: &Status, user_action: Option<UserActionIndex>) -> Option<MachineCommand> {
        if self.finished_executing {
            return None; // Routine has finished executing
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

        None
    }

    fn handle_exit(&mut self, exit: &RoutineExit, status: &Status) -> Option<MachineCommand> {
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

                None
            }
        }
    }

    pub fn transition_to(&mut self, step: usize, status: &Status) -> Option<MachineCommand> {
        info!("Transitioning to step {}", step);
        self.current_step = Some(step);
        self.step_start_time = Some(Instant::now());
        self.routine.steps[step].entry_command.as_ref()
            .map(|cmd| self.resolve_command(cmd, status))
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
            // @todo Fix water tap flow code
            StateCondition::WaterTapFlowRateAbove(_, _) => false,
            StateCondition::WaterTapFlowRateBelow(_, _) => false,
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

pub struct InMemoryRoutineRepository {
    routines: Vec<Routine>,
}

pub trait RoutineRepository {
    fn get_routine(&self, index: usize) -> Option<&Routine>;
    fn add_routine(&mut self, routine: Routine);
    fn remove_routine(&mut self, index: usize) -> Option<Routine>;
    fn update_routine(&mut self, index: usize, routine: Routine) -> Result<(), &'static str>;
    fn iterate_routines(&self) -> impl Iterator<Item = &Routine>;
    fn get_routine_count(&self) -> usize;
}

impl InMemoryRoutineRepository {
    pub fn new() -> Self {
        Self {
            routines: Vec::new(),
        }
    }

    pub fn get_routine(&self, index: usize) -> Option<&Routine> {
        self.routines.get(index)
    }

    pub fn add_routine(&mut self, routine: Routine) {
        self.routines.push(routine);
    }
    
    pub fn remove_routine(&mut self, index: usize) -> Option<Routine> {
        if index < self.routines.len() {
            Some(self.routines.remove(index))
        } else {
            None
        }
    }

    pub fn update_routine(&mut self, index: usize, routine: Routine) -> Result<(), &'static str> {
        if index < self.routines.len() {
            self.routines[index] = routine;
            Ok(())
        } else {
            Err("Index out of bounds")
        }
    }
    
    pub fn iterate_routines(&self) -> impl Iterator<Item = &Routine> {
        self.routines.iter()
    }
    
    pub fn get_routine_count(&self) -> usize {
        self.routines.len()
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
                entry_command: Some(RoutineCommand::SetGroupFullOn(group)),
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::Always,
                    RoutineStepExitType::NextStep
                )],
                description: None,
            },
            // Step 1/2: Start filling at FullOn for 1 second (to avoid swings), then until pressure is above 2.0 bar (where the grouphead is filled)
            RoutineStep {
                entry_command: Some(RoutineCommand::StartBrewing(group)),
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::After(ParameterValue::Static(1.0)),
                    RoutineStepExitType::NextStep
                )],
                description: Some("Fast fill".try_into().unwrap()),
            },
            RoutineStep {
                entry_command: None,
                exits: vec![RoutineExit::with_description(
                    RoutineExitCondition::StateConditionMet(StateCondition::BoilerPressureAbove(group, ParameterValue::Static(2.0))),
                    RoutineStepExitType::NextStep,
                    "Group filled?".try_into().unwrap()
                )],
                description: Some("Fast fill".try_into().unwrap()),
            },
            // Step 3: Set pump to Off, then wait for preinfusion time
            RoutineStep {
                entry_command: Some(RoutineCommand::SetGroupOff(group)),
                exits: vec![RoutineExit::with_description(
                    RoutineExitCondition::After(ParameterValue::Parameter(0)),
                    RoutineStepExitType::NextStep,
                    "Pre-infusing".try_into().unwrap()
                )],
                description: Some("Pre-infusion".try_into().unwrap()),
            },
            // Step 4: Set pressure target to pressure, keep going for 4 seconds (to allow the pressure and flow to stabilize)
            RoutineStep {
                entry_command: Some(RoutineCommand::SetGroupPressure(group, ParameterValue::Parameter(2))),
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
                entry_command: None,
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
                entry_command: Some(RoutineCommand::SetGroupFlowRate(group, ParameterValue::Parameter(4))),
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
                entry_command: Some(RoutineCommand::StopBrewing(group)),
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
                entry_command: Some(RoutineCommand::SetBoilerTemperature(boiler_index, ParameterValue::Parameter(0))),
                exits: vec![ RoutineExit::new(
                    RoutineExitCondition::StateConditionMet(StateCondition::BoilerTemperatureAbove(boiler_index, ParameterValue::Static(120.0))),
                    RoutineStepExitType::NextStep,
                )],
                description: Some("Heating to overshoot".try_into().unwrap()),
            },
            RoutineStep {
                entry_command: None,
                exits: vec![ RoutineExit::with_description(
                    RoutineExitCondition::After(ParameterValue::Static(300.0)),
                    RoutineStepExitType::NextStep,
                    "Waiting".try_into().unwrap()
                )],
                description: Some("Stabilizing temperature".try_into().unwrap()),
            },
            RoutineStep {
                entry_command: Some(RoutineCommand::SetBoilerTemperature(boiler_index, ParameterValue::Static(95.0))),
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

pub fn create_volumetric_shot_routine(group: GroupIndex, milliliters: f32, bloom_after: Option<Duration>, bloom_time: Option<Duration>) -> Routine {
    let mut steps = vec![
        RoutineStep {
            entry_command: Some(RoutineCommand::SetGroupFullOn(group)),
            exits: vec![RoutineExit::new(
                RoutineExitCondition::Always,
                RoutineStepExitType::NextStep
            )],
            description: None,
        },
        RoutineStep {
            entry_command: Some(RoutineCommand::StartBrewing(group)),
            exits: vec![RoutineExit::new(
                RoutineExitCondition::Always,
                RoutineStepExitType::NextStep
            )],
            description: Some("Filling".try_into().unwrap()),
        },
    ];

    if let (Some(bloom_after), Some(bloom_time)) = (bloom_after, bloom_time) {
        steps.push(RoutineStep {
            entry_command: None,
            exits: vec![RoutineExit::new(
                RoutineExitCondition::After(ParameterValue::Static(bloom_after.as_millis() as f32 / 1000.0)),
                RoutineStepExitType::NextStep
            )],
            description: Some("Filling".try_into().unwrap()),
        });
        steps.push(RoutineStep {
            entry_command: Some(RoutineCommand::SetGroupOff(group)),
            exits: vec![RoutineExit::with_description(
                RoutineExitCondition::After(ParameterValue::Static(bloom_after.as_millis() as f32 / 1000.0)),
                RoutineStepExitType::NextStep,
                "Blooming".try_into().unwrap()
            )],
            description: Some("Blooming".try_into().unwrap()),
        });
        steps.push(RoutineStep {
            entry_command: Some(RoutineCommand::SetGroupFullOn(group)),
            exits: vec![RoutineExit::new(
                RoutineExitCondition::Always,
                RoutineStepExitType::NextStep
            )],
            description: Some("Brewing".try_into().unwrap()),
        });
    }

    steps.push(RoutineStep {
        entry_command: None,
        exits: vec![RoutineExit::with_description(
            RoutineExitCondition::StateConditionMet(StateCondition::InputVolumeAboveRelativeToStart(group, ParameterValue::Static(milliliters))),
            RoutineStepExitType::NextStep,
            "Reached target volume".try_into().unwrap()
        )],
        description: Some("Brewing".try_into().unwrap()),
    });

    steps.push(RoutineStep {
        entry_command: Some(RoutineCommand::StopBrewing(group)),
        exits: vec![RoutineExit::with_description(
            RoutineExitCondition::Always,
            RoutineStepExitType::Finished,
            "Reached target volume".try_into().unwrap()
        )],
        description: Some("Brewing".try_into().unwrap()),
    });

    Routine {
        routine_type: RoutineType::UserDefined,
        name: "Volumetric shot".try_into().unwrap(),
        parameters: vec![],
        derived_parameters: vec![], // No derived parameters for this routine
        steps,
        finally: vec![],
    }
}