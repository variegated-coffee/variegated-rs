use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use alloc::format;
use core::fmt;
use defmt::{info, Format};
use embassy_time::{Duration, Instant};
use heapless::FnvIndexMap;
use variegated_controller_types::{BoilerControlTarget, BoilerIndex, ControlCurve, FlowRateType, GroupBrewControlTarget, GroupIndex, MachineCommand, PidLimits, PidParameters, PidTerm, PressureType, RoutineIndex, Status, TemperatureType, WaterTapIndex, WeightType};

type UserActionIndex = u8;
pub type RoutineParameters = FnvIndexMap<u8, f32, 8>;

#[derive(Clone, Copy, Debug, Format)]
pub enum ParameterValue {
    Static(f32),
    Parameter(u8), // index into parameter map
    DerivedParameter(u8), // index into derived parameter list (separate namespace)
}

impl fmt::Display for ParameterValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParameterValue::Static(value) => write!(f, "{:.1}", value),
            ParameterValue::Parameter(index) => write!(f, "P{}", index),
            ParameterValue::DerivedParameter(index) => write!(f, "D{}", index),
        }
    }
}

impl ParameterValue {
    // For display purposes, treat Parameter as a placeholder value
    pub fn as_secs(&self) -> u64 {
        match self {
            ParameterValue::Static(value) => *value as u64,
            ParameterValue::Parameter(_) => 0, // Placeholder - should be resolved
            ParameterValue::DerivedParameter(_) => 0, // Placeholder - should be resolved
        }
    }
}

#[derive(Clone, Copy, Debug, Format, PartialEq)]
pub enum ParameterUnit {
    Seconds,
    Celsius,
    Bar,
    MillilitersPerSecond,
    Grams,
    Percent,
}

#[derive(Clone, Debug)]
pub struct RoutineParameter {
    pub index: u8,
    pub name: String,  // User-facing, e.g. "Preinfusion Time", "Target Pressure"
    pub default: f32,
    pub unit: Option<ParameterUnit>,
}

#[derive(Clone, Debug)]
pub struct DerivedParameter {
    pub index: u8,  // Separate index space from regular parameters
    pub name: String,
    pub unit: Option<ParameterUnit>,
    pub formula: DerivedFormula,
}

#[derive(Clone, Debug)]
pub enum DerivedFormula {
    Linear {
        base_param: u8,      // Index of base parameter
        multiplier: f32,
        offset: f32,
    },
    Sum {
        params: Vec<u8>,     // Indices of parameters to sum
    },
    Difference {
        param_a: u8,
        param_b: u8,         // a - b
    },
    Product {
        params: Vec<u8>,     // Indices of parameters to multiply
    },
}

#[derive(Clone, Copy, Debug, Format)]
pub enum StateCondition {
    Brewing(GroupIndex),
    NotBrewing(GroupIndex),
    BoilerTemperatureAbove(BoilerIndex, ParameterValue),
    BoilerTemperatureBelow(BoilerIndex, ParameterValue),
    BoilerPressureAbove(BoilerIndex, ParameterValue),
    BoilerPressureBelow(BoilerIndex, ParameterValue),
    GroupInputFlowRateAbove(GroupIndex, ParameterValue),
    GroupInputFlowRateBelow(GroupIndex, ParameterValue),
    GroupPressureAbove(GroupIndex, ParameterValue),
    GroupPressureBelow(GroupIndex, ParameterValue),
    WaterTapFlowRateAbove(WaterTapIndex, ParameterValue),
    WaterTapFlowRateBelow(WaterTapIndex, ParameterValue),
    OutputWeightAbove(GroupIndex, ParameterValue),
    OutputWeightBelow(GroupIndex, ParameterValue),
}

#[derive(Clone, Copy, Debug)]
pub enum RoutineExitCondition {
    Always,
    Never,
    After(ParameterValue), // seconds as f32, converted to Duration at runtime
    AfterDurationRelativeToStart(ParameterValue),
    StateConditionMet(StateCondition),
    UserAction(UserActionIndex),
}

#[derive(Clone, Copy, Debug)]
enum RoutineStepExitType {
    NextStep,
    JumpToStep(usize),
    Finished,
}

#[derive(Clone, Debug)]
pub enum RoutineCommand {
    // Direct pass-through for non-parameterizable commands
    StartBrewing(GroupIndex),
    StopBrewing(GroupIndex),
    TareGroupScale(GroupIndex),
    
    // Parameterizable commands
    SetBoilerTemperature(BoilerIndex, ParameterValue),
    SetBoilerPressure(BoilerIndex, ParameterValue),
    SetGroupFlowRate(GroupIndex, ParameterValue),
    SetGroupPressure(GroupIndex, ParameterValue),
    SetGroupOutputFlowRate(GroupIndex, ParameterValue),
    SetGroupFixedDutyCycle(GroupIndex, ParameterValue),
    SetGroupFullOn(GroupIndex),
    SetGroupOff(GroupIndex),
    SetBoilerOff(BoilerIndex),
    
    // Transition-enabled commands (only for groups since only they support curves)
    SetGroupFlowRateWithTransition(GroupIndex, ParameterValue, ParameterValue), // target, transition_time
    SetGroupPressureWithTransition(GroupIndex, ParameterValue, ParameterValue), // target, transition_time
    SetGroupOutputFlowRateWithTransition(GroupIndex, ParameterValue, ParameterValue), // target, transition_time
    SetGroupFixedDutyCycleWithTransition(GroupIndex, ParameterValue, ParameterValue), // target, transition_time
}

#[derive(Clone, Debug)]
pub struct RoutineExit {
    pub condition: RoutineExitCondition,
    pub then: RoutineStepExitType,
    pub description: Option<String>,
}

#[derive(Clone, Debug)]
pub struct RoutineStep {
    entry_command: Option<RoutineCommand>,
    exits: Vec<RoutineExit>,
    description: Option<String>,
}

#[derive(Clone, Copy, Debug)]
enum RoutineType {
    HeatUp,
    UserDefined,
}

#[derive(Clone)]
pub struct Routine {
    routine_type: RoutineType,
    name: String,
    parameters: Vec<RoutineParameter>, // max 8
    derived_parameters: Vec<DerivedParameter>, // max 16
    steps: Vec<RoutineStep>,
}

impl Routine {
    pub fn new(routine_type: RoutineType, name: String, parameters: Vec<RoutineParameter>, derived_parameters: Vec<DerivedParameter>, steps: Vec<RoutineStep>) -> Self {
        // Validate limits
        assert!(parameters.len() <= 8, "Maximum 8 regular parameters allowed");
        assert!(derived_parameters.len() <= 16, "Maximum 16 derived parameters allowed");
        
        Self {
            routine_type,
            name,
            parameters,
            derived_parameters,
            steps,
        }
    }

    pub fn routine_type(&self) -> RoutineType {
        self.routine_type
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn steps(&self) -> &[RoutineStep] {
        &self.steps
    }

    pub fn parameters(&self) -> &[RoutineParameter] {
        &self.parameters
    }
    
    pub fn derived_parameters(&self) -> &[DerivedParameter] {
        &self.derived_parameters
    }
}

impl RoutineExit {
    pub fn new(condition: RoutineExitCondition, then: RoutineStepExitType) -> Self {
        Self {
            condition,
            then,
            description: None,
        }
    }

    pub fn with_description(condition: RoutineExitCondition, then: RoutineStepExitType, description: String) -> Self {
        Self {
            condition,
            then,
            description: Some(description),
        }
    }

    pub fn description(&self) -> Option<&str> {
        self.description.as_deref()
    }
}

impl RoutineStep {
    pub fn description(&self) -> Option<&str> {
        self.description.as_deref()
    }

    pub fn exits(&self) -> &[RoutineExit] {
        &self.exits
    }
}

pub fn create_water_dispersal_routine(group: GroupIndex) -> Routine {
    let parameters = vec![
        RoutineParameter { 
            index: 0, 
            name: "Tgt Flow Rate".into(), 
            default: 2.0, 
            unit: Some(ParameterUnit::MillilitersPerSecond)
        },
        RoutineParameter { 
            index: 1, 
            name: "Water amt".into(), 
            default: 30.0, 
            unit: Some(ParameterUnit::Grams)
        },
    ];
    
    Routine {
        routine_type: RoutineType::UserDefined,
        name: "Water dispersal".into(),
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
                description: Some("Taring".into()),
            },
            // Step 1: Set target to flow rate
            RoutineStep {
                entry_command: Some(RoutineCommand::SetGroupFixedDutyCycleWithTransition(group, ParameterValue::Static(50.0), ParameterValue::Static(5.0))),
//                entry_command: Some(RoutineCommand::SetGroupOutputFlowRateWithTransition(group, ParameterValue::Parameter(0), ParameterValue::Static(8.0))),
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::Always,
                    RoutineStepExitType::NextStep,
                )],
                description: Some("Setting target flow rate".into()),
            },
            // Step 2: Start brewing, wait for the group to reach output weight above the specified amount
            RoutineStep {
                entry_command: Some(RoutineCommand::StartBrewing(group)),
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::StateConditionMet(StateCondition::OutputWeightAbove(group, ParameterValue::Parameter(1))),
                    RoutineStepExitType::NextStep
                )],
                description: Some("Dispensing water".into()),
            },
            // Step 3: Stop brewing, then finish the routine
            RoutineStep {
                entry_command: Some(RoutineCommand::StopBrewing(group)),
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::Always,
                    RoutineStepExitType::Finished
                )],
                description: Some("Stopping water flow".into()),
            },
        ],
    }
}

pub fn create_shot_routine(group: GroupIndex) -> Routine {
    Routine {
        routine_type: RoutineType::UserDefined,
        name: "Smart shot".into(),
        parameters: vec![
            RoutineParameter {
                index: 0,
                name: "Preinf. Time".into(),
                default: 5.0,
                unit: Some(ParameterUnit::Seconds)
            },
            RoutineParameter {
                index: 1,
                name: "Brew Weight".into(),
                default: 50.0,
                unit: Some(ParameterUnit::Grams)
            },
            RoutineParameter {
                index: 2,
                name: "Tgt Press".into(),
                default: 8.0,
                unit: Some(ParameterUnit::Bar)
            },
            RoutineParameter {
                index: 3,
                name: "Resc Trigger".into(),
                default: 2.5,
                unit: Some(ParameterUnit::MillilitersPerSecond)
            },
            RoutineParameter {
                index: 4,
                name: "Rescue Flow".into(),
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
                description: Some("Fast fill".into()),
            },
            RoutineStep {
                entry_command: None,
                exits: vec![RoutineExit::with_description(
                    RoutineExitCondition::StateConditionMet(StateCondition::BoilerPressureAbove(group, ParameterValue::Static(2.0))),
                    RoutineStepExitType::NextStep,
                    "Group filled?".into()
                )],
                description: Some("Fast fill".into()),
            },
            // Step 3: Set pump to Off, then wait for preinfusion time
            RoutineStep {
                entry_command: Some(RoutineCommand::SetGroupOff(group)),
                exits: vec![RoutineExit::with_description(
                    RoutineExitCondition::After(ParameterValue::Parameter(0)),
                    RoutineStepExitType::NextStep,
                    "Pre-infusing".into()
                )],
                description: Some("Pre-infusion".into()),
            },
            // Step 4: Set pressure target to target_pressure, keep going for 4 seconds (to allow the pressure and flow to stabilize)
            RoutineStep {
                entry_command: Some(RoutineCommand::SetGroupPressure(group, ParameterValue::Parameter(2))),
                exits: vec![
                    RoutineExit::with_description(
                        RoutineExitCondition::After(ParameterValue::Static(2.0)),
                        RoutineStepExitType::NextStep,
                        "Stabilizing".into()
                    )
                ],
                description: Some("Ramping to pressure".into()),
            },
            // Step 5/6: Keep going at target_pressure for a total of total_brew_time seconds. If the flow rate is above rescue_trigger, switch to control by flow rate at rescue_flow_rate.
            RoutineStep {
                entry_command: None,
                exits: vec![
                    RoutineExit::with_description(
                        RoutineExitCondition::StateConditionMet(StateCondition::OutputWeightAbove(group, ParameterValue::Parameter(1))),
                        RoutineStepExitType::JumpToStep(7),
                        "Brew to weight".into()
                    ),
                    RoutineExit::with_description(
                        RoutineExitCondition::StateConditionMet(StateCondition::GroupInputFlowRateAbove(group, ParameterValue::Parameter(3))),
                        RoutineStepExitType::NextStep,
                        "Shot rescue".into()
                    )
                ],
                description: Some("Brewing".into()),
            },
            RoutineStep {
                entry_command: Some(RoutineCommand::SetGroupFlowRate(group, ParameterValue::Parameter(4))),
                exits: vec![
                    RoutineExit::with_description(
                        RoutineExitCondition::StateConditionMet(StateCondition::OutputWeightAbove(group, ParameterValue::Parameter(1))),
                        RoutineStepExitType::NextStep,
                        "Brew to weight".into()
                    ),
                ],
                description: Some("Shot rescue".into()),
            },
            // Step 7: Stop brewing, then finish the routine
            RoutineStep {
                entry_command: Some(RoutineCommand::StopBrewing(group)),
                exits: vec![RoutineExit::new(
                    RoutineExitCondition::Never,
                    RoutineStepExitType::Finished,
                )],
                description: Some("Finishing extraction".into()),
            },
        ],
    }
}

pub fn create_heatup_routine(boiler_index: BoilerIndex) -> Routine {
    let parameters = vec![
    ];
    
    Routine {
        routine_type: RoutineType::HeatUp,
        name: "Heat-up".into(),
        parameters,
        derived_parameters: vec![], // No derived parameters for this routine
        steps: vec![
            RoutineStep {
                entry_command: Some(RoutineCommand::SetBoilerTemperature(boiler_index, ParameterValue::Parameter(0))),
                exits: vec![ RoutineExit::new(
                    RoutineExitCondition::StateConditionMet(StateCondition::BoilerTemperatureAbove(boiler_index, ParameterValue::Static(120.0))),
                    RoutineStepExitType::NextStep,
                )],
                description: Some("Heating to overshoot".into()),
            },
            RoutineStep {
                entry_command: None,
                exits: vec![ RoutineExit::with_description(
                    RoutineExitCondition::After(ParameterValue::Static(300.0)),
                    RoutineStepExitType::NextStep,
                    "Waiting".into()
                )],
                description: Some("Stabilizing temperature".into()),
            },
            RoutineStep {
                entry_command: Some(RoutineCommand::SetBoilerTemperature(boiler_index, ParameterValue::Static(95.0))),
                exits: vec![ RoutineExit::with_description(
                    RoutineExitCondition::StateConditionMet(StateCondition::BoilerTemperatureBelow(boiler_index, ParameterValue::Static(96.0))),
                    RoutineStepExitType::Finished,
                    "Waiting".into()
                )],
                description: Some("Adjusting to target".into()),
            },
        ],
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
                    BoilerControlTarget::Temperature(self.resolve_value(pv)))
            }
            RoutineCommand::SetBoilerPressure(idx, pv) => {
                MachineCommand::SetBoilerControlTarget(*idx, 
                    BoilerControlTarget::Pressure(self.resolve_value(pv)))
            }
            RoutineCommand::SetGroupFlowRate(idx, pv) => {
                MachineCommand::SetGroupBrewControlTarget(*idx,
                    GroupBrewControlTarget::GroupFlowRate(self.resolve_value(pv)))
            }
            RoutineCommand::SetGroupPressure(idx, pv) => {
                MachineCommand::SetGroupBrewControlTarget(*idx,
                    GroupBrewControlTarget::Pressure(self.resolve_value(pv)))
            }
            RoutineCommand::SetGroupOutputFlowRate(idx, pv) => {
                MachineCommand::SetGroupBrewControlTarget(*idx,
                    GroupBrewControlTarget::OutputFlowRate(self.resolve_value(pv)))
            }
            RoutineCommand::SetGroupFixedDutyCycle(idx, pv) => {
                MachineCommand::SetGroupBrewControlTarget(*idx,
                    GroupBrewControlTarget::FixedDutyCycle(self.resolve_value(pv) as u8))
            }
            RoutineCommand::SetGroupFullOn(idx) => {
                MachineCommand::SetGroupBrewControlTarget(*idx, GroupBrewControlTarget::FullOn)
            }
            RoutineCommand::SetGroupOff(idx) => {
                MachineCommand::SetGroupBrewControlTarget(*idx, GroupBrewControlTarget::Off)
            }
            RoutineCommand::SetBoilerOff(idx) => {
                MachineCommand::SetBoilerControlTarget(*idx, BoilerControlTarget::Off)
            }
            
            // Transition-enabled commands
            RoutineCommand::SetGroupFlowRateWithTransition(idx, target_pv, transition_pv) => {
                let target_value = self.resolve_value(target_pv);
                let transition_time = self.resolve_value(transition_pv);
                
                if transition_time <= 0.0 {
                    // No transition - use direct command
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlTarget::GroupFlowRate(target_value))
                } else {
                    // Create linear transition curve from current value
                    let current_value = status.get_group_status(*idx)
                        .and_then(|gs| gs.input_flow_rate)
                        .unwrap_or(0.0);
                    let curve = self.create_linear_transition_curve(
                        current_value, target_value, transition_time
                    );
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlTarget::GroupFlowRateCurve(curve))
                }
            }
            
            RoutineCommand::SetGroupPressureWithTransition(idx, target_pv, transition_pv) => {
                let target_value = self.resolve_value(target_pv);
                let transition_time = self.resolve_value(transition_pv);
                
                if transition_time <= 0.0 {
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlTarget::Pressure(target_value))
                } else {
                    let current_value = status.get_group_status(*idx)
                        .and_then(|gs| gs.pressure)
                        .unwrap_or(0.0);
                    let curve = self.create_linear_transition_curve(
                        current_value, target_value, transition_time
                    );
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlTarget::PressureCurve(curve))
                }
            }
            
            RoutineCommand::SetGroupOutputFlowRateWithTransition(idx, target_pv, transition_pv) => {
                let target_value = self.resolve_value(target_pv);
                let transition_time = self.resolve_value(transition_pv);
                
                if transition_time <= 0.0 {
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlTarget::OutputFlowRate(target_value))
                } else {
                    let current_value = status.get_group_status(*idx)
                        .and_then(|gs| gs.output_flow_rate)
                        .unwrap_or(0.0);
                    let curve = self.create_linear_transition_curve(
                        current_value, target_value, transition_time
                    );
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlTarget::OutputFlowRateCurve(curve))
                }
            }
            
            RoutineCommand::SetGroupFixedDutyCycleWithTransition(idx, target_pv, transition_pv) => {
                let target_value = self.resolve_value(target_pv);
                let transition_time = self.resolve_value(transition_pv);
                
                if transition_time <= 0.0 {
                    // No transition - use direct command
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlTarget::FixedDutyCycle(target_value as u8))
                } else {
                    // Create linear transition curve from current duty cycle
                    let current_value = status.get_group_status(*idx)
                        .map(|gs| gs.pump_output.duty_cycle() as f32)
                        .unwrap_or(0.0);
                    let curve = self.create_linear_transition_curve(
                        current_value, target_value, transition_time
                    );
                    MachineCommand::SetGroupBrewControlTarget(*idx,
                        GroupBrewControlTarget::FixedDutyCycleCurve(curve))
                }
            }
        }
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
        }
    }
}

pub struct InMemoryRoutineRepository {
    routines: Vec<Routine>,
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
    
    pub fn iterate_routines(&self) -> impl Iterator<Item = &Routine> {
        self.routines.iter()
    }
    
    pub fn get_routine_count(&self) -> usize {
        self.routines.len()
    }
}