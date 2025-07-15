use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use defmt::{info, Format};
use embassy_time::{Duration, Instant};
use variegated_controller_types::{BoilerControlTarget, BoilerIndex, FlowRateType, GroupBrewControlTarget, GroupIndex, MachineCommand, PidLimits, PidParameters, PidTerm, PressureType, RoutineIndex, Status, TemperatureType, WaterTapIndex, WeightType};

type UserActionIndex = u8;

#[derive(Clone, Copy, Debug, Format)]
enum StateCondition {
    Brewing(GroupIndex),
    NotBrewing(GroupIndex),
    BoilerTemperatureAbove(BoilerIndex, TemperatureType),
    BoilerTemperatureBelow(BoilerIndex,TemperatureType),
    BoilerPressureAbove(BoilerIndex, PressureType),
    BoilerPressureBelow(BoilerIndex, PressureType),
    GroupInputFlowRateAbove(GroupIndex, FlowRateType),
    GroupInputFlowRateBelow(GroupIndex, FlowRateType),
    GroupPressureAbove(GroupIndex, PressureType),
    GroupPressureBelow(GroupIndex, PressureType),
    WaterTapFlowRateAbove(WaterTapIndex, FlowRateType),
    WaterTapFlowRateBelow(WaterTapIndex, FlowRateType),
    OutputWeightAbove(GroupIndex, WeightType),
    OutputWeightBelow(GroupIndex, WeightType),
}

#[derive(Clone, Copy, Debug)]
enum RoutineExitCondition {
    Always,
    Never,
    After(Duration),
    AfterDurationRelativeToStart(Duration),
    StateConditionMet(StateCondition),
    UserAction(UserActionIndex),
}

#[derive(Clone, Copy, Debug)]
enum RoutineStepExitType {
    NextStep,
    JumpToStep(usize),
    Finished,
}

#[derive(Clone, Copy, Debug)]
struct RoutineExit {
    condition: RoutineExitCondition,
    then: RoutineStepExitType,
}

#[derive(Clone, Debug)]
struct RoutineStep {
    entry_command: Option<MachineCommand>,
    exits: Vec<RoutineExit>,
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
    steps: Vec<RoutineStep>,
}

pub fn create_shot_routine(group: GroupIndex, preinfusion_time: Duration, total_brew_time: Duration, target_pressure: PressureType, rescue_trigger: FlowRateType, rescue_flow_rate: FlowRateType) -> Routine {
    Routine {
        routine_type: RoutineType::UserDefined,
        name: "Smart shot".into(),
        steps: vec![
            RoutineStep {
                entry_command: Some(MachineCommand::SetGroupBrewControlTarget(group, GroupBrewControlTarget::FullOn)),
                exits: vec![RoutineExit {
                    condition: RoutineExitCondition::Always,
                    then: RoutineStepExitType::NextStep
                }],
            },
            // Step 2/3: Start filling at FullOn for 1 second (to avoid swings), then until pressure is above 2.0 bar (where the grouphead is filled)
            RoutineStep {
                entry_command: Some(MachineCommand::StartBrewing(0)),
                exits: vec![RoutineExit {
                    condition: RoutineExitCondition::After(Duration::from_secs(1)),
                    then: RoutineStepExitType::NextStep
                }],
            },
            RoutineStep {
                entry_command: None,
                exits: vec![RoutineExit {
                    condition: RoutineExitCondition::StateConditionMet(StateCondition::BoilerPressureAbove(group, 2.0)),
                    then: RoutineStepExitType::NextStep
                }],
            },
            // Step 4: Set pump to Off, then wait for preinfusion time
            RoutineStep {
                entry_command: Some(MachineCommand::SetGroupBrewControlTarget(group, GroupBrewControlTarget::Off)),
                exits: vec![RoutineExit {
                    condition: RoutineExitCondition::After(preinfusion_time),
                    then: RoutineStepExitType::NextStep
                }],
            },
            // Step 5: Set pressure target to target_pressure, keep going for 4 seconds (to allow the pressure and flow to stabilize)
            RoutineStep {
                entry_command: Some(MachineCommand::SetGroupBrewControlTarget(group, GroupBrewControlTarget::Pressure(target_pressure))),
                exits: vec![
                    RoutineExit {
                        condition: RoutineExitCondition::After(Duration::from_secs(2)),
                        then: RoutineStepExitType::NextStep
                    }
                ],
            },
            // Step 6/7: Keep going at target_pressure for a total of total_brew_time seconds. If the flow rate is above rescue_trigger, switch to control by flow rate at rescue_flow_rate.
            RoutineStep {
                entry_command: None,
                exits: vec![
                    RoutineExit {
                        condition: RoutineExitCondition::AfterDurationRelativeToStart(total_brew_time),
                        then: RoutineStepExitType::JumpToStep(8)
                    },
                    RoutineExit {
                        condition: RoutineExitCondition::StateConditionMet(StateCondition::GroupInputFlowRateAbove(group, rescue_trigger)),
                        then: RoutineStepExitType::NextStep
                    }
                ],
            },
            RoutineStep {
                entry_command: Some(MachineCommand::SetGroupBrewControlTarget(group, GroupBrewControlTarget::GroupFlowRate(rescue_flow_rate))),
                exits: vec![
                    RoutineExit {
                        condition: RoutineExitCondition::AfterDurationRelativeToStart(total_brew_time),
                        then: RoutineStepExitType::NextStep
                    },
                ],
            },
            // Step 8: Stop brewing, then finish the routine
            RoutineStep {
                entry_command: Some(MachineCommand::StopBrewing(0)),
                exits: vec![RoutineExit {
                    condition: RoutineExitCondition::Never,
                    then: RoutineStepExitType::Finished
                }],
            },

        ],
    }
}

pub fn create_heatup_routine(boiler_index: BoilerIndex) -> Routine {
    Routine {
        routine_type: RoutineType::HeatUp,
        name: "Heat-up".into(),
        steps: vec![
            RoutineStep {
                entry_command: Some(MachineCommand::SetBoilerControlTarget(0, BoilerControlTarget::Temperature(120.0))),
                exits: vec![ RoutineExit {
                    condition: RoutineExitCondition::StateConditionMet(StateCondition::BoilerTemperatureAbove(boiler_index, 120.0)),
                    then: RoutineStepExitType::NextStep,
                }],
            },
            RoutineStep {
                entry_command: None,
                exits: vec![ RoutineExit {
                    condition: RoutineExitCondition::After(Duration::from_secs(300)),
                    then: RoutineStepExitType::NextStep,
                }],
            },
            RoutineStep {
                entry_command: Some(MachineCommand::SetBoilerControlTarget(0, BoilerControlTarget::Temperature(95.0))),
                exits: vec![ RoutineExit {
                    condition: RoutineExitCondition::StateConditionMet(StateCondition::BoilerTemperatureBelow(boiler_index, 96.0)),
                    then: RoutineStepExitType::Finished,
                }],
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
    
    pub(crate) saved_state: StateT,
    pub(crate) saved_configuration: ConfigurationT,
}

impl<StateT, ConfigurationT> RoutineExecutionContext<StateT, ConfigurationT> {
    pub fn new(routine_index: RoutineIndex, routine: Routine, saved_state: StateT, saved_configuration: ConfigurationT) -> Self {
        Self {
            routine_index,
            routine,
            currently_executing: false,
            finished_executing: false,
            current_step: None,
            execution_start_time: None,
            step_start_time: None,
            saved_state,
            saved_configuration,
        }
    }

    pub fn step(&mut self, status: &Status, user_action: Option<UserActionIndex>) -> Option<MachineCommand> {
        if self.finished_executing {
            return None; // Routine has finished executing
        }

        if self.current_step.is_none() {
            self.currently_executing = true;
            self.execution_start_time = Some(Instant::now());
            return self.transition_to(0);
        }

        // Check exit conditions
        for exit in self.routine.steps[self.current_step.unwrap()].exits.clone().iter() {
            match exit.condition {
                RoutineExitCondition::Always => {
                    return self.handle_exit(exit);
                }
                RoutineExitCondition::Never => continue,
                RoutineExitCondition::After(duration) => {
                    if self.step_start_time.expect("Step start time is None - Shouldn't happen").elapsed() >= duration {
                        return self.handle_exit(exit);
                    }
                }
                RoutineExitCondition::AfterDurationRelativeToStart(duration) => {
                    if self.execution_start_time.expect("Execution start time is None - Shouldn't happen").elapsed() >= duration {
                        return self.handle_exit(exit);
                    }
                }
                RoutineExitCondition::StateConditionMet(condition) => {
                    if self.state_condition_met(condition, status) {
                        info!("State condition met: {:?}", condition);
                        return self.handle_exit(exit);
                    }
                }
                RoutineExitCondition::UserAction(action_index) => {
                    if let Some(user_action_index) = user_action {
                        if user_action_index == action_index {
                            info!("User action condition met: {:?}", action_index);
                            return self.handle_exit(exit);
                        }
                    }
                }
            }

        }

        None
    }

    fn handle_exit(&mut self, exit: &RoutineExit) -> Option<MachineCommand> {
        match exit.then {
            RoutineStepExitType::NextStep => {
                self.transition_to(self.current_step.unwrap() + 1)
            }
            RoutineStepExitType::JumpToStep(step) => {
                self.transition_to(step)
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

    pub fn transition_to(&mut self, step: usize) -> Option<MachineCommand> {
        info!("Transitioning to step {}", step);
        self.current_step = Some(step);
        self.step_start_time = Some(Instant::now());
        self.routine.steps[step].entry_command
    }

    fn state_condition_met(&self, state_condition: StateCondition, status: &Status) -> bool {
        match state_condition {
            StateCondition::Brewing(idx) => status.get_group_status(idx).map_or(false, |s| s.is_brewing),
            StateCondition::NotBrewing(idx) => status.get_group_status(idx).map_or(false, |s| !s.is_brewing),
            StateCondition::BoilerTemperatureAbove(idx, temperature) => status.get_boiler_status(idx).map_or(false, |s| s.temperature.unwrap_or(0.0) > temperature),
            StateCondition::BoilerTemperatureBelow(idx, temperature) => status.get_boiler_status(idx).map_or(false, |s| s.temperature.unwrap_or(0.0) > temperature),
            StateCondition::BoilerPressureAbove(idx, pressure) => status.get_boiler_status(idx).map_or(false, |s| s.pressure.unwrap_or(0.0) > pressure),
            StateCondition::BoilerPressureBelow(idx, pressure) => status.get_boiler_status(idx).map_or(false, |s| s.pressure.unwrap_or(0.0) < pressure),
            StateCondition::GroupInputFlowRateAbove(idx, flow) => status.get_group_status(idx).map_or(false, |s| s.input_flow_rate.unwrap_or(0.0) > flow),
            StateCondition::GroupInputFlowRateBelow(idx, flow) => status.get_group_status(idx).map_or(false, |s| s.input_flow_rate.unwrap_or(0.0) < flow),
            StateCondition::GroupPressureAbove(idx, pressure) => status.get_group_status(idx).map_or(false, |s| s.pressure.unwrap_or(0.0) > pressure),
            StateCondition::GroupPressureBelow(idx, pressure) => status.get_group_status(idx).map_or(false, |s| s.pressure.unwrap_or(0.0) < pressure),
            // @todo Fix water tap flow code
            StateCondition::WaterTapFlowRateAbove(_, _) => false,
            StateCondition::WaterTapFlowRateBelow(_, _) => false,
            StateCondition::OutputWeightAbove(idx, weight) => status.get_group_status(idx).map_or(false, |s| s.output_weight.unwrap_or(0.0) > weight),
            StateCondition::OutputWeightBelow(idx, weight) => status.get_group_status(idx).map_or(false, |s| s.output_weight.unwrap_or(0.0) < weight),
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
}