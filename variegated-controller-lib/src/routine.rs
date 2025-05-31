use alloc::vec;
use alloc::vec::Vec;
use defmt::{info, Format};
use embassy_time::{Duration, Instant};
use variegated_controller_types::{BoilerControlTarget, BoilerIndex, FlowRateType, GroupBrewControlTarget, GroupIndex, MachineCommand, PidLimits, PidParameters, PidTerm, PressureType, Status, TemperatureType, WaterTapIndex, WeightType};

type UserActionIndex = u8;

#[derive(Clone, Copy, Debug, Format)]
enum StateCondition {
    Brewing(GroupIndex),
    NotBrewing(GroupIndex),
    BoilerTemperatureAbove(BoilerIndex, TemperatureType),
    BoilerTemperatureBelow(BoilerIndex,TemperatureType),
    BoilerPressureAbove(BoilerIndex, PressureType),
    BoilerPressureBelow(BoilerIndex, PressureType),
    GroupFlowRateAbove(GroupIndex, FlowRateType),
    GroupFlowRateBelow(GroupIndex, FlowRateType),
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

pub(crate) struct Routine {
    routine_type: RoutineType,
    steps: Vec<RoutineStep>,
}

pub(crate) fn create_shot_routine(group: GroupIndex) -> Routine {
    Routine {
        routine_type: RoutineType::UserDefined,
        steps: vec![
            RoutineStep {
                entry_command: Some(MachineCommand::SetGroupBrewControlTarget(group, GroupBrewControlTarget::FullOn)),
                exits: vec![RoutineExit {
                    condition: RoutineExitCondition::Always,
                    then: RoutineStepExitType::NextStep
                }],
            },
            RoutineStep {
                entry_command: Some(MachineCommand::StartBrewing(0)),
                exits: vec![RoutineExit {
                    condition: RoutineExitCondition::StateConditionMet(StateCondition::BoilerPressureAbove(group, 2.0)),
                    then: RoutineStepExitType::NextStep
                }],
            },
            RoutineStep {
                entry_command: Some(MachineCommand::SetGroupBrewControlTarget(group, GroupBrewControlTarget::Pressure(7.0))),
                exits: vec![
                    RoutineExit {
                        condition: RoutineExitCondition::After(Duration::from_secs(4)),
                        then: RoutineStepExitType::NextStep
                    }
                ],
            },
            RoutineStep {
                entry_command: None,
                exits: vec![
                    RoutineExit {
                        condition: RoutineExitCondition::AfterDurationRelativeToStart(Duration::from_secs(15)),
                        then: RoutineStepExitType::JumpToStep(5)
                    },
                    RoutineExit {
                        condition: RoutineExitCondition::StateConditionMet(StateCondition::GroupFlowRateAbove(group, 4.0)),
                        then: RoutineStepExitType::NextStep
                    }
                ],
            },
            RoutineStep {
                entry_command: Some(MachineCommand::SetGroupBrewControlTarget(group, GroupBrewControlTarget::GroupFlowRate(1.5))),
                exits: vec![
                    RoutineExit {
                        condition: RoutineExitCondition::AfterDurationRelativeToStart(Duration::from_secs(15)),
                        then: RoutineStepExitType::NextStep
                    },
                ],
            },
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

fn create_heatup_routine(boiler_index: BoilerIndex) -> Routine {
    Routine {
        routine_type: RoutineType::HeatUp,
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

pub struct RoutineExecutionContext {
    pub(crate) routine: Routine,
    pub(crate) currently_executing: bool,
    pub(crate) finished_executing: bool,
    pub(crate) current_step: Option<usize>,
    pub(crate) execution_start_time: Option<Instant>,
    pub(crate) step_start_time: Option<Instant>,
}

impl RoutineExecutionContext {
    pub fn new(routine: Routine) -> Self {
        Self {
            routine,
            currently_executing: false,
            finished_executing: false,
            current_step: None,
            execution_start_time: None,
            step_start_time: None,
        }
    }

    pub fn step(&mut self, status: &Status) -> Option<MachineCommand> {
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
                RoutineExitCondition::UserAction(_) => {
                    // Handle user action logic here
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
        // @todo The code here is indicative of the status being inadequate for the state condition checks.

        match state_condition {
            StateCondition::Brewing(_) => status.is_brewing,
            StateCondition::NotBrewing(_) => !status.is_brewing,
            StateCondition::BoilerTemperatureAbove(_, temperature) => status.boiler_temp > temperature,
            StateCondition::BoilerTemperatureBelow(_, temperature) => status.boiler_temp < temperature,
            StateCondition::BoilerPressureAbove(_, pressure) => status.boiler_pressure.map_or(false, |p| p > pressure),
            StateCondition::BoilerPressureBelow(_, pressure) => status.boiler_pressure.map_or(false, |p| p < pressure),
            StateCondition::GroupFlowRateAbove(_, flow) => status.group_flow_rate.map_or(false, |f| f > flow),
            StateCondition::GroupFlowRateBelow(_, flow) => status.group_flow_rate.map_or(false, |f| f < flow),
            StateCondition::GroupPressureAbove(_, pressure) => status.boiler_pressure.map_or(false, |p| p > pressure),
            StateCondition::GroupPressureBelow(_, pressure) => status.boiler_pressure.map_or(false, |p| p < pressure),
            StateCondition::WaterTapFlowRateAbove(_, _) => false,
            StateCondition::WaterTapFlowRateBelow(_, _) => false,
            StateCondition::OutputWeightAbove(_, _) => false,
            StateCondition::OutputWeightBelow(_, _) => false
        }
    }
}