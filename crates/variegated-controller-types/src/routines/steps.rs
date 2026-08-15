use crate::*;
use alloc::string::String;
use alloc::vec::Vec;

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub enum RoutineExitCondition {
    Always,
    Never,
    After(ParameterValue), // seconds as f32, converted to Duration at runtime
    AfterDurationRelativeToStart(ParameterValue),
    StateConditionMet(StateCondition),
    UserAction(UserActionIndex),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub enum RoutineStepExitType {
    NextStep,
    JumpToStep(u32),
    Finished,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug)]
pub enum RoutineCommand {
    // Direct pass-through for non-parameterizable commands
    StartBrewing(GroupIndex),
    StopBrewing(GroupIndex),
    TareGroupScale(GroupIndex),
    StartPumpingToWaterTap(WaterTapIndex),
    StopPumpingToWaterTap(WaterTapIndex),
    StartSteaming(SteamWandIndex),
    StopSteaming(SteamWandIndex),

    // Parameterizable commands
    SetSteamValveOpenness(SteamWandIndex, ParameterValue),
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

    // Bumpless transfer commands - infer PID integral for smooth mode transitions
    InferGroupPressureIntegral(GroupIndex, ParameterValue),
    InferGroupFlowRateIntegral(GroupIndex, ParameterValue),
    InferGroupOutputFlowRateIntegral(GroupIndex, ParameterValue),
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct RoutineExit {
    pub condition: RoutineExitCondition,
    pub then: RoutineStepExitType,
    pub description: Option<String>,
}

#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schema", derive(variegated_postcard_schema::PostcardSchema))]
#[derive(Clone, Debug)]
pub struct RoutineStep {
    pub entry_command: Vec<RoutineCommand>,
    pub exits: Vec<RoutineExit>,
    pub description: Option<String>,
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
