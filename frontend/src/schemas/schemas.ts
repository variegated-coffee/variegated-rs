// NOTE: This file is MANUALLY MAINTAINED (not auto-generated)
//
// IMPORTANT: Field order in struct schemas MUST match the Rust struct declaration order exactly.
// Postcard serialization is order-dependent - fields are serialized/deserialized in declaration order.
//
// When updating this file after Rust type changes:
// 1. Check the Rust struct definition in variegated-controller-types/src/lib.rs
// 2. Ensure schema fields appear in the exact same order as Rust struct fields
// 3. Mismatched field order causes deserialization errors like:
//    - "Invalid enum discriminant" (reading wrong bytes as enum)
//    - "Invalid option discriminant" (reading wrong bytes as Option)
//
// FUTURE: Create a custom proc-macro crate to generate these schemas directly from Rust types,
// preserving field order and avoiding the JSON Schema intermediate step (which loses order info).
// This would replace schemars + build.rs generation entirely.

import {
  string,
  f32,
  map,
  u64,
  seq,
  unitVariant,
  option,
  u32,
  struct,
  newtypeVariant,
  f64,
  enumType,
  bool,
  tupleVariant,
  u8,
  u16,
  InferType,
  i8
} from '@variegated-coffee/serde-postcard-ts';

export const PeripheralTypeSchema = enumType('PeripheralType', {
  Scale: unitVariant('Scale'),
  PressureSensor: unitVariant('PressureSensor'),
  FlowMeter: unitVariant('FlowMeter'),
  LevelSensor: unitVariant('LevelSensor')
});

export const SteamWandConfigurationSchema = struct({
  temperature_target: option(f32()),
  openness: option(u8()),
  purge_time_seconds: option(u32()),
  max_steam_time_seconds: option(u32()),
  auto_purge_enabled: bool(),
  supply_tank_index: option(u8())
});

export const TankStatusSchema = struct({
  water_level: option(u8())
});

export const ParameterUnitSchema = enumType('ParameterUnit', {
  Seconds: unitVariant('Seconds'),
  Celsius: unitVariant('Celsius'),
  Bar: unitVariant('Bar'),
  MillilitersPerSecond: unitVariant('MillilitersPerSecond'),
  Grams: unitVariant('Grams'),
  Percent: unitVariant('Percent')
});

export const BoilerTypeSchema = enumType('BoilerType', {
  BrewBoiler: unitVariant('BrewBoiler'),
  SteamBoiler: unitVariant('SteamBoiler'),
  VirtualSteamBoiler: unitVariant('VirtualSteamBoiler')
});

export const BoilerControlTargetValuesUpdateSchema = struct({
  temperature: option(f32()),
  pressure: option(f32())
});

export const PumpConfigurationSchema = struct({
  tacho_pulses_per_liter: option(f32()),
  max_duty_cycle: option(u8()),
  min_duty_cycle: option(u8()),
  ramp_up_time_ms: option(u32()),
  ramp_down_time_ms: option(u32())
});

export const GroupBrewControlModeSchema = enumType('GroupBrewControlMode', {
  GroupFlowRate: unitVariant('GroupFlowRate'),
  GroupFlowRateCurve: unitVariant('GroupFlowRateCurve'),
  Pressure: unitVariant('Pressure'),
  PressureCurve: unitVariant('PressureCurve'),
  OutputFlowRate: unitVariant('OutputFlowRate'),
  OutputFlowRateCurve: unitVariant('OutputFlowRateCurve'),
  FixedDutyCycle: unitVariant('FixedDutyCycle'),
  FixedDutyCycleCurve: unitVariant('FixedDutyCycleCurve'),
  FullOn: unitVariant('FullOn'),
  Off: unitVariant('Off')
});

export const EnvironmentalSensorTypeSchema = enumType('EnvironmentalSensorType', {
  AmbientTemperature: unitVariant('AmbientTemperature'),
  CaseTemperature: unitVariant('CaseTemperature'),
  ExternalTemperature: unitVariant('ExternalTemperature'),
  Humidity: unitVariant('Humidity')
});

export const DurationSchema = struct({
  secs: u64(),
  nanos: u32()
});

export const Limits_for_floatSchema = struct({
  lower: f32(),
  upper: f32()
});

export const RoutineTypeSchema = enumType('RoutineType', {
  HeatUp: unitVariant('HeatUp'),
  UserDefined: unitVariant('UserDefined'),
  Cleaning: unitVariant('Cleaning'),
  HardwareButtonMapped: unitVariant('HardwareButtonMapped')
});

export const ScheduleTriggerSchema = struct({
  on_minute: u8(),
  on_hour: u8(),
  on_days: option(seq(string())),
  on_date: option(string()),
  enabled: bool(),
  once: bool()
});

export const BoilerControlTargetValuesSchema = struct({
  target_temperature: f32(),
  target_pressure: f32()
});

export const MachineConfigurationSchema = struct({
  heating_element_interlock: bool(),
  max_shot_logs: u64(),
  log_sample_decimation: u8(),
  prevent_start_on_empty_tank: bool(),
  allow_continue_on_empty_tank: bool()
});

export const ControlCurveSchema = struct({
  a: f32(),
  b: f32(),
  c: f32(),
  min: f32(),
  max: f32()
});

export const WaterTapStatusSchema = struct({
  is_dispensing: bool()
});

export const SteamWandStatusSchema = struct({
  is_steaming: bool(),
  valve_openness: u8()
});

export const KalmanParametersSchema = struct({
  process_noise: f32(),
  measurement_noise: f32(),
  estimated_error: f32(),
  posterior_estimate: f32()
});

export const CommsStatusSchema = struct({
  timestamp: option(u64()),
  wifi_connected: bool(),
  wifi_rssi: option(i8())
});

export const MachineModeSchema = enumType('MachineMode', {
  On: unitVariant('On'),
  Off: unitVariant('Off'),
  PowerSaveStandby: unitVariant('PowerSaveStandby')
});

export const ShotStateSchema = enumType('ShotState', {
  HeadspaceFill: unitVariant('HeadspaceFill'),
  Saturation: unitVariant('Saturation'),
  PostFirstDrop: unitVariant('PostFirstDrop')
});

export const RoutineIndexSchema = enumType('RoutineIndex', {
  Internal: newtypeVariant('Internal', u32()),
  Function: newtypeVariant('Function', u32()),
  Custom: newtypeVariant('Custom', u32())
});

export const ControlModeCapabilitySchema = enumType('ControlModeCapability', {
  TemperaturePid: unitVariant('TemperaturePid'),
  PressurePid: unitVariant('PressurePid'),
  FlowRatePid: unitVariant('FlowRatePid'),
  OutputFlowRatePid: unitVariant('OutputFlowRatePid'),
  FixedDutyCycle: unitVariant('FixedDutyCycle'),
  FullOn: unitVariant('FullOn'),
  Off: unitVariant('Off')
});

export const BoilerControlModeSchema = enumType('BoilerControlMode', {
  Temperature: unitVariant('Temperature'),
  Pressure: unitVariant('Pressure'),
  Off: unitVariant('Off')
});

export const RoutineStepExitTypeSchema = enumType('RoutineStepExitType', {
  NextStep: unitVariant('NextStep'),
  JumpToStep: newtypeVariant('JumpToStep', u32()),
  Finished: unitVariant('Finished')
});

export const ParameterValueSchema = enumType('ParameterValue', {
  Static: newtypeVariant('Static', f32()),
  Parameter: newtypeVariant('Parameter', u8()),
  DerivedParameter: newtypeVariant('DerivedParameter', u8())
});

export const PidOut_for_floatSchema = struct({
  p: f32(),
  i: f32(),
  d: f32(),
  out: f32(),
  acting_kp: f32(),
  acting_ki: f32(),
  acting_kd: f32()
});

export const DerivedFormulaSchema = enumType('DerivedFormula', {
  Linear: newtypeVariant('Linear', struct({
  base_param: u8(),
  multiplier: f32(),
  offset: f32()
})),
  Sum: newtypeVariant('Sum', struct({
  params: seq(u8())
})),
  Difference: newtypeVariant('Difference', struct({
  param_a: u8(),
  param_b: u8()
})),
  Product: newtypeVariant('Product', struct({
  params: seq(u8())
}))
});

export const SensorCapabilitySchema = enumType('SensorCapability', {
  Temperature: unitVariant('Temperature'),
  Pressure: unitVariant('Pressure'),
  WaterLevel: unitVariant('WaterLevel'),
  InputFlowRate: unitVariant('InputFlowRate'),
  OutputFlowRate: unitVariant('OutputFlowRate'),
  Weight: unitVariant('Weight')
});

export const ActuatorCapabilitySchema = enumType('ActuatorCapability', {
  HeatingElement: unitVariant('HeatingElement'),
  Pump: unitVariant('Pump'),
  SolenoidValve: unitVariant('SolenoidValve'),
  ThreeWayValve: unitVariant('ThreeWayValve'),
  WaterMixer: unitVariant('WaterMixer'),
  ScaleTare: unitVariant('ScaleTare')
});

export const WaterDispersalPumpStrategySchema = enumType('WaterDispersalPumpStrategy', {
  AlwaysPump: newtypeVariant('AlwaysPump', u8()),
  NoPump: unitVariant('NoPump')
});

export const PeripheralInfoSchema = struct({
  peripheral_type: PeripheralTypeSchema,
  is_available: bool()
});

export const RoutineParameterSchema = struct({
  index: u8(),
  name: string(),
  default: f32(),
  unit: option(ParameterUnitSchema)
});

export const FillConfigurationSchema = struct({
  fill_threshold: option(u8()),
  pump_configuration: option(PumpConfigurationSchema)
});

export const EnvironmentalSensorDefinitionSchema = struct({
  name: string(),
  sensor_type: EnvironmentalSensorTypeSchema,
  measurement_range: option(struct({ _0: f32(), _1: f32() }))
});

export const PreviousBrewInfoSchema = struct({
  brew_time: DurationSchema,
  brew_input_volume: option(f64()),
  output_weight: option(f32()),
  started_at_millis: u64(),
  stopped_at_millis: u64()
});

export const PidTerm_for_floatSchema = struct({
  positive_scale: f32(),
  negative_scale: f32(),
  limits: Limits_for_floatSchema
});

export const GroupBrewControlTargetValuesSchema = struct({
  flow_rate: f32(),
  flow_rate_curve: ControlCurveSchema,
  pressure: f32(),
  pressure_curve: ControlCurveSchema,
  output_flow_rate: f32(),
  output_flow_rate_curve: ControlCurveSchema,
  duty_cycle: u8(),
  duty_cycle_curve: ControlCurveSchema
});

export const TankConfigurationSchema = struct({
  low_level_warning_threshold: option(u8()),
  water_level_sensor_kalman_parameters: option(KalmanParametersSchema),
  empty_threshold: option(u8())
});

export const RoutineExecutionStatusSchema = struct({
  routine_index: RoutineIndexSchema,
  current_step: option(u32()),
  step_elapsed_time: option(DurationSchema),
  total_elapsed_time: option(DurationSchema),
  resolved_parameters: map(u8(), f32())
});

export const BoilerControlStateSchema = struct({
  mode: BoilerControlModeSchema,
  values: BoilerControlTargetValuesSchema
});

export const ScheduleActionSchema = enumType('ScheduleAction', {
  RunRoutine: tupleVariant('RunRoutine', RoutineIndexSchema, option(map(u8(), f32()))),
  CancelRoutine: unitVariant('CancelRoutine'),
  SetMachineMode: newtypeVariant('SetMachineMode', MachineModeSchema),
  SetBoilerControlTarget: tupleVariant('SetBoilerControlTarget', u8(), BoilerControlModeSchema, option(BoilerControlTargetValuesUpdateSchema)),
  SetBoilerControlTargetValues: tupleVariant('SetBoilerControlTargetValues', u8(), BoilerControlTargetValuesUpdateSchema)
});

export const StateConditionSchema = enumType('StateCondition', {
  Brewing: newtypeVariant('Brewing', u8()),
  NotBrewing: newtypeVariant('NotBrewing', u8()),
  BoilerTemperatureAbove: tupleVariant('BoilerTemperatureAbove', u8(), ParameterValueSchema),
  BoilerTemperatureBelow: tupleVariant('BoilerTemperatureBelow', u8(), ParameterValueSchema),
  BoilerPressureAbove: tupleVariant('BoilerPressureAbove', u8(), ParameterValueSchema),
  BoilerPressureBelow: tupleVariant('BoilerPressureBelow', u8(), ParameterValueSchema),
  GroupInputFlowRateAbove: tupleVariant('GroupInputFlowRateAbove', u8(), ParameterValueSchema),
  GroupInputFlowRateBelow: tupleVariant('GroupInputFlowRateBelow', u8(), ParameterValueSchema),
  GroupPressureAbove: tupleVariant('GroupPressureAbove', u8(), ParameterValueSchema),
  GroupPressureBelow: tupleVariant('GroupPressureBelow', u8(), ParameterValueSchema),
  WaterTapFlowRateAbove: tupleVariant('WaterTapFlowRateAbove', u8(), ParameterValueSchema),
  WaterTapFlowRateBelow: tupleVariant('WaterTapFlowRateBelow', u8(), ParameterValueSchema),
  OutputWeightAbove: tupleVariant('OutputWeightAbove', u8(), ParameterValueSchema),
  OutputWeightBelow: tupleVariant('OutputWeightBelow', u8(), ParameterValueSchema),
  InputVolumeAboveRelativeToStart: tupleVariant('InputVolumeAboveRelativeToStart', u8(), ParameterValueSchema)
});

export const RoutineCommandSchema = enumType('RoutineCommand', {
  StartBrewing: newtypeVariant('StartBrewing', u8()),
  StopBrewing: newtypeVariant('StopBrewing', u8()),
  TareGroupScale: newtypeVariant('TareGroupScale', u8()),
  StartPumpingToWaterTap: newtypeVariant('StartPumpingToWaterTap', u8()),
  StopPumpingToWaterTap: newtypeVariant('StopPumpingToWaterTap', u8()),
  StartSteaming: newtypeVariant('StartSteaming', u8()),
  StopSteaming: newtypeVariant('StopSteaming', u8()),
  SetSteamValveOpenness: tupleVariant('SetSteamValveOpenness', u8(), ParameterValueSchema),
  SetBoilerTemperature: tupleVariant('SetBoilerTemperature', u8(), ParameterValueSchema),
  SetBoilerPressure: tupleVariant('SetBoilerPressure', u8(), ParameterValueSchema),
  SetGroupFlowRate: tupleVariant('SetGroupFlowRate', u8(), ParameterValueSchema),
  SetGroupPressure: tupleVariant('SetGroupPressure', u8(), ParameterValueSchema),
  SetGroupOutputFlowRate: tupleVariant('SetGroupOutputFlowRate', u8(), ParameterValueSchema),
  SetGroupFixedDutyCycle: tupleVariant('SetGroupFixedDutyCycle', u8(), ParameterValueSchema),
  SetGroupFullOn: newtypeVariant('SetGroupFullOn', u8()),
  SetGroupOff: newtypeVariant('SetGroupOff', u8()),
  SetBoilerOff: newtypeVariant('SetBoilerOff', u8()),
  SetGroupFlowRateWithTransition: tupleVariant('SetGroupFlowRateWithTransition', u8(), ParameterValueSchema, ParameterValueSchema),
  SetGroupPressureWithTransition: tupleVariant('SetGroupPressureWithTransition', u8(), ParameterValueSchema, ParameterValueSchema),
  SetGroupOutputFlowRateWithTransition: tupleVariant('SetGroupOutputFlowRateWithTransition', u8(), ParameterValueSchema, ParameterValueSchema),
  SetGroupFixedDutyCycleWithTransition: tupleVariant('SetGroupFixedDutyCycleWithTransition', u8(), ParameterValueSchema, ParameterValueSchema),
  InferGroupPressureIntegral: tupleVariant('InferGroupPressureIntegral', u8(), ParameterValueSchema),
  InferGroupFlowRateIntegral: tupleVariant('InferGroupFlowRateIntegral', u8(), ParameterValueSchema),
  InferGroupOutputFlowRateIntegral: tupleVariant('InferGroupOutputFlowRateIntegral', u8(), ParameterValueSchema)
});

export const OutputSchema = enumType('Output', {
  Off: unitVariant('Off'),
  FixedDutyCycle: newtypeVariant('FixedDutyCycle', u8()),
  PidOutput: newtypeVariant('PidOutput', PidOut_for_floatSchema)
});

export const DerivedParameterSchema = struct({
  index: u8(),
  name: string(),
  unit: option(ParameterUnitSchema),
  formula: DerivedFormulaSchema
});

export const PeripheralDefinitionSchema = struct({
  peripheral_type: PeripheralTypeSchema,
  location: string(),
  capabilities: seq(SensorCapabilitySchema),
  support_calibration: bool(),
  via_comms_mcu: bool()
});

export const TankDefinitionSchema = struct({
  name: string(),
  sensors: seq(SensorCapabilitySchema)
});

export const SteamWandDefinitionSchema = struct({
  name: string(),
  sensors: seq(SensorCapabilitySchema),
  actuators: seq(ActuatorCapabilitySchema),
  control_modes: seq(ControlModeCapabilitySchema)
});

export const BoilerDefinitionSchema = struct({
  name: string(),
  boiler_type: BoilerTypeSchema,
  sensors: seq(SensorCapabilitySchema),
  actuators: seq(ActuatorCapabilitySchema),
  control_modes: seq(ControlModeCapabilitySchema),
  has_fill_mechanism: bool()
});

export const GroupDefinitionSchema = struct({
  name: string(),
  sensors: seq(SensorCapabilitySchema),
  actuators: seq(ActuatorCapabilitySchema),
  control_modes: seq(ControlModeCapabilitySchema)
});

export const WaterTapDefinitionSchema = struct({
  name: string(),
  sensors: seq(SensorCapabilitySchema),
  actuators: seq(ActuatorCapabilitySchema),
  control_modes: seq(ControlModeCapabilitySchema)
});

export const WaterTapConfigurationSchema = struct({
  pump_strategy: WaterDispersalPumpStrategySchema,
  temperature_target: option(f32()),
  max_dispense_time_seconds: option(u32()),
  flow_rate_limit: option(f32()),
  pump_configuration: option(PumpConfigurationSchema),
  supply_tank_index: option(u8())
});

export const PeripheralStatusSchema = struct({
  peripherals: map(u16(), PeripheralInfoSchema)
});

export const PidParameters_for_floatSchema = struct({
  kp: PidTerm_for_floatSchema,
  ki: PidTerm_for_floatSchema,
  kd: PidTerm_for_floatSchema
});

export const GroupBrewControlStateSchema = struct({
  mode: GroupBrewControlModeSchema,
  values: GroupBrewControlTargetValuesSchema
});

export const ScheduleItemSchema = struct({
  trigger_at: ScheduleTriggerSchema,
  commands: seq(ScheduleActionSchema)
});

export const RoutineExitConditionSchema = enumType('RoutineExitCondition', {
  Always: unitVariant('Always'),
  Never: unitVariant('Never'),
  After: newtypeVariant('After', ParameterValueSchema),
  AfterDurationRelativeToStart: newtypeVariant('AfterDurationRelativeToStart', ParameterValueSchema),
  StateConditionMet: newtypeVariant('StateConditionMet', StateConditionSchema),
  UserAction: newtypeVariant('UserAction', u8())
});

export const BoilerStatusSchema = struct({
  temperature: option(f32()),
  pressure: option(f32()),
  water_level: option(u8()),
  output: OutputSchema,
  control_state: BoilerControlStateSchema
});

export const BoilerConfigurationSchema = struct({
  temperature_pid_parameters: PidParameters_for_floatSchema,
  pressure_pid_parameters: PidParameters_for_floatSchema,
  control_state: BoilerControlStateSchema,
  max_temperature: option(f32()),
  max_pressure: option(f32()),
  temperature_sensor_kalman_parameters: option(KalmanParametersSchema),
  pressure_sensor_kalman_parameters: option(KalmanParametersSchema),
  fill_config: option(FillConfigurationSchema),
  supply_tank_index: option(u8()),
  minimum_safe_level: option(u8())
});

export const GroupStatusSchema = struct({
  is_brewing: bool(),
  three_way_valve_open: option(bool()),
  brew_time: option(DurationSchema),
  brew_input_volume: option(f64()),
  input_flow_rate: option(f32()),
  input_volume: option(f64()),
  output_flow_rate: option(f32()),
  output_weight: option(f32()),
  pressure: option(f32()),
  temperature: option(f32()),
  pump_output: OutputSchema,
  control_state: GroupBrewControlStateSchema,
  previous_brew: option(PreviousBrewInfoSchema),
  shot_state: option(ShotStateSchema)
});

export const GroupConfigurationSchema = struct({
  flow_rate_pid_parameters: PidParameters_for_floatSchema,
  output_flow_rate_pid_parameters: PidParameters_for_floatSchema,
  pressure_pid_parameters: PidParameters_for_floatSchema,
  brew_control_state: GroupBrewControlStateSchema,
  max_brew_time_seconds: option(u32()),
  auto_tare_enabled: bool(),
  pump_configuration: option(PumpConfigurationSchema),
  pressure_sensor_kalman_parameters: option(KalmanParametersSchema),
  flow_sensor_pulses_per_liter: option(f32()),
  supply_tank_index: option(u8())
});

export const RoutineExitSchema = struct({
  condition: RoutineExitConditionSchema,
  then: RoutineStepExitTypeSchema,
  description: option(string())
});

export const RoutineStepSchema = struct({
  entry_command: seq(RoutineCommandSchema),
  exits: seq(RoutineExitSchema),
  description: option(string())
});

export const RoutineSchema = struct({
  routine_type: RoutineTypeSchema,
  name: string(),
  parameters: seq(RoutineParameterSchema),
  derived_parameters: seq(DerivedParameterSchema),
  steps: seq(RoutineStepSchema),
  finally: seq(RoutineCommandSchema)
});

export const StatusSchema = struct({
  boiler_statuses: map(u8(), BoilerStatusSchema),
  group_statuses: map(u8(), GroupStatusSchema),
  water_tap_statuses: map(u8(), WaterTapStatusSchema),
  steam_wand_statuses: map(u8(), SteamWandStatusSchema),
  tank_statuses: map(u8(), TankStatusSchema),
  mode: MachineModeSchema,
  routine_execution: option(RoutineExecutionStatusSchema),
  comms_status: option(CommsStatusSchema),
  peripheral_status: PeripheralStatusSchema,
  current_local_time: option(string())
});

export const ConfigurationSchema = struct({
  machine_config: MachineConfigurationSchema,
  boiler_configurations: map(u8(), BoilerConfigurationSchema),
  group_configurations: map(u8(), GroupConfigurationSchema),
  water_tap_configurations: map(u8(), WaterTapConfigurationSchema),
  tank_configurations: map(u8(), TankConfigurationSchema),
  steam_wand_configurations: map(u8(), SteamWandConfigurationSchema),
  schedules: seq(ScheduleItemSchema)
});

export const MachineDefinitionSchema = struct({
  name: string(),
  boilers: map(u8(), BoilerDefinitionSchema),
  groups: map(u8(), GroupDefinitionSchema),
  water_taps: map(u8(), WaterTapDefinitionSchema),
  tanks: map(u8(), TankDefinitionSchema),
  steam_wands: map(u8(), SteamWandDefinitionSchema),
  environmental_sensors: map(u8(), EnvironmentalSensorDefinitionSchema),
  peripherals: map(u16(), PeripheralDefinitionSchema),
  function_routines: map(u32(), string())
});

export const RoutineStorageSchema = struct({
  internal: map(u32(), RoutineSchema),
  function: map(u32(), RoutineSchema),
  custom: map(u32(), RoutineSchema)
});

export const SetBoilerControlRequestSchema = struct({
  boiler_index: u8(),
  mode: BoilerControlModeSchema,
  target_temperature: option(f32()),
  target_pressure: option(f32())
});

export const SetGroupControlRequestSchema = struct({
  group_index: u8(),
  mode: GroupBrewControlModeSchema,
  duty_cycle: option(u8()),
  flow_rate: option(f32()),
  pressure: option(f32()),
  output_flow_rate: option(f32()),
  duty_cycle_curve: option(ControlCurveSchema),
  flow_rate_curve: option(ControlCurveSchema),
  pressure_curve: option(ControlCurveSchema),
  output_flow_rate_curve: option(ControlCurveSchema)
});

export const SetPidParametersRequestSchema = struct({
  target_type: string(),
  index: u32(),
  pid_parameters: PidParameters_for_floatSchema
});

export const SetGroupPumpConfigurationRequestSchema = struct({
  group_index: u8(),
  pump_configuration: PumpConfigurationSchema
});

export const SetWaterTapPumpConfigurationRequestSchema = struct({
  water_tap_index: u8(),
  pump_configuration: PumpConfigurationSchema
});

export const SetFillPumpConfigurationRequestSchema = struct({
  boiler_index: u8(),
  pump_configuration: PumpConfigurationSchema
});

export type Status = InferType<typeof StatusSchema>;
export type Configuration = InferType<typeof ConfigurationSchema>;
export type MachineDefinition = InferType<typeof MachineDefinitionSchema>;
export type RoutineStorage = InferType<typeof RoutineStorageSchema>;
export type SetBoilerControlRequest = InferType<typeof SetBoilerControlRequestSchema>;
export type SetGroupControlRequest = InferType<typeof SetGroupControlRequestSchema>;
export type SetPidParametersRequest = InferType<typeof SetPidParametersRequestSchema>;
export type SetGroupPumpConfigurationRequest = InferType<typeof SetGroupPumpConfigurationRequestSchema>;
export type SetWaterTapPumpConfigurationRequest = InferType<typeof SetWaterTapPumpConfigurationRequestSchema>;
export type SetFillPumpConfigurationRequest = InferType<typeof SetFillPumpConfigurationRequestSchema>;
export type ActuatorCapability = InferType<typeof ActuatorCapabilitySchema>;
export type BoilerConfiguration = InferType<typeof BoilerConfigurationSchema>;
export type BoilerControlMode = InferType<typeof BoilerControlModeSchema>;
export type BoilerControlState = InferType<typeof BoilerControlStateSchema>;
export type BoilerControlTargetValues = InferType<typeof BoilerControlTargetValuesSchema>;
export type BoilerControlTargetValuesUpdate = InferType<typeof BoilerControlTargetValuesUpdateSchema>;
export type BoilerDefinition = InferType<typeof BoilerDefinitionSchema>;
export type BoilerStatus = InferType<typeof BoilerStatusSchema>;
export type BoilerType = InferType<typeof BoilerTypeSchema>;
export type CommsStatus = InferType<typeof CommsStatusSchema>;
export type ControlCurve = InferType<typeof ControlCurveSchema>;
export type ControlModeCapability = InferType<typeof ControlModeCapabilitySchema>;
export type DerivedFormula = InferType<typeof DerivedFormulaSchema>;
export type DerivedParameter = InferType<typeof DerivedParameterSchema>;
export type Duration = InferType<typeof DurationSchema>;
export type EnvironmentalSensorDefinition = InferType<typeof EnvironmentalSensorDefinitionSchema>;
export type EnvironmentalSensorType = InferType<typeof EnvironmentalSensorTypeSchema>;
export type FillConfiguration = InferType<typeof FillConfigurationSchema>;
export type GroupBrewControlMode = InferType<typeof GroupBrewControlModeSchema>;
export type GroupBrewControlState = InferType<typeof GroupBrewControlStateSchema>;
export type GroupBrewControlTargetValues = InferType<typeof GroupBrewControlTargetValuesSchema>;
export type GroupConfiguration = InferType<typeof GroupConfigurationSchema>;
export type GroupDefinition = InferType<typeof GroupDefinitionSchema>;
export type GroupStatus = InferType<typeof GroupStatusSchema>;
export type KalmanParameters = InferType<typeof KalmanParametersSchema>;
export type Limits_for_float = InferType<typeof Limits_for_floatSchema>;
export type MachineConfiguration = InferType<typeof MachineConfigurationSchema>;
export type MachineMode = InferType<typeof MachineModeSchema>;
export type Output = InferType<typeof OutputSchema>;
export type ParameterUnit = InferType<typeof ParameterUnitSchema>;
export type ParameterValue = InferType<typeof ParameterValueSchema>;
export type PeripheralDefinition = InferType<typeof PeripheralDefinitionSchema>;
export type PeripheralInfo = InferType<typeof PeripheralInfoSchema>;
export type PeripheralStatus = InferType<typeof PeripheralStatusSchema>;
export type PeripheralType = InferType<typeof PeripheralTypeSchema>;
export type PidOut_for_float = InferType<typeof PidOut_for_floatSchema>;
export type PidParameters_for_float = InferType<typeof PidParameters_for_floatSchema>;
export type PidTerm_for_float = InferType<typeof PidTerm_for_floatSchema>;
export type PreviousBrewInfo = InferType<typeof PreviousBrewInfoSchema>;
export type PumpConfiguration = InferType<typeof PumpConfigurationSchema>;
export type Routine = InferType<typeof RoutineSchema>;
export type RoutineCommand = InferType<typeof RoutineCommandSchema>;
export type RoutineExecutionStatus = InferType<typeof RoutineExecutionStatusSchema>;
export type RoutineExit = InferType<typeof RoutineExitSchema>;
export type RoutineExitCondition = InferType<typeof RoutineExitConditionSchema>;
export type RoutineIndex = InferType<typeof RoutineIndexSchema>;
export type RoutineParameter = InferType<typeof RoutineParameterSchema>;
export type RoutineStep = InferType<typeof RoutineStepSchema>;
export type RoutineStepExitType = InferType<typeof RoutineStepExitTypeSchema>;
export type RoutineType = InferType<typeof RoutineTypeSchema>;
export type ScheduleAction = InferType<typeof ScheduleActionSchema>;
export type ScheduleItem = InferType<typeof ScheduleItemSchema>;
export type ScheduleTrigger = InferType<typeof ScheduleTriggerSchema>;
export type ShotState = InferType<typeof ShotStateSchema>;
export type SensorCapability = InferType<typeof SensorCapabilitySchema>;
export type StateCondition = InferType<typeof StateConditionSchema>;
export type SteamWandConfiguration = InferType<typeof SteamWandConfigurationSchema>;
export type SteamWandDefinition = InferType<typeof SteamWandDefinitionSchema>;
export type TankConfiguration = InferType<typeof TankConfigurationSchema>;
export type TankDefinition = InferType<typeof TankDefinitionSchema>;
export type TankStatus = InferType<typeof TankStatusSchema>;
export type WaterDispersalPumpStrategy = InferType<typeof WaterDispersalPumpStrategySchema>;
export type WaterTapConfiguration = InferType<typeof WaterTapConfigurationSchema>;
export type WaterTapDefinition = InferType<typeof WaterTapDefinitionSchema>;
export type WaterTapStatus = InferType<typeof WaterTapStatusSchema>;
export type SteamWandStatus = InferType<typeof SteamWandStatusSchema>;

// Map entry type aliases for proper typing in components
export type BoilerEntry = [number, BoilerDefinition];
export type GroupEntry = [number, GroupDefinition];
export type WaterTapEntry = [number, WaterTapDefinition];
export type SteamWandEntry = [number, SteamWandDefinition];
export type TankEntry = [number, TankDefinition];
export type BoilerStatusEntry = [number, BoilerStatus];
export type GroupStatusEntry = [number, GroupStatus];
export type WaterTapStatusEntry = [number, WaterTapStatus];
export type SteamWandStatusEntry = [number, SteamWandStatus];
export type RoutineEntry = [number, Routine];

// Additional types for WebSocket support

export const HeatingElementContentionStrategySchema = enumType('HeatingElementContentionStrategy', {
  RoundRobin: unitVariant('RoundRobin'),
  PriorityBased: unitVariant('PriorityBased')
});

export const PidParameterTargetSchema = enumType('PidParameterTarget', {
  BoilerTemperature: newtypeVariant('BoilerTemperature', u8()),
  BoilerPressure: newtypeVariant('BoilerPressure', u8()),
  GroupFlowRate: newtypeVariant('GroupFlowRate', u8()),
  GroupOutputFlowRate: newtypeVariant('GroupOutputFlowRate', u8()),
  GroupPressure: newtypeVariant('GroupPressure', u8())
});

export const GroupBrewControlTargetValuesUpdateSchema = struct({
  flow_rate: option(f32()),
  flow_rate_curve: option(ControlCurveSchema),
  pressure: option(f32()),
  pressure_curve: option(ControlCurveSchema),
  output_flow_rate: option(f32()),
  output_flow_rate_curve: option(ControlCurveSchema),
  duty_cycle: option(u8()),
  duty_cycle_curve: option(ControlCurveSchema)
});

// MachineCommand enum - comprehensive variant coverage
export const MachineCommandSchema = enumType('MachineCommand', {
  StartBrewing: newtypeVariant('StartBrewing', u8()),
  StopBrewing: newtypeVariant('StopBrewing', u8()),
  StartPumpingToWaterTap: newtypeVariant('StartPumpingToWaterTap', u8()),
  StopPumpingToWaterTap: newtypeVariant('StopPumpingToWaterTap', u8()),
  StartSteaming: newtypeVariant('StartSteaming', u8()),
  StopSteaming: newtypeVariant('StopSteaming', u8()),
  SetSteamValveOpenness: tupleVariant('SetSteamValveOpenness', u8(), u8()),
  SetBoilerControlTarget: tupleVariant('SetBoilerControlTarget', u8(), BoilerControlModeSchema, option(BoilerControlTargetValuesUpdateSchema)),
  SetBoilerControlTargetValues: tupleVariant('SetBoilerControlTargetValues', u8(), BoilerControlTargetValuesUpdateSchema),
  SetGroupBrewControlTarget: tupleVariant('SetGroupBrewControlTarget', u8(), GroupBrewControlModeSchema, option(GroupBrewControlTargetValuesUpdateSchema)),
  SetGroupBrewControlTargetValues: tupleVariant('SetGroupBrewControlTargetValues', u8(), GroupBrewControlTargetValuesUpdateSchema),
  SetPidParameters: tupleVariant('SetPidParameters', PidParameterTargetSchema, PidParameters_for_floatSchema),
  RunRoutine: tupleVariant('RunRoutine', RoutineIndexSchema, option(map(u8(), f32()))),
  CancelRoutine: unitVariant('CancelRoutine'),
  EnableBoiler: newtypeVariant('EnableBoiler', u8()),
  DisableBoiler: newtypeVariant('DisableBoiler', u8()),
  TareGroupScale: newtypeVariant('TareGroupScale', u8()),
  ZeroCalibrateGroupScale: newtypeVariant('ZeroCalibrateGroupScale', u8()),
  CalibrateGroupScale100g: newtypeVariant('CalibrateGroupScale100g', u8()),
  UpdateCommsStatus: newtypeVariant('UpdateCommsStatus', CommsStatusSchema),
  AddScheduleItem: newtypeVariant('AddScheduleItem', ScheduleItemSchema),
  RemoveScheduleItem: newtypeVariant('RemoveScheduleItem', u32()),
  UpdateScheduleItem: tupleVariant('UpdateScheduleItem', u32(), ScheduleItemSchema),
  AddRoutine: newtypeVariant('AddRoutine', RoutineSchema),
  RemoveRoutine: newtypeVariant('RemoveRoutine', RoutineIndexSchema),
  UpdateRoutine: tupleVariant('UpdateRoutine', RoutineIndexSchema, RoutineSchema),
  SetMachineMode: newtypeVariant('SetMachineMode', MachineModeSchema),
  OptimizeConfigurationStorage: unitVariant('OptimizeConfigurationStorage'),
  OptimizeRoutineStorage: unitVariant('OptimizeRoutineStorage'),
  OptimizeScheduleStorage: unitVariant('OptimizeScheduleStorage'),
  SetGroupPumpConfiguration: tupleVariant('SetGroupPumpConfiguration', u8(), PumpConfigurationSchema),
  SetWaterTapPumpConfiguration: tupleVariant('SetWaterTapPumpConfiguration', u8(), PumpConfigurationSchema),
  SetFillPumpConfiguration: tupleVariant('SetFillPumpConfiguration', u8(), PumpConfigurationSchema),
  InferGroupPressureIntegral: tupleVariant('InferGroupPressureIntegral', u8(), f32()),
  InferGroupFlowRateIntegral: tupleVariant('InferGroupFlowRateIntegral', u8(), f32()),
  InferGroupOutputFlowRateIntegral: tupleVariant('InferGroupOutputFlowRateIntegral', u8(), f32()),
  SetHeatingElementInterlock: newtypeVariant('SetHeatingElementInterlock', bool()),
  SetHeatingElementContentionStrategy: newtypeVariant('SetHeatingElementContentionStrategy', HeatingElementContentionStrategySchema),
  SetWaterDispersalPumpStrategy: tupleVariant('SetWaterDispersalPumpStrategy', u8(), WaterDispersalPumpStrategySchema)
});

// CommandAck for acknowledgements
export const CommandAckSchema = struct({
  id: u32(),
  success: bool(),
  error: option(string())
});

// WsMessage enum for WebSocket communication
export const WsMessageSchema = enumType('WsMessage', {
  // Server -> Client
  StatusUpdate: newtypeVariant('StatusUpdate', StatusSchema),
  ConfigurationUpdate: newtypeVariant('ConfigurationUpdate', ConfigurationSchema),
  MachineDefinition: newtypeVariant('MachineDefinition', MachineDefinitionSchema),
  RoutinesUpdate: newtypeVariant('RoutinesUpdate', RoutineStorageSchema),
  CommandAck: newtypeVariant('CommandAck', CommandAckSchema),
  // Client -> Server
  RequestMachineDefinition: unitVariant('RequestMachineDefinition'),
  RequestRoutines: unitVariant('RequestRoutines'),
  SendMachineCommand: newtypeVariant('SendMachineCommand', MachineCommandSchema)
});

export type MachineCommand = InferType<typeof MachineCommandSchema>;
export type WsMessage = InferType<typeof WsMessageSchema>;
export type CommandAck = InferType<typeof CommandAckSchema>;
export type PidParameterTarget = InferType<typeof PidParameterTargetSchema>;
export type HeatingElementContentionStrategy = InferType<typeof HeatingElementContentionStrategySchema>;
export type GroupBrewControlTargetValuesUpdate = InferType<typeof GroupBrewControlTargetValuesUpdateSchema>;
