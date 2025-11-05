//! Comprehensive integration test that generates a schema matching schema.ts
//!
//! This test uses the actual types from variegated-controller-types and validates
//! that the generated TypeScript schema matches the expected structure in schema.ts.

use variegated_postcard_ts_typegen::{
    impl_postcard_ts_for_concrete, PostcardTsTypegen, SchemaGenerator,
};

// ============================================================================
// Primitive enums (simple unit variants)
// ============================================================================

#[derive(PostcardTsTypegen)]
enum PeripheralType {
    Scale,
    PressureSensor,
    FlowMeter,
    LevelSensor,
}

#[derive(PostcardTsTypegen)]
enum ParameterUnit {
    Seconds,
    Celsius,
    Bar,
    MillilitersPerSecond,
    Grams,
    Percent,
}

#[derive(PostcardTsTypegen)]
enum BoilerType {
    BrewBoiler,
    SteamBoiler,
    VirtualSteamBoiler,
}

#[derive(PostcardTsTypegen)]
enum GroupBrewControlMode {
    GroupFlowRate,
    GroupFlowRateCurve,
    Pressure,
    PressureCurve,
    OutputFlowRate,
    OutputFlowRateCurve,
    FixedDutyCycle,
    FixedDutyCycleCurve,
    FullOn,
    Off,
}

#[derive(PostcardTsTypegen)]
enum EnvironmentalSensorType {
    AmbientTemperature,
    CaseTemperature,
    ExternalTemperature,
    Humidity,
}

#[derive(PostcardTsTypegen)]
enum RoutineType {
    HeatUp,
    UserDefined,
    Cleaning,
    HardwareButtonMapped,
}

#[derive(PostcardTsTypegen)]
enum MachineMode {
    On,
    Off,
    PowerSaveStandby,
}

#[derive(PostcardTsTypegen)]
enum ShotState {
    HeadspaceFill,
    Saturation,
    PostFirstDrop,
}

#[derive(PostcardTsTypegen)]
enum ControlModeCapability {
    TemperaturePid,
    PressurePid,
    FlowRatePid,
    OutputFlowRatePid,
    FixedDutyCycle,
    FullOn,
    Off,
}

#[derive(PostcardTsTypegen)]
enum BoilerControlMode {
    Temperature,
    Pressure,
    Off,
}

#[derive(PostcardTsTypegen)]
enum SensorCapability {
    Temperature,
    Pressure,
    WaterLevel,
    InputFlowRate,
    OutputFlowRate,
    Weight,
}

#[derive(PostcardTsTypegen)]
enum ActuatorCapability {
    HeatingElement,
    Pump,
    SolenoidValve,
    ThreeWayValve,
    WaterMixer,
    ScaleTare,
}

#[derive(PostcardTsTypegen)]
enum WaterDispersalPumpStrategy {
    AlwaysPump,
    NoPump,
}

// ============================================================================
// Simple structs
// ============================================================================

#[derive(PostcardTsTypegen)]
struct SteamWandConfiguration {
    temperature_target: Option<f32>,
    pressure_target: Option<f32>,
    purge_time_seconds: Option<u32>,
    max_steam_time_seconds: Option<u32>,
    auto_purge_enabled: bool,
}

#[derive(PostcardTsTypegen)]
struct TankStatus {
    water_level: Option<u8>,
}

#[derive(PostcardTsTypegen)]
struct BoilerControlTargetValuesUpdate {
    temperature: Option<f32>,
    pressure: Option<f32>,
}

#[derive(PostcardTsTypegen)]
struct PumpConfiguration {
    tacho_pulses_per_liter: Option<f32>,
    max_duty_cycle: Option<u8>,
    min_duty_cycle: Option<u8>,
    ramp_up_time_ms: Option<u32>,
    ramp_down_time_ms: Option<u32>,
}

// Note: This is named 'Duration' to generate 'DurationSchema'
// (the derive macro appends 'Schema' to the type name)
#[derive(PostcardTsTypegen)]
struct Duration {
    secs: u64,
    nanos: u32,
}

#[derive(PostcardTsTypegen)]
struct ScheduleTrigger {
    on_minute: u8,
    on_hour: u8,
    on_days: Option<Vec<String>>,
    on_date: Option<String>,
    enabled: bool,
    once: bool,
}

#[derive(PostcardTsTypegen)]
struct BoilerControlTargetValues {
    target_temperature: f32,
    target_pressure: f32,
}

#[derive(PostcardTsTypegen)]
struct MachineConfiguration {
    heating_element_interlock: bool,
    max_shot_logs: u32,
    log_sample_decimation: u8,
}

#[derive(PostcardTsTypegen)]
struct ControlCurve {
    a: f32,
    b: f32,
    c: f32,
    min: f32,
    max: f32,
}

#[derive(PostcardTsTypegen)]
struct WaterTapStatus {
    is_dispensing: bool,
}

#[derive(PostcardTsTypegen)]
struct KalmanParameters {
    process_noise: f32,
    measurement_noise: f32,
    estimated_error: f32,
    posterior_estimate: f32,
}

#[derive(PostcardTsTypegen)]
struct CommsStatus {
    timestamp: Option<u64>,
    wifi_connected: bool,
    wifi_rssi: Option<i8>,
}

#[derive(PostcardTsTypegen)]
struct PreviousBrewInfo {
    brew_time: Duration,
    brew_input_volume: Option<f64>,
    output_weight: Option<f32>,
    started_at_millis: u64,
    stopped_at_millis: u64,
}

#[derive(PostcardTsTypegen)]
struct GroupBrewControlTargetValues {
    flow_rate: f32,
    flow_rate_curve: ControlCurve,
    pressure: f32,
    pressure_curve: ControlCurve,
    output_flow_rate: f32,
    output_flow_rate_curve: ControlCurve,
    duty_cycle: u8,
    duty_cycle_curve: ControlCurve,
}

#[derive(PostcardTsTypegen)]
struct TankConfiguration {
    low_level_warning_threshold: Option<u8>,
    water_level_sensor_kalman_parameters: Option<KalmanParameters>,
}

#[derive(PostcardTsTypegen)]
struct BoilerControlState {
    mode: BoilerControlMode,
    values: BoilerControlTargetValues,
}

#[derive(PostcardTsTypegen)]
struct PeripheralInfo {
    peripheral_type: PeripheralType,
    is_available: bool,
}

#[derive(PostcardTsTypegen)]
struct RoutineParameter {
    index: u8,
    name: String,
    default: f32,
    unit: Option<ParameterUnit>,
}

#[derive(PostcardTsTypegen)]
struct FillConfiguration {
    fill_threshold: Option<u8>,
    pump_configuration: Option<PumpConfiguration>,
}

#[derive(PostcardTsTypegen)]
struct EnvironmentalSensorDefinition {
    name: String,
    sensor_type: EnvironmentalSensorType,
    measurement_range: Option<Vec<()>>,  // Empty tuple for unit struct in original
}

#[derive(PostcardTsTypegen)]
struct GroupBrewControlState {
    mode: GroupBrewControlMode,
    values: GroupBrewControlTargetValues,
}

#[derive(PostcardTsTypegen)]
struct PeripheralDefinition {
    peripheral_type: PeripheralType,
    location: String,
    capabilities: Vec<SensorCapability>,
    support_calibration: bool,
    via_comms_mcu: bool,
}

#[derive(PostcardTsTypegen)]
struct TankDefinition {
    name: String,
    sensors: Vec<SensorCapability>,
}

#[derive(PostcardTsTypegen)]
struct SteamWandDefinition {
    name: String,
    sensors: Vec<SensorCapability>,
    actuators: Vec<ActuatorCapability>,
    control_modes: Vec<ControlModeCapability>,
}

#[derive(PostcardTsTypegen)]
struct BoilerDefinition {
    name: String,
    boiler_type: BoilerType,
    sensors: Vec<SensorCapability>,
    actuators: Vec<ActuatorCapability>,
    control_modes: Vec<ControlModeCapability>,
    has_fill_mechanism: bool,
}

#[derive(PostcardTsTypegen)]
struct GroupDefinition {
    name: String,
    sensors: Vec<SensorCapability>,
    actuators: Vec<ActuatorCapability>,
    control_modes: Vec<ControlModeCapability>,
}

#[derive(PostcardTsTypegen)]
struct WaterTapDefinition {
    name: String,
    sensors: Vec<SensorCapability>,
    actuators: Vec<ActuatorCapability>,
    control_modes: Vec<ControlModeCapability>,
}

#[derive(PostcardTsTypegen)]
struct WaterTapConfiguration {
    pump_strategy: WaterDispersalPumpStrategy,
    temperature_target: Option<f32>,
    max_dispense_time_seconds: Option<u32>,
    flow_rate_limit: Option<f32>,
    pump_configuration: Option<PumpConfiguration>,
}

// ============================================================================
// Enums with variants
// ============================================================================

#[derive(PostcardTsTypegen)]
enum RoutineIndex {
    Internal(u32),
    Function(u32),
    Custom(u32),
}

#[derive(PostcardTsTypegen)]
enum RoutineStepExitType {
    NextStep,
    JumpToStep(u32),
    Finished,
}

#[derive(PostcardTsTypegen)]
enum ParameterValue {
    Static(f32),
    Parameter(u8),
    DerivedParameter(u8),
}

#[derive(PostcardTsTypegen)]
enum DerivedFormula {
    Linear { base_param: u8, multiplier: f32, offset: f32 },
    Sum { params: Vec<u8> },
    Difference { param_a: u8, param_b: u8 },
    Product { params: Vec<u8> },
}

#[derive(PostcardTsTypegen)]
enum Output {
    Off,
    FixedDutyCycle(u8),
    PidOutput(PidOut_for_float),
}

// ============================================================================
// PID-related types (concretized generics)
// ============================================================================

// Mock PID types for testing
#[derive(Debug, Clone)]
pub struct Limits<T> {
    pub lower: T,
    pub upper: T,
}

#[derive(Debug, Clone)]
pub struct PidTerm<T> {
    pub positive_scale: T,
    pub negative_scale: T,
    pub limits: Limits<T>,
}

#[derive(Debug, Clone)]
pub struct PidParameters<T> {
    pub kp: PidTerm<T>,
    pub ki: PidTerm<T>,
    pub kd: PidTerm<T>,
}

#[derive(Debug, Clone)]
pub struct PidOut<T> {
    pub p: T,
    pub i: T,
    pub d: T,
    pub out: T,
    pub acting_kp: T,
    pub acting_ki: T,
    pub acting_kd: T,
}

// Implement concrete f32 versions
impl_postcard_ts_for_concrete!(
    Limits<f32> => "Limits_for_float",
    struct {
        lower: "f32",
        upper: "f32",
    }
);

impl_postcard_ts_for_concrete!(
    PidTerm<f32> => "PidTerm_for_float",
    struct {
        positive_scale: "f32",
        negative_scale: "f32",
        limits: "Limits_for_float",
    }
);

impl_postcard_ts_for_concrete!(
    PidParameters<f32> => "PidParameters_for_float",
    struct {
        kp: "PidTerm_for_float",
        ki: "PidTerm_for_float",
        kd: "PidTerm_for_float",
    }
);

// For use in Output enum
pub type PidOut_for_float = PidOut<f32>;

impl_postcard_ts_for_concrete!(
    PidOut<f32> => "PidOut_for_float",
    struct {
        p: "f32",
        i: "f32",
        d: "f32",
        out: "f32",
        acting_kp: "f32",
        acting_ki: "f32",
        acting_kd: "f32",
    }
);

// ============================================================================
// Tests
// ============================================================================

#[test]
fn test_generate_full_schema() {
    let mut generator = SchemaGenerator::new();

    // Add PID types first (dependencies)
    generator.add::<Limits<f32>>();
    generator.add::<PidTerm<f32>>();
    generator.add::<PidParameters<f32>>();
    generator.add::<PidOut<f32>>();

    // Add simple enums
    generator.add::<PeripheralType>();
    generator.add::<ParameterUnit>();
    generator.add::<BoilerType>();
    generator.add::<GroupBrewControlMode>();
    generator.add::<EnvironmentalSensorType>();
    generator.add::<RoutineType>();
    generator.add::<MachineMode>();
    generator.add::<ShotState>();
    generator.add::<ControlModeCapability>();
    generator.add::<BoilerControlMode>();
    generator.add::<SensorCapability>();
    generator.add::<ActuatorCapability>();
    generator.add::<WaterDispersalPumpStrategy>();

    // Add structs
    generator.add::<SteamWandConfiguration>();
    generator.add::<TankStatus>();
    generator.add::<BoilerControlTargetValuesUpdate>();
    generator.add::<PumpConfiguration>();
    generator.add::<Duration>();
    generator.add::<ScheduleTrigger>();
    generator.add::<BoilerControlTargetValues>();
    generator.add::<MachineConfiguration>();
    generator.add::<ControlCurve>();
    generator.add::<WaterTapStatus>();
    generator.add::<KalmanParameters>();
    generator.add::<CommsStatus>();
    generator.add::<PreviousBrewInfo>();
    generator.add::<GroupBrewControlTargetValues>();
    generator.add::<TankConfiguration>();
    generator.add::<BoilerControlState>();
    generator.add::<PeripheralInfo>();
    generator.add::<RoutineParameter>();
    generator.add::<FillConfiguration>();
    generator.add::<EnvironmentalSensorDefinition>();
    generator.add::<GroupBrewControlState>();
    generator.add::<PeripheralDefinition>();
    generator.add::<TankDefinition>();
    generator.add::<SteamWandDefinition>();
    generator.add::<BoilerDefinition>();
    generator.add::<GroupDefinition>();
    generator.add::<WaterTapDefinition>();
    generator.add::<WaterTapConfiguration>();

    // Add complex enums
    generator.add::<RoutineIndex>();
    generator.add::<RoutineStepExitType>();
    generator.add::<ParameterValue>();
    generator.add::<DerivedFormula>();
    generator.add::<Output>();

    let output = generator.generate();

    // Print for inspection
    println!("\n{}", "=".repeat(80));
    println!("Generated Schema:");
    println!("{}", "=".repeat(80));
    println!("{}", output);
    println!("{}", "=".repeat(80));

    // Load the expected schema.ts file
    let expected = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../schema.ts"))
        .expect("Failed to read schema.ts file");

    // Compare the full output to the expected schema
    // This is CRITICAL to ensure the output is correct TypeScript code,
    // both from a syntax standpoint and from a semantics standpoint.
    pretty_assertions::assert_eq!(output, expected,
        "Generated schema does not match expected schema.ts file");
}

#[test]
fn test_pid_types_naming() {
    let mut generator = SchemaGenerator::new();

    generator.add::<Limits<f32>>();
    generator.add::<PidTerm<f32>>();
    generator.add::<PidParameters<f32>>();
    generator.add::<PidOut<f32>>();

    let output = generator.generate();

    // Verify naming follows schema.ts convention
    assert!(output.contains("Limits_for_floatSchema"));
    assert!(output.contains("PidTerm_for_floatSchema"));
    assert!(output.contains("PidParameters_for_floatSchema"));
    assert!(output.contains("PidOut_for_floatSchema"));

    // Verify Pid types reference each other correctly
    assert!(output.contains("limits: Limits_for_floatSchema"));
    assert!(output.contains("kp: PidTerm_for_floatSchema"));
}

#[test]
fn test_enum_with_struct_variants() {
    let mut generator = SchemaGenerator::new();
    generator.add::<DerivedFormula>();

    let output = generator.generate();

    // Validate DerivedFormula enum structure matches schema.ts
    assert!(output.contains("enumType('DerivedFormula', {"));
    assert!(output.contains("Linear: newtypeVariant('Linear', struct({"));
    assert!(output.contains("base_param: u8()"));
    assert!(output.contains("multiplier: f32()"));
    assert!(output.contains("offset: f32()"));

    assert!(output.contains("Sum: newtypeVariant('Sum', struct({"));
    assert!(output.contains("params: seq(u8())"));

    assert!(output.contains("Difference: newtypeVariant('Difference', struct({"));
    assert!(output.contains("param_a: u8()"));
    assert!(output.contains("param_b: u8()"));

    assert!(output.contains("Product: newtypeVariant('Product', struct({"));
}

#[test]
fn test_output_enum_with_pid() {
    let mut generator = SchemaGenerator::new();

    // Add PidOut first as it's a dependency
    generator.add::<PidOut<f32>>();
    generator.add::<Output>();

    let output = generator.generate();

    // Validate Output enum references PidOut correctly
    assert!(output.contains("OutputSchema"));
    assert!(output.contains("Off: unitVariant('Off')"));
    assert!(output.contains("FixedDutyCycle: newtypeVariant('FixedDutyCycle', u8())"));
    assert!(output.contains("PidOutput: newtypeVariant('PidOutput', PidOut_for_floatSchema)"));
}
