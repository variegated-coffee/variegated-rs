//! Sample values, serialized both as postcard bytes and as JSON, for the TypeScript
//! round-trip harness to check itself against.
//!
//! # What this is actually testing
//!
//! Not "did we transcribe the schema correctly" -- that is no longer possible to get
//! wrong, since the schema is generated from the same declarations that produce the
//! bytes. What remains untested by construction is whether
//! `@variegated-coffee/serde-postcard-ts` and Rust's postcard agree about *composite*
//! structures: nested maps of structs, options inside options, enum payloads. That is a
//! third-party implementation, and this is what checks it.
//!
//! # Why two encodings
//!
//! The byte comparison catches wrong node kinds, wrong nesting, wrong field counts and
//! wrong variant order. It is blind to field *names* and to two same-typed fields being
//! swapped, because postcard encodes neither. Only the JSON comparison catches those,
//! and that is the exact bug class the hand-written file suffered from. Both are needed;
//! neither subsumes the other.
//!
//! # Rules for the values below
//!
//! - **Exhaustive struct literals, never `..Default::default()`.** This is the load
//!   bearing part: a new field on a wire type becomes a compile error *here*, in the
//!   same commit that adds it. No runtime check can do that. Several of these types
//!   have hand-written `Default` impls that would silently swallow a new field.
//! - **No `Limits::default()`.** It is `{lower: -inf, upper: +inf}`, and serde_json
//!   writes infinity as `null` -- an irreversible, deeply confusing comparison failure.
//!   Every float here is finite and exactly f32-representable, so the two decoders
//!   cannot disagree about rounding.
//! - **Every `u64` stays below 2^53**, because `JSON.parse` silently loses precision
//!   above that.
//! - **Timestamps have no subsecond component**, since chrono's `DateTime` serializes
//!   with `SecondsFormat::AutoSi` and the fraction length varies with the value.
//! - **Maps hold at least two entries.** A one-entry map hides length-prefix bugs.
//! - **Enum-typed fields prefer a non-first variant**, so a variant-index off-by-one
//!   shows up instead of encoding as zero either way.

use std::path::Path;

use std::collections::BTreeMap;

use heapless::index_map::FnvIndexMap;
use serde::Serialize;
use variegated_comms_api_types::api_types::RoutineSummaryStorage;
use variegated_comms_api_types::ws_types::WsMessage;
use variegated_control_algorithm::pid::{Limits, PidOut};
use variegated_controller_types::*;

/// One fixture: a name, the postcard bytes, and the same value as JSON.
pub struct Fixture {
    pub name: &'static str,
    /// The generated schema to decode it with, e.g. `StatusSchema`.
    pub schema: &'static str,
    pub bytes: Vec<u8>,
    pub json: String,
}

fn fixture<T: Serialize>(name: &'static str, schema: &'static str, value: &T) -> Fixture {
    Fixture {
        name,
        schema,
        bytes: postcard::to_allocvec(value).expect("fixture must serialize"),
        json: serde_json::to_string_pretty(value).expect("fixture must serialize as JSON"),
    }
}

fn limits(lower: f32, upper: f32) -> PidLimits {
    // The public constructor, because the fields are private. Deliberately not
    // `Limits::default()` -- see the module note about infinity.
    Limits::new_with_limits(lower, upper).expect("valid limits")
}

fn pid_term(positive: f32, negative: f32) -> PidTerm {
    PidTerm { positive_scale: positive, negative_scale: negative, limits: limits(-100.0, 100.0) }
}

fn pid_parameters() -> PidParameters {
    PidParameters {
        kp: pid_term(2.0, 1.5),
        ki: pid_term(0.5, 0.25),
        kd: pid_term(0.125, 0.0625),
    }
}

fn control_curve() -> ControlCurve {
    ControlCurve { a: 0.5, b: 1.5, c: 2.0, min: 0.0, max: 9.0 }
}

fn boiler_control_state() -> BoilerControlState {
    BoilerControlState {
        // Not the first variant: an index error would otherwise encode as 0 regardless.
        mode: BoilerControlMode::Pressure,
        values: BoilerControlTargetValues { target_temperature: 93.0, target_pressure: 9.0 },
    }
}

fn group_control_state() -> GroupBrewControlState {
    GroupBrewControlState {
        mode: GroupBrewControlMode::PressureCurve,
        values: GroupBrewControlTargetValues {
            flow_rate: 2.5,
            flow_rate_curve: control_curve(),
            pressure: 9.0,
            pressure_curve: control_curve(),
            output_flow_rate: 1.5,
            output_flow_rate_curve: control_curve(),
            duty_cycle: 75,
            duty_cycle_curve: control_curve(),
        },
    }
}

fn pump_configuration() -> PumpConfiguration {
    PumpConfiguration {
        tacho_pulses_per_liter: Some(1000.0),
        max_duty_cycle: Some(100),
        min_duty_cycle: Some(10),
        ramp_up_time_ms: Some(500),
        ramp_down_time_ms: Some(250),
    }
}

/// A `Status` with every `Option` populated and every map holding two entries.
///
/// The point of a maximal value is that `Status::default()` is a legal fixture that
/// serializes to a handful of zero bytes and proves nothing about the 90% of the tree
/// hidden behind `Option`s -- including `comms_status`, which is where the drift that
/// motivated all of this was hiding.
fn status_maximal() -> Status {
    let mut boiler_statuses = FnvIndexMap::new();
    for i in 0..2u8 {
        boiler_statuses
            .insert(
                i,
                BoilerStatus {
                    temperature: Some(93.0),
                    pressure: Some(9.0),
                    water_level: Some(80),
                    // Exercises a newtype variant carrying a struct.
                    output: Output::PidOutput(PidOut::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)),
                    control_state: boiler_control_state(),
                },
            )
            .expect("fits");
    }

    let mut group_statuses = FnvIndexMap::new();
    for i in 0..2u8 {
        group_statuses
            .insert(
                i,
                GroupStatus {
                    is_brewing: true,
                    three_way_valve_open: Some(true),
                    current_brew: Some(BrewStatus {
                        brew_time: core::time::Duration::new(25, 500_000_000),
                        brew_input_volume: Some(36.0),
                        shot_state: Some(ShotState::PostFirstDrop),
                        extracted_solids: Some(2.5),
                        output_volume: Some(30.0),
                    }),
                    input_flow_rate: Some(2.5),
                    input_volume: Some(40.0),
                    output_flow_rate: Some(1.5),
                    output_weight: Some(36.0),
                    pressure: Some(9.0),
                    temperature: Some(93.0),
                    output_temperature: Some(88.0),
                    output_electrical_conductivity: Some(0.5),
                    extraction_rate: Some(1.25),
                    pump_output: Output::FixedDutyCycle(80),
                    control_state: group_control_state(),
                    previous_brew: Some(PreviousBrewInfo {
                        brew_time: core::time::Duration::new(27, 0),
                        brew_input_volume: Some(38.0),
                        output_weight: Some(36.5),
                        started_at_millis: 1_000_000,
                        stopped_at_millis: 1_027_000,
                    }),
                },
            )
            .expect("fits");
    }

    let mut water_tap_statuses = FnvIndexMap::new();
    let mut steam_wand_statuses = FnvIndexMap::new();
    let mut tank_statuses = FnvIndexMap::new();
    for i in 0..2u8 {
        water_tap_statuses.insert(i, WaterTapStatus { is_dispensing: true }).expect("fits");
        steam_wand_statuses
            .insert(i, SteamWandStatus { is_steaming: true, valve_openness: 100 })
            .expect("fits");
        tank_statuses.insert(i, TankStatus { water_level: Some(75) }).expect("fits");
    }

    let mut resolved_parameters = FnvIndexMap::new();
    resolved_parameters.insert(0u8, 93.0f32).expect("fits");
    resolved_parameters.insert(1u8, 36.0f32).expect("fits");

    let mut peripheral_connection_status = FnvIndexMap::new();
    for i in 0..2u16 {
        peripheral_connection_status
            .insert(0xF000 + i, WirelessConnectionStatus { connected: true, rssi: Some(-70) })
            .expect("fits");
    }

    let mut peripherals = FnvIndexMap::new();
    for i in 0..2u16 {
        peripherals
            .insert(
                0xE000 + i,
                PeripheralInfo {
                    peripheral_type: PeripheralType::BrewSensor,
                    is_available: true,
                },
            )
            .expect("fits");
    }

    Status {
        boiler_statuses,
        group_statuses,
        water_tap_statuses,
        steam_wand_statuses,
        tank_statuses,
        mode: MachineMode::PowerSaveStandby,
        routine_execution: Some(RoutineExecutionStatus {
            routine_index: RoutineIndex::Custom(3),
            current_step: Some(2),
            step_elapsed_time: Some(core::time::Duration::new(5, 0)),
            total_elapsed_time: Some(core::time::Duration::new(42, 0)),
            resolved_parameters,
        }),
        // The field that was missing from the hand-written schema, which is why any
        // Status carrying comms state mis-decoded everything after it.
        comms_status: Some(CommsStatus {
            timestamp: Some(1_700_000_000),
            wifi_connected: true,
            wifi_rssi: Some(-55),
            // Deliberately not `Stopped`, which is the `Default`. A discriminant of zero
            // encodes to a byte that a decoder reading the wrong field would produce by
            // accident, so a fixture pinned to the default would pass whether or not the
            // schema knew this field existed -- which is the exact drift the note at the
            // top of this function is about.
            improv: ImprovState::Provisioning,
            peripheral_connection_status,
            // Non-zero for the same reason `improv` is not `Stopped`: zero is what a
            // decoder that does not know this field exists would produce by accident.
            sntp_sync_seq: 7,
        }),
        // Subsecond, so the `nanos` half of `Duration` is exercised rather than left at
        // zero -- postcard encodes the two varints separately.
        comms_status_age: Some(core::time::Duration::new(1, 250_000_000)),
        peripheral_status: PeripheralStatus { peripherals },
        // No subsecond component: chrono's formatting is value-dependent.
        current_local_time: Some(
            chrono::NaiveDate::from_ymd_opt(2026, 8, 7)
                .expect("valid date")
                .and_hms_opt(9, 30, 0)
                .expect("valid time"),
        ),
        bluetooth: BluetoothScanStatus {
            scanning: true,
            blocked: true,
            reports_dropped: 3,
            // Two entries, and deliberately not the same shape as each other: one
            // random-address device with a name, one public-address device with none.
            // A single entry would let a decoder that mixed up `address_random` and the
            // name's length prefix still round-trip.
            discovered: heapless::Vec::from_slice(&[
                DiscoveredBluetoothPeripheral {
                    address: [0x2F, 0xA0, 0x1A, 0x97, 0x1C, 0x00],
                    address_random: true,
                    name: bluetooth_name("ACAIA-1C00"),
                    rssi: -63,
                    suggested_driver: Some(BluetoothDriverKind::AcaiaOld),
                },
                DiscoveredBluetoothPeripheral {
                    address: [0x3E, 0x60, 0xEB, 0x3C, 0x1C, 0x78],
                    address_random: false,
                    name: BluetoothName::new(),
                    rssi: -91,
                    // The unrecognised case, so the `Option` discriminant is exercised in
                    // both directions within one fixture.
                    suggested_driver: None,
                },
            ])
            .expect("fits"),
        },
        // One of each value variant, and both kinds of key -- a named one and an
        // `Other`. A block of three identical-shaped entries would round-trip even if
        // the decoder confused the key's discriminant with the value's.
        pending_shot_annotations: {
            let mut annotations = ShotAnnotations::new();
            annotations
                .set(ShotAnnotationKey::DoseWeight, ShotAnnotationValue::Number(18.5))
                .expect("fits");
            annotations
                .set(
                    ShotAnnotationKey::Beans,
                    ShotAnnotationValue::Text(
                        heapless::String::try_from("Drop Coffee / Kenya Karimikui")
                            .expect("fits"),
                    ),
                )
                .expect("fits");
            annotations
                .set(
                    ShotAnnotationKey::Other(
                        heapless::String::try_from("basket").expect("fits"),
                    ),
                    ShotAnnotationValue::Text(
                        heapless::String::try_from("VST 18g").expect("fits"),
                    ),
                )
                .expect("fits");
            annotations
        },
        // `Some(true)` here against `None` in the minimal fixture. A field carrying the
        // same value in every fixture is one the round-trip test cannot tell apart from
        // a hard-coded constant -- and `Option` is exactly where postcard's encoding
        // differs between the cases.
        sd_card_present: Some(true),
    }
}

/// The other extreme: everything absent, every map empty.
///
/// Checks that the decoder agrees about *absence* -- an option discriminant read one
/// byte off looks like a populated value full of garbage.
fn status_minimal() -> Status {
    Status {
        boiler_statuses: FnvIndexMap::new(),
        group_statuses: FnvIndexMap::new(),
        water_tap_statuses: FnvIndexMap::new(),
        steam_wand_statuses: FnvIndexMap::new(),
        tank_statuses: FnvIndexMap::new(),
        mode: MachineMode::On,
        routine_execution: None,
        comms_status: None,
        comms_status_age: None,
        peripheral_status: PeripheralStatus { peripherals: FnvIndexMap::new() },
        current_local_time: None,
        bluetooth: BluetoothScanStatus::default(),
        pending_shot_annotations: ShotAnnotations::new(),
        // `None`, not `Some(false)` -- the third state, which is the one a `bool` could
        // never have expressed and the one a UI is most likely to render wrongly.
        sd_card_present: None,
    }
}

fn routine() -> Routine {
    Routine {
        routine_type: RoutineType::Cleaning,
        name: "Fixture".into(),
        parameters: vec![RoutineParameter {
            index: 0,
            name: "Dose".into(),
            default: 18.0,
            unit: Some(ParameterUnit::Grams),
        }],
        derived_parameters: vec![DerivedParameter {
            index: 1,
            name: "Yield".into(),
            unit: Some(ParameterUnit::Grams),
            formula: DerivedFormula::Linear { base_param: 0, multiplier: 2.0, offset: 0.0 },
        }],
        steps: vec![RoutineStep {
            entry_command: vec![RoutineCommand::StartBrewing(0)],
            exits: vec![RoutineExit {
                condition: RoutineExitCondition::StateConditionMet(
                    StateCondition::GroupPressureAbove(0, ParameterValue::Static(9.0)),
                ),
                then: RoutineStepExitType::JumpToStep(2),
                description: Some("Reached pressure".into()),
            }],
            description: Some("Brew".into()),
        }],
        finally: vec![RoutineCommand::StopBrewing(0)],
    }
}

fn schedule_item() -> ScheduleItem {
    let mut on_days = heapless::index_set::FnvIndexSet::new();
    on_days.insert(chrono::Weekday::Mon).expect("fits");
    on_days.insert(chrono::Weekday::Wed).expect("fits");

    ScheduleItem {
        trigger_at: ScheduleTrigger {
            on_minute: 30,
            on_hour: 6,
            on_days: Some(on_days),
            on_date: Some(chrono::NaiveDate::from_ymd_opt(2026, 8, 7).expect("valid date")),
            enabled: true,
            once: false,
        },
        commands: vec![ScheduleAction::SetMachineMode(MachineMode::On)],
    }
}

/// Every `MachineCommand` variant, in declaration order.
///
/// Serialized as one sequence, so the harness decodes it with `seq(MachineCommandSchema)`
/// and checks all of them at once. The exhaustive `match` below is what makes a newly
/// added variant a compile error rather than a silently untested one -- the runtime
/// coverage check in the harness catches the complementary mistake of updating the match
/// but forgetting the list.
fn machine_commands() -> Vec<MachineCommand> {
    use MachineCommand::*;

    #[allow(dead_code)]
    fn exhaustive(c: &MachineCommand) {
        // No `_` arm, deliberately. Adding a variant must break this.
        match c {
            StartBrewing(_) => {}
            StopBrewing(_) => {}
            StartPumpingToWaterTap(_) => {}
            StopPumpingToWaterTap(_) => {}
            StartSteaming(_) => {}
            StopSteaming(_) => {}
            SetSteamValveOpenness(..) => {}
            SetBoilerControlTarget(..) => {}
            SetBoilerControlTargetValues(..) => {}
            SetGroupBrewControlTarget(..) => {}
            SetGroupBrewControlTargetValues(..) => {}
            SetPidParameters(..) => {}
            RunRoutine(..) => {}
            CancelRoutine => {}
            EnableBoiler(_) => {}
            DisableBoiler(_) => {}
            TareGroupScale(_) => {}
            ZeroCalibrateGroupScale(_) => {}
            CalibrateGroupScale100g(_) => {}
            UpdateCommsStatus(_) => {}
            AddScheduleItem(_) => {}
            RemoveScheduleItem(_) => {}
            UpdateScheduleItem(..) => {}
            AddRoutine(_) => {}
            RemoveRoutine(_) => {}
            UpdateRoutine(..) => {}
            SetMachineMode(_) => {}
            OptimizeConfigurationStorage => {}
            OptimizeRoutineStorage => {}
            OptimizeScheduleStorage => {}
            SetGroupPumpConfiguration(..) => {}
            SetWaterTapPumpConfiguration(..) => {}
            SetFillPumpConfiguration(..) => {}
            InferGroupPressureIntegral(..) => {}
            InferGroupFlowRateIntegral(..) => {}
            InferGroupOutputFlowRateIntegral(..) => {}
            SetHeatingElementInterlock(_) => {}
            SetHeatingElementContentionStrategy(_) => {}
            SetWaterDispersalPumpStrategy(..) => {}
            AssociateBluetoothPeripheral(_) => {}
            RemoveBluetoothPeripheral(_) => {}
            SetBluetoothPeripheralEnabled(..) => {}
            ScanForBluetoothPeripherals => {}
            UpdateBluetoothScan(_) => {}
            SetShotAnnotations(..) => {}
            SetPendingShotAnnotations(_) => {}
            TagDoseFromScale(_) => {}
            OpenWifiProvisioningWindow { .. } => {}
            CloseWifiProvisioningWindow => {}
            SetWifiCredentials(_) => {}
            IdentifyMachine => {}
            RequestConfiguration => {}
            DeleteShotLog(_) => {}
            SetShotUploadConfig(_) => {}
            SetShotUploadSettings(_) => {}
        }
    }

    let mut resolved = FnvIndexMap::new();
    resolved.insert(0u8, 18.0f32).expect("fits");
    resolved.insert(1u8, 36.0f32).expect("fits");

    let mut comms_peripherals = FnvIndexMap::new();
    comms_peripherals
        .insert(0xF001u16, WirelessConnectionStatus { connected: false, rssi: None })
        .expect("fits");
    comms_peripherals
        .insert(0xF002u16, WirelessConnectionStatus { connected: true, rssi: Some(-61) })
        .expect("fits");

    vec![
        StartBrewing(0),
        StopBrewing(1),
        StartPumpingToWaterTap(0),
        StopPumpingToWaterTap(1),
        StartSteaming(0),
        StopSteaming(1),
        SetSteamValveOpenness(0, 50),
        SetBoilerControlTarget(
            0,
            BoilerControlMode::Temperature,
            Some(BoilerControlTargetValuesUpdate {
                temperature: Some(93.0),
                pressure: Some(9.0),
            }),
        ),
        SetBoilerControlTargetValues(
            1,
            BoilerControlTargetValuesUpdate { temperature: None, pressure: Some(1.5) },
        ),
        SetGroupBrewControlTarget(
            0,
            GroupBrewControlMode::FixedDutyCycle,
            Some(GroupBrewControlTargetValuesUpdate {
                flow_rate: Some(2.5),
                flow_rate_curve: Some(control_curve()),
                pressure: Some(9.0),
                pressure_curve: Some(control_curve()),
                output_flow_rate: Some(1.5),
                output_flow_rate_curve: Some(control_curve()),
                duty_cycle: Some(60),
                duty_cycle_curve: Some(control_curve()),
            }),
        ),
        SetGroupBrewControlTargetValues(
            1,
            GroupBrewControlTargetValuesUpdate {
                flow_rate: None,
                flow_rate_curve: None,
                pressure: Some(6.0),
                pressure_curve: None,
                output_flow_rate: None,
                output_flow_rate_curve: None,
                duty_cycle: None,
                duty_cycle_curve: None,
            },
        ),
        SetPidParameters(PidParameterTarget::GroupPressure(0), pid_parameters()),
        RunRoutine(RoutineIndex::Function(2), Some(resolved)),
        CancelRoutine,
        EnableBoiler(0),
        DisableBoiler(1),
        TareGroupScale(0),
        ZeroCalibrateGroupScale(0),
        CalibrateGroupScale100g(0),
        UpdateCommsStatus(CommsStatus {
            timestamp: Some(1_700_000_001),
            wifi_connected: true,
            wifi_rssi: Some(-60),
            // A different non-default state from the one in `status_maximal`, so the two
            // fixtures cannot both pass on a decoder that hardcodes one value.
            improv: ImprovState::Authorized,
            peripheral_connection_status: comms_peripherals,
            // Different again from `status_maximal`'s, so neither fixture can pass on a
            // decoder that hardcodes the other's value.
            sntp_sync_seq: 12,
        }),
        AddScheduleItem(schedule_item()),
        RemoveScheduleItem(3),
        UpdateScheduleItem(4, schedule_item()),
        AddRoutine(routine()),
        RemoveRoutine(RoutineIndex::Custom(1)),
        UpdateRoutine(RoutineIndex::Internal(0), routine()),
        SetMachineMode(MachineMode::PowerSaveStandby),
        OptimizeConfigurationStorage,
        OptimizeRoutineStorage,
        OptimizeScheduleStorage,
        SetGroupPumpConfiguration(0, pump_configuration()),
        SetWaterTapPumpConfiguration(0, pump_configuration()),
        SetFillPumpConfiguration(0, pump_configuration()),
        InferGroupPressureIntegral(0, 9.0),
        InferGroupFlowRateIntegral(0, 2.5),
        InferGroupOutputFlowRateIntegral(0, 1.5),
        SetHeatingElementInterlock(true),
        SetHeatingElementContentionStrategy(HeatingElementContentionStrategy::Proportional),
        SetWaterDispersalPumpStrategy(0, WaterDispersalPumpStrategy::NoPump),
        // `AcaiaOld` rather than the first variant, and a non-empty name, so a decoder
        // that defaulted either would not round-trip.
        AssociateBluetoothPeripheral(BluetoothPeripheralAssociation {
            id: 0xB5C0,
            address: [0x2F, 0xA0, 0x1A, 0x97, 0x1C, 0x00],
            address_random: true,
            driver: BluetoothDriverKind::AcaiaOld,
            enabled: true,
            name: bluetooth_name("Group 1 scale"),
        }),
        RemoveBluetoothPeripheral(0xB5D0),
        SetBluetoothPeripheralEnabled(0xB1CA, false),
        ScanForBluetoothPeripherals,
        // The second variant, and a device with no advertised name -- the empty-string
        // case a length-prefix bug would sail straight past.
        UpdateBluetoothScan(BluetoothScanUpdate::Discovered(DiscoveredBluetoothPeripheral {
            address: [0x3E, 0x60, 0xEB, 0x3C, 0x1C, 0x78],
            address_random: false,
            name: BluetoothName::new(),
            rssi: -78,
            suggested_driver: Some(BluetoothDriverKind::BelkaPortal),
        })),
        // A dated shot, and a maximal-ish annotation block. `Some(day)` here against the
        // `None` day exercised by `SetPendingShotAnnotations`'s neighbour below would be
        // the obvious pairing, but this command is the only one carrying a `ShotLogId`,
        // so it takes the `Some` case and the `None` case is covered by the id round-trip
        // tests in `variegated-controller-types`.
        SetShotAnnotations(
            ShotLogId { day: Some(20_260_809), time: 14_320_512 },
            shot_annotations(),
        ),
        SetPendingShotAnnotations(ShotAnnotations::new()),
        TagDoseFromScale(ScaleSelector::GroupScale(0)),
        OpenWifiProvisioningWindow { duration_ms: 300_000 },
        CloseWifiProvisioningWindow,
        // A password with a non-ASCII character in it, and one long enough to need a
        // multi-byte length prefix nowhere near the SSID's. A decoder that read the two
        // heapless strings back in the wrong order, or that assumed ASCII, round-trips a
        // short matched pair straight past.
        SetWifiCredentials(WifiCredentials {
            ssid: heapless::String::try_from("Café Réseau").unwrap(),
            password: heapless::String::try_from("correct horse battery staple \u{00e9}").unwrap(),
        }),
        IdentifyMachine,
        RequestConfiguration,
        // The undated case `SetShotAnnotations` above could not take: `day: None` reaches
        // the URL as the literal `NODATE`, and it is the encoding of the `Option` in the
        // id that a frontend gets wrong first.
        DeleteShotLog(ShotLogId { day: None, time: 42 }),
        // Both fields `Some`, and the endpoint deliberately long and non-ASCII: this is the
        // only `Option<heapless::String<N>>` pair on the wire, and a decoder that reads the
        // two `Option` tags in the wrong order round-trips a short matched pair straight
        // past. The 300-byte-class endpoint also forces the two-byte length prefix that the
        // 64-byte token does not have, so a fixed-width length read shows up here.
        SetShotUploadConfig(ShotUploadConfig {
            endpoint: Some(
                heapless::String::try_from("https://plantlet.example/caf\u{00e9}/api/shots?src=r\u{00e9}seau")
                    .unwrap(),
            ),
            token: Some(heapless::String::try_from("0123456789ABCDEFGHJKMNPQRSTVWXYZ0123456789ABCDEFGHJKMNPQRSTVWXYZ").unwrap()),
            enabled: true,
        }),
        // The `Set` arm, and `enabled: false` -- the two together are the combination a
        // decoder is most likely to get wrong, since a three-variant enum followed by a bool
        // is exactly where an off-by-one in the discriminant stops being visible.
        SetShotUploadSettings(ShotUploadSettings {
            endpoint: Some(heapless::String::try_from("https://plantlet.example/api/shots").unwrap()),
            enabled: false,
            token: ShotUploadTokenUpdate::Set(
                heapless::String::try_from("0123456789ABCDEFGHJKMNPQRSTVWXYZ0123456789ABCDEFGHJKMNPQRSTVWXYZ").unwrap(),
            ),
        }),
    ]
}

/// An annotation block using every key kind and every value kind.
///
/// One of each rather than eight of one: the failure a fixture catches here is a decoder
/// that confuses the key's discriminant with the value's, and identical entries would
/// round-trip straight past it.
fn shot_annotations() -> ShotAnnotations {
    let mut annotations = ShotAnnotations::new();
    annotations
        .set(ShotAnnotationKey::DoseWeight, ShotAnnotationValue::Number(18.0))
        .expect("fits");
    annotations
        .set(
            ShotAnnotationKey::Beans,
            ShotAnnotationValue::Text(
                heapless::String::try_from("Koppi / Ethiopia Guji").expect("fits"),
            ),
        )
        .expect("fits");
    annotations
        .set(
            ShotAnnotationKey::GrindSize,
            ShotAnnotationValue::Text(heapless::String::try_from("4.2").expect("fits")),
        )
        .expect("fits");
    annotations
        .set(
            ShotAnnotationKey::Other(heapless::String::try_from("water").expect("fits")),
            ShotAnnotationValue::Text(heapless::String::try_from("ZeroWater").expect("fits")),
        )
        .expect("fits");
    annotations
}

/// A complete shot, as `SHOTS/<day>/<time>.BIN` holds one.
///
/// Two samples rather than one, because a single sample cannot reveal a length-prefix bug
/// in the sample vector -- and that vector is both the largest repeated structure in the
/// file and the one a wrong schema desynchronizes on. The two differ in the fields that
/// change during a shot, so a decoder that reads the second sample's bytes as the first's
/// produces a visible value change rather than a plausible duplicate.
///
/// The maps hold two entries each where the type allows it, for the same reason the
/// module note gives: a one-entry map hides a length-prefix bug.
pub fn canonical_shot() -> ShotLog {
    let mut boiler_samples = FnvIndexMap::new();
    let _ = boiler_samples.insert(
        0u8,
        BoilerSample {
            temperature: Some(93.5),
            pressure: Some(1.25),
            water_level: None,
            // Not the first variant, so a variant-index off-by-one cannot encode as zero
            // either way and pass.
            output: Output::FixedDutyCycle(42),
        },
    );
    let _ = boiler_samples.insert(
        1u8,
        BoilerSample {
            temperature: Some(124.0),
            pressure: None,
            water_level: Some(75),
            output: Output::Off,
        },
    );

    let mut water_tap_samples = FnvIndexMap::new();
    let _ = water_tap_samples.insert(0u8, WaterTapSample { is_dispensing: false });

    let group_sample = |brewing: bool, pressure: f32, flow_out: f32| GroupSample {
        is_brewing: brewing,
        brew_time: Some(core::time::Duration::from_secs(12)),
        brew_input_volume: Some(38.5),
        input_flow_rate: Some(2.25),
        input_volume: Some(38.5),
        output_flow_rate: Some(flow_out),
        output_weight: Some(21.5),
        pressure: Some(pressure),
        temperature: Some(93.0),
        // The three fields version 3 added, in the middle of this struct rather than at
        // its end -- which is why a version 2 file decodes as nonsense under version 3.
        output_temperature: Some(78.5),
        output_electrical_conductivity: Some(1250.0),
        extraction_rate: Some(0.125),
        pump_output: Output::FixedDutyCycle(65),
        shot_state: Some(ShotState::Saturation),
        extracted_solids: Some(2.25),
        output_volume: Some(24.0),
    };

    let sample_at = |t: u64, brewing: bool, pressure: f32, flow_out: f32| {
        let mut group_samples = FnvIndexMap::new();
        let _ = group_samples.insert(0u8, group_sample(brewing, pressure, flow_out));
        ShotLogSample {
            timestamp_millis: t,
            boiler_samples: boiler_samples.clone(),
            group_samples,
            water_tap_samples: water_tap_samples.clone(),
        }
    };

    let mut resolved_parameters = FnvIndexMap::new();
    let _ = resolved_parameters.insert(0u8, 6.0f32);
    let _ = resolved_parameters.insert(1u8, 85.0f32);

    ShotLog {
        version: SHOT_LOG_FORMAT_VERSION,
        metadata: ShotLogMetadata {
            annotations: shot_annotations(),
            shot_type: ShotType::Routine,
            group_index: 0,
            routine_metadata: Some(RoutineExecutionMetadata {
                routine_index: RoutineIndex::Custom(1),
                routine_name: "6 bar, 85 mL".into(),
                routine_type: RoutineType::UserDefined,
                resolved_parameters,
            }),
            start_time_millis: 1_800_000,
            // A duration, not a timestamp, despite sitting beside one that is. See the
            // note on `ShotLogMetadata::end_time_millis`.
            end_time_millis: Some(28_500),
            final_status: ShotStatus::Completed,
            // The field version 4 added, and the only wall clock in the file. A real
            // date -- 2026-08-11T06:29:11.930Z -- rather than a round number, so a
            // decoder that lost or shifted it renders something obviously wrong instead
            // of a plausible epoch.
            recorded_at_unix_millis: Some(1_786_429_751_930),
        },
        samples: vec![
            sample_at(0, false, 1.0, 0.0),
            sample_at(250, true, 6.0, 1.75),
        ],
        routine_events: vec![RoutineEvent {
            timestamp_millis: 4_000,
            from_step: Some(0),
            to_step: 1,
            exit_condition_description: Some("pressure >= 6.0 bar".into()),
            step_description: Some("Ramp to 6 bar".into()),
        }],
    }
}

pub fn all() -> Vec<Fixture> {
    vec![
        fixture("status_maximal", "StatusSchema", &status_maximal()),
        fixture("status_minimal", "StatusSchema", &status_minimal()),
        fixture("machine_commands", "seq:MachineCommandSchema", &machine_commands()),
        // The envelope itself, including the struct variant with the borrowed field --
        // the one place in the tree with a `#[serde(borrow)]`.
        fixture(
            "ws_command_ack",
            "WsMessageSchema",
            &WsMessage::CommandAck { id: 7, success: false, error: Some("boiler offline") },
        ),
        fixture(
            "ws_status_update",
            "WsMessageSchema",
            &WsMessage::StatusUpdate(status_maximal()),
        ),
        fixture::<WsMessage>(
            "ws_request_routines",
            "WsMessageSchema",
            &WsMessage::RequestRoutines,
        ),
        // The unit variant appended after `SendMachineCommand`, here to pin its
        // discriminant. A unit variant encodes to one byte and nothing else, so this
        // fixture is worth exactly one thing -- catching the day someone tidies the
        // client-to-server variants into a group and renumbers the command beside them.
        fixture::<WsMessage>(
            "ws_request_configuration",
            "WsMessageSchema",
            &WsMessage::RequestConfiguration,
        ),
        fixture("shot_log_list", "ShotLogListSchema", &shot_log_list()),
        fixture(
            "shot_log_list_page_two",
            "ShotLogListSchema",
            &shot_log_list_page_two(),
        ),
        // Both event shapes. `Stored` carries an entry and `Deleted` a bare id, so a
        // decoder that confused the two would read the id as the front of an entry and
        // build a row out of whatever followed.
        fixture(
            "ws_shot_log_stored",
            "WsMessageSchema",
            &WsMessage::ShotLogEvent(ShotLogEvent::Stored(ShotLogListEntry {
                id: ShotLogId { day: Some(20_260_809), time: 17_000_101 },
                size_bytes: 48_112,
                annotations: ShotAnnotations::new(),
            })),
        ),
        fixture::<WsMessage>(
            "ws_shot_log_deleted",
            "WsMessageSchema",
            &WsMessage::ShotLogEvent(ShotLogEvent::Deleted(ShotLogId {
                day: None,
                time: 42,
            })),
        ),
        // The routine listing, which until now had **no fixture at all**: `roots.rs`
        // emitted its schema and nothing ever serialised one, so the response every
        // client depends on had never been round-tripped. It is here now because the
        // payload changed -- from whole definitions to summaries -- and that is exactly
        // the kind of change a missing fixture lets through green.
        fixture(
            "routine_summaries",
            "RoutineSummaryStorageSchema",
            &routine_summaries(),
        ),
        fixture(
            "ws_routines_update",
            "WsMessageSchema",
            &WsMessage::RoutinesUpdate(routine_summaries()),
        ),
        // One definition, as `GET /routines/{type}/{index}` returns it and as
        // `PUT` accepts it. `machine_commands` also pins `Routine`'s shape through
        // `AddRoutine`, but that command is no longer the path the frontend uses -- this
        // fixture covers the one that is.
        fixture("routine", "RoutineSchema", &routine()),
    ]
}

/// A listing as `GET /routines` returns it.
///
/// One routine in each of the three index kinds, because the split into three maps is
/// where a re-keying bug would live -- a client that read all three from one map would
/// still decode, and would show every routine under one tab.
///
/// The counts deliberately differ from each other and from the number of entries, so a
/// summary built by copying the wrong field cannot pass. `finally_count: 0` on one of
/// them is the case a UI hides rather than renders, which makes it the one worth pinning.
fn routine_summaries() -> RoutineSummaryStorage {
    let mut storage = RoutineSummaryStorage {
        internal: BTreeMap::new(),
        function: BTreeMap::new(),
        custom: BTreeMap::new(),
    };

    // Derived from the same `routine()` the definition fixture uses, so the two agree by
    // construction: if `From<&Routine>` ever miscounts, these two fixtures disagree.
    storage.custom.insert(3, RoutineSummary::from(&routine()));

    storage.internal.insert(
        0,
        RoutineSummary {
            routine_type: RoutineType::HeatUp,
            name: "Heat up".into(),
            step_count: 2,
            parameter_count: 1,
            derived_parameter_count: 0,
            finally_count: 0,
        },
    );
    storage.function.insert(
        2,
        RoutineSummary {
            routine_type: RoutineType::HardwareButtonMapped,
            name: "Button 2".into(),
            step_count: 11,
            parameter_count: 4,
            derived_parameter_count: 2,
            finally_count: 1,
        },
    );

    storage
}
// `canonical_shot` is deliberately absent from this list. Everything here is written to
// `frontend/fixtures/` and named in its `index.json`, which the frontend harness walks,
// resolving each entry's `schema` against `frontend/src/schemas/schemas.ts` -- and
// `ShotLog` is not one of that file's roots, so there is no `ShotLogSchema` there to
// resolve. Adding it would break the harness on a fixture it cannot use.
//
// The shot is written by `shot_log_export` instead, into the plantlet repository beside
// the schema that *can* decode it. It loses nothing by not being here: the compile-error
// guarantee this module exists for comes from the exhaustive struct literal being
// compiled, not from appearing in `all()`.

/// A listing as `GET /shots` returns it.
///
/// Three entries, deliberately unlike each other: a dated shot with a full annotation
/// block, a dated shot with none, and an **undated** one. The last is the case a
/// frontend is most likely to get wrong -- `day: None` renders as `NODATE` on the card
/// and has to reach the download URL as that literal rather than as "null" -- and it is
/// the only place `ShotLogId`'s `Option` is exercised as `None` in a fixture.
///
/// `truncated: true`, because that is the value a UI must not ignore and therefore the
/// one worth pinning: a capped list rendered as the whole card is a silent wrong answer.
fn shot_log_list() -> ShotLogList {
    let mut beans_only = ShotAnnotations::new();
    beans_only
        .set(
            ShotAnnotationKey::Beans,
            ShotAnnotationValue::Text(heapless::String::try_from("Morgon / Sisters").expect("fits")),
        )
        .expect("fits");

    ShotLogList {
        entries: vec![
            ShotLogListEntry {
                id: ShotLogId { day: Some(20_260_809), time: 16_423_349 },
                size_bytes: 51_291,
                annotations: shot_annotations(),
            },
            ShotLogListEntry {
                id: ShotLogId { day: Some(20_260_809), time: 15_495_678 },
                size_bytes: 5_847,
                annotations: beans_only,
            },
            ShotLogListEntry {
                id: ShotLogId { day: None, time: 42 },
                size_bytes: 1_024,
                annotations: ShotAnnotations::new(),
            },
        ],
        truncated: true,
    }
}

/// The page after [`shot_log_list`], as a cursor would fetch it.
///
/// `truncated: false`, so it is also the fixture that proves a UI stops offering "load
/// older" at the end rather than looping on an empty page. One entry rather than three,
/// because a short final page is the shape a real card produces and the one a client that
/// assumed full pages would mishandle.
fn shot_log_list_page_two() -> ShotLogList {
    ShotLogList {
        entries: vec![ShotLogListEntry {
            id: ShotLogId { day: Some(20_260_808), time: 8_150_000 },
            size_bytes: 12_004,
            annotations: ShotAnnotations::new(),
        }],
        truncated: false,
    }
}

/// Write every fixture to `dir` as a `.bin`/`.json` pair, plus an index the harness
/// reads so it does not need its own hardcoded list.
pub fn write_all(dir: &Path) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;

    let fixtures = all();
    let mut index = String::from("[\n");
    for (i, f) in fixtures.iter().enumerate() {
        write_if_changed(&dir.join(format!("{}.bin", f.name)), &f.bytes)?;
        write_if_changed(&dir.join(format!("{}.json", f.name)), f.json.as_bytes())?;
        let comma = if i + 1 == fixtures.len() { "" } else { "," };
        index.push_str(&format!(
            "  {{ \"name\": \"{}\", \"schema\": \"{}\" }}{}\n",
            f.name, f.schema, comma
        ));
    }
    index.push_str("]\n");
    write_if_changed(&dir.join("index.json"), index.as_bytes())?;
    Ok(())
}

/// Same read-compare-write discipline as the schema itself: this runs from a build
/// script, and rust-analyzer checks continuously in the background.
fn write_if_changed(path: &Path, contents: &[u8]) -> std::io::Result<()> {
    if let Ok(current) = std::fs::read(path) {
        if current == contents {
            return Ok(());
        }
    }
    std::fs::write(path, contents)
}
