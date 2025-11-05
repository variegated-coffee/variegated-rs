//! Test for generating TypeScript schemas from controller types.
//!
//! This demonstrates how to use variegated-postcard-ts-typegen to generate
//! TypeScript schemas for Rust types, including handling of concrete generic types.

#[cfg(all(feature = "ts-typegen", feature = "std"))]
mod tests {
    use variegated_controller_types::*;
    use variegated_postcard_ts_typegen::{
        impl_postcard_ts_for_concrete, EnumVariant, PostcardTsType, SchemaDefinition,
        SchemaGenerator, SchemaKind, VariantKind,
    };

    // Implement PostcardTsType for the simple enum types used in this test
    impl PostcardTsType for PeripheralType {
        fn ts_name() -> String {
            "PeripheralType".to_string()
        }

        fn generate_schema() -> SchemaDefinition {
            SchemaDefinition {
                name: "PeripheralTypeSchema".to_string(),
                kind: SchemaKind::Enum(vec![
                    EnumVariant {
                        name: "Scale".to_string(),
                        kind: VariantKind::Unit,
                    },
                    EnumVariant {
                        name: "PressureSensor".to_string(),
                        kind: VariantKind::Unit,
                    },
                    EnumVariant {
                        name: "FlowMeter".to_string(),
                        kind: VariantKind::Unit,
                    },
                    EnumVariant {
                        name: "LevelSensor".to_string(),
                        kind: VariantKind::Unit,
                    },
                ]),
            }
        }
    }

    impl PostcardTsType for MachineMode {
        fn ts_name() -> String {
            "MachineMode".to_string()
        }

        fn generate_schema() -> SchemaDefinition {
            SchemaDefinition {
                name: "MachineModeSchema".to_string(),
                kind: SchemaKind::Enum(vec![
                    EnumVariant {
                        name: "On".to_string(),
                        kind: VariantKind::Unit,
                    },
                    EnumVariant {
                        name: "Off".to_string(),
                        kind: VariantKind::Unit,
                    },
                    EnumVariant {
                        name: "PowerSaveStandby".to_string(),
                        kind: VariantKind::Unit,
                    },
                ]),
            }
        }
    }

    // Implement PostcardTsType for concrete PID algorithm types
    // These are generic types from the control-algorithm crate that we want to
    // export with specific type parameters (f32 in this case)

    impl_postcard_ts_for_concrete!(
        variegated_control_algorithm::pid::Limits<f32> => "Limits_for_float",
        struct {
            lower: "f32",
            upper: "f32",
        }
    );

    impl_postcard_ts_for_concrete!(
        variegated_control_algorithm::pid::PidTerm<f32> => "PidTerm_for_float",
        struct {
            positive_scale: "f32",
            negative_scale: "f32",
            limits: "Limits_for_float",
        }
    );

    impl_postcard_ts_for_concrete!(
        variegated_control_algorithm::pid::PidParameters<f32> => "PidParameters_for_float",
        struct {
            kp: "PidTerm_for_float",
            ki: "PidTerm_for_float",
            kd: "PidTerm_for_float",
        }
    );

    impl_postcard_ts_for_concrete!(
        variegated_control_algorithm::pid::PidOut<f32> => "PidOut_for_float",
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

    #[test]
    fn test_generate_basic_schemas() {
        let mut generator = SchemaGenerator::new();

        // Add primitive PID types
        generator.add::<variegated_control_algorithm::pid::Limits<f32>>();
        generator.add::<variegated_control_algorithm::pid::PidTerm<f32>>();
        generator.add::<variegated_control_algorithm::pid::PidParameters<f32>>();
        generator.add::<variegated_control_algorithm::pid::PidOut<f32>>();

        // Add simple enum types that have the derive
        generator.add::<PeripheralType>();
        generator.add::<MachineMode>();

        let output = generator.generate();

        // Verify the output contains expected schemas
        assert!(output.contains("Limits_for_floatSchema"));
        assert!(output.contains("PidTerm_for_floatSchema"));
        assert!(output.contains("PidParameters_for_floatSchema"));
        assert!(output.contains("PidOut_for_floatSchema"));
        assert!(output.contains("PeripheralTypeSchema"));
        assert!(output.contains("MachineModeSchema"));

        // Verify it has the correct structure
        assert!(output.contains("export const"));
        assert!(output.contains("struct({"));
        assert!(output.contains("enumType("));
        assert!(output.contains("export type"));

        // Print for manual inspection during test runs
        println!("\nGenerated TypeScript Schema:\n{}", output);
    }

    #[test]
    fn test_enum_generation() {
        let mut generator = SchemaGenerator::new();

        generator.add::<PeripheralType>();
        generator.add::<MachineMode>();

        let output = generator.generate();

        // Check enum structure
        assert!(output.contains("enumType('PeripheralType'"));
        assert!(output.contains("unitVariant('Scale')"));
        assert!(output.contains("unitVariant('PressureSensor')"));

        assert!(output.contains("enumType('MachineMode'"));
        assert!(output.contains("unitVariant('On')"));
        assert!(output.contains("unitVariant('Off')"));
    }
}
