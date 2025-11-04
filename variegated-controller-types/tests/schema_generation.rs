//! Test for generating TypeScript schemas from controller types.

#[cfg(all(feature = "ts-typegen", feature = "std"))]
mod tests {
    use variegated_controller_types::*;
    use variegated_postcard_ts_typegen::{impl_postcard_ts_for_concrete, PostcardTsType, SchemaGenerator, FieldDefinition, SchemaDefinition, SchemaKind};

    // Implement PostcardTsType for heapless::FnvIndexMap
    impl<K: PostcardTsType, V: PostcardTsType, const N: usize> PostcardTsType
        for heapless::FnvIndexMap<K, V, N>
    {
        fn ts_name() -> String {
            format!("FnvIndexMap<{}, {}>", K::ts_name(), V::ts_name())
        }

        fn generate_schema() -> SchemaDefinition {
            SchemaDefinition {
                name: format!("FnvIndexMap<{}, {}>", K::ts_name(), V::ts_name()),
                kind: SchemaKind::Unit,
            }
        }
    }

    // Implement PostcardTsType for concrete PID algorithm types
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
    fn test_generate_schema() {
        let mut generator = SchemaGenerator::new();

        // Add primitive PID types first
        generator.add::<variegated_control_algorithm::pid::Limits<f32>>();
        generator.add::<variegated_control_algorithm::pid::PidTerm<f32>>();
        generator.add::<variegated_control_algorithm::pid::PidParameters<f32>>();
        generator.add::<variegated_control_algorithm::pid::PidOut<f32>>();

        // Add some basic enums
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

        // Print for manual inspection
        println!("{}", output);
    }

    #[test]
    fn test_write_schema_file() {
        let mut generator = SchemaGenerator::new();

        // Add all the concrete PID types
        generator.add::<variegated_control_algorithm::pid::Limits<f32>>();
        generator.add::<variegated_control_algorithm::pid::PidTerm<f32>>();
        generator.add::<variegated_control_algorithm::pid::PidParameters<f32>>();
        generator.add::<variegated_control_algorithm::pid::PidOut<f32>>();

        // Add variegated types
        generator.add::<PeripheralType>();
        generator.add::<MachineMode>();

        // This would write to a file in a real build.rs
        // generator.write_to_file("../schema-generated.ts").unwrap();

        let output = generator.generate();
        assert!(output.len() > 100); // Should have substantial content
    }
}
