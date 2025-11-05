//! Test demonstrating variegated-postcard-ts-typegen functionality.
//!
//! This test shows how to use the crate without hitting orphan rule issues.
//! In a real application, you would add the derives to types in your own crates.

use variegated_postcard_ts_typegen::{PostcardTsTypegen, SchemaGenerator};

// Define test types with the derive
#[derive(PostcardTsTypegen)]
enum SimpleEnum {
    Variant1,
    Variant2,
    Variant3,
}

#[derive(PostcardTsTypegen)]
struct SimpleStruct {
    field1: u32,
    field2: String,
    field3: bool,
}

#[derive(PostcardTsTypegen)]
struct StructWithOptional {
    required: u32,
    optional: Option<String>,
}

#[derive(PostcardTsTypegen)]
enum ComplexEnum {
    Unit,
    Newtype(u32),
    Tuple(f32, String),
    Struct { x: f32, y: f32 },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_schemas() {
        let mut generator = SchemaGenerator::new();

        generator.add::<SimpleEnum>();
        generator.add::<SimpleStruct>();
        generator.add::<StructWithOptional>();
        generator.add::<ComplexEnum>();

        let output = generator.generate();

        // Verify the output contains expected schemas
        assert!(output.contains("SimpleEnumSchema"));
        assert!(output.contains("SimpleStructSchema"));
        assert!(output.contains("StructWithOptionalSchema"));
        assert!(output.contains("ComplexEnumSchema"));

        // Verify structure
        assert!(output.contains("export const"));
        assert!(output.contains("struct({"));
        assert!(output.contains("enumType("));
        assert!(output.contains("unitVariant("));
        assert!(output.contains("export type"));

        // Print for inspection
        println!("\n=== Generated TypeScript Schema ===\n{}", output);
    }

    #[test]
    fn test_enum_variants() {
        let mut generator = SchemaGenerator::new();
        generator.add::<ComplexEnum>();

        let output = generator.generate();

        // Check all variant types are present
        assert!(output.contains("Unit: unitVariant('Unit')"));
        assert!(output.contains("Newtype: newtypeVariant('Newtype'"));
        assert!(output.contains("Tuple: tupleVariant('Tuple'"));
        assert!(output.contains("Struct: newtypeVariant('Struct'"));
    }

    #[test]
    fn test_optional_fields() {
        let mut generator = SchemaGenerator::new();
        generator.add::<StructWithOptional>();

        let output = generator.generate();

        // Check optional field is wrapped with option()
        assert!(output.contains("required: u32()"));
        assert!(output.contains("optional: option(string())"));
    }
}
