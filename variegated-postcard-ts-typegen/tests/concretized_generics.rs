//! Tests for concretized generic types (e.g., Generic<f32> -> Generic_for_float).

use variegated_postcard_ts_typegen::{impl_postcard_ts_for_concrete, SchemaGenerator};

// Define a generic struct that we'll concretize
#[derive(Debug, Clone)]
pub struct GenericContainer<T> {
    pub value: T,
    pub multiplier: T,
}

// Define a more complex generic struct with nested generics
#[derive(Debug, Clone)]
pub struct ComplexGeneric<T, U> {
    pub primary: T,
    pub secondary: U,
    pub ratio: f32,
}

// Use the macro to implement PostcardTsType for concrete instantiations
impl_postcard_ts_for_concrete!(
    GenericContainer<f32> => "GenericContainer_for_float",
    struct {
        value: "f32",
        multiplier: "f32",
    }
);

impl_postcard_ts_for_concrete!(
    GenericContainer<u32> => "GenericContainer_for_u32",
    struct {
        value: "u32",
        multiplier: "u32",
    }
);

impl_postcard_ts_for_concrete!(
    ComplexGeneric<f32, u32> => "ComplexGeneric_for_float_u32",
    struct {
        primary: "f32",
        secondary: "u32",
        ratio: "f32",
    }
);

#[test]
fn test_concretized_generic_f32() {
    let mut generator = SchemaGenerator::new();
    generator.add::<GenericContainer<f32>>();

    let output = generator.generate();

    // Verify the schema name follows the naming convention
    assert!(output.contains("export const GenericContainer_for_floatSchema"));

    // Verify the struct structure
    assert!(output.contains("struct({"));
    assert!(output.contains("value: f32()"));
    assert!(output.contains("multiplier: f32()"));

    // Verify type export
    assert!(output.contains("export type GenericContainer_for_float"));
    assert!(output.contains("InferType<typeof GenericContainer_for_floatSchema>"));

    println!("Generated schema:\n{}", output);
}

#[test]
fn test_concretized_generic_u32() {
    let mut generator = SchemaGenerator::new();
    generator.add::<GenericContainer<u32>>();

    let output = generator.generate();

    // Verify the schema name
    assert!(output.contains("export const GenericContainer_for_u32Schema"));

    // Verify different type
    assert!(output.contains("value: u32()"));
    assert!(output.contains("multiplier: u32()"));
}

#[test]
fn test_multiple_concretizations() {
    let mut generator = SchemaGenerator::new();

    generator.add::<GenericContainer<f32>>();
    generator.add::<GenericContainer<u32>>();

    let output = generator.generate();

    // Both concretizations should be present
    assert!(output.contains("GenericContainer_for_floatSchema"));
    assert!(output.contains("GenericContainer_for_u32Schema"));

    // Each should have correct types
    let float_pos = output.find("GenericContainer_for_floatSchema").unwrap();
    let u32_pos = output.find("GenericContainer_for_u32Schema").unwrap();

    let float_section = &output[float_pos..float_pos + 200];
    let u32_section = &output[u32_pos..u32_pos + 200];

    assert!(float_section.contains("f32()"));
    assert!(u32_section.contains("u32()"));
}

#[test]
fn test_complex_generic_concretization() {
    let mut generator = SchemaGenerator::new();
    generator.add::<ComplexGeneric<f32, u32>>();

    let output = generator.generate();

    // Verify naming convention for multiple type parameters
    assert!(output.contains("export const ComplexGeneric_for_float_u32Schema"));

    // Verify all fields
    assert!(output.contains("primary: f32()"));
    assert!(output.contains("secondary: u32()"));
    assert!(output.contains("ratio: f32()"));
}

#[test]
fn test_concretized_field_order() {
    // Define a type with specific field order
    #[derive(Debug)]
    pub struct OrderedGeneric<T> {
        pub first: u8,
        pub second: T,
        pub third: u16,
        pub fourth: T,
    }

    impl_postcard_ts_for_concrete!(
        OrderedGeneric<f32> => "OrderedGeneric_for_float",
        struct {
            first: "u8",
            second: "f32",
            third: "u16",
            fourth: "f32",
        }
    );

    let mut generator = SchemaGenerator::new();
    generator.add::<OrderedGeneric<f32>>();
    let output = generator.generate();

    // Extract the struct definition
    let struct_start = output.find("export const OrderedGeneric_for_floatSchema").unwrap();
    let struct_section = &output[struct_start..];

    // Verify field order
    let first_pos = struct_section.find("first:").unwrap();
    let second_pos = struct_section.find("second:").unwrap();
    let third_pos = struct_section.find("third:").unwrap();
    let fourth_pos = struct_section.find("fourth:").unwrap();

    assert!(first_pos < second_pos, "first should come before second");
    assert!(second_pos < third_pos, "second should come before third");
    assert!(third_pos < fourth_pos, "third should come before fourth");
}

#[test]
fn test_nested_concretized_types() {
    // Define nested generic types
    #[derive(Debug)]
    pub struct Inner<T> {
        pub data: T,
    }

    #[derive(Debug)]
    pub struct Outer<T> {
        pub inner: Inner<T>,
        pub extra: u32,
    }

    impl_postcard_ts_for_concrete!(
        Inner<f32> => "Inner_for_float",
        struct {
            data: "f32",
        }
    );

    impl_postcard_ts_for_concrete!(
        Outer<f32> => "Outer_for_float",
        struct {
            inner: "Inner_for_float",
            extra: "u32",
        }
    );

    let mut generator = SchemaGenerator::new();
    generator.add::<Inner<f32>>();
    generator.add::<Outer<f32>>();

    let output = generator.generate();

    // Verify both types are present
    assert!(output.contains("Inner_for_floatSchema"));
    assert!(output.contains("Outer_for_floatSchema"));

    // Verify Outer references Inner correctly
    assert!(output.contains("inner: Inner_for_floatSchema"));
    assert!(output.contains("extra: u32()"));
}

#[test]
fn test_concretized_with_collections() {
    #[derive(Debug)]
    pub struct WithCollections<T> {
        pub values: Vec<T>,
        pub optional: Option<T>,
    }

    impl_postcard_ts_for_concrete!(
        WithCollections<u32> => "WithCollections_for_u32",
        struct {
            values: "Vec<u32>",
            optional: "Option<u32>",
        }
    );

    let mut generator = SchemaGenerator::new();
    generator.add::<WithCollections<u32>>();

    let output = generator.generate();

    // Verify collection types are properly mapped
    assert!(output.contains("values: seq(u32())"));
    assert!(output.contains("optional: option(u32())"));
}
