use variegated_postcard_ts_typegen::{PostcardTsType, PostcardTsTypegen, SchemaGenerator};

#[derive(PostcardTsTypegen)]
struct SimpleStruct {
    field1: u32,
    field2: String,
    field3: bool,
}

#[derive(PostcardTsTypegen)]
enum SimpleEnum {
    Variant1,
    Variant2,
    Variant3,
}

#[derive(PostcardTsTypegen)]
enum ComplexEnum {
    Unit,
    Newtype(u32),
    Tuple(u32, String),
    Struct { x: f32, y: f32 },
}

#[derive(PostcardTsTypegen)]
struct StructWithOptional {
    required: u32,
    optional: Option<String>,
}

#[derive(PostcardTsTypegen)]
struct StructWithVec {
    items: Vec<u32>,
}

#[test]
fn test_simple_struct_schema() {
    let schema = SimpleStruct::generate_schema();
    assert_eq!(schema.name, "SimpleStructSchema");
}

#[test]
fn test_simple_enum_schema() {
    let schema = SimpleEnum::generate_schema();
    assert_eq!(schema.name, "SimpleEnumSchema");
}

#[test]
fn test_complex_enum_schema() {
    let schema = ComplexEnum::generate_schema();
    assert_eq!(schema.name, "ComplexEnumSchema");
}

#[test]
fn test_struct_with_optional() {
    let schema = StructWithOptional::generate_schema();
    assert_eq!(schema.name, "StructWithOptionalSchema");
}

#[test]
fn test_struct_with_vec() {
    let schema = StructWithVec::generate_schema();
    assert_eq!(schema.name, "StructWithVecSchema");
}

#[test]
fn test_generator() {
    let mut generator = SchemaGenerator::new();

    generator.add::<SimpleStruct>();
    generator.add::<SimpleEnum>();
    generator.add::<ComplexEnum>();

    let output = generator.generate();

    // Check that the output contains the expected schemas
    assert!(output.contains("export const SimpleStructSchema"));
    assert!(output.contains("export const SimpleEnumSchema"));
    assert!(output.contains("export const ComplexEnumSchema"));

    // Check that imports are present
    assert!(output.contains("from '@variegated-coffee/serde-postcard-ts'"));

    // Check that type exports are present
    assert!(output.contains("export type SimpleStruct"));
    assert!(output.contains("export type SimpleEnum"));
    assert!(output.contains("export type ComplexEnum"));
}

#[test]
fn test_full_generation() {
    let mut generator = SchemaGenerator::new();

    generator.add::<SimpleStruct>();
    generator.add::<SimpleEnum>();
    generator.add::<ComplexEnum>();
    generator.add::<StructWithOptional>();
    generator.add::<StructWithVec>();

    let output = generator.generate();

    // Verify the structure is valid TypeScript-like code
    assert!(output.contains("struct({"));
    assert!(output.contains("enumType("));
    assert!(output.contains("unitVariant("));
    assert!(output.contains("InferType<typeof"));
}

#[test]
fn test_nested_types() {
    #[derive(PostcardTsTypegen)]
    struct Inner {
        value: u32,
    }

    #[derive(PostcardTsTypegen)]
    struct Outer {
        inner: Inner,
    }

    let mut generator = SchemaGenerator::new();
    generator.add::<Inner>();
    generator.add::<Outer>();

    let output = generator.generate();

    assert!(output.contains("InnerSchema"));
    assert!(output.contains("OuterSchema"));
}

#[test]
fn test_map_types() {
    use std::collections::BTreeMap;

    #[derive(PostcardTsTypegen)]
    struct WithMap {
        data: BTreeMap<u8, String>,
    }

    let schema = WithMap::generate_schema();
    assert_eq!(schema.name, "WithMapSchema");
}

#[test]
fn test_duration_type() {
    use std::time::Duration;

    #[derive(PostcardTsTypegen)]
    struct WithDuration {
        elapsed: Duration,
    }

    let schema = WithDuration::generate_schema();
    assert_eq!(schema.name, "WithDurationSchema");

    let mut generator = SchemaGenerator::new();
    generator.add::<Duration>();
    generator.add::<WithDuration>();

    let output = generator.generate();
    assert!(output.contains("DurationSchema"));
}
