//! End-to-end tests that validate complete schema generation and structure.

use variegated_postcard_ts_typegen::{PostcardTsTypegen, SchemaGenerator};
use std::collections::BTreeMap;

#[derive(PostcardTsTypegen)]
struct Point {
    x: f32,
    y: f32,
}

#[derive(PostcardTsTypegen)]
enum Status {
    Active,
    Inactive,
    Pending { reason: String },
}

#[derive(PostcardTsTypegen)]
struct Container {
    id: u32,
    name: String,
    point: Point,
    status: Status,
    tags: Vec<String>,
    metadata: BTreeMap<String, String>,
}

/// Validates that a generated schema line matches expected format
#[allow(dead_code)]
fn validate_schema_line(line: &str, expected_patterns: &[&str]) -> bool {
    expected_patterns.iter().all(|pattern| line.contains(pattern))
}

/// Extracts schema definitions from generated TypeScript
fn extract_schema_definitions(output: &str) -> Vec<String> {
    output
        .lines()
        .filter(|line| line.starts_with("export const ") && line.contains("Schema ="))
        .map(|s| s.to_string())
        .collect()
}

/// Extracts type exports from generated TypeScript
fn extract_type_exports(output: &str) -> Vec<String> {
    output
        .lines()
        .filter(|line| line.starts_with("export type ") && line.contains("InferType"))
        .map(|s| s.to_string())
        .collect()
}

#[test]
fn test_complete_schema_structure() {
    let mut generator = SchemaGenerator::new();

    generator.add::<Point>();
    generator.add::<Status>();
    generator.add::<Container>();

    let output = generator.generate();

    // Validate file header
    assert!(output.starts_with("// AUTO-GENERATED"));
    assert!(output.contains("DO NOT EDIT MANUALLY"));
    assert!(output.contains("Field order in struct schemas"));

    // Validate imports section
    assert!(output.contains("import {"));
    assert!(output.contains("} from '@variegated-coffee/serde-postcard-ts'"));

    // Validate all schema definitions exist
    let schema_defs = extract_schema_definitions(&output);
    assert!(schema_defs.len() >= 3, "Should have at least 3 schema definitions");

    // Validate all type exports exist
    let type_exports = extract_type_exports(&output);
    assert!(type_exports.len() >= 3, "Should have at least 3 type exports");

    // Validate Point schema structure
    assert!(output.contains("export const PointSchema = struct({"));
    assert!(output.contains("x: f32()"));
    assert!(output.contains("y: f32()"));
    assert!(output.contains("});"));

    // Validate Status enum structure
    assert!(output.contains("export const StatusSchema = enumType('Status', {"));
    assert!(output.contains("Active: unitVariant('Active')"));
    assert!(output.contains("Inactive: unitVariant('Inactive')"));
    assert!(output.contains("Pending: newtypeVariant('Pending', struct({"));
    assert!(output.contains("reason: string()"));

    // Validate Container schema with nested types
    assert!(output.contains("export const ContainerSchema = struct({"));
    assert!(output.contains("id: u32()"));
    assert!(output.contains("name: string()"));
    assert!(output.contains("point: PointSchema"));
    assert!(output.contains("status: StatusSchema"));
    assert!(output.contains("tags: seq(string())"));
    assert!(output.contains("metadata: map(string(), string())"));

    // Validate type exports
    assert!(output.contains("export type Point = InferType<typeof PointSchema>"));
    assert!(output.contains("export type Status = InferType<typeof StatusSchema>"));
    assert!(output.contains("export type Container = InferType<typeof ContainerSchema>"));
}

#[test]
fn test_field_order_preservation() {
    #[derive(PostcardTsTypegen)]
    struct OrderedFields {
        first: u8,
        second: u16,
        third: u32,
        fourth: u64,
    }

    let mut generator = SchemaGenerator::new();
    generator.add::<OrderedFields>();
    let output = generator.generate();

    // Extract the struct definition
    let struct_def_start = output.find("export const OrderedFieldsSchema").unwrap();
    let struct_def = &output[struct_def_start..];

    // Verify field order by finding positions
    let first_pos = struct_def.find("first:").unwrap();
    let second_pos = struct_def.find("second:").unwrap();
    let third_pos = struct_def.find("third:").unwrap();
    let fourth_pos = struct_def.find("fourth:").unwrap();

    assert!(first_pos < second_pos);
    assert!(second_pos < third_pos);
    assert!(third_pos < fourth_pos);
}

#[test]
fn test_enum_variant_types() {
    #[derive(PostcardTsTypegen)]
    enum AllVariantTypes {
        Unit,
        Newtype(String),
        Tuple(u32, f32),
        Struct { a: u8, b: u16 },
    }

    let mut generator = SchemaGenerator::new();
    generator.add::<AllVariantTypes>();
    let output = generator.generate();

    // Validate each variant type is correctly generated
    assert!(output.contains("Unit: unitVariant('Unit')"));
    assert!(output.contains("Newtype: newtypeVariant('Newtype', string())"));
    assert!(output.contains("Tuple: tupleVariant('Tuple', u32(), f32())"));
    assert!(output.contains("Struct: newtypeVariant('Struct', struct({"));
    assert!(output.contains("a: u8()"));
    assert!(output.contains("b: u16()"));
}

#[test]
fn test_nested_option_and_vec() {
    #[derive(PostcardTsTypegen)]
    struct Nested {
        optional_vec: Option<Vec<u32>>,
        vec_optional: Vec<Option<String>>,
        deeply_nested: Option<Vec<Option<Vec<u8>>>>,
    }

    let mut generator = SchemaGenerator::new();
    generator.add::<Nested>();
    let output = generator.generate();

    assert!(output.contains("optional_vec: option(seq(u32()))"));
    assert!(output.contains("vec_optional: seq(option(string()))"));
    // This tests proper nesting handling
    assert!(output.contains("deeply_nested: option(seq(option(seq(u8()))))"));
}

#[test]
fn test_map_types() {
    use std::collections::HashMap;

    #[derive(PostcardTsTypegen)]
    struct Maps {
        btree: BTreeMap<String, u32>,
        hash: HashMap<u8, String>,
    }

    let mut generator = SchemaGenerator::new();
    generator.add::<Maps>();
    let output = generator.generate();

    assert!(output.contains("btree: map(string(), u32())"));
    assert!(output.contains("hash: map(u8(), string())"));
}

#[test]
fn test_typescript_syntax_validity() {
    let mut generator = SchemaGenerator::new();
    generator.add::<Point>();
    generator.add::<Status>();
    generator.add::<Container>();

    let output = generator.generate();

    // Check for balanced braces
    let open_braces = output.matches('{').count();
    let close_braces = output.matches('}').count();
    assert_eq!(open_braces, close_braces, "Braces should be balanced");

    // Check for balanced parentheses
    let open_parens = output.matches('(').count();
    let close_parens = output.matches(')').count();
    assert_eq!(open_parens, close_parens, "Parentheses should be balanced");

    // Check that all lines end properly (no missing semicolons on schema definitions)
    for line in output.lines() {
        if line.starts_with("export const ") && line.contains("Schema =") {
            // This line should end in the schema definition section
            let has_closing = output[output.find(line).unwrap()..]
                .lines()
                .take(50)  // Look ahead reasonably
                .any(|l| l.trim().ends_with("});"));
            assert!(has_closing, "Schema definition should have closing braces");
        }
    }
}

#[test]
fn test_no_duplicate_definitions() {
    let mut generator = SchemaGenerator::new();

    // Add the same type multiple times
    generator.add::<Point>();
    generator.add::<Point>();
    generator.add::<Point>();

    let output = generator.generate();

    // Count occurrences of the schema definition
    let count = output.matches("export const PointSchema =").count();
    assert_eq!(count, 1, "Should only have one definition per type");
}
