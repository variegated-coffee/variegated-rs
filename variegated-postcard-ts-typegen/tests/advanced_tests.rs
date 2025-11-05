use variegated_postcard_ts_typegen::{PostcardTsType, PostcardTsTypegen, SchemaGenerator};

// Test nested enum variants similar to schema.ts
#[derive(PostcardTsTypegen)]
enum DerivedFormula {
    Linear { base_param: u8, multiplier: f32, offset: f32 },
    Sum { params: Vec<u8> },
    Difference { param_a: u8, param_b: u8 },
    Product { params: Vec<u8> },
}

#[derive(PostcardTsTypegen)]
enum StateCondition {
    Brewing(u8),
    NotBrewing(u8),
    BoilerTemperatureAbove(u8, f32),
    BoilerTemperatureBelow(u8, f32),
}

#[derive(PostcardTsTypegen)]
struct Limits {
    lower: f32,
    upper: f32,
}

#[derive(PostcardTsTypegen)]
struct PidTerm {
    positive_scale: f32,
    negative_scale: f32,
    limits: Limits,
}

#[derive(PostcardTsTypegen)]
struct PidParameters {
    kp: PidTerm,
    ki: PidTerm,
    kd: PidTerm,
}

#[test]
fn test_enum_with_struct_variants() {
    let schema = DerivedFormula::generate_schema();
    assert_eq!(schema.name, "DerivedFormulaSchema");

    let mut generator = SchemaGenerator::new();
    generator.add::<DerivedFormula>();

    let output = generator.generate();
    assert!(output.contains("Linear:"));
    assert!(output.contains("base_param:"));
    assert!(output.contains("multiplier:"));
}

#[test]
fn test_enum_with_tuple_variants() {
    let schema = StateCondition::generate_schema();
    assert_eq!(schema.name, "StateConditionSchema");

    let mut generator = SchemaGenerator::new();
    generator.add::<StateCondition>();

    let output = generator.generate();
    assert!(output.contains("Brewing:"));
    assert!(output.contains("BoilerTemperatureAbove:"));
}

#[test]
fn test_nested_struct_types() {
    let mut generator = SchemaGenerator::new();
    generator.add::<Limits>();
    generator.add::<PidTerm>();
    generator.add::<PidParameters>();

    let output = generator.generate();

    // Check that all types are present
    assert!(output.contains("LimitsSchema"));
    assert!(output.contains("PidTermSchema"));
    assert!(output.contains("PidParametersSchema"));

    // Check that nested references are correct
    assert!(output.contains("limits: LimitsSchema"));
    assert!(output.contains("kp: PidTermSchema"));
}

#[test]
fn test_option_and_vec_generation() {
    #[derive(PostcardTsTypegen)]
    struct ComplexFields {
        simple: u32,
        optional_simple: Option<u32>,
        list: Vec<String>,
        optional_list: Option<Vec<f32>>,
    }

    let mut generator = SchemaGenerator::new();
    generator.add::<ComplexFields>();

    let output = generator.generate();

    // Check for proper option() and seq() wrapping
    assert!(output.contains("option(u32())"));
    assert!(output.contains("seq(string())"));
}

#[test]
fn test_map_with_custom_types() {
    use std::collections::BTreeMap;

    #[derive(PostcardTsTypegen)]
    struct Status {
        value: u32,
    }

    #[derive(PostcardTsTypegen)]
    struct Container {
        items: BTreeMap<u8, Status>,
    }

    let mut generator = SchemaGenerator::new();
    generator.add::<Status>();
    generator.add::<Container>();

    let output = generator.generate();

    // Check that map references work correctly
    assert!(output.contains("StatusSchema"));
    assert!(output.contains("map("));
}

#[test]
fn test_unit_enum_variant() {
    #[derive(PostcardTsTypegen)]
    enum Mode {
        On,
        Off,
        Standby,
    }

    let schema = Mode::generate_schema();
    assert_eq!(schema.name, "ModeSchema");

    let mut generator = SchemaGenerator::new();
    generator.add::<Mode>();

    let output = generator.generate();

    assert!(output.contains("On: unitVariant('On')"));
    assert!(output.contains("Off: unitVariant('Off')"));
    assert!(output.contains("Standby: unitVariant('Standby')"));
}

#[test]
fn test_newtype_variant() {
    #[derive(PostcardTsTypegen)]
    enum Wrapper {
        Value(u32),
        Text(String),
    }

    let mut generator = SchemaGenerator::new();
    generator.add::<Wrapper>();

    let output = generator.generate();

    assert!(output.contains("newtypeVariant('Value', u32())"));
    assert!(output.contains("newtypeVariant('Text', string())"));
}

#[test]
fn test_tuple_variant() {
    #[derive(PostcardTsTypegen)]
    enum Coordinate {
        TwoD(f32, f32),
        ThreeD(f32, f32, f32),
    }

    let mut generator = SchemaGenerator::new();
    generator.add::<Coordinate>();

    let output = generator.generate();

    assert!(output.contains("tupleVariant('TwoD'"));
    assert!(output.contains("tupleVariant('ThreeD'"));
}
