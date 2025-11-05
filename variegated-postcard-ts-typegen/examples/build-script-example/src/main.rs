use variegated_postcard_ts_typegen::PostcardTsTypegen;

/// Example struct that will have a TypeScript schema generated for it
#[derive(PostcardTsTypegen)]
struct User {
    id: u32,
    name: String,
    email: Option<String>,
    age: u8,
}

/// Example enum with various variant types
#[derive(PostcardTsTypegen)]
enum Action {
    /// Unit variant
    Start,
    /// Newtype variant
    SetValue(u32),
    /// Tuple variant
    Move(f32, f32),
    /// Struct variant
    Update { field: String, value: u32 },
}

fn main() {
    println!("Build script example");
    println!("---------------------");
    println!();
    println!("This example demonstrates how to use variegated-postcard-ts-typegen");
    println!("in a build.rs script to generate TypeScript schemas from Rust types.");
    println!();
    println!("The types User and Action above have the #[derive(PostcardTsTypegen)]");
    println!("attribute, which allows them to be used with SchemaGenerator in build.rs");
}
