# variegated-postcard-ts-typegen

Generate TypeScript schema definitions from Rust types for use with Postcard serialization.

## Overview

This crate provides a derive macro and code generation tools to automatically create TypeScript schema definitions from Rust types. The generated schemas are compatible with the `@variegated-coffee/serde-postcard-ts` library and preserve field order for correct Postcard serialization.

## Why This Crate?

[Postcard](https://github.com/jamesmunns/postcard) is a `#![no_std]` friendly serialization format that is order-dependent - fields are serialized and deserialized in declaration order. This makes it critical that TypeScript schemas match the exact field order of Rust structs.

This crate solves the problem of manually maintaining TypeScript schemas by:
- Automatically generating schemas directly from Rust type definitions
- Preserving field order to match Postcard serialization
- Handling complex types including enums, generics, and nested structures
- Providing a build.rs integration for automatic schema generation

## Features

- **Derive Macro**: Simple `#[derive(PostcardTsTypegen)]` to mark types for export
- **Field Order Preservation**: Maintains exact field order for Postcard compatibility
- **Full Type Support**:
  - Structs with named fields
  - Enums (unit, newtype, tuple, and struct variants)
  - Generics (`Option<T>`, `Vec<T>`, `HashMap<K,V>`, etc.)
  - Primitive types
  - Nested types
- **Build Script Integration**: Call from `build.rs` to auto-generate schemas
- **Type-Safe**: Leverages Rust's type system to ensure correctness

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
variegated-postcard-ts-typegen = "0.1.0"

[build-dependencies]
variegated-postcard-ts-typegen = "0.1.0"
```

## Usage

### 1. Derive the Trait

Add `#[derive(PostcardTsTypegen)]` to types you want to export:

```rust
use variegated_postcard_ts_typegen::PostcardTsTypegen;

#[derive(PostcardTsTypegen)]
struct User {
    id: u32,
    name: String,
    email: Option<String>,
}

#[derive(PostcardTsTypegen)]
enum Status {
    Active,
    Inactive,
    Pending { reason: String },
}
```

### 2. Generate Schemas in build.rs

Create a `build.rs` file:

```rust
use variegated_postcard_ts_typegen::SchemaGenerator;

// Import your types
use my_crate::{User, Status};

fn main() {
    let mut generator = SchemaGenerator::new();

    // Add types to generate
    generator.add::<User>();
    generator.add::<Status>();

    // Write to file
    generator.write_to_file("schema.ts").unwrap();

    println!("cargo:rerun-if-changed=build.rs");
}
```

### 3. Generated Output

The crate generates TypeScript schemas like:

```typescript
export const UserSchema = struct({
    id: u32(),
    name: string(),
    email: option(string())
});

export const StatusSchema = enumType('Status', {
    Active: unitVariant('Active'),
    Inactive: unitVariant('Inactive'),
    Pending: newtypeVariant('Pending', struct({
        reason: string()
    }))
});

export type User = InferType<typeof UserSchema>;
export type Status = InferType<typeof StatusSchema>;
```

## Supported Types

### Primitives
- Integers: `u8`, `u16`, `u32`, `u64`, `i8`, `i16`, `i32`, `i64`
- Floats: `f32`, `f64`
- `bool`, `String`

### Generic Types
- `Option<T>`
- `Vec<T>`
- `HashMap<K, V>`
- `BTreeMap<K, V>`

### Custom Types
Any type that derives `PostcardTsTypegen` can be referenced by other types.

### Standard Library Types
- `std::time::Duration` (generates as struct with `secs` and `nanos` fields)

## Important Notes

### Field Order

**Field order matters!** Postcard serialization is order-dependent. The generated TypeScript schemas will match the exact field order in your Rust structs. Do not reorder fields in your Rust code without regenerating the TypeScript schemas.

### Build Script Tips

When using in `build.rs`:
1. Always add `println!("cargo:rerun-if-changed=build.rs");`
2. Consider adding rerun-if-changed for your source files if needed
3. The generated file will be created at build time

### Embedded/No-Std Compatibility

While this crate is designed to generate schemas for `no_std` Rust types, the code generation itself requires `std` and runs in a build script context (not on embedded targets).

## Example

See the `examples/build-script-example` directory for a complete working example.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
