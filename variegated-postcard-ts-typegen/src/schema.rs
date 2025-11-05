//! Schema definition types for TypeScript Postcard schemas.

// Forward declare SchemaGenerator to avoid circular dependency
use crate::SchemaGenerator;

/// Trait implemented by types that can be exported to TypeScript.
pub trait PostcardTsType {
    /// Returns the TypeScript schema name (e.g., "MyTypeSchema").
    fn ts_name() -> String;

    /// Generates the schema definition for this type.
    fn generate_schema() -> SchemaDefinition;

    /// Adds this type's dependencies to the generator.
    ///
    /// This is called automatically by `SchemaGenerator::add()` to ensure
    /// all dependent types are registered. The default implementation does nothing.
    ///
    /// The derive macro generates an implementation that adds all field types
    /// as dependencies, enabling automatic dependency resolution.
    fn add_dependencies(_generator: &mut SchemaGenerator) {
        // Default implementation: no dependencies
    }
}

/// A complete schema definition for a type.
#[derive(Debug, Clone)]
pub struct SchemaDefinition {
    /// The TypeScript name of the schema (e.g., "MyTypeSchema").
    pub name: String,
    /// The kind of schema (struct, enum, etc.).
    pub kind: SchemaKind,
}

/// The kind of schema being defined.
#[derive(Debug, Clone)]
pub enum SchemaKind {
    /// A struct with named fields.
    Struct(Vec<FieldDefinition>),
    /// A tuple struct with unnamed fields.
    TupleStruct(Vec<FieldDefinition>),
    /// A unit struct.
    Unit,
    /// An enum with variants.
    Enum(Vec<EnumVariant>),
}

/// A field definition in a struct.
#[derive(Debug, Clone)]
pub struct FieldDefinition {
    /// The field name.
    pub name: String,
    /// The TypeScript type reference (e.g., "u32Schema", "StringSchema").
    pub type_ref: String,
}

/// An enum variant definition.
#[derive(Debug, Clone)]
pub struct EnumVariant {
    /// The variant name.
    pub name: String,
    /// The kind of variant.
    pub kind: VariantKind,
}

/// The kind of enum variant.
#[derive(Debug, Clone)]
pub enum VariantKind {
    /// A unit variant (no data).
    Unit,
    /// A newtype variant (single unnamed field).
    Newtype(String),
    /// A tuple variant (multiple unnamed fields).
    Tuple(Vec<String>),
    /// A struct variant (named fields).
    Struct(Vec<FieldDefinition>),
}
