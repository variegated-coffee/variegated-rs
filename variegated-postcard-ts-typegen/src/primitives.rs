//! Implementations of PostcardTsType for primitive and standard library types.

use crate::schema::*;
use std::collections::{BTreeMap, HashMap};
use std::time::Duration;

// Implement for primitive types
macro_rules! impl_primitive {
    ($($ty:ty => $name:expr),* $(,)?) => {
        $(
            impl PostcardTsType for $ty {
                fn ts_name() -> String {
                    $name.to_string()
                }

                fn generate_schema() -> SchemaDefinition {
                    SchemaDefinition {
                        name: $name.to_string(),
                        kind: SchemaKind::Unit,
                    }
                }
            }
        )*
    };
}

impl_primitive! {
    u8 => "u8",
    u16 => "u16",
    u32 => "u32",
    u64 => "u64",
    u128 => "u128",
    usize => "u32",  // Map usize to u32 for TS (platform-dependent size)
    i8 => "i8",
    i16 => "i16",
    i32 => "i32",
    i64 => "i64",
    i128 => "i128",
    isize => "i32",  // Map isize to i32 for TS (platform-dependent size)
    f32 => "f32",
    f64 => "f64",
    bool => "bool",
    String => "String",
}

impl PostcardTsType for str {
    fn ts_name() -> String {
        "str".to_string()
    }

    fn generate_schema() -> SchemaDefinition {
        SchemaDefinition {
            name: "str".to_string(),
            kind: SchemaKind::Unit,
        }
    }
}

// Implement for Option<T>
impl<T: PostcardTsType> PostcardTsType for Option<T> {
    fn ts_name() -> String {
        format!("Option<{}>", T::ts_name())
    }

    fn generate_schema() -> SchemaDefinition {
        SchemaDefinition {
            name: format!("Option<{}>", T::ts_name()),
            kind: SchemaKind::Unit,
        }
    }
}

// Implement for Vec<T>
impl<T: PostcardTsType> PostcardTsType for Vec<T> {
    fn ts_name() -> String {
        format!("Vec<{}>", T::ts_name())
    }

    fn generate_schema() -> SchemaDefinition {
        SchemaDefinition {
            name: format!("Vec<{}>", T::ts_name()),
            kind: SchemaKind::Unit,
        }
    }
}

// Implement for HashMap<K, V>
impl<K: PostcardTsType, V: PostcardTsType> PostcardTsType for HashMap<K, V> {
    fn ts_name() -> String {
        format!("HashMap<{}, {}>", K::ts_name(), V::ts_name())
    }

    fn generate_schema() -> SchemaDefinition {
        SchemaDefinition {
            name: format!("HashMap<{}, {}>", K::ts_name(), V::ts_name()),
            kind: SchemaKind::Unit,
        }
    }
}

// Implement for BTreeMap<K, V>
impl<K: PostcardTsType, V: PostcardTsType> PostcardTsType for BTreeMap<K, V> {
    fn ts_name() -> String {
        format!("BTreeMap<{}, {}>", K::ts_name(), V::ts_name())
    }

    fn generate_schema() -> SchemaDefinition {
        SchemaDefinition {
            name: format!("BTreeMap<{}, {}>", K::ts_name(), V::ts_name()),
            kind: SchemaKind::Unit,
        }
    }
}

// Implement for Duration (commonly used in the types)
impl PostcardTsType for Duration {
    fn ts_name() -> String {
        "DurationSchema".to_string()
    }

    fn generate_schema() -> SchemaDefinition {
        SchemaDefinition {
            name: "DurationSchema".to_string(),
            kind: SchemaKind::Struct(vec![
                FieldDefinition {
                    name: "secs".to_string(),
                    type_ref: "u64".to_string(),
                },
                FieldDefinition {
                    name: "nanos".to_string(),
                    type_ref: "u32".to_string(),
                },
            ]),
        }
    }
}

// Implement for heapless::FnvIndexMap - treat it like a BTreeMap for TypeScript purposes
#[cfg(feature = "heapless")]
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

/// Helper macro to implement PostcardTsType for concrete generic instantiations.
/// This is useful for third-party generic types where we want to generate schemas
/// for specific instantiations (e.g., `PidOut<f32>` -> `PidOut_for_floatSchema`).
///
/// # Example
///
/// ```ignore
/// impl_postcard_ts_for_concrete!(
///     PidOut<f32> => "PidOut_for_float",
///     struct {
///         p: "f32",
///         i: "f32",
///         d: "f32",
///         out: "f32",
///         acting_kp: "f32",
///         acting_ki: "f32",
///         acting_kd: "f32",
///     }
/// );
/// ```
#[macro_export]
macro_rules! impl_postcard_ts_for_concrete {
    ($ty:ty => $schema_name:literal, struct { $($field:ident: $field_ty:expr),* $(,)? }) => {
        impl $crate::PostcardTsType for $ty {
            fn ts_name() -> String {
                $schema_name.to_string()
            }

            fn generate_schema() -> $crate::SchemaDefinition {
                $crate::SchemaDefinition {
                    name: concat!($schema_name, "Schema").to_string(),
                    kind: $crate::SchemaKind::Struct(vec![
                        $(
                            $crate::FieldDefinition {
                                name: stringify!($field).to_string(),
                                type_ref: $field_ty.to_string(),
                            }
                        ),*
                    ]),
                }
            }
        }
    };
}
