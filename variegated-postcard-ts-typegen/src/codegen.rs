//! Code generation for TypeScript schemas.

use crate::schema::*;

/// Generates TypeScript schema code from a schema definition.
pub fn generate_ts_schema(schema: &SchemaDefinition) -> String {
    let mut output = String::new();

    output.push_str(&format!("export const {} = ", schema.name));

    match &schema.kind {
        SchemaKind::Struct(fields) => {
            output.push_str("struct({\n");
            for (i, field) in fields.iter().enumerate() {
                output.push_str(&format!("    {}: {}", field.name, map_type_to_ts(&field.type_ref)));
                if i < fields.len() - 1 {
                    output.push(',');
                }
                output.push('\n');
            }
            output.push_str("});");
        }
        SchemaKind::TupleStruct(fields) => {
            // Tuple structs can be represented as tuples in TS
            output.push_str("tupleStruct([");
            for (i, field) in fields.iter().enumerate() {
                output.push_str(&map_type_to_ts(&field.type_ref));
                if i < fields.len() - 1 {
                    output.push_str(", ");
                }
            }
            output.push_str("]);");
        }
        SchemaKind::Unit => {
            output.push_str("unitVariant();");
        }
        SchemaKind::Enum(variants) => {
            let enum_name = schema.name.trim_end_matches("Schema");
            output.push_str(&format!("enumType('{}', {{\n", enum_name));

            for (i, variant) in variants.iter().enumerate() {
                output.push_str(&format!("    {}: ", variant.name));

                match &variant.kind {
                    VariantKind::Unit => {
                        output.push_str(&format!("unitVariant('{}')", variant.name));
                    }
                    VariantKind::Newtype(type_ref) => {
                        output.push_str(&format!(
                            "newtypeVariant('{}', {})",
                            variant.name,
                            map_type_to_ts(type_ref)
                        ));
                    }
                    VariantKind::Tuple(type_refs) => {
                        output.push_str(&format!("tupleVariant('{}'", variant.name));
                        for type_ref in type_refs {
                            output.push_str(&format!(", {}", map_type_to_ts(type_ref)));
                        }
                        output.push(')');
                    }
                    VariantKind::Struct(fields) => {
                        output.push_str(&format!("newtypeVariant('{}', struct({{\n", variant.name));
                        for (j, field) in fields.iter().enumerate() {
                            output.push_str(&format!(
                                "        {}: {}",
                                field.name,
                                map_type_to_ts(&field.type_ref)
                            ));
                            if j < fields.len() - 1 {
                                output.push(',');
                            }
                            output.push('\n');
                        }
                        output.push_str("    }))");
                    }
                }

                if i < variants.len() - 1 {
                    output.push(',');
                }
                output.push('\n');
            }

            output.push_str("});");
        }
    }

    output
}

/// Maps a Rust type reference to its TypeScript schema function call.
fn map_type_to_ts(type_ref: &str) -> String {
    // Handle primitive types
    match type_ref {
        "u8" => "u8()".to_string(),
        "u16" => "u16()".to_string(),
        "u32" => "u32()".to_string(),
        "u64" => "u64()".to_string(),
        "i8" => "i8()".to_string(),
        "i16" => "i16()".to_string(),
        "i32" => "i32()".to_string(),
        "i64" => "i64()".to_string(),
        "f32" => "f32()".to_string(),
        "f64" => "f64()".to_string(),
        "bool" => "bool()".to_string(),
        "String" | "str" => "string()".to_string(),
        "unit" | "()" => "struct({})".to_string(),  // Unit type maps to empty struct
        _ => {
            // Handle generic types
            if type_ref.starts_with("Option<") {
                let inner = type_ref.trim_start_matches("Option<").trim_end_matches('>');
                format!("option({})", map_type_to_ts(inner))
            } else if type_ref.starts_with("Vec<") {
                let inner = type_ref.trim_start_matches("Vec<").trim_end_matches('>');
                format!("seq({})", map_type_to_ts(inner))
            } else if type_ref.starts_with("HashMap<") || type_ref.starts_with("BTreeMap<") || type_ref.starts_with("FnvIndexMap<") {
                // Extract key and value types
                let inner = if type_ref.starts_with("HashMap<") {
                    type_ref.trim_start_matches("HashMap<")
                } else if type_ref.starts_with("BTreeMap<") {
                    type_ref.trim_start_matches("BTreeMap<")
                } else {
                    type_ref.trim_start_matches("FnvIndexMap<")
                };
                let inner = inner.trim_end_matches('>');

                // Simple parsing - this could be improved
                let parts: Vec<&str> = inner.split(',').collect();
                if parts.len() >= 2 {
                    let key = parts[0].trim();
                    let value = parts[1..].join(",").trim().to_string();
                    format!("map({}, {})", map_type_to_ts(key), map_type_to_ts(&value))
                } else {
                    // Fallback
                    format!("{}Schema", type_ref)
                }
            } else {
                // Assume it's a custom type - may have generics
                normalize_generic_type_name(type_ref)
            }
        }
    }
}

/// Normalizes a generic type name to match the schema naming convention.
/// E.g., "PidOut<f32>" -> "PidOut_for_floatSchema"
fn normalize_generic_type_name(type_name: &str) -> String {
    if type_name.ends_with("Schema") {
        return type_name.to_string();
    }

    if type_name.contains('<') {
        // Handle generic types like PidOut<f32> -> PidOut_for_float
        let base = type_name.split('<').next().unwrap();
        let generics = type_name
            .trim_start_matches(base)
            .trim_start_matches('<')
            .trim_end_matches('>')
            .split(',')
            .map(|s| s.trim())
            .collect::<Vec<_>>();

        let generic_suffix = generics
            .iter()
            .map(|g| type_to_suffix(g))
            .collect::<Vec<_>>()
            .join("_");

        format!("{}_for_{}Schema", base, generic_suffix)
    } else {
        format!("{}Schema", type_name)
    }
}

/// Converts a type name to a suffix for generic type names.
/// E.g., "f32" -> "float", "u32" -> "u32"
fn type_to_suffix(type_name: &str) -> String {
    match type_name {
        "f32" => "float".to_string(),
        "f64" => "double".to_string(),
        other => other.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_map_primitive_types() {
        assert_eq!(map_type_to_ts("u8"), "u8()");
        assert_eq!(map_type_to_ts("u32"), "u32()");
        assert_eq!(map_type_to_ts("f32"), "f32()");
        assert_eq!(map_type_to_ts("bool"), "bool()");
        assert_eq!(map_type_to_ts("String"), "string()");
    }

    #[test]
    fn test_map_option() {
        assert_eq!(map_type_to_ts("Option<u32>"), "option(u32())");
        assert_eq!(map_type_to_ts("Option<String>"), "option(string())");
    }

    #[test]
    fn test_map_vec() {
        assert_eq!(map_type_to_ts("Vec<u8>"), "seq(u8())");
        assert_eq!(map_type_to_ts("Vec<String>"), "seq(string())");
    }

    #[test]
    fn test_map_custom_type() {
        assert_eq!(map_type_to_ts("MyType"), "MyTypeSchema");
        assert_eq!(map_type_to_ts("MyTypeSchema"), "MyTypeSchema");
    }
}
