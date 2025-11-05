use proc_macro2::TokenStream;
use quote::quote;
use syn::{DataEnum, Fields, Ident, Type, GenericArgument, PathArguments};
use std::collections::HashSet;

pub fn generate_struct_schema(name: &Ident, fields: &Fields) -> TokenStream {
    let ts_name = format!("{}Schema", name);

    match fields {
        Fields::Named(fields_named) => {
            let field_schemas: Vec<_> = fields_named
                .named
                .iter()
                .map(|field| {
                    let field_name = field.ident.as_ref().unwrap().to_string();
                    let field_ty = &field.ty;

                    quote! {
                        ::variegated_postcard_ts_typegen::FieldDefinition {
                            name: #field_name.to_string(),
                            type_ref: <#field_ty as ::variegated_postcard_ts_typegen::PostcardTsType>::ts_name(),
                        }
                    }
                })
                .collect();

            quote! {
                ::variegated_postcard_ts_typegen::SchemaDefinition {
                    name: #ts_name.to_string(),
                    kind: ::variegated_postcard_ts_typegen::SchemaKind::Struct(
                        vec![#(#field_schemas),*]
                    ),
                }
            }
        }
        Fields::Unnamed(fields_unnamed) => {
            // Tuple structs - not commonly used in the schema.ts but we should support them
            let field_schemas: Vec<_> = fields_unnamed
                .unnamed
                .iter()
                .enumerate()
                .map(|(idx, field)| {
                    let field_ty = &field.ty;
                    quote! {
                        ::variegated_postcard_ts_typegen::FieldDefinition {
                            name: #idx.to_string(),
                            type_ref: <#field_ty as ::variegated_postcard_ts_typegen::PostcardTsType>::ts_name(),
                        }
                    }
                })
                .collect();

            quote! {
                ::variegated_postcard_ts_typegen::SchemaDefinition {
                    name: #ts_name.to_string(),
                    kind: ::variegated_postcard_ts_typegen::SchemaKind::TupleStruct(
                        vec![#(#field_schemas),*]
                    ),
                }
            }
        }
        Fields::Unit => {
            quote! {
                ::variegated_postcard_ts_typegen::SchemaDefinition {
                    name: #ts_name.to_string(),
                    kind: ::variegated_postcard_ts_typegen::SchemaKind::Unit,
                }
            }
        }
    }
}

pub fn generate_enum_schema(name: &Ident, data_enum: &DataEnum) -> TokenStream {
    let ts_name = format!("{}Schema", name);

    let variant_schemas: Vec<_> = data_enum
        .variants
        .iter()
        .map(|variant| {
            let variant_name = variant.ident.to_string();

            match &variant.fields {
                Fields::Unit => {
                    quote! {
                        ::variegated_postcard_ts_typegen::EnumVariant {
                            name: #variant_name.to_string(),
                            kind: ::variegated_postcard_ts_typegen::VariantKind::Unit,
                        }
                    }
                }
                Fields::Unnamed(fields) => {
                    let field_types: Vec<_> = fields
                        .unnamed
                        .iter()
                        .map(|field| {
                            let field_ty = &field.ty;
                            quote! {
                                <#field_ty as ::variegated_postcard_ts_typegen::PostcardTsType>::ts_name()
                            }
                        })
                        .collect();

                    if field_types.len() == 1 {
                        quote! {
                            ::variegated_postcard_ts_typegen::EnumVariant {
                                name: #variant_name.to_string(),
                                kind: ::variegated_postcard_ts_typegen::VariantKind::Newtype(
                                    #(#field_types)*
                                ),
                            }
                        }
                    } else {
                        quote! {
                            ::variegated_postcard_ts_typegen::EnumVariant {
                                name: #variant_name.to_string(),
                                kind: ::variegated_postcard_ts_typegen::VariantKind::Tuple(
                                    vec![#(#field_types),*]
                                ),
                            }
                        }
                    }
                }
                Fields::Named(fields) => {
                    let field_defs: Vec<_> = fields
                        .named
                        .iter()
                        .map(|field| {
                            let field_name = field.ident.as_ref().unwrap().to_string();
                            let field_ty = &field.ty;
                            quote! {
                                ::variegated_postcard_ts_typegen::FieldDefinition {
                                    name: #field_name.to_string(),
                                    type_ref: <#field_ty as ::variegated_postcard_ts_typegen::PostcardTsType>::ts_name(),
                                }
                            }
                        })
                        .collect();

                    quote! {
                        ::variegated_postcard_ts_typegen::EnumVariant {
                            name: #variant_name.to_string(),
                            kind: ::variegated_postcard_ts_typegen::VariantKind::Struct(
                                vec![#(#field_defs),*]
                            ),
                        }
                    }
                }
            }
        })
        .collect();

    quote! {
        ::variegated_postcard_ts_typegen::SchemaDefinition {
            name: #ts_name.to_string(),
            kind: ::variegated_postcard_ts_typegen::SchemaKind::Enum(
                vec![#(#variant_schemas),*]
            ),
        }
    }
}

/// Checks if a type is a primitive or standard library type that shouldn't be added as a dependency.
fn is_primitive_or_std_type(ty: &Type) -> bool {
    let ty_str = quote! { #ty }.to_string();

    // Remove whitespace for comparison
    let ty_str = ty_str.replace(" ", "");

    // Primitive types
    if matches!(ty_str.as_str(),
        "u8" | "u16" | "u32" | "u64" | "u128" | "usize" |
        "i8" | "i16" | "i32" | "i64" | "i128" | "isize" |
        "f32" | "f64" | "bool" | "char" | "str" | "()"
    ) {
        return true;
    }

    // Check for common std types (String, Vec, Option, BTreeMap, HashMap, etc.)
    // These are handled specially by the type system
    if ty_str.starts_with("String") ||
       ty_str.starts_with("Vec<") ||
       ty_str.starts_with("Option<") ||
       ty_str.starts_with("BTreeMap<") ||
       ty_str.starts_with("HashMap<") ||
       ty_str.starts_with("Box<") ||
       ty_str.starts_with("std::") ||
       ty_str.starts_with("alloc::") ||
       ty_str.starts_with("core::") {
        return true;
    }

    false
}

/// Recursively extracts all non-primitive types from a Type, including from generic parameters.
/// For example, from `Option<Vec<MyType>>`, extracts `MyType`.
fn extract_custom_types_recursively(ty: &Type, types: &mut Vec<Type>) {
    match ty {
        Type::Path(type_path) => {
            // First, check if this whole type is a custom type (not a primitive/std type)
            if !is_primitive_or_std_type(ty) {
                types.push(ty.clone());
            }

            // Then recursively extract from generic parameters
            if let Some(segment) = type_path.path.segments.last() {
                if let PathArguments::AngleBracketed(args) = &segment.arguments {
                    for arg in &args.args {
                        if let GenericArgument::Type(inner_ty) = arg {
                            extract_custom_types_recursively(inner_ty, types);
                        }
                    }
                }
            }
        }
        Type::Reference(type_ref) => {
            extract_custom_types_recursively(&type_ref.elem, types);
        }
        Type::Tuple(type_tuple) => {
            for elem in &type_tuple.elems {
                extract_custom_types_recursively(elem, types);
            }
        }
        Type::Array(type_array) => {
            extract_custom_types_recursively(&type_array.elem, types);
        }
        Type::Ptr(type_ptr) => {
            extract_custom_types_recursively(&type_ptr.elem, types);
        }
        Type::Slice(type_slice) => {
            extract_custom_types_recursively(&type_slice.elem, types);
        }
        _ => {}
    }
}

/// Extracts all unique field types from struct fields, including types within generics.
fn extract_types_from_fields(fields: &Fields) -> Vec<Type> {
    let mut types = Vec::new();

    let field_types: Vec<_> = match fields {
        Fields::Named(fields_named) => {
            fields_named.named.iter().map(|f| &f.ty).collect()
        }
        Fields::Unnamed(fields_unnamed) => {
            fields_unnamed.unnamed.iter().map(|f| &f.ty).collect()
        }
        Fields::Unit => Vec::new(),
    };

    for ty in field_types {
        extract_custom_types_recursively(ty, &mut types);
    }

    types
}

/// Extracts all unique field types from enum variants, including types within generics.
fn extract_types_from_enum(data_enum: &DataEnum) -> Vec<Type> {
    let mut types = Vec::new();

    for variant in &data_enum.variants {
        types.extend(extract_types_from_fields(&variant.fields));
    }

    types
}

/// Generates the add_dependencies method implementation.
pub fn generate_add_dependencies(fields: Option<&Fields>, data_enum: Option<&DataEnum>) -> TokenStream {
    let types: Vec<Type> = if let Some(fields) = fields {
        extract_types_from_fields(fields)
    } else if let Some(data_enum) = data_enum {
        extract_types_from_enum(data_enum)
    } else {
        Vec::new()
    };

    // Deduplicate by type string representation
    let mut seen = HashSet::new();
    let unique_types: Vec<_> = types
        .into_iter()
        .filter(|ty| {
            let ty_str = quote! { #ty }.to_string();
            seen.insert(ty_str)
        })
        .collect();

    if unique_types.is_empty() {
        // No dependencies - use default implementation
        return quote! {};
    }

    let add_calls: Vec<_> = unique_types
        .iter()
        .map(|ty| {
            quote! {
                generator.add::<#ty>();
            }
        })
        .collect();

    quote! {
        fn add_dependencies(generator: &mut ::variegated_postcard_ts_typegen::SchemaGenerator) {
            #(#add_calls)*
        }
    }
}
