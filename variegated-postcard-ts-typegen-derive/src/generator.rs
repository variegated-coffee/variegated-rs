use proc_macro2::TokenStream;
use quote::quote;
use syn::{DataEnum, Fields, Ident};

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
