use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, Data, DeriveInput};

mod generator;

/// Derive macro for generating TypeScript Postcard schema definitions.
///
/// This macro generates TypeScript schema code compatible with the
/// `@variegated-coffee/serde-postcard-ts` library, preserving field order
/// for correct Postcard serialization.
///
/// # Examples
///
/// ```ignore
/// #[derive(PostcardTsTypegen)]
/// struct MyStruct {
///     field1: u32,
///     field2: String,
/// }
/// ```
#[proc_macro_derive(PostcardTsTypegen, attributes(postcard_ts))]
pub fn derive_postcard_ts_typegen(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);

    let name = &input.ident;
    let (impl_generics, ty_generics, where_clause) = input.generics.split_for_impl();

    // Generate the schema based on the type structure
    let schema_gen = match &input.data {
        Data::Struct(data_struct) => {
            generator::generate_struct_schema(name, &data_struct.fields)
        }
        Data::Enum(data_enum) => {
            generator::generate_enum_schema(name, data_enum)
        }
        Data::Union(_) => {
            return syn::Error::new_spanned(
                &input.ident,
                "PostcardTsTypegen does not support unions"
            )
            .to_compile_error()
            .into();
        }
    };

    let expanded = quote! {
        impl #impl_generics ::variegated_postcard_ts_typegen::PostcardTsType for #name #ty_generics #where_clause {
            fn ts_name() -> ::std::string::String {
                #schema_gen.name
            }

            fn generate_schema() -> ::variegated_postcard_ts_typegen::SchemaDefinition {
                #schema_gen
            }
        }
    };

    TokenStream::from(expanded)
}
