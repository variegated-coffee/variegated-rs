use proc_macro::TokenStream as TS1;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use either::{Either, Left, Right};
use inflector::Inflector;
use proc_macro2::{ Span, TokenStream as TS2 };
use quote::{quote, ToTokens, format_ident};
use syn::{Attribute, Expr, Ident, ItemStruct, ItemType, LitStr, parse_macro_input, parse_str, braced, Token, Type, TypeParamBound, Result as SynResult};
use syn::parse::{Parse, ParseStream};
use serde::Deserialize;
use syn::punctuated::Punctuated;

#[derive(Deserialize, Clone, Debug)]
struct Config {
    #[serde(flatten)]
    sections: HashMap<String, Defn>,
}

#[derive(Deserialize, Clone, Debug, Default)]
struct Defn {
    #[serde(flatten)]
    vals: HashMap<String, toml::Value>,
}

#[derive(Clone)]
struct PeripheralField {
    ident: Ident,
//    original_type: Type,
//    altered_type: Type,
    alias: ItemType,
    impls: Option<Punctuated<TypeParamBound, Token![+]>>,
    field_value: Either<Ident, toml::Value>,
    attrs: Vec<Attribute>
}

impl PeripheralField {
    fn new(struct_ident: Ident, ident: Ident, original_type: Type, section_definition: &Defn, attrs: Vec<Attribute>) -> Self {
        let alias_value = struct_ident.to_string() + ident.to_string().to_class_case().as_str();

        let alias_value = Ident::new(
            alias_value.as_str(),
            Span::call_site(),
        );

        let config = section_definition.vals.get(&ident.to_string()).unwrap_or_else(|| panic!("Board config for field {:?} missing", ident.to_string()));

        let mut impls: Option<Punctuated<TypeParamBound, Token![+]>> = None;

        let mut should_be_const = false;

        let altered_type = match original_type {
            Type::ImplTrait(ref ty) => {
                let toml::Value::String(t) = config else {
                    panic!("Type of {:?} in board-cfg.toml is not a string", ident.to_string());
                };

                impls = Some(ty.bounds.clone());

                syn::parse_str::<Type>(t.as_str()).expect("Exp:6")
            },
            Type::Tuple(_) => {
                let toml::Value::String(t) = config else {
                    panic!("Type of {:?} in board-cfg.toml is not a string", ident.to_string());
                };

                syn::parse_str::<Type>(t.as_str()).expect("Exp:7")
            },
            _ => {
                should_be_const = true;
                original_type.clone()
            }
        };
        let alias_type = altered_type.clone();
        let alias: ItemType =
            syn::parse2(quote! { type #alias_value = #alias_type; }).expect("Exp:8");

        let field_value = if should_be_const {
            Right(config.clone())
        } else {
            match altered_type.clone() {
                Type::Path(ty) => {
                    let ident = &ty.path.segments.last().unwrap().ident;
                    Left(ident.clone())
                }
                _ => panic!("For some reason {:?} shouldn't be a const, but also isn't a path", ident.to_string()),
            }
        };

        PeripheralField {
            ident,
//            original_type,
//            altered_type,
            alias,
            impls,
            field_value,
            attrs
        }
    }
}


// Define a struct to represent the input syntax
struct AliasedBindInterrupts {
    #[allow(dead_code)]
    struct_token: Token![struct],
    struct_name: Ident,
    #[allow(dead_code)]
    brace_token: syn::token::Brace,
    fields: Punctuated<BindInterruptField, Token![;]>,
}

// Define a struct to represent each field in the input
struct BindInterruptField {
    left_ident: Ident,
    #[allow(dead_code)]
    arrow_token: Token![=>],
    right_type: Type,
}

// Implement the Parse trait for AliasedBindInterrupts
impl Parse for AliasedBindInterrupts {
    fn parse(input: ParseStream) -> SynResult<Self> {
        let struct_token = input.parse()?;
        let struct_name = input.parse()?;
        let content;
        let brace_token = braced!(content in input);
        let fields = content.parse_terminated(BindInterruptField::parse, Token![;])?;

        Ok(AliasedBindInterrupts {
            struct_token,
            struct_name,
            brace_token,
            fields,
        })
    }
}

// Implement the Parse trait for BindInterruptField
impl Parse for BindInterruptField {
    fn parse(input: ParseStream) -> SynResult<Self> {
        let left_ident = input.parse()?;
        let arrow_token = input.parse()?;
        let right_type = input.parse()?;

        Ok(BindInterruptField {
            left_ident,
            arrow_token,
            right_type,
        })
    }
}


#[proc_macro]
pub fn aliased_bind_interrupts(input: TS1) -> TS1
{
    let cfg_path = get_cfg_path();

    let maybe_cfg = get_board_cfg(&cfg_path);

    let Some(cfg) = maybe_cfg else {
        panic!("Couldn't find board-cfg.toml (searched at {:?})", cfg_path);
    };

    let Some(irq_aliases) = cfg.sections.get("irq_aliases") else {
        panic!("board-cfg.toml doesn't contain a section for irq_aliases");
    };

    let alias_map: HashMap<String, String> = irq_aliases.clone().vals.into_iter().map(|(k, v)| {
        (k, v.as_str().unwrap().to_string())
    } ).collect();

//    panic!("Alias map: {}", alias_map);

/*    // Define your mapping here
    let mut alias_map = HashMap::new();
    alias_map.insert("Nau7802Irq", "I2C0_IRQ");
    alias_map.insert("DispIrq", "I2C1_IRQ");*/

    // Parse the input tokens
    let aliased_input = parse_macro_input!(input as AliasedBindInterrupts);
    let struct_name = &aliased_input.struct_name;

    // Process each field, replacing identifiers according to the map
    let updated_fields = aliased_input.fields.iter().map(|field| {
        let left_ident_str = field.left_ident.to_string();
        let right_type = &field.right_type;

        // Check if the identifier is in our map and replace it if it is
        let mapped_ident = if let Some(new_ident) = alias_map.get(left_ident_str.as_str()) {
            format_ident!("{}", new_ident)
        } else {
            field.left_ident.clone()
        };

        quote! {
            #mapped_ident => #right_type
        }
    });

    // Generate the output token stream
    let output = quote! {
        bind_interrupts!(struct #struct_name {
            #(#updated_fields;)*
        });
    };

    output.into()
}

#[proc_macro]
pub fn type_aliases(_input: TS1) -> TS1 {
    let cfg_path = get_cfg_path();

    let maybe_cfg = get_board_cfg(&cfg_path);

    let Some(cfg) = maybe_cfg else {
        panic!("Couldn't find board-cfg.toml (searched at {:?})", cfg_path);
    };

    let Some(type_aliases) = cfg.sections.get("type_aliases") else {
        panic!("board-cfg.toml doesn't contain a section for type_aliases");
    };

    let aliases_map = type_aliases.clone().vals.into_iter().map(|(k, v)| {
        let v_str = v.as_str().unwrap();

        let alias_ident = format_ident!("{}", k);
        let parsed_type: Type = parse_str(v_str).unwrap();

        syn::parse2(quote! { type #alias_ident = #parsed_type; }).expect("ExpT:1")
    } );
    let aliases_vec: Vec<TS2> = aliases_map.into_iter().collect();

    // Create a single TS2 from all the tokens
    let ts: TS2 = aliases_vec.into_iter().flatten().collect();

    // Convert the proc_macro2::TokenStream to proc_macro::TokenStream
    ts.into()
}

/// Mark a struct as a resource for extraction from the `Peripherals` instance.
#[proc_macro_attribute]
pub fn board_cfg(args: TS1, item: TS1) -> TS1 {
    let mut s: ItemStruct = syn::parse2(item.into()).expect("Resource item must be a struct.");

    let cfg_path = get_cfg_path();

    let maybe_cfg = get_board_cfg(&cfg_path);

    let input = parse_macro_input!(args as LitStr);
    let section = input.value();

    let Some(cfg) = maybe_cfg else {
        panic!("Couldn't find board-cfg.toml (searched at {:?})", cfg_path);
    };

    let Some(defs) = cfg.sections.get(&section) else {
        panic!("board-cfg.toml doesn't contain a section for {}", section);
    };

    let cfg_path_str = cfg_path.to_str().unwrap();

    let macro_ident = Ident::new(
        inflector::cases::snakecase::to_snake_case(s.ident.to_string().as_str()).as_str(),
        Span::call_site(),
    );

    let field_data: HashMap<Ident, PeripheralField> = s.fields.iter().cloned().map(|f| {
        PeripheralField::new(s.ident.clone(), f.ident.expect("Exp:10"), f.ty, defs, f.attrs)
    }).map(|p| (p.ident.clone(), p.clone())).collect();

    let aliases: Vec<ItemType> = field_data.iter().map(|(_, f)| f.alias.clone()).collect();

    let ident = &s.ident;

    s.fields.iter_mut().for_each(
        |field| {
            let ident = &field.ident.clone().expect("Exp:2");

            let alias_ident = field_data.get(ident).expect("Exp:3").alias.clone().ident;

            field.ty = syn::parse2(quote! { #alias_ident }).expect("Exp:4");
        });

    let field_idents: Vec<Ident> = field_data.iter().map(|(i, _)| i.clone()).collect();

    let field_types: Vec<TS2> = field_data.iter().map(|(_, fd)|
        match fd.field_value.clone() {
            Left(v) => quote! { $P.#v },
            Right(v) => {
                let t_string = v.to_string();
                syn::parse_str::<Expr>(&t_string).expect(&format!(
                    "Failed to parse `{}` as a valid token!",
                    &t_string
                )).to_token_stream()
            }
        }
    ).collect();

    let mut where_clauses = field_data.iter().map(|(_, fd)| {
        if fd.impls.is_none() {
            return None;
        }
        let bounds_tokens = fd.impls.iter().map(|bound| bound.to_token_stream());

        let alias_ident = &fd.alias.ident;

        Some(quote! { #alias_ident: #(#bounds_tokens)+* })
    }).filter_map(|p| p).peekable();

    let impl_clause = if where_clauses.peek().is_none() {
        quote! { }
    } else {
        quote! { impl #ident where #(#where_clauses),* {} }
    };

    let field_attrs =
        field_data.iter().map(|(_, fd)| &fd.attrs);
    let doc = format!(
        "Extract `{}` from a `Peripherals` instance.",
        ident.to_string()
    );

    let toml_recompile_hack_mod = ident.to_string().to_snake_case() + "_toml_recompile_hack";
    let toml_recompile_hack_mod = Ident::new(
        toml_recompile_hack_mod.as_str(),
        Span::call_site(),
    );

    let q =
    quote! {
        #(
            #aliases
        )*

        #s

        #impl_clause

        #[doc = #doc]
        macro_rules! #macro_ident {
            ( $P:ident ) => {
                #ident {
                    #(
                        #(
                            #field_attrs
                        )*
                        #field_idents: #field_types
                    ),*
                }
            };
        }

        mod #toml_recompile_hack_mod {
            const _: &[u8] = include_bytes!(#cfg_path_str);
        }
    };

    q.into()

}

fn get_board_cfg(cfg_path: &PathBuf) -> Option<Config> {
    load_crate_cfg(cfg_path.as_path())
}

fn load_crate_cfg(path: &Path) -> Option<Config> {
    let contents = std::fs::read_to_string(&path).ok()?;

    let parsed = toml::from_str::<Config>(&contents).ok()?;

    Some(parsed)
}

// From https://stackoverflow.com/q/60264534
fn find_root_path() -> Option<PathBuf> {
    // First we get the arguments for the rustc invocation
    let mut args = std::env::args();

    // Then we loop through them all, and find the value of "out-dir"
    let mut out_dir = None;
    while let Some(arg) = args.next() {
        if arg == "--out-dir" {
            out_dir = args.next();
        }
    }

    if out_dir.is_none() {
        // Sometimes (like when RustRover expands macros, we don't have an out-dir.
        // In that case, let's use the current working directory, if one exists.

        let current_dir = std::env::current_dir();
        if let Ok(current_dir) = current_dir {
            return Some(current_dir);
        }
    }

    // Finally we clean out_dir by removing all trailing directories, until it ends with target
    let mut out_dir = PathBuf::from(out_dir?);
    while !out_dir.ends_with("target") {
        if !out_dir.pop() {
            // We ran out of directories...
            return None;
        }
    }

    out_dir.pop();

    Some(out_dir)
}

fn get_cfg_path() -> PathBuf {
    let arg = std::env::var("BOARD_CFG_PATH");

    let cfg_path = if let Ok(arg) = arg {
        Some(PathBuf::from(arg))
    } else {
        find_root_path().map(|path| {
            let mut cfg_path = path;
            cfg_path.push("board-cfg.toml");
            cfg_path
        })
    };

    if cfg_path.is_none() {
        panic!("Couldn't find root path, and BOARD_CFG_PATH is not set.");
    }

    cfg_path.unwrap()
}