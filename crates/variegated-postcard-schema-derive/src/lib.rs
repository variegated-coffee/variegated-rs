//! `#[derive(PostcardSchema)]`.
//!
//! Describes a struct or enum as a node in the serde data model, preserving the two
//! things postcard encodes positionally and therefore cannot recover: field
//! declaration order, and enum variant declaration order.
//!
//! The derive is small because the types it serves carry no serde attributes. It stays
//! small because it refuses to compile anything that does -- see [`reject_serde_attrs`].

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::punctuated::Punctuated;
use syn::spanned::Spanned;
use syn::{
    Attribute, Data, DeriveInput, Expr, ExprLit, Fields, Lit, Meta, Token, Variant, parse_quote,
};

#[proc_macro_derive(PostcardSchema, attributes(postcard_schema))]
pub fn derive_postcard_schema(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as DeriveInput);
    expand(input).unwrap_or_else(|e| e.to_compile_error()).into()
}

fn expand(input: DeriveInput) -> syn::Result<TokenStream2> {
    reject_serde_attrs(&input.attrs)?;

    let ident = &input.ident;
    let ident_str = ident.to_string();

    // Type parameters gain a `PostcardSchema` bound. Lifetimes and const parameters
    // are skipped: neither can change the serialized shape, and `WsMessage<'a>` must
    // stay derivable.
    let mut bounded = input.generics.clone();
    for param in bounded.type_params_mut() {
        param.bounds.push(parse_quote!(::variegated_postcard_schema::PostcardSchema));
    }
    let (impl_generics, _, where_clause) = bounded.split_for_impl();
    let (_, ty_generics, _) = input.generics.split_for_impl();

    // `core::any::type_name` of each type argument, so two monomorphizations of the
    // same generic type are distinguishable in one output rather than colliding.
    let type_args: Vec<TokenStream2> = input
        .generics
        .type_params()
        .map(|p| {
            let t = &p.ident;
            quote!(::core::any::type_name::<#t>())
        })
        .collect();
    let args = vec_expr(&type_args);

    let def = match &input.data {
        Data::Struct(data) => struct_def(&data.fields)?,
        Data::Enum(data) => enum_def(data, &input.attrs)?,
        Data::Union(_) => {
            return Err(syn::Error::new_spanned(
                ident,
                "PostcardSchema cannot describe a union: serde has no union representation",
            ));
        }
    };

    Ok(quote! {
        #[automatically_derived]
        impl #impl_generics ::variegated_postcard_schema::PostcardSchema
            for #ident #ty_generics #where_clause
        {
            fn type_id() -> ::core::option::Option<::variegated_postcard_schema::TypeId> {
                ::core::option::Option::Some(::variegated_postcard_schema::TypeId {
                    // Expands at the definition site, so this is the defining module.
                    module: ::core::module_path!(),
                    ident: #ident_str,
                    args: #args,
                })
            }

            fn register(reg: &mut ::variegated_postcard_schema::Registry) {
                let id = <Self as ::variegated_postcard_schema::PostcardSchema>::type_id()
                    .expect("derived impls always have a TypeId");
                // Marks the type in-progress before walking fields, so a back edge is
                // recognizable; returns false when already registered, which is what
                // makes a diamond emit one definition rather than two.
                if !reg.begin(id.clone()) {
                    return;
                }
                let def = #def;
                reg.finish(id, def);
            }

            fn node(reg: &mut ::variegated_postcard_schema::Registry) -> ::variegated_postcard_schema::Node {
                let id = <Self as ::variegated_postcard_schema::PostcardSchema>::type_id()
                    .expect("derived impls always have a TypeId");
                <Self as ::variegated_postcard_schema::PostcardSchema>::register(reg);
                // `reference` rather than `Node::Ref`: it panics on a back edge, which
                // the target vocabulary cannot express.
                reg.reference(id)
            }
        }
    })
}

/// Build a `Vec` expression without needing `alloc` in scope at the expansion site.
fn vec_expr(items: &[TokenStream2]) -> TokenStream2 {
    if items.is_empty() {
        quote!(::variegated_postcard_schema::__private::Vec::new())
    } else {
        quote!(::variegated_postcard_schema::__private::vec_from([#(#items),*]))
    }
}

fn node_of(ty: &syn::Type) -> TokenStream2 {
    quote!(<#ty as ::variegated_postcard_schema::PostcardSchema>::node(reg))
}

fn struct_def(fields: &Fields) -> syn::Result<TokenStream2> {
    Ok(match fields {
        Fields::Unit => quote!(::variegated_postcard_schema::Def::UnitStruct),
        Fields::Named(named) => {
            let entries = named_entries(named.named.iter())?;
            let v = vec_expr(&entries);
            quote!(::variegated_postcard_schema::Def::Struct(#v))
        }
        Fields::Unnamed(unnamed) => {
            for f in &unnamed.unnamed {
                reject_serde_attrs(&f.attrs)?;
            }
            let tys: Vec<_> = unnamed.unnamed.iter().map(|f| node_of(&f.ty)).collect();
            // serde distinguishes these: a one-field tuple struct is a newtype and
            // serializes as its inner value, with no length or wrapper.
            if tys.len() == 1 {
                let inner = &tys[0];
                quote!(::variegated_postcard_schema::Def::NewtypeStruct(#inner))
            } else {
                let v = vec_expr(&tys);
                quote!(::variegated_postcard_schema::Def::TupleStruct(#v))
            }
        }
    })
}

fn named_entries<'a>(
    fields: impl Iterator<Item = &'a syn::Field>,
) -> syn::Result<Vec<TokenStream2>> {
    let mut out = Vec::new();
    for f in fields {
        reject_serde_attrs(&f.attrs)?;
        let name = f.ident.as_ref().expect("named field").to_string();
        let node = node_of(&f.ty);
        out.push(quote!((#name, #node)));
    }
    Ok(out)
}

fn enum_def(data: &syn::DataEnum, container_attrs: &[Attribute]) -> syn::Result<TokenStream2> {
    let allow_discriminants = has_flag(container_attrs, "allow_discriminants");

    let mut variants = Vec::new();
    for (index, v) in data.variants.iter().enumerate() {
        reject_serde_attrs(&v.attrs)?;
        check_discriminant(v, index, allow_discriminants)?;
        variants.push(variant_expr(v)?);
    }

    let v = vec_expr(&variants);
    Ok(quote!(::variegated_postcard_schema::Def::Enum(#v)))
}

fn variant_expr(v: &Variant) -> syn::Result<TokenStream2> {
    let name = v.ident.to_string();
    Ok(match &v.fields {
        Fields::Unit => quote!(::variegated_postcard_schema::Variant::Unit(#name)),
        Fields::Named(named) => {
            let entries = named_entries(named.named.iter())?;
            let e = vec_expr(&entries);
            quote!(::variegated_postcard_schema::Variant::Struct(#name, #e))
        }
        Fields::Unnamed(unnamed) => {
            for f in &unnamed.unnamed {
                reject_serde_attrs(&f.attrs)?;
            }
            let tys: Vec<_> = unnamed.unnamed.iter().map(|f| node_of(&f.ty)).collect();
            if tys.len() == 1 {
                let inner = &tys[0];
                quote!(::variegated_postcard_schema::Variant::Newtype(#name, #inner))
            } else {
                let e = vec_expr(&tys);
                quote!(::variegated_postcard_schema::Variant::Tuple(#name, #e))
            }
        }
    })
}

/// Reject an explicit discriminant that disagrees with the variant's position.
///
/// serde ignores `#[repr]` discriminants entirely -- an externally tagged enum is
/// encoded by *index*, so `Foo = 5` in fifth position still goes on the wire as 4.
/// Someone writing an explicit discriminant almost certainly believes otherwise, and
/// silently disagreeing with them is exactly the class of mistake this crate exists to
/// prevent. Discriminants that match their index are accepted, since they cannot
/// mislead.
fn check_discriminant(v: &Variant, index: usize, allowed: bool) -> syn::Result<()> {
    let Some((_, expr)) = &v.discriminant else {
        return Ok(());
    };
    if allowed {
        return Ok(());
    }

    if let Expr::Lit(ExprLit { lit: Lit::Int(n), .. }) = expr {
        if n.base10_parse::<usize>().ok() == Some(index) {
            return Ok(());
        }
    }

    Err(syn::Error::new(
        expr.span(),
        format!(
            "explicit discriminant does not match the variant index ({index}).\n\
             serde encodes an externally tagged enum by declaration order and ignores \
             the discriminant, so this variant would go on the wire as {index}, not as \
             written. Reorder the variants, drop the discriminant, or add \
             `#[postcard_schema(allow_discriminants)]` if the mismatch is intended."
        ),
    ))
}

fn has_flag(attrs: &[Attribute], flag: &str) -> bool {
    attrs.iter().any(|attr| {
        if !attr.path().is_ident("postcard_schema") {
            return false;
        }
        attr.parse_args_with(Punctuated::<Meta, Token![,]>::parse_terminated)
            .map(|metas| metas.iter().any(|m| m.path().is_ident(flag)))
            .unwrap_or(false)
    })
}

/// Refuse any serde attribute other than `borrow`.
///
/// This is a denylist by default, not an allowlist: anything whose path is not
/// `borrow` is rejected, including attributes that did not exist when this was
/// written. That is the point. `rename`, `rename_all`, `tag`, `untagged`, `flatten`,
/// `skip`, `default`, `with`, `serialize_with` and `transparent` all change the data
/// model, and a schema generated in ignorance of them would be confidently wrong --
/// with no test able to notice, since postcard encodes neither field names nor the
/// attributes' effects in a recoverable way.
///
/// `borrow` is the sole exception because it is a zero-copy deserialization hint with
/// no effect on the bytes. `WsMessage::CommandAck::error` uses it.
///
/// A handful of genuinely harmless attributes (`deny_unknown_fields`, `expecting`) are
/// rejected too. That is a deliberate false positive: the cost is one line to allow
/// them the day someone wants one, and the benefit is never having to re-derive which
/// serde attributes are safe.
fn reject_serde_attrs(attrs: &[Attribute]) -> syn::Result<()> {
    for attr in attrs {
        if !attr.path().is_ident("serde") {
            continue;
        }
        let metas = attr.parse_args_with(Punctuated::<Meta, Token![,]>::parse_terminated)?;
        for meta in metas {
            let path = meta.path();
            if path.is_ident("borrow") {
                continue;
            }
            let name = path
                .get_ident()
                .map(|i| i.to_string())
                .unwrap_or_else(|| quote!(#path).to_string());
            return Err(syn::Error::new(
                meta.span(),
                format!(
                    "`#[serde({name})]` is not modelled by PostcardSchema.\n\
                     The generated schema would describe the type as if this attribute \
                     were absent, and because postcard encodes neither field names nor \
                     this attribute's effect, no round-trip test could detect the \
                     mismatch. Either teach the derive about it, or do not put this \
                     type on the wire.\n\
                     (Only `borrow` is accepted -- it is a borrowing hint and does not \
                     change the encoding.)"
                ),
            ));
        }
    }
    Ok(())
}
