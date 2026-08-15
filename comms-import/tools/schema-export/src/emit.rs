//! Turns a `Registry` into the TypeScript that `frontend/src/schemas/schemas.ts` is.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

use variegated_postcard_schema::{Def, Node, Registry, TypeId, Variant};

/// TypeScript names for the registered types.
pub struct Names(BTreeMap<TypeId, String>);

impl Names {
    pub fn get(&self, id: &TypeId) -> &str {
        &self.0[id]
    }
}

/// Assign a TypeScript name to every definition.
///
/// The bare Rust identifier, unless two distinct types share it. Collisions are
/// disambiguated first by generic arguments (`PidParameters_f32`) and then by module
/// path, and a collision that survives both is a panic rather than an arbitrary
/// winner -- silently emitting one of two types under a shared name is exactly the
/// kind of quiet wrongness this generator exists to remove.
///
/// Nothing collides today, so the PID types come out as plain `Limits`,
/// `PidParameters`, `PidTerm` and `PidOut`, dropping the `_for_float` suffixes the
/// schemars-era pipeline left behind. If someone later puts a `PidParameters<f64>` on
/// the wire, the existing name gains an `_f32` suffix -- a visible, reviewable diff in
/// a generated file, which beats carrying disambiguating noise forever on the chance
/// that it might one day be needed.
fn resolve_names(defs: &[(TypeId, Def)]) -> Names {
    let mut by_ident: BTreeMap<&str, Vec<&TypeId>> = BTreeMap::new();
    for (id, _) in defs {
        by_ident.entry(id.ident).or_default().push(id);
    }

    let mut names = BTreeMap::new();
    for (ident, ids) in by_ident {
        if ids.len() == 1 {
            names.insert(ids[0].clone(), ident.to_string());
            continue;
        }

        // Same short name, different types. Try generic arguments.
        let mut candidates: BTreeMap<String, Vec<&TypeId>> = BTreeMap::new();
        for id in &ids {
            let mut name = ident.to_string();
            for arg in &id.args {
                name.push('_');
                name.push_str(&sanitize(arg));
            }
            candidates.entry(name).or_default().push(id);
        }

        for (name, group) in candidates {
            if group.len() == 1 {
                names.insert(group[0].clone(), name);
                continue;
            }
            // Still ambiguous. Fall back to the module path.
            let mut seen = BTreeSet::new();
            for id in group {
                let full = format!("{}_{}", sanitize(id.module), name);
                assert!(
                    seen.insert(full.clone()),
                    "cannot give `{}` a unique TypeScript name: two types share a module \
                     path, identifier and generic arguments",
                    ident
                );
                names.insert(id.clone(), full);
            }
        }
    }

    Names(names)
}

fn sanitize(s: &str) -> String {
    s.chars().map(|c| if c.is_ascii_alphanumeric() { c } else { '_' }).collect()
}

fn refs_in_node(node: &Node, out: &mut BTreeSet<TypeId>) {
    match node {
        Node::Ref(id) => {
            out.insert(id.clone());
        }
        Node::Option(inner) | Node::Seq(inner) => refs_in_node(inner, out),
        Node::Tuple(items) => items.iter().for_each(|n| refs_in_node(n, out)),
        Node::Map { key, value } => {
            refs_in_node(key, out);
            refs_in_node(value, out);
        }
        _ => {}
    }
}

fn refs_in_def(def: &Def, out: &mut BTreeSet<TypeId>) {
    match def {
        Def::UnitStruct => {}
        Def::NewtypeStruct(n) => refs_in_node(n, out),
        Def::TupleStruct(ns) => ns.iter().for_each(|n| refs_in_node(n, out)),
        Def::Struct(fields) => fields.iter().for_each(|(_, n)| refs_in_node(n, out)),
        Def::Enum(variants) => {
            for v in variants {
                match v {
                    Variant::Unit(_) => {}
                    Variant::Newtype(_, n) => refs_in_node(n, out),
                    Variant::Tuple(_, ns) => ns.iter().for_each(|n| refs_in_node(n, out)),
                    Variant::Struct(_, fields) => {
                        fields.iter().for_each(|(_, n)| refs_in_node(n, out))
                    }
                }
            }
        }
    }
}

/// Order the definitions so every type appears after everything it references.
///
/// The registry already hands them over in a valid order (post-order DFS), but that
/// order depends on which root happened to reach a type first. Re-sorting with Kahn's
/// algorithm and an alphabetical tie-break makes the output depend only on the *set* of
/// types, so adding a root does not reshuffle hundreds of unrelated lines.
fn topo_sort(defs: Vec<(TypeId, Def)>, names: &Names) -> Vec<(TypeId, Def)> {
    let mut deps: BTreeMap<TypeId, BTreeSet<TypeId>> = BTreeMap::new();
    let present: BTreeSet<TypeId> = defs.iter().map(|(id, _)| id.clone()).collect();

    for (id, def) in &defs {
        let mut refs = BTreeSet::new();
        refs_in_def(def, &mut refs);
        refs.remove(id); // self-reference cannot happen, but do not deadlock on it
        refs.retain(|r| present.contains(r));
        deps.insert(id.clone(), refs);
    }

    let mut by_id: BTreeMap<TypeId, Def> = defs.into_iter().collect();
    let mut emitted: BTreeSet<TypeId> = BTreeSet::new();
    let mut out = Vec::new();

    while !by_id.is_empty() {
        // Everything whose dependencies are all already emitted, alphabetically.
        let mut ready: Vec<TypeId> = by_id
            .keys()
            .filter(|id| deps[*id].iter().all(|d| emitted.contains(d)))
            .cloned()
            .collect();

        assert!(
            !ready.is_empty(),
            "dependency cycle among: {:?}",
            by_id.keys().map(|id| names.get(id)).collect::<Vec<_>>()
        );

        ready.sort_by(|a, b| names.get(a).cmp(names.get(b)));

        for id in ready {
            let def = by_id.remove(&id).expect("ready ids come from by_id");
            emitted.insert(id.clone());
            out.push((id, def));
        }
    }

    out
}

/// Combinators used by the emitted source, so the import list can be exact.
///
/// `tsconfig.json` sets `noUnusedLocals`, which makes an unused import a *tsc error*
/// rather than a lint warning -- importing the whole vocabulary unconditionally would
/// not compile.
#[derive(Default)]
struct Used(BTreeSet<&'static str>);

impl Used {
    fn mark(&mut self, name: &'static str) -> &'static str {
        self.0.insert(name);
        name
    }
}

fn node_ts(node: &Node, names: &Names, used: &mut Used) -> String {
    match node {
        Node::Bool => format!("{}()", used.mark("bool")),
        Node::I8 => format!("{}()", used.mark("i8")),
        Node::I16 => format!("{}()", used.mark("i16")),
        Node::I32 => format!("{}()", used.mark("i32")),
        Node::I64 => format!("{}()", used.mark("i64")),
        Node::I128 => format!("{}()", used.mark("i128")),
        Node::U8 => format!("{}()", used.mark("u8")),
        Node::U16 => format!("{}()", used.mark("u16")),
        Node::U32 => format!("{}()", used.mark("u32")),
        Node::U64 => format!("{}()", used.mark("u64")),
        Node::U128 => format!("{}()", used.mark("u128")),
        Node::F32 => format!("{}()", used.mark("f32")),
        Node::F64 => format!("{}()", used.mark("f64")),
        Node::Char => format!("{}()", used.mark("char")),
        Node::Str => format!("{}()", used.mark("string")),
        Node::Bytes => format!("{}()", used.mark("bytes")),
        Node::Unit => format!("{}()", used.mark("unit")),

        // Refused rather than guessed. serde maps `usize` to u64 regardless of target
        // width, but the producer here is 32-bit, so u32 and u64 varints are
        // byte-identical for every value it can emit -- meaning the round-trip harness
        // could never tell a wrong choice from a right one. The width has to be pinned
        // in the Rust type instead. See the u32 conversions in variegated-controller-types.
        Node::Usize | Node::Isize => panic!(
            "`usize`/`isize` reached a wire type. Neither can be emitted safely: serde \
             encodes them as u64, but on a 32-bit producer that is indistinguishable \
             from u32 on the wire, so no test could catch the wrong choice. Declare the \
             field as an explicit width (u32) in the Rust type."
        ),

        Node::Option(inner) => {
            let i = node_ts(inner, names, used);
            format!("{}({})", used.mark("option"), i)
        }
        Node::Seq(inner) => {
            let i = node_ts(inner, names, used);
            format!("{}({})", used.mark("seq"), i)
        }
        Node::Tuple(items) => {
            let parts: Vec<String> = items.iter().map(|n| node_ts(n, names, used)).collect();
            format!("{}({})", used.mark("tuple"), parts.join(", "))
        }
        Node::Map { key, value } => {
            let k = node_ts(key, names, used);
            let v = node_ts(value, names, used);
            format!("{}({}, {})", used.mark("map"), k, v)
        }
        Node::Ref(id) => format!("{}Schema", names.get(id)),
    }
}

fn fields_ts(
    fields: &[(&'static str, Node)],
    names: &Names,
    used: &mut Used,
    indent: &str,
) -> String {
    let inner = format!("{indent}  ");
    let body: Vec<String> = fields
        .iter()
        .map(|(name, node)| format!("{inner}{name}: {}", node_ts(node, names, used)))
        .collect();
    format!("{{\n{}\n{indent}}}", body.join(",\n"))
}

fn variant_ts(v: &Variant, names: &Names, used: &mut Used, indent: &str) -> String {
    match v {
        Variant::Unit(n) => format!("{}('{n}')", used.mark("unitVariant")),
        Variant::Newtype(n, node) => {
            let inner = node_ts(node, names, used);
            format!("{}('{n}', {inner})", used.mark("newtypeVariant"))
        }
        Variant::Tuple(n, nodes) => {
            let parts: Vec<String> = nodes.iter().map(|x| node_ts(x, names, used)).collect();
            format!("{}('{n}', {})", used.mark("tupleVariant"), parts.join(", "))
        }
        Variant::Struct(n, fields) => {
            let body = fields_ts(fields, names, used, indent);
            format!("{}('{n}', {body})", used.mark("structVariant"))
        }
    }
}

fn def_ts(name: &str, def: &Def, names: &Names, used: &mut Used) -> String {
    match def {
        Def::UnitStruct => format!("{}('{name}')", used.mark("unitStruct")),
        Def::NewtypeStruct(node) => {
            let inner = node_ts(node, names, used);
            format!("{}('{name}', {inner})", used.mark("newtypeStruct"))
        }
        Def::TupleStruct(nodes) => {
            let parts: Vec<String> = nodes.iter().map(|n| node_ts(n, names, used)).collect();
            format!("{}('{name}', {})", used.mark("tupleStruct"), parts.join(", "))
        }
        Def::Struct(fields) => {
            let body = fields_ts(fields, names, used, "");
            format!("{}({body})", used.mark("struct"))
        }
        Def::Enum(variants) => {
            // Variant order is the wire format: the index of a key in this object is
            // the varint postcard writes. Nothing may sort these.
            let body: Vec<String> = variants
                .iter()
                .map(|v| format!("  {}: {}", v.name(), variant_ts(v, names, used, "  ")))
                .collect();
            format!(
                "{}('{name}', {{\n{}\n}})",
                used.mark("enumType"),
                body.join(",\n")
            )
        }
    }
}

const HEADER: &str = "\
// @generated by variegated-schema-export -- DO NOT EDIT.
//
// Runtime postcard schema descriptors, generated from the Rust types so they cannot
// drift from them. Field order and enum variant order ARE the wire format: postcard is
// non-self-describing, so a field out of order silently mis-decodes everything after
// it. That is why this file is no longer written by hand.
//
// Source of truth:
//   variegated-rs/variegated-controller-types      (Status, Configuration, ...)
//   variegated-rs/variegated-control-algorithm     (PidParameters, PidTerm, Limits)
//   crates/variegated-comms-api-types              (WsMessage, RoutineStorage, Set*Request)
//
// To regenerate:  scripts/generate-schemas.sh
// It also runs from variegated-comms-firmware's build.rs, so a normal firmware build
// keeps this file current.
//
// `usize` is deliberately absent. serde encodes it as u64 regardless of target width,
// but this producer is 32-bit, where u32 and u64 varints are identical for every value
// it can emit -- so a round-trip test could not tell a wrong choice from a right one.
// The generator refuses `usize` outright; wire-facing indices are declared u32 in Rust.
//
// Hand-written additions belong in epilogue.ts.in, which is appended below verbatim.
";

/// Generate the full contents of `schemas.ts`.
pub fn generate(reg: Registry, epilogue: &str) -> String {
    let roots: Vec<TypeId> = reg.roots().to_vec();
    let defs = reg.into_definitions();
    let names = resolve_names(&defs);
    let ordered = topo_sort(defs, &names);

    let mut used = Used::default();
    let mut body = String::new();
    for (id, def) in &ordered {
        let name = names.get(id);
        let rhs = def_ts(name, def, &names, &mut used);
        let _ = write!(body, "export const {name}Schema = {rhs};\n\n");
    }

    // Type aliases are hoisted, so their order is free -- roots first (the names a
    // reader is looking for), then everything else alphabetically, which keeps diffs
    // small when a type is added.
    let mut alias_names: Vec<&str> = Vec::new();
    for r in &roots {
        alias_names.push(names.get(r));
    }
    let mut rest: Vec<&str> = ordered
        .iter()
        .map(|(id, _)| names.get(id))
        .filter(|n| !alias_names.contains(n))
        .collect();
    rest.sort_unstable();
    alias_names.extend(rest);

    let mut aliases = String::new();
    for name in &alias_names {
        let _ = writeln!(
            aliases,
            "export type {name} = {}<typeof {name}Schema>;",
            "InferType"
        );
    }

    used.mark("InferType");
    let mut imports: Vec<&str> = used.0.iter().copied().collect();
    imports.sort_unstable();

    let mut out = String::new();
    out.push_str(HEADER);
    out.push('\n');
    let _ = writeln!(out, "import {{");
    for (i, name) in imports.iter().enumerate() {
        let comma = if i + 1 == imports.len() { "" } else { "," };
        let _ = writeln!(out, "  {name}{comma}");
    }
    let _ = writeln!(out, "}} from '@variegated-coffee/serde-postcard-ts';");
    out.push('\n');
    out.push_str(&body);
    out.push_str(&aliases);

    let epilogue = epilogue.trim_end();
    if !epilogue.is_empty() {
        out.push('\n');
        out.push_str(epilogue);
        out.push('\n');
    }

    validate_epilogue(epilogue, &alias_names);
    out
}

/// A stale epilogue should fail here, not as a confusing `tsc` error later.
fn validate_epilogue(epilogue: &str, known: &[&str]) {
    for line in epilogue.lines() {
        let line = line.trim();
        let Some(rest) = line.strip_prefix("export type ") else {
            continue;
        };
        let Some((_, rhs)) = rest.split_once('=') else {
            continue;
        };
        for word in rhs.split(|c: char| !c.is_ascii_alphanumeric() && c != '_') {
            if word.is_empty() || !word.starts_with(char::is_uppercase) {
                continue;
            }
            assert!(
                known.contains(&word),
                "epilogue.ts.in references `{word}`, which the generator does not emit. \
                 It was probably renamed or removed -- update the epilogue."
            );
        }
    }
}
