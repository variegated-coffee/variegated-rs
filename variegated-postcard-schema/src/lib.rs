#![no_std]

//! A description of a Rust type's *serde data model*, recovered at compile time.
//!
//! This exists so `variegated-comms-rs`'s `frontend/src/schemas/schemas.ts` can be
//! generated rather than hand-maintained. That file holds runtime postcard schema
//! descriptors, and postcard is a non-self-describing format: the decoder is told
//! nothing by the bytes, so a field in the wrong order or a variant with the wrong
//! index silently mis-decodes everything after it. Hand-maintaining that mapping did
//! not work -- see the drift the generator's first run fixes.
//!
//! # Why a local trait rather than `postcard-schema`
//!
//! The orphan rule. These types reach `chrono::NaiveDateTime`, `heapless::Vec`,
//! `core::time::Duration` and `variegated_control_algorithm::pid::PidParameters<f32>`,
//! all foreign. A foreign trait could not be implemented for them here. A local trait
//! can be implemented for anything.
//!
//! # Why resolution happens in the type system
//!
//! The alternative -- parsing the Rust source with `syn` -- would have to re-implement
//! type alias resolution (`TemperatureType`, `BoilerIndex`, `RoutineParameters`), const
//! generic resolution (`MAX_BOILERS`, and one bare `4`), and cross-crate lookup. Going
//! through trait resolution instead means all three are the compiler's problem, and a
//! type alias is transparent by construction rather than by our getting it right.
//!
//! # What this deliberately does not model
//!
//! Serde attributes. There are none on the types this describes, and the derive
//! rejects any it sees rather than ignoring them, so this stays true. Renames, tags,
//! `flatten`, `skip` and `default` all change the data model, and silently producing a
//! schema that ignores them would be worse than refusing.

extern crate alloc;

use alloc::boxed::Box;
use alloc::collections::BTreeSet;
use alloc::vec::Vec;

pub use variegated_postcard_schema_derive::PostcardSchema;

mod impls;

/// Support for the generated code. Not a stable API.
///
/// The derive expands at the call site, where `alloc` may not be in scope under any
/// particular path -- `extern crate alloc` is per-crate and nothing obliges a consumer
/// to have written it. Routing the one allocation the expansion needs through here
/// keeps the generated code independent of that.
#[doc(hidden)]
pub mod __private {
    pub use alloc::vec::Vec;

    pub fn vec_from<T, const N: usize>(items: [T; N]) -> Vec<T> {
        Vec::from(items)
    }
}

/// Identity of a named type.
///
/// `module` and `ident` come from the definition site. `args` carries
/// `core::any::type_name` of each generic *type* argument -- lifetimes and const
/// parameters are excluded, since neither can affect the serialized shape. It exists
/// so `PidParameters<f32>` and a hypothetical `PidParameters<f64>` are distinguishable
/// in one output; without it the registry would emit whichever monomorphization it saw
/// first and silently drop the other.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TypeId {
    pub module: &'static str,
    pub ident: &'static str,
    pub args: Vec<&'static str>,
}

/// A node in the serde data model.
///
/// Every variant corresponds to a serde `Serializer` method, not to a Rust construct.
/// The distinction matters in the cases where they disagree -- see `impls`, where fixed
/// arrays become `Tuple` (serde routes them through `serialize_tuple`, so there is no
/// length prefix) and `Vec<u8>` becomes `Seq` rather than `Bytes`.
#[derive(Clone, Debug, PartialEq)]
pub enum Node {
    Bool,
    I8,
    I16,
    I32,
    I64,
    I128,
    U8,
    U16,
    U32,
    U64,
    U128,
    /// `usize`/`isize`. serde maps these to u64/i64 regardless of target width.
    ///
    /// Kept distinct rather than folded into `U64`/`I64` so a consumer can decide what
    /// to do about them. The emitter in this tree refuses them outright: on a 32-bit
    /// producer u32 and u64 varints are byte-identical for every representable value,
    /// which means a round-trip test can never tell you which one you meant, so the
    /// width has to be pinned in the Rust type instead of guessed here.
    Usize,
    Isize,
    F32,
    F64,
    Char,
    Str,
    Bytes,
    Unit,
    Option(Box<Node>),
    Seq(Box<Node>),
    Tuple(Vec<Node>),
    Map { key: Box<Node>, value: Box<Node> },
    /// A reference to a named type defined elsewhere in the `Registry`.
    Ref(TypeId),
}

/// The definition of a named type.
#[derive(Clone, Debug, PartialEq)]
pub enum Def {
    UnitStruct,
    NewtypeStruct(Node),
    TupleStruct(Vec<Node>),
    /// Fields in declaration order. The order *is* the wire format.
    Struct(Vec<(&'static str, Node)>),
    /// Variants in declaration order. The index of a variant in this list is the
    /// varint postcard writes for it, which is why nothing may sort this.
    Enum(Vec<Variant>),
}

#[derive(Clone, Debug, PartialEq)]
pub enum Variant {
    Unit(&'static str),
    Newtype(&'static str, Node),
    Tuple(&'static str, Vec<Node>),
    Struct(&'static str, Vec<(&'static str, Node)>),
}

impl Variant {
    pub fn name(&self) -> &'static str {
        match self {
            Variant::Unit(n)
            | Variant::Newtype(n, _)
            | Variant::Tuple(n, _)
            | Variant::Struct(n, _) => n,
        }
    }
}

/// Describes a type as a node in the serde data model.
///
/// Implemented by `#[derive(PostcardSchema)]` for structs and enums, and by hand in
/// `impls` for primitives, containers and foreign types.
///
/// Hand-written impls are safe only for types with no field *names* to get wrong.
/// Postcard does not encode field names, so a hand-written impl for a struct could name
/// its fields incorrectly, compile, and round-trip byte-identically forever -- invisible
/// to every check. Anything with named fields must be derived at its definition site,
/// which is why `variegated-control-algorithm` grows a `schema` feature rather than
/// having its PID types described from here.
pub trait PostcardSchema {
    /// `Some` for types that get their own definition in the output; `None` for
    /// primitives and containers, which are written inline at each use.
    fn type_id() -> Option<TypeId> {
        None
    }

    /// Add this type, and everything reachable from it, to the registry.
    fn register(_reg: &mut Registry) {}

    /// This type as it appears inline at a use site.
    fn node(reg: &mut Registry) -> Node;
}

/// Collects type definitions, in an order the output can be emitted in.
///
/// Registration is split into `begin`/`finish` rather than being one call. That split
/// is what makes cycles detectable and ordering correct:
///
/// - `begin` marks a type as in-progress *before* its fields are walked, so a type
///   reached again while it is still being defined is recognizable as a back edge.
/// - definitions are recorded when they `finish`, which is post-order: every type is
///   recorded after everything it references. The output is a list of `const`s
///   evaluated top-down, so children must come first or the module throws at load.
///
/// Everything here is ordered (`Vec`, `BTreeSet`) rather than hashed, so the output is
/// byte-identical across runs.
#[derive(Default)]
pub struct Registry {
    known: BTreeSet<TypeId>,
    in_progress: Vec<TypeId>,
    order: Vec<(TypeId, Def)>,
    roots: Vec<TypeId>,
}

impl Registry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Begin defining `id`. Returns `false` if it is already known, in which case the
    /// caller must not walk its fields again.
    pub fn begin(&mut self, id: TypeId) -> bool {
        if self.known.contains(&id) {
            return false;
        }
        self.known.insert(id.clone());
        self.in_progress.push(id);
        true
    }

    /// Record the finished definition of `id`.
    pub fn finish(&mut self, id: TypeId, def: Def) {
        let popped = self.in_progress.pop();
        debug_assert_eq!(
            popped.as_ref(),
            Some(&id),
            "PostcardSchema::register must finish the type it began"
        );
        self.order.push((id, def));
    }

    /// A reference to `id` at a use site.
    ///
    /// Panics on a back edge. The target vocabulary (`@variegated-coffee/serde-postcard-ts`)
    /// has no `lazy`/`ref`/thunk combinator -- its schemas are eager values -- so a
    /// recursive type is not merely awkward to emit, it is inexpressible. Emitting a
    /// `const` that references itself would produce a temporal-dead-zone `ReferenceError`
    /// at module load: a blank page with no useful stack. Failing here instead names the
    /// cycle.
    pub fn reference(&self, id: TypeId) -> Node {
        if let Some(at) = self.in_progress.iter().position(|p| *p == id) {
            let mut path = alloc::string::String::new();
            for step in &self.in_progress[at..] {
                path.push_str(step.ident);
                path.push_str(" -> ");
            }
            path.push_str(id.ident);
            panic!(
                "recursive type: {path}\n\
                 The TypeScript schema vocabulary has no lazy combinator, so a cycle \
                 cannot be represented. Break it (e.g. replace the back edge with an \
                 index) before putting the type on the wire."
            );
        }
        Node::Ref(id)
    }

    /// Register `T` and remember it as a root of the output.
    pub fn root<T: PostcardSchema>(&mut self) {
        T::register(self);
        if let Some(id) = T::type_id() {
            if !self.roots.contains(&id) {
                self.roots.push(id);
            }
        }
    }

    pub fn roots(&self) -> &[TypeId] {
        &self.roots
    }

    /// Definitions in dependency order: every type appears after everything it
    /// references.
    pub fn definitions(&self) -> &[(TypeId, Def)] {
        &self.order
    }

    pub fn into_definitions(self) -> Vec<(TypeId, Def)> {
        self.order
    }
}
