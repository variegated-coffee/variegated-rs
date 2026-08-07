//! Tests for the mappings where the serde data model and the Rust type disagree.
//!
//! The obvious ones (`u8` is `U8`) are not tested. What is tested is every place a
//! reasonable person would guess wrong, and in those cases the assertion is made
//! against postcard's actual output rather than against the reasoning that produced the
//! impl -- the hand-written schema this crate replaces was wrong precisely because it
//! was written from reasoning.

use variegated_postcard_schema::{Def, Node, PostcardSchema, Registry, Variant};

fn node_of<T: PostcardSchema>() -> Node {
    let mut reg = Registry::new();
    T::node(&mut reg)
}

fn defs_of<T: PostcardSchema>() -> Vec<(String, Def)> {
    let mut reg = Registry::new();
    reg.root::<T>();
    reg.into_definitions().into_iter().map(|(id, def)| (id.ident.to_string(), def)).collect()
}

/// A fixed array is a tuple, so it carries no length prefix.
///
/// This is the mapping with the worst failure mode: treating `[u8; 6]` as a sequence
/// adds one varint byte and shifts every subsequent field, corrupting the rest of the
/// frame rather than just that value.
#[test]
fn fixed_array_is_a_tuple_with_no_length_prefix() {
    assert_eq!(node_of::<[u8; 6]>(), Node::Tuple(vec![Node::U8; 6]));

    let array_bytes = postcard::to_allocvec(&[1u8, 2, 3, 4, 5, 6]).unwrap();
    let vec_bytes = postcard::to_allocvec(&vec![1u8, 2, 3, 4, 5, 6]).unwrap();

    assert_eq!(array_bytes.len(), 6, "an array must not be length-prefixed");
    assert_eq!(vec_bytes.len(), 7, "a Vec must be length-prefixed");
    assert_eq!(&vec_bytes[1..], &array_bytes[..]);
}

/// `Vec<u8>` is a sequence of u8, not a byte string.
///
/// Postcard encodes both identically, so this is invisible on the wire -- but it
/// decides whether the generated TypeScript says `number[]` or `Uint8Array`.
#[test]
fn vec_of_u8_is_a_seq_not_bytes() {
    assert_eq!(node_of::<Vec<u8>>(), Node::Seq(Box::new(Node::U8)));
}

/// `Duration` is a two-field struct, and a named one so it is defined once.
#[test]
fn duration_is_a_named_two_field_struct() {
    let defs = defs_of::<core::time::Duration>();
    assert_eq!(defs.len(), 1);
    assert_eq!(defs[0].0, "Duration");
    assert_eq!(
        defs[0].1,
        Def::Struct(vec![("secs", Node::U64), ("nanos", Node::U32)])
    );

    // 2 s exactly: secs=2 as a varint, nanos=0 as a varint. Two bytes, in that order.
    let bytes = postcard::to_allocvec(&core::time::Duration::from_secs(2)).unwrap();
    assert_eq!(bytes, vec![2, 0]);
}

/// `usize` stays distinguishable from `u64` rather than being silently folded in.
#[test]
fn usize_is_its_own_node() {
    assert_eq!(node_of::<usize>(), Node::Usize);
}

#[test]
fn option_and_nesting() {
    assert_eq!(
        node_of::<Option<(f32, f32)>>(),
        Node::Option(Box::new(Node::Tuple(vec![Node::F32, Node::F32])))
    );
}

/// `Box<T>` is transparent: it must not introduce a wrapper node, and it must forward
/// the identity so `Box<Status>` and `Status` are the same definition.
#[test]
fn box_is_transparent() {
    #[derive(PostcardSchema)]
    struct Inner {
        a: u8,
    }

    assert_eq!(node_of::<Box<Inner>>(), node_of::<Inner>());
    assert_eq!(defs_of::<Box<Inner>>().len(), 1);
}

#[cfg(feature = "heapless")]
mod heapless_mappings {
    use super::*;

    #[test]
    fn index_map_is_a_map() {
        assert_eq!(
            node_of::<heapless::index_map::FnvIndexMap<u8, u16, 8>>(),
            Node::Map { key: Box::new(Node::U8), value: Box::new(Node::U16) }
        );
    }

    /// A set serializes as a sequence. There is no set in the data model, and the
    /// TypeScript vocabulary has no set combinator either.
    #[test]
    fn index_set_is_a_seq() {
        assert_eq!(
            node_of::<heapless::index_set::FnvIndexSet<u8, 8>>(),
            Node::Seq(Box::new(Node::U8))
        );
    }

    /// Capacity is a Rust-side bound and never reaches the wire.
    #[test]
    fn heapless_string_is_a_str_regardless_of_capacity() {
        assert_eq!(node_of::<heapless::String<32>>(), Node::Str);
        assert_eq!(node_of::<heapless::String<96>>(), Node::Str);
    }

    #[test]
    fn heapless_vec_is_a_seq() {
        assert_eq!(node_of::<heapless::Vec<u8, 8>>(), Node::Seq(Box::new(Node::U8)));
    }
}

#[cfg(feature = "chrono")]
mod chrono_mappings {
    use super::*;

    /// chrono types are strings even in a binary format -- they serialize via
    /// `collect_str` with no `is_human_readable` branch. The natural assumption
    /// (binary format, therefore a compact numeric timestamp) is wrong.
    #[test]
    fn chrono_types_are_strings() {
        assert_eq!(node_of::<chrono::NaiveDateTime>(), Node::Str);
        assert_eq!(node_of::<chrono::NaiveDate>(), Node::Str);
        assert_eq!(node_of::<chrono::Weekday>(), Node::Str);
        assert_eq!(node_of::<chrono::DateTime<chrono::Utc>>(), Node::Str);
    }

    #[test]
    fn weekday_really_encodes_as_a_string() {
        let bytes = postcard::to_allocvec(&chrono::Weekday::Mon).unwrap();
        // varint length 3, then "Mon".
        assert_eq!(bytes, b"\x03Mon".to_vec());
    }
}

mod derive_behaviour {
    use super::*;

    /// Field order is the wire format, so it must survive verbatim.
    #[test]
    fn struct_fields_keep_declaration_order() {
        #[derive(PostcardSchema)]
        #[allow(dead_code)]
        struct S {
            zebra: u8,
            apple: u16,
            mango: bool,
        }

        let defs = defs_of::<S>();
        assert_eq!(
            defs[0].1,
            Def::Struct(vec![("zebra", Node::U8), ("apple", Node::U16), ("mango", Node::Bool)]),
            "fields must not be sorted"
        );
    }

    /// All four variant shapes, in order. A one-field tuple variant is a *newtype*
    /// variant in serde and encodes with no wrapper, which is the distinction most
    /// easily lost.
    #[test]
    fn all_variant_shapes() {
        #[derive(PostcardSchema)]
        #[allow(dead_code)]
        enum E {
            Nothing,
            One(u8),
            Two(u8, bool),
            Named { x: f32 },
        }

        let defs = defs_of::<E>();
        assert_eq!(
            defs[0].1,
            Def::Enum(vec![
                Variant::Unit("Nothing"),
                Variant::Newtype("One", Node::U8),
                Variant::Tuple("Two", vec![Node::U8, Node::Bool]),
                Variant::Struct("Named", vec![("x", Node::F32)]),
            ])
        );
    }

    /// Children are defined before parents, because the output is a list of `const`s
    /// evaluated top-down.
    #[test]
    fn definitions_are_leaves_first() {
        #[derive(PostcardSchema)]
        #[allow(dead_code)]
        struct Leaf {
            a: u8,
        }
        #[derive(PostcardSchema)]
        #[allow(dead_code)]
        struct Branch {
            leaf: Leaf,
        }
        #[derive(PostcardSchema)]
        #[allow(dead_code)]
        struct Root {
            branch: Branch,
        }

        let names: Vec<String> = defs_of::<Root>().into_iter().map(|(n, _)| n).collect();
        assert_eq!(names, vec!["Leaf", "Branch", "Root"]);
    }

    /// A type reached by two paths is defined once and referenced twice.
    #[test]
    fn shared_type_is_defined_once() {
        #[derive(PostcardSchema)]
        #[allow(dead_code)]
        struct Shared {
            a: u8,
        }
        #[derive(PostcardSchema)]
        #[allow(dead_code)]
        struct Holder {
            left: Shared,
            right: Shared,
        }

        let defs = defs_of::<Holder>();
        assert_eq!(defs.len(), 2, "Shared must appear once, not once per use");
        assert_eq!(defs[0].0, "Shared");
    }

    /// Two monomorphizations of one generic type must not collide.
    #[test]
    fn generic_instantiations_are_distinct() {
        #[derive(PostcardSchema)]
        #[allow(dead_code)]
        struct Pair<T> {
            lo: T,
            hi: T,
        }
        #[derive(PostcardSchema)]
        #[allow(dead_code)]
        struct Both {
            floats: Pair<f32>,
            bytes: Pair<u8>,
        }

        let defs = defs_of::<Both>();
        assert_eq!(defs.len(), 3, "each instantiation needs its own definition");
        let pairs: Vec<&Def> =
            defs.iter().filter(|(n, _)| n == "Pair").map(|(_, d)| d).collect();
        assert_eq!(pairs.len(), 2);
        assert_ne!(pairs[0], pairs[1]);
    }

    /// A lifetime must not prevent the derive from applying, nor leak into identity.
    /// `#[serde(borrow)]` is the one serde attribute that is allowed through.
    #[test]
    fn lifetimes_and_borrow_are_accepted() {
        #[derive(PostcardSchema, serde::Serialize, serde::Deserialize)]
        #[allow(dead_code)]
        enum Msg<'a> {
            Ack {
                id: u32,
                #[serde(borrow)]
                error: Option<&'a str>,
            },
        }

        let defs = defs_of::<Msg<'static>>();
        assert_eq!(
            defs[0].1,
            Def::Enum(vec![Variant::Struct(
                "Ack",
                vec![("id", Node::U32), ("error", Node::Option(Box::new(Node::Str)))]
            )])
        );
    }

    /// A discriminant equal to its index is harmless and accepted.
    #[test]
    fn matching_discriminants_are_accepted() {
        #[derive(PostcardSchema)]
        #[allow(dead_code)]
        #[repr(u8)]
        enum E {
            A = 0,
            B = 1,
        }

        let defs = defs_of::<E>();
        assert_eq!(defs[0].1, Def::Enum(vec![Variant::Unit("A"), Variant::Unit("B")]));
    }
}
