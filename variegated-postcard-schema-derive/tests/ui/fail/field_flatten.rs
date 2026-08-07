// `flatten` splices the inner type's fields into this one, changing both the field
// count and the order. It is the attribute most likely to silently corrupt a schema.
use variegated_postcard_schema::PostcardSchema;

#[derive(PostcardSchema, serde::Serialize, serde::Deserialize)]
struct Inner {
    a: u8,
}

#[derive(PostcardSchema, serde::Serialize, serde::Deserialize)]
struct S {
    #[serde(flatten)]
    inner: Inner,
    b: u8,
}

fn main() {}
