// The shapes that must keep working: a lifetime, `#[serde(borrow)]` (the one serde
// attribute with no effect on the encoding), all four variant shapes, a generic, and a
// discriminant that agrees with its index.
use variegated_postcard_schema::PostcardSchema;

#[derive(PostcardSchema, serde::Serialize, serde::Deserialize)]
enum Msg<'a> {
    Unit,
    Newtype(u8),
    Tuple(u8, bool),
    Ack {
        id: u32,
        #[serde(borrow)]
        error: Option<&'a str>,
    },
}

#[derive(PostcardSchema, serde::Serialize, serde::Deserialize)]
struct Pair<T> {
    lo: T,
    hi: T,
}

#[derive(PostcardSchema, serde::Serialize, serde::Deserialize)]
#[repr(u8)]
enum Ordered {
    A = 0,
    B = 1,
}

#[derive(PostcardSchema, serde::Serialize, serde::Deserialize)]
struct Newtype(u8);

#[derive(PostcardSchema, serde::Serialize, serde::Deserialize)]
struct Unit;

fn main() {
    fn assert_impl<T: PostcardSchema>() {}
    assert_impl::<Msg<'static>>();
    assert_impl::<Pair<f32>>();
    assert_impl::<Ordered>();
    assert_impl::<Newtype>();
    assert_impl::<Unit>();
}
