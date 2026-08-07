// serde ignores the discriminant and encodes by declaration order, so `Third = 5`
// goes on the wire as 2. Someone writing this almost certainly believes otherwise.
use variegated_postcard_schema::PostcardSchema;

#[derive(PostcardSchema, serde::Serialize, serde::Deserialize)]
#[repr(u8)]
enum E {
    First = 0,
    Second = 1,
    Third = 5,
}

fn main() {}
