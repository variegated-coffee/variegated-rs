// An internally tagged enum is encoded as a map with a tag field, not as a variant
// index. Every variant's bytes differ from what this crate would describe.
use variegated_postcard_schema::PostcardSchema;

#[derive(PostcardSchema, serde::Serialize, serde::Deserialize)]
#[serde(tag = "kind")]
enum E {
    A { x: u8 },
    B { y: u8 },
}

fn main() {}
