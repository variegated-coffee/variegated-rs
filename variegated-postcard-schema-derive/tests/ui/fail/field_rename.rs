// `rename` changes the field name in the data model. Postcard does not encode field
// names, so a schema generated in ignorance of this would round-trip byte-identically
// while describing a field that does not exist under that name.
use variegated_postcard_schema::PostcardSchema;

#[derive(PostcardSchema, serde::Serialize, serde::Deserialize)]
struct S {
    #[serde(rename = "temperature")]
    temp: f32,
}

fn main() {}
