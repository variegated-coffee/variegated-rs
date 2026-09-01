use trouble_host::prelude::*;

/// Service UUID served by ACAIA's 2021-and-later scales:
/// `49535343-fe7d-4ae5-8fa9-9fafd205e455`.
///
/// This is the Microchip/ISSC "transparent UART" service, which ACAIA reuses rather than
/// defining one of their own.
///
/// # The byte order is reversed, and it is not obvious
///
/// **These are the first 128-bit UUIDs in this tree**, so nothing here has exercised this
/// before. `trouble_host::Uuid::new_long` stores the array verbatim and copies it straight
/// into GATT PDUs, which are little-endian — as its sibling `new_short(v)`, defined as
/// `Uuid16(v.to_le_bytes())`, makes explicit. It is also the order in which advertising
/// payloads hand out 128-bit UUIDs, which is what the scanner compares against.
///
/// So each constant below is the canonical UUID written **least-significant byte first**.
/// Written the other way round it matches nothing and fails as `Error::ServiceNotFound`,
/// mentioning neither UUIDs nor byte order — which is why this is the first thing to suspect
/// if a scale connects and discovery then fails.
pub const ACAIA_NEW_SERVICE_UUID: Uuid = Uuid::new_long([
    0x55, 0xe4, 0x05, 0xd2, 0xaf, 0x9f, 0xa9, 0x8f, 0xe5, 0x4a, 0x7d, 0xfe, 0x43, 0x53, 0x53,
    0x49,
]);

/// Characteristic carrying notifications: `49535343-1e4d-4bd9-ba61-23c647249616`.
///
/// Notify only. Distinct from [`ACAIA_NEW_WRITE_CHAR_UUID`] — which is the whole structural
/// difference from the pre-2021 protocol, where `0x2a80` served both directions.
pub const ACAIA_NEW_NOTIFY_CHAR_UUID: Uuid = Uuid::new_long([
    0x16, 0x96, 0x24, 0x47, 0xc6, 0x23, 0x61, 0xba, 0xd9, 0x4b, 0x4d, 0x1e, 0x43, 0x53, 0x53,
    0x49,
]);

/// Characteristic that accepts commands: `49535343-8841-43f4-a8d4-ecbe34729bb3`.
///
/// Write only — it carries no notify property. Written without response, which is what the
/// majority of implementations do and what the pre-2021 driver already does.
pub const ACAIA_NEW_WRITE_CHAR_UUID: Uuid = Uuid::new_long([
    0xb3, 0x9b, 0x72, 0x34, 0xbe, 0xec, 0xd4, 0xa8, 0xf4, 0x43, 0x41, 0x88, 0x43, 0x53, 0x53,
    0x49,
]);

/// Events a 2021+ ACAIA scale reports.
///
/// The codec's frame type verbatim rather than a wrapper around it: unlike the BooKoo
/// driver, whose `ScaleEvent` renames variants that mean something different in its
/// protocol, there is nothing here to translate. A wrapper would relocate the definition
/// without changing it.
pub use variegated_scale_codec::acaia::Frame as ScaleEvent;

/// Name-based recognition, re-exported so a caller has one place to look for everything
/// that identifies these scales.
///
/// It lives in the codec because the ordering rule it implements — `LUNAR-` beating a bare
/// `LUNAR` — is worth a test, and this crate cannot host one.
pub use variegated_scale_codec::acaia::{acaia_generation_from_name, MODERN_NAME_PREFIXES};
