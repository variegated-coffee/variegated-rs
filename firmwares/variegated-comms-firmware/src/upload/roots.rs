//! Trust anchors for shot-log upload.
//!
//! # One concatenated PEM, not a list of DERs
//!
//! `mbedtls-rs` accepts exactly one [`Certificate`] for `ClientSessionConfig::ca_chain`
//! and offers no way to chain two together. `Certificate::new_no_copy` takes a single DER;
//! only `Certificate::new(X509::PEM(..))` reaches `mbedtls_x509_crt_parse`, which walks a
//! concatenated PEM bundle and builds the whole linked list. So a multi-root store has to
//! be PEM, and the `pem-parse` and `base64` features have to stay in the curated set.
//!
//! The trailing NUL is inside the counted length -- MbedTLS requires it for PEM input.
//! [`CStr`] gives that for free and refuses at compile time if an interior NUL creeps in.
//!
//! # Which roots, and why this set is provisional
//!
//! **These must be confirmed against the endpoint actually in use**, and re-confirmed if
//! the hosting changes:
//!
//! ```sh
//! openssl s_client -connect <endpoint-host>:443 -showcerts </dev/null
//! ```
//!
//! Take the **root** the chain terminates at, not the intermediate -- an intermediate
//! rotates far more often, and pinning one turns a routine renewal into a machine that
//! silently stops uploading.
//!
//! Deliberately *not* a full Mozilla bundle. That is ~140 certificates: ~200 kB of flash,
//! and worse, every one of them is parsed into heap at handshake time. This firmware talks
//! to exactly one host, so the trust store should be the roots that host actually uses.
//!
//! Each root costs ~1.9 kB of `.rodata` here and ~1.5 kB of heap while parsed. The bundle
//! is parsed inside an upload attempt and dropped afterwards rather than held resident:
//! uploads happen once a shot, and holding kilobytes permanently to save a few tens of
//! milliseconds of parsing is the wrong trade on a machine with ~2.4 kB of `.stack` spare.
//!
//! [`Certificate`]: mbedtls_rs::Certificate
//! [`CStr`]: core::ffi::CStr

use core::ffi::CStr;


/// The bundle handed to MbedTLS, NUL-terminated.
///
/// Adding a root means concatenating its PEM into [`BUNDLE_TEXT`] below. `concat!` takes
/// only literals, not `const`s, so a second root goes in as another `include_str!` or
/// another literal -- not as `concat!(ROOT_A, ROOT_B)`.
///
/// The `match` is how a missing or interior NUL becomes a build failure rather than a
/// parse error at the first upload: a `panic!` reached in a `const` initializer is
/// evaluated at compile time.
const BUNDLE_TEXT: &str = concat!(
    include_str!("roots/isrg-root-x1.pem"),
    "\0"
);

pub const CA_BUNDLE_PEM: &CStr = match CStr::from_bytes_with_nul(BUNDLE_TEXT.as_bytes()) {
    Ok(bundle) => bundle,
    Err(_) => panic!("CA bundle contains an interior NUL"),
};
