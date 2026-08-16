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
//! # Which root, and how it was chosen
//!
//! **GTS Root R4 only**, because that is what `plantlet.variegated.coffee` actually chains
//! to. Confirmed, not assumed:
//!
//! ```sh
//! openssl s_client -connect plantlet.variegated.coffee:443 \
//!     -servername plantlet.variegated.coffee -showcerts </dev/null
//! #   0 s:CN=variegated.coffee          i:Google Trust Services, CN=WE1
//! #   1 s:Google Trust Services, CN=WE1 i:Google Trust Services LLC, CN=GTS Root R4
//! #   2 s:GTS Root R4                   i:GlobalSign nv-sa, CN=GlobalSign Root CA
//!
//! openssl s_client -connect plantlet.variegated.coffee:443 \
//!     -servername plantlet.variegated.coffee -CAfile roots/gts-root-r4.pem </dev/null
//! #   Verify return code: 0 (ok)
//! ```
//!
//! The **root**, not the intermediate: `WE1` rotates far more often than R4, and pinning it
//! would turn a routine renewal into a machine that silently stops uploading. The server
//! also sends R4 cross-signed by GlobalSign Root CA (element 2 above); that path is not
//! needed once R4 is a trust anchor in its own right, and MbedTLS stops there.
//!
//! **The first bundle here was ISRG Root X1** -- a reasonable guess at a self-hosted
//! endpoint, and wrong: Let's Encrypt appears nowhere in this chain, so every handshake
//! would have failed on an untrusted root once it got far enough to check. Guessing a trust
//! anchor is not a shortcut worth taking; the `s_client` line above takes ten seconds.
//!
//! R4 is ECDSA P-384, which is why `curve-secp384r1` and `alg-sha512` are in the curated
//! feature set. The whole chain is ECDSA -- the endpoint negotiates TLS 1.3 with
//! `TLS_AES_256_GCM_SHA384` and an `ecdsa_secp256r1_sha256` peer signature -- so `alg-rsa`,
//! `alg-rsa-pss` and `kex-ecdhe-rsa` are unused against *this* host. They are kept anyway:
//! they cost flash rather than the heap that is actually scarce, and dropping them would
//! make a move to an RSA-issued certificate fail as an obscure handshake error.
//!
//! Deliberately *not* a full Mozilla bundle. That is ~140 certificates: ~200 kB of flash,
//! and worse, every one of them is parsed into heap at handshake time. This firmware talks
//! to exactly one host, so the trust store should be the roots that host actually uses.
//!
//! A root costs its PEM in `.rodata` -- 765 bytes for R4, against 1,939 for the RSA-4096
//! ISRG X1 it replaced -- plus roughly a kilobyte of heap while parsed. The bundle is parsed
//! inside an upload attempt and dropped afterwards rather than held resident: uploads happen
//! once a shot, and the heap is the scarcest thing on this device during a handshake.
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
    include_str!("roots/gts-root-r4.pem"),
    "\0"
);

pub const CA_BUNDLE_PEM: &CStr = match CStr::from_bytes_with_nul(BUNDLE_TEXT.as_bytes()) {
    Ok(bundle) => bundle,
    Err(_) => panic!("CA bundle contains an interior NUL"),
};
