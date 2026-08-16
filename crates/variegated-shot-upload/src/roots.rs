//! Trust anchors for shot-log upload.
//!
//! # One concatenated PEM, not a list of DERs
//!
//! `mbedtls-rs` accepts exactly one `Certificate` for `ClientSessionConfig::ca_chain` and
//! offers no way to chain two together. `Certificate::new_no_copy` takes a single DER; only
//! `Certificate::new(X509::PEM(..))` reaches `mbedtls_x509_crt_parse`, which walks a
//! concatenated PEM bundle and builds the whole linked list. So a multi-root store has to be
//! PEM, and the `pem-parse` and `base64` features have to stay in the curated set.
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
//! ```
//!
//! The **root**, not the intermediate: `WE1` rotates far more often than R4, and pinning it
//! would turn a routine renewal into a machine that silently stops uploading. The server
//! also sends R4 cross-signed by GlobalSign Root CA (element 2 above); that path is not
//! needed once R4 is a trust anchor in its own right, and verification stops there.
//!
//! **The first bundle here was ISRG Root X1** -- a reasonable guess at a self-hosted
//! endpoint, and wrong: Let's Encrypt appears nowhere in this chain, so every handshake
//! failed on an untrusted root once it got far enough to check. It reached hardware twice.
//! The tests at the bottom of this file are the reason it cannot happen a third time, and
//! they are most of why this crate was split out of the firmware at all.
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
//! [`CStr`]: core::ffi::CStr

use core::ffi::CStr;

/// The bundle handed to MbedTLS, NUL-terminated.
///
/// Adding a root means concatenating its PEM into [`BUNDLE_TEXT`] below. `concat!` takes
/// only literals, not `const`s, so a second root goes in as another `include_str!` -- not as
/// `concat!(ROOT_A, ROOT_B)`.
///
/// The `match` is how a missing or interior NUL becomes a build failure rather than a parse
/// error at the first upload: a `panic!` reached in a `const` initializer is evaluated at
/// compile time.
const BUNDLE_TEXT: &str = concat!(include_str!("roots/gts-root-r4.pem"), "\0");

pub const CA_BUNDLE_PEM: &CStr = match CStr::from_bytes_with_nul(BUNDLE_TEXT.as_bytes()) {
    Ok(bundle) => bundle,
    Err(_) => panic!("CA bundle contains an interior NUL"),
};

/// The host these anchors have to work for.
///
/// Not used at runtime -- the endpoint is a setting and this crate never assumes a host. It
/// exists so the tests below can assert the leaf is valid for the name the machine actually
/// asks for, which is the half of verification that looks like it works when it is missing.
pub const EXPECTED_HOST: &str = "plantlet.variegated.coffee";

/// Trust-anchor tests, run against **real MbedTLS** rather than a stand-in.
///
/// That matters more than it might look. A pure-Rust verifier would answer "is this the
/// right root for this chain", which is the mistake that shipped -- but not "will MbedTLS,
/// compiled with *this* feature set, accept it". A missing `curve-secp384r1` or
/// `alg-sha512` produces exactly the same `BADCERT_NOT_TRUSTED` on the device while any
/// other library is perfectly happy, and that is a failure mode worth owning.
///
/// The cost is that these do not run on macOS -- see the `tls` feature in `Cargo.toml` for
/// the three upstream reasons. Run them with `scripts/test-mbedtls.sh`.
#[cfg(all(test, feature = "tls"))]
mod tests {
    use super::*;
    use mbedtls_rs::sys;
    use mbedtls_rs::{Certificate, X509};

    /// The chain `plantlet.variegated.coffee` served on 2026-08-16, captured with:
    ///
    /// ```sh
    /// openssl s_client -connect plantlet.variegated.coffee:443 \
    ///     -servername plantlet.variegated.coffee -showcerts </dev/null \
    ///     | awk '/BEGIN CERT/,/END CERT/' > fixtures/plantlet-chain.pem
    /// ```
    ///
    /// Refresh it when the leaf is renewed. Date failures are masked below, so a stale
    /// fixture stays useful indefinitely.
    const SERVED_CHAIN: &str = concat!(include_str!("../fixtures/plantlet-chain.pem"), "\0");

    /// ISRG Root X1 -- the wrong answer that shipped twice, kept as the negative control.
    const ISRG_ROOT_X1: &str = concat!(include_str!("../fixtures/isrg-root-x1.pem"), "\0");

    /// The host these anchors have to work for, NUL-terminated for the C API.
    const HOST: &CStr = c"plantlet.variegated.coffee";

    /// Bits of MbedTLS's verification bitmask, by name.
    const BADCERT_EXPIRED: u32 = 0x01;
    const BADCERT_CN_MISMATCH: u32 = 0x04;
    const BADCERT_NOT_TRUSTED: u32 = 0x08;
    const BADCERT_FUTURE: u32 = 0x200;

    /// A parsed chain, owned so it can be freed.
    ///
    /// The safe `Certificate` cannot be used for verification -- its `crt` handle is
    /// `pub(crate)` in `mbedtls-rs` and the crate exposes no `verify` -- so this goes
    /// through the raw bindings. That keeps the test on the same library, the same compiled
    /// feature set and the same PEM bytes the firmware uses.
    struct Chain(alloc::boxed::Box<sys::mbedtls_x509_crt>);

    impl Chain {
        fn parse(pem: &CStr) -> Self {
            // SAFETY: `mbedtls_x509_crt`'s documented initialisation is all-zero followed
            // by `_init`. The box keeps it at a stable address for as long as `Drop` needs.
            let mut crt: alloc::boxed::Box<sys::mbedtls_x509_crt> =
                alloc::boxed::Box::new(unsafe { core::mem::zeroed() });
            unsafe { sys::mbedtls_x509_crt_init(&mut *crt) };

            // `count_bytes() + 1`: PEM input must include the terminating NUL in the length.
            // The same call `Certificate::new` makes.
            let rc = unsafe {
                sys::mbedtls_x509_crt_parse(
                    &mut *crt,
                    pem.as_ptr() as *const u8,
                    pem.count_bytes() + 1,
                )
            };
            assert_eq!(rc, 0, "certificate parse failed: {rc:#x}");
            Self(crt)
        }

        /// MbedTLS's verification bitmask. Zero means fully verified.
        fn verify_against(&mut self, ca: &mut Chain, host: &CStr) -> u32 {
            let mut flags: u32 = 0;
            // The return code only says "were the flags non-zero", so the flags are what
            // gets asserted on.
            let _ = unsafe {
                sys::mbedtls_x509_crt_verify(
                    &mut *self.0,
                    &mut *ca.0,
                    core::ptr::null_mut(),
                    host.as_ptr(),
                    &mut flags,
                    None,
                    core::ptr::null_mut(),
                )
            };
            flags
        }
    }

    impl Drop for Chain {
        fn drop(&mut self) {
            unsafe { sys::mbedtls_x509_crt_free(&mut *self.0) };
        }
    }

    /// X.509 date checking needs a clock, and this crate compiles with `hook-wall-clock`.
    /// With no hook every certificate reads as both expired and not yet valid -- correct
    /// behaviour, and it would make these tests pass for the wrong reason.
    fn install_clock() {
        static CLOCK: sys::hook::backend::std::wall_clock::StdWallClock =
            sys::hook::backend::std::wall_clock::StdWallClock;
        unsafe { sys::hook::wall_clock::hook_wall_clock(Some(&CLOCK)) };
    }

    fn served_chain() -> Chain {
        Chain::parse(CStr::from_bytes_with_nul(SERVED_CHAIN.as_bytes()).expect("no interior NUL"))
    }

    fn bundle() -> Chain {
        Chain::parse(CA_BUNDLE_PEM)
    }

    /// Everything except a stale-fixture date failure. The leaf is renewed every few
    /// months, and a test that failed on an arbitrary Tuesday for a reason unrelated to the
    /// code is a test people learn to ignore.
    fn fatal(flags: u32) -> u32 {
        flags & !(BADCERT_EXPIRED | BADCERT_FUTURE)
    }

    #[test]
    fn the_bundle_parses_through_the_call_the_firmware_makes() {
        // A NUL in the wrong place, a mangled base64 line, a missing newline between two
        // concatenated roots: all land here rather than as an obscure handshake failure on
        // a machine in someone's kitchen.
        Certificate::new(X509::PEM(CA_BUNDLE_PEM)).expect("the CA bundle should parse");
    }

    /// **The test this crate exists for.**
    #[test]
    fn the_compiled_in_root_verifies_the_chain_the_endpoint_serves() {
        install_clock();
        let flags = served_chain().verify_against(&mut bundle(), HOST);

        assert_eq!(
            fatal(flags),
            0,
            "the compiled-in root does not verify the chain the endpoint serves: \
             flags {flags:#x} ({BADCERT_NOT_TRUSTED:#x} = NOT_TRUSTED, i.e. the wrong root; \
             {BADCERT_CN_MISMATCH:#x} = CN_MISMATCH, i.e. the leaf is not valid for this \
             host). Re-capture fixtures/plantlet-chain.pem with `openssl s_client \
             -showcerts` and take the *root* it terminates at, not the intermediate."
        );
    }

    /// A negative control, so the test above cannot pass by accident.
    ///
    /// Without it, a verify that returned zero for everything -- an empty trust store read
    /// as "nothing objected", a masking bug in `fatal` -- would look like success. This
    /// hands the same machinery the exact root that shipped by mistake and requires it to
    /// say no.
    #[test]
    fn the_root_that_shipped_by_mistake_is_rejected() {
        install_clock();
        let mut wrong =
            Chain::parse(CStr::from_bytes_with_nul(ISRG_ROOT_X1.as_bytes()).expect("no NUL"));
        let flags = served_chain().verify_against(&mut wrong, HOST);

        assert_ne!(
            flags & BADCERT_NOT_TRUSTED,
            0,
            "ISRG Root X1 does not sign this chain and must fail as NOT_TRUSTED, got {flags:#x}"
        );
    }

    /// The other half of `server_name`: the chain verifies, but for a host we are not
    /// talking to. Passing `None` for the name at runtime makes this case silently pass,
    /// which is why `client_config` takes it as a required argument.
    #[test]
    fn a_correctly_signed_chain_is_still_wrong_for_another_host() {
        install_clock();
        let flags = served_chain().verify_against(&mut bundle(), c"example.invalid");

        assert_ne!(
            flags & BADCERT_CN_MISMATCH,
            0,
            "the leaf is not valid for example.invalid and must fail as CN_MISMATCH, \
             got {flags:#x}"
        );
    }
}
