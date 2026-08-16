//! The TLS session: what is trusted, what is checked, and what is deliberately not.
//!
//! # The two settings that make verification real
//!
//! MbedTLS checks the chain against [`roots::CA_BUNDLE_PEM`] **and** the hostname, but only
//! if both are supplied:
//!
//! * `ca_chain: None` -- no chain verification at all.
//! * `server_name: None` -- the chain is checked, but *any* certificate from any trusted
//!   root passes. That is the subtler failure, because it looks like it works. It is why
//!   [`client_config`] takes the name as a required argument rather than an `Option`.
//!
//! `AuthMode::Required` is MbedTLS's default and is set explicitly so a future edit has to
//! think about it rather than inherit it.
//!
//! # The clock, and failing closed
//!
//! X.509 validity dates need a wall clock, and this crate is built with `hook-wall-clock`.
//! **The caller must install one**; with no hook, MbedTLS reads every certificate as both
//! expired and not yet valid, so every handshake fails. That is the right default for a
//! device whose clock starts at zero -- the safe answer to "I don't know what year it is"
//! is to refuse -- but it means a caller that forgets sees uniform, unexplained failure.

use core::ffi::CStr;

use mbedtls_rs::{
    AuthMode, Certificate, ClientSessionConfig, Session, SessionConfig, TlsReference, TlsVersion,
    X509,
};

use crate::roots;

/// Why a TLS connection attempt did not produce a session.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectError {
    /// The CA bundle did not parse. A build-time mistake, not a network one -- and one the
    /// tests in [`roots`] are there to catch before it ships.
    BadCaBundle,
    /// `Session::new` failed before a byte was sent. Almost always
    /// `MBEDTLS_ERR_SSL_ALLOC_FAILED` (`-0x7F00`): setting a session up allocates both
    /// record buffers, the SSL context, the config and the DRBG at once, and on a
    /// microcontroller that is the largest single demand the upload path makes.
    ///
    /// Kept distinct from [`Self::Handshake`] because reporting it as one sends you looking
    /// at the certificate for what is a memory problem.
    SessionSetup { code: i32 },
    /// The handshake failed. Both of MbedTLS's answers are carried, and they mean different
    /// things:
    ///
    /// * `verification_flags` non-zero: the **certificate** was rejected. `0x08`
    ///   NOT_TRUSTED (wrong root), `0x04` CN_MISMATCH (wrong host), `0x01` EXPIRED.
    /// * `verification_flags` zero: the certificate was not the problem, or the handshake
    ///   never got far enough to check one. Read `code`; `-0x7F00` is an allocation failure.
    Handshake { code: i32, verification_flags: u32 },
}

/// The client configuration this firmware uses, with the trust anchors compiled in.
///
/// `server_name` must be the host from the endpoint URL, NUL-terminated. It is used for both
/// SNI and the certificate's CN/SAN check -- see the module docs for why it is not optional.
pub fn client_config(server_name: &CStr) -> Result<SessionConfig<'_>, ConnectError> {
    // Parsed per call and dropped with the session rather than held resident: a root costs
    // roughly a kilobyte of heap while parsed, and an upload happens once a shot.
    // `SessionState` clones this into a refcount, so it lives exactly as long as needed.
    let ca_chain =
        Certificate::new(X509::PEM(roots::CA_BUNDLE_PEM)).map_err(|_| ConnectError::BadCaBundle)?;

    // `AuthMode::None` still *parses* the chain and still records what it thought of it in
    // `tls_verification_details()`; it just does not abort. So a build with this on will
    // complete the handshake and can still report the flags, which is what makes it useful
    // as a diagnostic rather than merely permissive.
    #[cfg(feature = "danger-skip-verification")]
    let auth_mode = AuthMode::None;
    #[cfg(not(feature = "danger-skip-verification"))]
    let auth_mode = AuthMode::Required;

    Ok(SessionConfig::Client(ClientSessionConfig {
        ca_chain: Some(ca_chain),
        creds: None,
        // Kept even when verification is off: it is what puts the name in SNI, and a server
        // that picks its certificate by SNI would otherwise serve a different one -- which
        // would make this diagnostic answer a question nobody asked.
        server_name: Some(server_name),
        auth_mode,
        min_version: TlsVersion::Tls1_2,
        alpn_protocols: Some(&[c"http/1.1"]),
    }))
}

/// Whether this build verifies certificates.
///
/// Exposed so the firmware can say so in a log line at every handshake. A machine that is
/// not checking certificates should never be quiet about it.
pub const VERIFIES_CERTIFICATES: bool = !cfg!(feature = "danger-skip-verification");

/// Complete a verified TLS handshake over an already-connected stream.
///
/// This crate opens no sockets and resolves no names: `stream` arrives connected, which is
/// what keeps the whole module portable and testable.
pub async fn connect<'a, T>(
    tls: TlsReference<'a>,
    stream: T,
    server_name: &'a CStr,
) -> Result<Session<'a, T>, ConnectError>
where
    T: embedded_io_async::Read + embedded_io_async::Write,
{
    let config = client_config(server_name)?;

    let mut session = Session::new(tls, stream, &config).map_err(|e| ConnectError::SessionSetup {
        code: session_error_code(e),
    })?;

    match session.connect().await {
        Ok(()) => Ok(session),
        Err(e) => Err(ConnectError::Handshake {
            code: session_error_code(e),
            verification_flags: session.tls_verification_details(),
        }),
    }
}

/// The raw MbedTLS code, or 0 for an I/O error from the stream underneath.
///
/// The number matters more than the variant: `-0x7F00` is `MBEDTLS_ERR_SSL_ALLOC_FAILED`,
/// and the difference between that and a certificate rejection is the difference between a
/// memory problem and a trust problem.
fn session_error_code(e: mbedtls_rs::SessionError) -> i32 {
    match e {
        mbedtls_rs::SessionError::MbedTls(e) => e.code(),
        mbedtls_rs::SessionError::Io(_) => 0,
    }
}
