//! Turning a configured endpoint string into a host, a port and a request target.
//!
//! Deliberately minimal -- a sanity gate, not a URL library. It refuses the shapes that
//! would otherwise fail confusingly much later (plaintext, userinfo, an empty host) and
//! passes everything else through for DNS and the server to judge.
//!
//! The rule that keeps the split with the firmware honest: nothing in
//! `variegated-comms-firmware` slices a URL. It calls [`parse_url`] once and uses the
//! [`Endpoint`] it gets back -- including its [`Scheme`], which is how the transport is
//! chosen.
//!
//! # Two schemes, and why only one of them may carry a token
//!
//! `https://` and `http+noise://`. Bare `http://` stays refused, and the reason is not
//! "plaintext is bad" in the abstract -- it is that [`body::request_head`] puts a bearer
//! token in an `Authorization` header. On `http+noise://` there is no such header: the
//! device's static public key is the credential and it travels encrypted inside the Noise
//! handshake, so there is nothing on the wire for a plaintext scheme to leak.
//!
//! [`body::request_head`]: crate::body::request_head

/// Which transport an endpoint asks for.
///
/// **Not gated on `feature = "noise"` or `feature = "tls"`.** Parsing has to give the same
/// answer in every build, so that a firmware compiled without a transport reports "this build
/// cannot speak that scheme" at dispatch rather than "malformed URL" at the parse. The two
/// are very different bug reports and only one of them is true.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scheme {
    /// `https://` -- TLS, and the only scheme a bearer token may travel on.
    Tls,
    /// `http+noise://` -- a plaintext TCP connection carrying a `Noise_X` body.
    ///
    /// Plaintext at the HTTP layer is not a downgrade here because nothing sensitive rides
    /// outside the Noise message: there is no `Authorization` header on this path, and the
    /// device's static public key -- the credential -- is encrypted inside the handshake.
    Noise,
}

impl Scheme {
    /// The port to use when the authority does not name one.
    pub const fn default_port(self) -> u16 {
        match self {
            Scheme::Tls => 443,
            Scheme::Noise => 80,
        }
    }
}

/// Where a parsed endpoint points.
///
/// Borrows from the input rather than copying: the caller holds the
/// `heapless::String<255>` for the whole attempt anyway, and the alternative is three more
/// allocations per upload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Endpoint<'a> {
    /// Which transport to open.
    pub scheme: Scheme,
    /// Host only -- no brackets on an IPv6 literal, no port, no userinfo.
    pub host: &'a str,
    /// Explicit port, or the scheme's default.
    pub port: u16,
    /// The HTTP request target: path plus query, verbatim, or `/`.
    pub path: &'a str,
}

/// Why an endpoint string was refused.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UrlError {
    /// No scheme, or an unrecognised one -- and also what [`parse_https_url`] returns for a
    /// perfectly good `http+noise://` URL, because from its caller's point of view that is
    /// still "not the scheme I require".
    ///
    /// Kept distinct from the rest so the log line can say *why* rather than "malformed
    /// URL". Bare `http://` is the mistake someone actually makes, and it would send a
    /// bearer token in the clear -- which is exactly what `http+noise://` does not do, and
    /// why that one is accepted.
    NotHttps,
    /// Nothing between the scheme and the path.
    EmptyHost,
    /// Longer than a DNS name may be.
    HostTooLong,
    /// A byte outside `[A-Za-z0-9.-]` (or the hex and colons of a bracketed literal).
    BadHostChar,
    /// An `@` in the authority. Credentials belong in the token, not the URL, and a
    /// `user:pass@` that reached SNI would be a confusing failure rather than an obvious
    /// one.
    UserinfoPresent,
    /// Not a number, or zero.
    BadPort,
    /// A `[` with no matching `]`.
    UnclosedBracket,
}

/// Longest legal DNS name.
const MAX_HOST_LEN: usize = 253;

/// Parse `https://host[:port][/path[?query]]`, refusing any other scheme.
///
/// Kept as its own entry point rather than folded into [`parse_url`]: it is what a caller
/// that must not send plaintext asks for, and refusing at the parse is better than
/// remembering to check [`Endpoint::scheme`] afterwards.
pub fn parse_https_url(url: &str) -> Result<Endpoint<'_>, UrlError> {
    match parse_url(url)? {
        endpoint @ Endpoint { scheme: Scheme::Tls, .. } => Ok(endpoint),
        _ => Err(UrlError::NotHttps),
    }
}

/// Parse any endpoint this crate knows how to upload to.
///
/// Deliberately minimal -- a sanity gate, not a URL library. It refuses the shapes that
/// would fail confusingly later (plaintext, userinfo, an empty host) and passes everything
/// else through for DNS and the server to judge.
pub fn parse_url(url: &str) -> Result<Endpoint<'_>, UrlError> {
    // ASCII-case-insensitive: `HTTPS://` is legal and someone will paste it.
    let (scheme, rest) = strip_scheme(url).ok_or(UrlError::NotHttps)?;

    // The authority ends at the first `/`, `?` or `#`.
    let authority_end = rest
        .find(['/', '?', '#'])
        .unwrap_or(rest.len());
    let (authority, tail) = rest.split_at(authority_end);

    if authority.contains('@') {
        return Err(UrlError::UserinfoPresent);
    }

    let (host, port) = split_host_port(authority, scheme.default_port())?;

    if host.is_empty() {
        return Err(UrlError::EmptyHost);
    }
    if host.len() > MAX_HOST_LEN {
        return Err(UrlError::HostTooLong);
    }
    // A bracketed literal has already had its brackets removed, so colons are only legal
    // inside one. `split_host_port` tells us which case we are in by whether it saw them.
    let bracketed = authority.starts_with('[');
    if !host.bytes().all(|b| is_host_byte(b, bracketed)) {
        return Err(UrlError::BadHostChar);
    }

    // The query is part of the request target and must survive verbatim: an endpoint with
    // `?src=x` that silently loses it reaches a different handler.
    let path = if tail.is_empty() { "/" } else { tail };

    Ok(Endpoint { scheme, host, port, path })
}

fn strip_scheme(url: &str) -> Option<(Scheme, &str)> {
    const HTTPS: &str = "https://";
    const NOISE: &str = "http+noise://";
    const HTTP: &str = "http://";

    // Neither prefix is a prefix of the other, so the order of these two does not matter.
    for (prefix, scheme) in [(HTTPS, Scheme::Tls), (NOISE, Scheme::Noise)] {
        if url.len() >= prefix.len() && url[..prefix.len()].eq_ignore_ascii_case(prefix) {
            return Some((scheme, &url[prefix.len()..]));
        }
    }
    // Falls through to `None` like any other unrecognised scheme, but the caller maps both
    // to `NotHttps`, which is the accurate description either way. Bare `http://` stays
    // refused: it is the one that would put a bearer token in the clear, and `http+noise://`
    // is acceptable precisely because it sends no token.
    let _ = HTTP;
    None
}

fn split_host_port(authority: &str, default_port: u16) -> Result<(&str, u16), UrlError> {
    let (host, port_str) = if let Some(rest) = authority.strip_prefix('[') {
        // IPv6 literal. The port, if any, follows the `]`.
        let close = rest.find(']').ok_or(UrlError::UnclosedBracket)?;
        let host = &rest[..close];
        let after = &rest[close + 1..];
        match after.strip_prefix(':') {
            Some(p) => (host, Some(p)),
            None if after.is_empty() => (host, None),
            None => return Err(UrlError::BadHostChar),
        }
    } else {
        // The *last* colon, so a malformed `a:b:c` fails on the port rather than silently
        // taking a prefix.
        match authority.rsplit_once(':') {
            Some((h, p)) => (h, Some(p)),
            None => (authority, None),
        }
    };

    let port = match port_str {
        None => default_port,
        Some(p) => {
            let parsed: u16 = p.parse().map_err(|_| UrlError::BadPort)?;
            if parsed == 0 {
                return Err(UrlError::BadPort);
            }
            parsed
        }
    };

    Ok((host, port))
}

fn is_host_byte(b: u8, bracketed: bool) -> bool {
    b.is_ascii_alphanumeric() || b == b'.' || b == b'-' || (bracketed && (b == b':' || b.is_ascii_hexdigit()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_plain_https_url_parses() {
        let e = parse_https_url("https://plantlet.example/api/shots").unwrap();
        assert_eq!(e.host, "plantlet.example");
        assert_eq!(e.port, 443);
        assert_eq!(e.path, "/api/shots");
    }

    #[test]
    fn the_scheme_is_case_insensitive() {
        assert_eq!(parse_https_url("HTTPS://a.example/x").unwrap().host, "a.example");
    }

    #[test]
    fn an_explicit_port_is_taken() {
        let e = parse_https_url("https://a.example:8443/x").unwrap();
        assert_eq!(e.port, 8443);
        assert_eq!(e.host, "a.example");
    }

    #[test]
    fn a_missing_path_becomes_a_slash() {
        // `POST ` with an empty target is not a request line any server will accept.
        assert_eq!(parse_https_url("https://a.example").unwrap().path, "/");
    }

    #[test]
    fn the_query_survives_verbatim() {
        // Part of the request target. An endpoint that loses its query reaches a different
        // handler, and the failure looks like a server bug.
        let e = parse_https_url("https://a.example/api/shots?src=machine1&v=2").unwrap();
        assert_eq!(e.path, "/api/shots?src=machine1&v=2");
    }

    #[test]
    fn plaintext_is_refused_with_its_own_error() {
        // Its own variant so the log says why. This one would put a bearer token on the
        // wire in the clear.
        assert_eq!(parse_https_url("http://a.example/x"), Err(UrlError::NotHttps));
        assert_eq!(parse_https_url("a.example/x"), Err(UrlError::NotHttps));
        assert_eq!(parse_https_url(""), Err(UrlError::NotHttps));
    }

    #[test]
    fn userinfo_is_refused() {
        assert_eq!(
            parse_https_url("https://user:pass@a.example/x"),
            Err(UrlError::UserinfoPresent)
        );
    }

    #[test]
    fn an_empty_host_is_refused() {
        assert_eq!(parse_https_url("https:///x"), Err(UrlError::EmptyHost));
    }

    #[test]
    fn a_bad_port_is_refused() {
        assert_eq!(parse_https_url("https://a.example:0/x"), Err(UrlError::BadPort));
        assert_eq!(parse_https_url("https://a.example:http/x"), Err(UrlError::BadPort));
        assert_eq!(parse_https_url("https://a.example:99999/x"), Err(UrlError::BadPort));
    }

    #[test]
    fn an_over_long_host_is_refused() {
        let host = "a".repeat(MAX_HOST_LEN + 1);
        let url = alloc::format!("https://{host}/x");
        assert_eq!(parse_https_url(&url), Err(UrlError::HostTooLong));
    }

    #[test]
    fn a_host_with_an_illegal_byte_is_refused() {
        assert_eq!(parse_https_url("https://a_b.example/x"), Err(UrlError::BadHostChar));
    }

    #[test]
    fn an_ipv6_literal_loses_its_brackets() {
        // The brackets are URL syntax, not part of the name -- passing them to DNS or SNI
        // would be a confusing failure.
        let e = parse_https_url("https://[2001:db8::1]:8443/x").unwrap();
        assert_eq!(e.host, "2001:db8::1");
        assert_eq!(e.port, 8443);

        let e = parse_https_url("https://[2001:db8::1]/x").unwrap();
        assert_eq!(e.host, "2001:db8::1");
        assert_eq!(e.port, 443);
    }

    #[test]
    fn an_unclosed_bracket_is_refused() {
        assert_eq!(
            parse_https_url("https://[2001:db8::1/x"),
            Err(UrlError::UnclosedBracket)
        );
    }

    #[test]
    fn an_https_url_reports_its_scheme() {
        assert_eq!(
            parse_url("https://a.example/x").unwrap().scheme,
            Scheme::Tls
        );
    }

    #[test]
    fn a_noise_url_parses_and_defaults_to_port_80() {
        let e = parse_url("http+noise://plantlet.variegated.coffee/api/noise-upload").unwrap();
        assert_eq!(e.scheme, Scheme::Noise);
        assert_eq!(e.host, "plantlet.variegated.coffee");
        assert_eq!(e.port, 80);
        assert_eq!(e.path, "/api/noise-upload");
    }

    #[test]
    fn the_noise_scheme_is_case_insensitive() {
        assert_eq!(
            parse_url("HTTP+NOISE://a.example/x").unwrap().scheme,
            Scheme::Noise
        );
    }

    #[test]
    fn an_explicit_port_overrides_the_noise_default() {
        let e = parse_url("http+noise://a.example:8787/x").unwrap();
        assert_eq!(e.port, 8787);
        assert_eq!(e.scheme, Scheme::Noise);
    }

    #[test]
    fn plaintext_http_is_still_refused_by_both_entry_points() {
        // `http+noise://` being acceptable must not make bare `http://` acceptable. The
        // distinction is the whole argument: one sends a bearer token in the clear, the
        // other sends no token at all.
        assert_eq!(parse_url("http://a.example/x"), Err(UrlError::NotHttps));
        assert_eq!(parse_https_url("http://a.example/x"), Err(UrlError::NotHttps));
    }

    #[test]
    fn parse_https_url_refuses_a_noise_url() {
        // The filter earns its existence here: a caller that must not send plaintext gets a
        // refusal rather than an `Endpoint` it has to remember to inspect.
        assert_eq!(
            parse_https_url("http+noise://a.example/x"),
            Err(UrlError::NotHttps)
        );
    }

    #[test]
    fn the_noise_scheme_shares_every_other_rule() {
        // The authority rules are scheme-independent; this pins that they were not
        // accidentally special-cased along with the port.
        assert_eq!(
            parse_url("http+noise://user:pass@a.example/x"),
            Err(UrlError::UserinfoPresent)
        );
        assert_eq!(parse_url("http+noise:///x"), Err(UrlError::EmptyHost));
        assert_eq!(parse_url("http+noise://a_b.example/x"), Err(UrlError::BadHostChar));
        let e = parse_url("http+noise://[2001:db8::1]/x").unwrap();
        assert_eq!(e.host, "2001:db8::1");
        assert_eq!(e.port, 80);
    }
}
