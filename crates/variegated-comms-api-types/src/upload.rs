//! The decidable half of shot-log upload: parsing the endpoint, and reading the response.
//!
//! # Why this is not in the firmware crate
//!
//! `variegated-comms-firmware` sets `[lib] harness = false`, which means cargo expects the
//! lib to supply its own `main` and **runs no tests at all -- silently, reporting success**.
//! That crate's own `scripts/test-host.sh` says so in as many words. Anything here that is
//! worth a test therefore has to live outside it, and this crate is portable `no_std` with
//! no such setting.
//!
//! The rule that keeps the split honest: `upload/mod.rs` in the firmware contains **no**
//! branching on HTTP status codes and **no** string slicing of URLs. It calls
//! [`parse_https_url`] once and [`classify_status`] once and matches on what comes back. An
//! `if code == 429` in the firmware crate means the logic has leaked into the half that
//! cannot be tested.

/// Where a parsed endpoint points.
///
/// Borrows from the input rather than copying: the caller holds the
/// `heapless::String<255>` for the whole attempt anyway, and the alternative is three more
/// allocations per upload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Endpoint<'a> {
    /// Host only -- no brackets on an IPv6 literal, no port, no userinfo.
    pub host: &'a str,
    /// Explicit port, or 443.
    pub port: u16,
    /// The HTTP request target: path plus query, verbatim, or `/`.
    pub path: &'a str,
}

/// Why an endpoint string was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UrlError {
    /// No scheme, or `http://`. Kept distinct from the rest so the log line can say *why*
    /// rather than "malformed URL" -- plaintext is the mistake someone actually makes, and
    /// it would send a bearer token in the clear.
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

/// Parse `https://host[:port][/path[?query]]`.
///
/// Deliberately minimal -- a sanity gate, not a URL library. It refuses the shapes that
/// would fail confusingly later (plaintext, userinfo, an empty host) and passes everything
/// else through for DNS and the server to judge.
pub fn parse_https_url(url: &str) -> Result<Endpoint<'_>, UrlError> {
    // ASCII-case-insensitive: `HTTPS://` is legal and someone will paste it.
    let rest = strip_scheme(url).ok_or(UrlError::NotHttps)?;

    // The authority ends at the first `/`, `?` or `#`.
    let authority_end = rest
        .find(['/', '?', '#'])
        .unwrap_or(rest.len());
    let (authority, tail) = rest.split_at(authority_end);

    if authority.contains('@') {
        return Err(UrlError::UserinfoPresent);
    }

    let (host, port) = split_host_port(authority)?;

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

    Ok(Endpoint { host, port, path })
}

fn strip_scheme(url: &str) -> Option<&str> {
    const HTTPS: &str = "https://";
    const HTTP: &str = "http://";

    if url.len() >= HTTPS.len() && url[..HTTPS.len()].eq_ignore_ascii_case(HTTPS) {
        return Some(&url[HTTPS.len()..]);
    }
    // Falls through to `None` like any other unrecognised scheme, but the caller maps both
    // to `NotHttps`, which is the accurate description either way.
    let _ = HTTP;
    None
}

fn split_host_port(authority: &str) -> Result<(&str, u16), UrlError> {
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
        None => 443,
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

/// What the server said, reduced to what the uploader does about it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UploadOutcome {
    /// `201` -- stored.
    Created,
    /// `200` -- these exact bytes were already here. Also a success: the endpoint is
    /// content-hash deduped, so a retry after a partial failure lands here rather than
    /// creating a second copy.
    Duplicate,
    /// `429`. Wait this many seconds, then it may be retried.
    RateLimited { retry_after_secs: u32 },
    /// `5xx`, or anything else transient. Retry.
    ServerError,
    /// `401`, `413`, `422`, or an unrecognised code. **Do not retry**: nothing about
    /// sending the same bytes again will change the answer, and a retry storm against a bad
    /// token is precisely what a rate limiter exists to stop.
    Permanent,
}

/// The longest `Retry-After` worth sleeping on.
///
/// Above this the uploader gives up on the shot rather than parking a task for an hour --
/// under live-only scope the shot is lost either way, and a task asleep for an hour is a
/// task that looks hung.
pub const MAX_RETRY_AFTER_SECS: u32 = 300;

/// Reduce a status line to a decision.
///
/// `retry_after` is the raw header value, if the response carried one.
pub fn classify_status(code: u16, retry_after: Option<&str>) -> UploadOutcome {
    match code {
        201 => UploadOutcome::Created,
        200 => UploadOutcome::Duplicate,
        429 => UploadOutcome::RateLimited {
            // No header, or one we cannot read, is not a reason to hammer: fall back to a
            // full minute rather than to zero.
            retry_after_secs: retry_after.and_then(parse_retry_after).unwrap_or(60),
        },
        500..=599 => UploadOutcome::ServerError,
        _ => UploadOutcome::Permanent,
    }
}

/// Parse a `Retry-After` given as delta-seconds.
///
/// HTTP also allows an absolute date there. This returns `None` for one deliberately
/// rather than growing a date parser: the caller's fallback is a fixed minute, and being
/// approximately right about when to retry costs nothing here.
pub fn parse_retry_after(value: &str) -> Option<u32> {
    let trimmed = value.trim();
    if trimmed.is_empty() || !trimmed.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    // Saturating rather than wrapping: an absurd value should read as "very long", which
    // the caller then refuses, not as a small one it would act on.
    Some(trimmed.parse::<u32>().unwrap_or(u32::MAX))
}

/// How many attempts one shot gets before it is abandoned.
pub const MAX_ATTEMPTS: u8 = 3;

/// Seconds to wait before attempt `attempt` (0-based), absent a `Retry-After`.
///
/// Returns `None` once the attempts are spent, which is what terminates the loop -- an
/// unbounded retry is the failure mode this whole path is shaped to avoid.
pub fn retry_delay(attempt: u8) -> Option<u32> {
    match attempt {
        0 => Some(5),
        1 => Some(30),
        _ => None,
    }
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
    fn the_two_success_codes_are_both_successes() {
        // 200 means the endpoint already had these exact bytes. Treating it as a failure
        // would make every retry-after-partial-failure look like an error.
        assert_eq!(classify_status(201, None), UploadOutcome::Created);
        assert_eq!(classify_status(200, None), UploadOutcome::Duplicate);
    }

    #[test]
    fn the_permanent_failures_do_not_retry() {
        // 401 above all: retrying a bad token is what a rate limiter exists to stop.
        for code in [401, 413, 422, 302, 404] {
            assert_eq!(classify_status(code, None), UploadOutcome::Permanent, "{code}");
        }
    }

    #[test]
    fn server_errors_retry() {
        for code in [500, 502, 503, 599] {
            assert_eq!(classify_status(code, None), UploadOutcome::ServerError, "{code}");
        }
    }

    #[test]
    fn rate_limiting_honours_retry_after_and_falls_back_to_a_minute() {
        assert_eq!(
            classify_status(429, Some("30")),
            UploadOutcome::RateLimited { retry_after_secs: 30 }
        );
        // No header, or an HTTP-date we do not parse: a full minute, never zero. Falling
        // back to zero would turn a rate limit into a hot loop.
        assert_eq!(
            classify_status(429, None),
            UploadOutcome::RateLimited { retry_after_secs: 60 }
        );
        assert_eq!(
            classify_status(429, Some("Wed, 21 Oct 2026 07:28:00 GMT")),
            UploadOutcome::RateLimited { retry_after_secs: 60 }
        );
    }

    #[test]
    fn retry_after_parses_only_delta_seconds() {
        assert_eq!(parse_retry_after("30"), Some(30));
        assert_eq!(parse_retry_after("  30  "), Some(30));
        assert_eq!(parse_retry_after("0"), Some(0));
        assert_eq!(parse_retry_after("abc"), None);
        assert_eq!(parse_retry_after(""), None);
        // Saturates rather than wrapping: an absurd value must read as "very long", which
        // the caller refuses, not as a small one it would act on.
        assert_eq!(parse_retry_after("99999999999"), Some(u32::MAX));
    }

    #[test]
    fn the_retry_schedule_terminates() {
        assert_eq!(retry_delay(0), Some(5));
        assert_eq!(retry_delay(1), Some(30));
        // The bound that makes the loop finite.
        assert_eq!(retry_delay(2), None);
        assert_eq!(retry_delay(MAX_ATTEMPTS), None);
    }
}
