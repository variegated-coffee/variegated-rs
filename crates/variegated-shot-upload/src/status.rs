//! What the endpoint's answer means, and how long to wait before asking again.
//!
//! The rule that keeps the split with the firmware honest: nothing in
//! `variegated-comms-firmware` branches on a status code. It calls [`classify_status`] once
//! and matches the [`UploadOutcome`]. An `if code == 429` over there means this policy has
//! leaked into the half that cannot be tested.

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

/// How many attempts one shot gets before it is abandoned.
pub const MAX_ATTEMPTS: u8 = 3;

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
/// HTTP also allows an absolute date there. This returns `None` for one deliberately rather
/// than growing a date parser: the caller's fallback is a fixed minute, and being
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
