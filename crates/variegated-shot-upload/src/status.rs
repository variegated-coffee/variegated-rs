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

/// Whether the status line was carried by something that authenticated it.
///
/// See [`temper`]. This exists because `http+noise://` encrypts the *body* and leaves the
/// HTTP response line in the clear, so on that transport the status is an unauthenticated
/// hint rather than a fact.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResponseTrust {
    /// TLS. The status line is as trustworthy as the connection.
    Authenticated,
    /// Plain HTTP carrying a Noise body. Anyone on path can write this status line.
    Unauthenticated,
}

/// Downgrade an outcome that arrived on a channel nobody authenticated.
///
/// # What this protects
///
/// [`UploadOutcome::Permanent`] means "give up on this shot", and under live-only scope
/// giving up means the shot is gone. On an unauthenticated channel a single injected
/// `HTTP/1.1 401` would trigger that, and it would be indistinguishable from a genuinely
/// revoked key. `RateLimited` is no better: its `Retry-After` is equally forgeable, and one
/// above [`MAX_RETRY_AFTER_SECS`] also abandons the shot.
///
/// So every non-success reads as transient. The cost of being wrong in that direction is
/// bounded -- [`retry_delay`] and [`MAX_ATTEMPTS`] cap it at three attempts over 35 seconds
/// -- while the cost of being wrong in the other direction is a lost shot.
///
/// # What this does not protect
///
/// **A forged success still loses the shot.** An injected `HTTP/1.1 201 Created` makes the
/// uploader log "stored" and move on, and nothing a plaintext status line says can be
/// trusted to contradict it. Closing that needs the responder to answer inside the Noise
/// session; the shape is a `NoiseSender::open` and a single transport frame, deliberately
/// deferred. Until then this is a known, accepted gap against an on-path attacker who is
/// specifically targeting one machine.
pub fn temper(outcome: UploadOutcome, trust: ResponseTrust) -> UploadOutcome {
    match trust {
        ResponseTrust::Authenticated => outcome,
        ResponseTrust::Unauthenticated => match outcome {
            // Successes pass through unchanged -- not because they are trustworthy, but
            // because there is nothing safer to turn them into. See above.
            UploadOutcome::Created => UploadOutcome::Created,
            UploadOutcome::Duplicate => UploadOutcome::Duplicate,
            // Everything else becomes the one outcome that retries and then stops.
            UploadOutcome::Permanent
            | UploadOutcome::ServerError
            | UploadOutcome::RateLimited { .. } => UploadOutcome::ServerError,
        },
    }
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

    /// Every outcome `classify_status` can produce, so the match in `temper` cannot grow a
    /// hole when a variant is added.
    const EVERY_OUTCOME: [UploadOutcome; 5] = [
        UploadOutcome::Created,
        UploadOutcome::Duplicate,
        UploadOutcome::RateLimited { retry_after_secs: 30 },
        UploadOutcome::ServerError,
        UploadOutcome::Permanent,
    ];

    #[test]
    fn an_authenticated_outcome_is_never_changed() {
        for outcome in EVERY_OUTCOME {
            assert_eq!(temper(outcome, ResponseTrust::Authenticated), outcome);
        }
    }

    #[test]
    fn an_unauthenticated_refusal_becomes_transient() {
        // The forged-401 case. `Permanent` abandons the shot, and on this transport anyone
        // on path can write a 401.
        assert_eq!(
            temper(UploadOutcome::Permanent, ResponseTrust::Unauthenticated),
            UploadOutcome::ServerError
        );
    }

    #[test]
    fn an_unauthenticated_rate_limit_cannot_abandon_the_shot() {
        // A forged `Retry-After` above MAX_RETRY_AFTER_SECS is the other way to make the
        // uploader give up without the server saying anything.
        assert_eq!(
            temper(
                UploadOutcome::RateLimited { retry_after_secs: MAX_RETRY_AFTER_SECS + 1 },
                ResponseTrust::Unauthenticated
            ),
            UploadOutcome::ServerError
        );
    }

    #[test]
    fn no_unauthenticated_failure_is_terminal() {
        // The property that matters, stated once over the whole space rather than per
        // variant: nothing an attacker can write makes the uploader stop retrying early.
        for outcome in EVERY_OUTCOME {
            let tempered = temper(outcome, ResponseTrust::Unauthenticated);
            assert_ne!(tempered, UploadOutcome::Permanent, "{outcome:?} stayed terminal");
            if let UploadOutcome::RateLimited { retry_after_secs } = tempered {
                assert!(retry_after_secs <= MAX_RETRY_AFTER_SECS, "{outcome:?} can still abandon");
            }
        }
    }

    #[test]
    fn an_unauthenticated_success_still_passes_through() {
        // Documenting the known gap rather than pretending it is closed: a forged 201 is
        // still believed. See `temper`'s docs.
        assert_eq!(
            temper(UploadOutcome::Created, ResponseTrust::Unauthenticated),
            UploadOutcome::Created
        );
        assert_eq!(
            temper(UploadOutcome::Duplicate, ResponseTrust::Unauthenticated),
            UploadOutcome::Duplicate
        );
    }
}
