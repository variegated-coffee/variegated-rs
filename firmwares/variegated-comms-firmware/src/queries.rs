//! Answering a question that only the application processor can answer.
//!
//! Two transports ask the same three questions -- the local WebSocket server and the Plantlet
//! uplink -- and the answers have nothing to do with either. A routine's definition is
//! reassembled from chunks over the inter-processor link, a write is a round trip with a
//! five-way refusal, and both are the same work whoever asked.
//!
//! So this module owns the asking, and neither transport owns a copy of it. That is not
//! tidiness: the reassembly below closes a specific hazard (see [`serve_query`]), and a second
//! copy of it is a second place for that hazard to come back.
//!
//! # What lives here and what does not
//!
//! Here: the timeouts that bound the *machine's* turnaround, and the loop that reassembles a
//! reply. Not here: anything about a socket, a frame, or a correlation id -- those differ per
//! transport and are the callers' business. [`serve_query`] takes a question and returns an
//! answer, and has never heard of either transport.

use alloc::vec::Vec;
use embassy_time::{with_timeout, Duration};
use variegated_log::{log_error, log_info, log_warn};

use crate::channels::{
    routine_request, routine_write, shot_log_request, RoutineReply, ShotLogReply, ShotLogRequest,
};
use crate::ws_types::{ClientQuery, QueryError, QueryOk, QueryOutcome};

/// How long a routine read may take before the machine is declared unresponsive.
///
/// The same figures the HTTP route used, and deliberately the same across transports: they
/// bound the *machine's* turnaround, which does not change with who asked.
pub const ROUTINE_TIMEOUT: Duration = Duration::from_secs(5);
/// Longer than a read, because a write is several chunks out plus a flash erase.
pub const ROUTINE_WRITE_TIMEOUT: Duration = Duration::from_secs(10);
pub const SHOT_LOG_TIMEOUT: Duration = Duration::from_secs(5);

/// Await a query while keeping the caller's check-in row honest.
///
/// **This exists because of a mismatch between two timeouts, not because of anything the
/// query does.** `variegated_checkin::HEARTBEAT` is 5 s and a caller's own loop is not running
/// while a query is being served. A routine write may take [`ROUTINE_WRITE_TIMEOUT`], which is
/// 10 s, so a perfectly healthy save would take that caller's row amber and then red. That is
/// a false alarm on the one table an operator consults to find out what is wedged, and a table
/// that cries wolf is worse than no table.
///
/// The future is created once and polled through `&mut`, never dropped and rebuilt, so this is
/// cancel-safe by construction -- which matters here more than usual: `routine_write` holds
/// `ROUTINE_LOCK` across a multi-chunk write, and dropping it midway would release the lock
/// with the far side still mid-sequence.
pub async fn await_query<F: core::future::Future>(
    future: F,
    checkin: &variegated_checkin::CheckinHandle,
) -> F::Output {
    let mut future = core::pin::pin!(future);
    loop {
        match with_timeout(variegated_checkin::HEARTBEAT / 2, &mut future).await {
            Ok(output) => return output,
            Err(_) => checkin.good(),
        }
    }
}

/// Serve a [`ClientQuery`], and say what happened.
///
/// Every arm answers -- there is no path that leaves a caller waiting for a reply that never
/// comes, because the id it correlates on would then leak until its own timeout fired.
pub async fn serve_query(
    query: ClientQuery,
    checkin: &variegated_checkin::CheckinHandle,
) -> QueryOutcome {
    match query {
        // Reassembled here rather than streamed, and that is a fix rather than a compromise.
        // The HTTP route wrote each chunk into an already-committed 200 as it arrived, taking
        // `ROUTINE_LOCK` per chunk -- so a save landing between two of them spliced an old
        // head onto a new tail, and postcard being positional, the result decoded into a
        // routine nobody wrote. Only a changed encoded length caught it. Buffering the whole
        // thing means one lock-consistent answer or none.
        ClientQuery::RoutineDefinition(index) => {
            let mut body: Vec<u8> = Vec::new();
            let mut offset: u16 = 0;

            loop {
                match await_query(routine_request(index, offset, ROUTINE_TIMEOUT), checkin).await {
                    Ok(RoutineReply::Chunk { offset: reply_offset, bytes, last, .. }) => {
                        // The link carries no correlation id; the echoed offset is the only
                        // evidence this chunk answers this request.
                        if reply_offset != offset {
                            log_error!("Routine chunk out of order: wanted {}, got {}", offset, reply_offset);
                            return QueryOutcome::Failed(QueryError::Unavailable);
                        }
                        if bytes.is_empty() && !last {
                            log_error!("Routine chunk was empty before the last one");
                            return QueryOutcome::Failed(QueryError::Unavailable);
                        }
                        offset += bytes.len() as u16;
                        body.extend_from_slice(&bytes);
                        if last {
                            return QueryOutcome::Ok(QueryOk::RoutineDefinition(body));
                        }
                    }
                    Ok(RoutineReply::NotFound(_)) => {
                        return QueryOutcome::Failed(QueryError::NotFound);
                    }
                    Ok(_) => {
                        log_error!("Routine fetch: unexpected reply kind");
                        return QueryOutcome::Failed(QueryError::Unavailable);
                    }
                    Err(_) => {
                        log_error!("Routine fetch timed out at offset {}", offset);
                        return QueryOutcome::Failed(QueryError::Unavailable);
                    }
                }
            }
        }

        // The bytes go to the application processor exactly as the caller sent them. Nothing
        // here decodes a `Routine` -- see `EncodedPayload` -- so `Malformed` keeps meaning
        // "what arrived is not a routine" rather than "the middle hop re-encoded it
        // differently".
        ClientQuery::WriteRoutine { index, routine } => {
            match await_query(routine_write(index, routine, ROUTINE_WRITE_TIMEOUT), checkin).await {
                Ok(variegated_controller_types::RoutineWriteOutcome::Stored(stored)) => {
                    log_info!("Routine stored");
                    QueryOutcome::Ok(QueryOk::RoutineStored(stored))
                }
                Ok(variegated_controller_types::RoutineWriteOutcome::Failed(error)) => {
                    log_warn!("Routine write refused");
                    QueryOutcome::Failed(QueryError::RoutineWrite(error))
                }
                Err(_) => {
                    log_error!("Routine write got no answer");
                    QueryOutcome::Failed(QueryError::Unavailable)
                }
            }
        }

        ClientQuery::ShotLogPage(request) => {
            match await_query(shot_log_request(ShotLogRequest::List(request), SHOT_LOG_TIMEOUT), checkin).await {
                Ok(ShotLogReply::List(list)) => QueryOutcome::Ok(QueryOk::ShotLogPage(list)),
                Ok(ShotLogReply::Error(error)) => {
                    log_warn!("Shot log listing refused");
                    QueryOutcome::Failed(QueryError::ShotLogStorage(error))
                }
                Ok(_) => {
                    log_error!("Shot log listing: unexpected reply kind");
                    QueryOutcome::Failed(QueryError::Unavailable)
                }
                Err(_) => {
                    log_error!("Shot log listing got no answer");
                    QueryOutcome::Failed(QueryError::Unavailable)
                }
            }
        }
    }
}
