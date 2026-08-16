//! HTTP server functionality

use alloc::vec;
use alloc::vec::Vec;
use core::fmt::{Debug, Display};
use core::net::SocketAddr;

// Embedded frontend files. `../frontend`, not `../../../frontend`: this crate used to be
// `crates/variegated-comms-firmware` in a repo whose root held `frontend/`, and it is now
// a workspace member sitting beside its own copy. `build.rs` runs `npm run build` to
// produce these, so a failure here usually means that step was skipped
// (`VARIEGATED_FRONTEND_SKIP`) or `npm ci` has never been run in `frontend/`.
static INDEX_HTML: &[u8] = include_bytes!("../frontend/dist/index.html");
static APP_JS_GZ: &[u8] = include_bytes!("../frontend/dist/assets/index.js.gz");

// No `log_warn`: this file's only `warn!` site is the one at line ~1293 that had to
// stay on `defmt`, because `defmt::Debug2Format` implements `Debug` but not
// `Display` and that site formats it with `{}`.
use variegated_log::{log_error, log_info};
use edge_http::io::server::{
    Connection as ServerConnection, DEFAULT_BUF_SIZE, Handler, Server,
};
use edge_http::DEFAULT_MAX_HEADERS_COUNT;

/// The HTTP server, with its handler-task count pinned to the size of the socket pool
/// in `main.rs`. See the note at its construction in `http_server_task`; these two
/// numbers are one decision and have to move together.
type HttpServer = Server<2, DEFAULT_BUF_SIZE, DEFAULT_MAX_HEADERS_COUNT>;

/// How long to wait for the application processor to answer a shot-log request.
///
/// Generous, because the answer comes off an SD card behind a bus lease the display also
/// wants, and `BUS_LEASE_TIMEOUT` alone is two seconds. Finite, because this blocks one
/// of only two HTTP handler slots, so an unbounded wait would let a wedged application
/// processor take the web UI down with it.
const SHOT_LOG_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_secs(5);

/// How long to wait for one chunk of a routine.
///
/// The same five seconds as a shot-log request, and for the same reasons -- this occupies
/// an HTTP handler slot, and the answer comes from a repository that may be reading flash.
/// It is per *chunk* rather than per routine, but a routine is at most two chunks.
const ROUTINE_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_secs(5);

/// How long to wait for a routine write to be stored.
///
/// Longer than a read: the application processor has to receive every chunk, decode, and
/// then write flash, and a flash write behind a contended bus is the slowest thing on
/// either processor.
const ROUTINE_WRITE_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_secs(10);

/// The largest routine body this server will read.
///
/// `ROUTINE_MAX_ENCODED_LEN` plus slack, because that is the ceiling the far side stores
/// through -- a larger body could not be saved whatever this said, so reading it would be
/// work spent on its way to a refusal.
///
/// This was 16384, and `read_body` allocates `vec![0u8; max_size]` **up front**: every
/// routine save took a 16 kB transient off a 56 kB heap shared with Wi-Fi, BLE and the
/// ESPHome server, to carry at most 2 kB.
const ROUTINE_BODY_LIMIT: usize = ROUTINE_MAX_ENCODED_LEN + 256;

/// Turn a refusal from the application processor into something worth showing a user.
///
/// Each of these is a different action on the user's part, which is the whole reason
/// `ShotLogError` was added to the wire rather than letting every failure time out into
/// one indistinguishable "not responding".
fn shot_log_error_message(error: ShotLogStorageError) -> &'static str {
    match error {
        ShotLogStorageError::CardNotPresent => "No SD card in the machine",
        ShotLogStorageError::NotExfat => "The SD card is not formatted exFAT",
        ShotLogStorageError::BusUnavailable => "The machine is busy; try again",
        ShotLogStorageError::NotFound => "No such shot",
        ShotLogStorageError::CrcError => "That shot is corrupt on the card",
        // Not corruption, and worth saying so: the file is intact, it was just written in
        // a format this firmware does not read. Nothing the user can do about it, but
        // "corrupt" would send them looking for a damaged card.
        ShotLogStorageError::UnsupportedVersion => {
            "That shot was recorded by a different firmware version"
        }
        ShotLogStorageError::ReadError
        | ShotLogStorageError::WriteError
        | ShotLogStorageError::SerializationError
        | ShotLogStorageError::DirectoryError => "The machine could not read the SD card",
    }
}
use edge_http::io::Error;
use edge_http::Method;
use edge_nal::TcpBind;
use edge_nal_embassy::Tcp;
use embedded_io_async::{Read, Write};
use embassy_futures::select::{select, Either};

// No `Routine` here, and that is the property this module is meant to have: a routine
// definition passes through this server as bytes in both directions. The types it does
// name are the summary (for the listing), the index (for addressing) and the write
// outcome (for the status code).
use variegated_controller_types::{
    BoilerControlTargetValuesUpdate, GroupBrewControlTargetValuesUpdate,
    MachineCommand, MachineMode, PidParameterTarget, RoutineIndex, RoutineWriteError,
    RoutineWriteOutcome, ScaleSelector, ScheduleItem, ROUTINE_MAX_ENCODED_LEN,
};
use variegated_controller_types::shot_log::{
    ShotAnnotations, ShotLogDayFilter, ShotLogId, ShotLogListRequest, ShotLogStorageError,
    SHOT_LOG_PAGE_LEN,
};

use crate::api_types::{
    RoutineSummaryStorage, SetBoilerControlRequest, SetFillPumpConfigurationRequest,
    SetGroupControlRequest, SetGroupPumpConfigurationRequest, SetPidParametersRequest,
    SetShotUploadSettingsRequest, SetSteamValveOpennessRequest,
    SetWaterTapPumpConfigurationRequest,
};
use crate::channels::{
    routine_request, routine_write, shot_log_request, ApplicationConfigurationSubscriber,
    ApplicationStatusSubscriber, MachineCommandSender, RoutineReply, ShotLogReply,
    ShotLogRequest, CONFIG_CACHE, MACHINE_DEFINITION, ROUTINE_CACHE, STATUS_CACHE,
};

/// HTTP request handler
pub struct HttpHandler {
    command_sender: &'static MachineCommandSender,
}

impl HttpHandler {
    pub fn new(command_sender: &'static MachineCommandSender) -> Self {
        Self { command_sender }
    }

    // Helper to send a simple response
    async fn send_response<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        status: u16,
        reason: &str,
        content_type: &str,
        body: &[u8],
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        conn.initiate_response(status, Some(reason), &[("Content-Type", content_type)])
            .await?;
        conn.write_all(body).await?;
        Ok(())
    }

    // Helper for binary (postcard) responses
    async fn send_binary<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        data: &[u8],
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        Self::send_response(conn, 200, "OK", "application/octet-stream", data).await
    }

    // Helper for text responses
    async fn send_text<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        status: u16,
        reason: &str,
        body: &str,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        Self::send_response(conn, status, reason, "text/plain", body.as_bytes()).await
    }

    // Helper for 404 response
    async fn send_not_found<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        Self::send_text(conn, 404, "Not Found", "Not found").await
    }

    // Helper for 503 response (service unavailable)
    async fn send_unavailable<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        message: &str,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        Self::send_text(conn, 503, "Service Unavailable", message).await
    }

    // Helper for 400 response (bad request)
    async fn send_bad_request<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        message: &str,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        Self::send_text(conn, 400, "Bad Request", message).await
    }

    // Helper for 500 response (internal server error)
    async fn send_internal_error<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        message: &str,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        Self::send_text(conn, 500, "Internal Server Error", message).await
    }

    // Read request body into a Vec
    async fn read_body<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        max_size: usize,
    ) -> Result<Vec<u8>, Error<T::Error>>
    where
        T: Read + Write,
    {
        let mut buf = vec![0u8; max_size];
        let mut total_read = 0;

        loop {
            match conn.read(&mut buf[total_read..]).await {
                Ok(0) => break,
                Ok(n) => {
                    total_read += n;
                    if total_read >= max_size {
                        break;
                    }
                }
                Err(e) => return Err(e),
            }
        }

        buf.truncate(total_read);
        Ok(buf)
    }

    // GET /status
    async fn handle_get_status<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("GET /status");

        let status = STATUS_CACHE.lock().await;
        if let Some(ref s) = *status {
            match postcard::to_allocvec(s) {
                Ok(binary) => {
                    drop(status);
                    Self::send_binary(conn, &binary).await
                }
                Err(e) => {
                    log_error!("Failed to serialize status: {:?}", defmt::Debug2Format(&e));
                    Self::send_internal_error(conn, "Failed to serialize status").await
                }
            }
        } else {
            Self::send_unavailable(conn, "Status not yet available").await
        }
    }

    // GET /configuration
    async fn handle_get_configuration<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("GET /configuration");

        let config = CONFIG_CACHE.lock().await;
        if let Some(ref c) = *config {
            match postcard::to_allocvec(c) {
                Ok(binary) => {
                    drop(config);
                    Self::send_binary(conn, &binary).await
                }
                Err(e) => {
                    log_error!("Failed to serialize configuration: {:?}", defmt::Debug2Format(&e));
                    Self::send_internal_error(conn, "Failed to serialize configuration").await
                }
            }
        } else {
            Self::send_unavailable(conn, "Configuration not yet available").await
        }
    }

    // GET /machine-definition
    async fn handle_get_machine_definition<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("GET /machine-definition");

        let machine_def = MACHINE_DEFINITION.lock().await;
        if let Some(ref md) = *machine_def {
            match postcard::to_allocvec(md) {
                Ok(binary) => {
                    drop(machine_def);
                    Self::send_binary(conn, &binary).await
                }
                Err(e) => {
                    log_error!("Failed to serialize machine definition: {:?}", defmt::Debug2Format(&e));
                    Self::send_internal_error(conn, "Failed to serialize machine definition").await
                }
            }
        } else {
            Self::send_unavailable(conn, "Machine definition not yet available").await
        }
    }

    /// `{type}/{index}` from a routine path, or `None` if either half is not one.
    ///
    /// The three type names are the same three the WebSocket and the frontend use, and
    /// they are the *index kind* -- `RoutineIndex`'s variants -- not `RoutineType`, which
    /// is a separate axis carried inside the routine itself.
    fn parse_routine_index(routine_type: &str, index: &str) -> Option<RoutineIndex> {
        let index: u32 = index.parse().ok()?;
        match routine_type {
            "internal" => Some(RoutineIndex::Internal(index)),
            "function" => Some(RoutineIndex::Function(index)),
            "custom" => Some(RoutineIndex::Custom(index)),
            _ => None,
        }
    }

    // GET /routines
    //
    // Summaries only -- names, types and counts. A definition comes from
    // `GET /routines/{type}/{index}`, one at a time, and is never held on this processor.
    async fn handle_get_routines<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("GET /routines");

        let routine_list = ROUTINE_CACHE.lock().await;
        if let Some(ref summaries) = *routine_list {
            let storage = RoutineSummaryStorage::from_list(summaries);

            match postcard::to_allocvec(&storage) {
                Ok(binary) => {
                    drop(routine_list);
                    Self::send_binary(conn, &binary).await
                }
                Err(e) => {
                    log_error!("Failed to serialize routines: {:?}", defmt::Debug2Format(&e));
                    Self::send_internal_error(conn, "Failed to serialize routines").await
                }
            }
        } else {
            Self::send_unavailable(conn, "Routines not yet available").await
        }
    }

    // GET /routines/<type>/<index>
    //
    // Streams the routine's postcard encoding, as the application processor stored it.
    // **Nothing here decodes it.** The bytes arrive from the link in chunks and go
    // straight onto the socket, so this handler holds one kilobyte at a time rather than
    // a `Routine` -- which is five to ten allocations per step, on the heap Wi-Fi and BLE
    // are sharing. It is the same trade `handle_get_shot` makes, for the same reason.
    async fn handle_get_routine<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        index: RoutineIndex,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("GET one routine");

        // The first chunk is fetched *before* the response is started, and that ordering
        // is the whole error-handling story: `initiate_response` commits to a status code,
        // after which "no such routine" can only be expressed by hanging up mid-body.
        let first = match routine_request(index, 0, ROUTINE_TIMEOUT).await {
            Ok(RoutineReply::Chunk { bytes, last, total, .. }) => (bytes, last, total),
            Ok(RoutineReply::NotFound(_)) => {
                return Self::send_not_found(conn).await;
            }
            Ok(_) => {
                log_error!("Routine fetch: unexpected reply kind");
                return Self::send_internal_error(conn, "Unexpected routine reply").await;
            }
            Err(_) => {
                return Self::send_unavailable(conn, "Machine did not answer in time").await;
            }
        };
        let (first_bytes, mut last, total) = first;

        conn.initiate_response(200, Some("OK"), &[("Content-Type", "application/octet-stream")])
            .await?;
        conn.write_all(&first_bytes).await?;

        let mut offset = first_bytes.len() as u16;
        while !last {
            match routine_request(index, offset, ROUTINE_TIMEOUT).await {
                Ok(RoutineReply::Chunk {
                    index: reply_index,
                    offset: reply_offset,
                    total: reply_total,
                    bytes,
                    last: is_last,
                }) => {
                    // The echo check. `ROUTINE_LOCK` should make a mismatch impossible,
                    // but splicing one routine's bytes into another's body is a corruption
                    // no client could detect -- postcard is positional, so the result
                    // would decode into a routine nobody wrote rather than fail.
                    //
                    // `total` is compared as well as the address, and that one catches a
                    // different failure: the lock is held per chunk, not for the whole
                    // download, so a save landing between two chunks would otherwise
                    // splice the head of the old routine onto the tail of the new. An
                    // edit that leaves the encoded length exactly unchanged still slips
                    // through -- but this client serialises its own writes against its own
                    // reads, so that needs a second browser editing the same routine
                    // during the download.
                    if reply_index != index || reply_offset != offset || reply_total != total {
                        log_error!("Routine chunk: reply does not match the request");
                        break;
                    }
                    if bytes.is_empty() {
                        break;
                    }
                    conn.write_all(&bytes).await?;
                    offset += bytes.len() as u16;
                    last = is_last;
                }
                // Past this point the status is already sent, so a failure can only be
                // expressed by ending the body early. Logged with the offset so a
                // truncated response is diagnosable from the device.
                Ok(_) => {
                    log_error!("Routine download failed at offset {}", offset);
                    break;
                }
                Err(_) => {
                    log_error!("Routine download timed out at offset {}", offset);
                    break;
                }
            }
        }

        Ok(())
    }

    // POST /schedules - Add new schedule
    async fn handle_post_schedule<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /schedules");

        let body = Self::read_body(conn, 8192).await?;

        let schedule_item: ScheduleItem = match postcard::from_bytes(&body) {
            Ok(item) => item,
            Err(e) => {
                log_error!("Failed to deserialize ScheduleItem: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd = MachineCommand::AddScheduleItem(schedule_item);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Schedule add command sent");
                Self::send_text(conn, 201, "Created", "Schedule added").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // PUT /schedules/{index} - Update schedule
    async fn handle_put_schedule<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        index: u32,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("PUT /schedules/{}", index);

        let body = Self::read_body(conn, 8192).await?;

        let schedule_item: ScheduleItem = match postcard::from_bytes(&body) {
            Ok(item) => item,
            Err(e) => {
                log_error!("Failed to deserialize ScheduleItem: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd = MachineCommand::UpdateScheduleItem(index, schedule_item);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Schedule update command sent for index {}", index);
                Self::send_text(conn, 200, "OK", "Schedule updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // DELETE /schedules/{index} - Remove schedule
    async fn handle_delete_schedule<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        index: u32,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("DELETE /schedules/{}", index);

        let cmd = MachineCommand::RemoveScheduleItem(index);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Schedule delete command sent for index {}", index);
                conn.initiate_response(204, Some("No Content"), &[]).await?;
                Ok(())
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /routines/{type} or POST /routines/{type}/{index}
    async fn handle_post_routine<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        path: &str,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /routines/{}", path);

        let parts: Vec<&str> = path.split('/').filter(|s| !s.is_empty()).collect();

        if parts.is_empty() || parts.len() > 2 {
            return Self::send_bad_request(conn, "Invalid path format").await;
        }

        let routine_type = parts[0];

        // The target index is decided before the body is read, so a malformed path costs
        // nothing. `None` means "let the repository assign one", which is what makes a
        // custom routine's index come back in the response.
        let target = match routine_type {
            "custom" => {
                if parts.len() > 1 {
                    return Self::send_bad_request(
                        conn,
                        "Custom routines auto-assign index. Use /routines/custom without index.",
                    )
                    .await;
                }
                None
            }
            "function" => {
                if parts.len() != 2 {
                    return Self::send_bad_request(
                        conn,
                        "Function routines require index. Use /routines/function/{index}",
                    )
                    .await;
                }
                let index: u32 = match parts[1].parse() {
                    Ok(idx) => idx,
                    Err(_) => {
                        return Self::send_bad_request(conn, "Invalid index").await;
                    }
                };
                Some(RoutineIndex::Function(index))
            }
            "internal" => {
                return Self::send_bad_request(conn, "Internal routines cannot be created via API")
                    .await;
            }
            _ => {
                return Self::send_bad_request(
                    conn,
                    "Invalid routine type: must be 'custom' or 'function'",
                )
                .await;
            }
        };

        let body = Self::read_body(conn, ROUTINE_BODY_LIMIT).await?;
        Self::write_routine(conn, target, body).await
    }

    /// Hand a routine's bytes to the application processor and answer with what happened.
    ///
    /// **The bytes are not decoded here.** They came off the socket as postcard and go
    /// onto the link as postcard; building a `Routine` in between would take it apart and
    /// put it back together identically, on the processor with the least memory to do it
    /// on. The far side has the flash, the 2 kB buffer and the room.
    ///
    /// The cost of not decoding is that a malformed body is diagnosed one hop later --
    /// which is why `RoutineWriteError::Malformed` exists and comes back as a 400 rather
    /// than a generic failure.
    async fn write_routine<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
        index: Option<RoutineIndex>,
        body: Vec<u8>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        match routine_write(index, body, ROUTINE_WRITE_TIMEOUT).await {
            Ok(RoutineWriteOutcome::Stored(stored)) => {
                log_info!("Routine stored");
                // The index comes back in the body because on a create the *repository*
                // chose it -- a client that posts a new routine has no other way to learn
                // where it landed.
                match postcard::to_allocvec(&stored) {
                    Ok(binary) => Self::send_binary(conn, &binary).await,
                    Err(_) => Self::send_internal_error(conn, "Failed to serialize routine index").await,
                }
            }
            Ok(RoutineWriteOutcome::Failed(RoutineWriteError::Malformed)) => {
                Self::send_bad_request(conn, "Invalid postcard data").await
            }
            Ok(RoutineWriteOutcome::Failed(RoutineWriteError::TooLarge)) => {
                Self::send_bad_request(conn, "Routine is too large to store").await
            }
            Ok(RoutineWriteOutcome::Failed(RoutineWriteError::Immutable)) => {
                Self::send_bad_request(conn, "Internal routines are read-only").await
            }
            Ok(RoutineWriteOutcome::Failed(RoutineWriteError::Storage)) => {
                Self::send_internal_error(conn, "The machine could not store the routine").await
            }
            Err(_) => Self::send_unavailable(conn, "Machine did not answer in time").await,
        }
    }

    // PUT /routines/{type}/{index}
    async fn handle_put_routine<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        routine_type: &str,
        index: u32,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("PUT /routines/{}/{}", routine_type, index);

        let routine_index = match routine_type {
            "internal" => RoutineIndex::Internal(index),
            "function" => RoutineIndex::Function(index),
            "custom" => RoutineIndex::Custom(index),
            _ => {
                return Self::send_bad_request(
                    conn,
                    "Invalid routine type: must be 'internal', 'function', or 'custom'",
                )
                .await;
            }
        };

        let body = Self::read_body(conn, ROUTINE_BODY_LIMIT).await?;
        Self::write_routine(conn, Some(routine_index), body).await
    }

    // DELETE /routines/{type}/{index}
    async fn handle_delete_routine<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        routine_type: &str,
        index: u32,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("DELETE /routines/{}/{}", routine_type, index);

        let routine_index = match routine_type {
            "internal" => RoutineIndex::Internal(index),
            "function" => RoutineIndex::Function(index),
            "custom" => RoutineIndex::Custom(index),
            _ => {
                return Self::send_bad_request(
                    conn,
                    "Invalid routine type: must be 'internal', 'function', or 'custom'",
                )
                .await;
            }
        };

        let cmd = MachineCommand::RemoveRoutine(routine_index);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Routine delete command sent");
                conn.initiate_response(204, Some("No Content"), &[]).await?;
                Ok(())
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/run-routine/{type}/{index}
    async fn handle_run_routine<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        routine_type: &str,
        index: u32,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/run-routine/{}/{}", routine_type, index);

        let routine_index = match routine_type {
            "internal" => RoutineIndex::Internal(index),
            "function" => RoutineIndex::Function(index),
            "custom" => RoutineIndex::Custom(index),
            _ => {
                return Self::send_bad_request(
                    conn,
                    "Invalid routine type: must be 'internal', 'function', or 'custom'",
                )
                .await;
            }
        };

        let cmd = MachineCommand::RunRoutine(routine_index, None);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Run routine command sent");
                Self::send_text(conn, 200, "OK", "Routine execution started").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/cancel-routine
    async fn handle_cancel_routine<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/cancel-routine");

        let cmd = MachineCommand::CancelRoutine;
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Cancel routine command sent");
                Self::send_text(conn, 200, "OK", "Routine cancelled").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/tare-group-scale/{index}
    async fn handle_tare_group_scale<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        index: u8,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/tare-group-scale/{}", index);

        let cmd = MachineCommand::TareGroupScale(index);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Tare group scale command sent");
                Self::send_text(conn, 200, "OK", "Group scale tared").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/zero-calibrate-group-scale/{index}
    async fn handle_zero_calibrate_group_scale<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        index: u8,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/zero-calibrate-group-scale/{}", index);

        let cmd = MachineCommand::ZeroCalibrateGroupScale(index);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Zero calibrate group scale command sent");
                Self::send_text(conn, 200, "OK", "Group scale zero calibrated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/calibrate-group-scale-100g/{index}
    async fn handle_calibrate_group_scale_100g<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        index: u8,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/calibrate-group-scale-100g/{}", index);

        let cmd = MachineCommand::CalibrateGroupScale100g(index);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Calibrate group scale 100g command sent");
                Self::send_text(conn, 200, "OK", "Group scale calibrated with 100g").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-mode/{mode}
    async fn handle_set_mode<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        mode_str: &str,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-mode/{}", mode_str);

        let mode = match mode_str.to_lowercase().as_str() {
            "on" => MachineMode::On,
            "off" => MachineMode::Off,
            "powersavestandby" | "power-save-standby" => MachineMode::PowerSaveStandby,
            _ => {
                return Self::send_bad_request(
                    conn,
                    "Invalid mode: must be 'on', 'off', or 'powersavestandby'",
                )
                .await;
            }
        };

        let cmd = MachineCommand::SetMachineMode(mode);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Set machine mode command sent");
                Self::send_text(conn, 200, "OK", "Machine mode updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-boiler-control
    async fn handle_set_boiler_control<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-boiler-control");

        let body = Self::read_body(conn, 512).await?;

        let req: SetBoilerControlRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!("Failed to deserialize SetBoilerControlRequest: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd = if req.target_temperature.is_some() || req.target_pressure.is_some() {
            MachineCommand::SetBoilerControlTarget(
                req.boiler_index,
                req.mode,
                Some(BoilerControlTargetValuesUpdate {
                    temperature: req.target_temperature,
                    pressure: req.target_pressure,
                }),
            )
        } else {
            MachineCommand::SetBoilerControlTarget(req.boiler_index, req.mode, None)
        };

        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Boiler control command sent");
                Self::send_text(conn, 200, "OK", "Boiler control updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-group-control
    async fn handle_set_group_control<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-group-control");

        let body = Self::read_body(conn, 512).await?;

        let req: SetGroupControlRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!("Failed to deserialize SetGroupControlRequest: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let has_values = req.flow_rate.is_some()
            || req.pressure.is_some()
            || req.output_flow_rate.is_some()
            || req.duty_cycle.is_some()
            || req.flow_rate_curve.is_some()
            || req.pressure_curve.is_some()
            || req.output_flow_rate_curve.is_some()
            || req.duty_cycle_curve.is_some();

        let cmd = if has_values {
            MachineCommand::SetGroupBrewControlTarget(
                req.group_index,
                req.mode,
                Some(GroupBrewControlTargetValuesUpdate {
                    flow_rate: req.flow_rate,
                    flow_rate_curve: req.flow_rate_curve,
                    pressure: req.pressure,
                    pressure_curve: req.pressure_curve,
                    output_flow_rate: req.output_flow_rate,
                    output_flow_rate_curve: req.output_flow_rate_curve,
                    duty_cycle: req.duty_cycle,
                    duty_cycle_curve: req.duty_cycle_curve,
                }),
            )
        } else {
            MachineCommand::SetGroupBrewControlTarget(req.group_index, req.mode, None)
        };

        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Group control command sent");
                Self::send_text(conn, 200, "OK", "Group control updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-pid-parameters
    async fn handle_set_pid_parameters<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-pid-parameters");

        let body = Self::read_body(conn, 1024).await?;

        let req: SetPidParametersRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!("Failed to deserialize SetPidParametersRequest: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let target = match req.target_type.as_str() {
            "BoilerTemperature" => PidParameterTarget::BoilerTemperature(req.index as u8),
            "BoilerPressure" => PidParameterTarget::BoilerPressure(req.index as u8),
            "GroupFlowRate" => PidParameterTarget::GroupFlowRate(req.index as u8),
            "GroupOutputFlowRate" => PidParameterTarget::GroupOutputFlowRate(req.index as u8),
            "GroupPressure" => PidParameterTarget::GroupPressure(req.index as u8),
            _ => {
                return Self::send_bad_request(
                    conn,
                    "Invalid target_type",
                )
                .await;
            }
        };

        let cmd = MachineCommand::SetPidParameters(target, req.pid_parameters);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("SetPidParameters command sent");
                Self::send_text(conn, 200, "OK", "PID parameters updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-group-pump-configuration
    async fn handle_set_group_pump_configuration<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-group-pump-configuration");

        let body = Self::read_body(conn, 512).await?;

        let req: SetGroupPumpConfigurationRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!("Failed to deserialize SetGroupPumpConfigurationRequest: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd = MachineCommand::SetGroupPumpConfiguration(req.group_index, req.pump_configuration);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("SetGroupPumpConfiguration command sent");
                Self::send_text(conn, 200, "OK", "Group pump configuration updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-water-tap-pump-configuration
    async fn handle_set_water_tap_pump_configuration<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-water-tap-pump-configuration");

        let body = Self::read_body(conn, 512).await?;

        let req: SetWaterTapPumpConfigurationRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!("Failed to deserialize SetWaterTapPumpConfigurationRequest: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd = MachineCommand::SetWaterTapPumpConfiguration(
            req.water_tap_index,
            req.pump_configuration,
        );
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("SetWaterTapPumpConfiguration command sent");
                Self::send_text(conn, 200, "OK", "Water tap pump configuration updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-fill-pump-configuration
    async fn handle_set_fill_pump_configuration<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-fill-pump-configuration");

        let body = Self::read_body(conn, 512).await?;

        let req: SetFillPumpConfigurationRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!("Failed to deserialize SetFillPumpConfigurationRequest: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd =
            MachineCommand::SetFillPumpConfiguration(req.boiler_index, req.pump_configuration);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("SetFillPumpConfiguration command sent");
                Self::send_text(conn, 200, "OK", "Fill pump configuration updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-steam-valve-openness
    async fn handle_set_steam_valve_openness<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-steam-valve-openness");

        let body = Self::read_body(conn, 512).await?;

        let req: SetSteamValveOpennessRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!("Failed to deserialize SetSteamValveOpennessRequest: {:?}", defmt::Debug2Format(&e));
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        let cmd = MachineCommand::SetSteamValveOpenness(req.steam_wand_index, req.openness);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("SetSteamValveOpenness command sent");
                Self::send_text(conn, 200, "OK", "Steam valve openness updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/set-shot-upload-settings
    //
    // The one settings write the SPA cannot send over the WebSocket: a maximal payload is
    // ~326 bytes and `websocket.rs`'s inbound frame buffer is a fixed 256, over which it
    // closes the connection rather than erroring. See `SetShotUploadSettingsRequest`.
    //
    // **This route accepts a bearer token on an unauthenticated server.** That is not new --
    // every `/command/*` route here is equally open, and the machine is expected to be on a
    // trusted network -- but it is worth knowing that this one carries a credential rather
    // than a setpoint. The read-back path deliberately does not return it; see
    // `ShotUploadView::token_set`.
    async fn handle_set_shot_upload_settings<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/set-shot-upload-settings");

        // 512, not the 326-byte maximum: postcard's framing and any future field would
        // otherwise sit one byte from a 400 that reads as "malformed" rather than "too big".
        let body = Self::read_body(conn, 512).await?;

        let req: SetShotUploadSettingsRequest = match postcard::from_bytes(&body) {
            Ok(r) => r,
            Err(e) => {
                log_error!(
                    "Failed to deserialize SetShotUploadSettingsRequest: {:?}",
                    defmt::Debug2Format(&e)
                );
                return Self::send_bad_request(conn, "Invalid postcard data").await;
            }
        };

        // No log of the settings themselves. `ShotUploadSettings` elides the token in its
        // own `Format` impl, but this line would be the place someone reached past it.
        let cmd = MachineCommand::SetShotUploadSettings(req.settings);
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("SetShotUploadSettings command sent");
                Self::send_text(conn, 200, "OK", "Shot upload settings updated").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/optimize-routine-storage
    async fn handle_optimize_routine_storage<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/optimize-routine-storage");

        let cmd = MachineCommand::OptimizeRoutineStorage;
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Optimize routine storage command sent");
                Self::send_text(conn, 200, "OK", "Routine storage optimization started").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/optimize-schedule-storage
    async fn handle_optimize_schedule_storage<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/optimize-schedule-storage");

        let cmd = MachineCommand::OptimizeScheduleStorage;
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Optimize schedule storage command sent");
                Self::send_text(conn, 200, "OK", "Schedule storage optimization started").await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/optimize-configuration-storage
    async fn handle_optimize_configuration_storage<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/optimize-configuration-storage");

        let cmd = MachineCommand::OptimizeConfigurationStorage;
        match self.command_sender.try_send(cmd) {
            Ok(_) => {
                log_info!("Optimize configuration storage command sent");
                Self::send_text(conn, 200, "OK", "Configuration storage optimization started")
                    .await
            }
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // Parse path parameters from a path like /schedules/123
    fn parse_path_index(path: &str, prefix: &str) -> Option<u32> {
        let index_str = path.strip_prefix(prefix)?;
        index_str.parse().ok()
    }

    /// Eight digits, exactly.
    ///
    /// Checked for width rather than merely parsed, because the card stores a time
    /// zero-padded: a shot filed as `00000042` must not be addressable as `42`.
    fn parse_shot_time(time: &str) -> Option<u32> {
        if time.len() != 8 || !time.bytes().all(|b| b.is_ascii_digit()) {
            return None;
        }
        time.parse().ok()
    }

    /// Split `<day>/<time>[/tail]` into the shot it names and whatever follows.
    ///
    /// One parser for every route that names a shot -- the download, the annotation edit,
    /// the delete and the paging cursor -- so they cannot disagree about what a valid id
    /// looks like.
    ///
    /// The day component goes through [`ShotLogId::parse_dir_name`], the same function
    /// the storage layer uses when walking the card, so `NODATE` is accepted here exactly
    /// where it is accepted there and a stray directory is rejected in both places.
    fn parse_shot_id(rest: &str) -> Option<(ShotLogId, &str)> {
        let (day_str, tail) = rest.split_once('/')?;
        let day = ShotLogId::parse_dir_name(day_str)?;

        let (time_str, remainder) = match tail.split_once('/') {
            Some((time, remainder)) => (time, remainder),
            None => (tail, ""),
        };
        let time = Self::parse_shot_time(time_str)?;

        Some((ShotLogId { day, time }, remainder))
    }

    /// `/shots/<day>/<time>[/tail]`.
    fn parse_shot_path(path: &str) -> Option<(ShotLogId, &str)> {
        Self::parse_shot_id(path.strip_prefix("/shots/")?)
    }

    /// `<day>` or `<day>/before/<time>`, from a `/shots/day/…` route.
    ///
    /// The cursor carries only a time, because the route has already fixed the day; the
    /// full id is rebuilt from both so the storage layer compares the same thing it
    /// compares everywhere else.
    fn parse_day_page(rest: &str) -> Option<(ShotLogDayFilter, Option<ShotLogId>)> {
        let (day_str, tail) = match rest.split_once('/') {
            Some((day, tail)) => (day, tail),
            None => (rest, ""),
        };
        let day = ShotLogId::parse_dir_name(day_str)?;
        let filter = match day {
            Some(value) => ShotLogDayFilter::Day(value),
            None => ShotLogDayFilter::Undated,
        };

        if tail.is_empty() {
            return Some((filter, None));
        }
        let time = Self::parse_shot_time(tail.strip_prefix("before/")?)?;
        Some((filter, Some(ShotLogId { day, time })))
    }

    // GET /shots, /shots/before/…, /shots/day/…
    async fn handle_get_shots<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        day: ShotLogDayFilter,
        before: Option<ShotLogId>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("GET shots (day {:?}, before {:?})", day, before);

        // `SHOT_LOG_PAGE_LEN` rather than anything the client said. The count is the
        // server's, so there is no parameter to validate and no way for a client to ask
        // for a page the link cannot carry.
        let request = ShotLogListRequest {
            limit: SHOT_LOG_PAGE_LEN,
            before,
            day,
        };

        match shot_log_request(ShotLogRequest::List(request), SHOT_LOG_TIMEOUT).await {
            Ok(ShotLogReply::List(list)) => match postcard::to_allocvec(&list) {
                Ok(binary) => Self::send_binary(conn, &binary).await,
                Err(e) => {
                    log_error!("Failed to serialize shot log list: {:?}", defmt::Debug2Format(&e));
                    Self::send_internal_error(conn, "Failed to serialize shot log list").await
                }
            },
            Ok(ShotLogReply::Error(e)) => {
                log_info!("Shot log list refused: {:?}", e);
                Self::send_unavailable(conn, shot_log_error_message(e)).await
            }
            Ok(_) => {
                // The far side answered a different question. Unreachable while
                // `SHOT_LOG_LOCK` holds requests to one at a time; reported rather than
                // ignored so that if it ever does happen it is visible.
                log_error!("Shot log list: unexpected reply kind");
                Self::send_internal_error(conn, "Unexpected shot log reply").await
            }
            Err(_) => Self::send_unavailable(conn, "Machine did not answer in time").await,
        }
    }

    // GET /shots/<day>/<time>
    //
    // Streams the record as it sits on the card, byte for byte -- the download *is* the
    // stored file, so a host can decode it with the same code the firmware wrote it with.
    async fn handle_get_shot<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        id: ShotLogId,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("GET /shots/{}/{}", id.dir_name().as_str(), id.file_name().as_str());

        // The first chunk is fetched *before* the response is started, and that ordering
        // is the whole error-handling story: `initiate_response` commits to a status
        // code, after which "no card" can only be expressed by hanging up mid-body. Ask
        // first, and a missing shot is a clean 404 and an absent card a clean 503.
        let first = match shot_log_request(
            ShotLogRequest::Chunk { id, offset: 0 },
            SHOT_LOG_TIMEOUT,
        )
        .await
        {
            Ok(ShotLogReply::Chunk { bytes, total, last, .. }) => (bytes, total, last),
            Ok(ShotLogReply::Error(ShotLogStorageError::NotFound)) => {
                return Self::send_not_found(conn).await;
            }
            Ok(ShotLogReply::Error(e)) => {
                return Self::send_unavailable(conn, shot_log_error_message(e)).await;
            }
            Ok(_) => {
                log_error!("Shot log chunk: unexpected reply kind");
                return Self::send_internal_error(conn, "Unexpected shot log reply").await;
            }
            Err(_) => {
                return Self::send_unavailable(conn, "Machine did not answer in time").await;
            }
        };
        let (first_bytes, total, mut last) = first;

        // `<day>-<time>.BIN`, so a browser's download folder keeps shots distinguishable
        // -- the on-card name is only unique within its day directory.
        let mut filename = heapless::String::<64>::new();
        let _ = core::fmt::Write::write_fmt(
            &mut filename,
            format_args!(
                "attachment; filename=\"{}-{}\"",
                id.dir_name().as_str(),
                id.file_name().as_str()
            ),
        );

        // No `Content-Length`. `initiate_response` never sets one, and edge-http falls
        // back to chunked transfer-encoding on HTTP/1.1 -- which is what lets this stream
        // a 50 kB record through a 1 kB buffer instead of assembling it in RAM first.
        conn.initiate_response(
            200,
            Some("OK"),
            &[
                ("Content-Type", "application/octet-stream"),
                ("Content-Disposition", filename.as_str()),
            ],
        )
        .await?;
        conn.write_all(&first_bytes).await?;

        let mut offset = first_bytes.len() as u32;
        while !last {
            // Lockstep: request, wait, write, advance. That halves the throughput a
            // windowed scheme could reach and is the right trade -- it needs no
            // reordering, no second buffer and no state beyond `offset`, and the peak
            // memory is one chunk on a device where that is the binding constraint.
            match shot_log_request(ShotLogRequest::Chunk { id, offset }, SHOT_LOG_TIMEOUT).await {
                Ok(ShotLogReply::Chunk {
                    id: reply_id,
                    offset: reply_offset,
                    bytes,
                    last: is_last,
                    ..
                }) => {
                    // The echo check. `SHOT_LOG_LOCK` should make this impossible, but
                    // splicing one shot's bytes into another's download is a corruption
                    // no consumer could detect, so it is worth the comparison.
                    if reply_id != id || reply_offset != offset {
                        log_error!("Shot log chunk: reply does not match the request");
                        break;
                    }
                    if bytes.is_empty() {
                        break;
                    }
                    conn.write_all(&bytes).await?;
                    offset += bytes.len() as u32;
                    last = is_last;
                }
                // Past this point the status is already sent, so a failure can only be
                // expressed by ending the body early. Logged with the offset so a
                // truncated download is diagnosable from the device rather than only
                // from a byte count on the client.
                Ok(ShotLogReply::Error(e)) => {
                    log_error!("Shot log download failed at offset {}: {:?}", offset, e);
                    break;
                }
                Ok(_) => {
                    log_error!("Shot log download: unexpected reply kind at offset {}", offset);
                    break;
                }
                Err(_) => {
                    log_error!("Shot log download timed out at offset {} of {}", offset, total);
                    break;
                }
            }
        }

        Ok(())
    }

    // PUT /shots/<day>/<time>/annotations
    async fn handle_put_shot_annotations<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        id: ShotLogId,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("PUT /shots/{}/{}/annotations", id.dir_name().as_str(), id.file_name().as_str());

        let Some(annotations) = Self::read_annotations_body(conn).await? else {
            return Self::send_bad_request(conn, "Invalid postcard data").await;
        };

        // A `MachineCommand` like every other write, rather than a shot-log request: the
        // application processor's controller is the single interpreter of commands, and
        // routing an edit around it would give the same operation two different paths
        // depending on whether it arrived over HTTP or over the debug link.
        //
        // Fire and forget, therefore. The confirmation comes back as a
        // `ShotLogAnnotations` reply that nothing is currently waiting on; a client that
        // wants to see the result re-reads the list.
        self.send_command(conn, MachineCommand::SetShotAnnotations(id, annotations), "Annotations updated")
            .await
    }

    // DELETE /shots/<day>/<time>
    async fn handle_delete_shot<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        id: ShotLogId,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!(
            "DELETE /shots/{}/{}",
            id.dir_name().as_str(),
            id.file_name().as_str()
        );

        // Queued, not confirmed. 200 here means the command reached the channel; whether
        // the shot is gone is reported by a `ShotLogEvent::Deleted` push, which is also
        // what tells every *other* connected browser.
        self.send_command(conn, MachineCommand::DeleteShotLog(id), "Delete queued")
            .await
    }

    // PUT /shots/pending
    async fn handle_put_pending_annotations<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("PUT /shots/pending");

        let Some(annotations) = Self::read_annotations_body(conn).await? else {
            return Self::send_bad_request(conn, "Invalid postcard data").await;
        };

        self.send_command(
            conn,
            MachineCommand::SetPendingShotAnnotations(annotations),
            "Pending annotations updated",
        )
        .await
    }

    /// Read a postcard-encoded [`ShotAnnotations`] body.
    ///
    /// The body is a bare `ShotAnnotations` rather than a wrapper request type, because
    /// the only other field such a wrapper would carry is the shot id -- and that is in
    /// the URL, where it also serves the download. One fewer type to keep in step with
    /// the frontend, and `ShotAnnotations` already reaches the generated schema through
    /// `Status`.
    ///
    /// 1024 bytes is comfortably above a maximal block (545), and bounded because this
    /// runs on a device with 512 kB of RAM and the length comes from the client.
    async fn read_annotations_body<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<Option<ShotAnnotations>, Error<T::Error>>
    where
        T: Read + Write,
    {
        let body = Self::read_body(conn, 1024).await?;
        match postcard::from_bytes::<ShotAnnotations>(&body) {
            Ok(annotations) => Ok(Some(annotations)),
            Err(e) => {
                log_error!("Failed to deserialize ShotAnnotations: {:?}", defmt::Debug2Format(&e));
                Ok(None)
            }
        }
    }

    /// Queue a command and answer, or report the queue full.
    ///
    /// Extracted because five shot-log routes would otherwise repeat the same
    /// `try_send` / 200 / 503 block, and a divergence between copies would show up as one
    /// route silently succeeding where another reports failure.
    async fn send_command<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        command: MachineCommand,
        success: &str,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        match self.command_sender.try_send(command) {
            Ok(_) => Self::send_text(conn, 200, "OK", success).await,
            Err(_) => {
                log_error!("Command channel full");
                Self::send_unavailable(conn, "Command channel full").await
            }
        }
    }

    // POST /command/tag-dose-from-scale/<group>
    async fn handle_tag_dose_from_scale<T, const N: usize>(
        &self,
        conn: &mut ServerConnection<'_, T, N>,
        group: u8,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("POST /command/tag-dose-from-scale/{}", group);

        self.send_command(
            conn,
            MachineCommand::TagDoseFromScale(ScaleSelector::GroupScale(group)),
            "Dose tagged",
        )
        .await
    }

    // Parse u8 index
    fn parse_path_u8_index(path: &str, prefix: &str) -> Option<u8> {
        let index_str = path.strip_prefix(prefix)?;
        index_str.parse().ok()
    }

    // Serve index.html
    async fn handle_index_html<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("Serving index.html");
        Self::send_response(conn, 200, "OK", "text/html; charset=utf-8", INDEX_HTML).await
    }

    // Serve JS file with Gzip encoding
    async fn handle_js_gz<T, const N: usize>(
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Error<T::Error>>
    where
        T: Read + Write,
    {
        log_info!("Serving app.js.gz");
        conn.initiate_response(
            200,
            Some("OK"),
            &[
                ("Content-Type", "application/javascript"),
                ("Content-Encoding", "gzip"),
            ],
        )
        .await?;
        conn.write_all(APP_JS_GZ).await?;
        Ok(())
    }
}

impl Handler for HttpHandler {
    type Error<E>
        = Error<E>
    where
        E: Debug;

    async fn handle<T, const N: usize>(
        &self,
        _task_id: impl Display + Copy,
        conn: &mut ServerConnection<'_, T, N>,
    ) -> Result<(), Self::Error<T::Error>>
    where
        T: Read + Write,
    {
        let headers = conn.headers()?;
        let method = headers.method;
        let path = headers.path;

        // Route dispatch
        match (method, path) {
            // GET endpoints
            (Method::Get, "/status") => self.handle_get_status(conn).await,
            (Method::Get, "/configuration") => self.handle_get_configuration(conn).await,
            (Method::Get, "/machine-definition") => self.handle_get_machine_definition(conn).await,
            (Method::Get, "/routines") => self.handle_get_routines(conn).await,

            // Shot log.
            //
            // `/shots/pending` is matched before the `/shots/` id pattern below. It has
            // to be: `parse_shot_path` would reject "pending" as a day, so the order is
            // not load-bearing for correctness -- but relying on that would mean a future
            // day format that happened to accept it would silently steal this route.
            (Method::Get, "/shots") => {
                self.handle_get_shots(conn, ShotLogDayFilter::All, None).await
            }
            (Method::Put, "/shots/pending") => self.handle_put_pending_annotations(conn).await,

            // The two paging routes, registered **above** the `/shots/<day>/<time>`
            // download arm below. Unlike the `/shots/pending` ordering note, this one is
            // load-bearing: `parse_shot_path` rejects "before" and "day" as day
            // components, so without these arms first both paths answer 400.
            (Method::Get, p) if p.starts_with("/shots/before/") => {
                match Self::parse_shot_id(p.strip_prefix("/shots/before/").unwrap_or("")) {
                    Some((id, "")) => {
                        self.handle_get_shots(conn, ShotLogDayFilter::All, Some(id)).await
                    }
                    _ => Self::send_bad_request(conn, "Invalid shot cursor").await,
                }
            }
            (Method::Get, p) if p.starts_with("/shots/day/") => {
                match Self::parse_day_page(p.strip_prefix("/shots/day/").unwrap_or("")) {
                    Some((day, before)) => self.handle_get_shots(conn, day, before).await,
                    None => Self::send_bad_request(conn, "Invalid shot day").await,
                }
            }
            (Method::Get, p) if p.starts_with("/shots/") => {
                match Self::parse_shot_path(p) {
                    Some((id, "")) => self.handle_get_shot(conn, id).await,
                    _ => Self::send_bad_request(conn, "Invalid shot path").await,
                }
            }
            (Method::Put, p) if p.starts_with("/shots/") => {
                match Self::parse_shot_path(p) {
                    Some((id, "annotations")) => {
                        self.handle_put_shot_annotations(conn, id).await
                    }
                    _ => Self::send_bad_request(conn, "Invalid shot path").await,
                }
            }
            (Method::Delete, p) if p.starts_with("/shots/") => {
                match Self::parse_shot_path(p) {
                    Some((id, "")) => self.handle_delete_shot(conn, id).await,
                    _ => Self::send_bad_request(conn, "Invalid shot path").await,
                }
            }

            // Schedule CRUD
            (Method::Post, "/schedules") => self.handle_post_schedule(conn).await,
            (Method::Put, p) if p.starts_with("/schedules/") => {
                if let Some(index) = Self::parse_path_index(p, "/schedules/") {
                    self.handle_put_schedule(conn, index).await
                } else {
                    Self::send_bad_request(conn, "Invalid schedule index").await
                }
            }
            (Method::Delete, p) if p.starts_with("/schedules/") => {
                if let Some(index) = Self::parse_path_index(p, "/schedules/") {
                    self.handle_delete_schedule(conn, index).await
                } else {
                    Self::send_bad_request(conn, "Invalid schedule index").await
                }
            }

            // Routine CRUD
            //
            // Registered after the exact `GET /routines` above, which it would otherwise
            // shadow -- the listing and one definition are different resources.
            (Method::Get, p) if p.starts_with("/routines/") => {
                let remainder = p.strip_prefix("/routines/").unwrap_or("");
                let parts: Vec<&str> = remainder.split('/').collect();
                if parts.len() != 2 {
                    Self::send_bad_request(conn, "Invalid path format").await
                } else if let Some(index) = Self::parse_routine_index(parts[0], parts[1]) {
                    self.handle_get_routine(conn, index).await
                } else {
                    Self::send_bad_request(conn, "Invalid routine type or index").await
                }
            }
            (Method::Post, p) if p.starts_with("/routines/") => {
                let remainder = p.strip_prefix("/routines/").unwrap_or("");
                self.handle_post_routine(conn, remainder).await
            }
            (Method::Put, p) if p.starts_with("/routines/") => {
                let remainder = p.strip_prefix("/routines/").unwrap_or("");
                let parts: Vec<&str> = remainder.split('/').collect();
                if parts.len() == 2 {
                    if let Ok(index) = parts[1].parse::<u32>() {
                        self.handle_put_routine(conn, parts[0], index).await
                    } else {
                        Self::send_bad_request(conn, "Invalid routine index").await
                    }
                } else {
                    Self::send_bad_request(conn, "Invalid path format").await
                }
            }
            (Method::Delete, p) if p.starts_with("/routines/") => {
                let remainder = p.strip_prefix("/routines/").unwrap_or("");
                let parts: Vec<&str> = remainder.split('/').collect();
                if parts.len() == 2 {
                    if let Ok(index) = parts[1].parse::<u32>() {
                        self.handle_delete_routine(conn, parts[0], index).await
                    } else {
                        Self::send_bad_request(conn, "Invalid routine index").await
                    }
                } else {
                    Self::send_bad_request(conn, "Invalid path format").await
                }
            }

            // Command endpoints
            (Method::Post, p) if p.starts_with("/command/run-routine/") => {
                let remainder = p.strip_prefix("/command/run-routine/").unwrap_or("");
                let parts: Vec<&str> = remainder.split('/').collect();
                if parts.len() == 2 {
                    if let Ok(index) = parts[1].parse::<u32>() {
                        self.handle_run_routine(conn, parts[0], index).await
                    } else {
                        Self::send_bad_request(conn, "Invalid routine index").await
                    }
                } else {
                    Self::send_bad_request(conn, "Invalid path format").await
                }
            }
            (Method::Post, "/command/cancel-routine") => self.handle_cancel_routine(conn).await,
            (Method::Post, p) if p.starts_with("/command/tag-dose-from-scale/") => {
                if let Some(group) = Self::parse_path_u8_index(p, "/command/tag-dose-from-scale/") {
                    self.handle_tag_dose_from_scale(conn, group).await
                } else {
                    Self::send_bad_request(conn, "Invalid group index").await
                }
            }
            (Method::Post, p) if p.starts_with("/command/tare-group-scale/") => {
                if let Some(index) = Self::parse_path_u8_index(p, "/command/tare-group-scale/") {
                    self.handle_tare_group_scale(conn, index).await
                } else {
                    Self::send_bad_request(conn, "Invalid group index").await
                }
            }
            (Method::Post, p) if p.starts_with("/command/zero-calibrate-group-scale/") => {
                if let Some(index) =
                    Self::parse_path_u8_index(p, "/command/zero-calibrate-group-scale/")
                {
                    self.handle_zero_calibrate_group_scale(conn, index).await
                } else {
                    Self::send_bad_request(conn, "Invalid group index").await
                }
            }
            (Method::Post, p) if p.starts_with("/command/calibrate-group-scale-100g/") => {
                if let Some(index) =
                    Self::parse_path_u8_index(p, "/command/calibrate-group-scale-100g/")
                {
                    self.handle_calibrate_group_scale_100g(conn, index).await
                } else {
                    Self::send_bad_request(conn, "Invalid group index").await
                }
            }
            (Method::Post, p) if p.starts_with("/command/set-mode/") => {
                let mode_str = p.strip_prefix("/command/set-mode/").unwrap_or("");
                self.handle_set_mode(conn, mode_str).await
            }
            (Method::Post, "/command/set-boiler-control") => {
                self.handle_set_boiler_control(conn).await
            }
            (Method::Post, "/command/set-group-control") => {
                self.handle_set_group_control(conn).await
            }
            (Method::Post, "/command/set-pid-parameters") => {
                self.handle_set_pid_parameters(conn).await
            }
            (Method::Post, "/command/set-group-pump-configuration") => {
                self.handle_set_group_pump_configuration(conn).await
            }
            (Method::Post, "/command/set-water-tap-pump-configuration") => {
                self.handle_set_water_tap_pump_configuration(conn).await
            }
            (Method::Post, "/command/set-fill-pump-configuration") => {
                self.handle_set_fill_pump_configuration(conn).await
            }
            (Method::Post, "/command/set-steam-valve-openness") => {
                self.handle_set_steam_valve_openness(conn).await
            }
            (Method::Post, "/command/set-shot-upload-settings") => {
                self.handle_set_shot_upload_settings(conn).await
            }
            (Method::Post, "/command/optimize-routine-storage") => {
                self.handle_optimize_routine_storage(conn).await
            }
            (Method::Post, "/command/optimize-schedule-storage") => {
                self.handle_optimize_schedule_storage(conn).await
            }
            (Method::Post, "/command/optimize-configuration-storage") => {
                self.handle_optimize_configuration_storage(conn).await
            }

            // Frontend static files
            (Method::Get, "/") | (Method::Get, "/index.html") => {
                Self::handle_index_html(conn).await
            }
            (Method::Get, p) if p.starts_with("/assets/") && p.ends_with(".js") => {
                Self::handle_js_gz(conn).await
            }

            // SPA fallback - serve index.html for unknown GET paths
            (Method::Get, _) => {
                log_info!("SPA fallback for: {}", path);
                Self::handle_index_html(conn).await
            }

            // 404 for non-GET methods on unknown paths
            _ => {
                defmt::warn!("Unknown endpoint: {} {}", defmt::Debug2Format(&method), path);
                Self::send_not_found(conn).await
            }
        }
    }
}

/// Cache update task - updates STATUS_CACHE and CONFIG_CACHE from subscribers
#[embassy_executor::task]
pub async fn cache_update_task(
    mut status_subscriber: ApplicationStatusSubscriber,
    mut config_subscriber: ApplicationConfigurationSubscriber,
) {
    log_info!("Cache update task started");

    loop {
        match select(
            status_subscriber.next_message_pure(),
            config_subscriber.next_message_pure(),
        )
        .await
        {
            Either::First(status) => {
                let mut cache = STATUS_CACHE.lock().await;
                *cache = Some(status);
            }
            Either::Second(config) => {
                let mut cache = CONFIG_CACHE.lock().await;
                *cache = Some(config);
            }
        }
    }
}

/// HTTP server task
#[embassy_executor::task]
pub async fn http_server_task(
    tcp_stack: &'static Tcp<'static>,
    command_sender: &'static MachineCommandSender,
) {
    log_info!("Starting HTTP server on port 80...");

    // The handler-task count MUST equal the socket count in `TcpBuffers` (see
    // `main.rs`). Every one of these tasks sits in `accept()` simultaneously and holds a
    // socket from that pool while it does -- edge-http works that way on purpose,
    // because smoltcp has no accept queue and a connection can only be accepted if some
    // task is already waiting in `accept()`.
    //
    // `DefaultServer` is `Server<4, ..>`, which silently matched the old
    // `TcpBuffers<4, ..>`. When the pool was reduced to 2 the two surplus tasks had no
    // socket to wait on, and the whole server stopped listening -- port 80 answered
    // "connection refused" rather than answering slowly. Spelling the count out here
    // makes the pairing visible instead of coincidental.
    //
    // Cheaper as well as correcter: the server's own buffers are `[[u8; 2048]; P]`, so
    // halving P returns 4 kB.
    let mut server = HttpServer::new();
    let handler = HttpHandler::new(command_sender);

    let bind_addr = SocketAddr::from(([0, 0, 0, 0], 80));

    match tcp_stack.bind(bind_addr).await {
        Ok(acceptor) => {
            log_info!("HTTP server bound to port 80");
            if let Err(e) = server.run(None, acceptor, handler).await {
                log_error!("HTTP server error: {:?}", e);
            }
        }
        Err(e) => {
            log_error!("Failed to bind HTTP server to port 80: {:?}", e);
        }
    }
}
