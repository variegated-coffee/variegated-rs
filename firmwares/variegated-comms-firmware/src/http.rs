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

// `ROUTINE_TIMEOUT`, `ROUTINE_WRITE_TIMEOUT` and `ROUTINE_BODY_LIMIT` moved to
// `websocket.rs` with the routes they bounded. The two timeouts kept their values there,
// deliberately: they bound the *machine's* turnaround, which does not change with who asked.
//
// `ROUTINE_BODY_LIMIT` did not move, because `MAX_CLIENT_FRAME_LEN` in `ws_types.rs` is the
// same expression -- `ROUTINE_MAX_ENCODED_LEN + 256` -- and was written to be. It is worth
// remembering what that number is for: the far side stores through a 2 kB buffer, so a larger
// body could not be saved whatever the transport said, and reading it would be work spent on
// its way to a refusal.

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
        // Same advice as `BusUnavailable`, different situation, and the wording is chosen
        // to be honest about which: that one means the request never got the card's
        // attention, this one means it did and the card stopped answering mid-operation.
        // A retry is still the right move -- the machine re-identifies the card first --
        // but "busy" would understate a fault worth mentioning if it keeps happening.
        ShotLogStorageError::OperationTimedOut => {
            "The SD card stopped responding; the machine is retrying it"
        }
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

// No routine types at all any more. A routine definition does not pass through this server
// in either direction -- reads, writes and the summary listing are all `WsMessage` now -- so
// the only stored-data types left here are the shot-log ones the download needs.
use variegated_controller_types::{MachineCommand, MachineMode, ScheduleItem};
use variegated_controller_types::shot_log::{ShotLogId, ShotLogStorageError};

use crate::channels::{
    shot_log_request, ApplicationConfigurationSubscriber, ApplicationStatusSubscriber,
    MachineCommandSender, ShotLogReply, ShotLogRequest, CONFIG_CACHE, MACHINE_DEFINITION,
    MACHINE_MODE_CHANGED, STATUS_CACHE,
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

    // `parse_day_page` and `handle_get_shots` were here. The listing is
    // `ClientQuery::ShotLogPage` now, and the day filter travels as a `ShotLogDayFilter`
    // field rather than being spelled out in three separate URL shapes and parsed back.

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

    // The shot *mutations* were here -- `PUT /shots/<day>/<time>/annotations`,
    // `DELETE /shots/<day>/<time>` and `PUT /shots/pending` -- along with
    // `read_annotations_body` and `send_command`, which existed only to serve them.
    //
    // All three were already `MachineCommand`s pushed fire-and-forget: their 200 meant the
    // command reached the channel, never that the card had been written. So they lost nothing
    // by becoming `SendMachineCommandWithId`, whose ack says exactly that much, and the
    // confirmation still arrives the way it always did -- as a `ShotLogEvent` push, which is
    // also what tells every *other* connected browser.
    //
    // `send_command`'s doc called itself out as shared by "five shot-log routes". Those five
    // are now one: the download below.

    // `parse_path_u8_index` was here. Every caller was a `/command/*` route with a `u8` in its
    // path -- the scale calibrations, the steam wand, `tag-dose-from-scale` -- and those
    // arguments travel inside the command itself now, typed, rather than being parsed back out
    // of a URL. `parse_path_index` above survives because `/schedules/{index}` still uses it.

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

            // **The shot log's one remaining route, and the only reason this server still
            // carries an API at all besides the SPA.**
            //
            // The listing, the annotations and the delete all moved onto the WebSocket. This
            // did not, and the reason is not size but *shape*: a stored shot is tens of
            // kilobytes streamed through a 1 kB buffer into an already-committed 200, and it
            // reaches the browser as a `Content-Disposition` filename that a bare
            // `<a download>` can use with no JavaScript at all. Carrying that over a frame-
            // based protocol would mean assembling the whole file in browser memory and
            // synthesising a blob URL — losing the streaming and the filename to gain nothing.
            //
            // The three *listing* routes that used to sit above this one were only ever three
            // spellings of one `ShotLogListRequest`, forced apart because this router matches
            // paths exactly and parses no query string. On the socket the filter is a field
            // again, so they collapsed into `ClientQuery::ShotLogPage`. That also retires the
            // ordering hazard they carried: they had to be registered before this arm, because
            // `parse_shot_path` rejects "before" and "day" as day components and both paths
            // would otherwise have answered 400.
            (Method::Get, p) if p.starts_with("/shots/") => {
                match Self::parse_shot_path(p) {
                    Some((id, "")) => self.handle_get_shot(conn, id).await,
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

            // No routine routes. All five -- the listing, one definition, create, update and
            // delete -- are `WsMessage::Query` and `SendMachineCommandWithId` now.
            //
            // The listing was already redundant: `WsMessage::RoutinesUpdate` has carried the
            // same summaries, pushed rather than polled, since routine definitions stopped
            // crossing the link. Only `variegated-cli` was still fetching it over HTTP.
            //
            // The read is *better* on the socket, not merely moved. This route took
            // `ROUTINE_LOCK` per chunk and wrote each one into an already-committed 200, so a
            // save landing mid-download spliced an old head onto a new tail -- and postcard
            // being positional, the result decoded into a routine nobody wrote. The socket
            // reassembles before it answers, so there is one lock-consistent reply or none.
            //
            // The write is better too: `RoutineWriteOutcome`'s five failure modes reach the
            // browser intact, where this projected them onto three status codes and the
            // frontend read only the number.

            // No `/command/*` routes. Every one of them deserialised a request struct, built a
            // `MachineCommand` and `try_send`'d it -- which is exactly what
            // `WsMessage::SendMachineCommandWithId` does, with an ack the HTTP 200 could not
            // give. Sixteen of the seventeen already had an equivalent method in
            // `services/websocket.ts` before they were removed, and the seven `Set*Request`
            // body types had been dead in the frontend since those methods landed.
            //
            // `set-pid-parameters` is the one worth remembering: it took `target_type` as a
            // `String` matched against five literals, where the WebSocket carries the typed
            // `PidParameterTarget`. Deleting it removed a stringly-typed edge, not just a route.

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
    let checkin = crate::checkin::MONITOR.claim(crate::checkin::CheckinId::CacheUpdate);

    // The mode the last status carried, for the edge the uplink cares about. Held here rather
    // than compared against the cache, because the cache has already been overwritten by the
    // time anyone else could look -- and because this task is the only reader of the status
    // stream that is not already busy with something else.
    let mut previous_mode: Option<MachineMode> = None;

    loop {
        checkin.good();

        match select(
            status_subscriber.next_message_pure(),
            config_subscriber.next_message_pure(),
        )
        .await
        {
            Either::First(status) => {
                let mode = status.mode;
                // Only on a real change, and never on the first status of all: the uplink sends
                // one the moment its session opens, so signalling here would only duplicate it.
                if previous_mode.is_some_and(|was| was != mode) {
                    MACHINE_MODE_CHANGED.signal(());
                }
                previous_mode = Some(mode);

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
            // `server.run` never returns on success, so this row's whole job is to catch
            // the two ways it goes quiet: the bind failing above (the slot stays
            // `NotStarted` and the log line says why), or `run` returning, which lands on
            // `TaskExited` and is currently only visible in a log nobody is watching.
            if let Err(e) = variegated_checkin::watch(
                crate::checkin::MONITOR.claim(crate::checkin::CheckinId::HttpServer),
                server.run(None, acceptor, handler),
            )
            .await
            {
                log_error!("HTTP server error: {:?}", e);
            }
        }
        Err(e) => {
            log_error!("Failed to bind HTTP server to port 80: {:?}", e);
        }
    }
}
