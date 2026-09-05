#![no_std]

extern crate alloc;
pub mod debug_relay;

use alloc::collections::BTreeMap;
use alloc::vec::Vec;
use alloc::boxed::Box;
use core::cell::RefCell;
use chrono::{DateTime, Utc};
use crc::{Crc, CRC_32_ISCSI};
use defmt::{error, info};
use embassy_futures::join::{join, join4, join5};
// `Error` is imported for its `kind()` method on the reader's associated error type; see
// the read arm in the reader loop.
use embedded_io_async::{Error as _, Read, Write};
use embassy_sync::pubsub::Subscriber;
use embassy_sync::blocking_mutex::Mutex;
use postcard::to_allocvec_cobs;
use variegated_controller_types::debug::{name, DebugEvent};
use variegated_controller_types::debug_command::DebugCommand;
use variegated_controller_types::shot_upload::ShotUploadConfig;
use variegated_controller_types::wifi::StoredWifiCredentials;
use variegated_controller_types::{
    ApplicationProcessorToCommsProcessorMessage,
    BrewSensorOp,
    CommsProcessorToApplicationProcessorMessage,
    Configuration,
    InputCommand,
    MachineCommand,
    MachineDefinition,
    PeripheralId,
    Routine,
    RoutineIndex,
    RoutineDeleteError,
    RoutineDeleteOutcome,
    RoutineSummary,
    RoutineSummaryList,
    RoutineWriteError,
    RoutineWriteOutcome,
    ScaleOp,
    Status,
    ROUTINE_CHUNK_LEN,
    ROUTINE_MAX_ENCODED_LEN,
};
use variegated_debug::bus;
use embassy_sync::channel::{Channel, Receiver as ChannelReceiver, Sender};
use postcard::accumulator::{CobsAccumulator, FeedResult};
use variegated_controller_lib::routine::{self, RoutineRepository};
use variegated_controller_lib::settings::SettingsStorage;
use variegated_controller_types::bluetooth::BluetoothBonds;
use variegated_controller_lib::shot_log_query::{ShotLogQuery, ShotLogReply};
use variegated_controller_lib::external_sensor_dispatcher::ExternalSensorDispatcher;
use variegated_timekeeping::TimeKeeper;

/// Earliest Unix timestamp we will accept from the comms processor as a real
/// wall-clock time: 2020-01-01T00:00:00Z.
///
/// Anything below this is the comms processor's RTC counting up from zero
/// before SNTP has synced, not an actual date.
const MIN_PLAUSIBLE_UNIX_TIME: u64 = 1_577_836_800;

/// The largest COBS frame the far side can reassemble.
///
/// Matches `CobsAccumulator::<4096>` on both ends of this link -- see the note beside
/// this side's accumulator below. A frame at or over this length is not truncated on
/// arrival, it is *lost*: the accumulator overruns, discards, and resynchronises on the
/// next sentinel, so an oversized reply looks exactly like a dead link.
const LINK_FRAME_LIMIT: usize = 4096;

/// Serialize a reply and hand it to the link, refusing to send one the far side cannot
/// reassemble.
///
/// This check did not exist, and its absence had already shipped a real failure: the old
/// `Routines` reply carried every routine's full definition in one frame, so a machine
/// with about a dozen routines silently stopped being able to answer `RequestRoutines`
/// at all. Nothing reported it, because on the wire an overrun and a missing reply are
/// the same event. Summaries make that ceiling remote rather than imminent, which is a
/// reason to keep the guard rather than to skip it.
///
/// Returns the frame, or `None` having already logged why not.
fn frame_for_link(
    response: &ApplicationProcessorToCommsProcessorMessage,
    what: &str,
) -> Option<Vec<u8>> {
    match to_allocvec_cobs(response) {
        Ok(output) if output.len() <= LINK_FRAME_LIMIT => Some(output),
        Ok(output) => {
            error!(
                "Refusing to send {}: {} bytes exceeds the {} byte link frame limit",
                what,
                output.len(),
                LINK_FRAME_LIMIT
            );
            None
        }
        Err(_) => {
            error!("Failed to serialize {}", what);
            None
        }
    }
}

/// A routine being reassembled from `RoutineWriteChunk`s.
///
/// One at a time, because the comms processor holds a lock for the whole sequence and
/// there is no correlation id to tell two interleaved writes apart. A chunk that does not
/// continue the sequence in progress is refused rather than merged -- splicing two
/// routines together would produce bytes that might still decode, into a routine nobody
/// wrote.
struct RoutineWriteAssembly {
    /// `None` for a create, `Some` to replace.
    index: Option<RoutineIndex>,
    /// The full encoded length the first chunk promised.
    total: u16,
    buffer: [u8; ROUTINE_MAX_ENCODED_LEN],
    len: usize,
}

/// Summarise every stored routine and frame the reply.
///
/// **No routine is cloned.** `iterate_routines_with_indices` yields references and
/// `RoutineSummary::from` copies only the name out of each, which is the whole point of
/// the summary: this used to deep-clone the repository's entire cache into a fresh map
/// before serialising it, every fifteen seconds, and the far side then cloned it twice
/// more.
async fn build_routine_summaries<M: embassy_sync::blocking_mutex::raw::RawMutex, R: RoutineRepository>(
    repository: &'static embassy_sync::mutex::Mutex<M, R>,
) -> Option<Vec<u8>> {
    let list = {
        let mut repo = repository.lock().await;
        let routines = repo
            .iterate_routines_with_indices()
            .await
            .map(|(index, routine)| (index, RoutineSummary::from(routine)))
            .collect::<BTreeMap<_, _>>();
        RoutineSummaryList { routines }
    };

    frame_for_link(
        &ApplicationProcessorToCommsProcessorMessage::RoutineSummaries(list),
        "routine summaries",
    )
}

/// Cut one chunk out of an encoded routine.
///
/// `offset` past the end yields an empty final chunk rather than an error, so a receiver
/// that miscounts stops instead of looping -- `last` is what terminates the download, and
/// it is true here whenever there is nothing after this slice.
fn routine_chunk(
    index: RoutineIndex,
    offset: u16,
    encoded: &[u8],
) -> ApplicationProcessorToCommsProcessorMessage {
    let total = encoded.len();
    let start = (offset as usize).min(total);
    let end = (start + ROUTINE_CHUNK_LEN).min(total);

    let mut bytes = heapless::Vec::new();
    // Cannot fail: the slice is at most `ROUTINE_CHUNK_LEN`, which is the vector's
    // capacity.
    let _ = bytes.extend_from_slice(&encoded[start..end]);

    ApplicationProcessorToCommsProcessorMessage::RoutineChunk {
        index,
        offset: start as u16,
        total: total as u16,
        last: end >= total,
        bytes,
    }
}

/// Fold one inbound chunk into the routine being assembled.
///
/// Returns `None` while more chunks are expected, `Some(Ok(()))` when the last one has
/// landed and `state` holds a complete encoding, and `Some(Err(_))` when the sequence is
/// refused.
///
/// Every refusal path here matters more than it looks. postcard is positional and
/// non-self-describing, so a routine assembled from chunks that skipped, repeated or
/// interleaved does not fail to decode -- it decodes into a *different routine*, which
/// would then be written to flash over the one the user was editing.
fn accept_routine_write_chunk(
    state: &mut Option<RoutineWriteAssembly>,
    index: Option<RoutineIndex>,
    offset: u16,
    total: u16,
    last: bool,
    bytes: &[u8],
) -> Option<Result<(), RoutineWriteError>> {
    if offset == 0 {
        // A fresh sequence displaces any half-finished one. The far side sends a whole
        // routine under a lock, so a new first chunk means the previous sequence died --
        // its sender timed out, or the link dropped mid-write.
        if state.is_some() {
            info!("Discarding an incomplete routine write");
        }

        if total as usize > ROUTINE_MAX_ENCODED_LEN {
            // Refused on the first chunk rather than after reassembling, which is why
            // `total` rides on every chunk instead of being inferred at the end.
            error!("Refusing a {} byte routine write: over the {} byte ceiling", total, ROUTINE_MAX_ENCODED_LEN);
            *state = None;
            return Some(Err(RoutineWriteError::TooLarge));
        }

        if matches!(index, Some(RoutineIndex::Internal(_))) {
            // Checked here rather than left to the repository, which reports it as an
            // opaque string this arm would have to match on.
            *state = None;
            return Some(Err(RoutineWriteError::Immutable));
        }

        *state = Some(RoutineWriteAssembly {
            index,
            total,
            buffer: [0u8; ROUTINE_MAX_ENCODED_LEN],
            len: 0,
        });
    }

    let assembly = match state.as_mut() {
        Some(assembly) => assembly,
        // A continuation with nothing to continue: the first chunk was refused, or this
        // is a stray from a sequence already abandoned.
        None => return Some(Err(RoutineWriteError::Malformed)),
    };

    // Contiguity, addressing and length, in that order. `index` and `total` are compared
    // because they are the only evidence that this chunk belongs to this sequence --
    // there is no correlation id on this link.
    if assembly.len != offset as usize
        || assembly.index != index
        || assembly.total != total
        || assembly.len + bytes.len() > assembly.total as usize
    {
        *state = None;
        return Some(Err(RoutineWriteError::Malformed));
    }

    assembly.buffer[assembly.len..assembly.len + bytes.len()].copy_from_slice(bytes);
    assembly.len += bytes.len();

    if !last {
        return None;
    }

    // `last` is not taken on trust: a sender that sets it early would otherwise store a
    // truncated routine, and a truncated postcard encoding can still decode.
    if assembly.len != assembly.total as usize {
        *state = None;
        return Some(Err(RoutineWriteError::Malformed));
    }

    Some(Ok(()))
}

/// Decode an assembled routine and put it in the repository.
///
/// The decode happens *here* rather than on the comms processor, which is the point of
/// the whole byte pass-through: this side has the flash, the 2 kB buffer and the room to
/// build a `Routine`; that side has a 56 kB heap shared with Wi-Fi and BLE.
async fn store_routine<M: embassy_sync::blocking_mutex::raw::RawMutex, R: RoutineRepository>(
    repository: &'static embassy_sync::mutex::Mutex<M, R>,
    index: Option<RoutineIndex>,
    bytes: &[u8],
) -> RoutineWriteOutcome {
    let routine: Routine = match postcard::from_bytes(bytes) {
        Ok(routine) => routine,
        Err(_) => {
            error!("A routine write did not decode");
            return RoutineWriteOutcome::Failed(RoutineWriteError::Malformed);
        }
    };

    // Refused here rather than at the flash, so a client gets a reason instead of a silent
    // failure to reload. This catches an old client as well as an old routine: the encoding
    // is positional, so a client built against an earlier `Routine` produces bytes that
    // decode into *something* -- and the version is the only field that says they should
    // not have.
    if let Err(e) = routine.validate() {
        error!("A routine write was refused: {}", e);
        return RoutineWriteOutcome::Failed(e);
    }

    // Bounded, like every other repository access on this task. This runs on the UART
    // reader, which also carries status and configuration; waiting indefinitely on a
    // contended repository would stall the whole link behind one save.
    let mut repo = match embassy_time::with_timeout(
        embassy_time::Duration::from_millis(100),
        repository.lock(),
    )
    .await
    {
        Ok(repo) => repo,
        Err(_) => {
            error!("Failed to acquire routine_repository lock for a write (timeout)");
            return RoutineWriteOutcome::Failed(RoutineWriteError::Storage);
        }
    };

    match index {
        // An upsert, deliberately: writing to an unoccupied index is how a routine gets
        // placed on a hardware button.
        Some(index) => match repo.update_routine(index, routine).await {
            Ok(()) => RoutineWriteOutcome::Stored(index),
            Err(e) => {
                error!("Failed to update routine: {}", e);
                RoutineWriteOutcome::Failed(RoutineWriteError::Storage)
            }
        },
        None => match repo.add_routine(routine).await {
            Ok(index) => RoutineWriteOutcome::Stored(index),
            Err(e) => {
                error!("Failed to add routine: {}", e);
                RoutineWriteOutcome::Failed(RoutineWriteError::Storage)
            }
        },
    }
}

/// Remove a routine, and say which of the four things happened.
///
/// The sibling of [`store_routine`], and it answers rather than logging for the reason that
/// one does: a delete used to travel as `MachineCommand::RemoveRoutine`, which is
/// fire-and-forget, so an erase that failed on a worn sector looked exactly like one that
/// worked. The application processor already told these apart in its own log and threw the
/// distinction away at the link.
///
/// Internal indices are refused here rather than at the repository, which is where the write
/// path refuses them too. It is not merely for symmetry: `remove_routine` reports an internal
/// index as `Ok(None)`, the same answer it gives for an empty slot, so a refusal that went
/// through it would come back as "there was nothing there".
async fn delete_routine<M: embassy_sync::blocking_mutex::raw::RawMutex, R: RoutineRepository>(
    repository: &'static embassy_sync::mutex::Mutex<M, R>,
    index: RoutineIndex,
) -> RoutineDeleteOutcome {
    if matches!(index, RoutineIndex::Internal(_)) {
        return RoutineDeleteOutcome::Failed(RoutineDeleteError::Immutable);
    }

    // Bounded, like every other repository access on this task -- see `store_routine`.
    let mut repo = match embassy_time::with_timeout(
        embassy_time::Duration::from_millis(100),
        repository.lock(),
    )
    .await
    {
        Ok(repo) => repo,
        Err(_) => {
            error!("Failed to acquire routine_repository lock for a delete (timeout)");
            return RoutineDeleteOutcome::Failed(RoutineDeleteError::Storage);
        }
    };

    match repo.remove_routine(index).await {
        Ok(Some(_)) => RoutineDeleteOutcome::Deleted,
        Ok(None) => RoutineDeleteOutcome::Failed(RoutineDeleteError::NotFound),
        Err(e) => {
            error!("Failed to remove routine: {}", e);
            RoutineDeleteOutcome::Failed(RoutineDeleteError::Storage)
        }
    }
}

/// Hand a shot-log request to the storage task, or refuse it in a way the far side can
/// act on.
///
/// Every failure path here answers rather than returning: the comms processor waits on a
/// timeout with no correlation id, so a request that produces nothing is
/// indistinguishable from a dead link, and the user is told "the machine is not
/// responding" about a machine that is merely busy or has no card.
///
/// **`try_send`, not `await`.** This runs on the UART reader, which is also carrying
/// status and configuration; a bulk path must never be able to stall telemetry behind it.
/// The queue is depth 1, so a full queue means a request is already in flight -- and with
/// the far side serialising behind a lock that should not happen, which is why it is
/// reported rather than silently retried.
fn forward_shot_log_query<M: embassy_sync::blocking_mutex::raw::RawMutex, SM: embassy_sync::blocking_mutex::raw::RawMutex>(
    sender: Option<&Sender<'static, SM, ShotLogQuery, 1>>,
    query: ShotLogQuery,
    tx_sender: &Sender<'_, M, Vec<u8>, 10>,
) {
    let refusal = match sender {
        Some(sender) => match sender.try_send(query) {
            Ok(()) => return,
            Err(_) => {
                info!("Shot log request refused: one is already in flight");
                variegated_controller_types::ShotLogStorageError::BusUnavailable
            }
        },
        None => {
            info!("Shot log request refused: this machine has no shot-log storage");
            variegated_controller_types::ShotLogStorageError::CardNotPresent
        }
    };

    let response = ApplicationProcessorToCommsProcessorMessage::ShotLogError(refusal);
    if let Ok(output) = to_allocvec_cobs(&response) {
        // `try_send` on the outbound queue too: this is the error path, and blocking the
        // UART reader to report that something could not be queued would be the same
        // mistake one level up.
        let _ = tx_sender.try_send(output);
    }
}

/// Generic ESP32-C6 transceiver task that handles bidirectional communication
///
/// This task manages five concurrent operations:
/// 1. Status sending from application processor to comms processor
/// 2. Message receiving from comms processor and command forwarding
/// 3. UART TX coordination for all outgoing data
/// 4. Configuration monitoring and proactive broadcasting
/// 5. Structured debug frame relaying (see [`debug_relay`])
///
/// `link_baud` is the baud rate `uart_tx`/`uart_rx` were configured with. It is passed
/// rather than read back because embassy exposes no getter, and it is needed because the
/// debug relay's byte budget is a *fraction* of the link rather than an absolute. See
/// [`debug_relay::relay`].
///
/// Both boards now run this link at 576 kbaud with hardware flow control, which is not a
/// choice either of them makes: `variegated-comms-firmware`'s `config::uart_config`
/// hardcodes that rate and flow-control pair, and nothing negotiates -- `ProtocolConfig`
/// describes the protocol, not the wire. An application processor that disagrees has no
/// link at all, which is the state `single-boiler` was in while it ran 115 200 without
/// RTS/CTS.
///
/// Callers should pass the same binding they set on `uart::Config` so the two cannot drift
/// apart.
///
/// `DM` is separate from `M` because the debug command channel's mutex is not this
/// caller's to choose: `variegated_debug::usb_cdc::CommandSink` fixes it to
/// `CriticalSectionRawMutex`, and injected commands from both transports have to
/// converge on that one channel.
///
/// `RX` and `TX` are `embedded_io_async` traits rather than embassy-rp's `UartRx`/`UartTx`,
/// which is what lets the callers hand over a `BufferedUart`. **The RX side must return
/// what is available rather than filling a fixed buffer** -- see the reader loop's own note.
/// Named parameters rather than `impl Trait` in argument position, because
/// `variegated-silvia-firmware` calls this with an explicit turbofish and Rust refuses
/// those on a function that uses argument-position `impl Trait`.
pub async fn esp_transceiver_main<M: embassy_sync::blocking_mutex::raw::RawMutex, R: RoutineRepository, D: ExternalSensorDispatcher, DM: embassy_sync::blocking_mutex::raw::RawMutex, SM: embassy_sync::blocking_mutex::raw::RawMutex, BS: SettingsStorage<BluetoothBonds>, RX: Read, TX: Write, const STATUS_SUBS: usize, const CONFIG_SUBS: usize>(
    mut uart_tx: TX,
    mut uart_rx: RX,
    // The baud rate `uart_tx`/`uart_rx` were configured with -- see the note on
    // `link_baud` in this function's docs.
    link_baud: u32,
    mut status_receiver: Subscriber<'static, M, Status, 1, STATUS_SUBS, 1>,
    mut configuration_receiver: Subscriber<'static, M, Configuration, 1, CONFIG_SUBS, 1>,
    routine_repository: &'static embassy_sync::mutex::Mutex<M, R>,
    command_sender: Sender<'static, M, MachineCommand, 10>,
    machine_definition: MachineDefinition,
    external_sensor_dispatcher: Option<&D>,
    debug_command_sender: Sender<'static, DM, DebugCommand, 4>,
    // Commands for a scale owned by the comms processor, from a
    // `variegated_hal::scale::bluetooth::BluetoothScaleController`. `None` on machines
    // with no such scale. The element type is the wire payload itself, so this arm
    // wraps rather than translates -- and it is `(PeripheralId, ScaleOp)` rather than a
    // bare op because one machine can carry several scales.
    scale_command_receiver: Option<ChannelReceiver<'static, SM, (PeripheralId, ScaleOp), 4>>,
    // The same shape for a brew sensor owned by the comms processor -- today the Belka
    // Portal's display. `None` on a machine with no such sensor, which is every machine but
    // the GS3.
    brew_sensor_command_receiver:
        Option<ChannelReceiver<'static, SM, (PeripheralId, BrewSensorOp), 4>>,
    // Accepted Bluetooth scan requests, carrying the duration in milliseconds. `None` on
    // a machine whose controller was not given the matching sender, in which case the
    // controller refuses scan requests rather than this arm dropping them.
    bluetooth_scan_receiver: Option<ChannelReceiver<'static, SM, u16, 2>>,
    // The two halves of the shot-log request path, to and from the storage task on the
    // other core. `None` on a machine without shot-log storage, in which case both arms
    // park forever and a request from the comms processor is answered with
    // `ShotLogError(CardNotPresent)` rather than being silently dropped -- a request that
    // gets no answer at all is indistinguishable from a dead link.
    shot_log_query_sender: Option<Sender<'static, SM, ShotLogQuery, 1>>,
    shot_log_reply_receiver: Option<ChannelReceiver<'static, SM, ShotLogReply, 1>>,
    // Unsolicited shot-log events -- a shot stored, or a shot deleted. A third channel
    // rather than a third use of the reply path above, because that path has no
    // correlation id and an unsolicited message on it can be collected by a client
    // waiting on a listing. `None` on a machine with no card, where the arm parks.
    shot_log_event_receiver: Option<
        ChannelReceiver<'static, SM, variegated_controller_types::ShotLogEvent, 2>,
    >,
    // The stored Wi-Fi credentials, published by the controller whenever they change.
    // `None` on a machine with no credential store, in which case `RequestWifiCredentials`
    // goes unanswered and the comms processor keeps asking -- which is the honest outcome,
    // since there is nothing to tell it. Answering `None` would instead say "no network
    // configured" and stop the retry forever.
    wifi_credentials_receiver: Option<embassy_sync::watch::Receiver<'static, SM, StoredWifiCredentials, 2>>,
    // Accepted provisioning-window requests, carrying the duration in milliseconds; zero
    // means close. `None` on a machine whose controller was not given the matching sender.
    wifi_provisioning_receiver: Option<ChannelReceiver<'static, SM, u32, 2>>,
    // The stored shot-log upload config, published by the controller whenever it changes.
    // `None` on a machine with no store, in which case `RequestShotUploadConfig` goes
    // unanswered and the comms processor keeps asking -- the same honest outcome, and the
    // same trap, as `wifi_credentials_receiver` above.
    shot_upload_config_receiver: Option<
        embassy_sync::watch::Receiver<'static, SM, ShotUploadConfig, 2>,
    >,
    // UI commands from an input device the comms processor owns -- a Bluetooth dial. `None`
    // on a machine whose input task was not given the matching receiver, in which case the
    // arm logs and drops rather than silently succeeding.
    //
    // This goes to the *input* task rather than onto `command_sender`, and that is the whole
    // reason it is a channel of its own. A `MachineCommand` reaches the controller, but
    // `Decrement` and `Activate` only mean anything against the menu state the input task
    // owns; routing them through the controller would need either a second copy of that
    // state or a second UI.
    input_command_sender: Option<Sender<'static, SM, InputCommand, 4>>,
    // The stored Bluetooth bonds. `None` on a machine with no bond store, in which case
    // `RequestBluetoothBonds` goes unanswered and the comms processor keeps asking -- the
    // honest outcome, and the same one `wifi_credentials_receiver` gives, since answering
    // an empty list would say "nothing is bonded" and stop the retry forever.
    //
    // Owned here rather than by the controller, and reached directly rather than through
    // `command_sender`. That is a departure from how Wi-Fi credentials are stored, and it is
    // deliberate: the controller gates on a Bluetooth *association* and publishes it inside
    // `Configuration`, but it has no use for the pairing keys. Routing them through it would
    // mean a new `MachineCommand` -- which also travels the debug wire, so it would cost a
    // `DEBUG_PROTOCOL_VERSION` bump -- plus a second store generic on both controllers, all
    // to reach a value neither controller reads. This task already owns `routine_repository`
    // and writes to it on `RoutineWriteChunk`, so a store it owns outright is not a new idea
    // here.
    bond_store: Option<&'static embassy_sync::mutex::Mutex<SM, BS>>,
) {

    // Use a channel to coordinate sending between the tasks
    let tx_channel: Channel<M, Vec<u8>, 10> = Channel::new();
    let tx_sender = tx_channel.sender();
    let tx_receiver = tx_channel.receiver();

    // Use shared state for last sent configuration (heap-allocated to save stack space)
    let last_sent_config: Mutex<M, RefCell<Option<Box<Configuration>>>> = Mutex::new(RefCell::new(None));

    // The last credentials the controller published, cached so `RequestWifiCredentials`
    // can be answered from this task.
    //
    // `Option<StoredWifiCredentials>` rather than a bare `StoredWifiCredentials`, and the
    // outer layer is load-bearing: `None` here means *nothing has been published yet* and
    // the request goes unanswered, while `Some(StoredWifiCredentials(None))` means the
    // controller has told us there is genuinely no network configured. Collapsing the two
    // would answer "no network" before the controller had said anything, and the comms
    // processor stops asking on receipt -- so it would never be corrected.
    let cached_wifi: Mutex<M, RefCell<Option<StoredWifiCredentials>>> = Mutex::new(RefCell::new(None));

    // The last upload config the controller published, cached so `RequestShotUploadConfig`
    // can be answered from this task.
    //
    // The outer `Option` is load-bearing for the same reason as `cached_wifi`'s above, and
    // the distinction is easier to lose here because the inner type has its own empty
    // state: `None` means *nothing has been published yet*, while
    // `Some(ShotUploadConfig::default())` means the controller has said there is genuinely
    // nothing configured. The comms processor stops asking on receipt, so collapsing the
    // two would tell it "uploads are off" before the controller had spoken, and it would
    // never ask again.
    let cached_shot_upload: Mutex<M, RefCell<Option<ShotUploadConfig>>> =
        Mutex::new(RefCell::new(None));

    // Box the machine definition to save stack space
    let machine_definition = Box::new(machine_definition);

    // Send initial machine definition to ESP32
    let initial_response = ApplicationProcessorToCommsProcessorMessage::MachineDefinition((*machine_definition).clone());
    if let Ok(output) = to_allocvec_cobs(&initial_response) {
        let _ = tx_sender.send(output).await;
        info!("Sent initial machine definition to ESP32");
    }

    join5(
        async {
            // Status sending task
            loop {
                let s = status_receiver.next_message_pure().await;
                let wrapped = ApplicationProcessorToCommsProcessorMessage::Status(s);
                let output: Vec<u8> = to_allocvec_cobs(&wrapped).unwrap();
                let _ = tx_sender.send(output).await;
            }
        },
        async {
            // Currently esp-hal doesn't support sending breaks. This has been fixed in main,
            // but until then, we just use a short buffer, an accumulator, and hope for the best. Since we're
            // using HW flow control, we won't miss any bytes.

            // 4096, matching `CobsAccumulator::<4096>` on the comms side of this same
            // link. It was 1024, which is half of `variegated_debug_codec::MAX_FRAME`
            // and therefore *below* what the two hops in front of it accept: a
            // `DebugCommand` passes the host codec at 2048 and the ESP's
            // `CommandDecoder` at 2048, and then anything over ~1024 bytes died here,
            // silently, at the third hop.
            //
            // 4096 rather than the obvious doubling to 2048, because 2048 is the
            // measured worst case with **zero** bytes to spare. The largest command
            // `encode_command` accepts comes to 2048 bytes on this link including the
            // COBS sentinel, and `CobsAccumulator<2048>` holds exactly that and not one
            // byte more -- and the two lengths are equal only by the coincidence that
            // the host envelope's version byte and this message's enum tag are both a
            // single byte, which is not a property anything guarantees: a different
            // leading byte can move a COBS block boundary. Matching the far end takes
            // the arithmetic out of the answer, and costs 3 kB in a 256 kB task arena.
            // Measured by `variegated_debug::relay`'s
            // `a_max_size_command_survives_the_inter_processor_hop`.
            let mut cobs_buf: CobsAccumulator<4096> = CobsAccumulator::new();

            // Where a routine is encoded on its way *out*, one chunk at a time. Sized to
            // the storage ceiling, because that is the largest routine that can exist:
            // the repository writes through a buffer of exactly this size, so anything
            // bigger was never persisted.
            //
            // Declared here rather than inside the arm so its cost is visible. This is
            // the RP2350, with 520 kB of SRAM and no contest for it -- the asymmetry with
            // the comms processor, which counts every static byte against its stack, is
            // the reason the two directions chunk at different sizes.
            let mut routine_tx_scratch = [0u8; ROUTINE_MAX_ENCODED_LEN];
            // And the routine being assembled on its way *in*, if any. `None` between
            // writes, which is most of the time.
            let mut routine_write: Option<RoutineWriteAssembly> = None;

            // Both of these exist to make typed events **edge triggered**, which is the
            // criterion `DebugEvent`'s own documentation sets for promoting a site --
            // and it is load-bearing rather than tidiness, because `bus::emit_event`
            // bypasses the log suppressor entirely. A level-triggered event on this
            // path would turn the 16-slot ring over on its own and evict everything the
            // stream exists to show.
            //
            // `time_synced`: the comms processor sends `CommsStatus` at 1 Hz and its
            // `timestamp` is `Some` on every one of them once SNTP has synced, so
            // reporting per message would mean one `TimeSynchronized` per second
            // forever. Only transitions are interesting.
            let mut time_synced: Option<bool> = None;
            // The last `CommsStatus::sntp_sync_seq` this loop acted on, so a re-anchor
            // happens once per *sync* rather than once per status message. See the field's
            // own documentation: the comms processor's RTC is an RC oscillator, and taking
            // its word once a second is what made the application processor's clock drift
            // despite having a 2 ppm TCXO on the I2C bus.
            //
            // `None` rather than `0` so that a comms processor which had already synced
            // before this loop started -- an application-processor reset, a reflash of one
            // side only -- still anchors on the first message instead of waiting for the
            // next hourly sync.
            let mut last_sntp_sync_seq: Option<u32> = None;
            // `link_healthy`: a garbage burst on the UART produces a COBS delimiter
            // roughly every 256 random bytes, which at 576 kbaud is a few hundred
            // `DeserError`s per second. One event per *burst* -- the first failure
            // after a message that decoded -- says the same thing without the flood.
            let mut link_healthy = true;

            // Sized for a burst, not for a message. `RX::read` returns what is *available*
            // -- one byte on an idle link, a whole burst on a busy one -- so this is a
            // ceiling on how much one wake may carry, not a quantum that has to be filled.
            //
            // That distinction is the entire point of the buffered reader. This used to be
            // `[0u8; 8]` fed to embassy-rp's DMA `read`, which fills its buffer exactly or
            // fails, and an `InputEvent` is 7 bytes on the wire for a keypress: a complete,
            // decodable frame sat here waiting for 1-7 bytes of *unrelated* traffic to
            // finish the block. With nothing else on the radio the next thing to arrive was
            // the comms processor's 1 Hz `CommsStatus`, so a button press could take up to
            // a second to be seen, and how long it actually took depended on whether a
            // Bluetooth scale happened to be streaming.
            let mut buf = [0u8; 64];

            // The raw bytes of the frame the accumulator is currently assembling, so a
            // failure can show what actually arrived rather than only that something did.
            //
            // `CobsAccumulator` does not hand back the frame it failed on -- `DeserError`
            // carries the *remaining* input -- so the only way to see it is to shadow it.
            //
            // 256 bytes rather than the accumulator's 4096: the messages worth reading are
            // this size (a fully provisioned `SetShotUploadSettings` is 166 on the wire), and
            // `raw_len` counts every byte regardless, so an oversized frame is reported with
            // its true length and a truncated dump rather than a wrong one.
            const RAW_DUMP_LEN: usize = 256;
            let mut raw = [0u8; RAW_DUMP_LEN];
            let mut raw_len: usize = 0;

            loop {
                // A failed read tells us nothing about how much of `buf` it touched, so
                // nothing from it may reach the accumulator: feeding a partial buffer to
                // an accumulator that is mid-frame corrupts the *next* message as well as
                // losing this one. Hence `continue` rather than feeding what is there --
                // the same reason this arm has always existed, restated for a `read` that
                // reports a count instead of filling.
                //
                // Reported on the same edge latch as the decode errors below, and with the
                // same event. `OverFull` already shares that latch on the grounds that both
                // mean "a message arrived on this link and we could not read it", which
                // describes a UART error exactly; and these arrive in bursts from the same
                // causes, so per-occurrence reporting would turn the 16-slot ring over on
                // its own. A distinct `DebugEvent` variant would say it better, but that is
                // a wire-format change and this is a warnings pass.
                //
                // The error is reported by `ErrorKind` rather than by its own type: `RX` is
                // now a trait parameter, so the concrete error is whatever the caller's
                // transport uses and this crate cannot bound it on `defmt::Format` without
                // constraining every future caller. `kind()` is the trait's own lossy
                // summary and is all this edge-latched line ever needed.
                let read = match uart_rx.read(&mut buf).await {
                    Ok(n) => n,
                    Err(e) => {
                        if link_healthy {
                            link_healthy = false;
                            error!(
                                "UART read error on the ESP32 link: {:?}",
                                defmt::Debug2Format(&e.kind())
                            );
                            bus::emit_event(DebugEvent::LinkDecodeError);
                        }
                        continue;
                    }
                };

                // `Ok(0)` is end-of-stream in the `embedded_io_async::Read` contract. A
                // UART has no end, so this should not happen -- but falling through with an
                // empty window would skip the `'cobs` loop and spin this task against the
                // executor as fast as it can be polled, which is a livelock rather than a
                // missed message. Treated as "nothing arrived" instead.
                if read == 0 {
                    continue;
                }

                let mut window = &buf[..read];

                'cobs: while !window.is_empty() {
                    // Shadow exactly what `feed` is about to take. It consumes up to and
                    // including the next sentinel, or the whole slice if there is none, so
                    // this stays aligned with the frame being assembled without having to
                    // feed byte by byte -- and an unaligned shadow would report a good frame
                    // as truncated, which is worse than no diagnostic at all.
                    let taken = window
                        .iter()
                        .position(|&b| b == 0)
                        .map_or(window.len(), |i| i + 1);
                    for &b in &window[..taken] {
                        if raw_len < RAW_DUMP_LEN {
                            raw[raw_len] = b;
                        }
                        raw_len += 1;
                    }

                    window = match cobs_buf.feed::<CommsProcessorToApplicationProcessorMessage>(&window) {
                        FeedResult::Consumed => break 'cobs,
                        // A message too large for the buffer above. Counted and
                        // reported exactly like a `DeserError`, and sharing its edge
                        // latch: both mean "a message arrived on this link and we could
                        // not read it", both are produced in bursts by the same causes,
                        // and one event per burst is what keeps the 16-slot ring from
                        // turning over on its own. This arm used to return `new_wind`
                        // and nothing else, so an oversized command vanished with no
                        // counter and no event -- indistinguishable, from the host's
                        // seat, from a command that was never sent.
                        // `bus::note_dropped` deliberately *not* called: that counter
                        // means "this device threw away a frame it wanted to send", and
                        // this is an inbound message it could not read. Conflating them
                        // would make `dev_dropped` on the host's Links row mean two
                        // different failures at once.
                        FeedResult::OverFull(new_wind) => {
                            if link_healthy {
                                link_healthy = false;
                                error!(
                                    "Message from ESP32 overflowed the COBS accumulator: {} bytes, first {}: {=[u8]:02x}",
                                    raw_len,
                                    raw_len.min(RAW_DUMP_LEN),
                                    &raw[..raw_len.min(RAW_DUMP_LEN)]
                                );
                                bus::emit_event(DebugEvent::LinkDecodeError);
                            }
                            raw_len = 0;
                            new_wind
                        }
                        FeedResult::DeserError(new_wind) => {
                            if link_healthy {
                                link_healthy = false;
                                // The bytes, not just the fact. Which of "the frame is short",
                                // "a byte is wrong" and "this is a different message than we
                                // think" it is cannot be told apart from a bare failure, and
                                // the sender already logs the length it wrote -- so the two
                                // numbers together say immediately whether anything was lost
                                // on the wire.
                                error!(
                                    "Failed to deserialize message from ESP32: {} byte frame, first {}: {=[u8]:02x}",
                                    raw_len,
                                    raw_len.min(RAW_DUMP_LEN),
                                    &raw[..raw_len.min(RAW_DUMP_LEN)]
                                );
                                bus::emit_event(DebugEvent::LinkDecodeError);
                            }
                            raw_len = 0;
                            new_wind
                        }
                        FeedResult::Success { data, remaining } => {
                            // Do something with `data: MyData` here.

                            // A message that decoded is what re-arms the decode-error
                            // edge, so a link that recovers can report its next burst.
                            link_healthy = true;
                            // This frame is done with, whatever came next starts a new one.
                            raw_len = 0;

                            let message = data;


                            match message {
                                CommsProcessorToApplicationProcessorMessage::CommsStatus(status) => {
                                    if let Some(now_unix) = status.timestamp {
                                        // The comms processor's RTC starts at zero and only
                                        // becomes a real wall-clock time once SNTP has synced,
                                        // so a small value here means "not synced yet" rather
                                        // than "it is 1970". Ignore those instead of dragging
                                        // our clock back to the epoch.
                                        //
                                        // Do not subtract `Instant::now().as_secs()` from
                                        // `now_unix`: whenever the comms processor reboots
                                        // (reflash, brownout, watchdog) while this one keeps
                                        // running, this processor's uptime exceeds the
                                        // freshly-booted RTC and the subtraction underflows.
                                        //
                                        // The `defmt` calls stay alongside the typed
                                        // events throughout this file, by design: the
                                        // probe view and the debug stream have
                                        // different audiences, and a probe user must
                                        // not lose lines because a host tool gained
                                        // them.
                                        // Only on a *new* sync. `timestamp` arrives on every
                                        // status message, but between syncs it is just the
                                        // comms processor's RC-based RTC free-running, and
                                        // anchoring to that once a second was strictly worse
                                        // than letting the application processor's own
                                        // crystal run -- to say nothing of the battery-backed
                                        // TCXO that re-anchors this clock every minute.
                                        //
                                        // The sequence number is what distinguishes the two.
                                        // It only advances when SNTP actually returned an
                                        // answer, so this branch is a correction from
                                        // outside the machine and nothing else.
                                        let is_new_sync =
                                            last_sntp_sync_seq != Some(status.sntp_sync_seq);

                                        if now_unix >= MIN_PLAUSIBLE_UNIX_TIME {
                                            // The plausibility check stays outside the
                                            // new-sync check, so that a repeated timestamp
                                            // is simply not acted on rather than being
                                            // reported as implausible in the arm below.
                                            if is_new_sync {
                                                last_sntp_sync_seq = Some(status.sntp_sync_seq);
                                                if let Some(now_datetime) = DateTime::<Utc>::from_timestamp(now_unix as i64, 0) {
                                                    // Authoritative, so whatever owns a
                                                    // hardware clock can write the correction
                                                    // through immediately rather than at its
                                                    // next poll.
                                                    let ok = TimeKeeper::set_time_authoritative(now_datetime).is_ok();
                                                    if ok {
                                                        info!("System time synchronized to UTC (timestamp: {})", now_unix);
                                                    } else {
                                                        info!("Failed to set system time");
                                                    }
                                                    // Edge only -- see `time_synced`.
                                                    if time_synced != Some(ok) {
                                                        time_synced = Some(ok);
                                                        bus::emit_event(if ok {
                                                            DebugEvent::TimeSynchronized { unix: now_unix }
                                                        } else {
                                                            DebugEvent::TimeSyncFailed
                                                        });
                                                    }
                                                }
                                            }
                                        } else {
                                            // Unreachable against a current comms build,
                                            // which sends `None` until SNTP syncs rather
                                            // than a small RTC value. Kept for an older
                                            // one, and edge-triggered for the same reason
                                            // the branch above is: it would otherwise fire
                                            // once a second for the whole pre-sync window.
                                            if time_synced != Some(false) {
                                                time_synced = Some(false);
                                                info!("Ignoring implausible timestamp from ESP32: {}", now_unix);
                                                bus::emit_event(DebugEvent::TimeSyncIgnoredImplausible { unix: now_unix });
                                            }
                                        }
                                    }

                                    // Dispatch connection status changes to external sensor handlers
                                    if let Some(dispatcher) = external_sensor_dispatcher {
                                        for (peripheral_id, conn_status) in status.peripheral_connection_status.iter() {
                                            dispatcher.dispatch_connection_status(*peripheral_id, conn_status.connected);
                                        }
                                    }

                                    // Forward CommsStatus to controller
                                    let _ = command_sender.try_send(MachineCommand::UpdateCommsStatus(status));
                                }
                                CommsProcessorToApplicationProcessorMessage::Command(command) => {
                                    info!("Forwarding command: {:?}", command);
                                    // `"machine"` rather than the variant name: `MachineCommand`
                                    // has no `label()`, only a hand-written `defmt::Format`,
                                    // and this is exactly the label
                                    // `DebugCommand::Machine(_)` reports -- so a command
                                    // reads the same in the event log whether it arrived
                                    // over the WebSocket, over USB, or over TCP.
                                    bus::emit_event(DebugEvent::CommandReceived { label: name("machine") });
                                    // Forward Command to controller
                                    let _ = command_sender.try_send(command);
                                }
                                CommsProcessorToApplicationProcessorMessage::DebugCommand(command) => {
                                    info!("Forwarding debug command: {:?}", command);
                                    match command {
                                        DebugCommand::Machine(machine) => {
                                            let _ = command_sender.try_send(machine);
                                        }
                                        // App-debug ops are applied by the example's debug
                                        // command task, which owns the sampler and snapshot
                                        // state. Comms ops arrive here only if the comms
                                        // processor forwarded one it should have handled
                                        // itself; the receiving task ignores them.
                                        //
                                        // `try_send`, never `send`: this future also drives
                                        // the CommsStatus and configuration paths, so
                                        // blocking on a full debug queue would stall the
                                        // link. A dropped injected command is the correct
                                        // trade -- but it must not be a silent one, which
                                        // is what `offer_command` adds over the bare
                                        // `let _ = ` this used to be.
                                        other => {
                                            bus::offer_command(&debug_command_sender, other);
                                        }
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestConfiguration => {
                                    info!("Configuration requested by ESP32");
                                    bus::emit_event(DebugEvent::ConfigurationRequested);

                                    // Serialize inside the lock to minimize clone lifetime.
                                    //
                                    // Through `frame_for_link` rather than `to_allocvec_cobs`,
                                    // which is what every other reply on this link already
                                    // does. A configuration larger than the far side's
                                    // accumulator is not truncated on arrival, it is lost --
                                    // and on the wire that is indistinguishable from this
                                    // reply never being sent, which is the same failure the
                                    // old full-definition `Routines` reply shipped. The guard
                                    // changes nothing for a frame that fits; it makes the
                                    // other case say so.
                                    let output = last_sent_config.lock(|cell| {
                                        cell.borrow().as_ref().and_then(|boxed_config| {
                                            let response = ApplicationProcessorToCommsProcessorMessage::Configuration((**boxed_config).clone());
                                            frame_for_link(&response, "configuration")
                                        })
                                    });

                                    if let Some(output) = output {
                                        let _ = tx_sender.send(output).await;
                                        info!("Sent current configuration to ESP32");
                                        bus::emit_event(DebugEvent::ConfigurationSent);
                                    } else {
                                        // An empty cache used to end here, which made the
                                        // request unanswerable exactly when it mattered
                                        // most: this cache is filled by *forwarding* a
                                        // configuration, so it is empty precisely on the
                                        // boot where no configuration has reached the link
                                        // yet -- the boot where the comms processor is
                                        // asking because it has nothing either.
                                        //
                                        // Ask the controller instead of reporting failure.
                                        // It always holds the real value, and its reply
                                        // goes out through the ordinary publish path a tick
                                        // later, which also fills this cache for next time.
                                        //
                                        // `try_send`, never `send`: this future also drives
                                        // the CommsStatus and configuration paths, and
                                        // blocking on a full command queue would stall the
                                        // link. A full queue means the controller is
                                        // already being asked plenty of things and will
                                        // publish soon regardless.
                                        info!("No configuration cached yet; asking the controller to republish");
                                        let _ = command_sender
                                            .try_send(MachineCommand::RequestConfiguration);
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestMachineDefinition => {
                                    info!("Machine definition requested by ESP32");

                                    // Send the machine definition
                                    let response = ApplicationProcessorToCommsProcessorMessage::MachineDefinition((*machine_definition).clone());
                                    if let Ok(output) = to_allocvec_cobs(&response) {
                                        let _ = tx_sender.send(output).await;
                                        info!("Sent machine definition to ESP32");
                                        bus::emit_event(DebugEvent::MachineDefinitionSent);
                                    } else {
                                        info!("Failed to serialize machine definition");
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestRoutines => {
                                   //info!("Routine summaries requested by ESP32");

                                    if let Some(output) = build_routine_summaries(routine_repository).await {
                                        let _ = tx_sender.send(output).await;
                                        //info!("Sent routine summaries to ESP32");
                                        bus::emit_event(DebugEvent::RoutinesSent);
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestRoutineChunk { index, offset } => {
                                    // Re-encoded per chunk rather than encoded once and
                                    // held. A maximal routine is two chunks, so the
                                    // repeated work is one extra `to_slice` against 2 kB
                                    // of `BTreeMap`-resident routine -- and the
                                    // alternative is cross-message state that has to be
                                    // invalidated when the routine is edited underneath
                                    // it, which is a correctness problem in exchange for
                                    // a memcpy.
                                    let response = {
                                        let mut repo_locked = routine_repository.lock().await;
                                        match repo_locked.get_routine(index).await {
                                            Some(routine) => {
                                                // Framed with the CRC-32C trailer, exactly as flash
                                                // frames it -- see `Routine`'s `Value` impl in
                                                // `routines/core.rs`. A served definition is the one
                                                // copy of a routine that used to travel bare, which
                                                // left every reader either trusting the two hops
                                                // below or, in Plantlet's case, slicing four bytes of
                                                // real postcard off the end and calling them a
                                                // checksum.
                                                //
                                                // The trailer is computed here, on the application
                                                // processor, so it covers the chunking *and* the
                                                // reassembly on the comms processor -- which is the
                                                // splice `serve_query` documents having already
                                                // shipped once, and which nothing else on this path
                                                // can detect.
                                                let crc = Crc::<u32>::new(&CRC_32_ISCSI);
                                                match postcard::to_slice_crc32(routine, &mut routine_tx_scratch, crc.digest()) {
                                                    Ok(encoded) => routine_chunk(index, offset, encoded),
                                                    Err(_) => {
                                                        // Storable but not encodable into
                                                        // 2 kB should be impossible -- the
                                                        // repository used the same ceiling
                                                        // to write it. Reported as absent
                                                        // rather than left to time out.
                                                        error!("Routine {:?} does not fit the chunk scratch buffer", index);
                                                        ApplicationProcessorToCommsProcessorMessage::RoutineNotFound(index)
                                                    }
                                                }
                                            }
                                            None => ApplicationProcessorToCommsProcessorMessage::RoutineNotFound(index),
                                        }
                                    };

                                    if let Some(output) = frame_for_link(&response, "routine chunk") {
                                        let _ = tx_sender.send(output).await;
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::RoutineWriteChunk { index, offset, total, last, bytes } => {
                                    let outcome = accept_routine_write_chunk(
                                        &mut routine_write,
                                        index,
                                        offset,
                                        total,
                                        last,
                                        &bytes,
                                    );

                                    // `Some` only on the last chunk, or on a refusal. The
                                    // intermediate chunks are silent: acknowledging each
                                    // one would double the traffic to tell the far side
                                    // something it already knows, since it is sending them
                                    // back to back under a lock.
                                    if let Some(assembled) = outcome {
                                        let result = match assembled {
                                            Err(e) => RoutineWriteOutcome::Failed(e),
                                            Ok(()) => {
                                                let state = routine_write.take();
                                                match state {
                                                    None => RoutineWriteOutcome::Failed(RoutineWriteError::Malformed),
                                                    Some(state) => {
                                                        store_routine(
                                                            routine_repository,
                                                            state.index,
                                                            &state.buffer[..state.len],
                                                        )
                                                        .await
                                                    }
                                                }
                                            }
                                        };

                                        if matches!(result, RoutineWriteOutcome::Failed(_)) {
                                            // A refusal ends the sequence: the far side
                                            // stops sending, and leaving a half-filled
                                            // buffer behind would make the next write look
                                            // like a continuation of this one.
                                            routine_write = None;
                                        }

                                        let response = ApplicationProcessorToCommsProcessorMessage::RoutineWriteResult(result);
                                        if let Some(output) = frame_for_link(&response, "routine write result") {
                                            let _ = tx_sender.send(output).await;
                                        }

                                        // No summary push here. `store_routine` went
                                        // through the repository, which raises
                                        // `ROUTINES_CHANGED`, and the arm at the bottom of
                                        // this function does the sending -- for this write
                                        // and for every other mutation, including the ones
                                        // that never pass through this task at all. A
                                        // refused write touches nothing and so says
                                        // nothing, where the push that used to be here
                                        // fired regardless and relied on the far side's
                                        // cache comparison to swallow it.
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::DeleteRoutine(index) => {
                                    let outcome = delete_routine(routine_repository, index).await;

                                    // No summary push here either. A successful delete went
                                    // through the repository, which raises
                                    // `ROUTINES_CHANGED`, and the arm at the bottom of this
                                    // function sends the new list -- see the longer note in
                                    // the write arm above.
                                    let response =
                                        ApplicationProcessorToCommsProcessorMessage::RoutineDeleteResult {
                                            index,
                                            outcome,
                                        };
                                    if let Some(output) = frame_for_link(&response, "routine delete result") {
                                        let _ = tx_sender.send(output).await;
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestShotLogList(request) => {
                                    info!(
                                        "Shot log page requested by ESP32 (limit {}, cursor {:?})",
                                        request.limit,
                                        request.before
                                    );
                                    forward_shot_log_query(
                                        shot_log_query_sender.as_ref(),
                                        ShotLogQuery::List(request),
                                        &tx_sender,
                                    );
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestShotLogChunk { id, offset } => {
                                    forward_shot_log_query(
                                        shot_log_query_sender.as_ref(),
                                        ShotLogQuery::Chunk { id, offset },
                                        &tx_sender,
                                    );
                                }
                                CommsProcessorToApplicationProcessorMessage::ExternalPeripheralSensorReading(reading) => {
                                    if let Some(dispatcher) = external_sensor_dispatcher {
                                        dispatcher.dispatch_reading(&reading);
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestBluetoothPeripherals => {
                                    info!("Bluetooth peripherals requested by ESP32");

                                    // Answered out of the cached configuration rather
                                    // than from a store of its own. The association list
                                    // travels inside `Configuration` for the browser's
                                    // benefit anyway, so this arm has the current list to
                                    // hand and a second source would only be a second
                                    // thing to keep in step.
                                    //
                                    // An empty list is a real answer, and the distinction
                                    // that matters is between "no associations" and "no
                                    // configuration published yet" -- the comms processor
                                    // stops asking on receipt, so answering before the
                                    // controller has published would tell it there are no
                                    // peripherals and never correct that.
                                    let output = last_sent_config.lock(|cell| {
                                        cell.borrow().as_ref().map(|boxed_config| {
                                            let response = ApplicationProcessorToCommsProcessorMessage::BluetoothPeripherals(
                                                boxed_config.bluetooth_peripherals.clone(),
                                            );
                                            to_allocvec_cobs(&response).ok()
                                        })
                                    });

                                    match output {
                                        Some(Some(output)) => {
                                            let _ = tx_sender.send(output).await;
                                            info!("Sent Bluetooth peripherals to ESP32");
                                        }
                                        Some(None) => info!("Failed to serialize Bluetooth peripherals"),
                                        None => info!("No configuration published yet; not answering"),
                                    }
                                }
                                // Scan results reach the controller as commands, the same
                                // route `CommsStatus` takes: this is the only channel from
                                // this task into the controller, and the controller is
                                // what assembles `Status`.
                                //
                                // `try_send`, never `send().await`. This runs in the UART
                                // reader, and back-pressure here stalls `Status` and every
                                // debug frame behind it. A scan result is the most
                                // droppable thing on this link -- the device is still
                                // advertising and will be reported again.
                                CommsProcessorToApplicationProcessorMessage::BluetoothPeripheralDiscovered(device) => {
                                    if command_sender
                                        .try_send(MachineCommand::UpdateBluetoothScan(
                                            variegated_controller_types::bluetooth::BluetoothScanUpdate::Discovered(device),
                                        ))
                                        .is_err()
                                    {
                                        info!("Dropped a Bluetooth scan result: command channel full");
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::BluetoothScanFinished { reports_dropped } => {
                                    info!("ESP32 reports the Bluetooth scan finished");
                                    if command_sender
                                        .try_send(MachineCommand::UpdateBluetoothScan(
                                            variegated_controller_types::bluetooth::BluetoothScanUpdate::Finished { reports_dropped },
                                        ))
                                        .is_err()
                                    {
                                        // Not fatal: the controller times the scan out on
                                        // its own deadline precisely because this message
                                        // is not guaranteed to arrive.
                                        info!("Dropped the Bluetooth scan-finished message: command channel full");
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestWifiCredentials => {
                                    info!("Wi-Fi credentials requested by ESP32");

                                    // Answered from the cached copy the controller
                                    // publishes, not from a store: this task holds no store
                                    // handle, and taking a flash lock on the UART reader
                                    // would put every message on the link behind a flash
                                    // read.
                                    //
                                    // Nothing published yet is *not* answered with `None`.
                                    // The comms processor stops asking on receipt, so a
                                    // premature `None` would tell it there is no network and
                                    // never correct itself -- the same trap the Bluetooth
                                    // arm above documents.
                                    let output = cached_wifi.lock(|cell| {
                                        cell.borrow().as_ref().map(|credentials| {
                                            let response = ApplicationProcessorToCommsProcessorMessage::WifiCredentials(
                                                credentials.0.clone(),
                                            );
                                            to_allocvec_cobs(&response).ok()
                                        })
                                    });

                                    match output {
                                        Some(Some(output)) => {
                                            let _ = tx_sender.send(output).await;
                                            info!("Sent Wi-Fi credentials to ESP32");
                                        }
                                        Some(None) => info!("Failed to serialize Wi-Fi credentials"),
                                        None => info!("No Wi-Fi credentials published yet; not answering"),
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestShotUploadConfig => {
                                    info!("Shot upload config requested by ESP32");

                                    // Answered from the cached copy, exactly like the
                                    // Wi-Fi arm above and for the same two reasons: this
                                    // task holds no store handle, and a flash read on the
                                    // UART reader would put every message on the link
                                    // behind it.
                                    //
                                    // Nothing published yet is not answered at all. An
                                    // empty config is a *complete* answer meaning "uploads
                                    // are not configured", and the comms processor stops
                                    // asking on receipt -- so sending one prematurely would
                                    // disable uploads until the next reboot.
                                    let output = cached_shot_upload.lock(|cell| {
                                        cell.borrow().as_ref().map(|config| {
                                            let response = ApplicationProcessorToCommsProcessorMessage::ShotUploadConfig(
                                                config.clone(),
                                            );
                                            to_allocvec_cobs(&response).ok()
                                        })
                                    });

                                    match output {
                                        Some(Some(output)) => {
                                            let _ = tx_sender.send(output).await;
                                            info!("Sent shot upload config to ESP32");
                                        }
                                        Some(None) => info!("Failed to serialize shot upload config"),
                                        None => info!("No shot upload config published yet; not answering"),
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::WifiCredentialsProvisioned(credentials) => {
                                    // Straight to the controller as a command, the same
                                    // route scan results take: this is the only channel from
                                    // this task into the controller.
                                    //
                                    // `try_send`, because this is the UART reader and it
                                    // must not block -- but a drop here is worth more than
                                    // the `info!` a dropped scan result gets. The user has
                                    // just provisioned a network and losing this means it is
                                    // never persisted, so the machine rejoins nothing after
                                    // a power cycle while the phone said it succeeded.
                                    if command_sender
                                        .try_send(MachineCommand::SetWifiCredentials(credentials))
                                        .is_err()
                                    {
                                        info!("Dropped provisioned Wi-Fi credentials: command channel full");
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::WifiProvisioningIdentify => {
                                    if command_sender.try_send(MachineCommand::IdentifyMachine).is_err() {
                                        info!("Dropped an identify request: command channel full");
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::RequestBluetoothBonds => {
                                    // Pruned against the current associations before it goes
                                    // out. A bond for a device the user has disassociated is
                                    // not merely untidy: there are only
                                    // `MAX_BLUETOOTH_PERIPHERALS` slots, so dead keys can
                                    // fill the list and leave a genuinely new pairing with
                                    // nowhere to be stored.
                                    //
                                    // Done here rather than when the association is removed
                                    // because the controller is what handles the removal, and
                                    // it does not own this store. This arm runs at every boot
                                    // of the comms processor, which is often enough for a
                                    // list that is at most four entries long.
                                    let associated: Option<heapless::Vec<[u8; 6], { variegated_controller_types::bluetooth::MAX_BLUETOOTH_PERIPHERALS }>> =
                                        last_sent_config.lock(|cell| {
                                            cell.borrow().as_ref().map(|boxed_config| {
                                                boxed_config
                                                    .bluetooth_peripherals
                                                    .iter()
                                                    .map(|association| association.address)
                                                    .collect()
                                            })
                                        });

                                    match (&bond_store, associated) {
                                        // No configuration published yet means the
                                        // association list is unknown, and pruning against an
                                        // unknown list would delete every bond on the
                                        // machine. Left unanswered instead: the comms
                                        // processor retries, and by then the controller has
                                        // published.
                                        (Some(_), None) => {
                                            info!("Deferred bonds: no configuration published yet");
                                        }
                                        (Some(store), Some(associated)) => {
                                            let mut store = store.lock().await;
                                            let mut bonds = store
                                                .load_settings()
                                                .await
                                                .unwrap_or_default();
                                            let dropped = bonds.retain_associated(&associated);
                                            if dropped > 0 {
                                                info!("Dropped {} bond(s) with no association", dropped);
                                                if store.save_settings(&bonds).await.is_err() {
                                                    info!("Failed to store pruned bonds");
                                                }
                                            }

                                            let response =
                                                ApplicationProcessorToCommsProcessorMessage::BluetoothBonds(
                                                    bonds,
                                                );
                                            match to_allocvec_cobs(&response) {
                                                Ok(output) => {
                                                    let _ = tx_sender.send(output).await;
                                                    info!("Sent Bluetooth bonds to ESP32");
                                                }
                                                Err(_) => info!("Failed to serialize Bluetooth bonds"),
                                            }
                                        }
                                        (None, _) => {
                                            info!("Ignored a bond request: no bond store on this machine");
                                        }
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::BluetoothBondStored(bond) => {
                                    // Sent once, immediately after pairing, with no reply and
                                    // no retry -- so a failure here is the difference between
                                    // a dial that survives a reboot and one that has to be
                                    // paired again. Logged loudly for that reason.
                                    match &bond_store {
                                        Some(store) => {
                                            let mut store = store.lock().await;
                                            let mut bonds =
                                                store.load_settings().await.unwrap_or_default();
                                            if bonds.upsert(bond) {
                                                if store.save_settings(&bonds).await.is_err() {
                                                    info!("Failed to store a Bluetooth bond");
                                                } else {
                                                    info!("Stored a Bluetooth bond");
                                                }
                                            } else {
                                                info!("Dropped a Bluetooth bond: no free slot");
                                            }
                                        }
                                        None => info!("Dropped a Bluetooth bond: no bond store"),
                                    }
                                }
                                CommsProcessorToApplicationProcessorMessage::InputEvent(id, command) => {
                                    // The id is deliberately not forwarded. It exists on the
                                    // wire so a second input device costs an association
                                    // rather than another message, but every input device
                                    // drives the same one UI -- two dials on a machine should
                                    // both work, not take turns -- so there is nothing here
                                    // to route on.
                                    match &input_command_sender {
                                        Some(sender) => {
                                            // Dropped rather than awaited, like every other
                                            // arm in this loop: blocking the UART reader to
                                            // deliver a keypress would stall the link for
                                            // status and shot data too. A lost step on a
                                            // dial is a step the user simply turns again.
                                            if sender.try_send(command).is_err() {
                                                info!("Dropped an input command from 0x{:04X}: channel full", id);
                                            }
                                        }
                                        None => {
                                            info!("Ignored an input command from 0x{:04X}: no input sink on this machine", id);
                                        }
                                    }
                                }
                                _ => {
                                    info!("Received unknown message type");
                                }
                            }



                            remaining
                        }
                    };
                }
            }
        },
        async {
            // UART TX task - handles all outgoing data
            loop {
                let data = tx_receiver.receive().await;
                // `write_all`, not `write`. `Write::write` is permitted to take only part
                // of the slice, and a buffered writer takes exactly what fits in its ring
                // -- so `write` here would silently truncate every frame larger than the
                // free space, which for a shot-log listing at `SHOT_LOG_LIST_BUDGET` is
                // most of it. embassy-rp's DMA `UartTx::write` happened to write the whole
                // slice, which is why the distinction did not matter before.
                let _ = uart_tx.write_all(data.as_slice()).await;
            }
        },
        async {
            // Configuration monitoring and proactive broadcasting
            loop {
                let config = configuration_receiver.next_message_pure().await;

                // Whether the association list changed, decided *before* the cached
                // configuration is replaced below.
                //
                // The comms processor is told about associations twice over, and this is
                // the deliberate half: it holds no configuration of its own, so an
                // association the user just created does not exist to the radio until a
                // message says so. Riding on this publish rather than a channel of its
                // own means there is one place that decides the list changed, and it is
                // the same place that already decided the configuration did.
                //
                // `None` -- nothing published yet -- counts as changed, so the first
                // configuration of a boot also delivers the list.
                let bluetooth_changed = last_sent_config.lock(|cell| {
                    cell.borrow()
                        .as_ref()
                        .map(|previous| previous.bluetooth_peripherals != config.bluetooth_peripherals)
                        .unwrap_or(true)
                });

                // Serialize and box in tight scope to minimize stack usage.
                //
                // Guarded like every other reply on this link -- see the note at the
                // `RequestConfiguration` arm. This is the send that matters most for it:
                // it now runs every ten seconds on both controllers, so a configuration
                // that has outgrown the link would otherwise be silently discarded by the
                // far accumulator six times a minute with nothing anywhere to say why the
                // browser never sees one.
                let response = ApplicationProcessorToCommsProcessorMessage::Configuration(config.clone());
                let output = frame_for_link(&response, "configuration");

                if let Some(output) = output {
                    let _ = tx_sender.send(output).await;
                    //info!("Sent updated configuration to ESP32");

                    if bluetooth_changed {
                        let response = ApplicationProcessorToCommsProcessorMessage::BluetoothPeripherals(
                            config.bluetooth_peripherals.clone(),
                        );
                        if let Ok(output) = to_allocvec_cobs(&response) {
                            let _ = tx_sender.send(output).await;
                            info!("Sent updated Bluetooth peripherals to ESP32");
                        } else {
                            info!("Failed to serialize Bluetooth peripherals");
                        }
                    }

                    // Box after successful send
                    last_sent_config.lock(|cell| {
                        cell.replace(Some(Box::new(config)));
                    });
                } else {
                    info!("Failed to serialize configuration");
                }
            }
        },
        // `join5` is embassy-futures' maximum arity, so the sixth concurrent future
        // is nested here rather than promoted. Nesting is free -- `join` polls both
        // arms on every wake exactly as a hypothetical `join6` would -- and it keeps
        // the five existing arms textually where they were.
        //
        // `tx_sender` is `Copy`, so the four futures above are unaffected by these
        // two taking handles of their own.
        join4(
            debug_relay::relay(tx_sender, link_baud),
            async {
                // Answers from the storage task on the other core, put on the wire.
                //
                // A separate arm rather than an inline reply in the reader above,
                // because the card is not reachable from here: the reader can only ask,
                // and the answer arrives whenever the storage task gets to it -- which
                // may be after a shot has finished storing.
                let Some(receiver) = shot_log_reply_receiver else {
                    core::future::pending::<()>().await;
                    return;
                };

                loop {
                    let reply = receiver.receive().await;
                    let response = match reply {
                        ShotLogReply::List(list) => {
                            ApplicationProcessorToCommsProcessorMessage::ShotLogList(list)
                        }
                        ShotLogReply::Chunk { id, offset, total, last, bytes } => {
                            ApplicationProcessorToCommsProcessorMessage::ShotLogChunk {
                                id,
                                offset,
                                total,
                                last,
                                bytes,
                            }
                        }
                        ShotLogReply::Annotations { id, annotations } => {
                            ApplicationProcessorToCommsProcessorMessage::ShotLogAnnotations {
                                id,
                                annotations,
                            }
                        }
                        ShotLogReply::Error(e) => {
                            info!("Shot log request failed: {:?}", e);
                            ApplicationProcessorToCommsProcessorMessage::ShotLogError(e)
                        }
                    };

                    // `send().await`, matching the scale-command arm below rather than
                    // `debug_relay`'s reserved-capacity dance. A shot-log reply is
                    // solicited traffic that arrives at most once per request, so it
                    // cannot flood the queue the way high-rate debug frames can -- and
                    // dropping it would leave the far side waiting out a timeout for an
                    // answer this processor had already produced.
                    //
                    // Through `frame_for_link`, not a bare encode. An oversized frame is
                    // *lost* on the far side rather than truncated, so without this an
                    // overrun and a dead link are the same event -- which is exactly how
                    // the old `Routines` reply failed. `SHOT_LOG_LIST_BUDGET` is what
                    // keeps a listing under the limit; this is what says so out loud on
                    // the day something else does not.
                    if let Some(output) = frame_for_link(&response, "a shot log reply") {
                        let _ = tx_sender.send(output).await;
                    }
                }
            },
            // The two peripheral-command arms, paired so that `join4` keeps its arity --
            // nesting is free, as the note on the fourth slot below explains. They are
            // together because they are the same job for two different peripherals.
            join(
            async {
                // Scale commands bound for a scale the comms processor owns.
                //
                // `Group`'s `Box<dyn ScaleController>` cannot reach `tx_channel` --
                // it is a local of this function, and the controller lives in a
                // different task entirely -- so a `BluetoothScaleController` pushes
                // onto a static channel and this arm is what puts it on the wire.
                //
                // `Option`, because only a machine that actually has a Bluetooth
                // scale has such a channel; `single-boiler` and the Belka-less
                // `dual-boiler` build pass `None` and this arm parks forever.
                let Some(receiver) = scale_command_receiver else {
                    core::future::pending::<()>().await;
                    return;
                };

                loop {
                    let (peripheral_id, op) = receiver.receive().await;

                    // `send().await`, not the `try_send` + reserved-capacity dance
                    // `debug_relay` performs. That reservation exists to stop
                    // high-rate debug frames from filling the shared ten-slot queue
                    // and blocking machine traffic. A tare *is* machine traffic and
                    // arrives at most once a shot, so it belongs on the same footing
                    // as `Status` and `Configuration` above.
                    let response = ApplicationProcessorToCommsProcessorMessage::ScaleCommand(peripheral_id, op);
                    if let Ok(output) = to_allocvec_cobs(&response) {
                        let _ = tx_sender.send(output).await;
                        info!("Sent scale command to ESP32 for peripheral {}", peripheral_id);
                    } else {
                        info!("Failed to serialize scale command");
                    }
                }
            },
            async {
                // The same arm again for a brew sensor's display. `None` on every machine
                // but the GS3, where it parks forever exactly as the scale arm above does.
                //
                // A separate channel rather than a widened `ScaleOp`: the two peripherals
                // share no vocabulary, and the wire keeps them apart for the same reason.
                let Some(receiver) = brew_sensor_command_receiver else {
                    core::future::pending::<()>().await;
                    return;
                };

                loop {
                    let (peripheral_id, op) = receiver.receive().await;

                    // `send().await`, like the scale arm: this is machine traffic, arriving
                    // twice a shot, and belongs on the same footing as `Status`.
                    let response = ApplicationProcessorToCommsProcessorMessage::BrewSensorCommand(peripheral_id, op);
                    if let Ok(output) = to_allocvec_cobs(&response) {
                        let _ = tx_sender.send(output).await;
                        info!("Sent brew sensor command to ESP32 for peripheral {}", peripheral_id);
                    } else {
                        info!("Failed to serialize brew sensor command");
                    }
                }
            },
            ),
            // `join4` is embassy-futures' maximum arity, so the fourth slot carries four
            // futures nested rather than one. Nesting is free -- a `join` polls both arms
            // on every wake exactly as a hypothetical `join7` would -- and it keeps the
            // three arms above textually where they were.
            join(
            async {
                // Bluetooth scan requests the controller has already accepted.
                //
                // The controller is what decides whether a scan may run -- it is the only
                // processor that knows a shot is in progress -- so anything arriving here
                // has been vetted and is simply put on the wire.
                let Some(receiver) = bluetooth_scan_receiver else {
                    core::future::pending::<()>().await;
                    return;
                };

                loop {
                    let duration_ms = receiver.receive().await;

                    // `send().await` for the same reason the scale command above uses it:
                    // this is machine traffic, arriving at most a few times a session, and
                    // belongs on the same footing as `Status` rather than behind the debug
                    // relay's reserved-capacity dance.
                    let response = ApplicationProcessorToCommsProcessorMessage::StartBluetoothScan { duration_ms };
                    if let Ok(output) = to_allocvec_cobs(&response) {
                        let _ = tx_sender.send(output).await;
                        info!("Sent Bluetooth scan request to ESP32 ({} ms)", duration_ms);
                    } else {
                        info!("Failed to serialize Bluetooth scan request");
                    }
                }
            },
            join(
                async {
                    // Credentials, pushed whenever the controller changes them.
                    //
                    // Unprompted, like the Bluetooth association list, and for the same
                    // reason: the comms processor holds no configuration of its own, so a
                    // network the user just provisioned does not exist to the radio until a
                    // message says so. Unlike that list this cannot ride on the
                    // `Configuration` publish -- that path ends at the browser.
                    let Some(mut receiver) = wifi_credentials_receiver else {
                        core::future::pending::<()>().await;
                        return;
                    };

                    loop {
                        let credentials = receiver.changed().await;

                        // Cached before sending, not after. The request arm answers from
                        // this, and a `RequestWifiCredentials` that arrives while the send
                        // below is queued should be answered with the new value rather than
                        // the old one.
                        cached_wifi.lock(|cell| {
                            cell.replace(Some(credentials.clone()));
                        });

                        let response = ApplicationProcessorToCommsProcessorMessage::WifiCredentials(
                            credentials.0.clone(),
                        );
                        if let Ok(output) = to_allocvec_cobs(&response) {
                            let _ = tx_sender.send(output).await;
                            // Logged as configured-or-not. The SSID alone would be harmless
                            // here, but a log line that prints half a credential is one edit
                            // away from printing all of it.
                            info!(
                                "Sent Wi-Fi credentials to ESP32 ({})",
                                if credentials.0.is_some() { "configured" } else { "none" }
                            );
                        } else {
                            info!("Failed to serialize Wi-Fi credentials");
                        }
                    }
                },
                join(
                async {
                    // Provisioning-window requests the controller has already accepted --
                    // it is the only processor that knows a shot is in progress, so
                    // anything arriving here has been vetted and is simply put on the wire.
                    let Some(receiver) = wifi_provisioning_receiver else {
                        core::future::pending::<()>().await;
                        return;
                    };

                    loop {
                        let duration_ms = receiver.receive().await;

                        // Zero means close. One channel carries both so that a close cannot
                        // overtake the open it was meant to cancel, which two channels
                        // polled by a `select` could do.
                        let response = if duration_ms == 0 {
                            ApplicationProcessorToCommsProcessorMessage::CloseWifiProvisioningWindow
                        } else {
                            ApplicationProcessorToCommsProcessorMessage::OpenWifiProvisioningWindow { duration_ms }
                        };

                        if let Ok(output) = to_allocvec_cobs(&response) {
                            let _ = tx_sender.send(output).await;
                            info!("Sent Wi-Fi provisioning window request to ESP32 ({} ms)", duration_ms);
                        } else {
                            info!("Failed to serialize the provisioning window request");
                        }
                    }
                },
                join(
                async {
                    // Shot-log events, pushed the moment the card changes.
                    //
                    // Its own channel rather than the reply path, which has no
                    // correlation id: an unsolicited message there can be collected by a
                    // client waiting on a listing.
                    let Some(receiver) = shot_log_event_receiver else {
                        core::future::pending::<()>().await;
                        return;
                    };

                    loop {
                        let event = receiver.receive().await;

                        // `send().await`, like the scale command and the shot-log reply:
                        // this is machine traffic arriving at most once a shot, so it
                        // belongs on the same footing as `Status` rather than behind the
                        // debug relay's reserved-capacity dance.
                        let response =
                            ApplicationProcessorToCommsProcessorMessage::ShotLogEvent(event);
                        if let Some(output) = frame_for_link(&response, "a shot log event") {
                            let _ = tx_sender.send(output).await;
                            info!("Sent a shot log event to ESP32");
                        }
                    }
                },
                join(
                async {
                    // A fresh summary list whenever the routines change, whoever changed
                    // them.
                    //
                    // The repository raises `ROUTINES_CHANGED` from `add`, `update` and
                    // `remove`, so this covers the chunked write handled a few hundred
                    // lines above *and* a `MachineCommand::RemoveRoutine` the controller
                    // handled on the other core, which nothing here ever sees. It replaces
                    // a push bolted to the write path, which reported the one mutation it
                    // was written next to and left deletion to a half-second guess on the
                    // comms processor.
                    //
                    // Unlike the two arms above there is no `Option`: every machine has a
                    // routine repository, and it is already a parameter of this function.
                    loop {
                        routine::ROUTINES_CHANGED.wait().await;

                        if let Some(output) = build_routine_summaries(routine_repository).await {
                            let _ = tx_sender.send(output).await;
                            info!("Sent routine summaries to ESP32 (unprompted)");
                            bus::emit_event(DebugEvent::RoutinesSent);
                        }
                    }
                },
                async {
                    // The shot-log upload config, pushed whenever the controller changes
                    // it -- so a token rotated over the CLI takes effect without either
                    // processor rebooting.
                    //
                    // Unprompted for the same reason as the credentials arm above: the
                    // comms processor holds no configuration of its own. And like that
                    // one, this cannot ride on the `Configuration` publish, because the
                    // token is a secret and that path ends at the browser.
                    let Some(mut receiver) = shot_upload_config_receiver else {
                        core::future::pending::<()>().await;
                        return;
                    };

                    loop {
                        let config = receiver.changed().await;

                        // Cached before sending, not after -- a `RequestShotUploadConfig`
                        // arriving while the send below is queued should be answered with
                        // the new value rather than the old one.
                        cached_shot_upload.lock(|cell| {
                            cell.replace(Some(config.clone()));
                        });

                        let response =
                            ApplicationProcessorToCommsProcessorMessage::ShotUploadConfig(
                                config.clone(),
                            );
                        if let Ok(output) = to_allocvec_cobs(&response) {
                            let _ = tx_sender.send(output).await;
                            // Configured-or-not, never the value. The endpoint would be
                            // harmless, but this line sits one edit away from the token
                            // beside it.
                            info!(
                                "Sent shot upload config to ESP32 (endpoint {}, token {})",
                                if config.endpoint.is_some() { "set" } else { "unset" },
                                if config.token.is_some() { "set" } else { "unset" }
                            );
                        } else {
                            info!("Failed to serialize the shot upload config");
                        }
                    }
                },
                ),
                ),
                ),
            ),
            ),
        ),
    ).await;
}