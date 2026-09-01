# The Plantlet uplink

A bidirectional link between the comms firmware and Plantlet, carrying status, shot logs and
routines over a Noise-encrypted WebSocket, with an HTTP POST fallback at the same URL.

Today the machine can only push, only shots, and only one at a time: `POST /api/noise-upload`
opens a TCP connection, performs a one-way `Noise_X` handshake, streams a sealed body and
closes. Plantlet cannot ask the machine anything, cannot tell whether a machine is switched
on, and cannot put a routine on one.

This document covers both halves. The firmware half lives in `variegated-rs`; the server half
lives in `variegated-plantlet-ts`, whose `ARCHITECTURE.md` is the authority on everything this
one does not change.

---

## 0. What is being reused, and what that constrains

Four existing pieces carry most of this, and their constraints set the shape of the rest.

**`Noise_X_25519_ChaChaPoly_SHA256`, over plain HTTP.** The initiator is
`crates/variegated-shot-upload/src/noise.rs`; the responder is hand-written in
`apps/worker/src/noise.ts`; the two are pinned against each other by
`fixtures/noise-vectors.json` rather than by each side's own round-trip, which would pass
against its own misunderstanding.

**There is no TLS in this build, and there cannot be.** MbedTLS wanted about six kilobytes for
a handshake, and when it ran out it reported a certificate error rather than an allocation
failure. So "WebSocket over HTTP" is a hard constraint, not a preference, and Noise is the only
confidentiality this link has.

**A bidirectional protocol already exists**, in `variegated-comms-api-types/src/ws_types.rs`:
`WsMessage` with `Query`/`QueryReply`, `ClientQuery`, `QueryOk` and `QueryError`. That is the
LAN socket to the SPA. Its `QueryOutcome` family is reused here verbatim; its envelope
deliberately is not. See §2.

**Plantlet already has a routine library** — the `routines` table, `/api/routines`,
`packages/routine` — and a `machine` table carrying `noise_public_key`. Routines are stored as
`postcard::to_slice_crc32(&Routine)`: the encoding followed by a little-endian **CRC-32C
trailer**, written and read identically by `packages/routine/src/encode.ts`. That trailer is
this design's routine identity, and it costs nothing on either side because both already hold
it.

Two hard numbers from `docs/comms-firmware-memory-budget.md`, measured across a full Improv
provisioning cycle:

* heap peak **82,356** of 122,880 — about 40 kB free
* stack peak **94,028** of 96,480 — **2,452 bytes of headroom**

`.bss` and `.stack` are the same pool, so an embassy task's arena comes straight out of that
2,452 bytes. §5 gates on measuring both rather than reasoning about them.

---

## 1. Transport

### 1.1 One URL, two shapes

`ShotUploadSettings.endpoint` is unchanged — still `http+noise://host/api/noise-upload`, still
`Scheme::Noise`, still port 80. No machine needs reconfiguring. The endpoint is method-routed:

```
GET  /api/noise-upload   Upgrade: websocket   -> MachineUplink Durable Object
POST /api/noise-upload   sealed body          -> worker, direct
```

Both carry the same payload type and identify the machine the same way. They differ only in
handshake pattern and framing, each chosen by what its transport can do.

### 1.2 Both handshakes travel in a header

```
X-Variegated-Noise: <base64url, unpadded>
```

For the socket this is a necessity rather than a preference. The worker must know *which*
machine before it can pick a Durable Object, and the machine's identity is inside the encrypted
handshake. Putting the handshake in the upgrade request means the machine is known before any
DO is instantiated, and — more valuable — no handshake state ever has to survive hibernation.
There is no lobby object and no unauthenticated routing hint in the URL.

For POST it buys symmetry: one place to look for a handshake, one decoder, one shape to
reason about. It also separates the credential from the payload, so the body is uniformly
"sealed frames" with nothing special about its first hundred bytes.

Nothing identifying travels in cleartext. The device's static public key is encrypted inside
the handshake exactly as it is today, and the header is opaque bytes.

### 1.3 The socket: `Noise_IK`

```text
<- s                       (the server's static key, provisioned into the machine)
...
-> e, es, s, ss            X-Variegated-Noise on the GET
<- e, ee, se               X-Variegated-Noise on the 101
   [split, both ciphers kept]
```

The first message is byte-identical in structure to today's `Noise_X`, so the same provisioned
static keys work unchanged and no machine is re-keyed.

`IK` rather than keeping `X` and un-discarding its second cipher, which would have been a
smaller diff. `X` gives the responder no ephemeral, so a session has **no forward secrecy at
all**: a later compromise of the server's static key retroactively decrypts every recorded
session, because both `es` and `ss` become computable from the transcript. That is tolerable
for a one-shot upload and not for a link that stays open for months. `IK` also lets the machine
authenticate the responder, so a session cannot be served by anyone holding only the transcript.

```rust
pub const UPLINK_PROLOGUE: &[u8] = b"variegated-uplink/noise-ik/1";

/// Fixed-width, version first, so a peer reads the version before committing to a layout.
pub struct UplinkHello {
    pub version: u16,   // UPLINK_HELLO_VERSION
    pub max_frame: u32, // the largest sealed frame this side will accept
}
```

`max_frame` is declared by both sides and means the same thing in both directions: the largest
sealed frame the *sender of this hello* will accept inbound. The machine declares
`MAX_CLIENT_FRAME_LEN`; Plantlet declares its record ceiling (§1.6).

Message 1 is 32 + 48 + (6 + 16) = **102 bytes**; message 2 is 32 + (6 + 16) = **54 bytes**.
Both fit a header comfortably.

An unknown device key is refused **before** message 2 is written, so a prober gets no server
ephemeral and learns nothing beyond "not accepted". Replaying a captured message 1 draws a
fresh server ephemeral, so `ee` differs and the replayer can neither read message 2 nor forge a
transport frame.

### 1.4 POST: `Noise_X`, and why the pattern differs

POST keeps `Noise_X` because it must. In a two-message pattern the initiator cannot produce a
transport message until it has processed the responder's reply, and HTTP will not deliver that
reply until the request body is complete. A streamed upload over a single POST therefore needs
a pattern that splits after its first message. That constraint has not changed.

So the firmware carries two patterns. `noise-protocol` supports both, so the cost is one more
pattern constant and a second vector file — not a second implementation.

```rust
pub const POST_PROLOGUE: &[u8] = b"variegated-uplink/noise-x/2";

pub struct PostHello {
    pub version: u16,    // POST_HELLO_VERSION = 2
    pub chunk_len: u32,  // plaintext bytes per frame
    pub total: u32,      // authenticated plaintext length
}
```

`total` stays, and stays inside the handshake, for the reason it exists today: per-frame AEAD
and the nonce chain already defeat reordering, dropping a middle frame, duplication and replay,
but **not** truncation of the tail — every frame that did arrive authenticates perfectly.
`Content-Length` cannot do this job because it is unauthenticated and an on-path attacker
rewrites it along with the body.

`day` and `time` are **dropped** from the hello. They existed to date a version 3 shot from the
filename, the responder passes `null` for that filename today, and shots on this path carry
their own timestamp. Verified rather than assumed: across `apps/worker/src`, `apps/worker/test`
and `packages`, the only field of `Accepted.hello` read anywhere is `total`, in `readBody`.

`NOISE_CONTENT_TYPE` becomes `application/vnd.variegated.uplink`, since the body is no longer
specifically a shot. It is documentation rather than routing — the route dispatches on method
and on the presence of the handshake header, and nothing branches on the content type.

`kind` is **not** added. An earlier draft discriminated shot logs from other messages in the
hello; making the POST payload always `postcard(UplinkMessage)` says the same thing with the
enum discriminant that is already there, and leaves exactly one payload shape across both
transports.

### 1.5 Flag day

There is no compatibility path for the current in-body handshake. `POST /api/noise-upload`
speaks v2 only: no `X-Variegated-Noise` header is a 400, not a fallback.

Every machine must be reflashed with the Plantlet deploy, and one that is not stops uploading
until it is. That is a real cost and it is accepted deliberately — the fleet is small enough to
enumerate, and the alternative is two wire formats, two sets of vectors and two decode paths
kept alive indefinitely for a capability v2 fully subsumes.

### 1.6 Framing on the socket

A WebSocket binary message is one **record**:

```text
[ u64 LE counter ][ sealed frame 0 ][ sealed frame 1 ] ... [ sealed frame n ]
```

Each sealed frame is `UPLINK_CHUNK` plaintext bytes plus a 16-byte tag, except the last, which
is whatever remains. The frame schedule is derived from the record's own length — a WebSocket
message is delivered whole or not at all, so unlike a streamed HTTP body there is no tail to
truncate and no declared total is needed. Counters increment per frame from the record's
declared counter. The concatenated plaintext is `postcard(UplinkMessage)`.

Text frames are refused and close the socket.

**The counter is explicit and in the clear, and that is load-bearing.** A hibernated Durable
Object loses its in-memory nonce counter. Restoring a stale one and encrypting fresh plaintext
under an already-used nonce is a total break of ChaCha20-Poly1305 rather than a weakening, and
it is silent — everything still round-trips. So:

* the sender durably records `n + 1` **before** encrypting under `n`;
* because the counter is on the wire, a crash that skips counters forward is harmless rather
  than desynchronising;
* the receiver requires strictly greater than the last accepted counter and closes otherwise.

Associated data is empty; the counter is already bound by being the nonce.

The POST body keeps **implicit** counters, exactly as today. The asymmetry is deliberate and
worth stating: a POST is a single stream with a declared total, decrypted by a receiver that
does not hibernate, so there is no counter to restore and nothing for eight bytes a frame to
buy. The socket's receiver hibernates; that is where they earn their place.

### 1.7 Keepalive

The machine sends an RFC 6455 ping with an empty payload every 20 s when otherwise idle. It
carries nothing and leaks nothing.

As shipped this is *both* the NAT keepalive and what liveness rests on. Cloudflare's runtime
answers a protocol-level ping without waking the hibernating Durable Object, so one costs the
server nothing however often it arrives — which is what lets an idle machine's status interval
grow to ten minutes without the link noticing, and what lets §4.4 read connectedness off the
socket rather than off the status schedule.

**A protocol-level ping does not wake a hibernated object**, which is what makes this free.
Cloudflare's documentation states it in as many words: incoming ping frames receive automatic
pong responses, ping/pong handling does not interrupt hibernation, and `webSocketMessage` is
not called for control frames. So the keepalive costs no wall-clock time and no request.

`state.setWebSocketAutoResponse()` with a `WebSocketRequestResponsePair` is therefore **not
used**. It exists, it is not deprecated, and it is explicitly not billed — but it answers an
*application-level* text ping, which is a mechanism for clients that cannot send control
frames. `edge-ws` sends control frames, so the simpler thing works.

`web_socket_auto_reply_to_close` (2026-04-07 and later) does **not** affect this. It makes the
runtime send the reciprocal Close frame and transition `readyState` before firing the close
event — it automates the reply, not the notification, and the event still fires. The
documentation is explicit that it has no effect on the hibernatable `webSocketMessage` handler
and says nothing about suppressing `webSocketClose`.

That said, §4.4 is designed not to depend on the answer either way. See there.

---

## 2. The message protocol

A new module `uplink_types` in `variegated-comms-api-types`, beside `ws_types`. Same crate,
because it shares every payload type and the `schema` feature; a separate crate would buy
nothing but workspace churn. A separate *enum* is what matters, and that is the point of the
whole module.

```rust
/// Append only. These discriminants are a contract with flashed firmware.
pub enum UplinkMessage {
    // machine -> Plantlet
    Status(Status),                             // 0
    RoutineList(RoutineSummaryStorage),         // 1
    ShotLog(EncodedPayload),                    // 2
    Reply { id: u32, outcome: QueryOutcome },   // 3
    // Plantlet -> machine
    RequestStatus,                              // 4
    RequestRoutineList,                         // 5
    Query { id: u32, query: UplinkQuery },      // 6
}

/// Reached only through `UplinkMessage::Query`. Append only.
pub enum UplinkQuery {
    RoutineDefinition(RoutineIndex),
    WriteRoutine {
        index: Option<RoutineIndex>,
        routine: EncodedPayload,
    },
}
```

`QueryOutcome`, `QueryOk` and `QueryError` are reused **verbatim** from `ws_types`. They
already say exactly the right things — `RoutineDefinition(EncodedPayload)`,
`RoutineStored(RoutineIndex)`, `NotFound`, `Unavailable`,
`RoutineWrite(RoutineWriteError)` — and the firmware's existing `ClientQuery` handler arms
transfer nearly unchanged. `QueryOk::ShotLogPage` is unreachable here because `UplinkQuery` has
no way to ask for a page; an unreachable *success* variant is harmless, since the boundary that
matters is what Plantlet is permitted to say.

`RequestStatus` and `RequestRoutineList` are **triggers, not queries**: they provoke the
ordinary unsolicited push rather than a correlated reply, so a requested status reaches Plantlet
by exactly the same code path as a scheduled one. This is not a shortcut — it is the pattern
`WsMessage::RequestConfiguration` already uses and documents.

### 2.1 The security boundary is structural

Each side's inbound dispatch is a `match` **with no `_` arm**. A variant appended later is
therefore a compile error on the side that must not accept it, rather than a default-permit. A
runtime allow-list would have been smaller and would have failed open the first time someone
wrote `_ => handle(msg)`.

`MachineCommand` is not reachable from this enum at all. Authority over what the machine
physically does never crosses this link.

### 2.2 Bounds

| bound | value | why |
|---|---|---|
| `UPLINK_CHUNK` | 1024 | matches `SHOT_LOG_CHUNK_LEN`; the frames are the chunks the link already delivers |
| machine inbound record | `MAX_CLIENT_FRAME_LEN` (`ROUTINE_MAX_ENCODED_LEN + 256`) | the largest thing Plantlet can legitimately say is a routine write |
| Plantlet inbound record | 1 MiB | Cloudflare's WebSocket message cap |
| POST body | 4 MiB | unchanged `MAX_UPLOAD_BYTES` |

Each bound is checked before anything is allocated, so an absurd length costs nothing to refuse.
On POST that means `Hello::total`, which is authenticated; on the socket it means the WebSocket
message length the runtime reports, which is not authenticated but is only ever used to refuse
— a record that passes the check still has to decrypt.

### 2.3 Schema generation

`variegated-schema-export` gains a `--uplink <out-dir>` frozen root emitting
`packages/uplink/schemas/v1.ts` into Plantlet, with byte fixtures produced by Rust — so the
TypeScript decoder is tested against bytes Rust actually wrote, not bytes TypeScript believes
Rust would write. Same discipline as the shot-log and routine exports.

---

## 3. Which transport carries what

One rule, applied to every machine-elicited message:

```text
socket is open  &&  encoded length <= 1 MiB   ->  WebSocket
otherwise                                      ->  POST
```

Nothing is hard-coded per message type. Shot logs are 80–98 kB in practice (the three samples
at the umbrella root are 80,698 / 97,785 / 90,141 bytes) so they normally take the socket,
saving a TCP connect, a second Noise handshake, and 5.6 kB of transient socket buffers on a
chip with 40 kB of heap free. A shot over 1 MiB, or any message at all while the socket is
down, takes POST — so the fallback is exercised routinely rather than rotting.

**The firmware never buffers a shot.** `edge-ws` supports fragmentation
(`FrameType::Binary(Fragmented)` then `Continue(Final)`), and Cloudflare reassembles fragments
before delivering to `webSocketMessage`. The machine streams sealed 1 kB frames as continuation
frames while pulling chunks from the application processor; the DO receives one complete record.

Encoding `UplinkMessage::ShotLog` this way cannot use `postcard::to_allocvec`, because the shot
is never held whole. The firmware writes the discriminant byte and the `Vec<u8>` varint length
by hand — both known up front, since the total comes from the shot log listing — and then
streams the bytes. That hand-rolled prefix is small, and it is tested against
`to_allocvec` output for a shot small enough to encode both ways.

That POST is the fallback for *everything*, not just shots, is what makes a machine behind a
proxy that blocks WebSocket upgrades still fully functional: it reports status on schedule,
pushes routine lists, and shows as connected. It simply cannot be asked anything.

This survived the amendment to §4.4. A status renews the connected lease whichever transport
carried it, so a POST-only machine is connected on the same terms as one holding a socket — it
just never gets the prompt disconnect a close provides, because it has no socket to close.

---

## 4. Plantlet

### 4.1 Routing and dispatch

The upgrade handler does the handshake and the machine lookup, then hands off:

```text
worker (GET):  decode X-Variegated-Noise -> acceptIk() -> devicePublic
               findMachineByNoiseKey     -> machine | 403
               writeIkMessage2 + split   -> (rxKey, txKey), msg2
               MACHINE_UPLINK.get(idFromName(machine.id)).fetch(...)
DO:            ctx.acceptWebSocket(server)          [hibernation]
               persist session; close any prior socket
               101 + X-Variegated-Noise: msg2
```

One session per DO. A second handshake closes the first, so there is never more than one set of
keys to reason about.

The POST handler decrypts, decodes, and then splits by variant for one reason:

* **`ShotLog`** is handled by the worker directly, straight into the existing `storeShot`. It
  is the one variant that can be megabytes, and routing four megabytes through a Durable Object
  is what choosing POST for oversized shots exists to avoid.
* **everything else** is forwarded to the DO, so there is exactly one dispatcher for control
  messages regardless of which transport delivered them.

On the socket path the DO receives `ShotLog` and calls the same `storeShot` helper. One storage
path, two callers, and the quota is charged in that helper rather than at either call site.

A `Reply` arriving over POST is unusual but legal — it can only follow a `Query`, which only
arrives over the socket, so it means the socket dropped between question and answer. It is
forwarded like any other control message and matched against `routine_push` there, which is
why the dispatcher is one function rather than two.

### 4.2 Hibernation

Hibernation is required for cost: a machine sends one status a minute while it is on and one
every ten minutes while it is off, and is otherwise silent — a DO billed for wall-clock residency
between those would cost more than the service. Statuses are most of what the service costs at
all: each is one D1 row written, one DO row written and one DO request, which is why the cadence
follows the machine rather than a fixed clock.

What must survive hibernation is *almost* all small — two 32-byte keys and two counters — and
lives in the DO's storage rather than in `serializeAttachment`, so that the counter write in §1.6 is an
ordinary durable write with ordinary ordering guarantees.

The one exception is the session sequence §4.4 relies on, which *is* an attachment. It has to be:
it identifies the socket rather than the object, and the question it answers — "is the socket
handing me this close the one I currently hold?" — cannot be answered by anything stored per
object. It is a single integer, written once when the socket is accepted and never again.

`alarm()` does two jobs: expiring routine pushes past their window, and continuing a deferred
routine-definition walk (§7). It is set only when there is something to do, so an idle machine's
object never wakes on its own.

**The object never calls `ctx.storage.deleteAll()`.** From compatibility date 2026-02-24,
`delete_all_deletes_alarm` makes `deleteAll()` cancel any pending alarm as well — so the obvious
way to reset a session on re-handshake would also silently drop the push-expiry alarm, and the
symptom would be pushes that never expire rather than an error. Session keys are deleted by
name, and the alarm is re-armed from `routine_push` whenever a socket opens.

**Hibernation only applies to a Durable Object acting as a WebSocket *server*** — to
connections accepted through `ctx.acceptWebSocket()`, never to a connection the object opens
itself. This design is on the right side of that line: the machine is the client and the object
is the server. It also forecloses an obvious-looking future change — having Plantlet dial out to
a machine would give up hibernation entirely.

**Self-hosting.** `ARCHITECTURE.md` requires the same bundle to run under `workerd` via
miniflare, and hibernation is supported there: local WebSockets stopped staying pinned in memory
as of `wrangler@3.13.2` and `miniflare@3.20231016.0`. Both the current pins and every upgrade
target in §4.6 are comfortably past that, so the self-hosted container and the test pool
exercise the same eviction behaviour as production rather than a resident approximation of it.

### 4.3 D1, migration 0011

```sql
CREATE TABLE machine_uplink (
  machine_id        TEXT PRIMARY KEY REFERENCES machine(id) ON DELETE CASCADE,
  -- Server clock, ms since epoch, stamped where the status is decoded. What the page shows as
  -- "last reported". Never taken from the machine's own clock.
  last_status_at    INTEGER,
  last_status       BLOB,        -- postcard Status, latest only
  -- Diagnostics only. Not itself the connected rule -- see §4.4.
  session_opened_at INTEGER,
  -- The whole of the connected rule (§4.4). Renewed by any status and by a session opening;
  -- set to now when the current socket closes. Added in migration 0014.
  connected_until   INTEGER
);

CREATE TABLE machine_routine (
  machine_id         TEXT NOT NULL REFERENCES machine(id) ON DELETE CASCADE,
  routine_index      TEXT NOT NULL,      -- 'custom:3', canonical text form
  library_routine_id TEXT REFERENCES routines(id) ON DELETE SET NULL,
  body_crc           INTEGER,            -- CRC-32C trailer of the stored routine
  name               TEXT NOT NULL,
  routine_type       TEXT NOT NULL,
  step_count         INTEGER NOT NULL,
  parameter_count    INTEGER NOT NULL,
  verified_at        INTEGER,
  updated_at         INTEGER NOT NULL,
  PRIMARY KEY (machine_id, routine_index)
);

CREATE TABLE routine_push (
  id                 TEXT PRIMARY KEY,
  machine_id         TEXT NOT NULL REFERENCES machine(id) ON DELETE CASCADE,
  library_routine_id TEXT NOT NULL REFERENCES routines(id) ON DELETE CASCADE,
  target_index       TEXT,               -- null = the machine assigns
  state              TEXT NOT NULL,      -- pending | sent | stored | failed | expired
  error              TEXT,
  created_at         INTEGER NOT NULL,
  expires_at         INTEGER NOT NULL,
  resolved_at        INTEGER
);

CREATE INDEX routine_push_pending ON routine_push (machine_id, state);
```

`last_status` follows the rule the shot index already follows: it is stored as the bytes the
machine sent, and anything displayed is decoded from those bytes, never written from the
request. A projection that can disagree with the thing it projects is a projection that
eventually does.

**The queue lives in D1, not in DO storage.** The UI reads D1, and one source of truth beats
two that must be reconciled. The DO reads pending rows when a socket opens and writes outcomes
back.

### 4.4 Connected

> **Amended.** This section originally specified a rule with no socket flag in it, and gave three
> reasons. The status cadence has since become adaptive — a minute while the machine is on, ten
> while it is off or in standby — and a subtraction sized for that would have had to reach about
> twenty-five minutes, which meant a machine switched off showing as connected for nearly half an
> hour. The rule below replaces it. The original text and its reasoning are kept beneath, because
> one of the three reasons was given up deliberately and should not be re-discovered as a
> surprise.
>
> **Amended again**, by the four-tier cadence in §5.3. Nothing here needed changing: the twenty-five
> minute lease is sized against the *longest* interval, which is still the 600 s idle one, and the
> new tiers are all shorter. `the_idle_interval_leaves_room_inside_the_server_lease` in
> `uplink_types.rs` is the joint that holds the two languages' constants together, and it is
> unchanged. What did change underneath this section is that the keepalive ping it relies on now
> actually fires — see §5.3.

```sql
connected := connected_until IS NOT NULL
             AND server_now < connected_until
```

One column, holding the moment a machine stops counting as connected unless something renews it:

| event | writes |
|---|---|
| a status is decoded, over the socket **or** by POST | `connected_until = now + 25 minutes` |
| the session opens | `connected_until = now + 25 minutes` |
| `webSocketClose`, for the socket the object currently holds | `connected_until = now` |

The clock is still the **server's**, for the reason it always was — see the paragraph below on
`Status::current_local_time`, which is unchanged.

Why a lease rather than the obvious `socket_open OR recent_status`: that disjunction does not
give a prompt disconnect. After a clean close, a status from three minutes ago still satisfies
the second half, so the machine keeps showing as connected for the rest of the window — which is
the thing the change was for.

The session opening renews the lease, and that is load-bearing rather than tidy. A machine that
has just booted may have nothing to report yet: `send_status` finds its cache empty and sends
nothing. Waiting for a status would leave a machine whose socket is plainly open reading as
disconnected for a whole interval.

Of the three properties the original rule bought, two survive and one is spent:

* **Spent.** Close-frame handling now affects what the listing shows. This is guarded rather than
  accepted: each socket is stamped with a session sequence, and `webSocketClose` ignores a close
  belonging to a session that has already been replaced. Without that guard a reconnect's late
  close tore down the session that replaced it — which was a live bug independently of this
  change, since it also deleted the new session's Noise counters.
* **Kept.** Connectedness still means "reporting". A machine behind a proxy that blocks WebSocket
  upgrades, reporting by POST (§3), renews its lease on exactly the same terms and is connected.
* **Kept, and better.** The cost is still bounded and known. A machine switched off cleanly now
  drops off at once rather than after eleven minutes. What is *worse* is the ungraceful case: a
  black-holed link has no server-side probe behind it — Cloudflare answers the machine's keepalive
  pings but never sends its own — so the lease is the only thing that notices, and that takes up
  to twenty-five minutes rather than eleven.

Twenty-five minutes is two and a half of the ten-minute interval an idle machine reports on, so
it cannot fire on a working machine. Note that statuses are not silently lost while a socket is
up: TCP means one either arrives or the socket breaks and we hear about it.

`session_opened_at` is still kept for diagnostics — "is there a socket, and since when" — and is
still not itself consulted by the rule; the lease is what the listing reads.

The machine sends a `Status` immediately after the handshake, so the lease is meaningful from the
first second instead of inheriting a stale one from a previous session.

### 4.4.1 The original rule, superseded

Retained for its reasoning, which still explains why the clock is the server's:

> ```sql
> connected := last_status_at IS NOT NULL
>              AND (server_now - last_status_at) < 11 minutes
> ```
>
> One input, and it is the **server's** clock at the moment the status was received — not
> `Status::current_local_time`, and not anything else the machine says about time. The machine's
> clock is `Option<NaiveDateTime>` and stays `None` until the comms processor has associated,
> taken a lease and completed SNTP, so a freshly booted machine reporting perfectly well has no
> time to offer. A rule keyed on the machine's clock would call that machine disconnected, and
> would call a machine with a badly wrong clock connected forever.
>
> **There is no socket flag in the rule.** An earlier draft had `webSocketClose` clear a
> `connected` column so a clean disconnect showed at once; that is gone.

### 4.5 Routes

| route | does |
|---|---|
| `GET /api/machines` | gains `connected`, `lastStatusAt` |
| `GET /api/machines/:id/routines` | the mapping, joined to library routines |
| `POST /api/machines/:id/routines/:routineId` | enqueue a push, wake the DO |
| `POST /api/machines/:id/refresh` | `RequestStatus` + `RequestRoutineList` |

Waking is `MACHINE_UPLINK.get(id).fetch('https://do/deliver')`; the object reads pending rows
and writes to whatever socket `ctx.getWebSockets()` returns. If there is none, the rows stay
pending and the alarm handles expiry.

### 4.6 The toolchain moves first

The `compatibility_date` is capped by the installed runtime — `wrangler.jsonc` says why in as
many words: a later date is not an error, workerd warns once and silently falls back, so the
field would claim a runtime this deployment is not running on. Raising the date therefore means
raising wrangler, miniflare, workerd and the test pool together.

The comment there defers this on the grounds that wrangler 4 needs a miniflare 5 alpha. That is
true of the *newest* wrangler and not of wrangler 4 as such — `@cloudflare/vitest-pool-workers`
still ships lines pinned to stable miniflare 4:

| tier | pool-workers | wrangler | miniflare | vitest | max date | stable |
|---|---|---|---|---|---|---|
| A | 0.12.0 | 4.57.0 | 4.20260103.0 | `2.0.x - 3.2.x` | 2026-01-03 | yes |
| B | 0.19.1 | 4.115.0 | 4.20260722.1 | `^4.1.0` | 2026-07-22 | yes |
| C | 0.22.0 | 4.125.0 | 5.20260820.0-alpha | `^4.1.0` | 2026-08-20 | **no** |

**Tier B is chosen**, and the target is exact:

```jsonc
"compatibility_date": "2026-07-22"
```

| package | from | to |
|---|---|---|
| `wrangler` | `^3.80.0` | `4.115.0` |
| `miniflare` | `^3.20250718.3` | `4.20260722.1` |
| `@cloudflare/vitest-pool-workers` | `^0.5.0` | `0.19.1` |
| `vitest` (root) | `^2.1.0` | `^4.1.0` |
| `@cloudflare/workers-types` | `^4.20241004.0` | matching `4.2026072x` |

Tier C is the newest date but puts a prerelease miniflare into the self-hosted container, which
*is* miniflare rather than merely tested against it, and nothing here needs a runtime feature
newer than B. Tier A avoids the vitest 2 → 4 jump entirely but lands before every flag this
design discusses, which would leave the spec carrying compatibility notes for behaviour it had
chosen not to adopt.

The vitest 2 → 4 jump is the bulk of the work and it is not optional: every
`vitest-pool-workers` line new enough to carry a 2026 runtime peers on `vitest ^4.1.0`.

**It is an API migration, not a version bump.** Vitest 4 removes `poolOptions`, so
`@cloudflare/vitest-pool-workers` has moved from a pool to a Vite plugin. In 0.19.1 the
`./config` export is gone — there is no `defineWorkersConfig` — and the package exports
`cloudflareTest` instead. The package ships a codemod, `@cloudflare/vitest-pool-workers/codemods/vitest-v3-to-v4`,
which rewrites the import, lifts the whole `test.poolOptions.workers` object into
`plugins: [cloudflareTest({ ... })]` and deletes `test.poolOptions`.

Two options this repository relies on **do not survive**, and neither fails loudly:

* **`singleWorker: true`** is not in 0.19.1's options schema, and that schema is zod `strip` —
  an unrecognised key is silently discarded rather than rejected. It is load-bearing here: the
  comment in `apps/worker/vitest.config.ts` records a measured 12,713 sockets and
  `EADDRNOTAVAIL` on macOS without it, surfacing as `Fallback service failed to fetch module`.
  The vitest 4 equivalent is `test.maxWorkers: 1`, set explicitly — the codemod will not add it.
* **`isolatedStorage`** is gone from the package entirely; the string appears nowhere in it.
  The current comment says the tests depend on it to reset D1 and R2 between files. Whether
  0.19.1 does that unconditionally or not at all is a question the 19 worker test files answer
  directly, so phase 0 finds out by running them rather than by reasoning about it.

Tier A would avoid this migration, since 0.12.0 still has `defineWorkersConfig`. It is still
not chosen — the migration is required by any line new enough to carry a 2026 runtime, so
tier A defers the work rather than removing it.

**The window is wider than the configured date suggests, because tests have never run at it.**
Two workerd versions are installed: `wrangler` brings `1.20250718.0`, but the test pool pins its
own `miniflare@3.20241230.0` → `workerd@1.20241230.0`, which silently falls back to a maximum of
**2024-12-30**. The `miniflare` version declared in `apps/worker/package.json` is not the one the
tests use. So production has been running 2025-07-18 semantics while the suite verified
2024-12-30 semantics, and the audit window for *tests* is 2024-12-30 → 2026-07-22.

Flags becoming default in that window that are worth checking beyond this design:

* **`assets_navigation_prefers_asset_serving` (2025-04-01)** — newly active *in tests* after the
  bump, having been active in production for over a year. This is the `not_found_handling: none`
  machinery whose failure mode "The Worker answers first" documents as a production incident, and
  `spa-fallback.test.ts` is what covers it. An earlier revision of this section claimed the flag
  was already default and therefore irrelevant; that was true of production and wrong of the
  suite.
* `strip_authorization_on_cross_origin_redirect` (2025-09-01) — better-auth's OAuth and the MCP
  token paths.
* `require_returns_default_export` (2026-01-22) — the bundle's CJS interop.
* `urlpattern_standard` (2025-05-01), `enable_weak_ref` (2025-05-05) and
  `nodejs_compat_populate_process_env` (2025-04-01) — all active in production today, none ever
  exercised locally.

That divergence is a pre-existing defect, not one this work introduces — and the bump **narrows it
rather than closing it**. `@cloudflare/vitest-pool-workers@0.19.1` still declares its own
`miniflare 4.20260730.0` (workerd `1.20260730.1`) and `wrangler 4.116.0`, resolved under the pool's
own `node_modules`, while `apps/worker` declares miniflare `4.20260722.1` / wrangler `4.115.0`
(workerd `1.20260722.1`) — which is what `serve.mjs` and `wrangler dev` run.

Two different quantities are easy to conflate here, and both matter. The **binary** gap was workerd
`1.20241230.0` against `1.20250718.0` — about six and a half months — and is now eight days. The
**compatibility-date** jump the tests take in this bump is separate and larger: 2024-12-30 to
2026-07-22, roughly nineteen months. Neither figure substitutes for the other.

**No compatibility flag differs between the two binaries at any date** — their capnp flag tables
are byte-identical — so nothing *semantic* diverges. The hazard is cruder than that, and it is
real: **for dates from 2026-07-30 through 2026-08-06 inclusive, the test suite starts and passes
while the deployment runtime refuses to boot at all.** The pool's workerd accepts up to 2026-08-06;
`apps/worker`'s accepts up to 2026-07-29. Inside that eight-day window the suite is green on a
configuration that `serve.mjs` and `wrangler dev` will not start.

Two things this is *not*. It is not a semantic difference — the failure is a hard startup refusal
naming the ceiling, not a behaviour change. And it is not established for `wrangler deploy`:
wrangler does no client-side ceiling check, and Cloudflare's own fleet is far ahead of both
binaries, so that path very likely succeeds. What verifiably breaks is the **self-hosted container
and local development** — which is precisely the deployment mode `ARCHITECTURE.md` treats as
non-negotiable.

So the date is load-bearing in a second way: it must stay at or below what the *deployment*
runtime accepts, and the suite will not tell you when it stops being. **No document here may claim
the tests run the runtime production runs.** The mechanism that allowed nineteen months of drift is
still present; only its magnitude changed.

One thing the upgrade buys back for free: `run_worker_first` exists in wrangler 4, and
`ARCHITECTURE.md` names it as the fix for the one Worker invocation currently spent on every
page navigation. Out of scope here, but it stops being unavailable.

---

## 5. Firmware

### 5.1 Where the code goes

Pure, testable logic — the IK handshake, record framing, seal and open, the transport-choice
rule — goes into `crates/variegated-shot-upload` as a new `uplink` module. That crate already
owns `noise`, `url`, `crockford` and the X25519 workaround, and it is already the host-testable
one: `variegated-comms-firmware` sets `[lib] harness = false`, under which cargo runs no tests
and reports success.

The crate's name will then undersell what it holds. Renaming it to `variegated-plantlet-link`
is mechanical and belongs in its own commit, not folded into this work.

The chip-facing half is `firmwares/variegated-comms-firmware/src/uplink/`: DNS, the TCP socket,
`edge-ws` client framing with masking (`FrameHeader.mask_key` is `Some(_)` for a client), and a
select loop over the status pubsub, the routine-change signal, the shot-log event stream and
inbound records. The rule that keeps the split honest is the one the upload path already states:
**no URL slicing and no protocol branching in the firmware module.**

### 5.2 Handlers are the ones that exist

`websocket.rs` already answers `ClientQuery::RoutineDefinition` and `ClientQuery::WriteRoutine`
against `ROUTINE_CACHE`, `routine_request` and `routine_write`, with timeouts that bound the
*machine's* turnaround and therefore do not change with who asked. The uplink dispatch calls
the same helpers.

Extracting a shared handler is worth doing — but once both call sites exist and their real
differences are visible, not speculatively before the second one is written.

### 5.3 Scheduling

> **Amended.** The flat "every 10 minutes" below became the two-tier adaptive cadence recorded in
> §4.4, and has since gained two more tiers and a rule about pushed updates. What the firmware
> actually does now is the table immediately below; the original is kept under it.

| trigger | sends |
|---|---|
| handshake complete | `Status`, then `RoutineList`, `MachineDefinition`, `Configuration` |
| machine brewing, or running a routine | `Status` **every second** |
| machine on, not busy | `Status` every 60 s |
| machine off or in standby | `Status` every 600 s |
| busy for over 5 minutes | falls back to the mode's interval — a failsafe against a stuck flag |
| `MachineCommand` received | 5 × `Status`, 1 s apart, starting immediately |
| machine mode changed | `Status`, floored at one per 5 s |
| `RequestStatus` | `Status`, and resets the interval |
| routine set changed | `RoutineList` — **held for the next `Status` while the machine is asleep** |
| a setting changed | `Configuration`, if the bytes moved — held the same way |
| `RequestRoutineList` / `RequestConfiguration` | that message, always, whatever the mode |
| `ShotLogEvent::Stored` | `ShotLog`, by whichever transport §3 selects — never held |
| every 20 s, unconditionally | a WebSocket ping, which wakes nothing on the server |

The decision itself is `status_interval_secs` and `defer_updates` in
`variegated-comms-api-types`, not in the firmware's loop: that crate sets `harness = false`, so a
rule kept beside the loop is one nothing can test.

**A table of intervals is not enough to make the brewing tier happen, and the first attempt at
this shipped without the part that does.** The uplink waits on an absolute deadline and only
recomputes it when it sends a status. So a shot starting just after a status sits behind a
deadline set a minute earlier, while the machine was idle — and is over before the first
one-second status goes out. The one-second tier was unreachable in practice for every shot
shorter than the interval it was replacing.

What makes it work is an edge: `MACHINE_ACTIVITY_CHANGED`, raised by `cache_update_task` when
either the mode or `Status::is_busy` changes, on which the uplink brings its next status forward
to whatever the new interval asks for (`min`, so it can only ever move it earlier). Two
properties worth keeping:

- **The clamp is what bounds it, not a rate limiter.** It cannot pull the deadline below the
  interval the machine's own state asks for, so a flapping `is_brewing` can produce nothing
  faster than the busy cadence it is already entitled to.
- **`STATUS_CACHE` is written before the signal is raised.** The uplink answers the signal by
  reading that cache to decide which interval applies; signalling first leaves a window in which
  it reads the previous status, concludes the machine is not busy, and keeps the deadline it had
  — a shot reported once a minute instead of once a second, intermittently.

**The keepalive in that last row had never fired.** It was
`with_timeout(KEEPALIVE_INTERVAL, read_record(..))`, rebuilt every turn of a loop that the
configuration republish woke every ten seconds — shorter than the twenty-second timeout, so it
could never expire. Consequences, both fixed together with the cadence: a machine in `Off` or
`PowerSaveStandby` sent nothing for ten minutes and was sent no pings, against a 120 s
`SOCKET_TIMEOUT`, so its session was reset and reconnected every two minutes on a healthy
network; and the same ten-second wake had been cancelling the in-flight `read_record` six times a
minute, which is not cancel-safe. Anyone reading §4.4 would reasonably have assumed pings were
going out all along. They were not.

The original, superseded:

| trigger | sends |
|---|---|
| handshake complete | `Status`, then `RoutineList` |
| every 10 minutes | `Status` |
| `RequestStatus` | `Status`, and resets the 10-minute timer |
| routine set changed | `RoutineList`, debounced 5 s so a burst of edits sends one list |
| `RequestRoutineList` | `RoutineList` |
| `ShotLogEvent::Stored` | `ShotLog`, by whichever transport §3 selects |

### 5.4 The memory gate

This is the largest risk in the firmware half and it gets an explicit gate rather than a
paragraph of reassurance.

A permanently open socket costs about 5.6 kB of heap (1536 rx + 4096 tx), which the ~40 kB of
free heap absorbs. The tight number is **2,452 bytes of stack headroom**, and a new embassy
task's arena lives in `.bss`, which is the same pool.

The plan therefore measures, before and after, using the instrumentation that already exists:
`scripts/memory-report.sh` for RAM-resident sections, and the 1 Hz heap and stack high-water
lines in `debug/snapshot.rs`. `heap_free()` brackets the uplink task specifically, because a
high-water figure only moves upward and is useless for attribution.

If it does not fit, the fallback is folding the uplink into the existing upload task — which is
idle except during an upload — rather than trading bytes between `.bss` and `.stack` on a
plausible mechanism. That trade has been made twice here without measuring and was wrong both
times.

---

## 6. Shot log format version 9

`SHOT_LOG_FORMAT_VERSION` goes 8 → 9 to add one field to `RoutineExecutionMetadata`:

```rust
pub struct RoutineExecutionMetadata {
    pub routine_index: RoutineIndex,
    pub routine_name: String,
    pub routine_type: RoutineType,
    pub resolved_parameters: FnvIndexMap<u8, f32, 8>,
    /// CRC-32C trailer of the routine as stored, so a log can be matched to the exact
    /// revision that produced it.
    ///
    /// A matching hint, and nothing more. CRC-32 is linear: four chosen bytes give a
    /// routine any CRC you like. Never gate on this, never dedup on it.
    pub routine_crc: u32,
}
```

It goes here rather than in `ShotAnnotations` because it is a machine-derived fact, and a
`SetShotAnnotations` can clear that block.

**Why CRC-32C and not a hash.** It is already computed and already stored: a routine is
`postcard::to_slice_crc32(&Routine)` on the machine, and `packages/routine/src/encode.ts`
writes and reads the same trailer over the same bytes. Reading four bytes costs nothing on
either side, and no driver has to be written.

Thirty-two bits is ample for what this does. The shot log already carries `routine_index`,
`routine_name` and `routine_type`, so the CRC only has to distinguish *revisions of one named
routine at one index* — an n in the low tens. Probability that a given lookup is ambiguous is
about n / 2³²: roughly 1 in 2·10⁸ at n = 20, and 1 in 4·10⁶ even if it had to disambiguate a
thousand routine revisions with no other discriminator. For `machine_routine` change detection
it is a one-against-one comparison, so a real change going unnoticed is 2⁻³².

The RP2350 does have a hardware SHA-256 block — `rp-pac` 7.0.0 exposes `sha256` for rp235x,
though `embassy-rp` 0.10 wraps no driver for it — so "too expensive to hash" was never the
reason. The reason is that a hash here would add a driver, a host fallback and sixteen bytes
per log to buy a property nothing needs.

Nothing else rides along in this bump. Stamping the control parameters in force was considered
and dropped.

The bump costs a frozen `packages/shot-log/schemas/v9.ts`, Rust-generated byte fixtures, a
decoder dispatch arm, and the standing round-trip test.

---

## 7. Routines: reconciliation, and the library

### 7.1 Reconciling a list

A `RoutineList` arrives on connect, debounced after any change, or on request. Per index:

* index known, and `name` / `routine_type` / `step_count` / `parameter_count` match the
  recorded row → trust it, bump `verified_at`
* index unknown, or that cheap signal moved → fetch the definition, read its CRC trailer,
  re-link against the library

Fetches are **serial and in the background**, at most four per reconciliation with the rest
deferred to the alarm. The comms processor pulls a definition over a link that reassembles
through a 4 kB accumulator and holds a ~650-byte routine on a 68 kB stack; the constrained side
sets the pace, and prefetching serially in the background is the pattern that already works
here.

An index matching no library routine renders as **not in library**, with an import action.
That is the honest rendering: someone edited a routine at the machine, and claiming the library
version is still there would be silently wrong in exactly the case a person would notice.

### 7.2 Send to machine

If `machine_routine` already links this library routine to an index on this machine, write in
place. Otherwise send `index: None`, let the machine assign a Custom slot, and record the link
from the `QueryOk::RoutineStored(index)` that comes back — which is also the only way a create
can learn where it landed.

The user never picks a slot number, and pushing an edit twice does not consume two slots.

### 7.3 When the machine is off

The push sits in `routine_push` as `pending`, shown in both the library and the machine page,
and is delivered when the DO next sees a socket. It expires after **24 hours** with the reason
surfaced.

An espresso machine is off most of the day, so a button that only worked while one happened to
be awake would be a button that mostly did not work. Indefinite queueing was rejected for the
opposite reason: a routine written in March arriving in June is a genuine surprise.

`QueryError::RoutineWrite` is surfaced whole. The HTTP route this replaces projected five
failure modes onto three status codes, and the frontend read only the number — so "internal
routines are read-only" and "this routine is too large to persist" both surfaced as
`HTTP error! status: 400`. They stay distinguishable here.

---

## 8. Failure

Any decrypt failure, counter regression, unrecognised variant, over-length declared record, or
inbound variant arriving in the wrong direction closes the socket. Reconnection is a fresh
handshake and therefore fresh keys, which is what makes closing a cheap answer rather than a
lazy one. The machine reconnects on the existing backoff schedule.

Refusals to a caller that has not completed a handshake stay deliberately coarse, for the reason
the upload path already gives: the difference between "unknown key" and "corrupt ciphertext" is
useful only to someone enumerating device keys.

Close reasons come from a fixed list of short strings, each **under 123 UTF-8 bytes**. From
compatibility date 2026-03-03, `websocket_close_reason_byte_limit` makes `WebSocket.close()`
*throw* on a longer one — so a reason built by interpolating a decoder error would turn a clean
refusal into an exception inside the error path, which is the worst place to find one. The
coarseness §8 already wanted for security reasons is what keeps them short, but the bound is
asserted in a test rather than left to good intentions.

---

## 9. Testing

* **`fixtures/noise-ik-vectors.json`** — generated by the Rust initiator, committed to both
  repositories, asserted byte-for-byte on both sides. Each side's own round trip would pass
  against its own misunderstanding; the vectors are what stop the two drifting apart. The
  existing `noise-vectors.json` is regenerated for the v2 POST hello.
* **Host tests** for the `uplink` module — `test-aarch64` plus `--no-default-features`, because
  defmt breaks the macOS linker.
* **A discriminant-pinning test** for `UplinkMessage` and `UplinkQuery`, mirroring the existing
  one in `ws_types`, with the same "append only" note. This transport carries no version byte,
  so reordering is undetectable at run time.
* **A direction test** in both directions: encode each machine→Plantlet variant, feed it to the
  firmware's inbound dispatch, assert refusal; and the reverse against the DO's.
* **A streaming-encode test**: the hand-written `ShotLog` discriminant-and-varint prefix must
  match `to_allocvec` for a shot small enough to encode both ways.
* **Worker tests** under `@cloudflare/vitest-pool-workers` against real D1, R2 and DO bindings.
  The pinned miniflare does evict hibernatable sockets (§4.2), so a test that drives a session
  across an eviction is exercising the real path — which matters more here than anywhere else,
  because the counter-restore in §1.6 is only wrong *after* an eviction. A test that never
  evicts would pass against a design that reuses nonces.

---

## 10. Phasing

This is one design but not one implementation plan — it spans two repositories, a stored
format version, a new Durable Object and a firmware task with a memory gate. Four plans, each
independently testable and each landing something that works on its own:

| phase | lands | verified by |
|---|---|---|
| **0. Toolchain** | the tier B bump in §4.6, `compatibility_date` to `2026-07-22`, vitest 2 → 4 across the workspace, and the flag audit | the existing suite green *before* any uplink code exists, so a failure is attributable to the bump and nothing else |
| **1. Protocol and format** | `uplink_types`, shot log v9 with `routine_crc`, both vector files, the `--uplink` schema root, the frozen `v9.ts` | host tests; discriminant and direction tests; round-trip against Rust-generated fixtures. No networking, no sockets |
| **2. Plantlet ingest** | POST v2, migration 0011, `storeShot` refactored to one helper, connected state | `vitest-pool-workers` against real D1/R2; a v1 POST is refused |
| **3. Socket** | `MachineUplink`, hibernation, the record framing, the firmware uplink task | the memory gate in §5.4 first, then a real machine against a real deployment |
| **4. Library and UI** | reconciliation, `machine_routine`, send-to-machine, pending pushes, the connected indicator | worker tests plus the SPA |

Phase 0 lands alone and first, on its own branch, with no uplink code in the diff — a toolchain
bump entangled with a feature is a bisect nobody wants. Phase 1 is a prerequisite for everything
after it. Phases 2 and 4 touch only Plantlet and can proceed while phase 3's memory question is
being answered. Phase 3 is where this design can still fail
on facts rather than on judgement — the memory gate and the hibernation-ping question both live
there, and both are measurements, not decisions.

**Branch hygiene comes first.** `variegated-rs` is currently on `frontend-brew-limits` with a
long list of unmerged feature branches; phase 1 starts by merging what is ready to `main` and
branching fresh from it, not by adding a fifth repository state.

---

## 11. Not in this version

* `MachineCommand` or `Configuration` over the uplink — the authority to change what the
  machine does stays off this link
* Routine deletion from Plantlet
* Pushing one routine to several machines in one action
* Shot logs over 1 MiB on the socket
* Any use of `routine_crc` as an identity: no dedup, no provenance claim
* TLS
