import { ShotUploadSettings } from '../schemas/schemas';
import { getWebSocketService } from '../services/websocket';

/**
 * Save the shot-log upload settings.
 *
 * # The WebSocket, like every other setting
 *
 * This used to be `POST /command/set-shot-upload-settings`, and the reason was purely
 * mechanical: a maximal payload is ~326 bytes and the firmware read inbound frames into a
 * fixed 256-byte buffer, **closing the connection** rather than erroring on anything longer.
 * An endpoint past roughly 186 characters silently dropped the socket. Inbound frames are now
 * heap-backed and bounded by `MAX_WS_MESSAGE_LEN` (8 KiB), so the reason is gone and this
 * travels the same way as everything else the settings UI writes.
 *
 * # Resolving means queued, not applied — and that is not a regression
 *
 * `sendCommandAwaitingAck` resolves when the machine's comms processor has taken the command,
 * which is what distinguishes "accepted" from "dropped, channel full". It is *not* a promise
 * that the setting is stored: the inter-processor link carries no correlation ids, so nothing
 * on this transport can promise that.
 *
 * **The HTTP route this replaced promised exactly as little.** It returned `200 OK` as soon
 * as `command_sender.try_send(cmd)` succeeded — the same queue, the same guarantee. So the
 * move loses no information, which is what separates this from routine CRUD: `PUT /routines/…`
 * genuinely waits on a write outcome, and that is why routines stayed on HTTP.
 *
 * The real confirmation is the `ConfigurationUpdate` that follows: the machine republishes
 * its configuration after a settings change, and `ShotUploadPanel` re-renders from it.
 * Resolving clears the button; the update is what proves the value landed.
 *
 * # The token is never round-tripped
 *
 * The firmware never sends the current token to the browser — `Configuration` carries only
 * `shot_upload.token_set` — so `settings.token` is a three-way instruction rather than a
 * value: `Keep` leaves the stored one alone, `Clear` removes it, `Set` replaces it. A blank
 * password field must map to `Keep`, never to `Clear`, or editing the endpoint would
 * de-provision the machine.
 */
export async function saveShotUploadSettings(settings: ShotUploadSettings): Promise<void> {
  const ws = getWebSocketService();
  if (!ws) {
    throw new Error('Not connected to the machine');
  }

  await ws.sendCommandAwaitingAck({
    type: 'SetShotUploadSettings',
    value: settings,
  });
}
