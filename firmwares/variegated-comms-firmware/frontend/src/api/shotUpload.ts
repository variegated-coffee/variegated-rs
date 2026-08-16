import {
  SetShotUploadSettingsRequestSchema,
  ShotUploadSettings,
} from '../schemas/schemas';
import { postPostcard } from '../utils/postcard';

/**
 * Save the shot-log upload settings.
 *
 * # HTTP, not the WebSocket, and that is not a style choice
 *
 * Every other setting in this app goes over the WebSocket as a machine command. This one
 * cannot: the firmware reads inbound frames into a fixed 256-byte buffer and **closes the
 * connection** on anything longer, and a maximal payload here is ~326 bytes — the endpoint
 * alone may be 255. An endpoint past roughly 186 characters would silently drop the socket
 * rather than returning an error. Routine saves moved to HTTP for the same reason.
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
  await postPostcard(
    '/command/set-shot-upload-settings',
    { settings },
    SetShotUploadSettingsRequestSchema,
  );
}
