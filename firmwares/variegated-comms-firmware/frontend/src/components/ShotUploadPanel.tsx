import { useEffect, useState } from 'preact/hooks';
import { memo } from 'preact/compat';
import { Alert, Button, Field, TextInput, tokens } from '@variegated-coffee/ui';
import { ShotUploadView } from '../schemas/schemas';
import { saveShotUploadSettings } from '../api/shotUpload';
import { ConfigurationSection } from './ConfigurationSection';

interface ShotUploadPanelProps {
  shotUpload: ShotUploadView | undefined;
}

/** Longest endpoint the firmware will store — `SHOT_UPLOAD_ENDPOINT_LEN`. */
const ENDPOINT_MAX = 255;
/** Longest token the firmware will store — `SHOT_UPLOAD_TOKEN_LEN`. */
const TOKEN_MAX = 64;
/** Length of a Noise key as provisioned — `SHOT_UPLOAD_KEY_LEN`. */
const KEY_MAX = 53;

/**
 * Whether to offer the token, and the `https://` transport it belongs to.
 *
 * Both still work: the firmware stores a token, `saveShotUploadSettings` still carries one,
 * and `Keep`/`Set`/`Clear` are untouched. Machines are provisioned over `http+noise://` with
 * the two keys below, and a form offering two transports asks an operator to decide something
 * they have no basis to decide.
 *
 * Gated rather than deleted for two reasons. It restores in one boolean; and `noUnusedLocals`
 * is on for this build, with `build.rs` failing the firmware build if `tsc` does — so the
 * state behind these controls has to stay referenced, which gating does and deleting does not.
 *
 * Hiding the field cannot clear a stored token: an untouched input leaves `token` empty and
 * `clearToken` false, which `handleSave` sends as `Keep`.
 */
const SHOW_UPLOAD_TOKEN = false;

/** A checkbox with its label, which this form needs four of. */
function CheckboxRow({
  checked,
  onChange,
  children,
}: {
  checked: boolean;
  onChange: (checked: boolean) => void;
  children: string;
}) {
  return (
    <label style={{ display: 'flex', alignItems: 'center', gap: tokens.space.xs, cursor: 'pointer' }}>
      <input
        type="checkbox"
        checked={checked}
        onChange={(e) => onChange((e.target as HTMLInputElement).checked)}
      />
      <span style={{ fontSize: '0.85rem' }}>{children}</span>
    </label>
  );
}

/**
 * Where finished shots are uploaded, and whether to upload them.
 *
 * # The token is write-only, by design rather than by omission
 *
 * The firmware never sends the token to the browser — `Configuration` carries only
 * `token_set` — because this HTTP server has no authentication of any kind and the token
 * grants write access to an account on a public service. So the field is never prefilled,
 * and a blank field means **keep the stored token**, not clear it. Clearing is a separate,
 * explicit action, because "I edited the endpoint" and "I want to de-provision this machine"
 * must not be the same gesture.
 *
 * The **device key** is the same kind of secret and gets exactly the same treatment. The
 * **server key** is not — it is a public key, so it is sent to the browser and prefilled like
 * the endpoint. That asymmetry is deliberate: "which server does this machine trust" is the
 * question you most want to see the answer to when a handshake is being refused.
 *
 * # Two transports, one form
 *
 * An `https://` endpoint uses the token; an `http+noise://` endpoint uses the two keys. The
 * form shows both sets rather than switching on the scheme, because a half-typed URL would
 * make the fields flicker, and because moving a machine between transports means editing
 * both at once.
 *
 * That is the shape of the form, and it is what returns when `SHOW_UPLOAD_TOKEN` goes back
 * to true. As shipped only the Noise half is offered — see the constant.
 */
export const ShotUploadPanel = memo(({ shotUpload }: ShotUploadPanelProps) => {
  const [endpoint, setEndpoint] = useState('');
  const [token, setToken] = useState('');
  const [enabled, setEnabled] = useState(false);
  const [clearToken, setClearToken] = useState(false);
  const [serverKey, setServerKey] = useState('');
  const [deviceKey, setDeviceKey] = useState('');
  const [clearDeviceKey, setClearDeviceKey] = useState(false);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [saved, setSaved] = useState(false);

  // Drafts, seeded from the device rather than derived from it. `ShotLogPanel` documents
  // why: configuration is pushed on change and would otherwise overwrite the field
  // mid-keystroke. Re-seeding is keyed on the *values*, so a push that changes nothing
  // leaves an in-progress edit alone.
  const deviceEndpoint = shotUpload?.endpoint ?? '';
  const deviceEnabled = shotUpload?.enabled ?? false;
  const deviceServerKey = shotUpload?.server_key ?? '';
  useEffect(() => {
    setEndpoint(deviceEndpoint);
    setEnabled(deviceEnabled);
    setServerKey(deviceServerKey);
    setToken('');
    setClearToken(false);
    setDeviceKey('');
    setClearDeviceKey(false);
  }, [deviceEndpoint, deviceEnabled, deviceServerKey]);

  if (!shotUpload) {
    return (
      <ConfigurationSection title="Shot upload">
        <div style={{ color: tokens.color.inkMuted, fontSize: '0.9rem' }}>
          Waiting for configuration…
        </div>
      </ConfigurationSection>
    );
  }

  const tokenStored = shotUpload.token_set;
  const deviceKeyStored = shotUpload.device_key_set;

  const handleSave = async () => {
    setError(null);
    setSaved(false);
    setSaving(true);
    try {
      await saveShotUploadSettings({
        endpoint: endpoint.trim() === '' ? null : endpoint.trim(),
        enabled,
        // Blank means Keep, never Clear — see the component doc. Clearing is only ever the
        // explicit checkbox.
        token: clearToken
          ? { type: 'Clear' }
          : token === ''
            ? { type: 'Keep' }
            : { type: 'Set', value: token },
        // Public, so it round-trips like the endpoint: what is in the box is what is stored.
        server_key: serverKey.trim() === '' ? null : serverKey.trim(),
        device_key: clearDeviceKey
          ? { type: 'Clear' }
          : deviceKey === ''
            ? { type: 'Keep' }
            : { type: 'Set', value: deviceKey.trim() },
      });
      setSaved(true);
      setToken('');
      setClearToken(false);
      setDeviceKey('');
      setClearDeviceKey(false);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Save failed');
    } finally {
      setSaving(false);
    }
  };

  return (
    <ConfigurationSection title="Shot upload">
      <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.md }}>
        <Field
          label="Endpoint"
          help={
            SHOW_UPLOAD_TOKEN
              ? 'https:// with a token, or http+noise:// with the two keys below. Leave blank to stop uploading.'
              : 'http+noise:// with the two keys below. Leave blank to stop uploading.'
          }
        >
          {(control) => (
            <TextInput
              {...control}
              kind="url"
              value={endpoint}
              maxLength={ENDPOINT_MAX}
              placeholder={
                SHOW_UPLOAD_TOKEN
                  ? 'https://plantlet.example/api/shots'
                  : 'http+noise://plantlet.example/api/shots'
              }
              onInput={setEndpoint}
            />
          )}
        </Field>

        {SHOW_UPLOAD_TOKEN && (
          <Field
            label="Token"
            help="The machine never sends its token back, so this cannot show the current one."
          >
            {(control) => (
              <TextInput
                {...control}
                kind="password"
                value={token}
                maxLength={TOKEN_MAX}
                disabled={clearToken}
                autoComplete="off"
                placeholder={tokenStored ? 'Stored — leave blank to keep' : 'Not set'}
                onInput={setToken}
              />
            )}
          </Field>
        )}

        {SHOW_UPLOAD_TOKEN && tokenStored && (
          <CheckboxRow checked={clearToken} onChange={setClearToken}>
            Clear the stored token
          </CheckboxRow>
        )}

        <Field
          label="Server key"
          help="The upload server's public key. Not a secret — it is shown here so you can check which server this machine trusts."
        >
          {(control) => (
            <TextInput
              {...control}
              // Monospace, not `numeric`: a base64 key is read character by character when
              // it is being compared against another one, but a decimal keypad cannot type
              // one.
              mono
              value={serverKey}
              maxLength={KEY_MAX}
              autoComplete="off"
              spellcheck={false}
              placeholder="Only for http+noise:// endpoints"
              onInput={setServerKey}
            />
          )}
        </Field>

        <Field
          label="Device key"
          help={`This machine's secret key, shown once when you generated it. ${
            SHOW_UPLOAD_TOKEN ? 'Like the token, the machine' : 'The machine'
          } never sends it back.`}
        >
          {(control) => (
            <TextInput
              {...control}
              kind="password"
              mono
              value={deviceKey}
              maxLength={KEY_MAX}
              disabled={clearDeviceKey}
              autoComplete="off"
              spellcheck={false}
              placeholder={deviceKeyStored ? 'Stored — leave blank to keep' : 'Not set'}
              onInput={setDeviceKey}
            />
          )}
        </Field>

        {deviceKeyStored && (
          <CheckboxRow checked={clearDeviceKey} onChange={setClearDeviceKey}>
            Clear the stored device key
          </CheckboxRow>
        )}

        <CheckboxRow checked={enabled} onChange={setEnabled}>
          Upload shots automatically
        </CheckboxRow>

        {error && <Alert role="danger">{error}</Alert>}
        {saved && !error && <Alert role="ok">Saved.</Alert>}

        <div>
          <Button
            variant="primary"
            size="sm"
            // `void` rather than passing the async function directly: an unhandled rejection
            // from an event handler is invisible, and every failure path here already ends in
            // `setError`.
            onClick={() => void handleSave()}
            disabled={saving}
          >
            {saving ? 'Saving…' : 'Save'}
          </Button>
        </div>
      </div>
    </ConfigurationSection>
  );
});
