import { useEffect, useState } from 'preact/hooks';
import { memo } from 'preact/compat';
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

const inputStyle = {
  width: '100%',
  padding: '0.4rem',
  border: '1px solid #ccc',
  borderRadius: '4px',
  fontSize: '0.9rem',
  boxSizing: 'border-box' as const,
};

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
 */
export const ShotUploadPanel = memo(({ shotUpload }: ShotUploadPanelProps) => {
  const [endpoint, setEndpoint] = useState('');
  const [token, setToken] = useState('');
  const [enabled, setEnabled] = useState(false);
  const [clearToken, setClearToken] = useState(false);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [saved, setSaved] = useState(false);

  // Drafts, seeded from the device rather than derived from it. `ShotLogPanel` documents
  // why: configuration is pushed on change and would otherwise overwrite the field
  // mid-keystroke. Re-seeding is keyed on the *values*, so a push that changes nothing
  // leaves an in-progress edit alone.
  const deviceEndpoint = shotUpload?.endpoint ?? '';
  const deviceEnabled = shotUpload?.enabled ?? false;
  useEffect(() => {
    setEndpoint(deviceEndpoint);
    setEnabled(deviceEnabled);
    setToken('');
    setClearToken(false);
  }, [deviceEndpoint, deviceEnabled]);

  if (!shotUpload) {
    return (
      <ConfigurationSection title="Shot upload">
        <div style={{ color: '#666', fontSize: '0.9rem' }}>Waiting for configuration…</div>
      </ConfigurationSection>
    );
  }

  const tokenStored = shotUpload.token_set;

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
      });
      setSaved(true);
      setToken('');
      setClearToken(false);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Save failed');
    } finally {
      setSaving(false);
    }
  };

  return (
    <ConfigurationSection title="Shot upload">
      <label style={{ display: 'block', marginBottom: '0.75rem' }}>
        <div style={{ fontSize: '0.85rem', marginBottom: '0.25rem' }}>Endpoint</div>
        <input
          type="url"
          value={endpoint}
          maxLength={ENDPOINT_MAX}
          placeholder="https://plantlet.example/api/shots"
          onInput={(e) => setEndpoint((e.target as HTMLInputElement).value)}
          style={inputStyle}
        />
        <div style={{ fontSize: '0.75rem', color: '#666', marginTop: '0.2rem' }}>
          Must be HTTPS. Leave blank to stop uploading.
        </div>
      </label>

      <label style={{ display: 'block', marginBottom: '0.75rem' }}>
        <div style={{ fontSize: '0.85rem', marginBottom: '0.25rem' }}>Token</div>
        <input
          type="password"
          value={token}
          maxLength={TOKEN_MAX}
          disabled={clearToken}
          autoComplete="off"
          placeholder={tokenStored ? 'Stored — leave blank to keep' : 'Not set'}
          onInput={(e) => setToken((e.target as HTMLInputElement).value)}
          style={{ ...inputStyle, backgroundColor: clearToken ? '#f0f0f0' : 'white' }}
        />
        <div style={{ fontSize: '0.75rem', color: '#666', marginTop: '0.2rem' }}>
          The machine never sends its token back, so this cannot show the current one.
        </div>
      </label>

      {tokenStored && (
        <label
          style={{ display: 'flex', alignItems: 'center', gap: '0.4rem', marginBottom: '0.75rem' }}
        >
          <input
            type="checkbox"
            checked={clearToken}
            onChange={(e) => setClearToken((e.target as HTMLInputElement).checked)}
          />
          <span style={{ fontSize: '0.85rem' }}>Clear the stored token</span>
        </label>
      )}

      <label
        style={{ display: 'flex', alignItems: 'center', gap: '0.4rem', marginBottom: '0.75rem' }}
      >
        <input
          type="checkbox"
          checked={enabled}
          onChange={(e) => setEnabled((e.target as HTMLInputElement).checked)}
        />
        <span style={{ fontSize: '0.85rem' }}>Upload shots automatically</span>
      </label>

      {error && (
        <div style={{ color: '#c00', fontSize: '0.85rem', marginBottom: '0.5rem' }}>{error}</div>
      )}
      {saved && !error && (
        <div style={{ color: '#080', fontSize: '0.85rem', marginBottom: '0.5rem' }}>Saved.</div>
      )}

      <button
        // `void` rather than passing the async function directly: an unhandled rejection
        // from an event handler is invisible, and every failure path here already ends in
        // `setError`.
        onClick={() => void handleSave()}
        disabled={saving}
        style={{
          padding: '0.4rem 0.9rem',
          borderRadius: '4px',
          border: '1px solid #ccc',
          backgroundColor: saving ? '#eee' : '#fff',
          cursor: saving ? 'default' : 'pointer',
          fontSize: '0.9rem',
        }}
      >
        {saving ? 'Saving…' : 'Save'}
      </button>
    </ConfigurationSection>
  );
});
