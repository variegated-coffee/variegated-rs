import { useEffect, useState } from 'preact/hooks';
import { memo } from 'preact/compat';
import { TimezoneSetting } from '../schemas/schemas';
import { setTimezone } from '../api/timezone';
import { ConfigurationSection } from './ConfigurationSection';

interface TimezonePanelProps {
  timezone: TimezoneSetting | undefined;
}

/** Longest name the firmware will store — `TIMEZONE_NAME_LEN`. */
const NAME_MAX = 64;

const inputStyle = {
  width: '100%',
  padding: '0.4rem',
  border: '1px solid #ccc',
  borderRadius: '4px',
  fontSize: '0.9rem',
  boxSizing: 'border-box' as const,
};

/**
 * The zones to offer as suggestions.
 *
 * From the browser rather than from the machine, because there is no message that asks the
 * machine which zones its build carries and adding one would be a wire change for a datalist.
 * The firmware's database is trimmed at build time and is normally much smaller than this, so
 * the list is a convenience and **not** a claim about what will be accepted — the machine
 * refuses what it does not have, and the panel says so below.
 *
 * `Intl.supportedValuesOf` is not in older browsers; the fallback is the shipped default plus
 * UTC, which is the pair that always resolves.
 */
function suggestedZones(): string[] {
  const fallback = ['UTC', 'Europe/Stockholm'];
  try {
    const supported = (Intl as unknown as {
      supportedValuesOf?: (key: string) => string[];
    }).supportedValuesOf?.('timeZone');
    return supported ? ['UTC', ...supported] : fallback;
  } catch {
    return fallback;
  }
}

/**
 * The machine's timezone.
 *
 * # Scheduling only — logs stay UTC
 *
 * This governs when schedules fire and what the machine's own clock reads. Every log timestamp
 * is UTC and stays UTC: shot logs record an epoch-millisecond `recorded_at_unix_millis` and
 * the SD card files them by UTC day, so changing this moves nothing already written and
 * reinterprets nothing already recorded.
 *
 * # The machine can refuse a zone, and this is how you find out
 *
 * The firmware carries a tz database trimmed at build time, so it knows only a few zones. A
 * name it does not have is refused and logged, and it keeps the zone it had — storing an
 * unresolvable name would leave this panel displaying a zone the scheduler is not using.
 *
 * Nothing on this transport reports that back: the inter-processor link has no correlation
 * ids, so the send resolving means *queued*, not *applied*. The confirmation is the
 * configuration update that follows, which is why `saved` is derived from the incoming
 * `timezone` prop rather than latched when the button clears — a refused zone therefore shows
 * as the field snapping back to what the machine still has.
 */
export const TimezonePanel = memo(({ timezone }: TimezonePanelProps) => {
  const stored = timezone?.name ?? '';
  const [name, setName] = useState(stored);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Follow the machine, but not while a save is in flight -- otherwise the ten-second
  // configuration republish would overwrite what is being typed.
  useEffect(() => {
    if (!saving) {
      setName(stored);
    }
  }, [stored, saving]);

  const dirty = name !== stored;

  const handleSave = async () => {
    setSaving(true);
    setError(null);
    try {
      await setTimezone(name.trim());
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setSaving(false);
    }
  };

  return (
    <ConfigurationSection title="Timezone">
      <div style={{ padding: '0.75rem' }}>
        <label style={{ display: 'block', marginBottom: '0.35rem', fontSize: '0.85rem', fontWeight: 500 }}>
          IANA zone name
        </label>
        <input
          type="text"
          list="timezone-suggestions"
          value={name}
          maxLength={NAME_MAX}
          placeholder="UTC"
          onInput={(e) => setName((e.target as HTMLInputElement).value)}
          style={inputStyle}
        />
        <datalist id="timezone-suggestions">
          {suggestedZones().map((zone) => (
            <option key={zone} value={zone} />
          ))}
        </datalist>

        <p style={{ margin: '0.5rem 0 0', fontSize: '0.75rem', color: '#666' }}>
          Schedules fire on this zone, and it is what the machine's own clock shows.
          Shot logs are always recorded in UTC and are unaffected.
          Blank means UTC. This firmware carries only a few zones — one it does not have is
          refused, and the field below will snap back to what it kept.
        </p>

        <p style={{ margin: '0.35rem 0 0', fontSize: '0.75rem', color: '#666' }}>
          Currently: <strong>{stored === '' ? 'UTC' : stored}</strong>
        </p>

        {error && (
          <p style={{ margin: '0.5rem 0 0', fontSize: '0.8rem', color: '#c00' }}>{error}</p>
        )}

        <button
          onClick={handleSave}
          disabled={saving || !dirty}
          style={{
            marginTop: '0.75rem',
            padding: '0.4rem 0.9rem',
            border: 'none',
            borderRadius: '4px',
            backgroundColor: saving || !dirty ? '#ccc' : '#0066cc',
            color: 'white',
            cursor: saving || !dirty ? 'not-allowed' : 'pointer',
            fontSize: '0.9rem',
          }}
        >
          {saving ? 'Saving…' : 'Save timezone'}
        </button>
      </div>
    </ConfigurationSection>
  );
});
