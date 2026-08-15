import { memo } from 'preact/compat';
import { useCallback, useEffect, useState } from 'preact/hooks';
import { ShotAnnotations, ShotLogId, ShotLogList, ShotLogListEntry } from '../schemas/schemas';
import * as shotLogApi from '../api/shotLogs';
import {
  doseWeight,
  formatKey,
  formatValue,
  MAX_SHOT_ANNOTATIONS,
  setText,
  textOf,
} from '../utils/shotAnnotations';

interface ShotLogPanelProps {
  /** From `status.pending_shot_annotations` -- what the next shot will be stamped with. */
  pending: ShotAnnotations;
  /**
   * From `status.sd_card_present`.
   *
   * Three states, and they are not interchangeable: `null` means this machine has no SD
   * storage at all, `false` means the slot is empty. The first is nothing a user can act
   * on; the second is "insert a card".
   */
  sdCardPresent: boolean | null;
  /** Group indices that have a scale, for the dose buttons. */
  groupIndices: number[];
}

const buttonStyle = {
  padding: '0.4rem 0.75rem',
  border: 'none',
  borderRadius: '4px',
  fontSize: '0.85rem',
  cursor: 'pointer',
  color: 'white',
  background: '#0066cc',
};

const secondaryButtonStyle = {
  ...buttonStyle,
  background: '#666',
};

const chipStyle = {
  display: 'inline-block',
  padding: '0.15rem 0.5rem',
  marginRight: '0.35rem',
  borderRadius: '10px',
  fontSize: '0.75rem',
  background: '#eef2f7',
  color: '#333',
};

const inputStyle = {
  padding: '0.35rem 0.5rem',
  border: '1px solid #ccc',
  borderRadius: '4px',
  fontSize: '0.85rem',
};

const noteStyle = { color: '#666', fontSize: '0.85rem' };

/**
 * `YYYYMMDD` + `HHMMSSxx` rendered as something readable.
 *
 * Undated shots -- those recorded before the machine's clock synced -- have no date to
 * show, and their `time` is a counter rather than a time of day, so it is deliberately
 * not formatted as one.
 */
function formatShotTime(id: ShotLogId): string {
  const time = String(id.time).padStart(8, '0');
  const clock = `${time.slice(0, 2)}:${time.slice(2, 4)}:${time.slice(4, 6)}`;

  if (id.day === null) {
    return `undated (#${id.time})`;
  }
  const day = String(id.day).padStart(8, '0');
  return `${day.slice(0, 4)}-${day.slice(4, 6)}-${day.slice(6, 8)} ${clock}`;
}

function formatSize(bytes: number): string {
  return bytes < 1024 ? `${bytes} B` : `${(bytes / 1024).toFixed(1)} kB`;
}

const ShotLogPanelComponent = ({ pending, sdCardPresent, groupIndices }: ShotLogPanelProps) => {
  const [logs, setLogs] = useState<ShotLogList | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Draft state for the next-shot strip. Held locally rather than driven straight from
  // `pending` so that typing is not overwritten by the 1 Hz status push mid-keystroke;
  // it is seeded from `pending` and reconciled on save.
  const [beansDraft, setBeansDraft] = useState<string | null>(null);
  const [grindDraft, setGrindDraft] = useState<string | null>(null);

  const beans = beansDraft ?? textOf(pending, 'Beans');
  const grind = grindDraft ?? textOf(pending, 'GrindSize');
  const dose = doseWeight(pending);

  const refresh = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      setLogs(await shotLogApi.fetchShotLogs());
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Could not load shots');
    } finally {
      setLoading(false);
    }
  }, []);

  // On mount and on an explicit Refresh only. A shot list changes once per shot, and
  // polling it would put an SD card read behind every tick of the status stream.
  useEffect(() => {
    void refresh();
  }, [refresh]);

  const run = async (action: () => Promise<void>) => {
    setError(null);
    try {
      await action();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Command failed');
    }
  };

  const savePending = async () => {
    let next: ShotAnnotations | null = setText(pending, 'Beans', beans);
    if (next) next = setText(next, 'GrindSize', grind);
    if (!next) {
      setError(`A shot can carry at most ${MAX_SHOT_ANNOTATIONS} annotations`);
      return;
    }
    await run(async () => {
      await shotLogApi.setPendingAnnotations(next);
      // Cleared so the fields fall back to whatever the machine reports next. Without
      // this the drafts would keep shadowing `pending`, and the strip would go on showing
      // what was typed even after the machine cleared it at the end of a shot.
      setBeansDraft(null);
      setGrindDraft(null);
    });
  };

  const entries: ShotLogListEntry[] = logs?.entries ?? [];

  // Three different situations that look identical if you only check for an empty list,
  // and lead somewhere completely different.
  const emptyMessage = (): string => {
    if (sdCardPresent === null) return 'This machine has no SD card storage.';
    if (sdCardPresent === false) return 'No SD card inserted.';
    return 'No shots recorded yet.';
  };

  return (
    <div>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1rem' }}>
        <h2 style={{ margin: 0 }}>Shot log</h2>
        <button style={secondaryButtonStyle} onClick={() => void refresh()} disabled={loading}>
          {loading ? 'Loading…' : 'Refresh'}
        </button>
      </div>

      {error && (
        <div style={{ padding: '0.5rem 0.75rem', marginBottom: '1rem', borderRadius: '4px', background: '#ffe6e6', color: '#660000', fontSize: '0.85rem' }}>
          {error}
        </div>
      )}

      {/* Next shot */}
      <div style={{ padding: '0.75rem', marginBottom: '1rem', background: '#f7f9fc', borderRadius: '6px' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', flexWrap: 'wrap' }}>
          <strong style={{ fontSize: '0.9rem' }}>Next shot</strong>
          <input
            style={inputStyle}
            placeholder="Beans"
            value={beans}
            onInput={(e) => setBeansDraft((e.target as HTMLInputElement).value)}
          />
          <input
            style={inputStyle}
            placeholder="Grind"
            value={grind}
            onInput={(e) => setGrindDraft((e.target as HTMLInputElement).value)}
          />
          <span style={noteStyle}>{dose === null ? 'no dose' : `dose ${dose} g`}</span>
          <button style={buttonStyle} onClick={() => void savePending()}>
            Save
          </button>
          {groupIndices.map((group) => (
            <button
              key={group}
              style={secondaryButtonStyle}
              onClick={() => void run(() => shotLogApi.tagDoseFromScale(group))}
            >
              {groupIndices.length > 1 ? `Take dose from group ${group + 1}` : 'Take dose from scale'}
            </button>
          ))}
        </div>
        <div style={{ ...noteStyle, marginTop: '0.5rem' }}>
          Put the basket on the scale before taking a dose &mdash; the reading is used as it
          stands, and the scale is not tared. Cleared by the machine when a shot ends.
        </div>
      </div>

      {entries.length === 0 ? (
        <div style={noteStyle}>{emptyMessage()}</div>
      ) : (
        <div>
          {entries.map((entry) => {
            const key = `${entry.id.day ?? 'nodate'}-${entry.id.time}`;
            return (
              <div
                key={key}
                style={{ display: 'flex', alignItems: 'center', gap: '0.75rem', padding: '0.5rem 0', borderBottom: '1px solid #eee', flexWrap: 'wrap' }}
              >
                <span style={{ fontFamily: 'monospace', fontSize: '0.85rem', minWidth: '11rem' }}>
                  {formatShotTime(entry.id)}
                </span>
                <span style={noteStyle}>{formatSize(entry.size_bytes)}</span>
                <span style={{ flex: 1 }}>
                  {entry.annotations.entries.map((annotation, i) => (
                    <span key={i} style={chipStyle}>
                      {formatKey(annotation.key)}: {formatValue(annotation.value)}
                    </span>
                  ))}
                </span>
                {/* A plain anchor is the whole download implementation: same origin, and
                    the firmware sets Content-Disposition, so the browser saves it under a
                    name that stays unique across days. */}
                <a
                  href={shotLogApi.shotDownloadUrl(entry.id)}
                  download
                  style={{ ...secondaryButtonStyle, textDecoration: 'none' }}
                >
                  Download
                </a>
              </div>
            );
          })}
          {logs?.truncated && (
            <div style={{ ...noteStyle, marginTop: '0.75rem' }}>
              Showing the {entries.length} most recent shots. There are more on the card.
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export const ShotLogPanel = memo(ShotLogPanelComponent);
