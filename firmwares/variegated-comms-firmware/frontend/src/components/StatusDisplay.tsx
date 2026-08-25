import type { ComponentChildren } from 'preact';
import { useState } from 'preact/hooks';
import { memo } from 'preact/compat';
import {
  Alert,
  Badge,
  Readout,
  Section,
  focusRingStyle,
  tokens,
  useInteractive,
} from '@variegated-coffee/ui';
import { Status, RoutineSummaryStorage } from '../schemas/schemas';
import { useMachine } from '../contexts/MachineContext';
import { BoilerStatusCard } from './BoilerStatusCard';
import { GroupStatusCard } from './GroupStatusCard';
import { SteamWandStatusCard } from './SteamWandStatusCard';
import { RoutineExecutionCard } from './RoutineExecutionCard';
import { getWebSocketService } from '../services/websocket';

interface StatusDisplayProps {
  status: Status;
  routines: RoutineSummaryStorage;
}

type MachineMode = 'On' | 'Off' | 'PowerSaveStandby';

const MODES: { value: MachineMode; label: string }[] = [
  { value: 'On', label: 'On' },
  { value: 'Off', label: 'Off' },
  { value: 'PowerSaveStandby', label: 'Power save' },
];

/**
 * One of three mutually exclusive machine modes.
 *
 * Rendered as a real radio group rather than three buttons. It was three `<button>`s, each
 * of which turned a different colour when it happened to be the active one -- green for
 * On, grey for Off, amber for Power save -- so the control had three selected appearances
 * and nothing that said "these are alternatives". Tab reached all three separately, and
 * nothing announced which was current.
 *
 * `role="radio"` gives it the semantics it always had, and one selected treatment means
 * the selected state is comparable across the three.
 */
function ModeSelector({
  mode,
  onSelect,
}: {
  mode: MachineMode;
  onSelect: (mode: MachineMode) => void;
}) {
  return (
    <div
      role="radiogroup"
      aria-label="Machine mode"
      style={{ display: 'flex', gap: tokens.space.sm }}
    >
      {MODES.map((option) => (
        <ModeOption
          key={option.value}
          label={option.label}
          selected={mode === option.value}
          onSelect={() => onSelect(option.value)}
        />
      ))}
    </div>
  );
}

function ModeOption({
  label,
  selected,
  onSelect,
}: {
  label: string;
  selected: boolean;
  onSelect: () => void;
}) {
  const { hovered, focusRing, handlers } = useInteractive();

  return (
    <button
      type="button"
      role="radio"
      aria-checked={selected}
      onClick={onSelect}
      style={{
        flex: 1,
        padding: `${tokens.space.sm} ${tokens.space.sm}`,
        background: selected
          ? tokens.color.info
          : hovered
            ? tokens.color.surface
            : tokens.color.surfaceRaised,
        color: selected ? tokens.color.surfaceRaised : tokens.color.ink,
        border: `1px solid ${selected ? tokens.color.info : tokens.color.border}`,
        borderRadius: tokens.radius.sm,
        font: `0.875rem ${tokens.font.sans}`,
        fontWeight: selected ? 500 : 400,
        cursor: 'pointer',
        ...(focusRing ? focusRingStyle(tokens.color.info) : {}),
      }}
      {...handlers}
    >
      {label}
    </button>
  );
}

/**
 * Wi-Fi signal strength, in words.
 *
 * The emoji is gone. All three branches of the old ternary rendered the same `📶`, so the
 * glyph carried no information at any strength -- and the dBm figure sat behind a `title`
 * tooltip, which a machine-side tablet has no way to show. The word and the number are
 * both visible now, which is what the tooltip was standing in for.
 */
function wifiQuality(rssi: number): { label: string; role: 'ok' | 'warn' | 'danger' } {
  if (rssi >= -60) return { label: 'Excellent', role: 'ok' };
  if (rssi >= -70) return { label: 'Good', role: 'warn' };
  return { label: 'Poor', role: 'danger' };
}

/** The strip of facts across the top: one plane, one set of paddings. */
function Tile({ children }: { children: ComponentChildren }) {
  return (
    <div
      style={{
        flex: '1 1 auto',
        display: 'flex',
        flexDirection: 'column',
        gap: tokens.space.sm,
        padding: tokens.space.sm,
        backgroundColor: tokens.color.surfaceSunken,
        border: `1px solid ${tokens.color.border}`,
        borderRadius: tokens.radius.sm,
      }}
    >
      {children}
    </div>
  );
}

const StatusDisplayComponent = ({ status, routines }: StatusDisplayProps) => {
  const { getBoilerEntries, getGroupEntries } = useMachine();
  const [error, setError] = useState<string | null>(null);
  const [notice, setNotice] = useState<string | null>(null);

  const showNotice = (message: string) => {
    setNotice(message);
    setTimeout(() => setNotice(null), 3000);
  };

  const showError = (message: string) => {
    setError(message);
    setTimeout(() => setError(null), 5000);
  };

  const handleSetMode = (modeType: MachineMode) => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('Not connected to the machine');
      return;
    }
    ws.setMode(modeType);
    // What is known: the command went out. The selected mode above updates when the
    // machine reports it, which is the actual confirmation.
    showNotice('Mode change sent');
  };

  const boilerEntries = getBoilerEntries();
  const groupEntries = getGroupEntries();
  const steamWandStatusEntries = Array.from(status.steam_wand_statuses.entries());
  const tankStatusEntries = Array.from(status.tank_statuses.entries());
  const waterTapStatusEntries = Array.from(status.water_tap_statuses.entries());

  const rssi = status.comms_status?.wifi_rssi;
  const quality = rssi !== null && rssi !== undefined ? wifiQuality(rssi) : null;

  const cardRow = {
    display: 'flex',
    gap: tokens.space.md,
    flexWrap: 'wrap' as const,
  };

  return (
    <div
      style={{
        display: 'flex',
        flexDirection: 'column',
        gap: tokens.space.md,
        backgroundColor: tokens.color.surfaceRaised,
        border: `1px solid ${tokens.color.border}`,
        borderRadius: tokens.radius.md,
        padding: tokens.space.lg,
      }}
    >
      <h2 style={{ margin: 0 }}>Machine status</h2>

      {error && <Alert role="danger">{error}</Alert>}
      {notice && <Alert role="ok">{notice}</Alert>}

      <div style={{ display: 'flex', gap: tokens.space.md, flexWrap: 'wrap' }}>
        <Tile>
          <div style={{ font: `0.85rem ${tokens.font.sans}`, color: tokens.color.inkMuted }}>
            Machine mode
          </div>
          <ModeSelector mode={status.mode.type as MachineMode} onSelect={handleSetMode} />
        </Tile>

        {/* SD card.
            Shown here rather than only inside the shot-log panel, because an absent card
            is the explanation for an empty shot list and is worth seeing without opening
            the panel to find out.

            Hidden entirely when `sd_card_present` is null -- this build has no SD
            storage, so there is nothing for a user to act on. Note the explicit `!==
            null`: `!status.sd_card_present` would be true for null as well and would
            render "No card" on a machine that never had a slot. */}
        {status.sd_card_present !== null && (
          <Tile>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm }}>
              <span style={{ font: `0.85rem ${tokens.font.sans}`, color: tokens.color.inkMuted }}>
                SD card
              </span>
              <Badge role={status.sd_card_present ? 'ok' : 'warn'}>
                {status.sd_card_present ? 'Inserted' : 'No card'}
              </Badge>
            </div>
          </Tile>
        )}

        {status.comms_status && (
          <Tile>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm, flexWrap: 'wrap' }}>
              <span style={{ font: `0.85rem ${tokens.font.sans}`, color: tokens.color.inkMuted }}>
                Wi-Fi
              </span>
              <div style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm }}>
                <Badge role={status.comms_status.wifi_connected ? 'ok' : 'danger'}>
                  {status.comms_status.wifi_connected ? 'Connected' : 'Disconnected'}
                </Badge>
                {status.comms_status.wifi_connected && quality && rssi !== null && rssi !== undefined && (
                  <>
                    <Badge role={quality.role}>{quality.label}</Badge>
                    <span
                      style={{
                        font: `0.8rem ${tokens.font.mono}`,
                        fontVariantNumeric: 'tabular-nums',
                        color: tokens.color.inkMuted,
                      }}
                    >
                      {rssi} dBm
                    </span>
                  </>
                )}
              </div>
            </div>
          </Tile>
        )}

        {status.current_local_time && (
          <Tile>
            {/*
              Rendered as sent, not re-parsed. `current_local_time` is a chrono `NaiveDateTime`
              -- an ISO-8601 string with no offset -- already in the *machine's* zone.

              `new Date(s)` parses an offset-less form in the *browser's* zone and
              `toLocaleString` formats it back in the same zone, so the two conversions cancel
              and the old code was right by accident. They stop cancelling across a DST
              transition in the browser's zone, where a wall-clock time that is ambiguous or
              does not exist there is shifted by an hour -- on a machine whose entire point is
              doing something at a particular hour. It also forced `en-US` MM/DD/YYYY on
              everyone regardless of locale.
            */}
            <Readout
              label="Machine time"
              value={status.current_local_time.replace('T', ' ')}
            />
          </Tile>
        )}
      </div>

      {status.routine_execution && (
        <RoutineExecutionCard execution={status.routine_execution} routines={routines} status={status} />
      )}

      {groupEntries.length > 0 && (
        <Section title="Groups" annotation={<Badge numeric>{groupEntries.length}</Badge>}>
          <div style={cardRow}>
            {groupEntries.map(([key], index) => {
              const groupStatus = status.group_statuses.get(key);
              return groupStatus ? (
                <GroupStatusCard key={key} index={index} status={groupStatus} />
              ) : null;
            })}
          </div>
        </Section>
      )}

      {boilerEntries.length > 0 && (
        <Section title="Boilers" annotation={<Badge numeric>{boilerEntries.length}</Badge>}>
          <div style={cardRow}>
            {boilerEntries.map(([key], index) => {
              const boilerStatus = status.boiler_statuses.get(key);
              return boilerStatus ? (
                <BoilerStatusCard key={key} index={index} status={boilerStatus} />
              ) : null;
            })}
          </div>
        </Section>
      )}

      {status.steam_wand_statuses.size > 0 && (
        <Section
          title="Steam wands"
          annotation={<Badge numeric>{status.steam_wand_statuses.size}</Badge>}
        >
          <div style={cardRow}>
            {steamWandStatusEntries.map(([key, wandStatus]) => (
              <SteamWandStatusCard key={key} index={key} status={wandStatus} />
            ))}
          </div>
        </Section>
      )}

      <Section title="Other components" defaultOpen={false}>
        <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.md }}>
          {status.tank_statuses.size > 0 && (
            <Tile>
              <h4 style={{ margin: 0 }}>Tanks</h4>
              {tankStatusEntries.map(([key, tank]) => (
                <Readout
                  key={key}
                  label={`Tank ${key}`}
                  value={
                    tank.water_level !== null && tank.water_level !== undefined
                      ? tank.water_level.toFixed(1)
                      : '—'
                  }
                  unit={
                    tank.water_level !== null && tank.water_level !== undefined ? '%' : undefined
                  }
                />
              ))}
            </Tile>
          )}

          {status.water_tap_statuses.size > 0 && (
            <Tile>
              <h4 style={{ margin: 0 }}>Water taps</h4>
              {waterTapStatusEntries.map(([key, tap]) => (
                <Readout
                  key={key}
                  label={`Tap ${key}`}
                  value={tap.is_dispensing ? 'Dispensing' : 'Idle'}
                />
              ))}
            </Tile>
          )}

          {status.peripheral_status.peripherals.size > 0 && (
            <Tile>
              <h4 style={{ margin: 0 }}>Peripherals</h4>
              {Array.from(status.peripheral_status.peripherals.entries()).map(
                ([key, peripheral]) => (
                  <div
                    key={key}
                    style={{
                      display: 'flex',
                      justifyContent: 'space-between',
                      alignItems: 'center',
                      gap: tokens.space.sm,
                      fontSize: '0.9rem',
                    }}
                  >
                    <span>
                      <strong>{key}</strong>{' '}
                      <span style={{ color: tokens.color.inkMuted }}>
                        ({peripheral.peripheral_type})
                      </span>
                    </span>
                    <Badge role={peripheral.is_available ? 'ok' : undefined}>
                      {peripheral.is_available ? 'Available' : 'Unavailable'}
                    </Badge>
                  </div>
                )
              )}
            </Tile>
          )}
        </div>
      </Section>
    </div>
  );
};

export const StatusDisplay = memo(StatusDisplayComponent);
