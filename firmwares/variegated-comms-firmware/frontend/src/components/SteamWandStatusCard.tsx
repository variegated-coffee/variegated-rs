import { memo } from 'preact/compat';
import { useId, useState } from 'preact/hooks';
import { Alert, Badge, tokens } from '@variegated-coffee/ui';
import { useMachine } from '../contexts/MachineContext';
import { SteamWandStatus } from '../schemas/schemas';
import { getWebSocketService } from '../services/websocket';

interface SteamWandStatusCardProps {
  index: number;
  status: SteamWandStatus;
}

const SteamWandStatusCardComponent = ({ index, status }: SteamWandStatusCardProps) => {
  const { getSteamWandName } = useMachine();
  const name = getSteamWandName(index);
  const [error, setError] = useState<string | null>(null);
  const [successMessage, setSuccessMessage] = useState<string | null>(null);
  const [opennessValue, setOpennessValue] = useState(status.valve_openness);
  // A range input is one of the controls `Field` cannot own -- it has no text to format --
  // so it takes the id directly and wires its own label.
  const sliderId = useId();

  const showSuccess = (message: string) => {
    setSuccessMessage(message);
    setTimeout(() => setSuccessMessage(null), 3000);
  };

  const showError = (message: string) => {
    setError(message);
    setTimeout(() => setError(null), 5000);
  };

  const handleOpennessChange = (newOpenness: number) => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('Not connected to the machine');
      setOpennessValue(status.valve_openness);
      return;
    }
    ws.setSteamValveOpenness(index, newOpenness);
    showSuccess(`Valve openness set to ${newOpenness}%`);
  };

  return (
    <div
      style={{
        display: 'flex',
        flexDirection: 'column',
        gap: tokens.space.sm,
        padding: tokens.space.md,
        backgroundColor: tokens.color.surfaceRaised,
        // Steaming is a state worth seeing from across the room, so the whole card carries
        // it. The border keeps its 1px width in both states -- a 2px border on one of them
        // shifted the card's contents by a pixel every time steaming started.
        border: `1px solid ${status.is_steaming ? tokens.color.ok : tokens.color.border}`,
        boxShadow: status.is_steaming ? `0 0 0 1px ${tokens.color.ok}` : undefined,
        borderRadius: tokens.radius.md,
        flex: '1 1 300px',
        minWidth: '250px',
      }}
    >
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm }}>
        <h3 style={{ margin: 0, fontSize: '1.1rem', fontWeight: 600 }}>{name}</h3>
        <Badge role={status.is_steaming ? 'ok' : undefined}>
          {status.is_steaming ? 'Steaming' : 'Idle'}
        </Badge>
      </div>

      <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.xs }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'baseline', gap: tokens.space.sm }}>
          <label
            for={sliderId}
            style={{ font: `0.85rem ${tokens.font.sans}`, color: tokens.color.inkMuted }}
          >
            Valve openness
          </label>
          <span
            style={{
              font: `0.85rem ${tokens.font.mono}`,
              fontVariantNumeric: 'tabular-nums',
              fontWeight: 500,
            }}
          >
            {opennessValue}
            <span style={{ fontSize: '0.85em', color: tokens.color.inkMuted }}> %</span>
          </span>
        </div>

        <input
          id={sliderId}
          type="range"
          min="0"
          max="100"
          value={opennessValue}
          // The value is announced as a percentage rather than as a bare number, which is
          // all a range input says by default.
          aria-valuetext={`${opennessValue}%`}
          onChange={(e) => setOpennessValue(Number(e.currentTarget.value))}
          onMouseUp={() => void handleOpennessChange(opennessValue)}
          onTouchEnd={() => void handleOpennessChange(opennessValue)}
          // Committed on key release as well as on pointer release. Without this the
          // slider could be moved with the arrow keys and never send anything -- the
          // control looked usable from the keyboard and silently was not.
          onKeyUp={() => void handleOpennessChange(opennessValue)}
          style={{
            width: '100%',
            height: '6px',
            borderRadius: '3px',
            background: `linear-gradient(to right, ${tokens.color.info} 0%, ${tokens.color.info} ${opennessValue}%, ${tokens.color.border} ${opennessValue}%, ${tokens.color.border} 100%)`,
            cursor: 'pointer',
          }}
        />
      </div>

      {successMessage && <Alert role="ok">{successMessage}</Alert>}
      {error && <Alert role="danger">{error}</Alert>}
    </div>
  );
};

export const SteamWandStatusCard = memo(SteamWandStatusCardComponent);
