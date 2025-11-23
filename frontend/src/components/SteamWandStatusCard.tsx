import { memo } from 'preact/compat';
import { useState } from 'preact/hooks';
import { useMachine } from '../contexts/MachineContext';
import { SteamWandStatus } from '../schemas/schemas';

interface SteamWandStatusCardProps {
  index: number;
  status: SteamWandStatus;
}

const SteamWandStatusCardComponent = ({ index, status }: SteamWandStatusCardProps) => {
  const { getSteamWandName } = useMachine();
  const name = getSteamWandName(index);
  const [error, setError] = useState<string | null>(null);
  const [successMessage, setSuccessMessage] = useState<string | null>(null);
  const [isAdjusting, setIsAdjusting] = useState(false);
  const [opennessValue, setOpennessValue] = useState(status.valve_openness);

  const showSuccess = (message: string) => {
    setSuccessMessage(message);
    setTimeout(() => setSuccessMessage(null), 3000);
  };

  const showError = (message: string) => {
    setError(message);
    setTimeout(() => setError(null), 5000);
  };

  const handleOpennessChange = async (newOpenness: number) => {
    try {
      setIsAdjusting(true);

      // Prepare request data
      const requestData = {
        steam_wand_index: index,
        openness: newOpenness
      };

      // Serialize to postcard
      const binary = await window.postcard.serialize(requestData);

      const response = await fetch('/command/set-steam-valve-openness', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/octet-stream',
        },
        body: binary as BodyInit,
      });

      if (!response.ok) {
        throw new Error(`Failed to set valve openness: ${response.statusText}`);
      }

      showSuccess(`Valve openness set to ${newOpenness}%`);
    } catch (err) {
      showError(err instanceof Error ? err.message : 'Unknown error occurred');
      // Revert to previous value on error
      setOpennessValue(status.valve_openness);
    } finally {
      setIsAdjusting(false);
    }
  };

  // Determine status color based on steaming state
  const getStatusColor = () => {
    if (status.is_steaming) {
      return '#28a745'; // green - active
    }
    return '#6c757d'; // gray - idle
  };

  return (
    <div
      style={{
        padding: '1rem',
        backgroundColor: 'white',
        border: status.is_steaming ? '2px solid #28a745' : '1px solid #ddd',
        borderRadius: '8px',
        flex: '1 1 300px',
        minWidth: '250px'
      }}
    >
      {/* Header */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '0.75rem' }}>
        <h3 style={{ margin: 0, fontSize: '1.1rem', fontWeight: '600' }}>{name}</h3>
        <span
          style={{
            padding: '0.25rem 0.75rem',
            backgroundColor: getStatusColor(),
            color: 'white',
            borderRadius: '12px',
            fontSize: '0.75rem',
            fontWeight: '500'
          }}
        >
          {status.is_steaming ? 'STEAMING' : 'IDLE'}
        </span>
      </div>

      {/* Valve Openness */}
      <div style={{ marginBottom: '0.75rem' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '0.5rem' }}>
          <span style={{ fontSize: '0.85rem', color: '#666' }}>Valve Openness:</span>
          <span style={{ fontSize: '0.85rem', fontWeight: '600' }}>{opennessValue}%</span>
        </div>

        {/* Slider */}
        <input
          type="range"
          min="0"
          max="100"
          value={opennessValue}
          disabled={isAdjusting}
          onChange={(e) => setOpennessValue(Number(e.currentTarget.value))}
          onMouseUp={() => void handleOpennessChange(opennessValue)}
          onTouchEnd={() => void handleOpennessChange(opennessValue)}
          style={{
            width: '100%',
            height: '6px',
            borderRadius: '3px',
            background: `linear-gradient(to right, #0066cc 0%, #0066cc ${opennessValue}%, #ddd ${opennessValue}%, #ddd 100%)`,
            outline: 'none',
            opacity: isAdjusting ? 0.5 : 1,
            cursor: isAdjusting ? 'wait' : 'pointer'
          }}
        />
      </div>

      {/* Success Message */}
      {successMessage && (
        <div
          style={{
            marginTop: '0.75rem',
            padding: '0.5rem',
            backgroundColor: '#d4edda',
            color: '#155724',
            borderRadius: '4px',
            fontSize: '0.85rem',
            textAlign: 'center'
          }}
        >
          {successMessage}
        </div>
      )}

      {/* Error Message */}
      {error && (
        <div
          style={{
            marginTop: '0.75rem',
            padding: '0.5rem',
            backgroundColor: '#f8d7da',
            color: '#721c24',
            borderRadius: '4px',
            fontSize: '0.85rem',
            textAlign: 'center'
          }}
        >
          {error}
        </div>
      )}
    </div>
  );
};

export const SteamWandStatusCard = memo(SteamWandStatusCardComponent);
