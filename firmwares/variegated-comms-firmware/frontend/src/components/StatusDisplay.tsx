import { useState } from 'preact/hooks';
import { memo } from 'preact/compat';
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

const StatusDisplayComponent = ({ status, routines }: StatusDisplayProps) => {
  const { getBoilerEntries, getGroupEntries } = useMachine();
  const [boilersExpanded, setBoilersExpanded] = useState(true);
  const [groupsExpanded, setGroupsExpanded] = useState(true);
  const [steamWandsExpanded, setSteamWandsExpanded] = useState(true);
  const [detailsExpanded, setDetailssExpanded] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [successMessage, setSuccessMessage] = useState<string | null>(null);

  const showSuccess = (message: string) => {
    setSuccessMessage(message);
    setTimeout(() => setSuccessMessage(null), 3000);
  };

  const showError = (message: string) => {
    setError(message);
    setTimeout(() => setError(null), 5000);
  };

  const handleSetMode = (modeType: 'On' | 'Off' | 'PowerSaveStandby') => {
    const ws = getWebSocketService();
    if (!ws) {
      showError('WebSocket not connected');
      return;
    }
    ws.setMode(modeType);
    showSuccess(`Machine mode set to ${modeType}`);
  };

  const boilerEntries = getBoilerEntries();
  const groupEntries = getGroupEntries();
  const steamWandStatusEntries = Array.from(status.steam_wand_statuses.entries());
  const tankStatusEntries = Array.from(status.tank_statuses.entries());
  const waterTapStatusEntries = Array.from(status.water_tap_statuses.entries());

  return (
    <div
      style={{
        backgroundColor: 'white',
        borderRadius: '8px',
        padding: '1.5rem',
        boxShadow: '0 2px 4px rgba(0,0,0,0.1)'
      }}
    >
      <h2 style={{ marginTop: 0, marginBottom: '1rem' }}>Machine Status</h2>

      {/* Error/Success Messages */}
      {error && (
        <div style={{
          fontSize: '0.9rem',
          color: '#721c24',
          marginBottom: '0.75rem',
          padding: '0.5rem',
          backgroundColor: '#f8d7da',
          borderRadius: '4px',
          border: '1px solid #f5c6cb'
        }}>
          ❌ {error}
        </div>
      )}

      {successMessage && (
        <div style={{
          fontSize: '0.9rem',
          color: '#155724',
          marginBottom: '0.75rem',
          padding: '0.5rem',
          backgroundColor: '#d4edda',
          borderRadius: '4px',
          border: '1px solid #c3e6cb'
        }}>
          ✅ {successMessage}
        </div>
      )}

      {/* Top Level Status */}
      <div style={{ display: 'flex', gap: '1rem', marginBottom: '1rem', flexWrap: 'wrap' }}>
        {/* Machine Mode */}
        <div
          style={{
            flex: '1 1 auto',
            padding: '0.75rem 1rem',
            backgroundColor: '#f8f9fa',
            borderRadius: '6px'
          }}
        >
          <div style={{ fontWeight: '500', marginBottom: '0.5rem' }}>Machine Mode:</div>
          <div style={{ display: 'flex', gap: '0.5rem' }}>
            <button
              onClick={() => void handleSetMode('On')}
              disabled={status.mode.type === 'On'}
              style={{
                flex: 1,
                padding: '0.5rem 0.75rem',
                backgroundColor: status.mode.type === 'On' ? '#28a745' : '#e0e0e0',
                color: status.mode.type === 'On' ? 'white' : '#333',
                border: status.mode.type === 'On' ? '2px solid #28a745' : '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '0.875rem',
                fontWeight: '500',
                cursor: status.mode.type === 'On' ? 'default' : 'pointer',
                opacity: status.mode.type === 'On' ? 1 : 0.9
              }}
            >
              On
            </button>
            <button
              onClick={() => void handleSetMode('Off')}
              disabled={status.mode.type === 'Off'}
              style={{
                flex: 1,
                padding: '0.5rem 0.75rem',
                backgroundColor: status.mode.type === 'Off' ? '#6c757d' : '#e0e0e0',
                color: status.mode.type === 'Off' ? 'white' : '#333',
                border: status.mode.type === 'Off' ? '2px solid #6c757d' : '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '0.875rem',
                fontWeight: '500',
                cursor: status.mode.type === 'Off' ? 'default' : 'pointer',
                opacity: status.mode.type === 'Off' ? 1 : 0.9
              }}
            >
              Off
            </button>
            <button
              onClick={() => void handleSetMode('PowerSaveStandby')}
              disabled={status.mode.type === 'PowerSaveStandby'}
              style={{
                flex: 1,
                padding: '0.5rem 0.75rem',
                backgroundColor: status.mode.type === 'PowerSaveStandby' ? '#ffc107' : '#e0e0e0',
                color: status.mode.type === 'PowerSaveStandby' ? '#333' : '#333',
                border: status.mode.type === 'PowerSaveStandby' ? '2px solid #ffc107' : '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '0.875rem',
                fontWeight: '500',
                cursor: status.mode.type === 'PowerSaveStandby' ? 'default' : 'pointer',
                opacity: status.mode.type === 'PowerSaveStandby' ? 1 : 0.9
              }}
            >
              Power Save
            </button>
          </div>
        </div>

        {/* SD card.
            Shown here rather than only inside the shot-log panel, because an absent card
            is the explanation for an empty shot list and is worth seeing without opening
            the panel to find out.

            Hidden entirely when `sd_card_present` is null -- this build has no SD
            storage, so there is nothing for a user to act on. Note the explicit `!==
            null`: `!status.sd_card_present` would be true for null as well and would
            render "No card" on a machine that never had a slot. */}
        {status.sd_card_present !== null && (
          <div
            style={{
              flex: '1 1 auto',
              padding: '0.75rem 1rem',
              backgroundColor: '#f8f9fa',
              borderRadius: '6px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <span style={{ fontWeight: '500' }}>SD card:</span>
            <span
              style={{
                padding: '0.25rem 0.75rem',
                backgroundColor: status.sd_card_present ? '#28a745' : '#dc3545',
                color: 'white',
                borderRadius: '12px',
                fontSize: '0.875rem',
                fontWeight: '500'
              }}
            >
              {status.sd_card_present ? 'Inserted' : 'No card'}
            </span>
          </div>
        )}

        {/* WiFi Status */}
        {status.comms_status && (
          <div
            style={{
              flex: '1 1 auto',
              padding: '0.75rem 1rem',
              backgroundColor: '#f8f9fa',
              borderRadius: '6px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <span style={{ fontWeight: '500' }}>WiFi:</span>
            <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
              <span
                style={{
                  padding: '0.25rem 0.75rem',
                  backgroundColor: status.comms_status.wifi_connected ? '#28a745' : '#dc3545',
                  color: 'white',
                  borderRadius: '12px',
                  fontSize: '0.875rem',
                  fontWeight: '500'
                }}
              >
                {status.comms_status.wifi_connected ? 'Connected' : 'Disconnected'}
              </span>
              {status.comms_status.wifi_connected && status.comms_status.wifi_rssi !== null && status.comms_status.wifi_rssi !== undefined && (
                <span
                  style={{
                    fontSize: '0.875rem',
                    color: '#666',
                    display: 'flex',
                    alignItems: 'center',
                    gap: '0.25rem'
                  }}
                  title={`Signal Strength: ${status.comms_status.wifi_rssi} dBm`}
                >
                  <span>{status.comms_status.wifi_rssi} dBm</span>
                  <span style={{ fontSize: '1rem' }}>
                    {status.comms_status.wifi_rssi >= -60 ? '📶' :
                     status.comms_status.wifi_rssi >= -70 ? '📶' :
                     '📶'}
                  </span>
                  <span style={{ fontSize: '0.75rem', fontWeight: '500', color: status.comms_status.wifi_rssi >= -60 ? '#28a745' : status.comms_status.wifi_rssi >= -70 ? '#ffc107' : '#dc3545' }}>
                    ({status.comms_status.wifi_rssi >= -60 ? 'Excellent' :
                      status.comms_status.wifi_rssi >= -70 ? 'Good' :
                      'Poor'})
                  </span>
                </span>
              )}
            </div>
          </div>
        )}

        {/* Current Time */}
        {status.current_local_time && (
          <div
            style={{
              flex: '1 1 auto',
              padding: '0.75rem 1rem',
              backgroundColor: '#f8f9fa',
              borderRadius: '6px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <span style={{ fontWeight: '500' }}>Time:</span>
            <span style={{ fontSize: '0.875rem', fontWeight: '500', fontFamily: 'monospace' }}>
              {new Date(status.current_local_time).toLocaleString('en-US', {
                year: 'numeric',
                month: '2-digit',
                day: '2-digit',
                hour: '2-digit',
                minute: '2-digit',
                second: '2-digit',
                hour12: false
              })}
            </span>
          </div>
        )}
      </div>

      {/* Routine Execution */}
      {status.routine_execution && (
        <RoutineExecutionCard execution={status.routine_execution} routines={routines} status={status} />
      )}


        {/* Groups Section */}
        {groupEntries.length > 0 && (
            <div style={{ marginBottom: '1rem' }}>
                <button
                    onClick={() => setGroupsExpanded(!groupsExpanded)}
                    style={{
                        width: '100%',
                        padding: '0.75rem',
                        backgroundColor: '#f8f9fa',
                        border: '1px solid #ddd',
                        borderRadius: '6px',
                        cursor: 'pointer',
                        display: 'flex',
                        justifyContent: 'space-between',
                        alignItems: 'center',
                        fontSize: '1rem',
                        fontWeight: '500'
                    }}
                >
                    <span>Groups ({groupEntries.length})</span>
                    <span style={{ fontSize: '1.2rem' }}>{groupsExpanded ? '▼' : '▶'}</span>
                </button>

                {groupsExpanded && (
                    <div
                        style={{
                            display: 'flex',
                            gap: '1rem',
                            marginTop: '1rem',
                            flexWrap: 'wrap'
                        }}
                    >
                        {groupEntries.map(([key], index) => {
                            const groupStatus = status.group_statuses.get(key);
                            return groupStatus ? (
                                <GroupStatusCard key={key} index={index} status={groupStatus} />
                            ) : null;
                        })}
                    </div>
                )}
            </div>
        )}

      {/* Boilers Section */}
      {boilerEntries.length > 0 && (
        <div style={{ marginBottom: '1rem' }}>
          <button
            onClick={() => setBoilersExpanded(!boilersExpanded)}
            style={{
              width: '100%',
              padding: '0.75rem',
              backgroundColor: '#f8f9fa',
              border: '1px solid #ddd',
              borderRadius: '6px',
              cursor: 'pointer',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
              fontSize: '1rem',
              fontWeight: '500'
            }}
          >
            <span>Boilers ({boilerEntries.length})</span>
            <span style={{ fontSize: '1.2rem' }}>{boilersExpanded ? '▼' : '▶'}</span>
          </button>

          {boilersExpanded && (
            <div
              style={{
                display: 'flex',
                gap: '1rem',
                marginTop: '1rem',
                flexWrap: 'wrap'
              }}
            >
              {boilerEntries.map(([key], index) => {
                const boilerStatus = status.boiler_statuses.get(key);
                return boilerStatus ? (
                  <BoilerStatusCard key={key} index={index} status={boilerStatus} />
                ) : null;
              })}
            </div>
          )}
        </div>
      )}

      {/* Steam Wands Section (collapsible) */}
      {status.steam_wand_statuses.size > 0 && (
        <div>
          <button
            onClick={() => setSteamWandsExpanded(!steamWandsExpanded)}
            style={{
              width: '100%',
              padding: '0.75rem',
              backgroundColor: '#f8f9fa',
              border: '1px solid #ddd',
              borderRadius: '6px',
              cursor: 'pointer',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
              fontSize: '1rem',
              fontWeight: '500'
            }}
          >
            <span>Steam Wands ({status.steam_wand_statuses.size})</span>
            <span style={{ fontSize: '1.2rem' }}>{steamWandsExpanded ? '▼' : '▶'}</span>
          </button>

          {steamWandsExpanded && (
            <div
              style={{
                display: 'flex',
                gap: '1rem',
                marginTop: '1rem',
                flexWrap: 'wrap'
              }}
            >
              {steamWandStatusEntries.map(([key, wandStatus]) => (
                <SteamWandStatusCard key={key} index={key} status={wandStatus} />
              ))}
            </div>
          )}
        </div>
      )}

      {/* Additional Details Section (collapsible) */}
      <div>
        <button
          onClick={() => setDetailssExpanded(!detailsExpanded)}
          style={{
            width: '100%',
            padding: '0.75rem',
            backgroundColor: '#f8f9fa',
            border: '1px solid #ddd',
            borderRadius: '6px',
            cursor: 'pointer',
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center',
            fontSize: '1rem',
            fontWeight: '500'
          }}
        >
          <span>Other Components</span>
          <span style={{ fontSize: '1.2rem' }}>{detailsExpanded ? '▼' : '▶'}</span>
        </button>

        {detailsExpanded && (
          <div style={{ marginTop: '1rem', display: 'flex', flexDirection: 'column', gap: '1rem' }}>
            {/* Tanks */}
            {status.tank_statuses.size > 0 && (
              <div
                style={{
                  padding: '1rem',
                  backgroundColor: '#f8f9fa',
                  borderRadius: '6px',
                  border: '1px solid #e0e0e0'
                }}
              >
                <h4 style={{ margin: '0 0 0.5rem 0' }}>Tanks</h4>
                {tankStatusEntries.map(([key, tank]) => (
                  <div key={key} style={{ fontSize: '0.9rem' }}>
                    <strong>{key}:</strong>
                    {tank.water_level !== null && tank.water_level !== undefined && (
                      <span> Water Level: {tank.water_level.toFixed(1)}%</span>
                    )}
                  </div>
                ))}
              </div>
            )}

            {/* Water Taps */}
            {status.water_tap_statuses.size > 0 && (
              <div
                style={{
                  padding: '1rem',
                  backgroundColor: '#f8f9fa',
                  borderRadius: '6px',
                  border: '1px solid #e0e0e0'
                }}
              >
                <h4 style={{ margin: '0 0 0.5rem 0' }}>Water Taps</h4>
                {waterTapStatusEntries.map(([key, tap]) => (
                  <div key={key} style={{ fontSize: '0.9rem' }}>
                    <strong>{key}:</strong> {tap.is_dispensing ? 'Dispensing' : 'Idle'}
                  </div>
                ))}
              </div>
            )}

            {/* Peripherals */}
            {status.peripheral_status.peripherals.size > 0 && (
              <div
                style={{
                  padding: '1rem',
                  backgroundColor: '#f8f9fa',
                  borderRadius: '6px',
                  border: '1px solid #e0e0e0'
                }}
              >
                <h4 style={{ margin: '0 0 0.5rem 0' }}>Peripherals</h4>
                {Array.from(status.peripheral_status.peripherals.entries()).map(([key, peripheral]) => (
                  <div key={key} style={{ fontSize: '0.9rem', display: 'flex', justifyContent: 'space-between' }}>
                    <span>
                      <strong>{key}</strong> ({peripheral.peripheral_type})
                    </span>
                    <span
                      style={{
                        padding: '0.125rem 0.5rem',
                        backgroundColor: peripheral.is_available ? '#28a745' : '#6c757d',
                        color: 'white',
                        borderRadius: '8px',
                        fontSize: '0.75rem'
                      }}
                    >
                      {peripheral.is_available ? 'Available' : 'Unavailable'}
                    </span>
                  </div>
                ))}
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
};

export const StatusDisplay = memo(StatusDisplayComponent);
