import { useEffect, useState } from 'preact/hooks';
import { ScheduleBuilder } from './components/ScheduleBuilder';
import { RoutineBuilder } from './components/routine/RoutineBuilder';
import { StatusDisplay } from './components/StatusDisplay';
import { ConfigurationPanel } from './components/ConfigurationPanel';
import { JsonModal } from './components/JsonModal';
import { MachineProvider } from './contexts/MachineContext';
import { createWebSocketService, getWebSocketService } from './services/websocket';
import {
  MachineDefinition,
  Status,
  Configuration,
  RoutineStorage
} from './schemas/schemas';

export function App() {
  const [status, setStatus] = useState<Status | null>(null);
  const [config, setConfig] = useState<Configuration | null>(null);
  const [machineDefinition, setMachineDefinition] = useState<MachineDefinition | null>(null);
  const [routines, setRoutines] = useState<RoutineStorage | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [connected, setConnected] = useState(false);

  // JSON modal states
  const [showStatusJson, setShowStatusJson] = useState(false);
  const [showConfigJson, setShowConfigJson] = useState(false);
  const [showRoutinesJson, setShowRoutinesJson] = useState(false);
  const [showMachineDefJson, setShowMachineDefJson] = useState(false);

  useEffect(() => {
    // Determine WebSocket URL based on current location
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.hostname;
    const wsUrl = `${protocol}//${host}:8080`;

    // Create WebSocket service with callbacks
    const wsService = createWebSocketService(wsUrl, {
      onStatusUpdate: (newStatus) => {
        setStatus(newStatus);
        // Clear loading state once we get first status
        if (loading && machineDefinition) {
          setLoading(false);
        }
      },
      onConfigurationUpdate: (newConfig) => {
        setConfig(newConfig);
      },
      onMachineDefinition: (def) => {
        setMachineDefinition(def);
        // Clear loading state if we already have status
        if (loading && status) {
          setLoading(false);
        }
      },
      onRoutinesUpdate: (newRoutines) => {
        setRoutines(newRoutines);
      },
      onConnect: () => {
        setConnected(true);
        setError(null);
      },
      onDisconnect: () => {
        setConnected(false);
      },
      onError: (err) => {
        setError(err.message);
      },
      onCommandAck: (id, success, errorMsg) => {
        if (!success && errorMsg) {
          console.error(`Command ${id} failed: ${errorMsg}`);
        }
      }
    });

    // Connect to WebSocket
    wsService.connect();

    // Set a timeout to clear loading state if initial data doesn't arrive
    const loadingTimeout = setTimeout(() => {
      if (loading) {
        setLoading(false);
        if (!machineDefinition) {
          setError('Timeout waiting for machine definition');
        }
      }
    }, 10000);

    return () => {
      clearTimeout(loadingTimeout);
      wsService.disconnect();
    };
  }, []);

  // Separate effect to track when we have all initial data
  useEffect(() => {
    if (machineDefinition && status && config && loading) {
      setLoading(false);
    }
  }, [machineDefinition, status, config, loading]);

  if (loading) {
    return (
      <div style={{ padding: '2rem', textAlign: 'center' }}>
        <h1>Connecting...</h1>
        <p style={{ color: '#666' }}>Establishing WebSocket connection...</p>
      </div>
    );
  }

  if (error && !connected) {
    return (
      <div style={{ padding: '2rem', textAlign: 'center', color: 'red' }}>
        <h1>Connection Error</h1>
        <p>{error}</p>
        <button
          onClick={() => {
            setError(null);
            setLoading(true);
            const ws = getWebSocketService();
            if (ws) {
              ws.connect();
            }
          }}
          style={{
            marginTop: '1rem',
            padding: '0.5rem 1rem',
            cursor: 'pointer'
          }}
        >
          Retry Connection
        </button>
      </div>
    );
  }

  return (
    <MachineProvider machineDefinition={machineDefinition as MachineDefinition}>
      <div style={{ padding: '2rem', maxWidth: '1200px', margin: '0 auto' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '2rem' }}>
          <h1>{machineDefinition?.name || 'Espresso Machine'}</h1>
          <span style={{
            padding: '0.25rem 0.5rem',
            borderRadius: '4px',
            fontSize: '0.75rem',
            background: connected ? '#e6ffe6' : '#ffe6e6',
            color: connected ? '#006600' : '#660000'
          }}>
            {connected ? 'Connected' : 'Disconnected'}
          </span>
        </div>

        {/* Status Display */}
        <StatusDisplay status={status as Status} routines={routines as RoutineStorage} />

        {/* Configuration Panel */}
        <ConfigurationPanel configuration={config as Configuration} />

        {/* Schedules */}
        <section style={{ marginTop: '1.5rem', background: 'white', padding: '1.5rem', borderRadius: '8px', boxShadow: '0 2px 4px rgba(0,0,0,0.1)' }}>
          <ScheduleBuilder schedules={(config as Configuration)?.schedules || []} />
        </section>

        {/* Routines */}
        {routines && (
          <section style={{ marginTop: '1.5rem' }}>
            <RoutineBuilder
              routines={routines}
              machineDefinition={machineDefinition as MachineDefinition}
              onRefresh={() => {
                const ws = getWebSocketService();
                if (ws) {
                  ws.requestRoutines();
                }
              }}
            />
          </section>
        )}

        {/* Footer with JSON Modal Triggers */}
        <footer style={{ marginTop: '2rem', paddingTop: '1.5rem', borderTop: '1px solid #ddd', textAlign: 'center', color: '#666', fontSize: '0.875rem' }}>
          <p>
            <button
              onClick={() => setShowStatusJson(true)}
              style={{
                background: 'none',
                border: 'none',
                color: '#0066cc',
                cursor: 'pointer',
                textDecoration: 'underline',
                marginRight: '1rem',
                fontSize: '0.875rem'
              }}
            >
              Show Status JSON
            </button>
            <button
              onClick={() => setShowConfigJson(true)}
              style={{
                background: 'none',
                border: 'none',
                color: '#0066cc',
                cursor: 'pointer',
                textDecoration: 'underline',
                marginRight: '1rem',
                fontSize: '0.875rem'
              }}
            >
              Show Configuration JSON
            </button>
            <button
              onClick={() => setShowRoutinesJson(true)}
              style={{
                background: 'none',
                border: 'none',
                color: '#0066cc',
                cursor: 'pointer',
                textDecoration: 'underline',
                marginRight: '1rem',
                fontSize: '0.875rem'
              }}
            >
              Show Routines JSON
            </button>
            <button
              onClick={() => setShowMachineDefJson(true)}
              style={{
                background: 'none',
                border: 'none',
                color: '#0066cc',
                cursor: 'pointer',
                textDecoration: 'underline',
                fontSize: '0.875rem'
              }}
            >
              Show Machine Definition JSON
            </button>
          </p>
        </footer>

        {/* JSON Modals */}
        <JsonModal
          title="Status JSON"
          data={status}
          isOpen={showStatusJson}
          onClose={() => setShowStatusJson(false)}
        />
        <JsonModal
          title="Configuration JSON"
          data={config}
          isOpen={showConfigJson}
          onClose={() => setShowConfigJson(false)}
        />
        <JsonModal
          title="Routines JSON"
          data={routines}
          isOpen={showRoutinesJson}
          onClose={() => setShowRoutinesJson(false)}
        />
        <JsonModal
          title="Machine Definition JSON"
          data={machineDefinition}
          isOpen={showMachineDefJson}
          onClose={() => setShowMachineDefJson(false)}
        />
      </div>
    </MachineProvider>
  );
}
