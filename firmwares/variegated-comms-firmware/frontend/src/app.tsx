import { useEffect, useRef, useState } from 'preact/hooks';
import { ScheduleBuilder } from './components/ScheduleBuilder';
import { BluetoothPanel } from './components/BluetoothPanel';
import { ShotUploadPanel } from './components/ShotUploadPanel';
import { ShotLogPanel } from './components/ShotLogPanel';
import { RoutineBuilder } from './components/routine/RoutineBuilder';
import { StatusDisplay } from './components/StatusDisplay';
import { ConfigurationPanel } from './components/ConfigurationPanel';
import { JsonModal } from './components/JsonModal';
import { MachineProvider } from './contexts/MachineContext';
import { createWebSocketService, getWebSocketService } from './services/websocket';
import { syncRoutineBodies } from './state/routineBodies';
import { publishShotLogEvent } from './state/shotLogEvents';
import {
  MachineDefinition,
  Status,
  Configuration,
  RoutineSummaryStorage
} from './schemas/schemas';

export function App() {
  const [status, setStatus] = useState<Status | null>(null);
  const [config, setConfig] = useState<Configuration | null>(null);
  const [machineDefinition, setMachineDefinition] = useState<MachineDefinition | null>(null);
  const [routines, setRoutines] = useState<RoutineSummaryStorage | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [connected, setConnected] = useState(false);

  // Whether a machine definition has arrived, readable from inside the mount effect.
  //
  // A ref rather than reading `machineDefinition`, because the effect below runs once with
  // `[]` deps and everything it closes over is frozen at the first render -- where that
  // state is `null`. The give-up timeout needs to know what is true when it *fires*, ten
  // seconds later, and state cannot tell it. Adding the state to the dep array is not the
  // alternative: it would tear down and recreate the WebSocket connection on every update.
  const machineDefinitionArrived = useRef(false);

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
      // Neither of these clears `loading` any more, and neither ever did.
      //
      // They used to read `if (loading && machineDefinition)` and `if (loading && status)`,
      // which could not fire: this effect has `[]` deps, so both callbacks closed over the
      // first render's values -- `loading` permanently `true`, `status` and
      // `machineDefinition` permanently `null` -- and the conjunctions were therefore
      // permanently false. The dedicated effect below, which has real dependencies, is what
      // has actually been clearing `loading`.
      onStatusUpdate: (newStatus) => {
        setStatus(newStatus);
      },
      onConfigurationUpdate: (newConfig) => {
        setConfig(newConfig);
      },
      onMachineDefinition: (def) => {
        machineDefinitionArrived.current = true;
        setMachineDefinition(def);
      },
      onRoutinesUpdate: (newRoutines) => {
        setRoutines(newRoutines);
        // Start walking the definitions in the background. A no-op when the list is
        // unchanged, which is the usual case -- see `syncRoutineBodies`.
        syncRoutineBodies(newRoutines);
      },
      // Straight into the module store rather than into component state. The panel is the
      // only consumer and it wants a stream, not a value -- see `state/shotLogEvents`.
      onShotLogEvent: (event) => {
        publishShotLogEvent(event);
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

    // Give up waiting for the initial data and show the app anyway.
    //
    // The condition reads the ref, not the state, and that is the whole point: this closure
    // was written as `if (!machineDefinition)` against a value frozen at `null` on the first
    // render, so it was **always** true and this **always** set "Timeout waiting for machine
    // definition" -- ten seconds into every session, including perfectly healthy ones.
    //
    // It was invisible while connected, because the error screen is gated on
    // `error && !connected`. It surfaced later: the first time the socket dropped, the
    // operator was told the machine definition had timed out rather than that the connection
    // had gone.
    //
    // `setLoading(false)` is now unconditional. It is a no-op if the effect below already
    // cleared it, and guarding it on a stale `loading` bought nothing.
    const loadingTimeout = setTimeout(() => {
      if (!machineDefinitionArrived.current) {
        setError('Timeout waiting for machine definition');
      }
      setLoading(false);
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
        <StatusDisplay status={status as Status} routines={routines as RoutineSummaryStorage} />

        {/* Configuration Panel */}
        <ConfigurationPanel configuration={config as Configuration} />

        {/* Shot upload */}
        <section style={{ marginTop: '1.5rem', background: 'white', padding: '1.5rem', borderRadius: '8px', boxShadow: '0 2px 4px rgba(0,0,0,0.1)' }}>
          <ShotUploadPanel shotUpload={(config as Configuration)?.shot_upload} />
        </section>

        {/* Bluetooth peripherals */}
        <section style={{ marginTop: '1.5rem', background: 'white', padding: '1.5rem', borderRadius: '8px', boxShadow: '0 2px 4px rgba(0,0,0,0.1)' }}>
          <BluetoothPanel
            associations={(config as Configuration)?.bluetooth_peripherals || []}
            scan={(status as Status).bluetooth}
            // The connection map is keyed by peripheral id and only exists once the comms
            // processor has reported in, so an absent entry reads as "not connected"
            // rather than as an error.
            connected={
              new Map(
                Array.from((status as Status).comms_status?.peripheral_connection_status ?? []).map(
                  ([id, wireless]) => [id, wireless.connected]
                )
              )
            }
          />
        </section>

        {/* Shot log */}
        <section style={{ marginTop: '1.5rem', background: 'white', padding: '1.5rem', borderRadius: '8px', boxShadow: '0 2px 4px rgba(0,0,0,0.1)' }}>
          <ShotLogPanel
            pending={(status as Status).pending_shot_annotations}
            sdCardPresent={(status as Status).sd_card_present}
            // The groups the machine actually has, rather than a hardcoded `[0]`: the
            // dose buttons address a group scale by index, and a two-group machine needs
            // two buttons.
            groupIndices={Array.from((status as Status).group_statuses.keys())}
          />
        </section>

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
              peripheralStatus={(status as Status)?.peripheral_status ?? null}
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
