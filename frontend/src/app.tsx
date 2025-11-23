import { useEffect, useState } from 'preact/hooks';
import { ScheduleBuilder } from './components/ScheduleBuilder';
import { RoutineBuilder } from './components/routine/RoutineBuilder';
import { StatusDisplay } from './components/StatusDisplay';
import { ConfigurationPanel } from './components/ConfigurationPanel';
import { JsonModal } from './components/JsonModal';
import { MachineProvider } from './contexts/MachineContext';
import { fetchPostcard } from './utils/postcard';
import {
  MachineDefinition,
  MachineDefinitionSchema,
  Status,
  StatusSchema,
  Configuration,
  ConfigurationSchema,
  RoutineStorage,
  RoutineStorageSchema
} from './schemas/schemas';

export function App() {
  const [status, setStatus] = useState<Status | null>(null);
  const [config, setConfig] = useState<Configuration | null>(null);
  const [machineDefinition, setMachineDefinition] = useState<MachineDefinition | null>(null);
  const [routines, setRoutines] = useState<RoutineStorage | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // JSON modal states
  const [showStatusJson, setShowStatusJson] = useState(false);
  const [showConfigJson, setShowConfigJson] = useState(false);
  const [showRoutinesJson, setShowRoutinesJson] = useState(false);
  const [showMachineDefJson, setShowMachineDefJson] = useState(false);

  useEffect(() => {
    // Track in-flight requests to prevent overlapping
    let statusInFlight = false;
    let configInFlight = false;
    let routinesInFlight = false;

    // Fetch data sequentially on initial load
    const loadInitialData = async () => {
      try {
        const machineDefData = await fetchPostcard('/machine-definition', MachineDefinitionSchema);
        setMachineDefinition(machineDefData);

        const statusData = await fetchPostcard('/status', StatusSchema);
        setStatus(statusData);

        const configData = await fetchPostcard('/configuration', ConfigurationSchema);
        setConfig(configData);

        const routinesData = await fetchPostcard('/routines', RoutineStorageSchema);
        setRoutines(routinesData);

        setLoading(false);
      } catch (err) {
        if (err instanceof Error) {
          setError(err.message);
        } else {
          setError('An unknown error occurred');
        }
        setLoading(false);
      }
    };

    void loadInitialData();

    // Poll status every second (only if not already in-flight)
    const statusInterval = setInterval(() => {
      if (statusInFlight) {
        console.log('Skipping status fetch - request already in flight');
        return;
      }

      statusInFlight = true;
      fetchPostcard('/status', StatusSchema)
        .then(data => setStatus(data))
        .catch(console.error)
        .finally(() => { statusInFlight = false; });
    }, 1000);

    // Poll configuration every second (only if not already in-flight)
    const configInterval = setInterval(() => {
      if (configInFlight) {
        console.log('Skipping configuration fetch - request already in flight');
        return;
      }

      configInFlight = true;
      fetchPostcard('/configuration', ConfigurationSchema)
        .then(data => setConfig(data))
        .catch(console.error)
        .finally(() => { configInFlight = false; });
    }, 1000);

    // Start configuration polling 300ms after status
    setTimeout(() => {
      if (!configInFlight) {
        configInFlight = true;
        fetchPostcard('/configuration', ConfigurationSchema)
          .then(data => setConfig(data))
          .catch(console.error)
          .finally(() => { configInFlight = false; });
      }
    }, 300);

    // Poll routines every 20 seconds (only if not already in-flight)
    const routinesInterval = setInterval(() => {
      if (routinesInFlight) {
        console.log('Skipping routines fetch - request already in flight');
        return;
      }

      routinesInFlight = true;
      fetchPostcard('/routines', RoutineStorageSchema)
        .then(data => setRoutines(data))
        .catch(console.error)
        .finally(() => { routinesInFlight = false; });
    }, 20000);

    return () => {
      clearInterval(statusInterval);
      clearInterval(configInterval);
      clearInterval(routinesInterval);
    };
  }, []);

  if (loading) {
    return (
      <div style={{ padding: '2rem', textAlign: 'center' }}>
        <h1>Loading...</h1>
      </div>
    );
  }

  if (error) {
    return (
      <div style={{ padding: '2rem', textAlign: 'center', color: 'red' }}>
        <h1>Error: {error}</h1>
      </div>
    );
  }

  return (
    <MachineProvider machineDefinition={machineDefinition as MachineDefinition}>
      <div style={{ padding: '2rem', maxWidth: '1200px', margin: '0 auto' }}>
        <h1 style={{ marginBottom: '2rem' }}>{machineDefinition?.name || 'Espresso Machine'}</h1>

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
                fetchPostcard('/routines', RoutineStorageSchema)
                  .then(data => setRoutines(data))
                  .catch(console.error);
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
