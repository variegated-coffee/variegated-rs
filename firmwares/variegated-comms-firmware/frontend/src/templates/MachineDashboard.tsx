import { Alert, Badge, tokens } from '@variegated-coffee/ui';
import { StatusDisplay } from '../components/StatusDisplay';
import { ConfigurationPanel } from '../components/ConfigurationPanel';
import { MachineDefinition, Status, Configuration, RoutineSummaryStorage } from '../schemas/schemas';

/**
 * The machine's main screen, assembled.
 *
 * # Why a template exists at all
 *
 * The audit's last finding: 62 components and zero templates. Nothing in the library said
 * what a machine dashboard is *supposed to look like put together*, so every screen was
 * assembled from scratch — which is how three button styles ended up in one card.
 *
 * These are not new screens. They are the composition `app.tsx` was already doing, lifted
 * out so it can be seen, reviewed and rendered as a card in the design system rather than
 * living only inside the app's root component. `app.tsx` renders these.
 *
 * # The order is the argument
 *
 * Connection state first, because it is the precondition for every control below it and
 * the panels grey themselves out when it is absent. Then live status, then configuration.
 * A machine owner opening this page is almost always answering "is it ready?", and that is
 * the first thing on it.
 */
export interface MachineDashboardProps {
  machineDefinition: MachineDefinition | null;
  status: Status;
  configuration: Configuration;
  routines: RoutineSummaryStorage;
  connected: boolean;
  /** Offered by the disconnected banner. */
  onReconnect: () => void;
}

export function MachineDashboard({
  machineDefinition,
  status,
  configuration,
  routines,
  connected,
  onReconnect,
}: MachineDashboardProps) {
  return (
    <>
      <div
        style={{
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
          marginBottom: tokens.space.xl,
          gap: tokens.space.md,
        }}
      >
        <h1 style={{ margin: 0 }}>{machineDefinition?.name || 'Espresso machine'}</h1>
        <Badge role={connected ? 'ok' : 'danger'}>
          {connected ? 'Connected' : 'Disconnected'}
        </Badge>
      </div>

      {/* The connection is the precondition for every control below, so it is stated once
          here rather than discovered one failed button at a time. The panels are also
          told, so they can disable what cannot work -- a control that looks available and
          answers "Not connected to the machine" after you press it is the failure this
          replaces. */}
      {!connected && (
        <div style={{ marginBottom: tokens.space.md }}>
          <Alert
            role="danger"
            title="Not connected to the machine"
            action={{ label: 'Retry', onClick: onReconnect }}
          >
            Readings are the last ones received. Controls that need the machine are
            unavailable until the connection is back.
          </Alert>
        </div>
      )}

      <StatusDisplay status={status} routines={routines} />

      <ConfigurationPanel configuration={configuration} />
    </>
  );
}
