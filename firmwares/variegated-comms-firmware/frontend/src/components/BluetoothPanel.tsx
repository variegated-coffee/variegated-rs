import { memo } from 'preact/compat';
import { useState } from 'preact/hooks';
import {
  Alert,
  Badge,
  Button,
  EmptyState,
  tokens,
  tokensFor,
  useDialogs,
  type Theme,
} from '@variegated-coffee/ui';
import {
  BluetoothPeripheralAssociation,
  BluetoothScanStatus,
  DiscoveredBluetoothPeripheral
} from '../schemas/schemas';
import { useMachine } from '../contexts/MachineContext';
import * as bluetoothApi from '../api/bluetooth';
import { formatAddress } from '../api/bluetooth';
import { BluetoothPeripheralEditor } from './BluetoothPeripheralEditor';

interface BluetoothPanelProps {
  /** From `configuration.bluetooth_peripherals`. */
  associations: BluetoothPeripheralAssociation[];
  /** From `status.bluetooth`. */
  scan: BluetoothScanStatus;
  /** From `status.comms_status.peripheral_connection_status`, keyed by peripheral id. */
  connected: Map<number, boolean>;
}

/** What the editor is currently working on, if anything. */
type EditorTarget =
  | { mode: 'new'; device: DiscoveredBluetoothPeripheral }
  | { mode: 'edit'; association: BluetoothPeripheralAssociation };

/*
 * These two were captures: built once at module-init from `tokens.color`, so a component
 * reading either got a copy and could not follow a palette however the palette was set.
 *
 * Now they are functions of a theme. Nothing in this frontend threads a scheme yet, so the
 * component below calls them with the light one and renders exactly as it did -- the point
 * of the change is that the value is read per render rather than once at import.
 */

/** One row in either list, so the two agree on their padding and border. */
const rowStyleFor = (t: Theme) => ({
  display: 'flex',
  alignItems: 'center',
  gap: t.space.sm,
  padding: t.space.sm,
  marginBottom: t.space.sm,
  border: `1px solid ${t.color.border}`,
  borderRadius: t.radius.sm,
  flexWrap: 'wrap' as const,
});

/** The MAC, which is read character by character when two devices share a name. */
const addressStyleFor = (t: Theme) => ({
  fontSize: '0.75rem',
  color: t.color.inkMuted,
  fontFamily: t.font.mono,
});

const BluetoothPanelComponent = ({ associations, scan, connected }: BluetoothPanelProps) => {
  const theme = tokensFor('light');
  const rowStyle = rowStyleFor(theme);
  const addressStyle = addressStyleFor(theme);
  const { getCommsPeripheralEntries } = useMachine();
  const { confirm } = useDialogs();
  const [editing, setEditing] = useState<EditorTarget | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [showAll, setShowAll] = useState(false);

  const peripheralOptions = getCommsPeripheralEntries();
  const takenIds = associations.map((a) => a.id);
  const associatedAddresses = new Set(associations.map((a) => formatAddress(a.address)));

  const run = (action: () => void) => {
    setError(null);
    try {
      action();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Command failed');
    }
  };

  const peripheralLabel = (id: number): string => {
    const entry = peripheralOptions.find(([entryId]) => entryId === id);
    const hex = `0x${id.toString(16).toUpperCase().padStart(4, '0')}`;
    return entry ? `${entry[1].location} (${hex})` : hex;
  };

  // Recognised devices first, then strongest signal. A user picking their scale wants
  // the thing the machine knows how to talk to at the top, and after that the one they
  // are standing next to.
  const discovered = [...scan.discovered].sort((a, b) => {
    const recognised = Number(b.suggested_driver !== null) - Number(a.suggested_driver !== null);
    return recognised !== 0 ? recognised : b.rssi - a.rssi;
  });

  // Hidden by default, not filtered away. A device that advertises no service UUID is
  // still perfectly usable -- the service is discovered on connect -- so hiding these
  // permanently would mean a scale the machine supports being impossible to add. The
  // count is shown so it is obvious there is more behind the toggle.
  const recognised = discovered.filter((d) => d.suggested_driver !== null);
  const unrecognised = discovered.filter((d) => d.suggested_driver === null);
  const visible = showAll ? discovered : recognised;

  if (editing) {
    return (
      <div>
        <h2 style={{ marginTop: 0, marginBottom: tokens.space.md }}>Bluetooth</h2>
        <BluetoothPeripheralEditor
          address={editing.mode === 'new' ? editing.device.address : editing.association.address}
          addressRandom={
            editing.mode === 'new' ? editing.device.address_random : editing.association.address_random
          }
          peripheralOptions={peripheralOptions}
          takenIds={takenIds}
          existing={editing.mode === 'edit' ? editing.association : null}
          suggestedName={editing.mode === 'new' ? editing.device.name : ''}
          suggestedDriver={
            editing.mode === 'new' ? editing.device.suggested_driver?.type ?? null : null
          }
          onSave={(association) => {
            run(() => bluetoothApi.associatePeripheral(association));
            setEditing(null);
          }}
          onCancel={() => setEditing(null)}
        />
      </div>
    );
  }

  return (
    <div>
      <h2 style={{ marginTop: 0, marginBottom: '1rem' }}>Bluetooth</h2>

      {error && (
        <div style={{ marginBottom: tokens.space.md }}>
          <Alert role="danger" onDismiss={() => setError(null)}>
            {error}
          </Alert>
        </div>
      )}

      <h3 style={{ fontSize: '1rem', marginBottom: tokens.space.sm }}>Associated peripherals</h3>

      {associations.length === 0 ? (
        <div style={{ marginBottom: tokens.space.xl }}>
          <EmptyState
            title="No Bluetooth peripherals associated"
            detail="Scan below to find a scale or another supported device."
          />
        </div>
      ) : (
        <div style={{ marginBottom: tokens.space.xl }}>
          {associations.map((association) => {
            const isConnected = connected.get(association.id) === true;
            return (
              <div
                key={association.id}
                style={{
                  ...rowStyle,
                  // Dimmed rather than hidden: a disabled peripheral is still configured,
                  // and the point of disabling is that you can find it again.
                  opacity: association.enabled ? 1 : 0.6,
                }}
              >
                <div style={{ flex: 1, minWidth: '12rem' }}>
                  <div style={{ fontWeight: 500 }}>{association.name || '(unnamed)'}</div>
                  <div style={{ fontSize: '0.8rem', color: tokens.color.inkMuted }}>
                    {peripheralLabel(association.id)} · {association.driver.type}
                  </div>
                  <div style={addressStyle}>{formatAddress(association.address)}</div>
                </div>

                {association.enabled ? (
                  <Badge role={isConnected ? 'ok' : undefined}>
                    {isConnected ? 'Connected' : 'Offline'}
                  </Badge>
                ) : (
                  <Badge>Disabled</Badge>
                )}

                <Button
                  variant="secondary"
                  size="sm"
                  onClick={() =>
                    run(() =>
                      bluetoothApi.setPeripheralEnabled(association.id, !association.enabled)
                    )
                  }
                >
                  {association.enabled ? 'Disable' : 'Enable'}
                </Button>
                <Button variant="secondary" size="sm" onClick={() => setEditing({ mode: 'edit', association })}>
                  Edit
                </Button>
                {/* "Forget", not "Delete" -- the peripheral is not going anywhere, the
                    machine just stops trying to connect to it. Naming it is what lets
                    someone notice they picked the wrong row. */}
                <Button
                  variant="destructive"
                  size="sm"
                  onClick={() => {
                    void confirm({
                      title: `Forget ${association.name || formatAddress(association.address)}?`,
                      body: 'The machine stops connecting to it. You can associate it again by scanning.',
                      confirmLabel: 'Forget',
                      destructive: true,
                    }).then((ok) => {
                      if (ok) run(() => bluetoothApi.removePeripheral(association.id));
                    });
                  }}
                >
                  Forget
                </Button>
              </div>
            );
          })}
        </div>
      )}

      <h3 style={{ fontSize: '1rem', marginBottom: tokens.space.sm }}>Nearby devices</h3>

      <div style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm, marginBottom: tokens.space.md, flexWrap: 'wrap' }}>
        <Button
          variant="primary"
          onClick={() => run(() => bluetoothApi.scanForPeripherals())}
          disabled={scan.scanning}
        >
          {scan.scanning ? 'Scanning…' : 'Scan for devices'}
        </Button>
        {scan.reports_dropped > 0 && (
          <span style={{ fontSize: '0.8rem', color: tokens.color.warnInk }}>
            {scan.reports_dropped} result{scan.reports_dropped === 1 ? '' : 's'} dropped — too many
            devices nearby
          </span>
        )}
      </div>

      {/*
        `blocked` is the machine refusing, not an error. It is the only feedback the user
        gets, because the command is fire-and-forget like everything else in this UI.
      */}
      {scan.blocked && !scan.scanning && (
        <div style={{ marginBottom: tokens.space.md }}>
          <Alert role="warn">
            The machine is busy. Scanning interrupts the radio, which can drop a connected
            scale mid-shot — try again once brewing has finished.
          </Alert>
        </div>
      )}

      {unrecognised.length > 0 && (
        <label
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: tokens.space.sm,
            marginBottom: tokens.space.sm,
            fontSize: '0.85rem',
            color: tokens.color.inkMuted,
            cursor: 'pointer',
          }}
        >
          <input
            type="checkbox"
            checked={showAll}
            onChange={(e) => setShowAll((e.target as HTMLInputElement).checked)}
          />
          Show {unrecognised.length} device{unrecognised.length === 1 ? '' : 's'} that did not
          advertise a supported service
        </label>
      )}

      {visible.length === 0 ? (
        <EmptyState
          title={
            scan.scanning
              ? 'Looking for devices…'
              : unrecognised.length > 0
                ? 'No recognised devices found'
                : 'No devices found yet'
          }
          detail={
            !scan.scanning && unrecognised.length > 0
              ? 'Some scales do not advertise their service — tick the box above to see the rest.'
              : undefined
          }
        />
      ) : (
        <div>
          {visible.map((device) => {
            const address = formatAddress(device.address);
            const already = associatedAddresses.has(address);
            return (
              <div key={address} style={rowStyle}>
                <div style={{ flex: 1, minWidth: '12rem' }}>
                  <div style={{ fontWeight: 500 }}>{device.name || '(no name advertised)'}</div>
                  <div style={addressStyle}>{address}</div>
                </div>
                {device.suggested_driver && <Badge role="info">{device.suggested_driver.type}</Badge>}
                <span
                  style={{
                    fontSize: '0.8rem',
                    color: tokens.color.inkMuted,
                    fontFamily: tokens.font.mono,
                    fontVariantNumeric: 'tabular-nums',
                  }}
                >
                  {device.rssi} dBm
                </span>
                {already ? (
                  <Badge role="ok">Associated</Badge>
                ) : (
                  <Button variant="primary" size="sm" onClick={() => setEditing({ mode: 'new', device })}>
                    Associate
                  </Button>
                )}
              </div>
            );
          })}
        </div>
      )}
    </div>
  );
};

export const BluetoothPanel = memo(BluetoothPanelComponent);
