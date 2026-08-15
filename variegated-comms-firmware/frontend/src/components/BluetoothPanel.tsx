import { memo } from 'preact/compat';
import { useState } from 'preact/hooks';
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

const pillStyle = {
  padding: '0.25rem 0.75rem',
  borderRadius: '12px',
  fontSize: '0.75rem',
  color: 'white'
};

const buttonStyle = {
  padding: '0.4rem 0.75rem',
  border: 'none',
  borderRadius: '4px',
  fontSize: '0.85rem',
  cursor: 'pointer',
  color: 'white'
};

const BluetoothPanelComponent = ({ associations, scan, connected }: BluetoothPanelProps) => {
  const { getCommsPeripheralEntries } = useMachine();
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
        <h2 style={{ marginTop: 0, marginBottom: '1rem' }}>Bluetooth</h2>
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
        <div
          style={{
            padding: '0.75rem',
            marginBottom: '1rem',
            backgroundColor: '#f8d7da',
            color: '#721c24',
            borderRadius: '4px',
            fontSize: '0.9rem',
            display: 'flex',
            justifyContent: 'space-between'
          }}
        >
          <span>{error}</span>
          <span style={{ cursor: 'pointer' }} onClick={() => setError(null)}>
            ×
          </span>
        </div>
      )}

      <h3 style={{ fontSize: '1rem', marginBottom: '0.75rem' }}>Associated peripherals</h3>

      {associations.length === 0 ? (
        <div
          style={{
            padding: '1.5rem',
            border: '1px dashed #ccc',
            borderRadius: '8px',
            textAlign: 'center',
            color: '#666',
            fontSize: '0.9rem',
            marginBottom: '2rem'
          }}
        >
          No Bluetooth peripherals associated. Scan below to find one.
        </div>
      ) : (
        <div style={{ marginBottom: '2rem' }}>
          {associations.map((association) => {
            const isConnected = connected.get(association.id) === true;
            return (
              <div
                key={association.id}
                style={{
                  display: 'flex',
                  alignItems: 'center',
                  gap: '0.75rem',
                  padding: '0.75rem',
                  marginBottom: '0.5rem',
                  border: '1px solid #eee',
                  borderRadius: '6px',
                  // Dimmed rather than hidden: a disabled peripheral is still configured,
                  // and the point of disabling is that you can find it again.
                  opacity: association.enabled ? 1 : 0.6
                }}
              >
                <div style={{ flex: 1, minWidth: 0 }}>
                  <div style={{ fontWeight: 500 }}>{association.name || '(unnamed)'}</div>
                  <div style={{ fontSize: '0.8rem', color: '#666' }}>
                    {peripheralLabel(association.id)} · {association.driver.type}
                  </div>
                  <div style={{ fontSize: '0.75rem', color: '#999', fontFamily: 'monospace' }}>
                    {formatAddress(association.address)}
                  </div>
                </div>

                {association.enabled && (
                  <span style={{ ...pillStyle, backgroundColor: isConnected ? '#28a745' : '#6c757d' }}>
                    {isConnected ? 'CONNECTED' : 'OFFLINE'}
                  </span>
                )}
                {!association.enabled && (
                  <span style={{ ...pillStyle, backgroundColor: '#6c757d' }}>DISABLED</span>
                )}

                <button
                  onClick={() =>
                    run(() =>
                      bluetoothApi.setPeripheralEnabled(association.id, !association.enabled)
                    )
                  }
                  style={{ ...buttonStyle, backgroundColor: '#007bff' }}
                  title={association.enabled ? 'Stop connecting' : 'Start connecting'}
                >
                  {association.enabled ? 'Disable' : 'Enable'}
                </button>
                <button
                  onClick={() => setEditing({ mode: 'edit', association })}
                  style={{ ...buttonStyle, backgroundColor: '#6c757d' }}
                >
                  Edit
                </button>
                <button
                  onClick={() => {
                    if (!confirm(`Forget ${association.name || formatAddress(association.address)}?`)) {
                      return;
                    }
                    run(() => bluetoothApi.removePeripheral(association.id));
                  }}
                  style={{ ...buttonStyle, backgroundColor: '#dc3545' }}
                >
                  Delete
                </button>
              </div>
            );
          })}
        </div>
      )}

      <h3 style={{ fontSize: '1rem', marginBottom: '0.75rem' }}>Nearby devices</h3>

      <div style={{ display: 'flex', alignItems: 'center', gap: '0.75rem', marginBottom: '1rem' }}>
        <button
          onClick={() => run(() => bluetoothApi.scanForPeripherals())}
          disabled={scan.scanning}
          style={{
            ...buttonStyle,
            padding: '0.6rem 1rem',
            fontSize: '0.95rem',
            backgroundColor: scan.scanning ? '#6c757d' : '#007bff',
            cursor: scan.scanning ? 'default' : 'pointer'
          }}
        >
          {scan.scanning ? 'Scanning…' : 'Scan for devices'}
        </button>
        {scan.reports_dropped > 0 && (
          <span style={{ fontSize: '0.8rem', color: '#856404' }}>
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
        <div
          style={{
            padding: '0.75rem',
            marginBottom: '1rem',
            backgroundColor: '#fff3cd',
            color: '#856404',
            borderRadius: '4px',
            fontSize: '0.9rem'
          }}
        >
          The machine is busy. Scanning interrupts the radio, which can drop a connected
          scale mid-shot — try again once brewing has finished.
        </div>
      )}

      {unrecognised.length > 0 && (
        <label
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: '0.5rem',
            marginBottom: '0.75rem',
            fontSize: '0.85rem',
            color: '#666'
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
        <div
          style={{
            padding: '1.5rem',
            border: '1px dashed #ccc',
            borderRadius: '8px',
            textAlign: 'center',
            color: '#666',
            fontSize: '0.9rem'
          }}
        >
          {scan.scanning
            ? 'Looking for devices…'
            : unrecognised.length > 0
              ? 'No recognised devices found. Some scales do not advertise their service — tick the box above to see the rest.'
              : 'No devices found yet.'}
        </div>
      ) : (
        <div>
          {visible.map((device) => {
            const address = formatAddress(device.address);
            const already = associatedAddresses.has(address);
            return (
              <div
                key={address}
                style={{
                  display: 'flex',
                  alignItems: 'center',
                  gap: '0.75rem',
                  padding: '0.75rem',
                  marginBottom: '0.5rem',
                  border: '1px solid #eee',
                  borderRadius: '6px'
                }}
              >
                <div style={{ flex: 1, minWidth: 0 }}>
                  <div style={{ fontWeight: 500 }}>{device.name || '(no name advertised)'}</div>
                  <div style={{ fontSize: '0.75rem', color: '#999', fontFamily: 'monospace' }}>
                    {address}
                  </div>
                </div>
                {device.suggested_driver && (
                  <span style={{ ...pillStyle, backgroundColor: '#007bff' }}>
                    {device.suggested_driver.type}
                  </span>
                )}
                <span style={{ fontSize: '0.8rem', color: '#666' }}>{device.rssi} dBm</span>
                {already ? (
                  <span style={{ ...pillStyle, backgroundColor: '#28a745' }}>ASSOCIATED</span>
                ) : (
                  <button
                    onClick={() => setEditing({ mode: 'new', device })}
                    style={{ ...buttonStyle, backgroundColor: '#28a745' }}
                  >
                    Associate →
                  </button>
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
