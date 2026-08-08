import { useState } from 'preact/hooks';
import { BluetoothDriverKind, BluetoothPeripheralAssociation, PeripheralEntry } from '../schemas/schemas';
import { formatAddress, BluetoothAddress } from '../api/bluetooth';

/** Longest name the wire type carries. Matches `BLUETOOTH_NAME_LEN`. */
const NAME_MAX_BYTES = 24;

const DRIVERS: { kind: BluetoothDriverKind['type']; label: string; hint: string }[] = [
  { kind: 'AcaiaOld', label: 'ACAIA (older protocol)', hint: 'Lunar, Pearl and similar' },
  { kind: 'BelkaPortal', label: 'Belka Portal', hint: 'Water sensor: conductivity, temperature' }
];

interface BluetoothPeripheralEditorProps {
  /** Fixed once chosen: an association is identified by its address. */
  address: BluetoothAddress;
  addressRandom: boolean;
  /** Roles this machine has, from the machine definition. */
  peripheralOptions: PeripheralEntry[];
  /** Ids that already have an association, so the form can warn about replacing one. */
  takenIds: number[];
  existing: BluetoothPeripheralAssociation | null;
  suggestedName: string;
  onSave: (association: BluetoothPeripheralAssociation) => void;
  onCancel: () => void;
}

const labelStyle = { display: 'block', marginBottom: '0.35rem', fontSize: '0.9rem', fontWeight: 500 };
const fieldStyle = {
  width: '100%',
  padding: '0.5rem',
  border: '1px solid #ccc',
  borderRadius: '4px',
  fontSize: '0.95rem',
  boxSizing: 'border-box' as const
};
const hintStyle = { fontSize: '0.75rem', color: '#666', marginTop: '0.25rem' };
const blockStyle = { marginBottom: '1.25rem' };

const BluetoothPeripheralEditorComponent = ({
  address,
  addressRandom,
  peripheralOptions,
  takenIds,
  existing,
  suggestedName,
  onSave,
  onCancel
}: BluetoothPeripheralEditorProps) => {
  const [peripheralId, setPeripheralId] = useState<string>(
    existing ? String(existing.id) : peripheralOptions.length > 0 ? String(peripheralOptions[0][0]) : ''
  );
  const [driver, setDriver] = useState<BluetoothDriverKind['type']>(
    existing ? existing.driver.type : 'AcaiaOld'
  );
  const [name, setName] = useState<string>(existing ? existing.name : suggestedName);
  const [enabled, setEnabled] = useState<boolean>(existing ? existing.enabled : true);
  const [error, setError] = useState<string | null>(null);

  const selectedId = parseInt(peripheralId, 10);

  // Warned about rather than blocked, because replacing is a legitimate thing to want —
  // it is how you move a role from an old scale to a new one. What matters is that it is
  // not a surprise.
  const replacing =
    !isNaN(selectedId) && takenIds.includes(selectedId) && (!existing || existing.id !== selectedId);

  const handleSave = () => {
    setError(null);

    if (isNaN(selectedId)) {
      setError('Choose which peripheral this device is');
      return;
    }

    const trimmed = name.trim();
    // Bytes, not characters: the wire type is a fixed-capacity byte buffer, so a name of
    // 24 accented characters is twice too long even though it looks short.
    if (new TextEncoder().encode(trimmed).length > NAME_MAX_BYTES) {
      setError(`Name is too long (limit is ${NAME_MAX_BYTES} bytes)`);
      return;
    }

    onSave({
      id: selectedId,
      address,
      address_random: addressRandom,
      driver: { type: driver },
      enabled,
      name: trimmed
    });
  };

  return (
    <div
      style={{
        padding: '1.5rem',
        backgroundColor: '#f8f9fa',
        borderRadius: '8px',
        border: '1px solid #ddd'
      }}
    >
      <h3 style={{ marginTop: 0, marginBottom: '0.35rem', fontSize: '1.2rem' }}>
        {existing ? 'Edit peripheral' : 'Associate peripheral'}
      </h3>
      <div style={{ ...hintStyle, marginTop: 0, marginBottom: '1.5rem', fontFamily: 'monospace' }}>
        {formatAddress(address)}
        {addressRandom ? ' (random address)' : ' (public address)'}
      </div>

      {error && (
        <div
          style={{
            padding: '0.75rem',
            marginBottom: '1rem',
            backgroundColor: '#f8d7da',
            color: '#721c24',
            borderRadius: '4px',
            fontSize: '0.9rem'
          }}
        >
          {error}
        </div>
      )}

      <div style={blockStyle}>
        <label style={labelStyle}>Peripheral</label>
        <select
          value={peripheralId}
          onChange={(e) => setPeripheralId((e.target as HTMLSelectElement).value)}
          style={fieldStyle}
        >
          {peripheralOptions.length === 0 && <option value="">No peripherals defined</option>}
          {peripheralOptions.map(([id, definition]) => (
            <option key={id} value={String(id)}>
              {definition.location} — {definition.peripheral_type.type} (0x
              {id.toString(16).toUpperCase().padStart(4, '0')})
            </option>
          ))}
        </select>
        <div style={hintStyle}>
          Which role this device fills. Readings arrive at the machine under this
          peripheral, whichever device is associated with it.
        </div>
        {replacing && (
          <div style={{ ...hintStyle, color: '#856404' }}>
            This peripheral already has a device associated. Saving will replace it.
          </div>
        )}
      </div>

      <div style={blockStyle}>
        <label style={labelStyle}>Driver</label>
        <select
          value={driver}
          onChange={(e) => setDriver((e.target as HTMLSelectElement).value as BluetoothDriverKind['type'])}
          style={fieldStyle}
        >
          {DRIVERS.map((d) => (
            <option key={d.kind} value={d.kind}>
              {d.label}
            </option>
          ))}
        </select>
        <div style={hintStyle}>{DRIVERS.find((d) => d.kind === driver)?.hint}</div>
      </div>

      <div style={blockStyle}>
        <label style={labelStyle}>Name</label>
        <input
          type="text"
          value={name}
          onInput={(e) => setName((e.target as HTMLInputElement).value)}
          style={fieldStyle}
          placeholder="Group 1 scale"
        />
        <div style={hintStyle}>A label for you. Changing it does not disturb the connection.</div>
      </div>

      <div style={blockStyle}>
        <label style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', fontSize: '0.9rem', fontWeight: 500 }}>
          <input
            type="checkbox"
            checked={enabled}
            onChange={(e) => setEnabled((e.target as HTMLInputElement).checked)}
          />
          Connect to this device
        </label>
        <div style={hintStyle}>
          Turn this off to stop connecting without forgetting the device, so you do not
          have to scan for it again later.
        </div>
      </div>

      <div style={{ display: 'flex', gap: '0.75rem' }}>
        <button
          onClick={handleSave}
          style={{
            flex: 1,
            padding: '0.75rem',
            backgroundColor: '#28a745',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            fontSize: '1rem',
            cursor: 'pointer'
          }}
        >
          Save
        </button>
        <button
          onClick={onCancel}
          style={{
            flex: 1,
            padding: '0.75rem',
            backgroundColor: '#6c757d',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            fontSize: '1rem',
            cursor: 'pointer'
          }}
        >
          Cancel
        </button>
      </div>
    </div>
  );
};

export const BluetoothPeripheralEditor = BluetoothPeripheralEditorComponent;
