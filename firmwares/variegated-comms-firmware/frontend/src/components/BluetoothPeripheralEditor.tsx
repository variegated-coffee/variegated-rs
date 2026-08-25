import { useState } from 'preact/hooks';
import { Alert, Button, Field, Select, TextInput, tokens } from '@variegated-coffee/ui';
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
  /** Driver inferred from the advertised service UUIDs, if the machine recognised one. */
  suggestedDriver: BluetoothDriverKind['type'] | null;
  onSave: (association: BluetoothPeripheralAssociation) => void;
  onCancel: () => void;
}

/** A note under a field that is not the field's own help text -- a live warning. */
const hintStyle = { fontSize: '0.75rem', color: tokens.color.inkMuted, marginTop: tokens.space.xs };

const BluetoothPeripheralEditorComponent = ({
  address,
  addressRandom,
  peripheralOptions,
  takenIds,
  existing,
  suggestedName,
  suggestedDriver,
  onSave,
  onCancel
}: BluetoothPeripheralEditorProps) => {
  const [peripheralId, setPeripheralId] = useState<string>(
    existing ? String(existing.id) : peripheralOptions.length > 0 ? String(peripheralOptions[0][0]) : ''
  );
  // An existing association's own driver wins, then whatever the device advertised, and
  // only then a default. The advertised one is right often enough to be worth
  // pre-selecting and is shown as a hint below, so a wrong guess is visible rather than
  // silent.
  const [driver, setDriver] = useState<BluetoothDriverKind['type']>(
    existing ? existing.driver.type : suggestedDriver ?? 'AcaiaOld'
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
        display: 'flex',
        flexDirection: 'column',
        gap: tokens.space.lg,
        padding: tokens.space.lg,
        backgroundColor: tokens.color.surfaceSunken,
        borderRadius: tokens.radius.md,
        border: `1px solid ${tokens.color.border}`,
      }}
    >
      <div>
        <h3 style={{ marginTop: 0, marginBottom: tokens.space.xs, fontSize: '1.2rem' }}>
          {existing ? 'Edit peripheral' : 'Associate peripheral'}
        </h3>
        <div style={{ ...hintStyle, marginTop: 0, fontFamily: tokens.font.mono }}>
          {formatAddress(address)}
          {addressRandom ? ' (random address)' : ' (public address)'}
        </div>
      </div>

      {error && <Alert role="danger">{error}</Alert>}

      <div>
        <Field
          label="Peripheral"
          help="Which role this device fills. Readings arrive at the machine under this peripheral, whichever device is associated with it."
        >
          {(control) => (
            <Select
              {...control}
              value={peripheralId}
              onChange={setPeripheralId}
              options={
                peripheralOptions.length === 0
                  ? [{ value: '', label: 'No peripherals defined' }]
                  : peripheralOptions.map(([id, definition]) => ({
                      value: String(id),
                      label: `${definition.location} — ${definition.peripheral_type.type} (0x${id
                        .toString(16)
                        .toUpperCase()
                        .padStart(4, '0')})`,
                    }))
              }
            />
          )}
        </Field>
        {/* A live consequence of the current selection rather than static guidance, so it
            is a warning below the field rather than part of its help. */}
        {replacing && (
          <div style={{ marginTop: tokens.space.sm }}>
            <Alert role="warn">
              This peripheral already has a device associated. Saving will replace it.
            </Alert>
          </div>
        )}
      </div>

      <div>
        <Field label="Driver" help={DRIVERS.find((d) => d.kind === driver)?.hint}>
          {(control) => (
            <Select
              {...control}
              value={driver}
              onChange={(value) => setDriver(value as BluetoothDriverKind['type'])}
              options={DRIVERS.map((d) => ({ value: d.kind, label: d.label }))}
            />
          )}
        </Field>
        {suggestedDriver && (
          <div style={{ marginTop: tokens.space.sm }}>
            <Alert role={suggestedDriver === driver ? 'ok' : 'warn'}>
              {suggestedDriver === driver
                ? 'This device advertised a service this driver supports.'
                : 'This device advertised a service the other driver supports — check before saving.'}
            </Alert>
          </div>
        )}
      </div>

      <Field label="Name" help="A label for you. Changing it does not disturb the connection.">
        {(control) => (
          <TextInput {...control} value={name} onInput={setName} placeholder="Group 1 scale" />
        )}
      </Field>

      <div>
        <label
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: tokens.space.sm,
            fontSize: '0.9rem',
            fontWeight: 500,
            cursor: 'pointer',
          }}
        >
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

      {/* Cancel first, primary last, and neither stretched to half the form. Two
          equally-wide filled buttons -- one green, one grey -- gave a destructive-free
          form two competing primaries. */}
      <div style={{ display: 'flex', gap: tokens.space.sm, justifyContent: 'flex-end' }}>
        <Button variant="secondary" onClick={onCancel}>
          Cancel
        </Button>
        <Button variant="primary" onClick={handleSave}>
          Save
        </Button>
      </div>
    </div>
  );
};

export const BluetoothPeripheralEditor = BluetoothPeripheralEditorComponent;
