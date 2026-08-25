import { useState } from 'preact/hooks';
import { Button, Dialog, Field, Select, TextInput, tokens } from '@variegated-coffee/ui';
import { RoutineParameter, ParameterUnit } from '../../schemas/schemas';
import { NumberField } from '../NumberField';

interface ParameterEditorProps {
  parameter: RoutineParameter | null;
  onSave: (parameter: RoutineParameter) => void;
  onCancel: () => void;
  existingIndices: number[];
}

/**
 * The units a parameter can carry, with the name a person uses for each.
 *
 * The options were the enum variants themselves, so the list read
 * "MillisiemensPerCentimeter" and "MillilitersPerSecond" -- names written for the wire
 * format, offered to someone choosing a unit for a dose.
 */
const PARAMETER_UNITS: { unit: ParameterUnit; label: string }[] = [
  { unit: { type: 'Seconds' }, label: 'Seconds (s)' },
  { unit: { type: 'Celsius' }, label: 'Degrees Celsius (°C)' },
  { unit: { type: 'Bar' }, label: 'Bar' },
  { unit: { type: 'MillilitersPerSecond' }, label: 'Millilitres per second (mL/s)' },
  { unit: { type: 'Grams' }, label: 'Grams (g)' },
  { unit: { type: 'Percent' }, label: 'Percent (%)' },
  { unit: { type: 'Milliliters' }, label: 'Millilitres (mL)' },
  { unit: { type: 'MillisiemensPerCentimeter' }, label: 'Conductivity (mS/cm)' },
  { unit: { type: 'ExtractionRate' }, label: 'Extraction rate (mS·mL/cm·s)' },
  { unit: { type: 'ExtractedSolids' }, label: 'Extracted solids (mS·mL/cm)' },
];

/// Shot attributes a parameter can mirror.
//
// Numeric keys only. A parameter is an f32, so only annotations holding a Number round-trip
// through one -- `Beans` and `GrindSize` are Text on purpose, because grinders number their
// settings incompatibly, and are deliberately absent from this list rather than offered and
// then rejected.
const LINKABLE_ATTRIBUTES = [
  { value: 'DoseWeight', label: 'Dose weight (g)' }
] as const;

/** The parameter slots the firmware stores. Mirrors `MAX_PARAMETERS` in ParametersTab. */
const MAX_INDEX = 7;

export function ParameterEditor({ parameter, onSave, onCancel, existingIndices }: ParameterEditorProps) {
  const [index, setIndex] = useState<number>(
    parameter?.index ?? (existingIndices.length > 0 ? Math.max(...existingIndices) + 1 : 0)
  );
  const [name, setName] = useState(parameter?.name || '');
  const [defaultValue, setDefaultValue] = useState(parameter?.default || 0);
  const [unit, setUnit] = useState<ParameterUnit | null>(parameter?.unit || null);
  const [linkedAttribute, setLinkedAttribute] = useState<RoutineParameter['linked_attribute']>(
    parameter?.linked_attribute ?? null
  );
  const [defaultInvalid, setDefaultInvalid] = useState(false);

  /*
   * Both of these used to be `alert()` calls inside `handleSave` -- a native dialog fired
   * after the button was pressed, telling the user about a field they had already left.
   * They are field errors now, and Save is disabled while either holds.
   */
  const nameError = name.trim() === '' ? 'A parameter needs a name.' : undefined;
  const indexError =
    !parameter && existingIndices.includes(index)
      ? `P${index} is already used by another parameter.`
      : index < 0 || index > MAX_INDEX
        ? `Must be between 0 and ${MAX_INDEX}.`
        : undefined;

  const invalid = Boolean(nameError) || Boolean(indexError) || defaultInvalid;

  const handleSave = () => {
    if (invalid) return;
    onSave({
      index,
      name: name.trim(),
      default: defaultValue,
      unit,
      linked_attribute: linkedAttribute
    });
  };

  return (
    <Dialog
      title={parameter ? 'Edit parameter' : 'New parameter'}
      onClose={onCancel}
      footer={
        <>
          <Button variant="secondary" onClick={onCancel}>
            Cancel
          </Button>
          <Button variant="primary" onClick={handleSave} disabled={invalid}>
            Save
          </Button>
        </>
      }
    >
      <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.lg }}>
        <Field
          label="Slot"
          help={indexError ? undefined : `Steps refer to this parameter as P${index}.`}
          error={indexError}
        >
          {(control) => (
            <TextInput
              {...control}
              numeric
              value={String(index)}
              // Fixed once the parameter exists: changing it would silently orphan every
              // step and derived parameter that refers to it.
              disabled={parameter !== null}
              onInput={(value) => {
                const parsed = Number.parseInt(value, 10);
                setIndex(Number.isNaN(parsed) ? -1 : parsed);
              }}
            />
          )}
        </Field>

        <Field label="Name" error={nameError} required>
          {(control) => (
            <TextInput
              {...control}
              value={name}
              onInput={setName}
              placeholder="Preinfusion time, Target pressure, …"
            />
          )}
        </Field>

        <NumberField
          label="Default value"
          value={defaultValue}
          onChange={setDefaultValue}
          onValidityChange={(valid) => setDefaultInvalid(!valid)}
          help="Used when the routine runs without being given a value."
        />

        <Field label="Unit" help="Optional. Shown next to the value wherever it appears.">
          {(control) => (
            <Select
              {...control}
              value={unit?.type ?? ''}
              onChange={(value) => setUnit(value ? ({ type: value } as ParameterUnit) : null)}
              options={[
                { value: '', label: 'No unit' },
                ...PARAMETER_UNITS.map((u) => ({ value: u.unit.type, label: u.label })),
              ]}
            />
          )}
        </Field>

        <Field
          label="Linked shot attribute"
          help="Optional. A linked parameter is seeded from the shot attribute before the routine runs, and the value it runs with is recorded against the shot. Only numeric attributes can be linked — beans and grind size are text."
        >
          {(control) => (
            <Select
              {...control}
              value={linkedAttribute?.type ?? ''}
              onChange={(value) =>
                setLinkedAttribute(
                  value ? ({ type: value } as RoutineParameter['linked_attribute']) : null
                )
              }
              options={[
                { value: '', label: 'Not linked' },
                ...LINKABLE_ATTRIBUTES.map((a) => ({ value: a.value, label: a.label })),
              ]}
            />
          )}
        </Field>
      </div>
    </Dialog>
  );
}
