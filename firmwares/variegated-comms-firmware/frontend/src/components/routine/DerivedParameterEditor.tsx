import type { ComponentChildren } from 'preact';
import { useState } from 'preact/hooks';
import { Alert, Button, Dialog, Field, Select, TextInput, tokens } from '@variegated-coffee/ui';
import { DerivedParameter, DerivedFormula, ParameterUnit, RoutineParameter } from '../../schemas/schemas';
import { NumberField } from '../NumberField';

interface DerivedParameterEditorProps {
  parameter: DerivedParameter | null;
  onSave: (parameter: DerivedParameter) => void;
  onCancel: () => void;
  existingIndices: number[];
  availableParameters: RoutineParameter[];
}

type FormulaType = 'Linear' | 'Sum' | 'Difference' | 'Product';

/** Named the way a person names them, not the way the wire format does. */
const PARAMETER_UNITS: { unit: ParameterUnit; label: string }[] = [
  { unit: { type: 'Seconds' }, label: 'Seconds (s)' },
  { unit: { type: 'Celsius' }, label: 'Degrees Celsius (°C)' },
  { unit: { type: 'Bar' }, label: 'Bar' },
  { unit: { type: 'MillilitersPerSecond' }, label: 'Millilitres per second (mL/s)' },
  { unit: { type: 'Grams' }, label: 'Grams (g)' },
  { unit: { type: 'Percent' }, label: 'Percent (%)' },
];

/** The derived slots the firmware stores. Mirrors `MAX_DERIVED_PARAMETERS`. */
const MAX_INDEX = 15;

function getFormulaType(formula: DerivedFormula): FormulaType {
  return formula.type as FormulaType;
}

export function DerivedParameterEditor({
  parameter,
  onSave,
  onCancel,
  existingIndices,
  availableParameters
}: DerivedParameterEditorProps) {
  const [index, setIndex] = useState<number>(
    parameter?.index ?? (existingIndices.length > 0 ? Math.max(...existingIndices) + 1 : 0)
  );
  const [name, setName] = useState(parameter?.name || '');
  const [unit, setUnit] = useState<ParameterUnit | null>(parameter?.unit || null);

  const initialFormula: DerivedFormula = parameter?.formula || {
    type: 'Linear',
    value: { base_param: 0, multiplier: 1, offset: 0 }
  };
  const [formulaType, setFormulaType] = useState<FormulaType>(getFormulaType(initialFormula));

  // Linear formula state
  const [linearBaseParam, setLinearBaseParam] = useState(
    initialFormula.type === 'Linear' ? initialFormula.value.base_param : 0
  );
  const [linearMultiplier, setLinearMultiplier] = useState(
    initialFormula.type === 'Linear' ? initialFormula.value.multiplier : 1
  );
  const [linearOffset, setLinearOffset] = useState(
    initialFormula.type === 'Linear' ? initialFormula.value.offset : 0
  );

  // Sum/Product formula state
  const [listParams, setListParams] = useState<number[]>(
    initialFormula.type === 'Sum' ? initialFormula.value.params :
    initialFormula.type === 'Product' ? initialFormula.value.params : [0]
  );

  // Difference formula state
  const [diffParamA, setDiffParamA] = useState(
    initialFormula.type === 'Difference' ? initialFormula.value.param_a : 0
  );
  const [diffParamB, setDiffParamB] = useState(
    initialFormula.type === 'Difference' ? initialFormula.value.param_b : 0
  );

  const [numbersInvalid, setNumbersInvalid] = useState<Record<string, true>>({});

  /*
   * Three `alert()` calls used to live inside `handleSave`, firing a native dialog after
   * the button was pressed about fields the user had already left. Two are field errors
   * now; the third is a precondition of the whole editor, so it is stated at the top.
   */
  const nameError = name.trim() === '' ? 'A derived parameter needs a name.' : undefined;
  const indexError =
    !parameter && existingIndices.includes(index)
      ? `D${index} is already used by another derived parameter.`
      : index < 0 || index > MAX_INDEX
        ? `Must be between 0 and ${MAX_INDEX}.`
        : undefined;
  const noBaseParameters = availableParameters.length === 0;

  const invalid =
    Boolean(nameError) ||
    Boolean(indexError) ||
    noBaseParameters ||
    Object.keys(numbersInvalid).length > 0;

  const numberValidity = (key: string) => (valid: boolean) =>
    setNumbersInvalid((prev) => {
      if (valid) {
        const { [key]: _unused, ...rest } = prev;
        return rest;
      }
      return { ...prev, [key]: true };
    });

  /** The parameters a formula can be built from, as select options. */
  const parameterOptions = availableParameters.map((p) => ({
    value: String(p.index),
    label: `P${p.index}: ${p.name}`,
  }));

  const handleSave = () => {
    if (invalid) return;

    let formula: DerivedFormula;
    switch (formulaType) {
      case 'Linear':
        formula = { type: 'Linear', value: { base_param: linearBaseParam, multiplier: linearMultiplier, offset: linearOffset } };
        break;
      case 'Sum':
        formula = { type: 'Sum', value: { params: listParams } };
        break;
      case 'Difference':
        formula = { type: 'Difference', value: { param_a: diffParamA, param_b: diffParamB } };
        break;
      case 'Product':
        formula = { type: 'Product', value: { params: listParams } };
        break;
    }

    onSave({
      index,
      name: name.trim(),
      unit,
      formula
    });
  };

  const handleFormulaTypeChange = (type: FormulaType) => {
    setFormulaType(type);
    // Reset to sensible defaults
    if (type === 'Linear') {
      setLinearBaseParam(availableParameters.length > 0 ? availableParameters[0].index : 0);
      setLinearMultiplier(1);
      setLinearOffset(0);
    } else if (type === 'Sum' || type === 'Product') {
      setListParams(availableParameters.length > 0 ? [availableParameters[0].index] : [0]);
    } else if (type === 'Difference') {
      setDiffParamA(availableParameters.length > 0 ? availableParameters[0].index : 0);
      setDiffParamB(availableParameters.length > 1 ? availableParameters[1].index : 0);
    }
  };

  const addListParam = () => {
    setListParams([...listParams, availableParameters.length > 0 ? availableParameters[0].index : 0]);
  };

  const removeListParam = (idx: number) => {
    setListParams(listParams.filter((_, i) => i !== idx));
  };

  const updateListParam = (idx: number, value: number) => {
    const newParams = [...listParams];
    newParams[idx] = value;
    setListParams(newParams);
  };

  return (
    <Dialog
      title={parameter ? 'Edit derived parameter' : 'New derived parameter'}
      onClose={onCancel}
      width="600px"
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
        {noBaseParameters && (
          <Alert role="warn">
            A derived parameter is computed from the regular ones, so there is nothing to
            build this from yet. Add a parameter first.
          </Alert>
        )}

        <Field
          label="Slot"
          help={indexError ? undefined : `Steps refer to this as D${index}.`}
          error={indexError}
        >
          {(control) => (
            <TextInput
              {...control}
              numeric
              value={String(index)}
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
              placeholder="Total time, Brew ratio, …"
            />
          )}
        </Field>

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

        <Field label="Formula">
          {(control) => (
            <Select
              {...control}
              value={formulaType}
              onChange={(value) => handleFormulaTypeChange(value as FormulaType)}
              options={[
                { value: 'Linear', label: 'Scale and offset one parameter' },
                { value: 'Sum', label: 'Add parameters together' },
                { value: 'Difference', label: 'Subtract one parameter from another' },
                { value: 'Product', label: 'Multiply parameters together' },
              ]}
            />
          )}
        </Field>

        {formulaType === 'Linear' && (
          <div style={{ padding: tokens.space.md, backgroundColor: tokens.color.surfaceSunken, border: `1px solid ${tokens.color.border}`, borderRadius: tokens.radius.sm, display: 'flex', flexDirection: 'column', gap: tokens.space.md }}>
            <Field label="Base parameter">
              {(control) => (
                <Select
                  {...control}
                  value={String(linearBaseParam)}
                  onChange={(value) => setLinearBaseParam(Number.parseInt(value, 10))}
                  options={parameterOptions}
                />
              )}
            </Field>
            <NumberField
              label="Multiplier"
              value={linearMultiplier}
              onChange={setLinearMultiplier}
              onValidityChange={numberValidity('multiplier')}
            />
            <NumberField
              label="Offset"
              value={linearOffset}
              onChange={setLinearOffset}
              onValidityChange={numberValidity('offset')}
            />
            <FormulaPreview>
              D{index} = {linearMultiplier} × P{linearBaseParam} + {linearOffset}
            </FormulaPreview>
          </div>
        )}

        {(formulaType === 'Sum' || formulaType === 'Product') && (
          <div style={{ padding: tokens.space.md, backgroundColor: tokens.color.surfaceSunken, border: `1px solid ${tokens.color.border}`, borderRadius: tokens.radius.sm, display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
            {listParams.map((paramIdx, idx) => (
              <div key={idx} style={{ display: 'flex', gap: tokens.space.sm, alignItems: 'flex-end' }}>
                <div style={{ flex: 1 }}>
                  <Field label={`Parameter ${idx + 1}`}>
                    {(control) => (
                      <Select
                        {...control}
                        value={String(paramIdx)}
                        onChange={(value) => updateListParam(idx, Number.parseInt(value, 10))}
                        options={parameterOptions}
                      />
                    )}
                  </Field>
                </div>
                <Button
                  variant="destructive"
                  size="sm"
                  onClick={() => removeListParam(idx)}
                  disabled={listParams.length === 1}
                  ariaLabel={`Remove parameter ${idx + 1} from the formula`}
                >
                  Remove
                </Button>
              </div>
            ))}
            <div>
              <Button variant="secondary" size="sm" onClick={addListParam}>
                Add parameter
              </Button>
            </div>
            <FormulaPreview>
              D{index} = {listParams.map(p => `P${p}`).join(formulaType === 'Sum' ? ' + ' : ' × ')}
            </FormulaPreview>
          </div>
        )}

        {formulaType === 'Difference' && (
          <div style={{ padding: tokens.space.md, backgroundColor: tokens.color.surfaceSunken, border: `1px solid ${tokens.color.border}`, borderRadius: tokens.radius.sm, display: 'flex', flexDirection: 'column', gap: tokens.space.md }}>
            <Field label="Subtract from">
              {(control) => (
                <Select
                  {...control}
                  value={String(diffParamA)}
                  onChange={(value) => setDiffParamA(Number.parseInt(value, 10))}
                  options={parameterOptions}
                />
              )}
            </Field>
            <Field label="Subtract">
              {(control) => (
                <Select
                  {...control}
                  value={String(diffParamB)}
                  onChange={(value) => setDiffParamB(Number.parseInt(value, 10))}
                  options={parameterOptions}
                />
              )}
            </Field>
            <FormulaPreview>
              D{index} = P{diffParamA} − P{diffParamB}
            </FormulaPreview>
          </div>
        )}

      </div>
    </Dialog>
  );
}

/**
 * The formula as it will actually be evaluated, spelled out.
 *
 * Monospace, because it is an expression -- the indices line up under each other when the
 * formula type is changed, which is how you notice you are subtracting the wrong way round.
 */
function FormulaPreview({ children }: { children: ComponentChildren }) {
  return (
    <div
      style={{
        fontSize: '0.9rem',
        color: tokens.color.inkMuted,
        fontFamily: tokens.font.mono,
      }}
    >
      {children}
    </div>
  );
}
