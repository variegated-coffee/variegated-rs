import { Field, Select, tokens } from '@variegated-coffee/ui';
import { ParameterValue, ParameterUnit, RoutineParameter, DerivedParameter } from '../../schemas/schemas';
import { getParameterValueDisplay } from './RoutineCommandSummary';
import { NumberField } from '../NumberField';

interface ParameterValueEditorProps {
  value: ParameterValue;
  onChange: (value: ParameterValue) => void;
  label: string;
  unit?: ParameterUnit | null;
  parameters: RoutineParameter[];
  derivedParameters: DerivedParameter[];
}

type ValueType = 'Static' | 'Parameter' | 'DerivedParameter';

function getValueType(value: ParameterValue): ValueType {
  return value.type as ValueType;
}

export function ParameterValueEditor({
  value,
  onChange,
  label,
  unit,
  parameters,
  derivedParameters
}: ParameterValueEditorProps) {
  const valueType = getValueType(value);

  const handleTypeChange = (type: ValueType) => {
    if (type === 'Static') {
      onChange({ type: 'Static', value: 0 });
    } else if (type === 'Parameter') {
      onChange({ type: 'Parameter', value: parameters.length > 0 ? parameters[0].index : 0 });
    } else if (type === 'DerivedParameter') {
      onChange({ type: 'DerivedParameter', value: derivedParameters.length > 0 ? derivedParameters[0].index : 0 });
    }
  };

  return (
    <div style={{ marginBottom: tokens.space.md }}>
      <div
        style={{
          display: 'grid',
          gridTemplateColumns: 'minmax(9rem, auto) 1fr auto',
          gap: tokens.space.sm,
          alignItems: 'end',
        }}
      >
        {/* Two controls that together are one value, so each is labelled for what it
            contributes rather than both borrowing the caller's single label. The old
            version had one `<label>` above the row pointing at neither of them. */}
        <Field label={label}>
          {(control) => (
            <Select
              {...control}
              value={valueType}
              onChange={(next) => handleTypeChange(next as ValueType)}
              options={[
                { value: 'Static', label: 'A fixed value' },
                { value: 'Parameter', label: 'A parameter' },
                { value: 'DerivedParameter', label: 'A derived parameter' },
              ]}
            />
          )}
        </Field>

        {valueType === 'Static' && (
          <NumberField
            label={`${label} value`}
            value={value.type === 'Static' ? value.value : 0}
            onChange={(next) => onChange({ type: 'Static', value: next })}
          />
        )}

        {valueType === 'Parameter' && (
          <Field label={`${label} parameter`}>
            {(control) => (
              <Select
                {...control}
                value={String(value.type === 'Parameter' ? value.value : 0)}
                onChange={(next) => onChange({ type: 'Parameter', value: Number.parseInt(next, 10) })}
                options={
                  parameters.length === 0
                    ? [{ value: '0', label: 'No parameters defined' }]
                    : parameters.map((param) => ({
                        value: String(param.index),
                        label: `P${param.index}: ${param.name}`,
                      }))
                }
              />
            )}
          </Field>
        )}

        {valueType === 'DerivedParameter' && (
          <Field label={`${label} derived parameter`}>
            {(control) => (
              <Select
                {...control}
                value={String(value.type === 'DerivedParameter' ? value.value : 0)}
                onChange={(next) =>
                  onChange({ type: 'DerivedParameter', value: Number.parseInt(next, 10) })
                }
                options={
                  derivedParameters.length === 0
                    ? [{ value: '0', label: 'No derived parameters defined' }]
                    : derivedParameters.map((param) => ({
                        value: String(param.index),
                        label: `D${param.index}: ${param.name}`,
                      }))
                }
              />
            )}
          </Field>
        )}

        {/* What the machine will actually use, resolved. Worth keeping visible: with a
            parameter selected, the two controls say *where* the number comes from and
            this is the only thing that says what it is. */}
        <div
          style={{
            padding: `0.4rem ${tokens.space.sm}`,
            backgroundColor: tokens.color.surfaceSunken,
            border: `1px solid ${tokens.color.border}`,
            borderRadius: tokens.radius.sm,
            font: `0.9rem ${tokens.font.mono}`,
            fontVariantNumeric: 'tabular-nums',
            minWidth: '5rem',
            textAlign: 'center',
            fontWeight: 500,
          }}
        >
          {getParameterValueDisplay(value, unit)}
        </div>
      </div>
    </div>
  );
}
