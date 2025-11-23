import { ParameterValue, ParameterUnit, RoutineParameter, DerivedParameter } from '../../schemas/schemas';
import { getParameterValueDisplay } from './RoutineCommandSummary';

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

  const handleStaticChange = (num: number) => {
    onChange({ type: 'Static', value: num });
  };

  const handleParameterChange = (index: number) => {
    onChange({ type: 'Parameter', value: index });
  };

  const handleDerivedParameterChange = (index: number) => {
    onChange({ type: 'DerivedParameter', value: index });
  };

  return (
    <div style={{ marginBottom: '1rem' }}>
      <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
        {label}
      </label>

      <div style={{ display: 'flex', gap: '0.5rem', alignItems: 'center' }}>
        {/* Type selector */}
        <select
          value={valueType}
          onChange={(e) => handleTypeChange(e.currentTarget.value as ValueType)}
          style={{
            padding: '0.5rem',
            border: '1px solid #ccc',
            borderRadius: '4px',
            fontSize: '0.9rem',
            minWidth: '120px'
          }}
        >
          <option value="Static">Static Value</option>
          <option value="Parameter">Parameter</option>
          <option value="DerivedParameter">Derived Param</option>
        </select>

        {/* Value input */}
        {valueType === 'Static' && (
          <input
            type="number"
            step="0.1"
            value={value.type === 'Static' ? value.value : 0}
            onChange={(e) => handleStaticChange(parseFloat(e.currentTarget.value) || 0)}
            style={{
              flex: 1,
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '0.9rem'
            }}
          />
        )}

        {valueType === 'Parameter' && (
          <select
            value={value.type === 'Parameter' ? value.value : 0}
            onChange={(e) => handleParameterChange(parseInt(e.currentTarget.value))}
            style={{
              flex: 1,
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '0.9rem'
            }}
          >
            {parameters.length === 0 ? (
              <option value={0}>No parameters defined</option>
            ) : (
              parameters.map(param => (
                <option key={param.index} value={param.index}>
                  P{param.index}: {param.name}
                </option>
              ))
            )}
          </select>
        )}

        {valueType === 'DerivedParameter' && (
          <select
            value={value.type === 'DerivedParameter' ? value.value : 0}
            onChange={(e) => handleDerivedParameterChange(parseInt(e.currentTarget.value))}
            style={{
              flex: 1,
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '0.9rem'
            }}
          >
            {derivedParameters.length === 0 ? (
              <option value={0}>No derived parameters defined</option>
            ) : (
              derivedParameters.map(param => (
                <option key={param.index} value={param.index}>
                  D{param.index}: {param.name}
                </option>
              ))
            )}
          </select>
        )}

        {/* Preview */}
        <div style={{
          padding: '0.5rem 0.75rem',
          backgroundColor: '#f0f0f0',
          borderRadius: '4px',
          fontSize: '0.9rem',
          minWidth: '80px',
          textAlign: 'center',
          fontWeight: '500'
        }}>
          {getParameterValueDisplay(value, unit)}
        </div>
      </div>
    </div>
  );
}
