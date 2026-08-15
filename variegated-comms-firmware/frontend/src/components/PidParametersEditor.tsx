import { useState } from 'preact/hooks';
import { PidParameters, Limits } from '../schemas/schemas';

interface PidParametersEditorProps {
  title: string;
  parameters: PidParameters;
  onSave: (params: PidParameters) => void;
  onCancel: () => void;
}

// Editable representation of limits - allows null during editing
type EditableLimits = {
  upper: number | null;
  lower: number | null;
};

// Convert schema Limits to editable form (Infinity becomes null for UI)
const fromLimits = (limits: Limits | null | undefined): EditableLimits => {
  if (!limits) {
    return { upper: null, lower: null };
  }
  return {
    upper: limits.upper === Infinity ? null : limits.upper,
    lower: limits.lower === -Infinity ? null : limits.lower
  };
};

// Convert editable limits back to schema form (null becomes Infinity, with validation)
const toLimits = (editable: EditableLimits): Limits => {
  const hasUpper = editable.upper !== null && isFinite(editable.upper);
  const hasLower = editable.lower !== null && isFinite(editable.lower);

  // If both are set and finite, use them
  if (hasUpper && hasLower) {
    // Type guard: we know both are non-null and finite at this point
    return { upper: editable.upper!, lower: editable.lower! };
  }

  // If only one is set, or neither, return no limits (Infinity)
  return { upper: Infinity, lower: -Infinity };
};

// Editable representation of a PID term
type EditablePidTerm = {
  positive_scale: number;
  negative_scale: number;
  limits: EditableLimits;
};

// Editable representation of PID parameters
type EditablePidParameters = {
  kp: EditablePidTerm;
  ki: EditablePidTerm;
  kd: EditablePidTerm;
};

// Convert schema parameters to editable form
const fromParameters = (params: PidParameters): EditablePidParameters => ({
  kp: {
    positive_scale: params.kp.positive_scale ?? 0,
    negative_scale: params.kp.negative_scale ?? 0,
    limits: fromLimits(params.kp.limits)
  },
  ki: {
    positive_scale: params.ki.positive_scale ?? 0,
    negative_scale: params.ki.negative_scale ?? 0,
    limits: fromLimits(params.ki.limits)
  },
  kd: {
    positive_scale: params.kd.positive_scale ?? 0,
    negative_scale: params.kd.negative_scale ?? 0,
    limits: fromLimits(params.kd.limits)
  }
});

// Convert editable parameters back to schema form
const toParameters = (editable: EditablePidParameters): PidParameters => ({
  kp: {
    positive_scale: editable.kp.positive_scale,
    negative_scale: editable.kp.negative_scale,
    limits: toLimits(editable.kp.limits)
  },
  ki: {
    positive_scale: editable.ki.positive_scale,
    negative_scale: editable.ki.negative_scale,
    limits: toLimits(editable.ki.limits)
  },
  kd: {
    positive_scale: editable.kd.positive_scale,
    negative_scale: editable.kd.negative_scale,
    limits: toLimits(editable.kd.limits)
  }
});

export const PidParametersEditor = ({ title, parameters, onSave, onCancel }: PidParametersEditorProps) => {
  // Use editable types for state management
  const [localParams, setLocalParams] = useState<EditablePidParameters>(
    fromParameters(parameters)
  );
  const [editingValues, setEditingValues] = useState<Record<string, string>>({});

  const handleSave = () => {
    // Convert back to schema types with validation
    onSave(toParameters(localParams));
  };

  const updateKpField = (field: 'positive_scale' | 'negative_scale', value: number) => {
    setLocalParams(prev => ({
      ...prev,
      kp: { ...prev.kp, [field]: value }
    }));
  };

  const updateKpLimit = (subfield: 'upper' | 'lower', value: number | null) => {
    setLocalParams(prev => ({
      ...prev,
      kp: {
        ...prev.kp,
        limits: { ...prev.kp.limits, [subfield]: value }
      }
    }));
  };

  const updateKiField = (field: 'positive_scale' | 'negative_scale', value: number) => {
    setLocalParams(prev => ({
      ...prev,
      ki: { ...prev.ki, [field]: value }
    }));
  };

  const updateKiLimit = (subfield: 'upper' | 'lower', value: number | null) => {
    setLocalParams(prev => ({
      ...prev,
      ki: {
        ...prev.ki,
        limits: { ...prev.ki.limits, [subfield]: value }
      }
    }));
  };

  const updateKdField = (field: 'positive_scale' | 'negative_scale', value: number) => {
    setLocalParams(prev => ({
      ...prev,
      kd: { ...prev.kd, [field]: value }
    }));
  };

  const updateKdLimit = (subfield: 'upper' | 'lower', value: number | null) => {
    setLocalParams(prev => ({
      ...prev,
      kd: {
        ...prev.kd,
        limits: { ...prev.kd.limits, [subfield]: value }
      }
    }));
  };

  const renderTermEditor = (
    termName: string,
    term: EditablePidTerm,
    updateField: (field: 'positive_scale' | 'negative_scale', value: number) => void,
    updateLimit: (subfield: 'upper' | 'lower', value: number | null) => void
  ) => {
    // Helper to format limit value for display (empty string for null)
    const formatLimitValue = (value: number | null): string => {
      if (value === null) {
        return '';
      }
      return value.toString();
    };

    return (
      <div
        style={{
          marginBottom: '1.5rem',
          padding: '1rem',
          backgroundColor: '#f8f9fa',
          borderRadius: '6px',
          border: '1px solid #e0e0e0'
        }}
      >
        <h4 style={{ marginTop: 0, marginBottom: '0.75rem', fontSize: '0.95rem' }}>
          {termName} Term
        </h4>

        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.75rem' }}>
          <div>
            <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
              Positive Scale
            </label>
            <input
              type="number"
              step="0.01"
              value={editingValues[`${termName}_positive_scale`] ?? term.positive_scale.toString()}
              onChange={(e) => {
                const val = e.currentTarget.value;
                setEditingValues(prev => ({ ...prev, [`${termName}_positive_scale`]: val }));
              }}
              onBlur={(e) => {
                const parsed = parseFloat(e.currentTarget.value) || 0;
                updateField('positive_scale', parsed);
                setEditingValues(prev => {
                  const { [`${termName}_positive_scale`]: _unused, ...rest } = prev;
                  return rest;
                });
              }}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '0.9rem'
              }}
            />
          </div>

          <div>
            <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
              Negative Scale
            </label>
            <input
              type="number"
              step="0.01"
              value={editingValues[`${termName}_negative_scale`] ?? term.negative_scale.toString()}
              onChange={(e) => {
                const val = e.currentTarget.value;
                setEditingValues(prev => ({ ...prev, [`${termName}_negative_scale`]: val }));
              }}
              onBlur={(e) => {
                const parsed = parseFloat(e.currentTarget.value) || 0;
                updateField('negative_scale', parsed);
                setEditingValues(prev => {
                  const { [`${termName}_negative_scale`]: _unused, ...rest } = prev;
                  return rest;
                });
              }}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '0.9rem'
              }}
            />
          </div>

          <div>
            <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
              Upper Limit <span style={{ color: '#999', fontSize: '0.8rem' }}>(empty = no limit)</span>
            </label>
            <input
              type="number"
              step="0.1"
              value={editingValues[`${termName}_limits_upper`] ?? formatLimitValue(term.limits.upper)}
              onChange={(e) => {
                const val = e.currentTarget.value;
                setEditingValues(prev => ({ ...prev, [`${termName}_limits_upper`]: val }));
              }}
              onBlur={(e) => {
                const val = e.currentTarget.value.trim();
                const parsed = val === '' ? null : parseFloat(val);
                updateLimit('upper', parsed);
                setEditingValues(prev => {
                  const { [`${termName}_limits_upper`]: _unused, ...rest } = prev;
                  return rest;
                });
              }}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '0.9rem'
              }}
            />
          </div>

          <div>
            <label style={{ display: 'block', fontSize: '0.85rem', color: '#666', marginBottom: '0.25rem' }}>
              Lower Limit <span style={{ color: '#999', fontSize: '0.8rem' }}>(empty = no limit)</span>
            </label>
            <input
              type="number"
              step="0.1"
              value={editingValues[`${termName}_limits_lower`] ?? formatLimitValue(term.limits.lower)}
              onChange={(e) => {
                const val = e.currentTarget.value;
                setEditingValues(prev => ({ ...prev, [`${termName}_limits_lower`]: val }));
              }}
              onBlur={(e) => {
                const val = e.currentTarget.value.trim();
                const parsed = val === '' ? null : parseFloat(val);
                updateLimit('lower', parsed);
                setEditingValues(prev => {
                  const { [`${termName}_limits_lower`]: _unused, ...rest } = prev;
                  return rest;
                });
              }}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
                fontSize: '0.9rem'
              }}
            />
          </div>
        </div>
      </div>
    );
  };

  return (
    <div>
      <div style={{ marginBottom: '1rem' }}>
        <h3 style={{ margin: 0 }}>{title}</h3>
      </div>

      {renderTermEditor('Proportional (Kp)', localParams.kp, updateKpField, updateKpLimit)}
      {renderTermEditor('Integral (Ki)', localParams.ki, updateKiField, updateKiLimit)}
      {renderTermEditor('Derivative (Kd)', localParams.kd, updateKdField, updateKdLimit)}

      <div style={{ display: 'flex', gap: '1rem', justifyContent: 'flex-end', marginTop: '1.5rem' }}>
        <button
          onClick={onCancel}
          style={{
            padding: '0.5rem 1.5rem',
            backgroundColor: 'white',
            border: '1px solid #ccc',
            borderRadius: '4px',
            cursor: 'pointer',
            fontSize: '0.9rem'
          }}
        >
          Cancel
        </button>
        <button
          onClick={handleSave}
          style={{
            padding: '0.5rem 1.5rem',
            backgroundColor: '#0066cc',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            cursor: 'pointer',
            fontSize: '0.9rem',
            fontWeight: '500'
          }}
        >
          Save Changes
        </button>
      </div>
    </div>
  );
};
