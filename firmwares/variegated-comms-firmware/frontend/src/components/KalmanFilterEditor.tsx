import { useState } from 'preact/hooks';
import { Alert, Button, tokens } from '@variegated-coffee/ui';
import { KalmanParameters } from '../schemas/schemas';
import { NumberField } from './NumberField';

interface KalmanFilterEditorProps {
  title: string;
  parameters: KalmanParameters | null;
  onSave: (params: KalmanParameters | null) => void;
  onCancel: () => void;
}

export const KalmanFilterEditor = ({ title, parameters, onSave, onCancel }: KalmanFilterEditorProps) => {
  const defaultParams: KalmanParameters = {
    process_noise: 0.01,
    measurement_noise: 0.1,
    estimated_error: 1.0,
    posterior_estimate: 20.0
  };

  const [enabled, setEnabled] = useState(parameters !== null);
  const [localParams, setLocalParams] = useState<KalmanParameters>(
    parameters ? (JSON.parse(JSON.stringify(parameters)) as KalmanParameters) : defaultParams
  );
  const [invalidFields, setInvalidFields] = useState<Record<string, true>>({});

  const invalid = Object.keys(invalidFields).length > 0;

  const handleSave = () => {
    if (invalid) return;
    onSave(enabled ? localParams : null);
  };

  const updateField = (field: keyof KalmanParameters, value: number) => {
    setLocalParams(prev => ({ ...prev, [field]: value }));
  };

  const validity = (field: string) => (valid: boolean) =>
    setInvalidFields((prev) => {
      if (valid) {
        const { [field]: _unused, ...rest } = prev;
        return rest;
      }
      return { ...prev, [field]: true };
    });

  return (
    <div>
      <h3 style={{ marginTop: 0, marginBottom: tokens.space.md }}>{title}</h3>

      <div style={{ marginBottom: tokens.space.lg }}>
        <label style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm, cursor: 'pointer' }}>
          <input
            type="checkbox"
            checked={enabled}
            onChange={(e) => setEnabled((e.target as HTMLInputElement).checked)}
            style={{ width: '1.2rem', height: '1.2rem' }}
          />
          <span style={{ fontSize: '0.95rem', fontWeight: 500 }}>Enable Kalman filtering</span>
        </label>
        <div style={{ fontSize: '0.75rem', color: tokens.color.inkMuted, marginTop: tokens.space.xs }}>
          Smooths a noisy sensor before the control loop sees it. Turning it off saves no
          settings — the four values below are discarded.
        </div>
      </div>

      {enabled && (
        <div
          style={{
            padding: tokens.space.md,
            backgroundColor: tokens.color.surfaceSunken,
            borderRadius: tokens.radius.md,
            border: `1px solid ${tokens.color.border}`,
          }}
        >
          <div
            style={{
              display: 'grid',
              gridTemplateColumns: 'repeat(auto-fit, minmax(12rem, 1fr))',
              gap: tokens.space.md,
            }}
          >
            {/* All four are variances and initial estimates, so none of them is meaningful
                below zero -- which the old fields did not enforce, and `|| 0` would have
                silently produced anyway. */}
            <NumberField
              label="Process noise"
              value={localParams.process_noise}
              onChange={(v) => updateField('process_noise', v)}
              onValidityChange={validity('process_noise')}
              help="How much the model is trusted. Lower means smoother and slower."
              min={0}
            />
            <NumberField
              label="Measurement noise"
              value={localParams.measurement_noise}
              onChange={(v) => updateField('measurement_noise', v)}
              onValidityChange={validity('measurement_noise')}
              help="How noisy the sensor is. Higher means the reading is trusted less."
              min={0}
            />
            <NumberField
              label="Estimated error"
              value={localParams.estimated_error}
              onChange={(v) => updateField('estimated_error', v)}
              onValidityChange={validity('estimated_error')}
              help="Initial uncertainty, before any reading has arrived."
              min={0}
            />
            <NumberField
              label="Posterior estimate"
              value={localParams.posterior_estimate}
              onChange={(v) => updateField('posterior_estimate', v)}
              onValidityChange={validity('posterior_estimate')}
              help="Initial value of the state, in the sensor's own unit."
            />
          </div>
        </div>
      )}

      {invalid && (
        <div style={{ marginTop: tokens.space.md }}>
          <Alert role="danger">Some fields do not hold a number. Fix them before saving.</Alert>
        </div>
      )}

      <div style={{ display: 'flex', gap: tokens.space.sm, justifyContent: 'flex-end', marginTop: tokens.space.lg }}>
        <Button variant="secondary" onClick={onCancel}>
          Cancel
        </Button>
        <Button variant="primary" onClick={handleSave} disabled={invalid}>
          Save changes
        </Button>
      </div>
    </div>
  );
};
