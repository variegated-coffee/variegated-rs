import { useState } from 'preact/hooks';
import { Alert, Button, tokens } from '@variegated-coffee/ui';
import { PumpConfiguration } from '../schemas/schemas';
import { NumberField } from './NumberField';

interface PumpConfigurationEditorProps {
  title: string;
  configuration: PumpConfiguration | null;
  onSave: (config: PumpConfiguration | null) => void;
  onCancel: () => void;
}

export const PumpConfigurationEditor = ({ title, configuration, onSave, onCancel }: PumpConfigurationEditorProps) => {
  const defaultConfig: PumpConfiguration = {
    min_duty_cycle: 10,
    max_duty_cycle: 100,
    ramp_up_time_ms: 500,
    ramp_down_time_ms: 500,
    tacho_pulses_per_liter: null
  };

  const [enabled, setEnabled] = useState(configuration !== null);
  const [localConfig, setLocalConfig] = useState<PumpConfiguration>(
    configuration ? (JSON.parse(JSON.stringify(configuration)) as PumpConfiguration) : defaultConfig
  );
  const [tachoEnabled, setTachoEnabled] = useState(
    configuration?.tacho_pulses_per_liter !== null && configuration?.tacho_pulses_per_liter !== undefined
  );
  const [invalidFields, setInvalidFields] = useState<Record<string, true>>({});

  const invalid = Object.keys(invalidFields).length > 0;

  /*
   * A duty cycle band that is inverted would be accepted by the fields individually and
   * rejected by neither, so it is checked across them. The firmware clamps rather than
   * refuses, which means the mistake would show up as a pump that never reaches the speed
   * it was told to.
   */
  const minDuty = localConfig.min_duty_cycle ?? 0;
  const maxDuty = localConfig.max_duty_cycle ?? 100;
  const bandInverted = minDuty > maxDuty;

  const validity = (key: string) => (valid: boolean) =>
    setInvalidFields((prev) => {
      if (valid) {
        const { [key]: _unused, ...rest } = prev;
        return rest;
      }
      return { ...prev, [key]: true };
    });

  const handleSave = () => {
    if (invalid || bandInverted) return;
    const configToSave = enabled ? {
      ...localConfig,
      tacho_pulses_per_liter: tachoEnabled ? localConfig.tacho_pulses_per_liter : null
    } : null;
    onSave(configToSave);
  };

  const updateField = (field: keyof PumpConfiguration, value: number | null) => {
    setLocalConfig(prev => ({ ...prev, [field]: value }));
  };

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
          <span style={{ fontSize: '0.95rem', fontWeight: 500 }}>Configure pump parameters</span>
        </label>
        <div style={{ fontSize: '0.75rem', color: tokens.color.inkMuted, marginTop: tokens.space.xs }}>
          Turning this off clears the settings below rather than keeping them.
        </div>
      </div>

      {enabled && (
        <div
          style={{
            display: 'flex',
            flexDirection: 'column',
            gap: tokens.space.md,
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
            {/*
              `min` is 0, and that matters. The old handler read
              `parseFloat(value) || null`, so a typed `0` -- which is falsy -- became
              `null`, and a minimum duty cycle of zero was impossible to set: the field
              accepted the keystroke and then silently cleared the value on blur.
            */}
            <NumberField
              label="Min duty cycle"
              unit="%"
              value={minDuty}
              onChange={(v) => updateField('min_duty_cycle', v)}
              onValidityChange={validity('min_duty_cycle')}
              min={0}
              max={100}
              help="Below this the pump stalls rather than running slowly."
            />
            <NumberField
              label="Max duty cycle"
              unit="%"
              value={maxDuty}
              onChange={(v) => updateField('max_duty_cycle', v)}
              onValidityChange={validity('max_duty_cycle')}
              min={0}
              max={100}
              help="The ceiling the control loop is allowed to ask for."
            />
            <NumberField
              label="Ramp up time"
              unit="ms"
              value={localConfig.ramp_up_time_ms ?? 0}
              onChange={(v) => updateField('ramp_up_time_ms', v)}
              onValidityChange={validity('ramp_up_time_ms')}
              min={0}
              help="How long the pump takes to reach a new higher duty."
            />
            <NumberField
              label="Ramp down time"
              unit="ms"
              value={localConfig.ramp_down_time_ms ?? 0}
              onChange={(v) => updateField('ramp_down_time_ms', v)}
              onValidityChange={validity('ramp_down_time_ms')}
              min={0}
              help="How long it takes to fall to a new lower duty."
            />
          </div>

          {bandInverted && (
            <Alert role="danger">
              The minimum duty cycle is above the maximum, so the pump has no range to work
              in.
            </Alert>
          )}

          <div style={{ paddingTop: tokens.space.md, borderTop: `1px solid ${tokens.color.border}` }}>
            <label
              style={{
                display: 'flex',
                alignItems: 'center',
                gap: tokens.space.sm,
                cursor: 'pointer',
                marginBottom: tokens.space.sm,
              }}
            >
              <input
                type="checkbox"
                checked={tachoEnabled}
                onChange={(e) => setTachoEnabled((e.target as HTMLInputElement).checked)}
                style={{ width: '1.2rem', height: '1.2rem' }}
              />
              <span style={{ fontSize: '0.9rem', fontWeight: 500 }}>This pump has a flow sensor</span>
            </label>

            {tachoEnabled && (
              <NumberField
                label="Tacho pulses per litre"
                unit="pulses/L"
                value={localConfig.tacho_pulses_per_liter ?? 0}
                onChange={(v) => updateField('tacho_pulses_per_liter', v)}
                onValidityChange={validity('tacho_pulses_per_liter')}
                min={0}
                help="From the sensor's datasheet, or measured by dispensing a known volume."
              />
            )}
          </div>
        </div>
      )}

      {invalid && (
        <div style={{ marginTop: tokens.space.md }}>
          <Alert role="danger">Fix the fields marked above before saving.</Alert>
        </div>
      )}

      <div style={{ display: 'flex', gap: tokens.space.sm, justifyContent: 'flex-end', marginTop: tokens.space.lg }}>
        <Button variant="secondary" onClick={onCancel}>
          Cancel
        </Button>
        <Button variant="primary" onClick={handleSave} disabled={invalid || bandInverted}>
          Save changes
        </Button>
      </div>
    </div>
  );
};
