import { useState } from 'preact/hooks';
import { Alert, Button, tokens } from '@variegated-coffee/ui';
import { ControlCurve } from '../schemas/schemas';
import { NumberField } from './NumberField';

interface ControlCurveEditorProps {
  title: string;
  curve: ControlCurve;
  unit: string;
  onSave: (curve: ControlCurve) => void;
  onCancel: () => void;
}

/** How far into a shot the preview looks, in seconds. */
const PREVIEW_SECONDS = 50;
const PREVIEW_STEP = 5;

export const ControlCurveEditor = ({ title, curve, unit, onSave, onCancel }: ControlCurveEditorProps) => {
  const [localCurve, setLocalCurve] = useState<ControlCurve>(JSON.parse(JSON.stringify(curve)) as ControlCurve);
  const [invalidFields, setInvalidFields] = useState<Record<string, true>>({});

  const invalid = Object.keys(invalidFields).length > 0;
  // A clamp band the wrong way round leaves the curve with nowhere to be, and neither
  // field is individually wrong -- so it is checked across the pair.
  const bandInverted = localCurve.min > localCurve.max;

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
    onSave(localCurve);
  };

  const updateField = (field: keyof ControlCurve, value: number) => {
    setLocalCurve(prev => ({ ...prev, [field]: value }));
  };

  const previewPoints = (() => {
    const points = [];
    for (let t = 0; t <= PREVIEW_SECONDS; t += PREVIEW_STEP) {
      const y = localCurve.a + localCurve.b * t + localCurve.c * t * t;
      const clamped = Math.max(localCurve.min, Math.min(localCurve.max, y));
      points.push({ t, y: clamped });
    }
    return points;
  })();

  const maxY = Math.max(...previewPoints.map(p => p.y));
  const minY = Math.min(...previewPoints.map(p => p.y));
  const rangeY = maxY - minY || 1;

  const position = (point: { t: number; y: number }) => ({
    x: (point.t / PREVIEW_SECONDS) * 100,
    y: 100 - ((point.y - minY) / rangeY) * 80,
  });

  return (
    <div>
      <h3 style={{ marginTop: 0, marginBottom: tokens.space.md }}>{title}</h3>

      <div
        style={{
          padding: tokens.space.md,
          backgroundColor: tokens.color.surfaceSunken,
          borderRadius: tokens.radius.md,
          border: `1px solid ${tokens.color.border}`,
          marginBottom: tokens.space.md,
        }}
      >
        <div style={{ fontSize: '0.9rem', color: tokens.color.inkMuted, marginBottom: tokens.space.md }}>
          <span style={{ fontFamily: tokens.font.mono }}>y = a + b·t + c·t²</span>
          <div style={{ fontSize: '0.8rem', marginTop: tokens.space.xs }}>
            where <em>t</em> is seconds from the start of the shot, and <em>y</em> is in {unit}
          </div>
        </div>

        <div
          style={{
            display: 'grid',
            gridTemplateColumns: 'repeat(auto-fit, minmax(11rem, 1fr))',
            gap: tokens.space.md,
          }}
        >
          <NumberField
            label="Coefficient a"
            unit={unit}
            value={localCurve.a}
            onChange={(v) => updateField('a', v)}
            onValidityChange={validity('a')}
            help="Where the curve starts."
          />
          <NumberField
            label="Coefficient b"
            unit={`${unit}/s`}
            value={localCurve.b}
            onChange={(v) => updateField('b', v)}
            onValidityChange={validity('b')}
            help="Linear rate of change."
          />
          <NumberField
            label="Coefficient c"
            unit={`${unit}/s²`}
            value={localCurve.c}
            onChange={(v) => updateField('c', v)}
            onValidityChange={validity('c')}
            help="Curvature. Negative bends the curve down."
          />
          <NumberField
            label="Minimum value"
            unit={unit}
            value={localCurve.min}
            onChange={(v) => updateField('min', v)}
            onValidityChange={validity('min')}
            help="The curve is clamped up to this."
          />
          <NumberField
            label="Maximum value"
            unit={unit}
            value={localCurve.max}
            onChange={(v) => updateField('max', v)}
            onValidityChange={validity('max')}
            help="The curve is clamped down to this."
          />
        </div>

        {bandInverted && (
          <div style={{ marginTop: tokens.space.md }}>
            <Alert role="danger">
              The minimum is above the maximum, so the curve is clamped to nothing.
            </Alert>
          </div>
        )}
      </div>

      <div
        style={{
          padding: tokens.space.md,
          backgroundColor: tokens.color.surfaceRaised,
          border: `1px solid ${tokens.color.border}`,
          borderRadius: tokens.radius.md,
          marginBottom: tokens.space.md,
        }}
      >
        <div style={{ fontSize: '0.9rem', fontWeight: 500, marginBottom: tokens.space.sm }}>
          Preview
        </div>
        <div
          style={{
            position: 'relative',
            height: '200px',
            backgroundColor: tokens.color.surfaceSunken,
            borderRadius: tokens.radius.sm,
            padding: tokens.space.md,
          }}
        >
          {/* Labelled as an image with its shape described, because the numbers behind it
              are not otherwise readable: an SVG of bare lines announces as nothing at all. */}
          <svg
            width="100%"
            height="100%"
            style={{ display: 'block' }}
            role="img"
            aria-label={`Curve from ${previewPoints[0]?.y.toFixed(1)} ${unit} at 0 seconds to ${previewPoints[previewPoints.length - 1]?.y.toFixed(1)} ${unit} at ${PREVIEW_SECONDS} seconds, clamped between ${localCurve.min} and ${localCurve.max} ${unit}`}
          >
            <text x="5" y="15" fontSize="10" fill={tokens.color.inkMuted}>
              {maxY.toFixed(1)} {unit}
            </text>
            <text x="5" y="185" fontSize="10" fill={tokens.color.inkMuted}>
              {minY.toFixed(1)} {unit}
            </text>
            <text x="5" y="195" fontSize="10" fill={tokens.color.inkMuted}>
              0s
            </text>
            <text x="95%" y="195" fontSize="10" fill={tokens.color.inkMuted} textAnchor="end">
              {PREVIEW_SECONDS}s
            </text>

            {previewPoints.map((point, idx) => {
              if (idx === 0) return null;
              const here = position(point);
              const before = position(previewPoints[idx - 1]!);
              return (
                <line
                  key={idx}
                  x1={`${before.x}%`}
                  y1={`${before.y}%`}
                  x2={`${here.x}%`}
                  y2={`${here.y}%`}
                  // The pen the shot charts draw pressure with, so a pressure curve here
                  // and the trace it produces are the same colour.
                  stroke={tokens.pen.pressure}
                  strokeWidth="2"
                />
              );
            })}

            {previewPoints.map((point, idx) => {
              const here = position(point);
              return (
                <circle key={idx} cx={`${here.x}%`} cy={`${here.y}%`} r="3" fill={tokens.pen.pressure} />
              );
            })}
          </svg>
        </div>
      </div>

      {invalid && (
        <div style={{ marginBottom: tokens.space.md }}>
          <Alert role="danger">Fix the fields marked above before saving.</Alert>
        </div>
      )}

      <div style={{ display: 'flex', gap: tokens.space.sm, justifyContent: 'flex-end' }}>
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
