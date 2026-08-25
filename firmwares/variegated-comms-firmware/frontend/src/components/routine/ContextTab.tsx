import { Alert, Field, TextInput, tokens } from '@variegated-coffee/ui';
import { RoutineParameter, RoutinePrerequisite, SensorCapability, ShotAnnotation } from '../../schemas/schemas';
import { MAX_ROUTINE_SHOT_ANNOTATIONS, isLinkedTo } from '../../utils/routineHelpers';

interface ContextTabProps {
  prerequisites: RoutinePrerequisite[];
  shotAnnotations: ShotAnnotation[];
  /** Only to warn about a key that is both a static attribute and a linked parameter. */
  parameters: RoutineParameter[];
  onPrerequisitesChange: (prerequisites: RoutinePrerequisite[]) => void;
  onShotAnnotationsChange: (annotations: ShotAnnotation[]) => void;
}

// The capabilities worth declaring. Not every `SensorCapability` variant: a prerequisite is
// only useful for something a machine might *not* have, and boiler temperature and pressure
// are on-board sensors that no espresso machine this firmware runs on lacks.
const CAPABILITIES: { value: SensorCapability['type']; label: string; hint: string }[] = [
  { value: 'Weight', label: 'Scale', hint: 'Brew by weight, dose capture' },
  { value: 'ElectricalConductivity', label: 'Conductivity probe', hint: 'Extraction rate, solids in cup' },
  { value: 'OutputFlowRate', label: 'Output flow', hint: 'Flow-rate control on what leaves the group' },
  { value: 'InputFlowRate', label: 'Input flow', hint: 'Volumetric dosing' },
  { value: 'WaterLevel', label: 'Tank level', hint: 'Tank-fed machines' }
];

// Static attributes may be text or numeric. Unlike a linked parameter -- which is an f32 and
// so needs a numeric key -- this block is written straight into the shot's annotations, so
// beans and grind are exactly what belongs here.
const ATTRIBUTE_KEYS = [
  { value: 'Beans', label: 'Beans', numeric: false },
  { value: 'GrindSize', label: 'Grind size', numeric: false },
  { value: 'DoseWeight', label: 'Dose weight (g)', numeric: true }
] as const;

export function ContextTab({
  prerequisites,
  shotAnnotations,
  parameters,
  onPrerequisitesChange,
  onShotAnnotationsChange
}: ContextTabProps) {
  const has = (capability: SensorCapability['type']) =>
    prerequisites.some(p => p.capability.type === capability);

  const toggle = (capability: SensorCapability['type']) => {
    onPrerequisitesChange(
      has(capability)
        ? prerequisites.filter(p => p.capability.type !== capability)
        : [...prerequisites, { capability: { type: capability } as SensorCapability }]
    );
  };

  const setAttribute = (key: string, raw: string, numeric: boolean) => {
    const rest = shotAnnotations.filter(a => a.key.type !== key);
    if (!raw.trim()) {
      onShotAnnotationsChange(rest);
      return;
    }
    const value = numeric
      ? { type: 'Number' as const, value: parseFloat(raw) || 0 }
      : { type: 'Text' as const, value: raw };
    onShotAnnotationsChange([...rest, { key: { type: key }, value } as ShotAnnotation]);
  };

  const attributeValue = (key: string): string => {
    const found = shotAnnotations.find(a => a.key.type === key);
    if (!found) return '';
    return found.value.type === 'Number' ? String(found.value.value) : found.value.value;
  };

  const sectionStyle = {
    padding: tokens.space.lg,
    borderBottom: `1px solid ${tokens.color.border}`,
  };
  const headingStyle = { margin: `0 0 ${tokens.space.xs} 0`, fontSize: '1.05rem' };
  const blurbStyle = {
    margin: `0 0 ${tokens.space.md} 0`,
    fontSize: '0.85rem',
    color: tokens.color.inkMuted,
  };

  return (
    <div>
      <div style={sectionStyle}>
        <h3 style={headingStyle}>Prerequisites</h3>
        <p style={blurbStyle}>
          What the machine must be able to sense for this routine to run. A routine whose
          prerequisites are not met is greyed out and cannot be started &mdash; and if one
          disappears while it is running, the routine stops.
        </p>
        {CAPABILITIES.map(c => (
          <label
            key={c.value}
            style={{
              display: 'flex',
              alignItems: 'baseline',
              gap: tokens.space.sm,
              marginBottom: tokens.space.sm,
              cursor: 'pointer',
            }}
          >
            <input type="checkbox" checked={has(c.value)} onChange={() => toggle(c.value)} />
            <span>
              {c.label}
              <span style={{ color: tokens.color.inkMuted, fontSize: '0.85rem' }}>
                {' '}&mdash; {c.hint}
              </span>
            </span>
          </label>
        ))}
      </div>

      <div style={sectionStyle}>
        <h3 style={headingStyle}>Shot attributes</h3>
        <p style={blurbStyle}>
          Recorded against every shot this routine pulls. These fill in blanks only &mdash; a
          value entered for the next shot on the Shots panel wins over the one set here.
          At most {MAX_ROUTINE_SHOT_ANNOTATIONS}.
        </p>
        {ATTRIBUTE_KEYS.map(a => {
          const clash = a.numeric && parameters.some(p => isLinkedTo(p, a.value));
          return (
            <div key={a.value} style={{ marginBottom: tokens.space.md }}>
              <Field
                label={a.label}
                // Not an error: the runtime resolves it, and predictably. Worth saying
                // because the value typed here will not be the one recorded, which is
                // otherwise indistinguishable from the field being ignored.
                help={
                  clash
                    ? 'A parameter is linked to this attribute. The parameter’s value is what gets recorded; this one only seeds it.'
                    : undefined
                }
              >
                {(control) => (
                  <TextInput
                    {...control}
                    // `numeric` rather than `type="number"`: a dose of 18.5 typed on a
                    // machine in a comma locale would otherwise render as 18,5 here and
                    // be recorded against the shot that way.
                    numeric={a.numeric}
                    value={attributeValue(a.value)}
                    onInput={(value) => setAttribute(a.value, value, a.numeric)}
                    placeholder="Leave blank to record nothing"
                  />
                )}
              </Field>
            </div>
          );
        })}
        {shotAnnotations.length > MAX_ROUTINE_SHOT_ANNOTATIONS && (
          <Alert role="danger">
            Too many shot attributes &mdash; the machine will refuse to store this routine.
          </Alert>
        )}
      </div>
    </div>
  );
}
