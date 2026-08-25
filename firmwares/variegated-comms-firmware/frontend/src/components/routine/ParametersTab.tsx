import { useState } from 'preact/hooks';
import { Badge, Button, EmptyState, tokens, useDialogs } from '@variegated-coffee/ui';
import { RoutineParameter, DerivedParameter } from '../../schemas/schemas';
import { ParameterEditor } from './ParameterEditor';
import { DerivedParameterEditor } from './DerivedParameterEditor';

interface ParametersTabProps {
  parameters: RoutineParameter[];
  derivedParameters: DerivedParameter[];
  onParametersChange: (parameters: RoutineParameter[]) => void;
  onDerivedParametersChange: (derivedParameters: DerivedParameter[]) => void;
}

/**
 * What the firmware will store.
 *
 * Exported because `RoutineEditor`'s tab label counts the same things: the tab used to read
 * *Parameters (2)* -- regular plus derived -- while the heading beneath it read
 * *Parameters (1/8)*, counting only the regular ones against their own ceiling. Two numbers
 * for the same word, on the same screen, four pixels apart.
 */
export const MAX_PARAMETERS = 8;
export const MAX_DERIVED_PARAMETERS = 16;

function getDerivedFormulaDescription(param: DerivedParameter): string {
  const formula = param.formula;
  if (formula.type === 'Linear') {
    return `${formula.value.multiplier} × P${formula.value.base_param} + ${formula.value.offset}`;
  }
  if (formula.type === 'Sum') {
    return formula.value.params.map((p: number) => `P${p}`).join(' + ');
  }
  if (formula.type === 'Difference') {
    return `P${formula.value.param_a} - P${formula.value.param_b}`;
  }
  if (formula.type === 'Product') {
    return formula.value.params.map((p: number) => `P${p}`).join(' × ');
  }
  return '';
}

export function ParametersTab({
  parameters,
  derivedParameters,
  onParametersChange,
  onDerivedParametersChange
}: ParametersTabProps) {
  const { confirm } = useDialogs();
  const [editingParam, setEditingParam] = useState<RoutineParameter | null>(null);
  const [isAddingParam, setIsAddingParam] = useState(false);
  const [editingDerived, setEditingDerived] = useState<DerivedParameter | null>(null);
  const [isAddingDerived, setIsAddingDerived] = useState(false);

  const handleSaveParameter = (param: RoutineParameter) => {
    const existingIndex = parameters.findIndex(p => p.index === param.index);
    if (existingIndex >= 0) {
      const newParams = [...parameters];
      newParams[existingIndex] = param;
      onParametersChange(newParams);
    } else {
      onParametersChange([...parameters, param]);
    }
    setEditingParam(null);
    setIsAddingParam(false);
  };

  const handleDeleteParameter = (param: RoutineParameter) => {
    // Named, and warned about what else it affects: a derived parameter built on this one
    // is left referring to an index that no longer exists.
    const dependents = derivedParameters.filter((d) => dependsOn(d, param.index));
    void confirm({
      title: `Delete P${param.index}: ${param.name}?`,
      body:
        dependents.length > 0
          ? `${dependents.length} derived parameter${dependents.length === 1 ? '' : 's'} use${
              dependents.length === 1 ? 's' : ''
            } it and will be left pointing at nothing.`
          : undefined,
      confirmLabel: 'Delete',
      destructive: true,
    }).then((ok) => {
      if (ok) onParametersChange(parameters.filter(p => p.index !== param.index));
    });
  };

  const handleSaveDerivedParameter = (param: DerivedParameter) => {
    const existingIndex = derivedParameters.findIndex(p => p.index === param.index);
    if (existingIndex >= 0) {
      const newParams = [...derivedParameters];
      newParams[existingIndex] = param;
      onDerivedParametersChange(newParams);
    } else {
      onDerivedParametersChange([...derivedParameters, param]);
    }
    setEditingDerived(null);
    setIsAddingDerived(false);
  };

  const handleDeleteDerivedParameter = (param: DerivedParameter) => {
    void confirm({
      title: `Delete D${param.index}: ${param.name}?`,
      confirmLabel: 'Delete',
      destructive: true,
    }).then((ok) => {
      if (ok) onDerivedParametersChange(derivedParameters.filter(p => p.index !== param.index));
    });
  };

  const parametersFull = parameters.length >= MAX_PARAMETERS;
  const derivedFull = derivedParameters.length >= MAX_DERIVED_PARAMETERS;
  const noBaseParameters = parameters.length === 0;

  const rowStyle = {
    padding: tokens.space.sm,
    backgroundColor: tokens.color.surfaceSunken,
    border: `1px solid ${tokens.color.border}`,
    borderRadius: tokens.radius.sm,
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    gap: tokens.space.sm,
    flexWrap: 'wrap' as const,
  };

  return (
    <div
      style={{
        padding: tokens.space.lg,
        display: 'grid',
        // One column below the width where two lists of parameter rows stop both fitting.
        // The two-column grid was unconditional.
        gridTemplateColumns: 'repeat(auto-fit, minmax(20rem, 1fr))',
        gap: tokens.space.xl,
      }}
    >
      <div>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm, marginBottom: tokens.space.md, flexWrap: 'wrap' }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm }}>
            <h3 style={{ margin: 0 }}>Parameters</h3>
            <Badge numeric role={parametersFull ? 'warn' : undefined}>
              {parameters.length}/{MAX_PARAMETERS}
            </Badge>
          </div>
          {/* Secondary, both of them. These were a blue *+ Add Parameter* and a green
              *+ Add Derived* sitting next to a blue *Save Routine* in the footer -- three
              weights all claiming to be the screen's primary action. Saving the routine is
              the primary; adding a parameter is a step on the way. */}
          <Button variant="secondary" size="sm" onClick={() => setIsAddingParam(true)} disabled={parametersFull}>
            Add parameter
          </Button>
        </div>

        {parameters.length === 0 ? (
          <EmptyState
            title="No parameters"
            detail="A parameter is a value the routine takes when it runs — a dose, a target temperature."
            action={{ label: 'Add parameter', onClick: () => setIsAddingParam(true) }}
          />
        ) : (
          <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
            {parameters.map(param => (
              <div key={param.index} style={rowStyle}>
                <div>
                  <div style={{ fontWeight: 500, marginBottom: tokens.space.xs }}>
                    <span style={{ fontFamily: tokens.font.mono }}>P{param.index}</span>{' '}
                    {param.name}
                  </div>
                  <div style={{ fontSize: '0.85rem', color: tokens.color.inkMuted }}>
                    Default {param.default}
                    {param.unit ? ` ${param.unit.type}` : ''}
                  </div>
                </div>
                <div style={{ display: 'flex', gap: tokens.space.sm }}>
                  <Button
                    variant="secondary"
                    size="sm"
                    onClick={() => setEditingParam(param)}
                    ariaLabel={`Edit P${param.index} ${param.name}`}
                  >
                    Edit
                  </Button>
                  <Button
                    variant="destructive"
                    size="sm"
                    onClick={() => handleDeleteParameter(param)}
                    ariaLabel={`Delete P${param.index} ${param.name}`}
                  >
                    Delete
                  </Button>
                </div>
              </div>
            ))}
          </div>
        )}
      </div>

      <div>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm, marginBottom: tokens.space.md, flexWrap: 'wrap' }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm }}>
            <h3 style={{ margin: 0 }}>Derived</h3>
            <Badge numeric role={derivedFull ? 'warn' : undefined}>
              {derivedParameters.length}/{MAX_DERIVED_PARAMETERS}
            </Badge>
          </div>
          <Button
            variant="secondary"
            size="sm"
            onClick={() => setIsAddingDerived(true)}
            disabled={derivedFull || noBaseParameters}
          >
            Add derived
          </Button>
        </div>

        {derivedParameters.length === 0 ? (
          <EmptyState
            title="No derived parameters"
            // The reason the button is disabled, said in the place the user is looking --
            // it used to be a `title` tooltip on a disabled button, which browsers do not
            // reliably show and touchscreens cannot show at all.
            detail={
              noBaseParameters
                ? 'A derived parameter is computed from the ones on the left, so add a parameter first.'
                : 'A derived parameter is computed from the others — a ratio, a sum, a scaled copy.'
            }
            action={
              noBaseParameters
                ? undefined
                : { label: 'Add derived', onClick: () => setIsAddingDerived(true) }
            }
          />
        ) : (
          <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
            {derivedParameters.map(param => (
              <div key={param.index} style={rowStyle}>
                <div>
                  <div style={{ fontWeight: 500, marginBottom: tokens.space.xs }}>
                    <span style={{ fontFamily: tokens.font.mono }}>D{param.index}</span>{' '}
                    {param.name}
                  </div>
                  <div style={{ fontSize: '0.85rem', color: tokens.color.inkMuted, fontFamily: tokens.font.mono }}>
                    {getDerivedFormulaDescription(param)}
                    {param.unit && ` ${param.unit.type}`}
                  </div>
                </div>
                <div style={{ display: 'flex', gap: tokens.space.sm }}>
                  <Button
                    variant="secondary"
                    size="sm"
                    onClick={() => setEditingDerived(param)}
                    ariaLabel={`Edit D${param.index} ${param.name}`}
                  >
                    Edit
                  </Button>
                  <Button
                    variant="destructive"
                    size="sm"
                    onClick={() => handleDeleteDerivedParameter(param)}
                    ariaLabel={`Delete D${param.index} ${param.name}`}
                  >
                    Delete
                  </Button>
                </div>
              </div>
            ))}
          </div>
        )}
      </div>

      {(isAddingParam || editingParam) && (
        <ParameterEditor
          parameter={editingParam}
          onSave={handleSaveParameter}
          onCancel={() => {
            setEditingParam(null);
            setIsAddingParam(false);
          }}
          existingIndices={parameters.map(p => p.index)}
        />
      )}

      {(isAddingDerived || editingDerived) && (
        <DerivedParameterEditor
          parameter={editingDerived}
          onSave={handleSaveDerivedParameter}
          onCancel={() => {
            setEditingDerived(null);
            setIsAddingDerived(false);
          }}
          existingIndices={derivedParameters.map(p => p.index)}
          availableParameters={parameters}
        />
      )}
    </div>
  );
}

/** Whether a derived parameter's formula reads the parameter at `index`. */
function dependsOn(derived: DerivedParameter, index: number): boolean {
  const formula = derived.formula;
  if (formula.type === 'Linear') return formula.value.base_param === index;
  if (formula.type === 'Sum' || formula.type === 'Product') {
    return formula.value.params.includes(index);
  }
  if (formula.type === 'Difference') {
    return formula.value.param_a === index || formula.value.param_b === index;
  }
  return false;
}
