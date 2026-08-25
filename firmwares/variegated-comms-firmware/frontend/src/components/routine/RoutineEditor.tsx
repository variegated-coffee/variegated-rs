import { useId, useState } from 'preact/hooks';
import {
  Button,
  Field,
  Select,
  Tabs,
  TextInput,
  tokens,
  useDialogs,
} from '@variegated-coffee/ui';
import { Routine, RoutineParameter, DerivedParameter, RoutineStep, RoutineCommand, RoutinePrerequisite, ShotAnnotation } from '../../schemas/schemas';
import { ParametersTab, MAX_PARAMETERS } from './ParametersTab';
import { StepsTab } from './StepsTab';
import { FinallyTab } from './FinallyTab';
import { ContextTab } from './ContextTab';
import { ROUTINE_FORMAT_VERSION } from '../../utils/routineHelpers';

interface RoutineEditorProps {
  routine: Routine | null;
  onSave: (routine: Routine) => void | Promise<void>;
  onCancel: () => void;
  functionSlotConfig?: {
    index: number;
    onChange: (index: number) => void;
    availableSlots: { [key: string]: string };
  };
}

type TabType = 'parameters' | 'steps' | 'finally' | 'context';

const TABS: { id: TabType; label: string }[] = [
  { id: 'parameters', label: 'Parameters' },
  { id: 'steps', label: 'Steps' },
  { id: 'finally', label: 'Finally' },
  { id: 'context', label: 'Context' },
];

export function RoutineEditor({ routine, onSave, onCancel, functionSlotConfig }: RoutineEditorProps) {
  const { notify } = useDialogs();
  const [activeTab, setActiveTab] = useState<TabType>('parameters');
  const [name, setName] = useState(routine?.name || '');
  const [parameters, setParameters] = useState<RoutineParameter[]>(routine?.parameters || []);
  const [derivedParameters, setDerivedParameters] = useState<DerivedParameter[]>(routine?.derived_parameters || []);
  const [steps, setSteps] = useState<RoutineStep[]>(routine?.steps || []);
  const [finallyCommands, setFinallyCommands] = useState<RoutineCommand[]>(routine?.finally || []);
  const [prerequisites, setPrerequisites] = useState<RoutinePrerequisite[]>(routine?.prerequisites || []);
  const [shotAnnotations, setShotAnnotations] = useState<ShotAnnotation[]>(routine?.shot_annotations || []);
  const titleId = useId();

  const handleSave = () => {
    if (!name.trim()) {
      void notify({ title: 'A routine needs a name', body: 'Give it one before saving.' });
      return;
    }

    if (steps.length === 0) {
      void notify({
        title: 'A routine needs at least one step',
        body: 'Add a step on the Steps tab before saving.',
      });
      return;
    }

    void onSave({
      // Always the version this bundle was built against, never the one the routine was
      // loaded with. Saving is a write in the current format; carrying an older number
      // forward would be claiming the machine can read something it has just refused.
      version: ROUTINE_FORMAT_VERSION,
      routine_type: routine?.routine_type || { type: 'UserDefined' },
      name: name.trim(),
      parameters,
      derived_parameters: derivedParameters,
      steps,
      finally: finallyCommands,
      prerequisites,
      shot_annotations: shotAnnotations
    });
  };

  /**
   * What each tab's badge counts.
   *
   * The Parameters tab counted `parameters + derived` while the heading inside it counted
   * only `parameters`, against a different ceiling -- so the tab read *Parameters (2)* and
   * the heading four pixels below read *Parameters (1/8)*. It counts the same thing the
   * tab is named after now, against the same ceiling, and the derived ones have their own
   * heading inside.
   */
  const tabCount = (tab: TabType): { count: number; max?: number } => {
    switch (tab) {
      case 'parameters':
        return { count: parameters.length, max: MAX_PARAMETERS };
      case 'steps':
        return { count: steps.length };
      case 'finally':
        return { count: finallyCommands.length };
      case 'context':
        return { count: prerequisites.length + shotAnnotations.length };
    }
  };

  return (
    <div style={{
      position: 'fixed',
      inset: 0,
      backgroundColor: 'rgba(0,0,0,0.5)',
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      zIndex: 1000,
      padding: tokens.space.md
    }}>
      {/* Not `Dialog`: that one owns its own scrolling body and footer, and this editor is
          a full-height three-part layout whose middle section scrolls independently. It
          carries the same semantics by hand rather than fighting the shell. */}
      <div
        role="dialog"
        aria-modal="true"
        aria-labelledby={titleId}
        style={{
          backgroundColor: tokens.color.surfaceRaised,
          borderRadius: tokens.radius.md,
          maxWidth: '1200px',
          width: '100%',
          maxHeight: '90vh',
          display: 'flex',
          flexDirection: 'column',
          overflow: 'hidden'
        }}
      >
        <div style={{ padding: tokens.space.lg, borderBottom: `1px solid ${tokens.color.border}`, display: 'flex', flexDirection: 'column', gap: tokens.space.md }}>
          <h2 id={titleId} style={{ margin: 0 }}>
            {routine ? 'Edit routine' : 'New routine'}
          </h2>

          {functionSlotConfig && (
            <Field
              label="Function routine slot"
              help="Saving replaces whatever routine is in this slot."
            >
              {(control) => (
                <Select
                  {...control}
                  value={String(functionSlotConfig.index)}
                  onChange={(value) => functionSlotConfig.onChange(Number.parseInt(value, 10))}
                  options={Object.entries(functionSlotConfig.availableSlots).map(([index, slotName]) => ({
                    value: index,
                    label: `Slot ${index}: ${slotName}`,
                  }))}
                />
              )}
            </Field>
          )}

          {/* The routine's name had a placeholder and nothing else -- no label, and so no
              accessible name at all. A placeholder is not a label: it disappears the moment
              anything is typed, which is exactly when a user might want to check what the
              field is. */}
          <Field label="Routine name" required>
            {(control) => (
              <TextInput
                {...control}
                value={name}
                onInput={setName}
                placeholder="Turbo shot, Lungo, …"
              />
            )}
          </Field>
        </div>

        {/* The type argument is explicit rather than inferred. `variegated-ds`, which
            builds the design-system declarations, compiles with `strict: false`, and
            inference there widens `Id` to `string` — so the same call that checks here
            fails in that build. Pinning it makes both agree. */}
        <Tabs<TabType>
          label="Routine sections"
          active={activeTab}
          onChange={setActiveTab}
          tabs={TABS.map((tab) => {
            const { count, max } = tabCount(tab.id);
            return {
              id: tab.id,
              label: tab.label,
              badge: max === undefined ? count : `${count}/${max}`,
            };
          })}
        >
          {activeTab === 'parameters' && (
            <ParametersTab
              parameters={parameters}
              derivedParameters={derivedParameters}
              onParametersChange={setParameters}
              onDerivedParametersChange={setDerivedParameters}
            />
          )}
          {activeTab === 'steps' && (
            <StepsTab
              steps={steps}
              onStepsChange={setSteps}
              parameters={parameters}
              derivedParameters={derivedParameters}
            />
          )}
          {activeTab === 'finally' && (
            <FinallyTab
              finallyCommands={finallyCommands}
              onFinallyCommandsChange={setFinallyCommands}
              parameters={parameters}
              derivedParameters={derivedParameters}
            />
          )}
          {activeTab === 'context' && (
            <ContextTab
              prerequisites={prerequisites}
              shotAnnotations={shotAnnotations}
              parameters={parameters}
              onPrerequisitesChange={setPrerequisites}
              onShotAnnotationsChange={setShotAnnotations}
            />
          )}
        </Tabs>

        <div style={{
          padding: `${tokens.space.md} ${tokens.space.lg}`,
          borderTop: `1px solid ${tokens.color.border}`,
          display: 'flex',
          gap: tokens.space.sm,
          justifyContent: 'flex-end',
          backgroundColor: tokens.color.surfaceSunken
        }}>
          <Button variant="secondary" onClick={onCancel}>
            Cancel
          </Button>
          <Button
            variant="primary"
            onClick={handleSave}
            disabled={!name.trim() || steps.length === 0}
          >
            Save routine
          </Button>
        </div>
      </div>
    </div>
  );
}

