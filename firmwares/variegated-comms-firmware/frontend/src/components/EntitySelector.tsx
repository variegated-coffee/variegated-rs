import { Field, Select, tokens } from '@variegated-coffee/ui';
import { useMachine } from '../contexts/MachineContext';

export type EntityType = 'boiler' | 'group' | 'water_tap' | 'steam_wand';

interface EntitySelectorProps {
  entityType: EntityType;
  index: number;
  onChange: (index: number) => void;
  label?: string;
}

export function EntitySelector({ entityType, index, onChange, label }: EntitySelectorProps) {
  const machine = useMachine();

  const getEntityName = (idx: number): string => {
    switch (entityType) {
      case 'boiler':
        return machine.getBoilerName(idx);
      case 'group':
        return machine.getGroupName(idx);
      case 'water_tap':
        return machine.getWaterTapName(idx);
      case 'steam_wand':
        return machine.getSteamWandName(idx);
    }
  };

  const getEntityCount = (): number => {
    switch (entityType) {
      case 'boiler':
        return machine.getBoilerCount();
      case 'group':
        return machine.getGroupCount();
      case 'water_tap':
        return machine.getWaterTapCount();
      case 'steam_wand':
        return machine.getSteamWandCount();
    }
  };

  const getDefaultLabel = (): string => {
    switch (entityType) {
      case 'boiler':
        return 'Boiler';
      case 'group':
        return 'Group';
      case 'water_tap':
        return 'Water tap';
      case 'steam_wand':
        return 'Steam wand';
    }
  };

  const count = getEntityCount();
  const displayLabel = label || getDefaultLabel();

  // One entity: there is nothing to choose, so it is stated rather than offered. A select
  // with a single option is a control that looks like a decision and is not one.
  if (count === 1) {
    return (
      <Field label={displayLabel}>
        {(control) => (
          <div
            {...control}
            style={{
              width: '100%',
              padding: `0.4rem ${tokens.space.sm}`,
              border: `1px solid ${tokens.color.border}`,
              borderRadius: tokens.radius.sm,
              backgroundColor: tokens.color.surface,
              font: `0.9rem ${tokens.font.sans}`,
              color: tokens.color.inkMuted,
            }}
          >
            {getEntityName(0)}
          </div>
        )}
      </Field>
    );
  }

  return (
    <Field label={displayLabel}>
      {(control) => (
        <Select
          {...control}
          value={String(index)}
          onChange={(value) => onChange(Number.parseInt(value, 10))}
          // The bare index is dropped from the label. It was "0: Brew boiler", which put
          // an array position in front of a name the machine definition already gives.
          options={Array.from({ length: count }, (_, i) => ({
            value: String(i),
            label: getEntityName(i),
          }))}
        />
      )}
    </Field>
  );
}
