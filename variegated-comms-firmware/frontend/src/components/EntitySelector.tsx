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
        return 'Water Tap';
      case 'steam_wand':
        return 'Steam Wand';
    }
  };

  const count = getEntityCount();
  const displayLabel = label || getDefaultLabel();

  // Single entity: show read-only display
  if (count === 1) {
    return (
      <div style={{ marginBottom: '1.5rem' }}>
        <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
          {displayLabel}
        </label>
        <div style={{
          padding: '0.5rem',
          border: '1px solid #e0e0e0',
          borderRadius: '4px',
          backgroundColor: '#f5f5f5',
          fontSize: '1rem',
          color: '#666'
        }}>
          {getEntityName(0)}
        </div>
      </div>
    );
  }

  // Multiple entities: show dropdown
  return (
    <div style={{ marginBottom: '1.5rem' }}>
      <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
        {displayLabel}
      </label>
      <select
        value={index}
        onChange={(e) => onChange(parseInt(e.currentTarget.value))}
        style={{
          width: '100%',
          padding: '0.5rem',
          border: '1px solid #ccc',
          borderRadius: '4px',
          fontSize: '1rem'
        }}
      >
        {Array.from({ length: count }, (_, i) => (
          <option key={i} value={i}>
            {i}: {getEntityName(i)}
          </option>
        ))}
      </select>
    </div>
  );
}
