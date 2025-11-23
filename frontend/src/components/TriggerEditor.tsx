import { ScheduleTrigger } from '../schemas/schemas';

interface TriggerEditorProps {
  trigger: ScheduleTrigger;
  onChange: (trigger: ScheduleTrigger) => void;
}

const WEEKDAYS = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'];

export function TriggerEditor({ trigger, onChange }: TriggerEditorProps) {
  const recurrenceType = trigger.on_date ? 'specific_date' : (trigger.on_days ? 'specific_days' : 'every_day');

  const handleRecurrenceChange = (type: string) => {
    if (type === 'every_day') {
      onChange({ ...trigger, on_days: null, on_date: null });
    } else if (type === 'specific_days') {
      onChange({ ...trigger, on_days: [], on_date: null });
    } else if (type === 'specific_date') {
      onChange({ ...trigger, on_days: null, on_date: new Date().toISOString().split('T')[0] });
    }
  };

  const handleDayToggle = (day: string) => {
    const days = trigger.on_days || [];
    const newDays = days.includes(day)
      ? days.filter(d => d !== day)
      : [...days, day];
    onChange({ ...trigger, on_days: newDays });
  };

  return (
    <div style={{ padding: '1rem' }}>
      <h3 style={{ marginBottom: '1rem', fontSize: '1.25rem' }}>Trigger Settings</h3>

      {/* Time */}
      <div style={{ marginBottom: '1.5rem' }}>
        <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>Time</label>
        <div style={{ display: 'flex', gap: '0.5rem', alignItems: 'center' }}>
          <input
            type="number"
            min="0"
            max="23"
            value={trigger.on_hour}
            onChange={(e) => onChange({ ...trigger, on_hour: parseInt(e.currentTarget.value) || 0 })}
            style={{
              width: '60px',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '1rem'
            }}
          />
          <span>:</span>
          <input
            type="number"
            min="0"
            max="59"
            value={trigger.on_minute}
            onChange={(e) => onChange({ ...trigger, on_minute: parseInt(e.currentTarget.value) || 0 })}
            style={{
              width: '60px',
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '1rem'
            }}
          />
        </div>
      </div>

      {/* Recurrence */}
      <div style={{ marginBottom: '1.5rem' }}>
        <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>Recurrence</label>
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
          <label style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
            <input
              type="radio"
              checked={recurrenceType === 'every_day'}
              onChange={() => handleRecurrenceChange('every_day')}
            />
            Every day
          </label>
          <label style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
            <input
              type="radio"
              checked={recurrenceType === 'specific_days'}
              onChange={() => handleRecurrenceChange('specific_days')}
            />
            Specific days
          </label>
          <label style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
            <input
              type="radio"
              checked={recurrenceType === 'specific_date'}
              onChange={() => handleRecurrenceChange('specific_date')}
            />
            Specific date
          </label>
        </div>
      </div>

      {/* Days of week (if specific days selected) */}
      {recurrenceType === 'specific_days' && (
        <div style={{ marginBottom: '1.5rem', marginLeft: '1.5rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500', fontSize: '0.9rem' }}>
            Select days
          </label>
          <div style={{ display: 'flex', flexWrap: 'wrap', gap: '0.5rem' }}>
            {WEEKDAYS.map(day => (
              <label
                key={day}
                style={{
                  display: 'flex',
                  alignItems: 'center',
                  gap: '0.25rem',
                  padding: '0.25rem 0.5rem',
                  border: '1px solid #ccc',
                  borderRadius: '4px',
                  cursor: 'pointer',
                  backgroundColor: trigger.on_days?.includes(day) ? '#e3f2fd' : 'white'
                }}
              >
                <input
                  type="checkbox"
                  checked={trigger.on_days?.includes(day) || false}
                  onChange={() => handleDayToggle(day)}
                />
                {day}
              </label>
            ))}
          </div>
        </div>
      )}

      {/* Specific date (if specific date selected) */}
      {recurrenceType === 'specific_date' && (
        <div style={{ marginBottom: '1.5rem', marginLeft: '1.5rem' }}>
          <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500', fontSize: '0.9rem' }}>
            Select date
          </label>
          <input
            type="date"
            value={trigger.on_date || ''}
            onChange={(e) => onChange({ ...trigger, on_date: e.currentTarget.value || null })}
            style={{
              padding: '0.5rem',
              border: '1px solid #ccc',
              borderRadius: '4px',
              fontSize: '1rem'
            }}
          />
        </div>
      )}

      {/* Options */}
      <div style={{ marginBottom: '1rem' }}>
        <label style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', marginBottom: '0.5rem' }}>
          <input
            type="checkbox"
            checked={trigger.once}
            onChange={(e) => onChange({ ...trigger, once: e.currentTarget.checked })}
          />
          Run once then delete
        </label>
        <label style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
          <input
            type="checkbox"
            checked={trigger.enabled}
            onChange={(e) => onChange({ ...trigger, enabled: e.currentTarget.checked })}
          />
          Enabled
        </label>
      </div>
    </div>
  );
}
