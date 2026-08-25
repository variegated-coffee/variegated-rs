import { useId } from 'preact/hooks';
import { Field, tokens } from '@variegated-coffee/ui';
import { ScheduleTrigger } from '../schemas/schemas';

interface TriggerEditorProps {
  trigger: ScheduleTrigger;
  onChange: (trigger: ScheduleTrigger) => void;
}

const WEEKDAYS = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun'];

/**
 * Clamp a typed time component into range.
 *
 * `parseInt(x) || 0` was the whole of the old handling, which turns an unparseable entry
 * into midnight silently -- and nothing stopped an hour of 47 reaching the firmware, since
 * `min`/`max` on a number input constrain the spinner but not what can be typed.
 */
function clampTime(raw: string, max: number): number {
  const parsed = Number.parseInt(raw, 10);
  if (Number.isNaN(parsed)) return 0;
  return Math.min(Math.max(parsed, 0), max);
}

export function TriggerEditor({ trigger, onChange }: TriggerEditorProps) {
  const recurrenceType = trigger.on_date ? 'specific_date' : (trigger.on_days ? 'specific_days' : 'every_day');
  // Shared across the three radios, which is what makes them one group. Without a `name`
  // they were three independent radios that merely looked like a set: arrow keys did not
  // move between them, and assistive technology announced each as its own control.
  const recurrenceName = useId();

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

  /*
   * `type="number"` here, deliberately, where the PID editor had to stop using it.
   *
   * The problem there was the decimal separator: a browser in a comma locale renders and
   * parses `1.4` as `1,4` in a number input, on a field the firmware then parses. An hour
   * and a minute are integers with a two-digit range, so there is no separator to localise
   * and no grouping to insert -- and the spinner and the numeric keypad it brings on a
   * tablet are worth having.
   */
  const timeInputStyle = {
    width: '4rem',
    padding: `0.4rem ${tokens.space.sm}`,
    font: `1rem ${tokens.font.mono}`,
    fontVariantNumeric: 'tabular-nums' as const,
    color: tokens.color.ink,
    border: `1px solid ${tokens.color.border}`,
    borderRadius: tokens.radius.sm,
  };

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.lg, padding: tokens.space.md }}>
      <h3 style={{ margin: 0, fontSize: '1.25rem' }}>When</h3>

      <div style={{ display: 'flex', gap: tokens.space.sm, alignItems: 'flex-end' }}>
        <Field label="Hour">
          {(control) => (
            <input
              {...control}
              type="number"
              min="0"
              max="23"
              value={trigger.on_hour}
              onChange={(e) => onChange({ ...trigger, on_hour: clampTime(e.currentTarget.value, 23) })}
              style={timeInputStyle}
            />
          )}
        </Field>
        <span style={{ paddingBottom: '0.5rem', color: tokens.color.inkMuted }}>:</span>
        <Field label="Minute">
          {(control) => (
            <input
              {...control}
              type="number"
              min="0"
              max="59"
              value={trigger.on_minute}
              onChange={(e) => onChange({ ...trigger, on_minute: clampTime(e.currentTarget.value, 59) })}
              style={timeInputStyle}
            />
          )}
        </Field>
      </div>

      {/* A fieldset, so the three radios announce as one question with three answers
          rather than as three unrelated controls. */}
      <fieldset style={{ margin: 0, padding: 0, border: 'none' }}>
        <legend
          style={{
            padding: 0,
            marginBottom: tokens.space.sm,
            font: `0.85rem ${tokens.font.sans}`,
            color: tokens.color.inkMuted,
          }}
        >
          Recurrence
        </legend>
        <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
          {[
            { value: 'every_day', label: 'Every day' },
            { value: 'specific_days', label: 'Specific days' },
            { value: 'specific_date', label: 'Specific date' },
          ].map((option) => (
            <label
              key={option.value}
              style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm, cursor: 'pointer' }}
            >
              <input
                type="radio"
                name={recurrenceName}
                value={option.value}
                checked={recurrenceType === option.value}
                onChange={() => handleRecurrenceChange(option.value)}
              />
              {option.label}
            </label>
          ))}
        </div>
      </fieldset>

      {recurrenceType === 'specific_days' && (
        <fieldset style={{ margin: 0, padding: 0, border: 'none', marginLeft: tokens.space.lg }}>
          <legend
            style={{
              padding: 0,
              marginBottom: tokens.space.sm,
              font: `0.85rem ${tokens.font.sans}`,
              color: tokens.color.inkMuted,
            }}
          >
            Days
          </legend>
          <div style={{ display: 'flex', flexWrap: 'wrap', gap: tokens.space.sm }}>
            {WEEKDAYS.map(day => {
              const selected = trigger.on_days?.includes(day) ?? false;
              return (
                <label
                  key={day}
                  style={{
                    display: 'flex',
                    alignItems: 'center',
                    gap: tokens.space.xs,
                    padding: `${tokens.space.xs} ${tokens.space.sm}`,
                    border: `1px solid ${selected ? tokens.color.info : tokens.color.border}`,
                    borderRadius: tokens.radius.sm,
                    cursor: 'pointer',
                    backgroundColor: selected ? tokens.color.infoSurface : tokens.color.surfaceRaised,
                  }}
                >
                  <input
                    type="checkbox"
                    checked={selected}
                    onChange={() => handleDayToggle(day)}
                  />
                  {day}
                </label>
              );
            })}
          </div>
        </fieldset>
      )}

      {recurrenceType === 'specific_date' && (
        <div style={{ marginLeft: tokens.space.lg }}>
          <Field label="Date">
            {(control) => (
              <input
                {...control}
                type="date"
                value={trigger.on_date || ''}
                onChange={(e) => onChange({ ...trigger, on_date: e.currentTarget.value || null })}
                style={{
                  padding: `0.4rem ${tokens.space.sm}`,
                  font: `1rem ${tokens.font.sans}`,
                  color: tokens.color.ink,
                  border: `1px solid ${tokens.color.border}`,
                  borderRadius: tokens.radius.sm,
                }}
              />
            )}
          </Field>
        </div>
      )}

      <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
        <label style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm, cursor: 'pointer' }}>
          <input
            type="checkbox"
            checked={trigger.once}
            onChange={(e) => onChange({ ...trigger, once: e.currentTarget.checked })}
          />
          Run once, then delete this schedule
        </label>
        <label style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm, cursor: 'pointer' }}>
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
