import { useState, useEffect } from 'preact/hooks';
import { memo } from 'preact/compat';
import { Alert, Badge, Button, EmptyState, tokens, useDialogs } from '@variegated-coffee/ui';
import { ScheduleItem } from '../schemas/schemas';
import { ScheduleItemEditor } from './ScheduleItemEditor';
import { useMachine } from '../contexts/MachineContext';
import { formatScheduleActionsSummary } from '../utils/commandFormatter';
import * as schedulesApi from '../api/schedules';
import { getWebSocketService } from '../services/websocket';

interface ScheduleBuilderProps {
  schedules: ScheduleItem[];
}

const MAX_SCHEDULES = 64;

/** Within this many of the ceiling, the remaining headroom is worth saying out loud. */
const CROWDED_AT = 60;

function getTriggerSummary(item: ScheduleItem): string {
  const trigger = item.trigger_at;
  const time = `${String(trigger.on_hour).padStart(2, '0')}:${String(trigger.on_minute).padStart(2, '0')}`;

  let recurrence = '';
  if (trigger.on_date) {
    recurrence = ` on ${trigger.on_date}`;
  } else if (trigger.on_days && trigger.on_days.length > 0) {
    recurrence = ` on ${trigger.on_days.join(', ')}`;
  } else {
    recurrence = ' every day';
  }

  const suffix = trigger.once ? ' (once)' : '';

  return `${time}${recurrence}${suffix}`;
}

const ScheduleBuilderComponent = ({ schedules: initialSchedules }: ScheduleBuilderProps) => {
  const { getBoilerName } = useMachine();
  const { confirm, notify } = useDialogs();
  const [schedules, setSchedules] = useState<ScheduleItem[]>(initialSchedules);
  const [editingIndex, setEditingIndex] = useState<number | null>(null);
  const [isAdding, setIsAdding] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [notice, setNotice] = useState<string | null>(null);

  // Sync with prop changes (from external updates)
  useEffect(() => {
    setSchedules(initialSchedules);
  }, [initialSchedules]);

  const atCapacity = schedules.length >= MAX_SCHEDULES;

  const handleAdd = (item: ScheduleItem) => {
    setError(null);
    try {
      schedulesApi.addSchedule(item);
      // Optimistically update local state
      setSchedules([...schedules, item]);
      setIsAdding(false);
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Failed to add schedule';
      setError(errorMsg);
      console.error('Failed to add schedule:', err);
    }
  };

  const handleEdit = (index: number, item: ScheduleItem) => {
    setError(null);
    try {
      schedulesApi.updateSchedule(index, item);
      // Optimistically update local state
      const newSchedules = [...schedules];
      newSchedules[index] = item;
      setSchedules(newSchedules);
      setEditingIndex(null);
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Failed to update schedule';
      setError(errorMsg);
      console.error('Failed to update schedule:', err);
    }
  };

  /**
   * Delete, after asking -- and the prompt names the schedule.
   *
   * "Are you sure you want to delete this schedule?" told the user nothing they did not
   * already know. Naming it is what lets them notice they clicked the wrong row, which is
   * the only thing a confirmation is actually for.
   */
  const handleDelete = async (index: number) => {
    const item = schedules[index];
    if (!item) return;

    const ok = await confirm({
      title: `Delete the ${getTriggerSummary(item).trim()} schedule?`,
      body: 'This cannot be undone.',
      confirmLabel: 'Delete',
      destructive: true,
    });
    if (!ok) return;

    setError(null);
    try {
      schedulesApi.deleteSchedule(index);
      // Optimistically update local state
      const newSchedules = schedules.filter((_, i) => i !== index);
      setSchedules(newSchedules);
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Failed to delete schedule';
      setError(errorMsg);
      console.error('Failed to delete schedule:', err);
    }
  };

  const handleToggleEnabled = (index: number) => {
    const newSchedules = [...schedules];
    const current = newSchedules[index];
    if (!current) return;

    const updatedItem = {
      ...current,
      trigger_at: {
        ...current.trigger_at,
        enabled: !current.trigger_at.enabled,
      },
    };

    setError(null);
    try {
      schedulesApi.updateSchedule(index, updatedItem);
      // Optimistically update local state
      newSchedules[index] = updatedItem;
      setSchedules(newSchedules);
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Failed to toggle schedule';
      setError(errorMsg);
      console.error('Failed to toggle schedule:', err);
    }
  };

  const handleDuplicate = (index: number) => {
    if (atCapacity) {
      void notify({
        title: 'No room for another schedule',
        body: `This machine stores at most ${MAX_SCHEDULES}. Delete one to make room.`,
      });
      return;
    }

    setError(null);
    try {
      const source = schedules[index];
      if (!source) return;
      const itemToDuplicate = { ...source };
      schedulesApi.addSchedule(itemToDuplicate);
      // Optimistically update local state
      const newSchedules = [...schedules, itemToDuplicate];
      setSchedules(newSchedules);
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Failed to duplicate schedule';
      setError(errorMsg);
      console.error('Failed to duplicate schedule:', err);
    }
  };

  /**
   * Compact the schedule store on the machine.
   *
   * The button used to be an unexplained *Optimize Storage* floating at the bottom right
   * with no indication of what it would do, and its confirmation was a `<div>` appended
   * straight to `document.body` with a `position: fixed` inline style -- outside the app's
   * tree, unstyleable, and invisible to anything that was not looking at that corner.
   */
  const handleOptimizeStorage = () => {
    setError(null);
    const ws = getWebSocketService();
    if (!ws) {
      setError('Not connected to the machine');
      return;
    }
    ws.optimizeScheduleStorage();
    setNotice('Compaction sent');
    setTimeout(() => setNotice(null), 3000);
  };

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.md, padding: tokens.space.md }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: tokens.space.sm, flexWrap: 'wrap' }}>
        {/* The capacity is an annotation beside the heading, not part of its name. It was
            "Schedules (2/64)" -- a firmware storage ceiling as the title of the screen. */}
        <div style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm }}>
          <h2 style={{ fontSize: '1.5rem', margin: 0 }}>Schedules</h2>
          <Badge numeric role={schedules.length >= CROWDED_AT ? 'warn' : undefined}>
            {schedules.length}/{MAX_SCHEDULES}
          </Badge>
        </div>
        <Button variant="primary" onClick={() => setIsAdding(true)} disabled={atCapacity}>
          Add schedule
        </Button>
      </div>

      {error && (
        <Alert role="danger" onDismiss={() => setError(null)}>
          {error}
        </Alert>
      )}

      {notice && <Alert role="ok">{notice}</Alert>}

      {schedules.length >= CROWDED_AT && (
        <Alert role="warn">
          {MAX_SCHEDULES - schedules.length} of {MAX_SCHEDULES} schedule slots left.
        </Alert>
      )}

      {schedules.length === 0 ? (
        <EmptyState
          title="No schedules configured"
          detail="A schedule turns the machine on, off or into power save at a time you choose."
          action={{ label: 'Add schedule', onClick: () => setIsAdding(true) }}
        />
      ) : (
        <div style={{ display: 'flex', flexDirection: 'column', gap: tokens.space.sm }}>
          {schedules.map((item, index) => (
            <div
              key={index}
              style={{
                padding: tokens.space.md,
                backgroundColor: tokens.color.surfaceRaised,
                border: `1px solid ${tokens.color.border}`,
                borderRadius: tokens.radius.md,
                opacity: item.trigger_at.enabled ? 1 : 0.6,
              }}
            >
              <div style={{ display: 'flex', alignItems: 'center', gap: tokens.space.md, flexWrap: 'wrap' }}>
                <div style={{ flex: 1, minWidth: '14rem' }}>
                  <div style={{ display: 'flex', alignItems: 'center', gap: tokens.space.sm, marginBottom: tokens.space.sm }}>
                    <Badge role={item.trigger_at.enabled ? 'ok' : undefined}>
                      {item.trigger_at.enabled ? 'Enabled' : 'Disabled'}
                    </Badge>
                    <span
                      style={{
                        fontWeight: 500,
                        fontSize: '1.1rem',
                        fontFamily: tokens.font.mono,
                        fontVariantNumeric: 'tabular-nums',
                      }}
                    >
                      {getTriggerSummary(item)}
                    </span>
                  </div>
                  <div style={{ fontSize: '0.9rem', color: tokens.color.inkMuted }}>
                    {formatScheduleActionsSummary(item.commands, getBoilerName)}
                  </div>
                </div>

                <div style={{ display: 'flex', gap: tokens.space.sm, flexWrap: 'wrap' }}>
                  {/* Labelled, like the three beside it. This was a bare `⏸`/`▶` glyph
                      whose only explanation was a `title` tooltip -- which does not exist
                      on a touchscreen, and this machine is driven from one. */}
                  <Button variant="secondary" size="sm" onClick={() => void handleToggleEnabled(index)}>
                    {item.trigger_at.enabled ? 'Disable' : 'Enable'}
                  </Button>
                  <Button variant="secondary" size="sm" onClick={() => setEditingIndex(index)}>
                    Edit
                  </Button>
                  <Button
                    variant="secondary"
                    size="sm"
                    onClick={() => void handleDuplicate(index)}
                    disabled={atCapacity}
                  >
                    Duplicate
                  </Button>
                  <Button variant="destructive" size="sm" onClick={() => void handleDelete(index)}>
                    Delete
                  </Button>
                </div>
              </div>
            </div>
          ))}
        </div>
      )}

      {schedules.length > 0 && (
        <div
          style={{
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between',
            gap: tokens.space.sm,
            flexWrap: 'wrap',
            paddingTop: tokens.space.sm,
            borderTop: `1px solid ${tokens.color.border}`,
          }}
        >
          <span style={{ fontSize: '0.8rem', color: tokens.color.inkMuted, maxWidth: '48ch' }}>
            Deleting schedules leaves gaps in the machine's storage. Compacting reclaims
            them; it does not change any schedule.
          </span>
          <Button variant="quiet" size="sm" onClick={() => void handleOptimizeStorage()}>
            Compact storage
          </Button>
        </div>
      )}

      {isAdding && (
        <ScheduleItemEditor
          item={null}
          onSave={(item) => void handleAdd(item)}
          onCancel={() => setIsAdding(false)}
        />
      )}

      {editingIndex !== null && (
        <ScheduleItemEditor
          item={schedules[editingIndex]}
          onSave={(item) => void handleEdit(editingIndex, item)}
          onCancel={() => setEditingIndex(null)}
        />
      )}
    </div>
  );
};

export const ScheduleBuilder = memo(ScheduleBuilderComponent);
