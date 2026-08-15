import { useState, useEffect } from 'preact/hooks';
import { memo } from 'preact/compat';
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
  const [schedules, setSchedules] = useState<ScheduleItem[]>(initialSchedules);
  const [editingIndex, setEditingIndex] = useState<number | null>(null);
  const [isAdding, setIsAdding] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Sync with prop changes (from external updates)
  useEffect(() => {
    setSchedules(initialSchedules);
  }, [initialSchedules]);

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

  const handleDelete = (index: number) => {
    if (!confirm('Are you sure you want to delete this schedule?')) {
      return;
    }

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
    const updatedItem = {
      ...newSchedules[index],
      trigger_at: {
        ...newSchedules[index].trigger_at,
        enabled: !newSchedules[index].trigger_at.enabled
      }
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
    if (schedules.length >= MAX_SCHEDULES) {
      alert('Maximum number of schedules reached (64)');
      return;
    }

    setError(null);
    try {
      const itemToDuplicate = { ...schedules[index] };
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

  const handleOptimizeStorage = () => {
    setError(null);
    const ws = getWebSocketService();
    if (!ws) {
      setError('WebSocket not connected');
      return;
    }
    ws.optimizeScheduleStorage();

    // Show temporary success message
    const successDiv = document.createElement('div');
    successDiv.textContent = 'Storage optimized';
    successDiv.style.cssText = 'position: fixed; top: 20px; right: 20px; background: #d4edda; color: #155724; padding: 1rem; border-radius: 4px; z-index: 9999;';
    document.body.appendChild(successDiv);
    setTimeout(() => successDiv.remove(), 2000);
  };

  return (
    <div style={{ padding: '1rem' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1rem' }}>
        <h2 style={{ fontSize: '1.5rem', margin: 0 }}>
          Schedules ({schedules.length}/{MAX_SCHEDULES})
        </h2>
        <button
          onClick={() => setIsAdding(true)}
          disabled={schedules.length >= MAX_SCHEDULES}
          style={{
            padding: '0.75rem 1.5rem',
            backgroundColor: schedules.length >= MAX_SCHEDULES ? '#ccc' : '#0066cc',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            fontSize: '1rem',
            cursor: schedules.length >= MAX_SCHEDULES ? 'not-allowed' : 'pointer'
          }}
        >
          + Add Schedule
        </button>
      </div>

      {error && (
        <div style={{
          padding: '0.75rem',
          marginBottom: '1rem',
          backgroundColor: '#f8d7da',
          border: '1px solid #f5c6cb',
          borderRadius: '4px',
          color: '#721c24',
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center'
        }}>
          <span>{error}</span>
          <button
            onClick={() => setError(null)}
            style={{
              background: 'none',
              border: 'none',
              color: '#721c24',
              fontSize: '1.2rem',
              cursor: 'pointer',
              padding: '0 0.5rem'
            }}
          >
            ×
          </button>
        </div>
      )}

      {schedules.length >= 60 && (
        <div style={{
          padding: '0.75rem',
          marginBottom: '1rem',
          backgroundColor: '#fff3cd',
          border: '1px solid #ffc107',
          borderRadius: '4px',
          color: '#856404'
        }}>
          Warning: Approaching maximum schedule limit ({schedules.length}/64)
        </div>
      )}

      {schedules.length === 0 ? (
        <div style={{
          padding: '3rem',
          textAlign: 'center',
          color: '#666',
          border: '2px dashed #ccc',
          borderRadius: '8px'
        }}>
          <p style={{ fontSize: '1.1rem', marginBottom: '0.5rem' }}>No schedules configured</p>
          <p style={{ fontSize: '0.9rem', margin: 0 }}>Click "Add Schedule" to create your first schedule</p>
        </div>
      ) : (
        <div style={{ display: 'flex', flexDirection: 'column', gap: '0.75rem' }}>
          {schedules.map((item, index) => (
            <div
              key={index}
              style={{
                padding: '1rem',
                backgroundColor: 'white',
                border: '1px solid #ddd',
                borderRadius: '8px',
                opacity: item.trigger_at.enabled ? 1 : 0.6
              }}
            >
              <div style={{ display: 'flex', alignItems: 'center', gap: '1rem' }}>
                <div style={{ flex: 1 }}>
                  <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', marginBottom: '0.5rem' }}>
                    <span style={{
                      padding: '0.25rem 0.5rem',
                      backgroundColor: item.trigger_at.enabled ? '#28a745' : '#6c757d',
                      color: 'white',
                      borderRadius: '4px',
                      fontSize: '0.75rem',
                      fontWeight: '500'
                    }}>
                      {item.trigger_at.enabled ? 'ENABLED' : 'DISABLED'}
                    </span>
                    <span style={{ fontWeight: '500', fontSize: '1.1rem' }}>
                      {getTriggerSummary(item)}
                    </span>
                  </div>
                  <div style={{ fontSize: '0.9rem', color: '#666' }}>
                    {formatScheduleActionsSummary(item.commands, getBoilerName)}
                  </div>
                </div>

                <div style={{ display: 'flex', gap: '0.5rem' }}>
                  <button
                    onClick={() => void handleToggleEnabled(index)}
                    title={item.trigger_at.enabled ? 'Disable' : 'Enable'}
                    style={{
                      padding: '0.5rem 0.75rem',
                      backgroundColor: 'white',
                      border: '1px solid #ccc',
                      borderRadius: '4px',
                      cursor: 'pointer',
                      fontSize: '0.9rem'
                    }}
                  >
                    {item.trigger_at.enabled ? '⏸' : '▶'}
                  </button>
                  <button
                    onClick={() => setEditingIndex(index)}
                    title="Edit"
                    style={{
                      padding: '0.5rem 0.75rem',
                      backgroundColor: 'white',
                      border: '1px solid #ccc',
                      borderRadius: '4px',
                      cursor: 'pointer',
                      fontSize: '0.9rem'
                    }}
                  >
                    Edit
                  </button>
                  <button
                    onClick={() => void handleDuplicate(index)}
                    title="Duplicate"
                    disabled={schedules.length >= MAX_SCHEDULES}
                    style={{
                      padding: '0.5rem 0.75rem',
                      backgroundColor: 'white',
                      border: '1px solid #ccc',
                      borderRadius: '4px',
                      cursor: schedules.length >= MAX_SCHEDULES ? 'not-allowed' : 'pointer',
                      fontSize: '0.9rem',
                      opacity: schedules.length >= MAX_SCHEDULES ? 0.5 : 1
                    }}
                  >
                    Copy
                  </button>
                  <button
                    onClick={() => void handleDelete(index)}
                    title="Delete"
                    style={{
                      padding: '0.5rem 0.75rem',
                      backgroundColor: '#dc3545',
                      color: 'white',
                      border: 'none',
                      borderRadius: '4px',
                      cursor: 'pointer',
                      fontSize: '0.9rem'
                    }}
                  >
                    Delete
                  </button>
                </div>
              </div>
            </div>
          ))}
        </div>
      )}

      {/* Storage Optimization - Housekeeping */}
      {schedules.length > 0 && (
        <div style={{
          marginTop: '1.5rem',
          paddingTop: '1rem',
          borderTop: '1px solid #eee',
          display: 'flex',
          justifyContent: 'flex-end'
        }}>
          <button
            onClick={() => void handleOptimizeStorage()}
            style={{
              padding: '0.5rem 0.75rem',
              fontSize: '0.8rem',
              color: '#666',
              backgroundColor: 'transparent',
              border: '1px solid #ddd',
              borderRadius: '4px',
              cursor: 'pointer',
              display: 'flex',
              alignItems: 'center',
              gap: '0.5rem'
            }}
            onMouseEnter={(e) => { e.currentTarget.style.backgroundColor = '#f5f5f5'; }}
            onMouseLeave={(e) => { e.currentTarget.style.backgroundColor = 'transparent'; }}
          >
            Optimize Storage
          </button>
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
