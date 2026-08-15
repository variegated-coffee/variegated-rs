import { useState } from 'preact/hooks';
import { ScheduleItem, ScheduleTrigger, ScheduleAction } from '../schemas/schemas';
import { TriggerEditor } from './TriggerEditor';
import { CommandList } from './CommandList';

interface ScheduleItemEditorProps {
  item: ScheduleItem | null;
  onSave: (item: ScheduleItem) => void;
  onCancel: () => void;
}

export function ScheduleItemEditor({ item, onSave, onCancel }: ScheduleItemEditorProps) {
  const [trigger, setTrigger] = useState<ScheduleTrigger>(
    item?.trigger_at || {
      on_hour: 7,
      on_minute: 0,
      on_days: null,
      on_date: null,
      enabled: true,
      once: false
    }
  );

  const [commands, setCommands] = useState<ScheduleAction[]>(item?.commands || []);

  const handleSave = () => {
    onSave({
      trigger_at: trigger,
      commands
    });
  };

  return (
    <div style={{
      position: 'fixed',
      top: 0,
      left: 0,
      right: 0,
      bottom: 0,
      backgroundColor: 'rgba(0,0,0,0.5)',
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      zIndex: 999,
      padding: '1rem'
    }}>
      <div style={{
        backgroundColor: 'white',
        borderRadius: '8px',
        maxWidth: '1200px',
        width: '100%',
        maxHeight: '90vh',
        display: 'flex',
        flexDirection: 'column',
        overflow: 'hidden'
      }}>
        {/* Header */}
        <div style={{
          padding: '1.5rem',
          borderBottom: '1px solid #ddd'
        }}>
          <h2 style={{ margin: 0 }}>
            {item ? 'Edit Schedule' : 'New Schedule'}
          </h2>
        </div>

        {/* Content */}
        <div style={{
          flex: 1,
          display: 'grid',
          gridTemplateColumns: '400px 1fr',
          overflow: 'hidden'
        }}>
          {/* Left: Trigger Editor */}
          <div style={{
            borderRight: '1px solid #ddd',
            overflow: 'auto'
          }}>
            <TriggerEditor trigger={trigger} onChange={setTrigger} />
          </div>

          {/* Right: Commands List */}
          <div style={{ overflow: 'auto' }}>
            <CommandList commands={commands} onChange={setCommands} />
          </div>
        </div>

        {/* Footer */}
        <div style={{
          padding: '1rem 1.5rem',
          borderTop: '1px solid #ddd',
          display: 'flex',
          gap: '1rem',
          justifyContent: 'flex-end'
        }}>
          <button
            onClick={onCancel}
            style={{
              padding: '0.75rem 1.5rem',
              backgroundColor: '#666',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              fontSize: '1rem',
              cursor: 'pointer'
            }}
          >
            Cancel
          </button>
          <button
            onClick={handleSave}
            disabled={commands.length === 0}
            style={{
              padding: '0.75rem 1.5rem',
              backgroundColor: commands.length === 0 ? '#ccc' : '#0066cc',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              fontSize: '1rem',
              cursor: commands.length === 0 ? 'not-allowed' : 'pointer'
            }}
          >
            Save Schedule
          </button>
        </div>
      </div>
    </div>
  );
}
