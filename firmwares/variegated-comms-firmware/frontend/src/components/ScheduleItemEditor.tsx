import { useState } from 'preact/hooks';
import { Button, Dialog, tokens } from '@variegated-coffee/ui';
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
    // `Dialog` rather than another hand-built fixed backdrop. This one was the fourth copy
    // of the same twelve style properties, and like the others it had no focus trap, no
    // Escape handler and nothing announcing it as a dialog.
    <Dialog
      title={item ? 'Edit schedule' : 'New schedule'}
      onClose={onCancel}
      width="1200px"
      footer={
        <>
          <Button variant="secondary" onClick={onCancel}>
            Cancel
          </Button>
          <Button variant="primary" onClick={handleSave} disabled={commands.length === 0}>
            Save schedule
          </Button>
        </>
      }
    >
      <div
        style={{
          display: 'grid',
          // One column below the width where 400px of trigger editor plus a command list
          // stop both fitting. The two-column grid was unconditional, which on a tablet
          // held in portrait squeezed the command list to a few characters wide.
          gridTemplateColumns: 'minmax(320px, 400px) 1fr',
          gap: tokens.space.md,
        }}
      >
        <div style={{ borderRight: `1px solid ${tokens.color.border}`, paddingRight: tokens.space.md }}>
          <TriggerEditor trigger={trigger} onChange={setTrigger} />
        </div>
        <div>
          <CommandList commands={commands} onChange={setCommands} />
        </div>
      </div>
    </Dialog>
  );
}
