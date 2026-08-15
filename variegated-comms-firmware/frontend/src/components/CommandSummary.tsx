import { ScheduleAction } from '../schemas/schemas';
import { useMachine } from '../contexts/MachineContext';

export function getCommandSummary(
  command: ScheduleAction,
  getBoilerName?: (index: number) => string
): string {
  // Helper function with fallback
  const boilerName = (idx: number) => getBoilerName ? getBoilerName(idx) : `boiler ${idx}`;

  // Handle CancelRoutine unit variant
  if (command.type === 'CancelRoutine') {
    return 'Cancel routine';
  }

  // Handle object variants
  if (command.type === 'SetBoilerControlTarget') {
    const [index, mode, values] = command.value;
    let summary = `Set ${boilerName(index)} to ${mode.type.toLowerCase()} mode`;

    if (values?.temperature !== null && values?.temperature !== undefined) {
      summary += ` at ${values.temperature}°C`;
    }
    if (values?.pressure !== null && values?.pressure !== undefined) {
      summary += ` at ${values.pressure} bar`;
    }

    return summary;
  }

  if (command.type === 'SetBoilerControlTargetValues') {
    const [index, values] = command.value;
    const parts: string[] = [`Set ${boilerName(index)}`];

    if (values.temperature !== null && values.temperature !== undefined) {
      parts.push(`temp: ${values.temperature}°C`);
    }
    if (values.pressure !== null && values.pressure !== undefined) {
      parts.push(`pressure: ${values.pressure} bar`);
    }

    return parts.join(', ');
  }

  if (command.type === 'RunRoutine') {
    const [routineIndex, params] = command.value;
    let routineLabel = '';

    if (routineIndex.type === 'Internal') {
      routineLabel = `Internal #${routineIndex.value}`;
    } else if (routineIndex.type === 'Function') {
      routineLabel = `Function #${routineIndex.value}`;
    } else if (routineIndex.type === 'Custom') {
      routineLabel = `Custom #${routineIndex.value}`;
    }

    const paramCount = params ? params.size : 0;
    return `Run routine ${routineLabel}${paramCount > 0 ? ` (${paramCount} params)` : ''}`;
  }

  if (command.type === 'SetMachineMode') {
    const modeLabel = command.value.type === 'PowerSaveStandby'
      ? 'Power Save Standby'
      : command.value.type;
    return `Set machine mode to ${modeLabel}`;
  }

  return 'Unknown command';
}

interface CommandSummaryProps {
  command: ScheduleAction;
}

export function CommandSummary({ command }: CommandSummaryProps) {
  const machine = useMachine();
  return <span>{getCommandSummary(command, machine.getBoilerName)}</span>;
}
