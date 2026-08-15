import { ScheduleAction, RoutineCommand } from '../schemas/schemas';

/**
 * Format a routine command for display (used in routine steps)
 */
export function formatCommand(
  command: RoutineCommand | null | undefined,
  getBoilerName?: (index: number) => string,
  getGroupName?: (index: number) => string,
  getWaterTapName?: (index: number) => string,
  getSteamWandName?: (index: number) => string
): string {
  if (!command) return 'No command';

  // Helper functions for names
  const boilerName = (idx: number) => getBoilerName?.(idx) || `Boiler ${idx}`;
  const groupName = (idx: number) => getGroupName?.(idx) || `Group ${idx}`;
  const waterTapName = (idx: number) => getWaterTapName?.(idx) || `Water Tap ${idx}`;
  const steamWandName = (idx: number) => getSteamWandName?.(idx) || `Steam Wand ${idx}`;

  // Brewing commands
  if (command.type === 'StartBrewing') {
    return `Start brewing (${groupName(command.value)})`;
  }
  if (command.type === 'StopBrewing') {
    return `Stop brewing (${groupName(command.value)})`;
  }

  // Group control commands
  if (command.type === 'SetGroupFullOn') {
    return `Set ${groupName(command.value)} full on`;
  }
  if (command.type === 'SetGroupOff') {
    return `Set ${groupName(command.value)} off`;
  }
  if (command.type === 'SetGroupFlowRate') {
    return `Set ${groupName(command.value[0])} flow rate`;
  }
  if (command.type === 'SetGroupPressure') {
    return `Set ${groupName(command.value[0])} pressure`;
  }
  if (command.type === 'SetGroupOutputFlowRate') {
    return `Set ${groupName(command.value[0])} output flow rate`;
  }
  if (command.type === 'SetGroupFixedDutyCycle') {
    return `Set ${groupName(command.value[0])} duty cycle`;
  }

  // Group control with transitions
  if (command.type === 'SetGroupFlowRateWithTransition') {
    return `Set ${groupName(command.value[0])} flow rate (transition)`;
  }
  if (command.type === 'SetGroupPressureWithTransition') {
    return `Set ${groupName(command.value[0])} pressure (transition)`;
  }
  if (command.type === 'SetGroupOutputFlowRateWithTransition') {
    return `Set ${groupName(command.value[0])} output flow rate (transition)`;
  }
  if (command.type === 'SetGroupFixedDutyCycleWithTransition') {
    return `Set ${groupName(command.value[0])} duty cycle (transition)`;
  }

  // Boiler commands
  if (command.type === 'SetBoilerOff') {
    return `Set ${boilerName(command.value)} off`;
  }
  if (command.type === 'SetBoilerTemperature') {
    return `Set ${boilerName(command.value[0])} temperature`;
  }
  if (command.type === 'SetBoilerPressure') {
    return `Set ${boilerName(command.value[0])} pressure`;
  }

  // Scale commands
  if (command.type === 'TareGroupScale') {
    return `Tare scale (${groupName(command.value)})`;
  }

  // Water tap commands
  if (command.type === 'StartPumpingToWaterTap') {
    return `Start pumping to ${waterTapName(command.value)}`;
  }
  if (command.type === 'StopPumpingToWaterTap') {
    return `Stop pumping to ${waterTapName(command.value)}`;
  }

  // Steam wand commands
  if (command.type === 'StartSteaming') {
    return `Start steaming on ${steamWandName(command.value)}`;
  }
  if (command.type === 'StopSteaming') {
    return `Stop steaming on ${steamWandName(command.value)}`;
  }
  if (command.type === 'SetSteamValveOpenness') {
    return `Set ${steamWandName(command.value[0])} valve openness`;
  }

  // Bumpless transfer commands
  if (command.type === 'InferGroupPressureIntegral') {
    return `Infer ${groupName(command.value[0])} pressure integral`;
  }
  if (command.type === 'InferGroupFlowRateIntegral') {
    return `Infer ${groupName(command.value[0])} flow rate integral`;
  }
  if (command.type === 'InferGroupOutputFlowRateIntegral') {
    return `Infer ${groupName(command.value[0])} output flow rate integral`;
  }

  // All RoutineCommand variants are handled above
  return 'Unknown command';
}

/**
 * Format a schedule action for display
 */
export function formatScheduleAction(
  action: ScheduleAction,
  getBoilerName?: (index: number) => string
): string {
  // Helper function for names
  const boilerName = (idx: number) => getBoilerName?.(idx) || `Boiler ${idx}`;

  // Handle CancelRoutine unit variant
  if (action.type === 'CancelRoutine') {
    return 'Cancel routine';
  }

  // Handle object variants
  if (action.type === 'SetBoilerControlTarget') {
    const [index, mode, values] = action.value;
    let summary = `Set ${boilerName(index)} to ${mode.type.toLowerCase()}`;

    if (values?.temperature !== null && values?.temperature !== undefined) {
      summary += ` at ${values.temperature}°C`;
    }
    if (values?.pressure !== null && values?.pressure !== undefined) {
      summary += ` at ${values.pressure} bar`;
    }

    return summary;
  }

  if (action.type === 'SetBoilerControlTargetValues') {
    const [index, values] = action.value;
    const parts: string[] = [`Set ${boilerName(index)}`];

    if (values.temperature !== null && values.temperature !== undefined) {
      parts.push(`${values.temperature}°C`);
    }
    if (values.pressure !== null && values.pressure !== undefined) {
      parts.push(`${values.pressure} bar`);
    }

    return parts.join(', ');
  }

  if (action.type === 'RunRoutine') {
    const [routineIndex, params] = action.value;
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

  if (action.type === 'SetMachineMode') {
    const modeLabel = action.value.type === 'PowerSaveStandby'
      ? 'Power Save Standby'
      : action.value.type;
    return `Set machine mode to ${modeLabel}`;
  }

  return 'Unknown action';
}

/**
 * Format multiple schedule actions for display (used in schedules)
 */
export function formatScheduleActionsSummary(
  actions: ScheduleAction[],
  getBoilerName?: (index: number) => string
): string {
  if (actions.length === 0) return 'No actions';
  if (actions.length > 3) return `${actions.length} actions`;

  const summaries = actions.map(action => formatScheduleAction(action, getBoilerName));
  return summaries.join(', ');
}
