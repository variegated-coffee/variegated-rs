import { RoutineExit, ParameterValue } from '../schemas/schemas';

/**
 * Format a parameter value for display
 */
function formatParameterValue(value: ParameterValue): number {
  if (value.type === 'Static') {
    return value.value;
  }
  return 0;
}

/**
 * Format a single exit condition for display (technical details only, no description)
 */
export function formatExitCondition(
  exit: RoutineExit,
  getBoilerName?: (index: number) => string,
  getGroupName?: (index: number) => string
): string {
  const boilerName = (idx: number) => getBoilerName?.(idx) || `Boiler ${idx}`;
  const groupName = (idx: number) => getGroupName?.(idx) || `Group ${idx}`;

  let conditionText = '';

  if (exit.condition.type === 'Always') {
    conditionText = 'Always';
  } else if (exit.condition.type === 'Never') {
    conditionText = 'Never';
  } else if (exit.condition.type === 'After') {
    const value = formatParameterValue(exit.condition.value);
    conditionText = `After ${value}s`;
  } else if (exit.condition.type === 'AfterDurationRelativeToStart') {
    const value = formatParameterValue(exit.condition.value);
    conditionText = `After ${value}s from start`;
  } else if (exit.condition.type === 'StateConditionMet') {
    const stateCondition = exit.condition.value;

    if (stateCondition.type === 'InputVolumeAboveRelativeToStart') {
      const [groupIdx, targetValue] = stateCondition.value;
      const value = formatParameterValue(targetValue);
      conditionText = `${groupName(groupIdx)} Input Volume > ${value} mL`;
    } else if (stateCondition.type === 'Brewing') {
      conditionText = `${groupName(stateCondition.value)} is brewing`;
    } else if (stateCondition.type === 'NotBrewing') {
      conditionText = `${groupName(stateCondition.value)} not brewing`;
    } else if (stateCondition.type === 'BoilerTemperatureAbove') {
      const [boilerIdx, targetValue] = stateCondition.value;
      const value = formatParameterValue(targetValue);
      conditionText = `${boilerName(boilerIdx)} Temperature > ${value}°C`;
    } else if (stateCondition.type === 'BoilerTemperatureBelow') {
      const [boilerIdx, targetValue] = stateCondition.value;
      const value = formatParameterValue(targetValue);
      conditionText = `${boilerName(boilerIdx)} Temperature < ${value}°C`;
    } else if (stateCondition.type === 'BoilerPressureAbove') {
      const [boilerIdx, targetValue] = stateCondition.value;
      const value = formatParameterValue(targetValue);
      conditionText = `${boilerName(boilerIdx)} Pressure > ${value} bar`;
    } else if (stateCondition.type === 'BoilerPressureBelow') {
      const [boilerIdx, targetValue] = stateCondition.value;
      const value = formatParameterValue(targetValue);
      conditionText = `${boilerName(boilerIdx)} Pressure < ${value} bar`;
    } else if (stateCondition.type === 'GroupPressureAbove') {
      const [groupIdx, targetValue] = stateCondition.value;
      const value = formatParameterValue(targetValue);
      conditionText = `${groupName(groupIdx)} Pressure > ${value} bar`;
    } else if (stateCondition.type === 'GroupPressureBelow') {
      const [groupIdx, targetValue] = stateCondition.value;
      const value = formatParameterValue(targetValue);
      conditionText = `${groupName(groupIdx)} Pressure < ${value} bar`;
    } else if (stateCondition.type === 'OutputWeightAbove') {
      const [groupIdx, targetValue] = stateCondition.value;
      const value = formatParameterValue(targetValue);
      conditionText = `${groupName(groupIdx)} Output Weight > ${value} g`;
    } else if (stateCondition.type === 'OutputWeightBelow') {
      const [groupIdx, targetValue] = stateCondition.value;
      const value = formatParameterValue(targetValue);
      conditionText = `${groupName(groupIdx)} Output Weight < ${value} g`;
    } else if (stateCondition.type === 'GroupInputFlowRateAbove') {
      const [groupIdx, targetValue] = stateCondition.value;
      const value = formatParameterValue(targetValue);
      conditionText = `${groupName(groupIdx)} Input Flow Rate > ${value} mL/s`;
    } else if (stateCondition.type === 'GroupInputFlowRateBelow') {
      const [groupIdx, targetValue] = stateCondition.value;
      const value = formatParameterValue(targetValue);
      conditionText = `${groupName(groupIdx)} Input Flow Rate < ${value} mL/s`;
    } else if (stateCondition.type === 'WaterTapFlowRateAbove') {
      const [tapIdx, targetValue] = stateCondition.value;
      const value = formatParameterValue(targetValue);
      conditionText = `Water Tap ${tapIdx} Flow Rate > ${value} mL/s`;
    } else if (stateCondition.type === 'WaterTapFlowRateBelow') {
      const [tapIdx, targetValue] = stateCondition.value;
      const value = formatParameterValue(targetValue);
      conditionText = `Water Tap ${tapIdx} Flow Rate < ${value} mL/s`;
    } else {
      conditionText = 'Unknown';
    }
  } else if (exit.condition.type === 'UserAction') {
    conditionText = `User Action ${exit.condition.value}`;
  } else {
    conditionText = 'Unknown';
  }

  // Return only the technical condition text (description handled by caller)
  return conditionText;
}

/**
 * Format the exit action (then) for display
 */
export function formatExitAction(exit: RoutineExit): string {
  if (exit.then.type === 'NextStep') {
    return 'Next';
  } else if (exit.then.type === 'Finished') {
    return 'Finished';
  } else if (exit.then.type === 'JumpToStep') {
    return `Jump to ${exit.then.value}`;
  }
  return 'Unknown';
}

/**
 * Format a full exit condition with action for display
 * Shows description with technical condition in parentheses when description exists
 */
export function formatExitWithAction(
  exit: RoutineExit,
  getBoilerName?: (index: number) => string,
  getGroupName?: (index: number) => string
): string {
  const condition = formatExitCondition(exit, getBoilerName, getGroupName);
  const action = formatExitAction(exit);

  if (exit.description) {
    // Show description with technical condition in parentheses
    return `${exit.description} (${condition}) → ${action}`;
  } else {
    // Show only technical condition
    return `${condition} → ${action}`;
  }
}

/**
 * Format multiple exit conditions as a compact summary
 */
export function formatExitConditionsSummary(
  exits: RoutineExit[],
  getBoilerName?: (index: number) => string,
  getGroupName?: (index: number) => string,
  maxDisplay: number = 2
): string {
  if (exits.length === 0) return 'No exits';

  const formatted = exits.slice(0, maxDisplay).map(exit =>
    formatExitWithAction(exit, getBoilerName, getGroupName)
  );

  if (exits.length > maxDisplay) {
    formatted.push(`+${exits.length - maxDisplay} more`);
  }

  return formatted.join(', ');
}
