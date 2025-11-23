import { RoutineCommand, ParameterValue, ParameterUnit } from '../../schemas/schemas';
import { useMachine } from '../../contexts/MachineContext';

export function getParameterValueDisplay(value: ParameterValue, unit?: ParameterUnit | null): string {
  if (value.type === 'Static') {
    const num = value.value;
    const unitStr = unit ? getUnitSymbol(unit) : '';
    return `${num}${unitStr}`;
  }
  if (value.type === 'Parameter') {
    return `P${value.value}`;
  }
  if (value.type === 'DerivedParameter') {
    return `D${value.value}`;
  }
  return '?';
}

function getUnitSymbol(unit: ParameterUnit): string {
  switch (unit.type) {
    case 'Seconds': return 's';
    case 'Celsius': return '°C';
    case 'Bar': return ' bar';
    case 'MillilitersPerSecond': return ' ml/s';
    case 'Grams': return 'g';
    case 'Percent': return '%';
    default: return '';
  }
}

export function getRoutineCommandSummary(
  command: RoutineCommand,
  getBoilerName?: (index: number) => string,
  getGroupName?: (index: number) => string,
  getWaterTapName?: (index: number) => string,
  getSteamWandName?: (index: number) => string
): string {
  // Helper functions with fallbacks
  const boilerName = (idx: number) => getBoilerName ? getBoilerName(idx) : `boiler ${idx}`;
  const groupName = (idx: number) => getGroupName ? getGroupName(idx) : `group ${idx}`;
  const waterTapName = (idx: number) => getWaterTapName ? getWaterTapName(idx) : `water tap ${idx}`;
  const steamWandName = (idx: number) => getSteamWandName ? getSteamWandName(idx) : `steam wand ${idx}`;

  if (command.type === 'StartBrewing') {
    return `Start brewing on ${groupName(command.value)}`;
  }

  if (command.type === 'StopBrewing') {
    return `Stop brewing on ${groupName(command.value)}`;
  }

  if (command.type === 'TareGroupScale') {
    return `Tare scale on ${groupName(command.value)}`;
  }

  if (command.type === 'StartPumpingToWaterTap') {
    return `Start pumping to ${waterTapName(command.value)}`;
  }

  if (command.type === 'StopPumpingToWaterTap') {
    return `Stop pumping to ${waterTapName(command.value)}`;
  }

  if (command.type === 'StartSteaming') {
    return `Start steaming on ${steamWandName(command.value)}`;
  }

  if (command.type === 'StopSteaming') {
    return `Stop steaming on ${steamWandName(command.value)}`;
  }

  if (command.type === 'SetSteamValveOpenness') {
    const [index, value] = command.value;
    return `Set ${steamWandName(index)} valve openness to ${getParameterValueDisplay(value, { type: 'Percent' })}`;
  }

  if (command.type === 'SetBoilerTemperature') {
    const [index, value] = command.value;
    return `Set ${boilerName(index)} temperature to ${getParameterValueDisplay(value, { type: 'Celsius' })}`;
  }

  if (command.type === 'SetBoilerPressure') {
    const [index, value] = command.value;
    return `Set ${boilerName(index)} pressure to ${getParameterValueDisplay(value, { type: 'Bar' })}`;
  }

  if (command.type === 'SetGroupFlowRate') {
    const [index, value] = command.value;
    return `Set ${groupName(index)} flow rate to ${getParameterValueDisplay(value, { type: 'MillilitersPerSecond' })}`;
  }

  if (command.type === 'SetGroupPressure') {
    const [index, value] = command.value;
    return `Set ${groupName(index)} pressure to ${getParameterValueDisplay(value, { type: 'Bar' })}`;
  }

  if (command.type === 'SetGroupOutputFlowRate') {
    const [index, value] = command.value;
    return `Set ${groupName(index)} output flow rate to ${getParameterValueDisplay(value, { type: 'MillilitersPerSecond' })}`;
  }

  if (command.type === 'SetGroupFixedDutyCycle') {
    const [index, value] = command.value;
    return `Set ${groupName(index)} duty cycle to ${getParameterValueDisplay(value, { type: 'Percent' })}`;
  }

  if (command.type === 'SetGroupFullOn') {
    return `Set ${groupName(command.value)} to full on`;
  }

  if (command.type === 'SetGroupOff') {
    return `Set ${groupName(command.value)} off`;
  }

  if (command.type === 'SetBoilerOff') {
    return `Set ${boilerName(command.value)} off`;
  }

  if (command.type === 'SetGroupFlowRateWithTransition') {
    const [index, target, time] = command.value;
    return `Set ${groupName(index)} flow rate to ${getParameterValueDisplay(target, { type: 'MillilitersPerSecond' })} over ${getParameterValueDisplay(time, { type: 'Seconds' })}`;
  }

  if (command.type === 'SetGroupPressureWithTransition') {
    const [index, target, time] = command.value;
    return `Set ${groupName(index)} pressure to ${getParameterValueDisplay(target, { type: 'Bar' })} over ${getParameterValueDisplay(time, { type: 'Seconds' })}`;
  }

  if (command.type === 'SetGroupOutputFlowRateWithTransition') {
    const [index, target, time] = command.value;
    return `Set ${groupName(index)} output flow rate to ${getParameterValueDisplay(target, { type: 'MillilitersPerSecond' })} over ${getParameterValueDisplay(time, { type: 'Seconds' })}`;
  }

  if (command.type === 'SetGroupFixedDutyCycleWithTransition') {
    const [index, target, time] = command.value;
    return `Set ${groupName(index)} duty cycle to ${getParameterValueDisplay(target, { type: 'Percent' })} over ${getParameterValueDisplay(time, { type: 'Seconds' })}`;
  }

  if (command.type === 'InferGroupPressureIntegral') {
    const [index, target] = command.value;
    return `Infer ${groupName(index)} pressure integral at ${getParameterValueDisplay(target, { type: 'Bar' })}`;
  }

  if (command.type === 'InferGroupFlowRateIntegral') {
    const [index, target] = command.value;
    return `Infer ${groupName(index)} flow rate integral at ${getParameterValueDisplay(target, { type: 'MillilitersPerSecond' })}`;
  }

  if (command.type === 'InferGroupOutputFlowRateIntegral') {
    const [index, target] = command.value;
    return `Infer ${groupName(index)} output flow rate integral at ${getParameterValueDisplay(target, { type: 'MillilitersPerSecond' })}`;
  }

  return 'Unknown command';
}

interface RoutineCommandSummaryProps {
  command: RoutineCommand;
}

export function RoutineCommandSummary({ command }: RoutineCommandSummaryProps) {
  const machine = useMachine();
  return <span>{getRoutineCommandSummary(command, machine.getBoilerName, machine.getGroupName, machine.getWaterTapName, machine.getSteamWandName)}</span>;
}
