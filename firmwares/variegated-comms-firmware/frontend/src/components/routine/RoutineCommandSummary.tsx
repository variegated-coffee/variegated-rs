import { RoutineCommand, ParameterValue, ParameterUnit, TransitionOrigin } from '../../schemas/schemas';
import { useMachine } from '../../contexts/MachineContext';

/// Where a transition starts, as a clause to append to its summary.
///
/// `CurrentTarget` renders as nothing: it is the default and the one that needs no
/// explanation, and repeating it on every transition would bury the two that do.
function originClause(origin: TransitionOrigin, unit?: ParameterUnit | null): string {
  if (origin.type === 'CurrentTarget') return '';
  if (origin.type === 'CurrentValue') return ', starting from the measured value';
  return `, starting from ${getParameterValueDisplay(origin.value, unit)}`;
}

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
    const [index, target, time, origin] = command.value;
    const unit: ParameterUnit = { type: 'MillilitersPerSecond' };
    return `Set ${groupName(index)} flow rate to ${getParameterValueDisplay(target, unit)} over ${getParameterValueDisplay(time, { type: 'Seconds' })}${originClause(origin, unit)}`;
  }

  if (command.type === 'SetGroupPressureWithTransition') {
    const [index, target, time, origin] = command.value;
    const unit: ParameterUnit = { type: 'Bar' };
    return `Set ${groupName(index)} pressure to ${getParameterValueDisplay(target, unit)} over ${getParameterValueDisplay(time, { type: 'Seconds' })}${originClause(origin, unit)}`;
  }

  if (command.type === 'SetGroupOutputFlowRateWithTransition') {
    const [index, target, time, origin] = command.value;
    const unit: ParameterUnit = { type: 'MillilitersPerSecond' };
    return `Set ${groupName(index)} output flow rate to ${getParameterValueDisplay(target, unit)} over ${getParameterValueDisplay(time, { type: 'Seconds' })}${originClause(origin, unit)}`;
  }

  if (command.type === 'SetGroupFixedDutyCycleWithTransition') {
    const [index, target, time, origin] = command.value;
    const unit: ParameterUnit = { type: 'Percent' };
    return `Set ${groupName(index)} duty cycle to ${getParameterValueDisplay(target, unit)} over ${getParameterValueDisplay(time, { type: 'Seconds' })}${originClause(origin, unit)}`;
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
